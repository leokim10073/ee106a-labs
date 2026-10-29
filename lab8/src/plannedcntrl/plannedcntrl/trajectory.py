#!/usr/bin/env python3
import sys

import numpy as np

GOAL_DISTANCE = 1.8288   # 6 ft

OBSTACLE_RADIUS = 0.35
SAFETY = 1.3
PUSH_STEP = 0.05
MAX_PUSH_STEPS = 40
MAX_AVOID_ITERS = 3
MAX_VERTICES = 12

# Velocity profile
V_CRUISE = 0.15
V_MIN = 0.04             # floor, or a polyline corner stops the clock dead
A_MAX = 0.30
MAX_ANGULAR = 2.0

FILLET_RADIUS = 0.30     # upper bound; each corner is sized from its own angle
MIN_FILLET = 0.03        # below this a fillet is a degenerate micro-arc
WAYPOINT_SPACING = 0.02


def plot_trajectory(waypoints, obstacles=(), compare=None):
    """Plot a path. `compare` is drawn dashed underneath, so a smoothed
    trajectory can be shown against the raw polyline it came from."""
    import matplotlib.pyplot as plt

    x_vals = [p[0] for p in waypoints]
    y_vals = [p[1] for p in waypoints]
    plt.figure(figsize=(10, 6))
    if compare is not None:
        plt.plot([p[0] for p in compare], [p[1] for p in compare],
                 '--', color='gray', linewidth=1.5, label='Unsmoothed')
    plt.plot(x_vals, y_vals, '-', linewidth=2, label='Trajectory')
    plt.scatter(x_vals[0], y_vals[0], color='green', s=100, zorder=5, label='Start')
    plt.scatter(x_vals[-1], y_vals[-1], color='red', s=100, zorder=5, label='Goal')
    for x, y, theta in waypoints[::5]:
        plt.arrow(x, y, 0.05*np.cos(theta), 0.05*np.sin(theta),
                  head_width=0.01, head_length=0.005, fc='blue', ec='blue')
    for ox, oy in obstacles:
        plt.scatter(ox, oy, color='orange', marker='^', s=140, zorder=5)
        plt.gca().add_patch(
            plt.Circle((ox, oy), OBSTACLE_RADIUS, color='orange',
                       alpha=0.25, zorder=1)
        )
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Trajectory')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()


def bezier_curve(p0, p1, p2, p3, t):
    """Cubic Bezier interpolation between 4 control points."""
    return (1 - t)**3 * p0 + 3*(1 - t)**2*t*p1 + 3*(1 - t)*t**2*p2 + t**3*p3


def closest_point_on_segment(a, b, p):
    ab = b - a
    denom = np.dot(ab, ab)
    if denom < 1e-12:
        return a.copy()
    t = np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0)
    return a + t * ab


def push_clear(closest, obstacle, obstacles, segment_dir,
               radius=OBSTACLE_RADIUS, safety=SAFETY, step=PUSH_STEP):
    """ Waypoint pushed out until it is clear of EVERY cone's buffer.
        closest - the closest point on the path segment to the obstacle
        obstacle - the (x,y) center of an obstacle that the path segment is too close to
        obstacles - a (what data structure is this) of all obstacle centers
        segment_dir - the direction of the path segment
        radius - the radius of each obstacle
        safety - a factor of safety for the new point
        step - the amount to step by if the initial push isn't enough

        returns new waypoint for the path
    """
    offset = closest - obstacle
    distance = np.linalg.norm(offset)

    if distance < 1e-9:
        direction = ...
    else:
        direction = ...

    #TODO: If the distance is very small, the path segment passes through the center of the cone. 
    # Which direction should the new point go in this case? Otherwise, which 
    # direction should closest get pushed?

    #TODO: Propose a new point safety*radius from the obstacle, and check if it's too close to any 
    # cone in obstacles. If it is, keep stepping out along direction by step until it no longer is. 
    # Return the new point
    raise NotImplementedError


def avoid_obstacles(vertices, obstacles, radius=OBSTACLE_RADIUS,
                    safety=SAFETY, max_iters=MAX_AVOID_ITERS):
    """Insert waypoints until no segment cuts through a cone's buffer."""
    vertices = [np.asarray(v, dtype=float) for v in vertices]
    obstacles = [np.asarray(o, dtype=float) for o in obstacles]

    for _ in range(max_iters):
        inserted = False
        index = 0
        while index < len(vertices) - 1 and len(vertices) < MAX_VERTICES:
            a, b = vertices[index], vertices[index + 1]
            length = np.linalg.norm(b - a)
            if length < 1e-9:
                index += 1
                continue
            segment_dir = (b - a) / length

            for obstacle in obstacles:
                closest = closest_point_on_segment(a, b, obstacle)
                if np.linalg.norm(closest - obstacle) >= radius:
                    continue
                vertices.insert(
                    index + 1,
                    push_clear(closest, obstacle, obstacles, segment_dir,
                               radius, safety),
                )
                inserted = True
                break
            index += 1

        if not inserted or len(vertices) >= MAX_VERTICES:
            break

    if len(vertices) >= MAX_VERTICES:
        print('WARNING: hit the waypoint cap.')

    # Cones whose buffers overlap can push a waypoint so far out that the
    # path doubles back on itself. The result is collision-free but silly,
    # and the cusp forces the velocity profile to crawl. Say so rather than
    # letting it look like a planner success.
    for i in range(1, len(vertices) - 1):
        before = vertices[i] - vertices[i - 1]
        after = vertices[i + 1] - vertices[i]
        n_b, n_a = np.linalg.norm(before), np.linalg.norm(after)
        if n_b > 1e-9 and n_a > 1e-9 and np.dot(before, after) / (n_b * n_a) < -0.5:
            print('WARNING: path reverses on itself -- cones are too close '
                  'together to route cleanly between.')
            break

    return vertices


def fillet_radius_for(turn_angle, v_cruise=V_CRUISE, w_max=MAX_ANGULAR):
    """Smallest fillet that still allows `v_cruise` through the corner.

    The cubic fillet below is not a circular arc; its peak curvature is
    (8/3) * sin(d/2) / (r * cos(d/2)^2) for a direction change d. Holding
    cruise speed needs curvature <= w_max / v_cruise, so invert for r.
    """
    half = 0.5 * abs(turn_angle)
    if half < 1e-6:
        return 0.0
    if half >= 0.5 * np.pi - 1e-6:        # reversal, no radius is enough
        return FILLET_RADIUS
    peak = (8.0 / 3.0) * np.sin(half) / (np.cos(half) ** 2)
    return peak * v_cruise / w_max


def fillet_corners(vertices, spacing=WAYPOINT_SPACING,
                   max_radius=FILLET_RADIUS):
    """Round each corner with a cubic Bezier, sized from the corner angle.

    Sampled at roughly `spacing` so the later resample cannot flatten the arc
    back into a polygon -- the velocity profile reads curvature off these
    points, and a polygon has none along its chords and spikes at its joins.
    """
    if len(vertices) < 3:
        return [np.asarray(v, dtype=float) for v in vertices]

    smoothed = [np.asarray(vertices[0], dtype=float)]
    for i in range(1, len(vertices) - 1):
        previous = np.asarray(vertices[i - 1], dtype=float)
        corner = np.asarray(vertices[i], dtype=float)
        following = np.asarray(vertices[i + 1], dtype=float)

        into = previous - corner
        out_of = following - corner
        into_len = np.linalg.norm(into)
        out_len = np.linalg.norm(out_of)
        if into_len < 1e-9 or out_len < 1e-9:
            smoothed.append(corner)
            continue

        into_u = into / into_len
        out_u = out_of / out_len
        # angle between the legs; the direction change is pi minus that
        cos_legs = float(np.clip(np.dot(into_u, out_u), -1.0, 1.0))
        turn = np.pi - np.arccos(cos_legs)

        wanted = fillet_radius_for(turn)
        # Never eat more than half a leg, or neighbouring fillets collide.
        radius = min(max(wanted, 0.0), max_radius, into_len / 2.0, out_len / 2.0)
        # Short legs would cap the radius down to a micro-arc whose curvature
        # is far worse than the sharp corner it replaces. Leave those alone
        # and let the velocity profile slow down through them instead.
        if radius < MIN_FILLET:
            smoothed.append(corner)
            continue

        p0 = corner + radius * into_u
        p3 = corner + radius * out_u
        n = max(int(np.ceil(2.0 * radius / spacing)), 4)
        for t in np.linspace(0.0, 1.0, n):
            smoothed.append(bezier_curve(p0, corner, corner, p3, t))

    smoothed.append(np.asarray(vertices[-1], dtype=float))
    return smoothed


def waypoints_from_vertices(vertices, spacing=WAYPOINT_SPACING):
    """Resample a vertex list into (x, y, theta) waypoints."""
    points = []
    for i in range(len(vertices) - 1):
        a = np.asarray(vertices[i], dtype=float)
        b = np.asarray(vertices[i + 1], dtype=float)
        length = np.linalg.norm(b - a)
        steps = max(int(length / spacing), 1)
        for step in range(steps):
            points.append(a + (b - a) * (step / steps))
    points.append(np.asarray(vertices[-1], dtype=float))

    # Drop near-duplicates. The threshold has to scale with `spacing`: two
    # points a micrometre apart are a resampling artefact, but discrete
    # curvature would read them as a turn over ~zero arc length and report
    # thousands of 1/m, which the velocity profile then crawls through.
    min_gap = 0.25 * spacing
    deduped = [points[0]]
    for p in points[1:]:
        if np.linalg.norm(p - deduped[-1]) > min_gap:
            deduped.append(p)
    if np.linalg.norm(points[-1] - deduped[-1]) > 1e-9:
        deduped.append(points[-1])     # never drop the goal
    points = deduped

    thetas = []
    for i in range(len(points) - 1):
        delta = points[i + 1] - points[i]
        thetas.append(np.arctan2(delta[1], delta[0]))
    thetas.append(thetas[-1] if thetas else 0.0)
    return [(points[i][0], points[i][1], thetas[i]) for i in range(len(points))]


def path_curvature(points):
    """Discrete curvature at each point, from the turn between neighbours."""
    n = len(points)
    kappa = np.zeros(n)
    for i in range(1, n - 1):
        before = points[i] - points[i - 1]
        after = points[i + 1] - points[i]
        len_b = np.linalg.norm(before)
        len_a = np.linalg.norm(after)
        if len_b < 1e-9 or len_a < 1e-9:
            continue
        # signed turn between consecutive tangents, wrapped to [-pi, pi]
        turn = np.arctan2(after[1], after[0]) - np.arctan2(before[1], before[0])
        turn = (turn + np.pi) % (2.0 * np.pi) - np.pi
        kappa[i] = abs(turn) / (0.5 * (len_b + len_a))
    if n > 2:
        kappa[0], kappa[-1] = kappa[1], kappa[-2]
    return kappa


def time_parameterise(waypoints, v_cruise=V_CRUISE, a_max=A_MAX,
                      w_max=MAX_ANGULAR, v_min=V_MIN):
    """[(x, y, theta)] -> [(t, x, y, theta, v)] at a feasible speed profile.

    Forward pass caps acceleration out of the start, backward pass guarantees
    we can still stop (and can slow down *before* a tight corner, which a
    pointwise limit cannot see). The trapezoid is emergent, not special-cased.
    """
    points = [np.asarray(w[:2], dtype=float) for w in waypoints]
    n = len(points)
    if n < 2:
        return [(0.0, waypoints[0][0], waypoints[0][1], waypoints[0][2], 0.0)]

    ds = np.array([np.linalg.norm(points[i + 1] - points[i])
                   for i in range(n - 1)])
    kappa = path_curvature(points)

    # Curvature limit. The floor matters: a polyline corner has effectively
    # infinite curvature, which would otherwise stop the clock outright.
    with np.errstate(divide='ignore'):
        v_lim = np.where(kappa > 1e-9, w_max / np.maximum(kappa, 1e-9), v_cruise)
    v_lim = np.clip(v_lim, v_min, v_cruise)

    v = v_lim.copy()
    v[0] = 0.0
    for i in range(1, n):                     # forward: acceleration limit
        v[i] = min(v[i], np.sqrt(v[i - 1]**2 + 2.0 * a_max * ds[i - 1]))
    v[-1] = 0.0
    for i in range(n - 2, -1, -1):            # backward: must be able to stop
        v[i] = min(v[i], np.sqrt(v[i + 1]**2 + 2.0 * a_max * ds[i]))

    t = np.zeros(n)
    for i in range(1, n):
        v_avg = 0.5 * (v[i - 1] + v[i])
        t[i] = t[i - 1] + ds[i - 1] / max(v_avg, 1e-6)

    return [(t[i], points[i][0], points[i][1], waypoints[i][2], v[i])
            for i in range(n)]


def sample_trajectory(timed, t):
    """Reference state at time t -> (x_d, y_d, xd_dot, yd_dot, finished).

    Past the end we hold the final point with zero velocity, which is what
    lets the controller settle onto the goal instead of running off the array.
    """
    if not timed:
        return 0.0, 0.0, 0.0, 0.0, True
    if t <= timed[0][0]:
        _, x, y, th, v = timed[0]
        return x, y, v * np.cos(th), v * np.sin(th), False
    if t >= timed[-1][0]:
        _, x, y, _, _ = timed[-1]
        return x, y, 0.0, 0.0, True

    lo, hi = 0, len(timed) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if timed[mid][0] <= t:
            lo = mid
        else:
            hi = mid

    t0, x0, y0, _, v0 = timed[lo]
    t1, x1, y1, _, v1 = timed[hi]
    alpha = (t - t0) / max(t1 - t0, 1e-9)

    x = x0 + alpha * (x1 - x0)
    y = y0 + alpha * (y1 - y0)
    speed = v0 + alpha * (v1 - v0)

    # Direction from the chord, never from interpolating theta -- theta wraps,
    # and blending 179 deg with -179 deg gives 0, which points backwards.
    dx, dy = x1 - x0, y1 - y0
    norm = np.hypot(dx, dy)
    if norm < 1e-9:
        return x, y, 0.0, 0.0, False
    return x, y, speed * dx / norm, speed * dy / norm, False


def path_clearance(waypoints, obstacles):
    """Closest the path gets to any cone. Below OBSTACLE_RADIUS is a hit."""
    if not obstacles or len(waypoints) < 2:
        return float('inf')
    points = [np.asarray(w[:2], dtype=float) for w in waypoints]
    return min(
        np.linalg.norm(closest_point_on_segment(points[i], points[i + 1],
                                                np.asarray(o, dtype=float)) - o)
        for i in range(len(points) - 1)
        for o in obstacles
    )


def plan_trajectory(obstacles_base, robot_pose, mode='bezier', plot=True):
    """Plan from `robot_pose` (x, y, yaw in odom) to a point GOAL_DISTANCE
    straight ahead. `obstacles_base` is cone (x, y) in base_link.

    Returns timed waypoints: [(t, x, y, theta, v)].
    """
    x1, y1, yaw = robot_pose

    def to_odom(point):
        px, py = point
        return np.array([
            x1 + px * np.cos(yaw) - py * np.sin(yaw),
            y1 + px * np.sin(yaw) + py * np.cos(yaw),
        ])

    start = np.array([x1, y1])
    goal = to_odom((GOAL_DISTANCE, 0.0))
    obstacles = [to_odom(o) for o in obstacles_base]

    vertices = avoid_obstacles([start, goal], obstacles)
    raw = vertices
    if mode == 'bezier':
        vertices = fillet_corners(vertices)

    waypoints = waypoints_from_vertices(vertices)
    # 'straight' is the naive path-to-trajectory conversion: constant velocity
    # along each segment, with no curvature awareness. That is what makes it
    # jerky at a corner. 'bezier' both rounds the corner and slows for whatever
    # curvature is left.
    timed = time_parameterise(
        waypoints,
        w_max=(MAX_ANGULAR if mode == 'bezier' else np.inf),
    )
    if plot:
        # In bezier mode, show the polyline it was smoothed from underneath.
        compare = (waypoints_from_vertices(raw) if mode == 'bezier'
                   and len(raw) > 2 else None)
        plot_trajectory(waypoints, [(o[0], o[1]) for o in obstacles], compare)
    return timed


def main(args=None):
    """python3 trajectory.py [--no-plot]"""
    show = '--no-plot' not in sys.argv
    robot_pose = (0.0, 0.0, 0.0)

    scenarios = {
        'cone dead ahead': [(0.9, 0.0)],
        'cone off to one side': [(0.9, 0.2)],
        'cone close in': [(0.6, -0.1)],
        'two cones (apart)': [(0.7, 0.25), (1.5, -0.25)],
        'two cones (buffers overlap)': [(0.7, 0.15), (1.3, -0.15)],
        'three cones': [(0.6, 0.2), (1.0, -0.2), (1.4, 0.2)],
        'no cones': [],
    }

    print(f'goal: {GOAL_DISTANCE:.3f} m   buffer: {OBSTACLE_RADIUS} m   '
          f'cruise: {V_CRUISE} m/s   w_max: {MAX_ANGULAR} rad/s')
    print('  (a sharp corner forces a slow "slowest"; a rounded one does not)')
    for name, obstacles_base in scenarios.items():
        for mode in ('straight', 'bezier'):
            timed = plan_trajectory(obstacles_base, robot_pose,
                                    mode=mode, plot=False)
            waypoints = [(r[1], r[2], r[3]) for r in timed]
            clear = path_clearance(waypoints, obstacles_base)
            duration = timed[-1][0]
            points = [np.array([r[1], r[2]]) for r in timed]
            k_max = path_curvature(points).max()
            # slowest point in the interior: the ends are zero by construction
            v_slow = min((r[4] for r in timed[1:-1]), default=0.0)
            verdict = 'HITS CONE' if clear < OBSTACLE_RADIUS else 'ok'
            print(f'  {name:28s} {mode:9s} clear={clear:6.3f}  '
                  f'{duration:5.1f}s  slowest={v_slow:.3f}  '
                  f'kmax={k_max:5.1f}  {verdict}')

    if show:
        for mode in ('straight', 'bezier'):
            plan_trajectory(scenarios['cone dead ahead'], robot_pose,
                            mode=mode, plot=True)


if __name__ == '__main__':
    main()
