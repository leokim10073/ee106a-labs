"""
Epipolar geometry (Section 4) and two-view reconstruction (Section 5).

    python3 two_view.py ../captures/<run> 3 4

Draws the epipolar lines for images 3 and 4, then (once Section 5 is done)
triangulates the matches and scales them to meters using the SLAM poses.
Everything is saved in output/<run>/.
"""
import argparse
import os

import cv2
import numpy as np

from common import (T_BASE_CAM, load_calibration, load_run, rotation_angle_deg,
                    run_output_dir, write_ply)
from features import detect_sift, match, side_by_side

RANSAC_THRESHOLD_PX = 1.5


# Section 4

def estimate_fundamental(x1, x2, threshold_px=RANSAC_THRESHOLD_PX):
    # x1, x2 are (N, 2) matching pixels. returns F (x2^T F x1 = 0) and a boolean
    # inlier mask, or (None, all False) if it doesn't work
    if len(x1) < 8:
        return None, np.zeros(len(x1), bool)
    # TODO 4.1: cv2.findFundamentalMat with method=cv2.FM_RANSAC,
    # ransacReprojThreshold=threshold_px and confidence=0.999. the mask it
    # gives back is (N, 1) of 0s and 1s. F can come back as None (or not 3x3)
    # if it fails.
    raise NotImplementedError('TODO 4.1')


def epipolar_lines(F, points, image_index):
    # lines (a, b, c) in the *other* image for points in image 1 or 2,
    # scaled so a^2 + b^2 = 1. returns (N, 3)
    # TODO 4.2: cv2.computeCorrespondEpilines. takes (N, 1, 2), gives back (N, 1, 3)
    raise NotImplementedError('TODO 4.2')


def epipolar_distances(F, x1, x2):
    # TODO 4.3: how far (in pixels) each x2 is from the epipolar line of its x1.
    # since a^2 + b^2 = 1 the distance is just |a*x + b*y + c|
    raise NotImplementedError('TODO 4.3')


# Section 5

def essential_from_fundamental(F, K):
    # TODO 5.1: E = K^T F K (same camera for both images)
    raise NotImplementedError('TODO 5.1')


def relative_pose(E, x1, x2, K):
    # returns R, t (X2 = R X1 + t, with |t| = 1) and a mask of the points
    # recoverPose kept
    # TODO 5.2: cv2.recoverPose(E, x1, x2, K) returns (count, R, t, mask).
    # t comes back as (3, 1) and the mask as (N, 1)
    raise NotImplementedError('TODO 5.2')


def triangulate(K, R, t, x1, x2):
    # 3D points in camera 1's frame, plus a mask of which ones are in front of
    # both cameras
    # TODO 5.3: P1 = K [I | 0], P2 = K [R | t]. cv2.triangulatePoints wants the
    # points as (2, N) and gives you homogeneous (4, N) back. then check the
    # depth (z) of each point in both cameras
    raise NotImplementedError('TODO 5.3')


def camera_pose_in_map(T_map_base):
    # TODO 5.4: camera pose in the map (4x4). you have the robot's pose from
    # SLAM, and T_BASE_CAM (in common.py) is where the camera is on the robot
    raise NotImplementedError('TODO 5.4')


def metric_scale(t_unit, baseline_m):
    # TODO 5.5: E only gives you the direction of t. baseline_m is how far apart
    # the cameras actually were according to SLAM
    raise NotImplementedError('TODO 5.5')


LINE_COLORS = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48),
               (145, 30, 180), (70, 240, 240), (240, 50, 230), (210, 245, 60), (250, 190, 190),
               (0, 128, 128), (170, 110, 40), (128, 0, 0), (0, 0, 128), (128, 128, 0)]


def draw_line(img, line, color, K, dist):
    # the line is in undistorted pixels, so put the distortion back before drawing
    # it on the original image
    a, b, c = line
    h, w = img.shape[:2]
    if abs(b) > abs(a):
        xs = np.linspace(-0.2 * w, 1.2 * w, 80)
        pts = np.stack([xs, -(a * xs + c) / b], 1)
    else:
        ys = np.linspace(-0.2 * h, 1.2 * h, 80)
        pts = np.stack([-(b * ys + c) / a, ys], 1)
    rays = np.hstack([(pts - K[:2, 2]) / np.diag(K)[:2], np.ones((len(pts), 1))])
    pix, _ = cv2.projectPoints(rays.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)
    cv2.polylines(img, [pix.reshape(-1, 1, 2).astype(np.int32)], False, color, 2, cv2.LINE_AA)


def draw_epipolar(img1, img2, f1, f2, pairs, F, K, dist, n=12, seed=0):
    rng = np.random.default_rng(seed)
    pick = rng.choice(len(pairs), min(n, len(pairs)), replace=False)
    a, b = img1.copy(), img2.copy()
    for k, idx in enumerate(pick):
        i, j = pairs[idx]
        color = LINE_COLORS[k % len(LINE_COLORS)][::-1]
        draw_line(a, epipolar_lines(F, f2.pixels[j][None], 2)[0], color, K, dist)
        draw_line(b, epipolar_lines(F, f1.pixels[i][None], 1)[0], color, K, dist)
        cv2.circle(a, tuple(int(v) for v in np.round(f1.raw_pixels[i])), 5, color, -1)
        cv2.circle(b, tuple(int(v) for v in np.round(f2.raw_pixels[j])), 5, color, -1)
    canvas, _ = side_by_side(a, b)
    return canvas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('i', type=int, nargs='?', default=0)
    ap.add_argument('j', type=int, nargs='?', default=1)
    args = ap.parse_args()
    i, j = args.i, args.j

    views = load_run(args.run_dir)
    K, dist = load_calibration()
    out = run_output_dir(args.run_dir)
    f1, f2 = detect_sift(views[i].image, K, dist), detect_sift(views[j].image, K, dist)
    m = match(f1, f2)
    x1, x2 = f1.pixels[m[:, 0]], f2.pixels[m[:, 1]]
    print(f'images {i} and {j}: {len(m)} matches')

    # Section 4
    F, inliers = estimate_fundamental(x1, x2)
    if F is None:
        raise SystemExit('couldn\'t find F, not enough matches. try two images that overlap more')
    d = epipolar_distances(F, x1[inliers], x2[inliers])
    print(f'{inliers.sum()}/{len(m)} matches are RANSAC inliers')
    print(f'inlier distance from epipolar line: median {np.median(d):.3f} px, max {d.max():.3f} px')
    path = os.path.join(out, f'epipolar_{i}_{j}.png')
    cv2.imwrite(path, draw_epipolar(views[i].image, views[j].image, f1, f2, m[inliers], F, K, dist))
    print(f'saved {path}')

    # Section 5
    x1i, x2i = x1[inliers], x2[inliers]
    try:
        E = essential_from_fundamental(F, K)
        R, t, pose_ok = relative_pose(E, x1i, x2i, K)
        X, in_front = triangulate(K, R, t, x1i, x2i)
        T_i = camera_pose_in_map(views[i].T_map_base)
        T_j = camera_pose_in_map(views[j].T_map_base)
        baseline = float(np.linalg.norm(T_j[:3, 3] - T_i[:3, 3]))
        scale = metric_scale(t, baseline)
    except NotImplementedError as e:
        print(f'\nstopping here, Section 5 isn\'t done yet ({e})')
        return

    print(f'\ncamera turned {rotation_angle_deg(R):.2f} deg, t = {np.round(t, 3)}')
    if baseline < 0.03:
        print(f'warning: the cameras are only {baseline * 100:.1f} cm apart, so t and the scale are unreliable')

    keep = in_front & pose_ok
    X = X[keep] * scale
    print(f'triangulated {keep.sum()} points in front of both cameras')
    if keep.sum() == 0:
        return
    print(f'SLAM baseline is {baseline * 100:.1f} cm, so scale = {scale:.3f} m')

    for name, (Rc, tc), x in zip((i, j), [(np.eye(3), np.zeros(3)), (R, t * scale)], (x1i[keep], x2i[keep])):
        proj = (X @ Rc.T + tc) @ K.T
        err = np.linalg.norm(proj[:, :2] / proj[:, 2:3] - x, axis=1)
        print(f'reprojection error in image {name}: median {np.median(err):.3f} px')
    print(f'depth from camera {i}: median {np.median(X[:, 2]):.2f} m '
          f'(5th-95th percentile {np.percentile(X[:, 2], 5):.2f} to {np.percentile(X[:, 2], 95):.2f} m)')

    # parallax = angle between the two rays to each point
    c2 = -R.T @ (t * scale)
    rays1 = X / np.linalg.norm(X, axis=1, keepdims=True)
    rays2 = (X - c2) / np.linalg.norm(X - c2, axis=1, keepdims=True)
    parallax = np.degrees(np.arccos(np.clip(np.sum(rays1 * rays2, axis=1), -1, 1)))
    print(f'parallax: median {np.median(parallax):.1f} deg')

    # save in the map frame (camera i at its SLAM pose) for the viewer
    X_map = X @ T_i[:3, :3].T + T_i[:3, 3]
    colors = f1.colors[m[inliers][keep][:, 0]]
    write_ply(os.path.join(out, 'two_view.ply'), X_map, colors)
    R_map_cam2 = T_i[:3, :3] @ R.T
    c2_map = T_i[:3, :3] @ c2 + T_i[:3, 3]
    np.savez(os.path.join(out, 'two_view.npz'), points=X_map, colors=colors, K=K,
             R_map_cam=np.stack([T_i[:3, :3], R_map_cam2]), centers=np.stack([T_i[:3, 3], c2_map]),
             names=np.array([views[i].name, views[j].name]), run_dir=os.path.abspath(args.run_dir))
    print(f'saved two_view.ply and two_view.npz to {out}')


if __name__ == '__main__':
    main()
