"""
Bundle adjustment (Section 7). This one is given, you just run it.

    python3 bundle_adjust.py ../captures/<run>

Starts from multiview.npz and refines all the camera poses and 3D points
together, while keeping the cameras from wandering too far from their SLAM
poses. Saves ba.ply, ba_result.npz and ba_history.npz (every iteration, for
view_viser.py) to output/<run>/.
"""
import argparse
import os

import numpy as np

import bundle
from common import rotation_angle_deg, run_output_dir, write_ply

MAX_REPROJ_PX = 3.0


def reprojection(state, cam_idx, pt_idx, uv, cx, cy):
    proj, Xc = bundle.project(state, cam_idx, pt_idx, cx, cy)
    return np.linalg.norm(proj - uv, axis=1), Xc[:, 2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--refine-focal', action='store_true',
                    help='also refine the focal length (if you didn\'t calibrate)')
    args = ap.parse_args()
    out = run_output_dir(args.run_dir)
    d = np.load(os.path.join(out, 'multiview.npz'))
    K = d['K']
    cx, cy = K[0, 2], K[1, 2]
    cam_idx, pt_idx, uv = d['cam_idx'], d['pt_idx'], d['uv']
    colors = d['colors']

    prior = bundle.Prior(d['R'].copy(), d['t'].copy(), rot_deg=1.0, pos_m=0.03)
    state = bundle.State(K[0, 0], d['R'].copy(), d['t'].copy(), d['points'].copy())
    err0, _ = reprojection(state, cam_idx, pt_idx, uv, cx, cy)
    print(f'starting with {len(state.X)} points, {len(uv)} observations, '
          f'RMS error {np.sqrt(np.mean(err0 ** 2)):.3f} px')

    # run it twice, throwing out observations that still don't fit in between
    history = None
    for rnd in range(2):
        result = bundle.bundle_adjust(state, cam_idx, pt_idx, uv, cx, cy, prior=prior,
                                      optimize_focal=args.refine_focal, record=(rnd == 0))
        if rnd == 0:
            history = dict(hist=result.history, colors=colors, cam_idx=cam_idx, pt_idx=pt_idx, uv=uv)
        state = result.state
        err, depth = reprojection(state, cam_idx, pt_idx, uv, cx, cy)
        good = (err < MAX_REPROJ_PX) & (depth > 0.05)
        keep_pt = np.bincount(pt_idx[good], minlength=len(state.X)) >= 2
        keep_obs = good & keep_pt[pt_idx]
        remap = -np.ones(len(state.X), int)
        remap[keep_pt] = np.arange(keep_pt.sum())
        state.X, colors = state.X[keep_pt], colors[keep_pt]
        cam_idx, pt_idx, uv = cam_idx[keep_obs], remap[pt_idx[keep_obs]], uv[keep_obs]
        note = '' if result.converged else " (didn't converge)"
        print(f'round {rnd + 1}: {result.iterations} iterations{note}, {len(state.X)} points left')

    err, _ = reprojection(state, cam_idx, pt_idx, uv, cx, cy)
    C0 = -np.einsum('nji,nj->ni', prior.R, prior.t)
    shifts = np.linalg.norm(state.centers() - C0, axis=1)
    turns = [rotation_angle_deg(R @ R0.T) for R, R0 in zip(state.R, prior.R)]
    print(f'RMS error after: {np.sqrt(np.mean(err ** 2)):.3f} px (median {np.median(err):.3f} px)')
    print(f'cameras moved {np.median(shifts) * 1000:.0f} mm (median, max {shifts.max() * 1000:.0f} mm) '
          f'and turned {np.median(turns):.2f} deg from the SLAM poses')
    if args.refine_focal:
        print(f'focal length {K[0, 0]:.1f} -> {state.f:.1f} px')

    write_ply(os.path.join(out, 'ba.ply'), state.X, colors)
    K_out = K.copy()
    K_out[0, 0] = K_out[1, 1] = state.f
    np.savez(os.path.join(out, 'ba_result.npz'), K=K_out, R=state.R, t=state.t, points=state.X,
             colors=colors, cam_idx=cam_idx, pt_idx=pt_idx, uv=uv)
    h = history['hist']
    np.savez(os.path.join(out, 'ba_history.npz'),
             cost=np.array([x['cost'] for x in h]), rms=np.array([x['rms'] for x in h]),
             median=np.array([x['median'] for x in h]), f=np.array([x['f'] for x in h]),
             R=np.array([x['R'] for x in h]), t=np.array([x['t'] for x in h]),
             X=np.array([x['X'] for x in h]), colors=history['colors'],
             cam_idx=history['cam_idx'], pt_idx=history['pt_idx'], uv=history['uv'],
             K=K, names=d['names'], run_dir=d['run_dir'])
    print(f'saved ba.ply, ba_result.npz and ba_history.npz to {out}')


if __name__ == '__main__':
    main()
