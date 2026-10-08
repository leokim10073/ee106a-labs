"""
Multi-view reconstruction (Section 6).

    python3 multiview.py ../captures/<run>

Matches every pair of images, links the matches into tracks (the same point
seen in several images), and triangulates each track using the SLAM poses.
Saves output/<run>/multiview.ply and multiview.npz, which bundle_adjust.py uses.
"""
import argparse
import itertools
import os

import cv2
import numpy as np

from common import load_calibration, load_run, run_output_dir, write_ply
from features import detect_sift, match
from two_view import camera_pose_in_map, epipolar_distances, estimate_fundamental

MIN_PAIR_INLIERS = 20
POSE_EPIPOLAR_PX = 4.0   # matches also have to agree with the SLAM poses
MIN_PARALLAX_DEG = 2.0
MAX_REPROJ_PX = 4.0


def world_to_camera(T_map_cam):
    # returns R, t such that X_cam = R X_map + t
    # TODO 6.1: T_map_cam goes from camera to map, so you need the inverse.
    # if its rotation is R_mc and the camera center is c, then R = R_mc^T and t = -R c
    raise NotImplementedError('TODO 6.1')


def reprojection_errors(K, R, t, X, uv):
    # pixel distance between X (N, 3) projected into the camera and the observed uv (N, 2)
    # TODO 6.2: turn R into a rotation vector with cv2.Rodrigues, then project
    # with cv2.projectPoints. use np.zeros(5) for the distortion since uv is
    # already undistorted
    raise NotImplementedError('TODO 6.2')


def triangulate_multiview(Ps, uv):
    # same DLT as two-view, just two rows per image instead of two images
    A = []
    for P, (u, v) in zip(Ps, uv):
        A.append(u * P[2] - P[0])
        A.append(v * P[2] - P[1])
    _, _, Vt = np.linalg.svd(np.array(A))
    X = Vt[-1]
    return X[:3] / X[3]


def pose_fundamental(K, pose_i, pose_j):
    # F we'd expect from the SLAM poses, plus the distance between the cameras
    (Ri, ti), (Rj, tj) = pose_i, pose_j
    R = Rj @ Ri.T
    t = tj - R @ ti
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    Kinv = np.linalg.inv(K)
    return Kinv.T @ tx @ R @ Kinv, float(np.linalg.norm(t))


def match_all_pairs(feats, poses, K):
    pairs = {}
    for i, j in itertools.combinations(range(len(feats)), 2):
        m = match(feats[i], feats[j])
        if len(m) < MIN_PAIR_INLIERS:
            continue
        x1, x2 = feats[i].pixels[m[:, 0]], feats[j].pixels[m[:, 1]]
        F, inliers = estimate_fundamental(x1, x2)
        if F is None or inliers.sum() < MIN_PAIR_INLIERS:
            continue
        m, x1, x2 = m[inliers], x1[inliers], x2[inliers]
        F_pose, baseline = pose_fundamental(K, poses[i], poses[j])
        if baseline > 0.02:  # F_pose isn't meaningful if the robot basically just turned
            m = m[epipolar_distances(F_pose, x1, x2) < POSE_EPIPOLAR_PX]
        if len(m) >= MIN_PAIR_INLIERS:
            pairs[(i, j)] = m
    return pairs


def build_tracks(pairs):
    # union-find over (image, keypoint) pairs
    parent = {}

    def find(a):
        parent.setdefault(a, a)
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for (i, j), m in pairs.items():
        for a, b in m:
            ra, rb = find((i, int(a))), find((j, int(b)))
            if ra != rb:
                parent[ra] = rb
    groups = {}
    for node in list(parent):
        groups.setdefault(find(node), []).append(node)
    # throw out tracks that use two keypoints from the same image
    return [sorted(g) for g in groups.values() if len({v for v, _ in g}) == len(g) >= 2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    args = ap.parse_args()

    views = load_run(args.run_dir)
    K, dist = load_calibration()
    out = run_output_dir(args.run_dir)
    feats = [detect_sift(v.image, K, dist) for v in views]
    print('keypoints per image:', [len(f.pixels) for f in feats])

    poses = [world_to_camera(camera_pose_in_map(v.T_map_base)) for v in views]
    centers = [-R.T @ t for R, t in poses]
    pairs = match_all_pairs(feats, poses, K)
    print(f'{len(pairs)} pairs of images with at least {MIN_PAIR_INLIERS} good matches')
    tracks = build_tracks(pairs)
    print(f'{len(tracks)} tracks')

    Ps = [K @ np.hstack([R, t[:, None]]) for R, t in poses]
    points, kept = [], []
    for track in tracks:
        uv = np.array([feats[v].pixels[k] for v, k in track])
        X = triangulate_multiview([Ps[v] for v, _ in track], uv)
        rays = np.array([(X - centers[v]) / np.linalg.norm(X - centers[v]) for v, _ in track])
        parallax = np.degrees(np.arccos(np.clip((rays @ rays.T).min(), -1, 1)))
        depths = [(poses[v][0] @ X + poses[v][1])[2] for v, _ in track]
        errs = [reprojection_errors(K, *poses[v], X[None], u[None])[0] for (v, _), u in zip(track, uv)]
        # in front of every camera, reprojects well everywhere, and enough parallax
        if min(depths) > 0.05 and max(errs) < MAX_REPROJ_PX and parallax > MIN_PARALLAX_DEG:
            points.append(X)
            kept.append(track)
    points = np.array(points)
    print(f'kept {len(points)} points')

    cam_idx = np.array([v for t in kept for v, _ in t])
    pt_idx = np.array([p for p, t in enumerate(kept) for _ in t])
    uv = np.array([feats[v].pixels[k] for t in kept for v, k in t])
    err = np.concatenate([reprojection_errors(K, *poses[v], points[pt_idx[cam_idx == v]], uv[cam_idx == v])
                          for v in np.unique(cam_idx)])
    print(f'{len(uv)} observations, RMS reprojection error {np.sqrt(np.mean(err ** 2)):.2f} px')
    colors = np.array([np.median([feats[v].colors[k] for v, k in t], axis=0) for t in kept]).astype(np.uint8)

    write_ply(os.path.join(out, 'multiview.ply'), points, colors)
    np.savez(os.path.join(out, 'multiview.npz'), K=K, R=np.array([R for R, _ in poses]),
             t=np.array([t for _, t in poses]), points=points, colors=colors,
             cam_idx=cam_idx, pt_idx=pt_idx, uv=uv, names=np.array([v.name for v in views]),
             run_dir=os.path.abspath(args.run_dir))
    print(f'saved multiview.ply and multiview.npz to {out}')


if __name__ == '__main__':
    main()
