"""
SIFT features and matching (Section 3).

    python3 features.py ../captures/<run>              # number of matches between neighboring images
    python3 features.py ../captures/<run> --image 3    # draw image 3's keypoints
    python3 features.py ../captures/<run> --pair 3 4   # draw the matches between 3 and 4

Pictures go in output/<run>/.
"""
import argparse
import os
from dataclasses import dataclass

import cv2
import numpy as np

from common import load_calibration, load_run, run_output_dir


@dataclass
class Features:
    pixels: np.ndarray       # (N, 2) undistorted positions, use these for any geometry
    raw_pixels: np.ndarray   # (N, 2) positions in the original image, only for drawing
    descriptors: np.ndarray  # (N, 128)
    sizes: np.ndarray        # (N,) keypoint size in pixels
    colors: np.ndarray       # (N, 3) RGB


def detect_sift(image, K, dist, n_features=4000):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # TODO 3.1: make a SIFT detector (at most n_features keypoints) and run
    # detectAndCompute on gray with no mask.
    # https://docs.opencv.org/4.x/da/df5/tutorial_py_sift_intro.html
    raise NotImplementedError('TODO 3.1')

    if not keypoints:
        empty = np.zeros((0, 2))
        return Features(empty, empty, np.zeros((0, 128), np.float32), np.zeros(0), np.zeros((0, 3), np.uint8))
    raw = np.array([kp.pt for kp in keypoints], dtype=np.float64)
    sizes = np.array([kp.size for kp in keypoints], dtype=np.float64)

    # TODO 3.2: undistort the keypoint positions with cv2.undistortPoints.
    # pass P=K, otherwise you get normalized coordinates back instead of pixels.
    # it takes (N, 1, 2), we want (N, 2) at the end.
    raise NotImplementedError('TODO 3.2')

    ij = np.round(raw).astype(int)
    ij[:, 0] = ij[:, 0].clip(0, image.shape[1] - 1)
    ij[:, 1] = ij[:, 1].clip(0, image.shape[0] - 1)
    colors = image[ij[:, 1], ij[:, 0], ::-1]
    return Features(pixels, raw, descriptors, sizes, colors)


def ratio_test(desc_a, desc_b, ratio=0.75):
    # returns {index in a: index in b} for the matches that pass the ratio test
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    # TODO 3.3: matcher.knnMatch(desc_a, desc_b, k=2) gives you the two closest
    # matches for each descriptor (closest first). each one has .queryIdx,
    # .trainIdx and .distance. keep the closest if it's less than ratio times
    # the second closest. (sometimes you only get one back, watch out for that)
    raise NotImplementedError('TODO 3.3')


def match(fa, fb, ratio=0.75):
    # ratio test both ways, and only keep matches where both points pick each other
    if len(fa.descriptors) < 2 or len(fb.descriptors) < 2:
        return np.zeros((0, 2), int)
    ab = ratio_test(fa.descriptors, fb.descriptors, ratio)
    ba = ratio_test(fb.descriptors, fa.descriptors, ratio)
    pairs = [(i, j) for i, j in ab.items() if ba.get(j) == i]
    return np.array(pairs, int).reshape(-1, 2)


def draw_keypoints(image, f):
    out = image.copy()
    for (x, y), s in zip(f.raw_pixels, f.sizes):
        cv2.circle(out, (int(round(x)), int(round(y))), max(int(s / 2), 2), (0, 255, 0), 1, cv2.LINE_AA)
    return out


def side_by_side(img_a, img_b, gap=10):
    h = max(img_a.shape[0], img_b.shape[0])
    canvas = np.full((h, img_a.shape[1] + gap + img_b.shape[1], 3), 255, np.uint8)
    canvas[:img_a.shape[0], :img_a.shape[1]] = img_a
    canvas[:img_b.shape[0], img_a.shape[1] + gap:] = img_b
    return canvas, img_a.shape[1] + gap


def draw_matches(img_a, img_b, fa, fb, matches, color=(0, 200, 0)):
    canvas, off = side_by_side(img_a, img_b)
    for i, j in matches:
        pa = tuple(int(v) for v in np.round(fa.raw_pixels[i]))
        pb = (int(round(fb.raw_pixels[j][0])) + off, int(round(fb.raw_pixels[j][1])))
        cv2.line(canvas, pa, pb, color, 1, cv2.LINE_AA)
        cv2.circle(canvas, pa, 3, color, -1)
        cv2.circle(canvas, pb, 3, color, -1)
    return canvas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--image', type=int, help='draw the keypoints for this image')
    ap.add_argument('--pair', type=int, nargs=2, metavar=('I', 'J'), help='draw the matches between I and J')
    args = ap.parse_args()

    views = load_run(args.run_dir)
    K, dist = load_calibration()
    out = run_output_dir(args.run_dir)

    if args.image is not None:
        v = views[args.image]
        f = detect_sift(v.image, K, dist)
        path = os.path.join(out, f'keypoints_{args.image}.png')
        cv2.imwrite(path, draw_keypoints(v.image, f))
        print(f'{v.name}: {len(f.pixels)} keypoints, saved to {path}')
    elif args.pair is not None:
        i, j = args.pair
        fi, fj = detect_sift(views[i].image, K, dist), detect_sift(views[j].image, K, dist)
        m = match(fi, fj)
        path = os.path.join(out, f'matches_{i}_{j}.png')
        cv2.imwrite(path, draw_matches(views[i].image, views[j].image, fi, fj, m))
        print(f'images {i} and {j}: {len(fi.pixels)} and {len(fj.pixels)} keypoints, '
              f'{len(m)} matches, saved to {path}')
    else:
        feats = [detect_sift(v.image, K, dist) for v in views]
        print('keypoints per image:', [len(f.pixels) for f in feats])
        print('matches between neighbors:')
        for i in range(len(views) - 1):
            print(f'  {i:2d} - {i + 1:2d}: {len(match(feats[i], feats[i + 1])):4d}')


if __name__ == '__main__':
    main()
