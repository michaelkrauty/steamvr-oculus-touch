#!/usr/bin/env python3
"""Generate fictional pose fixtures; requires opencv-python-headless 4.12.0.88.

Compile support/pose_reference.cpp against OpenCV 5.1, then pass its executable
as --opencv-5-reference. The checked-in output is consumed without OpenCV.
Both references receive identical float object/image points and float priors.
"""
import argparse
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np


def quaternion(r):
    angle = np.linalg.norm(r)
    return np.asarray([*(r * (np.sin(angle / 2) / angle if angle else 0.5)),
                       np.cos(angle / 2)], dtype=np.float32)


def cases():
    rng = np.random.default_rng(29741)
    model = rng.uniform([-.09, -.07, -.035], [.09, .07, .035], (64, 3)).astype(np.float32)
    K = np.array([[580., 0, 320.], [0, 597., 240.], [0, 0, 1.]])
    for lens in range(2):
        D = np.array([-.045, .006, -.0003, .00002, .0001])
        planar_pixels = None
        for name, n, noise, outliers in [
            ('four', 4, 0, 0), ('four-ambiguous', 4, 0, 0),
            ('five', 5, 0, 0), ('five-noisy', 5, .35, 0), ('many', 16, 0, 0),
            ('four-outlier', 4, 0, 1), ('five-outlier', 5, 0, 1),
            ('boundary64', 64, .12, 0), ('noisy', 16, .35, 0),
            ('mislabelled', 16, .2, 3), ('outliers', 16, .2, 6),
            ('zero', 16, 0, 0), ('tiny', 16, 0, 0), ('pi', 16, .1, 0),
            ('distant-prior', 16, .25, 0), ('opposite-prior', 16, .25, 0),
            ('near-planar-front', 12, .12, 0), ('near-planar-back', 12, .12, 0),
            ('collinear', 16, 0, 0), ('repeated', 16, 0, 0),
            ('split', 24, 0, 0), ('behind', 12, .1, 0),
        ]:
            obj = model[:n].copy()
            if name == 'four':
                obj[:] = [[-.12, -.10, -.08], [.12, -.08, .08],
                          [-.1, .1, .08], [.12, .1, -.08]]
            r = np.array([.17, -.24, .09])
            t = np.array([.035, -.02, .85])
            if name == 'zero': r[:] = 0
            if name == 'tiny': r[:] = [1e-9, -2e-9, 3e-9]
            if name == 'pi': r[:] = [np.pi, 0, 0]
            if name.startswith('near-planar'):
                obj[:, 2] *= .0005
                r[:] = [.065, -.08, .01]
                t[:] = [.01, -.015, 1.8]
            if name == 'collinear':
                obj[:, 0] = np.arange(n, dtype=np.float32) / 100
                obj[:, 1:] = 0
            if name == 'repeated': obj[:] = obj[0]
            # Two points behind the camera that still project onto their
            # observations: the baseline's inlier test has no depth condition.
            if name == 'behind': obj[:2, 2] = -3
            project = cv2.fisheye.projectPoints if lens else cv2.projectPoints
            pixels = project(obj.reshape(-1, 1, 3), r, t, K, D[:4] if lens else D)[0]
            # Two equally large consensus sets: the first one found must win.
            if name == 'split':
                pixels[12:] = project(obj[12:].reshape(-1, 1, 3), r + [.9, -.6, .4], t + [.15, .1, .3],
                                     K, D[:4] if lens else D)[0]
            pixels = (pixels.reshape(-1, 2) + rng.normal(0, noise, (n, 2))).astype(np.float32)
            if name == 'near-planar-front': planar_pixels = pixels.copy()
            if name == 'near-planar-back': pixels = planar_pixels.copy()
            # Move the observations, not model geometry, to simulate wrong labels.
            pixels[:outliers] += np.array([35., -27.], dtype=np.float32)
            guess_r = r + np.array([.025, -.018, .01])
            if name == 'near-planar-back': guess_r = np.array([-.11, .13, .01])
            guess_t = (t + np.array([.004, -.003, .018])).astype(np.float32)
            if name == 'distant-prior':
                guess_r = np.array([.7, -.9, .4])
                guess_t = np.array([2., -1., 10.], dtype=np.float32)
            if name == 'opposite-prior':
                guess_r = np.array([2.5, -.8, .4])
                guess_t = np.array([.1, -.05, .35], dtype=np.float32)
            if name in ('zero', 'tiny'): guess_r = np.zeros(3)
            for guess in (False, True):
                if name.startswith('near-planar') and not guess: continue
                if name.endswith('prior') and not guess: continue
                yield dict(name=f'{name}-lens{lens}-guess{int(guess)}', lens=lens,
                           obj=obj, pixels=pixels, K=K, D=D,
                           q=quaternion(guess_r), t=guess_t, guess=guess)


def initial_vector(q):
    x, y, z, w = np.asarray(q, dtype=np.float64) / np.linalg.norm(q.astype(np.float64))
    R = np.array([[1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y],
                  [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
                  [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y]])
    return cv2.Rodrigues(R)[0]


def reference(c):
    undistort = cv2.fisheye.undistortPoints if c['lens'] else cv2.undistortPoints
    u = undistort(c['pixels'].reshape(-1, 1, 2), c['K'], c['D'][:4] if c['lens'] else c['D'])
    r, t = initial_vector(c['q']), c['t'].astype(np.float64).reshape(3, 1)
    ok, rr, tt, inliers = cv2.solvePnPRansac(
        c['obj'], u, np.eye(3), np.zeros(4), r.copy(), t.copy(), c['guess'],
        iterationsCount=100, reprojectionError=float(np.float32(3 / c['K'][0, 0])),
        confidence=float(np.float32(.99)), flags=cv2.SOLVEPNP_ITERATIVE)
    return dict(baseline=[int(ok), 0 if inliers is None else len(inliers)],
                baseline_pose=[*rr.ravel(), *tt.ravel()] if ok else [0.] * 6)


# Exactly representable matrices whose decompositions need few roundings: rank
# deficient ones exercise the order of singular values and the fixed random
# completion of the left basis.
SVD_CASES = [
    [[1, 1, 0], [1, 1, 0], [0, 0, 0]],
    [[2, 0, 0], [0, 0, 0], [0, 0, 1]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[4, 1, 0, 2], [1, 3, 1, 0], [0, 1, 2, 1], [2, 0, 1, 5], [1, 1, 1, 1], [0, 2, 0, 1]],
    [[1, 2, 0, 0], [2, 4, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [1, 2, 0, 0], [0, 0, 0, 0]],
]


def svd_fixture(a):
    a = np.asarray(a, dtype=np.float64)
    w, u, vt = cv2.SVDecomp(a)
    m, n = a.shape
    return ('    {' + ','.join([str(m), str(n), array(np.pad(a.ravel(), (0, 24 - m * n))),
            array(np.pad(w.ravel(), (0, 4 - n))), array(np.pad(u.T.ravel(), (0, 24 - m * n))),
            array(np.pad(vt.ravel(), (0, 16 - n * n)))]) + '},')


def serial(c):
    fields = [len(c['obj']), c['lens'], int(c['guess'])]
    fields += [*c['K'].ravel(), *c['D'], *c['q'], *c['t']]
    fields += [*c['obj'].ravel(), *c['pixels'].ravel()]
    return ' '.join(format(float(x), '.17g') for x in fields) + '\n'


def array(a):
    if isinstance(a, (list, tuple, np.ndarray)):
        return '{' + ','.join(array(x) for x in a) + '}'
    return format(float(a), '.17g')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--opencv-5-reference', required=True)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'support/pose_fixtures.inc')
    args = parser.parse_args()
    assert cv2.__version__ == '4.12.0', cv2.__version__
    data = list(cases())
    result = subprocess.run([args.opencv_5_reference], input=''.join(serial(c) for c in data),
                            text=True, capture_output=True, check=True)
    lines = result.stdout.splitlines()
    assert lines.pop(0) == 'OpenCV 5.1.0'
    refs5 = [json.loads(line) for line in lines]
    assert len(refs5) == len(data)
    text = ['/* SPDX-License-Identifier: Apache-2.0',
            ' * Generated by generate_pose_fixtures.py; fictional inputs only.',
            ' * Baseline fields: solvePnPRansac success, inlier count and pose from',
            ' * OpenCV 4.12.0.88 and OpenCV 5.1.0. SVD fields: cv::SVDecomp from 4.12. */',
            'static const struct pose_fixture pose_fixtures[] = {']
    for c, ref5 in zip(data, refs5):
        ref4 = reference(c)
        text += ['    {"' + c['name'] + '", ' + ','.join([
            str(len(c['obj'])), str(c['lens']), str(int(c['guess'])),
            array(c['K'].ravel()), array(c['D']), array(c['q']),
            array(c['t']), array(c['obj']), array(c['pixels']),
            array([ref4['baseline'], ref5['baseline']]),
            array([ref4['baseline_pose'], ref5['baseline_pose']])]) + '},']
    text += ['};', 'static const struct svd_fixture svd_fixtures[] = {']
    text += [svd_fixture(a) for a in SVD_CASES]
    text += ['};', '']
    args.output.write_text('\n'.join(text))


if __name__ == '__main__':
    main()
