#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate a fictional near-half-turn fixture without OpenCV."""
import math
import struct

def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]

def project(q, t, point):
    n = math.sqrt(sum(v*v for v in q))
    x, y, z, w = (v/n for v in q)
    # Quaternion-vector product followed by multiplication by its conjugate.
    a, b, c = point
    u = w*a + y*c - z*b
    v = w*b + z*a - x*c
    h = w*c + x*b - y*a
    s = -x*a - y*b - z*c
    X = -s*x + u*w - v*z + h*y + t[0]
    Y = -s*y + u*z + v*w - h*x + t[1]
    Z = -s*z - u*y + v*x + h*w + t[2]
    X, Y = X/Z, Y/Z
    r = math.hypot(X, Y)
    theta = math.atan(r)
    k = (-.08, .012, .001, -.0005)
    theta2 = theta*theta
    factor = theta*(1 + theta2*(k[0] + theta2*(k[1] + theta2*(k[2] + theta2*k[3]))))/r if r else 1
    return (613*X*factor + 641, 587*Y*factor + 479)

def vals(xs):
    return ', '.join(float(x).hex() for x in xs)

q = tuple(map(f32, (.3, -.4, .8660254, .0000045)))
t = tuple(map(f32, (.02, -.01, .12)))
point = tuple(map(f32, (.12, .09, 0)))
pixel = project(q, t, point)
print('/* SPDX-License-Identifier: Apache-2.0 */')
print('static const struct projection_case exact_half_turn =')
print('    { 1, {%s}, {%s}, {%s}, {%s} };' % (vals(q), vals(t), vals(point), vals(pixel)))
