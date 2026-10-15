// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The SteamVR CV1 Touch render models against the tracked LED model: SteamVR draws
//! `oculus_cv1_controller_left/right` at the device pose, the engine tracks the factory LED frame.
//! The constant offset between them is measured by fitting the LEDs (positions and emission
//! directions) to the body mesh: a coarse fit onto the surface (ICP from 24 starting
//! orientations), then a refinement in which each LED is matched to the window its emission ray
//! leaves the mesh through and all LEDs sit a common depth under their windows (`refine`).

use std::path::PathBuf;

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};

use crate::calib::capture::Led;

#[derive(Debug)]
pub struct Mesh {
    pub tris: Vec<[Vector3<f64>; 3]>,
}

pub fn steamvr_body_obj(hand_left: bool) -> PathBuf {
    let side = if hand_left { "left" } else { "right" };
    crate::steamvr::runtime_root()
        .join("resources/rendermodels")
        .join(format!("oculus_cv1_controller_{side}"))
        .join(format!("oculus_cv1_controller_{side}_body.obj"))
}

/// `v x y z` and `f a b c ...` (indices may carry `/vt/vn`; polygons are fanned into triangles).
pub fn parse_obj(text: &str) -> Result<Mesh, String> {
    let mut verts: Vec<Vector3<f64>> = Vec::new();
    let mut tris = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                let c: Vec<f64> = it
                    .take(3)
                    .map(|s| s.parse::<f64>())
                    .collect::<Result<_, _>>()
                    .map_err(|e| format!("line {}: {e}", n + 1))?;
                if c.len() != 3 {
                    return Err(format!("line {}: vertex needs 3 coordinates", n + 1));
                }
                verts.push(Vector3::new(c[0], c[1], c[2]));
            }
            Some("f") => {
                let idx: Vec<usize> = it
                    .map(|s| {
                        s.split('/')
                            .next()
                            .unwrap_or("")
                            .parse::<usize>()
                            .map_err(|e| format!("line {}: {e}", n + 1))
                    })
                    .collect::<Result<_, _>>()?;
                let get = |i: usize| {
                    verts
                        .get(i.wrapping_sub(1))
                        .copied()
                        .ok_or(format!("line {}: face refers to missing vertex {i}", n + 1))
                };
                for k in 1..idx.len().saturating_sub(1) {
                    tris.push([get(idx[0])?, get(idx[k])?, get(idx[k + 1])?]);
                }
            }
            _ => {}
        }
    }
    Ok(Mesh { tris })
}

/// Closest point on triangle (a, b, c) to p (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_on_triangle(
    p: &Vector3<f64>,
    a: &Vector3<f64>,
    b: &Vector3<f64>,
    c: &Vector3<f64>,
) -> Vector3<f64> {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(&ap), ac.dot(&ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a;
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(&bp), ac.dot(&bp));
    if d3 >= 0.0 && d4 <= d3 {
        return *b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(&cp), ac.dot(&cp));
    if d6 >= 0.0 && d5 <= d6 {
        return *c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

/// The closest surface point and that triangle's unit normal (counter-clockwise winding = outward).
pub fn closest_point(m: &Mesh, p: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let mut best = (f64::MAX, Vector3::zeros(), Vector3::z());
    for t in &m.tris {
        let q = closest_on_triangle(p, &t[0], &t[1], &t[2]);
        let d = (q - p).norm_squared();
        if d < best.0 {
            let n = (t[1] - t[0]).cross(&(t[2] - t[0]));
            best = (
                d,
                q,
                if n.norm() > 0.0 {
                    n.normalize()
                } else {
                    Vector3::z()
                },
            );
        }
    }
    (best.1, best.2)
}

pub struct Fit {
    pub render_from_model: Isometry3<f64>,
    /// How far each LED's depth under its window departs from the common depth (rms, max), m.
    pub rms_m: f64,
    pub max_m: f64,
    /// The common depth of the LEDs under the surface, m (the cover over them).
    pub depth_m: f64,
}

/// Where the ray from `p` along `dir` leaves the mesh: the nearest triangle it crosses outward
/// (`dir` along the triangle's outward normal) within `reach` m behind `p` or ahead of it, with the
/// crossing point, the signed depth of `p` under it along `dir`, and the triangle's unit normal.
/// None when the ray leaves through no face (an LED pointing out of the mesh's gaps).
pub fn ray_exit(
    m: &Mesh,
    p: &Vector3<f64>,
    dir: &Vector3<f64>,
    reach: f64,
) -> Option<(Vector3<f64>, f64, Vector3<f64>)> {
    let mut best: Option<(f64, Vector3<f64>, Vector3<f64>)> = None;
    for t in &m.tris {
        let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
        let n = e1.cross(&e2);
        if n.dot(dir) <= 0.0 {
            continue; // entering, or edge-on
        }
        // Möller-Trumbore
        let h = dir.cross(&e2);
        let a = e1.dot(&h);
        if a.abs() < 1e-18 {
            continue;
        }
        let f = 1.0 / a;
        let s = p - t[0];
        let u = f * s.dot(&h);
        if !(-1e-9..=1.0 + 1e-9).contains(&u) {
            continue;
        }
        let q = s.cross(&e1);
        let v = f * dir.dot(&q);
        if v < -1e-9 || u + v > 1.0 + 1e-9 {
            continue;
        }
        let d = f * e2.dot(&q);
        if d < -reach || d > reach {
            continue;
        }
        // the nearest crossing in absolute distance: the LED's own window
        if best.is_none_or(|b| d.abs() < b.0.abs()) {
            best = Some((d, p + dir * d, n.normalize()));
        }
    }
    best.map(|(d, q, n)| (q, d, n))
}

/// How far the LEDs' rays reach to find their windows: generous against the coarse fit's error.
const WINDOW_REACH_M: f64 = 0.02;

/// The coarse fit refined: each LED matched to the window its emission ray leaves the mesh
/// through, and every LED a common depth under its window (the cover, fitted along).
/// Gauss-Newton on pose and depth, point-to-plane at the window. The emission direction only
/// chooses the window: the body meshes are faceted (neighbouring facets of the ring's tube differ
/// by 15 to 30 degrees), so a facet's normal says too little about the window's true normal to
/// pull the pose by. Returns the transform and the depth; the start is kept when fewer than six
/// LEDs find a window.
fn refine(
    leds: &[Led],
    m: &Mesh,
    start: Isometry3<f64>,
    iterations: usize,
) -> (Isometry3<f64>, f64) {
    let mut t = start;
    let mut depth = 0.0;
    for _ in 0..iterations {
        let mut h = nalgebra::SMatrix::<f64, 7, 7>::zeros();
        let mut g = nalgebra::SVector::<f64, 7>::zeros();
        let mut hits = 0usize;
        for l in leds {
            let p = (t * Point3::from(l.pos)).coords;
            let a = t.rotation * l.dir;
            let Some((q, _, n)) = ray_exit(m, &p, &a, WINDOW_REACH_M) else {
                continue;
            };
            hits += 1;
            // depth: p sits `depth` under the window along its normal
            let r = n.dot(&(p - q)) + depth;
            let mut j = nalgebra::SVector::<f64, 7>::zeros();
            let pxn = p.cross(&n);
            j[0] = pxn.x;
            j[1] = pxn.y;
            j[2] = pxn.z;
            j[3] = n.x;
            j[4] = n.y;
            j[5] = n.z;
            j[6] = 1.0;
            h += j * j.transpose();
            g -= j * r;
        }
        if hits < 6 {
            return (start, 0.0);
        }
        for k in 0..7 {
            h[(k, k)] += 1e-9 + 1e-6 * h[(k, k)];
        }
        let Some(d) = h.cholesky().map(|ch| ch.solve(&g)) else {
            break;
        };
        let step = Isometry3::from_parts(
            Translation3::new(d[3], d[4], d[5]),
            UnitQuaternion::from_scaled_axis(Vector3::new(d[0], d[1], d[2])),
        );
        t = step * t;
        depth = (depth + d[6]).max(0.0);
        if d.norm() < 1e-10 {
            break;
        }
    }
    (t, depth)
}

/// The fit's figures for `leds` placed by `render_from_model` with the LEDs `depth` under their
/// windows: depth deviation rms and max (m). An LED whose ray leaves through no face is measured
/// against the nearest surface point instead.
fn measure(leds: &[Led], m: &Mesh, render_from_model: &Isometry3<f64>, depth: f64) -> (f64, f64) {
    let (mut d2, mut dmax, mut n) = (0.0, 0.0, 0usize);
    for l in leds {
        let p = (render_from_model * Point3::from(l.pos)).coords;
        let a = render_from_model.rotation * l.dir;
        let (q, nn) = match ray_exit(m, &p, &a, WINDOW_REACH_M) {
            Some((q, _, nn)) => (q, nn),
            None => closest_point(m, &p),
        };
        let dev = nn.dot(&(p - q)) + depth;
        d2 += dev * dev;
        dmax = f64::max(dmax, dev.abs());
        n += 1;
    }
    let n = n.max(1) as f64;
    ((d2 / n).sqrt(), dmax)
}

/// One ICP run from `start`: Gauss-Newton on point-to-plane distances (an LED may slide along the
/// surface; point-to-point ICP crawls on smooth tubes). Returns the transform and its cost: mean
/// squared LED-to-surface distance, with an LED whose normal points into the surface charged as
/// if 5 mm off.
fn icp(leds: &[Led], m: &Mesh, start: Isometry3<f64>, iterations: usize) -> (Isometry3<f64>, f64) {
    let mut t = start;
    let mut cost = f64::MAX;
    for _ in 0..iterations {
        let mut h = nalgebra::Matrix6::<f64>::zeros();
        let mut g = nalgebra::Vector6::<f64>::zeros();
        let mut c = 0.0;
        for l in leds {
            let p = (t * nalgebra::Point3::from(l.pos)).coords;
            let (q, n) = closest_point(m, &p);
            let facing = (t.rotation * l.dir).dot(&n) > 0.0;
            let w = if facing { 1.0 } else { 0.05 };
            c += if facing {
                (q - p).norm_squared()
            } else {
                0.005f64.powi(2)
            };
            let r = n.dot(&(p - q));
            let j = nalgebra::Vector6::new(
                p.y * n.z - p.z * n.y,
                p.z * n.x - p.x * n.z,
                p.x * n.y - p.y * n.x,
                n.x,
                n.y,
                n.z,
            );
            h += j * j.transpose() * w;
            g -= j * (r * w);
        }
        cost = c / leds.len() as f64;
        for k in 0..6 {
            h[(k, k)] += 1e-9 + 1e-6 * h[(k, k)];
        }
        let Some(d) = h.cholesky().map(|ch| ch.solve(&g)) else {
            break;
        };
        let step = Isometry3::from_parts(
            Translation3::new(d[3], d[4], d[5]),
            UnitQuaternion::from_scaled_axis(Vector3::new(d[0], d[1], d[2])),
        );
        t = step * t;
        if d.norm() < 1e-10 {
            break;
        }
    }
    (t, cost)
}

pub fn fit(leds: &[Led], m: &Mesh) -> Fit {
    let centroid = |pts: &mut dyn Iterator<Item = Vector3<f64>>| {
        let (s, n) = pts.fold((Vector3::zeros(), 0usize), |(s, n), p| (s + p, n + 1));
        s / n.max(1) as f64
    };
    let c_mesh = centroid(&mut m.tris.iter().map(|t| (t[0] + t[1] + t[2]) / 3.0));
    let c_leds = centroid(&mut leds.iter().map(|l| l.pos));
    // 24 orientations of a cube, as starting points
    let axes = [
        Vector3::x(),
        -Vector3::x(),
        Vector3::y(),
        -Vector3::y(),
        Vector3::z(),
        -Vector3::z(),
    ];
    let mut starts = Vec::new();
    for a in axes {
        let base = UnitQuaternion::rotation_between(&Vector3::z(), &a).unwrap_or_else(|| {
            UnitQuaternion::from_scaled_axis(Vector3::x() * std::f64::consts::PI)
        });
        for k in 0..4 {
            let r = base
                * UnitQuaternion::from_scaled_axis(
                    Vector3::z() * (k as f64 * std::f64::consts::FRAC_PI_2),
                );
            starts.push(Isometry3::from_parts(
                Translation3::from(c_mesh - r * c_leds),
                r,
            ));
        }
    }
    let (mut best, mut best_cost) = (starts[0], f64::MAX);
    for s in starts {
        let (t, c) = icp(leds, m, s, 15);
        if c < best_cost {
            (best, best_cost) = (t, c);
        }
    }
    let (coarse, _) = icp(leds, m, best, 200);
    let (t, depth) = refine(leds, m, coarse, 60);
    placed(leds, m, t, depth)
}

/// The fit of `leds` placed by `render_from_model` with the LEDs `depth` under their windows.
fn placed(leds: &[Led], m: &Mesh, render_from_model: Isometry3<f64>, depth: f64) -> Fit {
    let (rms_m, max_m) = measure(leds, m, &render_from_model, depth);
    Fit {
        render_from_model,
        rms_m,
        max_m,
        depth_m: depth,
    }
}

/// The rigid move best taking `a` onto `b` (same order).
fn kabsch(a: &[Vector3<f64>], b: &[Vector3<f64>]) -> Isometry3<f64> {
    let n = a.len().max(1) as f64;
    let (ca, cb) = (
        a.iter().sum::<Vector3<f64>>() / n,
        b.iter().sum::<Vector3<f64>>() / n,
    );
    let h: Matrix3<f64> = a
        .iter()
        .zip(b)
        .map(|(x, y)| (y - cb) * (x - ca).transpose())
        .sum();
    let svd = h.svd(true, true);
    let (u, vt) = (
        svd.u.unwrap_or_else(Matrix3::identity),
        svd.v_t.unwrap_or_else(Matrix3::identity),
    );
    let d = if (u * vt).determinant() < 0.0 {
        -1.0
    } else {
        1.0
    };
    let r = u * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d)) * vt;
    Isometry3::from_parts(
        Translation3::from(cb - r * ca),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(r)),
    )
}

/// Both hands at once. The left and right bodies are mirror images (x -> -x), and so, to about a
/// millimetre, are the two LED layouts (same LED index, mirrored). Fitted alone, a hand can settle
/// several degrees off in a minimum that fits its LEDs almost as well (the left Touch did, by
/// 8 degrees, drawing its handle visibly off). Here the right mesh is fitted to the right LEDs
/// and the left ones mirrored onto them, and the left placement is that one mirrored back: one
/// fit, and the two hands drawn consistently. Layouts that do not correspond fall back to fitting
/// each hand alone.
pub fn fit_mirrored(
    left: &[Led],
    right: &[Led],
    left_mesh: &Mesh,
    right_mesh: &Mesh,
) -> (Fit, Fit) {
    let mx = |v: &Vector3<f64>| Vector3::new(-v.x, v.y, v.z);
    let alone = || (fit(left, left_mesh), fit(right, right_mesh));
    if left.len() != right.len() || left.len() < 6 {
        return alone();
    }
    let lm: Vec<Vector3<f64>> = left.iter().map(|l| mx(&l.pos)).collect();
    let rp: Vec<Vector3<f64>> = right.iter().map(|l| l.pos).collect();
    // right model <- mirrored left model
    let s = kabsch(&lm, &rp);
    let misfit = (lm
        .iter()
        .zip(&rp)
        .map(|(a, b)| (s * Point3::from(*a) - Point3::from(*b)).norm_squared())
        .sum::<f64>()
        / lm.len() as f64)
        .sqrt();
    if misfit > 0.003 {
        return alone();
    }
    let both: Vec<Led> = right
        .iter()
        .cloned()
        .chain(left.iter().map(|l| Led {
            pos: (s * Point3::from(mx(&l.pos))).coords,
            dir: s.rotation * mx(&l.dir),
        }))
        .collect();
    let right_fit = fit(&both, right_mesh);
    let right_from = right_fit.render_from_model;
    // a left LED p: in the right model frame s(M p), in the right render frame right_from(s(M p)),
    // and in the left render frame (the mirror of the right one) M of that
    let t = right_from * s;
    let m = Matrix3::from_diagonal(&Vector3::new(-1.0, 1.0, 1.0));
    let r = m * t.rotation.to_rotation_matrix().into_inner() * m;
    let left_from = Isometry3::from_parts(
        Translation3::from(mx(&t.translation.vector)),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(r)),
    );
    (
        placed(left, left_mesh, left_from, right_fit.depth_m),
        placed(right, right_mesh, right_from, right_fit.depth_m),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    /// An asymmetric closed body: a box with a torus on one end (a stand-in for the ring), triangulated.
    fn body() -> Mesh {
        let mut tris = Vec::new();
        // box 0.03 x 0.1 x 0.04, centred at (0, -0.05, 0)
        let (hx, hy, hz, cy) = (0.015, 0.05, 0.02, -0.05);
        let v = |sx: f64, sy: f64, sz: f64| Vector3::new(sx * hx, cy + sy * hy, sz * hz);
        let faces = [
            [(-1., -1., 1.), (1., -1., 1.), (1., 1., 1.), (-1., 1., 1.)],
            [
                (1., -1., -1.),
                (-1., -1., -1.),
                (-1., 1., -1.),
                (1., 1., -1.),
            ],
            [(1., -1., 1.), (1., -1., -1.), (1., 1., -1.), (1., 1., 1.)],
            [
                (-1., -1., -1.),
                (-1., -1., 1.),
                (-1., 1., 1.),
                (-1., 1., -1.),
            ],
            [(-1., 1., 1.), (1., 1., 1.), (1., 1., -1.), (-1., 1., -1.)],
            [
                (-1., -1., -1.),
                (1., -1., -1.),
                (1., -1., 1.),
                (-1., -1., 1.),
            ],
        ];
        for f in faces {
            let q: Vec<Vector3<f64>> = f.iter().map(|&(a, b, c)| v(a, b, c)).collect();
            tris.push([q[0], q[1], q[2]]);
            tris.push([q[0], q[2], q[3]]);
        }
        // ring in the x-z plane centred at (0.02, 0.01, 0): tube radius 8 mm around a centreline
        // whose radius varies (no rotational or mirror symmetry, unlike a circular torus)
        let (n, m) = (RING, TUBE);
        let p = |i: usize, j: usize| ring_point(i, j);
        for i in 0..n {
            for j in 0..m {
                let (a, b, c, d) = (p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1));
                // wound so (v1 - v0) x (v2 - v0) points out of the tube
                tris.push([a, c, b]);
                tris.push([a, d, c]);
            }
        }
        Mesh { tris }
    }

    const RING: usize = 48;
    const TUBE: usize = 12;

    fn ring_point(i: usize, j: usize) -> Vector3<f64> {
        let (u, w) = (
            i as f64 * std::f64::consts::TAU / RING as f64,
            j as f64 * std::f64::consts::TAU / TUBE as f64,
        );
        let r = 0.045 + 0.008 * u.cos() + 0.004 * (2.0 * u).sin() + 0.008 * w.cos();
        Vector3::new(0.02 + r * u.cos(), 0.01 + 0.008 * w.sin(), r * u.sin())
    }

    /// LEDs at mesh vertices along the outer equator and the upper side of 80% of the ring,
    /// with outward normals (so they lie exactly on the triangulated surface).
    fn leds() -> Vec<Led> {
        (0..20)
            .map(|k| {
                let (i, j) = (2 * k, if k % 2 == 0 { 0 } else { 2 });
                let (u, w) = (
                    i as f64 * std::f64::consts::TAU / RING as f64,
                    j as f64 * std::f64::consts::TAU / TUBE as f64,
                );
                let dir = Vector3::new(w.cos() * u.cos(), w.sin(), w.cos() * u.sin());
                Led {
                    pos: ring_point(i, j),
                    dir,
                }
            })
            .collect()
    }

    #[test]
    fn parses_obj_with_slashes_and_polygons() {
        let text = "# test\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nvn 0 0 1\nf 1/1/1 2/1/1 3/1/1 4/1/1\nf 1 3 4\n";
        let m = parse_obj(text).unwrap();
        assert_eq!(m.tris.len(), 3);
        assert_eq!(m.tris[0][2], Vector3::new(1.0, 1.0, 0.0));
        assert!(parse_obj("f 1 2 3\n").unwrap_err().contains("vertex"));
    }

    #[test]
    fn closest_point_on_a_face_and_its_normal() {
        let m = body();
        let (q, nrm) = closest_point(&m, &Vector3::new(0.0, -0.05, 0.05));
        assert!((q - Vector3::new(0.0, -0.05, 0.02)).norm() < 1e-9, "{q:?}");
        assert!((nrm - Vector3::z()).norm() < 1e-9, "{nrm:?}");
    }

    /// The body mirrored x -> -x, wound so its faces still point out.
    fn mirrored(m: &Mesh) -> Mesh {
        let mx = |v: &Vector3<f64>| Vector3::new(-v.x, v.y, v.z);
        Mesh {
            tris: m
                .tris
                .iter()
                .map(|t| [mx(&t[0]), mx(&t[2]), mx(&t[1])])
                .collect(),
        }
    }

    /// The two hands' bodies are mirror images and so are their LED layouts, each in its own model
    /// frame: fitted together, both come back where they are.
    #[test]
    fn a_mirrored_pair_is_fitted_together() {
        let right = body();
        let left = mirrored(&right);
        let truth_r = Isometry3::from_parts(
            Translation3::new(0.004, -0.012, 0.007),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.9, -2.1, 0.4)),
        );
        let truth_l = Isometry3::from_parts(
            Translation3::new(-0.009, 0.003, 0.011),
            UnitQuaternion::from_scaled_axis(Vector3::new(-0.4, 1.3, 0.2)),
        );
        let mx = |v: Vector3<f64>| Vector3::new(-v.x, v.y, v.z);
        let model_r: Vec<Led> = leds()
            .iter()
            .map(|l| Led {
                pos: truth_r.inverse_transform_point(&l.pos.into()).coords,
                dir: truth_r.rotation.inverse() * l.dir,
            })
            .collect();
        let model_l: Vec<Led> = leds()
            .iter()
            .map(|l| Led {
                pos: truth_l.inverse_transform_point(&mx(l.pos).into()).coords,
                dir: truth_l.rotation.inverse() * mx(l.dir),
            })
            .collect();
        let (fl, fr) = fit_mirrored(&model_l, &model_r, &left, &right);
        for (f, truth, hand) in [(fl, truth_l, "left"), (fr, truth_r, "right")] {
            let err = f.render_from_model * truth.inverse();
            assert!(
                err.translation.vector.norm() < 1e-4 && err.rotation.angle() < 0.1f64.to_radians(),
                "{hand}: {:?} {}",
                err.translation.vector,
                err.rotation.angle().to_degrees()
            );
            assert!(f.rms_m < 1e-4, "{hand}: {}", f.rms_m);
        }
    }

    /// A ray from inside the box leaves through the face it points at, at the right depth.
    #[test]
    fn a_ray_leaves_the_mesh_through_the_face_it_points_at() {
        let m = body();
        let p = Vector3::new(0.0, -0.05, 0.019);
        let (q, depth, n) = ray_exit(&m, &p, &Vector3::z(), 0.01).unwrap();
        assert!((q - Vector3::new(0.0, -0.05, 0.02)).norm() < 1e-9, "{q:?}");
        assert!((depth - 0.001).abs() < 1e-9, "{depth}");
        assert!((n - Vector3::z()).norm() < 1e-9, "{n:?}");
        // pointing along the face, it leaves through nothing within reach
        assert!(ray_exit(&m, &p, &Vector3::x(), 0.01).is_none());
    }

    /// LEDs under a cover: 1.5 mm inside the surface, each shining along its window's normal. The
    /// fit finds the pose and the depth; a fit of the LEDs onto the surface itself would tilt the
    /// asymmetric ring to bring them up.
    #[test]
    fn recovers_a_known_offset_with_recessed_leds() {
        let m = body();
        let truth = Isometry3::from_parts(
            Translation3::new(0.004, -0.012, 0.007),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.9, -2.1, 0.4)),
        );
        let model: Vec<Led> = leds()
            .iter()
            .map(|l| Led {
                pos: truth
                    .inverse_transform_point(&(l.pos - l.dir * 0.0015).into())
                    .coords,
                dir: truth.rotation.inverse() * l.dir,
            })
            .collect();
        let f = fit(&model, &m);
        let err = f.render_from_model * truth.inverse();
        assert!(
            err.translation.vector.norm() < 3e-4 && err.rotation.angle() < 0.2f64.to_radians(),
            "{:?} {}",
            err.translation.vector,
            err.rotation.angle().to_degrees()
        );
        assert!((f.depth_m - 0.0015).abs() < 3e-4, "{}", f.depth_m);
        assert!(f.rms_m < 3e-4, "{}", f.rms_m);
    }

    #[test]
    fn recovers_a_known_offset() {
        let m = body();
        let truth = Isometry3::from_parts(
            Translation3::new(0.004, -0.012, 0.007),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.9, -2.1, 0.4)),
        );
        // LEDs expressed in a "model" frame: render_from_model = truth
        let model: Vec<Led> = leds()
            .iter()
            .map(|l| Led {
                pos: truth.inverse_transform_point(&l.pos.into()).coords,
                dir: truth.rotation.inverse() * l.dir,
            })
            .collect();
        let f = fit(&model, &m);
        let err = f.render_from_model * truth.inverse();
        assert!(
            err.translation.vector.norm() < 1e-4 && err.rotation.angle() < 0.1f64.to_radians(),
            "{:?} {}",
            err.translation.vector,
            err.rotation.angle().to_degrees()
        );
        assert!(f.rms_m < 1e-4, "{}", f.rms_m);
    }
}
