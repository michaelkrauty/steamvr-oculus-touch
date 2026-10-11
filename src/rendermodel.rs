// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The SteamVR CV1 Touch render models against the tracked LED model: SteamVR draws
//! `oculus_cv1_controller_left/right` at the device pose, the engine tracks the factory LED frame.
//! The constant offset between them is measured by fitting the LEDs (positions and outward
//! normals) onto the body mesh's surface (ICP from 24 starting orientations).

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
    /// LED-to-surface distances after the fit.
    pub rms_m: f64,
    pub max_m: f64,
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
    let (t, _) = icp(leds, m, best, 200);
    let d: Vec<f64> = leds
        .iter()
        .map(|l| {
            (closest_point(m, &(t * nalgebra::Point3::from(l.pos)).coords).0
                - (t * nalgebra::Point3::from(l.pos)).coords)
                .norm()
        })
        .collect();
    Fit {
        render_from_model: t,
        rms_m: (d.iter().map(|x| x * x).sum::<f64>() / d.len().max(1) as f64).sqrt(),
        max_m: d.iter().copied().fold(0.0, f64::max),
    }
}

/// LED-to-surface distances of `leds` placed by `render_from_model` (rms, max).
fn surface_distances(leds: &[Led], m: &Mesh, render_from_model: &Isometry3<f64>) -> (f64, f64) {
    let d: Vec<f64> = leds
        .iter()
        .map(|l| {
            (closest_point(m, &(render_from_model * Point3::from(l.pos)).coords).0
                - (render_from_model * Point3::from(l.pos)).coords)
                .norm()
        })
        .collect();
    (
        (d.iter().map(|x| x * x).sum::<f64>() / d.len().max(1) as f64).sqrt(),
        d.iter().copied().fold(0.0, f64::max),
    )
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
    let right_from = fit(&both, right_mesh).render_from_model;
    // a left LED p: in the right model frame s(M p), in the right render frame right_from(s(M p)),
    // and in the left render frame (the mirror of the right one) M of that
    let t = right_from * s;
    let m = Matrix3::from_diagonal(&Vector3::new(-1.0, 1.0, 1.0));
    let r = m * t.rotation.to_rotation_matrix().into_inner() * m;
    let left_from = Isometry3::from_parts(
        Translation3::from(mx(&t.translation.vector)),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(r)),
    );
    let placed = |leds: &[Led], m: &Mesh, t: Isometry3<f64>| {
        let (rms_m, max_m) = surface_distances(leds, m, &t);
        Fit {
            render_from_model: t,
            rms_m,
            max_m,
        }
    };
    (
        placed(left, left_mesh, left_from),
        placed(right, right_mesh, right_from),
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
