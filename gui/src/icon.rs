// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The tray and window icon: a right Touch controller seen straight-on from the front with the
//! button faceplate level with the horizon, black/grey matte hardware with studio rim highlights,
//! visible thumbstick, buttons, index trigger and grip paddle, and the tracking ring in the
//! status colour. Drawn from signed distance fields.

/// The status colour of the ring.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Tint {
    Ready,
    Warning,
    Stopped,
}

/// Square sizes offered to the desktop, which picks the closest one.
#[cfg(target_os = "linux")]
pub const SIZES: [usize; 4] = [16, 24, 32, 64];

type Rgb = [f32; 3];
const OUTLINE: Rgb = rgb(0x0c, 0x0f, 0x14);
const RECESS: Rgb = rgb(0x0a, 0x0d, 0x12);
const BUTTON_SIDE: Rgb = [0.16, 0.18, 0.22];

const fn rgb(r: u8, g: u8, b: u8) -> Rgb {
    [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0]
}

impl Tint {
    fn rgb(self) -> Rgb {
        match self {
            Self::Ready => rgb(0x2e, 0xc8, 0x78),
            Self::Warning => rgb(0xf2, 0xab, 0x28),
            Self::Stopped => rgb(0x9a, 0xa8, 0xba),
        }
    }
}

fn length(x: f32, y: f32) -> f32 {
    x.hypot(y)
}

fn mix(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Smooth minimum for organic plastic fillets.
fn smin(a: f32, b: f32, k: f32) -> f32 {
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    mix(b, a, h) - k * h * (1.0 - h)
}

/// Smooth maximum for blending a cut into the ring.
fn smax(a: f32, b: f32, k: f32) -> f32 {
    -smin(-a, -b, k)
}

/// Cool charcoal/grey shade at luminance-like value `v`.
fn grey(v: f32) -> Rgb {
    [
        (v * 0.92).clamp(0.0, 1.0),
        (v * 0.96).clamp(0.0, 1.0),
        (v * 1.04).clamp(0.0, 1.0),
    ]
}

/// An axis-aligned ellipse centred on (`cx`, `cy`) with radii `a`, `b`.
fn ellipse((x, y): (f32, f32), (cx, cy): (f32, f32), a: f32, b: f32) -> f32 {
    (length((x - cx) / a, (y - cy) / b) - 1.0) * a.min(b)
}

/// An ellipse rotated clockwise by `deg` degrees (y down).
fn rot_ellipse((x, y): (f32, f32), (cx, cy): (f32, f32), a: f32, b: f32, deg: f32) -> f32 {
    let (s, c) = deg.to_radians().sin_cos();
    let (dx, dy) = (x - cx, y - cy);
    let rx = dx * c + dy * s;
    let ry = -dx * s + dy * c;
    (length(rx / a, ry / b) - 1.0) * a.min(b)
}

/// A capsule from `p0` (radius `r0`) to `p1` (radius `r1`).
fn capsule((x, y): (f32, f32), p0: (f32, f32), p1: (f32, f32), r0: f32, r1: f32) -> f32 {
    let (px, py, bx, by) = (x - p0.0, y - p0.1, p1.0 - p0.0, p1.1 - p0.1);
    let h = ((px * bx + py * by) / (bx * bx + by * by)).clamp(0.0, 1.0);
    length(px - bx * h, py - by * h) - (r0 + (r1 - r0) * h)
}

/// One circular faceplate button: dark vertical barrel plus lit grey top cap.
fn button(p: (f32, f32), (bx, by): (f32, f32), rx: f32, ry: f32, h: f32) -> [(f32, Rgb, bool); 2] {
    let barrel = capsule(p, (bx, by), (bx, by + h), rx, rx);
    let cap = ellipse(p, (bx, by), rx, ry);
    let u = ((p.0 - bx) / rx).clamp(-1.0, 1.0);
    let v = ((p.1 - by) / ry).clamp(-1.0, 1.0);
    let lit = (0.72 - 0.09 * u - 0.09 * v).clamp(0.60, 0.82);
    let cap_col = [
        (lit * 0.94).clamp(0.0, 1.0),
        (lit * 0.97).clamp(0.0, 1.0),
        (lit * 1.04).clamp(0.0, 1.0),
    ];
    [(barrel, BUTTON_SIDE, false), (cap, cap_col, false)]
}

/// The painted layers, back to front, at `p` in [-1, 1]² (y down): each a signed distance
/// (negative inside), its fill colour, and whether it gets the dark outline `w` wide.
fn layers(p: (f32, f32), ring_base: Rgb, w: f32) -> [(f32, Rgb, bool); 17] {
    let (x, y) = p;
    let s = (w - 0.036).max(0.0);
    let k = (w * 1.15 - 0.070).max(0.0);
    let rim_w = (w * 1.75).max(0.075);
    let rim_hi = if w > 0.06 { 0.74 } else { 0.68 };

    // 1. Tracking ring: arches up-left above the horizontal faceplate, sweeps around the left,
    // bottom and right, and joins the faceplate on both sides while leaving open air above the
    // faceplate for the thumbstick and buttons.
    let ring_outer = rot_ellipse(p, (-0.04, 0.06), 0.87, 0.80, -22.0);
    let ring_inner = rot_ellipse(p, (0.03, 0.07), 0.71 - k, 0.65 - k, -22.0);
    let ring_band = ring_outer.max(-ring_inner);
    let top_cut = ellipse(p, (0.45, -0.75), 0.48, 0.34);
    let arch_bridge = capsule(
        p,
        (-0.28, -0.64),
        (-0.04, -0.43),
        0.095 + 0.38 * k,
        0.095 + 0.38 * k,
    );
    let ring_d = smin(smax(ring_band, -top_cut, 0.03), arch_bridge, 0.04);

    let ring_half = 0.095 + 0.5 * k;
    let ring_u = ((ring_d + ring_half) / ring_half).clamp(-1.0, 1.0);
    let ang = (y - 0.06).atan2(x + 0.04);
    let ring_dir = 0.5 + 0.5 * (ang + 2.2).cos();
    let ring_round = (1.0 - ring_u * ring_u * 0.40).max(0.35).sqrt();
    let ring_lit = (0.78 + 0.24 * ring_round + 0.14 * ring_dir * ring_round).clamp(0.72, 1.14);
    let spec = (ring_lit - 1.0).max(0.0) * 0.35;
    let ring_col = ring_base.map(|c| (c * ring_lit + spec).clamp(0.0, 1.0));

    // 2. Side grip paddle recessed into the right-inner face of the handle below the trigger.
    let grip_d = capsule(
        p,
        (0.31, 0.03),
        (0.28, 0.26 - 1.5 * s),
        0.088 - 0.25 * s,
        0.072 - 0.25 * s,
    );
    let grip_recess_d = grip_d - 0.022;
    let g_edge = smoothstep(-(w * 1.5).max(0.068), -0.004, grip_d);
    let g_u = ((x - 0.22) / 0.18).clamp(0.0, 1.0);
    let g_val = mix(0.38 + 0.24 * g_u, rim_hi, g_edge * 0.92).clamp(0.34, 0.78);
    let grip_col = grey(g_val);

    // 3. Matte charcoal handle hanging inside the ring with a studio rim highlight.
    let neck = capsule(p, (0.12, -0.39), (0.44, -0.37), 0.19, 0.20);
    let shaft = capsule(
        p,
        (0.16, -0.33),
        (0.12, 0.37 - 2.1 * s),
        0.23 - 0.45 * s,
        0.155 - 0.35 * s,
    );
    let body_d = smin(neck, shaft, 0.08);
    let b_rim = smoothstep(-rim_w, -0.004, body_d);
    let bx = ((x - 0.17) / 0.24).clamp(-1.0, 1.0);
    let b_nz = (1.0 - bx * bx * 0.85).max(0.1).sqrt();
    let b_core = 0.09 + 0.12 * (-0.35 * bx + 0.85 * b_nz).max(0.0);
    let body_col = grey(mix(b_core, rim_hi, b_rim * 0.96));

    // 4. Curved index trigger recessed into the front-upper handle below the faceplate.
    let trig_d = capsule(
        p,
        (0.20, -0.31),
        (0.18, 0.05 - 1.3 * s),
        0.140 - 0.3 * s,
        0.100 - 0.3 * s,
    );
    let trig_recess_d = trig_d - 0.028;
    let tx = ((x - 0.22) / 0.14).clamp(-1.0, 1.0);
    let ty = ((y + 0.35) / 0.44).clamp(0.0, 1.0);
    let t_crest = (-((ty - 0.44) / 0.34).powi(2)).exp() * 0.28;
    let t_side = (1.0 - tx * tx * 0.75).max(0.18).sqrt();
    let t_bevel = smoothstep(-0.048, -0.004, trig_d) * 0.18;
    let trig_col = grey((0.18 + 0.20 * t_side + t_crest + t_bevel - 0.05 * tx).clamp(0.14, 0.70));

    // 5. Horizontal button faceplate, level with the horizon.
    let deck_skirt_d = capsule(p, (-0.02, -0.42), (0.64, -0.42), 0.125, 0.135);
    let deck_top_d = ellipse(p, (0.31, -0.475), 0.42, 0.10);
    let d_rim = smoothstep(-(w * 1.75).max(0.080), -0.003, deck_skirt_d);
    let lip = smoothstep(0.05, 0.0, (y + 0.36).abs())
        * smoothstep(-0.10, 0.08, x)
        * smoothstep(0.75, 0.56, x);
    let dx = ((x - 0.31) / 0.42).clamp(-1.0, 1.0);
    let skirt_core = 0.12 + 0.10 * (1.0 - 0.5 * dx) + 0.28 * lip;
    let deck_skirt_col = grey(mix(skirt_core, rim_hi, d_rim * 0.96).clamp(0.10, 0.76));

    let top_rim = smoothstep(-(w * 0.9).max(0.046), -0.003, deck_top_d);
    let top_core = 0.12 + 0.10 * (1.0 - 0.4 * dx);
    let top_hi = if w > 0.06 { 0.66 } else { 0.62 };
    let deck_top_col = grey(mix(top_core, top_hi, top_rim * 0.92).clamp(0.10, 0.70));

    // 6. Home, A and B buttons resting on the horizontal faceplate.
    let [bh_bar, bh_cap] = button(p, (0.32, -0.545), 0.052, 0.027, 0.024);
    let [ba_bar, ba_cap] = button(p, (0.47, -0.545), 0.074, 0.038, 0.038);
    let [bb_bar, bb_cap] = button(p, (0.615, -0.485), 0.076, 0.039, 0.040);

    // 7. Thumbstick rising straight up into open air above the horizontal faceplate.
    let stick_stem_d = capsule(p, (0.12, -0.51), (0.12, -0.71), 0.070, 0.058);
    let st_rim = smoothstep(-(w * 1.6).max(0.062), -0.003, stick_stem_d);
    let stick_stem_col = grey(mix(0.14, rim_hi, st_rim * 0.96));

    let stick_cap_d = ellipse(p, (0.12, -0.735), 0.180, 0.078);
    let dish_rx = (0.105 - 0.9 * s).max(0.035);
    let dish_ry = (0.034 - 0.3 * s).max(0.012);
    let stick_dish_d = ellipse(p, (0.12, -0.738), dish_rx, dish_ry);
    let scx = ((x - 0.12) / 0.180).clamp(-1.0, 1.0);
    let scy = ((y + 0.735) / 0.078).clamp(-1.0, 1.0);
    let cap_lit = (0.75 - 0.07 * scx - 0.05 * scy).clamp(0.66, 0.84);
    let stick_cap_col = [
        (cap_lit * 0.93).clamp(0.0, 1.0),
        (cap_lit * 0.97).clamp(0.0, 1.0),
        (cap_lit * 1.04).clamp(0.0, 1.0),
    ];
    let stick_dish_col = grey((0.16 + 0.12 * scx + 0.08 * scy).clamp(0.11, 0.34));

    [
        (ring_d, ring_col, true),
        (body_d, body_col, true),
        (grip_recess_d, RECESS, false),
        (grip_d, grip_col, true),
        (trig_recess_d, RECESS, false),
        (trig_d, trig_col, false),
        (deck_skirt_d, deck_skirt_col, true),
        (deck_top_d, deck_top_col, false),
        bh_bar,
        bh_cap,
        ba_bar,
        ba_cap,
        bb_bar,
        bb_cap,
        (stick_stem_d, stick_stem_col, true),
        (stick_cap_d, stick_cap_col, true),
        (stick_dish_d, stick_dish_col, false),
    ]
}

/// The icon at `size` × `size` pixels as straight-alpha RGBA, rows top to bottom,
/// supersampled 8 × 8 per pixel.
pub fn rgba(tint: Tint, size: usize) -> Vec<u8> {
    const SS: usize = 8;
    let unit = 2.0 / size as f32;
    let w = (unit * 1.05).max(0.036);
    let tint = tint.rgb();
    let mut out = vec![0; size * size * 4];
    for (i, pixel) in out.as_chunks_mut::<4>().0.iter_mut().enumerate() {
        let (px, py) = ((i % size) as f32, (i / size) as f32);
        let mut sum = [0.0f32; 4];
        for s in 0..SS * SS {
            let x = (px + ((s % SS) as f32 + 0.5) / SS as f32) * unit - 1.0;
            let y = (py + ((s / SS) as f32 + 0.5) / SS as f32) * unit - 1.0;
            let mut color = None;
            for (d, fill, outlined) in layers((x, y), tint, w) {
                if d < 0.0 {
                    color = Some(fill);
                } else if outlined && d < w {
                    color = Some(OUTLINE);
                }
            }
            if let Some(c) = color {
                for k in 0..3 {
                    sum[k] += c[k];
                }
                sum[3] += 1.0;
            }
        }
        if sum[3] > 0.0 {
            for k in 0..3 {
                pixel[k] = (sum[k] / sum[3] * 255.0).round() as u8;
            }
            pixel[3] = (sum[3] / (SS * SS) as f32 * 255.0).round() as u8;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const TINTS: [Tint; 3] = [Tint::Ready, Tint::Warning, Tint::Stopped];
    /// Breeze Dark's panel and a light panel.
    const PANELS: [[u8; 3]; 2] = [[0x31, 0x36, 0x3b], [0xef, 0xf0, 0xf1]];
    const TESTED: [usize; 6] = [16, 22, 24, 32, 64, 128];

    fn luminance(c: [f32; 3]) -> f32 {
        let l = |v: f32| {
            if v <= 0.04045 {
                v / 12.92
            } else {
                ((v + 0.055) / 1.055).powf(2.4)
            }
        };
        0.2126 * l(c[0]) + 0.7152 * l(c[1]) + 0.0722 * l(c[2])
    }

    /// The pixel's contrast ratio against the panel `bg`, composited as the panel shows it.
    fn contrast(p: &[u8; 4], bg: [u8; 3]) -> f32 {
        let a = p[3] as f32 / 255.0;
        let c: [f32; 3] =
            std::array::from_fn(|k| (p[k] as f32 * a + bg[k] as f32 * (1.0 - a)) / 255.0);
        let (lc, lb) = (luminance(c), luminance(bg.map(|v| v as f32 / 255.0)));
        (lc.max(lb) + 0.05) / (lc.min(lb) + 0.05)
    }

    /// The share of the silhouette's edge pixels that have a pixel standing out from the
    /// panel by 3:1 or more within an outline's width and a pixel.
    fn edge_legibility(px: &[[u8; 4]], size: usize, bg: [u8; 3]) -> f32 {
        let covered = |x: isize, y: isize| {
            (0..size as isize).contains(&x)
                && (0..size as isize).contains(&y)
                && px[y as usize * size + x as usize][3] >= 128
        };
        let reach = ((2.0 / size as f32 * 1.05).max(0.036) * size as f32 / 2.0).ceil() as isize + 1;
        let (mut edges, mut clear) = (0, 0);
        for y in 0..size as isize {
            for x in 0..size as isize {
                let edge = covered(x, y)
                    && [(1, 0), (-1, 0), (0, 1), (0, -1)]
                        .iter()
                        .any(|(dx, dy)| !covered(x + dx, y + dy));
                if !edge {
                    continue;
                }
                edges += 1;
                let near = (-reach..=reach).flat_map(|dy| (-reach..=reach).map(move |dx| (dx, dy)));
                if near
                    .filter(|(dx, dy)| covered(x + dx, y + dy))
                    .any(|(dx, dy)| {
                        contrast(&px[(y + dy) as usize * size + (x + dx) as usize], bg) >= 3.0
                    })
                {
                    clear += 1;
                }
            }
        }
        clear as f32 / edges as f32
    }

    /// Sampled mean colour on the tracking ring across left, bottom and right arcs.
    fn ring_sample(px: &[[u8; 4]], size: usize) -> [f32; 3] {
        let pts = [(-0.78_f32, 0.15_f32), (0.0, 0.77), (0.76, 0.10)];
        let mut sum = [0.0f32; 3];
        for (x, y) in pts {
            let ix = (((x + 1.0) / 2.0 * size as f32) as usize).min(size - 1);
            let iy = (((y + 1.0) / 2.0 * size as f32) as usize).min(size - 1);
            let p = px[iy * size + ix];
            assert!(p[3] > 200, "{size} px: ring missing at ({x}, {y}): {p:?}");
            for k in 0..3 {
                sum[k] += p[k] as f32;
            }
        }
        sum.map(|v| v / pts.len() as f32)
    }

    #[test]
    fn every_state_and_size_is_a_clean_legible_shape() {
        for size in TESTED {
            let mut rings = Vec::new();
            for tint in TINTS {
                let rgba = rgba(tint, size);
                assert_eq!(rgba.len(), size * size * 4);
                let px = rgba.as_chunks::<4>().0;
                for corner in [0, size - 1, size * (size - 1), size * size - 1] {
                    assert_eq!(px[corner][3], 0, "{size} px: corner covered");
                }
                assert!(
                    px.iter().any(|p| p[3] > 0 && p[3] < 255),
                    "{size} px: not anti-aliased"
                );
                assert!(
                    px.iter()
                        .any(|p| p[3] == 255 && p[..3].iter().all(|&v| v < 40)),
                    "{size} px: no dark outline"
                );
                for bg in PANELS {
                    let share = edge_legibility(px, size, bg);
                    assert!(share >= 0.98, "{size} px {tint:?} on {bg:?}: {share}");
                }
                rings.push(ring_sample(px, size));
            }
            // Green, amber and grey rings are told apart by hue, not only by brightness.
            let [ready, warning, stopped] = [rings[0], rings[1], rings[2]];
            assert!(ready[1] > ready[0] + 80.0 && ready[1] > ready[2] + 35.0);
            assert!(warning[0] > warning[2] + 130.0 && warning[1] > warning[2] + 70.0);
            assert!(stopped.iter().all(|v| (v - stopped[1]).abs() < 35.0));
        }
    }

    /// Front view with level horizontal faceplate, black/grey handle and visible controls
    /// (thumbstick, buttons, trigger, grip) framed inside the status-coloured tracking ring.
    #[test]
    fn silhouette_and_controls_match_front_view() {
        let neutral = |p: [u8; 4]| {
            p[3] == 255
                && p[0].abs_diff(p[1]) < 24
                && p[1].abs_diff(p[2]) < 28
                && p[0].abs_diff(p[2]) < 36
        };
        for size in TESTED {
            let rgba = rgba(Tint::Ready, size);
            let px = rgba.as_chunks::<4>().0;
            let at = |x: f32, y: f32| {
                let i = |v: f32| (((v + 1.0) / 2.0 * size as f32) as usize).min(size - 1);
                px[i(y) * size + i(x)]
            };
            let covered: Vec<_> = (0..size * size).filter(|&i| px[i][3] >= 128).collect();
            let (xs, ys) = (
                covered.iter().map(|i| i % size),
                covered.iter().map(|i| i / size),
            );
            let width = xs.clone().max().unwrap() - xs.min().unwrap() + 1;
            let height = ys.clone().max().unwrap() - ys.min().unwrap() + 1;
            let aspect = width as f32 / height as f32;
            assert!((0.95..1.18).contains(&aspect), "{size} px: aspect {aspect}");

            // Matte dark charcoal handle core inside the ring (at 16 px the handle is only a
            // couple of pixels wide so its studio rim highlight dominates to stay legible on dark
            // panels).
            let handle = at(0.14, 0.14);
            assert!(
                neutral(handle) && (size < 24 || handle[1] < 130),
                "{size} px: handle is not dark neutral grey: {handle:?}"
            );
            // Status-coloured ring on the left, right, bottom and upper-left arch.
            for (x, y) in [(-0.78, 0.15), (0.76, 0.10), (0.0, 0.77), (-0.42, -0.56)] {
                let p = at(x, y);
                assert!(
                    p[3] > 200 && p[1] > p[0] + 55,
                    "{size} px: no green ring at ({x}, {y}): {p:?}"
                );
            }
            // At 32 px and up, all four controls read cleanly in neutral grey tones.
            if size >= 32 {
                let stick_rim = at(0.24, -0.735);
                assert!(
                    neutral(stick_rim) && stick_rim[1] > 140,
                    "{size} px: thumbstick rim missing: {stick_rim:?}"
                );
                let trigger = at(0.19, -0.14);
                assert!(
                    neutral(trigger) && trigger[1] > 80,
                    "{size} px: trigger missing: {trigger:?}"
                );
                let grip = at(0.31, 0.14);
                assert!(
                    neutral(grip) && grip[1] > 70,
                    "{size} px: grip missing: {grip:?}"
                );
                let btn = at(0.47, -0.545);
                assert!(
                    neutral(btn) && btn[1] > 120,
                    "{size} px: button missing: {btn:?}"
                );
            }

            // Openings inside the ring survive to the left, right and below the handle.
            let opening = |from: (f32, f32), to: (f32, f32)| {
                (0..=64)
                    .map(|i| {
                        let t = i as f32 / 64.0;
                        at(from.0 + (to.0 - from.0) * t, from.1 + (to.1 - from.1) * t)
                    })
                    .min_by_key(|p| p[3])
                    .unwrap()
            };
            let openings = [
                ("left", opening((-0.62, 0.08), (-0.12, 0.08))),
                ("right", opening((0.42, 0.12), (0.68, 0.12))),
                ("bottom", opening((0.12, 0.44), (0.12, 0.66))),
            ];
            for (i, (name, p)) in openings.into_iter().enumerate() {
                if size >= 24 || i == 0 {
                    assert!(p[3] < 64, "{size} px: {name} opening closed: {p:?}");
                } else {
                    assert!(
                        p[3] < 128 || p[..3].iter().all(|&v| v < 60),
                        "{size} px: {name} opening filled: {p:?}"
                    );
                }
            }
        }
    }

    /// Writes the contact sheet of every state and size on both panels to
    /// GUI_SCREENSHOTS_DIR/icon-preview.png when that is set.
    #[test]
    fn icon_preview() {
        let Some(dir) = std::env::var_os("GUI_SCREENSHOTS_DIR") else {
            return;
        };
        const PAD: usize = 8;
        let w = TESTED.iter().map(|s| s + PAD).sum::<usize>() + PAD;
        let row = 128 + PAD;
        let h = row * PANELS.len() * TINTS.len() + PAD;
        let mut sheet = vec![0u8; w * h * 4];
        for (r, (bg, tint)) in PANELS
            .iter()
            .flat_map(|bg| TINTS.iter().map(move |t| (bg, t)))
            .enumerate()
        {
            let y0 = PAD + r * row;
            for y in y0 - PAD / 2..y0 + 128 + PAD / 2 {
                for x in 0..w {
                    sheet[(y * w + x) * 4..][..4].copy_from_slice(&[bg[0], bg[1], bg[2], 255]);
                }
            }
            let mut x0 = PAD;
            for size in TESTED {
                let icon = rgba(*tint, size);
                for (i, p) in icon.as_chunks::<4>().0.iter().enumerate() {
                    let o = ((y0 + i / size) * w + x0 + i % size) * 4;
                    let a = p[3] as f32 / 255.0;
                    for k in 0..3 {
                        let v = p[k] as f32 * a + sheet[o + k] as f32 * (1.0 - a);
                        sheet[o + k] = v.round() as u8;
                    }
                }
                x0 += size + PAD;
            }
        }
        std::fs::create_dir_all(&dir).unwrap();
        crate::png::write(
            &std::path::Path::new(&dir).join("icon-preview.png"),
            &sheet,
            (w, h),
        )
        .unwrap();
    }
}
