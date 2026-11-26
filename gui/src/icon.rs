// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! The tray and window icon: a right Touch controller seen from the front, looking back
//! along its pointing axis, with the tracking ring in the status colour. Drawn from signed
//! distance fields.

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
const OUTLINE: Rgb = rgb(0x18, 0x1c, 0x22);
const BODY: Rgb = rgb(0xe2, 0xe8, 0xf0);
const TRIGGER: Rgb = rgb(0xbf, 0xcb, 0xd9);

const fn rgb(r: u8, g: u8, b: u8) -> Rgb {
    [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0]
}

impl Tint {
    fn rgb(self) -> Rgb {
        match self {
            Self::Ready => rgb(0x34, 0xc7, 0x7b),
            Self::Warning => rgb(0xf0, 0xaa, 0x28),
            Self::Stopped => rgb(0x9a, 0xa8, 0xba),
        }
    }
}

fn length(x: f32, y: f32) -> f32 {
    x.hypot(y)
}

/// An axis-aligned ellipse centred on (`cx`, `cy`) with radii `a`, `b`; exact on its axes
/// and close enough elsewhere for these near-circles.
fn ellipse((x, y): (f32, f32), (cx, cy): (f32, f32), a: f32, b: f32) -> f32 {
    (length((x - cx) / a, (y - cy) / b) - 1.0) * a.min(b)
}

/// A capsule from `p0` (radius `r0`) to `p1` (radius `r1`).
fn capsule((x, y): (f32, f32), p0: (f32, f32), p1: (f32, f32), r0: f32, r1: f32) -> f32 {
    let (px, py, bx, by) = (x - p0.0, y - p0.1, p1.0 - p0.0, p1.1 - p0.1);
    let h = ((px * bx + py * by) / (bx * bx + by * by)).clamp(0.0, 1.0);
    length(px - bx * h, py - by * h) - (r0 + (r1 - r0) * h)
}

/// The painted layers, back to front, at `p` in [-1, 1]² (y down): each a distance (negative
/// inside), its fill, and whether it gets the dark outline `w` wide. Seen from the front,
/// the ring is a near circle, a little wider than tall and thicker on the left. The thumb
/// deck spans its top, right of centre where the right thumb rests, and the trigger and
/// grip hang from it to just short of the ring's bottom, leaving the ring's inside open to
/// the left, to the right and below. The ring thickens at small sizes so that its colour
/// still shows.
fn layers(p: (f32, f32), ring: Rgb, w: f32) -> [(f32, Rgb, bool); 3] {
    // How much wider the outline is than at large sizes: the grip narrows and shortens by
    // as much so that the openings beside and below it survive the outline.
    let s = w - 0.04;
    let k = (w * 1.2 - 0.10).max(0.0);
    let ring_d =
        ellipse(p, (0.0, -0.03), 0.86, 0.80).max(-ellipse(p, (0.035, -0.04), 0.725 - k, 0.69 - k));
    let deck = capsule(p, (0.0, -0.60), (0.365, -0.43), 0.23, 0.28);
    let grip = capsule(
        p,
        (0.14, -0.47),
        (0.045, 0.295 - 2.5 * s),
        0.31 - s,
        0.19 - s,
    );
    let trigger = capsule(
        p,
        (0.02, -0.30),
        (0.0, 0.24 - 2.5 * s),
        0.13 - s / 2.0,
        0.10 - s / 2.0,
    );
    [
        (ring_d, ring, true),
        (deck.min(grip), BODY, true),
        (trigger, TRIGGER, false),
    ]
}

/// The icon at `size` × `size` pixels as straight-alpha RGBA, rows top to bottom,
/// supersampled 8 × 8 per pixel.
pub fn rgba(tint: Tint, size: usize) -> Vec<u8> {
    const SS: usize = 8;
    let unit = 2.0 / size as f32;
    // A pixel of outline at small sizes, no thinner than a fiftieth of the icon at large ones.
    let w = (unit * 1.1).max(0.04);
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
    /// panel by 3:1 or more within an outline's width and a pixel: the dark outline on a
    /// light panel, the ring or the body on a dark one.
    fn edge_legibility(px: &[[u8; 4]], size: usize, bg: [u8; 3]) -> f32 {
        let covered = |x: isize, y: isize| {
            (0..size as isize).contains(&x)
                && (0..size as isize).contains(&y)
                && px[y as usize * size + x as usize][3] >= 128
        };
        let reach = ((2.0 / size as f32 * 1.1).max(0.04) * size as f32 / 2.0).ceil() as isize + 1;
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

    /// The mean colour of the pixels close to the ring's tint.
    fn ring(px: &[[u8; 4]], tint: Tint) -> Option<[f32; 3]> {
        let t = tint.rgb();
        let near: Vec<_> = px
            .iter()
            .filter(|p| p[3] == 255 && (0..3).all(|k| (p[k] as f32 / 255.0 - t[k]).abs() < 0.08))
            .collect();
        (near.len() >= 8).then(|| {
            std::array::from_fn(|k| {
                near.iter().map(|p| p[k] as f32).sum::<f32>() / near.len() as f32
            })
        })
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
                rings.push(ring(px, tint).unwrap_or_else(|| panic!("{size} px {tint:?}: no ring")));
            }
            // Green, amber and grey rings are told apart by hue, not only by brightness.
            let [ready, warning, stopped] = [rings[0], rings[1], rings[2]];
            assert!(ready[1] > ready[0] + 100.0 && ready[1] > ready[2] + 40.0);
            assert!(warning[0] > warning[2] + 150.0 && warning[1] > warning[2] + 80.0);
            assert!(stopped.iter().all(|v| (v - stopped[1]).abs() < 30.0));
        }
    }

    /// The front view's silhouette: a round ring, the grip hanging inside it from the top,
    /// and the ring's inside open to the left of the grip, to its right and below it.
    #[test]
    fn silhouette_is_the_ring_with_the_grip_hanging_inside() {
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
            assert!((1.0..1.16).contains(&aspect), "{size} px: aspect {aspect}");
            // Body colour in the grip, ring colour at the ring's left, right and bottom.
            let body = |p: [u8; 4]| p[3] == 255 && p[..3].iter().all(|&v| v > 170);
            assert!(body(at(0.06, -0.05)), "{size} px: no grip");
            // From the front, the right controller's thumb deck is right of centre.
            assert!(
                body(at(0.45, -0.45)) && !body(at(-0.45, -0.45)),
                "{size} px: the deck is not on the right"
            );
            for (x, y) in [(-0.80, 0.0), (0.80, 0.0), (0.0, 0.71)] {
                let p = at(x, y);
                assert!(
                    p[3] > 0 && p[1] > p[0] + 60,
                    "{size} px: no ring at {x}, {y}: {p:?}"
                );
            }
            // Each opening has a pixel at least three quarters clear from 22 px up; at 16 px the
            // left one still does,
            // and the others stay dark and unfilled.
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
                ("left", opening((-0.70, 0.05), (-0.10, 0.05))),
                ("right", opening((0.35, 0.05), (0.75, 0.05))),
                ("bottom", opening((0.05, 0.30), (0.05, 0.70))),
            ];
            for (i, (name, p)) in openings.into_iter().enumerate() {
                if size >= 22 || i == 0 {
                    assert!(p[3] < 64, "{size} px: {name} opening closed: {p:?}");
                } else {
                    assert!(
                        p[..3].iter().all(|&v| v < 60),
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
