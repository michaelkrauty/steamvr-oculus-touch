// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

//! Deterministic synthetic calibration scenes for tests: three wall-mounted sensors near the
//! ceiling looking at the play space (two upside down, as mounted), a Touch-like LED ring, noisy
//! LED sightings and noisy engine pose estimates, optionally with wrong-but-confident sightings.

use std::collections::BTreeMap;

use nalgebra::{
    Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector2, Vector3,
};

use super::camera::CameraModel;
use super::capture::{ImuSample, Led, Observation, View, WandSample};

pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(1))
    }
    pub fn uniform(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.uniform()
    }
    pub fn gauss(&mut self) -> f64 {
        let (u1, u2) = (self.uniform().max(1e-12), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
    pub fn gauss3(&mut self, sigma: f64) -> Vector3<f64> {
        Vector3::new(self.gauss(), self.gauss(), self.gauss()) * sigma
    }
}

pub fn camera_model() -> CameraModel {
    CameraModel {
        width: 1280,
        height: 960,
        fx: 715.0,
        fy: 715.0,
        cx: 640.0,
        cy: 480.0,
        k: [0.03, -0.01, 0.002, 0.0],
    }
}

/// world_from_cam for a camera at `pos` looking at `target` (OpenCV axes, world +Y up), rolled
/// 180 degrees when mounted upside down.
pub fn look_at(pos: Vector3<f64>, target: Vector3<f64>, upside_down: bool) -> Isometry3<f64> {
    let z = (target - pos).normalize();
    let y = (-Vector3::y() + z * Vector3::y().dot(&z)).normalize(); // image down
    let x = y.cross(&z);
    let (x, y) = if upside_down { (-x, -y) } else { (x, y) };
    let r = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[x, y, z]));
    Isometry3::from_parts(
        Translation3::from(pos),
        UnitQuaternion::from_rotation_matrix(&r),
    )
}

/// A ring of 24 LEDs (radius 45 mm, three heights) with outward, slightly upward normals.
pub fn touch_leds() -> Vec<Led> {
    (0..24)
        .map(|i| {
            let a = i as f64 * std::f64::consts::TAU / 24.0;
            let h = [-0.012, 0.0, 0.012][i % 3];
            Led {
                pos: Vector3::new(0.045 * a.cos(), h, 0.045 * a.sin()),
                dir: Vector3::new(a.cos(), 0.35, a.sin()).normalize(),
            }
        })
        .collect()
}

/// A real left CV1 Touch's LED model and IMU position (its factory calibration, model frame, m).
/// Unlike `touch_leds`, its ring is not exactly symmetric: turned about 28 degrees about the ring
/// axis, two thirds of its LEDs land on others' spots, and the rest tell the turn apart.
pub fn cv1_touch_left() -> (Vec<Led>, Vector3<f64>) {
    const LEDS: [([f64; 3], [f64; 3]); 24] = [
        ([0.02983, -0.00837, -0.01136], [0.8841, 0.3883, -0.2601]),
        ([0.03556, -0.03320, -0.01055], [0.9672, -0.0248, -0.2530]),
        ([0.02961, -0.05587, -0.01147], [0.8449, -0.4430, -0.2997]),
        ([0.01123, -0.07539, -0.01455], [0.4922, -0.8024, -0.3376]),
        ([-0.01049, -0.08232, -0.01856], [0.1151, -0.9043, -0.4110]),
        ([-0.03680, -0.07705, -0.02313], [-0.3608, -0.8039, -0.4728]),
        ([-0.05492, -0.06126, -0.02616], [-0.6802, -0.5182, -0.5184]),
        ([-0.06364, -0.03840, -0.02787], [-0.8033, -0.1040, -0.5865]),
        ([-0.06115, -0.01498, -0.02745], [-0.7496, 0.3302, -0.5736]),
        ([-0.04838, 0.00485, -0.02515], [-0.5021, 0.6476, -0.5732]),
        ([0.03066, -0.00955, 0.00556], [0.9098, 0.3467, 0.2280]),
        ([0.03551, -0.03344, 0.00638], [0.9657, -0.0479, 0.2551]),
        ([0.02983, -0.05618, 0.00538], [0.8520, -0.4575, 0.2543]),
        ([0.01583, -0.07364, 0.00292], [0.5794, -0.7906, 0.1980]),
        ([-0.00404, -0.08371, -0.00048], [0.2103, -0.9706, 0.1174]),
        ([-0.02612, -0.08458, -0.00441], [-0.2157, -0.9737, 0.0738]),
        ([-0.04767, -0.07545, -0.00803], [-0.6127, -0.7900, 0.0218]),
        ([-0.06220, -0.05948, -0.01079], [-0.8735, -0.4862, -0.0244]),
        ([-0.06953, -0.03688, -0.01212], [-0.9957, -0.0612, -0.0697]),
        ([-0.06643, -0.01323, -0.01162], [-0.9261, 0.3709, -0.0696]),
        ([-0.05531, 0.00496, -0.00972], [-0.6969, 0.6983, -0.1633]),
        ([-0.03976, 0.01188, -0.01375], [0.2846, 0.9449, 0.1617]),
        ([-0.00511, 0.00148, -0.02931], [0.0084, 0.7970, -0.6039]),
        ([0.01760, -0.00147, -0.02364], [0.5632, 0.6698, -0.4839]),
    ];
    (
        LEDS.iter()
            .map(|(p, d)| Led {
                pos: Vector3::from(*p),
                dir: Vector3::from(*d).normalize(),
            })
            .collect(),
        Vector3::new(0.0154, -0.0126, 0.0082),
    )
}

#[derive(Debug, Clone)]
pub struct SceneParams {
    pub samples: usize,
    pub pixel_noise: f64,
    pub pnp_rot_noise_rad: f64,
    pub pnp_pos_noise_m: f64,
    /// Fraction of sightings replaced by a wrong-but-confident one: pose rotated ~92 degrees,
    /// LED labels shifted.
    pub flipped_fraction: f64,
    pub seed: u64,
}

impl Default for SceneParams {
    fn default() -> Self {
        SceneParams {
            samples: 300,
            pixel_noise: 0.3,
            pnp_rot_noise_rad: 0.02,
            pnp_pos_noise_m: 0.01,
            flipped_fraction: 0.0,
            seed: 1,
        }
    }
}

pub struct Scene {
    /// Sorted, as in a Capture: camera index = position.
    pub serials: Vec<String>,
    pub cameras: Vec<CameraModel>,
    pub world_from_cam: Vec<Isometry3<f64>>,
    pub leds: BTreeMap<i32, Vec<Led>>,
    pub samples: Vec<WandSample>,
    pub world_from_obj: Vec<Isometry3<f64>>,
}

/// Noisy sightings of a controller at world pose `w` by every camera that sees at least 5 LEDs.
fn sight(
    p: &SceneParams,
    rng: &mut Rng,
    world_from_cam: &[Isometry3<f64>],
    cam: &CameraModel,
    leds: &[Led],
    w: &Isometry3<f64>,
) -> Vec<View> {
    let mut views = Vec::new();
    for (c, wc) in world_from_cam.iter().enumerate() {
        let cam_from_obj = wc.inverse() * w;
        let mut obs = Vec::new();
        for (li, led) in leds.iter().enumerate() {
            let pc = cam_from_obj * Point3::from(led.pos);
            let nc = cam_from_obj.rotation * led.dir;
            if nc.dot(&pc.coords) > -0.2 * pc.coords.norm() {
                continue; // facing away or grazing
            }
            let Some(uv) = cam.project(&pc.coords) else {
                continue;
            };
            if uv.x < 0.0 || uv.y < 0.0 || uv.x >= cam.width as f64 || uv.y >= cam.height as f64 {
                continue;
            }
            obs.push(Observation {
                led: li,
                xy: uv + Vector2::new(rng.gauss(), rng.gauss()) * p.pixel_noise,
            });
        }
        if obs.len() < 5 {
            continue;
        }
        let noisy = Isometry3::from_parts(
            Translation3::from(cam_from_obj.translation.vector + rng.gauss3(p.pnp_pos_noise_m)),
            cam_from_obj.rotation
                * UnitQuaternion::from_scaled_axis(rng.gauss3(p.pnp_rot_noise_rad)),
        );
        let flipped = rng.uniform() < p.flipped_fraction;
        let estimate = if flipped {
            for o in &mut obs {
                o.led = (o.led + 7) % leds.len();
            }
            Isometry3::from_parts(
                noisy.translation,
                noisy.rotation * UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, 1.6)),
            )
        } else {
            noisy
        };
        views.push(View {
            camera: c,
            cam_from_obj: Some(estimate),
            strong: true,
            obs,
        });
    }
    views
}

/// Samples are 1 s apart in device time, each at an independent random pose.
pub fn scene(p: &SceneParams) -> Scene {
    let mut rng = Rng::new(p.seed);
    let target = Vector3::new(0.0, 0.9, 0.0);
    let serials = vec![
        "WMTD000000001".to_string(),
        "WMTD000000002".to_string(),
        "WMTD000000003".to_string(),
    ];
    let world_from_cam = vec![
        look_at(Vector3::new(0.3, 2.2, -2.6), target, false), // above the TV, upright
        look_at(Vector3::new(-2.0, 2.3, -1.8), target, true), // above the desk, upside down
        look_at(Vector3::new(2.4, 2.3, 1.6), target, true),   // behind the couch, upside down
    ];
    let cam = camera_model();
    let leds = touch_leds();
    let (mut samples, mut world_from_obj) = (Vec::new(), Vec::new());
    for i in 0..p.samples {
        let pos = Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(0.5, 1.7),
            rng.range(-1.0, 1.0),
        );
        let w = Isometry3::from_parts(
            Translation3::from(pos),
            UnitQuaternion::from_scaled_axis(rng.gauss3(1.2)),
        );
        let t_ns = 1_000_000_000 + i as u64 * 1_000_000_000;
        let views = sight(p, &mut rng, &world_from_cam, &cam, &leds, &w);
        if views.is_empty() {
            continue;
        }
        samples.push(WandSample {
            device_id: 1,
            device_time_ns: t_ns,
            host_ns: t_ns,
            views,
        });
        world_from_obj.push(w);
    }
    Scene {
        serials,
        cameras: vec![cam; 3],
        world_from_cam,
        leds: BTreeMap::from([(1, leds)]),
        samples,
        world_from_obj,
    }
}

#[derive(Debug, Clone)]
pub struct TrajectoryParams {
    pub duration_s: f64,
    pub frame_hz: f64,
    pub imu_hz: f64,
    pub pixel_noise: f64,
    pub pnp_rot_noise_rad: f64,
    pub pnp_pos_noise_m: f64,
    pub flipped_fraction: f64,
    /// g * up in the world frame.
    pub gravity: Vector3<f64>,
    /// Per device (right, left): accelerometer bias (m/s^2) and IMU position in the LED model frame.
    pub accel_bias: [Vector3<f64>; 2],
    pub lever_arm: [Vector3<f64>; 2],
    /// Per device: rotation (axis * angle) of the gravity its accelerometer feels, so the two
    /// controllers' gravity estimates disagree (a test of the cross-check, not physics).
    pub device_gravity_tilt: [Vector3<f64>; 2],
    pub gyro_noise: f64,
    pub accel_noise: f64,
    pub seed: u64,
}

impl Default for TrajectoryParams {
    fn default() -> Self {
        TrajectoryParams {
            duration_s: 30.0,
            frame_hz: 52.0,
            imu_hz: 500.0,
            pixel_noise: 0.3,
            pnp_rot_noise_rad: 0.02,
            pnp_pos_noise_m: 0.01,
            flipped_fraction: 0.0,
            gravity: Vector3::new(0.0, 9.80665, 0.0),
            accel_bias: [
                Vector3::new(0.02, 0.45, -0.05),
                Vector3::new(-0.01, 0.10, -0.07),
            ],
            lever_arm: [
                Vector3::new(-0.005, -0.012, -0.008),
                Vector3::new(0.016, -0.010, 0.003),
            ],
            device_gravity_tilt: [Vector3::zeros(), Vector3::zeros()],
            gyro_noise: 0.0,
            accel_noise: 0.0,
            seed: 7,
        }
    }
}

/// A controller swept through the play space: position and Z-Y-X Euler angles as sums of sines.
#[derive(Debug, Clone)]
pub struct Motion {
    centre: Vector3<f64>,
    pos: [[(f64, f64, f64); 2]; 3],
    ang: [[(f64, f64, f64); 2]; 3],
}

fn sines(terms: &[(f64, f64, f64); 2], t: f64) -> (f64, f64) {
    terms.iter().fold((0.0, 0.0), |(v, d), &(a, w, ph)| {
        (v + a * (w * t + ph).sin(), d + a * w * (w * t + ph).cos())
    })
}

impl Motion {
    pub fn random(rng: &mut Rng, centre: Vector3<f64>) -> Motion {
        let mut term = |amp: f64| {
            (
                rng.range(0.5, 1.0) * amp,
                std::f64::consts::TAU * rng.range(0.15, 0.6),
                rng.range(0.0, std::f64::consts::TAU),
            )
        };
        let pos = [
            [term(0.45), term(0.2)],
            [term(0.3), term(0.15)],
            [term(0.45), term(0.2)],
        ];
        let ang = [
            [term(1.6), term(0.6)],
            [term(0.9), term(0.4)],
            [term(1.4), term(0.5)],
        ];
        Motion { centre, pos, ang }
    }

    /// A controller lying still at `centre`, turned by the angles `turn` (the same three angles
    /// `pose` composes).
    pub fn still(centre: Vector3<f64>, turn: [f64; 3]) -> Motion {
        let zero = [(0.0, 0.0, 0.0); 2];
        let fixed = |a: f64| [(a, 0.0, std::f64::consts::FRAC_PI_2), (0.0, 0.0, 0.0)];
        Motion {
            centre,
            pos: [zero; 3],
            ang: [fixed(turn[0]), fixed(turn[1]), fixed(turn[2])],
        }
    }

    pub fn pose(&self, t: f64) -> Isometry3<f64> {
        let p = Vector3::new(
            sines(&self.pos[0], t).0,
            sines(&self.pos[1], t).0,
            sines(&self.pos[2], t).0,
        );
        let (a, b, g) = (
            sines(&self.ang[0], t).0,
            sines(&self.ang[1], t).0,
            sines(&self.ang[2], t).0,
        );
        let r = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), a)
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), b)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), g);
        Isometry3::from_parts(Translation3::from(self.centre + p), r)
    }

    /// Body-frame angular velocity of `pose` (exact, from the Euler rates).
    pub fn gyro(&self, t: f64) -> Vector3<f64> {
        let ((_, da), (b, db), (g, dg)) = (
            sines(&self.ang[0], t),
            sines(&self.ang[1], t),
            sines(&self.ang[2], t),
        );
        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), g);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), b);
        rx.inverse() * (ry.inverse() * Vector3::new(0.0, 0.0, da))
            + rx.inverse() * Vector3::new(0.0, db, 0.0)
            + Vector3::new(dg, 0.0, 0.0)
    }

    /// Specific force at the IMU (`lever` in the model frame): R^T (p_imu'' + G), by central differences.
    pub fn accel(&self, t: f64, lever: &Vector3<f64>, gravity: &Vector3<f64>) -> Vector3<f64> {
        let h = 1e-3;
        let at = |t: f64| self.pose(t) * Point3::from(*lever);
        let acc = (at(t + h).coords - 2.0 * at(t).coords + at(t - h).coords) / (h * h);
        self.pose(t).rotation.inverse() * (acc + gravity)
    }
}

pub struct TrajectoryScene {
    pub scene: Scene,
    pub imu: BTreeMap<i32, Vec<ImuSample>>,
    pub motions: Vec<Motion>,
}

/// Both controllers swept through the play space for `duration_s`, sighted at `frame_hz` and
/// sampled by their IMUs at `imu_hz`, all on one device clock.
pub fn trajectory(p: &TrajectoryParams) -> TrajectoryScene {
    let mut rng = Rng::new(p.seed);
    let base = scene(&SceneParams {
        samples: 0,
        ..Default::default()
    });
    let sp = SceneParams {
        pixel_noise: p.pixel_noise,
        pnp_rot_noise_rad: p.pnp_rot_noise_rad,
        pnp_pos_noise_m: p.pnp_pos_noise_m,
        flipped_fraction: p.flipped_fraction,
        ..Default::default()
    };
    let leds = touch_leds();
    let devices = [super::DEVICE_RIGHT, super::DEVICE_LEFT];
    let motions: Vec<Motion> = [Vector3::new(0.3, 1.1, 0.0), Vector3::new(-0.3, 1.1, 0.0)]
        .iter()
        .map(|c| Motion::random(&mut rng, *c))
        .collect();
    let t0_ns = 1_000_000_000u64;
    let (mut samples, mut world_from_obj) = (Vec::new(), Vec::new());
    let frames = (p.duration_s * p.frame_hz) as u64;
    for f in 0..frames {
        let t = f as f64 / p.frame_hz;
        let t_ns = t0_ns + (t * 1e9).round() as u64;
        for (d, m) in devices.iter().zip(&motions) {
            let w = m.pose(t);
            let views = sight(
                &sp,
                &mut rng,
                &base.world_from_cam,
                &base.cameras[0],
                &leds,
                &w,
            );
            if views.is_empty() {
                continue;
            }
            samples.push(WandSample {
                device_id: *d,
                device_time_ns: t_ns,
                host_ns: t_ns,
                views,
            });
            world_from_obj.push(w);
        }
    }
    let mut imu = BTreeMap::new();
    for (k, (d, m)) in devices.iter().zip(&motions).enumerate() {
        let n = (p.duration_s * p.imu_hz) as u64 + 1;
        let gravity = UnitQuaternion::from_scaled_axis(p.device_gravity_tilt[k]) * p.gravity;
        let series: Vec<ImuSample> = (0..n)
            .map(|i| {
                let t = i as f64 / p.imu_hz;
                ImuSample {
                    device_time_ns: t0_ns + (t * 1e9).round() as u64,
                    accel: m.accel(t, &p.lever_arm[k], &gravity)
                        + p.accel_bias[k]
                        + rng.gauss3(p.accel_noise),
                    gyro: m.gyro(t) + rng.gauss3(p.gyro_noise),
                }
            })
            .collect();
        imu.insert(*d, series);
    }
    let scene = Scene {
        leds: BTreeMap::from([(devices[0], leds.clone()), (devices[1], leds)]),
        samples,
        world_from_obj,
        ..base
    };
    TrajectoryScene {
        scene,
        imu,
        motions,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The generator is self-consistent: integrating its gyro reproduces its orientation, and its
    /// accelerometer reads gravity plus the IMU point's acceleration.
    #[test]
    fn trajectory_imu_matches_its_motion() {
        let tr = trajectory(&TrajectoryParams {
            duration_s: 4.0,
            ..Default::default()
        });
        let m = &tr.motions[0];
        let (t0, t1) = (0.5, 1.5);
        let mut q = m.pose(t0).rotation;
        let steps = 20000;
        for i in 0..steps {
            let t = t0 + (t1 - t0) * (i as f64 + 0.5) / steps as f64;
            q *= UnitQuaternion::from_scaled_axis(m.gyro(t) * ((t1 - t0) / steps as f64));
        }
        assert!(
            q.angle_to(&m.pose(t1).rotation) < 1e-6,
            "{}",
            q.angle_to(&m.pose(t1).rotation)
        );
        // at the model origin with no motion of the frame the accelerometer would read R^T G; here
        // check the finite-difference acceleration against an independent smaller step
        let g = Vector3::new(0.0, 9.80665, 0.0);
        let lever = Vector3::new(0.01, -0.02, 0.005);
        let h = 2e-4;
        let at = |t: f64| m.pose(t) * Point3::from(lever);
        let acc = (at(1.0 + h).coords - 2.0 * at(1.0).coords + at(1.0 - h).coords) / (h * h);
        assert!(
            (m.accel(1.0, &lever, &g) - m.pose(1.0).rotation.inverse() * (acc + g)).norm() < 1e-3
        );
        // 208 frames: both controllers are seen most of the time, often by two sensors at once
        let frames = (4.0 * 52.0) as usize;
        for d in [super::super::DEVICE_RIGHT, super::super::DEVICE_LEFT] {
            let seen = tr.scene.samples.iter().filter(|s| s.device_id == d).count();
            let multi = tr
                .scene
                .samples
                .iter()
                .filter(|s| s.device_id == d && s.views.len() >= 2)
                .count();
            assert!(
                seen * 10 > frames * 7 && multi * 10 > frames * 3,
                "device {d}: seen {seen} multi {multi} of {frames}"
            );
        }
    }

    #[test]
    fn every_sensor_sees_the_wand_often_and_pairs_overlap() {
        let sc = scene(&SceneParams::default());
        assert!(sc.samples.len() > 250, "{}", sc.samples.len());
        for c in 0..3 {
            let n = sc
                .samples
                .iter()
                .filter(|s| s.views.iter().any(|v| v.camera == c))
                .count();
            assert!(n > 100, "sensor {c} sees {n} samples");
        }
        let shared = sc.samples.iter().filter(|s| s.views.len() >= 2).count();
        assert!(shared > 100, "{shared}");
    }

    #[test]
    fn upside_down_cameras_have_image_up_along_world_up() {
        let desk = look_at(
            Vector3::new(-2.0, 2.3, -1.8),
            Vector3::new(0.0, 0.9, 0.0),
            true,
        );
        assert!((desk.rotation * Vector3::y()).y > 0.5); // image "down" points up in the world
        let tv = look_at(
            Vector3::new(0.3, 2.2, -2.6),
            Vector3::new(0.0, 0.9, 0.0),
            false,
        );
        assert!((tv.rotation * Vector3::y()).y < -0.5);
    }
}
