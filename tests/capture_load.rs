//! Calibration loader regressions using disposable synthetic recording files.
use cv1touch::{
    calib::{capture, DEVICE_LEFT, DEVICE_RIGHT},
    recorder::*,
    shm::{ControllerState, HAND_LEFT, HAND_RIGHT},
};
fn record(out: &mut Vec<u8>, typ: u16, payload: &[u8]) {
    out.extend(typ.to_le_bytes());
    out.extend(0u16.to_le_bytes());
    out.extend((payload.len() as u32).to_le_bytes());
    out.extend(1u64.to_le_bytes());
    out.extend(payload);
}
fn fixture(hands: &[(u32, u64, f32)]) -> Vec<u8> {
    // Historical v2 header and state reserved fields remain zero.
    let mut b = vec![0; 32];
    b[..8].copy_from_slice(MAGIC);
    b[8..12].copy_from_slice(&VERSION.to_le_bytes());
    b[12..16].copy_from_slice(&32u32.to_le_bytes());
    let mut sensor = SensorInfo {
        width: 1280,
        height: 960,
        fx: 715.,
        fy: 715.,
        cx: 640.,
        cy: 480.,
        ..Default::default()
    };
    sensor.serial[0] = b'S' as _;
    record(&mut b, SENSOR_INFO, as_bytes(&sensor));
    let led = LedEntry {
        pos: [0.01, 0., 0.],
        dir: [1., 0., 0.],
    };
    record(
        &mut b,
        LED_MODEL,
        &[&led_model_prefix(DEVICE_LEFT, 1)[..], as_bytes(&led)].concat(),
    );
    for &(hand, time, accel) in hands {
        let s = ControllerState {
            device_time_ns: time,
            imu_accel: [accel, 2., 3.],
            ..Default::default()
        };
        record(
            &mut b,
            CONTROLLER_STATE,
            &[&hand.to_le_bytes()[..], &[0; 4], as_bytes(&s)].concat(),
        );
    }
    b
}

struct Fixture(std::path::PathBuf);
impl Fixture {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!(
            "cv1touch-capture-load-{}-{name}",
            std::process::id()
        ));
        std::fs::create_dir(&dir).unwrap();
        Self(dir)
    }
    fn load(&self, states: &[(u32, u64, f32)]) -> Result<capture::Capture, String> {
        let path = self.0.join("synthetic.rec");
        std::fs::write(&path, fixture(states)).unwrap();
        capture::load(&path)
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
#[test]
fn rejects_hand_two() {
    rejects_unknown_controller_hand(2);
}
#[test]
fn rejects_hand_max() {
    rejects_unknown_controller_hand(u32::MAX);
}
fn rejects_unknown_controller_hand(hand: u32) {
    let fixture = Fixture::new(&format!("invalid-{hand}"));
    // Identity must be checked even when timestamp filtering would discard the sample.
    for time in [0, 2] {
        let error = fixture
            .load(&[(0, 1, 10.), (hand, time, 99.), (1, 3, 20.)])
            .expect_err("unknown hand must not produce a usable calibration capture");
        assert!(error.contains("controller hand"), "{error}");
        assert!(error.contains(&hand.to_string()), "{error}");
    }
}
#[test]
fn historical_v2_preserves_hand_mapping_and_timestamp_filters() {
    assert_eq!((HAND_LEFT, HAND_RIGHT), (0, 1));
    assert_eq!(VERSION, 2);
    let fixture = Fixture::new("historical");
    let capture = fixture
        .load(&[
            (0, 0, 99.),
            (0, 2, 10.),
            (1, 1, 20.),
            (0, 2, 99.),
            (0, 1, 99.),
            (1, 0, 99.),
            (0, 3, 11.),
            (1, 2, 21.),
        ])
        .unwrap();
    for (device, expected) in [
        (DEVICE_LEFT, [(2, 10.), (3, 11.)]),
        (DEVICE_RIGHT, [(1, 20.), (2, 21.)]),
    ] {
        let actual: Vec<_> = capture.imu[&device]
            .iter()
            .map(|s| (s.device_time_ns, s.accel.x))
            .collect();
        assert_eq!(actual, expected);
    }
}
