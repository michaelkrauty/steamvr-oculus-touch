test_smoke = executable('test_smoke', 'test_smoke.c', dependencies: [openhmd_dep])
test('smoke', test_smoke)
test_engine_api = executable('test_engine_api', 'test_engine_api.c', dependencies: [openhmd_dep])
test('engine_api', test_engine_api)
engine_probe = executable('engine_probe', 'engine_probe.c', dependencies: [openhmd_dep])
# Unity build of the engine's blob extraction (meson forbids grabbing subproject sources directly)
rift_dir = meson.project_source_root() / 'third_party' / 'openhmd'
test_blob_shape = executable('test_blob_shape', 'test_blob_shape.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep])
test('blob_shape', test_blob_shape)
test_blob_skip = executable('test_blob_skip', 'test_blob_skip.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep])
test('blob_skip', test_blob_skip)
test_imu_calib = executable('test_imu_calib', 'test_imu_calib.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep], link_args: ['-lm'])
test('imu_calib', test_imu_calib)
test_fixed_pose = executable('test_fixed_pose', 'test_fixed_pose.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep], link_args: ['-lm'])
test('fixed_pose', test_fixed_pose)
test_device_clock = executable('test_device_clock', 'test_device_clock.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift')])
test('device_clock', test_device_clock)
test_radio_decode = executable('test_radio_decode', 'test_radio_decode.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep], link_args: ['-lm'])
test('radio_decode', test_radio_decode)
test_uvc_layout = executable('test_uvc_layout', 'test_uvc_layout.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift' / 'sensor')])
test('uvc_layout', test_uvc_layout)
# Deterministic callback/stop lock inversion; no device access.
test_uvc_shutdown = executable('test_uvc_shutdown', 'test_uvc_shutdown.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [threads_dep, dependency('libusb-1.0')],
  link_args: ['-Wl,--gc-sections'])
test('uvc_shutdown', test_uvc_shutdown, timeout: 10)
# Hardware-free discovery/lifetime regression using the actual native tracker.
test_sensor_discovery = executable('test_sensor_discovery', 'test_sensor_discovery.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [openhmd_dep, dependency('libusb-1.0')], link_args: ['-Wl,--gc-sections'])
test('sensor_discovery', test_sensor_discovery)
test_sensor_config = executable('test_sensor_config', 'test_sensor_config.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep], link_args: ['-lm'])
test('sensor_config', test_sensor_config)
test_sensor_lifetime = executable('test_sensor_lifetime', 'test_sensor_lifetime.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [openhmd_dep, dependency('libusb-1.0')], link_args: ['-Wl,--gc-sections', '-lm'])
test('sensor_lifetime', test_sensor_lifetime)

test_hmd_tracker_failure = executable('test_hmd_tracker_failure', 'test_hmd_tracker_failure.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [dependency('hidapi-hidraw')], link_args: ['-Wl,--gc-sections', '-lm'])
test('hmd_tracker_failure', test_hmd_tracker_failure)

# Optional diagnostic streams must also unwind allocation/connection failures.
test_pw_dep = dependency('libpipewire-0.3', required: false)
if test_pw_dep.found()
  test_pipewire_lifetime = executable('test_pipewire_lifetime', 'test_pipewire_lifetime.c',
    c_args: ['-DHAVE_PIPEWIRE=1', '-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
    dependencies: [test_pw_dep], link_args: ['-Wl,--gc-sections'])
  test('pipewire_lifetime', test_pipewire_lifetime)
endif
