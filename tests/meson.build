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
