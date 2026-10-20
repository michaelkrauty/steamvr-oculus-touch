windows = host_machine.system() == 'windows'
libm = windows ? [] : ['-lm']

test_smoke = executable('test_smoke', 'test_smoke.c', dependencies: [openhmd_dep])
test('smoke', test_smoke)
test_engine_api = executable('test_engine_api', 'test_engine_api.c', dependencies: [openhmd_dep])
test('engine_api', test_engine_api)
# Unity build of the engine's blob extraction (meson forbids grabbing subproject sources directly)
rift_dir = meson.project_source_root() / 'third_party' / 'openhmd'
test_daemon_log = executable('test_daemon_log', 'test_daemon_log.c',
  c_args: ['-DOHMD_NONBLOCK_LOG', '-DOHMD_STATIC', '-I' + (rift_dir / 'include')],
  dependencies: [dependency('libusb-1.0')])
test('daemon_log', test_daemon_log)
assert_python = import('python').find_installation()
test_log_reentry = executable('test_native_log_reentry', 'test_native_log_reentry.c',
  c_args: ['-DOHMD_NONBLOCK_LOG', '-DOHMD_STATIC', '-I' + (rift_dir / 'include')],
  dependencies: [dependency('libusb-1.0')])
test('native_log_reentry', assert_python,
  args: [files('check_native_log_reentry.py'), test_log_reentry])
test_assert_routed = executable('test_native_assert_routed', 'test_native_assert.cpp',
  dependencies: [openhmd_dep])
test('native_assert_routed', assert_python,
  args: [files('check_native_assert.py'), test_assert_routed, 'routed'])
test_assert_standalone = executable('test_native_assert_standalone', 'test_native_assert.cpp')
test('native_assert_standalone', assert_python,
  args: [files('check_native_assert.py'), test_assert_standalone, 'standalone'])
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
  dependencies: [openhmd_dep], link_args: libm)
test('imu_calib', test_imu_calib)
test_fixed_pose = executable('test_fixed_pose', 'test_fixed_pose.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep], link_args: libm)
test('fixed_pose', test_fixed_pose)
test_device_clock = executable('test_device_clock', 'test_device_clock.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift')])
test('device_clock', test_device_clock)
test_radio_decode = executable('test_radio_decode', 'test_radio_decode.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep], link_args: libm)
test('radio_decode', test_radio_decode)
test_uvc_layout = executable('test_uvc_layout', 'test_uvc_layout.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift' / 'sensor')])
test('uvc_layout', test_uvc_layout)
test_sensor_config = executable('test_sensor_config', 'test_sensor_config.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')],
  dependencies: [openhmd_dep], link_args: libm)
test('sensor_config', test_sensor_config)
# The engine's config cache directory is where cv1touch looks (a unity build of the platform layer).
test_config_dir = executable('test_config_dir', 'test_config_dir.c',
  c_args: ['-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-Wno-unused-parameter'] +
    (windows ? ['-D_CRT_SECURE_NO_WARNINGS', '-DOHMD_STATIC'] : []),
  dependencies: [threads_dep, rt_dep])
test('config_dir', test_config_dir)

# Below: a POSIX hardware probe, and harnesses that stub libusb or hidapi inside unity builds of the
# engine (GNU ld drops what they leave unresolved) or use POSIX threads and clocks. Linux only.
if windows
  subdir_done()
endif

# A dlopen-only test seam for the daemon's real logger and the engine's math assertion.
shared_library('test_native_assert_bridge', 'native_assert_bridge.c',
  c_args: ['-DOHMD_NONBLOCK_LOG'], dependencies: [openhmd_dep], link_args: libm)

engine_probe = executable('engine_probe', 'engine_probe.c', dependencies: [openhmd_dep])
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
# The brute-force search leaves out the devices cv1touch's tracker holds or that are asleep.
test_search_skip = executable('test_search_skip', 'test_search_skip.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections',
    '-Wno-sign-compare'],
  link_args: ['-Wl,--gc-sections', '-lm'])
test('search_skip', test_search_skip)
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
