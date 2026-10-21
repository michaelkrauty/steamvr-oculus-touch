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

# Pure Windows path/error logic runs on both hosts, without a share or Win32 API calls.
test_win32_cache = executable('test_win32_cache', 'test_win32_cache.c',
  c_args: ['-I' + (rift_dir / 'src')])
test('win32_cache_paths', test_win32_cache, args: ['paths'])
test('win32_cache_errors', test_win32_cache, args: ['errors'])
test('win32_cache_separators', test_win32_cache, args: ['separators'])

# Below: a POSIX hardware probe, and harnesses that stub libusb or hidapi inside unity builds of the
# engine (GNU ld drops what they leave unresolved) or use POSIX threads and clocks. Linux only.
if windows
  subdir_done()
endif

# Instrument the unity build itself so an ordinary native test run catches null member access.
blobwatch_ubsan_args = ['-fsanitize=undefined', '-fno-sanitize-recover=undefined']
test_blobwatch_ubsan = executable('test_blobwatch_ubsan', 'test_blobwatch_ubsan.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include')] + blobwatch_ubsan_args,
  dependencies: [openhmd_dep], link_args: blobwatch_ubsan_args)
test('blobwatch_ubsan', test_blobwatch_ubsan)

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
test_uvc_frames = executable('test_uvc_frames', 'test_uvc_frames.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [dependency('libusb-1.0')], link_args: ['-Wl,--gc-sections', '-lm'])
foreach scenario : ['stale-tail', 'nearly-complete', 'header-only', 'fid-sequence',
                    'pts-sequence', 'malformed-header', 'error-flag', 'truncated-header',
                    'initial-boundary', 'startup-same-fid', 'startup-new-fid',
                    'startup-matching-tail', 'startup-fid-only', 'startup-no-pts',
                    'overflow', 'trailing-data', 'buffer-retention',
                    'packet-error', 'timestamp', 'jpeg', 'missing-pts', 'missing-header-end',
                    'same-fid-after-stall', 'gap-damage', 'discard-summary', 'starvation-log',
                    'opening-error-raw', 'opening-error-jpeg', 'bad-traffic-start', 'bad-traffic-rejected',
                    'raw-overrun-report', 'quality-counts', 'latency']
  test('uvc_frames_' + scenario, test_uvc_frames, args: [scenario])
endforeach
test_jpeg_dep = dependency('libjpeg', required: false)
test_sensor_frame_quality = executable('test_sensor_frame_quality', 'test_sensor_frame_quality.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'] +
    (test_jpeg_dep.found() ? ['-DHAVE_LIBJPEG=1'] : []),
  dependencies: [threads_dep, dependency('libusb-1.0'), test_jpeg_dep],
  link_args: ['-Wl,--gc-sections', '-lm'])
foreach scenario : ['layout', 'capture-guard']
  test('sensor_frame_' + scenario.underscorify(), test_sensor_frame_quality, args: [scenario])
endforeach
if test_jpeg_dep.found()
  foreach scenario : ['jpeg', 'jpeg-corrupt', 'jpeg-padding', 'jpeg-extraneous', 'jpeg-extraneous-header',
                      'jpeg-truncated', 'jpeg-missing-eoi', 'jpeg-bad-code', 'jpeg-resync', 'jpeg-warning-codes',
                      'jpeg-summary', 'jpeg-embedded-eoi', 'jpeg-progression', 'jpeg-counters']
    test('sensor_frame_' + scenario.underscorify(), test_sensor_frame_quality, args: [scenario])
  endforeach
endif
# Constructor-installed format and counters, with the actual packet assembler.
test_sensor_usb_quality = executable('test_sensor_usb_quality', 'test_sensor_usb_quality.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'] +
    (test_jpeg_dep.found() ? ['-DHAVE_LIBJPEG=1'] : []),
  dependencies: [dependency('libusb-1.0'), test_jpeg_dep], link_args: ['-Wl,--gc-sections', '-lm'])
test('sensor_usb_quality_raw', test_sensor_usb_quality, args: ['raw'])
if test_jpeg_dep.found()
  test('sensor_usb_quality_jpeg', test_sensor_usb_quality, args: ['jpeg'])
else
  test('sensor_usb_quality_jpeg_unavailable', test_sensor_usb_quality, args: ['jpeg'])
endif

# Hardware-free discovery/lifetime regression using the actual native tracker.
test_sensor_discovery = executable('test_sensor_discovery', 'test_sensor_discovery.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [openhmd_dep, dependency('libusb-1.0')], link_args: ['-Wl,--gc-sections'])
test('sensor_discovery', test_sensor_discovery)
# Real USB start/copy/reset and sensor getter; USB operations are stubbed.
test_sensor_usb_start = executable('test_sensor_usb_start', 'test_sensor_usb_start.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [openhmd_dep, dependency('libusb-1.0')], link_args: ['-Wl,--gc-sections'])
test('sensor_usb_start', test_sensor_usb_start)

# Camera delivery is counted before analysis; releasing old buffers is separate.
test_sensor_video = executable('test_sensor_video', 'test_sensor_video.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [openhmd_dep, dependency('libusb-1.0')], link_args: ['-Wl,--gc-sections'])
test('sensor_video', test_sensor_video)

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
# Both production readers report Touch activity, sleep beacons included.
test_radio_activity = executable('test_radio_activity', 'test_radio_activity.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [dependency('hidapi-hidraw')], link_args: ['-Wl,--gc-sections', '-lm'])
test('radio_activity', test_radio_activity)

# Optional diagnostic streams must also unwind allocation/connection failures.
test_pw_dep = dependency('libpipewire-0.3', required: false)
if test_pw_dep.found()
  test_pipewire_lifetime = executable('test_pipewire_lifetime', 'test_pipewire_lifetime.c',
    c_args: ['-DHAVE_PIPEWIRE=1', '-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
    dependencies: [test_pw_dep], link_args: ['-Wl,--gc-sections'])
  test('pipewire_lifetime', test_pipewire_lifetime)
endif

# Calibration parsing and file-read failures release owned buffers, without real HID or files.
test_touch_calibration = executable('test_touch_calibration', 'test_touch_calibration.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections'],
  dependencies: [dependency('hidapi-hidraw')], link_args: ['-Wl,--gc-sections', '-lm'])
test('touch_calibration_parser', test_touch_calibration, args: ['parser'])
test('touch_calibration_cache', test_touch_calibration, args: ['cache'])
test('touch_calibration_radio', test_touch_calibration, args: ['radio'])
test('touch_calibration_leds', test_touch_calibration, args: ['leds'])
test('touch_calibration_fallback', test_touch_calibration, args: ['fallback'])
test('touch_calibration_retry', test_touch_calibration, args: ['retry'])
test('touch_calibration_slow', test_touch_calibration, args: ['slow'])
test('touch_calibration_naps', test_touch_calibration, args: ['naps'])
test('touch_calibration_identity', test_touch_calibration, args: ['identity'])
test('touch_calibration_cache_alloc', test_touch_calibration, args: ['cache_alloc'])
test('touch_calibration_pending', test_touch_calibration, args: ['pending'])
test('touch_calibration_deadline', test_touch_calibration, args: ['deadline'])
test('touch_calibration_status', test_touch_calibration, args: ['status'])
test('touch_calibration_errno', test_touch_calibration, args: ['errno'])
test('touch_calibration_reader_oom', test_touch_calibration, args: ['reader_oom'])
test('touch_calibration_hid_error', test_touch_calibration, args: ['hid_error'])
test('touch_calibration_nodes', test_touch_calibration, args: ['nodes'])
test('touch_calibration_roots', test_touch_calibration, args: ['roots'])
test('touch_calibration_schema', test_touch_calibration, args: ['schema'])
test('touch_calibration_root_cache', test_touch_calibration, args: ['root_cache'])
test('touch_calibration_root_radio', test_touch_calibration, args: ['root_radio'])
test_read_file = executable('test_read_file', 'test_read_file.c',
  c_args: ['-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections', '-Wno-unused-parameter'],
  dependencies: [threads_dep, rt_dep], link_args: ['-Wl,--gc-sections'])
test('read_file', test_read_file)
test('touch_calibration_download', test_touch_calibration, args: ['download'])
test('touch_calibration_resync', test_touch_calibration, args: ['resync'])
test('touch_calibration_optional', test_touch_calibration, args: ['optional'])
test('touch_calibration_diagnostics', test_touch_calibration, args: ['diagnostics'])
test('touch_calibration_drain', test_touch_calibration, args: ['drain'])
test('touch_calibration_drain_slow', test_touch_calibration, args: ['drain_slow'])
test('touch_calibration_drain_notice', test_touch_calibration, args: ['drain_notice'])
test('touch_calibration_wake', test_touch_calibration, args: ['wake'])
test('touch_calibration_wake_reason', test_touch_calibration, args: ['wake_reason'])

# Cache I/O failures reach the caller without making valid controller calibration unusable.
test_config_write = executable('test_config_write', 'test_config_write.c',
  c_args: ['-I' + (rift_dir / 'src' / 'drv_oculus_rift'), '-I' + (rift_dir / 'src'), '-I' + (rift_dir / 'include'), '-ffunction-sections', '-fdata-sections', '-Wno-unused-parameter'],
  dependencies: [threads_dep, rt_dep, dependency('hidapi-hidraw')], link_args: ['-Wl,--gc-sections', '-lm'])
test('config_write_api', test_config_write, args: ['api'])
test('config_write_radio', test_config_write, args: ['radio'])
test('config_write_fresh', test_config_write, args: ['fresh'])
test('config_write_mode', test_config_write, args: ['mode'])
