# SteamVR driver: a thin C++ shim over the shared-memory ABI.
openvr_dep = declare_dependency(include_directories: include_directories('../third_party/openvr/headers', is_system: true))
driver_inc = include_directories('../common', 'src')
driver_common = files('src/log.cpp', 'src/shm_client.cpp', 'src/pose.cpp', 'src/inputs.cpp', 'src/haptics.cpp', 'src/stations.cpp')

windows = host_machine.system() == 'windows'
if windows
  # the export is __declspec(dllexport); the static runtime comes from b_vscrt
  driver_link_args = []
  driver_link_depends = []
else
  driver_link_args = ['-static-libstdc++', '-static-libgcc', '-Wl,--exclude-libs,ALL', '-Wl,--no-undefined',
    '-Wl,--version-script=' + meson.current_source_dir() / 'driver.map']
  driver_link_depends = 'driver.map'
endif

driver_lib = shared_library('driver_cv1touch',
  driver_common, 'src/driver_main.cpp', 'src/provider.cpp', 'src/touch_device.cpp',
  include_directories: driver_inc,
  dependencies: [openvr_dep, threads_dep],
  name_prefix: '',
  gnu_symbol_visibility: 'hidden',
  link_args: driver_link_args,
  link_depends: driver_link_depends,
)

foreach t : ['test_pose', 'test_inputs', 'test_haptics', 'test_shm_client', 'test_stations']
  test(t, executable(t, 'tests/' + t + '.cpp', driver_common,
    include_directories: driver_inc,
    dependencies: [openvr_dep, threads_dep],
    cpp_args: ['-DCV1T_DRIVER_TEST']))
endforeach
# Run by examples/driver_contract.rs against the daemon's own shared-memory and control code.
contract_test = executable('test_daemon_contract', 'tests/test_daemon_contract.cpp', driver_common,
  include_directories: driver_inc,
  dependencies: [openvr_dep, threads_dep],
  cpp_args: ['-DCV1T_DRIVER_TEST'])
test('test_daemon_contract_timing', contract_test, args: ['--timing'])
if windows
  test('driver_exports', import('python').find_installation(), args: [files('tests/check_exports.py'), driver_lib])
  subdir_done()
endif
test('driver_exports', find_program('tests/check_exports.sh'), args: [driver_lib])

test('test_control_client', executable('test_control_client', 'tests/test_control_client.cpp', driver_common,
  include_directories: driver_inc,
  dependencies: [openvr_dep, threads_dep],
  cpp_args: ['-DCV1T_DRIVER_TEST']))

# A peer closing between connect and write must not signal the vrserver process.
test('test_control_disconnect', executable('test_control_disconnect',
  'tests/test_control_disconnect.cpp', driver_common,
  include_directories: driver_inc,
  dependencies: [openvr_dep, threads_dep],
  cpp_args: ['-DCV1T_DRIVER_TEST'],
  link_args: ['-Wl,--wrap=write', '-Wl,--wrap=send']))

# Check timeout configuration with replies already queued, independent of scheduling.
test('test_control_timeout', executable('test_control_timeout',
  'tests/test_control_timeout.cpp', driver_common,
  include_directories: driver_inc,
  dependencies: [openvr_dep, threads_dep],
  cpp_args: ['-DCV1T_DRIVER_TEST'],
  link_args: ['-Wl,--wrap=connect', '-Wl,--wrap=setsockopt']))
