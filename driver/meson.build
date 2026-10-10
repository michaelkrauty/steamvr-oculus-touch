# SteamVR driver: a thin C++ shim over the shared-memory ABI (spec "SteamVR driver").
openvr_dep = declare_dependency(include_directories: include_directories('../third_party/openvr/headers', is_system: true))
driver_inc = include_directories('../common', 'src')
driver_common = files('src/log.cpp', 'src/shm_client.cpp', 'src/pose.cpp', 'src/inputs.cpp', 'src/haptics.cpp', 'src/stations.cpp')

driver_lib = shared_library('driver_cv1touch',
  driver_common, 'src/driver_main.cpp', 'src/provider.cpp', 'src/touch_device.cpp',
  include_directories: driver_inc,
  dependencies: [openvr_dep, threads_dep],
  name_prefix: '',
  gnu_symbol_visibility: 'hidden',
  link_args: ['-static-libstdc++', '-static-libgcc', '-Wl,--exclude-libs,ALL', '-Wl,--no-undefined',
    '-Wl,--version-script=' + meson.current_source_dir() / 'driver.map'],
  link_depends: 'driver.map',
)

foreach t : ['test_pose', 'test_inputs', 'test_haptics', 'test_shm_client', 'test_stations']
  test(t, executable(t, 'tests/' + t + '.cpp', driver_common,
    include_directories: driver_inc,
    dependencies: [openvr_dep, threads_dep],
    cpp_args: ['-DCV1T_DRIVER_TEST']))
endforeach
test('driver_exports', find_program('tests/check_exports.sh'), args: [driver_lib])

# A peer closing between connect and write must not signal the vrserver process.
test('test_control_disconnect', executable('test_control_disconnect',
  'tests/test_control_disconnect.cpp', driver_common,
  include_directories: driver_inc,
  dependencies: [openvr_dep, threads_dep],
  cpp_args: ['-DCV1T_DRIVER_TEST'],
  link_args: ['-Wl,--wrap=write', '-Wl,--wrap=send']))
