project('cv1touch', 'c', 'cpp',
  version: '0.2.0',
  meson_version: '>= 1.3',
  subproject_dir: 'third_party',
  # b_vscrt (Windows): the driver carries its C/C++ runtime, as -static-libstdc++ does on Linux
  default_options: ['c_std=gnu11,c11', 'cpp_std=c++20', 'warning_level=2', 'buildtype=debugoptimized',
    'b_vscrt=static_from_buildtype'])

if host_machine.cpu_family() != 'x86_64' or host_machine.system() not in ['linux', 'windows']
  error('cv1touch supports Linux and Windows on x86_64')
endif

if host_machine.system() == 'windows'
  # The engine is not ported yet: the SteamVR driver and its tests only. Build with clang-cl
  # (CC=clang-cl CXX=clang-cl): the shared-memory helpers use the GNU __atomic builtins.
  if meson.get_compiler('cpp').get_id() != 'clang-cl'
    error('build the Windows driver with clang-cl (CC=clang-cl CXX=clang-cl)')
  endif
  threads_dep = dependency('threads')
  subdir('driver')
  subdir_done()
endif

openhmd_proj = subproject('openhmd', default_options: [
  'drivers=rift',
  'examples=[]',
  'tests=false',
  # PipeWire video of every sensor (raw and annotated); it costs nothing until a viewer connects
  'debugging_mode=true',
  'default_library=shared',
  # the kernel's usbhid keeps the headset's interrupt endpoints polled every frame and wakes the
  # reader itself; hidapi's libusb backend resubmits one transfer per report from a thread of its
  # own, and a late resubmit skips a poll (the headset then packs two IMU samples in one report)
  'hidapi=hidraw',
])
openhmd_dep = openhmd_proj.get_variable('openhmd_dep')

threads_dep = dependency('threads')
rt_dep = meson.get_compiler('c').find_library('rt', required: false)

subdir('tests')
subdir('driver')

# Standalone nxjson Unicode scalar and parser boundary regressions.
test('nxjson_unicode', executable('test_nxjson_unicode', 'tests/test_nxjson_unicode.c'))
