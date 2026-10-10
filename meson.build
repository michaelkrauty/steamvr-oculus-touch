project('cv1touch', 'c', 'cpp',
  version: '0.1.0',
  meson_version: '>= 1.3',
  subproject_dir: 'third_party',
  default_options: ['c_std=gnu11', 'cpp_std=c++20', 'warning_level=2', 'buildtype=debugoptimized'])

if host_machine.system() != 'linux' or host_machine.cpu_family() != 'x86_64'
  error('cv1touch supports Linux x86_64 only')
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
