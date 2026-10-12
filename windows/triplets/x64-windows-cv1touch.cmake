# Static libraries on the static C runtime, linked into openhmd.dll, except libusb (LGPL-2.1),
# which ships as its own DLL beside the executables.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE static)
set(VCPKG_LIBRARY_LINKAGE static)
# the engine links release libraries in every build type (b_vscrt picks /MT)
set(VCPKG_BUILD_TYPE release)
if(PORT STREQUAL "libusb")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()
