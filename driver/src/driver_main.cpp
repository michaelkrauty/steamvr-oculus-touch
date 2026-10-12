// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <cstring>

#include <openvr_driver.h>

#include "provider.h"

#ifdef _WIN32
#define CV1T_EXPORT __declspec(dllexport)
#else
#define CV1T_EXPORT __attribute__((visibility("default")))
#endif

static cv1t::Provider g_provider;

extern "C" CV1T_EXPORT void *HmdDriverFactory(const char *interface_name, int *return_code)
{
	if (std::strcmp(interface_name, vr::IServerTrackedDeviceProvider_Version) == 0)
		return &g_provider;
	if (return_code)
		*return_code = vr::VRInitError_Init_InterfaceNotFound;
	return nullptr;
}
