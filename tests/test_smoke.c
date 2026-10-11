// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <openhmd.h>

#include "test_util.h"

int main(void)
{
	int major = -1, minor = -1, patch = -1;
	ohmd_get_version(&major, &minor, &patch);
	CHECK(major == 0 && minor == 3);
	return 0;
}
