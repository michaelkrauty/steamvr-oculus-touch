// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

/* The engine's math symbols are hidden: expose the real source only to a subprocess test. */
#include "../third_party/openhmd/src/omath.c"

void cv1touch_test_native_assert(void)
{
    quatf quat = {{ NAN, 0.0f, 0.0f, 1.0f }};
    vec3f rotation;
    oquatf_to_rotation(&quat, &rotation);
}
