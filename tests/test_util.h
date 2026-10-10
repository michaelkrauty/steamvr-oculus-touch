// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Michael Krauty

#ifndef CV1T_TEST_UTIL_H
#define CV1T_TEST_UTIL_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(cond)                                                                    \
	do {                                                                               \
		if (!(cond)) {                                                                 \
			fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
			exit(1);                                                                   \
		}                                                                              \
	} while (0)

#define CHECK_NEAR(a, b, tol)                                                                   \
	do {                                                                                        \
		double _a = (a), _b = (b);                                                              \
		if (!(fabs(_a - _b) <= (tol))) {                                                        \
			fprintf(stderr, "%s:%d: %s = %.9g not within %.3g of %s = %.9g\n", __FILE__, __LINE__, \
			        #a, _a, (double)(tol), #b, _b);                                             \
			exit(1);                                                                            \
		}                                                                                       \
	} while (0)

#endif
