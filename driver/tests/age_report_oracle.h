// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace age_report_test {
// The original full-sort implementation, kept independent of production selection.
inline std::string age_report(const char *serial, std::vector<uint64_t> &ages, std::vector<uint64_t> &handoffs)
{
	std::sort(ages.begin(), ages.end());
	std::sort(handoffs.begin(), handoffs.end());
	const auto at = [](const std::vector<uint64_t> &v, double q) { return v[static_cast<size_t>(q * (v.size() - 1))] * 1e-6; };
	char buf[320];
	int n = snprintf(buf, sizeof buf, "cv1touch: %s pose age at submission over %zu poses: p50 %.2f ms p99 %.2f ms max %.2f ms", serial,
			 ages.size(), at(ages, 0.5), at(ages, 0.99), at(ages, 1.0));
	if (!handoffs.empty() && n > 0 && static_cast<size_t>(n) < sizeof buf)
		snprintf(buf + n, sizeof buf - n, "; of it after the daemon published: p50 %.2f ms p99 %.2f ms max %.2f ms", at(handoffs, 0.5),
			 at(handoffs, 0.99), at(handoffs, 1.0));
	return buf;
}

inline std::vector<uint64_t> samples(size_t n, int distribution, uint64_t seed)
{
	std::mt19937_64 rng(seed);
	std::vector<uint64_t> v(n);
	for (size_t i = 0; i < n; ++i) {
		switch (distribution) {
		case 0: v[i] = rng() % 100'000'000; break;
		case 1: v[i] = i * 1'000'001; break;
		case 2: v[i] = (n - i) * 1'000'001; break;
		case 3: v[i] = 12'345'678; break;
		case 4: v[i] = (rng() % 5) * 1'005'000; break;
		default: v[i] = rng(); break; // full uint64 range
		}
	}
	return v;
}
} // namespace age_report_test
