// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
// Deliberately includes the implementation to measure its private selection helper too.
#include "../src/pose.cpp"
#include "age_report_oracle.h"
#include <array>
#include <chrono>
#include <cstdlib>

namespace {
struct Stats { uint64_t p50, p99, max; };
Stats sorted(std::vector<uint64_t> &v)
{
	std::sort(v.begin(), v.end());
	return {v[static_cast<size_t>(0.5 * (v.size() - 1))],
		v[static_cast<size_t>(0.99 * (v.size() - 1))], v.back()};
}
Stats selected(std::vector<uint64_t> &v)
{
#ifdef AGE_REPORT_BASELINE
	return sorted(v);
#else
	const auto s = cv1t::age_percentiles(v);
	return {s.p50, s.p99, s.max};
#endif
}
uint64_t checksum = 0;
constexpr size_t batch = 32;
using Inputs = std::array<std::vector<uint64_t>, batch>;

double measure(const Inputs &ages, const Inputs &handoffs, bool candidate, bool selection)
{
	// All input allocation/copying is outside the measured region, for both implementations.
	auto a = ages, h = handoffs;
	std::array<std::string, batch> reports;
	std::array<Stats, batch> sa{}, sh{};
	const auto start = std::chrono::steady_clock::now();
	if (selection) {
		auto fn = candidate ? selected : sorted;
		for (size_t i = 0; i < batch; ++i) {
			sa[i] = fn(a[i]);
			if (!h[i].empty()) sh[i] = fn(h[i]);
		}
	} else {
		auto fn = candidate ? cv1t::age_report : age_report_test::age_report;
		for (size_t i = 0; i < batch; ++i) reports[i] = fn("CV1T-TEST", a[i], h[i]);
	}
	const auto stop = std::chrono::steady_clock::now();
	for (size_t i = 0; i < batch; ++i) {
		for (unsigned char c : reports[i]) checksum = checksum * 33 + c;
		checksum += sa[i].p50 + sa[i].p99 + sa[i].max + sh[i].p50 + sh[i].p99 + sh[i].max;
	}
	return std::chrono::duration<double, std::nano>(stop - start).count() / batch;
}
}

int main()
{
	const char *names[] = {"random", "ordered", "reversed", "constant", "duplicates", "large"};
	puts("size,distribution,handoffs,operation,round,sort_ns,candidate_ns");
	for (size_t n : {1, 2, 3, 100, 1000, 10000}) {
		for (int d = 0; d < 6; ++d) {
			for (bool handoffs : {false, true}) {
				Inputs a, h;
				for (size_t i = 0; i < batch; ++i) {
					a[i] = age_report_test::samples(n, d, 5100 + i);
					if (handoffs) h[i] = age_report_test::samples(n, d, 8100 + i);
					auto oa = a[i], oh = h[i], ca = a[i], ch = h[i];
					if (cv1t::age_report("CV1T-TEST", ca, ch) != age_report_test::age_report("CV1T-TEST", oa, oh))
						std::abort();
				}
				for (bool selection : {false, true}) {
					for (int round = -2; round < 9; ++round) {
						double old_ns, new_ns;
						if (round % 2 == 0) {
							old_ns = measure(a, h, false, selection);
							new_ns = measure(a, h, true, selection);
						} else {
							new_ns = measure(a, h, true, selection);
							old_ns = measure(a, h, false, selection);
						}
						if (round >= 0) printf("%zu,%s,%d,%s,%d,%.2f,%.2f\n", n, names[d], handoffs,
							selection ? "selection" : "report", round, old_ns, new_ns);
					}
				}
			}
		}
	}
	fprintf(stderr, "checksum=%llu\n", static_cast<unsigned long long>(checksum));
}
