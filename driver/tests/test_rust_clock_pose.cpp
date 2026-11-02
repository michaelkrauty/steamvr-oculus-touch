// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

// Actual Pipeline publications -> actual daemon apply_tracked -> padding-free shared ABI.
// Invoked only by the offline clock boundary runner, not a daemon or IPC transport.
#include "pose.h"
#include <cmath>
#include <cstdio>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

int main()
{
#ifdef _WIN32
	_setmode(_fileno(stdin), _O_BINARY);
#endif
	bool failed = false;
	int rows = 0;
	uint64_t now;
	while (fread(&now, sizeof now, 1, stdin) == 1) {
		cv1t_controller_state normal{}, stalled{};
		if (fread(&normal, sizeof normal, 1, stdin) != 1 ||
		    fread(&stalled, sizeof stalled, 1, stdin) != 1) return 2;
		const auto a = cv1t::make_pose(&normal, nullptr, 0, now);
		const auto b = cv1t::make_pose(&stalled, nullptr, 0, now);
		printf("FIFO publication: age_ns=%llu fix_shift_ns=%lld offset=%.6f/%.6f\n",
			(unsigned long long)(now - normal.sample_host_ns),
			(long long)(stalled.last_position_fix_host_ns - normal.last_position_fix_host_ns),
			a.poseTimeOffset, b.poseTimeOffset);
		failed |= normal.sample_host_ns != stalled.sample_host_ns ||
			normal.last_position_fix_host_ns != stalled.last_position_fix_host_ns ||
			!(stalled.flags & CV1T_FLAG_POSITION) || !b.deviceIsConnected || !b.poseIsValid ||
			a.result != b.result || std::fabs(a.poseTimeOffset - b.poseTimeOffset) > 1e-12 ||
			std::fabs(b.poseTimeOffset + static_cast<double>(now - normal.sample_host_ns) * 1e-9) > 1e-12;
		for (int j = 0; j < 3; ++j)
			failed |= a.vecVelocity[j] != b.vecVelocity[j] ||
				a.vecPosition[j] != b.vecPosition[j];
		// Isolate the fix-age cutoff with a current sample. This is not a simulated
		// later session: both old device fixes must expire at the same host deadline.
		const auto deadline = normal.last_position_fix_host_ns + cv1t::kCoastNs + 1;
		normal.sample_host_ns = stalled.sample_host_ns = deadline;
		const auto ca = cv1t::make_pose(&normal, nullptr, 0, deadline);
		const auto cb = cv1t::make_pose(&stalled, nullptr, 0, deadline);
		printf("  isolated coast deadline: result=%d/%d velocity=%.6f/%.6f\n",
			ca.result, cb.result, a.vecVelocity[0], b.vecVelocity[0]);
		failed |= ca.result != vr::TrackingResult_Running_OutOfRange || cb.result != ca.result;
		++rows;
	}
	return failed || rows != 7 ? 1 : 0;
}
