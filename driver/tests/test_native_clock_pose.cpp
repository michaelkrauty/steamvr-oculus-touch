// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

// Receives actual Hand publications from examples/native_clock_pose.rs through stdin.
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
	const uint64_t now[] = {11'000'000'000, 11'062'000'000, 11'064'000'000};
	const uint64_t samples[] = {11'000'000'000, 11'002'000'000, 11'064'000'000};
	for (int i = 0; i < 3; ++i) {
		cv1t_controller_state s{};
		if (fread(&s, sizeof s, 1, stdin) != 1) return 2;
		const auto pose = cv1t::make_pose(&s, nullptr, 0, now[i]);
		const auto expected = i == 0 ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
		const double offset = i == 1 ? -0.060 : 0.0;
		printf("publication %d: sample=%llu fix=%llu result=%d timeOffset=%.3f velocity=%.1f\n",
			i, (unsigned long long)s.sample_host_ns,
			(unsigned long long)s.last_position_fix_host_ns, pose.result,
			pose.poseTimeOffset, pose.vecVelocity[0]);
		if (s.sample_host_ns != samples[i] || s.last_position_fix_host_ns != 10'800'000'000 ||
		    !(s.flags & CV1T_FLAG_POSITION) || !pose.deviceIsConnected || !pose.poseIsValid ||
		    pose.result != expected || std::fabs(pose.poseTimeOffset - offset) > 1e-9 ||
		    std::fabs(pose.vecVelocity[0] - 3.0) > 1e-9) return 1;
	}
	return 0;
}
