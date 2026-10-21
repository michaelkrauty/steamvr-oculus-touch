// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#include <cstdint>

/* Socket tests check the exchange, not how quickly another thread gets CPU time. */
inline constexpr uint32_t kControlTestTimeoutMs = 5'000;
