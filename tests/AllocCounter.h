// SPDX-License-Identifier: MIT
#pragma once

// Allocation counter of crucible_engine_tests. operator new is replaced in PlatformTests.cpp: while gTrackAllocations
// is set, every allocation of the calling thread increments gAllocations.

#include "AllocCounterConfig.h"

namespace crucible::testing
{
extern thread_local bool gTrackAllocations;
extern thread_local long gAllocations;
} // namespace crucible::testing
