// SPDX-License-Identifier: MIT
#pragma once

// The one function the AVX unit of the half-band chain exports (src/fx/HalfBandAvx.cpp, built with /arch:AVX on MSVC x64
// only). This header is deliberately tiny: it includes <cstddef> and nothing else, because the AVX translation unit must not
// see any engine header, std container or inline function: every inline function or template instantiated in a TU built
// with /arch:AVX becomes a COMDAT with VEX encoding that the linker may keep for the WHOLE binary, so CPUs without AVX
// could execute AVX instructions. The AVX unit therefore contains only this declaration's definition, raw intrinsics and
// internal-linkage code.

#include <cstddef>

// MSVC x64 (not ARM64EC, not clang-cl): the platform on which the AVX unit is built.
#if defined(_MSC_VER) && !defined(__clang__) && defined(_M_X64) && !defined(_M_ARM64EC)
#define CRUCIBLE_HALFBAND_AVX_PLATFORM 1
#else
#define CRUCIBLE_HALFBAND_AVX_PLATFORM 0
#endif

#if CRUCIBLE_HALFBAND_AVX_PLATFORM
namespace crucible::fx::detail
{
// The outputs of HalfBandChain::convolve in groups of 32 (as many as fit in n); returns how many it computed. Callers must
// have checked halfBandAvx() (CPUID + XCR0) first.
size_t halfBandConvolveAvx(const float* w, int taps, const float* x, size_t n, float* y);
} // namespace crucible::fx::detail
#endif
