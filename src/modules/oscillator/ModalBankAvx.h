// SPDX-License-Identifier: MIT
#pragma once

// The modal resonator bank of ModalOscillator.cpp (Bank::process) with eight-lane AVX filters, for MSVC x64 builds:
// ModalBankAvx.cpp is the only file compiled with /arch:AVX, and ModalOscillator.cpp calls it only when the CPU and the OS
// support AVX. Every lane does the same multiplies and adds as the four-lane SSE code, and the per-pickup sums still add the
// modes four lanes at a time in mode order, so the output is bit-identical to Bank::process (no FMA: AVX has none, and MSVC
// does not contract under /fp:precise).

#if defined(_MSC_VER) && !defined(__clang__) && defined(_M_X64) && !defined(_M_ARM64EC)
#define CRUCIBLE_MODAL_AVX 1
#else
#define CRUCIBLE_MODAL_AVX 0
#endif

namespace crucible::modules::modal
{

// n samples of x through `padded` modes (a multiple of four), the two pickups (weights ramp from *0 to *1) added into c and
// s.
void bankProcessAvx(const float* x, int n, int padded, const float* g, const float* rg, const float* h, float* s1, float* s2,
                    const float* wc0, const float* wc1, const float* ws0, const float* ws1, float* c, float* s);

} // namespace crucible::modules::modal
