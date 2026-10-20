// SPDX-License-Identifier: MIT
#pragma once

// The regular step of the second-order ADAA of Saturation.cpp (adaa2Update with both differences above the expansion
// threshold) for the two channels of a pair, two samples at a time in the four lanes of an AVX register, for MSVC x64
// builds: SaturationAvx.cpp is compiled with /arch:AVX, and Saturation.cpp calls it only when the CPU and the OS support
// AVX. Every lane does the scalar step's multiplies, adds and division in their order (AVX has no FMA, and MSVC does not
// contract intrinsics), the table look-up picks the same cell, offset and extension node as ShapeTable::locate, and the
// DC blocker runs sample by sample as in adaa2Run, so the output is bit-identical to the scalar and SSE2 kernels. The
// kernel stops at the first pair of samples on which any lane needs the expansions; the caller runs those through
// adaa2Update.

#if defined(_MSC_VER) && !defined(__clang__) && defined(_M_X64) && !defined(_M_ARM64EC)
#define CRUCIBLE_SATURATION_AVX 1
#else
#define CRUCIBLE_SATURATION_AVX 0
#endif

namespace crucible::saturation::avx
{

// The state of both channels (index 0: the first channel): x[n-1], x[n-2], F2(x[n-1]), F2[x[n-1], x[n-2]], DC blocker.
struct PairState
{
    double xPrev[2], xPrev2[2], F2Prev[2], divPrev[2], hp[2];
};

// A ShapeTable's packed cells (f, F, F2, slope per node; cells + 1 nodes, then the left and the right extension).
struct TableView
{
    const double* nodes;
    double range, invStep, step;
    int cells;
};

// The DC blocker of adaa2Run: s' = s keep + in y, y' = out (y - s).
struct DcCoefficients
{
    bool on;
    double keep, in, out;
};

// From sample i = start, while i + 2 <= len and every lane of samples i and i + 1 is regular: y = 2 F2[x0, x1, x2] through
// the DC blocker into out0 / out1. Returns the first sample not processed; st holds the state before it.
int adaa2PairTable(const TableView& t, PairState& st, const DcCoefficients& dc, const double* xs0, const double* xs1,
                   float* out0, float* out1, int start, int len);
// The same with the wavefolder's closed-form F2 (FoldCurve::F2).
int adaa2PairFold(PairState& st, const DcCoefficients& dc, const double* xs0, const double* xs1, float* out0, float* out1,
                  int start, int len);

// The tube sag of both channels (adaa2Sag): the shift 0.6 sag, the envelope's attack and release coefficients, the envelope.
struct SagLanes
{
    double depth[2], atk[2], rel[2], env[2];
};
// adaa2Sag of both channels on a tile in place (the level of FastTableCurve::f at the unshifted input, x - depth env, then
// env + (a - env) (a > env ? attack : release)), the level of two samples at a time in the four lanes.
void sagPairTable(const TableView& t, SagLanes& sag, double* xs0, double* xs1, int len);

} // namespace crucible::saturation::avx
