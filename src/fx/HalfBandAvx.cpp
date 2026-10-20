// SPDX-License-Identifier: MIT
// The dot products of crucible::fx::HalfBandChain (include/crucible/fx/HalfBandChain.h) eight outputs per AVX register.
// Built with /arch:AVX on MSVC x64 (CMakeLists.txt) and called only when the CPU and the OS support AVX
// (detail::halfBandAvx(), src/fx/FxCore.cpp). Each lane is one output with the scalar loop's multiply and add per tap in
// tap order from an accumulator of 0 (AVX has no FMA, MSVC does not contract intrinsics), so the outputs are bit-identical
// to the scalar and SSE loops. Raw intrinsics only; this file includes ONLY crucible/fx/HalfBandAvxKernel.h (no engine
// header, no std container, no inline function from elsewhere) so that no VEX-encoded COMDAT can leak into the rest of the
// binary.
#include <crucible/fx/HalfBandAvxKernel.h>

#if CRUCIBLE_HALFBAND_AVX_PLATFORM

#include <immintrin.h>

namespace crucible::fx::detail
{

size_t halfBandConvolveAvx(const float* w, int taps, const float* x, size_t n, float* y)
{
    size_t i = 0;
    for (; i + 64 <= n; i += 64)
    {
        const float* xi = x + i;
        __m256 a0 = _mm256_setzero_ps(), a1 = a0, a2 = a0, a3 = a0, a4 = a0, a5 = a0, a6 = a0, a7 = a0;
        for (int k = 0; k < taps; ++k)
        {
            const __m256 wk = _mm256_broadcast_ss(w + k);
            const float* p = xi + k;
            a0 = _mm256_add_ps(a0, _mm256_mul_ps(wk, _mm256_loadu_ps(p)));
            a1 = _mm256_add_ps(a1, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 8)));
            a2 = _mm256_add_ps(a2, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 16)));
            a3 = _mm256_add_ps(a3, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 24)));
            a4 = _mm256_add_ps(a4, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 32)));
            a5 = _mm256_add_ps(a5, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 40)));
            a6 = _mm256_add_ps(a6, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 48)));
            a7 = _mm256_add_ps(a7, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 56)));
        }
        _mm256_storeu_ps(y + i, a0);
        _mm256_storeu_ps(y + i + 8, a1);
        _mm256_storeu_ps(y + i + 16, a2);
        _mm256_storeu_ps(y + i + 24, a3);
        _mm256_storeu_ps(y + i + 32, a4);
        _mm256_storeu_ps(y + i + 40, a5);
        _mm256_storeu_ps(y + i + 48, a6);
        _mm256_storeu_ps(y + i + 56, a7);
    }
    for (; i + 32 <= n; i += 32)
    {
        const float* xi = x + i;
        __m256 a0 = _mm256_setzero_ps(), a1 = a0, a2 = a0, a3 = a0;
        for (int k = 0; k < taps; ++k)
        {
            const __m256 wk = _mm256_broadcast_ss(w + k);
            const float* p = xi + k;
            a0 = _mm256_add_ps(a0, _mm256_mul_ps(wk, _mm256_loadu_ps(p)));
            a1 = _mm256_add_ps(a1, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 8)));
            a2 = _mm256_add_ps(a2, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 16)));
            a3 = _mm256_add_ps(a3, _mm256_mul_ps(wk, _mm256_loadu_ps(p + 24)));
        }
        _mm256_storeu_ps(y + i, a0);
        _mm256_storeu_ps(y + i + 8, a1);
        _mm256_storeu_ps(y + i + 16, a2);
        _mm256_storeu_ps(y + i + 24, a3);
    }
    _mm256_zeroupper();
    return i;
}

} // namespace crucible::fx::detail

#endif
