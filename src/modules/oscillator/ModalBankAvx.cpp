// SPDX-License-Identifier: MIT
// Compiled with /arch:AVX on MSVC x64 (CMakeLists.txt); see ModalBankAvx.h. Called only on CPUs with AVX.
//
// Raw intrinsics and internal-linkage helpers only: an inline function from a shared header (crucible/Simd.h, the standard
// library) instantiated here would be VEX-encoded, and the linker may keep this copy for the whole program.
#include "ModalBankAvx.h"

#if CRUCIBLE_MODAL_AVX

#include <immintrin.h>

namespace crucible::modules::modal
{
namespace
{
// (l0 + l1) + (l2 + l3), as simd::Float4::sum.
float sum4(__m128 v)
{
    const __m128 pairs = _mm_add_ps(v, _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 3, 0, 1)));
    return _mm_cvtss_f32(_mm_add_ss(pairs, _mm_movehl_ps(pairs, pairs)));
}
} // namespace

void bankProcessAvx(const float* x, int n, int padded, const float* g, const float* rg, const float* h, float* s1, float* s2,
                    const float* wc0, const float* wc1, const float* ws0, const float* ws1, float* c, float* s)
{
    const float inv = 1.0f / static_cast<float>(n);
    for (int t = 0; t < n; ++t)
    {
        const __m256 in8 = _mm256_set1_ps(x[t]);
        __m128 c0 = _mm_setzero_ps(), c1 = c0, d0 = c0, d1 = c0;
        int i = 0;
        // Eight modes per step: the filters eight lanes wide; each pickup adds the lower four modes, then the upper four
        // (the order of the four-lane loop).
        for (; i + 8 <= padded; i += 8)
        {
            const __m256 gv = _mm256_loadu_ps(g + i);
            const __m256 st1 = _mm256_loadu_ps(s1 + i);
            const __m256 st2 = _mm256_loadu_ps(s2 + i);
            const __m256 hp = _mm256_mul_ps(
                _mm256_sub_ps(_mm256_sub_ps(in8, _mm256_mul_ps(_mm256_loadu_ps(rg + i), st1)), st2), _mm256_loadu_ps(h + i));
            const __m256 ghp = _mm256_mul_ps(gv, hp);
            const __m256 bp = _mm256_add_ps(ghp, st1);
            _mm256_storeu_ps(s1 + i, _mm256_add_ps(ghp, bp));
            const __m256 gbp = _mm256_mul_ps(gv, bp);
            const __m256 lp = _mm256_add_ps(gbp, st2);
            _mm256_storeu_ps(s2 + i, _mm256_add_ps(gbp, lp));
            const __m256 p0 = _mm256_mul_ps(_mm256_loadu_ps(wc0 + i), lp);
            const __m256 p1 = _mm256_mul_ps(_mm256_loadu_ps(wc1 + i), lp);
            const __m256 q0 = _mm256_mul_ps(_mm256_loadu_ps(ws0 + i), lp);
            const __m256 q1 = _mm256_mul_ps(_mm256_loadu_ps(ws1 + i), lp);
            c0 = _mm_add_ps(_mm_add_ps(c0, _mm256_castps256_ps128(p0)), _mm256_extractf128_ps(p0, 1));
            c1 = _mm_add_ps(_mm_add_ps(c1, _mm256_castps256_ps128(p1)), _mm256_extractf128_ps(p1, 1));
            d0 = _mm_add_ps(_mm_add_ps(d0, _mm256_castps256_ps128(q0)), _mm256_extractf128_ps(q0, 1));
            d1 = _mm_add_ps(_mm_add_ps(d1, _mm256_castps256_ps128(q1)), _mm256_extractf128_ps(q1, 1));
        }
        // Four modes left: the four-lane form.
        for (; i < padded; i += 4)
        {
            const __m128 in = _mm256_castps256_ps128(in8);
            const __m128 gv = _mm_loadu_ps(g + i);
            const __m128 st1 = _mm_loadu_ps(s1 + i);
            const __m128 st2 = _mm_loadu_ps(s2 + i);
            const __m128 hp =
                _mm_mul_ps(_mm_sub_ps(_mm_sub_ps(in, _mm_mul_ps(_mm_loadu_ps(rg + i), st1)), st2), _mm_loadu_ps(h + i));
            const __m128 ghp = _mm_mul_ps(gv, hp);
            const __m128 bp = _mm_add_ps(ghp, st1);
            _mm_storeu_ps(s1 + i, _mm_add_ps(ghp, bp));
            const __m128 gbp = _mm_mul_ps(gv, bp);
            const __m128 lp = _mm_add_ps(gbp, st2);
            _mm_storeu_ps(s2 + i, _mm_add_ps(gbp, lp));
            c0 = _mm_add_ps(c0, _mm_mul_ps(_mm_loadu_ps(wc0 + i), lp));
            c1 = _mm_add_ps(c1, _mm_mul_ps(_mm_loadu_ps(wc1 + i), lp));
            d0 = _mm_add_ps(d0, _mm_mul_ps(_mm_loadu_ps(ws0 + i), lp));
            d1 = _mm_add_ps(d1, _mm_mul_ps(_mm_loadu_ps(ws1 + i), lp));
        }
        const float f = (static_cast<float>(t) + 1.0f) * inv;
        const float a0 = sum4(c0), a1 = sum4(c1), b0 = sum4(d0), b1 = sum4(d1);
        c[t] += a0 + (a1 - a0) * f;
        s[t] += b0 + (b1 - b0) * f;
    }
    _mm256_zeroupper();
}

} // namespace crucible::modules::modal

#endif
