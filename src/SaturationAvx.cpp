// SPDX-License-Identifier: MIT
// Built with /arch:AVX on MSVC x64 (CMakeLists.txt), called only on CPUs with AVX; see SaturationAvx.h. Raw intrinsics and
// internal-linkage helpers only, so that no VEX-encoded copy of an inline function shared with other files can be picked by
// the linker for the rest of the program.
#include "SaturationAvx.h"

#if CRUCIBLE_SATURATION_AVX

#include <immintrin.h>

namespace crucible::saturation::avx
{
namespace
{
constexpr double kEps = 1.0e-3; // adaa2Update's threshold for the expansions

// Lane masks of four doubles as four 32-bit lanes.
__forceinline __m128i mask32(__m256d m)
{
    const __m128 lo = _mm_castpd_ps(_mm256_castpd256_pd128(m)), hi = _mm_castpd_ps(_mm256_extractf128_pd(m, 1));
    return _mm_castps_si128(_mm_shuffle_ps(lo, hi, _MM_SHUFFLE(2, 0, 2, 0)));
}

// FastTableCurve::F2 in four lanes: ShapeTable::locate (u = (x + range) / step; inside 0 < u < cells the cell trunc(u) and
// the offset (u - cell) step, at or below 0 the left extension with x + range, at or above cells the right one with
// x - range, NaN in cell 0), then n.F2 + u (n.F + u (0.5 n.f + u n.slope (1 / 6))).
struct TableCurve4
{
    const double* nodes;
    __m256d range, invStep, step, top, zero, half, sixth;
    __m128i lowNode, highNode;
    explicit TableCurve4(const TableView& t)
        : nodes(t.nodes), range(_mm256_set1_pd(t.range)), invStep(_mm256_set1_pd(t.invStep)), step(_mm256_set1_pd(t.step)),
          top(_mm256_set1_pd(static_cast<double>(t.cells))), zero(_mm256_setzero_pd()), half(_mm256_set1_pd(0.5)),
          sixth(_mm256_set1_pd(1.0 / 6.0)), lowNode(_mm_set1_epi32(t.cells + 1)), highNode(_mm_set1_epi32(t.cells + 2))
    {
    }
    // The offset in the cell, and the four node indices.
    __forceinline __m256d locate(__m256d x, __m128i& idx) const
    {
        const __m256d xr = _mm256_add_pd(x, range);
        const __m256d u = _mm256_mul_pd(xr, invStep);
        const __m256d low = _mm256_cmp_pd(u, zero, _CMP_LE_OQ), high = _mm256_cmp_pd(u, top, _CMP_GE_OQ);
        const __m256d inside = _mm256_and_pd(_mm256_cmp_pd(u, zero, _CMP_GT_OQ), _mm256_cmp_pd(u, top, _CMP_LT_OQ));
        const __m128i lo32 = mask32(low), hi32 = mask32(high);
        const __m128i cell = _mm_and_si128(_mm256_cvttpd_epi32(u), mask32(inside));
        const __m256d offset = _mm256_mul_pd(_mm256_sub_pd(u, _mm256_cvtepi32_pd(cell)), step);
        idx = _mm_or_si128(cell, _mm_or_si128(_mm_and_si128(lo32, lowNode), _mm_and_si128(hi32, highNode)));
        return _mm256_blendv_pd(_mm256_blendv_pd(offset, _mm256_sub_pd(x, range), high), xr, low);
    }
    // FastTableCurve::f: n.f + n.slope u
    __forceinline __m256d f(__m256d x) const
    {
        __m128i idx;
        const __m256d t = locate(x, idx);
        const __m256d n0 = _mm256_loadu_pd(nodes + 4 * static_cast<long long>(_mm_cvtsi128_si32(idx)));
        const __m256d n1 = _mm256_loadu_pd(nodes + 4 * static_cast<long long>(_mm_extract_epi32(idx, 1)));
        const __m256d n2 = _mm256_loadu_pd(nodes + 4 * static_cast<long long>(_mm_extract_epi32(idx, 2)));
        const __m256d n3 = _mm256_loadu_pd(nodes + 4 * static_cast<long long>(_mm_extract_epi32(idx, 3)));
        const __m256d t0 = _mm256_unpacklo_pd(n0, n1), t1 = _mm256_unpackhi_pd(n0, n1); // (f f F2 F2), (F F slope slope)
        const __m256d t2 = _mm256_unpacklo_pd(n2, n3), t3 = _mm256_unpackhi_pd(n2, n3);
        const __m256d fv = _mm256_permute2f128_pd(t0, t2, 0x20), slope = _mm256_permute2f128_pd(t1, t3, 0x31);
        return _mm256_add_pd(fv, _mm256_mul_pd(slope, t));
    }
    __forceinline __m256d g2(__m256d x) const
    {
        __m128i idx;
        const __m256d t = locate(x, idx);
        const __m256d n0 = _mm256_loadu_pd(nodes + 4 * static_cast<long long>(_mm_cvtsi128_si32(idx)));
        const __m256d n1 = _mm256_loadu_pd(nodes + 4 * static_cast<long long>(_mm_extract_epi32(idx, 1)));
        const __m256d n2 = _mm256_loadu_pd(nodes + 4 * static_cast<long long>(_mm_extract_epi32(idx, 2)));
        const __m256d n3 = _mm256_loadu_pd(nodes + 4 * static_cast<long long>(_mm_extract_epi32(idx, 3)));
        const __m256d t0 = _mm256_unpacklo_pd(n0, n1), t1 = _mm256_unpackhi_pd(n0, n1); // (f f F2 F2), (F F slope slope)
        const __m256d t2 = _mm256_unpacklo_pd(n2, n3), t3 = _mm256_unpackhi_pd(n2, n3);
        const __m256d f = _mm256_permute2f128_pd(t0, t2, 0x20), F2 = _mm256_permute2f128_pd(t0, t2, 0x31);
        const __m256d F = _mm256_permute2f128_pd(t1, t3, 0x20), slope = _mm256_permute2f128_pd(t1, t3, 0x31);
        __m256d g = _mm256_add_pd(_mm256_mul_pd(half, f), _mm256_mul_pd(_mm256_mul_pd(t, slope), sixth));
        g = _mm256_add_pd(F, _mm256_mul_pd(t, g));
        return _mm256_add_pd(F2, _mm256_mul_pd(t, g));
    }
};

// FoldCurve::F2 in four lanes: k = floor((x + 1) / 4), t = x - 4 k, (t^3 + 1) / 6 or 1/2 + t^2 - t^3 / 6 - t, plus 2 k.
struct FoldCurve4
{
    __m256d one = _mm256_set1_pd(1.0), quarter = _mm256_set1_pd(0.25), four = _mm256_set1_pd(4.0), six = _mm256_set1_pd(6.0),
            half = _mm256_set1_pd(0.5), two = _mm256_set1_pd(2.0);
    __forceinline __m256d g2(__m256d x) const
    {
        const __m256d k = _mm256_floor_pd(_mm256_mul_pd(_mm256_add_pd(x, one), quarter));
        const __m256d t = _mm256_sub_pd(x, _mm256_mul_pd(four, k));
        const __m256d t2 = _mm256_mul_pd(t, t), t3 = _mm256_mul_pd(t2, t);
        const __m256d rise = _mm256_div_pd(_mm256_add_pd(t3, one), six);
        const __m256d fall = _mm256_sub_pd(_mm256_sub_pd(_mm256_add_pd(half, t2), _mm256_div_pd(t3, six)), t);
        const __m256d g = _mm256_blendv_pd(fall, rise, _mm256_cmp_pd(t, one, _CMP_LE_OQ));
        return _mm256_add_pd(g, _mm256_mul_pd(two, k));
    }
};

// Lanes: (sample i channel 0, sample i channel 1, sample i + 1 channel 0, sample i + 1 channel 1).
template <class Curve>
__forceinline int run(const Curve& curve, PairState& st, const DcCoefficients& dc, const double* xs0, const double* xs1,
                      float* out0, float* out1, int start, int len)
{
    const __m256d one = _mm256_set1_pd(1.0), two = _mm256_set1_pd(2.0), eps = _mm256_set1_pd(kEps),
                  sign = _mm256_set1_pd(-0.0);
    const __m128d keep = _mm_set1_pd(dc.keep), in = _mm_set1_pd(dc.in), out = _mm_set1_pd(dc.out);
    // the group before: x at i - 2 and i - 1, F2 and the first divided difference at i - 1 (upper half)
    __m256d xGroup = _mm256_set_pd(st.xPrev[1], st.xPrev[0], st.xPrev2[1], st.xPrev2[0]);
    __m256d gGroup = _mm256_set_pd(st.F2Prev[1], st.F2Prev[0], 0.0, 0.0);
    __m256d aGroup = _mm256_set_pd(st.divPrev[1], st.divPrev[0], 0.0, 0.0);
    __m128d hp = _mm_set_pd(st.hp[1], st.hp[0]);
    int i = start;
    for (; i + 2 <= len; i += 2)
    {
        const __m128d c0 = _mm_loadu_pd(xs0 + i), c1 = _mm_loadu_pd(xs1 + i);
        const __m256d x = _mm256_insertf128_pd(_mm256_castpd128_pd256(_mm_unpacklo_pd(c0, c1)), _mm_unpackhi_pd(c0, c1), 1);
        const __m256d d01 = _mm256_sub_pd(x, _mm256_permute2f128_pd(xGroup, x, 0x21)), d02 = _mm256_sub_pd(x, xGroup);
        const __m256d ok = _mm256_and_pd(_mm256_cmp_pd(_mm256_andnot_pd(sign, d01), eps, _CMP_GT_OQ),
                                         _mm256_cmp_pd(_mm256_andnot_pd(sign, d02), eps, _CMP_GT_OQ));
        if (_mm256_movemask_pd(ok) != 15)
            break;
        const __m256d g0 = curve.g2(x);
        const __m256d inv = _mm256_div_pd(one, _mm256_mul_pd(d01, d02));
        const __m256d a =
            _mm256_mul_pd(_mm256_mul_pd(_mm256_sub_pd(g0, _mm256_permute2f128_pd(gGroup, g0, 0x21)), d02), inv);
        const __m256d y = _mm256_mul_pd(
            _mm256_mul_pd(_mm256_mul_pd(two, _mm256_sub_pd(a, _mm256_permute2f128_pd(aGroup, a, 0x21))), d01), inv);
        __m128d y0 = _mm256_castpd256_pd128(y), y1 = _mm256_extractf128_pd(y, 1);
        if (dc.on)
        {
            __m128d s = hp;
            hp = _mm_add_pd(_mm_mul_pd(s, keep), _mm_mul_pd(in, y0));
            y0 = _mm_mul_pd(out, _mm_sub_pd(y0, s));
            s = hp;
            hp = _mm_add_pd(_mm_mul_pd(s, keep), _mm_mul_pd(in, y1));
            y1 = _mm_mul_pd(out, _mm_sub_pd(y1, s));
        }
        const __m128 f = _mm256_cvtpd_ps(_mm256_insertf128_pd(_mm256_castpd128_pd256(y0), y1, 1)); // (0 0, 0 1, 1 0, 1 1)
        _mm_storel_pi(reinterpret_cast<__m64*>(out0 + i), _mm_shuffle_ps(f, f, _MM_SHUFFLE(3, 1, 2, 0)));
        _mm_storel_pi(reinterpret_cast<__m64*>(out1 + i), _mm_shuffle_ps(f, f, _MM_SHUFFLE(2, 0, 3, 1)));
        xGroup = x;
        gGroup = g0;
        aGroup = a;
    }
    const __m128d xLast = _mm256_extractf128_pd(xGroup, 1), xBefore = _mm256_castpd256_pd128(xGroup);
    const __m128d gLast = _mm256_extractf128_pd(gGroup, 1), aLast = _mm256_extractf128_pd(aGroup, 1);
    _mm_storeu_pd(st.xPrev, xLast);
    _mm_storeu_pd(st.xPrev2, xBefore);
    _mm_storeu_pd(st.F2Prev, gLast);
    _mm_storeu_pd(st.divPrev, aLast);
    _mm_storeu_pd(st.hp, hp);
    _mm256_zeroupper();
    return i;
}
} // namespace

int adaa2PairTable(const TableView& t, PairState& st, const DcCoefficients& dc, const double* xs0, const double* xs1,
                   float* out0, float* out1, int start, int len)
{
    return run(TableCurve4(t), st, dc, xs0, xs1, out0, out1, start, len);
}

int adaa2PairFold(PairState& st, const DcCoefficients& dc, const double* xs0, const double* xs1, float* out0, float* out1,
                  int start, int len)
{
    return run(FoldCurve4{}, st, dc, xs0, xs1, out0, out1, start, len);
}

void sagPairTable(const TableView& t, SagLanes& sag, double* xs0, double* xs1, int len)
{
    const TableCurve4 curve(t);
    const __m256d sign = _mm256_set1_pd(-0.0);
    const __m128d depth = _mm_loadu_pd(sag.depth), atk = _mm_loadu_pd(sag.atk), rel = _mm_loadu_pd(sag.rel);
    __m128d env = _mm_loadu_pd(sag.env);
    int i = 0;
    for (; i + 2 <= len; i += 2)
    {
        const __m128d c0 = _mm_loadu_pd(xs0 + i), c1 = _mm_loadu_pd(xs1 + i);
        const __m128d x0 = _mm_unpacklo_pd(c0, c1), x1 = _mm_unpackhi_pd(c0, c1); // samples i and i + 1, both channels
        const __m256d level = _mm256_andnot_pd(sign, curve.f(_mm256_insertf128_pd(_mm256_castpd128_pd256(x0), x1, 1)));
        const __m128d l0 = _mm256_castpd256_pd128(level), l1 = _mm256_extractf128_pd(level, 1);
        const __m128d s0 = _mm_sub_pd(x0, _mm_mul_pd(depth, env));
        env = _mm_add_pd(env, _mm_mul_pd(_mm_sub_pd(l0, env), _mm_blendv_pd(rel, atk, _mm_cmp_pd(l0, env, _CMP_GT_OQ))));
        const __m128d s1 = _mm_sub_pd(x1, _mm_mul_pd(depth, env));
        env = _mm_add_pd(env, _mm_mul_pd(_mm_sub_pd(l1, env), _mm_blendv_pd(rel, atk, _mm_cmp_pd(l1, env, _CMP_GT_OQ))));
        _mm_storeu_pd(xs0 + i, _mm_unpacklo_pd(s0, s1));
        _mm_storeu_pd(xs1 + i, _mm_unpackhi_pd(s0, s1));
    }
    if (i < len)
    {
        const __m128d x0 = _mm_set_pd(xs1[i], xs0[i]);
        const __m128d l0 =
            _mm256_castpd256_pd128(_mm256_andnot_pd(sign, curve.f(_mm256_insertf128_pd(_mm256_castpd128_pd256(x0), x0, 1))));
        const __m128d s0 = _mm_sub_pd(x0, _mm_mul_pd(depth, env));
        env = _mm_add_pd(env, _mm_mul_pd(_mm_sub_pd(l0, env), _mm_blendv_pd(rel, atk, _mm_cmp_pd(l0, env, _CMP_GT_OQ))));
        _mm_storel_pd(xs0 + i, s0);
        _mm_storeh_pd(xs1 + i, s0);
    }
    _mm_storeu_pd(sag.env, env);
    _mm256_zeroupper();
}

} // namespace crucible::saturation::avx

#endif
