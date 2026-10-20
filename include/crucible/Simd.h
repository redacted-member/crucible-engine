// SPDX-License-Identifier: MIT
#pragma once

// Four-lane vector types with NEON, SSE (optionally AVX2 gathers) and scalar implementations. Use for hot loops
// over contiguous data (keep arrays 16-byte friendly and lengths a multiple of four); everything else can rely on
// compiler vectorisation.
//
// Defining CRUCIBLE_FORCE_SCALAR (CMake option CRUCIBLE_FORCE_SCALAR) selects the scalar implementation of the types
// and switches the engine's vectorised kernels off, so the plain scalar code runs everywhere. CRUCIBLE_SIMD_NATIVE is
// 1 when real vector instructions are in use.

#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>

#if defined(CRUCIBLE_FORCE_SCALAR)
#define CRUCIBLE_SIMD_NATIVE 0
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define CRUCIBLE_SIMD_NEON 1
#define CRUCIBLE_SIMD_NATIVE 1
#elif defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86)
#include <immintrin.h>
#define CRUCIBLE_SIMD_SSE 1
#define CRUCIBLE_SIMD_NATIVE 1
#else
#define CRUCIBLE_SIMD_NATIVE 0
#endif

#ifndef CRUCIBLE_SIMD_NEON
#define CRUCIBLE_SIMD_NEON 0
#endif
#ifndef CRUCIBLE_SIMD_SSE
#define CRUCIBLE_SIMD_SSE 0
#endif

// 1 where four-lane SSE code that multiplies and adds in the order of a scalar loop gives exactly that loop's bits: x86
// without FMA, where the scalar code cannot be contracted either, and with SSE scalar math (not x87, whose extended
// intermediates differ). Bit-identical vector rewrites of scalar loops use it; everywhere else (NEON, where clang may fuse
// the scalar code; FMA builds, including MSVC /arch:AVX2, which never defines __FMA__) the scalar loop itself runs.
#if CRUCIBLE_SIMD_SSE && !defined(__FMA__) && !defined(__AVX2__) &&                                                         \
    (defined(__SSE_MATH__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2))
#define CRUCIBLE_SIMD_SSE_EXACT 1
#else
#define CRUCIBLE_SIMD_SSE_EXACT 0
#endif

// Forces inlining of tiny hot helpers (GCC / clang attribute, MSVC __forceinline).
#ifndef CRUCIBLE_ALWAYS_INLINE
#if defined(_MSC_VER) && !defined(__clang__)
#define CRUCIBLE_ALWAYS_INLINE __forceinline
#else
#define CRUCIBLE_ALWAYS_INLINE [[gnu::always_inline]] inline
#endif
#endif

namespace crucible::simd
{

// Four 32-bit integers (table indices).
struct Int4
{
#if CRUCIBLE_SIMD_NEON
    int32x4_t v;
    static Int4 min(Int4 a, Int4 b) { return {vminq_s32(a.v, b.v)}; }
    template <int Lane> int lane() const { return vgetq_lane_s32(v, Lane); }
    static Int4 splat(int x) { return {vdupq_n_s32(x)}; }
    static Int4 load(const std::int32_t* p) { return {vld1q_s32(p)}; }
#elif CRUCIBLE_SIMD_SSE
    __m128i v;
    static Int4 splat(int x) { return {_mm_set1_epi32(x)}; }
    static Int4 load(const std::int32_t* p) { return {_mm_loadu_si128(reinterpret_cast<const __m128i*>(p))}; }
    static Int4 min(Int4 a, Int4 b)
    {
#if defined(__SSE4_1__)
        return {_mm_min_epi32(a.v, b.v)};
#else
        const __m128i gt = _mm_cmpgt_epi32(a.v, b.v);
        return {_mm_or_si128(_mm_and_si128(gt, b.v), _mm_andnot_si128(gt, a.v))};
#endif
    }
    template <int Lane> int lane() const
    {
        return _mm_cvtsi128_si32(_mm_shuffle_epi32(v, _MM_SHUFFLE(Lane, Lane, Lane, Lane)));
    }
#else
    std::int32_t v[4];
    static Int4 splat(int x) { return {{x, x, x, x}}; }
    static Int4 load(const std::int32_t* p) { return {{p[0], p[1], p[2], p[3]}}; }
    static Int4 min(Int4 a, Int4 b)
    {
        Int4 r;
        for (int i = 0; i < 4; ++i)
            r.v[i] = a.v[i] < b.v[i] ? a.v[i] : b.v[i];
        return r;
    }
    template <int Lane> int lane() const { return v[Lane]; }
#endif
};

// Four floats. Comparison results are Float4 lane masks (all bits set / clear) for select() and anyTrue().
struct Float4
{
#if CRUCIBLE_SIMD_NEON
    float32x4_t v;
    static Float4 load(const float* p) { return {vld1q_f32(p)}; }
    static Float4 splat(float x) { return {vdupq_n_f32(x)}; }
    void store(float* p) const { vst1q_f32(p, v); }
    friend Float4 operator+(Float4 a, Float4 b) { return {vaddq_f32(a.v, b.v)}; }
    friend Float4 operator-(Float4 a, Float4 b) { return {vsubq_f32(a.v, b.v)}; }
    friend Float4 operator*(Float4 a, Float4 b) { return {vmulq_f32(a.v, b.v)}; }
    // a + b * c
    static Float4 mulAdd(Float4 a, Float4 b, Float4 c) { return {vmlaq_f32(a.v, b.v, c.v)}; }
    static Float4 mulSub(Float4 a, Float4 b, Float4 c) { return {vmlsq_f32(a.v, b.v, c.v)}; }
    friend Float4 operator/(Float4 a, Float4 b) { return {vdivq_f32(a.v, b.v)}; }
    static Float4 min(Float4 a, Float4 b) { return {vminq_f32(a.v, b.v)}; }
    static Float4 max(Float4 a, Float4 b) { return {vmaxq_f32(a.v, b.v)}; }
    static Float4 abs(Float4 a) { return {vabsq_f32(a.v)}; }
    // Lanes N .. N + 3 of the eight lanes (a0 a1 a2 a3 b0 b1 b2 b3), N = 1..3.
    template <int N> static Float4 ext(Float4 a, Float4 b) { return {vextq_f32(a.v, b.v, N)}; }
    static Float4 floor(Float4 a) { return {vrndmq_f32(a.v)}; }
    static Float4 geq(Float4 a, Float4 b) { return {vreinterpretq_f32_u32(vcgeq_f32(a.v, b.v))}; }
    static Float4 gt(Float4 a, Float4 b) { return {vreinterpretq_f32_u32(vcgtq_f32(a.v, b.v))}; }
    // mask ? a : b
    static Float4 select(Float4 mask, Float4 a, Float4 b) { return {vbslq_f32(vreinterpretq_u32_f32(mask.v), a.v, b.v)}; }
    static bool anyTrue(Float4 mask) { return vmaxvq_u32(vreinterpretq_u32_f32(mask.v)) != 0; }
    // (l0 + l1) + (l2 + l3)
    float sum() const { return vaddvq_f32(v); }
    Int4 truncate() const { return {vcvtq_s32_f32(v)}; }
    static Float4 fromInt(Int4 i) { return {vcvtq_f32_s32(i.v)}; }
    // Table look-up: lane l reads p[l][idx[l] + offset].
    static Float4 gather(const float* const* p, Int4 idx, int offset)
    {
        float32x4_t r = vld1q_lane_f32(p[0] + idx.lane<0>() + offset, vdupq_n_f32(0.0f), 0);
        r = vld1q_lane_f32(p[1] + idx.lane<1>() + offset, r, 1);
        r = vld1q_lane_f32(p[2] + idx.lane<2>() + offset, r, 2);
        return {vld1q_lane_f32(p[3] + idx.lane<3>() + offset, r, 3)};
    }
#elif CRUCIBLE_SIMD_SSE
    __m128 v;
    static Float4 load(const float* p) { return {_mm_loadu_ps(p)}; }
    static Float4 splat(float x) { return {_mm_set1_ps(x)}; }
    void store(float* p) const { _mm_storeu_ps(p, v); }
    friend Float4 operator+(Float4 a, Float4 b) { return {_mm_add_ps(a.v, b.v)}; }
    friend Float4 operator-(Float4 a, Float4 b) { return {_mm_sub_ps(a.v, b.v)}; }
    friend Float4 operator*(Float4 a, Float4 b) { return {_mm_mul_ps(a.v, b.v)}; }
    static Float4 mulAdd(Float4 a, Float4 b, Float4 c) { return {_mm_add_ps(a.v, _mm_mul_ps(b.v, c.v))}; }
    static Float4 mulSub(Float4 a, Float4 b, Float4 c) { return {_mm_sub_ps(a.v, _mm_mul_ps(b.v, c.v))}; }
    friend Float4 operator/(Float4 a, Float4 b) { return {_mm_div_ps(a.v, b.v)}; }
    static Float4 min(Float4 a, Float4 b) { return {_mm_min_ps(a.v, b.v)}; }
    static Float4 max(Float4 a, Float4 b) { return {_mm_max_ps(a.v, b.v)}; }
    static Float4 abs(Float4 a) { return {_mm_andnot_ps(_mm_set1_ps(-0.0f), a.v)}; }
    // Lanes N .. N + 3 of the eight lanes (a0 a1 a2 a3 b0 b1 b2 b3), N = 1..3.
    template <int N> static Float4 ext(Float4 a, Float4 b)
    {
        static_assert(N >= 1 && N <= 3);
        if constexpr (N == 1)
        {
            const __m128 u = _mm_shuffle_ps(a.v, b.v, _MM_SHUFFLE(0, 0, 3, 3));
            return {_mm_shuffle_ps(a.v, u, _MM_SHUFFLE(2, 0, 2, 1))};
        }
        else if constexpr (N == 2)
            return {_mm_shuffle_ps(a.v, b.v, _MM_SHUFFLE(1, 0, 3, 2))};
        else
        {
            const __m128 t = _mm_shuffle_ps(a.v, b.v, _MM_SHUFFLE(0, 0, 3, 3));
            return {_mm_shuffle_ps(t, b.v, _MM_SHUFFLE(2, 1, 2, 0))};
        }
    }
    static Float4 floor(Float4 a)
    {
#if defined(__SSE4_1__)
        return {_mm_floor_ps(a.v)};
#else
        // Truncate, then step down where truncation rounded up (negative non-integers).
        const __m128 t = _mm_cvtepi32_ps(_mm_cvttps_epi32(a.v));
        return {_mm_sub_ps(t, _mm_and_ps(_mm_cmplt_ps(a.v, t), _mm_set1_ps(1.0f)))};
#endif
    }
    static Float4 geq(Float4 a, Float4 b) { return {_mm_cmpge_ps(a.v, b.v)}; }
    static Float4 gt(Float4 a, Float4 b) { return {_mm_cmpgt_ps(a.v, b.v)}; }
    static Float4 select(Float4 mask, Float4 a, Float4 b)
    {
        return {_mm_or_ps(_mm_and_ps(mask.v, a.v), _mm_andnot_ps(mask.v, b.v))};
    }
    static bool anyTrue(Float4 mask) { return _mm_movemask_ps(mask.v) != 0; }
    float sum() const
    {
        const __m128 pairs = _mm_add_ps(v, _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 3, 0, 1)));
        return _mm_cvtss_f32(_mm_add_ss(pairs, _mm_movehl_ps(pairs, pairs)));
    }
    Int4 truncate() const { return {_mm_cvttps_epi32(v)}; }
    static Float4 fromInt(Int4 i) { return {_mm_cvtepi32_ps(i.v)}; }
    static Float4 gather(const float* const* p, Int4 idx, int offset)
    {
#if defined(__AVX2__)
        // One gather from the first pointer; the other lanes are addressed relative to it.
        const __m256i rel = _mm256_set_epi64x(static_cast<long long>(p[3] - p[0]), static_cast<long long>(p[2] - p[0]),
                                              static_cast<long long>(p[1] - p[0]), 0);
        const __m256i at = _mm256_add_epi64(_mm256_cvtepi32_epi64(idx.v), rel);
        return {_mm256_i64gather_ps(p[0] + offset, at, 4)};
#else
        return {_mm_set_ps(p[3][idx.lane<3>() + offset], p[2][idx.lane<2>() + offset], p[1][idx.lane<1>() + offset],
                           p[0][idx.lane<0>() + offset])};
#endif
    }
#else
    float v[4];
    static Float4 load(const float* p) { return {{p[0], p[1], p[2], p[3]}}; }
    static Float4 splat(float x) { return {{x, x, x, x}}; }
    void store(float* p) const
    {
        for (int i = 0; i < 4; ++i)
            p[i] = v[i];
    }
    friend Float4 operator+(Float4 a, Float4 b)
    {
        return {{a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2], a.v[3] + b.v[3]}};
    }
    friend Float4 operator-(Float4 a, Float4 b)
    {
        return {{a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2], a.v[3] - b.v[3]}};
    }
    friend Float4 operator*(Float4 a, Float4 b)
    {
        return {{a.v[0] * b.v[0], a.v[1] * b.v[1], a.v[2] * b.v[2], a.v[3] * b.v[3]}};
    }
    static Float4 mulAdd(Float4 a, Float4 b, Float4 c) { return a + b * c; }
    static Float4 mulSub(Float4 a, Float4 b, Float4 c) { return a - b * c; }
    static Float4 min(Float4 a, Float4 b)
    {
        Float4 r;
        for (int i = 0; i < 4; ++i)
            r.v[i] = a.v[i] < b.v[i] ? a.v[i] : b.v[i];
        return r;
    }
    static Float4 max(Float4 a, Float4 b)
    {
        Float4 r;
        for (int i = 0; i < 4; ++i)
            r.v[i] = a.v[i] > b.v[i] ? a.v[i] : b.v[i];
        return r;
    }
    friend Float4 operator/(Float4 a, Float4 b)
    {
        return {{a.v[0] / b.v[0], a.v[1] / b.v[1], a.v[2] / b.v[2], a.v[3] / b.v[3]}};
    }
    static Float4 abs(Float4 a) { return {{std::abs(a.v[0]), std::abs(a.v[1]), std::abs(a.v[2]), std::abs(a.v[3])}}; }
    // Lanes N .. N + 3 of the eight lanes (a0 a1 a2 a3 b0 b1 b2 b3), N = 1..3.
    template <int N> static Float4 ext(Float4 a, Float4 b)
    {
        float both[8];
        for (int i = 0; i < 4; ++i)
        {
            both[i] = a.v[i];
            both[4 + i] = b.v[i];
        }
        return {{both[N], both[N + 1], both[N + 2], both[N + 3]}};
    }
    static Float4 floor(Float4 a)
    {
        Float4 r;
        for (int i = 0; i < 4; ++i)
            r.v[i] = std::floor(a.v[i]);
        return r;
    }
    static float maskBits(bool on) { return std::bit_cast<float>(on ? 0xffffffffU : 0U); }
    static Float4 geq(Float4 a, Float4 b)
    {
        Float4 r;
        for (int i = 0; i < 4; ++i)
            r.v[i] = maskBits(a.v[i] >= b.v[i]);
        return r;
    }
    static Float4 gt(Float4 a, Float4 b)
    {
        Float4 r;
        for (int i = 0; i < 4; ++i)
            r.v[i] = maskBits(a.v[i] > b.v[i]);
        return r;
    }
    static Float4 select(Float4 mask, Float4 a, Float4 b)
    {
        Float4 r;
        for (int i = 0; i < 4; ++i)
            r.v[i] = std::bit_cast<std::uint32_t>(mask.v[i]) != 0U ? a.v[i] : b.v[i];
        return r;
    }
    static bool anyTrue(Float4 mask)
    {
        for (int i = 0; i < 4; ++i)
            if (std::bit_cast<std::uint32_t>(mask.v[i]) != 0U)
                return true;
        return false;
    }
    float sum() const { return (v[0] + v[1]) + (v[2] + v[3]); }
    Int4 truncate() const
    {
        return {{static_cast<int>(v[0]), static_cast<int>(v[1]), static_cast<int>(v[2]), static_cast<int>(v[3])}};
    }
    static Float4 fromInt(Int4 i)
    {
        return {{static_cast<float>(i.v[0]), static_cast<float>(i.v[1]), static_cast<float>(i.v[2]),
                 static_cast<float>(i.v[3])}};
    }
    static Float4 gather(const float* const* p, Int4 idx, int offset)
    {
        return {{p[0][idx.v[0] + offset], p[1][idx.v[1] + offset], p[2][idx.v[2] + offset], p[3][idx.v[3] + offset]}};
    }
#endif
};

// Run-time switch for the engine's vectorised kernels (on by default when CRUCIBLE_SIMD_NATIVE). The tests switch it
// off to compare the scalar reference code with the vector kernels in one binary. Not for use while rendering.
inline std::atomic<bool>& kernelFlag()
{
    static std::atomic<bool> flag{CRUCIBLE_SIMD_NATIVE != 0};
    return flag;
}
inline bool kernelsEnabled()
{
    return CRUCIBLE_SIMD_NATIVE != 0 && kernelFlag().load(std::memory_order_relaxed);
}
inline void setKernelsEnabled(bool on)
{
    kernelFlag().store(on && CRUCIBLE_SIMD_NATIVE != 0, std::memory_order_relaxed);
}

} // namespace crucible::simd
