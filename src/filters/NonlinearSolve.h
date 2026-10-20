// SPDX-License-Identifier: MIT
#pragma once

// Building blocks of the nonlinear zero-delay-feedback filters (docs/plans/ENGINE-COMMUNITY-PLAN.md, section 2.6).
//
// Every filter integrates its capacitor equations with the trapezoidal rule, which turns one sample into a small
// set of coupled algebraic equations F(y) = 0 with tanh terms. They are solved by Newton's method in residual form:
// with the tanh values T and slopes D evaluated at the current estimate y, the update delta solves
// J(D) delta = -F(T, y) in closed form and y moves by delta. The estimate starts from the linear (small-signal)
// ladder, which is exact for small signals. The solver tiers differ in how often the step is taken:
//
//   Standard  one Newton step; Draft runs the same step at the host rate
//   High      Newton iterations until the update is below 1e-6, at most four
//
// (A cheaper step without the Newton correction, linearised at the previous sample, was tried for Draft: it keeps the
// loop only below fc / fs of about 1/8, above that the resonant loop runs away.)
//
// The kernels are written once over a lane type V (float for one voice, simd::Float4 for four voices in lock step).
//
// Design sources: trapezoidal integration of the Moog ladder with a tanh per stage (Huovilainen 2004, "Non-linear
// digital implementation of the Moog ladder filter"; Zavalishin, "The Art of VA Filter Design"), the OTA integrator
// y' = w tanh(a (x - y)) of the Roland-style cascade, and the coupled diode ladder analysed in Stinchcombe, "Analysis
// of the Moog transistor ladder and derivative filters" and Zavalishin's chapter on the diode ladder.

#include <crucible/Simd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

#define CRUCIBLE_FILTER_INLINE CRUCIBLE_ALWAYS_INLINE

namespace crucible::filters::nl
{

using simd::Float4;

// ---------------------------------------------------------------------------------------------------------------
// Lane operations: the same expression compiles for one float or four.

template <class V> inline V vconst(float x)
{
    if constexpr (std::is_same_v<V, float>)
        return x;
    else
        return Float4::splat(x);
}
template <class V> inline constexpr int vstride = std::is_same_v<V, float> ? 1 : 4;

template <class V> inline V load(const float* p)
{
    if constexpr (std::is_same_v<V, float>)
        return *p;
    else
        return Float4::load(p);
}
inline void store(float* p, float v)
{
    *p = v;
}
inline void store(float* p, Float4 v)
{
    v.store(p);
}

template <class V> inline V neg(V x)
{
    return vconst<V>(0.0f) - x;
}

inline float vmin(float a, float b)
{
    return a < b ? a : b;
}
inline float vmax(float a, float b)
{
    return a > b ? a : b;
}
inline Float4 vmin(Float4 a, Float4 b)
{
    return Float4::min(a, b);
}
inline Float4 vmax(Float4 a, Float4 b)
{
    return Float4::max(a, b);
}

inline float vdiv(float a, float b)
{
    return a / b;
}
inline Float4 vdiv(Float4 a, Float4 b)
{
#if CRUCIBLE_SIMD_NEON && defined(__aarch64__)
    return {vdivq_f32(a.v, b.v)};
#elif CRUCIBLE_SIMD_NEON
    float32x4_t r = vrecpeq_f32(b.v);
    r = vmulq_f32(vrecpsq_f32(b.v, r), r);
    r = vmulq_f32(vrecpsq_f32(b.v, r), r);
    return {vmulq_f32(a.v, r)};
#elif CRUCIBLE_SIMD_SSE
    return {_mm_div_ps(a.v, b.v)};
#else
    Float4 r;
    for (int i = 0; i < 4; ++i)
        r.v[i] = a.v[i] / b.v[i];
    return r;
#endif
}

// 1 / sqrt(x) for x >= 1 (relative error below 1e-6): the hardware estimate refined by Newton-Raphson steps.
inline float vrsqrt(float x)
{
    return 1.0f / std::sqrt(x);
}
inline Float4 vrsqrt(Float4 x)
{
#if CRUCIBLE_SIMD_NEON
    float32x4_t r = vrsqrteq_f32(x.v);
    r = vmulq_f32(vrsqrtsq_f32(vmulq_f32(x.v, r), r), r);
    r = vmulq_f32(vrsqrtsq_f32(vmulq_f32(x.v, r), r), r);
    return {r};
#elif CRUCIBLE_SIMD_SSE
    __m128 r = _mm_rsqrt_ps(x.v);
    const __m128 half = _mm_set1_ps(0.5f), three = _mm_set1_ps(3.0f);
    r = _mm_mul_ps(_mm_mul_ps(half, r), _mm_sub_ps(three, _mm_mul_ps(_mm_mul_ps(x.v, r), r)));
    r = _mm_mul_ps(_mm_mul_ps(half, r), _mm_sub_ps(three, _mm_mul_ps(_mm_mul_ps(x.v, r), r)));
    return {r};
#else
    Float4 r;
    for (int i = 0; i < 4; ++i)
        r.v[i] = 1.0f / std::sqrt(x.v[i]);
    return r;
#endif
}

// Largest absolute lane value (convergence test).
inline float vmaxAbs(float a)
{
    return std::abs(a);
}
inline float vmaxAbs(Float4 a)
{
    float t[4];
    a.store(t);
    return std::max(std::max(std::abs(t[0]), std::abs(t[1])), std::max(std::abs(t[2]), std::abs(t[3])));
}

// tanh as a [7/6] Pade approximant, valid to |x| = 4 where it equals tanh(4) within 2e-5; the argument is clamped
// there. Maximum error 1.5e-5, odd, monotonic, no branches. The clamp only matters for arguments that are already
// saturated by far.
template <class V> CRUCIBLE_FILTER_INLINE V tanhApprox(V x)
{
    x = vmin(vmax(x, vconst<V>(-4.0f)), vconst<V>(4.0f));
    const V x2 = x * x;
    const V num =
        vconst<V>(1.0f) + x2 * (vconst<V>(0.128205128f) + x2 * (vconst<V>(0.0027972028f) + x2 * vconst<V>(7.4000074e-6f)));
    const V den =
        vconst<V>(1.0f) + x2 * (vconst<V>(0.461538462f) + x2 * (vconst<V>(0.0233100233f) + x2 * vconst<V>(2.0720021e-4f)));
    return vdiv(x * num, den);
}

// ---------------------------------------------------------------------------------------------------------------
// Rational saturator g(u) = (27 u + u^3) / (27 + 9 u^2), the [3/2] Pade approximant of tanh with its slope
// g'(u) = 9 (9 - u^2)^2 / (27 + 9 u^2)^2. Slope 1 at zero, flat at u = 3 (g = 1), then a residual slope that tends to
// 1/9, like a diode clipper with a series resistance. Both values come from one division and a handful of
// multiplications, with no clamp: the tail keeps a loop with a gain below 9 bounded at any level, and the function is
// analytic, so it does not alias like a hard knee. The filters whose saturator is not meant to match tanh to the last
// digit (the Sallen-Key's diode clipper, the OTA SVF and the four-pole OTA) use it; the P7 ladders keep tanhApprox.
template <class V> CRUCIBLE_FILTER_INLINE void rationalSat(V u, V& value, V& slope)
{
    const V u2 = u * u;
    const V w = vdiv(vconst<V>(1.0f), vconst<V>(27.0f) + vconst<V>(9.0f) * u2);
    value = u * (vconst<V>(27.0f) + u2) * w;
    const V r = (vconst<V>(9.0f) - u2) * w;
    slope = vconst<V>(9.0f) * r * r;
}

// ---------------------------------------------------------------------------------------------------------------
// Saturating terms f(z) = tanh(alpha z) / alpha: value T and slope D at the point where the Newton step is taken.

template <class V, int N> struct Linearisation
{
    V T[N], D[N]; // f(z) and f'(z)

    // Exact evaluation at the arguments z (N tanh evaluations).
    CRUCIBLE_FILTER_INLINE void refresh(const V* z, V alpha, V invAlpha)
    {
        const V one = vconst<V>(1.0f);
        for (int j = 0; j < N; ++j)
        {
            const V tau = tanhApprox(alpha * z[j]);
            T[j] = tau * invAlpha;
            D[j] = one - tau * tau;
        }
    }

    // The same with the rational saturator: f(z) = sat(alpha z) / alpha.
    CRUCIBLE_FILTER_INLINE void refreshRational(const V* z, V alpha, V invAlpha)
    {
        for (int j = 0; j < N; ++j)
        {
            V value;
            rationalSat<V>(alpha * z[j], value, D[j]);
            T[j] = value * invAlpha;
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------
// Linear solves.

// a0 y0 + cf y3 = r0;  a_j y_j - c_j y_(j-1) = r_j for j = 1..3. Closed form: y3 is a ratio of polynomials in the
// coefficients (one division), the other outputs follow as y_j = p_j - q_j y3 with the reciprocals of a0..a2 computed
// beside it, so the dependency chain holds a single division after the coefficients.
template <class V> CRUCIBLE_FILTER_INLINE void solveCyclic4(const V* a, const V* c, V cf, const V* r, V* y)
{
    const V one = vconst<V>(1.0f);
    const V a01 = a[0] * a[1];
    const V a012 = a01 * a[2];
    const V denominator = a012 * a[3] + (c[1] * c[2]) * (c[3] * cf);
    const V inner = c[2] * (r[1] * a[0] + c[1] * r[0]) + r[2] * a01;
    const V numerator = r[3] * a012 + c[3] * inner;
    const V y3 = vdiv(numerator, denominator);
    const V rec0 = vdiv(one, a[0]), rec1 = vdiv(one, a[1]), rec2 = vdiv(one, a[2]);
    const V c1 = c[1] * rec1, c2 = c[2] * rec2;
    const V p0 = r[0] * rec0, q0 = cf * rec0;
    const V p1 = r[1] * rec1 + c1 * p0, q1 = c1 * q0;
    const V p2 = r[2] * rec2 + c2 * p1, q2 = c2 * q1;
    y[3] = y3;
    y[2] = p2 - q2 * y3;
    y[1] = p1 - q1 * y3;
    y[0] = p0 - q0 * y3;
}

// Tridiagonal 4x4 (lower l[1..3], diagonal b[0..3], upper u[0..2]) plus one corner entry cf in row 0, column 3.
// Cyclic reduction: v0 and v2 are eliminated at the same time, which leaves a 2x2 system in v1 and v3; v0 and v2
// follow by substitution. Three reciprocals, two of them side by side.
template <class V>
CRUCIBLE_FILTER_INLINE void solveTridiagonalCorner4(const V* l, const V* b, const V* u, V cf, const V* r, V* y)
{
    const V one = vconst<V>(1.0f);
    const V rec0 = vdiv(one, b[0]), rec2 = vdiv(one, b[2]);
    const V l1r0 = l[1] * rec0, u1r2 = u[1] * rec2, l3r2 = l[3] * rec2;
    const V A = b[1] - l1r0 * u[0] - u1r2 * l[2];
    const V B = neg(l1r0 * cf) - u1r2 * u[2];
    const V C = neg(l3r2 * l[2]);
    const V D = b[3] - l3r2 * u[2];
    const V e = r[1] - l1r0 * r[0] - u1r2 * r[2];
    const V f = r[3] - l3r2 * r[2];
    const V invDet = vdiv(one, A * D - B * C);
    y[1] = (e * D - B * f) * invDet;
    y[3] = (A * f - C * e) * invDet;
    y[0] = (r[0] - u[0] * y[1] - cf * y[3]) * rec0;
    y[2] = (r[2] - l[2] * y[1] - u[2] * y[3]) * rec2;
}

// ---------------------------------------------------------------------------------------------------------------
// Linear predictors. With every tanh replaced by its small-signal line (slope 1, no offset) the ladders are linear
// and the state s and the input xs determine the next outputs: y_j = sum_i m[j][i] s_i + m[j][4] xs. The matrix is
// built once per control chunk from the integrator gains and the feedback gain; a sample then costs a dot product
// instead of a solve, and the Newton step starts from a point that is exact for small signals.

// Four stages in a loop: (1 + g_j) y_j - g_j y_(j-1) = s_j, stage 0 closed through k y_3 and fed by xs.
template <class V> struct CyclicPredictor
{
    V m[4][5];

    CRUCIBLE_FILTER_INLINE void build(const V* g, V k)
    {
        const V one = vconst<V>(1.0f), zero = vconst<V>(0.0f);
        V a[4], c[4];
        for (int j = 0; j < 4; ++j)
        {
            a[j] = one + g[j];
            c[j] = g[j];
        }
        const V cf = g[0] * k;
        for (int col = 0; col < 5; ++col)
        {
            V r[4] = {zero, zero, zero, zero}, y[4];
            if (col < 4)
                r[col] = one;
            else
                r[0] = g[0];
            solveCyclic4(a, c, cf, r, y);
            for (int j = 0; j < 4; ++j)
                m[j][col] = y[j];
        }
    }
    CRUCIBLE_FILTER_INLINE void predict(const V* s, V xs, V* y) const
    {
        for (int j = 0; j < 4; ++j)
            y[j] = (m[j][0] * s[0] + m[j][1] * s[1]) + (m[j][2] * s[2] + m[j][3] * s[3]) + m[j][4] * xs;
    }
    CRUCIBLE_FILTER_INLINE void load(const float (*p)[5][4])
    {
        for (int j = 0; j < 4; ++j)
            for (int i = 0; i < 5; ++i)
                m[j][i] = nl::load<V>(p[j][i]);
    }
    CRUCIBLE_FILTER_INLINE void save(float (*p)[5][4]) const
    {
        for (int j = 0; j < 4; ++j)
            for (int i = 0; i < 5; ++i)
                store(p[j][i], m[j][i]);
    }
};

// The coupled diode ladder: tridiagonal, first stage with its own gain g0, closed through k v_3.
template <class V> struct TridiagonalPredictor
{
    V m[4][5];

    CRUCIBLE_FILTER_INLINE void build(const V* g, V k)
    {
        const V one = vconst<V>(1.0f), zero = vconst<V>(0.0f);
        V lo[4], di[4], up[4];
        lo[0] = zero;
        di[0] = one + g[0] + g[0];
        up[0] = zero - g[0];
        lo[1] = zero - g[1];
        di[1] = one + g[1] + g[1];
        up[1] = zero - g[1];
        lo[2] = zero - g[2];
        di[2] = one + g[2] + g[2];
        up[2] = zero - g[2];
        lo[3] = zero - g[3];
        di[3] = one + g[3];
        up[3] = zero;
        const V cf = g[0] * k;
        for (int col = 0; col < 5; ++col)
        {
            V r[4] = {zero, zero, zero, zero}, y[4];
            if (col < 4)
                r[col] = one;
            else
                r[0] = g[0];
            solveTridiagonalCorner4(lo, di, up, cf, r, y);
            for (int j = 0; j < 4; ++j)
                m[j][col] = y[j];
        }
    }
    CRUCIBLE_FILTER_INLINE void predict(const V* s, V xs, V* y) const
    {
        for (int j = 0; j < 4; ++j)
            y[j] = (m[j][0] * s[0] + m[j][1] * s[1]) + (m[j][2] * s[2] + m[j][3] * s[3]) + m[j][4] * xs;
    }
    CRUCIBLE_FILTER_INLINE void load(const float (*p)[5][4])
    {
        for (int j = 0; j < 4; ++j)
            for (int i = 0; i < 5; ++i)
                m[j][i] = nl::load<V>(p[j][i]);
    }
    CRUCIBLE_FILTER_INLINE void save(float (*p)[5][4]) const
    {
        for (int j = 0; j < 4; ++j)
            for (int i = 0; i < 5; ++i)
                store(p[j][i], m[j][i]);
    }
};

// ---------------------------------------------------------------------------------------------------------------
// Small deterministic generator for the part-to-part tolerances and the optional start-up noise.

struct SplitMix
{
    std::uint64_t state = 1;
    SplitMix() = default;
    explicit SplitMix(std::uint64_t seed) : state(seed) {}
    std::uint64_t next()
    {
        state += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    // Uniform in [-1, 1).
    float bipolar() { return static_cast<float>(static_cast<double>(next() >> 11) * (2.0 / 9007199254740992.0) - 1.0); }
};

} // namespace crucible::filters::nl
