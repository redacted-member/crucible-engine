// SPDX-License-Identifier: MIT
// Two-pole state-variable filter built from two OTA-C integrators, the arrangement of the Oberheim SEM filter: the
// differential pair at the input of each integrator saturates, so
//
//   hp = x - d bp - lp,   bp' = T(hp),   lp' = T(bp),   T(u) = tanh(a u) / a
//
// with the damping d = 1 / Q. The loop has positive damping at all settings (Q tops out at 20), so unlike the ladders
// it never oscillates: the saturating integrators only reduce the loop gain further as the level grows, and a
// resonant peak compresses with level. The pass band is unity (no bass loss). `character` morphs the output from
// low-pass (0) through the notch (0.5, low-pass plus high-pass) to high-pass (1); `resonance` sets Q geometrically from
// 0.5 to 20.
//
// Each sample is one trapezoidal step, solved from the linear (small-signal) solution by Newton's method on the two
// unknowns (bp, lp) with a closed-form 2x2 inverse (see NonlinearSolve.h).
// Design: Zavalishin, "The Art of VA Filter Design" (nonlinear state-variable filter), and Stinchcombe's notes on the
// SEM filter.

#include "LadderBase.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crucible::filters
{
namespace
{
constexpr float kQMin = 0.5f;
constexpr float kQMax = 20.0f;
constexpr float kAlpha = 0.5f; // differential pair input scale

constexpr int kCached = 12; // coefficients kept between calls (see kernel)

struct State
{
    alignas(16) float q[2][4]{};
    alignas(16) float cache[kCached][4]{}; // the coefficient set of the current block of four samples
    int phase = 0;                         // samples already used up in that block
};

// Integrator input T(u) = sat(a u) / a with sat the rational saturator of NonlinearSolve.h: for s = a u,
// sat = n / d with n = s (27 + s^2), d = 27 + 9 s^2, and its slope is 9 (9 - s^2)^2 / d^2. Because s d - n = 8 s^3, the
// residual of the linear solution is a pure cubic, which keeps the Newton step short (see kernel).
template <class V, SolverTier Tier> void kernel(State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    constexpr int stride = nl::vstride<V>;
    const V one = nl::vconst<V>(1.0f);
    const V two = nl::vconst<V>(2.0f);
    const V alpha = nl::vconst<V>(kAlpha);
    const V c27 = nl::vconst<V>(27.0f), c9 = nl::vconst<V>(9.0f), c8 = nl::vconst<V>(8.0f);
    V qb = nl::load<V>(st.q[0]), ql = nl::load<V>(st.q[1]);
    V g = nl::load<V>(b.g), dg = nl::load<V>(b.dg), k = nl::load<V>(b.k), dk = nl::load<V>(b.dk);
    const V gm0 = nl::load<V>(b.gm[0]), gm1 = nl::load<V>(b.gm[1]);
    // Output morph: low-pass weight falls over the upper half of the dial, high-pass weight rises over the lower half.
    const V chr = nl::load<V>(b.chr);
    const V wLp = nl::vmin(two * (one - chr), one), wHp = nl::vmin(two * chr, one);

    // Coefficients that depend on the cutoff and the damping alone, refreshed every fourth sample (middle of the four):
    // the linear solution of one trapezoidal step is bp = eb (.), lp = el (.), hp = eh (.) in (qb, ql, x).
    V gb, gl, d, bq, bl, bx, lq, ll, lx, hq, hl, hx;
    const auto visit = [&](auto&& fn)
    {
        int j = 0;
        for (V* v : {&gb, &gl, &d, &bq, &bl, &bx, &lq, &ll, &lx, &hq, &hl, &hx})
            fn(j++, *v);
    };
    visit([&](int j, V& v) { v = nl::load<V>(st.cache[j]); });
    for (int i = 0; i < n; ++i)
    {
        const V x = nl::load<V>(in + i * stride);
        if (((st.phase + i) & 3) == 0)
        {
            const V half = nl::vconst<V>(1.5f);
            const V gc = g + half * dg;
            gb = gc * gm0;
            gl = gc * gm1;
            d = nl::vdiv(one, k + half * dk);
            const V gg = gb * gl;
            const V invDen = nl::vdiv(one, one + gb * d + gg);
            // bp = (qb + gb (x - ql)) / den, lp = ql + gl bp, hp = x - d bp - lp
            bq = invDen;
            bl = nl::neg(gb * invDen);
            bx = gb * invDen;
            lq = gl * invDen;
            ll = one - gg * invDen;
            lx = gg * invDen;
            hq = nl::neg((d + gl) * invDen);
            hl = (d * gb + gg) * invDen - one;
            hx = one - (d * gb + gg) * invDen;
        }
        const V bpL = bq * qb + (bl * ql + bx * x);
        const V lpL = lq * qb + (ll * ql + lx * x);
        const V hpL = hq * qb + (hl * ql + hx * x);
        V bp = bpL, lp = lpL, hp = hpL;
        if constexpr (Tier != SolverTier::High)
        {
            // One Newton step from the linear solution. With s = a u, d = 27 + 9 s^2, r = 9 - s^2 the cubic residuals
            // are Rb = (8 gb / a) sh^3 / dh and Rl = (8 gl / a) sb^3 / db, and clearing the denominators dh^2 db^2
            // gives one division: step = -N / Det.
            const V sh = alpha * hpL, sb = alpha * bpL;
            const V sh2 = sh * sh, sb2 = sb * sb;
            const V dh = c27 + c9 * sh2, db = c27 + c9 * sb2;
            const V rh = c9 - sh2, rb = c9 - sb2;
            const V rh2 = rh * rh, rb2 = rb * rb;
            const V dh2 = dh * dh, db2 = db * db;
            const V sh3 = sh * sh2, sb3 = sb * sb2;
            const V gd = gb * d;
            const V Det = dh2 * db2 + c9 * gd * rh2 * db2 + c9 * c9 * gb * gl * rh2 * rb2;
            const V Nb = gb * (sh3 * dh * db2 - c9 * gl * rh2 * sb3 * db);
            const V Nl = gl * (sb3 * db * (dh2 + c9 * gd * rh2) + c9 * gb * rb2 * sh3 * dh);
            const V f = nl::vdiv(c8 * nl::vconst<V>(1.0f / kAlpha), Det);
            const V dBp = nl::neg(Nb * f), dLp = nl::neg(Nl * f);
            bp = bpL + dBp;
            lp = lpL + dLp;
            hp = hpL - d * dBp - dLp;
        }
        else
        {
            // Newton iterations on the two unknowns, residuals from the current estimate.
            const V invAlpha = nl::vconst<V>(1.0f / kAlpha);
            for (int it = 0; it < (Tier == SolverTier::High ? 4 : 1); ++it)
            {
                V satH, slopeH, satB, slopeB;
                nl::rationalSat<V>(alpha * hp, satH, slopeH);
                nl::rationalSat<V>(alpha * bp, satB, slopeB);
                const V rbRes = bp - qb - gb * satH * invAlpha;
                const V rlRes = lp - ql - gl * satB * invAlpha;
                const V A = one + gb * slopeH * d, B = gb * slopeH, C = nl::neg(gl * slopeB);
                const V invDet = nl::vdiv(one, A - B * C);
                const V dBp = nl::neg((rbRes - B * rlRes) * invDet);
                const V dLp = nl::neg((A * rlRes - C * rbRes) * invDet);
                bp = bp + dBp;
                lp = lp + dLp;
                hp = x - d * bp - lp;
                if (nl::vmaxAbs(nl::vmax(dBp, nl::neg(dBp)) + nl::vmax(dLp, nl::neg(dLp))) < 1.0e-6f)
                    break;
            }
        }
        qb = two * bp - qb;
        ql = two * lp - ql;
        nl::store(out + i * stride, wLp * lp + wHp * hp);
        g = g + dg;
        k = k + dk;
    }
    nl::store(st.q[0], qb);
    nl::store(st.q[1], ql);
    visit([&](int j, V& v) { nl::store(st.cache[j], v); });
    st.phase = (st.phase + n) & 3;
}

template <class V> void dispatch(SolverTier tier, State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    switch (tier)
    {
    // Draft takes the same single Newton step as Standard, at the host rate (see NonlinearSolve.h).
    case SolverTier::Draft:
    case SolverTier::Standard:
        kernel<V, SolverTier::Standard>(st, in, out, n, b);
        break;
    case SolverTier::High:
        kernel<V, SolverTier::High>(st, in, out, n, b);
        break;
    }
}

class SvfOta final : public LadderBase
{
public:
    std::unique_ptr<AnalogFilter> clone() const override { return std::make_unique<SvfOta>(*this); }
    // The loop quantity of this model is the quality factor.
    float feedbackForResonance(float resonance) const override
    {
        return kQMin * std::pow(kQMax / kQMin, std::clamp(resonance, 0.0f, 1.0f));
    }
    float selfOscillationGain() const override { return std::numeric_limits<float>::infinity(); }
    // Low-pass weight of the morph; the notch keeps the low-pass gain.
    float dcGain(float, float character) const override
    {
        return std::min(2.0f * (1.0f - std::clamp(character, 0.0f, 1.0f)), 1.0f);
    }
    // Low-pass / notch / high-pass morph of 1 / (p^2 + p / Q + 1) and p^2 / (p^2 + p / Q + 1); k is the quality factor.
    std::complex<double> kernelGain(std::complex<double> p, double k, double chr) const override
    {
        const double wLp = std::min(2.0 * (1.0 - chr), 1.0), wHp = std::min(2.0 * chr, 1.0);
        return (wLp + wHp * p * p) / (p * p + p / k + 1.0);
    }

protected:
    void run1(const float* in, float* out, int n, const LaneBlock& b) override
    {
        dispatch<float>(tier, state, in, out, n, b);
    }
    void run4(const float* in, float* out, int n, const LaneBlock& b) override
    {
        dispatch<nl::Float4>(tier, state, in, out, n, b);
    }
    int modelId() const override { return 5; }
    bool selfOscillates() const override { return false; }
    bool supportsHighpass() const override { return true; }
    void resetState() override { state = State{}; }
    void resetLaneState(int lane) override
    {
        const auto l = static_cast<size_t>(lane);
        for (auto& row : state.q)
            row[l] = 0.0f;
        for (auto& row : state.cache)
            row[l] = 0.0f;
    }
    // A control chunk starts on a fresh block of four samples, so the coefficient set always follows the new controls
    // (and a lane that joins a running quad finds a valid set at its first sample).
    void onRestart() override { state.phase = 0; }

private:
    State state;
};
} // namespace
} // namespace crucible::filters

namespace crucible
{
std::unique_ptr<AnalogFilter> makeSvfOta()
{
    auto f = std::make_unique<filters::SvfOta>();
    f->prepare(48000.0, Quality::Standard, 1, 512);
    return f;
}
} // namespace crucible
