// SPDX-License-Identifier: MIT
// Sallen-Key two-pole filter with a diode clipper in the feedback path (the topology of the Korg 35 / MS-20 filter
// family): a unity-gain buffer follows two resistors and two capacitors, and the capacitor that closes the loop is
// driven from a gain stage (gain Ka) whose output passes an asymmetric diode clipper. With R = C = 1 (the cutoff sets
// the integrator rate) and F the clipped feedback voltage the two states obey
//
//   low-pass  (B = buffer input):  B' = w + F - B,           w' = x + B - 2 w - 2 F
//   high-pass (v1, v2 = cap volts): v2' = B,   v1' = 2x - 2 v1 - v2 - F,   B = x - v1 - v2
//   F = f(Ka B)
//
// whose linear transfer functions are 1 / (s^2 + (3 - Ka) s + 1) and s^2 / (s^2 + (3 - Ka) s + 1): unity gain in the
// pass band whatever the resonance (no bass loss), Q = 1 / (3 - Ka) and self-oscillation at Ka = 3. Above that gain the
// clipper limits the amplitude of the oscillation, and because the clipper is asymmetric (diode pair with unequal
// thresholds, modelled as a shifted tanh) the limit cycle carries even as well as odd harmonics.
//
// Each sample is one trapezoidal step (z = P (q + G b x) + G P e F, an exact 2x2 inverse) plus one scalar nonlinear
// equation F = f(v), v = Ka B0 + alpha F, solved from the linear solution by Newton's method (see NonlinearSolve.h).
// The two caps carry their own mismatch factor, so the two integrator gains G1 and G2 differ slightly with tolerance.
// Design: the Sallen-Key circuit and the K35 feedback arrangement as analysed in Stinchcombe, "A study of the Korg
// MS10 & MS20 filters", and Pirkle, "Designing Software Synthesizer Plug-Ins in C++" (K35 chapter); the discretisation
// follows Zavalishin, "The Art of VA Filter Design".

#include "LadderBase.h"

#include <algorithm>
#include <cmath>

namespace crucible::filters
{
namespace
{
constexpr float kMaxFeedback = 3.15f; // Ka at resonance 1; the loop oscillates from Ka = 3
constexpr float kClipGain = 0.9f;     // steepness of the diode clipper (1 / volts of its knee)
constexpr float kClipHardness = 1.0f; // character 1 multiplies the steepness by 1 + this
constexpr float kClipBias = 0.12f;    // diode threshold asymmetry in volts

constexpr int kCached = 17; // coefficients kept between calls (see kernel)

struct State
{
    alignas(16) float q[2][4]{};
    alignas(16) float cache[kCached][4]{}; // the coefficient set of the current block of four samples
    int phase = 0;                         // samples already used up in that block
};

// One lane type, one solver tier, low-pass or high-pass.
template <class V, SolverTier Tier, bool HighPass>
void kernel(State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    constexpr int stride = nl::vstride<V>;
    constexpr int iterations = Tier == SolverTier::High ? 4 : 1;
    const V one = nl::vconst<V>(1.0f);
    const V two = nl::vconst<V>(2.0f);
    V q0 = nl::load<V>(st.q[0]), q1 = nl::load<V>(st.q[1]);
    V g = nl::load<V>(b.g), dg = nl::load<V>(b.dg), k = nl::load<V>(b.k), dk = nl::load<V>(b.dk);
    const V gm0 = nl::load<V>(b.gm[0]), gm1 = nl::load<V>(b.gm[1]);
    // Clipper f(v) = S (sat(u) - sat(u0)) with u = a (v + beta), u0 = a beta and sat the rational saturator of
    // NonlinearSolve.h (sat = n / d, n = u (27 + u^2), d = 27 + 9 u^2, sat' = 9 (9 - u^2)^2 / d^2); S = 1 / (a sat'(u0))
    // makes the slope one at zero. The difference of the two saturator values has the exact factor u - u0 = a v:
    //
    //   sat(u) - sat(u0) = a v Phi(u) / (d d0),  Phi = (729 + 27 u0^2) - 216 u0 u + (27 + 9 u0^2) u^2,
    //
    // so f(v) = K0 v Phi(u) / d with K0 = d0 / (9 (9 - u0^2)^2) and f'(v) = K1 (9 - u^2)^2 / d^2, K1 = (d0 / (9 - u0^2))^2.
    // No cancellation: the clipper is exactly linear for tiny signals (a difference of two saturator values would have a
    // dead zone near 1e-7), exactly zero at zero, and the loop starts from the noise floor. The Newton step of
    // F = f(v), v = Ka B0 + alpha F, over the common denominator d^2, leaves one division:
    //
    //   step = (F - f(v)) / (1 - alpha f'(v)) = d (F d - K0 v Phi) / (d^2 - alpha K1 (9 - u^2)^2).
    const V a = nl::vconst<V>(kClipGain) * (one + nl::vconst<V>(kClipHardness) * nl::load<V>(b.chr));
    const V c27 = nl::vconst<V>(27.0f), c9 = nl::vconst<V>(9.0f);
    const V u0 = a * nl::vconst<V>(kClipBias);
    const V u0sq = u0 * u0;
    const V d0 = c27 + c9 * u0sq, r0 = c9 - u0sq;
    const V k0 = nl::vdiv(d0, c9 * r0 * r0);
    const V k1 = nl::vdiv(d0 * d0, r0 * r0);
    const V phi0 = nl::vconst<V>(729.0f) + c27 * u0sq, phi1 = nl::vconst<V>(216.0f) * u0, phi2 = d0;
    const V phiBase = phi0;

    // The coefficients that depend on the cutoff and the feedback gain alone ramp slowly: they are refreshed every
    // fourth sample, at the middle of those four. With P the exact 2x2 inverse of the trapezoidal system divided by
    // its determinant, the free solution is z = P (q + G b x) and the feedback adds zf F; everything that multiplies
    // the states or the input is folded into one coefficient each, so a sample is a few multiply-adds.
    V Ka, alpha, c9alpha, cB, cLin, cA;
    V bq0, bq1, bx, aq0, aq1, ax, cq0, cq1, cx, zf0, zf1;
    const auto visit = [&](auto&& fn)
    {
        int j = 0;
        for (V* v : {&Ka, &alpha, &c9alpha, &cB, &cLin, &cA, &bq0, &bq1, &bx, &aq0, &aq1, &ax, &cq0, &cq1, &cx, &zf0, &zf1})
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
            Ka = k + half * dk;
            const V G1 = gc * gm0, G2 = gc * gm1;
            const V G12 = G1 * G2;
            const V D = HighPass ? one + two * G1 + G2 + G12 : one + G1 + two * G2 + G12;
            const V invD = nl::vdiv(one, D);
            cB = G1 * invD; // B = B0 + cB F
            alpha = Ka * cB;
            c9alpha = alpha * k1;
            cLin = nl::vdiv(Ka, one - alpha); // linear solution F = cLin B0
            cA = a * cLin;
            V p00, p01, p10, p11;
            if constexpr (HighPass)
            {
                const V a3 = one + G2, a4 = one + two * G1;
                p00 = a3 * invD;
                p01 = nl::neg(cB);
                p10 = nl::neg(G2 * invD);
                p11 = a4 * invD;
                zf0 = nl::neg(a3 * cB);
                zf1 = G2 * cB;
                // r = (q0 + 2 G1 x, q1 + G2 x)
                aq0 = p00;
                aq1 = p01;
                ax = p00 * two * G1 + p01 * G2;
                cq0 = p10;
                cq1 = p11;
                cx = p10 * two * G1 + p11 * G2;
                // B0 = x - za - zb
                bq0 = nl::neg(aq0 + cq0);
                bq1 = nl::neg(aq1 + cq1);
                bx = one - ax - cx;
            }
            else
            {
                const V a1 = one + two * G2, a2 = one + G1;
                p00 = a1 * invD;
                p01 = cB;
                p10 = G2 * invD;
                p11 = a2 * invD;
                zf0 = cB;
                zf1 = nl::neg(G2 * (two + G1) * invD);
                // r = (q0, q1 + G2 x)
                aq0 = p00;
                aq1 = p01;
                ax = p01 * G2;
                cq0 = p10;
                cq1 = p11;
                cx = p11 * G2;
                bq0 = aq0;
                bq1 = aq1;
                bx = ax;
            }
        }
        const V B0 = bq1 * q1 + (bq0 * q0 + bx * x);
        const V za = aq1 * q1 + (aq0 * q0 + ax * x), zb = cq1 * q1 + (cq0 * q0 + cx * x);
        const V w0 = two * za - q0, w1 = two * zb - q1; // the state update without the feedback term
        // The linear solution is the starting point (and the whole answer for small signals).
        V F = cLin * B0;
        V v = F;
        V u = cA * B0 + u0;
        for (int it = 0; it < iterations; ++it)
        {
            const V u2 = u * u;
            const V d = c27 + c9 * u2;
            const V r = c9 - u2;
            const V phi = (phi2 * u2 + (phiBase - phi1 * u));
            const V t = d * F - k0 * v * phi;
            const V Nd = d * d - c9alpha * (r * r);
            const V step = (t * d) * nl::vdiv(one, Nd);
            F = F - step;
            v = Ka * B0 + alpha * F;
            u = a * v + u0;
            if constexpr (Tier == SolverTier::High)
                if (nl::vmaxAbs(step) < 1.0e-6f)
                    break;
        }
        nl::store(out + i * stride, B0 + cB * F);
        q0 = w0 + (two * zf0) * F;
        q1 = w1 + (two * zf1) * F;
        g = g + dg;
        k = k + dk;
    }
    nl::store(st.q[0], q0);
    nl::store(st.q[1], q1);
    visit([&](int j, V& v) { nl::store(st.cache[j], v); });
    st.phase = (st.phase + n) & 3;
}

template <class V, bool HighPass>
void dispatch(SolverTier tier, State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    switch (tier)
    {
    // Draft takes the same single Newton step as Standard, at the host rate (see NonlinearSolve.h).
    case SolverTier::Draft:
    case SolverTier::Standard:
        kernel<V, SolverTier::Standard, HighPass>(st, in, out, n, b);
        break;
    case SolverTier::High:
        kernel<V, SolverTier::High, HighPass>(st, in, out, n, b);
        break;
    }
}

template <bool HighPass> class SallenKey final : public LadderBase
{
public:
    std::unique_ptr<AnalogFilter> clone() const override { return std::make_unique<SallenKey>(*this); }
    float feedbackForResonance(float resonance) const override
    {
        return 1.0f + (kMaxFeedback - 1.0f) * std::clamp(resonance, 0.0f, 1.0f);
    }
    float selfOscillationGain() const override { return 3.0f; }
    // Unity pass band for any resonance: 1 at zero frequency for the low-pass, 0 for the high-pass.
    float dcGain(float, float) const override { return HighPass ? 0.0f : 1.0f; }
    // 1 / (p^2 + (3 - Ka) p + 1) for the low-pass and p^2 over the same denominator for the high-pass.
    std::complex<double> kernelGain(std::complex<double> p, double k, double) const override
    {
        const std::complex<double> denominator = p * p + (3.0 - k) * p + 1.0;
        return (HighPass ? p * p : std::complex<double>(1.0)) / denominator;
    }

protected:
    void run1(const float* in, float* out, int n, const LaneBlock& b) override
    {
        dispatch<float, HighPass>(tier, state, in, out, n, b);
    }
    void run4(const float* in, float* out, int n, const LaneBlock& b) override
    {
        dispatch<nl::Float4, HighPass>(tier, state, in, out, n, b);
    }
    int modelId() const override { return HighPass ? 4 : 3; }
    bool supportsHighpass() const override { return !HighPass; }
    bool distortedOscillation() const override { return true; }
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
std::unique_ptr<AnalogFilter> makeSallenKeyLowpass()
{
    auto f = std::make_unique<filters::SallenKey<false>>();
    f->prepare(48000.0, Quality::Standard, 1, 512);
    return f;
}

std::unique_ptr<AnalogFilter> makeSallenKeyHighpass()
{
    auto f = std::make_unique<filters::SallenKey<true>>();
    f->prepare(48000.0, Quality::Standard, 1, 512);
    return f;
}
} // namespace crucible
