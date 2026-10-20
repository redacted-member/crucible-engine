// SPDX-License-Identifier: MIT
// Four-pole OTA filter with a resonance VCA and partial gain compensation (the arrangement of the CEM3320-style filters
// of 1980s polysynths): four trapezoidal OTA-C integrators y' = g tanh(a (u - y)) / a in series, and a feedback path
// that returns the last stage through a transconductance VCA to the input of the first. Compared with the
// transistor ladder it
//
//   * feeds the input with a built-in 15 % share of the resonance compensation, so its bass loss lies between the
//     ladder's (no compensation) and the OTA cascade's (35 %); the dcGain() law is (1 + k (0.15 + 0.85 character)) / (1 +
//     k);
//   * limits the feedback itself: the VCA is a saturating amplifier, F(v) = c tanh(v / c), which gives the self-
//     oscillation a rounder, more level-stable limit cycle than the integrators alone.
//
// Resonance maps linearly to the VCA gain k = 0 .. 4.3; the loop oscillates from k = 4. `character` blends in the rest
// of the compensation (1 = unity DC gain).
// Per sample: one trapezoidal step of the four stages and the Newton solve of the cyclic 4x4 system (see
// NonlinearSolve.h).
// Design: the OTA integrator as analysed in Zavalishin, "The Art of VA Filter Design", and Stinchcombe's notes on OTA
// filters.

#include "LadderBase.h"

#include <algorithm>
#include <cmath>

namespace crucible::filters
{
namespace
{
constexpr float kMaxFeedback = 4.3f;
constexpr float kAlpha = 0.9f;        // differential pair input scale
constexpr float kBuiltInComp = 0.15f; // share of the input compensation that is always present
constexpr float kVcaLimit = 2.0f;     // output limit of the resonance VCA, in volts

struct State
{
    alignas(16) float s[4][4]{};
    alignas(16) float predictor[4][5][4]{}; // the linear predictor of the current control chunk
};

template <class V, SolverTier Tier> void kernel(State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    constexpr int stride = nl::vstride<V>;
    constexpr int iterations = Tier == SolverTier::High ? 4 : 1;
    const V one = nl::vconst<V>(1.0f);
    const V two = nl::vconst<V>(2.0f);
    const V alpha = nl::vconst<V>(kAlpha), invAlpha = nl::vconst<V>(1.0f / kAlpha);
    const V comp0 = nl::vconst<V>(kBuiltInComp);
    const V limit = nl::vconst<V>(kVcaLimit), invLimit = nl::vconst<V>(1.0f / kVcaLimit);
    V s[4];
    for (int j = 0; j < 4; ++j)
        s[j] = nl::load<V>(st.s[j]);
    V g = nl::load<V>(b.g), dg = nl::load<V>(b.dg), k = nl::load<V>(b.k), dk = nl::load<V>(b.dk);
    const V chr = nl::load<V>(b.chr);
    const V gm[4] = {nl::load<V>(b.gm[0]), nl::load<V>(b.gm[1]), nl::load<V>(b.gm[2]), nl::load<V>(b.gm[3])};

    nl::CyclicPredictor<V> predictor;
    if (b.rebuild)
    {
        const V gMid = nl::load<V>(b.gMid);
        const V gp[4] = {gMid * gm[0], gMid * gm[1], gMid * gm[2], gMid * gm[3]};
        predictor.build(gp, nl::load<V>(b.kMid));
        predictor.save(st.predictor);
    }
    else
        predictor.load(st.predictor);

    for (int i = 0; i < n; ++i)
    {
        const V x = nl::load<V>(in + i * stride);
        const V comp = one + k * (comp0 + chr * (one - comp0));
        const V xs = x * comp;
        const V gi[4] = {g * gm[0], g * gm[1], g * gm[2], g * gm[3]};
        V y[4], z[4];
        nl::Linearisation<V, 4> lin;
        predictor.predict(s, xs, y);
        for (int it = 0; it < iterations; ++it)
        {
            // Resonance VCA: F = c sat(k y3 / c), slope k sat'(.) at the working point.
            V satF, slopeF;
            nl::rationalSat<V>(k * y[3] * invLimit, satF, slopeF);
            const V fb = limit * satF;
            const V dfb = slopeF * k;
            z[1] = y[0] - y[1];
            z[2] = y[1] - y[2];
            z[3] = y[2] - y[3];
            if constexpr (Tier == SolverTier::High)
            {
                z[0] = xs - fb - y[0];
                lin.refreshRational(z, alpha, invAlpha);
            }
            else
            {
                // One Newton step: the first stage is evaluated at the linear feedback and corrected to first order by
                // the compression of the VCA, which keeps the VCA's division off the dependency chain of the stage
                // saturators (the error is second order in that compression).
                z[0] = xs - k * y[3] - y[0];
                lin.refreshRational(z, alpha, invAlpha);
                lin.T[0] = lin.T[0] + lin.D[0] * (k * y[3] - fb);
            }
            V a[4], c[4], r[4], d[4];
            for (int j = 0; j < 4; ++j)
            {
                const V gd = gi[j] * lin.D[j];
                a[j] = one + gd;
                c[j] = gd;
                r[j] = s[j] - y[j] + gi[j] * lin.T[j];
            }
            nl::solveCyclic4(a, c, gi[0] * lin.D[0] * dfb, r, d);
            V change = nl::vconst<V>(0.0f);
            for (int j = 0; j < 4; ++j)
            {
                y[j] = y[j] + d[j];
                change = nl::vmax(change, nl::vmax(d[j], nl::neg(d[j])));
            }
            if constexpr (Tier == SolverTier::High)
                if (nl::vmaxAbs(change) < 1.0e-6f)
                    break;
        }
        for (int j = 0; j < 4; ++j)
            s[j] = two * y[j] - s[j];
        nl::store(out + i * stride, y[3]);
        g = g + dg;
        k = k + dk;
    }
    for (int j = 0; j < 4; ++j)
        nl::store(st.s[j], s[j]);
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

class Ota4Pole final : public LadderBase
{
public:
    std::unique_ptr<AnalogFilter> clone() const override { return std::make_unique<Ota4Pole>(*this); }
    float feedbackForResonance(float resonance) const override { return kMaxFeedback * std::clamp(resonance, 0.0f, 1.0f); }
    float selfOscillationGain() const override { return 4.0f; }

    // Small-signal loop of four one-pole stages H = 1 / (1 + p) closed through the feedback gain k.
    std::complex<double> kernelGain(std::complex<double> p, double k, double chr) const override
    {
        const std::complex<double> h = 1.0 / (1.0 + p);
        const std::complex<double> h4 = h * h * h * h;
        return (1.0 + k * (kBuiltInComp + chr * (1.0 - kBuiltInComp))) * h4 / (1.0 + k * h4);
    }
    float dcGain(float resonance, float character) const override
    {
        const float k = feedbackForResonance(resonance);
        return (1.0f + k * (kBuiltInComp + character * (1.0f - kBuiltInComp))) / (1.0f + k);
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
    int modelId() const override { return 6; }
    bool supportsHighpass() const override { return true; }
    void resetState() override { state = State{}; }
    void resetLaneState(int lane) override
    {
        const auto l = static_cast<size_t>(lane);
        for (auto& row : state.s)
            row[l] = 0.0f;
        for (auto& row : state.predictor)
            for (auto& col : row)
                col[l] = 0.0f;
    }

private:
    State state;
};
} // namespace
} // namespace crucible::filters

namespace crucible
{
std::unique_ptr<AnalogFilter> makeOta4Pole()
{
    auto f = std::make_unique<filters::Ota4Pole>();
    f->prepare(48000.0, Quality::Standard, 1, 512);
    return f;
}
} // namespace crucible
