// SPDX-License-Identifier: MIT
// OTA cascade: four trapezoidal OTA-C integrators y' = g tanh(a (u - y)) / a (the differential pair of a transconductance
// amplifier saturates on the difference of its inputs), a linear resonance feedback u1 = x - k y4, and an optional
// one-pole high-pass ahead of the cascade with four steps. Compared with the transistor ladder it keeps more of the
// bass (the input is fed with a built-in share of the compensation), its resonance dial reaches self-oscillation
// earlier (k = 0 .. 4.4) and its differential pairs saturate softly (a = 0.7): a full-scale input stays in the
// nearly linear part, the limit cycle is a near-sine at about -5 dBFS.
// Design: the OTA integrator of the Roland-style cascade as analysed in Zavalishin, "The Art of VA Filter Design",
// and Stinchcombe's notes on OTA filters.

#include "LadderBase.h"

#include <algorithm>
#include <cmath>

namespace crucible::filters
{
namespace
{
using nl::Linearisation;

constexpr float kMaxFeedback = 4.4f;
constexpr float kAlpha = 0.7f;        // differential pair input scale
constexpr float kBuiltInComp = 0.35f; // share of the input compensation that is always present

struct State
{
    alignas(16) float s[4][4]{};
    alignas(16) float predictor[4][5][4]{}; // the linear predictor of the current control chunk
    alignas(16) float hp[4]{};              // state of the optional input high-pass
    alignas(16) float hpG[4]{};             // its integrator gain, 0 = off
};

template <class V, SolverTier Tier, bool HighPass>
void kernel(State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    constexpr int stride = nl::vstride<V>;
    constexpr int iterations = Tier == SolverTier::High ? 4 : 1; // Newton steps per sample
    const V one = nl::vconst<V>(1.0f);
    const V two = nl::vconst<V>(2.0f);
    const V alpha = nl::vconst<V>(kAlpha), invAlpha = nl::vconst<V>(1.0f / kAlpha);
    const V comp0 = nl::vconst<V>(kBuiltInComp);
    V s[4];
    for (int j = 0; j < 4; ++j)
        s[j] = nl::load<V>(st.s[j]);
    V hpState = nl::load<V>(st.hp);
    const V hpG = nl::load<V>(st.hpG);
    const V hpK = nl::vdiv(hpG, one + hpG);
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
        V x = nl::load<V>(in + i * stride);
        if constexpr (HighPass)
        {
            // One-pole high-pass (trapezoidal) ahead of the cascade.
            const V v = (x - hpState) * hpK;
            const V lp = v + hpState;
            hpState = lp + v;
            x = x - lp;
        }
        // Built-in compensation c0 k, plus the character blend of what is left.
        const V comp = one + k * (comp0 + chr * (one - comp0));
        const V xs = x * comp;
        const V gi[4] = {g * gm[0], g * gm[1], g * gm[2], g * gm[3]};
        V y[4], z[4];
        Linearisation<V, 4> lin;
        const auto arguments = [&]
        {
            z[0] = xs - k * y[3] - y[0];
            z[1] = y[0] - y[1];
            z[2] = y[1] - y[2];
            z[3] = y[2] - y[3];
        };
        predictor.predict(s, xs, y);
        for (int it = 0; it < iterations; ++it)
        {
            arguments();
            lin.refresh(z, alpha, invAlpha);
            V a[4], c[4], r[4], d[4];
            for (int j = 0; j < 4; ++j)
            {
                const V gd = gi[j] * lin.D[j];
                a[j] = one + gd;
                c[j] = gd;
                r[j] = s[j] - y[j] + gi[j] * lin.T[j];
            }
            nl::solveCyclic4(a, c, gi[0] * lin.D[0] * k, r, d);
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
    nl::store(st.hp, hpState);
}

template <class V, bool HighPass>
void dispatchTier(SolverTier tier, State& st, const float* in, float* out, int n, const LaneBlock& b)
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

template <class V> void dispatch(SolverTier tier, State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    if (st.hpG[0] != 0.0f)
        dispatchTier<V, true>(tier, st, in, out, n, b);
    else
        dispatchTier<V, false>(tier, st, in, out, n, b);
}

class OtaCascade final : public LadderBase
{
public:
    std::unique_ptr<AnalogFilter> clone() const override { return std::make_unique<OtaCascade>(*this); }
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

    // Step 0 = off; steps 1..3 put a one-pole high-pass of 60 / 180 / 540 Hz ahead of the cascade.
    void setHighpassStep(int step) override
    {
        hpStep = std::clamp(step, 0, 3);
        applyHighpass();
    }

    float frontHighpassHz() const override
    {
        static constexpr float corner[4] = {0.0f, 60.0f, 180.0f, 540.0f};
        return corner[hpStep];
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
    int modelId() const override { return 1; }
    void resetState() override
    {
        state = State{};
        applyHighpass();
    }
    void resetLaneState(int lane) override
    {
        const auto l = static_cast<size_t>(lane);
        for (auto& row : state.s)
            row[l] = 0.0f;
        for (auto& row : state.predictor)
            for (auto& col : row)
                col[l] = 0.0f;
        state.hp[l] = 0.0f;
    }
    void onPrepared() override { applyHighpass(); }

private:
    void applyHighpass()
    {
        static constexpr float corner[4] = {0.0f, 60.0f, 180.0f, 540.0f};
        const float f = corner[hpStep];
        const float gHp = f > 0.0f ? std::tan(3.14159265f * f / static_cast<float>(osRate)) : 0.0f;
        for (float& v : state.hpG)
            v = gHp;
    }

    State state;
    int hpStep = 0;
};
} // namespace
} // namespace crucible::filters

namespace crucible
{
std::unique_ptr<AnalogFilter> makeOtaCascade()
{
    auto f = std::make_unique<filters::OtaCascade>();
    f->prepare(48000.0, Quality::Standard, 1, 512);
    return f;
}
} // namespace crucible
