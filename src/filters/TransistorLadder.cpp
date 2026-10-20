// SPDX-License-Identifier: MIT
// Transistor ladder: four trapezoidal integrator stages y' = g (tanh(u) - tanh(y)), the first fed with
// tanh(x - k y4). Resonance maps to the feedback gain k = 0 .. 4.2; the loop self-oscillates at k = 4 and the DC gain
// is 1 / (1 + k) (the bass loss of the circuit). `character` scales the input by 1 + character k, which cancels it.
// Design: Huovilainen 2004 and Zavalishin's "The Art of VA Filter Design" (zero-delay feedback, trapezoidal rule).

#include "LadderBase.h"

#include <algorithm>
#include <cmath>

namespace crucible::filters
{
namespace
{
using nl::Linearisation;

constexpr float kMaxFeedback = 4.2f;

struct State
{
    alignas(16) float s[4][4]{};
    alignas(16) float predictor[4][5][4]{}; // the linear predictor of the current control chunk
};

// One lane type, one solver tier: the whole per-sample loop.
template <class V, SolverTier Tier> void kernel(State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    constexpr int stride = nl::vstride<V>;
    constexpr int iterations = Tier == SolverTier::High ? 4 : 1; // Newton steps per sample
    const V one = nl::vconst<V>(1.0f);
    const V two = nl::vconst<V>(2.0f);
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
        const V xs = x * (one + chr * k);
        const V gi[4] = {g * gm[0], g * gm[1], g * gm[2], g * gm[3]};
        V y[4], z[5];
        Linearisation<V, 5> lin;
        const auto arguments = [&]
        {
            z[0] = xs - k * y[3];
            z[1] = y[0];
            z[2] = y[1];
            z[3] = y[2];
            z[4] = y[3];
        };
        predictor.predict(s, xs, y);
        for (int it = 0; it < iterations; ++it)
        {
            arguments();
            lin.refresh(z, one, one);
            V a[4], c[4], r[4], d[4];
            for (int j = 0; j < 4; ++j)
            {
                a[j] = one + gi[j] * lin.D[j + 1];
                c[j] = gi[j] * lin.D[j];
                r[j] = s[j] - y[j] - gi[j] * (lin.T[j + 1] - lin.T[j]);
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

class TransistorLadder final : public LadderBase
{
public:
    std::unique_ptr<AnalogFilter> clone() const override { return std::make_unique<TransistorLadder>(*this); }
    float feedbackForResonance(float resonance) const override { return kMaxFeedback * std::clamp(resonance, 0.0f, 1.0f); }
    float selfOscillationGain() const override { return 4.0f; }

    // Small-signal loop of four one-pole stages H = 1 / (1 + p) closed through the feedback gain k.
    std::complex<double> kernelGain(std::complex<double> p, double k, double chr) const override
    {
        const std::complex<double> h = 1.0 / (1.0 + p);
        const std::complex<double> h4 = h * h * h * h;
        return (1.0 + k * (chr)) * h4 / (1.0 + k * h4);
    }
    float dcGain(float resonance, float character) const override
    {
        const float k = feedbackForResonance(resonance);
        return (1.0f + character * k) / (1.0f + k);
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
    int modelId() const override { return 0; }
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
std::unique_ptr<AnalogFilter> makeTransistorLadder()
{
    auto f = std::make_unique<filters::TransistorLadder>();
    f->prepare(48000.0, Quality::Standard, 1, 512);
    return f;
}
} // namespace crucible
