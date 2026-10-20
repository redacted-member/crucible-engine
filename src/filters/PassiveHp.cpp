// SPDX-License-Identifier: MIT
// Passive high-pass: one or two buffered RC sections, y = x - lowpass(x) per section. There is no feedback and no
// nonlinearity, so the slopes are exact: 6 dB/oct below the corner with one section and 12 dB/oct with two. The dial
// frequency is the corner of each section (-3.01 dB with one section, -6.02 dB with two). `resonance` selects the order:
// at 0 one section, from 0.6 upwards both, and between the two outputs crossfade (a blend of the two slopes).
// The drive stage of the shared front end still applies, so drive 1 makes it a saturating high-pass; at drive 0 the
// model is linear. The same section, fixed at one pole, is the `highpass` step ahead of the other models.
// Design: the bilinear (trapezoidal) one-pole section of Zavalishin, "The Art of VA Filter Design"; two cascaded
// sections are the textbook second-order passive high-pass.

#include "LadderBase.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crucible::filters
{
namespace
{
struct State
{
    alignas(16) float s[2][4]{};
};

template <class V> void kernel(State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    constexpr int stride = nl::vstride<V>;
    const V one = nl::vconst<V>(1.0f);
    V s0 = nl::load<V>(st.s[0]), s1 = nl::load<V>(st.s[1]);
    V g = nl::load<V>(b.g), dg = nl::load<V>(b.dg), k = nl::load<V>(b.k), dk = nl::load<V>(b.dk);
    const V gm0 = nl::load<V>(b.gm[0]), gm1 = nl::load<V>(b.gm[1]);
    for (int i = 0; i < n; ++i)
    {
        const V x = nl::load<V>(in + i * stride);
        const V g0 = g * gm0, g1 = g * gm1;
        // Trapezoidal one-pole low-pass: v = (u - s) g / (1 + g), lp = v + s, s' = lp + v.
        const V v0 = (x - s0) * nl::vdiv(g0, one + g0);
        const V lp0 = v0 + s0;
        s0 = lp0 + v0;
        const V y1 = x - lp0;
        const V v1 = (y1 - s1) * nl::vdiv(g1, one + g1);
        const V lp1 = v1 + s1;
        s1 = lp1 + v1;
        const V y2 = y1 - lp1;
        // k is the crossfade between the sections, 0 .. 1.
        nl::store(out + i * stride, y1 + nl::vmin(k, one) * (y2 - y1));
        g = g + dg;
        k = k + dk;
    }
    nl::store(st.s[0], s0);
    nl::store(st.s[1], s1);
}

class PassiveHp final : public LadderBase
{
public:
    std::unique_ptr<AnalogFilter> clone() const override { return std::make_unique<PassiveHp>(*this); }
    // The loop quantity is the share of the second section in the output.
    float feedbackForResonance(float resonance) const override { return std::clamp((resonance - 0.0f) / 0.6f, 0.0f, 1.0f); }
    float selfOscillationGain() const override { return std::numeric_limits<float>::infinity(); }
    float dcGain(float, float) const override { return 0.0f; }
    // One section p / (1 + p), a second one in series, crossfaded by k.
    std::complex<double> kernelGain(std::complex<double> p, double k, double) const override
    {
        const std::complex<double> one = p / (1.0 + p);
        return one + std::min(k, 1.0) * (one * one - one);
    }

protected:
    void run1(const float* in, float* out, int n, const LaneBlock& b) override { kernel<float>(state, in, out, n, b); }
    void run4(const float* in, float* out, int n, const LaneBlock& b) override { kernel<nl::Float4>(state, in, out, n, b); }
    int modelId() const override { return 7; }
    bool selfOscillates() const override { return false; }
    void resetState() override { state = State{}; }
    void resetLaneState(int lane) override
    {
        for (auto& row : state.s)
            row[static_cast<size_t>(lane)] = 0.0f;
    }

private:
    State state;
};
} // namespace
} // namespace crucible::filters

namespace crucible
{
std::unique_ptr<AnalogFilter> makePassiveHp()
{
    auto f = std::make_unique<filters::PassiveHp>();
    f->prepare(48000.0, Quality::Standard, 1, 512);
    return f;
}
} // namespace crucible
