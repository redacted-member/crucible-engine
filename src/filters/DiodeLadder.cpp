// SPDX-License-Identifier: MIT
// Coupled diode ladder (the 18 dB/octave ladder of the TB-303 family). Four capacitor nodes are coupled through diode
// pairs, so each node equation involves its neighbours (a tridiagonal system, solved in closed form), the first
// capacitor is twice the others, and the diode current is tanh of the voltage difference:
//   v1' = g/2 (f(x - k v4 - v1) - f(v1 - v2)),  v2' = g (f(v1 - v2) - f(v2 - v3)),
//   v3' = g (f(v2 - v3) - f(v3 - v4)),          v4' = g f(v3 - v4).
// The linear loop self-oscillates at k = 16.6 and the poles are spread (0.11 .. 3.3 of the unit frequency), which
// gives the lower resonance peak, the gradual onset of the roll-off and about 18 dB/octave in the range the dial
// covers. The dial frequency is the self-oscillation frequency. Resonance 1 sits 5 % above the threshold, so the
// loop oscillates only weakly (about -30 dBFS) and takes a while to build up.
// Design: Stinchcombe, "Analysis of the Moog transistor ladder and derivative filters", and Zavalishin's analysis of the
// diode ladder; the nonlinear form uses the same trapezoidal solve as the other ladders.

#include "LadderBase.h"

#include <algorithm>
#include <cmath>

namespace crucible::filters
{
namespace
{
using nl::Linearisation;

constexpr float kCriticalGain = 16.6f;       // linear loop gain at which the coupled ladder self-oscillates
constexpr float kMaxFeedback = 17.4f;        // resonance 1: 5 % above the threshold
constexpr float kOscillationRatio = 0.9583f; // self-oscillation frequency over the unit stage rate
constexpr float kAlpha = 1.2f;               // diode characteristic scale
constexpr float kTapMid = 0.35f;             // output tap: share of the third node (the rest is the fourth)

struct State
{
    alignas(16) float s[4][4]{};
    alignas(16) float predictor[4][5][4]{}; // the linear predictor of the current control chunk
};

template <class V, SolverTier Tier> void kernel(State& st, const float* in, float* out, int n, const LaneBlock& b)
{
    constexpr int stride = nl::vstride<V>;
    constexpr int iterations = Tier == SolverTier::High ? 4 : 1; // Newton steps per sample
    const V one = nl::vconst<V>(1.0f);
    const V two = nl::vconst<V>(2.0f);
    const V half = nl::vconst<V>(0.5f);
    const V alpha = nl::vconst<V>(kAlpha), invAlpha = nl::vconst<V>(1.0f / kAlpha);
    const V tapMid = nl::vconst<V>(kTapMid), tapEnd = nl::vconst<V>(1.0f - kTapMid);
    V s[4];
    for (int j = 0; j < 4; ++j)
        s[j] = nl::load<V>(st.s[j]);
    V g = nl::load<V>(b.g), dg = nl::load<V>(b.dg), k = nl::load<V>(b.k), dk = nl::load<V>(b.dk);
    const V chr = nl::load<V>(b.chr);
    const V gm[4] = {nl::load<V>(b.gm[0]), nl::load<V>(b.gm[1]), nl::load<V>(b.gm[2]), nl::load<V>(b.gm[3])};

    nl::TridiagonalPredictor<V> predictor;
    if (b.rebuild)
    {
        const V gMid = nl::load<V>(b.gMid);
        const V gp[4] = {gMid * gm[0] * half, gMid * gm[1], gMid * gm[2], gMid * gm[3]};
        predictor.build(gp, nl::load<V>(b.kMid));
        predictor.save(st.predictor);
    }
    else
        predictor.load(st.predictor);

    for (int i = 0; i < n; ++i)
    {
        const V x = nl::load<V>(in + i * stride);
        const V xs = x * (one + chr * k);
        const V gi[4] = {g * gm[0] * half, g * gm[1], g * gm[2], g * gm[3]};
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
            V lo[4], di[4], up[4], r[4], d[4];
            const V* D = lin.D;
            const V* T = lin.T;
            di[0] = one + gi[0] * (D[0] + D[1]);
            up[0] = nl::neg(gi[0] * D[1]);
            r[0] = s[0] - y[0] + gi[0] * (T[0] - T[1]);
            lo[1] = nl::neg(gi[1] * D[1]);
            di[1] = one + gi[1] * (D[1] + D[2]);
            up[1] = nl::neg(gi[1] * D[2]);
            r[1] = s[1] - y[1] + gi[1] * (T[1] - T[2]);
            lo[2] = nl::neg(gi[2] * D[2]);
            di[2] = one + gi[2] * (D[2] + D[3]);
            up[2] = nl::neg(gi[2] * D[3]);
            r[2] = s[2] - y[2] + gi[2] * (T[2] - T[3]);
            lo[3] = nl::neg(gi[3] * D[3]);
            di[3] = one + gi[3] * D[3];
            r[3] = s[3] - y[3] + gi[3] * T[3];
            nl::solveTridiagonalCorner4(lo, di, up, gi[0] * D[0] * k, r, d);
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
        nl::store(out + i * stride, tapMid * y[2] + tapEnd * y[3]);
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

class DiodeLadder final : public LadderBase
{
public:
    std::unique_ptr<AnalogFilter> clone() const override { return std::make_unique<DiodeLadder>(*this); }
    float feedbackForResonance(float resonance) const override { return kMaxFeedback * std::clamp(resonance, 0.0f, 1.0f); }
    float selfOscillationGain() const override { return kCriticalGain; }
    float dcGain(float resonance, float character) const override
    {
        const float k = feedbackForResonance(resonance);
        return (1.0f + character * k) / (1.0f + k);
    }
    // The four coupled nodes as a linear system in the normalised frequency (node 0 has twice the capacitance), solved by
    // Gaussian elimination: (p + 1) v0 - v1 / 2 + k v3 / 2 = x / 2, -v0 + (p + 2) v1 - v2 = 0, -v1 + (p + 2) v2 - v3 = 0,
    // -v2 + (p + 1) v3 = 0; the input is scaled by 1 + character k and the output tapped from nodes 2 and 3.
    std::complex<double> kernelGain(std::complex<double> p, double k, double chr) const override
    {
        using Cx = std::complex<double>;
        Cx a[4][5] = {{p + 1.0, -0.5, 0.0, 0.5 * k, 0.5 * (1.0 + chr * k)},
                      {-1.0, p + 2.0, -1.0, 0.0, 0.0},
                      {0.0, -1.0, p + 2.0, -1.0, 0.0},
                      {0.0, 0.0, -1.0, p + 1.0, 0.0}};
        for (int col = 0; col < 4; ++col)
        {
            int pivot = col;
            for (int r = col + 1; r < 4; ++r)
                if (std::abs(a[r][col]) > std::abs(a[pivot][col]))
                    pivot = r;
            for (int c = 0; c < 5; ++c)
                std::swap(a[col][c], a[pivot][c]);
            for (int r = col + 1; r < 4; ++r)
            {
                const Cx f = a[r][col] / a[col][col];
                for (int c = col; c < 5; ++c)
                    a[r][c] -= f * a[col][c];
            }
        }
        Cx v[4];
        for (int r = 3; r >= 0; --r)
        {
            Cx sum = a[r][4];
            for (int c = r + 1; c < 4; ++c)
                sum -= a[r][c] * v[c];
            v[r] = sum / a[r][r];
        }
        return static_cast<double>(kTapMid) * v[2] + (1.0 - static_cast<double>(kTapMid)) * v[3];
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
    int modelId() const override { return 2; }
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
    float gScale() const override { return 1.0f / kOscillationRatio; }

private:
    State state;
};
} // namespace
} // namespace crucible::filters

namespace crucible
{
std::unique_ptr<AnalogFilter> makeDiodeLadder()
{
    auto f = std::make_unique<filters::DiodeLadder>();
    f->prepare(48000.0, Quality::Standard, 1, 512);
    return f;
}
} // namespace crucible
