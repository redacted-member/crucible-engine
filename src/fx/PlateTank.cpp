// SPDX-License-Identifier: MIT
// The plate: a figure-eight all-pass tank after Dattorro, "Effect Design Part 1: Reverberator and Other Filters", JAES
// 1997. The input arrives already diffused (a chain of all-passes in AlgorithmicReverb); two branches follow, each a
// modulated all-pass, a delay, a loss filter, a second all-pass and a delay, and each feeds the other. The lengths are the
// published ones (at 29761 Hz) scaled to the sample rate and the size control. The loss filter is the shelving absorption of
// the FDN, designed for the loop that the branch is part of, so the reverberation time is the requested one in every band.
// The output taps are my own.

#include <crucible/fx/Fdn.h>

#include <algorithm>
#include <cmath>

namespace crucible::fx
{
namespace
{
constexpr float kTiny = 1.0e-18f;
constexpr float kExcursionSeconds = 0.0010f;
constexpr float kRt60Log = 6.907755279f;
constexpr double kPaperRate = 29761.0;
constexpr int kPaperLength[8] = {672, 4453, 1800, 3720, 908, 4217, 2656, 3163};

std::uint32_t nextRandom(std::uint32_t& state)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

float uniform01(std::uint32_t& state)
{
    return static_cast<float>(nextRandom(state) >> 8) * (1.0f / 16777216.0f);
}
} // namespace

void PlateTank::prepare(double sampleRate, std::uint32_t seed)
{
    sr = sampleRate;
    seedValue = seed;
    const double ratio = sampleRate / kPaperRate;
    const int excursion = static_cast<int>(kExcursionSeconds * sampleRate) + 4;
    for (int i = 0; i < NodeCount; ++i)
    {
        baseSamples[static_cast<std::size_t>(i)] = static_cast<float>(kPaperLength[i] * ratio);
        line[static_cast<std::size_t>(i)].allocate(static_cast<int>(kPaperLength[i] * ratio * 2.0) + excursion +
                                                   2 * DelayBuffer::kGuard + 16);
    }
    // Output taps: mostly from the opposite branch, some from the own one; signs alternate.
    outputs[0] = {{{D1R, 0.07f, 1.0f},
                   {D1R, 0.66f, 1.0f},
                   {Ap2R, 0.39f, -1.0f},
                   {D2R, 0.22f, 1.0f},
                   {D1L, 0.52f, -1.0f},
                   {Ap2L, 0.81f, -1.0f},
                   {D2L, 0.12f, -1.0f}}};
    outputs[1] = {{{D1L, 0.11f, 1.0f},
                   {D1L, 0.58f, 1.0f},
                   {Ap2L, 0.27f, -1.0f},
                   {D2L, 0.44f, 1.0f},
                   {D1R, 0.71f, -1.0f},
                   {Ap2R, 0.15f, -1.0f},
                   {D2R, 0.63f, -1.0f}}};
    (void)sincKernel();
    resetState();
}

void PlateTank::addSpans(std::vector<ClearSpan>& out)
{
    for (auto& l : line)
        out.push_back(l.span());
}

void PlateTank::clear()
{
    for (auto& l : line)
        l.clear();
    resetState();
}

void PlateTank::resetState()
{
    lb0.fill(1.0f);
    lb1.fill(0.0f);
    la1.fill(0.0f);
    hb0.fill(1.0f);
    hb1.fill(0.0f);
    ha1.fill(0.0f);
    lz.fill(0.0f);
    hz.fill(0.0f);
    feedback.fill(0.0f);
    fresh = true;
    guard.reset();
    feedbackScale = 1.0f;
    energyAccum = 0.0f;
    lastScale = -1.0f;
    lastFreeze = -1;
    freezeState = 0;
    std::uint32_t s = seedValue * 2654435761u + 5u;
    for (std::size_t i = 0; i < noise.size(); ++i)
    {
        noise[i].seed(s + static_cast<std::uint32_t>(i) * 131u + 1u);
        noiseSpeed[i] = 0.7f + 0.6f * uniform01(s);
    }
}

void PlateTank::update(const Controls& c, int n)
{
    chunkSamples = n;
    chunkSeconds = static_cast<float>(n) / static_cast<float>(sr);
    freezeState = c.freeze;
    if (c.freeze == 2)
    {
        if (energyAccum > 0.0f)
            feedbackScale = guard.update(energyAccum / static_cast<float>(2 * std::max(1, n)), chunkSeconds);
    }
    else
    {
        guard.reset();
        feedbackScale = 1.0f;
    }
    energyAccum = 0.0f;

    const float d = std::clamp(c.diffusion, 0.0f, 1.0f);
    g1 = 0.25f + 0.43f * d;
    g2 = 0.15f + 0.36f * d;

    const bool changed = fresh || c.scale != lastScale || c.decay.decay != lastDecay.decay ||
                         c.decay.lowMult != lastDecay.lowMult || c.decay.highMult != lastDecay.highMult ||
                         c.decay.lowHz != lastDecay.lowHz || c.decay.highHz != lastDecay.highHz ||
                         (c.freeze != 0) != (lastFreeze != 0);
    if (changed)
    {
        absorptionCrossovers(c.decay, sr, klow, khigh);
        DecayTarget t = c.decay;
        t.decay = std::max(t.decay, 0.02f);
        t.lowMult = std::clamp(t.lowMult, 0.02f, 8.0f);
        t.highMult = std::clamp(t.highMult, 0.02f, 8.0f);
        const float paramDecay = t.decay;
        if (c.freeze != 0)
        {
            t.decay = 1.0e5f;
            t.lowMult = t.highMult = 1.0f;
        }
        float ring = 0.0f;
        for (int i = 0; i < NodeCount; ++i)
            ring += baseSamples[static_cast<std::size_t>(i)];
        // The signal passes both branches once per round of the ring, so each branch's filter sees half of it.
        const float u = 0.5f * ring * c.scale / static_cast<float>(sr);
        const AbsorptionCoefficients a = designAbsorption(u, t, klow, khigh);
        for (std::size_t h = 0; h < 2; ++h)
        {
            lb0[h] = a.lb0;
            lb1[h] = a.lb1;
            la1[h] = a.la1;
            hb0[h] = a.hb0;
            hb1[h] = a.hb1;
            ha1[h] = a.ha1;
        }
        // Two branches, a pass of energy each; same bookkeeping as the FDN with two lines.
        const float g2sq = std::exp(-2.0f * kRt60Log * u / paramDecay);
        gainNormal = std::sqrt(std::max(2.0f * (1.0f - g2sq), 1.0e-6f)) * 1.0f;
        lastScale = c.scale;
        lastDecay = c.decay;
    }
    lastFreeze = c.freeze;

    const float depth = std::clamp(c.modDepth, 0.0f, 1.0f) * kExcursionSeconds * static_cast<float>(sr);
    const float rate = std::clamp(c.modRate, 0.0f, 20.0f);
    for (int i = 0; i < NodeCount; ++i)
    {
        const auto k = static_cast<std::size_t>(i);
        float target = baseSamples[k] * c.scale;
        if (i == Ap1L || i == Ap1R)
        {
            const std::size_t h = i == Ap1L ? 0 : 1;
            target += depth * noise[h].next(rate * noiseSpeed[h] * chunkSeconds);
        }
        const float cap = static_cast<float>(line[k].capacity() - 2 * DelayBuffer::kGuard);
        target = std::clamp(target, 8.0f, cap);
        const std::int64_t targetQ = static_cast<std::int64_t>(target * 65536.0f);
        if (fresh)
        {
            delayQ[k] = targetQ;
            stepQ[k] = 0;
        }
        else
            stepQ[k] = (targetQ - delayQ[k]) / std::max(1, n);
        tapDelayQ[k] = targetQ;
    }
    fresh = false;
}

void PlateTank::process(const float* mono, float* outL, float* outR, int n)
{
    const float norm = 1.0f / std::sqrt(static_cast<float>(kOutTaps));
    const bool track = freezeState == 2;
    for (int s = 0; s < n; ++s)
    {
        const float x = mono[s] + kTiny;
        std::array<float, 2> next{};
        for (std::size_t h = 0; h < 2; ++h)
        {
            const std::size_t ap1 = h == 0 ? Ap1L : Ap1R, d1 = h == 0 ? D1L : D1R, ap2 = h == 0 ? Ap2L : Ap2R,
                              d2 = h == 0 ? D2L : D2R;
            for (const std::size_t node : {ap1, d1, ap2, d2})
                delayQ[node] += stepQ[node];
            // modulated all-pass
            const float in1 = x + feedback[h] * feedbackScale;
            const float dA = line[ap1].readSinc(delayQ[ap1]);
            const float wA = in1 + g1 * dA;
            line[ap1].write(wA);
            const float y1 = dA - g1 * wA;
            // delay and loss
            const float z = line[d1].readSinc(delayQ[d1]);
            line[d1].write(y1);
            const float t = lb0[h] * z + lz[h];
            lz[h] = lb1[h] * z - la1[h] * t;
            const float u = hb0[h] * t + hz[h];
            hz[h] = hb1[h] * t - ha1[h] * u;
            // second all-pass and delay
            const float dB = line[ap2].readSinc(delayQ[ap2]);
            const float wB = u + g2 * dB;
            line[ap2].write(wB);
            const float y2 = dB - g2 * wB;
            const float o = line[d2].readSinc(delayQ[d2]);
            line[d2].write(y2);
            next[1 - h] = o;
        }
        feedback = next;
        float oL = 0.0f, oR = 0.0f;
        for (int k = 0; k < kOutTaps; ++k)
        {
            const OutTap& a = outputs[0][static_cast<std::size_t>(k)];
            const OutTap& b = outputs[1][static_cast<std::size_t>(k)];
            const auto na = static_cast<std::size_t>(a.node), nb = static_cast<std::size_t>(b.node);
            oL += a.sign *
                  line[na].readSinc(static_cast<std::int64_t>(static_cast<double>(tapDelayQ[na]) * a.fraction) + 6 * 65536);
            oR += b.sign *
                  line[nb].readSinc(static_cast<std::int64_t>(static_cast<double>(tapDelayQ[nb]) * b.fraction) + 6 * 65536);
        }
        outL[s] = oL * norm;
        outR[s] = oR * norm;
        if (track)
            energyAccum += (oL * oL + oR * oR) * norm * norm;
    }
}

} // namespace crucible::fx
