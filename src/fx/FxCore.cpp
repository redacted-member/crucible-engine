// SPDX-License-Identifier: MIT
// Out-of-line parts of the FX core library (include/crucible/fx/): true-peak interpolation tables, box sum,
// lookahead and oversampled block set-up. Control-thread code; the per-sample paths are inline in the headers.

#include <crucible/fx/Detectors.h>
#include <crucible/fx/HalfBandChain.h>
#include <crucible/fx/Lookahead.h>
#include <crucible/fx/OversampledBlock.h>
#include <crucible/fx/Smoothing.h>

#include <algorithm>
#include <cmath>
#if CRUCIBLE_HALFBAND_AVX
#include <intrin.h>
#endif

namespace crucible::fx
{
#if CRUCIBLE_HALFBAND_AVX
namespace
{
// AVX in the CPU and its register state saved by the OS (CPUID.1:ECX bits 27 and 28, XCR0 bits 1 and 2).
bool cpuHasAvx()
{
    int regs[4];
    __cpuid(regs, 1);
    const bool osSaves = (regs[2] & (1 << 27)) != 0, avx = (regs[2] & (1 << 28)) != 0;
    return osSaves && avx && (_xgetbv(0) & 6U) == 6U;
}
// Evaluated once at load; a call before that sees false and takes the SSE loops, which give the same bits.
const bool kHalfBandAvx = cpuHasAvx();
} // namespace

bool detail::halfBandAvx()
{
    return kHalfBandAvx;
}
#endif

namespace
{
constexpr double kPi = 3.14159265358979323846;

double besselI0(double x)
{
    double sum = 1.0, term = 1.0;
    for (int k = 1; k < 80; ++k)
    {
        term *= (x / (2.0 * k)) * (x / (2.0 * k));
        sum += term;
    }
    return sum;
}

double sinc(double t)
{
    return std::abs(t) < 1.0e-12 ? 1.0 : std::sin(kPi * t) / (kPi * t);
}

size_t powerOfTwoAtLeast(size_t n)
{
    size_t p = 4;
    while (p < n)
        p <<= 1;
    return p;
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------
void TruePeakDetector::configure(int factor, bool parabolicRefinement, int tapsPerPhase)
{
    upFactor = factor >= 8 ? 8 : 4;
    taps = upFactor == 8 ? 16 : 12;
    if (tapsPerPhase > 0)
        taps = std::clamp(tapsPerPhase & ~1, 4, kMaxTaps);
    refine = parabolicRefinement;
    const int half = taps / 2;
    const double beta = upFactor == 8 ? 5.5 : 5.0;
    const double norm = besselI0(beta);
    for (auto& row : table)
        row.fill(0.0f);
    for (int p = 0; p < upFactor; ++p)
    {
        const double f = static_cast<double>(p) / upFactor;
        std::array<double, kMaxTaps> w{};
        double sum = 0.0;
        for (int k = 0; k < taps; ++k)
        {
            const double t = static_cast<double>(k - half) + f;
            const double r = t / half;
            const double window = std::abs(r) < 1.0 ? besselI0(beta * std::sqrt(1.0 - r * r)) / norm : 0.0;
            w[static_cast<size_t>(k)] = sinc(t) * window;
            sum += w[static_cast<size_t>(k)];
        }
        for (int k = 0; k < taps; ++k)
            table[static_cast<size_t>(k)][static_cast<size_t>(p)] = static_cast<float>(w[static_cast<size_t>(k)] / sum);
    }
    float worst = 0.0f;
    for (int p = 0; p < upFactor; ++p)
    {
        float sum = 0.0f;
        for (int k = 0; k < taps; ++k)
            sum += std::abs(table[static_cast<size_t>(k)][static_cast<size_t>(p)]);
        worst = std::max(worst, sum);
    }
    bound = worst * (parabolicRefinement ? 1.05f : 1.0f);
    reset();
}

void TruePeakDetector::reset()
{
    history.fill(0.0f);
    pos = 0;
    m1 = m2 = 0.0f;
    previous = simd::Float4::splat(0.0f);
}

// ---------------------------------------------------------------------------------------------------------------
void BoxSum::prepare(int maxWindow)
{
    capacityWindow = std::max(1, maxWindow);
    ring.assign(powerOfTwoAtLeast(static_cast<size_t>(capacityWindow)), 0.0);
    mask = ring.size() - 1;
    windowSize = std::min(windowSize, capacityWindow);
    reset();
}

void BoxSum::reset()
{
    std::fill(ring.begin(), ring.end(), 0.0);
    pos = 0;
    total = 0.0;
    sinceRebuild = 0;
}

void BoxSum::setWindow(int w)
{
    windowSize = std::clamp(w, 1, capacityWindow);
    rebuild();
}

void BoxSum::rebuild()
{
    double s = 0.0;
    for (int i = 1; i <= windowSize; ++i)
        s += ring[(pos - static_cast<size_t>(i)) & mask];
    total = s;
    sinceRebuild = 0;
}

// ---------------------------------------------------------------------------------------------------------------
void Lookahead::prepare(int numChannels, int maxDelaySamples, double sampleRate, double fadeSeconds)
{
    channels = std::clamp(numChannels, 1, kMaxChannels);
    maximum = std::max(0, maxDelaySamples);
    const size_t cap = powerOfTwoAtLeast(static_cast<size_t>(maximum) + 2);
    mask = cap - 1;
    for (int c = 0; c < kMaxChannels; ++c)
        buffer[c].assign(c < channels ? cap : 0, 0.0f);
    fadeLength = std::max(1, static_cast<int>(std::lround(fadeSeconds * sampleRate)));
    target = std::min(target, maximum);
    reset();
}

void Lookahead::reset()
{
    for (int c = 0; c < channels; ++c)
        std::fill(buffer[c].begin(), buffer[c].end(), 0.0f);
    writePos = 0;
    fadeFrom = fadeTo = target;
    fadePosition = fadeLength;
    hasPending = false;
    started = false;
}

void Lookahead::setDelay(int samples)
{
    const int s = std::clamp(samples, 0, maximum);
    if (!started)
    {
        target = fadeFrom = fadeTo = s;
        return;
    }
    if (s == target)
        return;
    target = s;
    if (fading())
    {
        hasPending = s != fadeTo;
        pending = s;
        return;
    }
    fadeTo = s;
    fadePosition = 0;
}

void Lookahead::advanceFade()
{
    if (!fading())
        return;
    if (++fadePosition >= fadeLength)
    {
        fadeFrom = fadeTo;
        if (hasPending && pending != fadeFrom)
        {
            fadeTo = pending;
            fadePosition = 0;
        }
        hasPending = false;
    }
}

void Lookahead::process(float* const* io, int n)
{
    started = true;
    for (int i = 0; i < n; ++i)
    {
        const bool fade = fading();
        const float w = fade ? smootherstep(static_cast<float>(fadePosition + 1) / static_cast<float>(fadeLength)) : 0.0f;
        for (int c = 0; c < channels; ++c)
        {
            auto& b = buffer[c];
            b[writePos & mask] = io[c][i];
            const float a = b[(writePos - static_cast<size_t>(fadeFrom)) & mask];
            if (fade)
            {
                const float t = b[(writePos - static_cast<size_t>(fadeTo)) & mask];
                io[c][i] = a + (t - a) * w;
            }
            else
                io[c][i] = a;
        }
        ++writePos;
        advanceFade();
    }
}

float Lookahead::processSample(float x)
{
    float* p = &x;
    process(&p, 1);
    return x;
}

// ---------------------------------------------------------------------------------------------------------------
void OversampledBlock::prepare(int numChannels, int stages, OversamplingFilter filter, Quality quality, int maxBlock,
                               DryPath dry)
{
    channels = std::clamp(numChannels, 1, kMaxChannels);
    stages = std::clamp(stages, 0, 3);
    upFactor = 1 << stages;
    dryMode = dry;
    blockLimit = std::max(1, maxBlock);
    measuredLatency = 0.0f;
    if (stages > 0)
    {
        for (int c = 0; c < channels; ++c)
        {
            os[static_cast<size_t>(c)].prepare(stages, filter, quality, blockLimit);
            if (dry == DryPath::Matched)
                dryOs[static_cast<size_t>(c)].prepare(stages, filter, quality, blockLimit);
        }
        measuredLatency = os[0].latency();
    }
    reported = static_cast<int>(std::lround(measuredLatency));
    for (int c = 0; c < kMaxChannels; ++c)
    {
        const bool on = c < channels;
        const auto cs = static_cast<size_t>(c);
        wet[cs].assign(on ? static_cast<size_t>(blockLimit) * static_cast<size_t>(upFactor) : 0, 0.0f);
        dryBlock[cs].assign(on ? static_cast<size_t>(blockLimit) : 0, 0.0f);
        dryRing[cs].assign(on ? powerOfTwoAtLeast(static_cast<size_t>(reported) + 2) : 0, 0.0f);
    }
    dryMask = dryRing[0].size() - 1;
    reset();
}

void OversampledBlock::reset()
{
    for (int c = 0; c < channels; ++c)
    {
        const auto cs = static_cast<size_t>(c);
        os[cs].reset();
        dryOs[cs].reset();
        std::fill(dryRing[cs].begin(), dryRing[cs].end(), 0.0f);
    }
    dryPos = 0;
    mixPrimed = false;
}

} // namespace crucible::fx
