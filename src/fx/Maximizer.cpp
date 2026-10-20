// SPDX-License-Identifier: MIT
#include <crucible/fx/Maximizer.h>
#include <crucible/fx/Smoothing.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace crucible::fx
{
namespace
{
constexpr float kDbPerOctave = 6.02059991f; // 20 log10(2)

// One step of a one-pole that lands exactly on its target (a constant control must give a constant, bit-exact value).
inline bool settle(float& v, float target, float coef)
{
    v = onePoleStepSnap(v, target, coef);
    if (std::abs(target - v) <= 1.0e-6f * std::max(std::abs(target), 1.0e-3f))
    {
        v = target;
        return true;
    }
    return false;
}

inline float finiteOrZero(float v)
{
    return (v == v && std::abs(v) < 1.0e18f) ? v : 0.0f;
}
} // namespace

void TruePeakLimiter::Lane::clear()
{
    truePeak.reset();
    hold.reset();
    box.reset();
    clip.reset();
    m1 = m2 = fast = slow = 0.0f;
    for (float& g : groupMax)
        g = 0.0f;
    groupFill = 0;
}

void TruePeakLimiter::updateLog2Ceiling()
{
    log2Ceiling = dyn::log2Fast(ceilSmoothed) - (kDetectorMarginDb + extraMarginDb) / kDbPerOctave;
}

void TruePeakLimiter::updateClipCeiling()
{
    clipCeiling = ceilSmoothed * dyn::dbToGain((1.0f - shareSmoothed) * 6.0f);
}

void TruePeakLimiter::prepare(double sampleRate, Detect mode, int g, int tapsPerPhase)
{
    sr = sampleRate > 0.0 ? sampleRate : 48000.0;
    detector = mode;
    granularity = std::max(1, g);
    maxLook = granularity * static_cast<int>(std::ceil(kMaxLookaheadMs * 0.001 * sr / granularity)) + granularity;
    const int maxHold = static_cast<int>(std::ceil(kMaxHoldMs * 0.001 * sr)) + maxLook + 8;
    LogCoshTable::instance();
    for (auto& l : lane)
    {
        l.truePeak.configure(4, true, tapsPerPhase);
        l.hold.prepare(maxHold);
        l.box.prepare(maxLook);
    }
    detectLatency = detector == Detect::TruePeak ? lane[0].truePeak.latencySamples() : 1;
    delayLine.prepare(2, maxLook + detectLatency + 8, sr);
    size_t ring = 1;
    while (ring < static_cast<size_t>(maxLook + detectLatency + 8 + 2 * kChunk))
        ring <<= 1;
    ceilRing.assign(ring, 0.0f);
    ceilMask = ring - 1;
    // The crest factor and the bass period are slow: they are tracked on every stride-th sample.
    stride = 2 * granularity;
    const double hk = sr / stride;
    lowpassCoef = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 300.0 / hk));
    bassEnvDecay = static_cast<float>(std::exp(-1.0 / (0.2 * hk)));
    crestPeakDecay = static_cast<float>(std::exp(-1.0 / (0.4 * hk)));
    crestPowerCoef = static_cast<float>(1.0 - std::exp(-1.0 / (0.4 * hk)));
    gainCoef = static_cast<float>(1.0 - std::exp(-1.0 / (0.020 * sr)));
    configured = false;
    reset();
}

void TruePeakLimiter::reset()
{
    for (auto& l : lane)
        l.clear();
    delayLine.reset();
    std::fill(ceilRing.begin(), ceilRing.end(), 0.0f);
    ceilPos = 0;
    primed = false;
    crestPeak = crestPower = 0.0f;
    crestScale = 1.0f;
    lowpassA = lowpassB = bassEnv = 0.0f;
    armed = haveCross = false;
    sinceCross = 0;
    period = 0.0f;
    autoWindow = 0;
    meter.store(0.0f, std::memory_order_relaxed);
}

int TruePeakLimiter::lookSamples(const LimiterSettings& s) const
{
    const float ms = std::clamp(s.lookaheadMs, 0.5f, kMaxLookaheadMs);
    int n = std::max(2, static_cast<int>(std::lround(static_cast<double>(ms) * 0.001 * sr)));
    n = ((n + granularity - 1) / granularity) * granularity;
    return std::min(n, maxLook);
}

int TruePeakLimiter::latencyFor(const LimiterSettings& s) const
{
    return lookSamples(s) - 1 + detectLatency;
}

void TruePeakLimiter::updateTimes(const LimiterSettings& s)
{
    const float scale = s.releaseMode == ReleaseMode::Adaptive ? crestScale : 1.0f;
    const double fastMs = std::clamp(static_cast<double>(s.releaseMs), 1.0, 5000.0) * scale;
    const double slowMs = std::min(6.0 * fastMs, 20000.0);
    const double chargeMs = std::max(4.0 * fastMs, 100.0);
    const auto perSample = [&](double ms) { return static_cast<float>(std::exp(-1.0 / (ms * 0.001 * sr))); };
    decayFast = perSample(fastMs);
    decaySlow = perSample(slowMs);
    chargeSlow = 1.0f - perSample(chargeMs);
}

void TruePeakLimiter::configure(const LimiterSettings& s)
{
    const float ceilingDb = std::clamp(s.ceilingDb, -60.0f, 0.0f);
    ceilingLinear = dyn::dbToGain(ceilingDb);
    gainLinear = dyn::dbToGain(std::clamp(s.gainDb, -24.0f, 48.0f));
    if (!primed)
        gainSmoothed = gainStage2 = gainStage1 = gainLinear;
    // Moving ceiling and clip share glide over at least the lookahead (and 5 ms).
    const double glideSeconds = std::max(0.005, static_cast<double>(lookSamples(s)) / sr);
    ceilCoef = static_cast<float>(1.0 - std::exp(-1.0 / (glideSeconds * sr)));
    if (!primed)
        ceilStage1 = ceilSmoothed = ceilingLinear;
    ceilMoving = ceilStage1 != ceilingLinear || ceilSmoothed != ceilingLinear;
    updateLog2Ceiling();

    const int newLook = lookSamples(s);
    if (!configured || newLook != look)
    {
        look = newLook;
        for (auto& l : lane)
            l.box.setWindow(look);
        const int delay = look - 1 + detectLatency;
        delayLine.setDelay(delay);
        if (latency.exchange(delay, std::memory_order_relaxed) != delay && configured)
            latencyChanged.store(true, std::memory_order_relaxed);
    }
    const int manual =
        static_cast<int>(std::lround(static_cast<double>(std::clamp(s.holdMs, 0.0f, kMaxHoldMs)) * 0.001 * sr));
    const int wanted = std::max({look, manual, s.autoHold ? autoWindow : 0});
    if (!configured || wanted != holdWindow)
    {
        holdWindow = wanted;
        for (auto& l : lane)
            l.hold.setWindow(holdWindow);
    }
    clipOn = s.clipShare > 0.001f;
    clipOrder = std::clamp(s.clipOrder, 0, 2);
    const float share = std::clamp(s.clipShare, 0.0f, 1.0f);
    shareTarget = share;
    const float fadeTarget = clipOn ? 1.0f : 0.0f;
    if (!primed)
    {
        shareStage1 = shareSmoothed = share;
        fadeStage1 = fadeSmoothed = fadeTarget;
    }
    if (primed && clipOn && fadeSmoothed == 0.0f && fadeStage1 == 0.0f)
        for (auto& l : lane)
            l.clip.reset(); // fading in from idle: start from a clean clip state
    shareMoving = shareStage1 != share || shareSmoothed != share;
    fadeMoving = fadeStage1 != fadeTarget || fadeSmoothed != fadeTarget;
    updateClipCeiling();
    for (auto& l : lane)
        l.clip.setSoftness(s.clipSoftness);
    if (!configured || s.releaseMs != applied.releaseMs || s.releaseMode != applied.releaseMode)
        updateTimes(s);
    applied = s;
    configured = true;
}

float TruePeakLimiter::detect(Lane& l, float x)
{
    const float a = std::abs(x);
    if (detector == Detect::TruePeak)
    {
        // The interpolator is evaluated only when the newest samples could reach the ceiling: nothing below it matters.
        l.groupMax[0] = std::max(l.groupMax[0], a);
        if (++l.groupFill == 16)
        {
            l.groupFill = 0;
            l.groupMax[2] = l.groupMax[1];
            l.groupMax[1] = l.groupMax[0];
            l.groupMax[0] = 0.0f;
        }
        const float recent = std::max(l.groupMax[0], std::max(l.groupMax[1], l.groupMax[2]));
        if (recent * l.truePeak.gainBound() <= ceilSmoothed)
        {
            l.truePeak.skip(x);
            return a;
        }
        return std::max(a, l.truePeak.process(x));
    }
    // Sample peaks refined by the vertex of the parabola through the three magnitudes around a local maximum.
    const float m = a;
    const float den = std::min(l.m2 - 2.0f * l.m1 + m, -1.0e-30f);
    const float diff = l.m2 - m;
    const float vertex = l.m1 - 0.125f * diff * diff / den;
    const bool peakHere = l.m1 >= l.m2 && l.m1 > m && l.m1 > 0.0f;
    const float p = std::max(l.m1, peakHere ? vertex : 0.0f);
    l.m2 = l.m1;
    l.m1 = m;
    return p;
}

float TruePeakLimiter::advance(Lane& l, float amount, ReleaseMode mode)
{
    const float h = l.hold.push(amount);
    l.fast = std::max(h, l.fast * decayFast);
    float r = l.fast;
    if (mode != ReleaseMode::Fixed)
    {
        l.slow = h > l.slow ? l.slow + (h - l.slow) * chargeSlow : std::max(h, l.slow * decaySlow);
        r = std::max(r, l.slow);
    }
    if (l.fast < 1.0e-5f)
        l.fast = 0.0f;
    if (l.slow < 1.0e-5f)
        l.slow = 0.0f;
    l.box.push(static_cast<double>(r));
    return static_cast<float>(l.box.average());
}

void TruePeakLimiter::process(float* left, float* right, int n, const LimiterSettings& s)
{
    configure(s);
    const bool linkedFully = s.link >= 0.999f;
    const bool needSlow = s.releaseMode == ReleaseMode::Adaptive || s.autoHold;
    if (!linkedFully && wasLinked)
    {
        // The second lane has not been running: it takes over the shared lane's gain state so that no peak is forgotten.
        lane[1].hold = lane[0].hold;
        lane[1].box = lane[0].box;
        lane[1].fast = lane[0].fast;
        lane[1].slow = lane[0].slow;
    }
    wasLinked = linkedFully;
    const float link = std::clamp(s.link, 0.0f, 1.0f);
    float meterMax = 0.0f;
    for (int base = 0; base < n; base += kChunk)
    {
        const int m = std::min(kChunk, n - base);
        // Stage 1: gain, clip stage, detection, gain computer.
        for (int i = 0; i < m; ++i)
        {
            gainStage1 = onePoleStepSnap(gainStage1, gainLinear, gainCoef);
            gainStage2 = onePoleStepSnap(gainStage2, gainStage1, gainCoef);
            gainSmoothed = onePoleStepSnap(gainSmoothed, gainStage2, gainCoef);
            if (ceilMoving)
            {
                const bool a = settle(ceilStage1, ceilingLinear, ceilCoef);
                const bool b = settle(ceilSmoothed, ceilStage1, ceilCoef);
                if (a && b)
                    ceilMoving = false;
                updateLog2Ceiling();
            }
            if (shareMoving)
            {
                const bool a = settle(shareStage1, shareTarget, ceilCoef);
                const bool b = settle(shareSmoothed, shareStage1, ceilCoef);
                if (a && b)
                    shareMoving = false;
            }
            if (ceilMoving || shareMoving)
                updateClipCeiling();
            ceilBuf[static_cast<size_t>(i)] = ceilSmoothed;
            if (fadeMoving)
            {
                const float ft = clipOn ? 1.0f : 0.0f;
                const bool a = settle(fadeStage1, ft, ceilCoef);
                const bool b = settle(fadeSmoothed, fadeStage1, ceilCoef);
                if (a && b)
                    fadeMoving = false;
            }
            float xl = finiteOrZero(left[base + i]) * gainSmoothed;
            float xr = finiteOrZero(right[base + i]) * gainSmoothed;
            if (clipOn || fadeSmoothed > 0.0f)
            {
                const float inv = 1.0f / clipCeiling;
                const float cl = lane[0].clip.process(xl * inv, clipOrder) * clipCeiling;
                const float cr = lane[1].clip.process(xr * inv, clipOrder) * clipCeiling;
                if (fadeSmoothed >= 1.0f)
                {
                    xl = cl;
                    xr = cr;
                }
                else
                {
                    xl += (cl - xl) * fadeSmoothed;
                    xr += (cr - xr) * fadeSmoothed;
                }
            }

            bufL[static_cast<size_t>(i)] = xl;
            bufR[static_cast<size_t>(i)] = xr;

            // Crest factor of the program and the bass period, both slow.
            if (needSlow && ++strideCount >= stride)
            {
                strideCount = 0;
                const float pk = std::max(std::abs(xl), std::abs(xr));
                crestPeak = std::max(pk, crestPeak * crestPeakDecay);
                crestPower += (pk * pk - crestPower) * crestPowerCoef;
                const float mono = 0.5f * (xl + xr);
                lowpassA += lowpassCoef * (mono - lowpassA);
                lowpassB += lowpassCoef * (lowpassA - lowpassB);
                bassEnv = std::max(std::abs(lowpassB), bassEnv * bassEnvDecay);
                sinceCross += stride;
                if (armed && lowpassB > 0.1f * bassEnv && bassEnv > 1.0e-4f)
                {
                    if (haveCross)
                    {
                        const auto interval = static_cast<float>(sinceCross);
                        if (interval >= sr / 400.0 && interval <= sr / 20.0)
                            period = period > 0.0f ? 0.6f * period + 0.4f * interval : interval;
                    }
                    haveCross = true;
                    sinceCross = 0;
                    armed = false;
                }
                else if (lowpassB < -0.1f * bassEnv)
                    armed = true;
                if (sinceCross > static_cast<int>(0.08 * sr))
                    period = 0.0f;
            }

            const float p0 = detect(lane[0], xl);
            const float p1 = detect(lane[1], xr);
            float a0 = std::max(0.0f, kDbPerOctave * (dyn::log2Fast(p0) - log2Ceiling));
            float a1 = std::max(0.0f, kDbPerOctave * (dyn::log2Fast(p1) - log2Ceiling));
            float rOut[2];
            if (linkedFully)
            {
                // One shared gain: the second lane's gain state is not run (it is copied when the link opens).
                rOut[0] = rOut[1] = advance(lane[0], std::max(a0, a1), s.releaseMode);
            }
            else
            {
                const float peak = std::max(a0, a1);
                a0 += link * (peak - a0);
                a1 += link * (peak - a1);
                rOut[0] = advance(lane[0], a0, s.releaseMode);
                rOut[1] = advance(lane[1], a1, s.releaseMode);
            }
            meterMax = std::max({meterMax, rOut[0], rOut[1]});
            gainA[static_cast<size_t>(i)] = dyn::dbToGain(-rOut[0]);
            gainB[static_cast<size_t>(i)] = linkedFully ? gainA[static_cast<size_t>(i)] : dyn::dbToGain(-rOut[1]);
        }
        // Slow housekeeping, once per chunk: release scaling from the crest factor, the hold window from the bass period.
        if (needSlow)
        {
            const float peakPower = crestPeak * crestPeak;
            const float crest = crestPower > 1.0e-12f
                                    ? std::clamp(dyn::powerToDb(peakPower) - dyn::powerToDb(crestPower), 0.0f, 30.0f)
                                    : 30.0f;
            const float scale = std::clamp(dyn::exp2Fast((8.0f - crest) * 0.25f), 0.25f, 4.0f);
            if (std::abs(scale - crestScale) > 0.02f * crestScale)
            {
                crestScale = scale;
                if (s.releaseMode == ReleaseMode::Adaptive)
                    updateTimes(s);
            }
            if (s.autoHold)
            {
                const int window =
                    period > 0.0f ? std::min(static_cast<int>(0.55f * period), static_cast<int>(0.008 * sr)) : 0;
                if (std::abs(window - autoWindow) > std::max(2, autoWindow / 16))
                {
                    autoWindow = window;
                    const int manual = static_cast<int>(
                        std::lround(static_cast<double>(std::clamp(s.holdMs, 0.0f, kMaxHoldMs)) * 0.001 * sr));
                    const int wanted = std::max({look, manual, autoWindow});
                    if (wanted != holdWindow)
                    {
                        holdWindow = wanted;
                        for (auto& l : lane)
                            l.hold.setWindow(holdWindow);
                    }
                }
            }
        }
        // Stage 2: delay the audio, apply the gain, hold the ceiling.
        float* chans[2] = {bufL.data(), bufR.data()};
        const int dFrom = delayLine.activeDelay(), dTo = delayLine.fadeTargetDelay(), dTarget = delayLine.delay();
        delayLine.process(chans, m);
        for (int i = 0; i < m; ++i)
            ceilRing[(ceilPos + static_cast<size_t>(i)) & ceilMask] = ceilBuf[static_cast<size_t>(i)];
        const size_t d0 = static_cast<size_t>(dFrom), d1 = static_cast<size_t>(dTo), d2 = static_cast<size_t>(dTarget);
        for (int i = 0; i < m; ++i)
        {
            const size_t k = static_cast<size_t>(i);
            const size_t at = ceilPos + k;
            float ceil = ceilRing[(at - d0) & ceilMask];
            if (d1 != d0)
                ceil = std::min(ceil, ceilRing[(at - d1) & ceilMask]);
            if (d2 != d0 && d2 != d1)
                ceil = std::min(ceil, ceilRing[(at - d2) & ceilMask]);
            if (!(ceil > 0.0f))
                ceil = ceilBuf[k]; // before the first delayed value has arrived
            float ol, orr;
            if (s.sideBias)
            {
                const float g = std::min(gainA[k], gainB[k]);
                const float mid = 0.5f * (bufL[k] + bufR[k]) * g;
                const float side = 0.5f * (bufL[k] - bufR[k]) * g * g;
                ol = mid + side;
                orr = mid - side;
            }
            else
            {
                ol = bufL[k] * gainA[k];
                orr = bufR[k] * gainB[k];
            }
            left[base + i] = std::clamp(ol, -ceil, ceil);
            right[base + i] = std::clamp(orr, -ceil, ceil);
        }
        ceilPos += static_cast<size_t>(m);
    }
    primed = true;
    meter.store(meterMax, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------------------------------------------
void Maximizer::prepare(double sampleRate, int maxBlock, Quality quality, OversamplingFilter filter, std::uint32_t seedValue)
{
    sr = sampleRate > 0.0 ? sampleRate : 48000.0;
    direct.prepare(sr, TruePeakLimiter::Detect::TruePeak, 1);
    over.prepare(sr * 4.0, TruePeakLimiter::Detect::Refined, 4);
    block.prepare(2, 2, filter, quality, std::max(1, maxBlock));
    filterGainDb = measureFilterGainDb(std::max(1, maxBlock));
    // The filters' pass-band gain plus 0.06 dB for the peaks the refined 4x-grid detector cannot place between its samples.
    over.setExtraMarginDb(filterGainDb + 0.06f);
    seed = seedValue == 0 ? 1u : seedValue;
    usingOver = false;
    started = false;
    latency.store(direct.latencyFor(LimiterSettings{}), std::memory_order_relaxed);
    reset();
}

// Largest gain of the 4x path's up and down filters over sines from 1 to 20 kHz (the ripple of the pass band): the limiter
// holds that much more headroom so that a peak it just reached does not leave the filter above the ceiling.
float Maximizer::measureFilterGainDb(int maxBlock)
{
    const int n = 4096, chunk = std::min(maxBlock, 512);
    std::vector<float> a(static_cast<size_t>(n)), b(static_cast<size_t>(n));
    double worst = 1.0;
    for (const double hz : {1000.0, 3000.0, 6000.0, 9000.0, 12000.0, 14000.0, 16000.0, 17500.0, 19000.0, 20000.0})
    {
        if (hz > 0.45 * sr)
            break;
        block.reset();
        for (int i = 0; i < n; ++i)
            a[static_cast<size_t>(i)] = b[static_cast<size_t>(i)] =
                0.5f * static_cast<float>(std::sin(2.0 * std::numbers::pi * hz * i / sr));
        for (int pos = 0; pos < n; pos += chunk)
        {
            float* io[2] = {a.data() + pos, b.data() + pos};
            block.process(io, std::min(chunk, n - pos), 1.0f, [](float* const*, int, int) {});
        }
        double sum = 0.0;
        for (int i = n - 2048; i < n; ++i)
            sum += static_cast<double>(a[static_cast<size_t>(i)]) * a[static_cast<size_t>(i)];
        worst = std::max(worst, std::sqrt(sum / 2048.0) * std::sqrt(2.0) / 0.5);
    }
    block.reset();
    return static_cast<float>(20.0 * std::log10(worst));
}

void Maximizer::reset()
{
    direct.reset();
    over.reset();
    block.reset();
    rng = seed;
    errorL[0] = errorL[1] = errorR[0] = errorR[1] = 0.0f;
    meter.store(0.0f, std::memory_order_relaxed);
}

int Maximizer::latencyFor(const MaximizerSettings& s) const
{
    if (!s.oversample)
        return direct.latencyFor(s.limiter);
    return block.latencySamples() + over.latencyFor(s.limiter) / 4;
}

void Maximizer::process(float* left, float* right, int n, const MaximizerSettings& s)
{
    if (s.oversample != usingOver || !started)
    {
        if (started)
        {
            // Switching paths: the one that takes over starts from silence.
            (s.oversample ? over : direct).reset();
            block.reset();
        }
        usingOver = s.oversample;
        started = true;
    }
    float gr;
    if (!usingOver)
    {
        direct.process(left, right, n, s.limiter);
        gr = direct.gainReductionDb();
    }
    else
    {
        float* io[2] = {left, right};
        LimiterSettings ls = s.limiter;
        ls.clipOrder = 1; // the clip stage runs at four times the rate: first-order ADAA costs 0.6 dB at 20 kHz there
        block.process(io, n, 1.0f, [&](float* const* x, int, int count) { over.process(x[0], x[1], count, ls); });
        gr = over.gainReductionDb();
        // The filter that brings the audio back down can overshoot the sample grid by a hair: the ceiling holds there as
        // well.
        const float ceil = dyn::dbToGain(std::clamp(ls.ceilingDb, -60.0f, 0.0f));
        for (int i = 0; i < n; ++i)
        {
            left[i] = std::clamp(left[i], -ceil, ceil);
            right[i] = std::clamp(right[i], -ceil, ceil);
        }
    }
    meter.store(gr, std::memory_order_relaxed);
    const int now = latencyFor(s);
    if (latency.exchange(now, std::memory_order_relaxed) != now)
        latencyChanged.store(true, std::memory_order_relaxed);
    if (s.dither != DitherMode::Off)
        applyDither(left, right, n, s);
}

void Maximizer::applyDither(float* l, float* r, int n, const MaximizerSettings& s)
{
    const int bits = std::clamp(s.ditherBits, 8, 24);
    const float q = std::ldexp(1.0f, 1 - bits); // one step of a signed word with +-1 as full scale
    const float inv = 1.0f / q;
    const float ceil = dyn::dbToGain(std::clamp(s.limiter.ceilingDb, -60.0f, 0.0f));
    const bool shaped = s.dither == DitherMode::Shaped;
    // Noise transfer function (1 - 0.9 z^-1)(1 - 0.5 z^-1): the quantisation error is pushed toward high frequencies.
    constexpr float c1 = 1.4f, c2 = -0.45f;
    const auto uniform = [this]()
    {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return static_cast<float>(rng >> 8) * (1.0f / 16777216.0f);
    };
    for (int i = 0; i < n; ++i)
    {
        float* chan[2] = {l, r};
        float* err[2] = {errorL, errorR};
        for (int c = 0; c < 2; ++c)
        {
            const float tpdf = uniform() + uniform() - 1.0f;
            const float x = chan[c][i] * inv;
            const float v = shaped ? x - c1 * err[c][0] - c2 * err[c][1] : x;
            const float y = std::nearbyint(v + tpdf);
            if (shaped)
            {
                err[c][1] = err[c][0];
                err[c][0] = std::clamp(y - v, -4.0f, 4.0f);
            }
            chan[c][i] = std::clamp(y * q, -ceil, ceil);
        }
    }
}

} // namespace crucible::fx
