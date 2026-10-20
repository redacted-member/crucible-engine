// SPDX-License-Identifier: MIT
#include <crucible/fx/Dynamics.h>
#include <crucible/fx/Smoothing.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace crucible::fx
{
namespace
{
constexpr float kFloor = 1.0e-30f;

// Filter and detector state: denormals, NaN and infinities become zero so that one bad input sample cannot lock the
// sidechain.
inline float flush(float v)
{
    return std::abs(v) >= kFloor && std::abs(v) <= 1.0e18f ? v : 0.0f;
}

// One-pole coefficient for a time constant (63 % of a step), 1 for zero.
inline float onePole(float ms, double sampleRate)
{
    if (ms <= 0.0f)
        return 1.0f;
    return 1.0f - static_cast<float>(std::exp(-1000.0 / (static_cast<double>(ms) * sampleRate)));
}

// Smoothing time of the gain-computer inputs, makeup and mix: long enough that a once-per-block change leaves no step or
// slope break, short enough to feel immediate.
constexpr float kSmoothMs = 6.0f;
constexpr float kFilterGlideMs = 20.0f; // sidechain filter corners
// The ratio acts on the gain through a slope, so it needs a longer glide to stay clean.
constexpr float kRatioSmoothMs = 15.0f;

// One smoothing step; snaps onto the target so that constant settings stay bit-identical.
inline float smoothTo(float v, float target, float coef)
{
    const float diff = target - v;
    return std::abs(diff) <= 1.0e-6f * std::max(1.0f, std::abs(target)) ? target : onePoleStepSnap(v, target, coef);
}

// Zavalishin's trapezoidal state-variable filter. v1 is the band-pass (peak gain 1 / k), v2 the low-pass.
struct SvfTick
{
    float lp, bp, hp;
};
inline SvfTick svfTick(float& ic1, float& ic2, float x, float a1, float a2, float a3, float k)
{
    const float v3 = x - ic2;
    const float v1 = a1 * ic1 + a2 * v3;
    const float v2 = ic2 + a2 * ic1 + a3 * v3;
    ic1 = flush(2.0f * v1 - ic1);
    ic2 = flush(2.0f * v2 - ic2);
    return {v2, v1, x - k * v1 - v2};
}
} // namespace

void DynamicsProcessor::SvfCoefficients::design(double sampleRate, float hz, float q)
{
    const double fc = std::clamp(static_cast<double>(hz), 1.0, 0.45 * sampleRate);
    const double g = std::tan(std::numbers::pi * fc / sampleRate);
    k = 1.0f / std::max(q, 0.05f);
    const double denom = 1.0 / (1.0 + g * (g + static_cast<double>(k)));
    a1 = static_cast<float>(denom);
    a2 = static_cast<float>(g * denom);
    a3 = static_cast<float>(g * g * denom);
}

void HoldMax::prepare(int maxWindow)
{
    capacity = std::max(1, maxWindow);
    size_t cap = 4;
    while (cap < static_cast<size_t>(capacity) + 1)
        cap <<= 1;
    ringMask = cap - 1;
    ring.assign(cap, 0.0f);
    current.assign(static_cast<size_t>(capacity) + 1, 0.0f);
    suffix.assign(static_cast<size_t>(capacity) + 2, 0.0f);
    size = std::min(size, capacity);
    reset();
}

void HoldMax::reset()
{
    std::fill(ring.begin(), ring.end(), 0.0f);
    ringPos = 0;
    rebuild();
}

void HoldMax::setWindow(int w)
{
    w = std::clamp(w, 1, capacity);
    if (w == size)
        return;
    size = w;
    rebuild();
}

// The previous block becomes the last `size` raw values; the current block starts empty.
void HoldMax::rebuild()
{
    pos = 0;
    prefix = 0.0f;
    suffix[static_cast<size_t>(size)] = 0.0f;
    float running = 0.0f;
    for (int j = size - 1; j >= 0; --j)
    {
        running = std::max(running, ring[(ringPos - static_cast<size_t>(size - j)) & ringMask]);
        suffix[static_cast<size_t>(j)] = running;
    }
}

void HoldMax::closeBlock()
{
    float running = 0.0f;
    for (int j = size - 1; j >= 0; --j)
    {
        running = std::max(running, current[static_cast<size_t>(j)]);
        suffix[static_cast<size_t>(j)] = running;
    }
    suffix[static_cast<size_t>(size)] = 0.0f;
    pos = 0;
}

void DynamicsProcessor::Lane::clear()
{
    hp.clear();
    lp.clear();
    bell.clear();
    truePeak.reset();
    peakWindow.reset();
    power1 = power2 = reduction = slow = lastAmount = feedbackSample = 0.0f;
    hold = triggerHold = 0;
    triggered = false;
}

void DynamicsProcessor::prepare(double sampleRate)
{
    sr = sampleRate;
    const int maxDelay = static_cast<int>(std::ceil(kMaxLookaheadMs * 0.001 * sampleRate)) + 32;
    look.prepare(2, maxDelay, sampleRate);
    for (auto& l : lane)
    {
        l.truePeak.configure(4, false);
        l.peakWindow.prepare(static_cast<int>(std::ceil(kMaxLookaheadMs * 0.001 * sampleRate)) + 1);
    }
    smoothCoef = onePole(kSmoothMs, sr);
    ratioCoef = onePole(kRatioSmoothMs, sr);
    configured = false;
    latencyKnown = false;
    reset();
}

void DynamicsProcessor::reset()
{
    for (auto& l : lane)
        l.clear();
    look.reset();
    meter.store(0.0f, std::memory_order_relaxed);
    configured = false;
    primed = 0;
}

int DynamicsProcessor::latencyFor(const DynamicsSettings& s) const
{
    if (s.feedback)
        return 0;
    int samples = static_cast<int>(std::lround(std::clamp(s.lookaheadMs, 0.0f, kMaxLookaheadMs) * 0.001 * sr));
    if (s.truePeak && s.detector != DetectorMode::Rms)
        samples += lane[0].truePeak.latencySamples();
    return std::min(samples, look.maxDelay());
}

void DynamicsProcessor::configure(const DynamicsSettings& s)
{
    const bool first = !configured;
    if (first || s.attackMs != applied.attackMs || s.releaseMs != applied.releaseMs || s.autoFastMs != applied.autoFastMs ||
        s.autoSlowMs != applied.autoSlowMs || s.autoChargeMs != applied.autoChargeMs || s.holdMs != applied.holdMs ||
        s.rmsWindowMs != applied.rmsWindowMs || s.peakHoldMs != applied.peakHoldMs || s.detector != applied.detector ||
        s.autoRelease != applied.autoRelease)
    {
        // The detector delays what the ballistics see: a 20 dB step of level reaches the gain computer late (the
        // RMS average rises quickly in dB but falls slowly, the peak window holds the old level for its length). The
        // one-pole times are shortened by that delay, measured on 20 dB steps, so the time to 63 % of the gain step matches
        // the label when it is at least a few times the detector time; shorter labels bottom out at a quarter of themselves.
        float attackComp = 0.0f, releaseComp = 0.0f;
        const float rmsAttack = 0.5f * s.rmsWindowMs, rmsRelease = 2.1f * s.rmsWindowMs, peakRelease = s.peakHoldMs;
        switch (s.detector)
        {
        case DetectorMode::Peak:
            releaseComp = peakRelease;
            break;
        case DetectorMode::Rms:
            attackComp = rmsAttack;
            releaseComp = rmsRelease;
            break;
        default:
            attackComp = 0.5f * rmsAttack;
            releaseComp = 0.5f * (rmsRelease + peakRelease);
            break;
        }
        const auto shorten = [](float ms, float comp) { return std::max(ms - comp, 0.25f * ms); };
        d.attack = onePole(shorten(s.attackMs, attackComp), sr);
        d.release = onePole(shorten(s.autoRelease ? s.autoFastMs : s.releaseMs, releaseComp), sr);
        d.slowRelease = onePole(s.autoSlowMs, sr);
        d.slowCharge = onePole(s.autoChargeMs, sr);
        d.peakWindow =
            std::max(1, static_cast<int>(std::lround(std::clamp(s.peakHoldMs, 0.0f, kMaxLookaheadMs) * 0.001 * sr)));
        lane[0].peakWindow.setWindow(d.peakWindow);
        lane[1].peakWindow.setWindow(d.peakWindow);
        d.rms = onePole(0.5f * s.rmsWindowMs, sr);
        d.holdSamples = static_cast<int>(std::lround(std::max(0.0f, s.holdMs) * 0.001 * sr));
    }

    // The audio is delayed only once the delay line holds that much real input: right after a reset (switch-on, transport)
    // the wet path is first aligned with the dry one and then crossfaded to the delayed tap, so a module that fades in
    // never exposes a gap of silence. The reported latency is the steady-state value throughout.
    const int want = latencyFor(s);
    // With no lookahead and none fading out the delay line is not even written (see process()); the history starts
    // again when a lookahead is next asked for.
    writing = want > 0 || look.delay() > 0 || look.fading();
    if (!writing)
        primed = 0;
    look.setDelay(primed >= want ? want : 0);
    if (want != latency.load(std::memory_order_relaxed))
    {
        latency.store(want, std::memory_order_relaxed);
        if (latencyKnown)
            latencyChanged.store(true, std::memory_order_relaxed);
    }
    latencyKnown = true;
    applied = s;
    configured = true;
}

float DynamicsProcessor::detect(Lane& l, float key, const DetectorMode mode, bool truePeak)
{
    float peakPower = 0.0f, rmsPower = 0.0f;
    if (mode != DetectorMode::Rms)
    {
        const float a = truePeak ? l.truePeak.process(key) : std::abs(key);
        const float held = l.peakWindow.push(a);
        peakPower = held * held;
    }
    if (mode != DetectorMode::Peak)
    {
        l.power1 = flush(l.power1 + d.rms * (key * key - l.power1));
        l.power2 = flush(l.power2 + d.rms * (l.power1 - l.power2));
        rmsPower = 2.0f * l.power2; // a sine of amplitude A reads A^2
    }
    switch (mode)
    {
    case DetectorMode::Peak:
        return peakPower;
    case DetectorMode::Rms:
        return rmsPower;
    default:
        return std::sqrt(peakPower * rmsPower);
    }
}

float DynamicsProcessor::Smooth2::step(float target, float coef)
{
    a = smoothTo(a, target, coef);
    b = smoothTo(b, a, coef);
    return b;
}

bool DynamicsProcessor::gainComputerSettled(const DynamicsSettings& s) const
{
    return gc.threshold.settled(s.thresholdDb) && gc.ratio.settled(s.ratio) && gc.knee.settled(s.kneeDb) &&
           gc.range.settled(s.rangeDb);
}

void DynamicsProcessor::advanceSidechain(const DynamicsSettings& s, int m, bool first, Plan& pl)
{
    const auto logHz = [](float hz) { return std::log(std::max(hz, 1.0f)); };
    float coef = 0.0f;
    bool coefKnown = false;
    const auto glide = [&](Smooth2& sm, float targetHz, float& designed, SvfCoefficients& c, float q, float& designedQ)
    {
        bool moving = false;
        const float tl = logHz(targetHz);
        if (first)
            sm.set(tl);
        float hz = targetHz;
        if (!sm.settled(tl))
        {
            if (!coefKnown)
            {
                coef = 1.0f - std::exp(-static_cast<float>(m) / (kFilterGlideMs * 0.001f * static_cast<float>(sr)));
                coefKnown = true;
            }
            sm.step(tl, coef);
            if (!sm.settled(tl))
            {
                hz = std::exp(sm.b);
                moving = true;
            }
        }
        if (first || hz != designed || q != designedQ)
        {
            c.design(sr, hz, q);
            designed = hz;
            designedQ = q;
        }
        return moving;
    };
    float unusedQ = 0.70710678f;
    // A filter that is gliding out of (or into) its 'off' setting keeps running until it has arrived.
    if (glide(scHpLog, s.scHighpassHz, designedHp, hpCoef, 0.70710678f, unusedQ))
        pl.highpass = true;
    unusedQ = 0.70710678f;
    if (glide(scLpLog, s.scLowpassHz, designedLp, lpCoef, 0.70710678f, unusedQ))
        pl.lowpass = true;
    glide(scBellLog, s.scBellHz, designedBell, bellCoef, s.scBellQ, designedBellQ);
}

void DynamicsProcessor::advanceGainComputer(const DynamicsSettings& s)
{
    gc.threshold.step(s.thresholdDb, smoothCoef);
    gc.ratio.step(s.ratio, ratioCoef);
    gc.knee.step(s.kneeDb, smoothCoef);
    gc.range.step(s.rangeDb, smoothCoef);
}

float DynamicsProcessor::targetAmount(Lane& l, float level, const DynamicsSettings& s) const
{
    const float range = std::abs(gc.range.b);
    float amount = 0.0f;
    switch (s.kind)
    {
    case DynamicsKind::Compressor:
        amount = dyn::compressorReduction(level, gc.threshold.b, gc.ratio.b, gc.knee.b);
        break;
    case DynamicsKind::Upward:
        amount = dyn::upwardLift(level, gc.threshold.b, gc.ratio.b, gc.knee.b);
        break;
    case DynamicsKind::Expander:
        amount = dyn::expanderReduction(level, gc.threshold.b, gc.ratio.b, gc.knee.b);
        break;
    case DynamicsKind::Gate:
    case DynamicsKind::Ducker:
    {
        const float open = gc.threshold.b, close = gc.threshold.b - std::max(0.0f, s.hysteresisDb);
        if (!l.triggered)
        {
            if (level > open)
            {
                l.triggered = true;
                l.triggerHold = d.holdSamples;
            }
        }
        else if (level >= close)
            l.triggerHold = d.holdSamples;
        else if (l.triggerHold > 0)
            --l.triggerHold;
        else
            l.triggered = false;

        const bool gate = s.kind == DynamicsKind::Gate;
        if (gate)
        {
            // Open: the curve with the lower threshold, and nothing at all while the hold runs. Closed: the curve with
            // the upper threshold, so the gain is continuous at both switching points.
            if (!l.triggered)
                amount = dyn::expanderReduction(level, open, gc.ratio.b, gc.knee.b);
            else
                amount = level >= close ? dyn::expanderReduction(level, close, gc.ratio.b, gc.knee.b) : 0.0f;
        }
        else
        {
            const float slope = std::max(gc.ratio.b, 1.0f) - 1.0f;
            if (!l.triggered)
                amount = dyn::kneeAmount(level - open, gc.knee.b, slope);
            else if (level >= close)
                amount = dyn::kneeAmount(level - close, gc.knee.b, slope);
            else
                amount = l.lastAmount;
            l.lastAmount = std::min(amount, range);
        }
        break;
    }
    }
    return std::min(amount, range);
}

float DynamicsProcessor::ballistics(Lane& l, float target, const DynamicsSettings& s) const
{
    const bool gateLike = s.kind == DynamicsKind::Expander || s.kind == DynamicsKind::Gate;
    if (gateLike)
    {
        // The amount is the attenuation: it falls while the gate opens (attack) and rises while it closes (release).
        l.reduction += (target < l.reduction ? d.attack : d.release) * (target - l.reduction);
        l.reduction = flush(l.reduction);
        return l.reduction;
    }
    if (s.kind == DynamicsKind::Upward && l.reduction > target)
    {
        // An upward compressor lifts what is below the threshold: a lift held from a quiet passage (hold, release) must
        // not carry on over a signal that is now loud, or the output would be the signal times the whole range.
        l.reduction = target;
        l.slow = std::min(l.slow, target);
    }
    if (target > l.reduction)
    {
        l.reduction += d.attack * (target - l.reduction);
        l.hold = d.holdSamples;
    }
    else if (l.hold > 0)
        --l.hold;
    else
        l.reduction += d.release * (target - l.reduction);
    l.reduction = flush(l.reduction);
    if (!s.autoRelease)
        return l.reduction;
    // Second stage: charges slowly while the signal is compressed, so a long passage releases slowly and a short burst
    // quickly. The larger of the two stages is applied.
    l.slow += (target > l.slow ? d.slowCharge : d.slowRelease) * (target - l.slow);
    l.slow = flush(l.slow);
    return std::max(l.reduction, l.slow);
}

void DynamicsProcessor::filterStage(float* k0, float* k1, int m, const Plan& pl)
{
    if (pl.highpass)
        for (int i = 0; i < m; ++i)
        {
            k0[i] = svfTick(lane[0].hp.ic1, lane[0].hp.ic2, k0[i], hpCoef.a1, hpCoef.a2, hpCoef.a3, hpCoef.k).hp;
            k1[i] = svfTick(lane[1].hp.ic1, lane[1].hp.ic2, k1[i], hpCoef.a1, hpCoef.a2, hpCoef.a3, hpCoef.k).hp;
        }
    if (pl.lowpass)
        for (int i = 0; i < m; ++i)
        {
            k0[i] = svfTick(lane[0].lp.ic1, lane[0].lp.ic2, k0[i], lpCoef.a1, lpCoef.a2, lpCoef.a3, lpCoef.k).lp;
            k1[i] = svfTick(lane[1].lp.ic1, lane[1].lp.ic2, k1[i], lpCoef.a1, lpCoef.a2, lpCoef.a3, lpCoef.k).lp;
        }
    if (pl.bell)
        for (int i = 0; i < m; ++i)
        {
            const float b0 =
                svfTick(lane[0].bell.ic1, lane[0].bell.ic2, k0[i], bellCoef.a1, bellCoef.a2, bellCoef.a3, bellCoef.k).bp;
            const float b1 =
                svfTick(lane[1].bell.ic1, lane[1].bell.ic2, k1[i], bellCoef.a1, bellCoef.a2, bellCoef.a3, bellCoef.k).bp;
            k0[i] += pl.bellGain * bellCoef.k * b0;
            k1[i] += pl.bellGain * bellCoef.k * b1;
        }
}

void DynamicsProcessor::detectStage(const float* k0, const float* k1, int m, float* p0, float* p1, const Plan& pl)
{
    const DetectorMode mode = pl.detector;
    const bool tp = pl.truePeak;
    if (pl.fullyLinked && mode == DetectorMode::Peak)
    {
        // Linked peak detection needs only the larger of the two lanes: one window over their maximum.
        for (int i = 0; i < m; ++i)
        {
            const float a0 = tp ? lane[0].truePeak.process(k0[i]) : std::abs(k0[i]);
            const float a1 = tp ? lane[1].truePeak.process(k1[i]) : std::abs(k1[i]);
            const float held = lane[0].peakWindow.push(std::max(a0, a1));
            p0[i] = p1[i] = held * held;
        }
        return;
    }
    if (mode == DetectorMode::Rms)
    {
        const float c = d.rms;
        float a1 = lane[0].power1, a2 = lane[0].power2, b1 = lane[1].power1, b2 = lane[1].power2;
        for (int i = 0; i < m; ++i)
        {
            a1 = flush(a1 + c * (k0[i] * k0[i] - a1));
            a2 = flush(a2 + c * (a1 - a2));
            b1 = flush(b1 + c * (k1[i] * k1[i] - b1));
            b2 = flush(b2 + c * (b1 - b2));
            p0[i] = 2.0f * a2;
            p1[i] = 2.0f * b2;
        }
        lane[0].power1 = a1;
        lane[0].power2 = a2;
        lane[1].power1 = b1;
        lane[1].power2 = b2;
        return;
    }
    for (int i = 0; i < m; ++i)
    {
        p0[i] = pl.active0 ? detect(lane[0], k0[i], mode, tp) : 0.0f;
        p1[i] = pl.active1 ? detect(lane[1], k1[i], mode, tp) : 0.0f;
    }
}

void DynamicsProcessor::gainStage(const float* p0, const float* p1, int m, float* g0, float* g1, float& maxAmount,
                                  const DynamicsSettings& s, const Plan& pl)
{
    const float sign = pl.sign;
    const bool moving = !gainComputerSettled(s);
    float peakAmount = maxAmount;
    if (pl.fullyLinked)
    {
        for (int i = 0; i < m; ++i)
        {
            if (moving)
                advanceGainComputer(s);
            const float level = dyn::powerToDb(std::max(p0[i], p1[i]));
            const float e = ballistics(lane[0], targetAmount(lane[0], level, s), s);
            peakAmount = std::max(peakAmount, e);
            g0[i] = g1[i] = dyn::dbToGain(sign * e);
        }
        // The second lane follows the first: same trigger and ballistics state, ready for the moment the link opens.
        lane[1].reduction = lane[0].reduction;
        lane[1].slow = lane[0].slow;
        lane[1].hold = lane[0].hold;
        lane[1].triggered = lane[0].triggered;
        lane[1].triggerHold = lane[0].triggerHold;
        lane[1].lastAmount = lane[0].lastAmount;
    }
    else
    {
        for (int i = 0; i < m; ++i)
        {
            if (moving)
                advanceGainComputer(s);
            float a = pl.active0 ? dyn::powerToDb(p0[i]) : -200.0f;
            float b = pl.active1 ? dyn::powerToDb(p1[i]) : -200.0f;
            if (pl.active0 && pl.active1)
                linkLevels(a, b, pl.link);
            float e0 = 0.0f, e1 = 0.0f;
            if (pl.active0)
                e0 = ballistics(lane[0], targetAmount(lane[0], a, s), s);
            if (pl.active1)
                e1 = ballistics(lane[1], targetAmount(lane[1], b, s), s);
            peakAmount = std::max(peakAmount, std::max(e0, e1));
            g0[i] = dyn::dbToGain(sign * e0);
            g1[i] = dyn::dbToGain(sign * e1);
        }
    }
    maxAmount = peakAmount;
}

void DynamicsProcessor::process(float* left, float* right, int n, const float* keyLeft, const float* keyRight,
                                const DynamicsSettings& s, bool externalKey)
{
    if (n <= 0)
        return;
    const bool first = !configured;
    configure(s);
    const bool ext = externalKey && keyLeft != nullptr;
    if (keyRight == nullptr)
        keyRight = keyLeft;
    const ChannelMode mode = s.channels;
    Plan pl;
    pl.active0 = laneActive(mode, 0);
    pl.active1 = laneActive(mode, 1);
    pl.link = effectiveLink(mode, s.link);
    pl.fullyLinked = pl.active0 && pl.active1 && pl.link >= 0.999f;
    pl.highpass = s.scHighpassHz > 20.5f;
    pl.lowpass = s.scLowpassHz > 0.0f && s.scLowpassHz < 19999.0f;
    pl.bell = std::abs(s.scBellDb) > 0.01f;
    pl.bellGain = dyn::dbToGain(s.scBellDb) - 1.0f;
    pl.sign = s.kind == DynamicsKind::Upward ? 1.0f : -1.0f;
    pl.detector = s.detector;
    pl.truePeak = s.truePeak;
    const bool feedback = s.feedback && !ext;
    const float cell = s.cellColour * 0.004f;

    float makeupDb = s.makeupDb;
    if (s.autoMakeup && s.kind == DynamicsKind::Compressor)
        makeupDb += std::min(dyn::compressorReduction(0.0f, s.thresholdDb, s.ratio, s.kneeDb), std::abs(s.rangeDb));
    const float makeupTarget = dyn::dbToGain(makeupDb);
    const float mixTarget = std::clamp(s.mix, 0.0f, 1.0f);
    if (first)
    {
        gc.threshold.set(s.thresholdDb);
        gc.ratio.set(s.ratio);
        gc.knee.set(s.kneeDb);
        gc.range.set(s.rangeDb);
        makeupSmooth.set(makeupTarget);
        mixSmooth.set(mixTarget);
        makeupLinear = makeupTarget;
        mixValue = mixTarget;
    }

    float maxAmount = 0.0f;
    float kl[kChunk], kr[kChunk], ol[kChunk], orr[kChunk], p0[kChunk], p1[kChunk], g0[kChunk], g1[kChunk];
    for (int pos = 0; pos < n; pos += kChunk)
    {
        const int m = std::min(kChunk, n - pos);
        float* l = left + pos;
        float* r = right + pos;
        if (ext)
        {
            std::copy(keyLeft + pos, keyLeft + pos + m, kl);
            std::copy(keyRight + pos, keyRight + pos + m, kr);
        }
        else
        {
            std::copy(l, l + m, kl);
            std::copy(r, r + m, kr);
        }
        advanceSidechain(s, m, first && pos == 0, pl);
        const bool gainMoving = !makeupSmooth.settled(makeupTarget) || !mixSmooth.settled(mixTarget);
        encode(mode, kl, kr, m);
        encode(mode, l, r, m);
        if (writing)
        {
            float* io[2] = {l, r};
            look.process(io, m);
        }

        if (feedback)
        {
            // The detector hears the output of the previous sample, so the stages run one sample at a time.
            for (int i = 0; i < m; ++i)
            {
                kl[i] = lane[0].feedbackSample;
                kr[i] = lane[1].feedbackSample;
                filterStage(kl + i, kr + i, 1, pl);
                ol[i] = kl[i];
                orr[i] = kr[i];
                detectStage(kl + i, kr + i, 1, p0 + i, p1 + i, pl);
                gainStage(p0 + i, p1 + i, 1, g0 + i, g1 + i, maxAmount, s, pl);
                lane[0].feedbackSample = l[i] * g0[i];
                lane[1].feedbackSample = r[i] * g1[i];
            }
        }
        else
        {
            filterStage(kl, kr, m, pl);
            std::copy(kl, kl + m, ol);
            std::copy(kr, kr + m, orr);
            detectStage(kl, kr, m, p0, p1, pl);
            gainStage(p0, p1, m, g0, g1, maxAmount, s, pl);
        }

        for (int i = 0; i < m; ++i)
        {
            if (gainMoving)
            {
                makeupLinear = makeupSmooth.step(makeupTarget, smoothCoef);
                mixValue = mixSmooth.step(mixTarget, smoothCoef);
            }
            const float v0 = l[i], v1 = r[i];
            float w0 = v0 * g0[i], w1 = v1 * g1[i];
            if (cell > 0.0f)
            {
                w0 -= cell * std::abs(1.0f - g0[i]) * w0 * w0 * w0;
                w1 -= cell * std::abs(1.0f - g1[i]) * w1 * w1 * w1;
            }
            w0 *= makeupLinear;
            w1 *= makeupLinear;
            l[i] = pl.active0 ? v0 + mixValue * (w0 - v0) : v0;
            r[i] = pl.active1 ? v1 + mixValue * (w1 - v1) : v1;
        }
        if (s.listen)
        {
            std::copy(ol, ol + m, l);
            std::copy(orr, orr + m, r);
        }
        decode(mode, l, r, m);
    }
    if (writing)
        primed = std::min(primed + n, 1 << 28);
    meter.store(s.kind == DynamicsKind::Upward ? -maxAmount : maxAmount, std::memory_order_relaxed);
    {
        const auto logHz = [](float hz) { return std::log(std::max(hz, 1.0f)); };
        settledAtEnd = gainComputerSettled(s) && makeupSmooth.settled(makeupTarget) && mixSmooth.settled(mixTarget) &&
                       scHpLog.settled(logHz(s.scHighpassHz)) && scLpLog.settled(logHz(s.scLowpassHz)) &&
                       scBellLog.settled(logHz(s.scBellHz));
    }
}

} // namespace crucible::fx
