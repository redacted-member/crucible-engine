// SPDX-License-Identifier: MIT
#include <crucible/FxExtra.h>

#include <crucible/Fft.h>
#include <crucible/Simd.h>

#include <algorithm>
#include <cmath>
#include <complex>

namespace crucible
{
namespace
{
constexpr double kPiD = 3.14159265358979323846;
constexpr float kTwoPiF = 6.28318530717958f;

float dbToGain(float db)
{
    return std::pow(10.0f, db / 20.0f);
}

float coefFor(double sr, float seconds)
{
    return 1.0f - std::exp(-1.0f / (std::max(seconds, 1.0e-5f) * static_cast<float>(sr)));
}

float toDb(float x)
{
    return 20.0f * std::log10(std::max(x, 1.0e-9f));
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
BiquadCoefs BiquadCoefs::design(BiquadType type, double freqHz, double gainDb, double q, double sampleRate)
{
    const double f = std::clamp(freqHz, 5.0, 0.49 * sampleRate);
    const double w0 = 2.0 * kPiD * f / sampleRate;
    const double cs = std::cos(w0);
    const double sn = std::sin(w0);
    const double Q = std::max(q, 0.05);
    const double A = std::pow(10.0, gainDb / 40.0);
    double b0 = 1, b1 = 0, b2 = 0, a0 = 1, a1 = 0, a2 = 0;
    switch (type)
    {
    case BiquadType::Peak:
    {
        const double alpha = sn / (2.0 * Q);
        b0 = 1.0 + alpha * A;
        b1 = -2.0 * cs;
        b2 = 1.0 - alpha * A;
        a0 = 1.0 + alpha / A;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha / A;
        break;
    }
    case BiquadType::LowShelf:
    case BiquadType::HighShelf:
    {
        // Q acts as the shelf slope control (0.707 = the classic maximally flat shelf).
        const double alpha = sn / (2.0 * Q);
        const double sa = 2.0 * std::sqrt(A) * alpha;
        if (type == BiquadType::LowShelf)
        {
            b0 = A * ((A + 1.0) - (A - 1.0) * cs + sa);
            b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cs);
            b2 = A * ((A + 1.0) - (A - 1.0) * cs - sa);
            a0 = (A + 1.0) + (A - 1.0) * cs + sa;
            a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cs);
            a2 = (A + 1.0) + (A - 1.0) * cs - sa;
        }
        else
        {
            b0 = A * ((A + 1.0) + (A - 1.0) * cs + sa);
            b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cs);
            b2 = A * ((A + 1.0) + (A - 1.0) * cs - sa);
            a0 = (A + 1.0) - (A - 1.0) * cs + sa;
            a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cs);
            a2 = (A + 1.0) - (A - 1.0) * cs - sa;
        }
        break;
    }
    case BiquadType::LowCut12:
    case BiquadType::LowCut24:
    {
        const double alpha = sn / (2.0 * Q);
        b0 = (1.0 + cs) * 0.5;
        b1 = -(1.0 + cs);
        b2 = (1.0 + cs) * 0.5;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    }
    case BiquadType::HighCut12:
    case BiquadType::HighCut24:
    {
        const double alpha = sn / (2.0 * Q);
        b0 = (1.0 - cs) * 0.5;
        b1 = 1.0 - cs;
        b2 = (1.0 - cs) * 0.5;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    }
    case BiquadType::Notch:
    {
        const double alpha = sn / (2.0 * Q);
        b0 = 1.0;
        b1 = -2.0 * cs;
        b2 = 1.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    }
    case BiquadType::BandPass:
    {
        // Constant 0 dB peak gain.
        const double alpha = sn / (2.0 * Q);
        b0 = alpha;
        b1 = 0.0;
        b2 = -alpha;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    }
    case BiquadType::Count:
        break;
    }
    return {b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0};
}

double BiquadCoefs::magnitudeDb(double freqHz, double sampleRate) const
{
    const double w = 2.0 * kPiD * std::clamp(freqHz, 0.0, 0.5 * sampleRate) / sampleRate;
    const std::complex<double> z1 = std::polar(1.0, -w);
    const std::complex<double> z2 = z1 * z1;
    const auto num = b0 + b1 * z1 + b2 * z2;
    const auto den = 1.0 + a1 * z1 + a2 * z2;
    return 20.0 * std::log10(std::max(std::abs(num / den), 1.0e-12));
}

// ---------------------------------------------------------------------------------------------------------------
void FxFilter::prepare(double sampleRate)
{
    sr = sampleRate;
    unit.prepare(sampleRate, Quality::Standard);
    reset();
}

void FxFilter::reset()
{
    primed = false;
    unit.reset();
    phase = 0.0f;
}

void FxFilter::process(float* l, float* r, int n, const FxFilterParams& p)
{
    if (!primed)
    {
        mixSmooth = p.mix;
        cutoffSmooth = p.cutoffHz;
        unit.reset();
        primed = true;
    }
    const float rate = p.lfoSyncBeats > 0.0f ? static_cast<float>(p.bpm / 60.0) / p.lfoSyncBeats : p.lfoRateHz;
    const float mc = coefFor(sr, 0.01f);
    constexpr int chunk = 16;
    const float cc = coefFor(sr / chunk, 0.015f);
    for (int start = 0; start < n; start += chunk)
    {
        const int m = std::min(chunk, n - start);
        cutoffSmooth =
            std::exp2(std::log2(cutoffSmooth) + (std::log2(std::max(p.cutoffHz, 20.0f)) - std::log2(cutoffSmooth)) * cc);
        const float lfo = std::sin(kTwoPiF * phase);
        const float hz = std::clamp(cutoffSmooth * std::exp2(std::clamp(p.lfoDepth, 0.0f, 4.0f) * lfo), 20.0f,
                                    0.45f * static_cast<float>(sr));
        FilterSettings s;
        s.model = std::clamp(p.model, 0, static_cast<int>(FilterModel::Count) - 1);
        s.cutoffHz = hz;
        s.resonance = std::clamp(p.resonance, 0.0f, 1.0f);
        unit.configure(s, std::clamp(p.drive, 0.0f, 1.0f), sr);
        for (int i = start; i < start + m; ++i)
        {
            mixSmooth += (p.mix - mixSmooth) * mc;
            float a = l[i], b = r[i];
            unit.process(a, b);
            l[i] += (a - l[i]) * mixSmooth;
            r[i] += (b - r[i]) * mixSmooth;
        }
        phase += rate * static_cast<float>(m) / static_cast<float>(sr);
        phase -= std::floor(phase);
    }
}

// ---------------------------------------------------------------------------------------------------------------
void FxMultiband::prepare(double sampleRate)
{
    sr = sampleRate;
    reset();
}

void FxMultiband::reset()
{
    primed = false;
    for (auto* set : {&lowLp, &lowHp, &highLp, &highHp, &lowAllpassLp, &lowAllpassHp})
        for (auto& b : *set)
            b.reset();
    envDb = {};
    for (auto& g : gr)
        g.store(0.0f);
    lastLow = lastHigh = -1.0f;
    rampLeft = 0;
    rampMoves = {};
}

void FxMultiband::applyLow(const BiquadCoefs& lp, const BiquadCoefs& hp)
{
    for (auto& b : lowLp)
        b.set(lp);
    for (auto& b : lowHp)
        b.set(hp);
}

void FxMultiband::applyHigh(const BiquadCoefs& lp, const BiquadCoefs& hp)
{
    for (auto& b : highLp)
        b.set(lp);
    for (auto& b : highHp)
        b.set(hp);
    for (auto& b : lowAllpassLp)
        b.set(lp);
    for (auto& b : lowAllpassHp)
        b.set(hp);
}

// Designs the filters for `hz` at once and drops any ramp towards an older design of the same crossover.
void FxMultiband::designLow(float hz)
{
    curLp[0] = BiquadCoefs::design(BiquadType::HighCut12, hz, 0.0, 0.70710678, sr);
    curHp[0] = BiquadCoefs::design(BiquadType::LowCut12, hz, 0.0, 0.70710678, sr);
    applyLow(curLp[0], curHp[0]);
    goalLp[0] = curLp[0];
    goalHp[0] = curHp[0];
    stepLp[0] = stepHp[0] = BiquadCoefs{0.0, 0.0, 0.0, 0.0, 0.0};
    lastLow = hz;
}

void FxMultiband::designHigh(float hz)
{
    curLp[1] = BiquadCoefs::design(BiquadType::HighCut12, hz, 0.0, 0.70710678, sr);
    curHp[1] = BiquadCoefs::design(BiquadType::LowCut12, hz, 0.0, 0.70710678, sr);
    applyHigh(curLp[1], curHp[1]);
    goalLp[1] = curLp[1];
    goalHp[1] = curHp[1];
    stepLp[1] = stepHp[1] = BiquadCoefs{0.0, 0.0, 0.0, 0.0, 0.0};
    lastHigh = hz;
}

// Starts a ramp of kRamp samples from the coefficients in use to the design for the current crossover frequencies.
void FxMultiband::startRamp()
{
    const auto toward = [](BiquadCoefs& cur, BiquadCoefs& step, BiquadCoefs& goal, const BiquadCoefs& to)
    {
        goal = to;
        const double k = 1.0 / kRamp;
        step = BiquadCoefs{(to.b0 - cur.b0) * k, (to.b1 - cur.b1) * k, (to.b2 - cur.b2) * k, (to.a1 - cur.a1) * k,
                           (to.a2 - cur.a2) * k};
    };
    rampMoves = {smoothLow != lastLow, smoothHigh != lastHigh};
    if (smoothLow != lastLow)
    {
        toward(curLp[0], stepLp[0], goalLp[0], BiquadCoefs::design(BiquadType::HighCut12, smoothLow, 0.0, 0.70710678, sr));
        toward(curHp[0], stepHp[0], goalHp[0], BiquadCoefs::design(BiquadType::LowCut12, smoothLow, 0.0, 0.70710678, sr));
        lastLow = smoothLow;
    }
    if (smoothHigh != lastHigh)
    {
        toward(curLp[1], stepLp[1], goalLp[1], BiquadCoefs::design(BiquadType::HighCut12, smoothHigh, 0.0, 0.70710678, sr));
        toward(curHp[1], stepHp[1], goalHp[1], BiquadCoefs::design(BiquadType::LowCut12, smoothHigh, 0.0, 0.70710678, sr));
        lastHigh = smoothHigh;
    }
    rampLeft = kRamp;
}

// One sample of the ramp; the last one lands exactly on the design. Only the crossover that is moving is touched.
void FxMultiband::stepRamp()
{
    --rampLeft;
    for (size_t k = 0; k < 2; ++k)
    {
        if (!rampMoves[k])
            continue;
        if (rampLeft == 0)
        {
            curLp[k] = goalLp[k];
            curHp[k] = goalHp[k];
        }
        else
        {
            curLp[k] = BiquadCoefs{curLp[k].b0 + stepLp[k].b0, curLp[k].b1 + stepLp[k].b1, curLp[k].b2 + stepLp[k].b2,
                                   curLp[k].a1 + stepLp[k].a1, curLp[k].a2 + stepLp[k].a2};
            curHp[k] = BiquadCoefs{curHp[k].b0 + stepHp[k].b0, curHp[k].b1 + stepHp[k].b1, curHp[k].b2 + stepHp[k].b2,
                                   curHp[k].a1 + stepHp[k].a1, curHp[k].a2 + stepHp[k].a2};
        }
    }
    if (rampMoves[0])
        applyLow(curLp[0], curHp[0]);
    if (rampMoves[1])
        applyHigh(curLp[1], curHp[1]);
}

void FxMultiband::process(float* l, float* r, int n, const FxMultibandParams& p)
{
    if (!primed)
    {
        mixSmooth = p.mix;
        primed = true;
    }
    const float lo = std::clamp(p.lowCrossHz, 30.0f, 2000.0f);
    const float hi = std::clamp(std::max(p.highCrossHz, lo * 1.5f), 300.0f, 16000.0f);
    if (lastLow < 0.0f)
    {
        stageLow = smoothLow = lo;
        stageHigh = smoothHigh = hi;
        designLow(lo);
        designHigh(hi);
    }
    // The crossovers glide towards their targets (two one-pole stages of 12 ms), so automating them leaves no step in
    // the output (BFX6). The glide runs every sample, but the filters are only redesigned every kRamp samples: their
    // coefficients move in a straight line to each new design (startRamp / stepRamp), and the last ramp lands exactly
    // on the design for the target frequency.
    const float glide = coefFor(sr, 0.012f);
    const float att = coefFor(sr, std::max(p.attackMs, 0.05f) * 0.001f);
    const float rel = coefFor(sr, std::max(p.releaseMs, 1.0f) * 0.001f);
    const float slope = 1.0f - 1.0f / std::max(p.ratio, 1.0f);
    std::array<float, 3> makeup{};
    for (size_t b = 0; b < 3; ++b)
        makeup[b] = p.gainDb[b];
    const float mc = coefFor(sr, 0.01f);
    std::array<float, 3> maxGr{};
    for (int i = 0; i < n; ++i)
    {
        mixSmooth += (p.mix - mixSmooth) * mc;
        if (smoothLow != lo || smoothHigh != hi)
        {
            stageLow += (lo - stageLow) * glide;
            stageHigh += (hi - stageHigh) * glide;
            smoothLow += (stageLow - smoothLow) * glide;
            smoothHigh += (stageHigh - smoothHigh) * glide;
            if (std::abs(lo - smoothLow) < 1.0e-4f * lo && std::abs(lo - stageLow) < 1.0e-3f * lo)
                stageLow = smoothLow = lo;
            if (std::abs(hi - smoothHigh) < 1.0e-4f * hi && std::abs(hi - stageHigh) < 1.0e-3f * hi)
                stageHigh = smoothHigh = hi;
        }
        if (rampLeft == 0 && (smoothLow != lastLow || smoothHigh != lastHigh))
            startRamp();
        if (rampLeft > 0)
            stepRamp();
        std::array<std::array<float, 3>, 2> band{};
        float* ch[2] = {l, r};
        for (int c = 0; c < 2; ++c)
        {
            const float x = ch[c][i];
            const float lowRaw = lowLp[1].process(c, lowLp[0].process(c, x));
            const float low = lowAllpassLp[1].process(c, lowAllpassLp[0].process(c, lowRaw)) +
                              lowAllpassHp[1].process(c, lowAllpassHp[0].process(c, lowRaw));
            const float rest = lowHp[1].process(c, lowHp[0].process(c, x));
            const float mid = highLp[1].process(c, highLp[0].process(c, rest));
            const float high = highHp[1].process(c, highHp[0].process(c, rest));
            band[static_cast<size_t>(c)] = {low, mid, high};
        }
        for (size_t b = 0; b < 3; ++b)
        {
            const float level = toDb(std::max(std::abs(band[0][b]), std::abs(band[1][b])));
            const float over = std::max(level - p.thresholdDb[b], 0.0f);
            const float target = over * slope;
            envDb[b] += (target - envDb[b]) * (target > envDb[b] ? att : rel);
            maxGr[b] = std::max(maxGr[b], envDb[b]);
            const float gain = dbToGain(makeup[b] - envDb[b]);
            band[0][b] *= gain;
            band[1][b] *= gain;
        }
        for (int c = 0; c < 2; ++c)
        {
            const float wet =
                band[static_cast<size_t>(c)][0] + band[static_cast<size_t>(c)][1] + band[static_cast<size_t>(c)][2];
            ch[c][i] += (wet - ch[c][i]) * mixSmooth;
        }
    }
    for (size_t b = 0; b < 3; ++b)
        gr[b].store(maxGr[b], std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------------------------------------------
const std::vector<std::string>& tranceGateRateNames()
{
    static const std::vector<std::string> names = {"1/4", "1/8", "1/16", "1/32", "1/8T", "1/16T", "1/8.", "1/16."};
    return names;
}

float tranceGateRateBeats(int index)
{
    constexpr float table[] = {1.0f, 0.5f, 0.25f, 0.125f, 1.0f / 3.0f, 1.0f / 6.0f, 0.75f, 0.375f};
    return table[std::clamp(index, 0, 7)];
}

void FxTranceGate::prepare(double sampleRate)
{
    sr = sampleRate;
    reset();
}

void FxTranceGate::reset()
{
    primed = false;
    beat = 0.0;
    level = 1.0f;
    step.store(0);
}

void FxTranceGate::process(float* l, float* r, int n, const FxTranceGateParams& p)
{
    if (!primed)
    {
        mixSmooth = p.mix;
        beat = 0.0;
        primed = true;
    }
    const int steps = std::clamp(p.steps, 1, 16);
    const double stepBeats = std::max(static_cast<double>(p.stepBeats), 1.0 / 64.0);
    const double beatsPerSample = std::clamp(p.bpm, 20.0, 400.0) / 60.0 / sr;
    const float att = coefFor(sr, std::max(p.attackMs, 0.1f) * 0.001f);
    const float rel = coefFor(sr, std::max(p.releaseMs, 0.1f) * 0.001f);
    const float floorLevel = 1.0f - std::clamp(p.depth, 0.0f, 1.0f);
    const float mc = coefFor(sr, 0.01f);
    int s = 0;
    for (int i = 0; i < n; ++i)
    {
        const double pos = beat / stepBeats;
        s = static_cast<int>(std::fmod(std::floor(pos), static_cast<double>(steps)));
        const double within = pos - std::floor(pos);
        // A step is open for 90 % of its length so consecutive on steps still pulse.
        const bool onStep = ((p.pattern >> static_cast<unsigned>(s)) & 1U) != 0U && within < 0.9;
        const float target = onStep ? 1.0f : floorLevel;
        level += (target - level) * (target > level ? att : rel);
        mixSmooth += (p.mix - mixSmooth) * mc;
        const float g = 1.0f + (level - 1.0f) * mixSmooth;
        l[i] *= g;
        r[i] *= g;
        beat += beatsPerSample;
    }
    const double cycle = stepBeats * steps;
    if (beat > cycle * 64.0)
        beat = std::fmod(beat, cycle);
    step.store(s, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------------------------------------------
const std::vector<std::string>& tremoloShapeNames()
{
    static const std::vector<std::string> names = {"Sine", "Triangle", "Square", "Saw Up", "Saw Down"};
    return names;
}

void FxTremolo::prepare(double sampleRate)
{
    sr = sampleRate;
    reset();
}

void FxTremolo::reset()
{
    phase = 0.0f;
    smooth = {1.0f, 1.0f};
}

void FxTremolo::process(float* l, float* r, int n, const FxTremoloParams& p)
{
    const float rate = p.syncBeats > 0.0f ? static_cast<float>(p.bpm / 60.0) / p.syncBeats : p.rateHz;
    const float inc = std::clamp(rate, 0.01f, 40.0f) / static_cast<float>(sr);
    const float depth = std::clamp(p.depth, 0.0f, 1.0f);
    const float offset = std::clamp(p.stereoPhaseDeg, 0.0f, 360.0f) / 360.0f;
    // Edge smoothing keeps the square shape click free (about 2 ms).
    const float sc = coefFor(sr, 0.002f);
    const auto shape = [&](float ph)
    {
        ph -= std::floor(ph);
        switch (p.shape)
        {
        case 1:
            return 1.0f - 4.0f * std::abs(ph - 0.5f);
        case 2:
            return ph < 0.5f ? 1.0f : -1.0f;
        case 3:
            return 2.0f * ph - 1.0f;
        case 4:
            return 1.0f - 2.0f * ph;
        default:
            return std::sin(kTwoPiF * ph);
        }
    };
    float* ch[2] = {l, r};
    for (int i = 0; i < n; ++i)
    {
        for (size_t c = 0; c < 2; ++c)
        {
            const float lfo = shape(phase + (c == 1 ? offset : 0.0f)); // -1..1
            const float gain = 1.0f - depth * 0.5f * (1.0f - lfo);     // 1 at the top, 1 - depth at the bottom
            smooth[c] += (gain - smooth[c]) * sc;
            ch[c][i] *= smooth[c];
        }
        phase += inc;
        phase -= std::floor(phase);
    }
}

// ---------------------------------------------------------------------------------------------------------------
void FxRingShift::Hilbert::reset()
{
    ax1 = ax2 = ay1 = ay2 = bx1 = bx2 = by1 = by2 = {};
    bDelay = 0.0;
}

void FxRingShift::Hilbert::process(double x, double& re, double& im, const HilbertCoefs& k)
{
    double a = x;
    for (int s = 0; s < k.sections; ++s)
    {
        const auto i = static_cast<size_t>(s);
        const double y = k.a[i] * (a + ay2[i]) - ax2[i];
        ax2[i] = ax1[i];
        ax1[i] = a;
        ay2[i] = ay1[i];
        ay1[i] = y;
        a = y;
    }
    double b = x;
    for (int s = 0; s < k.sections; ++s)
    {
        const auto i = static_cast<size_t>(s);
        const double y = k.b[i] * (b + by2[i]) - bx2[i];
        bx2[i] = bx1[i];
        bx1[i] = b;
        by2[i] = by1[i];
        by1[i] = y;
        b = y;
    }
    // Path A is delayed by one sample so the two outputs sit 90 degrees apart.
    re = bDelay;
    bDelay = a;
    im = b;
}

namespace
{
// Phase error (radians) of the Hilbert pair at the frequencies `hz`: the angle by which path B over delayed path A
// misses +90 degrees. logHz holds the natural log of the sections' pole frequencies in Hz, alternating path A,
// path B, A, B ...
void hilbertPhaseError(const double* logHz, int count, double rate, const std::vector<double>& hz, std::vector<double>& out)
{
    std::array<double, 12> c{};
    for (int k = 0; k < count; ++k)
    {
        const double t = std::tan(2.0 * kPiD * std::exp(logHz[k]) / rate);
        c[static_cast<size_t>(k)] = (1.0 - t) / (1.0 + t);
    }
    for (size_t i = 0; i < hz.size(); ++i)
    {
        const double w = 2.0 * kPiD * hz[i] / rate;
        const std::complex<double> z1 = std::polar(1.0, -w);
        const std::complex<double> z2 = z1 * z1;
        std::complex<double> ha = 1.0, hb = 1.0;
        for (int k = 0; k < count; ++k)
        {
            const double ck = c[static_cast<size_t>(k)];
            const std::complex<double> section = (ck - z2) / (1.0 - ck * z2);
            (k % 2 == 0 ? ha : hb) *= section;
        }
        out[i] = std::arg(hb / (z1 * ha) * std::complex<double>(0.0, -1.0));
    }
}

// Solves the n x n system m x = rhs in place (partial pivoting); false when singular.
bool solveSmall(std::array<std::array<double, 12>, 12>& m, std::array<double, 12>& rhs, int n)
{
    for (int col = 0; col < n; ++col)
    {
        int best = col;
        for (int row = col + 1; row < n; ++row)
            if (std::abs(m[static_cast<size_t>(row)][static_cast<size_t>(col)]) >
                std::abs(m[static_cast<size_t>(best)][static_cast<size_t>(col)]))
                best = row;
        if (std::abs(m[static_cast<size_t>(best)][static_cast<size_t>(col)]) < 1.0e-300)
            return false;
        std::swap(m[static_cast<size_t>(best)], m[static_cast<size_t>(col)]);
        std::swap(rhs[static_cast<size_t>(best)], rhs[static_cast<size_t>(col)]);
        for (int row = col + 1; row < n; ++row)
        {
            const double f = m[static_cast<size_t>(row)][static_cast<size_t>(col)] /
                             m[static_cast<size_t>(col)][static_cast<size_t>(col)];
            for (int k = col; k < n; ++k)
                m[static_cast<size_t>(row)][static_cast<size_t>(k)] -=
                    f * m[static_cast<size_t>(col)][static_cast<size_t>(k)];
            rhs[static_cast<size_t>(row)] -= f * rhs[static_cast<size_t>(col)];
        }
    }
    for (int row = n - 1; row >= 0; --row)
    {
        double v = rhs[static_cast<size_t>(row)];
        for (int k = row + 1; k < n; ++k)
            v -= m[static_cast<size_t>(row)][static_cast<size_t>(k)] * rhs[static_cast<size_t>(k)];
        rhs[static_cast<size_t>(row)] = v / m[static_cast<size_t>(row)][static_cast<size_t>(row)];
    }
    return true;
}
} // namespace

FxRingShift::HilbertCoefs FxRingShift::designHilbert(double sampleRate)
{
    HilbertCoefs k;
    if (sampleRate == 48000.0)
    {
        // The classic set (Niemitalo): 90 degrees within +-0.7 degrees from about 20 Hz to 24 kHz at 48 kHz.
        constexpr std::array<double, 4> ca = {0.6923878, 0.9360654322959, 0.9882295226860, 0.9987488452737};
        constexpr std::array<double, 4> cb = {0.4021921162426, 0.8561710882420, 0.9722909545651, 0.9952884791278};
        k.sections = 4;
        for (size_t i = 0; i < 4; ++i)
        {
            k.a[i] = ca[i] * ca[i];
            k.b[i] = cb[i] * cb[i];
        }
        return k;
    }

    // Other rates: the pole frequencies of the classic set, which put the lower edge at about 20 Hz, extended upwards
    // by the same spacing where the band is wider than at 48 kHz, then refined by a damped Gauss-Newton fit of the
    // phase difference to 90 degrees on a logarithmic grid from 20 Hz to min(20 kHz, 0.45 rate) with reweighting
    // that approaches an equal-ripple (minimax) solution. Runs once per prepare() on the control thread.
    const int poles = sampleRate <= 200000.0 ? 10 : 12;
    k.sections = poles / 2;
    constexpr std::array<double, 8> classicHz = {9.56, 36.08, 90.44, 214.56, 503.27, 1167.67, 2584.92, 4774.87};
    std::array<double, 12> x{};
    {
        const double spacing = 2.3;
        std::array<double, 12> f{};
        for (int i = 0; i < poles; ++i)
            f[static_cast<size_t>(i)] = i < 8 ? classicHz[static_cast<size_t>(i)] : f[static_cast<size_t>(i - 1)] * spacing;
        const double top = 0.45 * std::min(20000.0, 0.45 * sampleRate);
        const double stretch = std::log(top / f[0]) / std::log(f[static_cast<size_t>(poles - 1)] / f[0]);
        for (int i = 0; i < poles; ++i)
            x[static_cast<size_t>(i)] = std::log(f[0]) + stretch * std::log(f[static_cast<size_t>(i)] / f[0]);
    }
    const double maxLog = std::log(0.12 * sampleRate);
    constexpr int gridPoints = 160;
    std::vector<double> grid(gridPoints), r(gridPoints), rTrial(gridPoints), w(gridPoints, 1.0);
    const double lo = 20.0, hi = std::min(20000.0, 0.45 * sampleRate);
    for (int i = 0; i < gridPoints; ++i)
        grid[static_cast<size_t>(i)] = lo * std::pow(hi / lo, static_cast<double>(i) / (gridPoints - 1));
    const auto cost = [&](const std::vector<double>& res)
    {
        double s = 0.0;
        for (int i = 0; i < gridPoints; ++i)
            s += w[static_cast<size_t>(i)] * res[static_cast<size_t>(i)] * res[static_cast<size_t>(i)];
        return s;
    };
    double lambda = 1.0e-3;
    std::vector<std::array<double, 12>> jac(gridPoints);
    for (int iter = 0; iter < 60; ++iter)
    {
        hilbertPhaseError(x.data(), poles, sampleRate, grid, r);
        for (int j = 0; j < poles; ++j)
        {
            auto xp = x;
            xp[static_cast<size_t>(j)] += 1.0e-6;
            hilbertPhaseError(xp.data(), poles, sampleRate, grid, rTrial);
            for (int i = 0; i < gridPoints; ++i)
                jac[static_cast<size_t>(i)][static_cast<size_t>(j)] =
                    (rTrial[static_cast<size_t>(i)] - r[static_cast<size_t>(i)]) * 1.0e6;
        }
        std::array<std::array<double, 12>, 12> h{};
        std::array<double, 12> g{};
        for (int i = 0; i < gridPoints; ++i)
            for (int a = 0; a < poles; ++a)
            {
                const double wj = w[static_cast<size_t>(i)] * jac[static_cast<size_t>(i)][static_cast<size_t>(a)];
                g[static_cast<size_t>(a)] += wj * r[static_cast<size_t>(i)];
                for (int b = 0; b < poles; ++b)
                    h[static_cast<size_t>(a)][static_cast<size_t>(b)] +=
                        wj * jac[static_cast<size_t>(i)][static_cast<size_t>(b)];
            }
        const double before = cost(r);
        for (int attempt = 0; attempt < 8; ++attempt)
        {
            auto m = h;
            std::array<double, 12> step{};
            for (int a = 0; a < poles; ++a)
            {
                m[static_cast<size_t>(a)][static_cast<size_t>(a)] +=
                    lambda * (h[static_cast<size_t>(a)][static_cast<size_t>(a)] + 1.0e-12);
                step[static_cast<size_t>(a)] = -g[static_cast<size_t>(a)];
            }
            if (!solveSmall(m, step, poles))
            {
                lambda *= 4.0;
                continue;
            }
            auto trial = x;
            for (int a = 0; a < poles; ++a)
                trial[static_cast<size_t>(a)] =
                    std::min(trial[static_cast<size_t>(a)] + step[static_cast<size_t>(a)], maxLog);
            hilbertPhaseError(trial.data(), poles, sampleRate, grid, rTrial);
            const double after = cost(rTrial);
            if (std::isfinite(after) && after < before)
            {
                x = trial;
                lambda = std::max(lambda / 3.0, 1.0e-9);
                break;
            }
            lambda *= 4.0;
        }
        if (iter % 4 == 3)
        {
            // Lawson reweighting: grid points with the largest error count more, which flattens the ripple.
            hilbertPhaseError(x.data(), poles, sampleRate, grid, r);
            double sum = 0.0;
            for (int i = 0; i < gridPoints; ++i)
            {
                w[static_cast<size_t>(i)] *= std::max(std::abs(r[static_cast<size_t>(i)]), 1.0e-6);
                sum += w[static_cast<size_t>(i)];
            }
            for (auto& v : w)
                v *= gridPoints / sum;
        }
    }
    for (int i = 0; i < poles; ++i)
    {
        const double t = std::tan(2.0 * kPiD * std::exp(x[static_cast<size_t>(i)]) / sampleRate);
        const double c = std::clamp((1.0 - t) / (1.0 + t), 1.0e-6, 0.999999);
        (i % 2 == 0 ? k.a : k.b)[static_cast<size_t>(i / 2)] = c;
    }
    return k;
}

void FxRingShift::prepare(double sampleRate)
{
    sr = sampleRate;
    coefs = designHilbert(sampleRate);
    reset();
}

void FxRingShift::reset()
{
    primed = false;
    for (auto& h : hilbert)
        h.reset();
    phase = {};
}

void FxRingShift::process(float* l, float* r, int n, const FxRingShiftParams& p)
{
    if (!primed)
    {
        mixSmooth = p.mix;
        primed = true;
    }
    const float mc = coefFor(sr, 0.01f);
    const double spread = std::clamp(p.spread, 0.0f, 1.0f) * 0.25;
    const double f = std::clamp(static_cast<double>(p.freqHz), 0.0, 0.45 * sr);
    const std::array<double, 2> inc = {2.0 * kPiD * f * (1.0 - spread) / sr, 2.0 * kPiD * f * (1.0 + spread) / sr};
    float* ch[2] = {l, r};
    for (int i = 0; i < n; ++i)
    {
        mixSmooth += (p.mix - mixSmooth) * mc;
        for (size_t c = 0; c < 2; ++c)
        {
            const double x = ch[c][i];
            double wet;
            if (p.mode == 0)
            {
                wet = x * std::sin(phase[c]);
            }
            else
            {
                double re, im;
                hilbert[c].process(x, re, im, coefs);
                const double cs = std::cos(phase[c]), sn = std::sin(phase[c]);
                wet = p.mode == 1 ? re * cs + im * sn : re * cs - im * sn;
            }
            ch[c][i] += (static_cast<float>(wet) - ch[c][i]) * mixSmooth;
            phase[c] += inc[c];
            if (phase[c] > 2.0 * kPiD)
                phase[c] -= 2.0 * kPiD;
        }
    }
}

} // namespace crucible
