// SPDX-License-Identifier: MIT
#include "LadderBase.h"

#include <crucible/Oversampler.h>

#include <algorithm>
#include <cmath>
#include <mutex>

namespace crucible::filters
{
namespace
{
constexpr float kPi = 3.14159265358979f;
constexpr float kNoiseAmplitude = 1.0e-6f; // -120 dBFS
constexpr float kDriveMax = 5.0f;          // shaper amount a at drive 1

// Coefficients of the all-pass half-band pairs (the same designs as Oversampler's Polyphase filter: 20 kHz pass band,
// 28 kHz stop band at a 96 kHz stage rate), Standard and High. Draft runs at the host rate.
constexpr std::array<float, 6> kIir6 = {0.04413809392052576f, 0.16397135710502875f, 0.3302904125194603f,
                                        0.5145431356697716f,  0.7016255182888512f,  0.8947170769567319f};
constexpr std::array<float, 8> kIir8 = {0.02267514594189869f, 0.08896622732883862f, 0.1924908356302284f,
                                        0.32192921314336626f, 0.4646410633359093f,  0.6115595439031657f,
                                        0.7601938839594213f,  0.9157169952072117f};
} // namespace

template <class V, int Coefficients> void LadderBase::upStage(HalfBand& st, const float* in, int n, float* out)
{
    constexpr int stride = nl::vstride<V>;
    V x[Coefficients], y[Coefficients], a[Coefficients];
    for (int k = 0; k < Coefficients; ++k)
    {
        x[k] = nl::load<V>(st.x[k]);
        y[k] = nl::load<V>(st.y[k]);
        a[k] = nl::vconst<V>(coefficient[static_cast<size_t>(k)]);
    }
    for (int i = 0; i < n; ++i)
    {
        const V v = nl::load<V>(in + i * stride);
        V even = v, odd = v;
        for (int k = 0; k < Coefficients; k += 2)
        {
            const V o = a[k] * (even - y[k]) + x[k];
            x[k] = even;
            y[k] = o;
            even = o;
        }
        for (int k = 1; k < Coefficients; k += 2)
        {
            const V o = a[k] * (odd - y[k]) + x[k];
            x[k] = odd;
            y[k] = o;
            odd = o;
        }
        nl::store(out + 2 * i * stride, even);
        nl::store(out + (2 * i + 1) * stride, odd);
    }
    for (int k = 0; k < Coefficients; ++k)
    {
        nl::store(st.x[k], x[k]);
        nl::store(st.y[k], y[k]);
    }
}

template <class V, int Coefficients> void LadderBase::downStage(HalfBand& st, const float* in, int n, float* out)
{
    constexpr int stride = nl::vstride<V>;
    V x[Coefficients], y[Coefficients], a[Coefficients];
    for (int k = 0; k < Coefficients; ++k)
    {
        x[k] = nl::load<V>(st.x[k]);
        y[k] = nl::load<V>(st.y[k]);
        a[k] = nl::vconst<V>(coefficient[static_cast<size_t>(k)]);
    }
    V prev = nl::load<V>(st.prev);
    const V half = nl::vconst<V>(0.5f);
    for (int i = 0; i < n; ++i)
    {
        V even = nl::load<V>(in + 2 * i * stride);
        V odd = prev;
        prev = nl::load<V>(in + (2 * i + 1) * stride);
        for (int k = 0; k < Coefficients; k += 2)
        {
            const V o = a[k] * (even - y[k]) + x[k];
            x[k] = even;
            y[k] = o;
            even = o;
        }
        for (int k = 1; k < Coefficients; k += 2)
        {
            const V o = a[k] * (odd - y[k]) + x[k];
            x[k] = odd;
            y[k] = o;
            odd = o;
        }
        nl::store(out + i * stride, half * (even + odd));
    }
    for (int k = 0; k < Coefficients; ++k)
    {
        nl::store(st.x[k], x[k]);
        nl::store(st.y[k], y[k]);
    }
    nl::store(st.prev, prev);
}

// One stage between two rates: n samples in, 2n out (or back).
template <class V> void LadderBase::upAt(size_t stage, const float* in, int n, float* out)
{
    if (coefficients == 6)
        upStage<V, 6>(upState[stage], in, n, out);
    else
        upStage<V, 8>(upState[stage], in, n, out);
}

template <class V> void LadderBase::downAt(size_t stage, const float* in, int n, float* out)
{
    if (coefficients == 6)
        downStage<V, 6>(downState[stage], in, n, out);
    else
        downStage<V, 8>(downState[stage], in, n, out);
}

void LadderBase::prepare(double sampleRate, Quality q, int lanes, int maxBlock)
{
    hostRate = sampleRate;
    quality = q;
    tier = solverTierFor(q);
    stages = oversamplingStagesFor(q);
    osFactor = 1 << stages;
    osRate = sampleRate * osFactor;
    laneCount = lanes >= 4 ? 4 : 1;
    maxBlockSize = std::max(maxBlock, 1);
    coefficients = q == Quality::High ? 8 : 6;
    for (size_t k = 0; k < 8; ++k)
        coefficient[k] = coefficients == 8 ? kIir8[k] : (k < 6 ? kIir6[k] : 0.0f);
    drawTolerances();
    table = pitchOff || !selfOscillates() ? nullptr : pitchTableFor(*this, q, 0);
    tableHigh = table != nullptr && distortedOscillation() ? pitchTableFor(*this, q, 1) : nullptr;
    applyHighpass();
    onPrepared();
    reset();
    measureLatency();
}

void LadderBase::reset()
{
    upState = {};
    downState = {};
    std::fill(std::begin(hpState), std::end(hpState), 0.0f);
    resetState();
    snap = {true, true, true, true};
    dirty = {true, true, true, true};
    restart = true;
    phase = 0;
    blk = LaneBlock{};
    for (int l = 0; l < 4; ++l)
        noise[static_cast<size_t>(l)] = nl::SplitMix(0xA5A5ULL * (laneSeed[static_cast<size_t>(l)] + 1) +
                                                     static_cast<std::uint64_t>(laneDraw[static_cast<size_t>(l)]));
}

void LadderBase::resetLane(int lane)
{
    if (lane < 0 || lane >= 4)
        return;
    const auto l = static_cast<size_t>(lane);
    for (auto* half : {&upState, &downState})
        for (auto& st : *half)
        {
            for (size_t k = 0; k < 8; ++k)
                st.x[k][l] = st.y[k][l] = 0.0f;
            st.prev[l] = 0.0f;
        }
    hpState[l] = 0.0f;
    resetLaneState(lane);
    snap[l] = true;
    dirty[l] = true;
    blk.g[l] = blk.dg[l] = blk.k[l] = blk.dk[l] = blk.chr[l] = blk.gMid[l] = blk.kMid[l] = 0.0f;
    for (auto& stage : blk.gm)
        stage[l] = 0.0f;
    noise[l] = nl::SplitMix(0xA5A5ULL * (laneSeed[l] + 1) + static_cast<std::uint64_t>(laneDraw[l]));
    blk.rebuild = true;
}

void LadderBase::setHighpassStep(int step)
{
    if (!supportsHighpass())
        return;
    step = std::clamp(step, 0, 3);
    if (step == hpStep)
        return;
    hpStep = step;
    applyHighpass();
}

void LadderBase::applyHighpass()
{
    static constexpr float corner[4] = {0.0f, 60.0f, 180.0f, 540.0f};
    const float f = supportsHighpass() ? corner[hpStep] : 0.0f;
    if (f <= 0.0f)
    {
        hpCoefficient = 0.0f;
        return;
    }
    const float g = std::tan(kPi * f / static_cast<float>(osRate));
    hpCoefficient = g / (1.0f + g);
}

// One-pole high-pass (trapezoidal) at the filter rate, in place; lanes are interleaved for the four-lane type.
template <class V> void LadderBase::highpassRun(float* data, int n)
{
    constexpr int stride = nl::vstride<V>;
    const V hpK = nl::vconst<V>(hpCoefficient);
    V state = nl::load<V>(hpState);
    for (int i = 0; i < n; ++i)
    {
        const V x = nl::load<V>(data + i * stride);
        const V v = (x - state) * hpK;
        const V lp = v + state;
        state = lp + v;
        nl::store(data + i * stride, x - lp);
    }
    nl::store(hpState, state);
}

float LadderBase::frontHighpassHz() const
{
    static constexpr float corner[4] = {0.0f, 60.0f, 180.0f, 540.0f};
    return supportsHighpass() ? corner[hpStep] : 0.0f;
}

void LadderBase::responseDb(const float* freqHz, int n, float* outDb) const
{
    using Cx = std::complex<double>;
    constexpr double pi = 3.14159265358979323846;
    const auto& c = ctl[0];
    const double os = osRate;
    const double fc =
        std::clamp(static_cast<double>(c.cutoffHz * pitchRatio(c.cutoffHz, c.resonance, c.character)), 8.0, 0.45 * os);
    const double g = std::tan(pi * fc / os) * static_cast<double>(gScale());
    const double critical = static_cast<double>(selfOscillationGain());
    const double k = std::min(static_cast<double>(feedbackForResonance(c.resonance)), 0.995 * critical);
    const double shaper = c.drive > 0.0f ? std::sqrt(1.0 + static_cast<double>(kDriveMax * c.drive)) : 1.0;
    const double hpHz = static_cast<double>(frontHighpassHz());
    const double gHp = hpHz > 0.0 ? std::tan(pi * hpHz / os) : 0.0;
    for (int i = 0; i < n; ++i)
    {
        const double f = std::clamp(static_cast<double>(freqHz[i]), 1.0, 0.499 * os);
        const double w = std::tan(pi * f / os);
        Cx h = kernelGain(Cx(0.0, w / g), k, static_cast<double>(c.character));
        if (gHp > 0.0)
        {
            const Cx ph(0.0, w / gHp);
            h *= ph / (1.0 + ph);
        }
        outDb[i] = static_cast<float>(20.0 * std::log10(std::max(std::abs(h) * shaper, 1.0e-9)));
    }
}

void LadderBase::setSeed(std::uint32_t seed)
{
    seedValue = seed;
    for (int l = 0; l < 4; ++l)
    {
        laneSeed[static_cast<size_t>(l)] = seed;
        laneDraw[static_cast<size_t>(l)] = l;
    }
    drawTolerances();
    for (int l = 0; l < 4; ++l)
        noise[static_cast<size_t>(l)] = nl::SplitMix(0xA5A5ULL * (seed + 1) + static_cast<std::uint64_t>(l));
    dirty = {true, true, true, true};
    restart = true;
}

void LadderBase::setLaneSeed(int lane, std::uint32_t seed, int draw)
{
    if (lane < 0 || lane >= 4)
        return;
    const auto l = static_cast<size_t>(lane);
    if (laneSeed[l] == seed && laneDraw[l] == draw)
        return;
    laneSeed[l] = seed;
    laneDraw[l] = draw;
    drawLane(lane);
    noise[l] = nl::SplitMix(0xA5A5ULL * (seed + 1) + static_cast<std::uint64_t>(draw));
    dirty[l] = true;
    restart = true;
}

void LadderBase::drawTolerances()
{
    for (int l = 0; l < 4; ++l)
        drawLane(l);
}

void LadderBase::drawLane(int lane)
{
    const auto l = static_cast<size_t>(lane);
    nl::SplitMix gen((static_cast<std::uint64_t>(laneSeed[l]) << 8) + static_cast<std::uint64_t>(laneDraw[l]) +
                     0x1234567ULL);
    for (auto& r : rnd[l])
        r = gen.bipolar();
}

void LadderBase::setLaneControls(int lane, float cutoffHz, float resonance, float drive, float character, float tolerance)
{
    if (lane < 0 || lane >= 4)
        return;
    auto& c = ctl[static_cast<size_t>(lane)];
    c.cutoffHz = cutoffHz;
    c.resonance = std::clamp(resonance, 0.0f, 1.0f);
    c.drive = std::clamp(drive, 0.0f, 1.0f);
    c.character = std::clamp(character, 0.0f, 1.0f);
    c.tolerance = std::clamp(tolerance, 0.0f, 1.0f);
    dirty[static_cast<size_t>(lane)] = true;
    restart = true;
}

// Targets for the next control chunk of one lane; the integrator, feedback and compensation values ramp to them.
void LadderBase::updateTargets(int lane, const float* mod, int samplesLeft, int length)
{
    const auto l = static_cast<size_t>(lane);
    if (!dirty[l] && mod == nullptr)
    {
        // Nothing moved since the last chunk, which ended on the targets: hold them.
        blk.dg[l] = 0.0f;
        blk.dk[l] = 0.0f;
        blk.gMid[l] = blk.g[l];
        blk.kMid[l] = blk.k[l];
        return;
    }
    dirty[l] = mod != nullptr;
    const auto& c = ctl[l];
    const auto& r = rnd[l];
    const float octaves = mod != nullptr ? mod[std::min(samplesLeft, length) - 1] : 0.0f;
    const float tolCutoff = 1.0f + c.tolerance * 0.04f * r[0];
    const float tolFeedback = 1.0f + c.tolerance * 0.03f * r[1];
    const float fc =
        std::clamp(c.cutoffHz * pitchRatio(c.cutoffHz, c.resonance, c.character) * tolCutoff * std::exp2(octaves), 8.0f,
                   0.45f * static_cast<float>(osRate));
    const float gT = std::tan(kPi * fc / static_cast<float>(osRate)) * gScale();
    const float kT = feedbackForResonance(c.resonance) * tolFeedback;
    for (int s = 0; s < 4; ++s)
        blk.gm[s][l] = 1.0f + c.tolerance * 0.01f * r[static_cast<size_t>(2 + s)];
    blk.chr[l] = c.character;
    // Drive shaper: tanh(a x) / a * sqrt(1 + a), the identity for a = 0 and a small-signal gain of sqrt(1 + a).
    const float a = kDriveMax * c.drive;
    shaperA[l] = std::max(a, 1.0e-3f);
    shaperScale[l] = std::sqrt(1.0f + a) / shaperA[l];
    const float steps = static_cast<float>(length * osFactor);
    const auto set = [&](float* cur, float* d, float target)
    {
        if (snap[l])
        {
            cur[l] = target;
            d[l] = 0.0f;
        }
        else
            d[l] = (target - cur[l]) / steps;
    };
    set(blk.g, blk.dg, gT);
    set(blk.k, blk.dk, kT);
    blk.gMid[l] = 0.5f * (blk.g[l] + gT);
    blk.kMid[l] = 0.5f * (blk.k[l] + kT);
    blk.rebuild = true;
    snap[l] = false;
}

namespace
{
void advance(LaneBlock& b, int n)
{
    const auto step = static_cast<float>(n);
    for (int l = 0; l < 4; ++l)
    {
        b.g[l] += b.dg[l] * step;
        b.k[l] += b.dk[l] * step;
    }
}
} // namespace

// One chunk of m host-rate samples, in place. identity: only the resampling chain runs (latency measurement).
template <class V> void LadderBase::runChunk(float* data, int m, bool identity)
{
    constexpr int stride = nl::vstride<V>;
    // Scratch for one chunk on the stack: the data at the shaper rate (at most 8x), at the filter rate, and two
    // ping-pong buffers between the stages.
    constexpr size_t kScratch = static_cast<size_t>(kChunk) * (1u << (kMaxStages + 1)) * stride;
    alignas(16) float shaperBuf[kScratch], filterBuf[kScratch], tmpA[kScratch], tmpB[kScratch];
    const auto top = static_cast<size_t>(stages); // the stage that reaches the shaper rate
    // Up: host rate to the shaper rate (2x the filter rate).
    {
        const float* src = data;
        int len = m;
        for (size_t s = 0; s <= top; ++s)
        {
            float* dst = s == top ? shaperBuf : (s % 2 == 0 ? tmpA : tmpB);
            upAt<V>(s, src, len, dst);
            src = dst;
            len *= 2;
        }
    }
    const int shaperLen = m << (stages + 1);
    float* filterData = filterBuf;
    if (!identity && shaperActive)
    {
        const V a = nl::load<V>(shaperA), scale = nl::load<V>(shaperScale);
        float* p = shaperBuf;
        for (int i = 0; i < shaperLen; ++i)
            nl::store(p + i * stride, scale * nl::tanhApprox(a * nl::load<V>(p + i * stride)));
    }
    downAt<V>(top, shaperBuf, shaperLen / 2, filterData);
    const int total = m * osFactor;
    if (!identity)
    {
        if (hpCoefficient != 0.0f)
            highpassRun<V>(filterData, total);
        if (noiseOn)
            for (int i = 0; i < total; ++i)
                for (int l = 0; l < (stride == 1 ? 1 : 4); ++l)
                    filterData[i * stride + l] += kNoiseAmplitude * noise[static_cast<size_t>(l)].bipolar();
        if constexpr (stride == 1)
            run1(filterData, filterData, total, blk);
        else
            run4(filterData, filterData, total, blk);
        blk.rebuild = false;
        advance(blk, total);
    }
    // Down: filter rate to host rate.
    {
        const float* src = filterData;
        int len = total;
        for (size_t s = top; s-- > 0;)
        {
            len /= 2;
            float* dst = s == 0 ? data : (s % 2 == 0 ? tmpA : tmpB);
            downAt<V>(s, src, len, dst);
            src = dst;
        }
        if (top == 0)
            std::copy(src, src + static_cast<size_t>(m) * stride, data);
    }
}

void LadderBase::process(float* io, int n, const float* cutoffOctMod)
{
    if (restart)
    {
        phase = 0;
        restart = false;
        onRestart();
    }
    int done = 0;
    while (done < n)
    {
        if (phase == 0)
        {
            ctlLen = cutoffOctMod != nullptr ? kFmChunk : kChunk;
            updateTargets(0, cutoffOctMod != nullptr ? cutoffOctMod + done : nullptr, n - done, ctlLen);
            shaperActive = ctl[0].drive > 0.0f;
        }
        const int m = std::min(ctlLen - phase, n - done);
        runChunk<float>(io + done, m, false);
        phase = (phase + m) % ctlLen;
        done += m;
    }
}

void LadderBase::processQuad(float* const io[4], int n, const float* const cutoffOctMod[4])
{
    if (laneCount < 4)
    {
        process(io[0], n, cutoffOctMod != nullptr ? cutoffOctMod[0] : nullptr);
        return;
    }
    if (restart)
    {
        phase = 0;
        laneAge.fill(kChunk);
        restart = false;
        onRestart();
    }
    alignas(16) float chunk[kChunk * 4];
    int done = 0;
    while (done < n)
    {
        if (phase == 0)
        {
            shaperActive = false;
            ctlLen = kChunk;
            if (cutoffOctMod != nullptr)
                for (int l = 0; l < 4; ++l)
                    if (cutoffOctMod[l] != nullptr)
                        ctlLen = kFmChunk;
            for (int l = 0; l < 4; ++l)
            {
                const auto li = static_cast<size_t>(l);
                const float* mod = cutoffOctMod != nullptr ? cutoffOctMod[l] : nullptr;
                // A lane without modulation keeps the single path's control rate: it retargets every kChunk samples
                // and holds its ramp and predictor values in between, however short the shared chunk is.
                if (mod != nullptr || ctlLen == kChunk || laneAge[li] >= kChunk)
                {
                    updateTargets(l, mod != nullptr ? mod + done : nullptr, n - done, mod != nullptr ? ctlLen : kChunk);
                    laneAge[li] = 0;
                }
                shaperActive = shaperActive || ctl[li].drive > 0.0f;
            }
        }
        const int m = std::min(ctlLen - phase, n - done);
        for (int i = 0; i < m; ++i)
            for (int l = 0; l < 4; ++l)
                chunk[i * 4 + l] = io[l][done + i];
        runChunk<nl::Float4>(chunk, m, false);
        for (int i = 0; i < m; ++i)
            for (int l = 0; l < 4; ++l)
                io[l][done + i] = chunk[i * 4 + l];
        phase = (phase + m) % ctlLen;
        for (auto& age : laneAge)
            age += m;
        done += m;
    }
}

// Latency of the resampling chain in host-rate samples: an impulse through it, parabolic peak.
void LadderBase::measureLatency()
{
    upState = {};
    downState = {};
    std::array<float, 64> pulse{};
    std::array<float, 256> out{};
    for (size_t pos = 0; pos < out.size(); pos += kChunk)
    {
        for (size_t i = 0; i < kChunk; ++i)
            pulse[i] = (pos + i == 0) ? 1.0f : 0.0f;
        runChunk<float>(pulse.data(), kChunk, true);
        for (size_t i = 0; i < kChunk; ++i)
            out[pos + i] = pulse[i];
    }
    size_t peak = 0;
    for (size_t i = 1; i < out.size(); ++i)
        if (std::abs(out[i]) > std::abs(out[peak]))
            peak = i;
    float frac = 0.0f;
    if (peak > 0 && peak + 1 < out.size())
    {
        const float a = out[peak - 1], b = out[peak], c = out[peak + 1];
        const float denom = a - 2.0f * b + c;
        frac = std::abs(denom) > 1.0e-9f ? 0.5f * (a - c) / denom : 0.0f;
    }
    latencySamples = static_cast<float>(peak) + frac;
    upState = {};
    downState = {};
}

// ---------------------------------------------------------------------------------------------------------------
// Pitch correction. A nonlinear ladder at full resonance oscillates a little below the requested cutoff (the
// saturating stages slow the loop down), and at Draft the coarse step adds a frequency-dependent error. Both are
// measured once per model and tier by running the model itself, and the cutoff is divided by the measured ratio.

namespace
{
constexpr double kMeasureRate = 48000.0;
constexpr double kPi64 = 3.14159265358979323846;

// Frequency of a sampled sinusoid from the three-point recursion x[n+1] + x[n-1] = 2 cos(w) x[n], least squares.
double sinusoidFrequency(const std::vector<float>& x, size_t from, size_t count, double rate)
{
    double num = 0.0, den = 0.0;
    for (size_t i = from + 1; i + 1 < from + count; ++i)
    {
        num += static_cast<double>(x[i]) * (static_cast<double>(x[i - 1]) + static_cast<double>(x[i + 1]));
        den += static_cast<double>(x[i]) * static_cast<double>(x[i]);
    }
    if (den <= 1.0e-30)
        return 0.0;
    return std::acos(std::clamp(num / (2.0 * den), -1.0, 1.0)) * rate / (2.0 * kPi64);
}

// Frequency of a periodic signal from its rising crossings of the mean, linearly interpolated: the average period over
// the window, unaffected by the harmonics of a distorted limit cycle.
double crossingFrequency(const std::vector<float>& x, size_t from, size_t count, double rate)
{
    double mean = 0.0;
    for (size_t i = from; i < from + count; ++i)
        mean += static_cast<double>(x[i]);
    mean /= static_cast<double>(count);
    double first = -1.0, last = -1.0;
    int crossings = 0;
    for (size_t i = from + 1; i < from + count; ++i)
    {
        const double a = static_cast<double>(x[i - 1]) - mean, b = static_cast<double>(x[i]) - mean;
        if (a < 0.0 && b >= 0.0)
        {
            const double t = static_cast<double>(i - 1) + (-a) / (b - a);
            if (crossings == 0)
                first = t;
            last = t;
            ++crossings;
        }
    }
    return crossings >= 3 ? static_cast<double>(crossings - 1) * rate / (last - first) : 0.0;
}
} // namespace

LadderBase::PitchTable LadderBase::measurePitchTable(const LadderBase& prototype, Quality quality, float character)
{
    PitchTable t;
    auto probe = prototype.clone();
    auto* model = static_cast<LadderBase*>(probe.get());
    model->pitchOff = true;
    model->table = nullptr;
    model->tableHigh = nullptr;
    model->setHighpassStep(0);
    model->prepare(kMeasureRate, quality, 1, 256);
    const double xMin = 0.005, xMax = 0.34;
    std::vector<float> signal;
    for (int j = 0; j < PitchTable::kPoints; ++j)
    {
        const double x = xMin * std::pow(xMax / xMin, static_cast<double>(j) / (PitchTable::kPoints - 1));
        const double fc = x * kMeasureRate;
        const double period = kMeasureRate / fc;
        const size_t drive = static_cast<size_t>(40.0 * period), settle = static_cast<size_t>(110.0 * period),
                     measure = std::max<size_t>(static_cast<size_t>(50.0 * period), 1024);
        const size_t total = drive + settle + measure;
        signal.assign(total + 256, 0.0f);
        for (size_t i = 0; i < drive; ++i)
            signal[i] = 0.03f * static_cast<float>(std::sin(2.0 * kPi64 * fc * static_cast<double>(i) / kMeasureRate));
        model->reset();
        model->setControls(static_cast<float>(fc), 1.0f, 0.0f, character, 0.0f);
        for (size_t off = 0; off < total; off += 256)
            model->process(signal.data() + off, 256, nullptr);
        const double f = prototype.distortedOscillation() ? crossingFrequency(signal, drive + settle, measure, kMeasureRate)
                                                          : sinusoidFrequency(signal, drive + settle, measure, kMeasureRate);
        const double ratio = f > 0.0 ? std::clamp(f / fc, 0.5, 1.5) : 1.0;
        t.logSet[static_cast<size_t>(j)] = static_cast<float>(std::log2(x));
        t.logOsc[static_cast<size_t>(j)] = static_cast<float>(std::log2(x * ratio));
    }
    // The inverse needs a monotonic curve; a noisy point is pulled up to its predecessor.
    for (size_t j = 1; j < t.logOsc.size(); ++j)
        t.logOsc[j] = std::max(t.logOsc[j], t.logOsc[j - 1] + 1.0e-3f);
    return t;
}

const LadderBase::PitchTable* LadderBase::pitchTableFor(const LadderBase& prototype, Quality quality, int anchor)
{
    static std::mutex lock;
    static std::array<std::array<std::array<PitchTable, 2>, 3>, kModelIds> cache{};
    static std::array<std::array<std::array<bool, 2>, 3>, kModelIds> built{};
    const auto m = static_cast<size_t>(prototype.modelId());
    const auto q = static_cast<size_t>(quality);
    std::lock_guard<std::mutex> guard(lock);
    const auto a = static_cast<size_t>(anchor);
    if (!built[m][q][a])
    {
        cache[m][q][a] = measurePitchTable(prototype, quality, static_cast<float>(anchor));
        built[m][q][a] = true;
    }
    return &cache[m][q][a];
}

// The cutoff to hand to the integrators so that full resonance oscillates at cutoffHz; the correction fades in from
// the resonance at which the loop starts to oscillate.
float LadderBase::tableOffset(const PitchTable& t, float cutoffHz) const
{
    const float want = std::log2(std::max(cutoffHz, 1.0f) / static_cast<float>(hostRate));
    float set;
    if (want <= t.logOsc.front())
        set = want + (t.logSet.front() - t.logOsc.front());
    else if (want >= t.logOsc.back())
        set = want + (t.logSet.back() - t.logOsc.back());
    else
    {
        size_t i = 1;
        while (t.logOsc[i] < want)
            ++i;
        const float f = (want - t.logOsc[i - 1]) / (t.logOsc[i] - t.logOsc[i - 1]);
        set = t.logSet[i - 1] + (t.logSet[i] - t.logSet[i - 1]) * f;
    }
    return set - want;
}

float LadderBase::pitchRatio(float cutoffHz, float resonance, float character) const
{
    if (table == nullptr)
        return 1.0f;
    const float onset = selfOscillationGain() / feedbackForResonance(1.0f);
    const float weight = std::clamp((resonance - onset) / (1.0f - onset), 0.0f, 1.0f);
    if (weight <= 0.0f)
        return 1.0f;
    float offset = tableOffset(*table, cutoffHz);
    if (tableHigh != nullptr)
        offset += character * (tableOffset(*tableHigh, cutoffHz) - offset);
    return std::exp2(offset * weight);
}

} // namespace crucible::filters
