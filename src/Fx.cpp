// SPDX-License-Identifier: MIT
#include <crucible/Fx.h>

#include <algorithm>

namespace crucible
{
namespace
{
constexpr float kTwoPiF = 6.28318530717958f;

float dbToGain(float db)
{
    return std::pow(10.0f, db / 20.0f);
}

float coefFor(double sr, float seconds)
{
    return 1.0f - std::exp(-1.0f / (seconds * static_cast<float>(sr)));
}

float onePoleCoef(float hz, double sr)
{
    return 1.0f - std::exp(-kTwoPiF * std::min(hz, 0.45f * static_cast<float>(sr)) / static_cast<float>(sr));
}

float nextNoise(std::uint32_t& s)
{
    s ^= s << 13U;
    s ^= s >> 17U;
    s ^= s << 5U;
    return static_cast<float>(s >> 8U) * (2.0f / 16777216.0f) - 1.0f;
}

int nextPow2(int v)
{
    int p = 2;
    while (p < v)
        p <<= 1;
    return p;
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------
void DelayLine::allocate(int minimumSamples)
{
    const int size = nextPow2(minimumSamples + 8);
    buffer.assign(static_cast<size_t>(size), 0.0f);
    mask = size - 1;
    pos = 0;
}

void DelayLine::clear()
{
    std::fill(buffer.begin(), buffer.end(), 0.0f);
    pos = 0;
    clearCursor = 0;
}

bool DelayLine::clearStep(int& budget)
{
    const int size = static_cast<int>(buffer.size());
    const int n = std::min(std::max(budget, 0), size - clearCursor);
    std::fill_n(buffer.begin() + clearCursor, n, 0.0f);
    clearCursor += n;
    budget -= n;
    if (clearCursor < size)
        return false;
    clearCursor = 0;
    pos = 0;
    return true;
}

float DelayLine::read(float delay) const
{
    delay = std::clamp(delay, 2.0f, static_cast<float>(mask - 4));
    const float p = static_cast<float>(pos) - delay;
    const float fl = std::floor(p);
    const float f = p - fl;
    const int i = static_cast<int>(fl);
    const float y0 = buffer[static_cast<size_t>((i - 1) & mask)];
    const float y1 = buffer[static_cast<size_t>(i & mask)];
    const float y2 = buffer[static_cast<size_t>((i + 1) & mask)];
    const float y3 = buffer[static_cast<size_t>((i + 2) & mask)];
    const float c1 = 0.5f * (y2 - y0);
    const float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
    return ((c3 * f + c2) * f + c1) * f + y1;
}

// ---------------------------------------------------------------------------------------------------------------
void FxLimiter::prepare(double sampleRate)
{
    sr = sampleRate;
    audio.prepare(2, maxLook / 2 - 1, sampleRate);
    reset();
}

void FxLimiter::reset()
{
    primed = false;
    audio.reset();
    required.fill(1.0f);
    minimum.fill(1.0f);
    pos = 0;
    lastLook = -1;
    avgSum = 0.0;
    gain = 1.0f;
    mixSmooth = 1.0f;
}

void FxLimiter::process(float* l, float* r, int n, const FxLimiterParams& p)
{
    if (!primed)
    {
        mixSmooth = p.mix;
        primed = true;
    }
    const int look = std::clamp(static_cast<int>(p.lookaheadMs * 0.001f * static_cast<float>(sr)), 1, maxLook / 2 - 1);
    const float ceiling = dbToGain(p.ceilingDb);
    const float inGain = dbToGain(p.gainDb);
    const float release = coefFor(sr, std::max(p.releaseMs, 1.0f) * 0.001f);
    const float mc = coefFor(sr, 0.01f);
    constexpr int mask = maxLook - 1;
    if (lastLook < 0)
    {
        // First block after prepare() or reset(): start from an empty window (the delay applies at once).
        required.fill(1.0f);
        minimum.fill(1.0f);
        avgSum = look;
        gain = 1.0f;
    }
    else if (look != lastLook)
    {
        // The detector keeps its history and the box sum is rebuilt for the new window; the audio delay crossfades
        // to the new length, so nothing drops out.
        double exact = 0.0;
        for (int j = 1; j <= look; ++j)
            exact += minimum[static_cast<size_t>((pos - j) & mask)];
        avgSum = exact;
    }
    if (look != lastLook)
    {
        audio.setDelay(look);
        lastLook = look;
    }
    constexpr int piece = 512;
    float delayedL[piece], delayedR[piece], gainAt[piece], mixAt[piece];
    float* delayed[2] = {delayedL, delayedR};
    float* ch[2] = {l, r};
    for (int start = 0; start < n; start += piece)
    {
        const int len = std::min(piece, n - start);
        for (int i = 0; i < len; ++i)
        {
            mixSmooth += (p.mix - mixSmooth) * mc;
            mixAt[i] = mixSmooth;
            if ((pos & 4095) == 0)
            {
                double exact = 0.0;
                for (int j = 1; j <= look; ++j)
                    exact += minimum[static_cast<size_t>((pos - j) & mask)];
                avgSum = exact;
            }
            const float a = ch[0][start + i] * inGain;
            const float b = ch[1][start + i] * inGain;
            const float peak = std::max(std::abs(a), std::abs(b));
            required[static_cast<size_t>(pos)] = peak > ceiling ? ceiling / peak : 1.0f;
            delayedL[i] = a;
            delayedR[i] = b;

            float window = 1.0f;
            for (int j = 0; j <= look; ++j)
                window = std::min(window, required[static_cast<size_t>((pos - j) & mask)]);
            minimum[static_cast<size_t>(pos)] = window;

            avgSum += static_cast<double>(window) - static_cast<double>(minimum[static_cast<size_t>((pos - look) & mask)]);
            const float target = std::min(1.0f, static_cast<float>(avgSum / look));
            if (target < gain)
                gain = target;
            else
                gain += (target - gain) * release;
            gainAt[i] = gain;
            pos = (pos + 1) & mask;
        }
        audio.process(delayed, len);
        for (size_t c = 0; c < 2; ++c)
            for (int i = 0; i < len; ++i)
            {
                const float dry = delayed[c][i];
                const float wet = std::clamp(dry * gainAt[i], -ceiling, ceiling);
                ch[c][start + i] = dry + (wet - dry) * mixAt[i];
            }
    }
}

// ---------------------------------------------------------------------------------------------------------------
void FxCrusher::prepare(double sampleRate)
{
    sr = sampleRate;
    reset();
}

void FxCrusher::reset()
{
    primed = false;
    held = {};
    acc = 1.0f;
    mixSmooth = 1.0f;
    tracking = false;
    rngState = 0x2468ace1U; // the noise of jitter and dither starts again as it does for a new instance
}

void FxCrusher::process(float* l, float* r, int n, const FxCrusherParams& p)
{
    if (!primed)
    {
        mixSmooth = p.mix;
        primed = true;
    }
    const float levels = std::pow(2.0f, std::clamp(p.bits, 1.0f, 16.0f)) * 0.5f;
    const bool follow = p.track && p.noteHz > 0.0f;
    float rateHz = p.rateHz;
    if (follow)
        rateHz = p.noteHz * p.ratio * std::exp2(p.fineCents * (1.0f / 1200.0f));
    const float step = std::clamp(rateHz / static_cast<float>(sr), 0.001f, 1.0f);
    // Tracking glides the step so a note change does not click; the fixed rate is used as it is.
    if (!follow || !tracking)
        stepSmooth = step;
    tracking = follow;
    const float stepCoef = coefFor(sr, 0.01f);
    const float ditherAmp = p.dither / levels;
    const float mc = coefFor(sr, 0.01f);
    float* ch[2] = {l, r};
    for (int i = 0; i < n; ++i)
    {
        mixSmooth += (p.mix - mixSmooth) * mc;
        if (follow)
            stepSmooth += (step - stepSmooth) * stepCoef;
        acc += (follow ? stepSmooth : step) * (1.0f + p.jitter * 0.5f * nextNoise(rngState));
        if (acc >= 1.0f)
        {
            acc -= std::floor(acc);
            for (size_t c = 0; c < 2; ++c)
            {
                const float noise = ditherAmp * 0.5f * (nextNoise(rngState) + nextNoise(rngState));
                held[c] = std::round((ch[c][i] + noise) * levels) / levels;
            }
        }
        for (size_t c = 0; c < 2; ++c)
        {
            const float dry = ch[c][i];
            ch[c][i] = dry + (held[c] - dry) * mixSmooth;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
void FxWidth::prepare(double sampleRate)
{
    sr = sampleRate;
    reset();
}

void FxWidth::reset()
{
    primed = false;
    sideLp = 0.0f;
    mixSmooth = 1.0f;
}

void FxWidth::process(float* l, float* r, int n, const FxWidthParams& p)
{
    if (!primed)
    {
        mixSmooth = p.mix;
        primed = true;
    }
    const bool bassMono = p.bassMonoHz > 25.0f;
    const float coef = onePoleCoef(std::max(p.bassMonoHz, 20.0f), sr);
    const float angle = (std::clamp(p.pan, -1.0f, 1.0f) + 1.0f) * 0.78539816f;
    const float gl = std::cos(angle) * 1.41421356f;
    const float gr = std::sin(angle) * 1.41421356f;
    const float mc = coefFor(sr, 0.01f);
    for (int i = 0; i < n; ++i)
    {
        mixSmooth += (p.mix - mixSmooth) * mc;
        const float mid = 0.5f * (l[i] + r[i]);
        const float side = 0.5f * (l[i] - r[i]);
        sideLp += (side - sideLp) * coef;
        const float high = side - sideLp;
        const float widened = p.width * (bassMono ? high : side);
        const float wl = (mid + widened) * gl;
        const float wr = (mid - widened) * gr;
        l[i] += (wl - l[i]) * mixSmooth;
        r[i] += (wr - r[i]) * mixSmooth;
    }
}

} // namespace crucible
