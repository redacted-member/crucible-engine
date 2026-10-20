// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace crucible
{

constexpr float kTwoPi = 6.28318530717958647692f;

struct Rng
{
    std::uint32_t state = 0x12345678U;

    std::uint32_t next()
    {
        state ^= state << 13U;
        state ^= state >> 17U;
        state ^= state << 5U;
        return state;
    }

    // Uniform in [-1, 1).
    float bipolar() { return static_cast<float>(next() >> 8U) * (2.0f / 16777216.0f) - 1.0f; }

    // Uniform in [0, 1).
    float unipolar() { return static_cast<float>(next() >> 8U) * (1.0f / 16777216.0f); }
};

// Attack, hold, decay, sustain, release with one curve control shared by the timed segments.
// curve < 0 gives fast-start (exponential-like) segments, 0 is linear, > 0 is slow-start.
struct AdsrParams
{
    float attack = 0.005f;
    float decay = 0.3f;
    float sustain = 0.8f;
    float release = 0.25f;
    float hold = 0.0f;
    float curve = -0.8f;      // bends decay and release
    float attackCurve = 0.0f; // bends the attack: 0 is linear, so the peak is reached at the attack time
};

class Adsr
{
public:
    void prepare(double rate) { sampleRate = static_cast<float>(rate); }

    void setParams(const AdsrParams& p)
    {
        params = p;
        attackStep = 1.0f / (std::max(p.attack, 0.0005f) * sampleRate);
        decayStep = 1.0f / (std::max(p.decay, 0.0005f) * sampleRate);
        releaseStep = 1.0f / (std::max(p.release, 0.0005f) * sampleRate);
        holdSamples = static_cast<int>(std::max(p.hold, 0.0f) * sampleRate);
        curveK = std::clamp(p.curve, -1.0f, 1.0f) * 8.0f;
        curveDen = std::abs(curveK) < 1.0e-3f ? 1.0f : std::expm1(curveK);
        attackK = std::clamp(p.attackCurve, -1.0f, 1.0f) * 8.0f;
        attackDen = std::abs(attackK) < 1.0e-3f ? 1.0f : std::expm1(attackK);
    }

    void noteOn()
    {
        startLevel = level;
        t = 0.0f;
        stage = Stage::Attack;
    }

    void noteOff()
    {
        if (stage != Stage::Idle)
        {
            startLevel = level;
            t = 0.0f;
            stage = Stage::Release;
        }
    }

    void reset()
    {
        stage = Stage::Idle;
        level = 0.0f;
    }

    bool isIdle() const { return stage == Stage::Idle; }
    bool isReleasing() const { return stage == Stage::Release; }
    bool isSustaining() const { return stage == Stage::Sustain; }
    float value() const { return level; }

    float tick()
    {
        switch (stage)
        {
        case Stage::Idle:
            break;
        case Stage::Attack:
            t += attackStep;
            if (t >= 1.0f)
            {
                level = 1.0f;
                t = 0.0f;
                holdCount = holdSamples;
                stage = holdSamples > 0 ? Stage::Hold : Stage::Decay;
            }
            else
            {
                level = startLevel + (1.0f - startLevel) * attackShape(t);
            }
            break;
        case Stage::Hold:
            level = 1.0f;
            if (--holdCount <= 0)
            {
                t = 0.0f;
                stage = Stage::Decay;
            }
            break;
        case Stage::Decay:
            t += decayStep;
            if (t >= 1.0f)
            {
                level = params.sustain;
                stage = Stage::Sustain;
            }
            else
            {
                level = 1.0f - (1.0f - params.sustain) * shape(t);
            }
            break;
        case Stage::Sustain:
            level += (params.sustain - level) * 0.02f;
            break;
        case Stage::Release:
            t += releaseStep;
            if (t >= 1.0f)
            {
                level = 0.0f;
                stage = Stage::Idle;
            }
            else
            {
                level = startLevel * (1.0f - shape(t));
            }
            break;
        }
        return level;
    }

    float advance(int numSamples)
    {
        float v = level;
        for (int i = 0; i < numSamples; ++i)
            v = tick();
        return v;
    }

private:
    enum class Stage
    {
        Idle,
        Attack,
        Hold,
        Decay,
        Sustain,
        Release
    };

    // Nearly linear, with the corners at both ends rounded (a blend of the ramp and its raised-cosine version):
    // reaches 99 % at 0.98 of the attack time and keeps short attacks click free.
    float attackShape(float x) const
    {
        constexpr float smooth = 0.6f;
        const float base = x - smooth * std::sin(6.2831853f * x) * 0.15915494f;
        if (std::abs(attackK) < 1.0e-3f)
            return base;
        return std::expm1(attackK * base) / attackDen;
    }

    float shape(float x) const
    {
        if (std::abs(curveK) < 1.0e-3f)
            return x;
        return std::expm1(curveK * x) / curveDen;
    }

    float attackK = 0.0f;
    float attackDen = 1.0f;
    AdsrParams params;
    Stage stage = Stage::Idle;
    float sampleRate = 44100.0f;
    float level = 0.0f;
    float startLevel = 0.0f;
    float t = 0.0f;
    float attackStep = 0.01f;
    float decayStep = 0.001f;
    float releaseStep = 0.001f;
    float curveK = -6.4f;
    float curveDen = -1.0f;
    int holdSamples = 0;
    int holdCount = 0;
};

constexpr int kLfoShapePoints = 128;

// Beats per LFO cycle for each tempo-sync choice (index 0 is "Off", 14 entries).
inline float lfoSyncBeats(int index)
{
    constexpr float table[] = {0.0f,        16.0f, 8.0f, 4.0f,        2.0f,  4.0f / 3.0f, 1.0f,
                               2.0f / 3.0f, 0.75f, 0.5f, 1.0f / 3.0f, 0.25f, 1.0f / 6.0f, 0.125f};
    return table[std::clamp(index, 0, 13)];
}

enum class LfoTrigger
{
    Retrigger,
    Free,
    OneShot
};

struct LfoSettings
{
    float rateHz = 1.0f;
    int shape = 0;
    float fadeSeconds = 0.0f;
    float delaySeconds = 0.0f;
    float startPhase = 0.0f;
    float smooth = 0.0f;
    LfoTrigger trigger = LfoTrigger::Retrigger;
};

// Shape indices: 0 sine, 1 triangle, 2 saw up, 3 square, 4 sample and hold, 5 saw down, 6 smooth random,
// 7 pulse 25%, 8 stairs, 9 rectified sine, 10 exponential decay, 11 custom table.
constexpr int kNumLfoShapes = 12;

class Lfo
{
public:
    void reset(std::uint32_t seed, float startPhase)
    {
        phase = startPhase - std::floor(startPhase);
        rng.state = seed | 1U;
        held = rng.bipolar();
        randPrev = rng.bipolar();
        randNext = rng.bipolar();
        age = 0.0f;
        done = false;
        smoothed = 0.0f;
        primed = false;
    }

    // Advances by numSamples and returns the bipolar value in [-1, 1]. 'custom' may be null.
    float advance(const LfoSettings& s, int numSamples, double sampleRate, const float* custom)
    {
        const float dt = static_cast<float>(numSamples) / static_cast<float>(sampleRate);
        age += dt;
        const float runTime = age - s.delaySeconds;
        float value = 0.0f;
        if (runTime >= 0.0f)
        {
            if (!done)
            {
                phase += s.rateHz * dt;
                if (phase >= 1.0f)
                {
                    if (s.trigger == LfoTrigger::OneShot)
                    {
                        phase = 0.999999f;
                        done = true;
                    }
                    else
                    {
                        phase -= std::floor(phase);
                    }
                    held = rng.bipolar();
                    randPrev = randNext;
                    randNext = rng.bipolar();
                }
            }
            value = shapeValue(s.shape, phase, custom);
            if (s.fadeSeconds > 0.0f)
                value *= std::min(1.0f, runTime / s.fadeSeconds);
        }
        if (s.smooth > 0.0f && primed)
        {
            const float tau = 0.002f * std::pow(150.0f, s.smooth);
            const float coef = std::exp(-dt / tau);
            smoothed = value + (smoothed - value) * coef;
            value = smoothed;
        }
        else
        {
            smoothed = value;
        }
        primed = true;
        return value;
    }

    float shapeValue(int shape, float p, const float* custom) const
    {
        switch (shape)
        {
        case 0:
            return std::sin(kTwoPi * p);
        case 1:
            return p < 0.5f ? 4.0f * p - 1.0f : 3.0f - 4.0f * p;
        case 2:
            return 2.0f * p - 1.0f;
        case 3:
            return p < 0.5f ? 1.0f : -1.0f;
        case 4:
            return held;
        case 5:
            return 1.0f - 2.0f * p;
        case 6:
        {
            const float x = p * p * (3.0f - 2.0f * p);
            return randPrev + (randNext - randPrev) * x;
        }
        case 7:
            return p < 0.25f ? 1.0f : -1.0f;
        case 8:
            return std::floor(p * 4.0f) * (2.0f / 3.0f) - 1.0f;
        case 9:
            return 2.0f * std::abs(std::sin(3.14159265f * p)) - 1.0f;
        case 10:
            return 2.0f * std::exp(-5.0f * p) - 1.0f;
        default:
        {
            if (custom == nullptr)
                return 0.0f;
            const float x = p * static_cast<float>(kLfoShapePoints);
            const int i = std::min(static_cast<int>(x), kLfoShapePoints - 1);
            const float f = x - static_cast<float>(i);
            const float a = custom[i];
            const float b = custom[(i + 1) % kLfoShapePoints];
            return a + (b - a) * f;
        }
        }
    }

    float currentPhase() const { return phase; }

private:
    float phase = 0.0f;
    float held = 0.0f;
    float randPrev = 0.0f;
    float randNext = 0.0f;
    float age = 0.0f;
    float smoothed = 0.0f;
    bool done = false;
    bool primed = false;
    Rng rng;
};

} // namespace crucible
