// SPDX-License-Identifier: MIT
#pragma once

// The voice card (docs/plans/ENGINE-COMMUNITY-PLAN.md, section 2.9): the part-to-part tolerances of one hardware voice.
// A polysynth voice is a card with its own oscillator cores, filter and envelope components, and the values stay the
// same from note to note. Here a card is a function of the engine seed and the voice index: unit-variance draws (a sum
// of twelve uniforms, so bounded to +-6, mean 0, standard deviation 1) that the consumers scale by their tolerance
// amount. A null card pointer everywhere means the legacy behaviour, with no tolerance at all.
//
// Also here: the optional seeded noise floor and mains hum of the analog stage.

#include <crucible/Dsp.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace crucible
{

// Draws of one oscillator unison voice (osc.analog): pitch, pulse width, level and ramp curvature.
struct OscCard
{
    float pitch = 0.0f;
    float width = 0.0f;
    float level = 0.0f;
    float curve = 0.0f;
};

// How strongly each part of the card applies, 0..1 (0 = that part is off). All zero means no card.
struct CardAmounts
{
    float osc = 0.0f;    // oscillator pitch, pulse width, level and ramp curvature
    float filter = 0.0f; // component spread of the analog filter models
    float env = 0.0f;    // envelope time constants and sustain level
    float level = 0.0f;  // gain of the voice amplifier
    bool any() const { return osc > 0.0f || filter > 0.0f || env > 0.0f || level > 0.0f; }
};

constexpr int kCardOscSlots = 8;
constexpr int kCardUnison = 8;
constexpr int kCardFilters = 4;
constexpr int kCardEnvs = 4;

struct VoiceCard
{
    std::uint32_t seed = 0;
    int index = 0;
    float pitch = 0.0f; // tuning of the whole voice, for the oscillators that do not draw their own (unit variance)
    float level = 0.0f; // gain of the voice amplifier
    std::array<std::array<OscCard, kCardUnison>, kCardOscSlots> osc{};
    std::array<float, kCardFilters> filterSpread{};       // scales the filter tolerance amount
    std::array<std::uint32_t, kCardFilters> filterSeed{}; // seed of the filter's component values
    std::array<float, kCardEnvs> envTime{};               // log-normal factor on attack, decay and release
    std::array<float, kCardEnvs> envSustain{};            // sustain level offset
    std::uint32_t noiseSeed = 1;

    // Standard deviations at tolerance 1.
    static constexpr float kPitchCents = 5.0f;
    static constexpr float kLevelSpread = 0.02f;
    static constexpr float kFilterSpread = 0.25f;
    static constexpr float kEnvTimeSpread = 0.05f;
    static constexpr float kEnvSustainSpread = 0.01f;

    static float draw(Rng& g)
    {
        float s = -6.0f;
        for (int i = 0; i < 12; ++i)
            s += g.unipolar();
        return s;
    }

    static std::uint32_t mix(std::uint32_t x)
    {
        x ^= x >> 16U;
        x *= 0x85ebca6bU;
        x ^= x >> 13U;
        x *= 0xc2b2ae35U;
        x ^= x >> 16U;
        return x;
    }

    // The draws of the unison voices of one oscillator from a note seed: the oscillator's own source when no voice card
    // is supplied (unchanged since osc.analog first drew them).
    static void oscillatorFromSeed(std::uint32_t noteSeed, OscCard* out)
    {
        Rng g;
        g.state = (noteSeed * 3266489917U + 0x9e3779b9U) | 1U;
        for (int k = 0; k < kCardUnison; ++k)
        {
            out[k].pitch = draw(g);
            out[k].width = draw(g);
            out[k].level = draw(g);
            out[k].curve = draw(g);
        }
    }

    static VoiceCard make(std::uint32_t engineSeed, int voiceIndex)
    {
        VoiceCard c;
        c.seed = engineSeed;
        c.index = voiceIndex;
        Rng g;
        g.state = mix(engineSeed ^ mix(static_cast<std::uint32_t>(voiceIndex) * 0x9e3779b9U + 0x7f4a7c15U)) | 1U;
        for (int i = 0; i < 4; ++i)
            g.next();
        c.pitch = draw(g);
        c.level = draw(g);
        for (auto& slot : c.osc)
            for (auto& u : slot)
            {
                u.pitch = draw(g);
                u.width = draw(g);
                u.level = draw(g);
                u.curve = draw(g);
            }
        for (int f = 0; f < kCardFilters; ++f)
        {
            c.filterSpread[static_cast<size_t>(f)] = draw(g);
            c.filterSeed[static_cast<size_t>(f)] = g.next();
        }
        for (int e = 0; e < kCardEnvs; ++e)
        {
            c.envTime[static_cast<size_t>(e)] = draw(g);
            c.envSustain[static_cast<size_t>(e)] = draw(g);
        }
        c.noiseSeed = g.next() | 1U;
        return c;
    }

    // Factors for the consumers.
    float pitchSemitones(float amount) const { return pitch * kPitchCents * 0.01f * amount; }
    float gain(float amount) const { return 1.0f + level * kLevelSpread * amount; }
    float envTimeScale(int env, float amount) const
    {
        return std::exp(envTime[static_cast<size_t>(env)] * kEnvTimeSpread * amount);
    }
    float envSustainOffset(int env, float amount) const
    {
        return envSustain[static_cast<size_t>(env)] * kEnvSustainSpread * amount;
    }
    float filterTolerance(int slot, float amount) const
    {
        return std::clamp(amount * (1.0f + filterSpread[static_cast<size_t>(slot)] * kFilterSpread), 0.0f, 1.0f);
    }
};

// The noise floor and the mains hum of the analog stage. Both are stereo-added at a set RMS level in dBFS (-120 to -60),
// seeded, and off by default; a level change ramps over the chunk. The hum is a fundamental at 50 or 60 Hz with its
// second and third harmonics at 0.5 and 0.3 of its amplitude, the same phase on both channels; the noise is white and
// independent per channel.
class NoiseHum
{
public:
    void prepare(double rate, std::uint32_t seed)
    {
        sampleRate = rate;
        rng.state = seed | 1U;
        phase = 0.0;
        noiseGain = humGain = 0.0f;
    }

    void reset(std::uint32_t seed)
    {
        rng.state = seed | 1U;
        phase = 0.0;
        noiseGain = humGain = 0.0f;
    }

    // Adds the floor to left / right when either source is on (or still ramping down).
    void add(float* left, float* right, int n, bool noiseOn, float noiseDb, bool humOn, float humDb, float humHz)
    {
        const float noiseTarget = noiseOn ? levelToGain(noiseDb) * 1.7320508f : 0.0f; // uniform noise has RMS 1/sqrt(3)
        // fundamental amplitude for the requested total RMS: sqrt(2) / sqrt(1 + 0.25 + 0.09)
        const float humTarget = humOn ? levelToGain(humDb) * 1.2217305f : 0.0f;
        if (noiseGain == 0.0f && noiseTarget == 0.0f && humGain == 0.0f && humTarget == 0.0f)
            return;
        const double inc = static_cast<double>(humHz) / sampleRate;
        const float dn = (noiseTarget - noiseGain) / static_cast<float>(n);
        const float dh = (humTarget - humGain) / static_cast<float>(n);
        for (int i = 0; i < n; ++i)
        {
            noiseGain += dn;
            humGain += dh;
            const float w = static_cast<float>(phase);
            const float hum = humGain * (sinTurns(w) + 0.5f * sinTurns(2.0f * w) + 0.3f * sinTurns(3.0f * w));
            left[i] += noiseGain * rng.bipolar() + hum;
            right[i] += noiseGain * rng.bipolar() + hum;
            phase += inc;
            if (phase >= 1.0)
                phase -= 1.0;
        }
        noiseGain = noiseTarget;
        humGain = humTarget;
    }

private:
    static float levelToGain(float db) { return std::pow(10.0f, std::clamp(db, -120.0f, -60.0f) * 0.05f); }
    static float sinTurns(float turns) { return std::sin(kTwoPi * (turns - std::floor(turns))); }

    double sampleRate = 48000.0;
    Rng rng;
    double phase = 0.0;
    float noiseGain = 0.0f;
    float humGain = 0.0f;
};

} // namespace crucible
