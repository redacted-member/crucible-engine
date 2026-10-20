// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/filters/FilterModel.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace crucible
{

enum class FilterModel
{
    Lowpass12,
    Highpass12,
    Bandpass12,
    Notch,
    Peak,
    Lowpass24,
    Highpass24,
    Bandpass24,
    LadderLowpass,
    LadderHighpass,
    LadderBandpass,
    DiodeLowpass,
    CombPlus,
    CombMinus,
    Formant,
    Phaser,
    DirtyLowpass,
    // Nonlinear zero-delay-feedback models (filters/FilterModel.h); appended so the indices above never move.
    TransistorLadder,
    OtaCascade,
    CoupledDiodeLadder,
    // Second family (P8): Sallen-Key with feedback clipping, OTA state-variable filter, four-pole OTA with partial
    // compensation, passive high-pass.
    SallenKeyLowpass,
    SallenKeyHighpass,
    SvfOta,
    Ota4Pole,
    PassiveHighpass,
    Count
};

const std::vector<std::string>& filterModelNames();

// A user-defined vowel table for the Formant model: 2..8 vowels of 1..4 formants. The cutoff selects the position along the
// vowels (the same law as the built-in table, spread over vowels - 1 segments) and freq / bandwidth interpolate
// geometrically, gain linearly in dB, between neighbours. bandwidthHz == 0 keeps the resonance-derived damping of the
// built-in table for that formant. Immutable once published to the audio thread.
constexpr int kMaxFormantVowels = 8;
constexpr int kMaxFormantBands = 4;
constexpr float kMaxFormantGainDb = 24.0f;

struct FormantBand
{
    float freqHz = 1000.0f;
    float bandwidthHz = 0.0f;
    float gainDb = 0.0f;
};

struct FormantTable
{
    int vowels = 0;
    int bands = 0;
    float outputGain = 1.0f;
    // State saturation like a hardware-style vowel filter: the band-pass integrator of every formant is soft-clipped (see
    // crucible_set_formant_saturation). Off (default) is the linear filter, bit for bit. Not set by makeFormantTable.
    bool saturate = false;
    std::array<FormantBand, kMaxFormantVowels * kMaxFormantBands> band{}; // vowel-major
    const FormantBand& at(int vowel, int b) const { return band[static_cast<size_t>(vowel * kMaxFormantBands + b)]; }
};

// Validates and copies vowels x bands entries (row-major per vowel) into out. False with a reason in error: counts out of
// range, freq not finite or outside 20 Hz..20 kHz, bandwidth not finite or negative, gainDb not finite or above +24,
// outputGain not finite or <= 0.
bool makeFormantTable(const FormantBand* bands, int vowels, int bandsPerVowel, float outputGain, FormantTable& out,
                      std::string& error);

struct FilterSettings
{
    int model = 0;
    float cutoffHz = 8000.0f;
    float resonance = 0.0f; // 0..1
    // Used by the nonlinear models only (index 17 and up); the legacy models ignore them.
    float drive = 0.0f;     // 0..1
    float character = 0.0f; // 0..1, gain compensation blend
    float tolerance = 0.0f; // 0..1, seeded component spread
    int highpass = 0;       // 0..3, the passive high-pass step ahead of the model (off, 60, 180, 540 Hz)
    // Formant model only: a user vowel table (null = the built-in five vowels). Must outlive the call.
    const FormantTable* formants = nullptr;
};

// Small-signal magnitude response of a filter model in dB at n frequencies (Hz): the settings the voice would use,
// nominal parts, the gain of the drive stage in front (sqrt(1 + 8 drive) for the models 0..16 with the two-sample average of
// its anti-aliasing, the model's own shaper for the others). Exact for the linear models (the state variable and ladder
// families, comb, formant, phaser, with the pole placement and gains of FilterChannel::configure), the small-signal limit of
// the nonlinear ones (the clippers, the diode and the saturating integrators at zero amplitude). A loop that self-oscillates
// is shown at 99.5 % of its critical gain. Pure arithmetic apart from a one-time cache of prepared models (guarded by a
// lock) for the analog models; it reads no voice state, so any non-audio thread may call it. quality picks the oversampling
// the voices use.
void filterResponseDb(const FilterSettings& settings, float drive, double sampleRate, Quality quality, const float* freqHz,
                      int n, float* outDb);

// First-order antiderivative anti-aliased tanh (Parker, Zavalishin, Le Bivic 2016).
class AdaaTanh
{
public:
    void reset()
    {
        x1 = 0.0f;
        f1 = 0.0f;
    }

    float process(float x)
    {
        const float f = logCosh(x);
        const float dx = x - x1;
        float y;
        if (std::abs(dx) < 1.0e-4f)
            y = std::tanh(0.5f * (x + x1));
        else
            y = (f - f1) / dx;
        x1 = x;
        f1 = f;
        return y;
    }

private:
    static float logCosh(float x)
    {
        const float a = std::abs(x);
        return a + std::log1p(std::exp(-2.0f * a)) - 0.69314718f;
    }
    float x1 = 0.0f;
    float f1 = 0.0f;
};

// One audio channel of one filter. Zero-delay-feedback topologies (state variable, ladder, one-pole
// allpass) so cutoff and resonance can be modulated at control rate without instability.
class FilterChannel
{
public:
    static constexpr int combSize = 8192;
    static constexpr int kAnalogModels = 8;

    FilterChannel() = default;
    FilterChannel(const FilterChannel& other);
    FilterChannel& operator=(const FilterChannel& other);
    FilterChannel(FilterChannel&&) noexcept = default;
    FilterChannel& operator=(FilterChannel&&) noexcept = default;

    // Control thread: allocates the nonlinear models for this rate and quality tier. configure() does the same on
    // the first use of one of them when this was not called, so call it before the audio thread runs.
    void prepare(double sampleRate, Quality quality);
    void setSeed(std::uint32_t seed);
    void reset();
    void configure(const FilterSettings& s, double sampleRate);
    // Records the model as configure() does (resetting on a change) without touching the nonlinear model objects: used
    // when a FilterBank runs the model for this channel.
    void trackModel(int modelIndex);
    float process(float x);
    // n samples in place with a per-sample cutoff modulation in octaves (relative to the last configure()): the nonlinear
    // models take it at a control rate of 4 samples, the others re-derive their coefficients every 4 samples. A null
    // cutoffOct runs the settings of the last configure() unchanged, like calling process() per sample.
    void processBlock(float* x, int n, const float* cutoffOct);

    static bool isAnalogModel(int model) { return model >= static_cast<int>(FilterModel::TransistorLadder); }

private:
    struct SvfState
    {
        float ic1 = 0.0f, ic2 = 0.0f;
    };

    float svf(SvfState& st, float x, float a1, float a2, float a3, float k, int out) const;

    FilterModel model = FilterModel::Lowpass12;

    // State variable
    float g = 0.0f, k1 = 2.0f, k2 = 2.0f;
    float a1 = 1.0f, a2 = 0.0f, a3 = 0.0f;
    float b1 = 1.0f, b2 = 0.0f, b3 = 0.0f;
    SvfState s0, s1;

    // Ladder
    float G = 0.0f, kRes = 0.0f, ladderComp = 1.0f;
    std::array<float, 4> ls{};

    // Comb
    std::array<float, combSize> comb{};
    int combPos = 0;
    float combDelay = 100.0f, combFb = 0.0f, combLp = 0.0f, combDamp = 0.0f;

    // Formant
    std::array<SvfState, kMaxFormantBands> fs;
    std::array<float, kMaxFormantBands> fa1{}, fa2{}, fa3{}, fk{}, fgain{};
    bool fsat = false;
    int fcount = 3;
    float fout = 1.4f;

    // Phaser
    std::array<float, 6> ap{};
    float apG = 0.0f, phaserFb = 0.0f, phaserLast = 0.0f;

    FilterSettings lastSettings; // what configure() was given, for processBlock()
    double lastRate = 48000.0;

    // Nonlinear models, all allocated by prepare() so selecting one never allocates on the audio thread.
    std::array<std::unique_ptr<AnalogFilter>, kAnalogModels> analog;
    double analogRate = 0.0;
    Quality analogQuality = Quality::Standard;
    std::uint32_t analogSeed = 1;
    int analogIndex = -1;
};

// A stereo filter: two channels with identical settings plus an optional drive stage.
class FilterUnit
{
public:
    void prepare(double sampleRate, Quality quality);
    void setSeed(std::uint32_t seed);
    void reset();
    void configure(const FilterSettings& s, float drive, double sampleRate);
    void trackModel(int modelIndex);
    void process(float& l, float& r);
    // n samples in place, cutoffOct: per-sample cutoff modulation in octaves (see FilterChannel::processBlock).
    void processBlock(float* l, float* r, int n, const float* cutoffOct);

private:
    std::array<FilterChannel, 2> ch;
    std::array<AdaaTanh, 2> adaa;
    float driveGain = 1.0f;
    float driveComp = 1.0f;
    bool driven = false;
};

} // namespace crucible
