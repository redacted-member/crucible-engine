// SPDX-License-Identifier: MIT
#pragma once

// The rack effects that survive the first generation (limiter, crusher, width) and the second generation: filter FX,
// multiband compressor, trance gate, tremolo / auto-pan and ring modulator / frequency shifter. Each effect follows
// the rack conventions of Fx.h: prepare() allocates, reset() clears state, process() works in place on a stereo
// block and never allocates, locks or blocks; deactivate() is called while the effect is bypassed.

#include <crucible/DelayLine.h>
#include <crucible/Filters.h>
#include <crucible/RtShared.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace crucible
{

// ---------------------------------------------------------------------------------------------------------------
// Biquad (RBJ cookbook), transposed direct form II, two channels.
enum class BiquadType
{
    Peak,
    LowShelf,
    HighShelf,
    LowCut12,
    LowCut24,
    HighCut12,
    HighCut24,
    Notch,
    BandPass,
    Count
};


struct BiquadCoefs
{
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;

    static BiquadCoefs design(BiquadType type, double freqHz, double gainDb, double q, double sampleRate);
    // |H| in dB at freqHz.
    double magnitudeDb(double freqHz, double sampleRate) const;
};

class Biquad
{
public:
    void set(const BiquadCoefs& c) { k = c; }
    void reset() { z1[0] = z1[1] = z2[0] = z2[1] = 0.0; }
    float process(int ch, float x)
    {
        const double y = k.b0 * x + z1[ch];
        z1[ch] = k.b1 * x - k.a1 * y + z2[ch];
        z2[ch] = k.b2 * x - k.a2 * y;
        return static_cast<float>(y);
    }

private:
    BiquadCoefs k;
    double z1[2] = {0.0, 0.0};
    double z2[2] = {0.0, 0.0};
};

// ---------------------------------------------------------------------------------------------------------------
// Filter FX: any of the 17 voice filter models on the bus, with drive and an LFO sweeping the cutoff.
struct FxFilterParams
{
    bool on = false;
    float mix = 1.0f;
    int model = 0;
    float cutoffHz = 2000.0f;
    float resonance = 0.3f;
    float drive = 0.0f;
    float lfoRateHz = 0.5f;
    float lfoSyncBeats = 0.0f; // > 0 overrides lfoRateHz
    float lfoDepth = 0.0f;     // octaves, 0..4
    double bpm = 120.0;
};

class FxFilter
{
public:
    void deactivate() { primed = false; }
    void prepare(double sampleRate);
    void reset();
    void process(float* l, float* r, int n, const FxFilterParams& p);

private:
    FilterUnit unit;
    float phase = 0.0f;
    float cutoffSmooth = 2000.0f;
    float mixSmooth = 1.0f;
    bool primed = false;
    double sr = 48000.0;
};

// ---------------------------------------------------------------------------------------------------------------
// Three-band compressor with Linkwitz-Riley (24 dB/oct) crossovers.
struct FxMultibandParams
{
    bool on = false;
    float mix = 1.0f;
    float lowCrossHz = 200.0f;
    float highCrossHz = 3000.0f;
    float attackMs = 10.0f;
    float releaseMs = 150.0f;
    float ratio = 4.0f;
    std::array<float, 3> thresholdDb{-24.0f, -24.0f, -24.0f};
    std::array<float, 3> gainDb{0.0f, 0.0f, 0.0f};
};

class FxMultiband
{
public:
    void deactivate() { primed = false; }
    void prepare(double sampleRate);
    void reset();
    void process(float* l, float* r, int n, const FxMultibandParams& p);

    // Current gain reduction per band in dB (>= 0), for meters (any thread).
    float gainReductionDb(int band) const { return gr[static_cast<size_t>(band)].load(std::memory_order_relaxed); }

private:
    void designLow(float hz);
    void designHigh(float hz);
    void applyLow(const BiquadCoefs& lp, const BiquadCoefs& hp);
    void applyHigh(const BiquadCoefs& lp, const BiquadCoefs& hp);
    void startRamp();
    void stepRamp();

    std::array<Biquad, 2> lowLp, lowHp, highLp, highHp; // two sections each = LR4
    // The high crossover's low-pass and high-pass again, summed: the all-pass the low band goes through so that the
    // three bands add up flat (BFX2).
    std::array<Biquad, 2> lowAllpassLp, lowAllpassHp;
    std::array<float, 3> envDb{};
    std::array<std::atomic<float>, 3> gr{};
    float lastLow = -1.0f, lastHigh = -1.0f; // the frequencies the filters are designed for
    // Gliding crossover frequencies (BFX6): two one-pole stages, so the motion starts with zero speed.
    float stageLow = 0.0f, stageHigh = 0.0f, smoothLow = 0.0f, smoothHigh = 0.0f;
    // While a crossover moves the filters are redesigned only every kRamp samples. In between, their coefficients
    // move in a straight line to the new design, one small step per sample: coefficient jumps would click, because
    // the low crossovers sit close to the unit circle (index 0: low crossover, 1: high).
    static constexpr int kRamp = 16;
    std::array<BiquadCoefs, 2> curLp, curHp, stepLp, stepHp, goalLp, goalHp;
    int rampLeft = 0; // samples
    std::array<bool, 2> rampMoves{};
    float mixSmooth = 1.0f;
    bool primed = false;
    double sr = 48000.0;
};

// ---------------------------------------------------------------------------------------------------------------
// Trance gate: a tempo-synced 16-step on/off pattern with attack / release shaped edges.
struct FxTranceGateParams
{
    bool on = false;
    float mix = 1.0f;
    float stepBeats = 0.25f;         // length of one step in quarter notes
    std::uint32_t pattern = 0xB6DBU; // bit i = step i on
    int steps = 16;
    float attackMs = 2.0f;
    float releaseMs = 20.0f;
    float depth = 1.0f;
    double bpm = 120.0;
};

const std::vector<std::string>& tranceGateRateNames();
float tranceGateRateBeats(int index);

class FxTranceGate
{
public:
    void deactivate() { primed = false; }
    void prepare(double sampleRate);
    void reset();
    void process(float* l, float* r, int n, const FxTranceGateParams& p);
    int currentStep() const { return step.load(std::memory_order_relaxed); }

private:
    double beat = 0.0;
    float level = 1.0f;
    float mixSmooth = 1.0f;
    bool primed = false;
    std::atomic<int> step{0};
    double sr = 48000.0;
};

// ---------------------------------------------------------------------------------------------------------------
// Tremolo / auto-pan: amplitude LFO per channel; a stereo phase of 180 degrees turns it into an auto-pan.
struct FxTremoloParams
{
    bool on = false;
    float rateHz = 4.0f;
    float syncBeats = 0.0f;
    float depth = 0.5f;
    int shape = 0; // sine, triangle, square, saw up, saw down
    float stereoPhaseDeg = 0.0f;
    double bpm = 120.0;
};

const std::vector<std::string>& tremoloShapeNames();

class FxTremolo
{
public:
    void deactivate() {}
    void prepare(double sampleRate);
    void reset();
    void process(float* l, float* r, int n, const FxTremoloParams& p);

private:
    float phase = 0.0f;
    std::array<float, 2> smooth{1.0f, 1.0f};
    double sr = 48000.0;
};

// ---------------------------------------------------------------------------------------------------------------
// Ring modulator and single-sideband frequency shifter (Hilbert transformer from two allpass chains).
struct FxRingShiftParams
{
    bool on = false;
    float mix = 0.5f;
    int mode = 0; // 0 ring, 1 shift up, 2 shift down
    float freqHz = 200.0f;
    float spread = 0.0f; // 0..1, right channel offset up to +/- 25 % of the frequency
};

class FxRingShift
{
public:
    void deactivate() { primed = false; }
    void prepare(double sampleRate);
    void reset();
    void process(float* l, float* r, int n, const FxRingShiftParams& p);

private:
    // The Hilbert pair: two chains of all-pass sections in z^-2 (coefficient c each) whose outputs stay 90 degrees
    // apart from about 20 Hz to 20 kHz. The chains are designed for the sample rate in prepare() (BFX7).
    static constexpr int kMaxSections = 6;
    struct HilbertCoefs
    {
        int sections = 4;
        std::array<double, kMaxSections> a{}, b{}; // c = coefficient squared, path A and path B
    };
    struct Hilbert
    {
        std::array<double, kMaxSections> ax1{}, ax2{}, ay1{}, ay2{};
        std::array<double, kMaxSections> bx1{}, bx2{}, by1{}, by2{};
        double bDelay = 0.0; // one-sample delay of path A
        void reset();
        void process(double x, double& re, double& im, const HilbertCoefs& k);
    };
    static HilbertCoefs designHilbert(double sampleRate);

    HilbertCoefs coefs = designHilbert(48000.0);
    std::array<Hilbert, 2> hilbert;
    std::array<double, 2> phase{};
    float mixSmooth = 0.5f;
    bool primed = false;
    double sr = 48000.0;
};

} // namespace crucible
