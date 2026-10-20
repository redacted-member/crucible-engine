// SPDX-License-Identifier: MIT
#pragma once

// Lookahead true-peak limiter and maximiser (docs/plans/ENGINE-FX-PLAN.md, section 3.2).
//
// TruePeakLimiter, per channel and per sample:
//
//   input gain -> optional soft-clip stage (ADAA, a set share of the peak reduction) -> detector -> overshoot over the
//   ceiling in dB -> stereo link -> running maximum over the hold window -> two-stage release -> box average over the
//   lookahead -> gain applied to the audio, which is delayed by the lookahead so that the gain has arrived when the peak
//   does.
//
// The structure guarantees the ceiling by construction (the min-hold plus box-filter limiter of the published lookahead
// designs): the hold window is at least as long as the box, so every one of the box's taps contains the overshoot of a
// peak that is `lookahead - 1` samples ahead of the audio, and the box average of values that all reach that overshoot
// reaches it too. The release only ever raises the held value. Work is done in dB, where the average of the held
// reduction is the geometric mean of gains and still never falls below the peak's requirement.
//
//   (Maximizer's 4x path keeps the passband gain of its up / down filters, measured at prepare(), below the ceiling as
//   well.) detector        TruePeak: 4x polyphase interpolation of ITU-R BS.1770 Annex 2 (TruePeakDetector, 32 taps per
//   phase and
//                   parabolic refinement) on the sidechain only, evaluated only where the newest samples can reach the
//                   ceiling; Refined: sample peaks with a parabolic vertex, for audio that has been oversampled already
//   hold            at least the lookahead; with autoHold also half a period of the bass tone the sidechain follows (a
//                   running maximum over half a period of a sine is flat, so a 60 Hz tone is not amplitude-modulated at
//                   120 Hz, which is where the legacy limiter's distortion came from)
//   release         Fixed: one exponential stage. Dual: a fast stage (the release time) and a slow stage (six times as
//                   long, charging in four release times), so transients recover quickly and sustained reduction slowly.
//                   Adaptive: Dual with all times scaled by the crest factor of the input (2^((8 - crest dB) / 4),
//                   0.25 to 4: dense, flat material releases slowly, peaky material quickly)
//   stereo          link 1: one shared gain; below 1 each channel moves toward the larger reduction by `link`. sideBias:
//                   one gain g for the mid channel and g^2 for the side channel (the side is turned down more and the
//                   ceiling still holds because |g M +- g^2 S| <= g (|M| + |S|) = g max(|L|, |R|))
//
// The output is clamped to the ceiling as the last operation, so sample peaks never exceed it.
//
// Maximizer adds the optional 4x oversampled audio path (the whole limiter runs at four times the rate with the Refined
// detector, and the output is filtered back down) and dither. Latency is reported in whole samples; prepare() allocates,
// everything else is audio-thread safe.

#include <crucible/fx/Clipper.h>
#include <crucible/fx/Detectors.h>
#include <crucible/fx/Dynamics.h>
#include <crucible/fx/Lookahead.h>
#include <crucible/fx/OversampledBlock.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

namespace crucible::fx
{

enum class ReleaseMode : int
{
    Fixed = 0,
    Dual,
    Adaptive
};

enum class DitherMode : int
{
    Off = 0,
    Tpdf,
    Shaped
};

struct LimiterSettings
{
    float gainDb = 0.0f;
    float ceilingDb = -1.0f;
    float lookaheadMs = 3.0f; // 0.5 to 10
    float releaseMs = 150.0f;
    ReleaseMode releaseMode = ReleaseMode::Adaptive;
    float holdMs = 8.0f; // 0 to 50
    bool autoHold = true;
    float link = 1.0f; // 0 to 1
    bool sideBias = false;
    float clipShare = 0.0f;    // 0: the soft-clip stage is off; 1: it clips at the ceiling; between: up to 6 dB above it
    float clipSoftness = 0.5f; // knee of the clip stage, 0 hard to 1 tanh
    int clipOrder = 0; // ADAA order of the clip stage (0: the plain curve; ADAA low-passes the linear part, see Clipper.h)
};

class TruePeakLimiter
{
public:
    static constexpr float kMaxLookaheadMs = 10.0f;
    static constexpr float kMaxHoldMs = 50.0f;
    static constexpr int kChunk = 64;
    // The detector's reading is lowered by this much (the ceiling is held this far below the set value inside the gain
    // computer) because no short interpolator sees every inter-sample peak of hard-clipped, wide-band material.
    static constexpr float kDetectorMarginDb = 0.05f;

    enum class Detect
    {
        TruePeak,
        Refined
    };

    // Control thread. granularity: the lookahead (and so the audio delay) is rounded up to a multiple of it, which lets an
    // oversampled instance delay by a whole number of base-rate samples.
    // tapsPerPhase: length of the true-peak interpolator per phase (32 for a limiter; 12 is the BS.1770 filter, enough for a
    // guard).
    void prepare(double sampleRate, Detect detect, int granularity = 1, int tapsPerPhase = 32);
    void reset();
    // Extra headroom (dB, >= 0) the gain computer keeps below the ceiling: for a limiter whose output goes through a filter
    // that is not flat (the 4x path). Control thread.
    void setExtraMarginDb(float db) { extraMarginDb = db < 0.0f ? 0.0f : db; }

    // In place. Blocks of any length.
    void process(float* left, float* right, int n, const LimiterSettings& s);

    // Whole samples between input and output at the settings last processed (or given).
    int latencySamples() const { return latency.load(std::memory_order_relaxed); }
    int latencyFor(const LimiterSettings& s) const;
    bool takeLatencyChanged() { return latencyChanged.exchange(false, std::memory_order_relaxed); }
    // Largest gain reduction (dB, >= 0) of the last block.
    float gainReductionDb() const { return meter.load(std::memory_order_relaxed); }
    // True when gain, ceiling, clip share and clip fade all sit exactly on their targets (the static path is in use).
    bool controlsSettled() const
    {
        return !ceilMoving && !shareMoving && !fadeMoving && gainStage1 == gainLinear && gainStage2 == gainLinear &&
               gainSmoothed == gainLinear;
    }

private:
    struct Lane
    {
        TruePeakDetector truePeak;
        HoldMax hold;
        BoxSum box;
        AdaaClipper clip;
        float m1 = 0.0f, m2 = 0.0f;             // refined detector history
        float groupMax[3] = {0.0f, 0.0f, 0.0f}; // largest magnitude of the current and the two previous groups of samples
        int groupFill = 0;
        float fast = 0.0f, slow = 0.0f;
        void clear();
    };

    int lookSamples(const LimiterSettings& s) const;
    void configure(const LimiterSettings& s);
    float detect(Lane& lane, float x);
    float advance(Lane& lane, float amount, ReleaseMode mode);
    void updateTimes(const LimiterSettings& s);

    double sr = 48000.0;
    Detect detector = Detect::TruePeak;
    int granularity = 1, detectLatency = 6;
    Lookahead delayLine;
    Lane lane[2];
    LimiterSettings applied;
    bool configured = false;
    int look = 2, holdWindow = 2, maxLook = 2;
    float ceilingLinear = 0.891f, log2Ceiling = 0.0f, gainLinear = 1.0f, gainSmoothed = 1.0f, clipCeiling = 1.0f;
    bool clipOn = false, primed = false;
    int clipOrder = 0;
    // Release coefficients (per sample) for the current crest scaling.
    float decayFast = 0.999f, decaySlow = 0.9999f, chargeSlow = 0.001f;
    float crestScale = 1.0f, crestPeak = 0.0f, crestPower = 0.0f, crestPeakDecay = 0.999f, crestPowerCoef = 0.001f;
    int stride = 2, strideCount = 0;
    bool wasLinked = true;
    // Bass period follower for autoHold.
    float lowpassA = 0.0f, lowpassB = 0.0f, lowpassCoef = 0.04f, bassEnv = 0.0f, bassEnvDecay = 0.9999f;
    bool armed = false, haveCross = false;
    int sinceCross = 0;
    float period = 0.0f;
    int autoWindow = 0;
    float gainCoef = 0.01f, extraMarginDb = 0.0f;
    // Smoothed controls (two one-poles in cascade, per sample). The ceiling is detected with the smoothed value and the
    // output is clamped with the same value delayed like the audio, so every sample meets the ceiling it was detected at.
    float gainStage1 = 1.0f, gainStage2 = 1.0f;
    float ceilStage1 = 0.891f, ceilSmoothed = 0.891f, ceilCoef = 0.01f;
    float shareTarget = 0.0f, shareStage1 = 0.0f, shareSmoothed = 0.0f;
    float fadeStage1 = 0.0f, fadeSmoothed = 0.0f; // 0 / 1: the clip stage crossfades in and out
    bool ceilMoving = false, shareMoving = false, fadeMoving = false;
    std::vector<float> ceilRing;
    size_t ceilMask = 0, ceilPos = 0;
    std::array<float, kChunk> ceilBuf{};
    void updateLog2Ceiling();
    void updateClipCeiling();

    std::array<float, kChunk> bufL{}, bufR{}, gainA{}, gainB{};
    std::atomic<int> latency{0};
    std::atomic<bool> latencyChanged{false};
    std::atomic<float> meter{0.0f};
};

// ---------------------------------------------------------------------------------------------------------------
struct MaximizerSettings
{
    LimiterSettings limiter;
    bool oversample = false; // limit at four times the rate and filter back down
    DitherMode dither = DitherMode::Off;
    int ditherBits = 16;
};

class Maximizer
{
public:
    // Control thread. quality / filter choose the oversampling filters of the 4x path.
    void prepare(double sampleRate, int maxBlock, Quality quality, OversamplingFilter filter, std::uint32_t seed = 1);
    void reset();

    void process(float* left, float* right, int n, const MaximizerSettings& s);

    int latencySamples() const { return latency.load(std::memory_order_relaxed); }
    int latencyFor(const MaximizerSettings& s) const;
    bool takeLatencyChanged() { return latencyChanged.exchange(false, std::memory_order_relaxed); }
    float gainReductionDb() const { return meter.load(std::memory_order_relaxed); }

private:
    void applyDither(float* l, float* r, int n, const MaximizerSettings& s);
    float measureFilterGainDb(int maxBlock);

    double sr = 48000.0;
    TruePeakLimiter direct, over;
    OversampledBlock block;
    bool usingOver = false, started = false;
    std::uint32_t rng = 1, seed = 1;
    float errorL[2] = {0.0f, 0.0f}, errorR[2] = {0.0f, 0.0f};
    float filterGainDb = 0.0f;
    std::atomic<int> latency{0};
    std::atomic<bool> latencyChanged{false};
    std::atomic<float> meter{0.0f};
};

} // namespace crucible::fx
