// SPDX-License-Identifier: MIT
#pragma once

// Dynamics core shared by the clean compressor, the VCA bus compressor and the expander / gate / ducker
// (docs/plans/ENGINE-FX-PLAN.md, section 3.2). One processor, five gain computers:
//
//   Compressor  downward compression: above the threshold the output rises by 1 / ratio dB per dB of input
//   Upward      upward compression: below the threshold quiet signals are lifted toward it
//   Expander    below the threshold the output falls by `ratio` dB per dB of input (1:1 to 1:inf)
//   Gate        an expander behind a Schmitt trigger with hysteresis and hold (opens at the threshold, closes
//               `hysteresisDb` lower after the hold time)
//   Ducker      the key level above the threshold pulls the gain down toward -rangeDb (same trigger as the gate)
//
// Static curves are the quadratic soft-knee computers of Giannoulis, Massberg and Reiss, "Digital Dynamic Range
// Compressor Design: A Tutorial and Analysis", JAES 60(6), 2012, extended to the other kinds by mirroring.
//
// Signal path per sample: key (the input, or the host's auxiliary input) -> sidechain high-pass / low-pass / bell ->
// detector (peak, RMS or both, optionally a 4x true-peak reading) -> level in dB -> stereo link -> gain computer ->
// ballistics in the dB domain (attack and release are the times to 63 % of a step of gain, hold, optional
// program-dependent release from a fast and a slow stage) -> linear gain applied to the audio, which is delayed by the
// lookahead so that the gain arrives before the signal. Feed-forward by default; feedback detects the output.
//
// Channel modes (ChannelMode.h) run the whole path on left / right or mid / side lanes. The processor reports its latency
// (lookahead plus the true-peak interpolator) and the current gain reduction for meters. Out-of-line members live in
// src/fx/Dynamics.cpp; prepare() allocates, everything else is audio-thread safe.

#include <crucible/fx/ChannelMode.h>
#include <crucible/fx/Detectors.h>
#include <crucible/fx/Lookahead.h>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <vector>

namespace crucible::fx
{

// Largest value of the last `window` pushed values, in three simple operations per value (the block decomposition of van
// Herk and of Gil and Werman: running maximum of the current block, suffix maxima of the previous one). Unlike the monotonic
// deque of RunningMax the cost does not depend on the data. Changing the window rebuilds from a ring of the raw values, so
// the reading stays exact.
class HoldMax
{
public:
    void prepare(int maxWindow);
    void reset();
    void setWindow(int w);
    int window() const { return size; }
    float push(float x)
    {
        ring[ringPos & ringMask] = x;
        ++ringPos;
        current[static_cast<size_t>(pos)] = x;
        prefix = pos == 0 ? x : std::max(prefix, x);
        const float result = std::max(prefix, suffix[static_cast<size_t>(pos) + 1]);
        if (++pos == size)
            closeBlock();
        return result;
    }

private:
    void closeBlock();
    void rebuild();
    std::vector<float> ring, current, suffix; // suffix[j] = max of the previous block from j on; suffix[size] = 0
    size_t ringPos = 0, ringMask = 0;
    int size = 1, pos = 0, capacity = 1;
    float prefix = 0.0f;
};

enum class DynamicsKind : int
{
    Compressor = 0,
    Upward,
    Expander,
    Gate,
    Ducker
};

enum class DetectorMode : int
{
    Peak = 0, // largest sample of the last peakHoldMs (instant attack, then a hard window); a sine reads its amplitude
    Rms,      // true RMS, two cascaded one-pole power averages; a sine reads its amplitude
    Hybrid    // geometric mean of the two: follows transients like a peak detector, sustained material like RMS
};

// ---------------------------------------------------------------------------------------------------------------
// Static gain curves. All in dB. levelDb is the detector reading in dB relative to a full-scale sine.
namespace dyn
{
// Quadratic soft knee of width kneeDb centred on the threshold. `excess` is the distance past the threshold in the
// direction that is processed (positive: processed); the result is the amount of gain change in dB (>= 0), with
// `slope` dB of change per dB of excess beyond the knee.
inline float kneeAmount(float excess, float kneeDb, float slope)
{
    if (kneeDb <= 0.0f)
        return excess > 0.0f ? slope * excess : 0.0f;
    const float half = 0.5f * kneeDb;
    if (excess <= -half)
        return 0.0f;
    if (excess >= half)
        return slope * excess;
    const float v = excess + half;
    return slope * v * v / (2.0f * kneeDb);
}

// Gain reduction (dB, >= 0) of a downward compressor.
inline float compressorReduction(float levelDb, float thresholdDb, float ratio, float kneeDb)
{
    return kneeAmount(levelDb - thresholdDb, kneeDb, 1.0f - 1.0f / std::max(ratio, 1.0f));
}
// Gain lift (dB, >= 0) of an upward compressor.
inline float upwardLift(float levelDb, float thresholdDb, float ratio, float kneeDb)
{
    return kneeAmount(thresholdDb - levelDb, kneeDb, 1.0f - 1.0f / std::max(ratio, 1.0f));
}
// Gain reduction (dB, >= 0) of an expander with expansion ratio `ratio` (1: none, 1000: a gate).
inline float expanderReduction(float levelDb, float thresholdDb, float ratio, float kneeDb)
{
    return kneeAmount(thresholdDb - levelDb, kneeDb, std::max(ratio, 1.0f) - 1.0f);
}

// log2 and exp2 for the gain path: absolute error below 1e-6 (log2) and 2e-7 relative (exp2), a few times faster than
// libm. Arguments outside the normal float range are clamped.
inline float log2Fast(float x)
{
    const std::uint32_t bits = std::bit_cast<std::uint32_t>(std::max(x, 1.0e-30f));
    int e = static_cast<int>((bits >> 23) & 0xff) - 127;
    float m = std::bit_cast<float>((bits & 0x007fffffu) | 0x3f800000u); // [1, 2)
    if (m > 1.41421356f)
    {
        m *= 0.5f;
        ++e;
    }
    const float t = (m - 1.0f) / (m + 1.0f), t2 = t * t;
    return static_cast<float>(e) + 2.88539008f * t * (1.0f + t2 * (1.0f / 3.0f + t2 * (0.2f + t2 * (1.0f / 7.0f))));
}
inline float exp2Fast(float x)
{
    x = std::clamp(x, -120.0f, 120.0f);
    const float r = std::nearbyint(x);
    const float u = (x - r) * 0.693147181f;
    const float p =
        1.0f + u * (1.0f + u * (0.5f + u * (1.0f / 6.0f + u * (1.0f / 24.0f + u * (1.0f / 120.0f + u * (1.0f / 720.0f))))));
    const std::uint32_t scale = static_cast<std::uint32_t>(static_cast<int>(r) + 127) << 23;
    return p * std::bit_cast<float>(scale);
}
// NaN and values below -180 dB read as -180 dB.
inline float powerToDb(float power)
{
    return 3.01029996f * log2Fast(power > 1.0e-18f ? std::min(power, 1.0e18f) : 1.0e-18f);
}
inline float dbToGain(float db)
{
    return exp2Fast(db * 0.166096405f);
}
} // namespace dyn

// ---------------------------------------------------------------------------------------------------------------
struct DynamicsSettings
{
    DynamicsKind kind = DynamicsKind::Compressor;

    // Gain computer.
    float thresholdDb = -18.0f;
    float ratio = 4.0f;        // compression / expansion ratio, >= 1 (expansion: 1000 and above is a gate)
    float kneeDb = 6.0f;       // total width of the soft knee
    float rangeDb = 60.0f;     // largest gain change in either direction
    float hysteresisDb = 4.0f; // Gate and Ducker: the trigger closes this far below the threshold

    // Detector.
    DetectorMode detector = DetectorMode::Peak;
    bool truePeak = false;     // peak readings from a 4x interpolated signal (adds its latency)
    bool feedback = false;     // detect the output instead of the input (ignored with an external key)
    float rmsWindowMs = 10.0f; // time constant of each of the two power averages is half of this
    float peakHoldMs = 2.0f;   // window of the peak detector; at least half a period of the lowest frequency to follow

    // Ballistics: times to 63 % of a step of gain. Compressor kinds: attack is the response to more gain change, release
    // the recovery. Expander, Gate: attack is the opening (gain rising), release the closing.
    float attackMs = 10.0f;
    float releaseMs = 100.0f;
    float holdMs = 0.0f;
    bool autoRelease = false;    // dual-stage program-dependent release (compressor kinds)
    float autoFastMs = 60.0f;    // fast stage release
    float autoSlowMs = 1200.0f;  // slow stage release
    float autoChargeMs = 700.0f; // time the slow stage takes to charge (63 %)

    // Sidechain filters on the key. 0 switches a filter off.
    float scHighpassHz = 0.0f;
    float scLowpassHz = 0.0f;
    float scBellHz = 1000.0f;
    float scBellDb = 0.0f; // 0: off
    float scBellQ = 1.0f;
    bool listen = false; // the output is the filtered key

    // Stereo.
    ChannelMode channels = ChannelMode::StereoLinked;
    float link = 1.0f; // used by Mid, Side and Mid / Side (and unlinked-to-linked blends)

    // Output.
    float lookaheadMs = 0.0f;
    float makeupDb = 0.0f;
    bool autoMakeup = false; // adds the gain reduction the static curve has at 0 dBFS (compressors)
    float mix = 1.0f;        // parallel: dry is the latency-aligned input
    float cellColour = 0.0f; // 0: clean gain cell; 1: a VCA-like cell whose third harmonic grows with gain reduction
};

class DynamicsProcessor
{
public:
    static constexpr float kMaxLookaheadMs = 10.0f;

    // Control thread.
    void prepare(double sampleRate);
    void reset();

    // In place on left / right. keyLeft / keyRight: the auxiliary input (null for none; a null right equals the left); they
    // drive the detector when externalKey is true. Blocks of any length.
    void process(float* left, float* right, int n, const float* keyLeft, const float* keyRight, const DynamicsSettings& s,
                 bool externalKey = false);

    // Whole samples between input and output at the settings last processed.
    int latencySamples() const { return latency.load(std::memory_order_relaxed); }
    // True once after the latency changed (control thread).
    bool takeLatencyChanged() { return latencyChanged.exchange(false, std::memory_order_relaxed); }
    // Gain reduction in dB (positive: the signal is turned down, negative: up) of the loudest lane, largest value of
    // the last block.
    float gainReductionDb() const { return meter.load(std::memory_order_relaxed); }
    // Latency the settings would give; lets a module report it before audio runs.
    int latencyFor(const DynamicsSettings& s) const;
    // True when, at the end of the last block, every smoothed control (gain computer, makeup, mix, sidechain filter
    // corners) sits exactly on its target: the static, bit-identical path is in use.
    bool controlsSettled() const { return settledAtEnd; }

private:
    static constexpr int kChunk = 64;

    struct Svf
    {
        float ic1 = 0.0f, ic2 = 0.0f;
        void clear() { ic1 = ic2 = 0.0f; }
    };
    struct SvfCoefficients
    {
        float a1 = 0.0f, a2 = 0.0f, a3 = 0.0f, k = 1.4142135f;
        void design(double sampleRate, float hz, float q);
    };
    struct Lane
    {
        Svf hp, lp, bell;
        TruePeakDetector truePeak;
        HoldMax peakWindow;
        float power1 = 0.0f, power2 = 0.0f;
        float reduction = 0.0f, slow = 0.0f; // ballistics state (dB of gain change)
        int hold = 0;
        bool triggered = false;
        int triggerHold = 0;
        float lastAmount = 0.0f;
        float feedbackSample = 0.0f;
        void clear();
    };
    struct Derived
    {
        float attack = 0.0f, release = 0.0f, slowRelease = 0.0f, slowCharge = 0.0f; // one-pole coefficients
        float rms = 0.0f;
        int peakWindow = 1;
        int holdSamples = 0;
    };

    // What one process() call does, decided once from the settings.
    struct Plan
    {
        bool active0 = true, active1 = true, fullyLinked = true;
        bool highpass = false, lowpass = false, bell = false, truePeak = false;
        DetectorMode detector = DetectorMode::Peak;
        float link = 1.0f, bellGain = 0.0f, sign = -1.0f;
    };

    void configure(const DynamicsSettings& s);
    float detect(Lane& lane, float key, DetectorMode mode, bool truePeak);
    void filterStage(float* k0, float* k1, int m, const Plan& plan);
    void detectStage(const float* k0, const float* k1, int m, float* p0, float* p1, const Plan& plan);
    void gainStage(const float* p0, const float* p1, int m, float* g0, float* g1, float& maxAmount,
                   const DynamicsSettings& s, const Plan& plan);
    // Two cascaded one-poles: a settings step becomes a ramp with a continuous slope. Equals the target while it is
    // constant (the second stage is the output).
    struct Smooth2
    {
        float a = 0.0f, b = 0.0f;
        void set(float v) { a = b = v; }
        bool settled(float target) const { return a == target && b == target; }
        float step(float target, float coef);
    };
    // Gain-computer inputs, smoothed per sample towards the block's settings.
    struct GainComputer
    {
        Smooth2 threshold, ratio, knee, range;
    };
    void advanceGainComputer(const DynamicsSettings& s);
    bool gainComputerSettled(const DynamicsSettings& s) const;
    float targetAmount(Lane& lane, float levelDb, const DynamicsSettings& s) const;
    float ballistics(Lane& lane, float target, const DynamicsSettings& s) const;

    double sr = 48000.0;
    Lookahead look;
    Lane lane[2];
    SvfCoefficients hpCoef, lpCoef, bellCoef;
    // Sidechain filter corners glide (log frequency, per chunk) so a moving control never steps the coefficients.
    Smooth2 scHpLog, scLpLog, scBellLog;
    float designedHp = 0.0f, designedLp = 0.0f, designedBell = 0.0f, designedBellQ = 0.0f;
    void advanceSidechain(const DynamicsSettings& s, int m, bool first, Plan& plan);
    DynamicsSettings applied;
    Derived d;
    bool configured = false;
    bool latencyKnown = false; // a latency has been reported since prepare(); reset() keeps it
    int primed = 0;            // samples written to the delay line since the last reset
    bool writing = false;
    float makeupLinear = 1.0f, mixValue = 1.0f;
    Smooth2 makeupSmooth, mixSmooth;
    GainComputer gc;
    bool settledAtEnd = true;
    float smoothCoef = 1.0f, ratioCoef = 1.0f;
    std::atomic<int> latency{0};
    std::atomic<bool> latencyChanged{false};
    std::atomic<float> meter{0.0f};
};

} // namespace crucible::fx
