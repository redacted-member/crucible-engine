// SPDX-License-Identifier: MIT
#pragma once

// Loudness and level metering (docs/plans/ENGINE-FX-PLAN.md, sections 3.8 and 4.5), the one implementation behind the
// master analyzer, the maximiser's readout and the meter module:
//
//   KWeighting      ITU-R BS.1770-4 pre-filter (high shelf, then the RLB high-pass) for any sample rate, from the analog
//                   prototypes of the recommendation through the bilinear transform; at 48 kHz it reproduces the
//                   coefficients printed in the recommendation
//   LoudnessMeter   momentary (400 ms), short-term (3 s) and integrated loudness with the absolute (-70 LUFS) and relative
//                   (-10 LU) gates, loudness range (EBU Tech 3342: short-term values, absolute gate -70 LUFS, relative gate
//                   -20 LU, 10th to 95th percentile), sample peak and true peak per channel (4x interpolation of
//                   Annex 2 through TruePeakDetector)
//   PpmMeter        quasi-peak programme meter with IEC 60268-10 style ballistics (integration time, linear return)
//   VuMeter         rectified-average meter with IEC 60268-17 style ballistics (300 ms rise, 1.5 % overshoot)
//   CorrelationMeter  stereo correlation over an exponential window
//
// Time is kept on a grid of 10 ms sub-blocks (the rate-dependent length is rounded sample by sample, so 44.1 kHz and
// 22.05 kHz stay exact on average). A 400 ms window is 40 sub-blocks, so the momentary reading moves every 10 ms; the
// gating blocks of the recommendation start every 100 ms (every tenth sub-block) with 75 % overlap, the first one ending
// at 400 ms. The two gated statistics keep a histogram of block loudness in 0.01 LU bins that holds the exact energy of
// each bin, so a gate decision is off by at most one bin and the means are exact. Nothing allocates after prepare().

#include <crucible/fx/Detectors.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace crucible::fx
{

constexpr float kLoudnessFloorLufs = -120.0f;

// Loudness of a K-weighted mean square (sum over channels, weights applied), -120 for silence.
inline double lufsOfMeanSquare(double meanSquare)
{
    const double v = meanSquare > 1.0e-13 ? -0.691 + 10.0 * std::log10(meanSquare) : -120.0;
    return v < -120.0 ? -120.0 : v;
}

struct KWeightingCoefficients
{
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
};

// Both stages for a sample rate. shelf: head-related high shelf; highpass: the RLB weighting.
void designKWeighting(double sampleRate, KWeightingCoefficients& shelf, KWeightingCoefficients& highpass);

class KWeighting
{
public:
    void prepare(double sampleRate);
    void reset() { s1 = s2 = h1 = h2 = 0.0; }
    double process(double x)
    {
        const double y = shelf.b0 * x + s1;
        s1 = shelf.b1 * x - shelf.a1 * y + s2;
        s2 = shelf.b2 * x - shelf.a2 * y;
        const double z = hp.b0 * y + h1;
        h1 = hp.b1 * y - hp.a1 * z + h2;
        h2 = hp.b2 * y - hp.a2 * z;
        return z;
    }

    // Adds the squares of the weighted n samples of x (non-finite or beyond +-1e18 count as 0) to sum, one after the other,
    // and returns the total, so a sum carried over from the previous call is the same as if the samples came in one piece.
    // The loop keeps the state in registers; the two-channel overload runs two independent filter chains side by side.
    double energy(const float* x, int n, double sum);
    static void energy(KWeighting& a, const float* xa, KWeighting& b, const float* xb, int n, double& sumA, double& sumB);

private:
    KWeightingCoefficients shelf, hp;
    double s1 = 0, s2 = 0, h1 = 0, h2 = 0;
};

// ---------------------------------------------------------------------------------------------------------------
class LoudnessMeter
{
public:
    static constexpr int kMaxChannels = 6;

    // Control thread (allocates). channels 1..6; truePeak = false skips the interpolators (the loudness stays exact).
    void prepare(double sampleRate, int numChannels, bool truePeak = true);
    // Back to the state after prepare().
    void reset();
    // Forgets the integrated loudness, the loudness range and the maxima, keeps the running windows.
    void resetStatistics();
    // BS.1770 channel weights (1 for left, right and centre; 1.41 for the surround channels; 0 for LFE). Default 1.
    void setChannelWeight(int channel, double weight);

    // Audio thread. in[c] holds n samples of channel c.
    void process(const float* const* in, int n);
    void process(const float* left, const float* right, int n)
    {
        const float* chans[2] = {left, right};
        process(chans, n);
    }

    int channels() const { return numChannels; }
    // Readings in LUFS (-120 when there is nothing to report). Momentary and short-term use whatever has been received
    // while their window is still filling; the maxima count full windows only.
    float momentaryLufs() const { return momentary; }
    float shortTermLufs() const { return shortTerm; }
    float integratedLufs() const { return integrated; }
    float maxMomentaryLufs() const { return momentaryMax; }
    float maxShortTermLufs() const { return shortTermMax; }
    // Loudness range in LU and the 10th and 95th percentile levels it is the difference of (LUFS).
    float loudnessRange() const { return rangeLu; }
    float rangeLowLufs() const { return rangeLow; }
    float rangeHighLufs() const { return rangeHigh; }
    // Since reset: largest sample and largest interpolated magnitude (linear, one channel).
    float samplePeak(int channel) const { return peaks[static_cast<size_t>(channel)]; }
    float truePeak(int channel) const { return tpMax[static_cast<size_t>(channel)]; }
    float truePeakDb() const; // all channels
    float samplePeakDb() const;
    // Largest true peak seen by the most recent process() call (linear, all channels).
    float blockTruePeak() const { return blockTp; }
    float blockSamplePeak() const { return blockPeak; }
    // True peak with a 1.5 s release (linear, all channels): the reading of a peak meter. Samples that cannot reach it are
    // not interpolated, so it is exact above the decaying value and a lower bound of nothing below it.
    float decayingTruePeak() const { return std::max(hold[0], hold[1]); }
    float decayingTruePeak(int channel) const { return hold[static_cast<size_t>(channel)]; }
    // Samples between a true-peak reading and the input it belongs to.
    int truePeakLatency() const { return tp[0].latencySamples(); }
    // 10 ms sub-blocks received since reset.
    std::uint64_t subBlocks() const { return subBlockCount; }

private:
    static constexpr int kMomentaryBlocks = 40;
    static constexpr int kShortTermBlocks = 300;
    static constexpr int kBins = 8000; // -70 to +10 LUFS in 0.01 LU
    static constexpr double kBinWidth = 0.01;
    static constexpr double kBinLow = -70.0;

    struct Histogram
    {
        std::vector<std::uint32_t> count;
        std::vector<double> energy;
        std::uint64_t total = 0;
        double energySum = 0.0;
        void prepare()
        {
            count.assign(kBins, 0);
            energy.assign(kBins, 0.0);
            clear();
        }
        void clear();
        void add(double meanSquare);
    };

    void closeSubBlock();
    void updateIntegrated();
    void updateRange();
    double windowMeanSquare(int blocks) const;
    static int binOf(double lufs);

    double sr = 48000.0;
    int numChannels = 2;
    bool wantTruePeak = true;
    std::array<KWeighting, kMaxChannels> kfilter;
    std::array<double, kMaxChannels> weight{};
    std::array<TruePeakDetector, kMaxChannels> tp;
    std::array<float, kMaxChannels> peaks{}, tpMax{}, hold{};
    std::array<std::array<float, 3>, kMaxChannels> groupMax{};
    int groupCount = 0;
    float holdDecay = 0.99999f, bound = 1.5f;

    // Sub-block grid.
    double nextEdge = 0.0, subLength = 480.0, position = 0.0;
    double subSum = 0.0;
    std::array<double, kMaxChannels> channelSum{}; // K-weighted energy of the open sub-block, per channel
    int subCount = 0;
    std::array<double, kShortTermBlocks> ringSum{};
    std::array<int, kShortTermBlocks> ringCount{};
    int ringHead = 0;
    std::uint64_t subBlockCount = 0;

    Histogram gated, ranged;
    float momentary = kLoudnessFloorLufs, shortTerm = kLoudnessFloorLufs, integrated = kLoudnessFloorLufs;
    float momentaryMax = kLoudnessFloorLufs, shortTermMax = kLoudnessFloorLufs;
    float rangeLu = 0.0f, rangeLow = kLoudnessFloorLufs, rangeHigh = kLoudnessFloorLufs;
    float blockTp = 0.0f, blockPeak = 0.0f;
};

// ---------------------------------------------------------------------------------------------------------------
// Programme meters. Readings are dB relative to a full-scale sine's amplitude (a sine of amplitude A reads 20 log10 A once
// settled), -120 for silence.

enum class PpmType : int
{
    TypeI = 0, // return 20 dB in 1.5 s (DIN 45406 / IEC 60268-10 type I)
    TypeII     // return 24 dB in 2.8 s (BBC / IEC 60268-10 type II)
};

// Quasi-peak programme meter: the rectified input is held for 0.5 to 1 ms (so a steady tone reads its peak, not the ripple
// of the rectifier), rises through a first-order stage of about 5 ms (a 10 ms tone burst reads 1 dB below a steady tone) and
// falls at a constant number of dB per second.
class PpmMeter
{
public:
    void prepare(double sampleRate, PpmType type = PpmType::TypeI);
    void reset()
    {
        level = 0.0;
        groupMax = previousMax = 0.0f;
        groupFill = 0;
    }
    void process(float x)
    {
        groupMax = std::max(groupMax, std::abs(x));
        if (++groupFill >= groupLength)
        {
            groupFill = 0;
            previousMax = groupMax;
            groupMax = 0.0f;
        }
        const double a = std::max(groupMax, previousMax);
        if (a > level)
            level += rise * (a - level);
        else
            level *= fall;
        if (level < 1.0e-9)
            level = 0.0;
    }
    float levelDb() const { return level > 1.0e-6 ? static_cast<float>(20.0 * std::log10(level)) : -120.0f; }

private:
    float groupMax = 0.0f, previousMax = 0.0f;
    int groupLength = 24, groupFill = 0;
    double rise = 0.1, fall = 0.9999, level = 0.0;
};

class VuMeter
{
public:
    void prepare(double sampleRate);
    void reset() { z1 = z2 = last = 0.0; }
    // Full-wave rectified average through a second-order system; a steady sine settles at its amplitude.
    void process(float x)
    {
        const double v = std::abs(static_cast<double>(x)) * 1.5707963267948966;
        const double y = b0 * v + z1;
        z1 = b1 * v - a1 * y + z2;
        z2 = b2 * v - a2 * y;
        last = y < 0.0 ? 0.0 : y;
    }
    float levelDb() const { return last > 1.0e-6 ? static_cast<float>(20.0 * std::log10(last)) : -120.0f; }
    // Natural frequency (rad/s) that gives 99 % of a step after riseSeconds at the damping used.
    static double naturalFrequency(double riseSeconds, double damping);

private:
    double b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0, last = 0;
};

// Correlation coefficient of two channels over an exponential window (default 300 ms): +1 mono, 0 uncorrelated, -1 inverted.
class CorrelationMeter
{
public:
    void prepare(double sampleRate, double windowSeconds = 0.3)
    {
        coefficient = 1.0 - std::exp(-1.0 / (windowSeconds * sampleRate));
        reset();
    }
    void reset() { ll = rr = lr = 0.0; }
    void process(float l, float r)
    {
        const double a = l, b = r;
        processMeans(a * a, b * b, a * b);
    }
    // One step with the means of l^2, r^2 and l r over a short group of samples (prepare() with the group rate).
    void processMeans(double meanLL, double meanRR, double meanLR)
    {
        ll += coefficient * (meanLL - ll);
        rr += coefficient * (meanRR - rr);
        lr += coefficient * (meanLR - lr);
    }
    float value() const
    {
        const double d = std::sqrt(ll * rr);
        return d > 1.0e-12 ? static_cast<float>(std::clamp(lr / d, -1.0, 1.0)) : 0.0f;
    }

private:
    double coefficient = 1.0e-3, ll = 0, rr = 0, lr = 0;
};

} // namespace crucible::fx
