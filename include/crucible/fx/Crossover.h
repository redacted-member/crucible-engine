// SPDX-License-Identifier: MIT
#pragma once

// Crossovers that split a signal into 2 to 8 bands and sum back flat (docs/plans/ENGINE-FX-PLAN.md, sections 3.3 and
// 3.9): the splitting stage of multiband processors, imagers, de-essers and exciters.
//
// Crossover (minimum phase, no latency): Linkwitz-Riley filters of 12, 24, 36 and 48 dB per octave (the square of a
// Butterworth filter of order n = 1 .. 4; S. Linkwitz, "Active Crossover Networks for Noncoincident Drivers", JAES 1976;
// -6 dB at the crossover frequency, in phase) and a complementary first-order split (6 dB per octave). The bands are
// built as a tree: the signal is split at the lowest frequency, the upper part is split at the next frequency, and so on.
// With low-pass L and high-pass H of one split, L + H is an all-pass A of order n (with H inverted for odd n), so the
// bands below a later split pass through the all-pass of every split above them; the sum of all bands is then the product
// of all the all-passes: exactly flat in magnitude, for any frequencies in any spacing and for any band count. The digital
// filters are pre-warped bilinear transforms of the analog sections, which preserve this identity exactly.
//
// LinearCrossover (linear phase, latency reported): the band kernels are differences of zero-phase low-passes
// 1 / (1 + (f / f0)^m) (the magnitude of a Linkwitz-Riley filter of order m, which sums with its complement to exactly 1),
// so the bands add up to the input delayed by the kernel's centre: a null test of the sum against the delayed input
// reads below -100 dB. The convolution runs on PartitionedConvolver; set() designs on the calling (control) thread.
//
// Frequencies must be ascending; neighbouring frequencies closer than 1 % are pushed apart.

#include <crucible/fx/AnalogMatchedFilters.h>
#include <crucible/fx/LinearPhase.h>

#include <array>
#include <complex>
#include <vector>

namespace crucible::fx
{

constexpr int kMaxCrossoverBands = 8;

enum class CrossoverSlope
{
    Db6,  // complementary first order
    Db12, // Linkwitz-Riley 2
    Db24, // Linkwitz-Riley 4
    Db36, // Linkwitz-Riley 6
    Db48  // Linkwitz-Riley 8
};

constexpr int slopeDbPerOctave(CrossoverSlope s)
{
    switch (s)
    {
    case CrossoverSlope::Db6:
        return 6;
    case CrossoverSlope::Db12:
        return 12;
    case CrossoverSlope::Db24:
        return 24;
    case CrossoverSlope::Db36:
        return 36;
    default:
        return 48;
    }
}

class Crossover
{
public:
    // Control thread (allocation free, but not meant for the audio thread when the frequencies move: see set()).
    void prepare(double sampleRate);
    void reset();

    // 2 .. 8 bands, bands-1 ascending frequencies. The new filters are reached over rampSamples samples (0: at once).
    // Designs a handful of bilinear sections per split: cheap enough for a control block; audio thread safe.
    void set(int bands, const double* freqsHz, CrossoverSlope slope, int rampSamples = 0);
    int bands() const { return numBands; }

    // In place on nothing: reads l / r, writes band b to bandL[b] / bandR[b] (n floats each, any n).
    void process(const float* l, const float* r, int n, float* const* bandL, float* const* bandR) noexcept;

    // Analytic complex response of band b (0 = lowest) at hz, from the sections now designed.
    std::complex<double> response(int band, double hz) const;
    // Sum of the bands: an all-pass (unit magnitude).
    std::complex<double> sumResponse(double hz) const;

private:
    struct Design
    {
        std::array<Biquad, kMaxBandSections> section;
        int count = 0;
    };
    struct Split
    {
        BiquadChain lp, hp;
        Design lpDesign, hpDesign, apDesign;
        double frequency = 1000.0;
    };
    void designSplit(Split& s, double frequency, CrossoverSlope slope, int ramp);
    std::complex<double> chainResponse(const Design& d, double omega) const;

    double sr = 48000.0;
    int numBands = 2;
    CrossoverSlope slopeNow = CrossoverSlope::Db24;
    std::array<Split, kMaxCrossoverBands - 1> split;
    // comp[j][m - j - 1]: the all-pass of split m applied to band j (m > j).
    std::array<std::array<BiquadChain, kMaxCrossoverBands - 2>, kMaxCrossoverBands - 2> comp;
};

// Linear-phase crossover. set() allocates (control thread); process() is audio-thread safe.
class LinearCrossover
{
public:
    void prepare(double sampleRate);
    void reset() { convolver.reset(); }
    // lengthLog2: kernel of 2^lengthLog2 - 1 taps. order: Linkwitz-Riley order m (even, 2 .. 16; 4 = 24 dB per octave).
    void set(int bands, const double* freqsHz, int order, int lengthLog2 = 12);
    int bands() const { return numBands; }
    int latencySamples() const { return lengthLog2Now > 0 ? linearPhaseLatency(lengthLog2Now) : 0; }
    // True when a kernel has been installed by the audio thread.
    bool ready() const { return convolver.hasKernel(); }

    void process(const float* l, const float* r, int n, float* const* bandL, float* const* bandR) noexcept;

    // The band kernels most recently designed (centre at kernelCentre(log2)), for tests and views.
    const std::vector<std::vector<float>>& kernels() const { return taps; }

private:
    double sr = 48000.0;
    int numBands = 2;
    int lengthLog2Now = 0;
    std::uint64_t generation = 0;
    LinearPhaseDesigner designer;
    KernelSetBuilder builder;
    PartitionedConvolver convolver;
    std::vector<std::vector<float>> taps;
    std::vector<float> magnitude;
};

} // namespace crucible::fx
