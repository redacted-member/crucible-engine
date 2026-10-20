// SPDX-License-Identifier: MIT
#pragma once

// Bucket-brigade delay core (docs/plans/ENGINE-FX-PLAN.md, section 3.5, package FX10). A clean-room model from the
// published literature: Raffel and Smith, "Practical modeling of bucket-brigade device circuits" (DAFx 2010) for the
// structure (anti-aliasing filter, sampling at the clock, a chain of stages, hold, reconstruction filter), Holters and
// Parker's combined model of the filters and the BBD as one system for the idea of tying both filters to the clock, and
// Chowdhury's public write-up of a BBD emulation (2021) as reading on the clock handling. The circuit values of any
// particular chip are not modelled: this is a generic device.
//
// Signal path, per sample:
//
//   input -> drive -> compressor (optional, 2:1 in dB) -> input filter -> sampled at the clock -> N / 2 stages (a ring
//   of N / 2 held samples, one clock tick moves every sample one place) -> soft clip -> + noise and clock feed-through
//   -> held until the next tick -> reconstruction filter -> expander (optional, 1:2) -> output limiter -> output
//
//   * A device of N stages (256 to 4096) with the clock at fclk delays by N / (2 fclk) seconds. delaySeconds is turned
//     into a clock; when it changes the clock glides, so the delay changes with the pitch shift of a real device and
//     never jumps. The clock cannot exceed the audio rate (the sampling and the hold are modelled at the audio rate with
//     the exact fractional tick time), so the shortest delay is minDelaySeconds() = N / (2 x 0.95 x rate).
//   * Both filters are Butterworth low-pass filters whose cutoff is 0.3447 x fclk, retuned with the clock: 4th order
//     at the input, 8th order for the reconstruction. The input filter runs at the audio rate (zero-delay-feedback
//     state-variable sections, stable while the cutoff moves). The reconstruction filter is applied to the held (staircase)
//     signal in continuous time: the filter is written as its four complex modes (the partial fractions of its step
//     response), each mode is advanced by one audio sample per output sample, and a clock tick adds the step of the held
//     value to every mode with the exact fractional tick time, so the output is the filter's output at the exact
//     audio-sample instants. Sampling the staircase first and filtering afterwards would fold its high-frequency images into
//     the audio band. With the hold's sinc droop the whole chain is 3 dB down at bandwidthHz(fclk) = 0.30 x fclk; the images
//     of a tone at 0.1 x fclk are below -85 dB, of a tone at 0.2 x fclk below -70 dB.
//   * The compander is a pair of rectified-average level detectors; the compressor applies gain env^-1/2 to the input
//     and the expander gain env to the filtered output, so a steady tone passes at unity gain and noise added in between
//     is pushed down when the signal is quiet and rises with it ("breathing"). `breathing` slows the expander's
//     detector relative to the compressor's (0 = matched, 1 = three times slower), the mismatch that makes the noise
//     pump.
//   * The soft clip is L x tanh(x / L) (a rational approximation that reaches L smoothly at 3 L) with L = 3 on the
//   compressed signal (unity small-signal gain), applied at the
//     input of the stage chain, so a full-scale input is mildly compressed and driving it harder saturates smoothly.
//     The output goes through a smooth limiter, 4 x tanh(x / 4) (a rational approximation), so it is bounded by 4
//     whatever the drive.
//
// Latency beyond the nominal delay: the input is read at the exact (fractional) tick time with a 16-tap sinc, which
// adds 7 samples, and the two filters add their group delay (about 2.6 / (2 pi x 0.3447 fclk) seconds each).
//
// prepare() allocates; everything else is real-time safe. setSettings() with a different stage count clears the delay.

#include <crucible/fx/FractionalDelay.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace crucible::fx
{

struct BbdSettings
{
    int stages = 1024;    // 256 .. 4096
    float driveDb = 0.0f; // gain before the compressor, -24 .. +24 (changes glide: two poles of 5 ms)
    bool compander = true;
    float noiseDb = -70.0f; // RMS level of the device noise in the compressed domain; at or below -120 it is off
    float breathing = 0.5f; // 0 .. 1, see above
    float clockFeedthroughDb =
        -120.0f;          // amplitude of the clock pulse at the output of the chain; at or below -120 it is off
    float glideMs = 5.0f; // time constant of the clock following the requested delay
};

class Bbd
{
public:
    static constexpr int kMinStages = 256;
    static constexpr int kMaxStages = 4096;
    static constexpr double kBandwidthRatio = 0.30;      // -3 dB bandwidth of the whole chain, in clock frequencies
    static constexpr double kFilterCutoffRatio = 0.3447; // cutoff of both filters, in clock frequencies
    static constexpr double kMaxClockRatio = 0.95;       // highest clock, in audio sample rates
    static constexpr double kMinClockHz = 500.0;

    static double bandwidthHz(double clockHz) { return kBandwidthRatio * clockHz; }
    static double clockForDelay(int stages, double delaySeconds)
    {
        return static_cast<double>(stages) / (2.0 * delaySeconds);
    }

    void prepare(double sampleRate);
    void reset();
    void setSettings(const BbdSettings& s);
    const BbdSettings& settings() const { return cfg; }
    // Seed of the noise generator (two channels of one effect use different seeds so their noise is not the same).
    void setSeed(std::uint64_t seed)
    {
        rng = seed * 0x9E3779B97F4A7C15ull + 0xD1B54A32D192ED03ull;
        if (rng == 0)
            rng = 1;
    }

    // Requested delay; the clock moves to stages / (2 x delay) (limited to its range) with the glide of the settings.
    void setDelaySeconds(double seconds);
    double minDelaySeconds() const { return static_cast<double>(cfg.stages) / (2.0 * kMaxClockRatio * rate); }
    double maxDelaySeconds() const { return static_cast<double>(cfg.stages) / (2.0 * kMinClockHz); }
    double clockHz() const { return clock; }

    void processBlock(const float* in, float* out, int n);
    // The same with the input of each sample given by input(context, index, previous output), for a feedback loop around the
    // device: the sample is formed from the previous output of this call (0 for the first), with no call overhead per
    // sample.
    using InputFn = float (*)(void* context, int index, float previousOutput);
    void processWithInput(InputFn input, void* context, float* out, int n);
    float process(float x)
    {
        float y = 0.0f;
        processBlock(&x, &y, 1);
        return y;
    }

private:
    struct Section
    {
        double ic1 = 0.0, ic2 = 0.0;
        double a1 = 0.0, a2 = 0.0, a3 = 0.0;
        double k = 1.0;
        void setCoefficients(double g)
        {
            a1 = 1.0 / (1.0 + g * (g + k));
            a2 = g * a1;
            a3 = g * a2;
        }
        double process(double x)
        {
            const double v3 = x - ic2;
            const double v1 = a1 * ic1 + a2 * v3;
            const double v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2.0 * v1 - ic1;
            ic2 = 2.0 * v2 - ic2;
            return v2;
        }
        // A silent input leaves the states decaying; called every few samples, this keeps them out of the denormal range.
        void flush()
        {
            if (ic1 < 1.0e-25 && ic1 > -1.0e-25)
                ic1 = 0.0;
            if (ic2 < 1.0e-25 && ic2 > -1.0e-25)
                ic2 = 0.0;
        }
    };
    struct Lowpass
    {
        std::array<Section, 2> s;
        void init();
        void setCutoff(double g)
        {
            s[0].setCoefficients(g);
            s[1].setCoefficients(g);
            s[0].flush();
            s[1].flush();
        }
        double process(double x) { return s[1].process(s[0].process(x)); }
        void reset()
        {
            for (auto& e : s)
                e.ic1 = e.ic2 = 0.0;
        }
    };
    static constexpr int kModes = 4;     // complex modes of the reconstruction filter (8th order, conjugate pairs)
    static constexpr int kModeRes = 512; // entries of the mode tables per clock period
    struct ModeEntry
    {
        float re[kModes], im[kModes];
    };

    void updateCoefficients();
    template <class Input> void run(Input&& input, float* out, int n);
    ModeEntry lookup(const std::vector<ModeEntry>& table, double x) const;
    double nextNoise();

    BbdSettings cfg;
    double rate = 48000.0;
    double invRate = 1.0 / 48000.0;
    double targetDelay = 0.02;
    double clock = 50000.0;
    double clockTarget = 50000.0;
    double glideAlpha = 0.01;
    double phase = 0.0;
    int ringLength = 512;
    int ringPos = 0;
    std::vector<float> ring;
    FractionalDelay tapDelay;
    Lowpass inFilter;
    std::vector<ModeEntry> advance; // exp(p x) of the four modes, x in clock periods, kModeRes entries per period
    std::vector<ModeEntry> inject;  // residue x exp(p x): what a held step adds to each mode x periods before the output
    ModeEntry mode{};               // the modes' state
    float held = 0.0f;              // the staircase: value of the last tick
    int coefCountdown = 0;
    double driveGain = 1.0; // target of the drive gain; driveStage1 / driveStage2 follow it (two poles of kDriveMs)
    double driveStage1 = 1.0, driveStage2 = 1.0, driveAlpha = 0.01;
    double noiseAmp = 0.0;
    double feedAmp = 0.0;
    double feedSign = 1.0;
    double envC = 0.0, envE = 0.0;
    double attC = 0.0, relC = 0.0, attE = 0.0, relE = 0.0;
    std::uint64_t rng = 0x9E3779B97F4A7C15ull;
};

} // namespace crucible::fx
