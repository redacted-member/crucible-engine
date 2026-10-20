// SPDX-License-Identifier: MIT
#pragma once

// The clipper effect (docs/plans/ENGINE-FX-PLAN.md, section 3.2): drive into a clip curve (Clipper.h) that holds a ceiling,
// optionally oversampled 2x, 4x or 8x with first- or second-order ADAA at the high rate, then an optional true-peak guard (a
// one millisecond lookahead true-peak limiter with a fixed release, TruePeakLimiter) that trims the inter-sample overs the
// band-limited result can still have, then a parallel mix against the latency-aligned input. Latency is the oversampling
// filters, the ADAA delay, the droop equaliser and the guard's lookahead, reported in whole samples.
//
// ADAA filters the clipped waveform with a short kernel (a two-point mean for the first order, a three-point mean for the
// second), which also rolls off the linear part of the signal. A short symmetric FIR behind the clip curve, fitted by least
// squares to the inverse of that kernel over the band that matters (up to 20 kHz at the base rate, limited to 6 dB of
// boost), restores a flat response below the knee to within about 0.1 dB.

#include <crucible/fx/Clipper.h>
#include <crucible/fx/Lookahead.h>
#include <crucible/fx/Maximizer.h>
#include <crucible/fx/OversampledBlock.h>

#include <array>
#include <atomic>
#include <cmath>
#include <vector>

namespace crucible::fx
{

struct ClipperSettings
{
    float driveDb = 0.0f;
    float ceilingDb = -1.0f;
    float softness = 0.3f;    // 0 hard, 1 tanh
    int adaaOrder = 2;        // 0 off, 1 or 2: order of the antiderivative anti-aliasing
    int oversampleStages = 2; // 0 = 1x, 1 = 2x, 2 = 4x, 3 = 8x
    bool guard = true;
    float mix = 1.0f;
};

class ClipperEffect
{
public:
    static constexpr float kGuardLookaheadMs = 1.0f;

    // Control thread. Allocates all four oversampling paths.
    void prepare(double sampleRate, int maxBlock, Quality quality, OversamplingFilter filter);
    void reset();
    void process(float* left, float* right, int n, const ClipperSettings& s);

    int latencySamples() const { return latency.load(std::memory_order_relaxed); }
    int latencyFor(const ClipperSettings& s) const;
    int stagesFor(const ClipperSettings& s) const;
    bool takeLatencyChanged() { return latencyChanged.exchange(false, std::memory_order_relaxed); }

private:
    double sr = 48000.0;
    std::array<OversampledBlock, 3> blocks; // 2x, 4x, 8x
    AdaaClipper clippers[2];
    struct Equaliser
    {
        static constexpr int kMaxTaps = 10;
        std::array<float, kMaxTaps> taps{};
        int length = 0; // number of taps (0: none)
    };
    static Equaliser designEqualiser(int order, int stages, double sampleRate);
    TruePeakLimiter guardLimiter;
    std::array<std::array<Equaliser, 4>, 3> equalisers{}; // [adaa order][oversampling stages]
    std::array<std::array<float, Equaliser::kMaxTaps>, 2>
        eqHistory{}; // the newest taps - 1 inputs of the equaliser, oldest first
    std::vector<float> eqScratch;
    // Control smoothing (three one-poles each, about 4 ms): drive and ceiling in dB, softness and mix linear. Drive, ceiling
    // and softness run at the oversampled rate, mix at the base rate. Equal to the target (and then unused) when constant.
    struct Smooth3
    {
        double s1 = 0.0, s2 = 0.0, s3 = 0.0;
        void set(double v) { s1 = s2 = s3 = v; }
        bool at(double v) const { return s1 == v && s2 == v && s3 == v; }
        // Advances one sample; snaps onto the target once every stage is within 1e-7 of it.
        double step(double target, double c)
        {
            s1 += c * (target - s1);
            s2 += c * (s1 - s2);
            s3 += c * (s2 - s3);
            if (std::abs(s1 - target) < 1.0e-7 && std::abs(s2 - target) < 1.0e-7 && std::abs(s3 - target) < 1.0e-7)
                set(target);
            return s3;
        }
    };
    Smooth3 driveSm, ceilSm, softSm, mixSm;
    bool smoothInit = false;
    std::vector<float> ctlDrive, ctlInv, ctlCeil, ctlSoft;
    Lookahead dry;
    int active = -1;
    bool guardWasOn = false, started = false;
    std::atomic<int> latency{0};
    std::atomic<bool> latencyChanged{false};
};

} // namespace crucible::fx
