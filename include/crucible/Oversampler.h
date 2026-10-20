// SPDX-License-Identifier: MIT
#pragma once

// Oversampling for nonlinear stages (drive, waveshapers): cascaded 2x stages, each a half-band filter, either
// linear phase (FIR) or minimum latency (polyphase all-pass IIR). The length / order follows the quality policy
// (ModuleContext::quality): Draft is short, Standard is the engine default, High is long and steep. A half-band
// stage keeps everything up to 20 kHz at 48 kHz flat and rejects what would fold back below 20 kHz.

#include <crucible/module/Module.h>

#include <array>
#include <vector>

namespace crucible
{

class Oversampler
{
public:
    // stages: 1 = 2x, 2 = 4x, 3 = 8x. Control thread (allocates).
    void prepare(int stages, OversamplingFilter filter, Quality quality, int maxBlock);
    void reset();

    int factor() const { return 1 << stages; }
    // Latency of up() followed by down(), in base-rate samples (measured from an impulse at prepare time).
    float latency() const { return measuredLatency; }

    // out holds n * factor() samples / in holds n * factor() samples. Audio thread, allocation free. n <= maxBlock.
    void up(const float* in, int n, float* out);
    void down(const float* in, int n, float* out);

private:
    static constexpr int kMaxStages = 3;
    static constexpr int kMaxCoefficients = 8;
    static constexpr int kMaxPairs = 24;
    static constexpr int kRing = 128;

    struct Stage
    {
        // FIR
        std::array<float, kMaxPairs> odd{};
        int pairs = 0;
        int centre = 0;
        std::array<float, kRing> upBuffer{}, downBuffer{};
        int upPos = 0, downPos = 0;
        // IIR
        int coefficients = 0;
        std::array<float, kMaxCoefficients> a{};
        std::array<float, kMaxCoefficients> upX{}, upY{}, downX{}, downY{};
        float downPrev = 0.0f;
    };

    void upStage(Stage& s, const float* in, int n, float* out);
    void downStage(Stage& s, const float* in, int n, float* out);
    float allpass(const Stage& s, std::array<float, kMaxCoefficients>& x, std::array<float, kMaxCoefficients>& y, int first,
                  float v) const;
    float firAt(const std::array<float, kRing>& buf, int i) const { return buf[static_cast<size_t>(i & (kRing - 1))]; }

    int stages = 2;
    OversamplingFilter kind = OversamplingFilter::HalfBand;
    std::array<Stage, kMaxStages> stage{};
    std::vector<float> bufA, bufB;
    // Linear copies of the FIR history plus the new input (even / odd halves for the decimator), for the four-lane
    // half-band kernels (Oversampler.cpp).
    std::vector<float> linA, linB;
    float measuredLatency = 0.0f;
};

} // namespace crucible
