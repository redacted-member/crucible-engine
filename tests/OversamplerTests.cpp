// SPDX-License-Identifier: MIT
#include <crucible/Oversampler.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
const char* label(OversamplingFilter f, Quality q)
{
    static std::string s;
    s = std::string(f == OversamplingFilter::HalfBand ? "half-band FIR " : "polyphase IIR ") +
        (q == Quality::Draft ? "draft" : (q == Quality::Standard ? "standard" : "high"));
    return s.c_str();
}

// Peak of a tone after down(), measured over the last part of a long run.
double toneGainDb(Oversampler& os, double hzAtStageRate, double stageRate, int stages)
{
    (void)stageRate;
    const int n = 4096;
    const int factor = 1 << stages;
    std::vector<float> in(static_cast<size_t>(n * factor)), out(static_cast<size_t>(n));
    for (int i = 0; i < n * factor; ++i)
        in[static_cast<size_t>(i)] =
            static_cast<float>(std::sin(2.0 * 3.14159265358979 * hzAtStageRate * i / (48000.0 * factor)));
    os.reset();
    for (int pos = 0; pos < n; pos += 32)
        os.down(in.data() + static_cast<size_t>(pos * factor), 32, out.data() + pos);
    double sum = 0.0;
    for (int i = n / 2; i < n; ++i)
        sum += static_cast<double>(out[static_cast<size_t>(i)]) * out[static_cast<size_t>(i)];
    const double amplitude = std::sqrt(2.0 * sum / (n / 2));
    return 20.0 * std::log10(std::max(amplitude, 1.0e-12));
}
} // namespace

class OversamplerTests : public UnitTest
{
public:
    OversamplerTests() : UnitTest("Oversampler", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        for (const auto filter : {OversamplingFilter::HalfBand, OversamplingFilter::Polyphase})
            for (const auto quality : {Quality::Draft, Quality::Standard, Quality::High})
            {
                beginTest(std::string("pass band, stop band and latency: ") + label(filter, quality));
                const double stopMin =
                    filter == OversamplingFilter::Polyphase
                        ? (quality == Quality::Draft ? 60.0 : (quality == Quality::Standard ? 90.0 : 115.0))
                        : (quality == Quality::Draft ? 25.0 : (quality == Quality::Standard ? 55.0 : 85.0));
                for (int stages = 1; stages <= 3; ++stages)
                {
                    Oversampler os;
                    os.prepare(stages, filter, quality, 64);
                    expectEquals(os.factor(), 1 << stages);
                    // 15 kHz at the base rate passes untouched; the top of the stage-rate band (aliasing source)
                    // is rejected: 30 kHz at 2x folds to 18 kHz.
                    const double pass = toneGainDb(os, 15000.0, 48000.0, stages);
                    expect(std::abs(pass) < 0.1, "pass band gain " + std::to_string(pass));
                    const double stop = -toneGainDb(os, 30000.0, 48000.0, stages);
                    expect(stop > stopMin, "stop band " + std::to_string(stop) + " dB");
                    expect(os.latency() >= 0.0f && os.latency() < 200.0f);
                    // Up then down returns the input (delayed) for a band-limited signal.
                    std::vector<float> in(2048), mid(static_cast<size_t>(2048 << stages)), out(2048);
                    for (size_t i = 0; i < in.size(); ++i)
                        in[i] = 0.5f * static_cast<float>(std::sin(0.31 * static_cast<double>(i)) +
                                                          std::sin(0.083 * static_cast<double>(i)));
                    os.reset();
                    for (int pos = 0; pos < 2048; pos += 32)
                    {
                        os.up(in.data() + pos, 32, mid.data() + (static_cast<size_t>(pos) << stages));
                        os.down(mid.data() + (static_cast<size_t>(pos) << stages), 32, out.data() + pos);
                    }
                    const double lag = os.latency();
                    double err = 0.0, sig = 0.0;
                    for (int i = 600; i < 1800; ++i)
                    {
                        const double t = static_cast<double>(i) - lag;
                        const double ref = 0.5 * (std::sin(0.31 * t) + std::sin(0.083 * t));
                        const double d = out[static_cast<size_t>(i)] - ref;
                        err += d * d;
                        sig += ref * ref;
                    }
                    // The IIR is not linear phase, so only the FIR is held to a delayed copy.
                    if (filter == OversamplingFilter::HalfBand)
                        expect(10.0 * std::log10(err / sig) < -25.0,
                               "round trip " + std::to_string(10.0 * std::log10(err / sig)));
                }
            }
    }
};

static OversamplerTests oversamplerTests;
