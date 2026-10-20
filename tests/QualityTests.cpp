// SPDX-License-Identifier: MIT
// Quality measurements that follow the reference bench (an external reference-bench suite): aliasing below
// 20 kHz relative to the harmonics, THD+N of a sine, audible bandwidth, and the oversampling policy. The numbers
// here come from a short FFT (leakage limited, about -100 dB); tools/refbench.sh runs the real suite.

#include "GraphSupport.h"

#include <crucible/Fx.h>
#include <crucible/module/ModuleRack.h>

using namespace crucible;
using namespace crucible::testing;

namespace
{
std::vector<float> tonePatch(const char* wave)
{
    auto v = graphPatch();
    setValue(v, "filter_on", 0.0f);
    setValue(v, "osc1_level", 1.0f);
    setValue(v, "osc1_uni_rand", 0.0f);
    if (std::string(wave) == "saw")
    {
        setValue(v, "osc1_table", 0.0f);
        setValue(v, "osc1_pos", 1.0f);
    }
    else if (std::string(wave) == "sine")
    {
        setValue(v, "osc1_table", 0.0f);
        setValue(v, "osc1_pos", 0.0f);
    }
    else // square
    {
        setValue(v, "osc1_table",
                 static_cast<float>(
                     ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("osc1_table"))].defaultValue));
        for (size_t i = 0; i < WavetableBank::tableNames().size(); ++i)
            if (WavetableBank::tableNames()[i] == "Analog Square")
                setValue(v, "osc1_table", static_cast<float>(i));
        setValue(v, "osc1_pos", 0.5f);
    }
    return v;
}

// Amplitude of the sinusoid at hz (Hann window, last second of the signal).
double amplitudeAt(const std::vector<float>& s, size_t from, size_t count, double hz)
{
    double re = 0.0, im = 0.0;
    for (size_t i = 0; i < count; ++i)
    {
        const double w = 0.5 - 0.5 * std::cos(6.283185307179586 * static_cast<double>(i) / static_cast<double>(count));
        const double ph = 6.283185307179586 * hz * static_cast<double>(from + i) / kRate;
        re += w * s[from + i] * std::cos(ph);
        im += w * s[from + i] * std::sin(ph);
    }
    return 2.0 * std::sqrt(re * re + im * im) / (0.5 * static_cast<double>(count));
}
} // namespace

class QualityTests : public UnitTest
{
public:
    QualityTests() : UnitTest("Quality", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("saw and square alias less than -100 dB below 20 kHz at every pitch, sine THD+N below -100 dB");
        for (const char* wave : {"saw", "square", "sine"})
        {
            double worst = -300.0;
            for (const int note : {36, 48, 60, 72, 84, 96, 103, 108})
            {
                const auto r = playNote(EngineConfig{}, tonePatch(wave), note, 8192 + 16384 + 100);
                const double db = aliasingBelowDb(r.left, midiToHz(note), kRate, 20000.0, 8192);
                worst = std::max(worst, db);
            }
            std::printf("    %s: worst aliasing below 20 kHz over C2..C8 %.1f dB\n", wave, worst);
            expect(worst < -100.0, std::string(wave) + " aliasing " + std::to_string(worst));
        }

        beginTest("a played saw keeps its harmonics flat up to 20 kHz at every pitch");
        for (const int note : {36, 48, 60, 84, 96, 108})
        {
            const double f0 = midiToHz(note);
            const auto r = playNote(EngineConfig{}, tonePatch("saw"), note, 8192 + 32768);
            const int top = static_cast<int>(19900.0 / f0);
            const double a1 = amplitudeAt(r.left, 8192, 32768, f0);
            const double at = amplitudeAt(r.left, 8192, 32768, top * f0);
            const double ratioDb = 20.0 * std::log10(at / a1 * top); // a saw falls as 1/h
            expect(std::abs(ratioDb) < 3.0, "note " + std::to_string(note) + ": harmonic " + std::to_string(top) + " is " +
                                                std::to_string(ratioDb) + " dB off");
        }

        beginTest("quality policy: every quality and oversampling filter renders finite audio, saturator aliasing improves with "
                  "quality");
        {
            double alias[3][2];
            for (int q = 0; q < 3; ++q)
                for (int f = 0; f < 2; ++f)
                {
                    ModuleRack rack(ModuleRegistry::instance());
                    ModuleContext ctx;
                    ctx.sampleRate = kRate;
                    ctx.quality = static_cast<Quality>(q);
                    ctx.oversampling = static_cast<OversamplingFilter>(f);
                    ctx.maxBlockSize = 256;
                    rack.prepare(ctx);
                    const double f0 = 4217.0;
                    std::vector<float> sig(24000 + 16384 + 100);
                    for (size_t i = 0; i < sig.size(); ++i)
                        sig[i] =
                            0.9f * static_cast<float>(std::sin(6.283185307179586 * f0 * static_cast<double>(i) / kRate));
                    auto values = ParameterMap::defaultValues();
                    for (int m = 0; m < rack.size(); ++m)
                        setValue(values, rack.descriptor(m).onParam, 0.0f);
                    setValue(values, "fx_saturator_on", 1.0f);
                    setValue(values, "fx_saturator_mix", 1.0f);
                    setValue(values, "fx_saturator_model", 1.0f); // hard clip
                    setValue(values, "fx_saturator_drive", 28.0f);
                    const std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
                    std::vector<float> right = sig;
                    for (size_t pos = 0; pos < sig.size(); pos += 256)
                    {
                        const int n = static_cast<int>(std::min<size_t>(256, sig.size() - pos));
                        rack.process(sig.data() + pos, right.data() + pos, n, values.data(), legacy.data(), nullptr, 120.0);
                    }
                    for (const float x : sig)
                        expect(std::isfinite(x));
                    alias[q][f] = aliasingBelowDb(sig, f0, kRate, 20000.0, 24000);
                    std::printf("    saturator aliasing below 20 kHz, %s %s: %.1f dB\n",
                                q == 0 ? "draft" : (q == 1 ? "standard" : "high"), f == 0 ? "half-band" : "polyphase",
                                alias[q][f]);
                }
            for (int f = 0; f < 2; ++f)
            {
                expect(alias[1][f] < alias[0][f] - 6.0, "standard beats draft");
                // The saturator already reaches the measurement floor (about -100 dB) at Standard.
                expect(alias[2][f] <= alias[1][f] + 0.5, "high is no worse than standard");
            }
            // The quality also reaches the voice: warps render at every quality.
            for (const auto quality : {Quality::Draft, Quality::Standard, Quality::High})
                for (const auto filter : {OversamplingFilter::HalfBand, OversamplingFilter::Polyphase})
                {
                    EngineConfig cfg;
                    cfg.quality = quality;
                    cfg.oversampling = filter;
                    auto v = tonePatch("saw");
                    setValue(v, "osc1_warp", 14.0f); // fold
                    setValue(v, "osc1_warp_a", 0.7f);
                    setValue(v, "fx_saturator_on", 1.0f);
                    setValue(v, "fx_saturator_model", 4.0f);
                    const auto r = playNote(cfg, v, 72, 9600);
                    expect(r.finite && r.peak > 0.01f);
                }
        }
    }
};

static QualityTests qualityTests;
