// SPDX-License-Identifier: MIT
// Spectral drawbars of the cloud oscillator: eight octave-band gains (osc1_cloud_band1..8).

#include "GraphSupport.h"

#include <crucible/Cloud.h>
#include <crucible/Patch.h>

using namespace crucible;
using namespace crucible::testing;

namespace
{
// A cloud with one equal partial for every integer 1..count at morph position 0.
CloudModel flatCloud(int count)
{
    CloudModel m;
    for (int n = 1; n <= count; ++n)
        m.points.push_back({static_cast<float>(n), 1.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    return m;
}

constexpr int kFftOrder = 14;
constexpr int kFftSize = 1 << kFftOrder;

// Power spectrum of 2^14 samples with a Blackman-Harris window.
std::vector<double> power(const std::vector<float>& signal, size_t from)
{
    const Fft fft(kFftOrder);
    std::vector<float> data(static_cast<size_t>(2 * kFftSize), 0.0f);
    for (int i = 0; i < kFftSize; ++i)
    {
        const double x = 2.0 * 3.14159265358979 * i / (kFftSize - 1);
        const double w = 0.35875 - 0.48829 * std::cos(x) + 0.14128 * std::cos(2 * x) - 0.01168 * std::cos(3 * x);
        data[static_cast<size_t>(i)] = signal[from + static_cast<size_t>(i)] * static_cast<float>(w);
    }
    fft.forward(data.data());
    std::vector<double> p(static_cast<size_t>(kFftSize / 2 + 1));
    for (size_t b = 0; b < p.size(); ++b)
        p[b] = static_cast<double>(data[2 * b]) * data[2 * b] + static_cast<double>(data[2 * b + 1]) * data[2 * b + 1];
    return p;
}

// Energy within +-5 bins of the partials n in [first, last] of f0.
double partialEnergy(const std::vector<double>& p, double f0, int first, int last)
{
    double e = 0.0;
    for (int n = first; n <= last; ++n)
    {
        const int centre = static_cast<int>(std::lround(n * f0 * kFftSize / kRate));
        for (int b = centre - 5; b <= centre + 5; ++b)
            if (b >= 0 && b < static_cast<int>(p.size()))
                e += p[static_cast<size_t>(b)];
    }
    return e;
}

double db(double ratio)
{
    return 10.0 * std::log10(std::max(ratio, 1.0e-30));
}

constexpr int kFirst[kCloudBands] = {1, 2, 3, 5, 9, 17, 33, 65};
constexpr int kLast[kCloudBands] = {1, 2, 4, 8, 16, 32, 64, 100};
} // namespace

class CloudDrawbarTests : public UnitTest
{
public:
    CloudDrawbarTests() : UnitTest("Cloud spectral drawbars", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("band edges: 1, 2, 3-4, 5-8, 9-16, 17-32, 33-64, 65+ (inharmonic partials by their number)");
        {
            for (int b = 0; b < kCloudBands; ++b)
            {
                expectEquals(cloudBandOf(static_cast<float>(kFirst[b])), b);
                expectEquals(cloudBandOf(static_cast<float>(kLast[b])), b);
            }
            expectEquals(cloudBandOf(1.99f), 0);
            expectEquals(cloudBandOf(2.5f), 1);
            expectEquals(cloudBandOf(4.99f), 2);
            expectEquals(cloudBandOf(512.0f), 7);
        }

        beginTest("the parameters: eight per slot, 0..2, default 1, per-voice modulatable, log smoothing");
        {
            for (int b = 1; b <= kCloudBands; ++b)
            {
                const int i = ParameterMap::indexOf("osc1_cloud_band" + std::to_string(b));
                expect(i >= 0);
                const auto& def = ParameterMap::params()[static_cast<size_t>(i)];
                expect(def.min == 0.0f && def.max == 2.0f && def.defaultValue == 1.0f);
                expect(def.voiceMod && def.smoothing == Smoothing::Log);
            }
            expect(ParameterMap::indexOf("osc3_cloud_band8") >= 0);
            for (size_t page = 0; page < ParameterMap::pages().size(); ++page)
                expect(ParameterMap::paramsOnPage(static_cast<int>(page)).size() <= 6);
            expectEquals(ModuleRegistry::instance().find("osc.cloud")->version, 2);
        }

        beginTest("all bands at 1 render exactly as without drawbars");
        {
            const CloudModel cloud = flatCloud(60);
            std::vector<float> a(4096, 0.0f), b(4096, 0.0f), c(4096, 0.0f), d(4096, 0.0f);
            const std::array<float, kCloudBands> ones = {1, 1, 1, 1, 1, 1, 1, 1};
            CloudOscillator x, y;
            x.prepare(kRate);
            y.prepare(kRate);
            x.noteOn(0.0f, 7);
            y.noteOn(0.0f, 7);
            x.renderModel(&cloud, a.data(), b.data(), 4096, 0.0f, 220.0f, 0.0f, 1.0f);
            y.renderModel(&cloud, c.data(), d.data(), 4096, 0.0f, 220.0f, 0.0f, 1.0f, ones.data());
            expect(a == c && b == d);
        }

        beginTest("a band at zero removes its octave band (below -60 dB) and leaves the others");
        {
            const CloudModel cloud = flatCloud(100);
            const double f0 = kRate * 64.0 / kFftSize; // 187.5 Hz: every partial sits exactly on a bin
            for (int zeroed = 0; zeroed < kCloudBands; ++zeroed)
            {
                std::array<float, kCloudBands> gains = {1, 1, 1, 1, 1, 1, 1, 1};
                gains[static_cast<size_t>(zeroed)] = 0.0f;
                std::vector<float> l(kFftSize, 0.0f), r(kFftSize, 0.0f);
                CloudOscillator osc;
                osc.prepare(kRate);
                osc.noteOn(0.0f, 3);
                osc.renderModel(&cloud, l.data(), r.data(), kFftSize, 0.0f, static_cast<float>(f0), 0.0f, 1.0f,
                                gains.data());
                const auto p = power(l, 0);
                double others = 0.0;
                for (int b = 0; b < kCloudBands; ++b)
                    if (b != zeroed)
                        others += partialEnergy(p, f0, kFirst[b], kLast[b]);
                const double removed = partialEnergy(p, f0, kFirst[zeroed], kLast[zeroed]);
                expect(db(removed / others) < -60.0,
                       "band " + std::to_string(zeroed + 1) + " leaves " + std::to_string(db(removed / others)) + " dB");
                // Every other band is still there: within 12 dB of an even share.
                for (int b = 0; b < kCloudBands; ++b)
                    if (b != zeroed)
                        expect(partialEnergy(p, f0, kFirst[b], kLast[b]) > 0.0);
            }
        }

        beginTest("through the engine: the parameter, a doubled band and a per-voice route");
        {
            auto v = graphPatch();
            setValue(v, "osc1_type", 2.0f);
            setValue(v, "osc1_level", 0.5f);
            const double f0 = 220.0; // note 57
            const auto level = [&](const std::vector<float>& values, int first, int last, const ModRoute* route = nullptr)
            {
                SynthEngine engine;
                CloudModel cloud = flatCloud(24);
                engine.cloudBank().set(0, cloud);
                prepare(engine, values);
                if (route != nullptr)
                    engine.modRoutes().add(*route);
                engine.noteOn(57, 1.0f);
                const auto r = renderBlocks(engine, 12000 + kFftSize, 512, true);
                return partialEnergy(power(r.left, 12000), f0, first, last);
            };
            const double base = level(v, 3, 4);
            auto off = v;
            setValue(off, "osc1_cloud_band3", 0.0f);
            expect(db(level(off, 3, 4) / base) < -40.0, "band 3 off");
            auto loud = v;
            setValue(loud, "osc1_cloud_band3", 2.0f);
            const double doubled = level(loud, 3, 4);
            expect(doubled > 1.5 * base, "band 3 at 2 is louder");
            // Velocity 1 pulls band 3 down by 5 octaves of gain (x 2^-5) through a route.
            ModRoute route;
            route.source = static_cast<int>(ModSource::Velocity);
            route.target = "osc1_cloud_band3";
            route.amount = -1.0f;
            expect(db(level(v, 3, 4, &route) / base) < -20.0, "per-voice route");
        }

        beginTest("deterministic, and identical across block sizes on the 32-sample grid");
        {
            auto v = graphPatch();
            setValue(v, "osc1_type", 2.0f);
            setValue(v, "osc1_cloud_band2", 0.3f);
            setValue(v, "osc1_cloud_band5", 1.7f);
            const auto run = [&](int block)
            {
                SynthEngine engine;
                engine.cloudBank().set(0, flatCloud(40));
                prepare(engine, v);
                engine.noteOn(52, 0.8f);
                return renderBlocks(engine, 9600, block, true);
            };
            const auto a = run(512), b = run(512), c = run(128), d = run(32);
            expect(a.left == b.left && a.right == b.right, "same input, same output");
            expect(a.left == c.left && a.left == d.left && a.right == d.right, "block sizes");
        }

        beginTest("a patch saved before the drawbars loads with every band at 1 and sounds the same");
        {
            Patch patch;
            patch.meta.name = "old";
            patch.params.push_back({"osc1_type", 2.0f});
            patch.params.push_back({"osc1_level", 0.5f});
            patch.moduleVersions["osc.cloud"] = 1;
            std::string error;
            Patch back;
            expect(patchFromJson(patchToJson(patch), back, error), error);
            expect(back.moduleVersions["osc.cloud"] == 1);
            auto fromPatch = ParameterMap::defaultValues();
            for (const auto& [id, value] : back.params)
                setValue(fromPatch, id, value);
            for (int b = 1; b <= kCloudBands; ++b)
                expectEquals(fromPatch[static_cast<size_t>(ParameterMap::indexOf("osc1_cloud_band" + std::to_string(b)))],
                             1.0f);
            auto explicitOnes = fromPatch;
            for (int b = 1; b <= kCloudBands; ++b)
                setValue(explicitOnes, "osc1_cloud_band" + std::to_string(b), 1.0f);
            const auto a = playNote(EngineConfig{}, fromPatch, 57, 4800);
            const auto b = playNote(EngineConfig{}, explicitOnes, 57, 4800);
            expect(a.left == b.left && a.peak > 0.01f);
        }
    }
};

static CloudDrawbarTests cloudDrawbarTests;
