// SPDX-License-Identifier: MIT
// Convolution suite (FX7, docs/plans/ENGINE-FX-PLAN.md section 3.5): `fx.convolution` through the module rack. Equality with
// the double-precision direct convolution in both modes, reported latency against measured latency, the true-stereo kernel
// matrix by channel impulses, the impulse response edits, kernel switches (steps against -60 dB), the mix and the
// allocation-free audio thread.

#include "AllocCounter.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/crucible.h>
#include <crucible/Patch.h>
#include <crucible/Fx.h>
#include <crucible/fx/IrEdit.h>
#include <crucible/graph/Compiler.h>
#include <crucible/module/ConvolutionIr.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <cmath>
#include <cstdint>
#include <functional>
#include <random>
#include <thread>
#include <vector>

using namespace crucible;
using crucible::test::String;
using crucible::test::UnitTest;
using crucible::testing::kRate;

namespace
{
constexpr double kPi = 3.14159265358979323846;
using Samples = std::vector<float>;

struct NoiseSource
{
    std::mt19937 gen;
    explicit NoiseSource(unsigned seed) : gen(seed) {}
    float uniform() { return static_cast<float>(gen() >> 8) / 8388608.0f - 1.0f; }
};

ImpulseResponse makeIr(IrLayout layout, std::vector<Samples> channels, double rate = kRate)
{
    ImpulseResponse ir;
    ir.sampleRate = rate;
    ir.layout = layout;
    ir.channels = std::move(channels);
    return ir;
}

Samples taps(std::initializer_list<std::pair<int, float>> list, int length)
{
    Samples k(static_cast<size_t>(length), 0.0f);
    for (const auto& t : list)
        k[static_cast<size_t>(t.first)] = t.second;
    return k;
}

Samples decayingNoise(int length, unsigned seed, float gain)
{
    NoiseSource rng(seed);
    Samples k(static_cast<size_t>(length));
    for (int i = 0; i < length; ++i)
        k[static_cast<size_t>(i)] =
            gain * rng.uniform() * std::exp(-5.0f * static_cast<float>(i) / static_cast<float>(length));
    return k;
}

// A module through the rack with every other effect off. High quality makes the kernel builds deterministic.
struct Harness
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values, legacy;
    ConvolutionIr* ir = nullptr;
    LatencyReporter* latency = nullptr;
    explicit Harness(Quality quality = Quality::High, double rate = kRate)
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        ctx.quality = quality;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set("fx_cx_on", 1.0f);
        set("fx_cx_mix", 1.0f);
        ir = rack.findCapability<ConvolutionIr>("fx.convolution");
        latency = rack.findCapability<LatencyReporter>("fx.convolution");
    }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ParameterMap::indexOf(id))] = v; }
    void run(Samples& l, Samples& r, int block = 100)
    {
        for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
        {
            const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos));
            rack.process(l.data() + pos, r.data() + pos, n, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
        }
    }
    void runRaw(float* l, float* r, int n)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
    }
    // Silence long enough for the settle time, the build and the crossfade to finish.
    void settle(int samples = 14000)
    {
        Samples l(static_cast<size_t>(samples), 0.0f), r = l;
        run(l, r);
    }
    // Runs the same input on both channels (or distinct ones) and returns the output.
    std::pair<Samples, Samples> render(const Samples& inL, const Samples& inR, int block = 100)
    {
        Samples l = inL, r = inR;
        run(l, r, block);
        return {l, r};
    }
};

// Double-precision direct convolution.
std::vector<double> direct(const Samples& x, const Samples& h, size_t outLength)
{
    std::vector<double> y(outLength, 0.0);
    for (size_t n = 0; n < outLength; ++n)
    {
        double s = 0.0;
        const size_t kmax = std::min(h.size() - 1, n);
        for (size_t k = 0; k <= kmax && n - k < x.size(); ++k)
            s += static_cast<double>(h[k]) * static_cast<double>(x[n - k]);
        y[n] = s;
    }
    return y;
}

double magnitudeDb(const Samples& h, double hz, double rate)
{
    double re = 0.0, im = 0.0;
    const double w = 2.0 * kPi * hz / rate;
    for (size_t n = 0; n < h.size(); ++n)
    {
        re += h[n] * std::cos(w * static_cast<double>(n));
        im -= h[n] * std::sin(w * static_cast<double>(n));
    }
    return 10.0 * std::log10(re * re + im * im + 1.0e-30);
}

double maxStep(const Samples& y, size_t from, size_t to)
{
    double m = 0.0;
    for (size_t i = std::max<size_t>(from, 1); i < std::min(to, y.size()); ++i)
        m = std::max(m, std::abs(static_cast<double>(y[i]) - static_cast<double>(y[i - 1])));
    return m;
}

size_t peakAt(const Samples& y)
{
    size_t at = 0;
    for (size_t i = 0; i < y.size(); ++i)
        if (std::abs(y[i]) > std::abs(y[at]))
            at = i;
    return at;
}

double peakOutside(const Samples& y, size_t keep)
{
    double m = 0.0;
    for (size_t i = 0; i < y.size(); ++i)
        if (i != keep)
            m = std::max(m, static_cast<double>(std::abs(y[i])));
    return m;
}
} // namespace

class ConvolutionTests : public UnitTest
{
public:
    ConvolutionTests() : UnitTest("Convolution", "Effects") {}

    void runTest() override
    {
        registration();
        equalsDirect();
        latencyMeasured();
        trueStereo();
        edits();
        editsThroughModule();
        kernelSwitch();
        kernelSwitchRealTime();
        mixAndPassthrough();
        responseView();
        allocations();
        engineLoadAndPatch();
        commonSilenceAndUnity();
        commonRates();
        commonZipperAndClick();
        commonRandomExtremes();
        commonOfflineDeterminism();
        renderOfflineFlag();
    }

private:
    void engineLoadAndPatch()
    {
        beginTest("engine: loadIr addresses each convolution instance; unknown and non-IR modules are errors");
        {
            namespace fs = std::filesystem;
            const fs::path dir = fs::temp_directory_path() / "crucible_cx_loadir_test";
            fs::create_directories(dir);
            const std::string pathA = (dir / "a.wav").string();
            const std::string pathB = (dir / "b.wav").string();
            AudioData a, b;
            a.sampleRate = b.sampleRate = 44100.0;
            a.channels.push_back(decayingNoise(2000, 11, 0.3f));
            a.channels.push_back(decayingNoise(2000, 12, 0.3f));
            b.channels.push_back(decayingNoise(3000, 21, 0.3f));
            b.channels.push_back(decayingNoise(3000, 22, 0.3f));
            std::string error;
            expect(writeWavFile(pathA, a, 32) && writeWavFile(pathB, b, 32), "wrote test files");

            const char* text = R"({"format": "crucible-product", "version": 1, "id": "two-conv", "voice": "none",
                "fx": {"instances": [{"id": "c1", "type": "fx.convolution"}, {"id": "c2", "type": "fx.convolution"}],
                       "lanes": [{"id": "l1", "order": "listed", "input": ["aux"], "chain": ["c1"]},
                                 {"id": "l2", "order": "listed", "input": ["aux"], "chain": ["c2"]}]}})";
            graph::ProductDef def;
            expect(graph::productFromText(text, def, error), error);
            EngineConfig cfg;
            cfg.compiled = graph::compile(def, error);
            expect(cfg.compiled != nullptr, error);
            if (cfg.compiled == nullptr)
                return;
            Engine e(cfg);
            e.prepare(kRate, 128);
            auto& rack = e.coreEngine().engine().effects();
            auto* first = rack.findCapability<ConvolutionIr>("fx.convolution");
            auto* second = rack.findCapability<ConvolutionIr>("fx.convolution.2");
            expect(first != nullptr && second != nullptr && first != second, "two instances");
            if (first == nullptr || second == nullptr)
                return;
            expect(e.loadIr("fx.convolution.2", pathB, error), error);
            expect(!first->hasImpulseResponse() && second->hasImpulseResponse() && second->impulseResponseName() == "b.wav",
                   "only the second instance loaded");
            expect(e.loadIr("fx.convolution", pathA, error), error);
            expect(first->impulseResponseName() == "a.wav" && second->impulseResponseName() == "b.wav",
                   "each keeps its own");
            expect(!e.loadIr("fx.nothing", pathA, error) && error.find("unknown") != std::string::npos, "unknown instance");
            expect(!e.loadIr("fx.convolution", (dir / "missing.wav").string(), error) && !error.empty(), "missing file");
            expect(first->impulseResponseName() == "a.wav", "failed load keeps the response");
        }

        beginTest(
            "patch: every convolution instance keeps its own IR; a version 2 convolutionIr loads into the first instance");
        {
            namespace fs = std::filesystem;
            const fs::path dir = fs::temp_directory_path() / "crucible_cx_patch_multi_test";
            fs::create_directories(dir);
            const std::string pathA = (dir / "a.wav").string();
            const std::string pathB = (dir / "b.wav").string();
            AudioData a, b;
            a.sampleRate = b.sampleRate = 44100.0;
            a.channels.push_back(decayingNoise(2000, 11, 0.3f));
            a.channels.push_back(decayingNoise(2000, 12, 0.3f));
            b.channels.push_back(decayingNoise(3000, 21, 0.3f));
            std::string error;
            expect(writeWavFile(pathA, a, 32) && writeWavFile(pathB, b, 32), "wrote test files");
            const char* text = R"({"format": "crucible-product", "version": 1, "id": "two-conv", "voice": "none",
                "fx": {"instances": [{"id": "c1", "type": "fx.convolution"}, {"id": "c2", "type": "fx.convolution"}],
                       "lanes": [{"id": "l1", "order": "listed", "input": ["aux"], "chain": ["c1"]},
                                 {"id": "l2", "order": "listed", "input": ["aux"], "chain": ["c2"]}]}})";
            graph::ProductDef def;
            expect(graph::productFromText(text, def, error), error);
            EngineConfig cfg;
            cfg.compiled = graph::compile(def, error);
            expect(cfg.compiled != nullptr, error);
            if (cfg.compiled == nullptr)
                return;
            Engine e(cfg);
            e.prepare(kRate, 128);
            expect(e.loadIr("fx.convolution", pathA, error) && e.loadIr("fx.convolution.2", pathB, error), error);
            auto& rack = e.coreEngine().engine().effects();
            auto* first = rack.findCapability<ConvolutionIr>("fx.convolution");
            auto* second = rack.findCapability<ConvolutionIr>("fx.convolution.2");
            ImpulseResponse irA, irB;
            std::string nameA, nameB;
            expect(first->impulseResponse(irA, nameA) && second->impulseResponse(irB, nameB), "both responses present");

            PatchMeta meta;
            meta.name = "two";
            const Patch p = capturePatch(e, meta);
            expect(p.convolutionIrs.size() == 2 && p.convolutionIrs[0].instance == "fx.convolution" &&
                       p.convolutionIrs[1].instance == "fx.convolution.2",
                   "both instances captured, keyed by id");
            Patch parsed;
            expect(patchFromJson(patchToJson(p), parsed, error), error);
            expect(parsed.version == kPatchFormatVersion && parsed.convolutionIrs.size() == 2,
                   "round trip keeps both entries");

            Engine other(cfg);
            other.prepare(kRate, 128);
            applyPatch(other, parsed);
            auto& rack2 = other.coreEngine().engine().effects();
            ImpulseResponse backA, backB;
            std::string backNameA, backNameB;
            expect(rack2.findCapability<ConvolutionIr>("fx.convolution")->impulseResponse(backA, backNameA) &&
                       rack2.findCapability<ConvolutionIr>("fx.convolution.2")->impulseResponse(backB, backNameB),
                   "both instances restored");
            expect(backNameA == "a.wav" && backNameB == "b.wav", "each instance has its own name");
            expect(backA.channels == irA.channels && backB.channels == irB.channels && backA.channels != backB.channels,
                   "each instance has its own samples");
            expect(backA.channels.size() == 2 && backB.channels.size() == 1, "layouts kept per instance");

            // A version 2 file: the single field loads into the first instance and the second stays empty.
            Patch oneEntry = parsed;
            oneEntry.convolutionIrs.erase(oneEntry.convolutionIrs.begin() + 1);
            json::Value doc;
            expect(json::parse(patchToJson(oneEntry), doc, error), error);
            json::Value legacy = json::Value::object();
            for (const auto& [key, value] : doc.entries())
                if (key == "convolutionIrs")
                    legacy.set("convolutionIr", value[0]);
                else if (key == "version")
                    legacy.set("version", 2);
                else
                    legacy.set(key, value);
            Patch fromOld;
            expect(patchFromJson(legacy.dump(0), fromOld, error), error);
            expect(fromOld.convolutionIrs.size() == 1 && fromOld.convolutionIrs[0].instance == "fx.convolution" &&
                       fromOld.convolutionIrs[0].channels == oneEntry.convolutionIrs[0].channels,
                   "old field migrated to the first instance");
            Engine third(cfg);
            third.prepare(kRate, 128);
            third.loadIr("fx.convolution.2", pathB, error);
            applyPatch(third, fromOld);
            auto& rack3 = third.coreEngine().engine().effects();
            expect(rack3.findCapability<ConvolutionIr>("fx.convolution")->hasImpulseResponse() &&
                       !rack3.findCapability<ConvolutionIr>("fx.convolution.2")->hasImpulseResponse(),
                   "old patch fills the first instance and clears the second");
            fs::remove_all(dir);
        }

        beginTest("engine: loadIr on a module without an IR is an error");
        {
            namespace fs = std::filesystem;
            const fs::path dir = fs::temp_directory_path() / "crucible_cx_loadir_test";
            fs::create_directories(dir);
            const std::string path = (dir / "legacy.wav").string();
            AudioData a;
            a.sampleRate = 44100.0;
            a.channels.push_back(decayingNoise(2000, 31, 0.3f));
            a.channels.push_back(decayingNoise(2000, 32, 0.3f));
            std::string error;
            expect(writeWavFile(path, a, 32), "wrote test file");
            Engine e;
            e.prepare(kRate, 128);
            expect(!e.loadIr("fx.saturator", path, error) && error.find("no impulse response") != std::string::npos, error);
            expect(!e.loadIr("fx.conv", path, error) && error.find("unknown module instance") != std::string::npos, error);
            expect(e.loadIr("fx.convolution", path, error), error);
        }

        beginTest("engine: IR file load, patch save / reload round trip, clearing");
        namespace fs = std::filesystem;
        const fs::path dir = fs::temp_directory_path() / "crucible_cx_engine_test";
        fs::create_directories(dir);
        const std::string path = (dir / "quad.wav").string();
        AudioData quad;
        quad.sampleRate = 44100.0;
        const int n = 3000;
        for (int c = 0; c < 4; ++c)
            quad.channels.push_back(decayingNoise(n, 70 + c, 0.3f));
        std::string error;
        expect(writeWavFile(path, quad, 32), "wrote test file");

        Engine e;
        e.prepare(kRate, 128);
        auto* cx = e.coreEngine().engine().effects().findCapability<ConvolutionIr>();
        expect(cx != nullptr, "capability in the rack");
        expect(!e.loadConvolutionIrFile((dir / "missing.wav").string(), error) && !error.empty(), "missing file reported");
        expect(!cx->hasImpulseResponse(), "nothing loaded after the failure");
        expect(e.loadConvolutionIrFile(path, error), error);
        expect(cx->hasImpulseResponse() && cx->impulseResponseName() == "quad.wav", "loaded");

        ImpulseResponse stored;
        std::string name;
        expect(cx->impulseResponse(stored, name) && stored.layout == IrLayout::TrueStereo && stored.channels.size() == 4,
               "true-stereo kept");
        const double storedRate = stored.sampleRate;

        PatchMeta meta;
        meta.name = "cx";
        const Patch p = capturePatch(e, meta);
        expect(p.convolutionIrs.size() == 1 && p.convolutionIrs[0].instance == "fx.convolution" &&
                   p.convolutionIrs[0].channels.size() == 4 && p.convolutionIrs[0].layout == "truestereo",
               "captured");
        Patch parsed;
        expect(patchFromJson(patchToJson(p), parsed, error), error);
        expect(parsed.convolutionIrs.size() == 1 && parsed.convolutionIrs[0].channels == stored.channels &&
                   parsed.convolutionIrs[0].name == "quad.wav" && parsed.convolutionIrs[0].rate == storedRate &&
                   parsed.convolutionIrs[0].layout == "truestereo",
               "JSON round trip is exact");

        Engine other;
        other.prepare(kRate, 128);
        applyPatch(other, parsed);
        auto* cx2 = other.coreEngine().engine().effects().findCapability<ConvolutionIr>();
        ImpulseResponse back;
        std::string backName;
        expect(cx2->impulseResponse(back, backName) && back.channels == stored.channels &&
                   back.layout == IrLayout::TrueStereo && back.sampleRate == storedRate && backName == "quad.wav",
               "reloaded response equals the saved one");

        // A patch without the field (older patches) clears the response and loads as before.
        Patch old = parsed;
        old.convolutionIrs.clear();
        applyPatch(other, old);
        expect(!cx2->hasImpulseResponse(), "patch without a response clears it");
        const std::string oldJson = patchToJson(old);
        expect(oldJson.find("convolutionIr") == std::string::npos && oldJson.find("impulseResponse") == std::string::npos,
               "old patch format unchanged");

        {
            std::string clearError;
            expect(e.clearIr("fx.convolution", clearError), "clearIr succeeds");
            expect(!e.clearIr("fx.nothing", clearError) && !clearError.empty(), "clearIr rejects an unknown instance");
        }
        expect(!cx->hasImpulseResponse(), "cleared");
        expect(capturePatch(e, meta).convolutionIrs.empty(), "nothing captured after clearing");
        fs::remove_all(dir);
    }

    // ---- common acceptance (docs/plans/ENGINE-FX-PLAN.md section 3)
    static Samples sine(size_t n, double hz, double rate, double amp)
    {
        Samples x(n);
        for (size_t i = 0; i < n; ++i)
            x[i] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * static_cast<double>(i) / rate));
        return x;
    }
    static crucible::measure::Signal toSignal(const Samples& x) { return crucible::measure::Signal(x.begin(), x.end()); }
    static const ParamDef& paramDef(const std::string& id)
    {
        return ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf(id))];
    }

    // A smooth response around the test carrier (an exponentially decaying 1 kHz cosine, 0.15 s time constant): a sine that
    // starts abruptly or is modulated gives smooth output, so the step estimators of the zipper and click tests only see
    // steps the module itself makes.
    static Samples carrierIr(int length, double rate, double phase)
    {
        Samples k(static_cast<size_t>(length));
        for (int i = 0; i < length; ++i)
        {
            const double t = i / rate;
            k[static_cast<size_t>(i)] =
                static_cast<float>(2.8e-4 * std::exp(-t / 0.15) * std::cos(2.0 * kPi * 1000.0 * t + phase));
        }
        return k;
    }
    // Scaled so that the sum of the magnitudes is 1: a bound for the gain of the kernel on any input.
    static Samples unitL1(Samples k)
    {
        double s = 0.0;
        for (const float v : k)
            s += std::abs(v);
        for (auto& v : k)
            v = static_cast<float>(v / s);
        return k;
    }

    void commonSilenceAndUnity()
    {
        beginTest("silence in gives exact silence out; unity small-signal gain at neutral settings");
        for (const bool withIr : {false, true})
        {
            Harness h(Quality::Standard);
            if (withIr)
                h.ir->setImpulseResponse(
                    makeIr(IrLayout::TrueStereo, {decayingNoise(30000, 1, 0.3f), decayingNoise(30000, 2, 0.3f),
                                                  decayingNoise(30000, 3, 0.3f), decayingNoise(30000, 4, 0.3f)}),
                    "quad");
            Samples l(96000, 0.0f), r = l;
            h.run(l, r, 480);
            float peak = 0.0f;
            for (size_t i = 0; i < l.size(); ++i)
                peak = std::max({peak, std::abs(l[i]), std::abs(r[i])});
            expect(peak == 0.0f,
                   String(withIr ? "with" : "without") + " response: peak " + String(static_cast<double>(peak)));
        }
        for (const int mode : {0, 1})
        {
            Harness h;
            h.set("fx_cx_mode", static_cast<float>(mode));
            h.ir->setImpulseResponse(makeIr(IrLayout::Mono, {taps({{0, 1.0f}}, 64)}), "delta");
            h.settle();
            const size_t n = 48000;
            Samples l = sine(n, 1000.0, kRate, 0.01), r = l; // -40 dBFS
            h.run(l, r, 128);
            const auto y = toSignal(l);
            const double gain = crucible::measure::amplitudeToDb(
                crucible::measure::toneAmplitude(y.data() + 24000, 24000, 1000.0, kRate) / 0.01);
            logMessage(String("mode ") + String(mode) + ": small-signal gain " + String(gain, 3) + " dB");
            expect(std::abs(gain) <= 0.2, "unity within 0.2 dB");
        }
    }

    void commonRates()
    {
        beginTest("44.1 / 48 / 96 / 192 kHz: equals direct convolution, measured latency equals reported, unity gain");
        for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
            for (const int mode : {0, 1})
            {
                Harness h(Quality::High, rate);
                h.set("fx_cx_mode", static_cast<float>(mode));
                const Samples k = decayingNoise(6000, 11, 0.2f);
                h.ir->setImpulseResponse(makeIr(IrLayout::Mono, {k}, rate), "noise");
                h.settle(static_cast<int>(rate * 0.3));
                NoiseSource rng(5);
                const size_t n = static_cast<size_t>(rate * 0.25);
                Samples in(n);
                for (auto& v : in)
                    v = 0.4f * rng.uniform();
                auto out = h.render(in, in, 200);
                const int latency = h.latency->latencySamples();
                const auto ref = direct(in, k, n);
                double err = 0.0, sig = 0.0;
                for (size_t i = static_cast<size_t>(latency); i < n; ++i)
                {
                    const double d = static_cast<double>(out.first[i]) - ref[i - static_cast<size_t>(latency)];
                    err += d * d;
                    sig += ref[i - static_cast<size_t>(latency)] * ref[i - static_cast<size_t>(latency)];
                }
                const double errDb = 10.0 * std::log10(err / sig + 1.0e-30);
                expect(errDb < -100.0, String(rate, 0) + " Hz mode " + String(mode) + ": error " + String(errDb, 1) + " dB");

                Harness d(Quality::High, rate);
                d.set("fx_cx_mode", static_cast<float>(mode));
                d.ir->setImpulseResponse(makeIr(IrLayout::Mono, {taps({{0, 1.0f}}, 64)}, rate), "delta");
                d.settle(static_cast<int>(rate * 0.3));
                Samples l(4000, 0.0f), r = l;
                l[0] = r[0] = 1.0f;
                const auto o = d.render(l, r);
                expectEquals(static_cast<int>(peakAt(o.first)), d.latency->latencySamples(),
                             String(rate, 0) + " Hz latency");
                expectWithinAbsoluteError(o.first[peakAt(o.first)], 1.0f, 1.0e-4, "unity");
            }
    }

    void commonZipperAndClick()
    {
        beginTest("zipper below -90 dB for each continuous parameter, click below -60 dB for each choice change");
        const char* continuous[] = {"fx_cx_mix",     "fx_cx_out",     "fx_cx_trim",     "fx_cx_length",
                                    "fx_cx_stretch", "fx_cx_fadein",  "fx_cx_fadeout",  "fx_cx_balance",
                                    "fx_cx_lowcut",  "fx_cx_highcut", "fx_cx_predelay", "fx_cx_width"};
        const Samples ir = carrierIr(24000, kRate, 0.3);
        const Samples irR = carrierIr(24000, kRate, 1.1);
        for (const char* id : continuous)
        {
            const ParamDef& def = paramDef(id);
            const auto render = [&](const crucible::measure::Signal& in, const crucible::measure::Signal& automation)
            {
                Harness h(Quality::High);
                h.ir->setImpulseResponse(makeIr(IrLayout::Stereo, {ir, irR}), "carrier");
                h.settle();
                Samples l(in.begin(), in.end()), r = l;
                for (size_t pos = 0; pos < l.size(); pos += 32)
                {
                    const size_t end = std::min(pos + 32, l.size());
                    h.set(id, static_cast<float>(def.min + (def.max - def.min) * automation[end - 1]));
                    h.runRaw(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
                }
                return crucible::measure::Signal(l.begin(), l.end());
            };
            const auto z = crucible::measure::zipperTest(render, kRate, 1000.0, -6.0);
            logMessage(String(id) + ": ramp step " + String(z.rampDb, 1) + " dB, square " + String(z.squareDb, 1) + " dB");
            expect(z.rampDb <= -90.0 && z.squareDb <= -90.0, String(id) + " zipper");
        }
        // Choice changes: mode, reverse, replaced response, on / off.
        const auto clickAfter = [&](const std::function<void(Harness&)>& change, const char* what)
        {
            Harness h(Quality::High);
            h.set("fx_cx_mix", 0.6f);
            h.ir->setImpulseResponse(makeIr(IrLayout::Stereo, {ir, irR}), "carrier");
            h.settle();
            // 24000 samples of a 1 kHz sine at 48 kHz are whole periods, so the carrier continues across the halves.
            const Samples l2 = sine(48000, 1000.0, kRate, 0.5);
            Samples a(l2.begin(), l2.begin() + 24000), ar(a);
            h.run(a, ar, 64);
            change(h);
            Samples b(l2.begin() + 24000, l2.end()), br(b);
            h.run(b, br, 64);
            a.insert(a.end(), b.begin(), b.end());
            const auto c = crucible::measure::clickTest(toSignal(a), 1000.0, kRate, 6000);
            logMessage(String(what) + ": largest step " + String(c.stepDb, 1) + " dB");
            expect(c.stepDb <= -60.0, String(what) + " click");
        };
        clickAfter([](Harness& h) { h.set("fx_cx_mode", 1.0f); }, "mode change");
        clickAfter([](Harness& h) { h.set("fx_cx_reverse", 1.0f); }, "reverse");
        clickAfter([&](Harness& h)
                   { h.ir->setImpulseResponse(makeIr(IrLayout::Mono, {carrierIr(9000, kRate, 2.0)}), "other"); },
                   "replaced response");
        clickAfter([](Harness& h) { h.ir->clearImpulseResponse(); }, "cleared response");
        clickAfter([](Harness& h) { h.set("fx_cx_on", 0.0f); }, "off");
    }

    void commonRandomExtremes()
    {
        beginTest("60 s at random parameter extremes stays finite and bounded, in both quality settings");
        for (const Quality quality : {Quality::Standard, Quality::High})
        {
            Harness h(quality);
            std::mt19937 gen(quality == Quality::High ? 91u : 17u);
            const auto uni = [&] { return static_cast<float>(gen() >> 8) / 16777216.0f; };
            h.ir->setImpulseResponse(
                makeIr(IrLayout::TrueStereo, {unitL1(decayingNoise(60000, 1, 1.0f)), unitL1(decayingNoise(60000, 2, 1.0f)),
                                              unitL1(decayingNoise(60000, 3, 1.0f)), unitL1(decayingNoise(60000, 4, 1.0f))}),
                "quad");
            NoiseSource noise(3);
            const int block = 480;
            const int blocks = static_cast<int>(60.0 * kRate / block);
            double peak = 0.0;
            bool finite = true;
            for (int b = 0; b < blocks; ++b)
            {
                if (b % 25 == 0)
                    for (const auto& def : ParameterMap::params())
                        if (def.id.rfind("fx_cx_", 0) == 0 && def.id != "fx_cx_on")
                        {
                            const float u = uni();
                            const float v =
                                u < 0.3f ? def.min : (u < 0.6f ? def.max : def.min + (def.max - def.min) * uni());
                            h.set(def.id, def.type == ParamType::Float ? v : std::round(v));
                        }
                if (b % 2500 == 1200)
                    h.ir->setImpulseResponse(
                        makeIr(b % 5000 == 1200 ? IrLayout::Mono : IrLayout::Stereo,
                               {unitL1(decayingNoise(30000, 100 + b, 1.0f)), unitL1(decayingNoise(30000, 200 + b, 1.0f))}),
                        "swap");
                Samples l(block), r(block);
                for (int i = 0; i < block; ++i)
                {
                    l[static_cast<size_t>(i)] = 0.5f * noise.uniform();
                    r[static_cast<size_t>(i)] = 0.5f * noise.uniform();
                }
                h.run(l, r, block);
                for (int i = 0; i < block; ++i)
                {
                    finite = finite && std::isfinite(l[static_cast<size_t>(i)]) && std::isfinite(r[static_cast<size_t>(i)]);
                    peak = std::max({peak, static_cast<double>(std::abs(l[static_cast<size_t>(i)])),
                                     static_cast<double>(std::abs(r[static_cast<size_t>(i)]))});
                }
                if (quality == Quality::Standard)
                    std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
            logMessage(String(quality == Quality::High ? "High" : "Standard") + ": 60 s, peak " + String(peak, 2));
            expect(finite, "finite");
            // Input at most 0.5, each kernel's magnitudes sum to 1 (two of them add per output), at most +24 dB output,
            // +12 dB early / late and 2x width: 0.5 * 2 * 16 * 4 * 2 = 128, with the same margin again for the filters.
            expect(peak < 256.0, "bounded: " + std::to_string(peak));
        }
    }

    void commonOfflineDeterminism()
    {
        beginTest("offline (High quality) output does not depend on builder-thread timing: two renders are bit-identical");
        const auto render = [&](unsigned jitterSeed)
        {
            Harness h(Quality::High);
            h.ir->setImpulseResponse(
                makeIr(IrLayout::Stereo, {decayingNoise(30000, 1, 0.2f), decayingNoise(30000, 2, 0.2f)}), "a");
            std::mt19937 jitter(jitterSeed);
            std::mt19937 edits(4); // the same edits in both renders
            NoiseSource noise(8);
            Samples out;
            const int block = 64;
            for (int b = 0; b < 3000; ++b)
            {
                if (b % 40 == 0)
                {
                    h.set("fx_cx_highcut", 1000.0f + static_cast<float>(edits() % 19000));
                    h.set("fx_cx_predelay", static_cast<float>(edits() % 200));
                    h.set("fx_cx_mode", static_cast<float>(edits() % 2));
                    h.set("fx_cx_reverse", static_cast<float>(edits() % 2));
                }
                if (b % 700 == 350)
                    h.ir->setImpulseResponse(
                        makeIr(IrLayout::Stereo, {decayingNoise(20000, 50 + b, 0.2f), decayingNoise(20000, 60 + b, 0.2f)}),
                        "b");
                Samples l(block), r(block);
                for (int i = 0; i < block; ++i)
                    l[static_cast<size_t>(i)] = r[static_cast<size_t>(i)] = 0.4f * noise.uniform();
                if (jitterSeed != 0 && jitter() % 3 == 0)
                    std::this_thread::sleep_for(std::chrono::microseconds(jitter() % 1500));
                h.run(l, r, block);
                out.insert(out.end(), l.begin(), l.end());
                out.insert(out.end(), r.begin(), r.end());
            }
            return out;
        };
        const Samples first = render(0);
        const Samples second = render(1);
        const Samples third = render(2);
        expect(first.size() == second.size() && std::memcmp(first.data(), second.data(), first.size() * sizeof(float)) == 0,
               "render 1 equals render 2");
        expect(first.size() == third.size() && std::memcmp(first.data(), third.data(), first.size() * sizeof(float)) == 0,
               "render 1 equals render 3");
    }

    void renderOfflineFlag()
    {
        beginTest("crucible_set_render_offline: at Standard quality a render right after crucible_load_ir, with cabinet "
                  "changes, does not depend on builder-thread timing");
        namespace fs = std::filesystem;
        const fs::path dir = fs::temp_directory_path() / "crucible_cx_render_offline_test";
        fs::create_directories(dir);
        const std::string path = (dir / "ir.wav").string();
        AudioData data;
        data.sampleRate = 44100.0;
        data.channels.push_back(decayingNoise(30000, 31, 0.3f));
        data.channels.push_back(decayingNoise(30000, 32, 0.3f));
        expect(writeWavFile(path, data, 32), "wrote test file");

        const auto render = [&](unsigned jitterSeed)
        {
            CrucibleConfig config;
            crucible_config_init(&config);
            config.quality = 1;
            CrucibleEngine* ce = crucible_create(&config);
            crucible_prepare(ce, kRate, 256);
            expect(crucible_render_offline(ce) == 0, "real time by default");
            crucible_set_render_offline(ce, 1);
            expect(crucible_render_offline(ce) == 1, "offline once set");
            crucible_set_param_id(ce, "amp_sustain", 1.0f);
            crucible_set_param_id(ce, "fx_cx_on", 1.0f);
            crucible_set_param_id(ce, "fx_cx_mix", 1.0f);
            crucible_set_param_id(ce, "fx_cab_on", 1.0f);
            char error[256] = "";
            expect(crucible_load_ir(ce, "fx.convolution", path.c_str(), error, sizeof error) == 1, error);
            crucible_note_on(ce, 45, 1.0f);
            std::mt19937 jitter(jitterSeed);
            Samples out;
            std::vector<float> l(256), r(256);
            for (int b = 0; b < 400; ++b)
            {
                if (b % 100 == 50)
                    crucible_set_param_id(ce, "fx_cab_cabinet", static_cast<float>((b / 100) % 4));
                if (jitterSeed != 0 && jitter() % 3 == 0)
                    std::this_thread::sleep_for(std::chrono::microseconds(jitter() % 1500));
                crucible_process(ce, l.data(), r.data(), 256, nullptr, 0);
                out.insert(out.end(), l.begin(), l.end());
            }
            crucible_destroy(ce);
            return out;
        };
        const Samples first = render(0);
        const Samples second = render(1);
        const Samples third = render(2);
        double energy = 0.0;
        for (const float v : first)
            energy += static_cast<double>(v) * v;
        expect(energy > 0.0, "the render is not silent");
        expect(first.size() == second.size() && std::memcmp(first.data(), second.data(), first.size() * sizeof(float)) == 0,
               "render 1 equals render 2");
        expect(first.size() == third.size() && std::memcmp(first.data(), third.data(), first.size() * sizeof(float)) == 0,
               "render 1 equals render 3");
    }

    void registration()
    {
        beginTest("fx.convolution is registered after fx.reverb_algo and before fx.delay_pro, and appends its parameters");
        const auto& registry = ModuleRegistry::instance();
        const auto effects = registry.ofKind(ModuleKind::Effect);
        expect(registry.find("fx.convolution") != nullptr, "registered");
        expectEquals(static_cast<int>(effects.size()), kNumModuleFx, "effects in the registry");
        expect(effects.size() >= 12 && effects[effects.size() - 12]->id == "fx.convolution",
               "twelfth to last effect (fx.delay_pro, fx.transient, fx.deesser, the FX11 modules, fx.exciter, fx.saturator, "
               "fx.shaper, fx.multiband_drive and fx.cabinet follow)");
        expect(ParameterMap::indexOf("fx_pos_cx") + 1 == ParameterMap::indexOf("fx_delay_pro_on"),
               "its parameters are followed directly by those of fx.delay_pro");
        Harness h;
        expect(h.ir != nullptr && h.latency != nullptr, "capabilities");
    }

    // The module output equals the double-precision direct convolution, in both modes and for several block sizes.
    void equalsDirect()
    {
        beginTest("output equals the direct convolution to -100 dB in both modes");
        for (const int mode : {0, 1})
            for (const int block : {64, 100, 480})
            {
                Harness h;
                h.set("fx_cx_mode", static_cast<float>(mode));
                const Samples kernelL = decayingNoise(9000, 3, 0.2f), kernelR = decayingNoise(7000, 4, 0.2f);
                h.ir->setImpulseResponse(makeIr(IrLayout::Stereo, {kernelL, kernelR}), "noise");
                h.settle();
                NoiseSource rng(7);
                Samples inL(24000), inR(24000);
                for (size_t i = 0; i < inL.size(); ++i)
                {
                    inL[i] = 0.3f * rng.uniform();
                    inR[i] = 0.3f * rng.uniform();
                }
                const auto out = h.render(inL, inR, block);
                const int lat = h.latency->latencySamples();
                const auto refL = direct(inL, kernelL, inL.size()), refR = direct(inR, kernelR, inR.size());
                double err = 0.0, ref = 0.0;
                for (size_t n = static_cast<size_t>(lat); n < inL.size(); ++n)
                {
                    const double dl = out.first[n] - refL[n - static_cast<size_t>(lat)],
                                 dr = out.second[n] - refR[n - static_cast<size_t>(lat)];
                    err += dl * dl + dr * dr;
                    ref += refL[n - static_cast<size_t>(lat)] * refL[n - static_cast<size_t>(lat)] +
                           refR[n - static_cast<size_t>(lat)] * refR[n - static_cast<size_t>(lat)];
                }
                const double db = 10.0 * std::log10(err / ref + 1.0e-30);
                logMessage(String("mode ") + String(mode) + " block " + String(block) + ": error " + String(db, 1) + " dB");
                expect(db < -100.0, "error against the direct convolution");
            }
    }

    // Reported latency equals the measured delay of an impulse through a delta response.
    void latencyMeasured()
    {
        beginTest("reported latency equals measured latency in both modes");
        for (const int mode : {0, 1})
        {
            Harness h;
            h.set("fx_cx_mode", static_cast<float>(mode));
            h.ir->setImpulseResponse(makeIr(IrLayout::Mono, {taps({{0, 1.0f}}, 64)}), "delta");
            h.settle();
            Samples l(3000, 0.0f), r = l;
            l[0] = 1.0f;
            r[0] = 1.0f;
            const auto out = h.render(l, r);
            const int reported = h.latency->latencySamples();
            const size_t measured = peakAt(out.first);
            logMessage(String("mode ") + String(mode) + ": reported " + String(reported) + ", measured " +
                       String(static_cast<int>(measured)));
            expectEquals(static_cast<int>(measured), reported, "measured latency");
            expectEquals(static_cast<int>(peakAt(out.second)), reported, "right channel");
            expectWithinAbsoluteError(out.first[measured], 1.0f, 1.0e-4, "unity gain");
            expect(peakOutside(out.first, measured) < 1.0e-5, "nothing else");
            expectEquals(reported, mode == 0 ? 0 : 128, "latency of the mode");
            // The flag fires once after a mode change.
            h.latency->takeLatencyChanged();
            h.set("fx_cx_mode", mode == 0 ? 1.0f : 0.0f);
            h.settle(2000);
            expect(h.latency->takeLatencyChanged(), "latency change announced");
            expect(!h.latency->takeLatencyChanged(), "only once");
            expectEquals(h.latency->latencySamples(), mode == 0 ? 128 : 0, "new latency");
        }
    }

    // Channel impulses through every kernel of the true-stereo matrix.
    void trueStereo()
    {
        beginTest("true-stereo matrix: each input channel reaches each output channel through its own kernel");
        for (const int mode : {0, 1})
        {
            Harness h;
            h.set("fx_cx_mode", static_cast<float>(mode));
            // LL, LR, RL, RR: input side first.
            h.ir->setImpulseResponse(makeIr(IrLayout::TrueStereo, {taps({{10, 0.5f}}, 100), taps({{20, 0.25f}}, 100),
                                                                   taps({{30, -0.75f}}, 100), taps({{40, 0.125f}}, 100)}),
                                     "matrix");
            h.settle();
            const int lat = h.latency->latencySamples();
            Samples impulse(1000, 0.0f), silent(1000, 0.0f);
            impulse[0] = 1.0f;
            const auto fromLeft = h.render(impulse, silent);
            h.settle(2000);
            const auto fromRight = h.render(silent, impulse);
            const auto at = [&](const Samples& y, int pos)
            { return static_cast<double>(y[static_cast<size_t>(pos + lat)]); };
            expectWithinAbsoluteError(at(fromLeft.first, 10), 0.5, 1.0e-5, "L to L");
            expectWithinAbsoluteError(at(fromLeft.second, 20), 0.25, 1.0e-5, "L to R");
            expectWithinAbsoluteError(at(fromRight.first, 30), -0.75, 1.0e-5, "R to L");
            expectWithinAbsoluteError(at(fromRight.second, 40), 0.125, 1.0e-5, "R to R");
            expect(peakOutside(fromLeft.first, static_cast<size_t>(10 + lat)) < 1.0e-5, "L to L only one tap");
            expect(peakOutside(fromLeft.second, static_cast<size_t>(20 + lat)) < 1.0e-5, "L to R only one tap");
            expect(peakOutside(fromRight.first, static_cast<size_t>(30 + lat)) < 1.0e-5, "R to L only one tap");
            expect(peakOutside(fromRight.second, static_cast<size_t>(40 + lat)) < 1.0e-5, "R to R only one tap");
            logMessage(String("mode ") + String(mode) + ": LL " + String(at(fromLeft.first, 10), 4) + " LR " +
                       String(at(fromLeft.second, 20), 4) + " RL " + String(at(fromRight.first, 30), 4) + " RR " +
                       String(at(fromRight.second, 40), 4));
        }

        beginTest("stereo response keeps the channels apart; a mono response serves both");
        {
            Harness h;
            h.ir->setImpulseResponse(makeIr(IrLayout::Stereo, {taps({{10, 0.5f}}, 100), taps({{20, 0.25f}}, 100)}),
                                     "stereo");
            h.settle();
            Samples impulse(500, 0.0f), silent(500, 0.0f);
            impulse[0] = 1.0f;
            const auto a = h.render(impulse, silent);
            expectWithinAbsoluteError(a.first[10], 0.5, 1.0e-5, "L to L");
            expect(peakOutside(a.second, 100000) < 1.0e-6, "nothing reaches the right output from the left input");
            h.settle(2000);
            const auto b = h.render(silent, impulse);
            expectWithinAbsoluteError(b.second[20], 0.25, 1.0e-5, "R to R");
            expect(peakOutside(b.first, 100000) < 1.0e-6, "nothing reaches the left output from the right input");

            Harness m;
            m.ir->setImpulseResponse(makeIr(IrLayout::Mono, {taps({{15, 0.5f}}, 100)}), "mono");
            m.settle();
            const auto c = m.render(impulse, impulse);
            expectWithinAbsoluteError(c.first[15], 0.5, 1.0e-5, "left");
            expectWithinAbsoluteError(c.second[15], 0.5, 1.0e-5, "right");
        }
    }

    // The edits themselves (applyIrEdit), measured on the kernel.
    void edits()
    {
        beginTest("response edits: trim, length, reverse, stretch, fades, balance, cuts, pre-delay, width");
        {
            Samples ramp(48000);
            for (size_t i = 0; i < ramp.size(); ++i)
                ramp[i] = static_cast<float>(i);
            const ImpulseResponse source = makeIr(IrLayout::Mono, {ramp});
            expect(applyIrEdit(source, IrEditSettings{}).channels[0] == ramp,
                   "default settings leave the response bit-identical");

            IrEditSettings e;
            e.trimMs = 10.0;
            auto out = applyIrEdit(source, e).channels[0];
            expectEquals(out[0], 480.0f, "trim start drops 480 samples");
            expectEquals(static_cast<int>(out.size()), 48000 - 480, "trim length");

            e = IrEditSettings{};
            e.lengthPercent = 25.0;
            expectEquals(static_cast<int>(applyIrEdit(source, e).channels[0].size()), 12000, "length");

            e = IrEditSettings{};
            e.reverse = true;
            out = applyIrEdit(source, e).channels[0];
            expectEquals(out.front(), 47999.0f, "reverse first");
            expectEquals(out.back(), 0.0f, "reverse last");

            e = IrEditSettings{};
            e.predelayMs = 10.0;
            out = applyIrEdit(source, e).channels[0];
            expectEquals(static_cast<int>(out.size()), 48000 + 480, "pre-delay length");
            expectEquals(out[479], 0.0f, "silence before");
            expectEquals(out[480], 0.0f, "first sample after (ramp starts at 0)");
            expectEquals(out[481], 1.0f, "ramp follows");
        }
        {
            Samples tone(9600);
            for (size_t i = 0; i < tone.size(); ++i)
                tone[i] = static_cast<float>(std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
            IrEditSettings e;
            e.stretch = 2.0;
            const auto out = applyIrEdit(makeIr(IrLayout::Mono, {tone}), e).channels[0];
            expectWithinAbsoluteError(static_cast<int>(out.size()), 19200, 2, "stretch doubles the length");
            const double low = magnitudeDb(out, 500.0, kRate), high = magnitudeDb(out, 1000.0, kRate);
            logMessage(String("stretch 2 of a 1 kHz tone: 500 Hz ") + String(low, 1) + " dB, 1 kHz " + String(high, 1) +
                       " dB");
            expect(low - high > 40.0, "the tone moved down an octave");
            e.stretch = 0.5;
            const auto half = applyIrEdit(makeIr(IrLayout::Mono, {tone}), e).channels[0];
            expectWithinAbsoluteError(static_cast<int>(half.size()), 4800, 2, "stretch 0.5 halves the length");
            expect(magnitudeDb(half, 2000.0, kRate) - magnitudeDb(half, 1000.0, kRate) > 40.0,
                   "the tone moved up an octave");
        }
        {
            const ImpulseResponse ones = makeIr(IrLayout::Mono, {Samples(24000, 1.0f)});
            IrEditSettings e;
            e.fadeInMs = 10.0;
            e.fadeOutMs = 20.0;
            auto out = applyIrEdit(ones, e).channels[0];
            expectEquals(out.front(), 0.0f, "fade in starts at zero");
            expectWithinAbsoluteError(out[480], 1.0f, 1.0e-6, "fade in ends at one");
            expectWithinAbsoluteError(out.back(), 0.0f, 1.0e-6, "fade out ends at zero");
            expectWithinAbsoluteError(out[24000 - 961], 1.0f, 1.0e-6, "fade out starts at one");
            e = IrEditSettings{};
            e.balanceDb = 6.0;
            out = applyIrEdit(ones, e).channels[0];
            expectWithinAbsoluteError(out.front(), std::pow(10.0, -3.0 / 20.0), 1.0e-4,
                                      "early is lowered by half the balance");
            expectWithinAbsoluteError(out.back(), std::pow(10.0, 3.0 / 20.0), 1.0e-4, "late is raised by half the balance");
            expectWithinAbsoluteError(out[3840], 1.0, 1.0e-3, "neutral at 80 ms");
        }
        {
            const ImpulseResponse delta = makeIr(IrLayout::Mono, {taps({{0, 1.0f}}, 16384)});
            IrEditSettings e;
            e.lowCutHz = 1000.0;
            auto out = applyIrEdit(delta, e).channels[0];
            const double a = magnitudeDb(out, 100.0, kRate), b = magnitudeDb(out, 10000.0, kRate),
                         c = magnitudeDb(out, 1000.0, kRate);
            logMessage(String("low cut 1 kHz: 100 Hz ") + String(a, 1) + " dB, 1 kHz " + String(c, 1) + " dB, 10 kHz " +
                       String(b, 2) + " dB");
            expect(a < -60.0, "low cut attenuates");
            expectWithinAbsoluteError(b, 0.0, 0.1, "pass band");
            expectWithinAbsoluteError(c, -3.0, 0.3, "corner");
            e = IrEditSettings{};
            e.highCutHz = 2000.0;
            out = applyIrEdit(delta, e).channels[0];
            const double d = magnitudeDb(out, 10000.0, kRate), f = magnitudeDb(out, 200.0, kRate),
                         g = magnitudeDb(out, 2000.0, kRate);
            logMessage(String("high cut 2 kHz: 10 kHz ") + String(d, 1) + " dB, 200 Hz " + String(f, 2) + " dB, 2 kHz " +
                       String(g, 1) + " dB");
            expect(d < -40.0, "high cut attenuates");
            expectWithinAbsoluteError(f, 0.0, 0.1, "pass band");
            expectWithinAbsoluteError(g, -3.0, 0.3, "corner");
        }
        {
            const ImpulseResponse stereo = makeIr(IrLayout::Stereo, {taps({{5, 1.0f}}, 20), taps({{5, 0.0f}}, 20)});
            IrEditSettings e;
            e.width = 0.0;
            auto out = applyIrEdit(stereo, e);
            expectWithinAbsoluteError(out.channels[0][5], 0.5, 1.0e-6, "width 0: mid left");
            expectWithinAbsoluteError(out.channels[1][5], 0.5, 1.0e-6, "width 0: mid right");
            e.width = 2.0;
            out = applyIrEdit(stereo, e);
            expectWithinAbsoluteError(out.channels[0][5], 1.5, 1.0e-6, "width 2: left");
            expectWithinAbsoluteError(out.channels[1][5], -0.5, 1.0e-6, "width 2: right");
            const ImpulseResponse quad = makeIr(IrLayout::TrueStereo, {taps({{1, 1.0f}}, 4), taps({{1, 0.0f}}, 4),
                                                                       taps({{1, 0.0f}}, 4), taps({{1, 1.0f}}, 4)});
            e.width = 0.0;
            out = applyIrEdit(quad, e);
            for (size_t c = 0; c < 4; ++c)
                expectWithinAbsoluteError(out.channels[c][1], 0.5, 1.0e-6, "true stereo width 0");
            const ImpulseResponse mono = makeIr(IrLayout::Mono, {taps({{1, 1.0f}}, 4)});
            expect(applyIrEdit(mono, e).channels[0] == mono.channels[0], "mono is not touched by width");
        }
    }

    void editsThroughModule()
    {
        beginTest("edits set as parameters are applied off the audio thread and reach the output");
        Harness h;
        h.ir->setImpulseResponse(makeIr(IrLayout::Mono, {taps({{100, 1.0f}}, 2000)}), "delta");
        h.settle();
        Samples impulse(3000, 0.0f);
        impulse[0] = 1.0f;
        auto out = h.render(impulse, impulse);
        expectEquals(static_cast<int>(peakAt(out.first)), 100, "unedited position");
        h.set("fx_cx_predelay", 10.0f);
        h.settle();
        out = h.render(impulse, impulse);
        expectEquals(static_cast<int>(peakAt(out.first)), 580, "10 ms pre-delay moves the tap by 480 samples");
        h.set("fx_cx_predelay", 0.0f);
        h.set("fx_cx_trim", 1.0f); // 48 samples
        h.settle();
        out = h.render(impulse, impulse);
        expectEquals(static_cast<int>(peakAt(out.first)), 52, "1 ms trim moves it the other way");
        h.set("fx_cx_trim", 0.0f);
        h.set("fx_cx_reverse", 1.0f);
        h.settle();
        out = h.render(impulse, impulse);
        expectEquals(static_cast<int>(peakAt(out.first)), 1899, "reverse");
        h.set("fx_cx_reverse", 0.0f);
        h.set("fx_cx_length", 3.0f); // 60 samples: the tap at 100 is gone
        h.settle();
        out = h.render(impulse, impulse);
        expect(peakOutside(out.first, 100000) < 1.0e-6, "length cuts the tap off");
    }

    // A kernel switch leaves no step above -60 dB (a steady sine through the old and the new kernel).
    void kernelSwitch()
    {
        beginTest("kernel switches crossfade without a step above -60 dB");
        struct Case
        {
            const char* name;
            std::function<void(Harness&)> change;
        };
        const std::vector<Case> cases = {
            {"high cut 20 kHz to 600 Hz", [](Harness& h) { h.set("fx_cx_highcut", 600.0f); }},
            {"reverse", [](Harness& h) { h.set("fx_cx_reverse", 1.0f); }},
            {"stretch 1.5", [](Harness& h) { h.set("fx_cx_stretch", 1.5f); }},
            {"new response",
             [](Harness& h)
             {
                 h.ir->setImpulseResponse(
                     makeIr(IrLayout::Stereo, {decayingNoise(20000, 21, 0.4f), decayingNoise(20000, 22, 0.4f)}), "b");
             }},
            {"zero latency to low CPU (mix 0.5)", [](Harness& h) { h.set("fx_cx_mode", 1.0f); }},
            {"low CPU to zero latency (mix 0.5)", [](Harness& h) { h.set("fx_cx_mode", 0.0f); }},
            {"response removed", [](Harness& h) { h.ir->clearImpulseResponse(); }},
        };
        for (const auto& c : cases)
        {
            Harness h;
            const bool toZero = std::string(c.name).rfind("low CPU to zero", 0) == 0;
            h.set("fx_cx_mode", toZero ? 1.0f : 0.0f);
            h.set("fx_cx_mix", 0.5f);
            h.ir->setImpulseResponse(
                makeIr(IrLayout::Stereo, {decayingNoise(20000, 11, 0.3f), decayingNoise(20000, 12, 0.3f)}), "a");
            h.settle(20000);
            const auto sine = [](size_t begin, size_t count)
            {
                Samples s(count);
                for (size_t i = 0; i < count; ++i)
                    s[i] = static_cast<float>(0.5 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(begin + i) / kRate));
                return s;
            };
            size_t t = 0;
            // The sine runs for longer than any kernel before the switch, so that both kernels are in their steady state.
            h.render(sine(t, 36000), sine(t, 36000));
            t += 36000;
            const auto before = h.render(sine(t, 12000), sine(t, 12000));
            t += 12000;
            c.change(h);
            const auto during = h.render(sine(t, 30000), sine(t, 30000));
            t += 30000;
            const auto after = h.render(sine(t, 12000), sine(t, 12000));
            const double steadyBefore = maxStep(before.first, 6000, 12000), steadyAfter = maxStep(after.first, 6000, 12000);
            const double worst = maxStep(during.first, 0, during.first.size());
            const double excess = worst - std::max(steadyBefore, steadyAfter);
            const double excessDb = 20.0 * std::log10(std::max(excess, 1.0e-9));
            logMessage(String(c.name) + ": largest step " + String(worst, 5) + ", steady " +
                       String(std::max(steadyBefore, steadyAfter), 5) + ", excess " + String(excessDb, 1) + " dB");
            expect(excess < 0.001, String(c.name) + ": step below -60 dB");
        }
    }

    // The same with the builder thread running asynchronously and the new kernel primed over several blocks.
    void kernelSwitchRealTime()
    {
        beginTest("kernel switch while rendering in real-time pacing (asynchronous build, primed over several blocks)");
        for (const int which : {0, 1})
        {
            Harness h(Quality::Standard);
            h.set("fx_cx_mix", 0.5f);
            h.ir->setImpulseResponse(
                makeIr(IrLayout::Stereo, {decayingNoise(20000, 11, 0.3f), decayingNoise(20000, 12, 0.3f)}), "a");
            size_t t = 0;
            Samples collected;
            const auto block = [&](bool keep)
            {
                Samples l(480), r(480);
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = r[i] =
                        static_cast<float>(0.5 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(t + i) / kRate));
                t += 480;
                h.run(l, r, 480);
                if (keep)
                    collected.insert(collected.end(), l.begin(), l.end());
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            };
            for (int b = 0; b < 200; ++b)
                block(false);
            for (int b = 0; b < 40; ++b)
                block(true);
            const size_t changeAt = collected.size();
            if (which == 0)
                h.ir->setImpulseResponse(
                    makeIr(IrLayout::Stereo, {decayingNoise(20000, 21, 0.4f), decayingNoise(20000, 22, 0.4f)}), "b");
            else
                h.set("fx_cx_reverse", 1.0f);
            for (int b = 0; b < 300; ++b)
                block(true);
            for (int b = 0; b < 40; ++b)
                block(true);
            const double before = maxStep(collected, 5000, changeAt);
            const double after = maxStep(collected, collected.size() - 15000, collected.size());
            const double worst = maxStep(collected, changeAt, collected.size());
            const double excess = worst - std::max(before, after);
            logMessage(String(which == 0 ? "new response" : "reverse") + " (real-time pacing): largest step " +
                       String(worst, 5) + ", steady " + String(std::max(before, after), 5) + ", excess " +
                       String(20.0 * std::log10(std::max(excess, 1.0e-9)), 1) + " dB");
            expect(excess < 0.001, "step below -60 dB");
        }
    }

    void mixAndPassthrough()
    {
        beginTest("mix: dry path is delayed to match; unity gain; no response means pass-through; clearing returns to dry");
        for (const int mode : {0, 1})
        {
            Harness h;
            h.set("fx_cx_mode", static_cast<float>(mode));
            NoiseSource rng(3);
            Samples in(6000);
            for (auto& v : in)
                v = 0.4f * rng.uniform();
            // No response: the input passes unchanged and undelayed.
            auto out = h.render(in, in);
            double worst = 0.0;
            for (size_t i = 0; i < in.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(out.first[i] - in[i])));
            expectEquals(worst, 0.0, "no response: unchanged");
            h.ir->setImpulseResponse(makeIr(IrLayout::Mono, {taps({{0, 0.5f}}, 64)}), "half");
            h.settle();
            const int lat = h.latency->latencySamples();
            for (const float m : {0.0f, 0.5f, 1.0f})
            {
                h.set("fx_cx_mix", m);
                h.settle(4000);
                out = h.render(in, in);
                double err = 0.0;
                for (size_t n = static_cast<size_t>(lat); n < in.size(); ++n)
                {
                    const double x = in[n - static_cast<size_t>(lat)];
                    err = std::max(err, std::abs(out.first[n] - (x * (1.0 - m) + 0.5 * x * m)));
                }
                logMessage(String("mode ") + String(mode) + " mix " + String(m, 1) + ": largest error " + String(err, 8));
                expect(err < 2.0e-6, String("mix ") + String(m, 1));
            }
            h.ir->clearImpulseResponse();
            h.settle();
            out = h.render(in, in);
            worst = 0.0;
            for (size_t i = 0; i < in.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(out.first[i] - in[i])));
            expectEquals(worst, 0.0, "cleared: unchanged");
        }

        beginTest("output gain and bypass");
        {
            Harness h;
            h.ir->setImpulseResponse(makeIr(IrLayout::Mono, {taps({{0, 1.0f}}, 64)}), "unity");
            h.settle();
            h.set("fx_cx_out", 6.0f);
            h.settle(2000);
            Samples in(1000, 0.25f);
            const auto out = h.render(in, in);
            expectWithinAbsoluteError(out.first[900], 0.25 * std::pow(10.0, 6.0 / 20.0), 1.0e-4, "+6 dB");
            h.set("fx_cx_on", 0.0f);
            h.settle(20000);
            const auto off = h.render(in, in);
            expectWithinAbsoluteError(off.first[900], 0.25, 1.0e-6, "switched off");
        }
    }

    void responseView()
    {
        beginTest("response view shows the magnitude of the kernel");
        Harness h;
        auto* view = h.rack.findCapability<ResponseView>("fx.convolution");
        expect(view != nullptr, "capability");
        if (view == nullptr)
            return;
        h.ir->setImpulseResponse(makeIr(IrLayout::Mono, {taps({{0, 1.0f}}, 16384)}), "delta");
        h.set("fx_cx_highcut", 2000.0f);
        h.settle();
        Samples buffer(static_cast<size_t>(view->responsePoints()));
        view->fillResponse(buffer.data());
        expectWithinAbsoluteError(buffer.front(), 0.0, 0.2, "20 Hz");
        expect(buffer.back() < -40.0, "20 kHz is cut");
        logMessage(String("response: 20 Hz ") + String(buffer.front(), 2) + " dB, 20 kHz " + String(buffer.back(), 1) +
                   " dB");
    }

    void allocations()
    {
        beginTest("no allocation on the audio thread while kernels are built, switched and edited");
        for (const int mode : {0, 1})
        {
            Harness h(Quality::Standard);
            h.set("fx_cx_mode", static_cast<float>(mode));
            h.ir->setImpulseResponse(
                makeIr(IrLayout::TrueStereo, {decayingNoise(40000, 1, 0.2f), decayingNoise(40000, 2, 0.2f),
                                              decayingNoise(40000, 3, 0.2f), decayingNoise(40000, 4, 0.2f)}),
                "quad");
            NoiseSource rng(9);
            Samples l(480), r(480);
            long allocations = 0;
            int swaps = 0;
            for (int block = 0; block < 600; ++block)
            {
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = r[i] = 0.3f * rng.uniform();
                if (block % 60 == 20)
                {
                    h.set("fx_cx_highcut", block % 120 == 20 ? 3000.0f : 15000.0f);
                    h.set("fx_cx_predelay", static_cast<float>(block % 7));
                }
                if (block % 150 == 100)
                    h.ir->setImpulseResponse(makeIr(IrLayout::Stereo, {decayingNoise(20000, 30 + block, 0.2f),
                                                                       decayingNoise(20000, 50 + block, 0.2f)}),
                                             "swap");
                crucible::testing::gAllocations = 0;
                crucible::testing::gTrackAllocations = true;
                h.run(l, r, 480);
                crucible::testing::gTrackAllocations = false;
                allocations += crucible::testing::gAllocations;
                if (h.latency->tailSeconds() > 0.0)
                    ++swaps;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            logMessage(String("mode ") + String(mode) + ": allocations " + String(static_cast<int>(allocations)));
            expectEquals(static_cast<int>(allocations), 0, "allocations on the audio thread");
            expect(swaps > 300, "the kernels were adopted while running");
        }
    }
};

static ConvolutionTests convolutionTests;
