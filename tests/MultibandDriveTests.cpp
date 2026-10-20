// SPDX-License-Identifier: MIT
// fx.multiband_drive through the module rack (docs/plans/ENGINE-FX-PLAN.md, "Common acceptance for every new module"):
// unity at 0 dB drive, phase coherence with the crossover, band isolation, solo and bypass, latency, silence, reset,
// block independence, zipper, alias bounds, allocations and CPU. The printed lines are what the CHANGELOG quotes.

#include "AllocCounter.h"
#include "CpuTiming.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Fx.h>
#include <crucible/Saturation.h>
#include <crucible/ModMatrix.h>
#include <crucible/fx/Crossover.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || !defined(NDEBUG)
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#endif
#include <crucible/Simd.h>
#if !CRUCIBLE_SIMD_NATIVE
#define CRUCIBLE_TEST_UNTIMED 1
#endif

using namespace crucible;
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;
using crucible::testing::kRate;

namespace
{
using Samples = std::vector<float>;
constexpr const char* kBands[3] = {"low", "mid", "high"};
constexpr int kNumModels = 10;
enum Model
{
    Soft,
    Hard,
    Rail,
    Diode,
    Fold,
    Triode,
    PushPull,
    Pentode,
    Tape,
    Transformer
};

// fx.multiband_drive alone in the rack (every other effect off), every band at 0 dB drive until a test sets it.
struct Mb
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values, legacy;
    LatencyReporter* latency = nullptr;

    explicit Mb(Quality quality = Quality::Standard, double rate = kRate)
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        ctx.quality = quality;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            values[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 0.0f;
        set("fx_mbdrive_on", 1.0f);
        for (const char* b : kBands)
            set(band(b, "drive"), 0.0f);
        latency = rack.findCapability<LatencyReporter>("fx.multiband_drive");
    }
    static std::string band(const char* b, const char* what) { return std::string("fx_mbdrive_") + b + "_" + what; }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ParameterMap::indexOf(id))] = v; }
    void run(float* l, float* r, int n)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
    }
    Signal render(const Signal& in, int block = 256)
    {
        Samples l(in.size()), r(in.size());
        for (size_t i = 0; i < in.size(); ++i)
            l[i] = r[i] = static_cast<float>(in[i]);
        for (size_t p = 0; p < l.size(); p += static_cast<size_t>(block))
            run(l.data() + p, r.data() + p, static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - p)));
        return toDouble(l);
    }
};

Signal noise(size_t n, double peak, std::uint32_t seed = 7u)
{
    Signal x(n);
    std::uint32_t s = seed;
    for (auto& v : x)
    {
        s = s * 1664525u + 1013904223u;
        v = peak * static_cast<double>(static_cast<std::int32_t>(s)) / 2147483648.0;
    }
    return x;
}

// The crossover's band sum for the module's default frequencies: what the module outputs at 0 dB drive, without delay.
Signal crossoverSum(const Signal& in, double lo = 200.0, double hi = 3000.0)
{
    crucible::fx::Crossover x;
    x.prepare(kRate);
    const double f[2] = {lo, hi};
    x.set(3, f, crucible::fx::CrossoverSlope::Db24);
    Samples l(in.size()), r(in.size());
    for (size_t i = 0; i < in.size(); ++i)
        l[i] = r[i] = static_cast<float>(in[i]);
    std::vector<Samples> bl(3, Samples(in.size())), br(3, Samples(in.size()));
    float* pl[3] = {bl[0].data(), bl[1].data(), bl[2].data()};
    float* pr[3] = {br[0].data(), br[1].data(), br[2].data()};
    x.process(l.data(), r.data(), static_cast<int>(in.size()), pl, pr);
    Signal ref(in.size());
    for (size_t i = 0; i < in.size(); ++i)
        ref[i] = static_cast<double>(bl[0][i]) + bl[1][i] + bl[2][i];
    return ref;
}

double tonePeakDb(const Signal& out, double hz, size_t from)
{
    return amplitudeToDb(toneAmplitude(out.data() + from, out.size() - from, hz, kRate));
}
} // namespace

class MultibandDriveTests : public UnitTest
{
public:
    MultibandDriveTests() : UnitTest("MultibandDrive", "Effects") {}

    void runTest() override
    {
        descriptor();
        unity();
        mixPhase();
        coherence();
        isolation();
        soloBypass();
        latencyReported();
        silenceResetBypass();
        blockSizes();
        modelChange();
        zipper();
        aliasing();
        realtime();
        cpu();
    }

private:
    void descriptor()
    {
        beginTest("descriptor: current, spelled-out ids, three bands of six controls, ten models, latency capability");
        const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.multiband_drive");
        expect(d != nullptr);
        if (d == nullptr)
            return;
        expect(d->status == ModuleStatus::Current);
        int bandParams = 0;
        bool idsOk = true;
        for (const auto& p : d->params)
        {
            idsOk = idsOk && (p.id.rfind("fx_mbdrive_", 0) == 0 || p.id == "fx_pos_mbdrive");
            for (const char* b : kBands)
                if (p.id.rfind(std::string("fx_mbdrive_") + b + "_", 0) == 0)
                    ++bandParams;
            if (p.id == "fx_mbdrive_low_model")
                expect(p.choices.size() == static_cast<size_t>(kNumModels), "ten models");
        }
        expect(idsOk, "every id begins fx_mbdrive_");
        expect(bandParams == 18, "18 band controls, got " + String(bandParams));
        Mb m;
        expect(m.latency != nullptr, "reports latency");
    }

    // Every drive at 0 dB and mix 1: 20 Hz to 20 kHz within 0.1 dB at every quality.
    void unity()
    {
        beginTest("unity: drives at 0 dB, mix 1: flat within 0.1 dB from 20 Hz to 20 kHz");
        for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
        {
            double worst = 0.0;
            for (const double hz : {20.0, 31.0, 50.0, 80.0, 120.0, 200.0, 300.0, 500.0, 800.0, 1000.0, 1500.0, 2200.0,
                                    3000.0, 4500.0, 7000.0, 10000.0, 14000.0, 17000.0, 20000.0})
            {
                Mb m(q);
                const size_t settle = static_cast<size_t>(0.5 * kRate), count = static_cast<size_t>(0.5 * kRate);
                const Signal out = m.render(sineWave(settle + count, hz, kRate, dbToAmplitude(-6.0)));
                const double db =
                    tonePeakDb(out, hz, settle) - tonePeakDb(sineWave(count, hz, kRate, dbToAmplitude(-6.0)), hz, 0);
                worst = std::max(worst, std::abs(db));
            }
            std::printf("  unity, quality %d: worst deviation %.4f dB\n", static_cast<int>(q), worst);
            expect(worst <= 0.1, "quality " + String(static_cast<int>(q)) + ": " + String(worst, 4) + " dB");
        }
    }

    // The dry share of the global mix must pass the same all-pass as the bands: no notches at the crossovers.
    void mixPhase()
    {
        beginTest("mix: drives at 0 dB, global mix 0 / 0.25 / 0.5 / 0.75 / 1: flat within 0.1 dB from 20 Hz to 20 kHz");
        for (const float mixValue : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f})
        {
            double worst = 0.0;
            for (const double hz : {20.0, 50.0, 100.0, 150.0, 200.0, 250.0, 400.0, 800.0, 1500.0, 2200.0, 2800.0, 3000.0,
                                    3300.0, 4500.0, 7000.0, 10000.0, 14000.0, 20000.0})
            {
                Mb m;
                m.set("fx_mbdrive_mix", mixValue);
                const size_t settle = static_cast<size_t>(0.5 * kRate), count = static_cast<size_t>(0.5 * kRate);
                const Signal out = m.render(sineWave(settle + count, hz, kRate, dbToAmplitude(-6.0)));
                const double db =
                    tonePeakDb(out, hz, settle) - tonePeakDb(sineWave(count, hz, kRate, dbToAmplitude(-6.0)), hz, 0);
                worst = std::max(worst, std::abs(db));
            }
            std::printf("  mix %.2f: worst deviation %.4f dB\n", static_cast<double>(mixValue), worst);
            expect(worst <= 0.1, "mix " + String(static_cast<double>(mixValue), 2) + ": " + String(worst, 4) + " dB");
        }
    }

    // The sum at 0 dB drive is the crossover's all-pass sum, delayed by the reported latency.
    void coherence()
    {
        beginTest("coherence: at 0 dB drive the output is the crossover's band sum, delayed by the latency");
        Mb m;
        const Signal in = noise(static_cast<size_t>(1.0 * kRate), 0.3);
        const Signal out = m.render(in);
        const Signal ref = crossoverSum(in);
        const double lat = m.latency != nullptr ? m.latency->latencySamples() : 0;
        const double nullDb = nullTest(ref, out, lat, 4096);
        std::printf("  null against the crossover sum: %.1f dB (latency %.0f)\n", nullDb, lat);
        expect(nullDb < -100.0, String(nullDb, 1) + " dB");
    }

    // A 100 Hz tone driven in the low band only: harmonics appear; driven in the mid and high bands only, they stay far
    // lower.
    void isolation()
    {
        beginTest("isolation: a 100 Hz tone is distorted by the low band's drive and hardly by the other two");
        const auto thd = [](bool low, bool mid, bool high)
        {
            Mb m;
            const char* on[3] = {low ? "x" : nullptr, mid ? "x" : nullptr, high ? "x" : nullptr};
            for (int b = 0; b < 3; ++b)
            {
                m.set(Mb::band(kBands[b], "model"), static_cast<float>(Hard));
                m.set(Mb::band(kBands[b], "drive"), on[b] != nullptr ? 24.0f : 0.0f);
            }
            const size_t settle = static_cast<size_t>(0.4 * kRate), count = 1 << 15;
            const Signal out = m.render(sineWave(settle + count, 100.0, kRate, dbToAmplitude(-12.0)));
            return harmonicProfile(out.data() + settle, count, kRate, 100.0, 12).thdDb;
        };
        const double lowOnly = thd(true, false, false), others = thd(false, true, true), none = thd(false, false, false);
        std::printf("  THD of 100 Hz: low band driven %.1f dB, mid + high driven %.1f dB, none %.1f dB\n", lowOnly, others,
                    none);
        expect(lowOnly > -25.0, "low band distorts: " + String(lowOnly, 1));
        expect(others < lowOnly - 25.0, "other bands barely: " + String(others, 1));
        expect(none < -80.0, "undriven is clean: " + String(none, 1));
    }

    void soloBypass()
    {
        beginTest("solo keeps only the soloed bands; bypass returns a band to its undriven path");
        const size_t settle = static_cast<size_t>(0.4 * kRate), count = static_cast<size_t>(0.4 * kRate);
        Mb s;
        s.set(Mb::band("low", "solo"), 1.0f);
        const double low = tonePeakDb(s.render(sineWave(settle + count, 60.0, kRate, 0.25)), 60.0, settle);
        Mb s2;
        s2.set(Mb::band("low", "solo"), 1.0f);
        const double high = tonePeakDb(s2.render(sineWave(settle + count, 8000.0, kRate, 0.25)), 8000.0, settle);
        std::printf("  solo low: 60 Hz %.1f dB, 8 kHz %.1f dB (input -12.0 dB)\n", low, high);
        expect(low > -12.5 && low < -11.5, "60 Hz passes: " + String(low, 1));
        expect(high < -70.0, "8 kHz removed: " + String(high, 1));
        // Bypass: heavy drive on every band, all bypassed, equals the undriven output exactly.
        Mb a, b;
        for (const char* band : kBands)
        {
            a.set(Mb::band(band, "drive"), 30.0f);
            a.set(Mb::band(band, "bypass"), 1.0f);
        }
        const Signal in = noise(static_cast<size_t>(0.5 * kRate), 0.3);
        const Signal oa = a.render(in), ob = b.render(in);
        double worst = 0.0;
        for (size_t i = 0; i < oa.size(); ++i)
            worst = std::max(worst, std::abs(oa[i] - ob[i]));
        std::printf("  all bands bypassed against undriven: largest difference %.3g\n", worst);
        expect(worst < 1.0e-6, String(worst, 9));
    }

    void latencyReported()
    {
        beginTest("latency: the output is the crossover sum delayed by exactly the reported latency (no other delay fits)");
        for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
        {
            Mb m(q);
            const Signal in = noise(1 << 15, 0.3);
            const Signal out = m.render(in);
            const Signal ref = crossoverSum(in);
            const int reported = m.latency != nullptr ? m.latency->latencySamples() : -1;
            const double at = nullTest(ref, out, reported, 4096), before = nullTest(ref, out, reported - 1, 4096),
                         after = nullTest(ref, out, reported + 1, 4096);
            std::printf(
                "  quality %d: reported %d, null at the latency %.1f dB, one sample less %.1f dB, one more %.1f dB\n",
                static_cast<int>(q), reported, at, before, after);
            expect(at < -100.0 && before > -30.0 && after > -30.0, "quality " + String(static_cast<int>(q)));
            if (q != Quality::Draft)
                expect(reported > 0, "oversampling filters delay the path");
        }
        // The shaped path lines up with the dry one: a driven band with a tiny signal (linear region) keeps the delay.
        Mb m;
        m.set(Mb::band("high", "drive"), 1.0f);
        m.set(Mb::band("high", "model"), static_cast<float>(Soft));
        const Signal in = noise(1 << 15, 0.002);
        const Signal out = m.render(in);
        const double lat = m.latency->latencySamples(), d = measureLatency(crossoverSum(in), out, 0.05, 0.4);
        std::printf("  driven high band, linear region: delay %.2f samples against %.0f reported\n", d - 0.0, lat);
        expect(std::abs(d - lat) <= 0.6, "wet path delay " + String(d, 2));
    }

    void silenceResetBypass()
    {
        beginTest(
            "silence in gives silence out; reset and an off / on cycle leave no stale state; mix 0 is the phase-matched dry (the crossover sum)");
        {
            Mb m;
            for (const char* b : kBands)
            {
                m.set(Mb::band(b, "drive"), 24.0f);
                m.set(Mb::band(b, "model"), static_cast<float>(Tape));
            }
            const Signal out = m.render(Signal(static_cast<size_t>(0.5 * kRate), 0.0));
            double peak = 0.0;
            for (const double v : out)
                peak = std::max(peak, std::abs(v));
            expect(peak < 1.0e-7, "silence out: " + String(peak, 12));
        }
        {
            Mb m;
            for (const char* b : kBands)
            {
                m.set(Mb::band(b, "drive"), 30.0f);
                m.set(Mb::band(b, "model"), static_cast<float>(Tape));
            }
            m.render(noise(static_cast<size_t>(0.5 * kRate), 0.8));
            m.rack.reset();
            const Signal out = m.render(Signal(static_cast<size_t>(0.3 * kRate), 0.0));
            double peak = 0.0;
            for (const double v : out)
                peak = std::max(peak, std::abs(v));
            expect(peak < 1.0e-7, "after reset: " + String(peak, 12));
            // off and on again
            m.render(noise(static_cast<size_t>(0.2 * kRate), 0.8));
            m.set("fx_mbdrive_on", 0.0f);
            m.render(Signal(2048, 0.0));
            m.set("fx_mbdrive_on", 1.0f);
            const Signal again = m.render(Signal(static_cast<size_t>(0.3 * kRate), 0.0));
            peak = 0.0;
            for (const double v : again)
                peak = std::max(peak, std::abs(v));
            expect(peak < 1.0e-7, "after off / on: " + String(peak, 12));
        }
        {
            Mb m;
            for (const char* b : kBands)
                m.set(Mb::band(b, "drive"), 30.0f);
            m.set("fx_mbdrive_mix", 0.0f);
            const Signal in = noise(static_cast<size_t>(0.5 * kRate), 0.3);
            const Signal out = m.render(in);
            const double lat = m.latency != nullptr ? m.latency->latencySamples() : 0;
            const double nullDb = nullTest(crossoverSum(in), out, lat, 2048);
            std::printf("  mix 0 against the delayed crossover sum: %.1f dB\n", nullDb);
            expect(nullDb < -120.0, String(nullDb, 1) + " dB");
        }
    }

    // The output must not depend on how the host cuts its blocks.
    void blockSizes()
    {
        beginTest("block independence: blocks of 1, 7 and 512 render the same output to -105 dB, every model");
        const Signal in = noise(static_cast<size_t>(0.25 * kRate), 0.4);
        for (int model = 0; model < kNumModels; ++model)
        {
            Signal ref;
            for (const int block : {512, 7, 1})
            {
                Mb m;
                for (int b = 0; b < 3; ++b)
                {
                    m.set(Mb::band(kBands[b], "model"), static_cast<float>(model));
                    m.set(Mb::band(kBands[b], "drive"), 18.0f);
                    m.set(Mb::band(kBands[b], "mix"), 0.7f);
                }
                const Signal out = m.render(in, block);
                if (ref.empty())
                    ref = out;
                else
                {
                    double worst = 0.0;
                    for (size_t i = 0; i < out.size(); ++i)
                        worst = std::max(worst, std::abs(out[i] - ref[i]));
                    std::printf("  model %d block %d: largest difference to block 512 %.3g\n", model, block, worst);
                    expect(worst < 1.0e-5,
                           "model " + String(model) + ", block " + String(block) + " differs by " + String(worst, 9));
                }
            }
        }
    }

    // A model change dips the band's wet share and switches at a share below -80 dB: no step, and the new model settles.
    void modelChange()
    {
        beginTest("model change: no step in the output, and the new model is what a fresh instance renders afterwards");
        const size_t before = static_cast<size_t>(0.4 * kRate), after = static_cast<size_t>(0.6 * kRate);
        const Signal in = sineWave(before + after, 1000.0, kRate, dbToAmplitude(-12.0));
        for (const int target : {int(Hard), int(Tape), int(Fold)})
        {
            Mb m;
            m.set(Mb::band("mid", "model"), static_cast<float>(Soft));
            m.set(Mb::band("mid", "drive"), 12.0f);
            Samples l(in.size()), r(in.size());
            for (size_t i = 0; i < in.size(); ++i)
                l[i] = r[i] = static_cast<float>(in[i]);
            m.run(l.data(), r.data(), static_cast<int>(before));
            m.set(Mb::band("mid", "model"), static_cast<float>(target));
            m.run(l.data() + before, r.data() + before, static_cast<int>(after));
            const Signal out = toDouble(l);
            double steady = 0.0, change = 0.0;
            for (size_t i = 1; i < out.size(); ++i)
            {
                const double d = std::abs(out[i] - out[i - 1]);
                (i < before - 4800 || i > before + after - 4800 ? steady : change) =
                    std::max(i < before - 4800 || i > before + after - 4800 ? steady : change, d);
            }
            Mb fresh;
            fresh.set(Mb::band("mid", "model"), static_cast<float>(target));
            fresh.set(Mb::band("mid", "drive"), 12.0f);
            const Signal ref = fresh.render(in);
            double worst = 0.0;
            for (size_t i = out.size() - 4800; i < out.size(); ++i)
                worst = std::max(worst, std::abs(out[i] - ref[i]));
            std::printf("  Soft Clip to model %d: largest step %.4f (steady %.4f), 100 ms after the change: %.3g from a "
                        "fresh instance\n",
                        target, change, steady, worst);
            expect(change < 1.5 * steady, "step " + String(change, 4) + " against " + String(steady, 4));
            expect(worst < 1.0e-3, "settled model differs by " + String(worst, 6));
        }
    }

    // Continuous controls: ramp and square-wave automation, hosts blocks 1, 7, 512.
    void zipper()
    {
        beginTest("zipper: every continuous control, 0-to-1 ramp and 2 Hz square, blocks 1, 7 and 512, below -90 dB");
        struct Ctl
        {
            std::string id;
            double lo, hi;
        };
        std::vector<Ctl> ctls = {{"fx_mbdrive_mix", 0.0, 1.0},
                                 {"fx_mbdrive_output", -24.0, 3.0},
                                 {"fx_mbdrive_xlow", 40.0, 1000.0},
                                 {"fx_mbdrive_xhigh", 800.0, 12000.0}};
        for (const char* b : kBands)
            for (const char* w : {"drive", "output", "mix"})
                ctls.push_back({Mb::band(b, w), w == std::string("mix") ? 0.0 : (w == std::string("drive") ? 0.0 : -24.0),
                                w == std::string("mix") ? 1.0 : (w == std::string("drive") ? 36.0 : 6.0)});
        double worst = -300.0;
        for (const Ctl& c : ctls)
            for (const int block : {1, 7, 512})
            {
                const auto render = [&](const Signal& in, const Signal& automation)
                {
                    Mb m;
                    for (const char* b : kBands)
                    {
                        m.set(Mb::band(b, "model"), static_cast<float>(Soft));
                        m.set(Mb::band(b, "drive"), 9.0f);
                    }
                    const auto value = [&](size_t i) { return static_cast<float>(c.lo + (c.hi - c.lo) * automation[i]); };
                    const size_t warm = static_cast<size_t>(0.2 * kRate);
                    Samples l(warm + in.size());
                    for (size_t i = 0; i < l.size(); ++i)
                        l[i] = i < warm
                                   ? static_cast<float>(0.5 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate))
                                   : static_cast<float>(in[i - warm]);
                    Samples r = l;
                    for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
                    {
                        const size_t end = std::min(pos + static_cast<size_t>(block), l.size());
                        m.set(c.id, value(end - 1 >= warm ? end - 1 - warm : 0));
                        m.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
                    }
                    return toDouble(l, warm);
                };
                const ZipperResult z = zipperTest(render, kRate, 1000.0, -6.0);
                // The held floor: the module's nonlinear residue at either end of the range.
                double floorDb = -300.0;
                for (const double a : {0.0, 1.0})
                    floorDb = std::max(floorDb, clickTest(render(sineWave(static_cast<size_t>(0.6 * kRate), 1000.0, kRate,
                                                                          dbToAmplitude(-6.0)),
                                                                 Signal(static_cast<size_t>(0.6 * kRate), a)),
                                                          1000.0, kRate, static_cast<size_t>(0.1 * kRate))
                                                    .stepDb);
                const double limit = std::max(-90.0, floorDb + 3.0);
                worst = std::max(worst, std::max(z.rampDb, z.squareDb) - limit);
                expect(z.rampDb <= limit && z.squareDb <= limit, c.id + " block " + String(block) + ": ramp " +
                                                                     String(z.rampDb, 1) + ", square " +
                                                                     String(z.squareDb, 1) + ", limit " + String(limit, 1));
            }
        std::printf("  zipper: worst margin to the limit %.1f dB over %d cases\n", worst, static_cast<int>(ctls.size()) * 3);
    }

    void aliasing()
    {
        beginTest("alias bounds: 2.5 kHz through the mid band, 5 and 7 kHz through the high band, -6 dBFS, Standard -90 dB, "
                  "High -100 dB");
        // The plan's rows for the saturator, every model (the tape model included, which fx.saturator holds to -85 / -97
        // dB: here the high band runs a stage higher). The high band runs one oversampling stage more than the other two.
        struct Probe
        {
            const char* band;
            double hz;
        };
        // The tape model is the one exception of fx.saturator (maintainer decision: the switch of its relaxation loop
        // sets a floor at low drive, so it is held to -85 dB Standard and -97 dB High; 7 kHz is probed at 12, 18 and 36 dB
        // like there, 5 and 2.5 kHz at every drive.
        for (const Quality q : {Quality::Standard, Quality::High})
        {
            double worst = -300.0, worstTape = -300.0;
            for (const Probe probe : {Probe{"mid", 2500.0}, Probe{"high", 5000.0}, Probe{"high", 7000.0}})
                for (int model = 0; model < kNumModels; ++model)
                    for (const float drive : {6.0f, 12.0f, 18.0f, 24.0f, 36.0f})
                    {
                        if (probe.hz == 7000.0 && drive == 6.0f)
                            continue;
                        const auto render = [&](const Signal& in)
                        {
                            Mb m(q);
                            m.set(Mb::band(probe.band, "model"), static_cast<float>(model));
                            m.set(Mb::band(probe.band, "drive"), drive);
                            return m.render(in);
                        };
                        const double a = aliasAtDrive(render, kRate, probe.hz, {-6.0}, 0.3)[0].aliasDb;
                        (model == Tape ? worstTape : worst) = std::max(model == Tape ? worstTape : worst, a);
                    }
            std::printf("  quality %d: worst alias %.1f dB, tape %.1f dB\n", static_cast<int>(q), worst, worstTape);
            expect(worst <= (q == Quality::Standard ? -90.0 : -100.0), "nine models " + String(worst, 1) + " dB");
            expect(worstTape <= (q == Quality::Standard ? -85.0 : -97.0), "tape " + String(worstTape, 1) + " dB");
        }
    }

    void realtime()
    {
        beginTest("no allocation on the audio thread while models, drives, solo and crossovers change");
        Mb m(Quality::High);
        for (const char* b : kBands)
            m.set(Mb::band(b, "drive"), 12.0f);
        Samples l(256, 0.3f), r(256, 0.3f);
        for (int i = 0; i < 40; ++i)
            m.run(l.data(), r.data(), 256);
        // Parameter ids are built BEFORE tracking starts: std::string names longer than the small-string buffer (15
        // characters in the MSVC STL, 22 in libc++) allocate, so building them inside the tracked loop counted the test's
        // own allocations (13 per block on MSVC), not the engine's.
        std::string ids[3][4];
        for (int b = 0; b < 3; ++b)
        {
            ids[b][0] = Mb::band(kBands[b], "model");
            ids[b][1] = Mb::band(kBands[b], "drive");
            ids[b][2] = Mb::band(kBands[b], "solo");
            ids[b][3] = Mb::band(kBands[b], "bypass");
        }
        const std::string idXLow = "fx_mbdrive_xlow", idXHigh = "fx_mbdrive_xhigh";
        crucible::testing::gAllocations = 0;
        const long prepares = saturation::Saturator::prepareCalls();
        crucible::testing::gTrackAllocations = true;
        for (int i = 0; i < 400; ++i)
        {
            for (int b = 0; b < 3; ++b)
            {
                m.set(ids[b][0], static_cast<float>((i / 5 + b) % kNumModels));
                m.set(ids[b][1], static_cast<float>((i + 7 * b) % 37));
                m.set(ids[b][2], (i / 50) % 4 == b ? 1.0f : 0.0f);
                m.set(ids[b][3], (i / 30 + b) % 5 == 0 ? 1.0f : 0.0f);
            }
            m.set(idXLow, 60.0f + static_cast<float>(i % 40) * 20.0f);
            m.set(idXHigh, 1500.0f + static_cast<float>(i % 50) * 150.0f);
            for (size_t k = 0; k < 256; ++k)
                l[k] = r[k] = 0.4f * std::sin(0.05f * static_cast<float>(k + 256u * static_cast<size_t>(i)));
            m.run(l.data(), r.data(), 256);
        }
        crucible::testing::gTrackAllocations = false;
        std::printf("  allocations over 400 blocks: %ld\n", crucible::testing::gAllocations);
        expect(crucible::testing::gAllocations == 0, String(static_cast<int>(crucible::testing::gAllocations)));
        const long calls = saturation::Saturator::prepareCalls() - prepares;
        std::printf("  Saturator::prepare calls in process: %ld\n", calls);
        expect(calls == 0, "process never calls Saturator::prepare: " + String(static_cast<int>(calls)));
        // Model changes still work: a Hard Clip band at 24 dB distorts a 100 Hz tone.
        Mb h;
        const size_t settle = static_cast<size_t>(0.4 * kRate), count = 1 << 15;
        h.render(sineWave(settle, 100.0, kRate, dbToAmplitude(-12.0)));
        h.set(Mb::band("low", "model"), static_cast<float>(Hard));
        h.set(Mb::band("low", "drive"), 24.0f);
        const Signal out = h.render(sineWave(settle + count, 100.0, kRate, dbToAmplitude(-12.0)));
        const double thd = harmonicProfile(out.data() + settle, count, kRate, 100.0, 12).thdDb;
        expect(thd > -25.0, "model change takes effect: " + String(thd, 1));
    }

    void cpu()
    {
        beginTest("CPU (stereo, 48 kHz, 512-sample blocks, thread CPU time, best of three)");
#ifndef CRUCIBLE_TEST_UNTIMED
        // Budget (stereo, every band on Soft Clip at 12 dB): the minimum-phase crossover costs about 0.33 % of a core
        // (plan: 5-band multiband 1.2 %), and each driven band costs about one fx.saturator (plan 0.5 % Standard, 1.5 %
        // High) with the high band one oversampling stage higher: about 2.0 % at Standard and 3.5 % at High measured on an
        // idle core. Limits: 1.3 % Draft, 2.5 % Standard, 4.5 % High (a quarter of headroom for a loaded machine). A band at
        // 0 dB drive, bypassed or muted is not run: the three at 0 dB cost the crossover and glue alone, 0.48 %.
        crucible::testing::ScopedUserInteractiveQos qos;
        const double limit[3] = {1.3, 2.5, 4.5};
        for (const float drive : {0.0f, 12.0f})
            for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
            {
                Mb m(q);
                for (const char* b : kBands)
                    m.set(Mb::band(b, "drive"), drive);
                Samples l(512), r(512);
                double best = 1.0e9;
                for (int attempt = 0; attempt < 3; ++attempt)
                {
                    const int blocks = static_cast<int>(2.0 * kRate / 512.0);
                    long phase = 0;
                    double used = 0.0;
                    for (int b = 0; b < blocks; ++b)
                    {
                        for (size_t k = 0; k < 512; ++k)
                            l[k] = r[k] = static_cast<float>(
                                0.5 * std::sin(2.0 * 3.14159265358979 * 220.0 * static_cast<double>(phase++) / kRate));
                        const double c0 = crucible::testing::threadCpuSeconds();
                        m.run(l.data(), r.data(), 512);
                        used += crucible::testing::threadCpuSeconds() - c0;
                    }
                    best = std::min(best, 100.0 * used / (static_cast<double>(blocks) * 512.0 / kRate));
                }
                std::printf("  drive %2.0f dB quality %d: %.3f %% of a core\n", drive, static_cast<int>(q), best);
                if (drive > 0.0f)
                    expect(best <= limit[static_cast<int>(q)], String(best, 3));
            }
#else
        logMessage("CPU timing skipped (debug, sanitizer or non-native SIMD build)");
#endif
    }
};

static MultibandDriveTests multibandDriveTests;
