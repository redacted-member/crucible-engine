// SPDX-License-Identifier: MIT
// Maximiser, clipper and meter module (FX4, docs/plans/ENGINE-FX-PLAN.md sections 3.2 and 3.8): the output true peak against
// a 16x reference on an inter-sample-peak torture set, bass distortion, release behaviour, stereo handling, the soft-clip
// stage, dither, aliasing of the clipper, latency reported against measured, the views, allocations and CPU. The printed
// lines are what docs/MEASUREMENTS.md section 8.5 quotes.

#include "AllocCounter.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/fx/ClipperEffect.h>
#include <crucible/fx/Loudness.h>
#include <crucible/fx/Maximizer.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/MeterReadout.h>
#include <crucible/module/ModuleRack.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || !defined(NDEBUG) ||                                    \
    (defined(__x86_64__) && defined(__APPLE__))
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#include <crucible/Simd.h>
#if !CRUCIBLE_SIMD_NATIVE
#define CRUCIBLE_TEST_UNTIMED 1
#endif

using namespace crucible;
using namespace crucible::fx;
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;
using crucible::testing::kRate;

namespace
{
struct Stereo
{
    Signal l, r;
};

struct Result
{
    Stereo out;
    int latency = 0;
    float maxReduction = 0.0f;
};

Result runMaximizer(const Stereo& in, const MaximizerSettings& s, int block = 64, double rate = kRate)
{
    Maximizer m;
    m.prepare(rate, 512, Quality::Standard, OversamplingFilter::HalfBand);
    std::vector<float> a(in.l.begin(), in.l.end()), b(in.r.begin(), in.r.end());
    Result r;
    for (size_t pos = 0; pos < a.size(); pos += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), a.size() - pos));
        m.process(a.data() + pos, b.data() + pos, n, s);
        r.maxReduction = std::max(r.maxReduction, m.gainReductionDb());
    }
    r.latency = m.latencySamples();
    r.out = {Signal(a.begin(), a.end()), Signal(b.begin(), b.end())};
    return r;
}

double sampleMax(const Signal& x)
{
    double m = 0.0;
    for (const double v : x)
        m = std::max(m, std::abs(v));
    return m;
}

// Largest true peak of a stereo pair against the 16x windowed-sinc reference (dB).
double truePeakDb(const Stereo& s)
{
    return amplitudeToDb(std::max(truePeak(s.l.data(), s.l.size(), 16, 32), truePeak(s.r.data(), s.r.size(), 16, 32)));
}

Stereo mono(const Signal& x)
{
    return {x, x};
}

Signal fadeEnds(Signal x, size_t fade = 480)
{
    for (size_t i = 0; i < fade && i < x.size() / 2; ++i)
    {
        const double g = 0.5 - 0.5 * std::cos(kPi * static_cast<double>(i) / static_cast<double>(fade));
        x[i] *= g;
        x[x.size() - 1 - i] *= g;
    }
    return x;
}

struct Item
{
    std::string name;
    Stereo signal;
};

// The inter-sample-peak torture set: tones and pulses that read much lower as samples than as waveforms, loud clipped and
// band-limited material, a sweep and a bass-heavy mix. All of it exceeds a -1 dBTP ceiling by 4 to 12 dB.
std::vector<Item> tortureSet(double rate)
{
    std::vector<Item> items;
    const size_t n = static_cast<size_t>(1.2 * rate);
    for (const double phaseDeg : {0.0, 22.5, 45.0})
        items.push_back({"fs/4 sine at " + std::to_string(static_cast<int>(phaseDeg)) + " deg, 0 dBFS peak",
                         mono(sineWave(n, rate / 4.0, rate, 1.0, phaseDeg * kPi / 180.0))});
    {
        Signal x(n);
        for (size_t i = 0; i < n; ++i)
            x[i] = (i / 2) % 2 == 0 ? 1.0 : -1.0;
        items.push_back({"++-- square: 0 dBFS samples, +3 dBTP waveform", mono(x)});
    }
    {
        Random rng(11);
        Signal l(n), r(n);
        double lp = 0.0, lq = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            lp += 0.35 * (rng.gaussian() - lp);
            lq += 0.35 * (rng.gaussian() - lq);
            l[i] = std::clamp(4.0 * lp, -1.0, 1.0); // a hard-clipped master
            r[i] = std::clamp(4.0 * lq, -1.0, 1.0);
        }
        items.push_back({"hard-clipped loud noise (sample peaks at 0 dBFS)", {l, r}});
    }
    items.push_back({"1 kHz sine at +6 dBFS", mono(fadeEnds(sineWave(n, 1000.0, rate, 2.0)))});
    items.push_back({"11.2 kHz sine at +3 dBFS", mono(fadeEnds(sineWave(n, 11200.0, rate, 1.41)))});
    items.push_back({"19 kHz sine at +3 dBFS", mono(fadeEnds(sineWave(n, 19000.0, rate, 1.41)))});
    {
        // Band-limited pulses placed half a sample off the grid: samples read about 64 % of the waveform's peak.
        Signal x(n, 0.0);
        const double period = 0.05 * rate;
        for (double centre = 0.3 * rate + 0.5; centre < static_cast<double>(n) - 100.0; centre += period)
            for (int k = -40; k <= 40; ++k)
            {
                const size_t idx = static_cast<size_t>(static_cast<long>(centre) + k);
                const double t = static_cast<double>(idx) - centre;
                const double w = 0.5 + 0.5 * std::cos(kPi * t / 41.0);
                x[idx] += 2.2 * w * (t == 0.0 ? 1.0 : std::sin(kPi * 0.8 * t) / (kPi * 0.8 * t)) * 0.8;
            }
        items.push_back({"band-limited pulse train, half a sample off the grid", mono(x)});
    }
    {
        Signal x(n);
        double phase = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            const double hz = 100.0 * std::pow(190.0, static_cast<double>(i) / static_cast<double>(n));
            phase += 2.0 * kPi * hz / rate;
            x[i] = 1.6 * std::sin(phase);
        }
        items.push_back({"sweep 100 Hz to 19 kHz at +4 dBFS", mono(fadeEnds(x))});
    }
    {
        Random rng(31);
        Signal l(n), r(n);
        for (size_t i = 0; i < n; ++i)
        {
            const double t = static_cast<double>(i) / rate;
            const double bass = 0.9 * std::sin(2.0 * kPi * 45.0 * t) + 0.5 * std::sin(2.0 * kPi * 90.0 * t + 1.0);
            l[i] = bass + 0.35 * std::sin(2.0 * kPi * 7300.0 * t) + 0.1 * rng.gaussian();
            r[i] = bass + 0.35 * std::sin(2.0 * kPi * 8100.0 * t) + 0.1 * rng.gaussian();
        }
        items.push_back({"bass 45 / 90 Hz with 7 and 8 kHz tones and noise (+3 dBFS)", {fadeEnds(l), fadeEnds(r)}});
    }
    return items;
}

// A rack with every other module off and the one under test on.
struct RackHarness
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values;
    std::vector<float> legacy;
    explicit RackHarness(double rate = kRate)
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
    }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ParameterMap::indexOf(id))] = v; }
    void run(float* l, float* r, int n)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
    }
};

[[maybe_unused]] double cpuPercent(const std::vector<std::pair<std::string, float>>& setup)
{
    const size_t n = static_cast<size_t>(8.0 * kRate);
    Random rng(5);
    std::vector<float> src(n);
    for (auto& v : src)
        v = static_cast<float>(0.5 * rng.gaussian());
    const auto run = [&](bool enabled)
    {
        RackHarness h;
        if (enabled)
            for (const auto& kv : setup)
                h.set(kv.first, kv.second);
        std::vector<float> l = src, r = src;
        for (size_t pos = 0; pos < 48000; pos += 32)
            h.run(l.data() + pos, r.data() + pos, 32);
        const auto t0 = std::chrono::steady_clock::now();
        for (size_t pos = 48000; pos + 32 <= n; pos += 32)
            h.run(l.data() + pos, r.data() + pos, 32);
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };
    double best = 1.0e9;
    for (int i = 0; i < 3; ++i)
        best = std::min(best, run(true) - run(false));
    return 100.0 * best / (static_cast<double>(n - 48000) / kRate);
}
} // namespace

class MaximizerTests : public UnitTest
{
public:
    MaximizerTests() : UnitTest("Maximizer", "Effects") {}

    void runTest() override
    {
        ceilingOnTortureSet();
        bassDistortion();
        releaseBehaviour();
        stereoHandling();
        clipStage();
        dither();
        clipperAliasing();
        clipperBehaviour();
        latency();
        moduleAndViews();
        realTime();
    }

private:
    static MaximizerSettings base(double ceilingDb = -1.0, double gainDb = 0.0)
    {
        MaximizerSettings s;
        s.limiter.ceilingDb = static_cast<float>(ceilingDb);
        s.limiter.gainDb = static_cast<float>(gainDb);
        return s;
    }

    void ceilingOnTortureSet()
    {
        beginTest("output true peak never exceeds the ceiling on the torture set: 4x true-peak sidechain and 4x oversampled "
                  "audio");
        const auto items = tortureSet(kRate);
        // The hard-clipped wide-band noise has no well-defined true peak at 0.1 dB (the reference itself moves with its
        // length); it is reported and bounded separately.
        const std::string pathological = "hard-clipped loud noise (sample peaks at 0 dBFS)";
        for (const bool oversampled : {false, true})
        {
            double worst = -100.0, worstPathological = -100.0, worstSample = -100.0;
            for (const double ceiling : {-1.0, -0.3, -6.0})
                for (const auto& item : items)
                {
                    MaximizerSettings s = base(ceiling);
                    s.oversample = oversampled;
                    const Result r = runMaximizer(item.signal, s);
                    const double tp = truePeakDb(r.out), over = tp - ceiling;
                    const double sp = amplitudeToDb(std::max(sampleMax(r.out.l), sampleMax(r.out.r))) - ceiling;
                    const bool hard = item.name == pathological;
                    (hard ? worstPathological : worst) = std::max(hard ? worstPathological : worst, over);
                    worstSample = std::max(worstSample, sp);
                    if (ceiling == -1.0)
                        logMessage(std::string(oversampled ? "4x audio" : "sidechain") + " | " + item.name + ": true peak " +
                                   String(tp, 3) + " dBTP (ceiling -1), reduction " + String(r.maxReduction, 2) + " dB");
                    expect(over <= (hard ? (oversampled ? 0.4 : 0.15) : (oversampled ? 0.02 : 0.0)),
                           item.name + " true peak over the ceiling by " + String(over, 3) + " dB at ceiling " +
                               String(ceiling, 1));
                    expect(sp <= 1.0e-6, item.name + " sample peak over the ceiling by " + String(sp, 6) + " dB");
                }
            logMessage(std::string(oversampled ? "4x oversampled audio path" : "true-peak sidechain") +
                       ": worst output true peak minus ceiling " + String(worst, 4) + " dB over the " +
                       String(static_cast<int>(items.size()) - 1) + " other signals, " + String(worstPathological, 4) +
                       " dB on the hard-clipped wide-band noise; worst sample peak minus ceiling " + String(worstSample, 6) +
                       " dB (3 ceilings)");
        }

        beginTest(
            "a signal under the ceiling passes unchanged apart from the delay (no gain reduction, null below -120 dB)");
        {
            Random rng(4);
            Signal x(static_cast<size_t>(1.0 * kRate));
            for (double& v : x)
                v = 0.1 * rng.gaussian();
            const Result r = runMaximizer(mono(x), base(-1.0));
            const double residual = nullTest(x, r.out.l, static_cast<double>(r.latency), 4800);
            logMessage("below the ceiling: residual " + String(residual, 1) + " dB, gain reduction " +
                       String(r.maxReduction, 6) + " dB");
            expect(residual < -120.0, "null");
            expect(r.maxReduction < 1.0e-3f, "reduction");
        }
    }

    void bassDistortion()
    {
        beginTest("THD of a 60 Hz sine with 6 dB of gain reduction (plan: at most -60 dB in the transparent mode)");
        const size_t settle = static_cast<size_t>(1.0 * kRate), count = static_cast<size_t>(1.0 * kRate);
        double worstTransparent = -300.0;
        for (const double hz : {40.0, 60.0, 100.0})
        {
            const Signal x = sineWave(settle + count, hz, kRate, 1.0);
            for (const auto mode : {ReleaseMode::Fixed, ReleaseMode::Dual, ReleaseMode::Adaptive})
            {
                MaximizerSettings s = base(-6.0206);
                s.limiter.releaseMode = mode;
                s.limiter.autoHold = mode == ReleaseMode::Adaptive;
                const Result r = runMaximizer(mono(x), s);
                const HarmonicProfile p = harmonicProfile(r.out.l.data() + settle, count, kRate, hz, 10);
                const char* name = mode == ReleaseMode::Fixed
                                       ? "fixed, no auto hold"
                                       : (mode == ReleaseMode::Dual ? "dual, no auto hold" : "adaptive, auto hold");
                logMessage(String(hz, 0) + " Hz, " + name + ": THD " + String(p.thdDb, 1) + " dB, reduction " +
                           String(r.maxReduction, 2) + " dB");
                if (mode == ReleaseMode::Adaptive && hz >= 50.0)
                    worstTransparent = std::max(worstTransparent, p.thdDb);
            }
        }
        // The legacy limiter at the same reduction, for the record.
        {
            FxLimiter legacy;
            legacy.prepare(kRate);
            const Signal x = sineWave(settle + count, 60.0, kRate, 1.0);
            std::vector<float> l(x.begin(), x.end()), r = l;
            FxLimiterParams lp;
            lp.on = true;
            lp.ceilingDb = -6.0206f;
            lp.releaseMs = 100.0f;
            lp.lookaheadMs = 5.0f;
            for (size_t pos = 0; pos < l.size(); pos += 64)
                legacy.process(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(64, l.size() - pos)), lp);
            const Signal out(l.begin(), l.end());
            const HarmonicProfile p = harmonicProfile(out.data() + settle, count, kRate, 60.0, 10);
            logMessage("legacy fx.limiter, 60 Hz at the same reduction: THD " + String(p.thdDb, 1) + " dB");
        }
        expect(worstTransparent <= -60.0, "transparent mode THD " + String(worstTransparent, 1) + " dB");
    }

    void releaseBehaviour()
    {
        beginTest(
            "release: fixed time within 20 %, dual stage recovers fast after a click and slowly after sustained reduction");
        // A burst of 0 dBFS 1 kHz against a -10 dB ceiling, then a quiet tone: time for the gain to climb back through 63 %
        // of the reduction.
        const auto recovery = [](ReleaseMode mode, double burstSeconds, double releaseMs)
        {
            const size_t pre = static_cast<size_t>(0.2 * kRate), burst = static_cast<size_t>(burstSeconds * kRate),
                         tail = static_cast<size_t>(6.0 * kRate);
            Signal x;
            for (size_t i = 0; i < pre + burst + tail; ++i)
                x.push_back((i >= pre && i < pre + burst ? 1.0 : 0.01) *
                            std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
            MaximizerSettings s = base(-10.0);
            s.limiter.releaseMode = mode;
            s.limiter.releaseMs = static_cast<float>(releaseMs);
            s.limiter.holdMs = 0.0f;
            s.limiter.autoHold = false;
            s.limiter.lookaheadMs = 2.0f;
            const Result r = runMaximizer(mono(x), s);
            const Signal gain = gainTrace(x, r.out.l, 1000.0, kRate);
            const size_t start = pre + burst + 800;
            const double full = gain[pre + burst - 100];
            for (size_t i = start; i + 48 < gain.size(); ++i)
                if (gain[i] > full * 0.37)
                    return static_cast<double>(i - (pre + burst)) / kRate * 1000.0;
            return -1.0;
        };
        const double fixed = recovery(ReleaseMode::Fixed, 1.0, 200.0);
        logMessage("fixed release 200 ms: 63 % recovery after " + String(fixed, 1) + " ms");
        const double shortDual = recovery(ReleaseMode::Dual, 0.05, 200.0),
                     longDual = recovery(ReleaseMode::Dual, 3.0, 200.0);
        logMessage("dual release 200 ms: 63 % recovery after a 50 ms burst " + String(shortDual, 1) +
                   " ms, after a 3 s burst " + String(longDual, 1) + " ms");
        expectWithinAbsoluteError(fixed, 200.0, 40.0, "fixed release");
        expect(longDual > 2.0 * shortDual, "program dependence");
    }

    void stereoHandling()
    {
        beginTest("stereo: link 100 % reduces both channels alike, link 0 % leaves the quiet side alone, M/S bias keeps the "
                  "ceiling");
        const size_t n = static_cast<size_t>(1.5 * kRate);
        Signal loud = sineWave(n, 997.0, kRate, 1.5), quiet = sineWave(n, 1500.0, kRate, 0.2);
        MaximizerSettings s = base(-3.0);
        s.limiter.link = 1.0f;
        Result linked = runMaximizer({loud, quiet}, s);
        s.limiter.link = 0.0f;
        Result unlinked = runMaximizer({loud, quiet}, s);
        const size_t from = static_cast<size_t>(0.8 * kRate);
        const auto gainOf = [&](const Signal& in, const Signal& out, double hz)
        {
            return amplitudeToDb(toneAmplitude(out.data() + from, n - from, hz, kRate)) -
                   amplitudeToDb(toneAmplitude(in.data() + from, n - from, hz, kRate));
        };
        const double quietLinked = gainOf(quiet, linked.out.r, 1500.0),
                     quietUnlinked = gainOf(quiet, unlinked.out.r, 1500.0);
        const double loudLinked = gainOf(loud, linked.out.l, 997.0);
        {
            // Opening and closing the link while the signal is loud: the second lane takes over the shared gain state.
            Random rng(14);
            Signal l(n), r(n);
            for (size_t i = 0; i < n; ++i)
            {
                l[i] = 1.6 * std::sin(2.0 * kPi * 180.0 * static_cast<double>(i) / kRate) + 0.3 * rng.gaussian();
                r[i] = 1.4 * std::sin(2.0 * kPi * 230.0 * static_cast<double>(i) / kRate) + 0.3 * rng.gaussian();
            }
            Maximizer m;
            m.prepare(kRate, 512, Quality::Standard, OversamplingFilter::HalfBand);
            std::vector<float> a(l.begin(), l.end()), b(r.begin(), r.end());
            for (size_t pos = 0; pos + 64 <= a.size(); pos += 64)
            {
                MaximizerSettings ms = base(-3.0);
                ms.limiter.link = (pos / 4800) % 2 == 0 ? 1.0f : 0.0f;
                m.process(a.data() + pos, b.data() + pos, 64, ms);
            }
            const Stereo out{Signal(a.begin(), a.end()), Signal(b.begin(), b.end())};
            const double tp = truePeakDb(out);
            logMessage("link switched between 100 % and 0 % every 100 ms on loud material: true peak " + String(tp, 3) +
                       " dBTP (ceiling -3)");
            expect(tp <= -3.0 + 0.12, "ceiling while the link changes");
        }
        logMessage("loud channel reduced by " + String(-loudLinked, 2) + " dB; quiet channel: " + String(quietLinked, 2) +
                   " dB linked, " + String(quietUnlinked, 2) + " dB unlinked");
        expectWithinAbsoluteError(quietLinked, loudLinked, 0.05, "linked reduction equal");
        expectWithinAbsoluteError(quietUnlinked, 0.0, 0.02, "unlinked quiet channel");

        // M/S bias: wide content; the ceiling still holds and the side comes out lower than without the bias.
        Random rng(8);
        Signal l(n), r(n);
        for (size_t i = 0; i < n; ++i)
        {
            const double m = 0.9 * std::sin(2.0 * kPi * 220.0 * static_cast<double>(i) / kRate), side = 0.4 * rng.gaussian();
            l[i] = m + side;
            r[i] = m - side;
        }
        MaximizerSettings ms = base(-2.0);
        ms.limiter.sideBias = true;
        const Result biased = runMaximizer({l, r}, ms);
        const Result flat = runMaximizer({l, r}, base(-2.0));
        const double tp = truePeakDb(biased.out);
        const auto sideRms = [&](const Stereo& o)
        {
            double acc = 0.0;
            for (size_t i = from; i < n; ++i)
                acc += 0.25 * (o.l[i] - o.r[i]) * (o.l[i] - o.r[i]);
            return std::sqrt(acc / static_cast<double>(n - from));
        };
        logMessage("M/S bias: true peak " + String(tp, 3) + " dBTP (ceiling -2), side RMS " +
                   String(amplitudeToDb(sideRms(biased.out)), 2) + " dB against " +
                   String(amplitudeToDb(sideRms(flat.out)), 2) + " dB without");
        expect(tp <= -2.0 + 0.1, "ceiling with M/S bias");
        expect(sideRms(biased.out) < sideRms(flat.out), "side turned down more");
    }

    void clipStage()
    {
        beginTest(
            "soft-clip stage takes a share of the peak reduction: less limiter reduction on peaky material, ceiling kept");
        Random rng(6);
        const size_t n = static_cast<size_t>(2.0 * kRate);
        Signal x(n);
        for (size_t i = 0; i < n; ++i)
        {
            const double t = static_cast<double>(i) / kRate;
            x[i] = 0.35 * std::sin(2.0 * kPi * 110.0 * t) + 0.1 * rng.gaussian() +
                   (std::fmod(t, 0.25) < 0.002 ? 1.6 * std::sin(2.0 * kPi * 3000.0 * t) : 0.0);
        }
        MaximizerSettings on = base(-1.0);
        on.limiter.clipShare = 1.0f;
        on.limiter.clipSoftness = 0.4f;
        const Result a = runMaximizer(mono(x), base(-1.0)), b = runMaximizer(mono(x), on);
        const double tp = truePeakDb(b.out);
        logMessage("peak reduction of the limiter: " + String(a.maxReduction, 2) + " dB without the clip stage, " +
                   String(b.maxReduction, 2) + " dB with it; true peak with it " + String(tp, 3) + " dBTP");
        expect(b.maxReduction < a.maxReduction - 1.0f, "share taken by the clip stage");
        expect(tp <= -1.0 + 0.1, "ceiling");
    }

    void dither()
    {
        beginTest("dither: TPDF noise at 16 bits reads -96 dBFS RMS, noise shaping moves it above the low band");
        const size_t n = static_cast<size_t>(2.0 * kRate);
        const Signal x = sineWave(n, 1000.0, kRate, 0.25);
        const auto noise = [&](DitherMode mode, int bits)
        {
            MaximizerSettings s = base(-1.0);
            s.dither = mode;
            s.ditherBits = bits;
            const Result r = runMaximizer(mono(x), s);
            Signal e(n);
            for (size_t i = 0; i < n; ++i)
                e[i] = r.out.l[i] - x[i < static_cast<size_t>(r.latency) ? 0 : i - static_cast<size_t>(r.latency)];
            return e;
        };
        const Signal tpdf = noise(DitherMode::Tpdf, 16), shaped = noise(DitherMode::Shaped, 16),
                     tpdf24 = noise(DitherMode::Tpdf, 24);
        const auto rmsDbOf = [](const Signal& e, size_t from)
        {
            double acc = 0.0;
            for (size_t i = from; i < e.size(); ++i)
                acc += e[i] * e[i];
            return powerToDb(acc / static_cast<double>(e.size() - from));
        };
        // Power below 3 kHz from the spectrum of the error.
        const auto lowBandDb = [&](const Signal& e)
        {
            const Spectrum sp = computeSpectrum(e.data() + 4096, 1 << 16, kRate, Window::Harris7);
            double acc = 0.0;
            for (size_t k = 0; k < sp.bins(); ++k)
                if (static_cast<double>(k) * sp.binHz < 3000.0)
                    acc += sp.power[k];
            return powerToDb(acc);
        };
        logMessage("16-bit TPDF noise " + String(rmsDbOf(tpdf, 4800), 1) + " dBFS RMS, shaped " +
                   String(rmsDbOf(shaped, 4800), 1) + " dBFS RMS; below 3 kHz: " + String(lowBandDb(tpdf), 1) +
                   " dB TPDF against " + String(lowBandDb(shaped), 1) + " dB shaped; 24-bit TPDF " +
                   String(rmsDbOf(tpdf24, 4800), 1) + " dBFS");
        expectWithinAbsoluteError(rmsDbOf(tpdf, 4800), -96.3, 1.5, "TPDF 16 bit level");
        expect(lowBandDb(shaped) < lowBandDb(tpdf) - 6.0, "noise shaping");
        expectWithinAbsoluteError(rmsDbOf(tpdf24, 4800), -144.3, 1.5, "TPDF 24 bit level");
    }

    // Alias power of the clipper (power outside the harmonics below 20 kHz against the fundamental), 5 kHz, 6 dB over.
    static AliasPoint clipperAlias(int stages, int adaaOrder, double softness, bool guard = false)
    {
        const auto render = [&](const Signal& in)
        {
            ClipperEffect c;
            c.prepare(kRate, 512, stages >= 3 ? Quality::High : Quality::Standard, OversamplingFilter::HalfBand);
            ClipperSettings s;
            s.driveDb = 6.0206f;
            s.ceilingDb = 0.0f;
            s.softness = static_cast<float>(softness);
            s.adaaOrder = adaaOrder;
            s.oversampleStages = stages;
            s.guard = guard;
            std::vector<float> l(in.begin(), in.end()), r = l;
            for (size_t pos = 0; pos < l.size(); pos += 64)
                c.process(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(64, l.size() - pos)), s);
            return Signal(l.begin(), l.end());
        };
        return aliasAtDrive(render, kRate, 5000.0, {0.0}, 0.2, 1 << 15)[0];
    }

    void clipperAliasing()
    {
        beginTest("clipper: alias power of 5 kHz at 6 dB over the ceiling (plan: at most -90 dB Standard = 2x, -100 dB High "
                  "= 4x)");
        struct Row
        {
            const char* name;
            int stages;
            int order;
            double softness;
        };
        const Row rows[] = {{"hard, 1x, no ADAA", 0, 0, 0.0},
                            {"hard, 1x, ADAA 1", 0, 1, 0.0},
                            {"hard, 1x, ADAA 2", 0, 2, 0.0},
                            {"hard, 2x, ADAA 1", 1, 1, 0.0},
                            {"hard, 2x, ADAA 2", 1, 2, 0.0},
                            {"hard, 4x, ADAA 1", 2, 1, 0.0},
                            {"hard, 4x, ADAA 2", 2, 2, 0.0},
                            {"hard, 8x, ADAA 1", 3, 1, 0.0},
                            {"hard, 8x, ADAA 2", 3, 2, 0.0},
                            {"shape 40 % (default), 2x, ADAA 2 (Standard)", 1, 2, 0.4},
                            {"shape 40 % (default), 4x, ADAA 2 (High)", 2, 2, 0.4},
                            {"shape 40 % (default), 8x, ADAA 2", 3, 2, 0.4},
                            {"shape 100 % (tanh), 2x, ADAA 2", 1, 2, 1.0}};
        double hard4 = -300.0, hard8 = -300.0, standardDefault = -300.0, highDefault = -300.0;
        for (const Row& row : rows)
        {
            const AliasPoint a = clipperAlias(row.stages, row.order, row.softness);
            logMessage(String(row.name) + ": alias " + String(a.aliasDb, 1) + " dB, worst spur " + String(a.worstSpurDb, 1) +
                       " dB at " + String(a.worstSpurHz, 0) + " Hz, THD " + String(a.thdDb, 1) + " dB");
            if (row.order == 2 && row.softness == 0.0)
            {
                if (row.stages == 2)
                    hard4 = a.aliasDb;
                if (row.stages == 3)
                    hard8 = a.aliasDb;
            }
            if (row.order == 2 && row.softness == 0.4)
            {
                if (row.stages == 1)
                    standardDefault = a.aliasDb;
                if (row.stages == 2)
                    highDefault = a.aliasDb;
            }
        }
        // Standard is 2x and High 4x (plan section 3.9); the default shape meets the plan's rows there, a hard clip needs
        // one step more.
        expect(standardDefault <= -90.0, "default shape, Standard (2x) alias " + String(standardDefault, 1));
        expect(highDefault <= -100.0, "default shape, High (4x) alias " + String(highDefault, 1));
        expect(hard4 <= -100.0 && hard8 <= -100.0, "hard clip at 4x and 8x " + String(hard4, 1) + " / " + String(hard8, 1));
    }

    void clipperBehaviour()
    {
        beginTest(
            "clipper: output never above the ceiling, curve slopes, shape morph, true-peak guard trims inter-sample overs");
        {
            ClipCurve hard, soft;
            hard.setSoftness(0.0);
            soft.setSoftness(1.0);
            expectWithinAbsoluteError(hard.f(0.5), 0.5, 1.0e-12);
            expectWithinAbsoluteError(hard.f(3.0), 1.0, 1.0e-12);
            expectWithinAbsoluteError(soft.f(0.5), std::tanh(0.5), 1.0e-8);
            expectWithinAbsoluteError(soft.f(-2.0), -std::tanh(2.0), 1.0e-8);
            // F is the antiderivative of f: central difference.
            double worst = 0.0;
            for (const double w : {0.0, 0.3, 0.7, 1.0})
            {
                ClipCurve c;
                c.setSoftness(w);
                for (double x = -4.0; x <= 4.0; x += 0.0371)
                {
                    const double h = 1.0e-5;
                    worst = std::max(worst, std::abs((c.F(x + h) - c.F(x - h)) / (2.0 * h) - c.f(x)));
                }
            }
            logMessage("clip curve: antiderivative against the curve (central difference) " + String(worst, 9));
            expect(worst < 1.0e-6, "antiderivative");
        }
        {
            // Below the ceiling the clipper is transparent: ADAA low-passes the linear part, the equaliser behind it undoes
            // that.
            double worstDroop = 0.0;
            std::string table;
            for (const int order : {1, 2})
                for (int stages = 1; stages <= 3; ++stages)
                {
                    double rowWorst = 0.0;
                    for (const double hz : {1000.0, 5000.0, 10000.0, 15000.0, 18000.0, 20000.0})
                    {
                        ClipperEffect c;
                        c.prepare(kRate, 512, Quality::Standard, OversamplingFilter::HalfBand);
                        ClipperSettings s;
                        s.driveDb = 0.0f;
                        s.ceilingDb = 0.0f;
                        s.softness = 0.4f;
                        s.guard = false;
                        s.adaaOrder = order;
                        s.oversampleStages = stages;
                        const Signal x = sineWave(48000, hz, kRate, 0.1);
                        std::vector<float> l(x.begin(), x.end()), r = l;
                        for (size_t pos = 0; pos < l.size(); pos += 64)
                            c.process(l.data() + pos, r.data() + pos, 64, s);
                        const Signal y(l.begin(), l.end());
                        rowWorst =
                            std::max(rowWorst, std::abs(amplitudeToDb(toneAmplitude(y.data() + 8000, 32000, hz, kRate)) -
                                                        amplitudeToDb(toneAmplitude(x.data() + 8000, 32000, hz, kRate))));
                    }
                    table += " ADAA" + std::to_string(order) + "/" + std::to_string(1 << stages) + "x " +
                             String(rowWorst, 2).str();
                    worstDroop = std::max(worstDroop, rowWorst);
                }
            logMessage("clipper small-signal response (1 to 20 kHz), worst deviation from flat in dB:" + table);
            expect(worstDroop <= 0.25, "response below the knee " + String(worstDroop, 3));
        }
        {
            const auto items = tortureSet(kRate);
            double worstSampleGuard = -100.0, worstSampleNoGuard = -100.0, worstWithGuard = -100.0, worstNoGuard = -100.0,
                   pathological = -100.0;
            for (const auto& item : items)
                for (const bool guard : {false, true})
                {
                    ClipperEffect c;
                    c.prepare(kRate, 512, Quality::Standard, OversamplingFilter::HalfBand);
                    ClipperSettings s;
                    s.driveDb = 3.0f;
                    s.ceilingDb = -3.0f;
                    s.softness = 0.3f;
                    s.guard = guard;
                    std::vector<float> l(item.signal.l.begin(), item.signal.l.end()),
                        r(item.signal.r.begin(), item.signal.r.end());
                    for (size_t pos = 0; pos < l.size(); pos += 64)
                        c.process(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(64, l.size() - pos)), s);
                    const Stereo out{Signal(l.begin(), l.end()), Signal(r.begin(), r.end())};
                    const double tp = truePeakDb(out) + 3.0;
                    const double sp = amplitudeToDb(std::max(sampleMax(out.l), sampleMax(out.r))) + 3.0;
                    (guard ? worstSampleGuard : worstSampleNoGuard) =
                        std::max(guard ? worstSampleGuard : worstSampleNoGuard, sp);
                    if (guard && item.name.rfind("hard-clipped", 0) == 0)
                        pathological = tp;
                    else
                        (guard ? worstWithGuard : worstNoGuard) = std::max(guard ? worstWithGuard : worstNoGuard, tp);
                }
            logMessage("clipper, ceiling -3 dBTP, torture set: without the guard the band-limited result rises above the "
                       "ceiling by up to " +
                       String(worstSampleNoGuard, 2) + " dB (samples) and " + String(worstNoGuard, 2) +
                       " dB (true peak); with the guard samples " + String(worstSampleGuard, 4) + " dB, true peak " +
                       String(worstWithGuard, 3) + " dB, hard-clipped wide-band noise " + String(pathological, 3) + " dB");
            expect(worstSampleGuard <= 1.0e-4, "sample peak with the guard");
            expect(worstWithGuard <= 0.06, "guarded true peak");
            expect(pathological <= 0.3, "guarded true peak of the wide-band noise");
        }
    }

    void latency()
    {
        beginTest("latency: reported against measured (maximiser lookahead 0.5 to 10 ms, with the 4x path; clipper at every "
                  "oversampling)");
        Random rng(7);
        Signal in(static_cast<size_t>(1.5 * kRate));
        for (double& v : in)
            v = 0.05 * rng.gaussian();
        double worst = 0.0;
        for (const bool os : {false, true})
            for (const double ms : {0.5, 1.0, 3.0, 5.0, 10.0})
            {
                MaximizerSettings s = base(0.0);
                s.oversample = os;
                s.limiter.lookaheadMs = static_cast<float>(ms);
                const Result r = runMaximizer(mono(in), s);
                const size_t skip = static_cast<size_t>(0.4 * kRate);
                const Signal a(in.begin() + static_cast<long>(skip), in.end()),
                    b(r.out.l.begin() + static_cast<long>(skip), r.out.l.end());
                const double err = measureLatency(a, b) - static_cast<double>(r.latency);
                worst = std::max(worst, std::abs(err));
                logMessage(std::string(os ? "4x audio" : "sidechain") + ", lookahead " + String(ms, 1) + " ms: reported " +
                           String(r.latency) + " samples, measured minus reported " + String(err, 4));
                if (!os)
                    expectEquals(r.latency, static_cast<int>(std::lround(ms * 0.001 * kRate)) - 1 + 16, "latency formula");
            }
        for (const int stages : {0, 1, 2, 3})
            for (const bool guard : {false, true})
            {
                ClipperEffect c;
                c.prepare(kRate, 512, Quality::Standard, OversamplingFilter::HalfBand);
                ClipperSettings s;
                s.driveDb = 0.0f;
                s.ceilingDb = 0.0f;
                s.oversampleStages = stages;
                s.guard = guard;
                s.softness = 0.0f;
                std::vector<float> l(in.begin(), in.end()), r = l;
                for (size_t pos = 0; pos < l.size(); pos += 64)
                    c.process(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(64, l.size() - pos)), s);
                const size_t skip = static_cast<size_t>(0.4 * kRate);
                const Signal a(in.begin() + static_cast<long>(skip), in.end()),
                    b(l.begin() + static_cast<long>(skip), l.end());
                const double err = measureLatency(a, b) - static_cast<double>(c.latencySamples());
                worst = std::max(worst, std::abs(err));
                logMessage(String("clipper ") + String(1 << stages) + "x, guard " + (guard ? "on" : "off") + ": reported " +
                           String(c.latencySamples()) + " samples, measured minus reported " + String(err, 4));
            }
        {
            // Second-order ADAA at a very high drive: input a thousand times the ceiling creeping upward by one float step
            // (about 6e-5) per sample, default softness. Every sample is far past the knee, so the plateau must read exactly
            // 1; a difference quotient of F2 (about x^2 / 2) cancels catastrophically there.
            AdaaClipper clip;
            clip.setSoftness(0.3);
            double worstPlateau = 0.0;
            float x = 1000.0f;
            for (int i = 0; i < 20000; ++i)
            {
                x = std::nextafter(x, 2000.0f);
                const float y = clip.process(x, 2);
                if (i > 2)
                    worstPlateau = std::max(worstPlateau, static_cast<double>(std::abs(y - 1.0f)));
            }
            logMessage("second-order ADAA, 1000x crest: worst plateau error " + String(worstPlateau, 6));
            expect(worstPlateau < 1.0e-4, "ADAA plateau at high drive");
        }
        logMessage("worst latency error " + String(worst, 4) + " samples");
        expect(worst < 0.6, "latency");

        // The clipper's dry line delays by the reported whole number of samples, so the wet path must land on that sample at
        // every oversampling and ADAA order (a half sample off combs the 50 % mix by about 2 dB at 20 kHz).
        double worstClipper = 0.0;
        for (const int order : {1, 2})
            for (const int stages : {1, 2, 3})
            {
                ClipperEffect c;
                c.prepare(kRate, 512, Quality::Standard, OversamplingFilter::HalfBand);
                ClipperSettings s;
                s.driveDb = 0.0f;
                s.ceilingDb = 0.0f;
                s.softness = 0.0f;
                s.adaaOrder = order;
                s.oversampleStages = stages;
                s.guard = false;
                std::vector<float> l(in.begin(), in.end()), r = l;
                for (size_t pos = 0; pos < l.size(); pos += 64)
                    c.process(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(64, l.size() - pos)), s);
                const size_t skip = static_cast<size_t>(0.4 * kRate);
                const Signal a(in.begin() + static_cast<long>(skip), in.end()),
                    b(l.begin() + static_cast<long>(skip), l.end());
                const double err = measureLatency(a, b) - static_cast<double>(c.latencySamples());
                worstClipper = std::max(worstClipper, std::abs(err));
                logMessage(String("clipper ADAA ") + String(order) + ", " + String(1 << stages) +
                           "x: measured minus reported " + String(err, 4));
            }
        expect(worstClipper < 0.15, "clipper wet path lands on the reported whole sample");
    }

    void moduleAndViews()
    {
        beginTest("modules in the rack: ceiling, latency reporter, gain reduction meter");
        {
            RackHarness h;
            h.set("fx_mx_on", 1.0f);
            h.set("fx_mx_gain", 12.0f);
            h.set("fx_mx_ceiling", -2.0f);
            const Stereo tort = tortureSet(kRate)[9].signal; // the sweep
            std::vector<float> l(tort.l.begin(), tort.l.end()), r(tort.r.begin(), tort.r.end());
            for (size_t pos = 0; pos < l.size(); pos += 32)
                h.run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(32, l.size() - pos)));
            const Stereo out{Signal(l.begin(), l.end()), Signal(r.begin(), r.end())};
            const double tp = truePeakDb(out);
            auto* lat = h.rack.findCapability<LatencyReporter>("fx.maximizer");
            auto* gr = h.rack.findCapability<GainReductionMeter>("fx.maximizer");
            logMessage("module fx.maximizer with 12 dB gain, ceiling -2: true peak " + String(tp, 3) + " dBTP, latency " +
                       String(lat != nullptr ? lat->latencySamples() : -1) + " samples, reduction " +
                       String(gr != nullptr ? gr->gainReductionDb(0) : -1.0f, 2) + " dB");
            expect(tp <= -2.0 + 0.1, "module ceiling");
            expect(lat != nullptr && lat->latencySamples() == static_cast<int>(std::lround(0.003 * kRate)) - 1 + 16,
                   "reported latency");
            expect(gr != nullptr, "gain reduction meter");
        }

        beginTest("util.meter module passes audio through bit for bit and reads the loudness of a -23 LUFS tone");
        {
            RackHarness h;
            h.set("fx_mtr_on", 1.0f);
            KWeightingCoefficients shelf, hp;
            designKWeighting(kRate, shelf, hp);
            const auto mag = [&](const KWeightingCoefficients& c)
            {
                const std::complex<double> z = std::polar(1.0, -2.0 * kPi * 1000.0 / kRate), z2 = z * z;
                return std::abs((c.b0 + c.b1 * z + c.b2 * z2) / (1.0 + c.a1 * z + c.a2 * z2));
            };
            const double a = std::pow(10.0, (-23.0 + 0.691) / 20.0) / (mag(shelf) * mag(hp));
            const Signal tone = sineWave(static_cast<size_t>(10.0 * kRate), 1000.0, kRate, a);
            std::vector<float> l(tone.begin(), tone.end()), r = l;
            const std::vector<float> ref = l;
            for (size_t pos = 0; pos < l.size(); pos += 128)
                h.run(l.data() + pos, r.data() + pos, 128);
            expect(std::memcmp(l.data(), ref.data(), l.size() * sizeof(float)) == 0 &&
                       std::memcmp(r.data(), ref.data(), l.size() * sizeof(float)) == 0,
                   "bit-exact pass through");
            auto* readout = h.rack.findCapability<MeterReadout>("util.meter");
            expect(readout != nullptr, "MeterReadout");
            if (readout != nullptr)
            {
                float v[MeterReadout::kLoudnessValues], lv[MeterReadout::kLevelValues];
                readout->loudness(v);
                const bool hasLevels = readout->levels(lv);
                logMessage("util.meter: momentary " + String(v[0], 2) + ", short-term " + String(v[1], 2) + ", integrated " +
                           String(v[2], 2) + " LUFS, true peak " + String(v[4], 2) + " dB; sample peak " + String(lv[0], 2) +
                           ", RMS " + String(lv[4], 2) + ", PPM " + String(lv[6], 2) + ", VU " + String(lv[8], 2) +
                           " dB, correlation " + String(lv[10], 3));
                expectWithinAbsoluteError(v[0], -23.0f, 0.1f, "momentary");
                expectWithinAbsoluteError(v[1], -23.0f, 0.1f, "short-term");
                expectWithinAbsoluteError(v[2], -23.0f, 0.1f, "integrated");
                expect(hasLevels, "levels");
                expectWithinAbsoluteError(lv[10], 1.0f, 1.0e-3f, "correlation");
                expectWithinAbsoluteError(lv[4], static_cast<float>(amplitudeToDb(a)), 0.2f,
                                          "RMS of a sine reads its amplitude");
                expectWithinAbsoluteError(readout->suggestedGainDb(-14.0f), 9.0f, 0.1f, "gain suggestion to -14 LUFS");
            }
        }

        beginTest("views: <module>/loudness and <module>/levels are published through the engine");
        {
            Engine engine;
            engine.prepare(kRate, 512);
            engine.setParam("fx_mtr_on", 1.0f);
            engine.setParam("fx_mx_on", 1.0f);
            auto& hub = engine.views();
            const int vLoud = hub.find("util.meter", "loudness");
            const int vLevels = hub.find("util.meter", "levels");
            const int vMax = hub.find("fx.maximizer", "loudness");
            const int vGr = hub.find("fx.maximizer", "gain_reduction");
            expect(vLoud >= 0 && vLevels >= 0 && vMax >= 0 && vGr >= 0, "views exist");
            hub.subscribe(vLoud, 0.0f);
            hub.subscribe(vLevels, 0.0f);
            engine.noteOn(57, 1.0f);
            std::vector<float> l(512), r(512);
            for (int i = 0; i < 100; ++i)
                engine.process(l.data(), r.data(), 512);
            float v[MeterReadout::kLoudnessValues] = {};
            const int count = hub.read(vLoud, v, MeterReadout::kLoudnessValues, nullptr);
            float lv[MeterReadout::kLevelValues] = {};
            const int count2 = hub.read(vLevels, lv, MeterReadout::kLevelValues, nullptr);
            logMessage("view util.meter/loudness: " + String(count) + " values, momentary " + String(v[0], 2) +
                       " LUFS; levels: " + String(count2) + " values, sample peak " + String(lv[0], 2) + " dB");
            expectEquals(count, MeterReadout::kLoudnessValues);
            expectEquals(count2, MeterReadout::kLevelValues);
            expect(v[0] > -100.0f, "the engine's note is metered");
        }
    }

    void realTime()
    {
        beginTest("no allocation on the audio thread: all three modules, parameter changes every block");
        {
            RackHarness h;
            for (const char* on : {"fx_mx_on", "fx_clp_on", "fx_mtr_on"})
                h.set(on, 1.0f);
            h.set("fx_mx_gain", 10.0f);
            h.set("fx_clp_drive", 6.0f);
            Random rng(3);
            std::vector<float> l(32), r(32);
            for (int warm = 0; warm < 8; ++warm)
                h.run(l.data(), r.data(), 32);
            crucible::testing::gAllocations = 0;
            crucible::testing::gTrackAllocations = true;
            for (int block = 0; block < 800; ++block)
            {
                for (size_t i = 0; i < 32; ++i)
                {
                    l[i] = static_cast<float>(0.5 * rng.gaussian());
                    r[i] = static_cast<float>(0.5 * rng.gaussian());
                }
                h.set("fx_mx_lookahead", 0.5f + static_cast<float>(block % 20) * 0.5f);
                h.set("fx_mx_release", 10.0f + static_cast<float>(block % 50) * 10.0f);
                h.set("fx_mx_relmode", static_cast<float>(block % 3));
                h.set("fx_mx_os", static_cast<float>((block / 40) % 2));
                h.set("fx_mx_clip", static_cast<float>(block % 7) * 15.0f);
                h.set("fx_mx_stereo", static_cast<float>((block / 3) % 2));
                h.set("fx_mx_dither", static_cast<float>(block % 3));
                h.set("fx_mx_ceiling", -6.0f + static_cast<float>(block % 6));
                h.set("fx_mx_hold", static_cast<float>(block % 30));
                h.set("fx_mx_link", (block / 4) % 2 == 0 ? 100.0f : 30.0f);
                h.set("fx_clp_os", static_cast<float>(block % 5));
                h.set("fx_clp_guard", static_cast<float>((block / 2) % 2));
                h.set("fx_clp_adaa", static_cast<float>((block / 5) % 3));
                h.set("fx_clp_shape", static_cast<float>(block % 101));
                h.set("fx_mtr_ppm", static_cast<float>((block / 7) % 2));
                h.set("fx_mtr_reset", static_cast<float>(block % 50 == 0));
                h.run(l.data(), r.data(), 32);
            }
            crucible::testing::gTrackAllocations = false;
            expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
            bool finite = true;
            for (size_t i = 0; i < 32; ++i)
                finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
            expect(finite, "finite output");
        }

#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest("CPU: percent of one core per stereo instance at 48 kHz (plan: maximiser 0.6 / 1.2 % with 4x audio, "
                  "clipper 0.3, meter 0.2)");
        {
            const double sidechain = cpuPercent({{"fx_mx_on", 1.0f}, {"fx_mx_gain", 10.0f}});
            const double extras =
                cpuPercent({{"fx_mx_on", 1.0f}, {"fx_mx_gain", 10.0f}, {"fx_mx_clip", 50.0f}, {"fx_mx_dither", 2.0f}});
            const double oversampled = cpuPercent({{"fx_mx_on", 1.0f}, {"fx_mx_gain", 10.0f}, {"fx_mx_os", 1.0f}});
            const double clipper = cpuPercent({{"fx_clp_on", 1.0f}, {"fx_clp_drive", 9.0f}, {"fx_clp_guard", 0.0f}});
            const double clipperGuard = cpuPercent({{"fx_clp_on", 1.0f}, {"fx_clp_drive", 9.0f}});
            const double clipperTypical = cpuPercent({{"fx_clp_on", 1.0f}, {"fx_clp_guard", 0.0f}});
            const double clipper4 =
                cpuPercent({{"fx_clp_on", 1.0f}, {"fx_clp_drive", 9.0f}, {"fx_clp_os", 3.0f}, {"fx_clp_guard", 0.0f}});
            const double meter = cpuPercent({{"fx_mtr_on", 1.0f}});
            logMessage("fx.maximizer " + String(sidechain, 3) + " % (with clip stage and shaped dither " +
                       String(extras, 3) + " %), with the 4x audio path " + String(oversampled, 3) +
                       " %; fx.clipper at the Standard 2x without the guard " + String(clipper, 3) + " % (" +
                       String(clipperTypical, 3) + " % when the signal stays below the ceiling), with the guard " +
                       String(clipperGuard, 3) + " %, 4x without the guard " + String(clipper4, 3) + " %; util.meter " +
                       String(meter, 3) + " %");
            expectCpu(sidechain <= 0.6 * 1.2, "maximiser CPU");
            expectCpu(extras <= 0.9, "maximiser with clip stage CPU");
            expectCpu(oversampled <= 1.2 * 1.2, "maximiser 4x audio CPU");
            expectCpu(clipper <= 0.5, "clipper CPU while clipping continuously");
            expectCpu(clipperTypical <= 0.3 * 1.2, "clipper CPU");
            expectCpu(clipperGuard <= 1.2, "clipper with guard CPU");
            expectCpu(meter <= 0.2 * 1.2, "meter CPU");
        }
#endif
    }
};

static MaximizerTests maximizerTests;
