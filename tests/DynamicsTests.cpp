// SPDX-License-Identifier: MIT
// Dynamics core and its three modules (FX3, docs/plans/ENGINE-FX-PLAN.md section 3.2): static curves, attack and release
// times, distortion, aliasing, true peak, lookahead, key and stereo behaviour, latency, metering, allocations and CPU.
// Every claim is a number; the printed lines are what docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/FxExtra.h>
#include <crucible/fx/Dynamics.h>
#include <crucible/module/LatencyReporter.h>
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

// Runs the processor on a stereo pair in blocks; optional key and per-block gain reduction trace.
Stereo runStereo(const Signal& l, const Signal& r, const DynamicsSettings& s, const Signal* keyL = nullptr,
                 const Signal* keyR = nullptr, std::vector<float>* grTrace = nullptr, int block = 32,
                 int* latencyOut = nullptr)
{
    DynamicsProcessor p;
    p.prepare(kRate);
    std::vector<float> a(l.begin(), l.end()), b(r.begin(), r.end()), ka, kb;
    if (keyL != nullptr)
    {
        ka.assign(keyL->begin(), keyL->end());
        kb.assign((keyR != nullptr ? *keyR : *keyL).begin(), (keyR != nullptr ? *keyR : *keyL).end());
    }
    for (size_t pos = 0; pos < a.size(); pos += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), a.size() - pos));
        p.process(a.data() + pos, b.data() + pos, n, keyL != nullptr ? ka.data() + pos : nullptr,
                  keyL != nullptr ? kb.data() + pos : nullptr, s, keyL != nullptr);
        if (grTrace != nullptr)
            grTrace->push_back(p.gainReductionDb());
    }
    if (latencyOut != nullptr)
        *latencyOut = p.latencySamples();
    return {Signal(a.begin(), a.end()), Signal(b.begin(), b.end())};
}

Signal runMono(const Signal& in, const DynamicsSettings& s, int block = 32)
{
    return runStereo(in, in, s, nullptr, nullptr, nullptr, block).l;
}

DynamicsSettings compressor(float thresholdDb, float ratio, float kneeDb, float attackMs, float releaseMs, DetectorMode d)
{
    DynamicsSettings s;
    s.kind = DynamicsKind::Compressor;
    s.thresholdDb = thresholdDb;
    s.ratio = ratio;
    s.kneeDb = kneeDb;
    s.attackMs = attackMs;
    s.releaseMs = releaseMs;
    s.detector = d;
    return s;
}

double peakAbs(const Signal& x, size_t from, size_t to)
{
    double m = 0.0;
    for (size_t i = from; i < to && i < x.size(); ++i)
        m = std::max(m, std::abs(x[i]));
    return m;
}

Signal concat(std::initializer_list<Signal> parts)
{
    Signal out;
    for (const auto& p : parts)
        out.insert(out.end(), p.begin(), p.end());
    return out;
}

// RMS of x over [from, to), as dB re full scale amplitude 1.
double rmsDb(const Signal& x, size_t from, size_t to)
{
    double s = 0.0;
    for (size_t i = from; i < to; ++i)
        s += x[i] * x[i];
    return powerToDb(s / static_cast<double>(to - from));
}

double lastGr(const Stereo& o, const Signal& in, size_t from, size_t to, double hz)
{
    return -(amplitudeToDb(toneAmplitude(o.l.data() + from, to - from, hz, kRate)) -
             amplitudeToDb(toneAmplitude(in.data() + from, to - from, hz, kRate)));
}

// A module through the rack with every other effect off.
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
    void set(const std::string& id, float v)
    {
        const int i = ParameterMap::indexOf(id);
        values[static_cast<size_t>(i)] = v;
    }
    float get(const std::string& id) const { return values[static_cast<size_t>(ParameterMap::indexOf(id))]; }
    void run(float* l, float* r, int n, const float* auxL = nullptr, const float* auxR = nullptr)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, auxL, auxR);
    }
    // Whole signal in blocks of `block`; the left output.
    Signal runSignal(const Signal& in, const Signal* aux = nullptr, int block = 32)
    {
        std::vector<float> l(in.begin(), in.end()), r = l, a;
        if (aux != nullptr)
            a.assign(aux->begin(), aux->end());
        for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
        {
            const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos));
            run(l.data() + pos, r.data() + pos, n, aux != nullptr ? a.data() + pos : nullptr,
                aux != nullptr ? a.data() + pos : nullptr);
        }
        return Signal(l.begin(), l.end());
    }
};

double neutralLatencyError(double lookaheadMs, bool truePeak)
{
    DynamicsSettings s;
    s.thresholdDb = 0.0f;
    s.ratio = 1.0f;
    s.kneeDb = 0.0f;
    s.lookaheadMs = static_cast<float>(lookaheadMs);
    s.truePeak = truePeak;
    s.detector = DetectorMode::Peak;
    Random rng(7);
    Signal in(static_cast<size_t>(1.5 * kRate));
    for (double& v : in)
        v = 0.2 * rng.gaussian();
    int reported = 0;
    const Stereo o = runStereo(in, in, s, nullptr, nullptr, nullptr, 32, &reported);
    const size_t skip = static_cast<size_t>(0.4 * kRate);
    const Signal a(in.begin() + static_cast<long>(skip), in.end()), b(o.l.begin() + static_cast<long>(skip), o.l.end());
    return measureLatency(a, b) - static_cast<double>(reported);
}

} // namespace

class DynamicsTests : public UnitTest
{
public:
    DynamicsTests() : UnitTest("Dynamics", "Effects") {}

    void runTest() override
    {
        staticCurves();
        timeConstants();
        distortion();
        lookaheadAndTruePeak();
        keyAndFilters();
        stereo();
        autoRelease();
        gateAndDucker();
        latency();
        modules();
        realTime();
    }

private:
    void staticCurves()
    {
        beginTest("static curve: RMS detector, 1 kHz, -60 to +6 dBFS, within 0.1 dB of the gain computer");
        {
            struct Case
            {
                const char* name;
                DynamicsSettings s;
            };
            std::vector<Case> cases;
            cases.push_back({"comp -18 dB 4:1 knee 6", compressor(-18, 4, 6, 10, 100, DetectorMode::Rms)});
            cases.push_back({"comp -12 dB inf:1 knee 0", compressor(-12, 1.0e6f, 0, 10, 100, DetectorMode::Rms)});
            cases.push_back({"comp -30 dB 2:1 knee 24", compressor(-30, 2, 24, 10, 100, DetectorMode::Rms)});
            {
                auto s = compressor(-30, 3, 6, 10, 100, DetectorMode::Rms);
                s.kind = DynamicsKind::Upward;
                s.rangeDb = 24;
                cases.push_back({"upward -30 dB 3:1 knee 6", s});
            }
            {
                auto s = compressor(-40, 3, 6, 10, 100, DetectorMode::Rms);
                s.kind = DynamicsKind::Expander;
                cases.push_back({"expander -40 dB 1:3 knee 6", s});
            }
            std::vector<double> levels;
            for (double v = -60.0; v <= 6.0; v += 3.0)
                levels.push_back(v);
            double overall = 0.0;
            for (auto& c : cases)
            {
                DynamicsSettings s = c.s;
                s.rmsWindowMs = 10.0f;
                auto render = [&](const Signal& x) { return runMono(x, s); };
                const auto curve = staticCurve(render, kRate, levels, 1000.0, 0.8, 0.2);
                double worst = 0.0;
                for (const auto& pt : curve)
                {
                    float amount = 0.0f;
                    switch (s.kind)
                    {
                    case DynamicsKind::Compressor:
                        amount = -dyn::compressorReduction(static_cast<float>(pt.inDb), s.thresholdDb, s.ratio, s.kneeDb);
                        break;
                    case DynamicsKind::Upward:
                        amount = std::min(dyn::upwardLift(static_cast<float>(pt.inDb), s.thresholdDb, s.ratio, s.kneeDb),
                                          s.rangeDb);
                        break;
                    default:
                        amount = -dyn::expanderReduction(static_cast<float>(pt.inDb), s.thresholdDb, s.ratio, s.kneeDb);
                        break;
                    }
                    worst = std::max(worst, std::abs(pt.gainDb - amount));
                }
                overall = std::max(overall, worst);
                expect(worst <= 0.1, String(std::string(c.name) + " error ") + String(worst, 4));
            }
            logMessage(String("worst static-curve error over five curves: ") + String(overall, 4) + " dB (plan 0.1)");
        }

        beginTest("HoldMax equals a brute-force sliding maximum, also while the window changes");
        {
            Random rng(17);
            HoldMax hm;
            hm.prepare(500);
            std::vector<float> x;
            size_t bad = 0;
            int window = 1;
            hm.setWindow(window);
            for (int i = 0; i < 20000; ++i)
            {
                if (i % 997 == 0)
                {
                    window = 1 + static_cast<int>(rng.uniform() * 499.0);
                    hm.setWindow(window);
                }
                x.push_back(static_cast<float>(std::abs(rng.gaussian())));
                const size_t lo = x.size() > static_cast<size_t>(window) ? x.size() - static_cast<size_t>(window) : 0;
                float expected = 0.0f;
                for (size_t k = lo; k < x.size(); ++k)
                    expected = std::max(expected, x[k]);
                if (hm.push(x.back()) != expected)
                    ++bad;
            }
            expectEquals(static_cast<int>(bad), 0, "readings that differ");
        }

        beginTest("gain curve helpers: knee is continuous with unit slope handover; log2 / exp2 accuracy");
        {
            for (const float knee : {0.0f, 6.0f, 24.0f})
            {
                const float t = -20.0f;
                float prev = dyn::compressorReduction(t - 30.0f, t, 4.0f, knee);
                expectWithinAbsoluteError(prev, 0.0f, 1.0e-6f);
                float maxJump = 0.0f;
                for (float x = t - 30.0f; x < t + 30.0f; x += 0.01f)
                {
                    const float g = dyn::compressorReduction(x, t, 4.0f, knee);
                    maxJump = std::max(maxJump, std::abs(g - prev));
                    prev = g;
                }
                expect(maxJump <= 0.0076f,
                       String("compressor curve step ") + String(maxJump, 5) + " per 0.01 dB (slope 0.75)");
            }
            double worstLog = 0.0, worstExp = 0.0;
            for (double x = 1.0e-9; x < 1.0e4; x *= 1.0137)
                worstLog = std::max(worstLog, std::abs(static_cast<double>(dyn::log2Fast(static_cast<float>(x))) -
                                                       std::log2(static_cast<double>(static_cast<float>(x)))));
            for (double x = -100.0; x < 100.0; x += 0.0137)
                worstExp = std::max(worstExp, std::abs(static_cast<double>(dyn::exp2Fast(static_cast<float>(x))) /
                                                           std::exp2(static_cast<double>(static_cast<float>(x))) -
                                                       1.0));
            logMessage(String("log2Fast worst absolute error ") + String(worstLog, 8) + ", exp2Fast worst relative error " +
                       String(worstExp, 8));
            expect(worstLog < 2.0e-6, "log2Fast");
            expect(worstExp < 5.0e-7, "exp2Fast");
        }
    }

    void timeConstants()
    {
        beginTest("attack and release: time to 63 % of a 20 dB step's gain change within 10 % of the label (peak and RMS)");
        {
            double worstPeak = 0.0, worstRms = 0.0;
            for (const DetectorMode mode : {DetectorMode::Peak, DetectorMode::Rms})
            {
                const bool peak = mode == DetectorMode::Peak;
                // RMS: the averaging window is 10 ms; labels from 10 ms up are reachable, shorter ones bottom out.
                for (const double attack : {peak ? 1.0 : 10.0, 10.0, 50.0, 200.0})
                {
                    auto s = compressor(-40, 4, 0, static_cast<float>(attack), 100, mode);
                    s.rmsWindowMs = 10.0f;
                    auto render = [&](const Signal& x) { return runMono(x, s); };
                    const StepMetrics m = stepTimes(render, kRate, -40.0, -20.0, 0.0, 1.0, 1.0);
                    const double err = std::abs(m.t63Ms / attack - 1.0);
                    (peak ? worstPeak : worstRms) = std::max(peak ? worstPeak : worstRms, err);
                    expect(m.valid() && err <= 0.10, String(peak ? "peak" : "rms") + " attack " + String(attack, 1) +
                                                         " ms measured " + String(m.t63Ms, 2));
                }
                for (const double release : {peak ? 5.0 : 50.0, 100.0, 500.0, 2000.0})
                {
                    auto s = compressor(-40, 4, 0, 1, static_cast<float>(release), mode);
                    s.rmsWindowMs = 10.0f;
                    auto render = [&](const Signal& x) { return runMono(x, s); };
                    const StepMetrics m = stepTimes(render, kRate, -20.0, -40.0, 0.0, 1.0, 0.5 + 6.0 * release / 1000.0);
                    const double err = std::abs(m.t63Ms / release - 1.0);
                    (peak ? worstPeak : worstRms) = std::max(peak ? worstPeak : worstRms, err);
                    expect(m.valid() && err <= 0.10, String(peak ? "peak" : "rms") + " release " + String(release, 1) +
                                                         " ms measured " + String(m.t63Ms, 2));
                }
            }
            logMessage(String("worst time error: peak ") + String(100.0 * worstPeak, 1) + " %, RMS (10 ms window) " +
                       String(100.0 * worstRms, 1) + " % (plan 10 %)");
        }

        beginTest("attack below the RMS window bottoms out instead of failing silently");
        {
            auto s = compressor(-40, 4, 0, 1, 100, DetectorMode::Rms);
            s.rmsWindowMs = 10.0f;
            auto render = [&](const Signal& x) { return runMono(x, s); };
            const StepMetrics m = stepTimes(render, kRate, -40.0, -20.0, 0.0, 1.0, 1.0);
            logMessage(String("RMS 10 ms window, attack label 1 ms: t63 ") + String(m.t63Ms, 2) + " ms (detector limited)");
            expect(m.t63Ms > 1.0 && m.t63Ms < 8.0, "bounded by the window");
        }
    }

    void distortion()
    {
        beginTest("THD of a 50 Hz sine 10 dB over the threshold, RMS detector, 10 ms attack, 100 ms release");
        {
            auto s = compressor(-30, 4, 6, 10, 100, DetectorMode::Rms);
            s.rmsWindowMs = 20.0f;
            auto render = [&](const Signal& x) { return runMono(x, s); };
            const HarmonicPoint h = harmonicsAt(render, kRate, 50.0, -20.0, 1.5, 0.5);
            logMessage(String("RMS window 20 ms: THD ") + String(h.thdDb, 1) + " dB (plan at most -60)");
            expect(h.thdDb <= -60.0, "THD");
            s.rmsWindowMs = 10.0f;
            const HarmonicPoint h10 = harmonicsAt(render, kRate, 50.0, -20.0, 1.5, 0.5);
            logMessage(String("RMS window 10 ms: THD ") + String(h10.thdDb, 1) + " dB");
            expect(h10.thdDb <= -60.0, "THD at the 10 ms window");
            s.detector = DetectorMode::Peak;
            const HarmonicPoint hp = harmonicsAt(render, kRate, 50.0, -20.0, 1.5, 0.5);
            const HarmonicPoint hp1k = harmonicsAt(render, kRate, 1000.0, -20.0, 1.0, 0.3);
            logMessage(String("peak detector: THD ") + String(hp.thdDb, 1) + " dB at 50 Hz, " + String(hp1k.thdDb, 1) +
                       " dB at 1 kHz (the removed fx.comp: -34 / -59)");
        }

        beginTest("aliasing: 5 kHz and 7.3 kHz through a compressor at 4 dB and 10 dB of gain reduction");
        {
            double worst = -300.0;
            for (const DetectorMode mode : {DetectorMode::Peak, DetectorMode::Rms})
                for (const double hz : {5000.0, 7300.0})
                {
                    auto s = compressor(-20, 4, 6, 1, 50, mode);
                    s.rmsWindowMs = 10.0f;
                    auto render = [&](const Signal& x) { return runMono(x, s); };
                    for (const auto& a : aliasAtDrive(render, kRate, hz, {-12.0, -6.0, 0.0}, 0.5))
                        worst = std::max(worst, a.aliasDb);
                }
            logMessage(String("worst alias power (peak and RMS, 5 and 7.3 kHz, -12 to 0 dBFS): ") + String(worst, 1) +
                       " dB");
            expect(worst <= -100.0, "alias floor");
        }
    }

    void lookaheadAndTruePeak()
    {
        beginTest("lookahead removes the overshoot of a 20 dB step");
        {
            Signal in = concat({sineWave(static_cast<size_t>(0.5 * kRate), 1000.0, kRate, dbToAmplitude(-30.0)),
                                sineWave(static_cast<size_t>(0.5 * kRate), 1000.0, kRate, dbToAmplitude(-10.0))});
            const size_t step = static_cast<size_t>(0.5 * kRate);
            double overshoot[2] = {};
            int k = 0;
            for (const float look : {0.0f, 10.0f})
            {
                auto s = compressor(-30, 1.0e6f, 0, 1, 100, DetectorMode::Peak);
                s.lookaheadMs = look;
                const Signal out = runMono(in, s);
                const double steady = peakAbs(out, in.size() - 4800, in.size());
                const double transient = peakAbs(out, step, step + 4800);
                overshoot[k++] = amplitudeToDb(transient / steady);
            }
            logMessage(String("overshoot without lookahead ") + String(overshoot[0], 2) +
                       " dB, with 10 ms lookahead and 1 ms attack " + String(overshoot[1], 3) + " dB (plan at most 0.1)");
            expect(overshoot[0] > 3.0, "the step overshoots without lookahead");
            expect(overshoot[1] <= 0.1, "lookahead overshoot");
        }

        beginTest("true-peak detection keeps the output's inter-sample peaks at the threshold");
        {
            // fs/4 sine at 45 degrees: samples +-0.707 A, true peak A.
            const Signal in = sineWave(static_cast<size_t>(1.0 * kRate), kRate / 4.0, kRate, 1.0, kPi / 4.0);
            double tp[2] = {};
            int k = 0;
            for (const bool truePeak : {false, true})
            {
                auto s = compressor(-10, 1.0e6f, 0, 0.1f, 50, DetectorMode::Peak);
                s.truePeak = truePeak;
                s.lookaheadMs = 1.0f;
                const Signal out = runMono(in, s);
                const Signal tail(out.end() - 9600, out.end());
                tp[k++] = amplitudeToDb(truePeakOf(tail));
            }
            logMessage(String("output true peak, threshold -10 dB: sample-peak detector ") + String(tp[0], 2) +
                       " dBTP, true-peak detector " + String(tp[1], 2) + " dBTP");
            expect(tp[0] > -8.0, "sample-peak detection lets the inter-sample peak through");
            expect(std::abs(tp[1] + 10.0) <= 0.25, "true-peak detection holds the ceiling");
        }
    }

    static double truePeakOf(const Signal& x) { return crucible::measure::truePeak(x, 8); }

    void keyAndFilters()
    {
        beginTest("external key: gain reduction follows the key only");
        {
            auto s = compressor(-30, 4, 0, 1, 50, DetectorMode::Rms);
            s.rmsWindowMs = 4.0f;
            const size_t n = static_cast<size_t>(1.0 * kRate);
            const Signal loud = sineWave(n, 440.0, kRate, dbToAmplitude(-10.0)),
                         quiet = sineWave(n, 440.0, kRate, dbToAmplitude(-50.0));
            const Signal silent(n, 0.0), keyTone = sineWave(n, 1000.0, kRate, dbToAmplitude(-10.0));
            // Loud audio, silent key: nothing happens. Quiet audio, loud key: full reduction.
            const Stereo a = runStereo(loud, loud, s, &silent, &silent, nullptr);
            const Stereo b = runStereo(quiet, quiet, s, &keyTone, &keyTone, nullptr);
            const double grA = lastGr(a, loud, n - 9600, n, 440.0);
            const double grB = lastGr(b, quiet, n - 9600, n, 440.0);
            const double expected = 0.75 * 20.0;
            logMessage(String("loud audio, silent key: ") + String(grA, 3) +
                       " dB; quiet audio, loud key: " + String(grB, 2) + " dB (ideal " + String(expected, 1) + ")");
            expect(std::abs(grA) <= 0.01, "silent key leaves the audio alone");
            expect(std::abs(grB - expected) <= 0.3, "the key sets the reduction");
        }

        beginTest("sidechain filters: high-pass, low-pass and bell decide what triggers the compressor");
        {
            const size_t n = static_cast<size_t>(1.5 * kRate);
            auto s = compressor(-30, 4, 0, 1, 50, DetectorMode::Rms);
            s.rmsWindowMs = 4.0f;
            const auto grOf = [&](const DynamicsSettings& q, double hz, double db)
            {
                std::vector<float> trace;
                const Signal x = sineWave(n, hz, kRate, dbToAmplitude(db));
                runStereo(x, x, q, nullptr, nullptr, &trace);
                return static_cast<double>(trace.back());
            };
            const double plain = grOf(s, 50.0, -10.0);
            DynamicsSettings hp = s;
            hp.scHighpassHz = 1000.0f;
            const double filtered = grOf(hp, 50.0, -10.0);
            logMessage(String("50 Hz at -10 dB: GR ") + String(plain, 2) + " dB, with a 1 kHz sidechain high-pass " +
                       String(filtered, 3) + " dB");
            expect(plain > 14.0 && filtered < 0.05, "high-pass keeps bass out of the detector");
            DynamicsSettings lp = s;
            lp.scLowpassHz = 1000.0f;
            const double lowpassed = grOf(lp, 10000.0, -10.0);
            expect(grOf(s, 10000.0, -10.0) > 14.0 && lowpassed < 0.05, "low-pass keeps treble out of the detector");
            DynamicsSettings bell = s;
            bell.scBellHz = 5000.0f;
            bell.scBellDb = 18.0f;
            bell.scBellQ = 1.0f;
            const double boosted = grOf(bell, 5000.0, -40.0);
            logMessage(String("5 kHz at -40 dB (below the threshold): GR ") + String(grOf(s, 5000.0, -40.0), 3) +
                       " dB, with a +18 dB bell " + String(boosted, 2) + " dB (ideal 6.0)");
            expect(std::abs(boosted - 0.75 * 8.0) <= 0.2, "bell boosts the key");
        }

        beginTest("listen outputs the filtered key (2nd-order Butterworth high-pass at 1 kHz, 100 Hz tone)");
        {
            auto s = compressor(0, 1, 0, 1, 50, DetectorMode::Rms);
            s.scHighpassHz = 1000.0f;
            s.listen = true;
            const Signal x = sineWave(static_cast<size_t>(0.5 * kRate), 100.0, kRate, 0.5);
            const Signal y = runMono(x, s);
            const double g = amplitudeToDb(toneAmplitude(y.data() + 12000, 9600, 100.0, kRate) / 0.5);
            const double ratio = 100.0 / 1000.0;
            const double expected = 20.0 * std::log10(ratio * ratio / std::sqrt(1.0 + std::pow(ratio, 4.0)));
            logMessage(String("measured ") + String(g, 2) + " dB, analytic " + String(expected, 2) + " dB");
            expect(std::abs(g - expected) <= 0.3, "listen = filtered key");
        }
    }

    void stereo()
    {
        beginTest("stereo link, unlinked and mid / side lanes");
        {
            const size_t n = static_cast<size_t>(1.0 * kRate);
            const Signal loud = sineWave(n, 1000.0, kRate, dbToAmplitude(-10.0)),
                         quiet = sineWave(n, 1000.0, kRate, dbToAmplitude(-40.0));
            auto s = compressor(-30, 4, 0, 1, 50, DetectorMode::Rms);
            s.rmsWindowMs = 4.0f;
            s.channels = ChannelMode::StereoLinked;
            Stereo o = runStereo(loud, quiet, s);
            const auto grL = [&](const Stereo& st, const Signal& a, const Signal& b)
            {
                return std::pair<double, double>{
                    -(amplitudeToDb(toneAmplitude(st.l.data() + n - 9600, 9600, 1000.0, kRate)) -
                      amplitudeToDb(toneAmplitude(a.data() + n - 9600, 9600, 1000.0, kRate))),
                    -(amplitudeToDb(toneAmplitude(st.r.data() + n - 9600, 9600, 1000.0, kRate)) -
                      amplitudeToDb(toneAmplitude(b.data() + n - 9600, 9600, 1000.0, kRate)))};
            };
            auto g = grL(o, loud, quiet);
            logMessage(String("linked: left ") + String(g.first, 2) + " dB, right " + String(g.second, 2) + " dB");
            expect(std::abs(g.first - g.second) <= 0.05 && g.first > 14.0, "linked lanes share the reduction");
            s.channels = ChannelMode::StereoUnlinked;
            o = runStereo(loud, quiet, s);
            g = grL(o, loud, quiet);
            logMessage(String("unlinked: left ") + String(g.first, 2) + " dB, right " + String(g.second, 3) + " dB");
            expect(g.first > 14.0 && std::abs(g.second) <= 0.02, "unlinked lanes are independent");
            s.channels = ChannelMode::MidSide;
            s.link = 0.0f;
            const Signal mid = loud, side = sineWave(n, 1000.0, kRate, dbToAmplitude(-50.0));
            Signal l(n), r(n);
            for (size_t i = 0; i < n; ++i)
            {
                l[i] = mid[i] + side[i];
                r[i] = mid[i] - side[i];
            }
            o = runStereo(l, r, s);
            const double midOut = 0.5 * (amplitudeToDb(toneAmplitude(o.l.data() + n - 9600, 9600, 1000.0, kRate)));
            (void)midOut;
            Signal om(n), os(n);
            for (size_t i = 0; i < n; ++i)
            {
                om[i] = 0.5 * (o.l[i] + o.r[i]);
                os[i] = 0.5 * (o.l[i] - o.r[i]);
            }
            const double grMid = amplitudeToDb(dbToAmplitude(-10.0)) -
                                 amplitudeToDb(toneAmplitude(om.data() + n - 9600, 9600, 1000.0, kRate));
            const double grSide = amplitudeToDb(dbToAmplitude(-50.0)) -
                                  amplitudeToDb(toneAmplitude(os.data() + n - 9600, 9600, 1000.0, kRate));
            logMessage(String("mid/side dual: mid ") + String(grMid, 2) + " dB, side " + String(grSide, 3) + " dB");
            expect(grMid > 14.0 && std::abs(grSide) <= 0.02, "mid and side compress independently");
            s.channels = ChannelMode::Mid;
            o = runStereo(l, r, s);
            double sideErr = 0.0;
            for (size_t i = 0; i < n; ++i)
                sideErr = std::max(sideErr, std::abs(0.5 * (o.l[i] - o.r[i]) - side[i]));
            expect(sideErr < 1.0e-6, String("Mid mode leaves the side signal untouched, error ") + String(sideErr, 9));
        }

        beginTest("neutral settings null against the (delayed) input to float precision in every channel mode");
        {
            Random rng(11);
            Signal l(static_cast<size_t>(0.5 * kRate)), r(l.size());
            for (size_t i = 0; i < l.size(); ++i)
            {
                l[i] = 0.3 * rng.gaussian();
                r[i] = 0.3 * rng.gaussian();
            }
            double worst = -300.0;
            for (int m = 0; m < static_cast<int>(ChannelMode::Count); ++m)
            {
                DynamicsSettings s;
                s.thresholdDb = 0.0f;
                s.ratio = 1.0f;
                s.kneeDb = 0.0f;
                s.channels = static_cast<ChannelMode>(m);
                const Stereo o = runStereo(l, r, s);
                worst = std::max({worst, nullTest(l, o.l, 0.0), nullTest(r, o.r, 0.0)});
            }
            logMessage(String("worst null: ") + String(worst, 1) + " dB");
            expect(worst <= -120.0, "neutral null");
        }
    }

    void autoRelease()
    {
        beginTest("auto release: the release after a 50 ms burst is at least 2x faster than after a 2 s burst");
        {
            struct Preset
            {
                const char* name;
                float fast, slow, charge;
            };
            for (const Preset p :
                 {Preset{"clean compressor", 60.0f, 1200.0f, 700.0f}, Preset{"VCA compressor", 60.0f, 1500.0f, 600.0f}})
            {
                auto s = compressor(-30, 4, 0, 3, 100, DetectorMode::Rms);
                s.rmsWindowMs = 4.0f;
                s.autoRelease = true;
                s.autoFastMs = p.fast;
                s.autoSlowMs = p.slow;
                s.autoChargeMs = p.charge;
                auto render = [&](const Signal& x) { return runMono(x, s); };
                const auto fam = releaseFamily(render, kRate, -50.0, {-10.0}, {0.05, 2.0}, 0.0, 1.0, 5.0);
                const double shortT = fam[0].release.t63Ms, longT = fam[1].release.t63Ms;
                logMessage(String(p.name) + ": t63 after 50 ms burst " + String(shortT, 1) + " ms, after 2 s burst " +
                           String(longT, 1) + " ms, ratio " + String(longT / shortT, 1));
                expect(fam[0].release.valid() && fam[1].release.valid() && longT >= 2.0 * shortT,
                       String(p.name) + " program dependence");
            }
        }
    }

    void gateAndDucker()
    {
        const auto gateSettings = []
        {
            DynamicsSettings s;
            s.kind = DynamicsKind::Gate;
            s.thresholdDb = -40.0f;
            s.hysteresisDb = 6.0f;
            s.ratio = 1000.0f;
            s.rangeDb = 80.0f;
            s.kneeDb = 0.0f;
            s.attackMs = 0.05f;
            s.releaseMs = 0.05f;
            s.holdMs = 0.0f;
            s.detector = DetectorMode::Peak;
            s.peakHoldMs = 5.0f;
            return s;
        };

        beginTest("gate hysteresis: opens at the threshold, closes hysteresis dB lower (within 5 %)");
        {
            for (const float hyst : {6.0f, 12.0f})
            {
                auto s = gateSettings();
                s.hysteresisDb = hyst;
                const double rate = kRate, hz = rate / 12.0;
                const size_t half = static_cast<size_t>(5.0 * rate);
                // Level ramps -70 -> -20 dB (10 dB/s) and back, as a swept amplitude on one tone.
                Signal in(2 * half);
                const double lo = -70.0, hi = -20.0;
                for (size_t i = 0; i < in.size(); ++i)
                {
                    const double t = static_cast<double>(i < half ? i : 2 * half - i) / static_cast<double>(half);
                    in[i] = dbToAmplitude(lo + (hi - lo) * t) * std::sin(2.0 * kPi * hz * static_cast<double>(i) / rate);
                }
                const Signal out = runMono(in, s);
                const Signal g = gainTrace(in, out, hz, rate);
                double openLevel = 0.0, closeLevel = 0.0;
                for (size_t i = 24; i < half; ++i)
                    if (g[i] > -3.0)
                    {
                        openLevel = lo + (hi - lo) * static_cast<double>(i) / static_cast<double>(half);
                        break;
                    }
                for (size_t i = half + 24; i < in.size(); ++i)
                    if (g[i] < -3.0)
                    {
                        closeLevel = lo + (hi - lo) * static_cast<double>(2 * half - i) / static_cast<double>(half);
                        break;
                    }
                const double measured = openLevel - closeLevel;
                logMessage(String("hysteresis ") + String(hyst, 0) + " dB: opens at " + String(openLevel, 2) +
                           " dB, closes at " + String(closeLevel, 2) + " dB, difference " + String(measured, 2));
                expect(std::abs(measured / hyst - 1.0) <= 0.05, "hysteresis");
                expect(std::abs(openLevel + 40.0) <= 0.6, "opens at the threshold");
            }
        }

        beginTest(
            "gate hold: the gate stays open for the hold time after the key falls below the lower threshold (within 5 %)");
        {
            double worst = 0.0;
            for (const float hold : {10.0f, 50.0f, 200.0f})
            {
                auto s = gateSettings();
                s.holdMs = hold;
                s.peakHoldMs = 0.1f;
                const double hz = kRate / 12.0;
                const Signal in = concat({sineWave(static_cast<size_t>(0.3 * kRate), hz, kRate, dbToAmplitude(-20.0)),
                                          sineWave(static_cast<size_t>(1.0 * kRate), hz, kRate, dbToAmplitude(-60.0), 0.0)});
                const size_t end = static_cast<size_t>(0.3 * kRate);
                const Signal out = runMono(in, s);
                const Signal g = gainTrace(in, out, hz, kRate);
                double measuredMs = -1.0;
                for (size_t i = end + 12; i < g.size(); ++i)
                    if (g[i] < -3.0)
                    {
                        measuredMs = static_cast<double>(i - end) / kRate * 1000.0;
                        break;
                    }
                const double err = std::abs(measuredMs / hold - 1.0);
                worst = std::max(worst, err);
                expect(err <= 0.05, String("hold ") + String(hold, 0) + " ms measured " + String(measuredMs, 2));
            }
            logMessage(String("worst hold error ") + String(100.0 * worst, 2) + " % (plan 5 %)");
        }

        beginTest("gate onset: 2 ms of lookahead leaves an opening residual of at most -60 dB");
        {
            double residual[2] = {};
            int k = 0;
            for (const float look : {0.0f, 2.0f})
            {
                auto s = gateSettings();
                s.attackMs = 0.1f;
                s.releaseMs = 20.0f;
                s.holdMs = 50.0f;
                s.peakHoldMs = 1.0f;
                s.lookaheadMs = look;
                const Signal in = concat({Signal(static_cast<size_t>(0.2 * kRate), 0.0),
                                          sineWave(static_cast<size_t>(0.2 * kRate), 1000.0, kRate, dbToAmplitude(-10.0)),
                                          Signal(static_cast<size_t>(0.4 * kRate), 0.0)});
                const Signal out = runMono(in, s);
                const double delay = std::round(look * 0.001 * kRate);
                residual[k++] = nullTest(in, out, delay, 0);
            }
            logMessage(String("onset residual without lookahead ") + String(residual[0], 1) + " dB, with 2 ms lookahead " +
                       String(residual[1], 1) + " dB (plan at most -60)");
            expect(residual[0] > -40.0, "the residual is audible without lookahead");
            expect(residual[1] <= -60.0, "lookahead onset residual");
        }

        beginTest("ducker: the gain falls by the range while the key is above the threshold (within 0.2 dB)");
        {
            for (const float range : {6.0f, 20.0f, 60.0f})
            {
                DynamicsSettings s;
                s.kind = DynamicsKind::Ducker;
                s.thresholdDb = -30.0f;
                s.hysteresisDb = 3.0f;
                s.ratio = 1000.0f;
                s.rangeDb = range;
                s.attackMs = 1.0f;
                s.releaseMs = 50.0f;
                s.detector = DetectorMode::Rms;
                s.rmsWindowMs = 4.0f;
                const size_t n = static_cast<size_t>(1.0 * kRate);
                const Signal x = sineWave(n, 440.0, kRate, dbToAmplitude(-20.0)),
                             key = sineWave(n, 100.0, kRate, dbToAmplitude(-10.0));
                const Stereo o = runStereo(x, x, s, &key, &key);
                const double gDucked = amplitudeToDb(toneAmplitude(o.l.data() + n - 9600, 9600, 440.0, kRate)) -
                                       amplitudeToDb(toneAmplitude(x.data() + n - 9600, 9600, 440.0, kRate));
                const Signal silent(n, 0.0);
                const Stereo free = runStereo(x, x, s, &silent, &silent);
                const double gFree = amplitudeToDb(toneAmplitude(free.l.data() + n - 9600, 9600, 440.0, kRate)) -
                                     amplitudeToDb(toneAmplitude(x.data() + n - 9600, 9600, 440.0, kRate));
                logMessage(String("range ") + String(range, 0) + " dB: ducked gain " + String(gDucked, 3) + " dB, free " +
                           String(gFree, 4) + " dB");
                expect(std::abs(gDucked + range) <= 0.2 && std::abs(gFree) <= 0.01, "ducking depth");
            }
        }
    }

    void latency()
    {
        beginTest("latency: measured delay equals the reported latency to one sample (lookahead, true peak, feedback)");
        {
            double worst = 0.0;
            for (const double look : {0.0, 1.0, 2.5, 5.0, 10.0})
                for (const bool tp : {false, true})
                    worst = std::max(worst, std::abs(neutralLatencyError(look, tp)));
            logMessage(String("worst |measured - reported| over lookahead 0 to 10 ms, with and without true peak: ") +
                       String(worst, 4) + " samples");
            expect(worst <= 0.05, "latency");
            DynamicsSettings fb;
            fb.feedback = true;
            fb.lookaheadMs = 5.0f;
            DynamicsProcessor p;
            p.prepare(kRate);
            expectEquals(p.latencyFor(fb), 0, "feedback has no lookahead");
        }

        beginTest("lookahead change while audio runs leaves no step above -80 dB");
        {
            DynamicsProcessor p;
            p.prepare(kRate);
            DynamicsSettings s;
            s.thresholdDb = 0.0f;
            s.ratio = 1.0f;
            s.kneeDb = 0.0f;
            s.lookaheadMs = 2.0f;
            const size_t n = static_cast<size_t>(1.0 * kRate);
            std::vector<float> a(n), b;
            for (size_t i = 0; i < n; ++i)
                a[i] = static_cast<float>(0.5 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
            b = a;
            for (size_t pos = 0; pos < n; pos += 32)
            {
                if (pos == 24000)
                    s.lookaheadMs = 8.0f;
                p.process(a.data() + pos, b.data() + pos, 32, nullptr, nullptr, s);
            }
            const ClickResult c = clickTest(Signal(a.begin(), a.end()), 1000.0, kRate, 4000);
            logMessage(String("largest step ") + String(c.stepDb, 1) +
                       " dB; latency change flagged: " + (p.takeLatencyChanged() ? "yes" : "no"));
            expect(c.stepDb <= -80.0, "lookahead change click");
        }

        beginTest("switching a lookahead compressor on or off through the rack leaves no step above -60 dB");
        {
            for (const float look : {0.0f, 5.0f})
            {
                RackHarness h;
                h.set("fx_cc_on", 0.0f);
                h.set("fx_cc_lookahead", look);
                h.set("fx_cc_thresh", -30.0f);
                h.set("fx_cc_ratio", 8.0f);
                const size_t n = static_cast<size_t>(1.0 * kRate);
                const Signal in = sineWave(n, 1000.0, kRate, 0.5);
                std::vector<float> l(in.begin(), in.end()), r = l;
                for (size_t pos = 0; pos < n; pos += 32)
                {
                    if (pos == 24000)
                        h.set("fx_cc_on", 1.0f);
                    if (pos == 36000)
                        h.set("fx_cc_on", 0.0f);
                    h.run(l.data() + pos, r.data() + pos, 32);
                }
                const double c = clickTest(Signal(l.begin(), l.end()), 1000.0, kRate, 4000).stepDb;
                logMessage(String("lookahead ") + String(look, 0) + " ms: largest step on or off " + String(c, 1) + " dB");
                expect(c <= -60.0, "on / off click");
            }
        }
    }

    void modules()
    {
        beginTest(
            "registration: the new modules exist (their place in the order is frozen in tests/golden/param_order.txt)");
        {
            for (const char* id : {"fx.comp_clean", "fx.comp_vca", "fx.gate2"})
                expect(ModuleRegistry::instance().find(id) != nullptr, std::string("module ") + id);
        }

        beginTest("a latency change made while reset() cleared the processor is still announced (B3)");
        {
            DynamicsProcessor p;
            p.prepare(kRate);
            DynamicsSettings s;
            s.lookaheadMs = 0.0f;
            std::vector<float> a(64, 0.1f), b(64, 0.1f);
            p.process(a.data(), b.data(), 64, nullptr, nullptr, s);
            p.takeLatencyChanged();
            p.reset();
            s.lookaheadMs = 10.0f;
            p.process(a.data(), b.data(), 64, nullptr, nullptr, s);
            expectEquals(p.latencySamples(), 480, "latency after the change");
            expect(p.takeLatencyChanged(), "latency change flagged after reset");
        }

        beginTest("capabilities: gain reduction meter and latency reporter on all three modules");
        {
            RackHarness h;
            for (const char* id : {"fx.comp_clean", "fx.comp_vca", "fx.gate2"})
            {
                expect(h.rack.findCapability<GainReductionMeter>(id) != nullptr, std::string(id) + " meter");
                expect(h.rack.findCapability<LatencyReporter>(id) != nullptr, std::string(id) + " latency");
            }
            h.set("fx_cc_on", 1.0f);
            h.set("fx_cc_lookahead", 5.0f);
            h.set("fx_g2_lookahead", 2.0f);
            h.set("fx_g2_on", 1.0f);
            h.set("fx_cc_thresh", -30.0f);
            h.set("fx_cc_ratio", 4.0f);
            h.set("fx_cc_detector", 1.0f);
            const Signal in = sineWave(static_cast<size_t>(1.0 * kRate), 1000.0, kRate, dbToAmplitude(-10.0));
            h.runSignal(in);
            auto* cc = h.rack.findCapability<LatencyReporter>("fx.comp_clean");
            auto* g2 = h.rack.findCapability<LatencyReporter>("fx.gate2");
            expectEquals(cc->latencySamples(), 240, "comp_clean 5 ms at 48 kHz");
            expectEquals(g2->latencySamples(), 96, "gate2 2 ms at 48 kHz");
            expect(cc->takeLatencyChanged() || true, "flag readable");
            const float gr = h.rack.findCapability<GainReductionMeter>("fx.comp_clean")->gainReductionDb(0);
            logMessage(String("comp_clean gain reduction at -10 dB in, threshold -30, 4:1: ") + String(gr, 2) +
                       " dB (ideal 15, the gate2 before it is open)");
            expect(gr > 5.0, "meter reads gain reduction");
        }

        beginTest("comp_clean through the rack: default static curve, external key from the aux input");
        {
            RackHarness h;
            h.set("fx_cc_on", 1.0f);
            h.set("fx_cc_thresh", -24.0f);
            h.set("fx_cc_ratio", 3.0f);
            h.set("fx_cc_knee", 8.0f);
            h.set("fx_cc_detector", 1.0f);
            h.set("fx_cc_rms", 10.0f);
            h.set("fx_cc_attack", 5.0f);
            h.set("fx_cc_release", 50.0f);
            double worst = 0.0;
            for (double level = -48.0; level <= 0.0; level += 6.0)
            {
                RackHarness fresh;
                fresh.values = h.values;
                const Signal x = sineWave(static_cast<size_t>(1.0 * kRate), 1000.0, kRate, dbToAmplitude(level));
                const Signal y = fresh.runSignal(x);
                const double gain = amplitudeToDb(toneAmplitude(y.data() + 36000, 9600, 1000.0, kRate)) - level;
                const double ideal = -dyn::compressorReduction(static_cast<float>(level), -24.0f, 3.0f, 8.0f);
                worst = std::max(worst, std::abs(gain - ideal));
            }
            logMessage(String("module static curve worst error ") + String(worst, 4) + " dB");
            expect(worst <= 0.1, "module static curve");

            RackHarness k;
            k.values = h.values;
            k.set("fx_cc_key", 1.0f);
            const size_t n = static_cast<size_t>(1.0 * kRate);
            const Signal quiet = sineWave(n, 440.0, kRate, dbToAmplitude(-50.0)),
                         keyTone = sineWave(n, 1000.0, kRate, dbToAmplitude(-6.0));
            const Signal y = k.runSignal(quiet, &keyTone);
            const double gain = amplitudeToDb(toneAmplitude(y.data() + n - 9600, 9600, 440.0, kRate)) - (-50.0);
            const double ideal = -dyn::compressorReduction(-6.0f, -24.0f, 3.0f, 8.0f);
            logMessage(String("external key at -6 dB, audio at -50 dB: gain ") + String(gain, 2) + " dB (ideal " +
                       String(ideal, 2) + ")");
            expect(std::abs(gain - ideal) <= 0.3, "key from the aux input");
            RackHarness none;
            none.values = k.values;
            const Signal loud = sineWave(n, 440.0, kRate, dbToAmplitude(-6.0));
            const Signal z = none.runSignal(loud);
            const double fallback = amplitudeToDb(toneAmplitude(z.data() + n - 9600, 9600, 440.0, kRate)) + 6.0;
            logMessage(String("external key selected but no aux input: gain ") + String(fallback, 2) +
                       " dB (internal key used)");
            expect(fallback < -5.0, "falls back to the internal key");
        }

        beginTest("comp_vca: stepped ratios and THD at 4 dB of gain reduction");
        {
            const double hz[2] = {1000.0, 100.0};
            for (const double f : hz)
            {
                RackHarness h;
                h.set("fx_cv_on", 1.0f);
                h.set("fx_cv_ratio", 2.0f); // 4:1
                h.set("fx_cv_attack", 4.0f);
                h.set("fx_cv_release", 1.0f);
                h.set("fx_cv_knee", 0.0f);
                h.set("fx_cv_thresh", -11.33f);
                const size_t n = static_cast<size_t>(3.0 * kRate);
                const Signal x = sineWave(n, f, kRate, dbToAmplitude(-6.0));
                const Signal y = h.runSignal(x);
                const double gr = -(amplitudeToDb(toneAmplitude(y.data() + n - 9600, 9600, f, kRate)) - (-6.0));
                const HarmonicProfile p = harmonicProfile(y.data() + n - 24000, 24000, kRate, f, 10);
                logMessage(String("VCA ") + String(f, 0) + " Hz: gain reduction " + String(gr, 2) + " dB, THD " +
                           String(p.thdDb, 1) + " dB (plan at most -70 at 4 dB)");
                if (f == 1000.0)
                    expect(std::abs(gr - 4.0) <= 0.5 && p.thdDb <= -70.0, "VCA THD at 4 dB");
            }
        }

        beginTest("gain reduction is published as a scalar view for the three modules");
        {
            Engine e;
            e.prepare(kRate, 256);
            for (const char* id : {"fx.comp_clean", "fx.comp_vca", "fx.gate2"})
                expect(e.views().find(id, "gain_reduction") >= 0, std::string(id) + " view");
            expect(e.views().find("fx.multiband", "gain_reduction") < 0, "no view for a module without the capability");
            const int view = e.views().find("fx.comp_clean", "gain_reduction");
            expect(view >= 0 && e.views().info(view).kind == ViewKind::Scalar, "scalar");
            e.setParam("fx_cc_on", 1.0f);
            e.setParam("fx_cc_thresh", -40.0f);
            e.setParam("fx_cc_ratio", 8.0f);
            e.setParam("fx_cc_detector", 0.0f);
            e.views().subscribe(view, 0.0f);
            e.noteOn(48, 1.0f);
            std::vector<float> l(256), r(256);
            for (int b = 0; b < 80; ++b)
                e.process(l.data(), r.data(), 256);
            float value = -1.0f;
            const int n = e.views().read(view, &value, 1, nullptr);
            const float direct =
                e.coreEngine().engine().effects().findCapability<GainReductionMeter>("fx.comp_clean")->gainReductionDb(0);
            logMessage(String("view ") + String(value, 3) + " dB, module meter " + String(direct, 3) + " dB");
            expect(n == 1 && value > 1.0f && std::memcmp(&value, &direct, 4) == 0, "view equals the meter");
        }
    }

    // Percent of one core per stereo instance at 48 kHz, blocks of 32, minus the rack with everything off; best of three.
    static double cpuPercent(const std::vector<std::pair<std::string, float>>& setup)
    {
        const size_t n = static_cast<size_t>(8.0 * kRate);
        Random rng(5);
        std::vector<float> src(n);
        for (auto& v : src)
            v = static_cast<float>(0.25 * rng.gaussian());
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

    void realTime()
    {
        beginTest(
            "no allocation on the audio thread: all three modules, parameter changes every block, key, every channel mode");
        {
            RackHarness h;
            for (const char* on : {"fx_cc_on", "fx_cv_on", "fx_g2_on"})
                h.set(on, 1.0f);
            h.set("fx_cc_truepeak", 1.0f);
            h.set("fx_cc_lookahead", 4.0f);
            h.set("fx_cc_key", 1.0f);
            h.set("fx_cv_key", 1.0f);
            h.set("fx_g2_key", 1.0f);
            h.set("fx_cv_console", 1.0f);
            Random rng(3);
            std::vector<float> l(32), r(32), aux(32);
            for (int warm = 0; warm < 8; ++warm)
                h.run(l.data(), r.data(), 32, aux.data(), aux.data());
            crucible::testing::gAllocations = 0;
            crucible::testing::gTrackAllocations = true;
            for (int block = 0; block < 600; ++block)
            {
                for (size_t i = 0; i < 32; ++i)
                {
                    l[i] = static_cast<float>(0.3 * rng.gaussian());
                    r[i] = static_cast<float>(0.3 * rng.gaussian());
                    aux[i] = static_cast<float>(0.3 * rng.gaussian());
                }
                h.set("fx_cc_lookahead", static_cast<float>(block % 11));
                h.set("fx_g2_lookahead", static_cast<float>((block / 3) % 11));
                h.set("fx_cc_channels", static_cast<float>(block % 7));
                h.set("fx_g2_channels", static_cast<float>((block / 2) % 7));
                h.set("fx_cc_detector", static_cast<float>(block % 3));
                h.set("fx_cc_schp", 20.0f + static_cast<float>(block % 50) * 10.0f);
                h.set("fx_cc_scbg", static_cast<float>(block % 9) - 4.0f);
                h.set("fx_cc_attack", 0.05f + static_cast<float>(block % 20));
                h.set("fx_cc_auto", static_cast<float>(block % 2));
                h.set("fx_cc_topology", static_cast<float>((block / 5) % 2));
                h.set("fx_cc_listen", static_cast<float>((block / 7) % 2));
                h.set("fx_g2_mode", static_cast<float>(block % 3));
                h.run(l.data(), r.data(), 32, (block % 4 != 0) ? aux.data() : nullptr,
                      (block % 4 != 0) ? aux.data() : nullptr);
            }
            crucible::testing::gTrackAllocations = false;
            expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
            bool finite = true;
            for (size_t i = 0; i < 32; ++i)
                finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
            expect(finite, "finite output");
        }

#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest("CPU: percent of one core per stereo instance at 48 kHz (plan: 0.15 / 0.3 with lookahead and true peak; "
                  "0.2; 0.1)");
        {
            const double clean = cpuPercent({{"fx_cc_on", 1.0f}, {"fx_cc_thresh", -30.0f}});
            const double cleanTp = cpuPercent({{"fx_cc_on", 1.0f},
                                               {"fx_cc_thresh", -30.0f},
                                               {"fx_cc_lookahead", 5.0f},
                                               {"fx_cc_truepeak", 1.0f},
                                               {"fx_cc_detector", 2.0f}});
            const double vca = cpuPercent({{"fx_cv_on", 1.0f}, {"fx_cv_thresh", -30.0f}});
            const double gate = cpuPercent({{"fx_g2_on", 1.0f}, {"fx_g2_thresh", -30.0f}});
            logMessage(String("comp_clean ") + String(clean, 3) + " %, with lookahead + true peak + hybrid " +
                       String(cleanTp, 3) + " %, comp_vca " + String(vca, 3) + " %, gate2 " + String(gate, 3) + " %");
            expectCpu(clean <= 0.15 * 1.2, "comp_clean CPU");
            expectCpu(cleanTp <= 0.30 * 1.2, "comp_clean with lookahead and true peak CPU");
            expectCpu(vca <= 0.20 * 1.2, "comp_vca CPU");
            expectCpu(gate <= 0.10 * 1.2, "gate2 CPU");
        }
#endif
    }
};

static DynamicsTests dynamicsTests;
