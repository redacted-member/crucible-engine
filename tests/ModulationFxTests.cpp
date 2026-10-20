// SPDX-License-Identifier: MIT
// Modulation effects of FX11 (docs/plans/ENGINE-FX-PLAN.md section 3.6): fx.ensemble here, the phaser and the flanger
// as they arrive. Every claim is a number; the printed lines are what docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "CpuTiming.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/fx/Bbd.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
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

namespace
{
// The sample rate of the rack; runTest repeats the main accept rows at 44.1 / 96 / 192 kHz by changing it.
double kRate = 48000.0;

// A length given in samples at 48 kHz, in samples at the rack rate.
size_t scaled(size_t samplesAt48k)
{
    return static_cast<size_t>(std::llround(static_cast<double>(samplesAt48k) * kRate / 48000.0));
}

std::string rateTag()
{
    if (kRate == 48000.0)
        return std::string();
    char tag[32];
    std::snprintf(tag, sizeof(tag), "[%g kHz] ", kRate / 1000.0);
    return tag;
}

// One module effect through the rack, every other effect off.
struct Rack
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values;
    std::vector<float> legacy;
    explicit Rack(const std::string& onParam)
    {
        ModuleContext ctx;
        ctx.sampleRate = kRate;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set(onParam, 1.0f);
    }
    static int ix(const std::string& id) { return ParameterMap::indexOf(id); }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ix(id))] = v; }
    void run(float* l, float* r, int n)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
    }
    void runAux(float* l, float* r, int n, const float* auxL, const float* auxR)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, auxL, auxR);
    }
};

struct Pair
{
    std::vector<float> l, r;
};

template <class Fn> Pair runRack(Rack& h, Pair in, Fn&& atBlock)
{
    const size_t n = in.l.size();
    for (size_t pos = 0; pos < n; pos += 128)
    {
        atBlock(pos);
        h.run(in.l.data() + pos, in.r.data() + pos, static_cast<int>(std::min<size_t>(128, n - pos)));
    }
    return in;
}

Pair runRack(Rack& h, Pair in)
{
    return runRack(h, std::move(in), [](size_t) {});
}

Pair sineBoth(size_t n, double hz, double amp)
{
    Pair p{std::vector<float>(n), std::vector<float>(n)};
    for (size_t i = 0; i < n; ++i)
        p.l[i] = p.r[i] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * static_cast<double>(i) / kRate));
    return p;
}

Pair noiseBoth(size_t n, double rms, bool independent, std::uint64_t seed = 11)
{
    Random rng(seed);
    Pair p{std::vector<float>(n), std::vector<float>(n)};
    for (size_t i = 0; i < n; ++i)
    {
        p.l[i] = static_cast<float>(rms * rng.gaussian());
        p.r[i] = independent ? static_cast<float>(rms * rng.gaussian()) : p.l[i];
    }
    return p;
}

Signal toSignal(const std::vector<float>& x, size_t from = 0)
{
    return Signal(x.begin() + static_cast<long>(from), x.end());
}

double meanSquare(const std::vector<float>& x, size_t from)
{
    double s = 0.0;
    for (size_t i = from; i < x.size(); ++i)
        s += static_cast<double>(x[i]) * x[i];
    return s / static_cast<double>(x.size() - from);
}

double maxSecondDifference(const std::vector<float>& x, size_t from, size_t to)
{
    double m = 0.0;
    for (size_t i = from + 2; i < to; ++i)
        m = std::max(m, std::abs(static_cast<double>(x[i]) - 2.0 * x[i - 1] + x[i - 2]));
    return m;
}

// Amplitude of the hz component of x over [from, x.size()), by correlation.
double toneLevel(const std::vector<float>& x, size_t from, double hz)
{
    std::complex<double> acc(0.0, 0.0);
    for (size_t i = from; i < x.size(); ++i)
        acc += static_cast<double>(x[i]) * std::polar(1.0, -2.0 * kPi * hz * static_cast<double>(i) / kRate);
    return 2.0 * std::abs(acc) / static_cast<double>(x.size() - from);
}

// Clock of a device set to a delay (seconds) for an ensemble that subtracts the device's known extra delay.
double ensembleClock(int stages, double delay)
{
    double clock = Bbd::clockForDelay(stages, delay);
    for (int i = 0; i < 4; ++i)
    {
        const double extra = 7.0 / kRate + 5.2 / (2.0 * kPi * Bbd::kFilterCutoffRatio * std::min(clock, 0.95 * kRate));
        clock = Bbd::clockForDelay(stages, delay - extra);
    }
    return clock;
}

// Magnitude of the transfer function y / x at hz from a Hann-windowed DFT of both over [from, from + len). Exact for a
// linear time-invariant system whose response varies slowly across the window's main lobe; the same noise realisation is in
// both.
double transferAt(const std::vector<float>& x, const std::vector<float>& y, size_t from, size_t len, double hz)
{
    const std::complex<double> rot = std::polar(1.0, -2.0 * kPi * hz / kRate);
    std::complex<double> w(1.0, 0.0), ax(0.0, 0.0), ay(0.0, 0.0);
    for (size_t i = 0; i < len; ++i)
    {
        const double hann = 0.5 - 0.5 * std::cos(2.0 * kPi * (static_cast<double>(i) + 0.5) / static_cast<double>(len));
        ax += hann * static_cast<double>(x[from + i]) * w;
        ay += hann * static_cast<double>(y[from + i]) * w;
        w *= rot;
        if ((i & 1023) == 1023)
            w /= std::abs(w);
    }
    return std::abs(ay) / std::max(std::abs(ax), 1.0e-30);
}

// Frequency of the transfer function's notch near `guess` (within +-span): a grid search on |H|, then a parabola through
// |H|^2 (which is quadratic at a null) with shrinking spacing. depthDb receives 20 log10 of |H| at the refined frequency.
double findNotch(const std::vector<float>& x, const std::vector<float>& y, size_t from, size_t len, double guess,
                 double span, double* depthDb = nullptr)
{
    double best = guess, bestH = 1.0e30;
    for (double f = guess * (1.0 - span); f <= guess * (1.0 + span); f *= 1.004)
    {
        const double h = transferAt(x, y, from, len, f);
        if (h < bestH)
        {
            bestH = h;
            best = f;
        }
    }
    // A deep notch can be narrower than the coarse grid: look again around the best point on a finer one.
    {
        const double centre = best;
        bestH = 1.0e30;
        for (double f = centre * 0.995; f <= centre * 1.005; f *= 1.0002)
        {
            const double h = transferAt(x, y, from, len, f);
            if (h < bestH)
            {
                bestH = h;
                best = f;
            }
        }
    }
    double step = 0.0004;
    for (int pass = 0; pass < 5; ++pass)
    {
        const double ym = transferAt(x, y, from, len, best * (1.0 - step)), y0 = transferAt(x, y, from, len, best),
                     yp = transferAt(x, y, from, len, best * (1.0 + step));
        const double a = ym * ym, b = y0 * y0, c = yp * yp;
        const double den = a - 2.0 * b + c;
        if (den > 0.0)
            best *= 1.0 + step * 0.5 * (a - c) / den;
        step *= 0.35;
    }
    if (depthDb)
        *depthDb = amplitudeToDb(transferAt(x, y, from, len, best));
    return best;
}

// Analytic notch k (0 based) of n identical first-order bilinear all-pass stages at break frequency fc, mixed 1:1 with the
// dry signal.
double allpassNotch(int stages, int k, double fc)
{
    const double g = std::tan(kPi * fc / kRate);
    return kRate / kPi * std::atan(g * std::tan((2.0 * k + 1.0) * kPi / (2.0 * stages)));
}

#if !defined(CRUCIBLE_TEST_UNTIMED)
// Percent of one core per stereo instance at 48 kHz, blocks of 32, minus the rack with the effect off; best of four.
double cpuPercent(const std::string& onParam, const std::vector<std::pair<std::string, float>>& setup)
{
    const size_t n = static_cast<size_t>(6.0 * kRate);
    Random rng(5);
    std::vector<float> src(n);
    for (auto& v : src)
        v = static_cast<float>(0.25 * rng.gaussian());
    const auto run = [&](bool enabled)
    {
        Rack h(onParam);
        h.set(onParam, enabled ? 1.0f : 0.0f);
        if (enabled)
            for (const auto& kv : setup)
                h.set(kv.first, kv.second);
        std::vector<float> l = src, r = src;
        for (size_t pos = 0; pos < 48000; pos += 32)
            h.run(l.data() + pos, r.data() + pos, 32);
        const double t0 = crucible::testing::threadCpuSeconds();
        for (size_t pos = 48000; pos + 32 <= n; pos += 32)
            h.run(l.data() + pos, r.data() + pos, 32);
        return crucible::testing::threadCpuSeconds() - t0;
    };
    crucible::testing::ScopedUserInteractiveQos qos;
    double best = 1.0e9;
    for (int i = 0; i < 4; ++i)
        best = std::min(best, run(true) - run(false));
    return 100.0 * best / (static_cast<double>(n - 48000) / kRate);
}
#endif
} // namespace

class ModulationFxTests : public UnitTest
{
public:
    ModulationFxTests() : UnitTest("ModulationFx", "Effects") {}

    void runTest() override
    {
        ensembleRegistered();
        ensemblePitchDeviation();
        ensembleTriangleDeviation();
        ensembleDimensional();
        ensembleBandwidth();
        ensembleMonoSum();
        ensembleNoZipper();
        ensembleBlockSizeZipper();
        ensembleAllocations();
        ensembleCpu();
        phaserRegistered();
        phaserNotches();
        phaserSweepRange();
        phaserStageTypes();
        phaserFeedbackBounded();
        phaserFeedbackBoundedMatrix();
        phaserModulationSources();
        phaserBarberPole();
        phaserNoZipper();
        phaserExtremesBounded();
        phaserAllocations();
        phaserCpu();
        flangerRegistered();
        flangerLatency();
        flangerNotches();
        flangerThroughZero();
        flangerFeedbackBounded();
        flangerEnvelope();
        flangerBbd();
        flangerNoZipper();
        flangerAllocations();
        flangerCpu();
        // The main accept rows again at the other sample rates (the rows above are the 48 kHz run).
        for (const double rate : {44100.0, 96000.0, 192000.0})
        {
            kRate = rate;
            ensemblePitchDeviation();
            ensembleBandwidth();
            ensembleMonoSum();
            phaserNotches();
            phaserSweepRange();
            phaserStageTypes();
            phaserFeedbackBounded();
            flangerLatency();
            flangerNotches();
            flangerThroughZero();
            flangerFeedbackBounded();
            flangerBbd();
        }
        kRate = 48000.0;
    }

private:
    static void ensembleBase(Rack& h)
    {
        h.set("fx_ensemble_mix", 1.0f);
        h.set("fx_ensemble_lines", 0.0f);
        h.set("fx_ensemble_shape", 1.0f);
        h.set("fx_ensemble_delay", 20.0f);
        h.set("fx_ensemble_noise", -100.0f);
    }

    void ensembleRegistered()
    {
        beginTest(rateTag() +
                  "fx.ensemble: registered after fx.deesser, appends its parameters after the de-esser, status Current");
        const auto effects = ModuleRegistry::instance().ofKind(ModuleKind::Effect);
        size_t at = 0;
        while (at < effects.size() && effects[at]->id != "fx.ensemble")
            ++at;
        expect(at > 0 && at < effects.size() && effects[at - 1]->id == "fx.deesser",
               "directly after fx.deesser in step order");
        expect(ParameterMap::indexOf("fx_ensemble_on") > ParameterMap::indexOf("fx_delay_pro_on"), "after fx.delay_pro");
        expect(ParameterMap::indexOf("fx_deesser_link") + 1 == ParameterMap::indexOf("fx_ensemble_on"),
               "directly after fx.deesser");
    }

    // Peak pitch deviation of a 1 kHz tone through one line (sine oscillator) against the set depth.
    void ensemblePitchDeviation()
    {
        beginTest(rateTag() + "fx.ensemble: pitch deviation equals the set depth within 5 % (sine, one line)");
        double worst = 0.0;
        for (const float rateHz : {0.5f, 1.0f, 3.0f})
            for (const float cents : {4.0f, 10.0f, 20.0f})
            {
                Rack h("fx_ensemble_on");
                ensembleBase(h);
                h.set("fx_ensemble_rate", rateHz);
                h.set("fx_ensemble_depth", cents);
                const Pair out = runRack(h, sineBoth(static_cast<size_t>(8.0 * kRate), 1000.0, 0.1));
                const ModulationResult m = modulationDepth(toSignal(out.l, 24000), kRate);
                const double err = std::abs(m.peakCents - cents) / cents;
                worst = std::max(worst, err);
                logMessage(String("rate ") + String(rateHz, 1) + " Hz depth " + String(cents, 0) + " cents: measured " +
                           String(m.peakCents, 2) + " cents at " + String(m.rateHz, 3) + " Hz");
                expect(err < 0.05, "deviation within 5 %");
            }
        logMessage(String("fx.ensemble worst depth error ") + String(100.0 * worst, 2) + " % (target 5 %)");
    }

    void ensembleTriangleDeviation()
    {
        beginTest(rateTag() + "fx.ensemble: triangle oscillator holds the set depth on its slopes (median of the deviation "
                              "magnitude within 5 %)");
        for (const float cents : {6.0f, 15.0f})
        {
            Rack h("fx_ensemble_on");
            ensembleBase(h);
            h.set("fx_ensemble_shape", 0.0f);
            h.set("fx_ensemble_rate", 1.0f);
            h.set("fx_ensemble_depth", cents);
            const Pair out = runRack(h, sineBoth(static_cast<size_t>(8.0 * kRate), 1000.0, 0.1));
            const Signal f = detail::instantaneousFrequency(toSignal(out.l, 24000), kRate);
            std::vector<double> dev;
            for (size_t i = static_cast<size_t>(0.3 * kRate); i + static_cast<size_t>(0.3 * kRate) < f.size(); ++i)
                dev.push_back(std::abs(centsBetween(f[i], 1000.0)));
            std::nth_element(dev.begin(), dev.begin() + static_cast<long>(dev.size() / 2), dev.end());
            const double med = dev[dev.size() / 2];
            logMessage(String("triangle depth ") + String(cents, 0) + " cents: median magnitude " + String(med, 2));
            expect(std::abs(med - cents) / cents < 0.05, "slope deviation within 5 %");
        }
    }

    void ensembleDimensional()
    {
        beginTest(rateTag() + "fx.ensemble: Dimensional I-IV deviate by their table depth (rate 0.25 / 0.4 / 0.65 / 1.0 Hz, "
                              "5 / 8 / 11 / 14 cents)");
        const double rates[4] = {0.25, 0.4, 0.65, 1.0}, depths[4] = {5.0, 8.0, 11.0, 14.0};
        for (int k = 0; k < 4; ++k)
        {
            Rack h("fx_ensemble_on");
            ensembleBase(h);
            h.set("fx_ensemble_mode", static_cast<float>(k + 1));
            h.set("fx_ensemble_crossfeed", 0.0f);
            h.set("fx_ensemble_shape", 1.0f);
            const Pair out = runRack(h, sineBoth(static_cast<size_t>(12.0 * kRate), 1000.0, 0.5));
            const ModulationResult m = modulationDepth(toSignal(out.l, 24000), kRate);
            logMessage(String("mode ") + String(k + 1) + ": " + String(m.peakCents, 2) + " cents at " + String(m.rateHz, 3) +
                       " Hz");
            expect(std::abs(m.peakCents - depths[k]) / depths[k] < 0.05, "table depth");
            expect(std::abs(m.rateHz - rates[k]) / rates[k] < 0.05, "table rate");
        }
    }

    // -3 dB frequency of the wet path (no modulation, compander off) against the bandwidth law 0.30 x clock.
    void ensembleBandwidth()
    {
        beginTest(rateTag() + "fx.ensemble: -3 dB bandwidth follows the BBD law (0.30 x clock, within 6 %)");
        struct Case
        {
            int stagesChoice, stages;
            float ms;
        };
        for (const Case c : {Case{0, 256, 6.0f}, Case{1, 512, 10.0f}, Case{2, 1024, 20.0f}})
        {
            Rack h("fx_ensemble_on");
            ensembleBase(h);
            h.set("fx_ensemble_depth", 0.0f);
            h.set("fx_ensemble_compander", 0.0f);
            h.set("fx_ensemble_stages", static_cast<float>(c.stagesChoice));
            h.set("fx_ensemble_delay", c.ms);
            const double clock = ensembleClock(c.stages, 0.001 * c.ms);
            const double expected = Bbd::bandwidthHz(clock);
            const auto level = [&](double hz)
            {
                Pair in = sineBoth(static_cast<size_t>(0.7 * kRate), hz, 0.1);
                Rack g("fx_ensemble_on");
                g.values = h.values;
                const Pair out = runRack(g, std::move(in));
                return toneLevel(out.l, static_cast<size_t>(0.4 * kRate), hz);
            };
            const double ref = level(150.0);
            double prevF = 0.05 * clock, prevG = 0.0, found = 0.0;
            for (double f = 0.1 * clock; f < 0.9 * clock && found == 0.0; f *= 1.04)
            {
                const double g = amplitudeToDb(level(f) / ref);
                if (g < -3.0103)
                    found = prevF + (prevG + 3.0103) / (prevG - g) * (f - prevF);
                prevF = f;
                prevG = g;
            }
            logMessage(String("stages ") + String(c.stages) + " delay " + String(c.ms, 0) + " ms: clock " +
                       String(clock / 1000.0, 2) + " kHz, -3 dB at " + String(found, 0) + " Hz, law " + String(expected, 0) +
                       " Hz (" + String(100.0 * (found / expected - 1.0), 2) + " %)");
            expect(found > 0.0 && std::abs(found / expected - 1.0) < 0.06, "bandwidth within 6 % of the law");
        }
    }

    void ensembleMonoSum()
    {
        beginTest(rateTag() + "fx.ensemble: mono sum within +-1 dB of the channel energy at the default settings");
        for (const bool independent : {false})
        {
            Rack h("fx_ensemble_on");
            const Pair out = runRack(h, noiseBoth(static_cast<size_t>(6.0 * kRate), 0.25, independent));
            const size_t from = 24000;
            std::vector<float> sum(out.l.size());
            for (size_t i = 0; i < sum.size(); ++i)
                sum[i] = 0.5f * (out.l[i] + out.r[i]);
            const double chan = 0.5 * (meanSquare(out.l, from) + meanSquare(out.r, from));
            const double db = powerToDb(meanSquare(sum, from) / chan);
            logMessage(String(independent ? "independent L / R noise" : "mono noise") + ": mono sum " + String(db, 2) +
                       " dB against the channel energy");
            expect(std::abs(db) <= 1.0, "within 1 dB");
        }
    }

    void ensembleNoZipper()
    {
        beginTest(rateTag() + "fx.ensemble: no zipper on rate, depth, delay, mode and line changes (second difference of a "
                              "200 Hz sine below -60 dB)");
        struct Change
        {
            const char* name;
            const char* id;
            float a, b;
        };
        for (const Change c :
             {Change{"rate", "fx_ensemble_rate", 7.0f, 0.1f}, Change{"depth", "fx_ensemble_depth", 30.0f, 2.0f},
              Change{"delay", "fx_ensemble_delay", 14.0f, 9.0f}, Change{"lines", "fx_ensemble_lines", 0.0f, 1.0f},
              Change{"mode", "fx_ensemble_mode", 3.0f, 0.0f}, Change{"stages", "fx_ensemble_stages", 2.0f, 0.0f},
              Change{"shape", "fx_ensemble_shape", 0.0f, 2.0f}})
        {
            Rack h("fx_ensemble_on");
            h.set("fx_ensemble_mix", 0.7f);
            h.set("fx_ensemble_noise", -100.0f);
            h.set("fx_ensemble_delay", 11.0f);
            const Pair out = runRack(h, sineBoth(static_cast<size_t>(5.0 * kRate), 200.0, 0.5),
                                     [&](size_t pos)
                                     {
                                         if (pos == 96000)
                                             h.set(c.id, c.a);
                                         if (pos == 160000)
                                             h.set(c.id, c.b);
                                     });
            const double step =
                std::max(maxSecondDifference(out.l, 40000, 240000), maxSecondDifference(out.r, 40000, 240000));
            logMessage(String(c.name) + " change: largest second difference " + String(step, 6) + " (" +
                       String(amplitudeToDb(step), 1) + " dBFS)");
            expect(step < 0.001, String(c.name) + " change has no step above -60 dB");
        }
    }

    void ensembleBlockSizeZipper()
    {
        beginTest(rateTag() + "fx.ensemble: spread, crossfeed and mix automation leaves no zipper above -90 dB at host "
                              "blocks of 1, 7 and 512");
        struct Control
        {
            const char* name;
            const char* id;
            float lo, hi;
            std::vector<std::pair<const char*, float>> setup;
        };
        const std::vector<Control> controls = {
            {"spread", "fx_ensemble_spread", 0.0f, 1.0f, {{"fx_ensemble_depth", 0.0f}, {"fx_ensemble_mix", 1.0f}}},
            {"crossfeed", "fx_ensemble_crossfeed", -1.0f, 1.0f, {{"fx_ensemble_mode", 1.0f}, {"fx_ensemble_mix", 1.0f}}},
            {"mix", "fx_ensemble_mix", 0.0f, 1.0f, {{"fx_ensemble_depth", 0.0f}}}};
        for (const Control& c : controls)
            for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
            {
                const auto renderAutomated = [&](const Signal& in, const Signal& automation)
                {
                    Rack h("fx_ensemble_on");
                    h.set("fx_ensemble_noise", -100.0f);
                    for (const auto& kv : c.setup)
                        h.set(kv.first, kv.second);
                    std::vector<float> l(in.begin(), in.end()), r = l;
                    // Settle on the starting value so that the device fade-in is over before the test signal's measured
                    // part.
                    for (size_t pos = 0; pos < l.size(); pos += block)
                    {
                        const size_t end = std::min(pos + block, l.size());
                        h.set(c.id, static_cast<float>(c.lo + (c.hi - c.lo) * automation[end - 1]));
                        h.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
                    }
                    return toSignal(l);
                };
                const ZipperResult z = zipperTest(renderAutomated, kRate, 1000.0, -6.0);
                // The devices leave a small residue of their own (clock ripple): the same estimator on a held setting is the
                // floor.
                const size_t held = static_cast<size_t>(0.6 * kRate);
                const Signal hold = renderAutomated(sineWave(held, 1000.0, kRate, dbToAmplitude(-6.0)), Signal(held, 1.0));
                const double floorDb = clickTest(hold, 1000.0, kRate, static_cast<size_t>(0.1 * kRate)).stepDb;
                const double limit = std::max(-90.0, floorDb + 3.0);
                logMessage(String("fx.ensemble ") + c.name + ", host block " + String(static_cast<int>(block)) +
                           ": ramp step " + String(z.rampDb, 1) + " dB, 2 Hz square " + String(z.squareDb, 1) +
                           " dB, held-setting floor " + String(floorDb, 1) + " dB");
                expect(z.rampDb <= limit, String(c.name) + " ramp zipper at block " + String(static_cast<int>(block)));
                expect(z.squareDb <= limit, String(c.name) + " square zipper at block " + String(static_cast<int>(block)));
            }

        // A jump of each control at a sample where every block size starts a block (3584 = 7 x 512) gives the same output
        // within 0.005 (the gains match exactly; the oscillator of the Dimensional modes still updates the delay once per
        // chunk of up to 128 samples).
        for (const Control& c : controls)
        {
            const size_t total = 24000, jumpAt = 3584 * 3;
            const Pair in = sineBoth(total, 1000.0, 0.5);
            std::vector<float> first;
            for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
            {
                Rack h("fx_ensemble_on");
                h.set("fx_ensemble_noise", -100.0f);
                for (const auto& kv : c.setup)
                    h.set(kv.first, kv.second);
                Pair io = in;
                for (size_t pos = 0; pos < total; pos += block)
                {
                    const size_t end = std::min(pos + block, total);
                    h.set(c.id, end - 1 >= jumpAt ? c.hi : c.lo);
                    h.run(io.l.data() + pos, io.r.data() + pos, static_cast<int>(end - pos));
                }
                if (first.empty())
                {
                    first = io.l;
                    continue;
                }
                double worst = 0.0;
                for (size_t i = 0; i < total; ++i)
                    worst = std::max(worst, static_cast<double>(std::abs(io.l[i] - first[i])));
                logMessage(String("fx.ensemble ") + c.name + " jump, block " + String(static_cast<int>(block)) +
                           " against block 1: largest difference " + String(worst, 6));
                expect(worst < 0.005, String(c.name) + " jump behaves alike at block " + String(static_cast<int>(block)));
            }
        }
    }

    void ensembleAllocations()
    {
        beginTest(rateTag() + "fx.ensemble: no allocation on the audio thread with every parameter changing every block");
        Rack h("fx_ensemble_on");
        const char* ids[] = {"mix",   "mode",   "lines",  "shape",     "rate",  "depth",
                             "delay", "stages", "spread", "crossfeed", "noise", "compander"};
        std::vector<size_t> index;
        for (const char* id : ids)
            index.push_back(static_cast<size_t>(Rack::ix(std::string("fx_ensemble_") + id)));
        const size_t onIndex = static_cast<size_t>(Rack::ix("fx_ensemble_on"));
        Random rng(3);
        std::vector<float> l(64), r(64);
        for (int warm = 0; warm < 8; ++warm)
            h.run(l.data(), r.data(), 64);
        crucible::testing::gAllocations = 0;
        crucible::testing::gTrackAllocations = true;
        for (int block = 0; block < 3000; ++block)
        {
            for (size_t i = 0; i < 64; ++i)
            {
                l[i] = static_cast<float>(0.3 * rng.gaussian());
                r[i] = static_cast<float>(0.3 * rng.gaussian());
            }
            auto& v = h.values;
            v[index[0]] = static_cast<float>(block % 11) / 10.0f;
            v[index[1]] = static_cast<float>((block / 40) % 5);
            v[index[2]] = static_cast<float>((block / 25) % 3);
            v[index[3]] = static_cast<float>((block / 60) % 3);
            v[index[4]] = 0.05f + static_cast<float>(block % 17) * 0.4f;
            v[index[5]] = static_cast<float>(block % 31);
            v[index[6]] = 3.0f + static_cast<float>(block % 37);
            v[index[7]] = static_cast<float>((block / 90) % 3);
            v[index[8]] = static_cast<float>(block % 5) / 4.0f;
            v[index[9]] = -1.0f + static_cast<float>(block % 9) * 0.25f;
            v[index[10]] = -100.0f + static_cast<float>(block % 61);
            v[index[11]] = static_cast<float>(block % 2);
            v[onIndex] = block % 700 > 680 ? 0.0f : 1.0f;
            h.run(l.data(), r.data(), 64);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
    }

    void ensembleCpu()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest(rateTag() + "fx.ensemble: CPU, percent of one core per stereo instance at 48 kHz (plan: 0.5 %)");
        const std::vector<std::pair<std::string, float>> defaults = {{"fx_ensemble_mix", 0.5f}};
        const std::vector<std::pair<std::string, float>> dimensional = {{"fx_ensemble_mix", 0.5f},
                                                                        {"fx_ensemble_mode", 3.0f}};
        const std::vector<std::pair<std::string, float>> noisy = {{"fx_ensemble_mix", 0.5f},
                                                                  {"fx_ensemble_shape", 2.0f},
                                                                  {"fx_ensemble_noise", -60.0f},
                                                                  {"fx_ensemble_depth", 20.0f},
                                                                  {"fx_ensemble_delay", 12.0f}};
        const std::vector<std::pair<std::string, float>> bright = {
            {"fx_ensemble_mix", 0.5f}, {"fx_ensemble_stages", 1.0f}, {"fx_ensemble_delay", 10.0f}};
        const std::vector<std::pair<std::string, float>> brightest = {
            {"fx_ensemble_mix", 0.5f}, {"fx_ensemble_stages", 2.0f}, {"fx_ensemble_delay", 20.0f}};
        const double d = cpuPercent("fx_ensemble_on", defaults), dm = cpuPercent("fx_ensemble_on", dimensional),
                     nz = cpuPercent("fx_ensemble_on", noisy), br = cpuPercent("fx_ensemble_on", bright),
                     bb = cpuPercent("fx_ensemble_on", brightest);
        logMessage(String("CPU ensemble: defaults (3 lines, 256 stages, 8 ms) ") + String(d, 3) + " %, Dimensional III " +
                   String(dm, 3) + " %, smooth random with noise and compander " + String(nz, 3) + " %");
        logMessage(String("CPU ensemble, brighter voicings (information; the clock sets the cost): 512 stages 10 ms ") +
                   String(br, 3) + " %, 1024 stages 20 ms " + String(bb, 3) + " %");
        expectCpu(std::max({d, dm, nz}) <= 0.5 * 1.2,
                  "defaults, Dimensional and the noisy voicing within 0.5 % (20 % measurement margin)");
#endif
    }

    // ---- fx.phaser2

    static void phaserBase(Rack& h, int stagesChoice, double centerHz)
    {
        h.set("fx_phaser2_mix", 0.5f);
        h.set("fx_phaser2_stages", static_cast<float>(stagesChoice));
        h.set("fx_phaser2_center", static_cast<float>(centerHz));
        h.set("fx_phaser2_range", 0.0f);
        h.set("fx_phaser2_feedback", 0.0f);
        h.set("fx_phaser2_type", 0.0f);
    }

    void phaserRegistered()
    {
        beginTest(rateTag() + "fx.phaser2: registered after fx.ensemble, parameters appended after it");
        const auto effects = ModuleRegistry::instance().ofKind(ModuleKind::Effect);
        size_t at = 0;
        while (at < effects.size() && effects[at]->id != "fx.phaser2")
            ++at;
        expect(at > 0 && at < effects.size() && effects[at - 1]->id == "fx.ensemble",
               "directly after fx.ensemble in step order");
        expect(ParameterMap::indexOf("fx_pos_ensemble") + 1 == ParameterMap::indexOf("fx_phaser2_on"),
               "parameters directly after fx.ensemble");
        int count = 0;
        for (const auto& pd : effects[at]->params)
            ++count, expect(pd.id.rfind("fx_phaser2_", 0) == 0 || pd.id == "fx_pos_phaser2", "id " + pd.id);
        logMessage(String("fx.phaser2 parameters: ") + String(count));
    }

    void phaserNotches()
    {
        beginTest(rateTag() + "fx.phaser2: notches at the analytic all-pass prediction within 1 %, depth at least 40 dB at "
                              "mix 0.5 (linear, no feedback)");
        const int counts[] = {2, 4, 6, 8, 12};
        double worst = 0.0, shallow = 1000.0;
        int notches = 0, measured = 0;
        for (int sc = 0; sc < 5; ++sc)
            for (const double fc : {300.0, 900.0, 2500.0})
            {
                Rack h("fx_phaser2_on");
                phaserBase(h, sc, fc);
                const Pair in = noiseBoth(scaled(65536 + 8192), 0.2, false, 21 + static_cast<std::uint64_t>(sc));
                const Pair out = runRack(h, in);
                for (int k = 0; k < counts[sc] / 2; ++k)
                {
                    const double want = allpassNotch(counts[sc], k, fc);
                    // Depth: a tone at the predicted frequency through the same settings.
                    Rack t("fx_phaser2_on");
                    t.values = h.values;
                    const Pair tone = runRack(t, sineBoth(static_cast<size_t>(2.0 * kRate), want, 0.2));
                    const double depth = amplitudeToDb(toneLevel(tone.l, static_cast<size_t>(kRate), want) / 0.2);
                    shallow = std::min(shallow, -depth);
                    ++notches;
                    expect(-depth >= 40.0, "notch at least 40 dB deep at the predicted frequency");
                    // Frequency: the noise-probe transfer function (a notch below 100 Hz is narrower than the probe's
                    // resolution: depth only).
                    if (want >= 100.0)
                    {
                        const double got = findNotch(in.l, out.l, scaled(8192), scaled(65536), want, 0.03);
                        worst = std::max(worst, std::abs(got / want - 1.0));
                        ++measured;
                        expect(std::abs(got / want - 1.0) < 0.01,
                               "notch within 1 % of the prediction (" + std::to_string(counts[sc]) + " stages, break " +
                                   std::to_string(fc) + " Hz, k " + std::to_string(k) + ": predicted " +
                                   std::to_string(want) + " Hz, measured " + std::to_string(got) + " Hz)");
                    }
                }
            }
        logMessage(String("fx.phaser2: ") + String(notches) +
                   " notches (2-12 stages, break 300 / 900 / 2500 Hz), worst frequency error " + String(100.0 * worst, 3) +
                   " % over the " + String(measured) + " above 100 Hz (target 1 %), shallowest notch " + String(shallow, 1) +
                   " dB (target 40 dB)");
    }

    // The notch k with tan((2k + 1) pi / 2N) = 1 sits at the break frequency itself (k = 0 for two stages, k = 1 for six);
    // others are read from the analytic prediction. The sweep runs at 0.1 Hz and its extremes are read at the peak (2.5 s)
    // and the trough (7.5 s).
    void phaserSweepRange()
    {
        beginTest(rateTag() + "fx.phaser2: sweep range (ratio of the notch at the top and the bottom of the sweep) within 2 "
                              "% of 2^(2 x range)");
        struct Case
        {
            int stagesChoice, stages, k;
            double range, center;
        };
        double worst = 0.0;
        for (const Case c : {Case{0, 2, 0, 1.0, 1000.0}, Case{2, 6, 1, 1.5, 800.0}, Case{1, 4, 1, 0.6, 1500.0}})
        {
            Rack h("fx_phaser2_on");
            phaserBase(h, c.stagesChoice, c.center);
            h.set("fx_phaser2_range", static_cast<float>(c.range));
            h.set("fx_phaser2_rate", 0.1f);
            h.set("fx_phaser2_shape", 0.0f);
            const Pair in = noiseBoth(static_cast<size_t>(8.2 * kRate), 0.2, false, 5);
            const Pair out = runRack(h, in);
            const size_t len = scaled(8192);
            const size_t top = static_cast<size_t>(2.5 * kRate) - len / 2,
                         bottom = static_cast<size_t>(7.5 * kRate) - len / 2;
            const double wantTop = allpassNotch(c.stages, c.k, c.center * std::exp2(c.range)),
                         wantBottom = allpassNotch(c.stages, c.k, c.center * std::exp2(-c.range));
            const double gotTop = findNotch(in.l, out.l, top, len, wantTop, 0.12),
                         gotBottom = findNotch(in.l, out.l, bottom, len, wantBottom, 0.12);
            const double ratio = gotTop / gotBottom, want = wantTop / wantBottom;
            worst = std::max(worst, std::abs(ratio / want - 1.0));
            logMessage(String(c.stages) + " stages, center " + String(c.center, 0) + " Hz, range " + String(c.range, 1) +
                       " oct: notch " + String(gotBottom, 1) + " .. " + String(gotTop, 1) + " Hz (predicted " +
                       String(wantBottom, 1) + " .. " + String(wantTop, 1) + "), ratio error " +
                       String(100.0 * (ratio / want - 1.0), 3) + " %");
            expect(std::abs(ratio / want - 1.0) < 0.02, "sweep range within 2 %");
        }
        logMessage(String("fx.phaser2 worst sweep range error ") + String(100.0 * worst, 2) + " % (target 2 %)");
    }

    void phaserStageTypes()
    {
        beginTest(rateTag() + "fx.phaser2: OTA and JFET stages keep the linear small-signal response and add odd (OTA) or "
                              "even (JFET) harmonics");
        const auto tone = [](int type, double amp, double db)
        {
            Rack h("fx_phaser2_on");
            phaserBase(h, 1, 800.0);
            h.set("fx_phaser2_mix", 1.0f);
            h.set("fx_phaser2_type", static_cast<float>(type));
            h.set("fx_phaser2_drive", static_cast<float>(db));
            const Pair out = runRack(h, sineBoth(static_cast<size_t>(1.0 * kRate), 1000.0, amp));
            return out.l;
        };
        const auto level = [](const std::vector<float>& x, double hz)
        { return toneLevel(x, static_cast<size_t>(kRate / 2.0), hz); };
        const auto lin = tone(0, 0.001, 6.0), ota = tone(1, 0.001, 6.0), jfet = tone(2, 0.001, 6.0);
        const double errOta = std::abs(amplitudeToDb(level(ota, 1000.0) / level(lin, 1000.0))),
                     errJfet = std::abs(amplitudeToDb(level(jfet, 1000.0) / level(lin, 1000.0)));
        logMessage(String("small signal (0.001): OTA ") + String(errOta, 3) + " dB, JFET " + String(errJfet, 3) +
                   " dB from the linear stage");
        expect(errOta < 0.1 && errJfet < 0.1, "small-signal response equals the linear stage");
        const auto hl = tone(0, 0.7, 18.0), ho = tone(1, 0.7, 18.0), hj = tone(2, 0.7, 18.0);
        const auto h2 = [&](const std::vector<float>& x) { return amplitudeToDb(level(x, 2000.0) / level(x, 1000.0)); };
        const auto h3 = [&](const std::vector<float>& x) { return amplitudeToDb(level(x, 3000.0) / level(x, 1000.0)); };
        logMessage(String("0.7 amplitude, drive 18 dB: H2 / H3 linear ") + String(h2(hl), 1) + " / " + String(h3(hl), 1) +
                   " dB, OTA " + String(h2(ho), 1) + " / " + String(h3(ho), 1) + " dB, JFET " + String(h2(hj), 1) + " / " +
                   String(h3(hj), 1) + " dB");
        expect(h2(hl) < -100.0 && h3(hl) < -100.0, "the linear stage adds no harmonics");
        expect(h3(ho) > -60.0 && h3(ho) > h2(ho) + 30.0, "OTA: odd harmonics dominate");
        expect(h2(hj) > -60.0 && h2(hj) > h2(ho) + 30.0, "JFET: even harmonics");
    }

    void phaserFeedbackBounded()
    {
        beginTest(rateTag() +
                  "fx.phaser2: feedback +-0.95 is bounded with and without saturation, on every stage count and type");
        double worst = 0.0;
        for (const float fb : {0.95f, -0.95f})
            for (const float sat : {0.0f, 1.0f})
                for (const int type : {0, 1, 2})
                {
                    Rack h("fx_phaser2_on");
                    phaserBase(h, 4, 700.0);
                    h.set("fx_phaser2_mix", 1.0f);
                    h.set("fx_phaser2_feedback", fb);
                    h.set("fx_phaser2_fb_sat", sat);
                    h.set("fx_phaser2_type", static_cast<float>(type));
                    h.set("fx_phaser2_range", 1.5f);
                    h.set("fx_phaser2_rate", 3.0f);
                    const Pair out = runRack(h, noiseBoth(static_cast<size_t>(6.0 * kRate), 0.25, false, 8));
                    double peak = 0.0;
                    bool finite = true;
                    for (const float v : out.l)
                    {
                        finite = finite && std::isfinite(v);
                        peak = std::max(peak, static_cast<double>(std::abs(v)));
                    }
                    worst = std::max(worst, peak);
                    expect(finite && peak < 30.0, "bounded");
                }
        logMessage(String("fx.phaser2: largest peak with feedback 0.95 / -0.95 on 0.25 rms noise: ") + String(worst, 2));
    }

    void phaserExtremesBounded()
    {
        beginTest(rateTag() + "fx.phaser2: output finite and bounded at feedback +-0.95 with feedback saturation and drive "
                              "at their maxima (12 stages, every type, barber-pole and source)");
        const size_t n = static_cast<size_t>(10.0 * kRate);
        const Pair noise = noiseBoth(n, 0.25, false, 9);
        for (const int type : {0, 1, 2})
            for (const int barber : {0, 1, 2})
                for (const int src : {0, 1, 2})
                    for (const float fb : {0.95f, -0.95f})
                        for (const float sat : {0.0f, 1.0f})
                            for (const float drive : {6.0f, 24.0f})
                            {
                                Rack h("fx_phaser2_on");
                                phaserBase(h, 4, 700.0);
                                h.set("fx_phaser2_mix", 1.0f);
                                h.set("fx_phaser2_feedback", fb);
                                h.set("fx_phaser2_fb_sat", sat);
                                h.set("fx_phaser2_drive", drive);
                                h.set("fx_phaser2_type", static_cast<float>(type));
                                h.set("fx_phaser2_barber", static_cast<float>(barber));
                                h.set("fx_phaser2_source", static_cast<float>(src));
                                h.set("fx_phaser2_env_sens", 40.0f);
                                h.set("fx_phaser2_spread", 1.0f);
                                h.set("fx_phaser2_range", 3.0f);
                                h.set("fx_phaser2_rate", 10.0f);
                                h.set("fx_phaser2_center", 6000.0f);
                                h.set("fx_phaser2_shape", 1.0f);
                                h.set("fx_phaser2_env_attack", 200.0f);
                                h.set("fx_phaser2_env_release", 2000.0f);
                                const Pair out = runRack(h, noise);
                                double peak = 0.0;
                                bool finite = true;
                                for (const float v : out.l)
                                {
                                    finite = finite && std::isfinite(v);
                                    peak = std::max(peak, static_cast<double>(std::abs(v)));
                                }
                                expect(finite && peak < 100.0, "type " + std::to_string(type) + " barber " +
                                                                   std::to_string(barber) + " source " +
                                                                   std::to_string(src) + " fb " + std::to_string(fb) +
                                                                   " sat " + std::to_string(sat) + " drive " +
                                                                   std::to_string(drive) + " peak " + std::to_string(peak));
                            }
    }

    void phaserFeedbackBoundedMatrix()
    {
        beginTest(
            rateTag() +
            "fx.phaser2: output finite and within the linear loop bound (peak of the input / (1 - |feedback|)) for every "
            "stage count, type and barber-pole state, feedback +-0.95, saturation 0 (30 s of noise and of a swept sine)");
        const size_t n = static_cast<size_t>(30.0 * kRate);
        const Pair noise = noiseBoth(n, 0.25, false, 8);
        Pair sweep{std::vector<float>(n), std::vector<float>(n)};
        {
            double phase = 0.0;
            const double lo = 20.0, hi = 18000.0, total = static_cast<double>(n);
            for (size_t i = 0; i < n; ++i)
            {
                const double hz = lo * std::pow(hi / lo, static_cast<double>(i) / total);
                phase += 2.0 * kPi * hz / kRate;
                sweep.l[i] = sweep.r[i] = static_cast<float>(0.25 * std::sin(phase));
            }
        }
        double worst = 0.0;
        for (const int stages : {0, 1, 2, 3, 4})
            for (const int type : {0, 1, 2})
                for (const int barber : {0, 1})
                    for (const float fb : {0.95f, -0.95f})
                        for (const int src : {0, 1})
                        {
                            Rack h("fx_phaser2_on");
                            phaserBase(h, stages, 700.0);
                            h.set("fx_phaser2_mix", 1.0f);
                            h.set("fx_phaser2_feedback", fb);
                            h.set("fx_phaser2_fb_sat", 0.0f);
                            h.set("fx_phaser2_type", static_cast<float>(type));
                            h.set("fx_phaser2_barber", static_cast<float>(barber));
                            h.set("fx_phaser2_range", 1.5f);
                            h.set("fx_phaser2_rate", 3.0f);
                            const Pair out = runRack(h, src == 0 ? noise : sweep);
                            double peak = 0.0;
                            bool finite = true;
                            for (const float v : out.l)
                            {
                                finite = finite && std::isfinite(v);
                                peak = std::max(peak, static_cast<double>(std::abs(v)));
                            }
                            worst = std::max(worst, peak);
                            double inPeak = 0.0;
                            for (const float v : (src == 0 ? noise : sweep).l)
                                inPeak = std::max(inPeak, static_cast<double>(std::abs(v)));
                            const double bound = 1.05 * inPeak / (1.0 - std::abs(static_cast<double>(fb)));
                            expect(finite && peak < bound, "stages " + std::to_string(stages) + " type " +
                                                               std::to_string(type) + " barber " + std::to_string(barber) +
                                                               " fb " + std::to_string(fb) +
                                                               (src == 0 ? " noise" : " sweep") + " peak " +
                                                               std::to_string(peak) + " bound " + std::to_string(bound));
                        }
        logMessage(String("fx.phaser2: largest peak over the stage / type / barber-pole matrix: ") + String(worst, 2));
    }

    void phaserModulationSources()
    {
        beginTest(rateTag() + "fx.phaser2: the envelope and the sidechain sources move the notch up with level");
        // Input-driven envelope: the noise level sets the notch.
        const auto levelNotch = [&](float rms)
        {
            Rack h("fx_phaser2_on");
            phaserBase(h, 0, 1000.0);
            h.set("fx_phaser2_range", 1.0f);
            h.set("fx_phaser2_source", 1.0f);
            h.set("fx_phaser2_env_attack", 2.0f);
            h.set("fx_phaser2_env_release", 50.0f);
            const Pair in = noiseBoth(static_cast<size_t>(1.2 * kRate), rms, false, 9);
            const Pair out = runRack(h, in);
            return findNotch(in.l, out.l, static_cast<size_t>(0.9 * kRate), 4096, 1000.0, 0.6);
        };
        const double quiet = levelNotch(0.002), loud = levelNotch(0.5);
        logMessage(String("envelope source: notch ") + String(quiet, 0) + " Hz at 0.002 rms, " + String(loud, 0) +
                   " Hz at 0.5 rms");
        expect(loud > 1.8 * quiet, "a louder input moves the notch up");
        // Sidechain: a quiet input, the key decides.
        const auto keyNotch = [&](float keyAmp)
        {
            Rack h("fx_phaser2_on");
            phaserBase(h, 0, 1000.0);
            h.set("fx_phaser2_range", 1.0f);
            h.set("fx_phaser2_source", 2.0f);
            h.set("fx_phaser2_env_attack", 2.0f);
            h.set("fx_phaser2_env_release", 50.0f);
            const size_t n = static_cast<size_t>(1.2 * kRate);
            const Pair in = noiseBoth(n, 0.05, false, 9);
            Pair out = in;
            std::vector<float> key(n);
            for (size_t i = 0; i < n; ++i)
                key[i] = static_cast<float>(keyAmp * std::sin(2.0 * kPi * 100.0 * static_cast<double>(i) / kRate));
            for (size_t pos = 0; pos < n; pos += 128)
                h.runAux(out.l.data() + pos, out.r.data() + pos, static_cast<int>(std::min<size_t>(128, n - pos)),
                         key.data() + pos, key.data() + pos);
            return findNotch(in.l, out.l, static_cast<size_t>(0.9 * kRate), 4096, 1000.0, 0.6);
        };
        const double kq = keyNotch(0.002), kl = keyNotch(0.5);
        logMessage(String("sidechain source: notch ") + String(kq, 0) + " Hz at key 0.002, " + String(kl, 0) +
                   " Hz at key 0.5");
        expect(kl > 1.8 * kq, "a louder key moves the notch up");
    }

    // |H| in dB on a logarithmic grid (200 Hz to 6 kHz, 120 points) from a noise probe, over a window of 2048 samples.
    static std::vector<double> responseGrid(const std::vector<float>& x, const std::vector<float>& y, size_t from)
    {
        std::vector<double> a;
        for (int i = 0; i < 120; ++i)
            a.push_back(std::max(-20.0, amplitudeToDb(transferAt(x, y, from, 2048, 200.0 * std::pow(30.0, i / 119.0)))));
        return a;
    }

    // Shift in grid cells of the later response against the earlier one that fits best (positive: features moved up).
    static int bestShift(const std::vector<double>& early, const std::vector<double>& late)
    {
        int best = 0;
        double bestErr = 1.0e30;
        for (int s = -8; s <= 8; ++s)
        {
            double err = 0.0;
            for (int i = 10; i < 110; ++i)
                err += (early[static_cast<size_t>(i)] - late[static_cast<size_t>(i + s)]) *
                       (early[static_cast<size_t>(i)] - late[static_cast<size_t>(i + s)]);
            if (err < bestErr)
            {
                bestErr = err;
                best = s;
            }
        }
        return best;
    }

    void phaserBarberPole()
    {
        beginTest(rateTag() + "fx.phaser2: barber-pole notches travel one way for ever (direction, periodicity of the stage "
                              "conveyor, no step)");
        for (const int dir : {1, 2})
        {
            Rack h("fx_phaser2_on");
            phaserBase(h, 1, 800.0);
            h.set("fx_phaser2_range", 2.0f);
            h.set("fx_phaser2_rate", 1.0f);
            h.set("fx_phaser2_spread", 0.0f);
            h.set("fx_phaser2_barber", static_cast<float>(dir));
            const Pair sine = runRack(h, sineBoth(static_cast<size_t>(8.0 * kRate), 200.0, 0.3));
            const double step = maxSecondDifference(sine.l, 24000, sine.l.size());
            logMessage(String(dir == 1 ? "up" : "down") + ": largest second difference " + String(step, 6) + " (" +
                       String(amplitudeToDb(step), 1) + " dBFS)");
            expect(step < 0.001, "no step above -60 dB");
            // Noise probe: the response at t, t + 50 ms, and one conveyor period later (two pairs at 1 Hz: 1 / 2 s).
            Rack g("fx_phaser2_on");
            g.values = h.values;
            const Pair noise = noiseBoth(static_cast<size_t>(3.0 * kRate), 0.2, false, 4);
            const Pair res = runRack(g, noise);
            const size_t t0 = static_cast<size_t>(1.5 * kRate);
            const size_t period = static_cast<size_t>(kRate / 2.0);
            const auto a0 = responseGrid(noise.l, res.l, t0), a1 = responseGrid(noise.l, res.l, t0 + 2400),
                       a2 = responseGrid(noise.l, res.l, t0 + period);
            const int shift = bestShift(a0, a1);
            double periodErr = 0.0, halfErr = 0.0;
            const auto a3 = responseGrid(noise.l, res.l, t0 + period / 2);
            for (size_t i = 10; i < 110; ++i)
            {
                periodErr += std::abs(a0[i] - a2[i]) / 100.0;
                halfErr += std::abs(a0[i] - a3[i]) / 100.0;
            }
            logMessage(String("response moved ") + String(shift) +
                       " grid cells (0.041 octave each) in 50 ms (expected about 5); mean dB difference (clipped at -20 dB) "
                       "after one conveyor period " +
                       String(periodErr, 2) + ", after half a period " + String(halfErr, 2));
            if (dir == 1)
                expect(shift >= 3 && shift <= 7, "about 0.2 octave in 50 ms");
            expect(dir == 1 ? shift > 0 : shift < 0, "the notches move in the set direction");
            expect(periodErr < 1.2, "the response repeats after one pair spacing");
            expect(halfErr > 2.0 * periodErr, "and differs half-way");
        }
    }

    void phaserNoZipper()
    {
        beginTest(rateTag() + "fx.phaser2: no zipper on center, range, rate, feedback, drive, spread, mix, stage count, "
                              "type, source and barber-pole changes");
        struct Change
        {
            const char* name;
            const char* id;
            float a, b;
            int type; // stage type of the base setting
        };
        for (const Change c :
             {Change{"center", "fx_phaser2_center", 4000.0f, 150.0f, 0}, Change{"range", "fx_phaser2_range", 3.0f, 0.2f, 0},
              Change{"rate", "fx_phaser2_rate", 9.0f, 0.05f, 0}, Change{"feedback", "fx_phaser2_feedback", 0.7f, -0.7f, 0},
              Change{"mix", "fx_phaser2_mix", 1.0f, 0.1f, 0}, Change{"spread", "fx_phaser2_spread", 1.0f, 0.0f, 0},
              Change{"drive", "fx_phaser2_drive", 24.0f, 0.0f, 1}, Change{"stages", "fx_phaser2_stages", 4.0f, 0.0f, 0},
              Change{"type", "fx_phaser2_type", 2.0f, 1.0f, 0}, Change{"source", "fx_phaser2_source", 1.0f, 0.0f, 0},
              Change{"barber", "fx_phaser2_barber", 1.0f, 0.0f, 0}, Change{"shape", "fx_phaser2_shape", 1.0f, 0.0f, 0}})
        {
            const auto render = [&](bool automate, float fixed)
            {
                Rack h("fx_phaser2_on");
                h.set("fx_phaser2_mix", 0.6f);
                h.set("fx_phaser2_type", static_cast<float>(c.type));
                h.set("fx_phaser2_feedback", 0.3f);
                if (!automate)
                    h.set(c.id, fixed);
                const Pair out = runRack(h, sineBoth(static_cast<size_t>(5.0 * kRate), 200.0, 0.3),
                                         [&](size_t pos)
                                         {
                                             if (automate && pos == 96000)
                                                 h.set(c.id, c.a);
                                             if (automate && pos == 160000)
                                                 h.set(c.id, c.b);
                                         });
                return std::max(maxSecondDifference(out.l, 40000, 240000), maxSecondDifference(out.r, 40000, 240000));
            };
            // The end states themselves set what a smooth signal looks like: a change must not exceed them (and never -60
            // dBFS outright).
            const double step = render(true, 0.0f), ref = std::max(render(false, c.a), render(false, c.b));
            logMessage(String(c.name) + " change: largest second difference " + String(step, 6) + " (" +
                       String(amplitudeToDb(step), 1) + " dBFS), end states " + String(ref, 6));
            expect(step < std::max(0.001, 1.25 * ref),
                   String(c.name) + " change has no step above -60 dB (or above the end states)");
        }
    }

    void phaserAllocations()
    {
        beginTest(rateTag() + "fx.phaser2: no allocation on the audio thread with every parameter changing every block");
        Rack h("fx_phaser2_on");
        const char* ids[] = {"mix",    "stages", "center", "range",  "rate",     "shape",      "feedback",    "fb_sat",
                             "spread", "type",   "drive",  "source", "env_sens", "env_attack", "env_release", "barber"};
        std::vector<size_t> index;
        for (const char* id : ids)
            index.push_back(static_cast<size_t>(Rack::ix(std::string("fx_phaser2_") + id)));
        const size_t onIndex = static_cast<size_t>(Rack::ix("fx_phaser2_on"));
        Random rng(3);
        std::vector<float> l(64), r(64);
        for (int warm = 0; warm < 8; ++warm)
            h.run(l.data(), r.data(), 64);
        crucible::testing::gAllocations = 0;
        crucible::testing::gTrackAllocations = true;
        for (int block = 0; block < 3000; ++block)
        {
            for (size_t i = 0; i < 64; ++i)
            {
                l[i] = static_cast<float>(0.3 * rng.gaussian());
                r[i] = static_cast<float>(0.3 * rng.gaussian());
            }
            auto& v = h.values;
            v[index[0]] = static_cast<float>(block % 11) / 10.0f;
            v[index[1]] = static_cast<float>((block / 30) % 5);
            v[index[2]] = 100.0f + static_cast<float>(block % 37) * 100.0f;
            v[index[3]] = static_cast<float>(block % 13) * 0.25f;
            v[index[4]] = 0.05f + static_cast<float>(block % 17) * 0.5f;
            v[index[5]] = static_cast<float>((block / 50) % 2);
            v[index[6]] = -0.95f + static_cast<float>(block % 20) * 0.1f;
            v[index[7]] = static_cast<float>(block % 5) / 4.0f;
            v[index[8]] = static_cast<float>(block % 7) / 6.0f;
            v[index[9]] = static_cast<float>((block / 20) % 3);
            v[index[10]] = static_cast<float>(block % 25);
            v[index[11]] = static_cast<float>((block / 70) % 3);
            v[index[12]] = -40.0f + static_cast<float>(block % 81);
            v[index[13]] = 0.5f + static_cast<float>(block % 40);
            v[index[14]] = 5.0f + static_cast<float>(block % 100);
            v[index[15]] = static_cast<float>((block / 45) % 3);
            v[onIndex] = block % 700 > 680 ? 0.0f : 1.0f;
            h.run(l.data(), r.data(), 64);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
    }

    void phaserCpu()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest(rateTag() + "fx.phaser2: CPU, percent of one core per stereo instance at 48 kHz (plan: 0.3 %)");
        const std::vector<std::pair<std::string, float>> defaults = {{"fx_phaser2_mix", 0.5f}};
        const std::vector<std::pair<std::string, float>> six = {
            {"fx_phaser2_mix", 0.5f}, {"fx_phaser2_stages", 2.0f}, {"fx_phaser2_feedback", 0.6f}};
        const std::vector<std::pair<std::string, float>> twelve = {
            {"fx_phaser2_mix", 0.5f}, {"fx_phaser2_stages", 4.0f}, {"fx_phaser2_feedback", 0.6f}};
        const std::vector<std::pair<std::string, float>> ota = {{"fx_phaser2_mix", 0.5f}, {"fx_phaser2_type", 1.0f}};
        const std::vector<std::pair<std::string, float>> barber = {
            {"fx_phaser2_mix", 0.5f}, {"fx_phaser2_stages", 3.0f}, {"fx_phaser2_barber", 1.0f}};
        const double d = cpuPercent("fx_phaser2_on", defaults), s6 = cpuPercent("fx_phaser2_on", six),
                     s12 = cpuPercent("fx_phaser2_on", twelve), o = cpuPercent("fx_phaser2_on", ota),
                     bp = cpuPercent("fx_phaser2_on", barber);
        logMessage(String("CPU phaser2: defaults (4 stages, linear) ") + String(d, 3) + " %, 6 stages with feedback " +
                   String(s6, 3) + " %, 12 stages with feedback " + String(s12, 3) + " %, 4 stages OTA " + String(o, 3) +
                   " %, 8 stages barber-pole " + String(bp, 3) + " %");
        expectCpu(std::max({d, s6, s12}) <= 0.3 * 1.2,
                  "linear phasers up to 12 stages within 0.3 % (20 % measurement margin)");
#endif
    }

    // ---- fx.flanger2

    static void flangerBase(Rack& h, double manualMs)
    {
        h.set("fx_flanger2_mix", 0.5f);
        h.set("fx_flanger2_mode", 0.0f);
        h.set("fx_flanger2_source", 0.0f);
        h.set("fx_flanger2_manual", static_cast<float>(manualMs));
        h.set("fx_flanger2_depth", 0.0f);
        h.set("fx_flanger2_feedback", 0.0f);
        h.set("fx_flanger2_fb_sat", 0.0f);
    }

    static int flangerLatencyOf(Rack& h)
    {
        auto* lat = h.rack.findCapability<LatencyReporter>("fx.flanger2");
        return lat != nullptr ? lat->latencySamples() : -1;
    }

    static std::vector<float> dropFront(const std::vector<float>& x, int n)
    {
        return std::vector<float>(x.begin() + n, x.end());
    }

    void flangerRegistered()
    {
        beginTest(rateTag() + "fx.flanger2: registered after fx.phaser2, parameters appended after it");
        const auto effects = ModuleRegistry::instance().ofKind(ModuleKind::Effect);
        expect(effects.size() >= 7 && effects[effects.size() - 6]->id == "fx.flanger2" &&
                   effects[effects.size() - 5]->id == "fx.exciter",
               "sixth to last effect (fx.exciter, fx.saturator, fx.shaper, fx.multiband_drive and fx.cabinet follow)");
        expect(effects.size() >= 7 && effects[effects.size() - 7]->id == "fx.phaser2",
               "directly after fx.phaser2 in step order");
        expect(ParameterMap::indexOf("fx_pos_phaser2") + 1 == ParameterMap::indexOf("fx_flanger2_on"),
               "parameters directly after fx.phaser2");
        int count = 0;
        for (const auto& pd : effects[effects.size() - 6]->params)
            ++count, expect(pd.id.rfind("fx_flanger2_", 0) == 0 || pd.id == "fx_pos_flanger2", "id " + pd.id);
        logMessage(String("fx.flanger2 parameters: ") + String(count));
    }

    void flangerLatency()
    {
        beginTest(rateTag() +
                  "fx.flanger2: the reported latency is the measured delay of the reference path (to one sample)");
        Rack h("fx_flanger2_on");
        flangerBase(h, 1.0);
        h.set("fx_flanger2_mix", 0.0f);
        const Pair in = noiseBoth(32768, 0.2, false, 3);
        const Pair out = runRack(h, in);
        const double measured = measureLatency(toSignal(in.l), toSignal(out.l));
        const int reported = flangerLatencyOf(h);
        logMessage(String("fx.flanger2 latency: reported ") + String(reported) + " samples (" +
                   String(1000.0 * reported / kRate, 2) + " ms), measured " + String(measured, 3));
        expect(std::abs(measured - reported) < 1.0, "latency equals the measured delay");
    }

    void flangerNotches()
    {
        beginTest(rateTag() + "fx.flanger2: comb notches at (2k + 1) / (2 |tau|) within 0.5 % (mix 0.5, no feedback, read "
                              "with tones), tau of either sign");
        double worst = 0.0, shallow = 1000.0;
        String worstAt;
        int notches = 0;
        for (const double tauMs : {0.4, 1.0, 2.5, 4.8, -0.8, -1.7, -4.0})
        {
            Rack h("fx_flanger2_on");
            flangerBase(h, tauMs);
            // Gain of the whole effect for a tone (the same rack, settled for 0.1 s).
            const auto gain = [&](double hz)
            {
                std::vector<float> l(static_cast<size_t>(0.3 * kRate)), r;
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = static_cast<float>(0.1 * std::sin(2.0 * kPi * hz * static_cast<double>(i) / kRate));
                r = l;
                for (size_t pos = 0; pos < l.size(); pos += 128)
                    h.run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(128, l.size() - pos)));
                return toneLevel(l, static_cast<size_t>(0.1 * kRate), hz) / 0.1;
            };
            // k = 0, 1, 2, 3, then every few, up to 0.35 of the sample rate.
            std::vector<int> ks = {0, 1, 2, 3, 4, 5, 6, 8, 10, 13, 17, 21, 27, 34, 44, 55, 70, 89};
            for (const int k : ks)
            {
                const double want = (2.0 * k + 1.0) / (2.0 * std::abs(tauMs) * 0.001);
                if (want < 100.0 || want > 0.35 * kRate)
                    continue;
                double best = want, bestG = 1.0e9;
                for (const double scale : {0.98, 0.99, 1.0, 1.01, 1.02})
                    if (const double g = gain(want * scale); g < bestG)
                    {
                        bestG = g;
                        best = want * scale;
                    }
                for (int pass = 0; pass < 2; ++pass)
                {
                    const double step = pass == 0 ? 0.004 : 0.0012;
                    const double gm = gain(best * (1.0 - step)), g0 = gain(best), gp = gain(best * (1.0 + step));
                    const double den = gm * gm - 2.0 * g0 * g0 + gp * gp;
                    if (den > 0.0)
                        best *= 1.0 + step * 0.5 * (gm * gm - gp * gp) / den;
                }
                const double err = std::abs(best / want - 1.0);
                if (err > worst)
                {
                    worst = err;
                    worstAt = String("tau ") + String(tauMs, 1) + " ms, notch " + String(want, 0) + " Hz";
                }
                ++notches;
                expect(err < 0.005, "notch within 0.5 % of the analytic frequency");
                if (k < 4)
                    shallow = std::min(shallow, -amplitudeToDb(gain(want)));
            }
        }
        logMessage(String("fx.flanger2: ") + String(notches) +
                   " notches (tau +-0.4 .. 4.8 ms, 100 Hz to 0.35 of the sample rate), worst frequency error " +
                   String(100.0 * worst, 3) + " % at " + worstAt + " (target 0.5 %); the first four of each are at least " +
                   String(shallow, 1) + " dB deep at the analytic frequency");
        expect(shallow > 30.0, "notches are deep at the analytic frequencies");
        // Full level at tau = 0.
        Rack z("fx_flanger2_on");
        flangerBase(z, 0.0);
        const Pair tone = runRack(z, sineBoth(static_cast<size_t>(1.0 * kRate), 1500.0, 0.2));
        const double zero = amplitudeToDb(toneLevel(tone.l, 24000, 1500.0) / 0.2);
        logMessage(String("fx.flanger2: level at tau = 0, mix 0.5: ") + String(zero, 3) + " dB");
        expect(std::abs(zero) < 0.1, "unity at the zero crossing");
    }

    void flangerThroughZero()
    {
        beginTest(rateTag() +
                  "fx.flanger2: passing through zero produces no step above -60 dB (tau sweeping across zero, 200 Hz sine)");
        for (const float fb : {0.0f, 0.6f, -0.6f})
        {
            Rack h("fx_flanger2_on");
            flangerBase(h, 0.0);
            h.set("fx_flanger2_depth", 3.0f);
            h.set("fx_flanger2_rate", 2.0f);
            h.set("fx_flanger2_feedback", fb);
            const Pair out = runRack(h, sineBoth(static_cast<size_t>(4.0 * kRate), 200.0, 0.3));
            const double step =
                std::max(maxSecondDifference(out.l, 24000, out.l.size()), maxSecondDifference(out.r, 24000, out.r.size()));
            logMessage(String("feedback ") + String(fb, 1) +
                       ", tau +-3 ms at 2 Hz (about 16 crossings): largest second difference " + String(step, 6) + " (" +
                       String(amplitudeToDb(step), 1) + " dBFS)");
            expect(step < 0.001, "no step above -60 dB");
        }
    }

    void flangerFeedbackBounded()
    {
        beginTest(rateTag() + "fx.flanger2: feedback +-0.95 is bounded with and without saturation, in both modes");
        double worst = 0.0;
        for (const float fb : {0.95f, -0.95f})
            for (const float sat : {0.0f, 1.0f})
                for (const int mode : {0, 1})
                {
                    Rack h("fx_flanger2_on");
                    flangerBase(h, 1.0);
                    h.set("fx_flanger2_mix", 1.0f);
                    h.set("fx_flanger2_mode", static_cast<float>(mode));
                    h.set("fx_flanger2_feedback", fb);
                    h.set("fx_flanger2_fb_sat", sat);
                    h.set("fx_flanger2_depth", 2.0f);
                    h.set("fx_flanger2_rate", 3.0f);
                    const Pair out = runRack(h, noiseBoth(static_cast<size_t>(6.0 * kRate), 0.25, false, 8));
                    double peak = 0.0;
                    bool finite = true;
                    for (const float v : out.l)
                    {
                        finite = finite && std::isfinite(v);
                        peak = std::max(peak, static_cast<double>(std::abs(v)));
                    }
                    worst = std::max(worst, peak);
                    expect(finite && peak < 30.0, "bounded");
                }
        logMessage(String("fx.flanger2: largest peak with feedback 0.95 / -0.95 on 0.25 rms noise: ") + String(worst, 2));
    }

    void flangerEnvelope()
    {
        beginTest(rateTag() + "fx.flanger2: the envelope source moves tau with the input level (quiet near Manual - Depth, "
                              "loud near Manual + Depth)");
        const auto tauFor = [&](float rms, double guessHz)
        {
            Rack h("fx_flanger2_on");
            flangerBase(h, 2.0);
            h.set("fx_flanger2_depth", 1.5f);
            h.set("fx_flanger2_source", 1.0f);
            h.set("fx_flanger2_env_attack", 2.0f);
            h.set("fx_flanger2_env_release", 50.0f);
            const int latency = flangerLatencyOf(h);
            const Pair in = noiseBoth(static_cast<size_t>(1.2 * kRate) + static_cast<size_t>(latency), rms, false, 9);
            const Pair out = runRack(h, in);
            const std::vector<float> aligned = dropFront(out.l, latency);
            return 1.0 / (2.0 * findNotch(in.l, aligned, static_cast<size_t>(0.8 * kRate), 8192, guessHz, 0.4));
        };
        const double quiet = tauFor(0.002, 540.0), loud = tauFor(0.5, 147.0);
        logMessage(String("envelope source: tau ") + String(1000.0 * quiet, 2) + " ms at 0.002 rms, " +
                   String(1000.0 * loud, 2) + " ms at 0.5 rms (range 0.5 .. 3.5 ms)");
        expect(loud > 2.5 * quiet, "a louder input moves tau up");
    }

    void flangerBbd()
    {
        beginTest(
            rateTag() +
            "fx.flanger2: BBD mode combs at the set delay (first notch within 2 %, read with tones) and stays bounded");
        for (const int stagesChoice : {0, 1})
        {
            Rack h("fx_flanger2_on");
            flangerBase(h, 3.0);
            h.set("fx_flanger2_mode", 1.0f);
            h.set("fx_flanger2_bbd_stages", static_cast<float>(stagesChoice));
            const auto gain = [&](double hz)
            {
                Rack t("fx_flanger2_on");
                t.values = h.values;
                const Pair tone = runRack(t, sineBoth(static_cast<size_t>(1.2 * kRate), hz, 0.1));
                return toneLevel(tone.l, 24000, hz) / 0.1;
            };
            const double want = 1.0 / (2.0 * 0.003);
            double best = want, bestG = 1.0e9;
            for (double f = want * 0.88; f <= want * 1.12; f *= 1.02)
                if (const double g = gain(f); g < bestG)
                {
                    bestG = g;
                    best = f;
                }
            const double gm = gain(best * 0.99), g0 = gain(best), gp = gain(best * 1.01);
            const double den = gm * gm - 2.0 * g0 * g0 + gp * gp;
            const double got = den > 0.0 ? best * (1.0 + 0.01 * 0.5 * (gm * gm - gp * gp) / den) : best;
            logMessage(String(stagesChoice == 0 ? "256" : "512") + " stages, tau 3 ms: first notch " + String(got, 1) +
                       " Hz, analytic " + String(want, 1) + " Hz (" + String(100.0 * (got / want - 1.0), 2) + " %), depth " +
                       String(-amplitudeToDb(gain(got)), 1) + " dB");
            expect(std::abs(got / want - 1.0) < 0.02, "BBD notch near the analytic frequency");
        }
    }

    void flangerNoZipper()
    {
        beginTest(rateTag() + "fx.flanger2: no zipper on manual, depth, rate, feedback, saturation, mix, spread, mode, "
                              "device size, source and shape changes");
        struct Change
        {
            const char* name;
            const char* id;
            float a, b;
        };
        for (const Change c :
             {Change{"manual", "fx_flanger2_manual", 1.5f, -1.5f}, Change{"depth", "fx_flanger2_depth", 5.0f, 0.2f},
              Change{"rate", "fx_flanger2_rate", 9.0f, 0.05f}, Change{"feedback", "fx_flanger2_feedback", 0.8f, -0.8f},
              Change{"saturation", "fx_flanger2_fb_sat", 1.0f, 0.0f}, Change{"mix", "fx_flanger2_mix", 1.0f, 0.1f},
              Change{"spread", "fx_flanger2_spread", 1.0f, 0.0f}, Change{"mode", "fx_flanger2_mode", 1.0f, 0.0f},
              Change{"device size", "fx_flanger2_bbd_stages", 1.0f, 0.0f},
              Change{"source", "fx_flanger2_source", 1.0f, 0.0f}, Change{"shape", "fx_flanger2_shape", 1.0f, 0.0f}})
        {
            const auto render = [&](bool automate, float fixed)
            {
                Rack h("fx_flanger2_on");
                h.set("fx_flanger2_mix", 0.6f);
                h.set("fx_flanger2_depth", 2.0f);
                h.set("fx_flanger2_feedback", 0.3f);
                if (std::string(c.id) == "fx_flanger2_bbd_stages")
                    h.set("fx_flanger2_mode", 1.0f);
                if (!automate)
                    h.set(c.id, fixed);
                const Pair out = runRack(h, sineBoth(static_cast<size_t>(5.0 * kRate), 200.0, 0.3),
                                         [&](size_t pos)
                                         {
                                             if (automate && pos == 96000)
                                                 h.set(c.id, c.a);
                                             if (automate && pos == 160000)
                                                 h.set(c.id, c.b);
                                         });
                return std::max(maxSecondDifference(out.l, 40000, 240000), maxSecondDifference(out.r, 40000, 240000));
            };
            const double step = render(true, 0.0f), ref = std::max(render(false, c.a), render(false, c.b));
            logMessage(String(c.name) + " change: largest second difference " + String(step, 6) + " (" +
                       String(amplitudeToDb(step), 1) + " dBFS), end states " + String(ref, 6));
            expect(step < std::max(0.001, 1.25 * ref),
                   String(c.name) + " change has no step above -60 dB (or above the end states)");
        }
    }

    void flangerAllocations()
    {
        beginTest(rateTag() + "fx.flanger2: no allocation on the audio thread with every parameter changing every block");
        Rack h("fx_flanger2_on");
        const char* ids[] = {"mix",      "mode",   "source", "manual",     "depth",    "rate",       "shape",
                             "feedback", "fb_sat", "spread", "bbd_stages", "env_sens", "env_attack", "env_release"};
        std::vector<size_t> index;
        for (const char* id : ids)
            index.push_back(static_cast<size_t>(Rack::ix(std::string("fx_flanger2_") + id)));
        const size_t onIndex = static_cast<size_t>(Rack::ix("fx_flanger2_on"));
        Random rng(3);
        std::vector<float> l(64), r(64);
        for (int warm = 0; warm < 8; ++warm)
            h.run(l.data(), r.data(), 64);
        crucible::testing::gAllocations = 0;
        crucible::testing::gTrackAllocations = true;
        for (int block = 0; block < 3000; ++block)
        {
            for (size_t i = 0; i < 64; ++i)
            {
                l[i] = static_cast<float>(0.3 * rng.gaussian());
                r[i] = static_cast<float>(0.3 * rng.gaussian());
            }
            auto& v = h.values;
            v[index[0]] = static_cast<float>(block % 11) / 10.0f;
            v[index[1]] = static_cast<float>((block / 40) % 2);
            v[index[2]] = static_cast<float>((block / 70) % 2);
            v[index[3]] = -5.0f + static_cast<float>(block % 21) * 0.5f;
            v[index[4]] = static_cast<float>(block % 11) * 0.5f;
            v[index[5]] = 0.05f + static_cast<float>(block % 17) * 0.5f;
            v[index[6]] = static_cast<float>((block / 50) % 2);
            v[index[7]] = -0.95f + static_cast<float>(block % 20) * 0.1f;
            v[index[8]] = static_cast<float>(block % 5) / 4.0f;
            v[index[9]] = static_cast<float>(block % 7) / 6.0f;
            v[index[10]] = static_cast<float>((block / 90) % 2);
            v[index[11]] = -40.0f + static_cast<float>(block % 81);
            v[index[12]] = 0.5f + static_cast<float>(block % 40);
            v[index[13]] = 5.0f + static_cast<float>(block % 100);
            v[onIndex] = block % 700 > 680 ? 0.0f : 1.0f;
            h.run(l.data(), r.data(), 64);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
    }

    void flangerCpu()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest(rateTag() + "fx.flanger2: CPU, percent of one core per stereo instance at 48 kHz (plan: 0.3 %)");
        const std::vector<std::pair<std::string, float>> defaults = {{"fx_flanger2_mix", 0.5f}};
        const std::vector<std::pair<std::string, float>> envelope = {{"fx_flanger2_mix", 0.5f},
                                                                     {"fx_flanger2_source", 1.0f},
                                                                     {"fx_flanger2_feedback", -0.7f},
                                                                     {"fx_flanger2_fb_sat", 0.5f}};
        const std::vector<std::pair<std::string, float>> bbd = {
            {"fx_flanger2_mix", 0.5f}, {"fx_flanger2_mode", 1.0f}, {"fx_flanger2_manual", 3.0f}};
        const double d = cpuPercent("fx_flanger2_on", defaults), e = cpuPercent("fx_flanger2_on", envelope),
                     b = cpuPercent("fx_flanger2_on", bbd);
        logMessage(String("CPU flanger2: defaults (digital, LFO, feedback 0.4) ") + String(d, 3) +
                   " %, envelope source with negative saturated feedback " + String(e, 3) + " %, BBD mode (256 stages) " +
                   String(b, 3) + " %");
        expectCpu(std::max(d, e) <= 0.3 * 1.2, "digital mode within 0.3 % (20 % measurement margin)");
#endif
    }
};

static ModulationFxTests modulationFxTests;
