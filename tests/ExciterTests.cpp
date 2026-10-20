// SPDX-License-Identifier: MIT
// fx.exciter (FX13, docs/plans/ENGINE-FX-PLAN.md section 3.4): accept rows of the plan as numbers. The printed lines are
// what docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "CpuTiming.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/fx/AnalogMatchedFilters.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <memory>
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
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;

namespace
{
// The sample rate of the rack; runTest repeats the main accept rows at 44.1 / 96 / 192 kHz by changing it (and the
// one-second window).
double kRate = 48000.0;
using Floats = std::vector<float>;

// The exciter through the rack, every other effect off.
struct Rack
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values;
    std::vector<float> legacy;
    explicit Rack(Quality quality = Quality::Standard)
    {
        ModuleContext ctx;
        ctx.sampleRate = kRate;
        ctx.quality = quality;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set("fx_exciter_on", 1.0f);
    }
    static int ix(const std::string& id) { return ParameterMap::indexOf(id); }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ix(id))] = v; }
    void p(const char* name, float v) { set(std::string("fx_exciter_") + name, v); }
    void run(float* l, float* r, int n)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
    }
    int latency()
    {
        auto* lat = rack.findCapability<LatencyReporter>("fx.exciter");
        return lat != nullptr ? lat->latencySamples() : -1;
    }
};

// Both channels carry the same input; returns the left output. Blocks of 128.
Floats render(Rack& h, const Floats& in)
{
    Floats l = in, r = in;
    for (size_t pos = 0; pos < l.size(); pos += 128)
        h.run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(128, l.size() - pos)));
    return l;
}

Floats tone(size_t n, double hz, double amp)
{
    Floats out(n);
    for (size_t i = 0; i < n; ++i)
        out[i] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * static_cast<double>(i) / kRate));
    return out;
}

// Amplitude of the hz component over [from, from + len) by correlation (len a multiple of the period).
double amplitudeAt(const Floats& x, size_t from, size_t len, double hz)
{
    std::complex<double> acc(0.0, 0.0);
    for (size_t i = 0; i < len; ++i)
        acc += static_cast<double>(x[from + i]) * std::polar(1.0, -2.0 * kPi * hz * static_cast<double>(from + i) / kRate);
    return 2.0 * std::abs(acc) / static_cast<double>(len);
}

double db(double a)
{
    return 20.0 * std::log10(std::max(a, 1.0e-12));
}

size_t kSettle = 48000;
size_t kWindow = 48000; // one second: every tone at a whole number of hertz is periodic in it

std::string rateTag()
{
    if (kRate == 48000.0)
        return std::string();
    char tag[32];
    std::snprintf(tag, sizeof(tag), "[%g kHz] ", kRate / 1000.0);
    return tag;
}

void setRate(double rate)
{
    kRate = rate;
    kSettle = kWindow = static_cast<size_t>(rate);
}

// The saturator's curve (the rail model): a cubic inside the knee, the rail beyond it.
double railCurve(double x)
{
    const double u = std::clamp(x / 1.5, -1.0, 1.0);
    return 1.5 * (u - u * u * u / 3.0);
}

// The exciter's documented law: amplitude of harmonic k of the curve at a sine of peak a plus an offset, by numerical
// Fourier integration.
double curveHarmonic(int k, double a, double offset)
{
    constexpr int steps = 8192;
    std::complex<double> acc(0.0, 0.0);
    for (int i = 0; i < steps; ++i)
    {
        const double t = 2.0 * kPi * (i + 0.5) / steps;
        acc += railCurve(a * std::sin(t) + offset) * std::polar(1.0, -static_cast<double>(k) * t);
    }
    return 2.0 * std::abs(acc) / steps;
}

// Magnitude of a Butterworth filter of the module's design (the same sections) at hz.
double butterworthMagnitude(crucible::fx::SectionKind kind, int order, double cornerHz, double hz)
{
    crucible::fx::PolePair poles[8];
    const int count = crucible::fx::butterworthPoles(order, poles);
    std::complex<double> h = 1.0;
    for (int i = 0; i < count; ++i)
    {
        const auto section = crucible::fx::bilinearSection(
            crucible::fx::analogPrototype(kind, 0.0, poles[i].q, poles[i].real ? 1 : 2), cornerHz, kRate);
        h *= crucible::fx::biquadResponse(section, 2.0 * kPi * hz / kRate);
    }
    return std::abs(h);
}

// Predicted amplitude of harmonic k at amount 1: the curve's harmonic times the high-pass behind the saturator and the droop
// of the first-order antiderivative (a two-point mean at the oversampled rate).
struct Law
{
    double corner;
    bool highMode;
    Law(double hz, bool high) : corner(hz), highMode(high) {}
    double harmonic(int k, double toneHz, double amp, double driveDb, double asymmetryPercent, int factor) const
    {
        const double g = std::pow(10.0, driveDb / 20.0);
        const double band =
            amp * butterworthMagnitude(highMode ? crucible::fx::SectionKind::HighPass : crucible::fx::SectionKind::LowPass,
                                       2, corner, toneHz);
        const double offset = 0.1 * 0.01 * asymmetryPercent;
        const double droop = std::cos(kPi * k * toneHz / (kRate * factor));
        return curveHarmonic(k, g * band, offset) *
               butterworthMagnitude(crucible::fx::SectionKind::HighPass, 6, corner, k * toneHz) * droop;
    }
};

#if !defined(CRUCIBLE_TEST_UNTIMED)
// Percent of one core per stereo instance at 48 kHz, blocks of 32, minus the rack with the effect off; best of repeats.
double cpuPercent(Quality quality, const std::vector<std::pair<std::string, float>>& setup, double* loadOut = nullptr)
{
    const size_t n = static_cast<size_t>(6.0 * kRate);
    Random rng(5);
    Floats src(n);
    for (auto& v : src)
        v = static_cast<float>(0.25 * rng.gaussian());
    const auto run = [&](bool enabled)
    {
        Rack h(quality);
        h.set("fx_exciter_on", enabled ? 1.0f : 0.0f);
        if (enabled)
            for (const auto& kv : setup)
                h.set(kv.first, kv.second);
        Floats l = src, r = src;
        for (size_t pos = 0; pos < 48000; pos += 32)
            h.run(l.data() + pos, r.data() + pos, 32);
        const double t0 = crucible::testing::threadCpuSeconds();
        for (size_t pos = 48000; pos + 32 <= n; pos += 32)
            h.run(l.data() + pos, r.data() + pos, 32);
        return crucible::testing::threadCpuSeconds() - t0;
    };
    crucible::testing::ScopedUserInteractiveQos qos;
    double best = 1.0e9;
    for (int i = 0; i < 8; ++i)
        best = std::min(best, run(true) - run(false));
    if (loadOut != nullptr)
    {
        double avg[3] = {0, 0, 0};
#if !defined(_WIN32) // no load average on Windows: the line prints 0.0 there
        getloadavg(avg, 3);
#endif
        *loadOut = avg[0];
    }
    return 100.0 * best / (static_cast<double>(n - 48000) / 48000.0);
}
#endif
} // namespace

class ExciterTests : public UnitTest
{
public:
    ExciterTests() : UnitTest("Exciter", "Effects") {}

    void runTest() override
    {
        basics();
        nullAtZeroAmount();
        belowFrequency();
        harmonicLaw();
        asymmetryLaw();
        bassLaw();
        listenAndContinuity();
        blockSizeIndependence();
        aliasing();
        stability();
        allocations();
        cpu();
        // The main accept rows again at the other sample rates (the rows above are the 48 kHz run).
        for (const double rate : {44100.0, 96000.0, 192000.0})
        {
            setRate(rate);
            basics();
            nullAtZeroAmount();
            belowFrequency();
            harmonicLaw();
            asymmetryLaw();
            bassLaw();
        }
        setRate(48000.0);
    }

private:
    void basics()
    {
        beginTest(rateTag() + "fx.exciter: current, spelled-out ids, registered before only fx.saturator, fx.shaper, fx.multiband_drive and fx.cabinet, latency reported");
        const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.exciter");
        expect(d != nullptr, "registered");
        if (d == nullptr)
            return;
        expect(d->status == ModuleStatus::Current, "status Current");
        bool names = true;
        for (const auto& q : d->params)
            names = names && (q.id.rfind("fx_exciter_", 0) == 0 || q.id == "fx_pos_exciter");
        expect(names, "every id is fx_exciter_* (or the slot)");
        logMessage(String("fx.exciter parameters: ") + String(static_cast<int>(d->params.size())));
        expect(ModuleRegistry::instance().ofKind(ModuleKind::Effect).at(kNumModuleFx - 5)->id == "fx.exciter",
               "registered after every other effect but fx.saturator, fx.shaper, fx.multiband_drive and fx.cabinet");
        const int draft = Rack(Quality::Draft).latency(), standard = Rack(Quality::Standard).latency(),
                  high = Rack(Quality::High).latency();
        logMessage(String("latency Draft / Standard / High: ") + String(draft) + " / " + String(standard) + " / " +
                   String(high) + " samples");
        expectEquals(draft, 0, "Draft runs at the base rate");
        expect(standard > 0 && high >= standard, "oversampling latency reported");
    }

    void nullAtZeroAmount()
    {
        beginTest(rateTag() + "fx.exciter: at amount 0 the output is the input delayed by the reported latency exactly "
                              "(both modes, all qualities)");
        Random rng(11);
        Floats in(24000);
        for (auto& v : in)
            v = static_cast<float>(0.3 * rng.gaussian());
        for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
            for (int mode = 0; mode < 2; ++mode)
            {
                Rack h(q);
                h.p("mode", static_cast<float>(mode));
                h.p("amount", 0.0f);
                h.p("asymmetry", 100.0f);
                h.p("drive", 9.0f);
                const Floats out = render(h, in);
                const size_t lag = static_cast<size_t>(h.latency());
                double worst = 0.0;
                for (size_t i = lag; i < in.size(); ++i)
                    worst = std::max(worst, static_cast<double>(std::abs(out[i] - in[i - lag])));
                expect(worst == 0.0, "mode " + String(mode) + " quality " + String(static_cast<int>(q)) + " null " +
                                         String(db(worst), 1) + " dB");
            }
    }

    // Gain of the output at a tone, against the input, with everything set to be as hard on the rule as possible.
    double gainChangeDb(Rack& h, double hz, double amp)
    {
        const Floats in = tone(kSettle + kWindow, hz, amp);
        const Floats out = render(h, in);
        return db(amplitudeAt(out, kSettle, kWindow, hz) / amp);
    }

    void belowFrequency()
    {
        beginTest(rateTag() + "fx.exciter: below the frequency the output changes by at most 0.5 dB (maximum drive, "
                              "asymmetry and amount; both modes)");
        double worstHigh = 0.0, worstLow = 0.0;
        for (const double f : {1000.0, 3500.0, 10000.0})
            for (const double amp : {0.25, 1.0})
                for (const double ratio : {0.5, 0.25, 0.05})
                {
                    Rack h;
                    h.p("mode", 0.0f);
                    h.p("frequency", static_cast<float>(f));
                    h.p("drive", 9.0f);
                    h.p("asymmetry", 100.0f);
                    h.p("amount", 100.0f);
                    const double hz = std::round(f * ratio);
                    worstHigh = std::max(worstHigh, std::abs(gainChangeDb(h, hz, amp)));
                }
        for (const double f : {40.0, 100.0, 250.0})
            for (const double amp : {0.25, 1.0})
                for (const double ratio : {0.5, 0.25})
                {
                    Rack h;
                    h.p("mode", 1.0f);
                    h.p("bass_frequency", static_cast<float>(f));
                    h.p("drive", 9.0f);
                    h.p("asymmetry", 100.0f);
                    h.p("amount", 100.0f);
                    const double hz = std::round(f * ratio);
                    worstLow = std::max(worstLow, std::abs(gainChangeDb(h, hz, amp)));
                }
        logMessage(String("fx.exciter below the frequency (at most half of it), worst change: High ") +
                   String(worstHigh, 3) + " dB, Low " + String(worstLow, 3) + " dB");
        expect(worstHigh <= 0.5, "High mode " + String(worstHigh, 3) + " dB");
        expect(worstLow <= 0.5, "Low mode " + String(worstLow, 3) + " dB");
    }

    // Harmonic k of a tone through the exciter at amount 100 %, in dB against the law.
    double lawErrorDb(int k, double f, bool high, double toneHz, double amp, double drive, double asym, Quality q,
                      double* levelOut = nullptr)
    {
        Rack h(q);
        h.p("mode", high ? 0.0f : 1.0f);
        h.p(high ? "frequency" : "bass_frequency", static_cast<float>(f));
        h.p("drive", static_cast<float>(drive));
        h.p("asymmetry", static_cast<float>(asym));
        h.p("amount", 100.0f);
        h.p("listen", 1.0f);
        const Floats out = render(h, tone(kSettle + kWindow, toneHz, amp));
        const double measured = amplitudeAt(out, kSettle, kWindow, k * toneHz);
        const Law law(f, high);
        const int factor = q == Quality::Draft ? 1 : 2;
        const double expected = law.harmonic(k, toneHz, amp, drive, asym, factor);
        if (levelOut != nullptr)
            *levelOut = db(measured);
        // A harmonic the law puts below -90 dBFS (a cubic has none above its third) must stay below -90 dBFS.
        if (expected < 3.0e-5)
            return std::max(0.0, db(measured) + 90.0);
        return db(measured) - db(expected);
    }

    void harmonicLaw()
    {
        beginTest(rateTag() + "fx.exciter High mode: the added harmonics 3 and 5 follow the documented law (Fourier "
                              "coefficients of the rail curve) within 0.3 dB");
        double worst = 0.0;
        for (const Quality q : {Quality::Standard, Quality::High})
            for (const double drive : {3.0, 6.0, 9.0})
                for (const double amp : {0.1, 0.3, 0.6})
                    for (const int k : {3, 5})
                    {
                        const double e = lawErrorDb(k, 1000.0, true, 2000.0, amp, drive, 0.0, q);
                        worst = std::max(worst, std::abs(e));
                        expect(std::abs(e) <= 0.3, "quality " + String(static_cast<int>(q)) + " drive " + String(drive, 0) +
                                                       " amplitude " + String(amp, 1) + " H" + String(k) + " error " +
                                                       String(e, 3) + " dB");
                    }
        logMessage(String("fx.exciter High mode harmonic law, worst error over 36 points: ") + String(worst, 3) + " dB");
        // Even harmonics of the symmetric curve.
        double even = -300.0;
        for (const double amp : {0.1, 0.3, 0.6})
        {
            Rack h;
            h.p("frequency", 1000.0f);
            h.p("drive", 9.0f);
            h.p("amount", 100.0f);
            const Floats out = render(h, tone(kSettle + kWindow, 2000.0, amp));
            const double fundamental = amplitudeAt(out, kSettle, kWindow, 2000.0);
            for (const int k : {2, 4})
                even = std::max(even, db(amplitudeAt(out, kSettle, kWindow, k * 2000.0) / fundamental));
        }
        logMessage(String("fx.exciter symmetric curve, even harmonics against the fundamental: ") + String(even, 1) + " dB");
        expect(even <= -90.0, "even harmonics of the symmetric setting at most -90 dB");
    }

    void asymmetryLaw()
    {
        beginTest(rateTag() +
                  "fx.exciter High mode: the asymmetry adds the even harmonics, and they follow the law within 0.3 dB");
        double worst = 0.0, h2 = 0.0;
        for (const double asym : {25.0, 50.0, 100.0})
            for (const double amp : {0.1, 0.3, 0.6})
                for (const int k : {2, 3, 4})
                {
                    double level = 0.0;
                    const double e = lawErrorDb(k, 1000.0, true, 2000.0, amp, 6.0, asym, Quality::Standard, &level);
                    worst = std::max(worst, std::abs(e));
                    if (k == 2 && asym == 100.0 && amp == 0.3)
                        h2 = level;
                    expect(std::abs(e) <= 0.3, "asymmetry " + String(asym, 0) + " amplitude " + String(amp, 1) + " H" +
                                                   String(k) + " error " + String(e, 3) + " dB");
                }
        logMessage(String("fx.exciter asymmetry law, worst error over 27 points: ") + String(worst, 3) +
                   " dB; H2 at 100 % asymmetry, 6 dB drive, 0.3 amplitude: " + String(h2, 1) + " dB");
        expect(h2 > -50.0, "second harmonic present");
    }

    void bassLaw()
    {
        beginTest(rateTag() + "fx.exciter Low mode: the harmonics of a bass tone follow the law within 0.3 dB and sit above "
                              "the original band");
        double worst = 0.0;
        for (const double tone_ : {40.0, 60.0, 80.0})
            for (const double amp : {0.1, 0.3, 0.6})
                for (const int k : {3, 5})
                {
                    const double e = lawErrorDb(k, 120.0, false, tone_, amp, 9.0, 0.0, Quality::Standard);
                    worst = std::max(worst, std::abs(e));
                    expect(std::abs(e) <= 0.3, "tone " + String(tone_, 0) + " Hz amplitude " + String(amp, 1) + " H" +
                                                   String(k) + " error " + String(e, 3) + " dB");
                }
        logMessage(String("fx.exciter Low mode harmonic law, worst error over 18 points: ") + String(worst, 3) + " dB");
        // The added fundamental is removed by the high-pass: a 60 Hz tone gains 180 Hz and 300 Hz but its 60 Hz changes by
        // at most 0.5 dB.
        Rack h;
        h.p("mode", 1.0f);
        h.p("bass_frequency", 120.0f);
        h.p("drive", 9.0f);
        h.p("amount", 100.0f);
        const Floats out = render(h, tone(kSettle + kWindow, 60.0, 0.5));
        const double fundamental = amplitudeAt(out, kSettle, kWindow, 60.0) / 0.5;
        const double third = amplitudeAt(out, kSettle, kWindow, 180.0);
        logMessage(String("fx.exciter Low mode, 60 Hz at 0.5: fundamental ") + String(db(fundamental), 3) +
                   " dB, 180 Hz at " + String(db(third), 1) + " dBFS");
        expect(std::abs(db(fundamental)) <= 0.5, "fundamental unchanged within 0.5 dB");
        expect(third > 0.005, "overtone added");
    }

    void listenAndContinuity()
    {
        beginTest(
            rateTag() +
            "fx.exciter: listen outputs exactly the added signal; a mode change and a sweep of the frequency make no step");
        Random rng(21);
        Floats in(24000);
        for (auto& v : in)
            v = static_cast<float>(0.3 * rng.gaussian());
        Rack a, b;
        a.p("frequency", 2500.0f);
        b.p("frequency", 2500.0f);
        b.p("listen", 1.0f);
        const Floats full = render(a, in), added = render(b, in);
        Rack c;
        c.p("amount", 0.0f);
        const Floats dry = render(c, in);
        double worst = 0.0, scale = 0.0;
        for (size_t i = 0; i < in.size(); ++i)
        {
            worst = std::max(worst, static_cast<double>(std::abs(full[i] - dry[i] - added[i])));
            scale = std::max(scale, static_cast<double>(std::abs(added[i])));
        }
        logMessage(String("fx.exciter listen against full minus dry: ") + String(db(worst / std::max(scale, 1.0e-9)), 1) +
                   " dB");
        expect(db(worst / std::max(scale, 1.0e-9)) <= -100.0, "listen equals output minus the delayed input to -100 dB");

        // Mode toggles and a frequency sweep on a loud 220 Hz tone: the largest step of the output stays near that of the
        // tone.
        Rack h;
        h.p("drive", 9.0f);
        h.p("amount", 100.0f);
        Floats l = tone(96000, 220.0, 0.5), r = l;
        double step = 0.0, previous = 0.0;
        for (size_t pos = 0; pos < l.size(); pos += 128)
        {
            const size_t block = pos / 128;
            h.p("mode", static_cast<float>((block / 150) % 2));
            h.p("frequency", 1000.0f + static_cast<float>(block % 100) * 90.0f);
            h.p("bass_frequency", 60.0f + static_cast<float>(block % 50) * 3.0f);
            h.run(l.data() + pos, r.data() + pos, 128);
        }
        for (size_t i = 1; i < l.size(); ++i)
            step = std::max(step, static_cast<double>(std::abs(l[i] - l[i - 1])));
        (void)previous;
        logMessage(String("fx.exciter largest sample step during mode changes and sweeps: ") + String(step, 4) +
                   " (a 220 Hz tone at 0.5 steps 0.0144)");
        expect(step < 0.15, "no click");
    }

    // Runs the exciter on `in` in host blocks of `block`, calling apply(rack, lastSampleOfBlock) before each block.
    template <class Apply>
    static Floats renderBlocks(const Floats& in, size_t block, const Apply& apply,
                               const std::vector<std::pair<std::string, float>>& setup)
    {
        Rack h;
        for (const auto& kv : setup)
            h.set(kv.first, kv.second);
        Floats l = in, r = in;
        for (size_t pos = 0; pos < l.size(); pos += block)
        {
            const size_t end = std::min(pos + block, l.size());
            apply(h, end - 1);
            h.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
        }
        return l;
    }

    void blockSizeIndependence()
    {
        beginTest(rateTag() + "fx.exciter: amount, mode and frequency ramps are smooth at host blocks of 1, 7 and 512 "
                              "(zipper below -90 dB) and behave alike");
        struct Control
        {
            const char* name;
            const char* id;
            float lo, hi;
            bool choice;
            std::vector<std::pair<std::string, float>> setup;
        };
        const std::vector<Control> controls = {
            {"amount", "fx_exciter_amount", 0.0f, 100.0f, false, {{"fx_exciter_drive", 9.0f}}},
            {"frequency",
             "fx_exciter_frequency",
             1000.0f,
             10000.0f,
             false,
             {{"fx_exciter_drive", 9.0f}, {"fx_exciter_amount", 100.0f}}},
            {"bass frequency",
             "fx_exciter_bass_frequency",
             40.0f,
             250.0f,
             false,
             {{"fx_exciter_mode", 1.0f}, {"fx_exciter_drive", 9.0f}, {"fx_exciter_amount", 100.0f}}},
            {"mode", "fx_exciter_mode", 0.0f, 1.0f, true, {{"fx_exciter_drive", 9.0f}, {"fx_exciter_amount", 100.0f}}}};
        for (const Control& c : controls)
            for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
            {
                const auto renderAutomated = [&](const Signal& in, const Signal& automation)
                {
                    Floats f(in.size());
                    for (size_t i = 0; i < in.size(); ++i)
                        f[i] = static_cast<float>(in[i]);
                    const Floats out = renderBlocks(
                        f, block,
                        [&](Rack& h, size_t at)
                        {
                            const double a = automation[at];
                            h.set(c.id, c.choice ? (a > 0.5 ? c.hi : c.lo) : static_cast<float>(c.lo + (c.hi - c.lo) * a));
                        },
                        c.setup);
                    return Signal(out.begin(), out.end());
                };
                const ZipperResult z = zipperTest(renderAutomated, kRate, 1000.0, -6.0);
                logMessage(String("fx.exciter ") + c.name + ", host block " + String(static_cast<int>(block)) +
                           ": ramp step " + String(z.rampDb, 1) + " dB, 2 Hz square " + String(z.squareDb, 1) + " dB");
                expect(z.rampDb <= -90.0, String(c.name) + " ramp zipper at block " + String(static_cast<int>(block)));
                expect(z.squareDb <= -90.0, String(c.name) + " square zipper at block " + String(static_cast<int>(block)));
            }

        // The same jump, at a sample where every block size starts a block (3584 = 7 x 512), gives the same output within
        // 0.001.
        for (const Control& c : controls)
        {
            const size_t total = 24000, jumpAt = 3584 * 3;
            const Floats in = tone(total, 1000.0, 0.5);
            Floats first;
            for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
            {
                const Floats out = renderBlocks(
                    in, block, [&](Rack& h, size_t last) { h.set(c.id, last >= jumpAt ? c.hi : c.lo); }, c.setup);
                if (first.empty())
                {
                    first = out;
                    continue;
                }
                double worst = 0.0;
                for (size_t i = 0; i < total; ++i)
                    worst = std::max(worst, static_cast<double>(std::abs(out[i] - first[i])));
                logMessage(String("fx.exciter ") + c.name + " jump, block " + String(static_cast<int>(block)) +
                           " against block 1: largest difference " + String(worst, 6));
                expect(worst < 0.001, String(c.name) + " jump behaves alike at block " + String(static_cast<int>(block)));
            }
        }
    }

    void aliasing()
    {
        beginTest(rateTag() + "fx.exciter: alias power below 20 kHz for 5 and 7 kHz tones at maximum drive, asymmetry and "
                              "amount (Standard -90 dB, High -100 dB)");
        struct Row
        {
            Quality quality;
            double limit;
        };
        for (const Row row : {Row{Quality::Standard, -90.0}, Row{Quality::High, -100.0}})
        {
            double worst = -400.0;
            for (const double hz : {5000.0, 7000.0})
                for (const double level : {-12.0, -6.0})
                    for (const double asym : {0.0, 100.0})
                        for (const double f : {1000.0, 3500.0})
                        {
                            Rack h(row.quality);
                            h.p("frequency", static_cast<float>(f));
                            h.p("drive", 9.0f);
                            h.p("asymmetry", static_cast<float>(asym));
                            h.p("amount", 100.0f);
                            const Floats out = render(h, tone((1 << 15) + 16384, hz, dbToAmplitude(level)));
                            const Signal y = toDouble(out, 16384, 1 << 15);
                            const double a = measureAliasing(y.data(), y.size(), kRate, hz).aliasDb;
                            worst = std::max(worst, a);
                            expect(a <= row.limit, "quality " + String(static_cast<int>(row.quality)) + " " + String(hz, 0) +
                                                       " Hz at " + String(level, 0) + " dBFS, asymmetry " + String(asym, 0) +
                                                       ", frequency " + String(f, 0) + ": " + String(a, 1) + " dB");
                        }
            logMessage(String("fx.exciter worst alias, quality ") + String(static_cast<int>(row.quality)) + ": " +
                       String(worst, 1) + " dB (limit " + String(row.limit, 0) + " dB)");
        }
    }

    void stability()
    {
        beginTest(rateTag() +
                  "fx.exciter: finite and bounded for 20 s of loud noise with every parameter at an extreme, held, with the "
                  "mode changing, with the frequencies changing and with both changing every 16 blocks");
        for (const int variant : {0, 1, 2, 3})
        {
            const bool changing = variant != 0;
            Random rng(31);
            Rack h(Quality::High);
            h.p("amount", 100.0f);
            h.p("drive", 9.0f);
            h.p("asymmetry", 100.0f);
            Floats l(128), r(128);
            double peak = 0.0;
            bool finite = true;
            for (int block = 0; block < 20 * 375; ++block)
            {
                for (size_t i = 0; i < 128; ++i)
                {
                    l[i] = static_cast<float>(std::clamp(0.7 * rng.gaussian(), -1.0, 1.0));
                    r[i] = static_cast<float>(std::clamp(0.7 * rng.gaussian(), -1.0, 1.0));
                }
                if (changing)
                {
                    const int step = block / 16;
                    if (variant == 1 || variant == 3)
                        h.p("mode", static_cast<float>((step / 5) % 2));
                    if (variant == 2 || variant == 3)
                    {
                        h.p("frequency", (step % 2) != 0 ? 1000.0f : 10000.0f);
                        h.p("bass_frequency", (step % 3) != 0 ? 40.0f : 250.0f);
                    }
                }
                h.run(l.data(), r.data(), 128);
                for (size_t i = 0; i < 128; ++i)
                {
                    finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
                    peak = std::max(peak, static_cast<double>(std::max(std::abs(l[i]), std::abs(r[i]))));
                }
            }
            expect(finite, "finite");
            logMessage(String("fx.exciter peak over 20 s of loud noise, ") +
                       (variant == 0
                            ? "held"
                            : (variant == 1 ? "mode changing"
                                            : (variant == 2 ? "frequencies changing" : "mode and frequencies changing"))) +
                       ": " + String(peak, 2));
            expect(peak < 4.0, "bounded");
        }
    }

    void allocations()
    {
        beginTest(rateTag() + "fx.exciter: no allocation on the audio thread with every parameter changing every block");
        Rack h;
        const char* ids[] = {"mode", "frequency", "bass_frequency", "drive", "asymmetry", "amount", "listen"};
        int index[7];
        for (int i = 0; i < 7; ++i)
            index[i] = Rack::ix(std::string("fx_exciter_") + ids[i]);
        Random rng(3);
        Floats l(64), r(64);
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
            const auto set = [&](int k, float value) { v[static_cast<size_t>(index[k])] = value; };
            set(0, static_cast<float>((block / 5) % 2));
            set(1, 1000.0f + static_cast<float>(block % 37) * 240.0f);
            set(2, 40.0f + static_cast<float>(block % 29) * 7.0f);
            set(3, static_cast<float>(block % 10));
            set(4, static_cast<float>(block % 5) * 25.0f);
            set(5, static_cast<float>(block % 7) * 16.0f);
            set(6, block % 53 == 0 ? 1.0f : 0.0f);
            h.run(l.data(), r.data(), 64);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
    }

    void cpu()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest(rateTag() + "fx.exciter: CPU, percent of one core per stereo instance at 48 kHz (plan: 0.3 %)");
        double load = 0.0;
        const std::vector<std::pair<std::string, float>> setup = {
            {"fx_exciter_drive", 9.0f}, {"fx_exciter_asymmetry", 50.0f}, {"fx_exciter_amount", 100.0f}};
        const double standard = cpuPercent(Quality::Standard, setup, &load);
        logMessage(String("CPU fx.exciter Standard: ") + String(standard, 3) + " % (best of 8, load average " +
                   String(load, 1) + ")");
        const double high = cpuPercent(Quality::High, setup, &load);
        logMessage(String("CPU fx.exciter High: ") + String(high, 3) + " % (best of 8, load average " + String(load, 1) +
                   ")");
        expect(standard <= 0.3 * 1.2, "Standard within 0.3 % (20 % measurement margin)");
        expect(high <= 0.5 * 1.2, "High within 0.5 % (20 % measurement margin)");
#endif
    }
};

static ExciterTests exciterTests;
