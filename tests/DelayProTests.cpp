// SPDX-License-Identifier: MIT
// Fractional delay and bucket-brigade core of the pro delay (FX10, docs/plans/ENGINE-FX-PLAN.md section 3.5): delay
// accuracy, flatness at every fractional position, also while modulated, CPU per read. Every claim is a number; the
// printed lines are what docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "CpuTiming.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/fx/Bbd.h>
#include <crucible/fx/FractionalDelay.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <string>
#include <memory>
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
// Impulse response of the fractional delay at a fixed delay.
Signal impulseResponse(int taps, double delay, size_t length = 512)
{
    FractionalDelay d;
    d.prepare(300.0, taps);
    Signal h(length);
    for (size_t i = 0; i < length; ++i)
        h[i] = static_cast<double>(d.process(i == 0 ? 1.0f : 0.0f, delay));
    return h;
}

std::complex<double> transferAt(const Signal& h, double hz, double rate)
{
    const double w = 2.0 * kPi * hz / rate;
    std::complex<double> s(0.0, 0.0);
    for (size_t n = 0; n < h.size(); ++n)
        s += h[n] * std::polar(1.0, -w * static_cast<double>(n));
    return s;
}

// Largest deviation from 0 dB over 100 Hz .. limit at every fraction in 1/64 steps; delay = whole + fraction.
double worstFlatnessDb(int taps, double rate, double limitHz, double whole, double* worstFraction = nullptr)
{
    double worst = 0.0;
    for (int j = 0; j < 64; ++j)
    {
        const double frac = j / 64.0;
        const Signal h = impulseResponse(taps, whole + frac);
        for (double hz = 100.0; hz <= limitHz; hz += 100.0)
        {
            const double dev = std::abs(amplitudeToDb(std::abs(transferAt(h, hz, rate))));
            if (dev > worst)
            {
                worst = dev;
                if (worstFraction != nullptr)
                    *worstFraction = frac;
            }
        }
    }
    return worst;
}

// Phase delay (samples) of the impulse response at hz, relative to the requested delay.
double phaseDelayError(const Signal& h, double delay, double hz, double rate)
{
    const double w = 2.0 * kPi * hz / rate;
    const std::complex<double> H = transferAt(h, hz, rate) * std::polar(1.0, w * delay);
    return -std::arg(H) / w;
}

// Gain in dB of a tone through the modulated delay: delay = centre + depth * sin(2 pi fm t).
double modulatedGainDb(int taps, double rate, double hz, double centre, double depth, double fm)
{
    FractionalDelay d;
    d.prepare(centre + depth + 8.0, taps);
    const size_t total = static_cast<size_t>(2.0 * rate);
    const size_t skip = static_cast<size_t>(0.5 * rate);
    Signal out;
    out.reserve(total - skip);
    for (size_t i = 0; i < total; ++i)
    {
        const double t = static_cast<double>(i) / rate;
        const float x = static_cast<float>(0.5 * std::sin(2.0 * kPi * hz * t));
        const float y = d.process(x, centre + depth * std::sin(2.0 * kPi * fm * t));
        if (i >= skip)
            out.push_back(static_cast<double>(y));
    }
    double ss = 0.0;
    for (const double v : out)
        ss += v * v;
    const double rms = std::sqrt(ss / static_cast<double>(out.size()));
    return amplitudeToDb(rms / (0.5 / std::sqrt(2.0)));
}
// A BBD wrapped as a one-sample processor for the stepped-sine measurements.
auto makeBbd(double rate, int stages, double clockHz, bool compander, float noiseDb = -120.0f)
{
    return [=]()
    {
        auto b = std::make_shared<Bbd>();
        b->prepare(rate);
        BbdSettings s;
        s.stages = stages;
        s.compander = compander;
        s.noiseDb = noiseDb;
        b->setSettings(s);
        b->setDelaySeconds(static_cast<double>(stages) / (2.0 * clockHz));
        b->reset();
        return [b](double x) { return static_cast<double>(b->process(static_cast<float>(x))); };
    };
}

// -3 dB frequency of the chain relative to its low-frequency gain, by scanning and interpolating.
double bandwidth3dB(double rate, int stages, double clockHz)
{
    const auto make = makeBbd(rate, stages, clockHz, false);
    const double ref = steppedGainDb(make, 0.02 * clockHz, rate, 0.02, 0.5, 0.2);
    double prevF = 0.02 * clockHz, prevG = 0.0;
    for (double f = 0.06 * clockHz; f < 0.9 * clockHz; f *= 1.03)
    {
        const double g = steppedGainDb(make, f, rate, 0.02, 0.5, 0.2) - ref;
        if (g < -3.0103)
            return prevF + (prevG + 3.0103) / (prevG - g) * (f - prevF);
        prevF = f;
        prevG = g;
    }
    return 0.9 * clockHz;
}

// ---- fx.delay_pro through the rack, every other effect off
// The sample rate of the module rows that repeat at 44.1 / 96 / 192 kHz (runTest changes it); seconds in samples at that
// rate.
double gRate = 48000.0;
size_t sec(double seconds)
{
    return static_cast<size_t>(std::llround(seconds * gRate));
}
std::string rateTag()
{
    if (gRate == 48000.0)
        return std::string();
    char tag[32];
    std::snprintf(tag, sizeof(tag), "[%g kHz] ", gRate / 1000.0);
    return tag;
}

struct Rack
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values;
    std::vector<float> legacy;
    explicit Rack(double rate = 48000.0)
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set("fx_delay_pro_on", 1.0f);
        set("fx_delay_pro_mix", 1.0f);
        set("fx_delay_pro_feedback", 0.0f);
        set("fx_delay_pro_high_cut", 20000.0f);
    }
    static int ix(const std::string& id) { return ParameterMap::indexOf(id); }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ix(id))] = v; }
    void run(float* l, float* r, int n, const float* auxL = nullptr, const float* auxR = nullptr)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, auxL, auxR);
    }
};

struct Pair
{
    std::vector<float> l, r;
};

// Runs the rack over the input in blocks of 128; `at(sample)` may change parameters at block starts.
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

Pair silence(size_t n)
{
    return {std::vector<float>(n, 0.0f), std::vector<float>(n, 0.0f)};
}

Pair sineBoth(size_t n, double hz, double amp, double rate = 48000.0)
{
    Pair p = silence(n);
    for (size_t i = 0; i < n; ++i)
        p.l[i] = p.r[i] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * static_cast<double>(i) / rate));
    return p;
}

size_t peakIndex(const std::vector<float>& x, size_t from = 0, size_t to = 0)
{
    if (to == 0 || to > x.size())
        to = x.size();
    size_t best = from;
    for (size_t i = from; i < to; ++i)
        if (std::abs(x[i]) > std::abs(x[best]))
            best = i;
    return best;
}

double rmsDbOf(const std::vector<float>& x, size_t from, size_t to)
{
    double s = 0.0;
    for (size_t i = from; i < to; ++i)
        s += static_cast<double>(x[i]) * x[i];
    return powerToDb(s / static_cast<double>(to - from));
}

// Largest second difference: a step of height s adds s here, a clean sine of amplitude a at w rad / sample adds a w^2.
double maxSecondDifference(const std::vector<float>& x, size_t from, size_t to)
{
    double m = 0.0;
    for (size_t i = from + 2; i < to; ++i)
        m = std::max(m, std::abs(static_cast<double>(x[i]) - 2.0 * x[i - 1] + x[i - 2]));
    return m;
}

// Amplitude and phase (radians) of the hz component of x over [from, from + 4800) by correlation.
std::complex<double> componentAt(const std::vector<float>& x, size_t from, double hz, double rate = 48000.0)
{
    std::complex<double> acc(0.0, 0.0);
    const size_t len =
        static_cast<size_t>(rate / 5.0 + 0.5); // 200 ms: whole cycles of every whole-hertz tone that divides 5 Hz
    for (size_t i = 0; i < len; ++i)
        acc += static_cast<double>(x[from + i]) * std::polar(1.0, -2.0 * kPi * hz * static_cast<double>(from + i) / rate);
    return acc * (2.0 / static_cast<double>(len));
}

#if !defined(CRUCIBLE_TEST_UNTIMED)
// Percent of one core per stereo instance at 48 kHz, blocks of 32, minus the rack with the effect off; best of four.
double cpuPercent(const std::vector<std::pair<std::string, float>>& setup)
{
    const size_t n = static_cast<size_t>(6.0 * 48000.0);
    Random rng(5);
    std::vector<float> src(n);
    for (auto& v : src)
        v = static_cast<float>(0.25 * rng.gaussian());
    const auto run = [&](bool enabled)
    {
        Rack h;
        h.set("fx_delay_pro_on", enabled ? 1.0f : 0.0f);
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
    return 100.0 * best / (static_cast<double>(n - 48000) / 48000.0);
}
#endif
} // namespace

class DelayProTests : public UnitTest
{
public:
    DelayProTests() : UnitTest("DelayPro", "Effects") {}

    void runTest() override
    {
        fractionalDelayAccuracy();
        fractionalDelayFlatness();
        fractionalDelayModulated();
        fractionalDelayCost();
        bbdBandwidth();
        bbdImaging();
        bbdBoundedAndSilent();
        bbdCost();
        moduleBasics();
        moduleDigitalTiming();
        moduleStereoModes();
        moduleFeedbackBounded();
        moduleDucking();
        moduleFreeze();
        moduleFreezeLong();
        moduleTimeChanges();
        moduleAnalog();
        analogDriveZipper();
        moduleAllocations();
        moduleCpu();
        // The main accept rows again at the other sample rates (the rows above are the 48 kHz run).
        for (const double rate : {44100.0, 96000.0, 192000.0})
        {
            gRate = rate;
            moduleDigitalTiming();
            moduleStereoModes();
            moduleAnalog();
        }
        gRate = 48000.0;
    }

private:
    void fractionalDelayAccuracy()
    {
        beginTest("FractionalDelay: delay exact to 0.01 sample at fixed times (phase delay at 500 Hz, 2 kHz, 8 kHz)");
        double worst = 0.0;
        for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
            for (const double delay : {15.0, 15.25, 16.5, 40.75, 100.1, 200.9, 201.0})
            {
                const Signal h = impulseResponse(32, delay);
                for (const double hz : {500.0, 2000.0, 8000.0})
                    worst = std::max(worst, std::abs(phaseDelayError(h, delay, hz, rate)));
            }
        logMessage(String("FractionalDelay 32 taps: worst phase-delay error ") + String(worst, 5) +
                   " samples (target 0.01)");
        expect(worst < 0.01, "delay accurate to 0.01 sample");
        // The integer positions are exact.
        {
            FractionalDelay d;
            d.prepare(300.0, 32);
            double e = 0.0;
            for (int i = 0; i < 400; ++i)
            {
                const float y = d.process(i == 3 ? 1.0f : 0.0f, 20.0);
                e = std::max(e, static_cast<double>(std::abs(y - (i == 23 ? 1.0f : 0.0f))));
            }
            logMessage(String("integer delay 20: largest error against a pure shift ") + String(e, 8));
            expect(e < 1.0e-6, "integer delays are a pure shift");
        }
    }

    void fractionalDelayFlatness()
    {
        beginTest("FractionalDelay: magnitude flat within 0.1 dB to 18 kHz at every fractional position (1/64 steps)");
        for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
        {
            double frac = 0.0;
            const double dev = worstFlatnessDb(32, rate, 18000.0, 40.0, &frac);
            logMessage(String("FractionalDelay 32 taps at ") + String(rate, 0) + " Hz: worst deviation to 18 kHz " +
                       String(dev, 4) + " dB at fraction " + String(frac, 3) + " (target 0.1)");
            expect(dev < 0.1, "flat within 0.1 dB to 18 kHz");
        }
        const double d16 = worstFlatnessDb(16, 48000.0, 16000.0, 40.0);
        const double d64 = worstFlatnessDb(64, 44100.0, 19800.0, 40.0);
        logMessage(String("16 taps at 48 kHz: ") + String(d16, 4) + " dB to 16 kHz; 64 taps at 44.1 kHz: " + String(d64, 4) +
                   " dB to 19.8 kHz");
        expect(d16 < 0.1, "16 taps flat to 16 kHz at 48 kHz");
        expect(d64 < 0.1, "64 taps flat to 19.8 kHz at 44.1 kHz");
    }

    void fractionalDelayModulated()
    {
        beginTest("FractionalDelay: flat within 0.1 dB while the delay is modulated (+-3 samples at 1 Hz and +-1.5 samples "
                  "at 6.5 Hz)");
        for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
        {
            double worst = 0.0;
            for (const double hz : {1000.0, 8000.0, 14000.0, 17000.0, 18000.0})
            {
                worst = std::max(worst, std::abs(modulatedGainDb(32, rate, hz, 60.37, 3.0, 1.0)));
                worst = std::max(worst, std::abs(modulatedGainDb(32, rate, hz, 60.37, 1.5, 6.5)));
            }
            logMessage(String("modulated at ") + String(rate, 0) + " Hz: worst gain deviation " + String(worst, 4) +
                       " dB up to 18 kHz (target 0.1)");
            expect(worst < 0.1, "flat within 0.1 dB while modulated");
        }
    }

    void fractionalDelayCost()
    {
        beginTest("FractionalDelay: no allocation in write / read; CPU per read");
        FractionalDelay d;
        d.prepare(2000.0, 32);
        crucible::testing::gAllocations = 0;
        crucible::testing::gTrackAllocations = true;
        float acc = 0.0f;
        for (int i = 0; i < 4096; ++i)
            acc += d.process(0.001f * static_cast<float>(i % 17), 100.0 + 0.37 * (i % 29));
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
        expect(std::isfinite(acc), "finite");
#if !defined(CRUCIBLE_TEST_UNTIMED)
        crucible::testing::ScopedUserInteractiveQos qos;
        for (const int taps : {16, 32, 64})
        {
            FractionalDelay t;
            t.prepare(2000.0, taps);
            const size_t n = 48000 * 4;
            float sink = 0.0f;
            double best = 1.0e9;
            for (int rep = 0; rep < 3; ++rep)
            {
                const double t0 = crucible::testing::threadCpuSeconds();
                for (size_t i = 0; i < n; ++i)
                    sink += t.process(static_cast<float>(i & 255) * 0.001f, 100.0 + 0.0123 * static_cast<double>(i % 997));
                best = std::min(best, crucible::testing::threadCpuSeconds() - t0);
            }
            const double nsPerRead = best / static_cast<double>(n) * 1.0e9;
            logMessage(String("FractionalDelay ") + String(taps) + " taps: " + String(nsPerRead, 1) +
                       " ns per read = " + String(nsPerRead * 48000.0 * 1.0e-9 * 100.0, 4) +
                       " % of a core per read per sample at 48 kHz" + (sink == 12345.678f ? "!" : ""));
            expect(nsPerRead < 400.0, "read cost sane");
        }
#endif
    }
    void bbdBandwidth()
    {
        beginTest("Bbd: -3 dB bandwidth of the chain tracks the clock (documented 0.30 x clock) within 10 %");
        for (const double clk : {8000.0, 20000.0, 40000.0})
        {
            const double bw = bandwidth3dB(48000.0, 1024, clk);
            const double doc = Bbd::bandwidthHz(clk);
            logMessage(String("BBD 1024 stages, clock ") + String(clk, 0) + " Hz: -3 dB at " + String(bw, 0) +
                       " Hz, documented " + String(doc, 0) + " Hz (" + String(100.0 * (bw / doc - 1.0), 1) + " %)");
            expect(std::abs(bw / doc - 1.0) < 0.10, "bandwidth within 10 % of the documented value");
        }
        const double bw4096 = bandwidth3dB(96000.0, 4096, 60000.0);
        logMessage(String("BBD 4096 stages at 96 kHz, clock 60000 Hz: -3 dB at ") + String(bw4096, 0) + " Hz, documented " +
                   String(Bbd::bandwidthHz(60000.0), 0) + " Hz");
        expect(std::abs(bw4096 / Bbd::bandwidthHz(60000.0) - 1.0) < 0.10, "bandwidth within 10 % at 96 kHz");
    }

    void bbdImaging()
    {
        beginTest("Bbd: images of the clock below -60 dB (tone at 0.1 and 0.2 x clock, at 96 kHz)");
        const double rate = 96000.0;
        for (const double clk : {20000.0, 40000.0})
            for (const double ratio : {0.1, 0.2})
            {
                auto proc = makeBbd(rate, 1024, clk, false)();
                const double f0 = ratio * clk;
                const size_t settle = static_cast<size_t>(0.4 * rate), count = static_cast<size_t>(0.5 * rate);
                Signal out(count);
                for (size_t i = 0; i < settle + count; ++i)
                {
                    const double y = proc(0.05 * std::sin(2.0 * kPi * f0 * static_cast<double>(i) / rate));
                    if (i >= settle)
                        out[i - settle] = y;
                }
                const double carrier = toneAmplitude(out.data(), count, f0, rate);
                const double lower = amplitudeToDb(toneAmplitude(out.data(), count, clk - f0, rate) / carrier);
                const double upper = amplitudeToDb(toneAmplitude(out.data(), count, clk + f0, rate) / carrier);
                logMessage(String("BBD clock ") + String(clk, 0) + " Hz, tone " + String(f0, 0) +
                           " Hz: images at clock -/+ tone " + String(lower, 1) + " / " + String(upper, 1) +
                           " dB (target below -60)");
                expect(lower < -60.0 && upper < -60.0, "imaging below -60 dB");
            }
    }

    void bbdBoundedAndSilent()
    {
        beginTest("Bbd: bounded at full drive with a moving clock; silence in gives exact silence out with the noise off");
        {
            Bbd b;
            b.prepare(48000.0);
            BbdSettings s;
            s.stages = 2048;
            s.driveDb = 24.0f;
            s.noiseDb = -50.0f;
            s.clockFeedthroughDb = -40.0f;
            b.setSettings(s);
            double peak = 0.0;
            bool finite = true;
            for (size_t i = 0; i < 48000 * 4; ++i)
            {
                const double t = static_cast<double>(i) / 48000.0;
                b.setDelaySeconds(0.04 + 0.03 * std::sin(2.0 * kPi * 0.7 * t));
                const float x = static_cast<float>((std::fmod(t * 220.0, 1.0) < 0.5 ? 1.0 : -1.0) * (t < 2.0 ? 1.0 : 0.0));
                const float y = b.process(x);
                finite = finite && std::isfinite(y);
                peak = std::max(peak, static_cast<double>(std::abs(y)));
            }
            logMessage(String("BBD full-scale square +24 dB drive, noise and feed-through on, delay moving 10 to 70 ms: "
                              "peak output ") +
                       String(peak, 3) + " (bound 4)");
            expect(finite, "finite");
            expect(peak < 4.0, "bounded");
        }
        for (const bool comp : {false, true})
        {
            Bbd b;
            b.prepare(48000.0);
            BbdSettings s;
            s.stages = 1024;
            s.compander = comp;
            s.noiseDb = -120.0f;
            b.setSettings(s);
            b.setDelaySeconds(0.03);
            double silent = 0.0;
            for (size_t i = 0; i < 48000; ++i)
                silent = std::max(silent, static_cast<double>(std::abs(b.process(0.0f))));
            double tail = 0.0;
            for (size_t i = 0; i < 48000 * 4; ++i)
            {
                const float x = i < 4800 ? 0.5f * static_cast<float>(std::sin(0.05 * static_cast<double>(i))) : 0.0f;
                const float y = b.process(x);
                if (i >= 48000 * 3)
                    tail = std::max(tail, static_cast<double>(std::abs(y)));
            }
            logMessage(String("BBD ") + (comp ? "with" : "without") + " compander: output for 1 s of zeros " +
                       String(silent, 3) + ", tail 3 to 4 s after a 0.1 s burst " + String(tail, 10));
            expect(silent == 0.0, "zero in gives zero out");
            expect(tail < 1.0e-7, "the tail dies away");
        }
    }

    void bbdCost()
    {
        beginTest("Bbd: no allocation after prepare; CPU per instance");
        Bbd b;
        b.prepare(48000.0);
        BbdSettings s;
        s.stages = 1024;
        b.setSettings(s);
        b.setDelaySeconds(0.05);
        std::vector<float> in(512), out(512);
        for (size_t i = 0; i < in.size(); ++i)
            in[i] = 0.3f * static_cast<float>(std::sin(0.07 * static_cast<double>(i)));
        crucible::testing::gAllocations = 0;
        crucible::testing::gTrackAllocations = true;
        for (int blk = 0; blk < 200; ++blk)
        {
            b.setDelaySeconds(0.03 + 0.0001 * (blk % 50));
            b.processBlock(in.data(), out.data(), 512);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
#if !defined(CRUCIBLE_TEST_UNTIMED)
        crucible::testing::ScopedUserInteractiveQos qos;
        for (const double delay : {0.01, 0.05})
        {
            b.setDelaySeconds(delay);
            double best = 1.0e9;
            for (int rep = 0; rep < 4; ++rep)
            {
                const double t0 = crucible::testing::threadCpuSeconds();
                for (int blk = 0; blk < 400; ++blk)
                    b.processBlock(in.data(), out.data(), 512);
                best = std::min(best, crucible::testing::threadCpuSeconds() - t0);
            }
            const double ns = best / (400.0 * 512.0) * 1.0e9;
            logMessage(String("BBD delay ") + String(delay * 1000.0, 0) + " ms (clock " + String(b.clockHz(), 0) +
                       " Hz): " + String(ns, 1) + " ns per sample = " + String(ns * 48000.0 * 1.0e-9 * 100.0, 3) +
                       " % of a core per channel at 48 kHz");
            expect(ns * 48000.0 * 1.0e-9 * 100.0 < 0.25, "BBD core within its share of the 0.6 % budget per channel");
        }
#endif
    }

    // ---- fx.delay_pro
    void moduleBasics()
    {
        beginTest("fx.delay_pro: registered before fx.transient, current, spelled-out ids, latency 0, dry path exact");
        const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.delay_pro");
        expect(d != nullptr, "registered");
        if (d == nullptr)
            return;
        expect(d->status == ModuleStatus::Current, "status Current");
        int count = 0;
        bool names = true;
        for (const auto& p : d->params)
        {
            ++count;
            names = names && (p.id.rfind("fx_delay_pro_", 0) == 0 || p.id == "fx_pos_delay_pro");
        }
        expect(names, "every id is fx_delay_pro_* (or the slot)");
        logMessage(String("fx.delay_pro parameters: ") + String(count));
        {
            const auto effects = ModuleRegistry::instance().ofKind(ModuleKind::Effect);
            size_t at = 0;
            while (at < effects.size() && effects[at]->id != "fx.delay_pro")
                ++at;
            expect(
                at + 2 < effects.size() && effects[at + 1]->id == "fx.transient" && effects[at + 2]->id == "fx.deesser",
                "registered after every earlier effect (fx.transient, fx.deesser, the FX11 modules and fx.exciter follow)");
        }
        Rack h;
        auto* lat = h.rack.findCapability<LatencyReporter>("fx.delay_pro");
        expect(lat != nullptr, "latency capability");
        if (lat != nullptr)
            expectEquals(lat->latencySamples(), 0, "reported latency");
        h.set("fx_delay_pro_mix", 0.5f);
        Pair in = silence(256);
        in.l[0] = 1.0f;
        const Pair out = runRack(h, in);
        expectEquals(static_cast<double>(out.l[0]), 0.5, "dry impulse at sample 0 at mix 0.5");
    }

    void moduleDigitalTiming()
    {
        beginTest(rateTag() +
                  "fx.delay_pro Digital: delay time exact (integer samples, fractional by phase delay), flat to 18 kHz");
        {
            Rack h(gRate);
            h.set("fx_delay_pro_time", 100.0f);
            Pair in = silence(sec(0.25));
            in.l[0] = 1.0f;
            const Pair out = runRack(h, in);
            const size_t k = peakIndex(out.l);
            logMessage(String("impulse at 100 ms: peak at sample ") + String(static_cast<int>(k)) + ", value " +
                       String(static_cast<double>(out.l[k]), 5));
            expectEquals(static_cast<int>(k), static_cast<int>(sec(0.1)), "peak position");
            expect(std::abs(out.l[k] - 1.0f) < 1.0e-3f, "peak value");
        }
        {
            Rack h(gRate);
            h.set("fx_delay_pro_sync", 6.0f); // 1/4 note at 120 BPM
            Pair in = silence(sec(0.625));
            in.r[0] = 1.0f;
            const Pair out = runRack(h, in);
            expectEquals(static_cast<int>(peakIndex(out.r)), static_cast<int>(sec(0.5)), "1/4 at 120 BPM is 500 ms");
        }
        double worstPhase = 0.0, worstFlat = 0.0;
        for (const double ms : {100.01, 123.457, 250.31})
        {
            Rack h(gRate);
            h.set("fx_delay_pro_time", static_cast<float>(ms));
            const double delaySamples = static_cast<double>(static_cast<float>(ms)) * gRate / 1000.0;
            const Pair out = runRack(h, sineBoth(sec(2.0), 1000.0, 0.5, gRate));
            const size_t from = sec(1.0);
            const std::complex<double> c = componentAt(out.l, from, 1000.0, gRate);
            // Input phase is zero at sample 0: the output phase is -w * delay.
            double expected = -2.0 * kPi * 1000.0 * delaySamples / gRate -
                              0.5 * kPi; // the correlation with exp(-jwt) turns a sine into -j
            double diff = std::arg(c) - expected;
            diff -= 2.0 * kPi * std::round(diff / (2.0 * kPi));
            worstPhase = std::max(worstPhase, std::abs(diff) / (2.0 * kPi * 1000.0 / gRate));
            const double ref = std::abs(c);
            for (const double hz : {8000.0, 16000.0, 18000.0})
            {
                const Pair o2 = runRack(h, sineBoth(sec(2.0), hz, 0.5, gRate));
                worstFlat = std::max(worstFlat, std::abs(amplitudeToDb(std::abs(componentAt(o2.l, from, hz, gRate)) / ref)));
            }
        }
        logMessage(String("Digital: delay error ") + String(worstPhase, 5) +
                   " samples (target 0.01), response at 8 / 16 / 18 kHz within " + String(worstFlat, 4) +
                   " dB of 1 kHz (target 0.1)");
        expect(worstPhase < 0.01, "delay exact to 0.01 sample");
        expect(worstFlat < 0.1, "flat to 18 kHz within 0.1 dB");
    }

    void moduleStereoModes()
    {
        beginTest(rateTag() + "fx.delay_pro: stereo, ping-pong and cross feedback route the repeats");
        const char* names[] = {"Stereo", "Ping-pong", "Cross"};
        for (int mode = 0; mode < 3; ++mode)
        {
            Rack h(gRate);
            h.set("fx_delay_pro_time", 100.0f);
            h.set("fx_delay_pro_feedback", 0.5f);
            h.set("fx_delay_pro_saturation", 0.0f);
            h.set("fx_delay_pro_stereo", static_cast<float>(mode));
            Pair in = silence(sec(0.42));
            in.l[0] = 1.0f;
            const Pair out = runRack(h, in);
            const auto at = [&](const std::vector<float>& x, size_t k) { return static_cast<double>(std::abs(x[k])); };
            const double l1 = at(out.l, sec(0.1)), r1 = at(out.r, sec(0.1)), l2 = at(out.l, sec(0.2)),
                         r2 = at(out.r, sec(0.2)), l3 = at(out.l, sec(0.3)), r3 = at(out.r, sec(0.3));
            logMessage(String(names[mode]) + ": L " + String(l1, 3) + " " + String(l2, 3) + " " + String(l3, 3) + "  R " +
                       String(r1, 3) + " " + String(r2, 3) + " " + String(r3, 3));
            if (mode == 0)
                expect(l1 > 0.99 && std::abs(l2 - 0.5) < 0.01 && r1 < 1e-6 && r2 < 1e-6, "stereo stays on its side");
            else if (mode == 1)
                expect(std::abs(l1 - 0.5) < 0.01 && r1 < 1e-6 && std::abs(r2 - 0.25) < 0.01 && l2 < 1e-6 &&
                           std::abs(l3 - 0.125) < 0.01,
                       "ping-pong alternates, mono input");
            else
                expect(l1 > 0.99 && r1 < 1e-6 && std::abs(r2 - 0.5) < 0.01 && l2 < 1e-6 && std::abs(l3 - 0.25) < 0.01,
                       "cross alternates");
        }
    }

    void moduleFeedbackBounded()
    {
        beginTest("fx.delay_pro: feedback 0.99 for 60 s stays bounded with the saturation in the loop (every stereo mode, "
                  "both engines)");
        for (int engine = 0; engine < 2; ++engine)
            for (int mode = 0; mode < 3; ++mode)
            {
                Rack h;
                h.set("fx_delay_pro_mode", static_cast<float>(engine));
                h.set("fx_delay_pro_time", 80.0f);
                h.set("fx_delay_pro_feedback", 0.99f);
                h.set("fx_delay_pro_saturation", 0.0f);
                h.set("fx_delay_pro_mix", 0.5f);
                h.set("fx_delay_pro_stereo", static_cast<float>(mode));
                Random rng(11 + static_cast<std::uint64_t>(mode));
                std::vector<float> l(128), r(128);
                double peak = 0.0;
                bool finite = true;
                const int blocks = static_cast<int>(60.0 * 48000.0 / 128.0);
                for (int b = 0; b < blocks; ++b)
                {
                    const bool loud = b * 128 < 96000;
                    for (size_t i = 0; i < 128; ++i)
                    {
                        l[i] = loud ? static_cast<float>(0.9 * std::clamp(rng.gaussian() * 0.5, -1.0, 1.0)) : 0.0f;
                        r[i] = loud ? static_cast<float>(0.9 * std::clamp(rng.gaussian() * 0.5, -1.0, 1.0)) : 0.0f;
                    }
                    h.run(l.data(), r.data(), 128);
                    for (size_t i = 0; i < 128; ++i)
                    {
                        finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
                        peak = std::max({peak, std::abs(static_cast<double>(l[i])), std::abs(static_cast<double>(r[i]))});
                    }
                }
                logMessage(String(engine == 0 ? "Digital " : "Analog ") +
                           String(mode == 0 ? "stereo" : (mode == 1 ? "ping-pong" : "cross")) + ": peak " + String(peak, 3));
                expect(finite, "finite");
                expect(peak < 6.0, "bounded (limit 4 / (1 - 0.99 x 0.5) at the loosest saturation, 6 asserted)");
            }
    }

    void moduleDucking()
    {
        beginTest("fx.delay_pro: ducking depth within 0.2 dB (input and sidechain)");
        double worst = 0.0;
        for (const float depth : {6.0f, 12.0f, 24.0f})
        {
            const auto level = [&](float d, bool side)
            {
                Rack h;
                h.set("fx_delay_pro_time", 5.0f);
                h.set("fx_delay_pro_duck_depth", d);
                h.set("fx_delay_pro_duck_threshold", -30.0f);
                h.set("fx_delay_pro_duck_release", 100.0f);
                const size_t n = 96000;
                Pair in = sineBoth(n, 1000.0, 0.3);
                Pair aux = sineBoth(n, 440.0, 0.3);
                if (side)
                {
                    h.set("fx_delay_pro_duck_source", 1.0f);
                    // The delayed signal is the quiet input; the sidechain carries the loud one.
                    for (auto& v : in.l)
                        v *= 0.1f;
                    in.r = in.l;
                }
                Pair out = in;
                for (size_t pos = 0; pos < n; pos += 128)
                    h.run(out.l.data() + pos, out.r.data() + pos, 128, side ? aux.l.data() + pos : nullptr,
                          side ? aux.r.data() + pos : nullptr);
                return rmsDbOf(out.l, 48000, 96000);
            };
            for (const bool side : {false, true})
            {
                const double dev = std::abs((level(0.0f, side) - level(depth, side)) - static_cast<double>(depth));
                worst = std::max(worst, dev);
            }
        }
        {
            // A sidechain source with nothing connected does not duck.
            const auto level = [&](float source)
            {
                Rack h;
                h.set("fx_delay_pro_time", 5.0f);
                h.set("fx_delay_pro_duck_depth", 12.0f);
                h.set("fx_delay_pro_duck_source", source);
                Pair out = runRack(h, sineBoth(96000, 1000.0, 0.3));
                return rmsDbOf(out.l, 48000, 96000);
            };
            const double diff = level(1.0f) - level(0.0f);
            logMessage(String("sidechain source without an auxiliary input: ") + String(diff, 2) +
                       " dB louder than ducking from the input");
            expect(std::abs(diff - 12.0) < 0.3, "no ducking without a sidechain signal");
        }
        logMessage(String("ducking depth error at 6 / 12 / 24 dB (input and sidechain): worst ") + String(worst, 3) +
                   " dB (target 0.2)");
        expect(worst < 0.2, "ducking depth within 0.2 dB");
    }

    void moduleFreeze()
    {
        beginTest("fx.delay_pro: freeze holds the level (Digital within 0.5 dB for 40 s; Analog sinks slowly)");
        for (int engine = 0; engine < 2; ++engine)
        {
            Rack h;
            h.set("fx_delay_pro_mode", static_cast<float>(engine));
            h.set("fx_delay_pro_time", 200.0f);
            h.set("fx_delay_pro_analog_noise", -90.0f);
            h.set("fx_delay_pro_analog_compander", 0.0f);
            h.set("fx_delay_pro_analog_stages", 4.0f);
            h.set("fx_delay_pro_feedback", 0.3f);
            h.set("fx_delay_pro_high_cut", 3000.0f);
            Random rng(4);
            std::vector<float> l(128), r(128), keep;
            const int blocks = static_cast<int>(40.0 * 48000.0 / 128.0);
            std::vector<float> rec;
            for (int b = 0; b < blocks; ++b)
            {
                if (b * 128 == 128 * 750)
                    h.set("fx_delay_pro_freeze", 1.0f);
                for (size_t i = 0; i < 128; ++i)
                {
                    // Noise for Digital; the Analog loop loses the content above its bandwidth on every pass, so it gets low
                    // sines.
                    const double t = static_cast<double>(b * 128 + static_cast<int>(i)) / 48000.0;
                    const float v = engine == 0 ? static_cast<float>(0.15 * rng.gaussian())
                                                : static_cast<float>(0.1 * std::sin(2.0 * kPi * 220.0 * t) +
                                                                     0.1 * std::sin(2.0 * kPi * 330.0 * t));
                    l[i] = r[i] = v;
                }
                h.run(l.data(), r.data(), 128);
                rec.insert(rec.end(), l.begin(), l.end());
            }
            const size_t a0 = static_cast<size_t>(6.0 * 48000.0), a1 = static_cast<size_t>(36.0 * 48000.0);
            const double d = rmsDbOf(rec, a1, a1 + 96000) - rmsDbOf(rec, a0, a0 + 96000);
            logMessage(String(engine == 0 ? "Digital" : "Analog") + " freeze: level change from 6 s to 36 s " +
                       String(d, 3) + " dB (target 0.5)");
            // The Analog loop loses a little on every pass in the device's soft clip (about 0.3 % of amplitude at this
            // level), so its frozen level sinks slowly (as 1 / sqrt(time)); Digital holds.
            expect(std::abs(d) < (engine == 0 ? 0.5 : 4.0),
                   engine == 0 ? "frozen level held" : "frozen level sinks by less than 4 dB in 30 s");
        }
    }

    void moduleFreezeLong()
    {
        beginTest("fx.delay_pro: the frozen loop stays bounded for 10 minutes of audio (Digital and Analog, steady, "
                  "modulated with time glides, ping-pong, cross)");
        struct Case
        {
            const char* name;
            int engine;
            float timeMs, modDepth, modRate, glideMs;
            int stereo;
        };
        for (const Case c : {Case{"Digital steady", 0, 200.0f, 0.0f, 0.7f, 0.0f, 0},
                             Case{"Digital modulated, glided", 0, 200.0f, 4.0f, 0.7f, 60.0f, 0},
                             Case{"Digital short, fast modulation", 0, 40.0f, 6.0f, 3.0f, 15.0f, 0},
                             Case{"Digital ping-pong, modulated", 0, 120.0f, 3.0f, 1.3f, 40.0f, 1},
                             Case{"Digital cross, deep modulation", 0, 80.0f, 10.0f, 5.0f, 25.0f, 2},
                             Case{"Analog steady", 1, 200.0f, 0.0f, 0.7f, 0.0f, 0},
                             Case{"Analog modulated, glided", 1, 200.0f, 3.0f, 0.7f, 60.0f, 0},
                             Case{"Analog cross, modulated", 1, 120.0f, 2.0f, 1.3f, 30.0f, 2}})
        {
            Rack h;
            h.set("fx_delay_pro_mode", static_cast<float>(c.engine));
            h.set("fx_delay_pro_time", c.timeMs);
            h.set("fx_delay_pro_analog_noise", -90.0f);
            h.set("fx_delay_pro_analog_compander", 0.0f);
            h.set("fx_delay_pro_analog_stages", 4.0f);
            h.set("fx_delay_pro_feedback", 0.3f);
            h.set("fx_delay_pro_high_cut", 3000.0f);
            h.set("fx_delay_pro_stereo", static_cast<float>(c.stereo));
            h.set("fx_delay_pro_mod_depth", c.modDepth);
            h.set("fx_delay_pro_mod_rate", c.modRate);
            Random rng(7);
            const int block = 512;
            std::vector<float> l(block), r(block);
            const size_t total = static_cast<size_t>(600.0 * 48000.0), freezeAt = static_cast<size_t>(3.0 * 48000.0),
                         window = static_cast<size_t>(30.0 * 48000.0);
            // Mean square of the left output over every 30 s window, and over the second before the freeze.
            std::vector<double> energies;
            double ss = 0.0, preSs = 0.0, peak = 0.0;
            size_t inWindow = 0;
            bool finite = true;
            for (size_t pos = 0; pos < total; pos += block)
            {
                if (pos >= freezeAt && pos < freezeAt + block)
                    h.set("fx_delay_pro_freeze", 1.0f);
                if (c.glideMs > 0.0f && pos >= freezeAt)
                    h.set("fx_delay_pro_time",
                          c.timeMs + c.glideMs * static_cast<float>(((pos / (7 * 48000)) % 2 == 0) ? 1 : -1));
                for (int i = 0; i < block; ++i)
                {
                    const double t = static_cast<double>(pos + static_cast<size_t>(i)) / 48000.0;
                    const float v = pos < freezeAt
                                        ? (c.engine == 0 ? static_cast<float>(0.15 * rng.gaussian())
                                                         : static_cast<float>(0.1 * std::sin(2.0 * kPi * 220.0 * t) +
                                                                              0.1 * std::sin(2.0 * kPi * 330.0 * t)))
                                        : 0.0f;
                    l[static_cast<size_t>(i)] = r[static_cast<size_t>(i)] = v;
                }
                h.run(l.data(), r.data(), block);
                for (int i = 0; i < block; ++i)
                {
                    const double v = l[static_cast<size_t>(i)];
                    finite = finite && std::isfinite(v);
                    peak = std::max(peak, std::abs(v));
                    ss += v * v;
                    if (pos + static_cast<size_t>(i) >= freezeAt - 48000 && pos + static_cast<size_t>(i) < freezeAt)
                        preSs += v * v;
                }
                inWindow += block;
                if (inWindow >= window)
                {
                    energies.push_back(ss / static_cast<double>(inWindow));
                    ss = 0.0;
                    inWindow = 0;
                }
            }
            const auto db = [](double e) { return 10.0 * std::log10(std::max(e, 1.0e-20)); };
            const double pre = db(preSs / 48000.0);
            double worst = -200.0, early = 0.0, late = 0.0;
            for (size_t i = 1; i < energies.size(); ++i)
                worst = std::max(worst, db(energies[i]));
            for (size_t i = 8; i < 13; ++i)
                early += db(energies[i]) / 5.0;
            for (size_t i = energies.size() - 5; i < energies.size(); ++i)
                late += db(energies[i]) / 5.0;
            logMessage(String(c.name) + ": level before the freeze " + String(pre, 2) + " dB, highest 30 s window " +
                       String(worst, 2) + " dB, windows 4-6 min " + String(early, 2) + " dB, last 2.5 min " +
                       String(late, 2) + " dB, peak " + String(peak, 3));
            expect(finite, String(c.name) + " finite");
            expect(worst < pre + 1.5,
                   String(c.name) + " the frozen level never rises more than 1.5 dB above the level before the freeze");
            expect(late < pre + 1.5, String(c.name) + " and the last 2.5 minutes are within that bound too");
            expect(peak < 2.0, String(c.name) + " the frozen peak is bounded");
        }
    }

    void moduleTimeChanges()
    {
        beginTest("fx.delay_pro: time changes are click-free (second difference of a 200 Hz sine; the sine itself gives "
                  "0.00034, limit 0.001 = -60 dB)");
        struct Case
        {
            const char* name;
            float mode, change;
        };
        for (const Case c :
             {Case{"Digital crossfade", 0.0f, 1.0f}, Case{"Digital glide", 0.0f, 0.0f}, Case{"Analog", 1.0f, 0.0f}})
        {
            Rack h;
            h.set("fx_delay_pro_mode", c.mode);
            h.set("fx_delay_pro_time_change", c.change);
            h.set("fx_delay_pro_analog_noise", -90.0f);
            h.set("fx_delay_pro_time", 300.0f);
            const Pair out = runRack(h, sineBoth(240000, 200.0, 0.5),
                                     [&](size_t pos)
                                     {
                                         if (pos == 96000)
                                             h.set("fx_delay_pro_time", 500.0f);
                                         if (pos == 160000)
                                             h.set("fx_delay_pro_time", 250.0f);
                                     });
            const double step = maxSecondDifference(out.l, 40000, 240000);
            logMessage(String(c.name) + ": largest second difference " + String(step, 6) + " (" +
                       String(amplitudeToDb(step), 1) + " dBFS)");
            expect(step < 0.001, String(c.name) + " has no step above -60 dB");
        }
        beginTest("fx.delay_pro: switching engine fades out, clears and fades in without a click");
        {
            Rack h;
            h.set("fx_delay_pro_time", 80.0f);
            h.set("fx_delay_pro_feedback", 0.5f);
            h.set("fx_delay_pro_analog_noise", -90.0f);
            const Pair out = runRack(h, sineBoth(240000, 200.0, 0.25),
                                     [&](size_t pos)
                                     {
                                         if (pos == 96000)
                                             h.set("fx_delay_pro_mode", 1.0f);
                                         if (pos == 160000)
                                             h.set("fx_delay_pro_mode", 0.0f);
                                     });
            const double step = maxSecondDifference(out.l, 20000, 240000);
            logMessage(String("engine switch: largest second difference ") + String(step, 6));
            for (size_t i = 20002; i < 240000; ++i)
                if (std::abs(static_cast<double>(out.l[i]) - 2.0 * out.l[i - 1] + out.l[i - 2]) > 0.002)
                {
                    logMessage(String("  jump at ") + String(static_cast<int>(i)) + " value " +
                               String(static_cast<double>(out.l[i]), 4));
                    break;
                }
            expect(step < 0.001, "no step above -60 dB");
        }
    }

    void moduleAnalog()
    {
        beginTest(rateTag() + "fx.delay_pro Analog: the echo arrives at the set time (centroid of a smooth pulse), "
                              "bandwidth follows the clock");
        for (const float ms : {300.0f, 100.0f, 40.0f})
        {
            Rack h(gRate);
            h.set("fx_delay_pro_mode", 1.0f);
            h.set("fx_delay_pro_time", ms);
            h.set("fx_delay_pro_analog_noise", -90.0f);
            h.set("fx_delay_pro_analog_compander", 0.0f);
            Pair in = silence(static_cast<size_t>(gRate / 1000.0 * ms) + sec(0.5));
            const int pulse = static_cast<int>(sec(0.008)), pulseAt = static_cast<int>(sec(1000.0 / 48000.0));
            for (int i = 0; i < pulse; ++i) // 8 ms Hann pulse
                in.l[static_cast<size_t>(pulseAt + i)] = static_cast<float>(0.5 - 0.5 * std::cos(2.0 * kPi * i / pulse));
            const Pair out = runRack(h, in);
            double sw = 0.0, sx = 0.0;
            for (size_t i = 0; i < out.l.size(); ++i)
            {
                const double e = static_cast<double>(out.l[i]) * out.l[i];
                if (i > static_cast<size_t>(pulseAt + pulse + 16) + static_cast<size_t>(gRate / 1000.0 * ms * 0.5))
                {
                    sw += e;
                    sx += e * static_cast<double>(i);
                }
            }
            const double delay = (sx / sw - static_cast<double>(pulseAt) - 0.5 * pulse) / (gRate / 1000.0);
            logMessage(String("Analog ") + String(static_cast<double>(ms), 0) + " ms: echo centroid at " + String(delay, 2) +
                       " ms");
            expect(std::abs(delay - static_cast<double>(ms)) < 0.03 * static_cast<double>(ms) + 0.3, "echo time within 3 %");
        }
        beginTest("fx.delay_pro Analog: the two channels carry different noise");
        {
            Rack h;
            h.set("fx_delay_pro_mode", 1.0f);
            h.set("fx_delay_pro_time", 100.0f);
            h.set("fx_delay_pro_analog_noise", -50.0f);
            const Pair out = runRack(h, silence(48000));
            double same = 0.0, tot = 0.0;
            for (size_t i = 12000; i < 48000; ++i)
            {
                same += static_cast<double>(out.l[i]) * out.r[i];
                tot += static_cast<double>(out.l[i]) * out.l[i];
            }
            expect(tot > 0.0 && std::abs(same / tot) < 0.2, "noise decorrelated");
        }
    }

    static constexpr double kDriveRate = 48000.0;
    static Signal analogDriveRender(size_t block, const Signal& in, const Signal& automation)
    {
        Rack h;
        h.set("fx_delay_pro_mode", 1.0f);
        h.set("fx_delay_pro_time", 20.0f);
        h.set("fx_delay_pro_analog_noise", -120.0f);
        std::vector<float> l(in.begin(), in.end()), r = l;
        for (size_t pos = 0; pos < l.size(); pos += block)
        {
            const size_t end = std::min(pos + block, l.size());
            h.set("fx_delay_pro_analog_drive", static_cast<float>(-12.0 + 36.0 * automation[end - 1]));
            h.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
        }
        return Signal(l.begin(), l.end());
    }

    void analogDriveZipper()
    {
        beginTest("fx.delay_pro Analog Drive: ramp and 2 Hz square leave no step above the engine's own residue at host "
                  "blocks of 1, 7 and 512, and the output matches across block sizes");
        for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
        {
            const auto render = [&](const Signal& in, const Signal& automation)
            { return analogDriveRender(block, in, automation); };
            const ZipperResult z = zipperTest(render, kDriveRate, 1000.0, -6.0);
            // The analog engine leaves a small residue of its own (clock ripple): the same estimator on a held setting is
            // the floor.
            const size_t held = static_cast<size_t>(0.6 * kDriveRate);
            const Signal hold =
                analogDriveRender(block, sineWave(held, 1000.0, kDriveRate, dbToAmplitude(-6.0)), Signal(held, 1.0));
            const double floorDb = clickTest(hold, 1000.0, kDriveRate, static_cast<size_t>(0.1 * kDriveRate)).stepDb;
            const double limit = std::max(-90.0, floorDb + 3.0);
            logMessage(String("fx.delay_pro analog drive, host block ") + String(static_cast<int>(block)) + ": ramp step " +
                       String(z.rampDb, 1) + " dB, 2 Hz square " + String(z.squareDb, 1) + " dB, held-setting floor " +
                       String(floorDb, 1) + " dB");
            expect(z.rampDb <= limit, String("drive ramp zipper at block ") + String(static_cast<int>(block)));
            expect(z.squareDb <= limit, String("drive square zipper at block ") + String(static_cast<int>(block)));
        }
        const size_t total = 24000, jumpAt = 3584 * 3;
        const Signal in = sineWave(total, 1000.0, kDriveRate, 0.5);
        Signal automation(total);
        for (size_t i = 0; i < total; ++i)
            automation[i] = i >= jumpAt ? 1.0 : 0.0;
        const Signal first = analogDriveRender(1, in, automation);
        for (const size_t block : {size_t{7}, size_t{512}})
        {
            const Signal out = analogDriveRender(block, in, automation);
            double worst = 0.0;
            for (size_t i = 0; i < total; ++i)
                worst = std::max(worst, std::abs(out[i] - first[i]));
            logMessage(String("fx.delay_pro analog drive jump, block ") + String(static_cast<int>(block)) +
                       " against block 1: largest difference " + String(worst, 6));
            expect(worst < 0.001, String("drive jump behaves alike at block ") + String(static_cast<int>(block)));
        }
    }

    void moduleAllocations()
    {
        beginTest(
            "fx.delay_pro: no allocation on the audio thread with every parameter changing every block (both engines)");
        Rack h;
        const char* ids[] = {
            "mix",         "mode",          "time",         "sync",         "feedback",         "stereo",
            "right_ratio", "low_cut",       "high_cut",     "saturation",   "diffusion",        "mod_depth",
            "mod_rate",    "time_change",   "freeze",       "duck_depth",   "duck_threshold",   "duck_release",
            "duck_source", "analog_stages", "analog_drive", "analog_noise", "analog_compander", "analog_breathing"};
        std::vector<int> index;
        for (const char* id : ids)
            index.push_back(Rack::ix(std::string("fx_delay_pro_") + id));
        const auto at = [&](const char* id)
        {
            return static_cast<size_t>(index[static_cast<size_t>(
                std::find_if(std::begin(ids), std::end(ids), [&](const char* s) { return std::strcmp(s, id) == 0; }) -
                std::begin(ids))]);
        };
        const size_t iMix = at("mix"), iMode = at("mode"), iTime = at("time"), iSync = at("sync"), iFb = at("feedback"),
                     iSt = at("stereo"), iRatio = at("right_ratio"), iLo = at("low_cut"), iHi = at("high_cut"),
                     iSat = at("saturation"), iDif = at("diffusion"), iMd = at("mod_depth"), iMr = at("mod_rate"),
                     iTc = at("time_change"), iFr = at("freeze"), iDd = at("duck_depth"), iDt = at("duck_threshold"),
                     iDr = at("duck_release"), iDs = at("duck_source"), iSg = at("analog_stages"), iDrv = at("analog_drive"),
                     iNz = at("analog_noise"), iCp = at("analog_compander"), iBr = at("analog_breathing");
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
            v[iMix] = static_cast<float>(block % 11) / 10.0f;
            v[iMode] = (block / 200) % 2 == 1 ? 1.0f : 0.0f;
            v[iTime] = 5.0f + static_cast<float>(block % 97) * 20.0f;
            v[iSync] = block % 400 > 380 ? static_cast<float>(block % 14) : 0.0f;
            v[iFb] = static_cast<float>(block % 13) / 12.0f;
            v[iSt] = static_cast<float>((block / 50) % 3);
            v[iRatio] = 0.25f + static_cast<float>(block % 8) * 0.25f;
            v[iLo] = 20.0f + static_cast<float>(block % 40) * 20.0f;
            v[iHi] = 600.0f + static_cast<float>(block % 36) * 520.0f;
            v[iSat] = static_cast<float>(block % 6) / 5.0f;
            v[iDif] = static_cast<float>(block % 5) / 4.0f;
            v[iMd] = static_cast<float>(block % 7);
            v[iMr] = 0.1f + static_cast<float>(block % 9);
            v[iTc] = static_cast<float>((block / 30) % 2);
            v[iFr] = block % 350 > 330 ? 1.0f : 0.0f;
            v[iDd] = static_cast<float>(block % 4) * 6.0f;
            v[iDt] = -60.0f + static_cast<float>(block % 20) * 3.0f;
            v[iDr] = 20.0f + static_cast<float>(block % 30) * 60.0f;
            v[iDs] = static_cast<float>(block % 2);
            v[iSg] = static_cast<float>((block / 100) % 5);
            v[iDrv] = -12.0f + static_cast<float>(block % 37);
            v[iNz] = -90.0f + static_cast<float>(block % 50);
            v[iCp] = static_cast<float>(block % 2);
            v[iBr] = static_cast<float>(block % 5) / 4.0f;
            v[static_cast<size_t>(Rack::ix("fx_delay_pro_on"))] = block % 700 > 680 ? 0.0f : 1.0f;
            h.run(l.data(), r.data(), 64);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
    }

    void moduleCpu()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest(
            "fx.delay_pro: CPU, percent of one core per stereo instance at 48 kHz (plan: 0.3 % Digital, 0.6 % Analog)");
        const std::vector<std::pair<std::string, float>> digital = {
            {"fx_delay_pro_mix", 0.5f},       {"fx_delay_pro_feedback", 0.5f},   {"fx_delay_pro_stereo", 1.0f},
            {"fx_delay_pro_diffusion", 0.5f}, {"fx_delay_pro_mod_depth", 3.0f},  {"fx_delay_pro_high_cut", 6000.0f},
            {"fx_delay_pro_low_cut", 100.0f}, {"fx_delay_pro_duck_depth", 12.0f}};
        std::vector<std::pair<std::string, float>> plain = {{"fx_delay_pro_mix", 0.5f}, {"fx_delay_pro_feedback", 0.4f}};
        auto analog = digital;
        analog.push_back({"fx_delay_pro_mode", 1.0f});
        analog.push_back({"fx_delay_pro_time", 12.0f}); // 1024 stages: the clock is at 43 kHz
        auto analogPlain = plain;
        analogPlain.push_back({"fx_delay_pro_mode", 1.0f});
        analogPlain.push_back({"fx_delay_pro_time", 375.0f});
        auto analogFull = analog;
        analogFull.push_back({"fx_delay_pro_analog_stages", 0.0f});
        analogFull.push_back({"fx_delay_pro_time", 3.0f}); // 256 stages at 3 ms: the highest clock
        const double d = cpuPercent(digital), dp = cpuPercent(plain), a = cpuPercent(analog), ap = cpuPercent(analogPlain),
                     af = cpuPercent(analogFull);
        logMessage(String("CPU Digital: all features ") + String(d, 3) + " %, defaults " + String(dp, 3) + " %");
        logMessage(String("CPU Analog: 1024 stages 12 ms all features ") + String(a, 3) + " %, 375 ms defaults " +
                   String(ap, 3) + " %, 256 stages 3 ms all features " + String(af, 3) + " %");
        expect(d <= 0.3 * 1.2, "Digital within 0.3 % (20 % measurement margin)");
        expect(std::max({a, ap, af}) <= 0.6 * 1.2, "Analog within 0.6 % (20 % measurement margin)");
#endif
    }
};

static DelayProTests delayProTests;
