// SPDX-License-Identifier: MIT
// fx.transient and fx.deesser (FX13, docs/plans/ENGINE-FX-PLAN.md section 3.2): accept rows of the plan as numbers. The
// printed lines are what docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "CpuTiming.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/fx/Dynamics.h>
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
// The sample rate of the rack; runTest repeats the main accept rows at 44.1 / 96 / 192 kHz by changing it.
double kRate = 48000.0;

// A length given in samples at 48 kHz, in samples at the rack rate.
size_t S(size_t samplesAt48k)
{
    return static_cast<size_t>(std::llround(static_cast<double>(samplesAt48k) * kRate / 48000.0));
}

// Latency of the de-esser at the default lookahead (1 ms) and of the transient shaper with 2 ms of lookahead, in samples.
int deesserLatency()
{
    return 2 * static_cast<int>(std::lround(0.001 * kRate));
}
int twoMsSamples()
{
    return static_cast<int>(std::lround(0.002 * kRate));
}

std::string rateTag()
{
    if (kRate == 48000.0)
        return std::string();
    char tag[32];
    std::snprintf(tag, sizeof(tag), "[%g kHz] ", kRate / 1000.0);
    return tag;
}

// One effect through the rack, every other effect off.
struct Rack
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values;
    std::vector<float> legacy;
    std::string prefix;
    explicit Rack(const std::string& name, double rate = kRate) : prefix("fx_" + name + "_")
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set(prefix + "on", 1.0f);
    }
    static int ix(const std::string& id) { return ParameterMap::indexOf(id); }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ix(id))] = v; }
    void p(const char* name, float v) { set(prefix + name, v); }
    void run(float* l, float* r, int n, const float* auxL = nullptr, const float* auxR = nullptr)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, auxL, auxR);
    }
};

using Floats = std::vector<float>;

// Mono render through the rack in blocks of 128 (right channel copies the left); returns the left output.
template <class Fn> Floats render(Rack& h, const Floats& in, Fn&& atBlock)
{
    Floats l = in, r = in;
    for (size_t pos = 0; pos < l.size(); pos += 128)
    {
        atBlock(pos);
        h.run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(128, l.size() - pos)));
    }
    return l;
}
Floats render(Rack& h, const Floats& in)
{
    return render(h, in, [](size_t) {});
}

Signal toSignal(const Floats& x)
{
    return Signal(x.begin(), x.end());
}

double peakOf(const Floats& x, size_t from = 0, size_t to = 0)
{
    if (to == 0 || to > x.size())
        to = x.size();
    double m = 0.0;
    for (size_t i = from; i < to; ++i)
        m = std::max(m, static_cast<double>(std::abs(x[i])));
    return m;
}

double rmsOf(const Floats& x, size_t from, size_t to)
{
    double s = 0.0;
    for (size_t i = from; i < to; ++i)
        s += static_cast<double>(x[i]) * x[i];
    return std::sqrt(s / static_cast<double>(to - from));
}

// A hit: silence, then white noise under an exponential decay with time constant tauMs, then silence.
Floats hit(double amp, double tauMs, size_t lead = 9600, size_t length = 48000, size_t total = 96000, std::uint64_t seed = 7)
{
    lead = S(lead);
    length = S(length);
    total = S(total);
    Floats x(total, 0.0f);
    Random rng(seed);
    for (size_t i = 0; i < length; ++i)
        x[lead + i] = static_cast<float>(amp * rng.gaussian() * std::exp(-static_cast<double>(i) / (tauMs * 0.001 * kRate)));
    return x;
}

// ---- de-esser signals
struct Stereo
{
    Floats l, r;
};

// Stereo through the rack in blocks of 128; `atBlock(position)` may change parameters and read meters. The right channel of
// a mono input is a copy of the left.
template <class Fn> Stereo renderStereo(Rack& h, Stereo io, Fn&& atBlock)
{
    for (size_t pos = 0; pos < io.l.size(); pos += 128)
    {
        atBlock(pos);
        h.run(io.l.data() + pos, io.r.data() + pos, static_cast<int>(std::min<size_t>(128, io.l.size() - pos)));
    }
    return io;
}

// Vowel-like tone: 140 Hz fundamental and 14 harmonics with 1/k amplitudes (nothing above 2 kHz), scaled to the RMS.
Floats vowel(size_t n, double rmsDb)
{
    Signal s(n, 0.0);
    for (int k = 1; k <= 14; ++k)
        for (size_t i = 0; i < n; ++i)
            s[i] += std::sin(2.0 * kPi * 140.0 * k * static_cast<double>(i) / kRate + 0.7 * k) / k;
    double e = 0.0;
    for (const double v : s)
        e += v * v;
    const double gain = dbToAmplitude(rmsDb) / std::sqrt(e / static_cast<double>(n));
    Floats out(n);
    for (size_t i = 0; i < n; ++i)
        out[i] = static_cast<float>(s[i] * gain);
    return out;
}

// Noise with a Gaussian band (sigma octaves around centreHz), scaled to the RMS over the whole signal, then gated by bursts
// of `burstMs` every `periodMs` with 5 ms raised-cosine edges (the first burst starts at `startMs`).
Floats sibilant(size_t n, double rmsDb, double sigmaOctaves, double startMs, double burstMs, double periodMs,
                std::uint64_t seed = 11, double centreHz = 6500.0)
{
    Random rng(seed);
    Signal noise(n);
    for (auto& v : noise)
        v = rng.gaussian();
    Signal band = detail::bandFilter(noise, kRate, centreHz, sigmaOctaves);
    double e = 0.0;
    for (const double v : band)
        e += v * v;
    const double gain = dbToAmplitude(rmsDb) / std::sqrt(e / static_cast<double>(n));
    Floats out(n, 0.0f);
    const double edge = 0.005 * kRate;
    for (size_t i = 0; i < n; ++i)
    {
        const double t = static_cast<double>(i) / kRate * 1000.0 - startMs;
        if (t < 0.0)
            continue;
        const double inPeriod = std::fmod(t, periodMs);
        if (inPeriod >= burstMs)
            continue;
        const double pos = inPeriod * 0.001 * kRate, len = burstMs * 0.001 * kRate;
        double env = 1.0;
        if (pos < edge)
            env = 0.5 - 0.5 * std::cos(kPi * pos / edge);
        else if (len - pos < edge)
            env = 0.5 - 0.5 * std::cos(kPi * (len - pos) / edge);
        out[i] = static_cast<float>(band[i] * gain * env);
    }
    return out;
}

Floats sum(const Floats& a, const Floats& b)
{
    Floats out(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        out[i] = a[i] + b[i];
    return out;
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

#if !defined(CRUCIBLE_TEST_UNTIMED)
// Percent of one core per stereo instance at 48 kHz, blocks of 32, minus the rack with the effect off; best of repeats.
double cpuPercent(const std::string& name, const std::vector<std::pair<std::string, float>>& setup,
                  double* loadOut = nullptr)
{
    const size_t n = static_cast<size_t>(6.0 * kRate);
    Random rng(5);
    Floats src(n);
    for (auto& v : src)
        v = static_cast<float>(0.25 * rng.gaussian());
    const auto run = [&](bool enabled)
    {
        Rack h(name);
        h.set("fx_" + name + "_on", enabled ? 1.0f : 0.0f);
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

class TransientDeEsserTests : public UnitTest
{
public:
    TransientDeEsserTests() : UnitTest("TransientDeEsser", "Effects") {}

    void runTest() override
    {
        transientBasics();
        transientLevelIndependence();
        transientPeakChange();
        transientSteadySine();
        transientAllocations();
        blockSizeZipper();
        transientCpu();
        deesserBasics();
        deesserNull();
        lookaheadLatencyPublished();
        deesserOutsideBand();
        deesserTracking();
        deesserRelative();
        deesserWideBand();
        deesserListenAndChannels();
        deesserAllocations();
        deesserCpu();
        // The main accept rows again at the other sample rates (the rows above are the 48 kHz run).
        for (const double rate : {44100.0, 96000.0, 192000.0})
        {
            kRate = rate;
            transientBasics();
            transientPeakChange();
            transientSteadySine();
            deesserNull();
            deesserOutsideBand();
            deesserTracking();
            deesserRelative();
            deesserWideBand();
            deesserListenAndChannels();
        }
        kRate = 48000.0;
    }

private:
    // ---- smoothing that does not depend on the host block size
    static Signal renderAutomated(const char* module, const std::vector<std::pair<std::string, float>>& setup,
                                  const std::string& id, float lo, float hi, size_t block, const Signal& in,
                                  const Signal& automation)
    {
        Rack h(module);
        for (const auto& kv : setup)
            h.set(kv.first, kv.second);
        std::vector<float> l(in.begin(), in.end()), r = l;
        for (size_t pos = 0; pos < l.size(); pos += block)
        {
            const size_t end = std::min(pos + block, l.size());
            h.set(id, static_cast<float>(lo + (hi - lo) * automation[end - 1]));
            h.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
        }
        return Signal(l.begin(), l.end());
    }

    void blockSizeZipper()
    {
        beginTest(rateTag() + "fx.transient output and fx.deesser frequency / width: ramps and a 2 Hz square leave no step "
                              "above -90 dB (or the module's own held-setting residue) at host blocks of 1, 7 and 512");
        struct Control
        {
            const char* name;
            const char* module;
            const char* id;
            float lo, hi;
            double carrierHz;
            std::vector<std::pair<std::string, float>> setup;
        };
        const std::vector<std::pair<std::string, float>> deesserSetup = {{"fx_deesser_threshold_mode", 1.0f},
                                                                         {"fx_deesser_threshold", -50.0f},
                                                                         {"fx_deesser_ratio", 8.0f},
                                                                         {"fx_deesser_range", 12.0f}};
        std::vector<std::pair<std::string, float>> widthSetup = deesserSetup;
        widthSetup.emplace_back("fx_deesser_frequency", 4000.0f);
        const std::vector<Control> controls = {
            {"transient output", "transient", "fx_transient_output", -12.0f, 12.0f, 1000.0, {{"fx_transient_guard", 0.0f}}},
            {"deesser frequency", "deesser", "fx_deesser_frequency", 2000.0f, 16000.0f, 6000.0, deesserSetup},
            {"deesser width", "deesser", "fx_deesser_width", 0.5f, 3.0f, 6000.0, widthSetup}};
        for (const Control& c : controls)
            for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
            {
                const auto render = [&](const Signal& in, const Signal& automation)
                { return renderAutomated(c.module, c.setup, c.id, c.lo, c.hi, block, in, automation); };
                const ZipperResult z = zipperTest(render, kRate, c.carrierHz, -6.0);
                // The de-esser's gain computer runs at a quarter of the rate and leaves a residue of its own on a held
                // setting: that is the floor the automated runs are compared with.
                const size_t held = static_cast<size_t>(0.6 * kRate);
                const Signal hold = render(sineWave(held, c.carrierHz, kRate, dbToAmplitude(-6.0)), Signal(held, 1.0));
                const double floorDb = clickTest(hold, c.carrierHz, kRate, static_cast<size_t>(0.1 * kRate)).stepDb;
                const double limit = std::max(-90.0, floorDb + 3.0);
                logMessage(String("fx.") + c.module + " " + c.name + ", host block " + String(static_cast<int>(block)) +
                           ": ramp step " + String(z.rampDb, 1) + " dB, 2 Hz square " + String(z.squareDb, 1) +
                           " dB, held-setting floor " + String(floorDb, 1) + " dB");
                expect(z.rampDb <= limit, String(c.name) + " ramp zipper at block " + String(static_cast<int>(block)));
                expect(z.squareDb <= limit, String(c.name) + " square zipper at block " + String(static_cast<int>(block)));
            }

        // A jump at a sample where every block size starts a block (3584 = 7 x 512) gives the same output within 0.001.
        for (const Control& c : controls)
        {
            const size_t total = 24000, jumpAt = 3584 * 3;
            Signal in(total), automation(total);
            for (size_t i = 0; i < total; ++i)
            {
                in[i] = 0.5 * std::sin(2.0 * kPi * c.carrierHz * static_cast<double>(i) / kRate);
                automation[i] = i >= jumpAt ? 1.0 : 0.0;
            }
            Signal first;
            for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
            {
                const Signal out = renderAutomated(c.module, c.setup, c.id, c.lo, c.hi, block, in, automation);
                if (first.empty())
                {
                    first = out;
                    continue;
                }
                double worst = 0.0;
                for (size_t i = 0; i < total; ++i)
                    worst = std::max(worst, std::abs(out[i] - first[i]));
                logMessage(String("fx.") + c.module + " " + c.name + " jump, block " + String(static_cast<int>(block)) +
                           " against block 1: largest difference " + String(worst, 6));
                expect(worst < 0.001, String(c.name) + " jump behaves alike at block " + String(static_cast<int>(block)));
            }
        }
    }

    // ---- fx.transient
    void transientBasics()
    {
        beginTest(rateTag() +
                  "fx.transient: current, spelled-out ids, neutral setting is bit-exact, lookahead is reported as latency");
        const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.transient");
        expect(d != nullptr, "registered");
        if (d == nullptr)
            return;
        expect(d->status == ModuleStatus::Current, "status Current");
        bool names = true;
        for (const auto& q : d->params)
            names = names && (q.id.rfind("fx_transient_", 0) == 0 || q.id == "fx_pos_transient");
        expect(names, "every id is fx_transient_* (or the slot)");
        logMessage(String("fx.transient parameters: ") + String(static_cast<int>(d->params.size())));
        Rack h("transient");
        auto* lat = h.rack.findCapability<LatencyReporter>("fx.transient");
        expect(lat != nullptr, "latency capability");
        // Peaks stay below the clip guard's knee (-1 dBFS), above which the guard bends the signal even at the neutral
        // setting.
        const Floats in = hit(0.15, 40.0);
        const Floats out = render(h, in);
        expect(out == in, "attack 0 dB, sustain 0 dB: output equals input bit for bit");
        if (lat != nullptr)
        {
            expectEquals(lat->latencySamples(), 0, "latency without lookahead");
            h.p("lookahead", 2.0f);
            h.p("attack", 6.0f);
            render(h, Floats(256, 0.0f));
            expectEquals(lat->latencySamples(), twoMsSamples(), "latency with 2 ms lookahead");
            expect(lat->takeLatencyChanged(), "latency change is announced");
        }
        Rack g("transient");
        g.p("lookahead", 2.0f);
        const Floats delayed = render(g, in);
        double worst = 0.0;
        for (size_t i = static_cast<size_t>(twoMsSamples()); i < in.size(); ++i)
            worst = std::max(worst, static_cast<double>(std::abs(delayed[i] - in[i - static_cast<size_t>(twoMsSamples())])));
        expect(worst == 0.0, "neutral with lookahead: input delayed by the 2 ms latency, bit for bit");
    }

    void transientLevelIndependence()
    {
        beginTest(rateTag() + "fx.transient: the same gain curve at -40 and -6 dBFS input within 0.5 dB");
        double worstAll = 0.0;
        for (const double tau : {15.0, 60.0})
        {
            const double quiet = dbToAmplitude(-40.0), loud = dbToAmplitude(-6.0);
            const auto gainCurve = [&](double amp)
            {
                Rack h("transient");
                h.p("attack", 9.0f);
                h.p("sustain", -9.0f);
                h.p("guard", 0.0f);
                const Floats in = hit(amp, tau);
                const Floats out = render(h, in);
                const double peak = peakOf(in);
                Signal g(in.size(), 0.0);
                for (size_t i = 0; i < in.size(); ++i)
                    g[i] = std::abs(in[i]) > 0.05 * peak ? amplitudeToDb(static_cast<double>(out[i]) / in[i]) : 1.0e9;
                return g;
            };
            const Signal a = gainCurve(quiet), b = gainCurve(loud);
            double worst = 0.0, span = 0.0;
            for (size_t i = 0; i < a.size(); ++i)
                if (a[i] < 1.0e8 && b[i] < 1.0e8)
                {
                    worst = std::max(worst, std::abs(a[i] - b[i]));
                    span = std::max(span, std::abs(b[i]));
                }
            logMessage(String("gain curve -40 vs -6 dBFS, tau ") + String(tau, 0) + " ms: worst difference " +
                       String(worst, 4) + " dB, curve spans up to " + String(span, 2) + " dB");
            worstAll = std::max(worstAll, worst);
            expect(span > 3.0, "the curve is not trivial");
        }
        expect(worstAll <= 0.5, "gain curve difference within 0.5 dB");
    }

    void transientPeakChange()
    {
        beginTest(rateTag() + "fx.transient: on a noise burst with an exponential decay the peak change equals the attack "
                              "setting within 1 dB");
        double worst = 0.0;
        // A cut lasts as long as the follower ratio stays above the dead zone, so on a decay much slower than the slow
        // follower (40 ms and up) the later, quieter part of the burst is cut less than its onset: negative settings use 10
        // and 20 ms.
        for (const double setting : {-12.0, -6.0, 6.0, 12.0, 15.0})
            for (const double tau : {setting < 0.0 ? 10.0 : 20.0, setting < 0.0 ? 20.0 : 80.0})
            {
                Rack h("transient");
                h.p("attack", static_cast<float>(setting));
                h.p("lookahead", 2.0f);
                h.p("guard", 0.0f);
                const Floats in = hit(0.1, tau);
                const Floats out = render(h, in);
                const double change = amplitudeToDb(peakOf(out) / peakOf(in));
                worst = std::max(worst, std::abs(change - setting));
                logMessage(String("attack ") + String(setting, 0) + " dB, tau " + String(tau, 0) + " ms: peak change " +
                           String(change, 2) + " dB");
            }
        expect(worst <= 1.0, String("worst deviation ") + String(worst, 3) + " dB");

        beginTest(rateTag() + "fx.transient: sustain moves the tail toward its setting, attack 0 leaves the onset alone");
        for (const double setting : {-12.0, 9.0})
        {
            Rack h("transient");
            h.p("sustain", static_cast<float>(setting));
            h.p("lookahead", 2.0f);
            h.p("guard", 0.0f);
            const Floats in = hit(0.1, 80.0);
            const Floats out = render(h, in);
            const size_t lat = static_cast<size_t>(twoMsSamples());
            const size_t onset = S(9600) + lat;
            const double onsetChange =
                amplitudeToDb(peakOf(out, onset - lat, onset + lat) / peakOf(in, onset - lat, onset + lat));
            const size_t t0 = S(9600) + S(6000), t1 = S(9600) + S(9600); // 125 to 200 ms into the decay
            const double tailChange = amplitudeToDb(rmsOf(out, t0 + lat, t1 + lat) / rmsOf(in, t0, t1));
            logMessage(String("sustain ") + String(setting, 0) + " dB: onset " + String(onsetChange, 2) + " dB, tail " +
                       String(tailChange, 2) + " dB");
            expect(std::abs(onsetChange) <= 1.5, "onset nearly untouched");
            expect(tailChange * setting > 0.0 && std::abs(tailChange) >= 0.4 * std::abs(setting),
                   "tail moves toward the setting by at least 40 % of it");
        }
    }

    void transientSteadySine()
    {
        beginTest(rateTag() + "fx.transient: a steady sine changes by less than 0.1 dB with THD at most -80 dB (attack and "
                              "sustain at +-15 dB)");
        double worstChange = 0.0, worstThd = -300.0;
        for (const double hz : {100.0, 1000.0, 5000.0})
            for (const double amp : {-40.0, -12.0})
                for (const float setting : {15.0f, -15.0f})
                {
                    Rack h("transient");
                    h.p("attack", setting);
                    h.p("sustain", setting);
                    const size_t n = S(96000);
                    Floats in(n);
                    for (size_t i = 0; i < n; ++i)
                        in[i] = static_cast<float>(dbToAmplitude(amp) *
                                                   std::sin(2.0 * kPi * hz * static_cast<double>(i) / kRate));
                    // Fade the sine in over 0.5 s, as a held note: the follower ratio stays inside the dead zone.
                    for (size_t i = 0; i < S(24000); ++i)
                        in[i] *= static_cast<float>(
                            0.5 - 0.5 * std::cos(kPi * static_cast<double>(i) / static_cast<double>(S(24000))));
                    const Floats out = render(h, in);
                    const double change = amplitudeToDb(rmsOf(out, S(48000), n) / rmsOf(in, S(48000), n));
                    const Signal tail = toSignal(Floats(out.begin() + static_cast<long>(S(48000)), out.end()));
                    const HarmonicProfile hp = harmonicProfile(tail, kRate, hz, 12);
                    logMessage(String(hz, 0) + " Hz " + String(amp, 0) + " dB, +-" +
                               String(static_cast<double>(std::abs(setting)), 0) + " dB: change " + String(change, 5) +
                               " dB, THD " + String(hp.thdDb, 1) + " dB");
                    worstChange = std::max(worstChange, std::abs(change));
                    worstThd = std::max(worstThd, hp.thdDb);
                }
        logMessage(String("steady sine: worst level change ") + String(worstChange, 5) + " dB, worst THD " +
                   String(worstThd, 1) + " dB");
        expect(worstChange < 0.1, "level change below 0.1 dB");
        expect(worstThd <= -80.0, "THD at most -80 dB");
    }

    void transientAllocations()
    {
        beginTest(rateTag() + "fx.transient: no allocation on the audio thread with every parameter changing every block");
        Rack h("transient");
        const char* ids[] = {"mix", "attack", "sustain", "speed", "lookahead", "guard", "output"};
        int index[7];
        for (int i = 0; i < 7; ++i)
            index[i] = Rack::ix(std::string("fx_transient_") + ids[i]);
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
            v[static_cast<size_t>(index[0])] = static_cast<float>(block % 11) / 10.0f;
            v[static_cast<size_t>(index[1])] = -15.0f + static_cast<float>(block % 31);
            v[static_cast<size_t>(index[2])] = 15.0f - static_cast<float>(block % 31);
            v[static_cast<size_t>(index[3])] = 0.25f + static_cast<float>(block % 16) * 0.25f;
            v[static_cast<size_t>(index[4])] = static_cast<float>(block % 6);
            v[static_cast<size_t>(index[5])] = static_cast<float>(block % 2);
            v[static_cast<size_t>(index[6])] = -12.0f + static_cast<float>(block % 25);
            h.run(l.data(), r.data(), 64);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
    }

    void transientCpu()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest(rateTag() + "fx.transient: CPU, percent of one core per stereo instance at 48 kHz (plan: 0.1 %)");
        double load = 0.0;
        const double c = cpuPercent(
            "transient", {{"fx_transient_attack", 6.0f}, {"fx_transient_sustain", -6.0f}, {"fx_transient_lookahead", 2.0f}},
            &load);
        logMessage(String("CPU fx.transient: ") + String(c, 3) + " % (best of 8, load average " + String(load, 1) + ")");
        expect(c <= 0.1 * 1.2, "within 0.1 % (20 % measurement margin)");
#endif
    }

    // ---- fx.deesser
    static Floats speech(size_t n, double vowelDb, double sibilantDb, double sigma = 0.12)
    {
        return sum(vowel(n, vowelDb), sibilant(n, sibilantDb, sigma, 200.0, 300.0, 1000.0));
    }
    // Zero-phase Gaussian band (half an octave sigma) around the sibilant centre: the level the detector reads on
    // band-limited noise.
    static Floats gaussianBand(const Floats& x)
    {
        const Signal y = detail::bandFilter(toSignal(x), kRate, 6500.0, 0.5);
        return Floats(y.begin(), y.end());
    }
    static double rmsDbOf(const Floats& x, size_t from, size_t to) { return amplitudeToDb(rmsOf(x, from, to)); }

    void deesserBasics()
    {
        beginTest(rateTag() + "fx.deesser: current, spelled-out ids, registered last, capabilities");
        const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.deesser");
        expect(d != nullptr, "registered");
        if (d == nullptr)
            return;
        expect(d->status == ModuleStatus::Current, "status Current");
        bool names = true;
        for (const auto& q : d->params)
            names = names && (q.id.rfind("fx_deesser_", 0) == 0 || q.id == "fx_pos_deesser");
        expect(names, "every id is fx_deesser_* (or the slot)");
        logMessage(String("fx.deesser parameters: ") + String(static_cast<int>(d->params.size())));
        {
            const auto effects = ModuleRegistry::instance().ofKind(ModuleKind::Effect);
            expect(effects.size() >= 9 && effects[effects.size() - 9]->id == "fx.deesser",
                   "registered after every other effect but the FX11 modules, fx.exciter, fx.saturator, fx.shaper, fx.multiband_drive and fx.cabinet");
        }
        Rack h("deesser");
        expect(h.rack.findCapability<LatencyReporter>("fx.deesser") != nullptr, "latency capability");
        expect(h.rack.findCapability<GainReductionMeter>("fx.deesser") != nullptr, "gain reduction capability");
    }

    void deesserNull()
    {
        beginTest(rateTag() +
                  "fx.deesser split-band: with no reduction the output nulls against the input minus latency to -100 dB");
        const size_t n = S(96000);
        Floats in = sum(speech(n, -24.0, -14.0, 0.35), vowel(n, -40.0));
        Random rng(2);
        for (auto& v : in)
            v += static_cast<float>(0.01 * rng.gaussian());
        double worst = -300.0;
        for (const float lookahead : {0.0f, 1.0f, 2.0f})
        {
            Rack h("deesser");
            h.p("lookahead", lookahead);
            h.p("range", 0.0f);
            const Floats out = render(h, in);
            const int delay =
                static_cast<int>(std::lround(0.001 * kRate)) + static_cast<int>(std::lround(lookahead * 0.001 * kRate));
            const double null = nullTest(toSignal(in), toSignal(out), delay);
            logMessage(String("lookahead ") + String(static_cast<double>(lookahead), 0) + " ms (latency " + String(delay) +
                       " samples): null " + String(null, 1) + " dB");
            worst = std::max(worst, null);
            auto* lat = h.rack.findCapability<LatencyReporter>("fx.deesser");
            expectEquals(lat != nullptr ? lat->latencySamples() : -1, delay, "reported latency");
        }
        expect(worst <= -100.0, "null at most -100 dB");
    }

    // Delay in samples at which the output best matches the input (cross-correlation), 0..1000.
    static int measuredDelay(const Floats& in, const Floats& out)
    {
        int best = 0;
        double bestValue = -1.0e30;
        for (int lag = 0; lag <= 1000; ++lag)
        {
            double acc = 0.0;
            for (size_t i = 2000; i < in.size(); ++i)
                acc += static_cast<double>(out[i]) * in[i - static_cast<size_t>(lag)];
            if (acc > bestValue)
            {
                bestValue = acc;
                best = lag;
            }
        }
        return best;
    }

    void lookaheadLatencyPublished()
    {
        beginTest(rateTag() +
                  "fx.deesser / fx.transient: the full latency (FIR half length + lookahead) is reported from prepare, "
                  "constant across enable / disable, and equals the measured delay at 44.1 / 48 / 96 / 192 kHz");
        Random rng(5);
        Floats noise(12000);
        for (auto& v : noise)
            v = static_cast<float>(0.2 * rng.gaussian());
        for (const char* name : {"deesser", "transient"})
            for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
            {
                Rack h(name, rate);
                if (std::string(name) == "deesser")
                    h.p("range", 0.0f);
                auto* lat = h.rack.findCapability<LatencyReporter>(std::string("fx.") + name);
                if (lat == nullptr)
                {
                    expect(false, "latency capability");
                    continue;
                }
                const std::string tag = std::string(name) + " at " + std::to_string(static_cast<int>(rate)) + " Hz: ";
                const int half =
                    std::string(name) == "deesser" ? std::max(8, static_cast<int>(std::lround(0.001 * rate))) : 0;
                const int lookahead = std::string(name) == "deesser" ? static_cast<int>(std::lround(0.001 * rate)) : 0;
                const int before = lat->latencySamples();
                expectEquals(before, half + lookahead, tag + "latency after prepare (default lookahead)");
                const Floats out = render(h, noise);
                expectEquals(measuredDelay(noise, out), before, tag + "measured delay equals the reported latency");
                expectEquals(lat->latencySamples(), before, tag + "unchanged by the first block");
                expect(!lat->takeLatencyChanged(), tag + "no change announced by the first block");
                // Disable (the rack calls deactivate), re-enable: the latency is the same before any block.
                h.set(h.prefix + "on", 0.0f);
                render(h, Floats(512, 0.0f));
                expectEquals(lat->latencySamples(), before, tag + "latency while disabled");
                h.set(h.prefix + "on", 1.0f);
                expectEquals(lat->latencySamples(), before, tag + "latency after the toggle, before the block");
                const Floats again = render(h, noise);
                expectEquals(measuredDelay(noise, again), before, tag + "measured delay after the toggle");
                expect(!lat->takeLatencyChanged(), tag + "no change announced by the toggle");
                // A non-default lookahead set by the host survives the toggle.
                h.p("lookahead", 2.0f);
                render(h, noise);
                const int two = lat->latencySamples();
                expectEquals(two, half + static_cast<int>(std::lround(0.002 * rate)), tag + "latency with 2 ms lookahead");
                h.set(h.prefix + "on", 0.0f);
                render(h, Floats(512, 0.0f));
                h.set(h.prefix + "on", 1.0f);
                expectEquals(lat->latencySamples(), two, tag + "2 ms lookahead latency kept across the toggle");
                lat->takeLatencyChanged();
                const Floats third = render(h, noise);
                expectEquals(measuredDelay(noise, third), two, tag + "measured delay with 2 ms after the toggle");
                expect(!lat->takeLatencyChanged(), tag + "no second change announced");
            }
    }

    void deesserOutsideBand()
    {
        beginTest(
            rateTag() +
            "fx.deesser split-band: outside the band the output changes by at most 0.1 dB while 18 dB of reduction happens");
        const size_t n = S(96000);
        double worst = 0.0, deepest = 0.0;
        for (const double hz : {250.0, 500.0, 1000.0})
        {
            Rack h("deesser");
            h.p("threshold_mode", 1.0f);
            h.p("threshold", -50.0f);
            h.p("ratio", 100.0f);
            h.p("range", 18.0f);
            const Floats in = sum(tone(n, hz, 0.1), sibilant(n, -12.0, 0.2, 200.0, 400.0, 600.0));
            auto* meter = h.rack.findCapability<GainReductionMeter>("fx.deesser");
            double gr = 0.0;
            const Stereo out = renderStereo(h, {in, in},
                                            [&](size_t pos)
                                            {
                                                if (pos >= S(9600) && meter != nullptr)
                                                    gr = std::max(gr, static_cast<double>(meter->gainReductionDb(0)));
                                            });
            const size_t len = S(38400), lat = static_cast<size_t>(deesserLatency());
            const double change =
                amplitudeToDb(amplitudeAt(out.l, S(9600) + lat, len, hz) / amplitudeAt(in, S(9600), len, hz));
            logMessage(String("tone ") + String(hz, 0) + " Hz: change " + String(change, 4) + " dB with up to " +
                       String(gr, 1) + " dB reduction");
            worst = std::max(worst, std::abs(change));
            deepest = std::max(deepest, gr);
        }
        expect(deepest >= 15.0, "the reduction really happens");
        expect(worst <= 0.1, "change at most 0.1 dB");
    }

    // Mean of the gain reduction meter over the blocks that start in [from, to).
    template <class Fn>
    double meanReduction(Rack& h, const Stereo& in, size_t from, size_t to, Fn&& atBlock, Stereo* out = nullptr)
    {
        auto* meter = h.rack.findCapability<GainReductionMeter>("fx.deesser");
        double total = 0.0;
        int count = 0;
        Stereo result = renderStereo(h, in,
                                     [&](size_t pos)
                                     {
                                         atBlock(pos);
                                         if (pos >= from + 128 && pos < to && meter != nullptr)
                                         {
                                             total += meter->gainReductionDb(0);
                                             ++count;
                                         }
                                     });
        if (out != nullptr)
            *out = std::move(result);
        return count > 0 ? total / count : 0.0;
    }

    void deesserTracking()
    {
        beginTest(
            rateTag() +
            "fx.deesser: reduction tracks the threshold within 0.5 dB on band-limited noise bursts over a vowel-like tone");
        const size_t n = S(96000);
        double worstMeter = 0.0, worstOutput = 0.0;
        for (const double sigma : {0.12, 0.35})
            for (const double burstDb : {-20.0, -14.0})
                for (const double ratioValue : {100.0, 6.0})
                {
                    const double thr = -30.0, kneeDb = 2.0;
                    const Floats in = speech(n, -26.0, burstDb, sigma);
                    Rack h("deesser");
                    h.p("threshold_mode", 1.0f);
                    h.p("threshold", static_cast<float>(thr));
                    h.p("ratio", static_cast<float>(ratioValue));
                    h.p("range", 18.0f);
                    h.p("attack", 20.0f);
                    h.p("release", 20.0f);
                    Stereo out;
                    const double meterMean = meanReduction(h, {in, in}, S(14400), S(23040), [](size_t) {}, &out);
                    const size_t from = S(14400), to = S(23040),
                                 lat = static_cast<size_t>(deesserLatency()); // 300 to 480 ms
                    const Floats bandIn = gaussianBand(in), bandOut = gaussianBand(out.l);
                    const double levelIn = rmsDbOf(bandIn, from, to), levelOut = rmsDbOf(bandOut, from + lat, to + lat);
                    const double expected = std::min(18.0, static_cast<double>(crucible::fx::dyn::compressorReduction(
                                                               static_cast<float>(levelIn), static_cast<float>(thr),
                                                               static_cast<float>(ratioValue), static_cast<float>(kneeDb))));
                    const double actual = levelIn - levelOut;
                    logMessage(String("sigma ") + String(sigma, 2) + " oct, band " + String(levelIn, 1) + " dB, ratio " +
                               String(ratioValue, 0) + ": expected " + String(expected, 2) + " dB, meter " +
                               String(meterMean, 2) + " dB, band level drop " + String(actual, 2) + " dB");
                    if (sigma < 0.2)
                        worstMeter = std::max(worstMeter, std::abs(meterMean - expected));
                    if (sigma < 0.2)
                        worstOutput = std::max(worstOutput, std::abs(actual - expected));
                }
        expect(worstMeter <= 0.5, String("meter within 0.5 dB of the static curve, worst ") + String(worstMeter, 3));
        expect(worstOutput <= 0.5, String("band level drop within 0.5 dB of the static curve (1/3 octave noise), worst ") +
                                       String(worstOutput, 3));
    }

    void deesserRelative()
    {
        beginTest(rateTag() +
                  "fx.deesser: the relative threshold does not depend on the level; a vowel alone is not reduced");
        const size_t n = S(96000);
        double reduction[2] = {0.0, 0.0};
        for (int k = 0; k < 2; ++k)
        {
            const double offset = k == 0 ? 0.0 : -26.0;
            const Floats in = speech(n, -22.0 + offset, -18.0 + offset);
            Rack h("deesser");
            h.p("relative", -12.0f);
            h.p("ratio", 100.0f);
            reduction[k] = meanReduction(h, {in, in}, S(14400), S(23040), [](size_t) {});
        }
        logMessage(String("relative threshold -12 dB: reduction ") + String(reduction[0], 3) + " dB at the set level, " +
                   String(reduction[1], 3) + " dB at 26 dB lower");
        expect(reduction[0] > 3.0, "the sibilant is reduced");
        expect(std::abs(reduction[0] - reduction[1]) <= 0.2, "same reduction within 0.2 dB at both levels");
        Rack h("deesser");
        h.p("relative", -12.0f);
        h.p("ratio", 100.0f);
        const double vowelOnly = meanReduction(h, {vowel(n, -22.0), vowel(n, -22.0)}, S(14400), S(23040), [](size_t) {});
        logMessage(String("vowel alone: reduction ") + String(vowelOnly, 4) + " dB");
        expect(vowelOnly < 0.5, "a vowel alone is not reduced");
    }

    void deesserWideBand()
    {
        beginTest(rateTag() + "fx.deesser: wide-band mode lowers the whole signal by the meter's reduction; split-band "
                              "leaves a low tone alone");
        const size_t n = S(96000);
        const Floats in = sum(tone(n, 500.0, 0.1), sibilant(n, -12.0, 0.2, 200.0, 400.0, 600.0));
        double change[2] = {0.0, 0.0}, meterMean[2] = {0.0, 0.0};
        for (int k = 0; k < 2; ++k)
        {
            Rack h("deesser");
            h.p("mode", static_cast<float>(k));
            h.p("threshold_mode", 1.0f);
            h.p("threshold", -30.0f);
            h.p("ratio", 100.0f);
            h.p("release", 400.0f);
            Stereo out;
            meterMean[k] = meanReduction(h, {in, in}, S(14400), S(38400), [](size_t) {}, &out);
            change[k] = amplitudeToDb(amplitudeAt(out.l, S(14400) + static_cast<size_t>(deesserLatency()), S(14400), 500.0) /
                                      amplitudeAt(in, S(14400), S(14400), 500.0));
            logMessage(String(k == 0 ? "split-band" : "wide-band") + ": 500 Hz tone changes " + String(change[k], 2) +
                       " dB, meter " + String(meterMean[k], 2) + " dB");
        }
        expect(std::abs(change[0]) <= 0.1, "split-band leaves the tone");
        expect(meterMean[1] > 5.0 && std::abs(change[1] + meterMean[1]) <= 1.0,
               "wide-band: tone change equals minus the mean reduction within 1 dB");
    }

    void deesserListenAndChannels()
    {
        beginTest(rateTag() +
                  "fx.deesser: listen outputs the detector band; Left mode leaves the right channel exactly alone");
        const size_t n = S(48000);
        const Floats in = speech(n, -24.0, -14.0, 0.35);
        {
            Rack h("deesser");
            h.p("listen", 1.0f);
            h.p("lookahead", 0.0f);
            const Floats two = sum(tone(n, 500.0, 0.2), tone(n, 6500.0, 0.2));
            const Floats out = render(h, two);
            const double inBand = amplitudeToDb(amplitudeAt(out, S(9600), S(9600), 6500.0) / 0.2);
            const double outBand = amplitudeToDb(amplitudeAt(out, S(9600), S(9600), 500.0) / 0.2);
            logMessage(String("listen: 6.5 kHz ") + String(inBand, 3) + " dB, 500 Hz " + String(outBand, 1) + " dB");
            expect(std::abs(inBand) <= 0.1, "the band passes at unity");
            expect(outBand <= -40.0, "below the band at most -40 dB");
        }
        {
            Rack h("deesser");
            h.p("channels", static_cast<float>(crucible::fx::ChannelMode::Left));
            h.p("threshold_mode", 1.0f);
            h.p("threshold", -40.0f);
            h.p("ratio", 100.0f);
            Stereo io{in, sum(in, tone(n, 300.0, 0.05))};
            const Stereo original = io;
            const Stereo out = renderStereo(h, io, [](size_t) {});
            const size_t delay = static_cast<size_t>(deesserLatency());
            double right = 0.0, left = 0.0;
            for (size_t i = delay; i < n; ++i)
            {
                right = std::max(right, static_cast<double>(std::abs(out.r[i] - original.r[i - delay])));
                left = std::max(left, static_cast<double>(std::abs(out.l[i] - original.l[i - delay])));
            }
            expect(right == 0.0, "right channel is the input delayed, bit for bit");
            expect(left > 0.01, "left channel is reduced");
        }
    }

    void deesserAllocations()
    {
        beginTest(rateTag() + "fx.deesser: no allocation on the audio thread with every parameter changing every block");
        Rack h("deesser");
        const char* ids[] = {"mode",      "detector",  "frequency", "width",    "threshold_mode",
                             "threshold", "relative",  "ratio",     "range",    "attack",
                             "release",   "lookahead", "listen",    "channels", "link"};
        int index[15];
        for (int i = 0; i < 15; ++i)
            index[i] = Rack::ix(std::string("fx_deesser_") + ids[i]);
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
            set(0, static_cast<float>(block % 2));
            set(1, static_cast<float>((block / 3) % 2));
            set(2, 2000.0f + static_cast<float>(block % 29) * 480.0f);
            set(3, 0.5f + static_cast<float>(block % 6) * 0.5f);
            set(4, static_cast<float>((block / 5) % 2));
            set(5, -60.0f + static_cast<float>(block % 31) * 2.0f);
            set(6, -30.0f + static_cast<float>(block % 31));
            set(7, 1.0f + static_cast<float>(block % 20) * 5.0f);
            set(8, static_cast<float>(block % 19));
            set(9, 0.05f + static_cast<float>(block % 10));
            set(10, 5.0f + static_cast<float>(block % 50) * 10.0f);
            set(11, static_cast<float>(block % 3));
            set(12, block % 97 == 0 ? 1.0f : 0.0f);
            set(13, static_cast<float>((block / 7) % static_cast<int>(crucible::fx::ChannelMode::Count)));
            set(14, static_cast<float>(block % 5) * 25.0f);
            h.run(l.data(), r.data(), 64);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
    }

    void deesserCpu()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest(rateTag() + "fx.deesser: CPU, percent of one core per stereo instance at 48 kHz (plan: 0.2 %)");
        double load = 0.0;
        const double c = cpuPercent(
            "deesser",
            {{"fx_deesser_threshold_mode", 1.0f}, {"fx_deesser_threshold", -30.0f}, {"fx_deesser_lookahead", 1.5f}}, &load);
        logMessage(String("CPU fx.deesser: ") + String(c, 3) + " % (best of 8, load average " + String(load, 1) + ")");
        expect(c <= 0.2 * 1.2, "within 0.2 % (20 % measurement margin)");
#endif
    }
};

static TransientDeEsserTests transientDeEsserTests;
