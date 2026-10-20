// SPDX-License-Identifier: MIT
// fx.cabinet: the procedural cabinet model (include/crucible/fx/CabinetModel.h) and the module built on it. Features are
// measured on the generated / rendered impulse response, not read back from the design tables.

#include "AllocCounter.h"
#include "CpuTiming.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fft.h>
#include <crucible/Fx.h>
#include <crucible/fx/CabinetModel.h>
#include <crucible/module/CabinetProbe.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <chrono>
#include <cstring>
#include <thread>

#include <algorithm>
#include <cmath>
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
constexpr double kPi = 3.14159265358979323846;
using Floats = std::vector<float>;

// Magnitude response in dB of an impulse response on a dense grid (zero-padded transform of 2^17 points).
struct Response
{
    std::vector<double> db;
    double rate = 48000.0;
    double binHz = 0.0;
    double at(double hz) const
    {
        const double pos = std::min(hz, 0.499 * rate) / binHz;
        const size_t k = std::min(static_cast<size_t>(pos), db.size() - 2);
        const double f = pos - static_cast<double>(k);
        return db[k] * (1.0 - f) + db[k + 1] * f;
    }
    // Power averaged over a one-octave window around hz, in dB.
    double smoothed(double hz) const
    {
        double sum = 0.0;
        const int n = 21;
        for (int i = 0; i < n; ++i)
            sum += std::pow(10.0, at(hz * std::pow(2.0, 0.5 * (2.0 * i / (n - 1) - 1.0))) / 10.0);
        return 10.0 * std::log10(sum / n);
    }
    // Highest smoothed level in [lo, hi]: its frequency, and the last frequency still within `drop` dB of it.
    double smoothedPeakHz(double lo, double hi) const
    {
        double best = -1.0e9, bestHz = lo;
        for (double f = lo; f <= hi; f *= 1.01)
            if (smoothed(f) > best)
            {
                best = smoothed(f);
                bestHz = f;
            }
        return bestHz;
    }
    double smoothedFallHz(double from, double drop) const
    {
        // The last frequency still within `drop` dB of the level at `from`, below 18 kHz (dips in between do not count).
        const double ref = smoothed(from);
        double f = std::min(18000.0, 0.45 * rate);
        while (f > from && smoothed(f) < ref - drop)
            f /= 1.003;
        return f;
    }
    // Mean level in dB over [lo, hi] Hz (log-spaced samples).
    double meanDb(double lo, double hi) const
    {
        double sum = 0.0;
        int n = 0;
        for (double f = lo; f <= hi; f *= 1.01, ++n)
            sum += at(f);
        return sum / n;
    }
    // Frequency of the highest level in [lo, hi].
    double peakHz(double lo, double hi) const
    {
        double best = -1.0e9, bestHz = lo;
        for (double f = lo; f <= hi; f *= 1.002)
            if (at(f) > best)
            {
                best = at(f);
                bestHz = f;
            }
        return bestHz;
    }
    // First frequency above `from` where the level falls `drop` dB below the level at `from`.
    double fallHz(double from, double drop) const
    {
        const double ref = at(from);
        for (double f = from; f < 0.49 * rate; f *= 1.002)
            if (at(f) <= ref - drop)
                return f;
        return 0.49 * rate;
    }
};

Response spectrumOf(const Floats& ir, double rate)
{
    constexpr int order = 17;
    const size_t n = static_cast<size_t>(1) << order;
    Fft fft(order);
    std::vector<float> buf(2 * n, 0.0f);
    std::copy(ir.begin(), ir.end(), buf.begin());
    fft.forward(buf.data());
    Response s;
    s.rate = rate;
    s.binHz = rate / static_cast<double>(n);
    s.db.resize(n / 2 + 1);
    for (size_t k = 0; k <= n / 2; ++k)
    {
        const double re = buf[2 * k], im = buf[2 * k + 1];
        s.db[k] = 10.0 * std::log10(std::max(re * re + im * im, 1.0e-24));
    }
    return s;
}

Response cabinetSpectrum(int cabinet, float pos, float dist, double rate)
{
    CabinetSettings st;
    st.cabinet = cabinet;
    st.micPosition = pos;
    st.micDistanceCm = dist;
    Floats ir;
    generateCabinetIr(st, rate, ir);
    return spectrumOf(ir, rate);
}

double kRate = 48000.0;

// fx.cabinet through the rack, every other effect off.
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
        set("fx_cab_on", 1.0f);
    }
    static int ix(const std::string& id) { return ParameterMap::indexOf(id); }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ix(id))] = v; }
    void p(const char* name, float v) { set(std::string("fx_cab_") + name, v); }
    void run(float* l, float* r, int n)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, nullptr, nullptr);
    }
    // Runs `n` samples of the same signal on both channels in blocks of `block`; returns the left output.
    Floats render(const Floats& in, int block = 128)
    {
        Floats l = in, r = in;
        for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
            run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos)));
        return l;
    }
    // Lets a pending kernel build, prime and fade in (the module passes the input through until then).
    void settle(double seconds = 0.5)
    {
        Floats z(static_cast<size_t>(seconds * kRate), 0.0f);
        render(z);
    }
    int latency()
    {
        auto* lat = rack.findCapability<LatencyReporter>("fx.cabinet");
        return lat != nullptr ? lat->latencySamples() : -1;
    }
};

// The response of the cabinet measured on the rendered output: an impulse through the settled rack.
Response renderedSpectrum(Rack& h)
{
    h.settle();
    Floats in(8192, 0.0f);
    in[0] = 1.0f;
    const Floats out = h.render(in);
    Floats ir(out.begin(), out.begin() + static_cast<long>(cabinetTapCount(kRate)));
    return spectrumOf(ir, kRate);
}

Floats sine(size_t n, double hz, double amp)
{
    Floats out(n);
    for (size_t i = 0; i < n; ++i)
        out[i] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * static_cast<double>(i) / kRate));
    return out;
}

// Largest ratio between the peak amplitudes of adjacent windows (1 ms unless a lower tone needs longer ones) over [from, to) (a smooth change stays near 1).
double worstEnvelopeStep(const Floats& x, size_t from, size_t to, double windowMs = 1.0)
{
    const size_t w = static_cast<size_t>(kRate * 0.001 * windowMs);
    double worst = 1.0, previous = 0.0;
    for (size_t pos = from; pos + w <= to; pos += w)
    {
        double peak = 0.0;
        for (size_t i = 0; i < w; ++i)
            peak = std::max(peak, static_cast<double>(std::abs(x[pos + i])));
        if (previous > 1.0e-4 && peak > 1.0e-4)
            worst = std::max(worst, std::max(peak / previous, previous / peak));
        previous = peak;
    }
    return worst;
}

#if !defined(CRUCIBLE_TEST_UNTIMED)
double cpuPercent(double* loadOut)
{
    const size_t n = static_cast<size_t>(6.0 * kRate);
    Random rng(5);
    Floats src(n);
    for (auto& v : src)
        v = static_cast<float>(0.25 * rng.gaussian());
    const auto run = [&](bool enabled)
    {
        Rack h;
        h.set("fx_cab_on", enabled ? 1.0f : 0.0f);
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
    double avg[3] = {0, 0, 0};
#if !defined(_WIN32)
    getloadavg(avg, 3);
#endif
    *loadOut = avg[0];
    return 100.0 * best / (static_cast<double>(n - 48000) / 48000.0);
}
#endif
} // namespace

class CabinetTests : public UnitTest
{
public:
    CabinetTests() : UnitTest("Cabinet", "Effects") {}

    void runTest() override
    {
        modelFeatures();
        modelMic();
        modelShape();
        registration();
        for (const double rate : {48000.0, 44100.0, 96000.0, 192000.0})
        {
            kRate = rate;
            renderedFeatures();
            latencyAndSilence();
        }
        kRate = 48000.0;
        micThroughRack();
        smoothness();
        resetAndBypass();
        realTime();
        determinism();
        firstKernelAtStandardQuality();
        allocations();
        staleSets();
        retireNeverLeaks();
        cpu();
    }

private:
    void modelFeatures()
    {
        beginTest("cabinet model: every cabinet's resonance peak is within 5 % of its design and its roll-off corner within 10 %");
        for (const double rate : {44100.0, 48000.0, 96000.0})
            for (int c = 0; c < kCabinetCount; ++c)
            {
                const CabinetSpec& spec = cabinetSpec(c);
                const Response s = cabinetSpectrum(c, 0.0f, 8.0f, rate);
                const double peak = s.peakHz(25.0, 400.0);
                const double top = s.smoothedPeakHz(spec.rolloffHz * 0.3, spec.rolloffHz);
                const double corner = s.smoothedFallHz(top, 3.0);
                std::printf("  %-16s %6.0f Hz: peak %.1f Hz (design %.1f), corner %.0f Hz (design %.0f)\n", spec.name, rate, peak,
                            spec.resonanceHz, corner, spec.rolloffHz);
                expect(std::abs(corner / spec.rolloffHz - 1.0) < 0.10, std::string(spec.name) + " corner at " + std::to_string(rate));
                expect(std::abs(peak / spec.resonanceHz - 1.0) < 0.05,
                       std::string(spec.name) + " resonance at " + std::to_string(rate));
            }
    }

    void modelMic()
    {
        beginTest("cabinet model: moving the microphone from the centre to the edge lowers the high-frequency tilt monotonically "
                  "(every cabinet)");
        for (int c = 0; c < kCabinetCount; ++c)
        {
            double previous = 1.0e9;
            bool monotonic = true;
            double first = 0.0, last = 0.0;
            for (int i = 0; i <= 10; ++i)
            {
                const Response s = cabinetSpectrum(c, static_cast<float>(i) / 10.0f, 8.0f, 48000.0);
                const double tilt = s.meanDb(3500.0, 7000.0) - s.meanDb(400.0, 1000.0);
                if (i == 0)
                    first = tilt;
                last = tilt;
                monotonic = monotonic && tilt < previous;
                previous = tilt;
            }
            std::printf("  %-16s tilt centre %.1f dB, edge %.1f dB\n", cabinetSpec(c).name, first, last);
            expect(monotonic, std::string(cabinetSpec(c).name) + " tilt falls at every step");
            expect(first - last > 5.0, std::string(cabinetSpec(c).name) + " centre to edge moves the top end by more than 5 dB");
        }

        beginTest("cabinet model: the dust-cap notch moves down in frequency and the proximity bass falls with distance");
        for (int c = 0; c < kCabinetCount; ++c)
        {
            const CabinetSpec& spec = cabinetSpec(c);
            const double centre = spec.notchHz, edge = spec.notchHz * 0.7;
            const Response a = cabinetSpectrum(c, 0.0f, 8.0f, 48000.0), b = cabinetSpectrum(c, 1.0f, 8.0f, 48000.0);
            // Depth of the notch against its surroundings, at the edge position, is largest near edge * (1 +- 10 %).
            const auto dip = [&](const Response& s, double hz) { return s.at(hz) - 0.5 * (s.at(hz * 0.7) + s.at(hz * 1.4)); };
            expect(dip(b, edge) < dip(a, edge) - 2.0, std::string(spec.name) + " notch at the edge position deepens near " +
                                                         std::to_string(edge));
            expect(dip(b, edge) < dip(b, centre) + 1.0, std::string(spec.name) + " edge notch sits lower than the centre one");
            const Response nearIr = cabinetSpectrum(c, 0.0f, 0.0f, 48000.0), farIr = cabinetSpectrum(c, 0.0f, 40.0f, 48000.0);
            const double bassNear = nearIr.meanDb(30.0, 70.0) - nearIr.meanDb(600.0, 1200.0);
            const double bassFar = farIr.meanDb(30.0, 70.0) - farIr.meanDb(600.0, 1200.0);
            expect(bassNear > bassFar + 2.0, std::string(spec.name) + " proximity: 0 cm has more bass than 40 cm");
        }
    }

    void modelShape()
    {
        beginTest("cabinet model: minimum phase, length and level, room reflection, open-back notch (every cabinet)");
        for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
            expectEquals(cabinetTapCount(rate), static_cast<int>(std::lround(2048.0 * rate / 48000.0)) > 8192 ? 8192
                                                 : static_cast<int>(std::lround(2048.0 * rate / 48000.0)),
                         "tap count at " + std::to_string(rate));
        expect(cabinetTapCount(48000.0) <= 2048, "at most 2048 taps at 48 kHz");
        for (int c = 0; c < kCabinetCount; ++c)
        {
            const CabinetSpec& spec = cabinetSpec(c);
            CabinetSettings st;
            st.cabinet = c;
            st.micDistanceCm = 8.0f;
            Floats ir;
            generateCabinetIr(st, 48000.0, ir);
            expectEquals(static_cast<int>(ir.size()), cabinetTapCount(48000.0), "length");
            double total = 0.0, early = 0.0, tail = 0.0;
            bool finite = true;
            for (size_t i = 0; i < ir.size(); ++i)
            {
                const double e = static_cast<double>(ir[i]) * ir[i];
                finite = finite && std::isfinite(ir[i]);
                total += e;
                if (i < 480)
                    early += e;
                if (i >= ir.size() - ir.size() / 8)
                    tail += e;
            }
            expect(finite, std::string(spec.name) + " finite");
            expect(early > 0.97 * total, std::string(spec.name) + " minimum phase: 97 % of the energy in the first 10 ms");
            expect(tail < 1.0e-5 * total, std::string(spec.name) + " tail below -50 dB at the end");
            const Response s = spectrumOf(ir, 48000.0);
            double power = 0.0;
            int n = 0;
            for (double f = 400.0; f <= 2500.0; f *= 1.02, ++n)
                power += std::pow(10.0, s.at(f) / 10.0);
            const double meanDb = 10.0 * std::log10(power / n);
            expect(std::abs(meanDb) < 0.5, std::string(spec.name) + " 400 Hz to 2.5 kHz mean power 0 dB, got " + std::to_string(meanDb));
        }
        beginTest("cabinet model: the open backs show a comb notch at 1 / (2 * delay), closed and bass cabinets do not");
        for (int c = 0; c < kCabinetCount; ++c)
        {
            const CabinetSpec& spec = cabinetSpec(c);
            if (spec.combGain < 0.2)
                continue;
            const double notch = 1.0 / (2.0 * spec.combDelayMs * 1.0e-3);
            const Response s = cabinetSpectrum(c, 0.0f, 8.0f, 48000.0);
            const double depth = 0.5 * (s.at(notch * 0.6) + s.at(notch * 1.5)) - s.at(notch);
            std::printf("  %-16s notch %.0f Hz, %.1f dB below its neighbours\n", spec.name, notch, depth);
            expect(depth > 1.5, std::string(spec.name) + " notch present");
        }
        beginTest("cabinet model: the microphone distance adds a room reflection at the expected delay");
        {
            CabinetSettings st;
            st.cabinet = 2;
            st.micDistanceCm = 0.0f;
            Floats nearIr, farIr;
            generateCabinetIr(st, 48000.0, nearIr);
            st.micDistanceCm = 50.0f;
            generateCabinetIr(st, 48000.0, farIr);
            // Autocorrelation at the reflection delay.
            const int delay = static_cast<int>(std::lround(48000.0 * (0.0004 + 2.0 * 50.0 * 0.01 / 343.0)));
            const auto corr = [&](const Floats& x, int lag)
            {
                double acc = 0.0, norm = 0.0;
                for (size_t i = 0; i + static_cast<size_t>(lag) < x.size(); ++i)
                    acc += static_cast<double>(x[i]) * x[i + static_cast<size_t>(lag)];
                for (float v : x)
                    norm += static_cast<double>(v) * v;
                return acc / norm;
            };
            expect(corr(farIr, delay) > corr(nearIr, delay) + 0.05, "echo at " + std::to_string(delay) + " samples is stronger at 50 cm");
        }
    }
    std::string tag() const
    {
        if (kRate == 48000.0)
            return std::string();
        char t[32];
        std::snprintf(t, sizeof(t), "[%g kHz] ", kRate / 1000.0);
        return t;
    }

    void registration()
    {
        beginTest("fx.cabinet: registered last (after fx.multiband_drive), spelled-out ids, latency capability, parameters appended");
        const auto& registry = ModuleRegistry::instance();
        const auto effects = registry.ofKind(ModuleKind::Effect);
        expect(registry.find("fx.cabinet") != nullptr, "registered");
        expectEquals(static_cast<int>(effects.size()), kNumModuleFx, "effects in the registry");
        expect(effects.size() >= 2 && effects.back()->id == "fx.cabinet" && effects[effects.size() - 2]->id == "fx.multiband_drive",
               "last effect, after fx.multiband_drive");
        bool names = true;
        for (const auto& pd : effects.back()->params)
            names = names && (pd.id.rfind("fx_cab_", 0) == 0 || pd.id == "fx_pos_cab");
        expect(names, "every id is fx_cab_* (or the slot)");
        logMessage(String("fx.cabinet parameters: ") + String(static_cast<int>(effects.back()->params.size())));
        expect(ParameterMap::indexOf("fx_cab_on") > ParameterMap::indexOf("fx_crush_fine"),
               "parameters follow fx.crusher's appended ones (frozen order unchanged)");
        Rack h;
        expect(h.rack.findCapability<LatencyReporter>("fx.cabinet") != nullptr, "latency capability");
        expectEquals(static_cast<int>(effects.back()->params.size()), 9, "parameter count");
    }

    void renderedFeatures()
    {
        beginTest(tag() + "fx.cabinet: on the rendered output each cabinet shows its resonance within 5 % and its roll-off corner "
                          "within 10 % of the design");
        for (int c = 0; c < kCabinetCount; ++c)
        {
            Rack h(Quality::High);
            h.p("cabinet", static_cast<float>(c));
            h.p("mix", 1.0f);
            const CabinetSpec& spec = cabinetSpec(c);
            const Response s = renderedSpectrum(h);
            const double peak = s.peakHz(25.0, 400.0);
            const double top = s.smoothedPeakHz(spec.rolloffHz * 0.3, spec.rolloffHz);
            const double corner = s.smoothedFallHz(top, 3.0);
            if (kRate == 48000.0)
                logMessage(String(spec.name) + ": resonance " + String(peak, 1) + " Hz (design " + String(spec.resonanceHz, 0) +
                           "), corner " + String(corner, 0) + " Hz (design " + String(spec.rolloffHz, 0) + ")");
            expect(std::abs(peak / spec.resonanceHz - 1.0) < 0.05, std::string(spec.name) + " resonance " + std::to_string(peak));
            expect(std::abs(corner / spec.rolloffHz - 1.0) < 0.10, std::string(spec.name) + " corner " + std::to_string(corner));
        }
    }

    void latencyAndSilence()
    {
        beginTest(tag() + "fx.cabinet: latency 0 reported and true (the response starts at the first sample), silence in gives "
                          "silence out");
        Rack h(Quality::High);
        h.p("mix", 1.0f);
        expectEquals(h.latency(), 0, "reported latency");
        h.settle();
        Floats in(4096, 0.0f);
        in[100] = 1.0f;
        const Floats out = h.render(in);
        size_t at = 0;
        for (size_t i = 0; i < out.size(); ++i)
            if (std::abs(out[i]) > std::abs(out[at]))
                at = i;
        expect(at >= 100 && at < 100 + 40, "response peak within 40 samples of the impulse, at " + std::to_string(at));
        for (size_t i = 0; i < 100; ++i)
            if (out[i] != 0.0f)
            {
                expect(false, "output before the impulse is zero");
                break;
            }
        Rack z(Quality::High);
        z.p("mix", 1.0f);
        z.settle(0.2);
        const Floats silent = z.render(Floats(48000, 0.0f));
        bool allZero = true;
        for (const float v : silent)
            allZero = allZero && v == 0.0f;
        expect(allZero, "silence in, silence out");
    }

    void micThroughRack()
    {
        beginTest("fx.cabinet: mic position moves the high-frequency tilt monotonically on the rendered output; the cabinet "
                  "control changes the response");
        Rack h(Quality::High);
        h.p("mix", 1.0f);
        h.p("cabinet", 0.0f);
        double previous = 1.0e9;
        bool monotonic = true;
        double first = 0.0, last = 0.0;
        for (int i = 0; i <= 5; ++i)
        {
            h.p("mic_position", 20.0f * static_cast<float>(i));
            const Response s = renderedSpectrum(h);
            const double tilt = s.meanDb(3500.0, 7000.0) - s.meanDb(400.0, 1000.0);
            if (i == 0)
                first = tilt;
            last = tilt;
            monotonic = monotonic && tilt < previous;
            previous = tilt;
        }
        logMessage(String("rendered tilt, centre to edge: ") + String(first, 1) + " dB to " + String(last, 1) + " dB");
        expect(monotonic, "tilt falls at every step");
        expect(first - last > 5.0, "centre to edge moves the top end by more than 5 dB");

        Rack a(Quality::High), b(Quality::High);
        a.p("cabinet", 0.0f);
        b.p("cabinet", 5.0f);
        a.p("mix", 1.0f);
        b.p("mix", 1.0f);
        const Response sa = renderedSpectrum(a), sb = renderedSpectrum(b);
        expect(sa.peakHz(25.0, 400.0) > 1.5 * sb.peakHz(25.0, 400.0), "1x12 resonates higher than the 1x15 bass");
    }

    void smoothness()
    {
        beginTest("fx.cabinet: output level, mix, low cut, high cut and a cabinet switch move without zipper or click");
        struct Step
        {
            const char* name;
            const char* param;
            float from, to;
            double hz;
            double limit;
            double windowMs;
        };
        const Step steps[] = {{"output", "out", -12.0f, 6.0f, 1000.0, 1.45, 1.0},
                              {"mix", "mix", 1.0f, 0.0f, 1000.0, 1.45, 1.0},
                              {"low cut", "lowcut", 20.0f, 300.0f, 100.0, 2.6, 10.0},
                              {"high cut", "highcut", 20000.0f, 2000.0f, 5000.0, 1.6, 1.0},
                              {"cabinet", "cabinet", 0.0f, 5.0f, 1000.0, 1.3, 1.0}};
        for (const Step& st : steps)
            for (const int block : {1, 64, 512})
            {
                Rack h(Quality::High);
                h.p("mix", 1.0f);
                h.p(st.param, st.from);
                h.settle();
                const Floats in = sine(static_cast<size_t>(kRate), st.hz, 0.3);
                Floats l = in, r = in;
                const size_t half = in.size() / 4;
                for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
                {
                    if (pos >= half)
                        h.p(st.param, st.to);
                    h.run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos)));
                }
                const double worst = worstEnvelopeStep(l, half / 2, l.size(), st.windowMs);
                if (block == 64)
                    logMessage(String("worst 1 ms envelope step, ") + st.name + ": " + String(worst, 3) + "x");
                expect(worst <= st.limit, std::string(st.name) + " at block " + std::to_string(block) + ": " + std::to_string(worst));
            }
    }

    void resetAndBypass()
    {
        beginTest("fx.cabinet: bypass passes the input exactly, reset clears the tail and brings the response back unchanged");
        {
            Rack h;
            h.set("fx_cab_on", 0.0f);
            const Floats in = sine(4800, 440.0, 0.4);
            const Floats out = h.render(in);
            expect(std::memcmp(in.data(), out.data(), in.size() * sizeof(float)) == 0, "bypassed output equals the input");
        }
        Rack h(Quality::High);
        h.p("mix", 1.0f);
        h.settle();
        Floats burst(4800);
        Random rng(9);
        for (auto& v : burst)
            v = static_cast<float>(0.5 * rng.gaussian());
        h.render(burst);
        h.rack.reset();
        const Floats silent = h.render(Floats(4096, 0.0f));
        double peak = 0.0;
        for (const float v : silent)
            peak = std::max(peak, static_cast<double>(std::abs(v)));
        expect(peak == 0.0, "after reset silence stays silent, peak " + std::to_string(peak));
        Floats in(2048, 0.0f);
        in[0] = 1.0f;
        const Floats a = h.render(in);
        Rack fresh(Quality::High);
        fresh.p("mix", 1.0f);
        fresh.settle();
        const Floats b = fresh.render(in);
        double diff = 0.0, ref = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
        {
            diff += (a[i] - b[i]) * (a[i] - b[i]);
            ref += b[i] * b[i];
        }
        expect(diff < 1.0e-8 * ref, "response after reset equals a fresh instance's");
    }

    void realTime()
    {
        beginTest("fx.cabinet: in real time (Standard quality, builder thread) the input passes until the first kernel is "
                  "ready, then the cabinet fades in, and a cabinet change crossfades to the new one");
        Rack h;
        h.p("mix", 1.0f);
        h.p("cabinet", 0.0f);
        const auto settled = [&](int cabinet)
        {
            // Impulse responses measured with a short burst of blocks, polled until the response has the wanted resonance.
            for (int attempt = 0; attempt < 400; ++attempt)
            {
                h.render(Floats(4096, 0.0f));
                Floats in(2048, 0.0f);
                in[0] = 1.0f;
                const Response s = spectrumOf(h.render(in), kRate);
                const double peak = s.peakHz(25.0, 400.0);
                if (std::abs(peak / cabinetSpec(cabinet).resonanceHz - 1.0) < 0.05)
                    return true;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            return false;
        };
        {
            // Before anything is built the output is the input.
            Rack fresh;
            fresh.p("mix", 1.0f);
            Floats in(16, 0.0f);
            in[0] = 1.0f;
            Floats l = in, r = in;
            fresh.run(l.data(), r.data(), 16);
            expect(std::abs(l[0] - 1.0f) < 0.5f && std::abs(l[8]) < 0.2f, "first block passes the input (through the cuts only), got " + std::to_string(l[0]));
        }
        expect(settled(0), "1x12 response arrives");
        h.p("cabinet", 5.0f);
        expect(settled(5), "1x15 bass response arrives after the change");
    }

    void staleSets()
    {
        beginTest("fx.cabinet: a re-prepare at another rate while a build is in flight never plays the old-rate set");
        kRate = 48000.0;
        Rack h;
        h.p("mix", 1.0f);
        auto* probe = h.rack.findCapability<CabinetProbe>("fx.cabinet");
        expect(probe != nullptr, "probe capability");
        if (probe == nullptr)
            return;
        h.settle(0.3); // the first 48 kHz set is built, primed and playing (the first build is waited for)
        probe->holdBuilder(true);
        h.p("cabinet", 4.0f);
        h.settle(0.2); // posts the build of the new cabinet once the control has rested; the builder finishes, waits to publish
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        ModuleContext ctx;
        ctx.sampleRate = 44100.0;
        h.rack.prepare(ctx);
        probe->holdBuilder(false);
        for (int i = 0; i < 600 && !probe->setPending(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        expect(probe->setPending(), "the old-rate set was published after prepare (the race under test)");
        kRate = 44100.0;
        int stale = 0, played44 = 0;
        Floats in(128, 0.0f);
        for (int i = 0; i < 700; ++i)
        {
            Floats l = in, r = in;
            if (i == 0)
                l[0] = r[0] = 1.0f;
            h.run(l.data(), r.data(), 128);
            const double rate = probe->playingRate();
            stale += rate == 48000.0 ? 1 : 0;
            played44 += rate == 44100.0 ? 1 : 0;
            if (i > 20 && i % 50 == 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        kRate = 48000.0;
        expect(stale == 0, String(stale) + " blocks played or primed the 48 kHz set at 44.1 kHz");
        expect(played44 > 0, "the 44.1 kHz set takes over");
    }

    void retireNeverLeaks()
    {
        beginTest("fx.cabinet: retiring more sets than the ring and the pending list hold frees every one of them");
        kRate = 48000.0;
        Rack h;
        h.p("mix", 1.0f);
        auto* probe = h.rack.findCapability<CabinetProbe>("fx.cabinet");
        expect(probe != nullptr, "probe capability");
        if (probe == nullptr)
            return;
        // Until the first set is built, taken and faded in, sets come and go: wait for the steady state.
        for (int i = 0; i < 800 && probe->playingRate() != kRate; ++i)
        {
            h.settle(0.01);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        h.settle(0.2);
        const int base = CabinetProbe::liveSets();
        probe->holdBuilder(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        probe->retireSpares(40); // the ring holds 16: the rest cannot be freed while the builder is held
        expectEquals(CabinetProbe::liveSets(), base + 40, "nothing is freed while the builder is held");
        probe->holdBuilder(false);
        Floats z(128, 0.0f);
        for (int i = 0; i < 1000 && CabinetProbe::liveSets() != base; ++i)
        {
            h.run(z.data(), z.data(), 128);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        expectEquals(CabinetProbe::liveSets(), base, "every retired set was freed");
    }

    void firstKernelAtStandardQuality()
    {
        beginTest("fx.cabinet: the render after prepare does not depend on when the first kernel is built (any quality)");
        // delayMs > 0: the builder is held and released after that many milliseconds. The blocks are paced so that the render lasts longer than the delay.
        const auto render = [this](int delayMs)
        {
            kRate = 48000.0;
            Rack h(Quality::Standard);
            h.p("mix", 1.0f);
            h.p("cabinet", 3.0f);
            auto* probe = h.rack.findCapability<CabinetProbe>("fx.cabinet");
            std::thread releaser;
            if (probe != nullptr && delayMs != 0)
            {
                probe->holdBuilder(true);
                if (delayMs > 0)
                    releaser = std::thread(
                        [probe, delayMs]
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
                            probe->holdBuilder(false);
                        });
            }
            Random rng(9);
            Floats in(96000);
            for (auto& v : in)
                v = static_cast<float>(0.3 * rng.gaussian());
            Floats l = in, r = in;
            for (size_t pos = 0, block = 0; pos < l.size(); pos += 128, ++block)
            {
                h.run(l.data() + pos, r.data() + pos, 128);
                if (block % 4 == 0)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if (releaser.joinable())
                releaser.join();
            return l;
        };
        const Floats quick = render(0), slow = render(120);
        // Input and output of a render are the same noise: the difference of the first block (input through the cuts only)
        // against the last blocks (the cabinet) shows that the cabinet plays.
        Random rng(9);
        Floats dry(quick.size());
        for (auto& v : dry)
            v = static_cast<float>(0.3 * rng.gaussian());
        double worst = 0.0, head = 0.0, tail = 0.0;
        for (size_t i = 0; i < quick.size(); ++i)
            worst = std::max(worst, static_cast<double>(std::abs(quick[i] - slow[i])));
        for (size_t i = 0; i < 128; ++i)
            head += static_cast<double>(quick[i] - dry[i]) * (quick[i] - dry[i]);
        for (size_t i = quick.size() - 4096; i < quick.size(); ++i)
            tail += static_cast<double>(quick[i] - dry[i]) * (quick[i] - dry[i]) * (128.0 / 4096.0);
        expect(tail > 2.0 * head, "the cabinet plays by the end of the render");
        expect(worst == 0.0, "a builder that is 120 ms late changes the output by " + String(worst));
    }

    void determinism()
    {
        beginTest("fx.cabinet: an offline (High quality) render is bit-identical from run to run, whatever the thread timing");
        const auto render = []
        {
            Rack h(Quality::High);
            h.p("mix", 1.0f);
            Random rng(4);
            Floats in(96000);
            for (auto& v : in)
                v = static_cast<float>(0.3 * rng.gaussian());
            Floats l = in, r = in;
            for (size_t pos = 0; pos < l.size(); pos += 128)
            {
                if (pos == 48000)
                {
                    h.p("cabinet", 4.0f);
                    h.p("mic_position", 60.0f);
                }
                h.run(l.data() + pos, r.data() + pos, 128);
            }
            return l;
        };
        const Floats first = render(), second = render(), third = render();
        expect(std::memcmp(first.data(), second.data(), first.size() * sizeof(float)) == 0, "run 1 equals run 2");
        expect(std::memcmp(first.data(), third.data(), first.size() * sizeof(float)) == 0, "run 1 equals run 3");
    }

    void allocations()
    {
        beginTest("fx.cabinet: no allocation on the audio thread with every parameter changing every block");
        Rack h;
        const char* ids[] = {"mix", "cabinet", "mic_position", "mic_distance", "lowcut", "highcut", "out"};
        int index[7];
        for (int i = 0; i < 7; ++i)
            index[i] = Rack::ix(std::string("fx_cab_") + ids[i]);
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
            const auto set = [&](int k, float value) { h.values[static_cast<size_t>(index[k])] = value; };
            set(0, static_cast<float>(block % 5) * 0.25f);
            set(1, static_cast<float>((block / 40) % 8));
            set(2, static_cast<float>((block / 9) % 11) * 10.0f);
            set(3, static_cast<float>((block / 13) % 7) * 9.0f);
            set(4, 20.0f + static_cast<float>(block % 17) * 20.0f);
            set(5, 2000.0f + static_cast<float>(block % 23) * 800.0f);
            set(6, static_cast<float>(block % 9) - 4.0f);
            h.run(l.data(), r.data(), 64);
        }
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
        // Let the builder finish before the rack goes away, and check that the module still produces finite output.
        Floats z(48000, 0.1f);
        h.render(z);
        bool finite = true;
        for (const float v : z)
            finite = finite && std::isfinite(v);
        expect(finite, "finite output after the sweep");
    }

    void cpu()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest("fx.cabinet: CPU, percent of one core per stereo instance at 48 kHz (blocks of 32, Release)");
        double load = 0.0;
        const double pct = cpuPercent(&load);
        logMessage(String("CPU fx.cabinet: ") + String(pct, 3) + " % (best of 8, load average " + String(load, 1) + ")");
        expect(pct <= 1.5, "within 1.5 % of one core");
#endif
    }
};

static CabinetTests cabinetTests;
