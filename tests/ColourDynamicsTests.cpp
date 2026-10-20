// SPDX-License-Identifier: MIT
// FX13 colour and dynamics suites (docs/plans/ENGINE-FX-PLAN.md section 3.4): `fx.saturator` through the module rack.
// Harmonic signatures per model, THD against drive, aliasing at full drive, loudness-matched auto gain on pink noise,
// the null at drive minimum and at mix 0, the tilt pair, model switches, extremes, allocations, determinism and CPU.
// Every claim is a number; the printed lines are what docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "CpuTiming.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Fx.h>
#include <crucible/ModMatrix.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
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
using crucible::testing::kRate;

namespace
{
using Samples = std::vector<float>;

constexpr int kNumModels = 10;
const char* const kModelNames[kNumModels] = {"Soft Clip",   "Hard Clip",      "Op-Amp Rail",  "Diode", "Wavefolder",
                                             "Tube Triode", "Tube Push-Pull", "Tube Pentode", "Tape",  "Transformer"};
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

// fx.saturator alone in the rack (every other effect off).
struct Sat
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values, legacy;
    LatencyReporter* latency = nullptr;
    int idOn, idMix, idModel, idDrive, idBias, idOut, idTilt, idTiltHz, idChar, idAuto;

    explicit Sat(Quality quality = Quality::Standard, double rate = kRate)
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        ctx.quality = quality;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            values[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 0.0f;
        const auto ix = [](const char* id) { return ParameterMap::indexOf(id); };
        idOn = ix("fx_saturator_on");
        idMix = ix("fx_saturator_mix");
        idModel = ix("fx_saturator_model");
        idDrive = ix("fx_saturator_drive");
        idBias = ix("fx_saturator_bias");
        idOut = ix("fx_saturator_output");
        idTilt = ix("fx_saturator_tilt");
        idTiltHz = ix("fx_saturator_tilt_freq");
        idChar = ix("fx_saturator_character");
        idAuto = ix("fx_saturator_autogain");
        set(idOn, 1.0f);
        latency = rack.findCapability<LatencyReporter>("fx.saturator");
    }
    void set(int id, float v) { values[static_cast<size_t>(id)] = v; }
    void config(int model, float drive, float mix = 1.0f, bool autoGain = false)
    {
        set(idModel, static_cast<float>(model));
        set(idDrive, drive);
        set(idMix, mix);
        set(idAuto, autoGain ? 1.0f : 0.0f);
    }
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

// A fresh instance per measurement (a Render callable of FxMeasure.h).
auto renderer(int model, float drive, Quality q = Quality::Standard, float bias = 0.0f, bool autoGain = false)
{
    return [=](const Signal& in)
    {
        Sat s(q);
        s.config(model, drive, 1.0f, autoGain);
        s.set(s.idBias, bias);
        return s.render(in);
    };
}

// Pink noise (Paul Kellet's filter) on a seed of its own, scaled to a given RMS in dBFS.
Signal pinkNoise(size_t n, double rmsDb, std::uint32_t seed)
{
    std::uint32_t st = seed;
    double b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
    Signal x(n);
    double sq = 0.0;
    for (auto& v : x)
    {
        st = st * 22695477u + 1u;
        const double w = static_cast<double>(static_cast<std::int32_t>(st)) / 2147483648.0;
        b0 = 0.99886 * b0 + w * 0.0555179;
        b1 = 0.99332 * b1 + w * 0.0750759;
        b2 = 0.96900 * b2 + w * 0.1538520;
        b3 = 0.86650 * b3 + w * 0.3104856;
        b4 = 0.55000 * b4 + w * 0.5329522;
        b5 = -0.7616 * b5 - w * 0.0168980;
        v = b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362;
        b6 = w * 0.115926;
        sq += v * v;
    }
    const double g = dbToAmplitude(rmsDb) / std::sqrt(sq / static_cast<double>(n));
    for (auto& v : x)
        v *= g;
    return x;
}

// Area enclosed by the input-output curve (shoelace) per cycle, as a share of the box spanned by the peaks of both.
// The input is delayed by the module's latency; a memoryless curve then traces a line and the area is zero.
double loopArea(const Signal& in, const Signal& out, size_t from, size_t count, size_t delay, double cycles)
{
    double a = 0.0, peakIn = 0.0, peakOut = 0.0;
    for (size_t i = from; i + 1 < from + count; ++i)
    {
        a += in[i - delay] * out[i + 1] - in[i + 1 - delay] * out[i];
        peakIn = std::max(peakIn, std::abs(in[i]));
        peakOut = std::max(peakOut, std::abs(out[i]));
    }
    return std::abs(a) * 0.5 / cycles / (4.0 * peakIn * peakOut);
}
} // namespace

class SaturatorTests : public UnitTest
{
public:
    SaturatorTests() : UnitTest("Saturator", "Effects") {}

    void runTest() override
    {
        descriptor();
        harmonics();
        thd();
        aliasing();
        response();
        autoGain();
        nulls();
        blockSizes();
        tilt();
        switching();
        pathSwitching();
        extremes();
        realtime();
        cpu();
    }

private:
    void descriptor()
    {
        beginTest("descriptor: current, spelled-out ids, short names of six characters or fewer, ten models");
        const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.saturator");
        expect(d != nullptr);
        if (d == nullptr)
            return;
        expect(d->status == ModuleStatus::Current);
        bool idsOk = true, shortOk = true;
        for (const auto& p : d->params)
        {
            idsOk = idsOk && (p.id.rfind("fx_saturator_", 0) == 0 || p.id == "fx_pos_saturator");
            shortOk = shortOk && !p.shortName.empty() && p.shortName.size() <= 6;
        }
        expect(idsOk, "every id begins fx_saturator_");
        expect(shortOk, "short names");
        for (const auto& p : d->params)
            if (p.id == "fx_saturator_model")
            {
                expect(static_cast<int>(p.choices.size()) == kNumModels + 6, "ten earlier models, six shapers appended");
                for (int i = 0; i < kNumModels && i < static_cast<int>(p.choices.size()); ++i)
                    expect(p.choices[static_cast<size_t>(i)] == kModelNames[i], p.choices[static_cast<size_t>(i)]);
            }
    }

    void harmonics()
    {
        beginTest("triode: H2 above H3 at low drive; symmetric models: even harmonics below -90 dB");
        {
            const HarmonicPoint t = harmonicsAt(renderer(Triode, 6.0f), kRate, 1000.0, -24.0, 0.3, 0.3);
            std::printf("  triode 6 dB drive, -24 dBFS: H2 %.1f dB H3 %.1f dB\n", t.h2Db, t.h3Db);
            expect(t.h2Db > t.h3Db, "H2 " + String(t.h2Db, 1) + " H3 " + String(t.h3Db, 1));
            for (const int m : {Soft, Hard, Rail, Fold, PushPull, Tape, Transformer})
            {
                double worst = -300.0;
                for (const float drive : {6.0f, 18.0f, 30.0f})
                {
                    const HarmonicPoint h = harmonicsAt(renderer(m, drive), kRate, 1000.0, -12.0, 0.3, 0.3);
                    worst = std::max({worst, h.h2Db, h.h4Db});
                }
                std::printf("  %-14s even harmonics at most %.1f dB\n", kModelNames[m], worst);
                expect(worst <= -90.0, String(kModelNames[m]) + " " + String(worst, 1));
            }
        }

        beginTest("transformer: H3 at 50 Hz at least 10 dB above 1 kHz; tape loop area against a memoryless model");
        {
            const HarmonicPoint lo = harmonicsAt(renderer(Transformer, 18.0f), kRate, 50.0, -12.0, 0.6, 0.4);
            const HarmonicPoint hi = harmonicsAt(renderer(Transformer, 18.0f), kRate, 1000.0, -12.0, 0.3, 0.3);
            std::printf("  transformer H3 at 50 Hz %.1f dB, at 1 kHz %.1f dB, difference %.1f dB\n", lo.h3Db, hi.h3Db,
                        lo.h3Db - hi.h3Db);
            expect(lo.h3Db - hi.h3Db >= 10.0, String(lo.h3Db - hi.h3Db, 1));
            const size_t settle = 24000, count = 9600; // 200 cycles of 1 kHz (the loop is rate independent)
            const Signal in = sineWave(settle + count, 1000.0, kRate, 0.5);
            Sat t, s;
            t.config(Tape, 18.0f);
            t.set(t.idChar, 100.0f); // widest loop
            s.config(Soft, 18.0f);
            const size_t lat = static_cast<size_t>(t.latency->latencySamples());
            const double tape = loopArea(in, t.render(in), settle, count, lat, 200.0);
            const double soft = loopArea(in, s.render(in), settle, count, lat, 200.0);
            std::printf("  loop area per cycle, as a share of the peak box: tape %.4f, soft clip %.6f\n", tape, soft);
            expect(tape > 0.02 && tape > 3.0 * soft, "tape loop area " + String(tape, 5));
        }
    }

    void thd()
    {
        beginTest("THD is monotonic with drive (1 kHz, -20 dBFS, 3 dB steps, 0.3 dB slack)");
        for (int m = 0; m < kNumModels; ++m)
        {
            if (m == Fold) // folding is non-monotonic by construction: each fold re-opens a cleaner region
                continue;
            double previous = -300.0;
            bool ok = true;
            std::string row;
            for (int drive = 0; drive <= 36; drive += 3)
            {
                const HarmonicPoint h = harmonicsAt(renderer(m, static_cast<float>(drive)), kRate, 1000.0, -20.0, 0.3, 0.3);
                {
                    char cell[16];
                    std::snprintf(cell, sizeof cell, " %.0f", h.thdDb);
                    row += cell;
                }
                if (previous > -100.0 && h.thdDb < previous - 0.3) // below -100 dB it is the float noise floor
                    ok = false;
                previous = std::max(previous, h.thdDb);
            }
            std::printf("  %-14s THD dB at drive 0..36:%s\n", kModelNames[m], row.c_str());
            expect(ok, String(kModelNames[m]) + " THD not monotonic");
        }
    }

    void aliasing()
    {
        beginTest(
            "aliasing over the whole drive range (5 and 7 kHz, -6 dBFS): Standard at most -90 dB, High at most -100 dB");
        // The plan's rows hold for every model at every drive: the module moves from the lower oversampling factor to the
        // higher one (Standard 4x to 8x, High 8x to 16x) as the drive rises. The hysteresis has no antiderivative and
        // always runs at the higher factor; the switch of its relaxation (a jump in the second derivative at each
        // crossing of the anhysteretic curve) sets the floor at low drive: about -87 dB at Standard (8x) and -99 dB at High
        // (16x) on the test's drives, -82 and -95 over the whole sweep; at 36 dB the rows are met (-96 and -128).
        // Maintainer decision: the tape model is the one documented exception to the plan's alias rows. It is held
        // to -85 dB at Standard and -97 dB at High over the whole drive range (it meets -90 / -100 at full drive); every
        // other model is asserted at the plan's -90 / -100.
        constexpr double kRowStandardDb = -90.0;
        constexpr double kRowHighDb = -100.0;
        constexpr double kTapeExceptionStandardDb = -85.0;
        constexpr double kTapeExceptionHighDb = -97.0;
        const float drives[] = {6.0f, 12.0f, 15.0f, 18.0f, 21.0f, 24.0f, 30.0f, 36.0f};
        double worstStd = -300.0, worstHigh = -300.0, worstTapeStd = -300.0, worstTapeHigh = -300.0;
        for (int m = 0; m < kNumModels; ++m)
        {
            double std5 = -300.0, high5 = -300.0, std7 = -300.0, high7 = -300.0;
            for (const float d : drives)
            {
                std5 =
                    std::max(std5, aliasAtDrive(renderer(m, d, Quality::Standard), kRate, 5000.0, {-6.0}, 0.3)[0].aliasDb);
                high5 = std::max(high5, aliasAtDrive(renderer(m, d, Quality::High), kRate, 5000.0, {-6.0}, 0.3)[0].aliasDb);
                if (d == 18.0f || d == 36.0f || d == 12.0f)
                {
                    std7 = std::max(std7,
                                    aliasAtDrive(renderer(m, d, Quality::Standard), kRate, 7000.0, {-6.0}, 0.3)[0].aliasDb);
                    high7 =
                        std::max(high7, aliasAtDrive(renderer(m, d, Quality::High), kRate, 7000.0, {-6.0}, 0.3)[0].aliasDb);
                }
            }
            const double s36 = aliasAtDrive(renderer(m, 36.0f, Quality::Standard), kRate, 5000.0, {-6.0}, 0.3)[0].aliasDb;
            std::printf("  %-14s worst over drive 6..36: Standard 5 kHz %.1f, 7 kHz %.1f; High 5 kHz %.1f, 7 kHz %.1f "
                        "(Standard at 36 dB %.1f)\n",
                        kModelNames[m], std5, std7, high5, high7, s36);
            if (m == Tape)
            {
                worstTapeStd = std::max(std5, std7);
                worstTapeHigh = std::max(high5, high7);
                continue;
            }
            worstStd = std::max(worstStd, std::max(std5, std7));
            worstHigh = std::max(worstHigh, std::max(high5, high7));
            expect(std::max(std5, std7) <= kRowStandardDb,
                   String(kModelNames[m]) + " Standard " + String(std::max(std5, std7), 1));
            expect(std::max(high5, high7) <= kRowHighDb,
                   String(kModelNames[m]) + " High " + String(std::max(high5, high7), 1));
        }
        std::printf("  worst: Standard %.1f, High %.1f; hysteresis Standard %.1f, High %.1f\n", worstStd, worstHigh,
                    worstTapeStd, worstTapeHigh);
        expect(worstTapeStd <= kTapeExceptionStandardDb, "hysteresis Standard " + String(worstTapeStd, 1));
        expect(worstTapeHigh <= kTapeExceptionHighDb, "hysteresis High " + String(worstTapeHigh, 1));
    }

    void response()
    {
        beginTest("small-signal frequency response is flat to 20 kHz (soft clip, 6 dB drive, -60 dBFS)");
        for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
        {
            double worst = 0.0;
            std::string row;
            for (const double hz : {100.0, 1000.0, 5000.0, 10000.0, 15000.0, 18000.0, 20000.0})
            {
                Sat s(q);
                s.config(Soft, 6.0f);
                const Signal y = s.render(sineWave(24000, hz, kRate, 0.001));
                const double g =
                    amplitudeToDb(harmonicProfile(y.data() + 12000, 8192, kRate, hz, 2).fundamentalAmplitude / 0.001) -
                    6.0206;
                char cell[24];
                std::snprintf(cell, sizeof cell, " %.2f", g);
                row += cell;
                if (hz >= 1000.0)
                    worst = std::max(worst, std::abs(g));
            }
            std::printf("  quality %d gain in dB at 100 Hz, 1, 5, 10, 15, 18, 20 kHz:%s\n", static_cast<int>(q),
                        row.c_str());
            expect(worst <= (q == Quality::Draft ? 2.0 : 0.2),
                   "quality " + String(static_cast<int>(q)) + " " + String(worst, 3));
        }
    }

    void autoGain()
    {
        beginTest("auto gain keeps loudness within 0.5 LU across the drive range (pink noise, -18 dB RMS)");
        const Signal noise = pinkNoise(static_cast<size_t>(8.0 * kRate), -18.0, 987654321u);
        const double ref = measureLoudness({noise}, kRate).integratedLufs;
        double worstAll = 0.0;
        for (int m = 0; m < kNumModels; ++m)
        {
            double lo = 1.0e9, hi = -1.0e9;
            for (int drive = 0; drive <= 36; drive += 6)
            {
                const Signal y = renderer(m, static_cast<float>(drive), Quality::Standard, 0.0f, true)(noise);
                const double lu = measureLoudness({Signal(y.begin() + 4800, y.end())}, kRate).integratedLufs;
                lo = std::min(lo, lu - ref);
                hi = std::max(hi, lu - ref);
            }
            std::printf("  %-14s loudness change %.2f to %.2f LU across drive 0..36 dB\n", kModelNames[m], lo, hi);
            worstAll = std::max({worstAll, std::abs(lo), std::abs(hi)});
            expect(std::max(std::abs(lo), std::abs(hi)) <= 0.5,
                   String(kModelNames[m]) + " " + String(lo, 2) + " " + String(hi, 2));
        }
        std::printf("  worst deviation %.2f LU\n", worstAll);
        // The gain is calibrated at one level: the readout 6 dB hotter, for the record.
        const Signal hot = pinkNoise(static_cast<size_t>(8.0 * kRate), -12.0, 24681357u);
        const double refHot = measureLoudness({hot}, kRate).integratedLufs;
        double spread = 0.0;
        for (int m = 0; m < kNumModels; ++m)
        {
            const Signal y = renderer(m, 24.0f, Quality::Standard, 0.0f, true)(hot);
            spread = std::max(spread,
                              std::abs(measureLoudness({Signal(y.begin() + 4800, y.end())}, kRate).integratedLufs - refHot));
        }
        std::printf("  at -12 dB RMS (6 dB hotter than calibration), drive 24 dB: worst deviation %.2f LU\n", spread);
    }

    void blockSizes()
    {
        beginTest("the output does not depend on the host block size (1, 7, 512; every model, auto gain on)");
        const Signal noise = pinkNoise(static_cast<size_t>(0.6 * kRate), -14.0, 4242u);
        double worst = -300.0;
        for (int m = 0; m < kNumModels; ++m)
            for (const Quality q : {Quality::Standard, Quality::High})
            {
                Sat ref(q), a(q), b(q);
                for (Sat* s : {&ref, &a, &b})
                    s->config(m, 24.0f, 0.8f, true);
                const Signal y = ref.render(noise, 512), y1 = a.render(noise, 1), y7 = b.render(noise, 7);
                double diff = 0.0, peak = 1.0e-9;
                for (size_t i = 0; i < y.size(); ++i)
                {
                    diff = std::max({diff, std::abs(y[i] - y1[i]), std::abs(y[i] - y7[i])});
                    peak = std::max(peak, std::abs(y[i]));
                }
                const double db = diff <= 0.0 ? -300.0 : 20.0 * std::log10(diff / peak);
                worst = std::max(worst, db);
                expect(db < -90.0,
                       String(kModelNames[m]) + " quality " + String(static_cast<int>(q)) + " " + String(db, 1) + " dB");
            }
        std::printf("  worst difference between host blocks of 1, 7 and 512: %.1f dB\n", worst);
    }

    void nulls()
    {
        beginTest("mix 0 and drive at minimum null against the dry signal");
        const Signal noise = pinkNoise(static_cast<size_t>(1.0 * kRate), -12.0, 5551212u);
        for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
        {
            double worstMix = -300.0, worstDrive = -300.0;
            int latency = 0;
            for (int m = 0; m < kNumModels; ++m)
            {
                Sat a(q), b(q);
                a.config(m, 30.0f, 0.0f);
                b.config(m, 0.0f, 1.0f, true);
                latency = a.latency->latencySamples();
                worstMix = std::max(worstMix, nullTest(noise, a.render(noise), latency, 2400));
                worstDrive = std::max(worstDrive, nullTest(noise, b.render(noise), latency, 2400));
            }
            std::printf("  quality %d: latency %d samples, null at mix 0 %.1f dB, at drive 0 %.1f dB\n", static_cast<int>(q),
                        latency, worstMix, worstDrive);
            expect(worstMix <= -120.0, "mix 0 " + String(worstMix, 1));
            expect(worstDrive <= -100.0, "drive 0 " + String(worstDrive, 1));
        }

        beginTest("latency is a whole number of samples, the same for every model, and matches the measurement");
        {
            const Signal impulse = []
            {
                Signal x(4096, 0.0);
                x[1000] = 0.01;
                return x;
            }();
            for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
            {
                int reported = 0;
                double worstError = 0.0;
                for (int m = 0; m < kNumModels; ++m)
                {
                    Sat s(q);
                    reported = s.latency->latencySamples();
                    s.config(m, 1.0f, 1.0f);
                    const double measured = measureLatency(impulse, s.render(impulse));
                    worstError = std::max(worstError, std::abs(measured - reported));
                }
                std::printf(
                    "  quality %d: reported %d samples, worst measured difference over the ten models %.3f samples\n",
                    static_cast<int>(q), reported, worstError);
                expect(worstError < 0.15, "quality " + String(static_cast<int>(q)) + " " + String(worstError, 3));
            }
        }
    }

    void tilt()
    {
        beginTest("tilt: the emphasis pair is transparent while linear, and moves the distortion with frequency");
        {
            // A small signal through a model that is linear there: the pair of shelves is the identity (gain within 0.05
            // dB).
            double worst = 0.0;
            const auto gainAt = [](double hz, float tilt)
            {
                Sat s;
                s.config(Soft, 6.0f);
                s.set(s.idTilt, tilt);
                const Signal y = s.render(sineWave(24000, hz, kRate, 0.001));
                return amplitudeToDb(harmonicProfile(y.data() + 12000, 8192, kRate, hz, 2).fundamentalAmplitude / 0.001);
            };
            for (const double hz : {100.0, 1000.0, 8000.0})
            {
                const double flat = gainAt(hz, 0.0f);
                for (const float tilt : {-9.0f, 9.0f})
                {
                    const double e = gainAt(hz, tilt) - flat;
                    std::printf("    tilt %+.0f dB at %.0f Hz: gain change %.3f dB (flat gain %.3f dB)\n",
                                static_cast<double>(tilt), hz, e, flat);
                    worst = std::max(worst, std::abs(e));
                }
            }
            std::printf("  tilt +/-9 dB, 0.001 amplitude, 100 Hz / 1 kHz / 8 kHz: worst gain change %.3f dB\n", worst);
            expect(worst <= 0.06, String(worst, 3));
            const auto thdAt = [](double hz, float tilt)
            {
                Sat s;
                s.config(Soft, 12.0f);
                s.set(s.idTilt, tilt);
                const Signal y = s.render(sineWave(static_cast<size_t>(0.6 * kRate), hz, kRate, 0.1));
                return harmonicProfile(y.data() + 14400, 14400, kRate, hz, 5).thdDb;
            };
            const double hiUp = thdAt(3000.0, 12.0f), hiDown = thdAt(3000.0, -12.0f);
            const double loUp = thdAt(100.0, 12.0f), loDown = thdAt(100.0, -12.0f);
            std::printf("  THD 3 kHz: tilt +12 %.1f dB, -12 %.1f dB; 100 Hz: tilt +12 %.1f dB, -12 %.1f dB\n", hiUp, hiDown,
                        loUp, loDown);
            expect(hiUp > hiDown + 6.0, "highs distort more with a positive tilt");
            expect(loDown > loUp + 6.0, "lows distort more with a negative tilt");
        }
    }

    void switching()
    {
        beginTest("model changes while playing do not click (largest step against full scale)");
        {
            Sat s;
            s.config(Soft, 18.0f, 1.0f, true);
            const size_t n = static_cast<size_t>(1.2 * kRate);
            Samples l(n), r(n);
            for (size_t i = 0; i < n; ++i)
                l[i] = r[i] =
                    static_cast<float>(0.5 * std::sin(2.0 * 3.14159265358979 * 440.0 * static_cast<double>(i) / kRate));
            int next = 0;
            const int order[] = {Triode, Tape, Fold, Transformer, Diode, Soft};
            for (size_t p = 0; p < n; p += 128)
            {
                if (p > 0 && p % 7680 == 0 && next < 6)
                    s.set(s.idModel, static_cast<float>(order[next++]));
                s.run(l.data() + p, r.data() + p, static_cast<int>(std::min<size_t>(128, n - p)));
            }
            // Steps inside the 30 ms after each switch against the steps the models make by themselves (a folded or clipped
            // sine has corners of its own), both against full scale.
            const Signal y = toDouble(l);
            double during = -300.0, steady = -300.0;
            for (int k = 0; k < 6; ++k)
            {
                const size_t p = 7680u * static_cast<size_t>(k + 1);
                during = std::max(during, clickTest(y, 440.0, kRate, p, p + 1440).stepDb);
                steady = std::max(steady, clickTest(y, 440.0, kRate, p + 3000, p + 7000).stepDb);
            }
            std::printf("  six model changes at drive 18 dB: largest step %.1f dB in the 30 ms after a change, %.1f dB "
                        "between changes\n",
                        during, steady);
            expect(during <= std::max(steady, -50.0) + 3.0, String(during, 1));
        }
    }

    void pathSwitching()
    {
        beginTest("moving between the two oversampling factors with the drive does not click");
        // The drive hops across the limit of a model (Hard 15 dB at Standard, Soft 30 dB at High) every 160 ms, from just
        // above it to below the return point (1.5 dB under it); the largest step of the output after each hop is compared
        // with the steps of the steady sine.
        struct Case
        {
            int model;
            Quality quality;
            float low, high;
        };
        const Case cases[] = {{Hard, Quality::Standard, 13.0f, 15.5f}, {Pentode, Quality::Standard, 10.0f, 12.5f},
                              {Soft, Quality::Standard, 19.0f, 21.5f}, {Tape, Quality::Standard, 10.0f, 12.5f},
                              {Soft, Quality::High, 28.0f, 30.5f},     {Fold, Quality::High, 22.0f, 24.5f}};
        for (const Case& c : cases)
        {
            Sat s(c.quality);
            s.config(c.model, c.low, 1.0f, false);
            const size_t n = static_cast<size_t>(1.1 * kRate);
            Samples l(n), r(n);
            for (size_t i = 0; i < n; ++i)
                l[i] = r[i] =
                    static_cast<float>(0.25 * std::sin(2.0 * 3.14159265358979 * 440.0 * static_cast<double>(i) / kRate));
            constexpr size_t hop = 7680;
            for (size_t p = 0; p < n; p += 128)
            {
                s.set(s.idDrive, (p / hop) % 2 == 1 ? c.high : c.low);
                s.run(l.data() + p, r.data() + p, static_cast<int>(std::min<size_t>(128, n - p)));
            }
            const Signal y = toDouble(l);
            double during = -300.0, steady = -300.0;
            for (size_t k = 1; k <= 5; ++k)
            {
                during = std::max(during, clickTest(y, 440.0, kRate, hop * k, hop * k + 1440).stepDb);
                steady = std::max(steady, clickTest(y, 440.0, kRate, hop * k + 3000, hop * k + 7000).stepDb);
            }
            std::printf("  %-14s quality %d drive %.1f / %.1f dB: largest step %.1f dB in the 30 ms after a hop, %.1f dB "
                        "between hops\n",
                        kModelNames[c.model], static_cast<int>(c.quality), c.low, c.high, during, steady);
            expect(during <= std::max(steady, -60.0) + 3.0, String(kModelNames[c.model]) + " " + String(during, 1));
        }
    }

    void extremes()
    {
        beginTest("stable and finite at the extremes of every control (hot noise, 10 s per model)");
        {
            double peak = 0.0;
            bool finite = true;
            for (int m = 0; m < kNumModels; ++m)
            {
                Sat s;
                s.config(m, 36.0f, 1.0f, true);
                s.set(s.idBias, (m & 1) ? 100.0f : -100.0f);
                s.set(s.idTilt, (m & 2) ? 12.0f : -12.0f);
                s.set(s.idChar, (m & 4) ? 0.0f : 100.0f);
                s.set(s.idOut, 12.0f);
                const Signal x = pinkNoise(static_cast<size_t>(10.0 * kRate), -3.0, 77u + static_cast<std::uint32_t>(m));
                const Signal y = s.render(x);
                double modelPeak = 0.0;
                for (const double v : y)
                {
                    finite = finite && std::isfinite(v);
                    modelPeak = std::max(modelPeak, std::abs(v));
                }
                std::printf("  %-14s peak %.2f\n", kModelNames[m], modelPeak);
                peak = std::max(peak, modelPeak);
            }
            std::printf("  extremes: peak %.2f\n", peak);
            expect(finite);
            expect(peak < 100.0, String(peak, 2));
        }

        beginTest("deterministic: two renders are bit-identical");
        {
            const Signal x = pinkNoise(48000, -12.0, 4242u);
            bool same = true;
            for (int m = 0; m < kNumModels; ++m)
                same = same && renderer(m, 20.0f)(x) == renderer(m, 20.0f)(x);
            expect(same);
        }
    }

    void realtime()
    {
        beginTest("no allocation on the audio thread while models, drive and tilt change");
        {
            Sat s(Quality::High);
            s.config(Soft, 12.0f, 1.0f, true);
            Samples l(256, 0.3f), r(256, 0.3f);
            for (int i = 0; i < 40; ++i)
                s.run(l.data(), r.data(), 256);
            crucible::testing::gAllocations = 0;
            crucible::testing::gTrackAllocations = true;
            for (int i = 0; i < 400; ++i)
            {
                s.set(s.idModel, static_cast<float>((i / 5) % kNumModels));
                s.set(s.idDrive, static_cast<float>(i % 37));
                s.set(s.idTilt, static_cast<float>((i % 25) - 12));
                for (size_t k = 0; k < 256; ++k)
                    l[k] = r[k] = 0.4f * std::sin(0.05f * static_cast<float>(k + 256u * static_cast<size_t>(i)));
                s.run(l.data(), r.data(), 256);
            }
            crucible::testing::gTrackAllocations = false;
            std::printf("  allocations over 400 blocks: %ld\n", crucible::testing::gAllocations);
            expect(crucible::testing::gAllocations == 0, String(static_cast<int>(crucible::testing::gAllocations)));
        }
    }

    void cpu()
    {
        beginTest("CPU per quality (stereo, 48 kHz, 512-sample blocks, thread CPU time, best of three)");
#ifndef CRUCIBLE_TEST_UNTIMED
        // Plan: at most 0.5 % Standard and 1.5 % High, measured at the default drive (12 dB), where the lower oversampling
        // factor (4x / 8x) serves every ADAA model. At 36 dB the higher factor (8x / 16x) runs and the cost doubles. The
        // hysteresis always runs at the higher factor and is reported on its own. The limits below are ceilings with
        // headroom for a loaded machine (Standard 1.5 times the plan, High below it); measured on an idle core: Standard
        // 0.26 to 0.35 %, High 0.46 to 0.64 %; at 36 dB Standard 0.42 to 0.58 %, High 0.78 to 1.1 %; hysteresis 0.18,
        // 0.55 and 1.04 %, the plan's 0.5 and 1.5 % within the bench's 20 % margin).
        crucible::testing::ScopedUserInteractiveQos qos;
        const double limitDefault[3] = {0.5, 0.5, 1.5};
        const double limitMax[3] = {0.5, 0.6, 1.8};
        const double limitTape[3] = {0.4, 0.6, 1.5};
        for (const float drive : {12.0f, 36.0f})
        {
            for (const Quality q : {Quality::Draft, Quality::Standard, Quality::High})
            {
                double worst = 0.0;
                int worstModel = 0;
                double tape = 0.0;
                for (int m = 0; m < kNumModels; ++m)
                {
                    Sat s(q);
                    s.config(m, drive, 1.0f, true);
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
                            s.run(l.data(), r.data(), 512);
                            used += crucible::testing::threadCpuSeconds() - c0;
                        }
                        best = std::min(best, 100.0 * used / (static_cast<double>(blocks) * 512.0 / kRate));
                    }
                    std::printf("  drive %2.0f quality %d %-14s %.3f %% of a core\n", drive, static_cast<int>(q),
                                kModelNames[m], best);
                    if (m == Tape)
                        tape = best;
                    else if (best > worst)
                    {
                        worst = best;
                        worstModel = m;
                    }
                }
                std::printf("  drive %2.0f quality %d: worst model %s %.3f %%, hysteresis %.3f %%\n", drive,
                            static_cast<int>(q), kModelNames[worstModel], worst, tape);
                const double limit = drive < 20.0f ? limitDefault[static_cast<int>(q)] : limitMax[static_cast<int>(q)];
                expectCpu(worst <= limit,
                       "drive " + String(drive, 0) + " quality " + String(static_cast<int>(q)) + " " + String(worst, 3));
                expectCpu(tape <= limitTape[static_cast<int>(q)],
                       "hysteresis quality " + String(static_cast<int>(q)) + " " + String(tape, 3));
            }
        }
#endif
    }
};

static SaturatorTests saturatorTests;
