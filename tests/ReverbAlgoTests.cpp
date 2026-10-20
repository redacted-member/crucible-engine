// SPDX-License-Identifier: MIT
// Algorithmic reverb suite (FX6, docs/plans/ENGINE-FX-PLAN.md section 3.5): reverberation time per octave band, decay
// curve, echo density build-up, tail flatness, stereo decorrelation, modulation pitch spread, silence and denormals,
// freeze, algorithm changes, controls, bounded clearing, allocations, determinism and CPU of `fx.reverb_algo` and the
// Fdn / EarlyReflections / PlateTank cores. Every claim is a number; the printed lines are what docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/fx/Fdn.h>
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
const char* const kNames[] = {"Room", "Hall", "Chamber", "Plate", "Ambience"};

struct Stereo
{
    Signal l, r;
};

Stereo runReverb(const ReverbSettings& s, const std::vector<float>& inL, const std::vector<float>& inR, double rate = kRate,
                 int lines = 16, int block = 128, std::uint32_t seed = 1)
{
    AlgorithmicReverb rv;
    rv.prepare(rate, lines, seed);
    std::vector<float> l = inL, r = inR;
    for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos));
        rv.process(l.data() + pos, r.data() + pos, n, s);
    }
    return {Signal(l.begin(), l.end()), Signal(r.begin(), r.end())};
}

Stereo impulseResponse(const ReverbSettings& s, double seconds, double rate = kRate, int lines = 16)
{
    std::vector<float> a(static_cast<size_t>(seconds * rate), 0.0f);
    a[0] = 1.0f;
    return runReverb(s, a, a, rate, lines);
}

// Wet only, flat decay in every band, no cuts, no pre-delay: the setting every reverberation-time number is taken at.
ReverbSettings neutral(ReverbAlgorithm a, double decay)
{
    ReverbSettings s;
    s.algorithm = a;
    s.mix = 1.0f;
    s.decaySeconds = static_cast<float>(decay);
    s.lowMult = 1.0f;
    s.highMult = 1.0f;
    s.lowCutHz = 20.0f;
    s.highCutHz = 20000.0f;
    s.predelayMs = 0.0f;
    return s;
}

double decayFor(ReverbAlgorithm a)
{
    return a == ReverbAlgorithm::Ambience ? 0.8 : 2.0;
}

double rmsDb(const Signal& x, size_t from, size_t to)
{
    double s = 0.0;
    to = std::min(to, x.size());
    for (size_t i = from; i < to; ++i)
        s += x[i] * x[i];
    return powerToDb(s / static_cast<double>(std::max<size_t>(1, to - from)));
}

double peakAbs(const Signal& x, size_t from = 0, size_t to = 0)
{
    if (to == 0 || to > x.size())
        to = x.size();
    double m = 0.0;
    for (size_t i = from; i < to; ++i)
        m = std::max(m, std::abs(x[i]));
    return m;
}

// Level in a band of the part of x after the first second.
double bandDb(const Signal& x, double lo, double hi)
{
    const size_t start = static_cast<size_t>(kRate);
    const size_t n = floorPow2(x.size() - start);
    return bandRmsDbfs(x.data() + start, n, kRate, lo, hi);
}

// Largest deviation (dB) of the Schroeder decay curve from its least-squares line between -5 and -35 dB.
double decayCurveDeviation(const Signal& ir)
{
    const Signal edc = detail::schroederDb(ir);
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    size_t n = 0;
    for (size_t i = 0; i < edc.size(); ++i)
        if (edc[i] <= -5.0 && edc[i] >= -35.0)
        {
            const double t = static_cast<double>(i);
            sx += t;
            sy += edc[i];
            sxx += t * t;
            sxy += t * edc[i];
            ++n;
        }
    if (n < 8)
        return 99.0;
    const double dn = static_cast<double>(n);
    const double slope = (dn * sxy - sx * sy) / (dn * sxx - sx * sx);
    const double icpt = (sy - slope * sx) / dn;
    double worst = 0.0;
    for (size_t i = 0; i < edc.size(); ++i)
        if (edc[i] <= -5.0 && edc[i] >= -35.0)
            worst = std::max(worst, std::abs(edc[i] - (slope * static_cast<double>(i) + icpt)));
    return worst;
}

// First time (ms) at which the normalised echo density is at least `level`.
double densityTimeMs(const Signal& density, double rate, double level)
{
    for (size_t i = 0; i < density.size(); ++i)
        if (density[i] >= level)
            return 1000.0 * static_cast<double>(i) / rate;
    return -1.0;
}

// Power-weighted spread of the spectrum around a steady sine, in cents (the pitch smear the modulation causes).
double spreadCents(const Signal& x, double hz, double rate)
{
    const size_t n = floorPow2(x.size());
    std::vector<Cx> a(n);
    for (size_t i = 0; i < n; ++i)
        a[i] = x[x.size() - n + i] * (0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(i) / static_cast<double>(n)));
    fft(a);
    double sw = 0, sf = 0, sff = 0;
    for (size_t k = 1; k < n / 2; ++k)
    {
        const double f = static_cast<double>(k) * rate / static_cast<double>(n);
        if (std::abs(f - hz) > 150.0)
            continue;
        const double p = std::norm(a[k]);
        sw += p;
        sf += p * f;
        sff += p * f * f;
    }
    const double mean = sf / sw, var = sff / sw - mean * mean;
    return 1731.2 * std::sqrt(std::max(var, 0.0)) / hz;
}

// A module through the rack with every other effect off.
struct RackHarness
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values;
    std::vector<float> legacy;
    explicit RackHarness(double rate = kRate, Quality quality = Quality::Standard)
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        ctx.quality = quality;
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

#if !defined(CRUCIBLE_TEST_UNTIMED)
// Percent of one core per stereo instance at 48 kHz, blocks of 32, minus the rack with everything off; best of three.
double cpuPercent(const std::vector<std::pair<std::string, float>>& setup, Quality quality)
{
    const size_t n = static_cast<size_t>(8.0 * kRate);
    Random rng(5);
    std::vector<float> src(n);
    for (auto& v : src)
        v = static_cast<float>(0.25 * rng.gaussian());
    const auto run = [&](bool enabled)
    {
        RackHarness h(kRate, quality);
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
#endif
} // namespace

class ReverbAlgoTests : public UnitTest
{
public:
    ReverbAlgoTests() : UnitTest("Algorithmic reverb", "Effects") {}

    void runTest() override
    {
        buildingBlocks();
        reverberationTime();
        shapedDecay();
        decayCurveAndDensity();
        spectrumAndStereo();
        modulation();
        silenceAndDenormals();
        freeze();
        algorithmChange();
        controls();
        clearing();
        stress();
        module();
        realTime();
    }

private:
    // ---- cores
    void buildingBlocks()
    {
        beginTest("sinc kernel: DC gain, no phase gains above one, droop of the worst phase");
        {
            const float* k = sincKernel();
            double peak = 0.0, dcWorst = 1.0;
            double droop[3] = {0, 0, 0};
            const double fractions[3] = {0.1, 0.2, 0.3};
            for (int p = 0; p < kSincPhases; ++p)
            {
                double dc = 0.0;
                for (int t = 0; t < kSincTaps; ++t)
                    dc += k[p * kSincTaps + t];
                dcWorst = std::min(dcWorst, dc);
                for (int f = 0; f < 3; ++f)
                {
                    const double w = 2.0 * kPi * fractions[f];
                    Cx h(0, 0);
                    for (int t = 0; t < kSincTaps; ++t)
                        h += static_cast<double>(k[p * kSincTaps + t]) * std::polar(1.0, -w * t);
                    droop[f] = std::min(droop[f], amplitudeToDb(std::abs(h)));
                }
                for (int m = 0; m <= 200; ++m)
                {
                    const double w = kPi * m / 200.0;
                    Cx h(0, 0);
                    for (int t = 0; t < kSincTaps; ++t)
                        h += static_cast<double>(k[p * kSincTaps + t]) * std::polar(1.0, -w * t);
                    peak = std::max(peak, std::abs(h));
                }
            }
            logMessage(String("sinc kernel: largest magnitude ") + String(amplitudeToDb(peak), 4) +
                       " dB, smallest DC gain " + String(amplitudeToDb(dcWorst), 4) +
                       " dB, worst-phase droop at 0.1 / 0.2 / 0.3 fs " + String(droop[0], 3) + " / " + String(droop[1], 3) +
                       " / " + String(droop[2], 3) + " dB");
            expect(peak <= 1.0 + 1.0e-6, "no gain above one");
            expect(amplitudeToDb(dcWorst) > -0.05, "DC gain");
            expect(droop[1] > -0.2, "droop at 0.2 fs");
        }

        beginTest("absorption design: the filter's decay time at DC and Nyquist is the target (Jot)");
        {
            double worst = 0.0;
            for (const float u : {0.005f, 0.03f, 0.12f})
                for (const float decay : {0.4f, 2.5f, 20.0f})
                {
                    DecayTarget t;
                    t.decay = decay;
                    t.lowMult = 1.6f;
                    t.highMult = 0.4f;
                    t.lowHz = 300.0f;
                    t.highHz = 3000.0f;
                    const float mid = absorptionDecayAt(1000.0f, kRate, u, t);
                    const float lo = absorptionDecayAt(2.0f, kRate, u, t);
                    const float hi = absorptionDecayAt(23990.0f, kRate, u, t);
                    // The mid band sits one octave from each crossover, so it is only roughly the middle value; DC and
                    // Nyquist are exact.
                    worst = std::max({worst, static_cast<double>(std::abs(lo / (decay * 1.6f) - 1.0f)),
                                      static_cast<double>(std::abs(hi / (decay * 0.4f) - 1.0f))});
                    expect(mid > decay * 0.45f && mid < decay * 1.2f, "middle band");
                }
            logMessage(String("absorption design: worst error of the DC and Nyquist decay times ") +
                       String(100.0 * worst, 4) + " %");
            expect(worst < 0.01, "DC and Nyquist decay times");
        }
    }

    // ---- reverberation time
    void reverberationTime()
    {
        beginTest("RT60 (T30) per octave band, 125 Hz to 8 kHz, flat decay setting");
        {
            for (const int lines : {16, 32})
                for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
                {
                    if (lines == 32 && a == static_cast<int>(ReverbAlgorithm::Plate))
                        continue;
                    const double decay = decayFor(static_cast<ReverbAlgorithm>(a));
                    const Stereo ir =
                        impulseResponse(neutral(static_cast<ReverbAlgorithm>(a), decay), 4.5 * decay + 1.0, kRate, lines);
                    const auto bands = energyDecay(ir.l, kRate);
                    double worst = 0.0;
                    String line = String(kNames[a]) + " " + String(lines) + " lines, set " + String(decay, 1) + " s: T30";
                    for (const auto& b : bands)
                    {
                        worst = std::max(worst, std::abs(b.t30S / decay - 1.0));
                        line = line + " " + String(b.t30S, 2);
                    }
                    logMessage(line + " s, worst error " + String(100.0 * worst, 1) + " %");
                    expect(worst < 0.10, String(kNames[a]) + " " + String(lines) + " lines RT60 within 10 %");
                }
        }

        beginTest("RT60 does not depend on the size setting, and holds at short and long decay settings");
        {
            double worst = 0.0;
            for (const float size : {0.0f, 1.0f})
                for (const int a : {1, 3})
                {
                    ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(a), 2.0);
                    s.size = size;
                    const Stereo ir = impulseResponse(s, 9.0);
                    const auto bands = energyDecay(ir.l, kRate, {250, 1000, 4000});
                    String line = String(kNames[a]) + " size " + String(size, 1) + ": T30";
                    for (const auto& b : bands)
                    {
                        worst = std::max(worst, std::abs(b.t30S / 2.0 - 1.0));
                        line = line + " " + String(b.t30S, 3);
                    }
                    logMessage(line);
                }
            logMessage(String("size 0 and 1, Hall and Plate, set 2 s: worst RT60 error ") + String(100.0 * worst, 1) + " %");
            expect(worst < 0.10, "size independent RT60");
            double worstDecay = 0.0;
            for (const int a : {1, 3})
                for (const double decay : {0.3, 10.0})
                {
                    const Stereo ir = impulseResponse(neutral(static_cast<ReverbAlgorithm>(a), decay), 4.5 * decay + 1.0);
                    const auto bands = energyDecay(ir.l, kRate, {250, 1000, 4000});
                    String line = String(kNames[a]) + " set " + String(decay, 1) + " s: T30";
                    for (const auto& b : bands)
                    {
                        worstDecay = std::max(worstDecay, std::abs(b.t30S / decay - 1.0));
                        line = line + " " + String(b.t30S, 3);
                    }
                    logMessage(line);
                }
            logMessage(String("Hall and Plate at 0.3 s and 10 s: worst RT60 error ") + String(100.0 * worstDecay, 1) + " %");
            expect(worstDecay < 0.10, "short and long decay");
        }

        beginTest("RT60 at 44.1, 96 and 192 kHz");
        {
            for (const double rate : {44100.0, 96000.0, 192000.0})
            {
                double worst = 0.0;
                for (const int a : {1, 3})
                {
                    const Stereo ir = impulseResponse(neutral(static_cast<ReverbAlgorithm>(a), 2.0), 8.0, rate);
                    const auto bands = energyDecay(ir.l, rate, {250, 1000, 4000});
                    for (const auto& b : bands)
                        worst = std::max(worst, std::abs(b.t30S / 2.0 - 1.0));
                }
                logMessage(String("rate ") + String(rate, 0) + " Hz, Hall and Plate: worst RT60 error " +
                           String(100.0 * worst, 1) + " %");
                expect(worst < 0.10, String("RT60 at ") + String(rate, 0));
            }
        }
    }

    void shapedDecay()
    {
        beginTest("low and high decay multipliers: each band follows the shelf design (Hall, Chamber and Plate)");
        {
            for (const int a : {1, 2, 3})
            {
                ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(a), 3.0);
                s.lowMult = 1.6f;
                s.highMult = 0.35f;
                s.lowHz = 400.0f;
                s.highHz = 3000.0f;
                AlgorithmicReverb probe;
                probe.prepare(kRate, 16, 1);
                std::vector<float> a0(64, 0.0f), b0(64, 0.0f);
                probe.process(a0.data(), b0.data(), 64, s);
                const double u = probe.loopSeconds();
                DecayTarget t;
                t.decay = 3.0f;
                t.lowMult = 1.6f;
                t.highMult = 0.35f;
                t.lowHz = 400.0f;
                t.highHz = 3000.0f;
                const Stereo ir = impulseResponse(s, 15.0);
                const auto bands = energyDecay(ir.l, kRate);
                double worst = 0.0;
                String line = String(kNames[a]) + ": T30 measured / designed";
                for (const auto& b : bands)
                {
                    const double expected =
                        absorptionDecayAt(static_cast<float>(b.centreHz), kRate, static_cast<float>(u), t);
                    worst = std::max(worst, std::abs(b.t30S / expected - 1.0));
                    line = line + " " + String(b.t30S, 2) + "/" + String(expected, 2);
                }
                logMessage(line + " s, worst " + String(100.0 * worst, 1) + " %");
                expect(worst < 0.15, String(kNames[a]) + " shaped RT60 within 15 % of the design");
                // The ends of the range reach for the multipliers: 125 Hz is longer than the middle, 8 kHz much shorter.
                expect(bands.front().t30S > 3.0 * 1.2, "low band longer");
                expect(bands.back().t30S < 3.0 * 0.6, "high band shorter");
            }
        }
    }

    // ---- decay curve, build-up
    void decayCurveAndDensity()
    {
        beginTest("energy decay curve straight within 1 dB from -5 to -35 dB; echo density build-up");
        {
            for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
            {
                const double decay = decayFor(static_cast<ReverbAlgorithm>(a));
                const Stereo ir = impulseResponse(neutral(static_cast<ReverbAlgorithm>(a), decay), 4.5 * decay + 1.0);
                const double dev = decayCurveDeviation(ir.l);
                const Signal head(ir.l.begin(), ir.l.begin() + static_cast<long>(0.6 * kRate));
                const Signal d = echoDensity(head, kRate);
                const double t95 = densityTimeMs(d, kRate, 0.95), t100 = densityTimeMs(d, kRate, 1.0);
                logMessage(String(kNames[a]) + ": decay curve deviation " + String(dev, 2) +
                           " dB, echo density reaches 0.95 at " + String(t95, 1) + " ms, 1.0 at " + String(t100, 1) + " ms");
                expect(dev <= 1.0, String(kNames[a]) + " decay curve");
                // The estimator of a Gaussian tail scatters by 0.05 (one sigma) around 1: "reached" is 0.95.
                const double limit = a == static_cast<int>(ReverbAlgorithm::Plate) ? 20.0 : 80.0;
                expect(t95 >= 0.0 && t95 <= limit, String(kNames[a]) + " echo density build-up");
            }
        }
    }

    // ---- spectrum and stereo
    void spectrumAndStereo()
    {
        beginTest(
            "late-tail spectrum: 1/3-octave levels flat (no metallic ringing); interaural correlation; mono compatibility");
        {
            for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
            {
                const double decay = decayFor(static_cast<ReverbAlgorithm>(a));
                const Stereo ir = impulseResponse(neutral(static_cast<ReverbAlgorithm>(a), decay), 4.0 * decay + 1.0);
                const double flat = tailSpectralFlatness(ir.l, kRate, 0.3);
                const double flatR = tailSpectralFlatness(ir.r, kRate, 0.3);
                const double iaccValue = iacc(ir.l, ir.r, kRate);
                const Signal mono = monoSumPerBand(ir.l, ir.r, kRate, {125, 250, 500, 1000, 2000, 4000, 8000});
                double lo = 99.0, hi = -99.0;
                for (const double v : mono)
                {
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                }
                logMessage(String(kNames[a]) + ": tail flatness " + String(flat, 2) + " / " + String(flatR, 2) +
                           " dB (L / R), IACC " + String(iaccValue, 3) + ", mono sum against stereo energy " +
                           String(lo, 2) + " to " + String(hi, 2) + " dB per octave band");
                expect(flat <= 2.0 && flatR <= 2.0, String(kNames[a]) + " tail flatness");
                expect(iaccValue <= 0.3, String(kNames[a]) + " IACC");
                // Independent channels sum to -3 dB; the rule is no more than 3 dB either way.
                expect(lo >= -6.0 && hi <= 0.5, String(kNames[a]) + " mono sum");
            }
        }

        beginTest("stereo width: 0 is mono, the correlation falls as the width grows");
        {
            ReverbSettings s = neutral(ReverbAlgorithm::Hall, 2.0);
            s.width = 0.0f;
            const Stereo mono = impulseResponse(s, 3.0);
            double worst = 0.0;
            for (size_t i = 0; i < mono.l.size(); ++i)
                worst = std::max(worst, std::abs(mono.l[i] - mono.r[i]));
            expect(worst < 1.0e-6, "width 0 gives equal channels");
            double previous = 2.0;
            String line = "IACC over width 0 / 0.5 / 1: ";
            for (const float w : {0.0f, 0.5f, 1.0f})
            {
                s.width = w;
                const Stereo ir = impulseResponse(s, 1.0);
                const double v = iacc(ir.l, ir.r, kRate);
                line = line + String(v, 3) + " ";
                expect(v < previous + 1.0e-9, "correlation falls as the width grows");
                previous = v;
            }
            logMessage(line);
        }
    }

    // ---- modulation
    void modulation()
    {
        beginTest("modulation: pitch spread of a 1 kHz sine at the default depth stays within 3 cents, grows with depth, is "
                  "nil without");
        {
            const size_t n = static_cast<size_t>(10.0 * kRate);
            std::vector<float> in(n);
            for (size_t i = 0; i < n; ++i)
                in[i] = static_cast<float>(0.3 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
            double floorCents = 0.0;
            {
                ReverbSettings s = neutral(ReverbAlgorithm::Hall, 3.0);
                s.modDepth = 0.0f;
                const Stereo o = runReverb(s, in, in);
                floorCents = spreadCents(Signal(o.l.begin() + 5 * 48000, o.l.end()), 1000.0, kRate);
            }
            String line = String("pitch spread (cents RMS, measurement floor ") + String(floorCents, 2) + "): ";
            for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
            {
                double previous = 0.0;
                for (const float depth : {0.3f, 1.0f})
                {
                    ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(a), 3.0);
                    s.modDepth = depth;
                    const Stereo o = runReverb(s, in, in);
                    const double c = spreadCents(Signal(o.l.begin() + 5 * 48000, o.l.end()), 1000.0, kRate);
                    if (depth == 0.3f)
                        expect(c <= 3.0, String(kNames[a]) + " default depth pitch spread");
                    else
                        expect(c > previous, String(kNames[a]) + " spread grows with depth");
                    line = line + String(kNames[a]) + " " + String(depth, 1) + ":" + String(c, 2) + " ";
                    previous = c;
                }
            }
            logMessage(line);
            expect(floorCents < 0.6, "no modulation, no spread");
        }
    }

    // ---- silence, denormals
    void silenceAndDenormals()
    {
        beginTest("silence in gives silence out below -150 dBFS; a decayed tail stays out of the denormal range and costs "
                  "no more than a live one");
        {
            for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
            {
                ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(a), 1.5);
                s.mix = 0.6f;
                std::vector<float> silent(static_cast<size_t>(2.0 * kRate), 0.0f);
                const Stereo o = runReverb(s, silent, silent);
                expect(peakAbs(o.l) < 3.0e-8 && peakAbs(o.r) < 3.0e-8,
                       String(kNames[a]) + " silence (" + String(amplitudeToDb(peakAbs(o.l)), 1) + " dBFS)");
            }
            // Live noise, then digital silence for 40 s: no subnormal number anywhere, and the cost per block does not rise.
            double worstRatio = 0.0;
            long subnormal = 0;
            for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
            {
                ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(a), 1.5);
                AlgorithmicReverb rv;
                rv.prepare(kRate, 16, 1);
                std::vector<float> l(64, 0.0f), r(64, 0.0f);
                Random rng(2);
                const auto timeBlocks = [&](bool noise, int blocks)
                {
                    const auto t0 = std::chrono::steady_clock::now();
                    for (int b = 0; b < blocks; ++b)
                    {
                        for (size_t i = 0; i < 64; ++i)
                            l[i] = r[i] = noise ? static_cast<float>(0.2 * rng.gaussian()) : 0.0f;
                        rv.process(l.data(), r.data(), 64, s);
                        for (size_t i = 0; i < 64; ++i)
                            if (std::fpclassify(l[i]) == FP_SUBNORMAL || std::fpclassify(r[i]) == FP_SUBNORMAL)
                                ++subnormal;
                    }
                    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                };
                const double live = timeBlocks(true, 1500);
                (void)timeBlocks(false, 4000); // the tail fades out
                const double silentTime = timeBlocks(false, 1500);
                worstRatio = std::max(worstRatio, silentTime / std::max(live, 1.0e-9));
                for (int b = 0; b < 40; ++b)
                    (void)timeBlocks(false, 750);
            }
            logMessage(String("silent / live block time, worst algorithm ") + String(worstRatio, 2) +
                       ", subnormal output samples " + String(static_cast<double>(subnormal), 0));
            expectEquals(static_cast<int>(subnormal), 0, "subnormal samples");
#if !defined(CRUCIBLE_TEST_UNTIMED)
            expect(worstRatio < 1.5, "silence costs no more than live input");
#endif
        }
    }

    // ---- freeze
    void freeze()
    {
        beginTest("freeze: 60 s hold within 0.5 dB, bounded; the tail decays again after release");
        {
            for (const int lines : {16, 32})
                for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
                {
                    if (lines == 32 && a == static_cast<int>(ReverbAlgorithm::Plate))
                        continue;
                    ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(a), 2.0);
                    AlgorithmicReverb rv;
                    rv.prepare(kRate, lines, 1);
                    const size_t n = static_cast<size_t>(62.0 * kRate);
                    std::vector<float> l(n, 0.0f), r(n, 0.0f);
                    Random rng(3);
                    for (size_t i = 0; i < 48000; ++i)
                        l[i] = r[i] = static_cast<float>(0.3 * rng.gaussian());
                    for (size_t pos = 0; pos < n; pos += 128)
                    {
                        s.freeze = pos >= 48000 && pos < static_cast<size_t>(60.0 * kRate);
                        rv.process(l.data() + pos, r.data() + pos, 128, s);
                    }
                    const Signal sl(l.begin(), l.end()), sr(r.begin(), r.end());
                    double lo = 99.0, hi = -99.0;
                    for (int w = 0; w < 5; ++w)
                    {
                        const size_t from = static_cast<size_t>((2 + 10 * w) * kRate),
                                     to = static_cast<size_t>((12 + 10 * w) * kRate);
                        const double v = powerToDb(std::pow(10.0, rmsDb(sl, from, to) / 10.0) +
                                                   std::pow(10.0, rmsDb(sr, from, to) / 10.0));
                        lo = std::min(lo, v);
                        hi = std::max(hi, v);
                    }
                    const double peak = std::max(peakAbs(sl), peakAbs(sr));
                    const double afterRelease =
                        rmsDb(sl, static_cast<size_t>(61.0 * kRate), static_cast<size_t>(62.0 * kRate));
                    const double before = rmsDb(sl, static_cast<size_t>(50.0 * kRate), static_cast<size_t>(55.0 * kRate));
                    logMessage(String(kNames[a]) + " " + String(lines) + " lines: level over five 10 s windows varies by " +
                               String(hi - lo, 2) + " dB, peak " + String(peak, 2) + ", 1.5 s after release " +
                               String(afterRelease - before, 1) + " dB");
                    expect(hi - lo <= 0.5, String(kNames[a]) + " " + String(lines) + " frozen level");
                    expect(peak < 8.0, "bounded");
                    expect(afterRelease - before < -20.0, "decays after release");
                }
        }
    }

    // ---- algorithm change
    void algorithmChange()
    {
        beginTest("changing the algorithm fades out, clears and fades in: no click above -60 dB, no old tail");
        {
            double worstClick = -300.0, worstLeak = -300.0;
            for (int from = 0; from < 5; ++from)
                for (int to = 0; to < 5; ++to)
                {
                    if (from == to)
                        continue;
                    ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(from), 2.0);
                    s.mix = 0.5f;
                    AlgorithmicReverb rv;
                    rv.prepare(kRate, 16, 1);
                    const size_t n = static_cast<size_t>(3.0 * kRate);
                    std::vector<float> l(n), r(n);
                    for (size_t i = 0; i < n; ++i)
                        l[i] = r[i] = static_cast<float>(0.4 * std::sin(2.0 * kPi * 997.0 * static_cast<double>(i) / kRate));
                    const size_t switchAt = static_cast<size_t>(1.5 * kRate);
                    for (size_t pos = 0; pos < n; pos += 32)
                    {
                        s.algorithm = static_cast<ReverbAlgorithm>(pos >= switchAt ? to : from);
                        rv.process(l.data() + pos, r.data() + pos, 32, s);
                    }
                    const ClickResult c = clickTest(toDouble(l, switchAt - 4800, 28800), 997.0, kRate);
                    worstClick = std::max(worstClick, c.stepDb);
                    // The input stops at the change: the old tail must not come back through the new algorithm.
                    ReverbSettings q = neutral(static_cast<ReverbAlgorithm>(from), 2.0);
                    AlgorithmicReverb rv2;
                    rv2.prepare(kRate, 16, 1);
                    std::vector<float> a2(static_cast<size_t>(4.0 * kRate), 0.0f), b2;
                    Random rng(9);
                    for (size_t i = 0; i < 24000; ++i)
                        a2[i] = static_cast<float>(0.3 * rng.gaussian());
                    b2 = a2;
                    const size_t sw2 = static_cast<size_t>(1.0 * kRate);
                    for (size_t pos = 0; pos < a2.size(); pos += 32)
                    {
                        q.algorithm = static_cast<ReverbAlgorithm>(pos >= sw2 ? to : from);
                        rv2.process(a2.data() + pos, b2.data() + pos, 32, q);
                    }
                    const Signal wet(a2.begin(), a2.end());
                    worstLeak = std::max(worstLeak, rmsDb(wet, sw2 + static_cast<size_t>(0.15 * kRate),
                                                          sw2 + static_cast<size_t>(0.35 * kRate)) -
                                                        rmsDb(wet, sw2 - 4800, sw2));
                }
            logMessage(String("algorithm change: largest click ") + String(worstClick, 1) +
                       " dBFS, level 150 to 350 ms after the change against the level before it " + String(worstLeak, 1) +
                       " dB (worst of the 20 pairs)");
            expect(worstClick < -60.0, "click");
            expect(worstLeak < -20.0, "old tail gone");
        }
    }

    // ---- controls
    void controls()
    {
        beginTest("pre-delay: the wet signal starts at the set time");
        {
            double worst = 0.0;
            for (const float ms : {10.0f, 50.0f, 200.0f})
                for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
                {
                    ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(a), 1.0);
                    s.predelayMs = ms;
                    const Stereo ir = impulseResponse(s, 0.5);
                    size_t first = 0;
                    while (first < ir.l.size() && std::abs(ir.l[first]) < 1.0e-4)
                        ++first;
                    const double at = 1000.0 * static_cast<double>(first) / kRate;
                    worst = std::max(worst, std::abs(at - ms));
                    expect(at >= ms - 0.1, "never before the pre-delay (the sinc read leads by at most four samples)");
                }
            logMessage(String("pre-delay 10 / 50 / 200 ms: first sample above -80 dB within ") + String(worst, 2) +
                       " ms of the setting (all algorithms)");
            expect(worst < 1.0, "pre-delay");
        }

        beginTest("mix 0 is the input bit for bit; wet low and high cut; ducking");
        {
            Random rng(4);
            std::vector<float> in(static_cast<size_t>(1.0 * kRate));
            for (auto& v : in)
                v = static_cast<float>(0.3 * rng.gaussian());
            ReverbSettings s = neutral(ReverbAlgorithm::Hall, 2.0);
            s.mix = 0.0f;
            const Stereo dry = runReverb(s, in, in);
            long different = 0;
            for (size_t i = 0; i < in.size(); ++i)
                if (static_cast<float>(dry.l[i]) != in[i])
                    ++different;
            expectEquals(static_cast<int>(different), 0, "samples that differ at mix 0");

            // Wet filters on noise: a 500 Hz low cut removes the lows, a 2 kHz high cut the highs.
            ReverbSettings f = neutral(ReverbAlgorithm::Hall, 2.0);
            std::vector<float> noise(static_cast<size_t>(4.0 * kRate));
            for (auto& v : noise)
                v = static_cast<float>(0.3 * rng.gaussian());
            const Stereo flat = runReverb(f, noise, noise);
            f.lowCutHz = 500.0f;
            const Stereo hp = runReverb(f, noise, noise);
            f.lowCutHz = 20.0f;
            f.highCutHz = 2000.0f;
            const Stereo lp = runReverb(f, noise, noise);
            const double lowLoss = bandDb(hp.l, 50, 125) - bandDb(flat.l, 50, 125);
            const double highLoss = bandDb(lp.l, 6000, 12000) - bandDb(flat.l, 6000, 12000);
            const double passLow = bandDb(hp.l, 2000, 4000) - bandDb(flat.l, 2000, 4000);
            const double passHigh = bandDb(lp.l, 200, 800) - bandDb(flat.l, 200, 800);
            logMessage(String("wet cuts: low cut 500 Hz -> ") + String(lowLoss, 1) + " dB at 50-125 Hz (" +
                       String(passLow, 2) + " dB at 2-4 kHz); high cut 2 kHz -> " + String(highLoss, 1) +
                       " dB at 6-12 kHz (" + String(passHigh, 2) + " dB at 200-800 Hz)");
            expect(lowLoss < -12.0 && std::abs(passLow) < 1.0, "low cut");
            expect(highLoss < -12.0 && std::abs(passHigh) < 1.0, "high cut");

            // Ducking: a loud burst, then silence; the wet level during the burst falls by the ducking depth and returns.
            std::vector<float> burst(static_cast<size_t>(3.0 * kRate), 0.0f);
            for (size_t i = 0; i < static_cast<size_t>(1.0 * kRate); ++i)
                burst[i] = static_cast<float>(0.5 * rng.gaussian());
            ReverbSettings d = neutral(ReverbAlgorithm::Hall, 2.0);
            d.duckReleaseMs = 200.0f;
            const Stereo plain = runReverb(d, burst, burst);
            d.duck = 1.0f;
            const Stereo ducked = runReverb(d, burst, burst);
            const double during = rmsDb(ducked.l, 24000, 40000) - rmsDb(plain.l, 24000, 40000);
            const size_t t0 = static_cast<size_t>(2.0 * kRate), t1 = static_cast<size_t>(2.5 * kRate);
            const double later = rmsDb(ducked.l, t0, t1) - rmsDb(plain.l, t0, t1);
            logMessage(String("ducking 100 %: wet level during the burst ") + String(during, 1) + " dB, 1 s after it " +
                       String(later, 1) + " dB against no ducking");
            expect(during < -25.0 && during > -35.0, "ducking depth");
            expect(later > -1.0, "ducking releases");
        }

        beginTest("low cut set to 20 Hz (bypass) leaves no DC offset from the frozen high-pass state (C2)");
        {
            Random rng(11);
            ReverbSettings s = neutral(ReverbAlgorithm::Hall, 2.0);
            s.lowCutHz = 80.0f;
            AlgorithmicReverb rv;
            rv.prepare(kRate, 16, 1);
            const size_t total = static_cast<size_t>(6.0 * kRate), noiseEnd = static_cast<size_t>(1.0 * kRate);
            std::vector<float> l(total, 0.0f), r(total, 0.0f);
            for (size_t i = 0; i < noiseEnd; ++i)
                l[i] = r[i] = static_cast<float>(0.3 * rng.gaussian());
            const size_t switchAt = static_cast<size_t>(0.5 * kRate);
            for (size_t pos = 0; pos < total; pos += 128)
            {
                if (pos >= switchAt)
                    s.lowCutHz = 20.0f;
                rv.process(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(128, total - pos)), s);
            }
            // Long after the tail has decayed the output must be silent, not a constant.
            double mean = 0.0;
            const size_t from = static_cast<size_t>(5.5 * kRate);
            for (size_t i = from; i < total; ++i)
                mean += l[i];
            mean /= static_cast<double>(total - from);
            logMessage(String("mean of the wet output 5.5 s after switching the low cut off: ") + String(mean, 8));
            expect(std::abs(mean) < 1.0e-5, "no DC offset after the low cut is bypassed: " + std::to_string(mean));
        }

        beginTest("parameters take effect: early level, density, damping");
        {
            ReverbSettings s = neutral(ReverbAlgorithm::Hall, 2.0);
            s.early = 0.0f;
            const Stereo noEarly = impulseResponse(s, 2.0);
            s.early = 1.0f;
            const Stereo early = impulseResponse(s, 2.0);
            expect(rmsDb(early.l, 0, 4800) > rmsDb(noEarly.l, 0, 4800) + 3.0, "early level raises the first 100 ms");
            s.early = 0.5f;
            s.density = 0.0f;
            const Stereo thin = impulseResponse(s, 2.0);
            s.density = 1.0f;
            const Stereo dense = impulseResponse(s, 2.0);
            double diff = 0.0;
            for (size_t i = 0; i < 9600; ++i)
                diff += std::abs(thin.l[i] - dense.l[i]);
            expect(diff > 1.0e-3, "density changes the build-up");
            ReverbSettings damp = neutral(ReverbAlgorithm::Hall, 2.0);
            damp.highMult = 0.3f;
            const Stereo dark = impulseResponse(damp, 6.0);
            const auto bands = energyDecay(dark.l, kRate, {500, 8000});
            expect(bands[1].t30S < 0.6 * bands[0].t30S, "high decay multiplier shortens the highs");
        }
    }

    // ---- bounded clearing
    void clearing()
    {
        beginTest("state is cleared in bounded chunks: 65536 samples per deactivate(), nothing replays on switch-on");
        {
            SpanClearer clearer;
            std::vector<float> a(100000, 1.0f), b(30000, 1.0f);
            clearer.setSpans({{a.data(), a.size()}, {b.data(), b.size()}});
            int calls = 0, largest = 0;
            bool done = false;
            while (!done)
            {
                int budget = 65536;
                done = clearer.step(budget);
                largest = std::max(largest, 65536 - budget);
                ++calls;
            }
            double sum = 0.0;
            for (const float v : a)
                sum += static_cast<double>(v);
            for (const float v : b)
                sum += static_cast<double>(v);
            expectEquals(static_cast<int>(sum), 0, "all cleared");
            expect(largest <= 65536 && calls == 2, "two calls of at most 65536 samples for 130000");

            AlgorithmicReverb rv;
            rv.prepare(kRate, 32, 1);
            const double calls32 = std::ceil(static_cast<double>(rv.stateSamples()) / 65536.0);
            logMessage(String("32-line reverb: ") + String(static_cast<double>(rv.stateSamples()), 0) +
                       " samples of state, cleared in " + String(calls32, 0) + " deactivate() calls of at most 65536");

            ReverbSettings s = neutral(ReverbAlgorithm::Hall, 4.0);
            std::vector<float> l(4800, 0.0f), r(4800, 0.0f);
            Random rng(1);
            for (int block = 0; block < 20; ++block)
            {
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = r[i] = static_cast<float>(0.4 * rng.gaussian());
                rv.process(l.data(), r.data(), static_cast<int>(l.size()), s);
            }
            for (int call = 0; call < static_cast<int>(calls32) + 1; ++call)
                rv.deactivate();
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            rv.process(l.data(), r.data(), static_cast<int>(l.size()), s);
            double peak = 0.0;
            for (const float v : l)
                peak = std::max(peak, static_cast<double>(std::abs(v)));
            logMessage(String("after deactivate() and switching on again, silent input gives peak ") +
                       String(amplitudeToDb(peak), 1) + " dBFS");
            expect(peak < 1.0e-6, "no replay");

            // Switched on again before the clear has finished: the rest is cleared first.
            for (int block = 0; block < 5; ++block)
            {
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = r[i] = static_cast<float>(0.4 * rng.gaussian());
                rv.process(l.data(), r.data(), static_cast<int>(l.size()), s);
            }
            rv.deactivate();
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            rv.process(l.data(), r.data(), static_cast<int>(l.size()), s);
            peak = 0.0;
            for (const float v : l)
                peak = std::max(peak, static_cast<double>(std::abs(v)));
            expect(peak < 1.0e-6, "no replay after a short deactivate");
        }

        beginTest("switching on before the clear has finished does not clear the rest in one callback (C6)");
        {
            AlgorithmicReverb rv;
            rv.prepare(192000.0, 32, 1);
            const double calls32 = std::ceil(static_cast<double>(rv.stateSamples()) / 65536.0);
            ReverbSettings s = neutral(ReverbAlgorithm::Hall, 4.0);
            std::vector<float> l(4800, 0.0f), r(4800, 0.0f);
            Random rng(2);
            for (int block = 0; block < 8; ++block)
            {
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = r[i] = static_cast<float>(0.4 * rng.gaussian());
                rv.process(l.data(), r.data(), static_cast<int>(l.size()), s);
            }
            rv.deactivate(); // one bounded step of a clear that needs several
            expect(calls32 > 2.0, "the clear takes several deactivate() calls: " + String(calls32, 0));
            // The reverb is switched on again. The first callbacks must not memset all state: they pass the dry input and
            // step the clear, then the reverb runs on cleared memory.
            int dryCallbacks = 0;
            bool live = false;
            for (int call = 0; call < static_cast<int>(calls32) + 2 && !live; ++call)
            {
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = r[i] = static_cast<float>(0.4 * rng.gaussian());
                const std::vector<float> in = l;
                rv.process(l.data(), r.data(), static_cast<int>(l.size()), s);
                if (l == in)
                    ++dryCallbacks;
                else
                    live = true;
            }
            logMessage(String("callbacks that passed the dry input while the clear finished: ") +
                       String(static_cast<double>(dryCallbacks), 0));
            expect(dryCallbacks >= 1, "the first callback after switching on only steps the clear");
            expect(dryCallbacks <= static_cast<int>(calls32),
                   "the clear finishes within the bounded steps: " + String(static_cast<double>(dryCallbacks), 0));
            expect(live, "the reverb runs once the clear is done");
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            for (int block = 0; block < 3; ++block)
                rv.process(l.data(), r.data(), static_cast<int>(l.size()), s);
        }
    }

    // ---- stress
    void stress()
    {
        beginTest("determinism: two fresh instances and a reset instance render the same bits");
        {
            Random rng(6);
            std::vector<float> in(static_cast<size_t>(1.0 * kRate));
            for (auto& v : in)
                v = static_cast<float>(0.3 * rng.gaussian());
            for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
            {
                ReverbSettings s = neutral(static_cast<ReverbAlgorithm>(a), 1.5);
                s.mix = 0.5f;
                const Stereo one = runReverb(s, in, in), two = runReverb(s, in, in);
                AlgorithmicReverb rv;
                rv.prepare(kRate, 16, 1);
                std::vector<float> l = in, r = in;
                for (size_t pos = 0; pos < l.size(); pos += 128)
                    rv.process(l.data() + pos, r.data() + pos, 128, s);
                rv.reset();
                l = in;
                r = in;
                for (size_t pos = 0; pos < l.size(); pos += 128)
                    rv.process(l.data() + pos, r.data() + pos, 128, s);
                long differ = 0;
                for (size_t i = 0; i < in.size(); ++i)
                    if (one.l[i] != two.l[i] || one.r[i] != two.r[i] || one.l[i] != static_cast<double>(l[i]) ||
                        one.r[i] != static_cast<double>(r[i]))
                        ++differ;
                expectEquals(static_cast<int>(differ), 0, String(kNames[a]) + " samples that differ");
            }
        }

        beginTest("12 s of random parameter extremes at five block sizes stays finite and bounded");
        {
            for (const int block : {1, 7, 32, 64, 500})
            {
                AlgorithmicReverb rv;
                rv.prepare(kRate, block == 500 ? 32 : 16, 3);
                Random rng(static_cast<std::uint64_t>(block));
                std::vector<float> l(static_cast<size_t>(block)), r(static_cast<size_t>(block));
                bool finite = true;
                double peak = 0.0;
                ReverbSettings s;
                const size_t blocks = static_cast<size_t>(12.0 * kRate) / static_cast<size_t>(block);
                for (size_t b = 0; b < blocks; ++b)
                {
                    if (b % std::max<size_t>(1, 4800 / static_cast<size_t>(block)) == 0)
                    {
                        const auto u = [&] { return static_cast<float>(0.5 + 0.5 * std::sin(1000.0 * rng.gaussian())); };
                        s.algorithm = static_cast<ReverbAlgorithm>(static_cast<int>(u() * 4.99f));
                        s.mix = u();
                        s.decaySeconds = 0.1f + u() * u() * 59.9f;
                        s.size = u();
                        s.predelayMs = u() * 500.0f;
                        s.density = u();
                        s.early = u();
                        s.lowMult = 0.1f + u() * 3.9f;
                        s.highMult = 0.05f + u() * 1.95f;
                        s.lowHz = 50.0f + u() * 950.0f;
                        s.highHz = 1000.0f + u() * 15000.0f;
                        s.modDepth = u();
                        s.modRate = 0.05f + u() * 4.95f;
                        s.width = u();
                        s.lowCutHz = 20.0f + u() * 980.0f;
                        s.highCutHz = 1000.0f + u() * 19000.0f;
                        s.freeze = u() > 0.8f;
                        s.duck = u();
                        s.duckReleaseMs = 20.0f + u() * 1980.0f;
                    }
                    for (size_t i = 0; i < l.size(); ++i)
                    {
                        const float x = static_cast<float>((b % 20 < 10 ? 1.5 : 0.0) * rng.gaussian());
                        l[i] = x;
                        r[i] = -x;
                    }
                    rv.process(l.data(), r.data(), block, s);
                    for (size_t i = 0; i < l.size(); ++i)
                    {
                        finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
                        peak = std::max({peak, static_cast<double>(std::abs(l[i])), static_cast<double>(std::abs(r[i]))});
                    }
                }
                logMessage(String("block ") + String(block) + ": peak " + String(peak, 2));
                expect(finite, "finite");
                expect(peak < 30.0, "bounded");
            }
        }
    }

    // ---- the module
    void module()
    {
        beginTest("fx.reverb_algo is registered, reports its tail, and the effect count follows");
        {
            const auto& registry = ModuleRegistry::instance();
            const ModuleDescriptor* d = registry.find("fx.reverb_algo");
            expect(d != nullptr, "registered");
            if (d == nullptr)
                return;
            expectEquals(static_cast<int>(registry.ofKind(ModuleKind::Effect).size()), kNumModuleFx,
                         "effects in the registry");
            RackHarness h;
            auto* lat = h.rack.findCapability<LatencyReporter>("fx.reverb_algo");
            expect(lat != nullptr, "reports a tail");
            if (lat == nullptr)
                return;
            h.set("fx_ra_on", 1.0f);
            h.set("fx_ra_decay", 3.0f);
            h.set("fx_ra_predelay", 20.0f);
            h.set("fx_ra_himult", 1.0f);
            std::vector<float> l(256, 0.1f), r(256, 0.1f);
            h.run(l.data(), r.data(), 256);
            logMessage(String("tail reported: ") + String(lat->tailSeconds(), 2) + " s for decay 3 s and pre-delay 20 ms");
            expectWithinAbsoluteError(lat->tailSeconds(), 3.02, 0.01, "tail length");
            expect(!lat->tailInfinite(), "finite without freeze");
            h.set("fx_ra_freeze", 1.0f);
            h.run(l.data(), r.data(), 256);
            expect(lat->tailInfinite(), "infinite with freeze");
            expectEquals(lat->latencySamples(), 0, "no latency");
        }

        beginTest("through the rack: switching on after being off replays nothing");
        {
            RackHarness h;
            h.set("fx_ra_on", 1.0f);
            h.set("fx_ra_mix", 1.0f);
            h.set("fx_ra_decay", 5.0f);
            Random rng(8);
            std::vector<float> l(480), r(480);
            for (int b = 0; b < 100; ++b)
            {
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = r[i] = static_cast<float>(0.4 * rng.gaussian());
                h.run(l.data(), r.data(), 480);
            }
            h.set("fx_ra_on", 0.0f);
            for (int b = 0; b < 200; ++b)
            {
                std::fill(l.begin(), l.end(), 0.0f);
                std::fill(r.begin(), r.end(), 0.0f);
                h.run(l.data(), r.data(), 480);
            }
            h.set("fx_ra_on", 1.0f);
            double peak = 0.0;
            for (int b = 0; b < 40; ++b)
            {
                std::fill(l.begin(), l.end(), 0.0f);
                std::fill(r.begin(), r.end(), 0.0f);
                h.run(l.data(), r.data(), 480);
                for (const float v : l)
                    peak = std::max(peak, static_cast<double>(std::abs(v)));
            }
            logMessage(String("switch off for 2 s and on again with silent input: peak ") + String(amplitudeToDb(peak), 1) +
                       " dBFS");
            expect(peak < 1.0e-6, "no replay");
        }

        beginTest("no allocation on the audio thread: every parameter changed every block, algorithm switches, freeze, 16 "
                  "and 32 lines");
        {
            for (const Quality q : {Quality::Standard, Quality::High})
            {
                RackHarness h(kRate, q);
                h.set("fx_ra_on", 1.0f);
                h.set("fx_ra_mix", 0.6f);
                Random rng(3);
                std::vector<float> l(64), r(64);
                for (int warm = 0; warm < 8; ++warm)
                    h.run(l.data(), r.data(), 64);
                crucible::testing::gAllocations = 0;
                crucible::testing::gTrackAllocations = true;
                for (int block = 0; block < 1500; ++block)
                {
                    for (size_t i = 0; i < 64; ++i)
                    {
                        l[i] = static_cast<float>(0.3 * rng.gaussian());
                        r[i] = static_cast<float>(0.3 * rng.gaussian());
                    }
                    h.set("fx_ra_algo", static_cast<float>((block / 60) % 5));
                    h.set("fx_ra_decay", 0.2f + static_cast<float>(block % 50) * 0.5f);
                    h.set("fx_ra_size", static_cast<float>(block % 17) / 16.0f);
                    h.set("fx_ra_predelay", static_cast<float>(block % 40) * 10.0f);
                    h.set("fx_ra_density", static_cast<float>(block % 11) / 10.0f);
                    h.set("fx_ra_early", static_cast<float>(block % 7) / 6.0f);
                    h.set("fx_ra_lomult", 0.3f + static_cast<float>(block % 13) * 0.2f);
                    h.set("fx_ra_himult", 0.1f + static_cast<float>(block % 9) * 0.2f);
                    h.set("fx_ra_lofreq", 80.0f + static_cast<float>(block % 30) * 20.0f);
                    h.set("fx_ra_hifreq", 1500.0f + static_cast<float>(block % 30) * 400.0f);
                    h.set("fx_ra_moddepth", static_cast<float>(block % 5) / 4.0f);
                    h.set("fx_ra_modrate", 0.1f + static_cast<float>(block % 8));
                    h.set("fx_ra_width", static_cast<float>(block % 6) / 5.0f);
                    h.set("fx_ra_lowcut", 20.0f + static_cast<float>(block % 40) * 20.0f);
                    h.set("fx_ra_highcut", 2000.0f + static_cast<float>(block % 36) * 500.0f);
                    h.set("fx_ra_freeze", (block / 100) % 3 == 2 ? 1.0f : 0.0f);
                    h.set("fx_ra_duck", static_cast<float>(block % 4) / 3.0f);
                    h.set("fx_ra_on", (block / 300) % 2 == 1 && block % 300 > 280 ? 0.0f : 1.0f);
                    h.run(l.data(), r.data(), 64);
                }
                crucible::testing::gTrackAllocations = false;
                expectEquals(static_cast<int>(crucible::testing::gAllocations), 0,
                             String("allocations ") + (q == Quality::High ? "High" : "Standard"));
            }
        }
    }

    // ---- CPU
    void realTime()
    {
#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest("CPU: percent of one core per stereo instance at 48 kHz (plan: 1.0 % with 16 lines, 2.0 % with 32)");
        {
            String line16, line32;
            for (int a = 0; a < static_cast<int>(ReverbAlgorithm::Count); ++a)
            {
                const std::vector<std::pair<std::string, float>> setup = {
                    {"fx_ra_on", 1.0f}, {"fx_ra_algo", static_cast<float>(a)}, {"fx_ra_decay", 3.0f}, {"fx_ra_mix", 0.5f}};
                const double c16 = cpuPercent(setup, Quality::Standard);
                line16 = line16 + String(kNames[a]) + " " + String(c16, 3) + "  ";
                expectCpu(c16 <= 1.0 * 1.2, String(kNames[a]) + " 16-line CPU");
                if (a != static_cast<int>(ReverbAlgorithm::Plate))
                {
                    const double c32 = cpuPercent(setup, Quality::High);
                    line32 = line32 + String(kNames[a]) + " " + String(c32, 3) + "  ";
                    expectCpu(c32 <= 2.0 * 1.2, String(kNames[a]) + " 32-line CPU");
                }
            }
            logMessage(String("CPU, 16 lines (%): ") + line16);
            logMessage(String("CPU, 32 lines (%): ") + line32);
        }
#endif
    }
};

static ReverbAlgoTests reverbAlgoTests;
