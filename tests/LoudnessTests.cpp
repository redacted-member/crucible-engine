// SPDX-License-Identifier: MIT
// Loudness and true-peak metering (FX4, docs/plans/ENGINE-FX-PLAN.md section 3.8): the K-weighting of ITU-R BS.1770-4, the
// test signals of EBU Tech 3341 (momentary, short-term, integrated loudness) and Tech 3342 (loudness range) built from their
// published definitions, agreement with the vendored libebur128 (third_party/libebur128, tests only) on program material at
// four sample rates, true peak against a 16x windowed-sinc reference on inter-sample-peak torture signals, the programme
// meters, and the real-time contract. The printed lines are what docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/fx/Loudness.h>

extern "C"
{
#include "ebur128.h"
}

#include <chrono>
#include <cmath>
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
using namespace crucible::fx;
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;

namespace
{
using Channels = std::vector<Signal>;

struct Reading
{
    double integrated = -120, momentary = -120, shortTerm = -120, momentaryMax = -120, shortTermMax = -120, range = 0;
    double samplePeakDb = -120, truePeakDb = -120;
};

// Whole signal through LoudnessMeter in blocks of `block` samples.
Reading measureMine(const Channels& in, double rate, int block = 480, const std::vector<double>& weights = {})
{
    LoudnessMeter m;
    m.prepare(rate, static_cast<int>(in.size()));
    for (size_t c = 0; c < weights.size(); ++c)
        m.setChannelWeight(static_cast<int>(c), weights[c]);
    const size_t n = in[0].size();
    std::vector<std::vector<float>> buf(in.size(), std::vector<float>(static_cast<size_t>(block)));
    std::vector<const float*> ptr(in.size());
    for (size_t pos = 0; pos < n; pos += static_cast<size_t>(block))
    {
        const int len = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), n - pos));
        for (size_t c = 0; c < in.size(); ++c)
        {
            for (int i = 0; i < len; ++i)
                buf[c][static_cast<size_t>(i)] = static_cast<float>(in[c][pos + static_cast<size_t>(i)]);
            ptr[c] = buf[c].data();
        }
        m.process(ptr.data(), len);
    }
    Reading r;
    r.integrated = m.integratedLufs();
    r.momentary = m.momentaryLufs();
    r.shortTerm = m.shortTermLufs();
    r.momentaryMax = m.maxMomentaryLufs();
    r.shortTermMax = m.maxShortTermLufs();
    r.range = m.loudnessRange();
    r.samplePeakDb = m.samplePeakDb();
    r.truePeakDb = m.truePeakDb();
    return r;
}

// The same through libebur128 (exact block lists, no histogram). Momentary and short-term maxima are sampled every 100 ms.
Reading measureOracle(const Channels& in, double rate, const std::vector<int>& channelMap = {})
{
    const unsigned channels = static_cast<unsigned>(in.size());
    ebur128_state* st = ebur128_init(channels, static_cast<unsigned long>(rate),
                                     EBUR128_MODE_I | EBUR128_MODE_S | EBUR128_MODE_LRA | EBUR128_MODE_TRUE_PEAK);
    for (size_t c = 0; c < channelMap.size(); ++c)
        ebur128_set_channel(st, static_cast<unsigned>(c), channelMap[c]);
    const size_t n = in[0].size();
    const size_t step = static_cast<size_t>(rate / 10.0);
    std::vector<float> inter(step * channels);
    Reading r;
    for (size_t pos = 0; pos < n; pos += step)
    {
        const size_t len = std::min(step, n - pos);
        for (size_t i = 0; i < len; ++i)
            for (size_t c = 0; c < channels; ++c)
                inter[i * channels + c] = static_cast<float>(in[c][pos + i]);
        ebur128_add_frames_float(st, inter.data(), len);
        double m = -HUGE_VAL, s = -HUGE_VAL;
        if (pos + len >= static_cast<size_t>(0.4 * rate))
            ebur128_loudness_momentary(st, &m);
        if (pos + len >= static_cast<size_t>(3.0 * rate))
            ebur128_loudness_shortterm(st, &s);
        r.momentaryMax = std::max(r.momentaryMax, std::isfinite(m) ? m : -120.0);
        r.shortTermMax = std::max(r.shortTermMax, std::isfinite(s) ? s : -120.0);
    }
    double v = 0.0;
    ebur128_loudness_global(st, &v);
    r.integrated = std::isfinite(v) ? v : -120.0;
    ebur128_loudness_range(st, &v);
    r.range = v;
    double peak = 0.0, tp = 0.0;
    for (unsigned c = 0; c < channels; ++c)
    {
        ebur128_sample_peak(st, c, &peak);
        r.samplePeakDb = std::max(r.samplePeakDb, 20.0 * std::log10(std::max(peak, 1.0e-6)));
        ebur128_true_peak(st, c, &tp);
        r.truePeakDb = std::max(r.truePeakDb, 20.0 * std::log10(std::max(tp, 1.0e-6)));
    }
    ebur128_destroy(&st);
    return r;
}

// |H| of the K-weighting at a frequency.
double kGain(double hz, double rate)
{
    KWeightingCoefficients shelf, hp;
    designKWeighting(rate, shelf, hp);
    const auto mag = [&](const KWeightingCoefficients& c)
    {
        const std::complex<double> z = std::polar(1.0, -2.0 * kPi * hz / rate), z2 = z * z;
        return std::abs((c.b0 + c.b1 * z + c.b2 * z2) / (1.0 + c.a1 * z + c.a2 * z2));
    };
    return mag(shelf) * mag(hp);
}

// 1 kHz stereo (identical channels) whose loudness is `lufs`: L = -0.691 + 10 log10(2 * (|H| A)^2 / 2).
Signal tone(double lufs, double seconds, double rate, double hz = 1000.0, double phase = 0.0)
{
    const double a = std::pow(10.0, (lufs + 0.691) / 20.0) / kGain(hz, rate);
    return sineWave(static_cast<size_t>(std::llround(seconds * rate)), hz, rate, a, phase);
}

Signal join(std::initializer_list<Signal> parts)
{
    Signal out;
    for (const auto& p : parts)
        out.insert(out.end(), p.begin(), p.end());
    return out;
}

// Phase-continuous steps of 1 kHz at the given loudness levels.
Channels steps(const std::vector<std::pair<double, double>>& levelSeconds, double rate)
{
    Signal x;
    size_t n = 0;
    for (const auto& [level, seconds] : levelSeconds)
    {
        const double a = std::pow(10.0, (level + 0.691) / 20.0) / kGain(1000.0, rate);
        const size_t count = static_cast<size_t>(std::llround(seconds * rate));
        for (size_t i = 0; i < count; ++i, ++n)
            x.push_back(a * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(n) / rate));
    }
    return {x, x};
}

// Program-like material: coloured noise with level steps, a bass line, bursts and a quiet passage, uncorrelated left and
// right.
Channels programme(double rate, double seconds, unsigned seed)
{
    const size_t n = static_cast<size_t>(seconds * rate);
    Channels out(2, Signal(n));
    Random rng(seed);
    for (size_t c = 0; c < 2; ++c)
    {
        double lp1 = 0.0, lp2 = 0.0;
        const double a1 = 1.0 - std::exp(-2.0 * kPi * (400.0 + 200.0 * static_cast<double>(c)) / rate),
                     a2 = 1.0 - std::exp(-2.0 * kPi * 3000.0 / rate);
        for (size_t i = 0; i < n; ++i)
        {
            const double t = static_cast<double>(i) / rate;
            const double w = rng.gaussian();
            lp1 += a1 * (w - lp1);
            lp2 += a2 * (lp1 - lp2);
            const double envelope = (t < 0.18 * seconds)   ? 0.35
                                    : (t < 0.4 * seconds)  ? 0.03
                                    : (t < 0.43 * seconds) ? 0.0005
                                    : (t < 0.7 * seconds)  ? 0.12 * (0.6 + 0.4 * std::sin(2.0 * kPi * 0.7 * t))
                                                           : 0.25 * (std::fmod(t, 2.0) < 0.3 ? 1.0 : 0.15);
            out[c][i] = envelope * (6.0 * lp2 + 0.4 * std::sin(2.0 * kPi * 55.0 * t + static_cast<double>(c)));
        }
    }
    return out;
}

double truePeakReferenceDb(const Channels& in)
{
    double m = 0.0;
    for (const auto& c : in)
        m = std::max(m, truePeak(c.data(), c.size(), 16, 32));
    return amplitudeToDb(m);
}
} // namespace

class LoudnessTests : public UnitTest
{
public:
    LoudnessTests() : UnitTest("Loudness", "Effects") {}

    void runTest() override
    {
        kWeightingTable();
        ebuTech3341();
        ebuTech3342();
        oracle();
        truePeakTorture();
        programmeMeters();
        robustness();
        realTime();
    }

private:
    void kWeightingTable()
    {
        beginTest("K-weighting at 48 kHz equals the coefficients printed in ITU-R BS.1770-4");
        KWeightingCoefficients shelf, hp;
        designKWeighting(48000.0, shelf, hp);
        const double worst = std::max({std::abs(shelf.b0 - 1.53512485958697), std::abs(shelf.b1 + 2.69169618940638),
                                       std::abs(shelf.b2 - 1.19839281085285), std::abs(shelf.a1 + 1.69065929318241),
                                       std::abs(shelf.a2 - 0.73248077421585), std::abs(hp.a1 + 1.99004745483398),
                                       std::abs(hp.a2 - 0.99007225036621)});
        logMessage("K-weighting 48 kHz: worst coefficient difference " + String(worst, 12) + "; |H(1 kHz)| " +
                   String(20.0 * std::log10(kGain(1000.0, 48000.0)), 4) + " dB");
        expect(worst < 1.0e-7, "coefficients");
        // The same response (in dB at 1 kHz, 100 Hz, 10 kHz) at the other rates to within 0.05 dB below 15 kHz.
        double rateSpread = 0.0;
        for (const double hz : {50.0, 100.0, 1000.0, 4000.0, 10000.0})
            for (const double rate : {44100.0, 88200.0, 96000.0, 192000.0})
                rateSpread = std::max(rateSpread, std::abs(20.0 * std::log10(kGain(hz, rate) / kGain(hz, 48000.0))));
        logMessage("K-weighting response across 44.1 to 192 kHz: worst deviation from 48 kHz " + String(rateSpread, 3) +
                   " dB (50 Hz to 10 kHz)");
        expect(rateSpread < 0.12, "response is rate independent");
    }

    void ebuTech3341()
    {
        beginTest("EBU Tech 3341 cases 1-5: integrated, momentary and short-term loudness of the stereo 1 kHz test signals");
        const double rate = 48000.0;
        struct Case
        {
            const char* name;
            std::vector<std::pair<double, double>> program;
            double expected;
        };
        const std::vector<Case> cases = {
            {"1: -23 LUFS, 20 s", {{-23.0, 20.0}}, -23.0},
            {"2: -33 LUFS, 20 s", {{-33.0, 20.0}}, -33.0},
            {"3: -36 / -23 / -36 LUFS, 10 / 60 / 10 s (relative gate)",
             {{-36.0, 10.0}, {-23.0, 60.0}, {-36.0, 10.0}},
             -23.0},
            {"4: -72 / -36 / -23 / -36 / -72 LUFS, 10 / 10 / 60 / 10 / 10 s (absolute gate)",
             {{-72.0, 10.0}, {-36.0, 10.0}, {-23.0, 60.0}, {-36.0, 10.0}, {-72.0, 10.0}},
             -23.0},
            {"5: -26 / -20 / -26 LUFS, 20 s each", {{-26.0, 20.0}, {-20.0, 20.0}, {-26.0, 20.0}}, -23.0}};
        double worst = 0.0;
        for (const auto& c : cases)
        {
            const Reading r = measureMine(steps(c.program, rate), rate);
            const double err = std::abs(r.integrated - c.expected);
            worst = std::max(worst, err);
            logMessage(String("case ") + c.name + ": integrated " + String(r.integrated, 3) + " LUFS (expected " +
                       String(c.expected, 1) + ")");
            expectWithinAbsoluteError(r.integrated, c.expected, 0.1, c.name);
            if (c.program.size() == 1)
            {
                expectWithinAbsoluteError(r.momentary, c.expected, 0.1, "momentary");
                expectWithinAbsoluteError(r.shortTerm, c.expected, 0.1, "short-term");
            }
        }
        logMessage("worst integrated error over the five cases " + String(worst, 4) + " LU (tolerance 0.1)");

        beginTest("EBU Tech 3341 style bursts: momentary maximum of a 400 ms burst, short-term maximum of a 3 s burst");
        {
            const Signal silence(static_cast<size_t>(1.0 * rate), 0.0);
            const Signal burstM = tone(-23.0, 0.4, rate), burstS = tone(-23.0, 3.0, rate);
            const Signal m = join({silence, burstM, silence}), s = join({silence, burstS, silence});
            const Reading rm = measureMine({m, m}, rate), rs = measureMine({s, s}, rate);
            logMessage("400 ms burst at -23 LUFS: momentary maximum " + String(rm.momentaryMax, 3) +
                       " LUFS; 3 s burst: short-term maximum " + String(rs.shortTermMax, 3) + " LUFS, momentary maximum " +
                       String(rs.momentaryMax, 3));
            expectWithinAbsoluteError(rm.momentaryMax, -23.0, 0.1, "momentary maximum");
            expectWithinAbsoluteError(rs.shortTermMax, -23.0, 0.1, "short-term maximum");
            expectWithinAbsoluteError(rs.momentaryMax, -23.0, 0.1, "momentary maximum of the long burst");
        }

        beginTest("five channels: surround channels weighted by 1.41, LFE ignored, against the analytic sum");
        {
            // L, R, C at -28 dBFS peak, Ls, Rs at -30 dBFS peak, 1 kHz.
            const double h2 = kGain(1000.0, rate) * kGain(1000.0, rate);
            const auto power = [&](double peakDb) { return 0.5 * std::pow(10.0, peakDb / 10.0) * h2; };
            const double z = 3.0 * power(-28.0) + 2.0 * 1.41 * power(-30.0);
            const double expected = -0.691 + 10.0 * std::log10(z);
            Channels ch;
            for (const double level : {-28.0, -28.0, -28.0, -30.0, -30.0})
                ch.push_back(sineWave(static_cast<size_t>(20.0 * rate), 1000.0, rate, std::pow(10.0, level / 20.0)));
            const Reading r = measureMine(ch, rate, 480, {1.0, 1.0, 1.0, 1.41, 1.41});
            logMessage("5.0 tones: integrated " + String(r.integrated, 3) + " LUFS, analytic " + String(expected, 3));
            expectWithinAbsoluteError(r.integrated, expected, 0.02, "five channel sum");
            ch.push_back(sineWave(static_cast<size_t>(20.0 * rate), 1000.0, rate, 0.9));
            const Reading withLfe = measureMine(ch, rate, 480, {1.0, 1.0, 1.0, 1.41, 1.41, 0.0});
            expectWithinAbsoluteError(withLfe.integrated, r.integrated, 1.0e-4, "LFE weight 0");
        }
    }

    void ebuTech3342()
    {
        beginTest("EBU Tech 3342 cases 1-4: loudness range of the stereo 1 kHz test signals");
        const double rate = 48000.0;
        struct Case
        {
            const char* name;
            std::vector<std::pair<double, double>> program;
            double expected;
        };
        const std::vector<Case> cases = {{"1: -20 / -30 LUFS, 20 s each", {{-20.0, 20.0}, {-30.0, 20.0}}, 10.0},
                                         {"2: -20 / -15 LUFS, 20 s each", {{-20.0, 20.0}, {-15.0, 20.0}}, 5.0},
                                         {"3: -40 / -20 LUFS, 20 s each", {{-40.0, 20.0}, {-20.0, 20.0}}, 20.0},
                                         {"4: -50 / -35 / -20 / -35 / -50 LUFS, 20 s each",
                                          {{-50.0, 20.0}, {-35.0, 20.0}, {-20.0, 20.0}, {-35.0, 20.0}, {-50.0, 20.0}},
                                          15.0}};
        double worst = 0.0;
        for (const auto& c : cases)
        {
            const Reading r = measureMine(steps(c.program, rate), rate);
            const Reading o = measureOracle(steps(c.program, rate), rate);
            worst = std::max(worst, std::abs(r.range - c.expected));
            logMessage(String("case ") + c.name + ": range " + String(r.range, 3) + " LU (expected " +
                       String(c.expected, 1) + "), libebur128 " + String(o.range, 3));
            expectWithinAbsoluteError(r.range, c.expected, 1.0, c.name);
            expectWithinAbsoluteError(r.range, o.range, 0.1, "against libebur128");
        }
        logMessage("worst range error over the four cases " + String(worst, 3) + " LU (tolerance 1)");
    }

    void oracle()
    {
        beginTest("agreement with libebur128 on 100 s of program material at 44.1, 48, 96 and 192 kHz");
        double worstI = 0, worstM = 0, worstS = 0, worstLra = 0, worstPeak = 0, worstTp = -100;
        for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
        {
            const Channels in = programme(rate, 100.0, 7 + static_cast<unsigned>(rate / 1000.0));
            const Reading mine = measureMine(in, rate, 256), ref = measureOracle(in, rate);
            const double di = mine.integrated - ref.integrated, dm = mine.momentaryMax - ref.momentaryMax,
                         ds = mine.shortTermMax - ref.shortTermMax;
            const double dl = mine.range - ref.range, dp = mine.samplePeakDb - ref.samplePeakDb,
                         dt = mine.truePeakDb - ref.truePeakDb;
            logMessage("rate " + String(rate, 0) + ": integrated " + String(mine.integrated, 3) + " vs " +
                       String(ref.integrated, 3) + " LUFS, momentary max " + String(mine.momentaryMax, 3) + " vs " +
                       String(ref.momentaryMax, 3) + ", short-term max " + String(mine.shortTermMax, 3) + " vs " +
                       String(ref.shortTermMax, 3) + ", range " + String(mine.range, 3) + " vs " + String(ref.range, 3) +
                       " LU, sample peak " + String(mine.samplePeakDb, 3) + " vs " + String(ref.samplePeakDb, 3) +
                       ", true peak " + String(mine.truePeakDb, 3) + " vs " + String(ref.truePeakDb, 3) + " dB");
            if (rate == 48000.0)
            {
                // The offline reference of FxMeasure.h (every block kept, no histogram) agrees as well.
                const LoudnessResult offline = measureLoudness(in, rate);
                logMessage("offline reference (FxMeasure.h): integrated " + String(offline.integratedLufs, 3) +
                           " LUFS, range " + String(offline.rangeLu, 3) + " LU");
                expectWithinAbsoluteError(mine.integrated, offline.integratedLufs, 0.02,
                                          "integrated against the offline reference");
                expectWithinAbsoluteError(mine.range, offline.rangeLu, 0.15, "range against the offline reference");
            }
            worstI = std::max(worstI, std::abs(di));
            worstM = std::max(worstM, std::abs(dm));
            worstS = std::max(worstS, std::abs(ds));
            worstLra = std::max(worstLra, std::abs(dl));
            worstPeak = std::max(worstPeak, std::abs(dp));
            worstTp = std::max(worstTp, std::abs(dt));
            expectWithinAbsoluteError(di, 0.0, 0.05, "integrated at " + String(rate, 0));
            expectWithinAbsoluteError(dm, 0.0, 0.1, "momentary maximum at " + String(rate, 0));
            expectWithinAbsoluteError(ds, 0.0, 0.05, "short-term maximum at " + String(rate, 0));
            expectWithinAbsoluteError(dl, 0.0, 0.15, "range at " + String(rate, 0));
            expectWithinAbsoluteError(dp, 0.0, 1.0e-3, "sample peak at " + String(rate, 0));
            expectWithinAbsoluteError(dt, 0.0, 0.2, "true peak at " + String(rate, 0));
        }
        logMessage("worst difference to libebur128: integrated " + String(worstI, 4) + " LU, momentary max " +
                   String(worstM, 4) + ", short-term max " + String(worstS, 4) + ", range " + String(worstLra, 4) +
                   " LU, sample peak " + String(worstPeak, 5) + " dB, true peak " + String(worstTp, 4) + " dB");
    }

    void truePeakTorture()
    {
        beginTest("true peak against a 16x windowed-sinc reference on inter-sample-peak torture signals");
        const double rate = 48000.0;
        struct Item
        {
            std::string name;
            Signal x;
            double tolerance = 0.2;
        };
        std::vector<Item> items;
        const size_t n = 9600;
        for (const double phaseDeg : {0.0, 22.5, 45.0, 67.5})
            items.push_back({"fs/4 sine, phase " + std::to_string(static_cast<int>(phaseDeg)) + " deg, -6 dBFS",
                             sineWave(n, rate / 4.0, rate, 0.5, phaseDeg * kPi / 180.0)});
        {
            Signal x(n);
            for (size_t i = 0; i < n; ++i)
                x[i] = (i / 2) % 2 == 0 ? 1.0 : -1.0; // ++-- : a sine of amplitude 1.414 sampled at 45 degrees
            items.push_back({"++-- square (0 dBFS samples, +3 dBTP true)", x});
        }
        for (const double hz : {997.0, 7919.0, 15013.0, 19001.0, 21001.0})
            items.push_back(
                {"sine " + std::to_string(static_cast<int>(hz)) + " Hz, -3 dBFS", sineWave(n, hz, rate, 0.7071)});
        {
            // Hard-clipped 3 kHz sine, then a gentle low-pass: the overshoot of a clipped, band-limited master.
            Signal x(n);
            double lp = 0.0;
            for (size_t i = 0; i < n; ++i)
            {
                const double v = std::clamp(2.0 * std::sin(2.0 * kPi * 3000.0 * static_cast<double>(i) / rate), -1.0, 1.0);
                lp += 0.6 * (v - lp);
                x[i] = lp;
            }
            items.push_back({"clipped 3 kHz sine, low-passed", x});
        }
        for (const bool wideband : {false, true})
        {
            Random rng(21);
            Signal white(n + 64);
            for (double& v : white)
                v = rng.gaussian();
            Signal x(n);
            if (wideband)
            {
                double lp = 0.0;
                for (size_t i = 0; i < n; ++i)
                {
                    lp += 0.7 * (white[i] - lp);
                    x[i] = lp;
                }
            }
            else
            {
                // Low-passed at 16 kHz by a 63-tap Hann-windowed sinc: noise with the bandwidth of programme material.
                Signal h(63);
                double sum = 0.0;
                for (int k = 0; k < 63; ++k)
                {
                    const double t = k - 31;
                    h[static_cast<size_t>(k)] =
                        (t == 0 ? 2.0 * 16000.0 / rate : std::sin(2.0 * kPi * 16000.0 / rate * t) / (kPi * t)) *
                        (0.5 + 0.5 * std::cos(kPi * t / 32.0));
                    sum += h[static_cast<size_t>(k)];
                }
                for (size_t i = 0; i < n; ++i)
                {
                    double acc = 0.0;
                    for (size_t k = 0; k < 63; ++k)
                        acc += h[k] * white[i + k];
                    x[i] = acc / sum;
                }
            }
            double m = 0.0;
            for (double v : x)
                m = std::max(m, std::abs(v));
            for (double& v : x)
                v *= 0.95 / m;
            items.push_back({wideband
                                 ? "noise with energy up to Nyquist, normalised to -0.45 dBFS (outside the 4x design band)"
                                 : "noise low-passed at 16 kHz, normalised to -0.45 dBFS",
                             x, wideband ? 0.5 : 0.2});
        }
        double worstLow = 0.0, worstHigh = 0.0;
        for (auto& item : items)
        {
            // A raised-cosine fade at both ends: a signal that starts abruptly really does overshoot there, which the
            // reference (it skips the first and last samples) does not look at.
            const size_t fade = 2400;
            for (size_t i = 0; i < fade; ++i)
            {
                const double g = 0.5 - 0.5 * std::cos(kPi * static_cast<double>(i) / static_cast<double>(fade));
                item.x[i] *= g;
                item.x[item.x.size() - 1 - i] *= g;
            }
            const double ref = truePeakReferenceDb({item.x});
            const Reading r = measureMine({item.x}, rate);
            const double d = r.truePeakDb - ref;
            worstLow = std::min(worstLow, d);
            worstHigh = std::max(worstHigh, d);
            {
                // The detector variants on the same signal, for the record.
                std::string variants;
                for (const auto& [factor, refine] :
                     {std::pair{4, false}, std::pair{4, true}, std::pair{8, false}, std::pair{8, true}})
                {
                    TruePeakDetector d;
                    d.configure(factor, refine);
                    float best = 0.0f;
                    for (const double v : item.x)
                        best = std::max(best, d.process(static_cast<float>(v)));
                    variants += " " + std::to_string(factor) + "x" + (refine ? "+r " : " ") +
                                std::to_string(amplitudeToDb(best) - ref).substr(0, 6);
                }
                logMessage("  detector minus reference (dB):" + variants);
            }
            logMessage(item.name + ": sample peak " + String(r.samplePeakDb, 3) + ", true peak " + String(r.truePeakDb, 3) +
                       " dBTP, 16x reference " + String(ref, 3) + " (difference " + String(d, 3) + " dB)");
            expectWithinAbsoluteError(d, 0.0, item.tolerance, item.name);
        }
        logMessage("true peak minus 16x reference: from " + String(worstLow, 3) + " to +" + String(worstHigh, 3) +
                   " dB (tolerance 0.2, 0.5 for the noise that reaches Nyquist)");
    }

    void programmeMeters()
    {
        const double rate = 48000.0;
        beginTest("VU meter: 99 % of a step after 300 ms, overshoot 1 to 1.5 %, a sine reads its level");
        {
            VuMeter vu;
            vu.prepare(rate);
            const size_t n = static_cast<size_t>(2.0 * rate);
            double reach = -1.0, over = 0.0;
            for (size_t i = 0; i < n; ++i)
            {
                // A constant input of 2/pi is what a unit sine averages to after rectification: the meter then rests at 1.
                vu.process(static_cast<float>(2.0 / kPi));
                const double level = std::pow(10.0, vu.levelDb() / 20.0);
                if (reach < 0.0 && level >= 0.99)
                    reach = static_cast<double>(i + 1) / rate;
                over = std::max(over, level - 1.0);
            }
            logMessage("VU: step reaches 99 % after " + String(reach * 1000.0, 1) + " ms, overshoot " +
                       String(over * 100.0, 2) + " %");
            expectWithinAbsoluteError(reach, 0.300, 0.01, "rise time");
            expect(over > 0.005 && over < 0.02, "overshoot");
            VuMeter sine;
            sine.prepare(rate);
            const Signal x = sineWave(static_cast<size_t>(3.0 * rate), 1000.0, rate, 0.5);
            for (const double v : x)
                sine.process(static_cast<float>(v));
            logMessage("VU: 1 kHz sine at -6.02 dBFS reads " + String(sine.levelDb(), 3) + " dB");
            expectWithinAbsoluteError(sine.levelDb(), -6.0206, 0.1, "steady sine");
        }

        beginTest("PPM: a 10 ms burst of 5 kHz reads -1 dB below a steady tone, a steady sine reads its peak, return 20 dB "
                  "in 1.5 s / 24 dB in 2.8 s");
        for (const PpmType type : {PpmType::TypeI, PpmType::TypeII})
        {
            PpmMeter ppm;
            ppm.prepare(rate, type);
            const Signal burst = sineWave(static_cast<size_t>(0.010 * rate), 5000.0, rate, 1.0, kPi / 2.0);
            double burstDb = -120.0;
            for (const double v : burst)
            {
                ppm.process(static_cast<float>(v));
                burstDb = std::max(burstDb, static_cast<double>(ppm.levelDb()));
            }
            for (size_t i = 0; i < static_cast<size_t>(0.002 * rate); ++i)
            {
                ppm.process(0.0f);
                burstDb = std::max(burstDb, static_cast<double>(ppm.levelDb()));
            }
            PpmMeter steady;
            steady.prepare(rate, type);
            for (const double v : sineWave(static_cast<size_t>(1.0 * rate), 5000.0, rate, 1.0))
                steady.process(static_cast<float>(v));
            const double steadyDb = steady.levelDb();
            // Fall: silence after the steady tone.
            const double start = steady.levelDb();
            const double seconds = type == PpmType::TypeI ? 1.5 : 2.8;
            for (size_t i = 0; i < static_cast<size_t>(seconds * rate); ++i)
                steady.process(0.0f);
            const double fall = start - steady.levelDb();
            logMessage(std::string(type == PpmType::TypeI ? "type I" : "type II") + ": burst " + String(burstDb, 2) +
                       " dB against steady " + String(steadyDb, 2) + " dB (expected about -1 dB), fall in " +
                       String(seconds, 1) + " s " + String(fall, 2) + " dB");
            expectWithinAbsoluteError(burstDb - steadyDb, -1.0, 0.4, "burst");
            expectWithinAbsoluteError(steadyDb, 0.0, 0.1, "steady sine");
            expectWithinAbsoluteError(fall, type == PpmType::TypeI ? 20.0 : 24.0, 0.3, "return time");
        }

        beginTest("correlation meter: mono +1, inverted -1, independent noise near 0, one silent channel 0");
        {
            CorrelationMeter cm;
            cm.prepare(rate);
            Random rng(5);
            const size_t n = static_cast<size_t>(3.0 * rate);
            for (size_t i = 0; i < n; ++i)
            {
                const float v = static_cast<float>(rng.gaussian() * 0.2);
                cm.process(v, v);
            }
            const float mono = cm.value();
            cm.reset();
            for (size_t i = 0; i < n; ++i)
            {
                const float v = static_cast<float>(rng.gaussian() * 0.2);
                cm.process(v, -v);
            }
            const float inverted = cm.value();
            cm.reset();
            for (size_t i = 0; i < n; ++i)
                cm.process(static_cast<float>(rng.gaussian() * 0.2), static_cast<float>(rng.gaussian() * 0.2));
            const float independent = cm.value();
            cm.reset();
            for (size_t i = 0; i < n; ++i)
                cm.process(static_cast<float>(rng.gaussian() * 0.2), 0.0f);
            logMessage("correlation: mono " + String(mono, 4) + ", inverted " + String(inverted, 4) + ", independent " +
                       String(independent, 4) + ", silent right " + String(cm.value(), 4));
            expectWithinAbsoluteError(mono, 1.0f, 1.0e-3f);
            expectWithinAbsoluteError(inverted, -1.0f, 1.0e-3f);
            expectWithinAbsoluteError(independent, 0.0f, 0.1f);
            expectWithinAbsoluteError(cm.value(), 0.0f, 1.0e-3f);
        }
    }

    void robustness()
    {
        beginTest("silence reads the floor, NaN and infinity are ignored, reset clears the statistics, any block size gives "
                  "one result");
        const double rate = 48000.0;
        LoudnessMeter m;
        m.prepare(rate, 2);
        std::vector<float> z(4800, 0.0f);
        for (int i = 0; i < 100; ++i)
            m.process(z.data(), z.data(), 4800);
        expect(m.momentaryLufs() <= -119.0f && m.shortTermLufs() <= -119.0f && m.integratedLufs() <= -119.0f, "silence");
        std::vector<float> bad(4800, 0.1f);
        bad[10] = std::nanf("");
        bad[20] = HUGE_VALF;
        for (int i = 0; i < 20; ++i)
            m.process(bad.data(), bad.data(), 4800);
        expect(std::isfinite(m.momentaryLufs()) && std::isfinite(m.integratedLufs()) && std::isfinite(m.truePeakDb()),
               "non-finite input");
        m.resetStatistics();
        expect(m.integratedLufs() <= -119.0f && m.maxMomentaryLufs() <= -119.0f && m.truePeakDb() <= -119.0f,
               "reset statistics");
        // Block-size independence, bit for bit.
        const Channels in = programme(rate, 20.0, 3);
        const Reading a = measureMine(in, rate, 1), b = measureMine(in, rate, 4800), c = measureMine(in, rate, 777);
        expect(a.integrated == b.integrated && b.integrated == c.integrated && a.range == c.range &&
                   a.momentaryMax == b.momentaryMax,
               "block sizes");
        // 22.05 kHz: fractional sub-block length.
        const Channels low = steps({{-23.0, 20.0}}, 22050.0);
        expectWithinAbsoluteError(measureMine(low, 22050.0).integrated, -23.0, 0.1, "22.05 kHz");
    }

    void realTime()
    {
        beginTest("real time: no allocation while processing; cost of the loudness meter with true peak, stereo, 48 kHz");
        LoudnessMeter m;
        m.prepare(48000.0, 2);
        const Channels in = programme(48000.0, 6.0, 9);
        std::vector<float> l(in[0].begin(), in[0].end()), r(in[1].begin(), in[1].end());
        crucible::testing::gAllocations = 0;
        crucible::testing::gTrackAllocations = true;
        for (size_t pos = 0; pos + 32 <= l.size(); pos += 32)
            m.process(l.data() + pos, r.data() + pos, 32);
        crucible::testing::gTrackAllocations = false;
        expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
#if !defined(CRUCIBLE_TEST_UNTIMED)
        double percent[2] = {0.0, 0.0};
        for (int withPeak = 0; withPeak < 2; ++withPeak)
        {
            LoudnessMeter timed;
            timed.prepare(48000.0, 2, withPeak == 1);
            double best = 1.0e9;
            for (int rep = 0; rep < 4; ++rep)
            {
                const auto t0 = std::chrono::steady_clock::now();
                for (size_t pos = 0; pos + 32 <= l.size(); pos += 32)
                    timed.process(l.data() + pos, r.data() + pos, 32);
                best = std::min(best, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            }
            percent[withPeak] = 100.0 * best / (static_cast<double>(l.size()) / 48000.0);
        }
        logMessage("LoudnessMeter, two channels at 48 kHz: " + String(percent[0], 3) + " % of one core without true peak, " +
                   String(percent[1], 3) + " % with the 4x true-peak interpolators");
        expect(percent[1] < 0.2 * 1.2, "CPU");
#endif
    }
};

static LoudnessTests loudnessTests;
