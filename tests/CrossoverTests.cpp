// SPDX-License-Identifier: MIT
// Crossovers (FX5, docs/plans/ENGINE-FX-PLAN.md sections 3.3 and 3.9): Linkwitz-Riley trees of 6 to 48 dB per octave with
// all-pass compensated bands, and the linear-phase FIR crossover. Flat sums, slopes, phase alignment, null tests, latency,
// allocations and CPU. Every claim is a number.

#include "AllocCounter.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/fx/Crossover.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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

using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;
using crucible::testing::kRate;
namespace cfx = crucible::fx;

namespace
{
constexpr double kTau = 6.28318530717958647692;

struct Bands
{
    std::vector<std::vector<float>> left, right;
    std::vector<float*> pl, pr;
    explicit Bands(int bands, size_t n) : left(static_cast<size_t>(bands), std::vector<float>(n)), right(left)
    {
        for (int b = 0; b < bands; ++b)
        {
            pl.push_back(left[static_cast<size_t>(b)].data());
            pr.push_back(right[static_cast<size_t>(b)].data());
        }
    }
};

std::vector<double> randomFrequencies(int bands, unsigned seed, double minRatio, double maxHz = 18000.0)
{
    Random rng(seed);
    std::vector<double> f;
    while (static_cast<int>(f.size()) < bands - 1)
        f.push_back(std::exp(std::log(40.0) + rng.uniform() * (std::log(maxHz) - std::log(40.0))));
    std::sort(f.begin(), f.end());
    for (size_t i = 1; i < f.size(); ++i)
        f[i] = std::max(f[i], f[i - 1] * minRatio);
    return f;
}

double sumMagnitudeDb(const Bands& b, int bands, size_t n, double hz)
{
    std::vector<double> s(n, 0.0);
    for (int k = 0; k < bands; ++k)
        for (size_t i = 0; i < n; ++i)
            s[i] += b.left[static_cast<size_t>(k)][i];
    const Signal rect(n, 1.0);
    return amplitudeToDb(std::abs(windowedDtft(s.data(), rect.data(), n, hz, kRate)));
}
} // namespace

class CrossoverTests : public UnitTest
{
public:
    CrossoverTests() : UnitTest("Crossover", "Effects") {}

    void runTest() override
    {
        flatSums();
        shapes();
        linearPhase();
        movingFrequencies();
        realTime();
    }

private:
    void flatSums()
    {
        beginTest("minimum phase: the sum of 2 to 5 bands is flat within 0.01 dB at any spacing, 6 to 48 dB per octave");
        {
            double worstAnalytic = 0.0, worstMeasured = 0.0;
            int cases = 0;
            for (const auto slope : {cfx::CrossoverSlope::Db6, cfx::CrossoverSlope::Db12, cfx::CrossoverSlope::Db24,
                                     cfx::CrossoverSlope::Db36, cfx::CrossoverSlope::Db48})
                for (int bands = 2; bands <= 5; ++bands)
                    for (int trial = 0; trial < 12; ++trial)
                    {
                        // Widely spaced, random, and tightly packed (1.02 to 1.1 apart) frequency sets.
                        const double ratio = trial % 3 == 0 ? 1.02 : (trial % 3 == 1 ? 1.1 : 1.0);
                        const auto f = randomFrequencies(bands, static_cast<unsigned>(trial * 31 + bands), ratio);
                        cfx::Crossover x;
                        x.prepare(kRate);
                        x.set(bands, f.data(), slope);
                        for (int i = 0; i <= 300; ++i)
                        {
                            const double hz = 20.0 * std::pow(1000.0, i / 300.0);
                            worstAnalytic = std::max(worstAnalytic, std::abs(amplitudeToDb(std::abs(x.sumResponse(hz)))));
                        }
                        if (trial % 4 == 0)
                        {
                            // And the same through process(), on an impulse.
                            const size_t n = 32768;
                            Bands out(bands, n);
                            std::vector<float> in(n, 0.0f);
                            in[0] = 1.0f;
                            x.reset();
                            x.process(in.data(), in.data(), static_cast<int>(n), out.pl.data(), out.pr.data());
                            for (const double hz : {30.0, 120.0, 440.0, 1000.0, 3000.0, 9000.0, 16000.0})
                                worstMeasured = std::max(worstMeasured, std::abs(sumMagnitudeDb(out, bands, n, hz)));
                        }
                        ++cases;
                    }
            logMessage(String("flat sum over ") + String(cases) + " settings: analytic worst " + String(worstAnalytic, 7) +
                       " dB, measured through process() " + String(worstMeasured, 5) + " dB (plan 0.01)");
            expect(worstAnalytic <= 0.01, "analytic sum");
            expect(worstMeasured <= 0.01, "measured sum");
        }
    }

    void shapes()
    {
        beginTest(
            "Linkwitz-Riley bands: -6 dB and equal phase at the crossover, 12 / 24 / 36 / 48 dB per octave asymptotes");
        {
            const double expected[] = {12.0, 24.0, 36.0, 48.0};
            int k = 0;
            for (const auto slope : {cfx::CrossoverSlope::Db12, cfx::CrossoverSlope::Db24, cfx::CrossoverSlope::Db36,
                                     cfx::CrossoverSlope::Db48})
            {
                cfx::Crossover x;
                x.prepare(192000.0); // the bilinear transform cramps the stop band of a low corner only slightly here
                const double f[] = {1000.0};
                x.set(2, f, slope);
                const double lo = amplitudeToDb(std::abs(x.response(0, 1000.0))),
                             hi = amplitudeToDb(std::abs(x.response(1, 1000.0)));
                const double phase = std::arg(x.response(0, 1000.0) / x.response(1, 1000.0)) * 180.0 / 3.14159265358979;
                // Stop band slope: the low band an octave apart well above the corner, and the high band well below it.
                const double s1 =
                    amplitudeToDb(std::abs(x.response(0, 8000.0))) - amplitudeToDb(std::abs(x.response(0, 16000.0)));
                const double s2 =
                    amplitudeToDb(std::abs(x.response(1, 125.0))) - amplitudeToDb(std::abs(x.response(1, 62.5)));
                logMessage(String(expected[k], 0) + " dB/oct: corner " + String(lo, 3) + " / " + String(hi, 3) +
                           " dB, phase difference " + String(phase, 3) + " deg, slopes " + String(s1, 1) + " / " +
                           String(s2, 1) + " dB per octave");
                expect(std::abs(lo + 6.0206) <= 0.02 && std::abs(hi + 6.0206) <= 0.02, "corner -6 dB");
                expect(std::abs(phase) <= 0.01 || std::abs(std::abs(phase) - 180.0) <= 0.01,
                       "phase relation at the corner (in phase, or inverted for odd orders)");
                // The bilinear transform maps an octave of the analog filter to slightly more than an octave of the digital
                // one.
                const auto octaves = [](double f1, double f2)
                {
                    return std::log2(std::tan(3.14159265358979 * f2 / 192000.0) /
                                     std::tan(3.14159265358979 * f1 / 192000.0));
                };
                expect(std::abs(s1 - expected[k] * octaves(8000.0, 16000.0)) <= 0.25 &&
                           std::abs(s2 - expected[k] * octaves(62.5, 125.0)) <= 0.25,
                       "slope");
                ++k;
            }
            cfx::Crossover x;
            x.prepare(kRate);
            const double f[] = {1000.0};
            x.set(2, f, cfx::CrossoverSlope::Db6);
            const double sum = std::abs(x.response(0, 777.0) + x.response(1, 777.0));
            expect(std::abs(sum - 1.0) <= 1.0e-9, "first-order split sums to exactly 1");
        }
    }

    void linearPhase()
    {
        beginTest(
            "linear phase: the sum is the input delayed by the kernel centre, null at -100 dB or better, for 2 to 5 bands");
        {
            double worst = -300.0;
            for (const int order : {2, 4, 8})
                for (int bands = 2; bands <= 5; ++bands)
                {
                    const auto f = randomFrequencies(bands, static_cast<unsigned>(bands * 7 + order), 1.3, 12000.0);
                    cfx::LinearCrossover x;
                    x.prepare(kRate);
                    x.set(bands, f.data(), order, 12);
                    const int latency = x.latencySamples();
                    const size_t n = static_cast<size_t>(latency * 3);
                    Bands out(bands, n);
                    Random rng(5);
                    std::vector<float> in(n);
                    Signal ref(n);
                    for (size_t i = 0; i < n; ++i)
                    {
                        in[i] = static_cast<float>(0.4 * rng.gaussian());
                        ref[i] = in[i];
                    }
                    x.process(in.data(), in.data(), static_cast<int>(n), out.pl.data(), out.pr.data());
                    Signal sum(n, 0.0);
                    for (int b = 0; b < bands; ++b)
                        for (size_t i = 0; i < n; ++i)
                            sum[i] += out.left[static_cast<size_t>(b)][i];
                    const double null = nullTest(ref, sum, latency, static_cast<size_t>(latency));
                    worst = std::max(worst, null);
                }
            logMessage(String("linear-phase crossover, 4095 taps, orders 2 / 4 / 8, 2-5 bands: sum against the delayed "
                              "input, worst ") +
                       String(worst, 1) + " dB (plan -100)");
            expect(worst <= -100.0, "null");
        }

        beginTest("linear phase: band shapes (-6 dB at the corner, stop band depth) and the reported latency");
        {
            cfx::LinearCrossover x;
            x.prepare(kRate);
            const double f[] = {300.0, 3000.0};
            x.set(3, f, 4, 13);
            const int latency = x.latencySamples();
            expectEquals(latency, cfx::linearPhaseLatency(13), "latency formula");
            const size_t n = static_cast<size_t>(latency * 2 + 8192);
            Bands out(3, n);
            std::vector<float> in(n, 0.0f);
            in[100] = 1.0f;
            x.process(in.data(), in.data(), static_cast<int>(n), out.pl.data(), out.pr.data());
            size_t peak = 0;
            for (size_t i = 0; i < n; ++i)
            {
                double s = 0.0;
                for (int b = 0; b < 3; ++b)
                    s += out.left[static_cast<size_t>(b)][i];
                if (std::abs(s) > 0.5)
                    peak = i;
            }
            expect(peak == static_cast<size_t>(100 + latency),
                   String("impulse comes out ") + String(static_cast<double>(peak) - 100.0, 0) + " samples later");
            const Signal rect(n, 1.0);
            const auto db = [&](int b, double hz)
            {
                std::vector<double> s(out.left[static_cast<size_t>(b)].begin(), out.left[static_cast<size_t>(b)].end());
                return amplitudeToDb(std::abs(windowedDtft(s.data(), rect.data(), n, hz, kRate)));
            };
            const double corner = db(0, 300.0), upper = db(1, 300.0), deep = db(0, 4800.0), flat = db(1, 950.0);
            logMessage(String("corner 300 Hz: low band ") + String(corner, 3) + " dB, mid band " + String(upper, 3) +
                       " dB; low band at 4.8 kHz " + String(deep, 1) + " dB; mid band at 950 Hz " + String(flat, 3) + " dB");
            expect(std::abs(corner + 6.0206) <= 0.1 && std::abs(upper + 6.0206) <= 0.1, "-6 dB at the corner");
            expect(deep <= -70.0, "stop band");
            // The Linkwitz-Riley magnitudes are not flat inside a band that is only a decade wide: 0.17 dB is the analytic
            // value.
            const double analytic = amplitudeToDb(1.0 / (1.0 + std::pow(950.0 / 3000.0, 4.0))) +
                                    amplitudeToDb(1.0 - 1.0 / (1.0 + std::pow(950.0 / 300.0, 4.0)));
            expect(std::abs(flat - analytic) <= 0.02, String("pass band against the analytic ") + String(analytic, 3));
        }
    }

    void movingFrequencies()
    {
        beginTest("moving a crossover frequency under a running signal stays finite and bounded");
        {
            cfx::Crossover x;
            x.prepare(kRate);
            Random rng(2);
            const size_t n = 32;
            Bands out(4, n);
            std::vector<float> in(n);
            double peak = 0.0;
            bool finite = true;
            for (int block = 0; block < 4000; ++block)
            {
                for (auto& v : in)
                    v = static_cast<float>(0.5 * rng.gaussian());
                const double f[] = {100.0 + 90.0 * (1.0 + std::sin(block * 0.01)), 1500.0 + 1000.0 * std::sin(block * 0.013),
                                    9000.0 + 5000.0 * std::sin(block * 0.007)};
                x.set(4, f, cfx::CrossoverSlope::Db24, 32);
                x.process(in.data(), in.data(), static_cast<int>(n), out.pl.data(), out.pr.data());
                for (int b = 0; b < 4; ++b)
                    for (size_t i = 0; i < n; ++i)
                    {
                        finite = finite && std::isfinite(out.left[static_cast<size_t>(b)][i]);
                        peak = std::max(peak, static_cast<double>(std::abs(out.left[static_cast<size_t>(b)][i])));
                    }
            }
            logMessage(String("4 bands, three frequencies swept for 2.7 s: peak ") + String(peak, 2) +
                       " (input noise peak about 2)");
            expect(finite && peak < 10.0, "bounded");
        }
    }

    void realTime()
    {
        beginTest("no allocation on the audio thread: minimum-phase and linear-phase crossovers, frequencies changing every "
                  "block");
        {
            cfx::Crossover x;
            x.prepare(kRate);
            cfx::LinearCrossover lx;
            lx.prepare(kRate);
            const double f0[] = {200.0, 2000.0, 8000.0};
            lx.set(4, f0, 4, 12);
            Random rng(4);
            const size_t n = 32;
            Bands out(4, n);
            std::vector<float> in(n);
            for (int warm = 0; warm < 4000 / 32; ++warm)
            {
                for (auto& v : in)
                    v = static_cast<float>(0.4 * rng.gaussian());
                lx.process(in.data(), in.data(), static_cast<int>(n), out.pl.data(), out.pr.data());
            }
            crucible::testing::gAllocations = 0;
            crucible::testing::gTrackAllocations = true;
            for (int block = 0; block < 1200; ++block)
            {
                for (auto& v : in)
                    v = static_cast<float>(0.4 * rng.gaussian());
                const double f[] = {100.0 + block % 300, 2000.0 + (block * 7) % 900, 9000.0 + (block * 3) % 5000};
                x.set(4, f, static_cast<cfx::CrossoverSlope>(block / 240 % 5), 32);
                x.process(in.data(), in.data(), static_cast<int>(n), out.pl.data(), out.pr.data());
                lx.process(in.data(), in.data(), static_cast<int>(n), out.pl.data(), out.pr.data());
            }
            crucible::testing::gTrackAllocations = false;
            expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
        }

#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest("CPU: five bands, percent of one core per stereo instance at 48 kHz");
        {
            Random rng(6);
            const size_t total = static_cast<size_t>(6.0 * kRate), block = 64;
            std::vector<float> src(total);
            for (auto& v : src)
                v = static_cast<float>(0.3 * rng.gaussian());
            Bands out(5, block);
            const double f[] = {100.0, 500.0, 2500.0, 9000.0};
            const auto percent = [&](auto&& process)
            {
                double best = 1.0e9;
                for (int rep = 0; rep < 3; ++rep)
                {
                    const auto t0 = std::chrono::steady_clock::now();
                    for (size_t pos = 0; pos + block <= total; pos += block)
                        process(src.data() + pos, static_cast<int>(block));
                    best = std::min(best, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
                }
                return 100.0 * best / (static_cast<double>(total) / kRate);
            };
            double minimum[3] = {};
            int i = 0;
            for (const auto slope : {cfx::CrossoverSlope::Db12, cfx::CrossoverSlope::Db24, cfx::CrossoverSlope::Db48})
            {
                cfx::Crossover x;
                x.prepare(kRate);
                x.set(5, f, slope);
                minimum[i++] = percent([&](const float* p, int n) { x.process(p, p, n, out.pl.data(), out.pr.data()); });
            }
            cfx::LinearCrossover lx;
            lx.prepare(kRate);
            lx.set(5, f, 4, 12);
            const double linear = percent([&](const float* p, int n) { lx.process(p, p, n, out.pl.data(), out.pr.data()); });
            logMessage(String("five bands: minimum phase 12 / 24 / 48 dB/oct ") + String(minimum[0], 3) + " / " +
                       String(minimum[1], 3) + " / " + String(minimum[2], 3) + " %, linear phase (4095 taps) " +
                       String(linear, 3) + " %");
            expectCpu(minimum[1] <= 1.2, "minimum-phase 5-band CPU (the multiband plan row is 1.2 % including the processing)");
            expectCpu(linear <= 2.5, "linear-phase 5-band CPU (2.5 % in the multiband plan row)");
        }
#endif
    }
};

static CrossoverTests crossoverTests;
