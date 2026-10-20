// SPDX-License-Identifier: MIT
// Self-tests of the measurement primitives (AnalogMeasure.h): each one is given a synthetic signal whose answer is
// known exactly and has to recover it to the tolerances of the plan (THD within 0.1 dB, slope within 0.2 dB/oct,
// pitch within 0.1 cent, OU standard deviation within 5 %, floor of a clean sine below -130 dB).

#include "AnalogMeasure.h"
#include "TestFramework.h"

#include <cmath>
#include <complex>

using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;

namespace
{
// Direct-form biquad in double precision with its exact transfer function, used as a filter of known response.
struct Biquad
{
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    double z1 = 0, z2 = 0;

    double operator()(double x)
    {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }

    double gainDbAt(double hz, double rate) const
    {
        const Cx z = std::polar(1.0, 2.0 * kPi * hz / rate);
        const Cx zi = 1.0 / z;
        const Cx num = b0 + b1 * zi + b2 * zi * zi;
        const Cx den = 1.0 + a1 * zi + a2 * zi * zi;
        return amplitudeToDb(std::abs(num / den));
    }

    static Biquad lowpass(double hz, double q, double rate)
    {
        const double w0 = 2.0 * kPi * hz / rate, cw = std::cos(w0), alpha = std::sin(w0) / (2.0 * q);
        const double a0 = 1.0 + alpha;
        Biquad b;
        b.b0 = (1.0 - cw) / 2.0 / a0;
        b.b1 = (1.0 - cw) / a0;
        b.b2 = b.b0;
        b.a1 = -2.0 * cw / a0;
        b.a2 = (1.0 - alpha) / a0;
        return b;
    }
};

// Fourth-order Butterworth low-pass: two biquads with the Q values 0.5412 and 1.3066.
struct Butterworth4
{
    Biquad a, b;
    double operator()(double x) { return b(a(x)); }
    double gainDbAt(double hz, double rate) const { return a.gainDbAt(hz, rate) + b.gainDbAt(hz, rate); }
    static Butterworth4 make(double hz, double rate)
    {
        return {Biquad::lowpass(hz, 0.5411961, rate), Biquad::lowpass(hz, 1.3065630, rate)};
    }
};

// A pure rotation of a two-dimensional state: a sine that neither decays nor grows.
struct Rotator
{
    double c, s, radius;
    double x = 0.0, y = 0.0;
    Rotator(double hz, double rate, double r = 1.0)
        : c(std::cos(2.0 * kPi * hz / rate)), s(std::sin(2.0 * kPi * hz / rate)), radius(r)
    {
    }
    double operator()(double in)
    {
        x += in;
        const double nx = radius * (c * x - s * y), ny = radius * (s * x + c * y);
        x = nx;
        y = ny;
        return x;
    }
};

double rmsError(const Signal& a, const Signal& b)
{
    double s = 0.0;
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i)
        s += (a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(s / static_cast<double>(n));
}
} // namespace

class AnalogMeasureTests : public UnitTest
{
public:
    AnalogMeasureTests() : UnitTest("AnalogMeasure", "Verification") {}

    void runTest() override
    {
        beginTest("FFT matches the direct DFT and inverts");
        {
            std::vector<Cx> a(64);
            Random rng(7);
            for (auto& v : a)
                v = Cx(rng.gaussian(), 0.0);
            auto f = a;
            fft(f);
            double worst = 0.0;
            for (size_t k = 0; k < a.size(); ++k)
            {
                Cx d(0, 0);
                for (size_t i = 0; i < a.size(); ++i)
                    d += a[i] * std::polar(1.0, -2.0 * kPi * static_cast<double>(k * i) / 64.0);
                worst = std::max(worst, std::abs(d - f[k]));
            }
            expect(worst < 1.0e-11, "forward FFT differs from the DFT by " + String(worst));
            fft(f, true);
            double back = 0.0;
            for (size_t i = 0; i < a.size(); ++i)
                back = std::max(back, std::abs(f[i] - a[i]));
            expect(back < 1.0e-12, "inverse FFT error " + String(back));
        }

        beginTest("a clean sine measures below -130 dB in every window and at every rate");
        {
            double worstFloor = -300.0;
            for (const double rate : {44100.0, 48000.0, 96000.0})
                for (const double hz : {55.0, 440.0, 1234.567, 9876.54})
                {
                    const Signal s = sineWave(1 << 16, hz, rate, 0.9, 0.3);
                    worstFloor = std::max(worstFloor, measureAliasing(s.data(), s.size(), rate, hz).aliasDb);
                }
            logMessage("clean-sine floor, worst of 12 cases: " + String(worstFloor, 1) + " dB");
        }
        for (const double rate : {44100.0, 48000.0, 96000.0})
            for (const double hz : {55.0, 440.0, 1234.567, 9876.54})
            {
                const Signal s = sineWave(1 << 16, hz, rate, 0.9, 0.3);
                const AliasResult r = measureAliasing(s.data(), s.size(), rate, hz);
                expect(r.aliasDb < -130.0,
                       "sine " + String(hz) + " Hz at " + String(rate) + ": floor " + String(r.aliasDb, 1) + " dB");
                expect(r.worstSpurDb < -140.0, "sine " + String(hz) + " Hz: worst spur " + String(r.worstSpurDb, 1) + " dB");
                const HarmonicProfile p = harmonicProfile(s, rate, hz, 10);
                expect(p.thdPlusNoiseDb < -150.0,
                       "sine " + String(hz) + " Hz: THD+N " + String(p.thdPlusNoiseDb, 1) + " dB");
            }

        beginTest("frequency estimate recovers pitch within 0.1 cent (noise at -60 dB included)");
        {
            Random rng(11);
            double worst = 0.0;
            for (const double rate : {44100.0, 48000.0, 96000.0})
                for (const double hz : {40.0, 82.41, 261.63, 1000.0, 4186.0, 8000.0})
                    for (const double phase : {0.0, 1.7})
                    {
                        Signal s = sineWave(1 << 16, hz, rate, 0.5, phase);
                        for (auto& v : s)
                            v += 0.0005 * rng.gaussian();
                        const double est = estimateFrequency(s, rate);
                        worst = std::max(worst, std::abs(centsBetween(est, hz)));
                    }
            expect(worst < 0.1, "worst pitch error " + String(worst, 5) + " cent");
            logMessage("worst pitch error " + String(worst, 6) + " cent");
            // A length that is not a power of two, and a hint.
            const Signal odd = sineWave(70001, 1500.0, 48000.0);
            expectWithinAbsoluteError(centsBetween(estimateFrequency(odd, 48000.0, 1400.0), 1500.0), 0.0, 0.01);
        }

        beginTest("harmonic levels and THD recover the synthetic truth within 0.1 dB");
        double worstThd = 0.0, worstHarmonic = 0.0;
        for (const double rate : {44100.0, 48000.0, 96000.0})
        {
            const double f0 = 1000.7;
            const double levelsDb[] = {0.0, -60.0, -72.0, -81.0, -90.0, -100.0};
            Signal s(1 << 17, 0.0);
            double harmonicPower = 0.0, evenPower = 0.0;
            for (int k = 1; k <= 6; ++k)
            {
                const double amp = 0.5 * dbToAmplitude(levelsDb[k - 1]);
                const Signal h = sineWave(s.size(), f0 * k, rate, amp, 0.4 * k);
                for (size_t i = 0; i < s.size(); ++i)
                    s[i] += h[i];
                if (k >= 2)
                {
                    harmonicPower += std::pow(dbToAmplitude(levelsDb[k - 1]), 2.0);
                    if (k % 2 == 0)
                        evenPower += std::pow(dbToAmplitude(levelsDb[k - 1]), 2.0);
                }
            }
            const HarmonicProfile p = harmonicProfile(s, rate, 0.0, 6);
            expectWithinAbsoluteError(p.f0Hz, f0, 0.001, "estimated fundamental");
            for (int k = 2; k <= 6; ++k)
                expectWithinAbsoluteError(p.harmonicDb(k), levelsDb[k - 1], 0.1,
                                          "harmonic " + String(k) + " at " + String(rate));
            worstThd = std::max(worstThd, std::abs(p.thdDb - powerToDb(harmonicPower)));
            for (int k = 2; k <= 6; ++k)
                worstHarmonic = std::max(worstHarmonic, std::abs(p.harmonicDb(k) - levelsDb[k - 1]));
            expectWithinAbsoluteError(p.thdDb, powerToDb(harmonicPower), 0.1, "THD");
            expectWithinAbsoluteError(p.evenDb, powerToDb(evenPower), 0.1, "even-harmonic share");
            expectWithinAbsoluteError(p.thdPlusNoiseDb, powerToDb(harmonicPower), 0.1, "THD+N without noise equals THD");
        }

        logMessage("worst THD error " + String(worstThd, 5) + " dB, worst harmonic level error " + String(worstHarmonic, 5) +
                   " dB");

        beginTest("THD+N counts noise, and the 1/n law of a band-limited saw is recovered");
        {
            const double rate = 48000.0, f0 = 220.0;
            Random rng(3);
            Signal s(1 << 17, 0.0);
            for (int k = 1; k * f0 < 0.49 * rate; ++k)
            {
                const Signal h = sineWave(s.size(), f0 * k, rate, 0.3 / k, 0.1 * k);
                for (size_t i = 0; i < s.size(); ++i)
                    s[i] += h[i];
            }
            const HarmonicProfile saw = harmonicProfile(s, rate, f0, 90);
            const ShapeError e = shapeError(saw, [](int k) { return -20.0 * std::log10(static_cast<double>(k)); }, 10000.0);
            expect(e.maxAbsDb < 0.01, "saw law error " + String(e.maxAbsDb, 5) + " dB");
            expect(measureAliasing(s.data(), 1 << 17, rate, f0).aliasDb < -130.0, "additive saw has no aliasing");

            const double sigma = 1.0e-3;
            for (auto& v : s)
                v += sigma * rng.gaussian();
            const HarmonicProfile noisy = harmonicProfile(s, rate, f0, 90);
            // Truth: noise power against the fundamental's power (amplitude 0.3 -> power 0.045) plus the harmonics.
            double harm = 0.0;
            for (int k = 2; k * f0 < 0.49 * rate; ++k)
                harm += 0.5 * std::pow(0.3 / k, 2.0);
            const double truth = powerToDb((harm + sigma * sigma) / (0.5 * 0.09));
            expectWithinAbsoluteError(noisy.thdPlusNoiseDb, truth, 0.1, "THD+N with noise");
        }

        beginTest("aliasing: an inharmonic spur at -80 dB reads -80 dB");
        for (const double rate : {44100.0, 48000.0, 96000.0})
        {
            const double f0 = 440.0;
            Signal s = sineWave(1 << 16, f0, rate, 0.5);
            const Signal spur = sineWave(s.size(), 7777.7, rate, 0.5e-4, 1.1);
            for (size_t i = 0; i < s.size(); ++i)
                s[i] += spur[i];
            const AliasResult r = measureAliasing(s.data(), s.size(), rate, f0);
            expectWithinAbsoluteError(r.aliasDb, -80.0, 0.1, "alias level at " + String(rate));
            expectWithinAbsoluteError(r.worstSpurDb, -80.0, 0.1, "spur level");
            expectWithinAbsoluteError(r.worstSpurHz, 7777.7, rate / (1 << 16) * 1.5, "spur frequency");
            // Limited to 5 kHz the spur is out of band.
            expect(measureAliasing(s.data(), s.size(), rate, f0, 5000.0).aliasDb < -130.0, "band limit hides the spur");
        }

        beginTest("band RMS, DC and noise floor of white noise");
        {
            const double rate = 48000.0, sigma = 1.0e-3;
            Random rng(5);
            Signal n(1 << 18);
            for (auto& v : n)
                v = sigma * rng.gaussian();
            expectWithinAbsoluteError(rmsDbfs(n), -60.0, 0.05, "broadband RMS");
            expectWithinAbsoluteError(bandRmsDbfs(n.data(), n.size(), rate, 20.0, 0.5 * rate - 100.0), -60.0, 0.1,
                                      "full band");
            expectWithinAbsoluteError(bandRmsDbfs(n.data(), n.size(), rate, 1000.0, 3000.0),
                                      -60.0 + 10.0 * std::log10(2000.0 / 24000.0), 0.1, "1-3 kHz");
            Signal d = n;
            for (auto& v : d)
                v += 0.01;
            expectWithinAbsoluteError(dcOffset(d), 0.01, 2.0e-5, "DC offset");
            expectWithinAbsoluteError(dcDbfs(d), -40.0, 0.2, "DC level");
            // A tone that does not fit the window a whole number of times leaves a bias in the plain mean only.
            Signal tone = sineWave(1 << 16, 65.4074, rate, 0.9, 0.3);
            expect(dcDbfs(tone) > -90.0, "plain mean of a partial period is biased: " + String(dcDbfs(tone), 1) + " dBFS");
            expect(amplitudeToDb(std::abs(dcOffsetWindowed(tone.data(), tone.size()))) < -140.0,
                   "windowed mean of a tone reads no DC");
            for (auto& v : tone)
                v += 1.0e-4;
            expectWithinAbsoluteError(amplitudeToDb(dcOffsetWindowed(tone.data(), tone.size())), -80.0, 0.05,
                                      "windowed DC under a tone");
            const Signal sine = sineWave(48000, 1000.0, rate, 1.0);
            expectWithinAbsoluteError(rmsDbfs(sine), -3.0103, 0.001, "sine RMS");
        }

        beginTest("true peak finds the inter-sample maximum");
        {
            // A sine at fs/4 with 45 degrees of phase has samples of only 0.7071 but peaks at 1.0.
            const Signal s = sineWave(4096, 12000.0, 48000.0, 1.0, kPi / 4.0);
            double samplePeak = 0.0;
            for (const double v : s)
                samplePeak = std::max(samplePeak, std::abs(v));
            expectWithinAbsoluteError(amplitudeToDb(samplePeak), -3.0103, 0.01, "sample peak");
            expectWithinAbsoluteError(amplitudeToDb(truePeak(s, 4)), 0.0, 0.1, "true peak");
            expectWithinAbsoluteError(amplitudeToDb(truePeak(sineWave(4096, 997.0, 48000.0, 0.5), 4)), -6.0206, 0.01,
                                      "slow sine");
        }

        beginTest("filter response, slope, -3 dB point and resonance peak of known filters");
        for (const double rate : {48000.0, 96000.0})
        {
            const double fc = 100.0; // low, so that the bilinear transform's frequency warping stays under 0.2 dB/oct
            const Butterworth4 ref = Butterworth4::make(fc, rate);
            const auto freqs = logSpacedFrequencies(25.0, 3200.0, 24);
            const ResponseCurve c =
                steppedResponse([&] { return Butterworth4::make(fc, rate); }, freqs, rate, 0.01, 0.3, 0.2);
            double worst = 0.0;
            for (size_t i = 0; i < freqs.size(); ++i)
                worst = std::max(worst, std::abs(c.gainDb[i] - ref.gainDbAt(freqs[i], rate)));
            expect(worst < 0.01, "stepped response off by " + String(worst, 5) + " dB at " + String(rate));
            // Truth: the same slope computed from the exact transfer function.
            ResponseCurve exact;
            for (const double f : freqs)
            {
                exact.freqHz.push_back(f);
                exact.gainDb.push_back(ref.gainDbAt(f, rate));
            }
            const double slope = slopeDbPerOctave(c, 4.0 * fc, 16.0 * fc);
            expectWithinAbsoluteError(slope, slopeDbPerOctave(exact, 4.0 * fc, 16.0 * fc), 0.05,
                                      "slope against the transfer function");
            logMessage("slope " + String(slope, 4) + " dB/oct against " +
                       String(slopeDbPerOctave(exact, 4.0 * fc, 16.0 * fc), 4) + " exact at " + String(rate));
            expectWithinAbsoluteError(slope, -24.0, 0.2, "24 dB/oct Butterworth slope");
            expectWithinAbsoluteError(crossingBelow(c, -3.0103, 50.0), fc, fc * 0.005, "-3 dB frequency");

            // Resonant 2-pole low-pass: peak height and position from its transfer function.
            const double q = 8.0, f0 = 1000.0;
            const auto fine = logSpacedFrequencies(300.0, 3000.0, 48);
            const ResponseCurve rc =
                steppedResponse([&] { return Biquad::lowpass(f0, q, rate); }, fine, rate, 0.001, 0.3, 0.2);
            const Biquad rb = Biquad::lowpass(f0, q, rate);
            const double fineGrid = 0.001;
            double bestF = 0, bestG = -300;
            for (double f = 800.0; f < 1200.0; f += fineGrid)
            {
                const double g = rb.gainDbAt(f, rate);
                if (g > bestG)
                {
                    bestG = g;
                    bestF = f;
                }
            }
            const PeakInfo pk = resonancePeak(rc);
            expectWithinAbsoluteError(pk.gainDb, bestG, 0.05, "resonance peak height");
            expectWithinAbsoluteError(pk.freqHz, bestF, bestF * 0.002, "resonance peak frequency");
        }

        beginTest("sweep impulse response reproduces the magnitude of a known filter");
        {
            const double rate = 48000.0, fc = 1000.0;
            const Butterworth4 ref = Butterworth4::make(fc, rate);
            const Signal ir =
                sweepImpulseResponse([&] { return Butterworth4::make(fc, rate); }, rate, 20.0, 22000.0, 4.0, 0.01, 0.25);
            const auto freqs = logSpacedFrequencies(100.0, 10000.0, 4);
            const ResponseCurve c = responseFromImpulse(ir, rate, freqs);
            double worst = 0.0;
            for (size_t i = 0; i < freqs.size(); ++i)
                worst = std::max(worst, std::abs(c.gainDb[i] - ref.gainDbAt(freqs[i], rate)));
            expect(worst < 0.1, "sweep response off by " + String(worst, 4) + " dB");
            logMessage("sweep response error " + String(worst, 5) + " dB");
        }

        beginTest("self-oscillation: pitch tracking within 0.1 cent, steady amplitude, decay is not oscillation");
        {
            double worstCents = 0.0, worstVariation = 0.0;
            for (const double rate : {44100.0, 48000.0, 96000.0})
                for (const double hz : {40.0, 100.0, 440.0, 1000.0, 3000.0, 8000.0})
                {
                    const SelfOscillation r = measureSelfOscillation([&] { return Rotator(hz, rate); }, rate, 1.5, 0.01);
                    expect(r.oscillating, "rotator at " + String(hz) + " Hz is oscillating");
                    worstCents = std::max(worstCents, std::abs(centsBetween(r.freqHz, hz)));
                    worstVariation = std::max(worstVariation, r.amplitudeVariationDb);
                }
            expect(worstCents < 0.1, "tracking error " + String(worstCents, 5) + " cent");
            expect(worstVariation < 0.01, "amplitude variation " + String(worstVariation, 5) + " dB");
            const SelfOscillation decaying =
                measureSelfOscillation([] { return Rotator(1000.0, 48000.0, 0.99995); }, 48000.0, 2.0, 0.01);
            expect(!decaying.oscillating, "a decaying resonator is not self-oscillating");
        }

        beginTest("Ornstein-Uhlenbeck statistics: standard deviation, time constant and PSD slope");
        {
            const double dt = 0.05, tau = 2.0, sigma = 6.0;
            const Signal x = generateOu(400000, dt, tau, sigma, 21);
            const OuStats s = ouStats(x, dt, 0.5, 3.0);
            expect(std::abs(s.stdDev / sigma - 1.0) < 0.05, "std " + String(s.stdDev, 4) + " against 6");
            expect(std::abs(s.tauSeconds / tau - 1.0) < 0.05, "tau " + String(s.tauSeconds, 4) + " against 2");
            expectWithinAbsoluteError(s.psdSlope, -2.0, 0.3, "PSD slope above the corner");
            logMessage("OU std " + String(s.stdDev, 4) + ", tau " + String(s.tauSeconds, 4) + ", slope " +
                       String(s.psdSlope, 3));
            expect(generateOu(1000, dt, tau, sigma, 21) == generateOu(1000, dt, tau, sigma, 21), "same seed, same series");
            expect(generateOu(1000, dt, tau, sigma, 21) != generateOu(1000, dt, tau, sigma, 22), "other seed, other series");

            // Pooled runs of 10 s, one per note (200 notes in the plan; more here to keep the estimate tight).
            std::vector<Signal> runs;
            for (int i = 0; i < 400; ++i)
                runs.push_back(generateOu(100, 0.1, 1.0, 6.0, 1000 + static_cast<std::uint64_t>(i)));
            const OuStats pooled = ouStats(runs, 0.1, 0.5, 3.0);
            expect(std::abs(pooled.stdDev / 6.0 - 1.0) < 0.05, "pooled std " + String(pooled.stdDev, 4));
            expect(std::abs(pooled.tauSeconds / 1.0 - 1.0) < 0.1, "pooled tau " + String(pooled.tauSeconds, 4));
        }

        beginTest("pitch trace follows a slow vibrato to a few hundredths of a cent");
        {
            const double rate = 48000.0;
            Signal s(static_cast<size_t>(8 * rate));
            double phase = 0.0;
            for (size_t i = 0; i < s.size(); ++i)
            {
                const double t = static_cast<double>(i) / rate;
                const double cents = 5.0 * std::sin(2.0 * kPi * 0.5 * t);
                phase += 2.0 * kPi * 440.0 * std::pow(2.0, cents / 1200.0) / rate;
                s[i] = 0.5 * std::sin(phase);
            }
            const Signal trace = pitchTraceCents(s, rate, 0.1, 0.0854, 440.0);
            Signal truth;
            for (size_t k = 0; k < trace.size(); ++k)
                truth.push_back(5.0 * std::sin(2.0 * kPi * 0.5 * (0.1 * static_cast<double>(k) + 2048.0 / rate)));
            expect(rmsError(trace, truth) < 0.05, "vibrato trace error " + String(rmsError(trace, truth), 4) + " cent");
        }

        beginTest("envelope fits recover time constants and levels");
        {
            Signal t, y;
            for (int i = 0; i <= 1000; ++i)
            {
                t.push_back(i * 0.001);
                y.push_back(0.1 + 0.9 * std::exp(-i * 0.001 / 0.25));
            }
            const ExpFit d = fitExponential(t, y);
            expect(std::abs(d.tau / 0.25 - 1.0) < 1.0e-4, "decay tau " + String(d.tau, 6));
            expectWithinAbsoluteError(d.offset, 0.1, 1.0e-6);
            expect(d.residualRms < 1.0e-6, "decay residual " + String(d.residualRms));

            // RC charge towards 1.2 (an attack that overshoots its target level), with a little noise.
            Random rng(9);
            Signal ya;
            for (const double tt : t)
                ya.push_back(1.2 * (1.0 - std::exp(-tt / 0.05)) + 0.0005 * rng.gaussian());
            const ExpFit a = fitExponential(t, ya);
            expect(std::abs(a.tau / 0.05 - 1.0) < 0.01, "attack tau " + String(a.tau, 6));
            expectWithinAbsoluteError(a.offset, 1.2, 0.002, "attack target");
            expect(a.residualRms < 0.005, "attack residual " + String(a.residualRms));
            expectWithinAbsoluteError(timeToLevel(t, y, 0.1 + 0.9 * std::exp(-1.0), false), 0.25, 0.001, "time to 1/e");

            // The same envelope read back from audio: a 440 Hz sine under an exponential release.
            const double rate = 48000.0;
            Signal audio(static_cast<size_t>(1.5 * rate));
            for (size_t i = 0; i < audio.size(); ++i)
                audio[i] = std::exp(-static_cast<double>(i) / rate / 0.3) *
                           std::sin(2.0 * kPi * 440.0 * static_cast<double>(i) / rate);
            const EnvelopeTrace env = rmsEnvelope(audio, rate, 9.0 / 440.0, 0.005);
            const ExpFit e = fitExponential(env.timeSeconds, env.level);
            expect(std::abs(e.tau / 0.3 - 1.0) < 0.01, "audio envelope tau " + String(e.tau, 5));
        }
    }
};

static AnalogMeasureTests analogMeasureTests;
