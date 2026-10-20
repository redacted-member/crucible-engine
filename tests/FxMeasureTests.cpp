// SPDX-License-Identifier: MIT
// Self-tests of the effect measurement primitives (FxMeasure.h), each against a synthetic truth, and tests of the FX
// core library (crucible/fx/*.h): smoothing, channel modes, detectors, lookahead and the oversampled block.

#include "FxMeasure.h"
#include "TestFramework.h"

#include <crucible/fx/ChannelMode.h>
#include <crucible/fx/Detectors.h>
#include <crucible/fx/Lookahead.h>
#include <crucible/fx/OversampledBlock.h>
#include <crucible/fx/Smoothing.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/Module.h>

#include <cmath>
#include <deque>

using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;
namespace fx = crucible::fx;

namespace
{
std::vector<float> toFloat(const Signal& s)
{
    return std::vector<float>(s.begin(), s.end());
}
Signal fromFloat(const std::vector<float>& s)
{
    return Signal(s.begin(), s.end());
}

Signal noise(size_t n, std::uint64_t seed, double amplitude = 0.3)
{
    Random rng(seed);
    Signal x(n);
    for (double& v : x)
        v = amplitude * rng.gaussian();
    return x;
}

// Sum of many bin-centred sines below 0.3 fs, so that it is periodic in a power-of-two length.
Signal multisine(size_t n, std::uint64_t seed, int tones = 120)
{
    Random rng(seed);
    Signal x(n, 0.0);
    for (int k = 0; k < tones; ++k)
    {
        const size_t bin = 4 + static_cast<size_t>(rng.uniform() * 0.3 * static_cast<double>(n));
        const double phase = 2.0 * kPi * rng.uniform();
        for (size_t i = 0; i < n; ++i)
            x[i] += 0.02 * std::sin(2.0 * kPi * static_cast<double>(bin * i % n) / static_cast<double>(n) + phase);
    }
    return x;
}

// Ideal static gain computer: threshold -20 dB, ratio 4; the gain depends only on the level of the whole input.
Signal idealCompressor(const Signal& in)
{
    double peak = 0.0;
    for (const double v : in)
        peak = std::max(peak, std::abs(v));
    const double level = amplitudeToDb(peak);
    const double gainDb = level > -20.0 ? (-20.0 + (level + 20.0) / 4.0) - level : 0.0;
    Signal out(in.size());
    for (size_t i = 0; i < in.size(); ++i)
        out[i] = in[i] * dbToAmplitude(gainDb);
    return out;
}
} // namespace

class FxMeasureTests : public UnitTest
{
public:
    FxMeasureTests() : UnitTest("FxMeasure", "Verification") {}

    void runTest() override
    {
        const double rate = 48000.0;

        beginTest("staticCurve recovers an ideal gain computer within 0.01 dB");
        {
            std::vector<double> levels;
            for (double l = -60.0; l <= 6.0; l += 6.0)
                levels.push_back(l);
            const auto curve = staticCurve([](const Signal& x) { return idealCompressor(x); }, rate, levels);
            double worst = 0.0;
            for (const auto& p : curve)
            {
                const double ideal = p.inDb > -20.0 ? (-20.0 + (p.inDb + 20.0) / 4.0) - p.inDb : 0.0;
                worst = std::max(worst, std::abs(p.gainDb - ideal));
            }
            logMessage("static curve, worst error " + String(worst, 5) + " dB");
            expect(worst < 0.01, "static curve error " + String(worst));
        }

        beginTest("stepTimes recovers a one-pole attack and release within 1 %");
        for (const double sr : {44100.0, 48000.0})
        {
            const double hz = sr / 12.0;
            for (const double tauMs : {5.0, 12.0, 150.0})
            {
                const size_t event = static_cast<size_t>(1.0 * sr);
                auto render = [&](const Signal& in)
                {
                    Signal out(in.size());
                    double g = 0.0; // dB
                    const double c = 1.0 - std::exp(-1.0 / (tauMs * 1.0e-3 * sr));
                    for (size_t i = 0; i < in.size(); ++i)
                    {
                        g += ((i >= event ? -10.0 : 0.0) - g) * c;
                        out[i] = in[i] * dbToAmplitude(g);
                    }
                    return out;
                };
                const StepMetrics m = stepTimes(render, sr, -40.0, -20.0, hz, 1.0, 2.0);
                const double rise = tauMs * std::log(9.0);
                expect(m.valid(), "step not measured");
                expect(std::abs(m.t63Ms / tauMs - 1.0) < 0.01, "t63 " + String(m.t63Ms, 3) + " ms for tau " + String(tauMs));
                expect(std::abs(m.riseMs() / rise - 1.0) < 0.01,
                       "10-90 " + String(m.riseMs(), 3) + " ms for " + String(rise));
                expect(std::abs(m.endDb + 10.0) < 0.02 && std::abs(m.startDb) < 0.02, "gain levels");
            }
        }

        beginTest("releaseFamily recovers a two-state model within 2 %");
        {
            const double tauFast = 0.060, tauSlow = 1.2, chargeFast = 0.005, chargeSlow = 0.8, target = 10.0;
            const double pre = 1.0;
            auto modelT63 = [&](double burst)
            {
                const double f0 = target * (1.0 - std::exp(-burst / chargeFast)),
                             s0 = target * (1.0 - std::exp(-burst / chargeSlow));
                const double gr0 = 0.5 * f0 + 0.5 * s0;
                auto gr = [&](double t) { return 0.5 * f0 * std::exp(-t / tauFast) + 0.5 * s0 * std::exp(-t / tauSlow); };
                double lo = 0.0, hi = 20.0;
                for (int i = 0; i < 100; ++i)
                {
                    const double mid = 0.5 * (lo + hi);
                    (gr(mid) > gr0 * std::exp(-1.0) ? lo : hi) = mid;
                }
                return 0.5 * (lo + hi) * 1000.0;
            };
            double worst = 0.0;
            for (const double burst : {0.2, 2.0})
            {
                auto render = [&](const Signal& in)
                {
                    Signal out(in.size());
                    double f = 0.0, s = 0.0;
                    const size_t b0 = static_cast<size_t>(pre * rate), b1 = b0 + static_cast<size_t>(burst * rate);
                    for (size_t i = 0; i < in.size(); ++i)
                    {
                        const bool on = i >= b0 && i < b1;
                        f += ((on ? target : 0.0) - f) * (1.0 - std::exp(-1.0 / ((on ? chargeFast : tauFast) * rate)));
                        s += ((on ? target : 0.0) - s) * (1.0 - std::exp(-1.0 / ((on ? chargeSlow : tauSlow) * rate)));
                        out[i] = in[i] * dbToAmplitude(-(0.5 * f + 0.5 * s));
                    }
                    return out;
                };
                const auto fam = releaseFamily(render, rate, -40.0, {-10.0}, {burst}, 0.0, pre, 14.0);
                const double truth = modelT63(burst);
                const double err = std::abs(fam[0].release.t63Ms / truth - 1.0);
                worst = std::max(worst, err);
                logMessage("burst " + String(burst, 1) + " s: t63 " + String(fam[0].release.t63Ms, 1) + " ms, model " +
                           String(truth, 1));
                expect(err < 0.02, "release t63 off by " + String(100.0 * err) + " %");
            }
            logMessage("release family, worst error " + String(100.0 * worst, 3) + " %");
        }

        beginTest("thdCurve and harmonicCurve recover a polynomial's harmonics within 0.1 dB");
        {
            const double a2 = 0.05, a3 = 0.03;
            auto poly = [&](const Signal& x)
            {
                Signal y(x.size());
                for (size_t i = 0; i < x.size(); ++i)
                    y[i] = x[i] + a2 * x[i] * x[i] + a3 * x[i] * x[i] * x[i];
                return y;
            };
            auto truth = [&](double a, double& h2, double& h3, double& thd)
            {
                const double fund = a + 0.75 * a3 * a * a * a, c2 = 0.5 * a2 * a * a, c3 = 0.25 * a3 * a * a * a;
                h2 = amplitudeToDb(c2 / fund);
                h3 = amplitudeToDb(c3 / fund);
                thd = amplitudeToDb(std::hypot(c2, c3) / fund);
            };
            double worst = 0.0;
            const auto byLevel = thdCurve(poly, rate, 1000.0, {-20.0, -10.0, -4.0});
            for (const auto& p : byLevel)
            {
                double h2, h3, thd;
                truth(dbToAmplitude(p.x), h2, h3, thd);
                worst = std::max({worst, std::abs(p.h2Db - h2), std::abs(p.h3Db - h3), std::abs(p.thdDb - thd)});
            }
            const auto byFreq = harmonicCurve(poly, rate, -6.0, {100.0, 1234.5, 5000.0});
            for (const auto& p : byFreq)
            {
                double h2, h3, thd;
                truth(dbToAmplitude(-6.0), h2, h3, thd);
                worst = std::max({worst, std::abs(p.h2Db - h2), std::abs(p.h3Db - h3), std::abs(p.thdDb - thd)});
            }
            logMessage("harmonic levels, worst error " + String(worst, 5) + " dB");
            expect(worst < 0.1, "harmonic error " + String(worst));
        }

        beginTest("imd recovers a polynomial's intermodulation lines within 0.1 dB");
        {
            const double a2 = 0.02, a3 = 0.01;
            auto poly = [&](const Signal& x)
            {
                Signal y(x.size());
                for (size_t i = 0; i < x.size(); ++i)
                    y[i] = x[i] + a2 * x[i] * x[i] + a3 * x[i] * x[i] * x[i];
                return y;
            };
            const double peak = dbToAmplitude(-6.0);
            {
                const double A1 = peak * 0.8, A2 = peak * 0.2;
                const double ref = A2 + a3 * (0.75 * A2 * A2 * A2 + 1.5 * A1 * A1 * A2);
                const ImdResult r = imdSmpte(poly, rate);
                double worst = 0.0;
                for (const ImdLine& l : r.lines)
                {
                    const double k = std::round(std::abs(l.hz - 7000.0) / 60.0);
                    const double amp = k == 1 ? a2 * A1 * A2 : (k == 2 ? a3 * 0.75 * A1 * A1 * A2 : 0.0);
                    if (amp > 0.0)
                        worst = std::max(worst, std::abs(l.db - amplitudeToDb(amp / ref)));
                }
                logMessage("SMPTE lines, worst error " + String(worst, 4) + " dB, total " + String(r.totalDb, 2) + " dB");
                expect(worst < 0.1, "SMPTE line error " + String(worst));
                expect(r.lines.size() == 8, "SMPTE line count");
            }
            {
                const double A = 0.5 * peak;
                const double tone = A + a3 * 2.25 * A * A * A;
                const ImdResult r = imdCcif(poly, rate);
                double worst = 0.0;
                for (const ImdLine& l : r.lines)
                {
                    double amp = 0.0;
                    if (std::abs(l.hz - 1000.0) < 1.0)
                        amp = a2 * A * A;
                    else if (std::abs(l.hz - 18000.0) < 1.0 || std::abs(l.hz - 21000.0) < 1.0)
                        amp = a3 * 0.75 * A * A * A;
                    if (amp > 0.0)
                        worst = std::max(worst, std::abs(l.db - amplitudeToDb(amp / (2.0 * tone))));
                }
                logMessage("CCIF lines, worst error " + String(worst, 4) + " dB, total " + String(r.totalDb, 2) + " dB");
                expect(worst < 0.1, "CCIF line error " + String(worst));
            }
        }

        beginTest("aliasAtDrive reads an inharmonic spur at -80 dB and no alias for a harmonic distortion");
        {
            auto spur = [&](const Signal& x)
            {
                Signal y = x;
                const Signal s = sineWave(x.size(), 7777.7, rate, 0.5e-4, 1.1);
                for (size_t i = 0; i < y.size(); ++i)
                    y[i] += s[i];
                return y;
            };
            const auto r = aliasAtDrive(spur, rate, 5000.0, {-6.0});
            // The spur is 1e-4 of the 0.5 fundamental's amplitude: -80 dB.
            expectWithinAbsoluteError(r[0].worstSpurDb, -80.0 + 0.0, 0.1, "worst spur");
            expectWithinAbsoluteError(r[0].worstSpurHz, 7777.7, 3.0, "spur frequency");
            auto poly = [&](const Signal& x)
            {
                Signal y(x.size());
                for (size_t i = 0; i < x.size(); ++i)
                    y[i] = x[i] + 0.05 * x[i] * x[i] * x[i];
                return y;
            };
            const auto p = aliasAtDrive(poly, rate, 1000.0, {-6.0});
            logMessage("harmonic distortion alias floor " + String(p[0].aliasDb, 1) + " dB");
            expect(p[0].aliasDb < -130.0, "harmonic distortion shows alias " + String(p[0].aliasDb));
        }

        beginTest("nullTest reads a pure delay below -150 dB");
        {
            const Signal x = noise(1 << 14, 5);
            expect(nullTest(x, delaySignal(x, 37.0), 37.0) < -150.0, "integer delay");
            const Signal m = multisine(1 << 14, 9);
            const double fractional = 5.37;
            Signal shifted(m.size());
            // The multisine is periodic, so its delayed copy is analytic: rebuild it with the same tones.
            shifted = delaySignal(m, fractional, true);
            const double d = nullTest(m, shifted, fractional, 0, true);
            logMessage("fractional pure delay residual " + String(d, 1) + " dB");
            expect(d < -150.0, "fractional delay residual " + String(d));
            expect(nullTest(m, m, 0.0) < -150.0, "zero delay");
            Signal off = m;
            for (size_t i = 0; i < off.size(); ++i)
                off[i] += 1.0e-3 * (i % 2 == 0 ? 1.0 : -1.0);
            expectWithinAbsoluteError(nullTest(m, off, 0.0), amplitudeToDb(1.0e-3 / rmsOf(m.data(), m.size())), 0.1,
                                      "known residual");
        }

        beginTest("measureLatency finds fractional delays within 0.01 sample");
        {
            const size_t n = 1 << 15;
            Random rng(11);
            std::vector<double> f, ph;
            for (int k = 0; k < 200; ++k)
            {
                f.push_back(0.01 + 0.34 * rng.uniform());
                ph.push_back(2.0 * kPi * rng.uniform());
            }
            auto make = [&](double delay)
            {
                Signal x(n);
                for (size_t i = 0; i < n; ++i)
                {
                    double v = 0.0;
                    for (size_t k = 0; k < f.size(); ++k)
                        v += std::sin(2.0 * kPi * f[k] * (static_cast<double>(i) - delay) + ph[k]);
                    x[i] = 0.05 * v;
                }
                return x;
            };
            const Signal ref = make(0.0);
            double worst = 0.0;
            for (const double d : {0.0, 3.0, 7.37, 45.5, 100.01})
                worst = std::max(worst, std::abs(measureLatency(ref, make(d)) - d));
            logMessage("latency, worst error " + String(worst, 5) + " sample");
            expect(worst < 0.01, "latency error " + String(worst));
        }

        beginTest("clickTest reads a jump of the sample value and ignores smooth modulation");
        {
            const double hz = 1000.0, w = 2.0 * kPi * hz / rate;
            const size_t n = 20000;
            for (const size_t first : {size_t(9000), size_t(9007), size_t(9013)})
            {
                size_t at = first;
                while (std::abs(std::sin(w * static_cast<double>(at) + 0.3)) < 0.8)
                    ++at;
                for (const double stepDb : {-40.0, -60.0, -80.0})
                {
                    const double eps = dbToAmplitude(stepDb), amp = 0.5;
                    Signal y(n);
                    for (size_t i = 0; i < n; ++i)
                        y[i] = amp * (i >= at ? 1.0 + eps : 1.0) * std::sin(w * static_cast<double>(i) + 0.3);
                    const double jump = amp * eps * std::abs(std::sin(w * static_cast<double>(at) + 0.3));
                    const ClickResult c = clickTest(y, hz, rate);
                    expectWithinAbsoluteError(c.stepDb, amplitudeToDb(jump), 0.5, "step height");
                    expect(c.at == at, "step position " + String(static_cast<double>(c.at)));
                }
            }
            Signal smooth(n);
            for (size_t i = 0; i < n; ++i)
                smooth[i] = 0.5 * (1.0 + 0.5 * std::sin(2.0 * kPi * 3.0 * static_cast<double>(i) / rate)) *
                            std::sin(w * static_cast<double>(i));
            expect(clickTest(smooth, hz, rate).stepDb < -120.0,
                   "smooth modulation reads " + String(clickTest(smooth, hz, rate).stepDb));
            // A phase jump registers too, with the value jump it causes.
            const size_t at = 9000;
            Signal jump(n);
            for (size_t i = 0; i < n; ++i)
                jump[i] = 0.5 * std::sin(w * static_cast<double>(i) + (i >= at ? 0.001 : 0.0));
            const double expected = 0.5 * std::abs(std::sin(w * 9000.0 + 0.001) - std::sin(w * 9000.0));
            expectWithinAbsoluteError(clickTest(jump, hz, rate).stepDb, amplitudeToDb(expected), 0.3, "phase step");
            // The carrier frequency can be anywhere: 100 Hz at 96 kHz.
            Signal low(n);
            const double wl = 2.0 * kPi * 100.0 / 96000.0;
            for (size_t i = 0; i < n; ++i)
                low[i] = 0.5 * (i >= at ? 1.001 : 1.0) * std::sin(wl * static_cast<double>(i) + 1.0);
            expectWithinAbsoluteError(clickTest(low, 100.0, 96000.0).stepDb,
                                      amplitudeToDb(0.5e-3 * std::abs(std::sin(wl * 9000.0 + 1.0))), 0.3, "low carrier");
        }

        beginTest("zipperTest separates a block staircase from smooth ramps");
        {
            const double A = dbToAmplitude(-6.0);
            auto stair = [&](const Signal& in, const Signal& a)
            {
                Signal out(in.size());
                double held = 0.0;
                for (size_t i = 0; i < in.size(); ++i)
                {
                    if (i % 32 == 0)
                        held = a[i];
                    out[i] = in[i] * (0.5 + 0.5 * held);
                }
                return out;
            };
            auto smooth = [&](const Signal& in, const Signal& a)
            {
                Signal out(in.size());
                for (size_t i = 0; i < in.size(); ++i)
                    out[i] = in[i] * (0.5 + 0.5 * a[i]);
                return out;
            };
            const ZipperResult z = zipperTest(stair, rate);
            const double expectedRamp = amplitudeToDb(A * 0.5 * 32.0 * 1.875 / (0.05 * rate));
            logMessage("staircase: ramp " + String(z.rampDb, 1) + " dB (expected " + String(expectedRamp, 1) + "), square " +
                       String(z.squareDb, 1));
            expectWithinAbsoluteError(z.rampDb, expectedRamp, 1.0, "staircase ramp step");
            expect(z.squareDb > -60.0, "staircase square wave should show steps, got " + String(z.squareDb));
            const ZipperResult s = zipperTest(smooth, rate);
            logMessage("smooth: ramp " + String(s.rampDb, 1) + " dB, square " + String(s.squareDb, 1) + " dB");
            expect(s.rampDb < -90.0 && s.squareDb < -90.0,
                   "smooth automation reads " + String(s.rampDb) + " / " + String(s.squareDb));
        }

        beginTest("energyDecay recovers decay times per octave band within 2 %");
        {
            const double centres[] = {125, 250, 500, 1000, 2000, 4000, 8000};
            const double t60[] = {1.5, 1.2, 1.0, 0.8, 0.6, 0.5, 0.4};
            Signal ir(static_cast<size_t>(3.0 * rate), 0.0);
            for (size_t b = 0; b < 7; ++b)
                for (size_t i = 0; i < ir.size(); ++i)
                {
                    const double t = static_cast<double>(i) / rate;
                    ir[i] += std::sin(2.0 * kPi * centres[b] * t + 0.2 * static_cast<double>(b)) *
                             std::exp(-6.907755 * t / t60[b]);
                }
            const auto bands = energyDecay(ir, rate);
            double worst = 0.0, curve = 0.0;
            for (size_t b = 0; b < 7; ++b)
            {
                worst = std::max(worst, std::abs(bands[b].t30S / t60[b] - 1.0));
                worst = std::max(worst, std::abs(bands[b].t20S / t60[b] - 1.0));
                worst = std::max(worst, std::abs(bands[b].edtS / t60[b] - 1.0));
                curve = std::max(curve, std::abs(bands[b].curvaturePct));
            }
            logMessage("decay times, worst error " + String(100.0 * worst, 3) + " %, curvature " + String(curve, 2) + " %");
            expect(worst < 0.02, "decay time error " + String(100.0 * worst) + " %");
            // A noise decay: broadband T30 within 5 % (a statistical estimate).
            Random rng(21);
            Signal nz(static_cast<size_t>(3.0 * rate));
            for (size_t i = 0; i < nz.size(); ++i)
                nz[i] = rng.gaussian() * std::exp(-6.907755 * static_cast<double>(i) / rate / 1.0);
            const double t30 = reverbTimeT30(nz, rate);
            expect(std::abs(t30 - 1.0) < 0.05, "noise decay T30 " + String(t30, 3));
        }

        beginTest("echo density, IACC and tail flatness on constructed responses");
        {
            Random rng(33);
            Signal g(static_cast<size_t>(1.0 * rate));
            for (double& v : g)
                v = rng.gaussian();
            const Signal d = echoDensity(g, rate);
            double mean = 0.0;
            for (size_t i = 4800; i < d.size() - 2400; ++i)
                mean += d[i];
            mean /= static_cast<double>(d.size() - 2400 - 4800);
            expect(std::abs(mean - 1.0) < 0.05, "gaussian echo density " + String(mean, 3));
            expect(echoDensityTime(d, rate) >= 0.0, "density reaches 1");
            Signal sparse(static_cast<size_t>(1.0 * rate), 0.0);
            for (size_t i = 2400; i < sparse.size(); i += 4800)
                sparse[i] = 1.0;
            const Signal ds = echoDensity(sparse, rate);
            double peak = 0.0;
            for (const double v : ds)
                peak = std::max(peak, v);
            expect(peak < 0.05, "sparse impulses density " + String(peak, 3));
            Signal l(static_cast<size_t>(0.2 * rate)), r(l.size(), 0.0), other(l.size());
            for (size_t i = 0; i < l.size(); ++i)
            {
                l[i] = rng.gaussian();
                other[i] = rng.gaussian();
                if (i >= 24)
                    r[i] = l[i - 24];
            }
            expect(iacc(l, l, rate) > 0.999999, "identical channels");
            expect(iacc(l, r, rate) > 0.99, "delayed copy within 1 ms: " + String(iacc(l, r, rate), 4));
            expect(iacc(l, other, rate) < 0.15, "independent channels: " + String(iacc(l, other, rate), 3));
            Signal tail(static_cast<size_t>(2.0 * rate));
            for (double& v : tail)
                v = rng.gaussian();
            const double flat = tailSpectralFlatness(tail, rate, 0.0);
            Signal ringing = tail;
            for (size_t i = 0; i < ringing.size(); ++i)
                ringing[i] += 12.0 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / rate);
            const double rough = tailSpectralFlatness(ringing, rate, 0.0);
            logMessage("tail flatness: white " + String(flat, 2) + " dB, with a resonance " + String(rough, 2) + " dB");
            expect(flat < 1.5, "white tail " + String(flat));
            expect(rough > 3.0, "resonant tail " + String(rough));
        }

        beginTest("modulationDepth recovers a synthetic FM sine within 0.1 cent");
        {
            const double f0 = 1000.0, depth = 25.0, fm = 5.0;
            const size_t n = static_cast<size_t>(4.0 * rate);
            Signal x(n);
            double phase = 0.0;
            for (size_t i = 0; i < n; ++i)
            {
                x[i] = std::sin(phase);
                const double t = static_cast<double>(i) / rate;
                phase += 2.0 * kPi * f0 * std::pow(2.0, depth * std::sin(2.0 * kPi * fm * t) / 1200.0) / rate;
            }
            const ModulationResult m = modulationDepth(x, rate);
            logMessage("FM sine: depth " + String(m.peakCents, 4) + " cent, rate " + String(m.rateHz, 4) + " Hz");
            expectWithinAbsoluteError(m.peakCents, depth, 0.1, "depth");
            expectWithinAbsoluteError(m.rateHz, fm, 0.01, "rate");
        }

        beginTest("wowFlutter recovers a weighted synthetic FM tone within 2 %");
        {
            const double f0 = 3150.0, dev = 0.002;
            for (const double fm : {4.0, 0.5, 20.0})
            {
                const size_t n = static_cast<size_t>(8.0 * rate);
                Signal x(n);
                for (size_t i = 0; i < n; ++i)
                {
                    const double t = static_cast<double>(i) / rate;
                    x[i] = std::sin(2.0 * kPi * f0 * (t - dev * std::cos(2.0 * kPi * fm * t) / (2.0 * kPi * fm)));
                }
                const WowFlutterResult w = wowFlutter(x, rate);
                const double expected = 100.0 * dev * wowFlutterWeight(fm);
                logMessage("fm " + String(fm, 1) + " Hz: weighted peak " + String(w.weightedPeakPct, 4) + " % (expected " +
                           String(expected, 4) + "), unweighted " + String(w.unweightedPeakPct, 4));
                expect(std::abs(w.weightedPeakPct / expected - 1.0) < 0.02, "weighted peak at " + String(fm));
                expect(std::abs(w.unweightedPeakPct / (100.0 * dev) - 1.0) < 0.02, "unweighted peak at " + String(fm));
                expect(std::abs(w.weightedRmsPct / (expected / std::sqrt(2.0)) - 1.0) < 0.02,
                       "weighted rms at " + String(fm));
            }
            expectWithinAbsoluteError(wowFlutterWeight(4.0), 1.0, 1.0e-12, "0 dB at 4 Hz");
        }

        beginTest("measureLoudness follows BS.1770 on constructed signals");
        {
            // The two filter stages at 48 kHz match the coefficients printed in the recommendation.
            auto [shelf, hp] = kWeighting(48000.0);
            expectWithinAbsoluteError(shelf.b0, 1.53512485958697, 1.0e-6, "shelf b0");
            expectWithinAbsoluteError(shelf.b1, -2.69169618940638, 1.0e-6, "shelf b1");
            expectWithinAbsoluteError(shelf.b2, 1.19839281085285, 1.0e-6, "shelf b2");
            expectWithinAbsoluteError(shelf.a1, -1.69065929318241, 1.0e-6, "shelf a1");
            expectWithinAbsoluteError(shelf.a2, 0.73248077421585, 1.0e-6, "shelf a2");
            expectWithinAbsoluteError(hp.a1, -1.99004745483398, 1.0e-6, "hp a1");
            expectWithinAbsoluteError(hp.a2, 0.99007225036621, 1.0e-6, "hp a2");
            const Signal tone = sineWave(static_cast<size_t>(30.0 * rate), 997.0, rate, dbToAmplitude(-23.0));
            const LoudnessResult st = measureLoudness({tone, tone}, rate);
            expectWithinAbsoluteError(st.integratedLufs, -23.0, 0.05, "stereo -23 dBFS tone, integrated");
            expectWithinAbsoluteError(st.momentaryMaxLufs, -23.0, 0.05, "momentary");
            expectWithinAbsoluteError(st.shortTermMaxLufs, -23.0, 0.05, "short term");
            const LoudnessResult mono = measureLoudness({tone}, rate);
            expectWithinAbsoluteError(mono.integratedLufs, -26.01, 0.05, "one channel is 3 LU lower");
            // A quiet passage below the relative gate does not count.
            Signal gated = tone;
            const Signal quiet = sineWave(static_cast<size_t>(20.0 * rate), 997.0, rate, dbToAmplitude(-60.0));
            gated.insert(gated.end(), quiet.begin(), quiet.end());
            expectWithinAbsoluteError(measureLoudness({gated, gated}, rate).integratedLufs, -23.0, 0.05, "relative gate");
            // Two levels, 10 LU apart, equal time: loudness range 10 LU.
            Signal two = sineWave(static_cast<size_t>(30.0 * rate), 997.0, rate, dbToAmplitude(-20.0));
            const Signal low = sineWave(static_cast<size_t>(30.0 * rate), 997.0, rate, dbToAmplitude(-30.0));
            two.insert(two.end(), low.begin(), low.end());
            const LoudnessResult lra = measureLoudness({two, two}, rate);
            logMessage("loudness range of a 10 LU staircase: " + String(lra.rangeLu, 3) + " LU");
            expectWithinAbsoluteError(lra.rangeLu, 10.0, 0.1, "loudness range");
        }

        beginTest("stereoStats separates identical, inverted and independent channels");
        {
            const Signal a = noise(1 << 16, 41), b = noise(1 << 16, 42);
            Signal inv(a.size());
            for (size_t i = 0; i < a.size(); ++i)
                inv[i] = -a[i];
            const StereoStats same = stereoStats(a, a), opposite = stereoStats(a, inv), indep = stereoStats(a, b);
            expectWithinAbsoluteError(same.correlation, 1.0, 1.0e-9, "identical correlation");
            expectWithinAbsoluteError(opposite.correlation, -1.0, 1.0e-9, "inverted correlation");
            expect(std::abs(indep.correlation) < 0.02, "independent correlation " + String(indep.correlation, 4));
            expect(same.sideDb < -250.0 && opposite.midDb < -250.0, "side of identical, mid of inverted");
            expectWithinAbsoluteError(indep.midSideDb, 0.0, 0.1, "independent noise has equal mid and side");
            const std::vector<double> centres = {500.0, 1000.0, 4000.0};
            const Signal ms = monoSumPerBand(a, a, rate, centres), mo = monoSumPerBand(a, inv, rate, centres),
                         mi = monoSumPerBand(a, b, rate, centres);
            for (size_t i = 0; i < centres.size(); ++i)
            {
                expectWithinAbsoluteError(ms[i], 0.0, 0.01, "identical mono sum");
                expect(mo[i] < -120.0, "inverted mono sum " + String(mo[i]));
                expectWithinAbsoluteError(mi[i], -3.0, 0.6, "independent mono sum");
            }
        }
    }
};

static FxMeasureTests fxMeasureTests;

// ---------------------------------------------------------------------------------------------------------------
namespace
{
class ReportingModule final : public crucible::EffectModule, public crucible::LatencyReporter
{
public:
    void prepare(const crucible::ModuleContext&) override {}
    void reset() override {}
    void process(float*, float*, int, const crucible::EffectInputs&) override {}
    int latencySamples() const override { return 37; }
    double tailSeconds() const override { return 2.5; }
    bool takeLatencyChanged() override
    {
        const bool c = changed;
        changed = false;
        return c;
    }
    bool changed = true;
};

// Brute-force extreme of the last w samples ending at i.
float bruteExtreme(const std::vector<float>& x, size_t i, int w, bool max)
{
    const size_t lo = i + 1 >= static_cast<size_t>(w) ? i + 1 - static_cast<size_t>(w) : 0;
    float e = x[lo];
    for (size_t k = lo; k <= i; ++k)
        e = max ? std::max(e, x[k]) : std::min(e, x[k]);
    return e;
}
} // namespace

class FxCoreTests : public UnitTest
{
public:
    FxCoreTests() : UnitTest("FxCore", "Effects") {}

    void runTest() override
    {
        const double rate = 48000.0;

        beginTest("LatencyReporter is found as a capability next to EffectModule");
        {
            ReportingModule m;
            crucible::EffectModule* base = &m;
            auto* r = dynamic_cast<crucible::LatencyReporter*>(base);
            expect(r != nullptr, "capability not found");
            if (r != nullptr)
            {
                expectEquals(r->latencySamples(), 37);
                expectWithinAbsoluteError(r->tailSeconds(), 2.5, 1.0e-12);
                expect(!r->tailInfinite(), "tail finite");
                expect(r->takeLatencyChanged() && !r->takeLatencyChanged(), "change flag is consumed");
            }
        }

        beginTest("smoothing: ramps reach their target exactly and move linearly or geometrically");
        {
            fx::LinearRamp lin;
            lin.reset(1.0f);
            lin.setTarget(3.0f, 100);
            float v = 0.0f;
            for (int i = 0; i < 50; ++i)
                v = lin.next();
            expectWithinAbsoluteError(v, 2.0f, 1.0e-5, "linear midpoint");
            for (int i = 50; i < 100; ++i)
                v = lin.next();
            expectEquals(v, 3.0f);
            expect(!lin.ramping(), "ramp ended");
            fx::LogRamp lg;
            lg.reset(100.0f);
            lg.setTarget(400.0f, 64);
            for (int i = 0; i < 32; ++i)
                v = lg.next();
            expectWithinAbsoluteError(v, 200.0f, 0.01, "an octave in half the time");
            for (int i = 32; i < 64; ++i)
                v = lg.next();
            expectEquals(v, 400.0f);
            fx::OnePole pole;
            pole.setTime(0.010f, rate);
            pole.reset(0.0f);
            int steps = 0;
            while (pole.next(1.0f) < 1.0f - std::exp(-1.0f))
                ++steps;
            expectWithinAbsoluteError(static_cast<double>(steps), 480.0, 4.0, "63 % at one time constant");
            fx::Crossfader cf;
            cf.prepare(rate, 0.020);
            expect(!cf.active() && cf.next() == 1.0f, "idle crossfader");
            expect(cf.start() && !cf.start(), "start is refused while a fade runs");
            float last = 0.0f;
            bool monotone = true;
            int n = 0;
            while (cf.active())
            {
                const float w = cf.next();
                monotone = monotone && w >= last - 1.0e-6f;
                last = w;
                ++n;
            }
            expectEquals(n, 960);
            expect(monotone && last == 1.0f, "weight rises to 1");
        }

        beginTest("channel modes: encode / decode round trip, lanes, link");
        {
            const Signal l0 = noise(4096, 3), r0 = noise(4096, 4);
            std::vector<float> l = toFloat(l0), r = toFloat(r0);
            for (const auto mode : {fx::ChannelMode::Mid, fx::ChannelMode::Side, fx::ChannelMode::MidSide})
            {
                fx::encode(mode, l.data(), r.data(), 4096);
                fx::decode(mode, l.data(), r.data(), 4096);
            }
            expect(nullTest(l0, fromFloat(l), 0.0) < -130.0, "round trip " + String(nullTest(l0, fromFloat(l), 0.0)));
            expect(nullTest(r0, fromFloat(r), 0.0) < -130.0, "round trip right");
            // A gain on the mid lane of a mono signal is exactly that gain; the side lane stays untouched.
            float a = 0.4f, b = 0.4f;
            fx::encode(fx::ChannelMode::Mid, a, b);
            expectEquals(b, 0.0f);
            a *= 0.5f;
            fx::decode(fx::ChannelMode::Mid, a, b);
            expectEquals(a, 0.2f);
            expectEquals(b, 0.2f);
            expect(fx::laneActive(fx::ChannelMode::Mid, 0) && !fx::laneActive(fx::ChannelMode::Mid, 1), "mid lane");
            expect(!fx::laneActive(fx::ChannelMode::Side, 0) && fx::laneActive(fx::ChannelMode::Side, 1), "side lane");
            expect(fx::laneActive(fx::ChannelMode::MidSide, 0) && fx::laneActive(fx::ChannelMode::MidSide, 1), "dual");
            expect(!fx::laneActive(fx::ChannelMode::Left, 1) && !fx::laneActive(fx::ChannelMode::Right, 0), "left / right");
            expectEquals(fx::effectiveLink(fx::ChannelMode::StereoLinked, 0.2f), 1.0f);
            expectEquals(fx::effectiveLink(fx::ChannelMode::StereoUnlinked, 0.9f), 0.0f);
            expectEquals(fx::effectiveLink(fx::ChannelMode::MidSide, 0.5f), 0.5f);
            float p = 2.0f, q = 6.0f;
            fx::linkLevels(p, q, 0.5f);
            expectEquals(p, 4.0f);
            expectEquals(q, 6.0f);
            fx::linkLevels(p, q, 1.0f);
            expectEquals(p, 6.0f);
        }

        beginTest("true peak reads fs/4 sines at every phase within 0.1 dB");
        {
            for (const int factor : {4, 8})
            {
                fx::TruePeakDetector tp;
                tp.configure(factor, true);
                double worst = 0.0, worstSamplePeak = 0.0;
                for (int k = 0; k < 90; ++k)
                {
                    const double phase = kPi * k / 90.0;
                    tp.reset();
                    float peak = 0.0f, sample = 0.0f;
                    for (int i = 0; i < 400; ++i)
                    {
                        const float x = static_cast<float>(std::sin(kPi / 2.0 * i + phase));
                        sample = std::max(sample, std::abs(x));
                        const float p = tp.process(x);
                        if (i >= 40)
                            peak = std::max(peak, p);
                    }
                    worst = std::max(worst, std::abs(amplitudeToDb(peak)));
                    worstSamplePeak = std::min(worstSamplePeak, amplitudeToDb(sample));
                }
                logMessage(String(factor) + "x with refinement: worst error " + String(worst, 4) +
                           " dB (sample peak reads down to " + String(worstSamplePeak, 2) + " dB)");
                expect(worst < 0.1, String(factor) + "x true-peak error " + String(worst));
                expect(worstSamplePeak < -2.5, "sample peak shows the problem");
            }
            fx::TruePeakDetector raw;
            raw.configure(4, false);
            double worst = 0.0;
            for (int k = 0; k < 90; ++k)
            {
                raw.reset();
                float peak = 0.0f;
                for (int i = 0; i < 400; ++i)
                {
                    const float p = raw.process(static_cast<float>(std::sin(kPi / 2.0 * i + kPi * k / 90.0)));
                    if (i >= 40)
                        peak = std::max(peak, p);
                }
                worst = std::max(worst, std::abs(amplitudeToDb(peak)));
            }
            logMessage("4x without refinement: worst error " + String(worst, 3) + " dB");
            expect(worst < 0.25, "4x grid error");
            // Against the harness's 32-tap reference on a signal with content to 0.35 fs.
            fx::TruePeakDetector tp;
            tp.configure(4, true);
            Signal x(8192, 0.0);
            Random rng(5);
            for (const double f : {0.11, 0.23, 0.31, 0.34})
            {
                const double ph = 6.28 * rng.uniform();
                for (size_t i = 0; i < x.size(); ++i)
                    x[i] += 0.25 * std::sin(2.0 * kPi * f * static_cast<double>(i) + ph);
            }
            float peak = 0.0f;
            for (size_t i = 0; i < x.size(); ++i)
            {
                const float p = tp.process(static_cast<float>(x[i]));
                if (i >= 64)
                    peak = std::max(peak, p);
            }
            const double ref = truePeak(x.data(), x.size(), 8, 64);
            logMessage("broadband: detector " + String(amplitudeToDb(peak), 3) + " dB, reference " +
                       String(amplitudeToDb(ref), 3) + " dB");
            expect(std::abs(amplitudeToDb(peak) - amplitudeToDb(ref)) < 0.1, "agrees with the harness true peak");
            // An impulse produces its full value exactly latencySamples() calls later (the table's phase 0 is the identity).
            tp.reset();
            int at = -1;
            for (int i = 0; i < 40; ++i)
                if (tp.process(i == 0 ? 1.0f : 0.0f) > 0.999f)
                    at = i;
            expectEquals(at, tp.latencySamples());
        }

        beginTest("running min / max equal brute force, also when the window shrinks");
        {
            Random rng(77);
            std::vector<float> x(30000);
            for (auto& v : x)
                v = static_cast<float>(rng.gaussian());
            for (const int w : {1, 2, 3, 7, 64, 480, 1500})
            {
                fx::RunningMax mx;
                fx::RunningMin mn;
                mx.prepare(2000);
                mn.prepare(2000);
                mx.setWindow(w);
                mn.setWindow(w);
                int bad = 0;
                for (size_t i = 0; i < x.size(); ++i)
                {
                    bad += mx.push(x[i]) != bruteExtreme(x, i, w, true);
                    bad += mn.push(x[i]) != bruteExtreme(x, i, w, false);
                }
                expectEquals(bad, 0, "window " + String(w));
            }
            fx::RunningMax mx;
            mx.prepare(2000);
            mx.setWindow(1500);
            int bad = 0, w = 1500;
            for (size_t i = 0; i < x.size(); ++i)
            {
                if (i == 10000)
                {
                    w = 100;
                    mx.setWindow(w);
                }
                bad += mx.push(x[i]) != bruteExtreme(x, i, w, true);
            }
            expectEquals(bad, 0, "shrinking window");
            // Monotone and constant inputs (the deque's extreme cases).
            for (int mode = 0; mode < 3; ++mode)
            {
                std::vector<float> y(5000);
                for (size_t i = 0; i < y.size(); ++i)
                    y[i] = mode == 0 ? static_cast<float>(i) : (mode == 1 ? static_cast<float>(5000 - i) : 0.25f);
                fx::RunningMax m2;
                m2.prepare(300);
                m2.setWindow(300);
                int b2 = 0;
                for (size_t i = 0; i < y.size(); ++i)
                    b2 += m2.push(y[i]) != bruteExtreme(y, i, 300, true);
                expectEquals(b2, 0, "monotone mode " + String(mode));
            }
        }

        beginTest("box sum and RMS detectors");
        {
            fx::BoxSum box;
            box.prepare(1000);
            box.setWindow(480);
            Random rng(8);
            std::vector<double> v(20000);
            double worst = 0.0;
            for (size_t i = 0; i < v.size(); ++i)
            {
                v[i] = rng.gaussian();
                const double s = box.push(v[i]);
                double brute = 0.0;
                for (size_t k = i + 1 >= 480 ? i + 1 - 480 : 0; k <= i; ++k)
                    brute += v[k];
                worst = std::max(worst, std::abs(s - brute));
            }
            expect(worst < 1.0e-9, "box sum error " + String(worst));
            box.reset();
            for (int i = 0; i < 2000; ++i)
                box.push(i < 700 ? 1000.0 : 0.0);
            expectEquals(box.sum(), 0.0);
            fx::RmsDetector rms;
            rms.setTimeConstant(0.010f, rate);
            rms.reset();
            int steps = 0;
            // Power of a unit step reaches 63.2 % after one time constant, so the reading reaches sqrt(0.632).
            while (rms.process(1.0f) < std::sqrt(1.0 - std::exp(-1.0)))
                ++steps;
            expectWithinAbsoluteError(static_cast<double>(steps), 480.0, 4.0, "RMS time constant");
            for (int i = 0; i < 20000; ++i)
                rms.process(0.5f * static_cast<float>(std::sin(2.0 * kPi * 1000.0 * i / rate)));
            float r = 0.0f;
            fx::RmsDetector settle;
            settle.setTimeConstant(0.050f, rate);
            for (int i = 0; i < 48000; ++i)
                r = settle.process(0.5f * static_cast<float>(std::sin(2.0 * kPi * 1000.0 * i / rate)));
            expectWithinAbsoluteError(amplitudeToDb(r), amplitudeToDb(0.5 / std::sqrt(2.0)), 0.02, "sine reads A / sqrt 2");
            fx::SlidingRms sliding;
            sliding.prepare(960);
            sliding.setWindow(480);
            for (int i = 0; i < 4800; ++i)
                r = sliding.process(0.5f * static_cast<float>(std::sin(2.0 * kPi * 1000.0 * i / rate)));
            expectWithinAbsoluteError(amplitudeToDb(r), amplitudeToDb(0.5 / std::sqrt(2.0)), 0.02, "sliding RMS of a sine");
        }

        beginTest("lookahead delays by its length and changes it without a step above -90 dB");
        {
            fx::Lookahead la;
            la.prepare(2, 2048, rate);
            la.setDelay(100);
            std::vector<float> a(400, 0.0f), b(400, 0.0f);
            a[0] = 1.0f;
            float* io[2] = {a.data(), b.data()};
            la.process(io, 400);
            expectEquals(a[100], 1.0f);
            expectEquals(la.delay(), 100);
            const double hz = 1000.0;
            const size_t total = 60000;
            const std::vector<std::pair<size_t, int>> changes = {{12000, 300}, {30000, 50}, {30200, 700}, {30400, 120}};
            fx::Lookahead cl;
            cl.prepare(1, 2048, rate);
            cl.setDelay(100);
            std::vector<float> y(total);
            for (size_t i = 0; i < total; ++i)
            {
                for (const auto& c : changes)
                    if (i == c.first)
                        cl.setDelay(c.second);
                y[i] = cl.processSample(0.5f * static_cast<float>(std::sin(2.0 * kPi * hz * static_cast<double>(i) / rate)));
            }
            const ClickResult c = clickTest(fromFloat(y), hz, rate, 3000);
            logMessage("lookahead delay changes: largest step " + String(c.stepDb, 1) + " dB");
            expect(c.stepDb < -90.0, "step " + String(c.stepDb) + " dB at " + String(static_cast<double>(c.at)));
            expectEquals(cl.delay(), 120);
            expect(!cl.fading() && cl.activeDelay() == 120, "queued changes settle on the last request");
            // A change before the first sample applies at once.
            fx::Lookahead early;
            early.prepare(1, 512, rate);
            early.setDelay(40);
            expect(!early.fading() && early.activeDelay() == 40, "immediate before start");
            // The legacy approach (clearing the buffer) produces a large step; the harness sees it.
            std::vector<float> legacy(total);
            std::deque<float> line(100, 0.0f);
            for (size_t i = 0; i < total; ++i)
            {
                line.push_back(0.5f * static_cast<float>(std::sin(2.0 * kPi * hz * static_cast<double>(i) / rate)));
                if (i == 12000)
                {
                    line.assign(300, 0.0f);
                    line.push_back(0.0f);
                }
                legacy[i] = line.front();
                line.pop_front();
            }
            expect(clickTest(fromFloat(legacy), hz, rate, 3000).stepDb > -40.0, "a cleared buffer shows as a large step");
        }

        beginTest("oversampled block: reported latency, null at mix 0, matched dry path, block-size independence");
        {
            for (const auto family : {crucible::OversamplingFilter::HalfBand, crucible::OversamplingFilter::Polyphase})
                for (const int stages : {1, 2})
                {
                    const String tag = String(family == crucible::OversamplingFilter::HalfBand ? "FIR " : "IIR ") +
                                       String(1 << stages) + "x";
                    fx::OversampledBlock ob;
                    ob.prepare(2, stages, family, crucible::Quality::Standard, 512, fx::DryPath::Delay);
                    auto identity = [](float* const*, int, int) {};
                    const Signal in = noise(16384, 99, 0.25);
                    // Latency: mix 1, identity kernel.
                    {
                        std::vector<float> l = toFloat(in), r = l;
                        float* io[2] = {l.data(), r.data()};
                        for (size_t pos = 0; pos < l.size(); pos += 512)
                        {
                            float* blk[2] = {io[0] + pos, io[1] + pos};
                            ob.process(blk, 512, 1.0f, identity);
                        }
                        const double measured = measureLatency(in, fromFloat(l));
                        logMessage(tag + ": reported " + String(ob.latencySamples()) + ", measured " + String(measured, 3) +
                                   ", filter " + String(ob.latency(), 3));
                        expect(std::abs(measured - ob.latencySamples()) <= 1.0, tag + " latency " + String(measured));
                        expect(std::abs(ob.latency() - measured) <= 1.0, tag + " filter latency");
                    }
                    // Mix 0, Delay path: the input delayed by latencySamples() exactly.
                    {
                        ob.reset();
                        std::vector<float> l = toFloat(in), r = l;
                        for (size_t pos = 0; pos < l.size(); pos += 512)
                        {
                            float* blk[2] = {l.data() + pos, r.data() + pos};
                            ob.process(blk, 512, 0.0f, identity);
                        }
                        const double null = nullTest(in, fromFloat(l), ob.latencySamples());
                        logMessage(tag + " mix 0 null: " + String(null, 1) + " dB");
                        expect(null < -120.0, tag + " mix 0 null " + String(null));
                    }
                    // Matched path: with a linear kernel every mix setting gives the same output.
                    {
                        fx::OversampledBlock mb;
                        mb.prepare(2, stages, family, crucible::Quality::Standard, 512, fx::DryPath::Matched);
                        std::vector<float> one = toFloat(in), half = one, zero = one, o2 = one, h2 = one, z2 = one;
                        auto run = [&](std::vector<float>& l, std::vector<float>& r, float mix)
                        {
                            mb.reset();
                            for (size_t pos = 0; pos < l.size(); pos += 512)
                            {
                                float* blk[2] = {l.data() + pos, r.data() + pos};
                                mb.process(blk, 512, mix, identity);
                            }
                        };
                        run(one, o2, 1.0f);
                        run(half, h2, 0.5f);
                        run(zero, z2, 0.0f);
                        const double d = std::max(nullTest(fromFloat(one), fromFloat(half), 0.0),
                                                  nullTest(fromFloat(one), fromFloat(zero), 0.0));
                        logMessage(tag + " matched path, mix 0 / 0.5 against mix 1: " + String(d, 1) + " dB");
                        expect(d < -120.0, tag + " matched path combs: " + String(d));
                    }
                    // Block size does not change the result.
                    {
                        auto clip = [](float* const* x, int ch, int n)
                        {
                            for (int c = 0; c < ch; ++c)
                                for (int i = 0; i < n; ++i)
                                    x[c][i] = std::tanh(2.0f * x[c][i]);
                        };
                        fx::OversampledBlock p, q;
                        p.prepare(2, stages, family, crucible::Quality::Standard, 512, fx::DryPath::Matched);
                        q.prepare(2, stages, family, crucible::Quality::Standard, 512, fx::DryPath::Matched);
                        std::vector<float> a1 = toFloat(in), a2 = a1, b1 = a1, b2 = a1;
                        float* ioA[2] = {a1.data(), a2.data()};
                        p.process(ioA, 6000, 0.7f, clip);
                        size_t pos = 0;
                        while (pos < 6000)
                        {
                            const int len = static_cast<int>(std::min<size_t>(37, 6000 - pos));
                            float* blk[2] = {b1.data() + pos, b2.data() + pos};
                            q.process(blk, len, 0.7f, clip);
                            pos += static_cast<size_t>(len);
                        }
                        int diff = 0;
                        for (size_t i = 0; i < 6000; ++i)
                            diff += std::abs(a1[i] - b1[i]) > 1.0e-6f;
                        expectEquals(diff, 0, tag + " block size independence");
                    }
                }
            fx::OversampledBlock one;
            one.prepare(1, 0, crucible::OversamplingFilter::HalfBand, crucible::Quality::Standard, 256);
            std::vector<float> x(100, 0.5f);
            float* io[1] = {x.data()};
            int seen = 0;
            one.process(io, 100, 1.0f, [&](float* const*, int, int count) { seen = count; });
            expectEquals(seen, 100);
            expectEquals(one.latencySamples(), 0);
        }
    }
};

static FxCoreTests fxCoreTests;
