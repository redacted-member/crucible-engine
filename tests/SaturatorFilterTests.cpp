// SPDX-License-Identifier: MIT
// The resonant filter of fx.saturator (mode Off / Pre / Post, type, cutoff, resonance), measured on rendered output:
// Off is bit-identical, a post filter follows the analytic state-variable response within 0.5 dB, resonance raises the
// peak monotonically, a pre low-pass reduces the distortion the shaper generates, cutoff and resonance moves are
// glitch-free and settle, the output does not depend on the host block size, and the CPU cost is stated.

#include "AllocCounter.h"
#include "AnalogMeasure.h"
#include "TestSupport.h"

#include <crucible/Fx.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <vector>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_SANITIZED 1
#endif
#endif
#ifndef CRUCIBLE_TEST_SANITIZED
#define CRUCIBLE_TEST_SANITIZED 0
#endif

using namespace crucible;
using namespace crucible::measure;
using crucible::test::UnitTest;
using crucible::testing::kRate;

namespace
{
constexpr double kPiT = 3.14159265358979323846;

struct SatRack
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values, legacy;
    SatRack()
    {
        ModuleContext ctx;
        ctx.sampleRate = kRate;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set("fx_saturator_on", 1.0f);
        set("fx_saturator_mix", 1.0f);
        set("fx_saturator_model", 0.0f);
        set("fx_saturator_drive", 3.0f);
        set("fx_saturator_autogain", 0.0f);
    }
    void set(const std::string& id, float v) { values[static_cast<size_t>(ParameterMap::indexOf(id))] = v; }
    void filter(int mode, int type, float hz, float res)
    {
        set("fx_saturator_filter_mode", static_cast<float>(mode));
        set("fx_saturator_filter_type", static_cast<float>(type));
        set("fx_saturator_filter_cutoff", hz);
        set("fx_saturator_filter_res", res);
    }
    void run(float* l, float* r, int n) { rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0); }
    std::vector<float> render(const std::vector<float>& in, int block = 64)
    {
        std::vector<float> l = in, r = in;
        for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
            run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos)));
        return l;
    }
};

std::vector<float> sine(size_t n, double hz, double amp)
{
    std::vector<float> x(n);
    for (size_t i = 0; i < n; ++i)
        x[i] = static_cast<float>(amp * std::sin(2.0 * kPiT * hz * static_cast<double>(i) / kRate));
    return x;
}

// Amplitude of the tone at hz in the last count samples (windowed DTFT).
double toneAmp(const std::vector<float>& x, double hz, size_t count = 16384)
{
    const Signal d = toDouble(x, x.size() - count, count);
    const Signal w = makeWindow(Window::BlackmanHarris, count);
    double sw = 0.0;
    for (const double v : w)
        sw += v;
    return 2.0 * std::abs(windowedDtft(d.data(), w.data(), count, hz, kRate)) / sw;
}

// The ideal state-variable response (bilinear: s = j tan(pi f / fs) / tan(pi fc / fs)), damping 1 / Q with Q from
// 0.707 to 21 over the resonance control; band-pass peak normalised to unity.
double referenceDb(int type, double hz, double fc, double res)
{
    const double k = 1.0 / (0.70710678 * std::pow(30.0, res));
    const std::complex<double> s(0.0, std::tan(kPiT * hz / kRate) / std::tan(kPiT * fc / kRate));
    const std::complex<double> den = s * s + k * s + 1.0;
    std::complex<double> h;
    if (type == 0)
        h = 1.0 / den;
    else if (type == 1)
        h = k * s / den;
    else
        h = s * s / den;
    return 20.0 * std::log10(std::abs(h));
}

// Gain of the filter at hz: the filtered render against the same render with the filter off (a 3 dB soft clip of a
// 0.01 sine is linear to well under 0.01 dB), so the oversampler's and the shaper's own response cancels.
double measuredDb(int mode, int type, double hz, double fc, double res)
{
    const auto in = sine(static_cast<size_t>(kRate * 0.6), hz, 0.01);
    SatRack off, on;
    on.filter(mode, type, static_cast<float>(fc), static_cast<float>(res));
    return 20.0 * std::log10(toneAmp(on.render(in), hz) / toneAmp(off.render(in), hz));
}

double bandPower(const std::vector<float>& x, double lo, double hi)
{
    const size_t n = 32768;
    const Spectrum s = computeSpectrum(toDouble(x, x.size() - n, n).data(), n, kRate, Window::BlackmanHarris);
    double p = 0.0;
    for (size_t k = static_cast<size_t>(lo / s.binHz); k < static_cast<size_t>(hi / s.binHz); ++k)
        p += s.power[k];
    return p;
}

} // namespace

class SaturatorFilterTests : public UnitTest
{
public:
    SaturatorFilterTests() : UnitTest("Saturator filter", "Effects") {}

    void runTest() override
    {
        beginTest("parameters: four appended ones on their own page, off by default");
        {
            for (const char* id : {"fx_saturator_filter_mode", "fx_saturator_filter_type", "fx_saturator_filter_cutoff",
                                   "fx_saturator_filter_res"})
                expect(ParameterMap::indexOf(id) >= 0, std::string("parameter ") + id);
            const auto def = ParameterMap::defaultValues();
            expectEquals(def[static_cast<size_t>(ParameterMap::indexOf("fx_saturator_filter_mode"))], 0.0f, "mode defaults to Off");
        }

        beginTest("at 0 dB drive a Pre or Post filter is heard: the output equals the state-variable reference");
        {
            std::vector<float> in = sine(static_cast<size_t>(kRate * 0.4), 440.0, 0.3);
            for (size_t i = 0; i < in.size(); ++i)
                in[i] += 0.2f * static_cast<float>(std::sin(2.1 * static_cast<double>(i) * 0.01 * static_cast<double>(i) * 0.002));
            for (const int mode : {1, 2})
            {
                SatRack s;
                s.set("fx_saturator_drive", 0.0f);
                s.filter(mode, 0, 1000.0f, 0.0f);
                const auto* lr = s.rack.findCapability<LatencyReporter>("fx.saturator");
                const size_t lat = static_cast<size_t>(lr->latencySamples());
                const auto out = s.render(in);
                // Reference: the input delayed by the latency through a double precision trapezoidal SVF (Q = 0.7071).
                const double g = std::tan(kPiT * 1000.0 / kRate), k = 1.0 / 0.70710678;
                const double a1 = 1.0 / (1.0 + g * (g + k)), a2 = g * a1, a3 = g * a2;
                double s1 = 0.0, s2 = 0.0, worst = 0.0, peak = 0.0;
                for (size_t i = 0; i < in.size(); ++i)
                {
                    const double x = i >= lat ? static_cast<double>(in[i - lat]) : 0.0;
                    const double v3 = x - s2, v1 = a1 * s1 + a2 * v3, v2 = s2 + a2 * s1 + a3 * v3;
                    s1 = 2.0 * v1 - s1;
                    s2 = 2.0 * v2 - s2;
                    worst = std::max(worst, std::abs(v2 - static_cast<double>(out[i])));
                    peak = std::max(peak, std::abs(v2));
                }
                expect(peak > 0.1 && worst < 1.0e-4, "mode " + crucible::test::String(mode) + ": worst error " + crucible::test::String(worst));
                // And it is audibly a filter: the 440 Hz tone passes, a 8 kHz tone is attenuated by well over 20 dB.
                SatRack hi;
                hi.set("fx_saturator_drive", 0.0f);
                hi.filter(mode, 0, 1000.0f, 0.0f);
                const auto top = hi.render(sine(static_cast<size_t>(kRate * 0.6), 8000.0, 0.1));
                expect(toneAmp(top, 8000.0) < 0.1 * 0.1, "8 kHz attenuated, mode " + crucible::test::String(mode));
            }
        }

        beginTest("Off is bit-identical whatever the other filter settings are");
        {
            SatRack a, b;
            b.filter(0, 2, 333.0f, 0.9f);
            std::vector<float> in = sine(static_cast<size_t>(kRate * 0.5), 440.0, 0.5);
            for (size_t i = 0; i < in.size(); ++i)
                in[i] += 0.2f * static_cast<float>(std::sin(0.37 * static_cast<double>(i)));
            a.set("fx_saturator_drive", 18.0f);
            b.set("fx_saturator_drive", 18.0f);
            const auto x = a.render(in), y = b.render(in);
            expect(std::memcmp(x.data(), y.data(), x.size() * sizeof(float)) == 0, "outputs identical");
        }

        beginTest("a post filter follows the state-variable reference within 0.5 dB (LP, BP, HP; three resonances)");
        {
            const char* names[3] = {"LP", "BP", "HP"};
            double worst = 0.0;
            for (int type = 0; type < 3; ++type)
                for (const double res : {0.0, 0.5, 1.0})
                    for (const double fc : {500.0, 4000.0})
                        for (const double f : {80.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0, 15000.0})
                        {
                            const double ref = referenceDb(type, f, fc, res);
                            const double got = measuredDb(2, type, f, fc, res);
                            if (ref > -60.0)
                                worst = std::max(worst, std::abs(got - ref));
                            if (ref > -60.0 && std::abs(got - ref) > 0.5)
                                std::printf("  %s fc %.0f res %.1f at %.0f Hz: ref %.2f dB, measured %.2f dB\n", names[type], fc,
                                            res, f, ref, got);
                            if (ref > -60.0)
                                expectWithinAbsoluteError(got, ref, 0.5, "filter response");
                        }
            std::printf("  saturator filter: worst deviation from the reference %.3f dB\n", worst);
        }

        beginTest("a pre filter has the same response on a near-linear shaper");
        {
            for (const double f : {250.0, 1000.0, 3000.0, 9000.0})
                expectWithinAbsoluteError(measuredDb(1, 0, f, 2000.0, 0.6), referenceDb(0, f, 2000.0, 0.6), 0.5, "pre LP");
        }

        beginTest("resonance raises the peak monotonically, the band-pass peak stays at unity");
        {
            double prev = -100.0;
            for (const double res : {0.0, 0.2, 0.4, 0.6, 0.8, 1.0})
            {
                const double peak = measuredDb(2, 0, 2000.0, 2000.0, res);
                std::printf("  saturator filter: LP resonance %.1f: peak %.2f dB (reference %.2f)\n", res, peak,
                            referenceDb(0, 2000.0, 2000.0, res));
                expect(peak > prev + 0.5, "peak rises with resonance");
                prev = peak;
            }
            expectWithinAbsoluteError(measuredDb(2, 1, 2000.0, 2000.0, 1.0), 0.0, 0.5, "band-pass peak at unity");
        }

        beginTest("a pre low-pass reduces the distortion the shaper generates");
        {
            std::vector<float> in(static_cast<size_t>(kRate * 0.8));
            for (size_t i = 0; i < in.size(); ++i)
                in[i] = static_cast<float>(0.15 * std::sin(2.0 * kPiT * 2000.0 * static_cast<double>(i) / kRate) +
                                           0.15 * std::sin(2.0 * kPiT * 9000.0 * static_cast<double>(i) / kRate));
            SatRack off, pre, post;
            for (auto* h : {&off, &pre, &post})
                h->set("fx_saturator_drive", 12.0f);
            pre.filter(1, 0, 1500.0f, 0.0f);
            post.filter(2, 0, 1500.0f, 0.0f);
            const auto o = off.render(in), p = pre.render(in), q = post.render(in);
            const double offHi = bandPower(o, 10000.0, 20000.0), preHi = bandPower(p, 10000.0, 20000.0),
                         postHi = bandPower(q, 10000.0, 20000.0);
            std::printf("  saturator filter: power above 10 kHz, off %.1f dB, pre LP 1.5 kHz %.1f dB (%.1f dB lower), post LP %.1f dB\n",
                        10.0 * std::log10(offHi), 10.0 * std::log10(preHi), 10.0 * std::log10(offHi / preHi),
                        10.0 * std::log10(postHi));
            expect(preHi < offHi * 0.1, "pre low-pass cuts the upper products by more than 10 dB");
            // The two positions are different effects: pre feeds a quieter, duller signal to the shaper.
            double diff = 0.0, ref = 0.0;
            for (size_t i = in.size() / 2; i < in.size(); ++i)
            {
                diff += (p[i] - q[i]) * static_cast<double>(p[i] - q[i]);
                ref += static_cast<double>(q[i]) * q[i];
            }
            expect(diff > 0.01 * ref, "pre and post do not render the same");
        }

        beginTest("cutoff and resonance moves follow a 128-sample glide, settle exactly, and are smoother than a hard switch");
        {
            // Reference: the same state-variable filter in double on the filter-off render, with the cutoff gliding
            // geometrically and the resonance linearly over the grid chunk (128 samples) after the change, or jumping.
            const size_t n = static_cast<size_t>(kRate * 1.0), at = 128 * 190;
            const double fc0 = 600.0, fc1 = 8000.0, r0 = 0.1, r1 = 0.8;
            const auto in = sine(n, 500.0, 0.01);
            SatRack off, h;
            h.filter(2, 0, static_cast<float>(fc0), static_cast<float>(r0));
            std::vector<float> l = in, r = in;
            h.run(l.data(), r.data(), static_cast<int>(at));
            h.filter(2, 0, static_cast<float>(fc1), static_cast<float>(r1));
            h.run(l.data() + at, r.data() + at, static_cast<int>(n - at));
            const auto plain = off.render(in);
            const auto reference = [&](bool glide)
            {
                std::vector<double> y(n);
                double s1 = 0.0, s2 = 0.0;
                for (size_t i = 0; i < n; ++i)
                {
                    double fc = fc0, res = r0;
                    if (i >= at)
                    {
                        const double t = std::min(1.0, static_cast<double>(i - at + 1) / 128.0);
                        const double w = glide ? t : 1.0;
                        fc = fc0 * std::pow(fc1 / fc0, w);
                        res = r0 + (r1 - r0) * w;
                    }
                    const double g = std::tan(kPiT * fc / kRate), k = 1.0 / (0.70710678 * std::pow(30.0, res));
                    const double a1 = 1.0 / (1.0 + g * (g + k)), a2 = g * a1, a3 = g * a2;
                    const double v3 = plain[i] - s2, v1 = a1 * s1 + a2 * v3, v2 = s2 + a2 * s1 + a3 * v3;
                    s1 = 2.0 * v1 - s1;
                    s2 = 2.0 * v2 - s2;
                    y[i] = v2;
                }
                return y;
            };
            const auto glided = reference(true), jumped = reference(false);
            double worst = 0.0;
            for (size_t i = 0; i < n; ++i)
                worst = std::max(worst, std::abs(static_cast<double>(l[i]) - glided[i]));
            const auto third = [&](const std::vector<double>& y)
            {
                double m = 0.0;
                for (size_t i = at; i < at + 600; ++i)
                    m = std::max(m, std::abs(y[i] - 3.0 * y[i - 1] + 3.0 * y[i - 2] - y[i - 3]));
                return m;
            };
            const double tGlide = third(glided), tJump = third(jumped);
            std::printf("  saturator filter: module vs glide reference max %.2e; max third difference, glide %.2e, hard switch %.2e\n",
                        worst, tGlide, tJump);
            expect(worst < 2.0e-6, "the module follows the specified glide (float precision)");
            expect(tGlide < 0.7 * tJump, "the glide is smoother than a hard switch");
            // Settled: the final stretch is the new setting's steady state, to 0.001 dB.
            SatRack fixed;
            fixed.filter(2, 0, static_cast<float>(fc1), static_cast<float>(r1));
            const auto steady = fixed.render(in);
            expectWithinAbsoluteError(20.0 * std::log10(toneAmp(l, 500.0, 8192) / toneAmp(steady, 500.0, 8192)), 0.0, 0.001,
                                      "settled level");
        }

        beginTest("the output does not depend on the host block size, silence stays silent, no allocations");
        {
            for (const int mode : {0, 1, 2})
            {
                SatRack a, b;
                a.filter(mode, 1, 1500.0f, 0.7f);
                b.filter(mode, 1, 1500.0f, 0.7f);
                std::vector<float> in = sine(20000, 700.0, 0.4);
                const auto x = a.render(in, 64), y = b.render(in, 333);
                double worst = 0.0;
                for (size_t i = 0; i < x.size(); ++i)
                    worst = std::max(worst, std::abs(static_cast<double>(x[i]) - y[i]));
                std::printf("  mode %d: block 64 vs 333 max difference %g (the rack's own block handling gives 1e-7 with the filter off)\n", mode, worst);
                expect(worst < 1.0e-6, "block sizes agree");
                SatRack z;
                z.filter(mode, 0, 800.0f, 1.0f);
                const auto s = z.render(std::vector<float>(4096, 0.0f));
                float peak = 0.0f;
                for (const float v : s)
                    peak = std::max(peak, std::abs(v));
                expect(peak < 1.0e-12f, "silence in, silence out (the saturator's own residue is 1e-16)");
                std::vector<float> l(64, 0.1f), r(64, 0.1f);
                crucible::testing::gTrackAllocations = true;
                crucible::testing::gAllocations = 0;
                const size_t cutIdx = static_cast<size_t>(ParameterMap::indexOf("fx_saturator_filter_cutoff"));
                for (int i = 0; i < 100; ++i)
                {
                    a.values[cutIdx] = 200.0f + 90.0f * static_cast<float>(i);
                    a.run(l.data(), r.data(), 64);
                }
                crucible::testing::gTrackAllocations = false;
                expectEquals(crucible::testing::gAllocations, 0L, "allocations on the audio thread");
            }
        }

        beginTest("CPU: the filter costs a small share on top of the saturator");
        {
            const auto in = sine(static_cast<size_t>(kRate * 4.0), 440.0, 0.4);
            double best[3] = {1.0e9, 1.0e9, 1.0e9};
            for (int mode = 0; mode < 3; ++mode)
                for (int rep = 0; rep < 3; ++rep)
                {
                    SatRack h;
                    h.set("fx_saturator_drive", 12.0f);
                    h.filter(mode, 0, 1500.0f, 0.5f);
                    const auto start = std::chrono::steady_clock::now();
                    h.render(in, 256);
                    best[mode] = std::min(best[mode], 100.0 * std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() / 4.0);
                }
            std::printf("  saturator filter CPU (one core, %% of real time, 48 kHz stereo): off %.2f %%, pre %.2f %%, post %.2f %%\n",
                        best[0], best[1], best[2]);
            if (!CRUCIBLE_TEST_SANITIZED)
                expectCpu(best[1] < best[0] + 1.0 && best[2] < best[0] + 1.0, "filter adds under one percent of a core");
        }
    }
};

static SaturatorFilterTests saturatorFilterTests;
