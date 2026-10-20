// SPDX-License-Identifier: MIT
// Regression tests of the effect fixes BFX2-BFX7 (BFX1, the drive DC blocker, went with the first-generation drive) (docs/plans/ENGINE-FX-PLAN.md, section 2.3 and package FX2).
// Each test measures the symptom the bug produced; the numbers before the fix are in docs/SOUND_CHANGES.md.

#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Fx.h>
#include <crucible/FxExtra.h>
#include <crucible/module/ModuleRack.h>

#include <cmath>
#include <utility>
#include <vector>

using namespace crucible;
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;
using crucible::testing::kRate;

namespace
{
// Runs a stereo effect on a mono signal (the same signal on both channels) in blocks of 32; returns the left channel.
template <class FxType, class Params> Signal runFx(FxType& fx, const Params& p, const Signal& in, int block = 32)
{
    std::vector<float> l(in.begin(), in.end()), r = l;
    for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min<size_t>(static_cast<size_t>(block), l.size() - pos));
        fx.process(l.data() + pos, r.data() + pos, n, p);
    }
    return Signal(l.begin(), l.end());
}

double peakOf(const Signal& x)
{
    double m = 0.0;
    for (const double v : x)
        m = std::max(m, std::abs(v));
    return m;
}

Signal noiseBurst(double seconds, double rate, std::uint64_t seed = 3)
{
    Random rng(seed);
    Signal x(static_cast<size_t>(seconds * rate));
    for (double& v : x)
        v = 0.3 * rng.gaussian();
    return x;
}

// Worst deviation (dB) of the multiband's output level from the input level over 50 Hz - 12 kHz, dynamics neutral.
double bandSumWorstDb(float low, float high)
{
    auto render = [&](const Signal& x)
    {
        FxMultiband m;
        m.prepare(kRate);
        FxMultibandParams p;
        p.on = true;
        p.lowCrossHz = low;
        p.highCrossHz = high;
        p.ratio = 1.0f;
        p.thresholdDb = {0.0f, 0.0f, 0.0f};
        return runFx(m, p, x);
    };
    double worst = 0.0;
    for (const double f : logSpacedFrequencies(50.0, 12000.0, 12))
    {
        const double g = staticCurve(render, kRate, {-30.0}, f, 0.5, 0.2)[0].gainDb;
        worst = std::max(worst, std::abs(g));
    }
    return worst;
}

// Largest step of a carrier through the multiband while a crossover moves from `from` to `to` halfway.
double crossoverAutomationStepDb(bool highCrossover, float from, float to, double carrierHz)
{
    FxMultiband m;
    m.prepare(kRate);
    FxMultibandParams p;
    p.on = true;
    p.ratio = 1.0f;
    p.thresholdDb = {0.0f, 0.0f, 0.0f};
    (highCrossover ? p.highCrossHz : p.lowCrossHz) = from;
    const size_t n = static_cast<size_t>(1.0 * kRate), change = n / 2;
    std::vector<float> l(n), r;
    for (size_t i = 0; i < n; ++i)
        l[i] = static_cast<float>(0.4 * std::sin(2.0 * kPi * carrierHz * static_cast<double>(i) / kRate));
    r = l;
    for (size_t pos = 0; pos < n; pos += 32)
    {
        if (pos == change - change % 32)
            (highCrossover ? p.highCrossHz : p.lowCrossHz) = to;
        m.process(l.data() + pos, r.data() + pos, 32, p);
    }
    return clickTest(Signal(l.begin(), l.end()), carrierHz, kRate, 4000).stepDb;
}

// Image of a 30 Hz tone shifted up by 100 Hz (at 70 Hz) against the wanted tone (at 130 Hz), dB.
double shifterImageDb(double rate)
{
    FxRingShift s;
    s.prepare(rate);
    FxRingShiftParams p;
    p.on = true;
    p.mix = 1.0f;
    p.mode = 1;
    p.freqHz = 100.0f;
    const Signal y = runFx(s, p, sineWave(static_cast<size_t>(4.0 * rate), 30.0, rate, 0.3));
    const size_t from = static_cast<size_t>(2.0 * rate), n = static_cast<size_t>(2.0 * rate);
    const double wanted = toneAmplitude(y.data() + from, n, 130.0, rate);
    const double image = toneAmplitude(y.data() + from, n, 70.0, rate);
    return amplitudeToDb(image / wanted);
}

// The width module (mono sum) switched through the rack on a left-only 1 kHz sine; returns the left output.
Signal rackSwitch(bool startOn, size_t switchAt, size_t total)
{
    ModuleRack rack(ModuleRegistry::instance());
    ModuleContext ctx;
    ctx.sampleRate = kRate;
    rack.prepare(ctx);
    auto v = ParameterMap::defaultValues();
    std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
    for (int i = 0; i < rack.size(); ++i)
        v[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 0.0f;
    const auto set = [&](const char* id, float value) { v[static_cast<size_t>(ParameterMap::indexOf(id))] = value; };
    set("fx_width_amt", 0.0f);
    set("fx_width_on", startOn ? 1.0f : 0.0f);
    std::vector<float> l(total), r(total, 0.0f);
    for (size_t i = 0; i < total; ++i)
        l[i] = static_cast<float>(0.5 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
    for (size_t p = 0; p < total; p += 32)
    {
        if (p == switchAt)
            set("fx_width_on", startOn ? 0.0f : 1.0f);
        rack.process(l.data() + p, r.data() + p, 32, v.data(), legacy.data(), nullptr, 120.0);
    }
    return Signal(l.begin(), l.end());
}
// Rack run of one time-based module: a noise burst with the module on (mix 1), then switched off for 100 blocks, then
// switched on again over silence; returns the peak of that silence.
double moduleReplayPeak(const char* onId, const char* mixId)
{
    ModuleRack rack(ModuleRegistry::instance());
    ModuleContext ctx;
    ctx.sampleRate = kRate;
    rack.prepare(ctx);
    auto v = ParameterMap::defaultValues();
    std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
    for (int i = 0; i < rack.size(); ++i)
        v[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 0.0f;
    const auto set = [&](const char* id, float value) { v[static_cast<size_t>(ParameterMap::indexOf(id))] = value; };
    set(mixId, 1.0f);
    set(onId, 1.0f);
    const Signal burst = noiseBurst(1.0, kRate);
    std::vector<float> l(32), r(32);
    for (size_t pos = 0; pos + 32 <= burst.size(); pos += 32)
    {
        for (size_t i = 0; i < 32; ++i)
            l[i] = r[i] = static_cast<float>(burst[pos + i]);
        rack.process(l.data(), r.data(), 32, v.data(), legacy.data(), nullptr, 120.0);
    }
    set(onId, 0.0f);
    for (int b = 0; b < 100; ++b)
    {
        std::fill(l.begin(), l.end(), 0.0f);
        std::fill(r.begin(), r.end(), 0.0f);
        rack.process(l.data(), r.data(), 32, v.data(), legacy.data(), nullptr, 120.0);
    }
    set(onId, 1.0f);
    double peak = 0.0;
    for (int b = 0; b < static_cast<int>(1.5 * kRate / 32); ++b)
    {
        std::fill(l.begin(), l.end(), 0.0f);
        std::fill(r.begin(), r.end(), 0.0f);
        rack.process(l.data(), r.data(), 32, v.data(), legacy.data(), nullptr, 120.0);
        for (size_t i = 0; i < 32; ++i)
            peak = std::max({peak, static_cast<double>(std::abs(l[i])), static_cast<double>(std::abs(r[i]))});
    }
    return peak;
}
} // namespace

class FxFixTests : public UnitTest
{
public:
    FxFixTests() : UnitTest("Legacy effect fixes", "Effects") {}

    void runTest() override
    {
        beginTest("BFX2: the three bands of the multiband add up flat for close crossovers");
        {
            const double a = bandSumWorstDb(500.0f, 750.0f), b = bandSumWorstDb(300.0f, 1200.0f),
                         c = bandSumWorstDb(200.0f, 3000.0f);
            logMessage("band sum worst deviation: 500/750 " + String(a, 4) + " dB, 300/1200 " + String(b, 4) +
                       " dB, defaults " + String(c, 4) + " dB");
            expect(a < 0.01, "500 / 750 Hz: " + String(a) + " dB");
            expect(b < 0.01, "300 / 1200 Hz: " + String(b) + " dB");
            expect(c < 0.01, "200 / 3000 Hz: " + String(c) + " dB");
        }

        beginTest("BFX3: a time-based module switched on again does not replay the audio from before it was switched off");
        {
            for (const auto& [on, mix] : {std::pair<const char*, const char*>{"fx_delay_pro_on", "fx_delay_pro_mix"},
                                          std::pair<const char*, const char*>{"fx_ra_on", "fx_ra_mix"}})
            {
                const double peak = moduleReplayPeak(on, mix);
                expect(peak < 1.0e-6, std::string(on) + " replays " + String(peak));
            }
        }

        beginTest("BFX3: the clear of a long delay line is spread over deactivate() calls, in bounded steps");
        {
            DelayLine line;
            line.allocate(800000);
            for (int i = 0; i < 1000; ++i)
                line.write(1.0f);
            int calls = 0;
            int largest = 0;
            for (bool done = false; !done && calls < 1000; ++calls)
            {
                int budget = 65536;
                done = line.clearStep(budget);
                largest = std::max(largest, 65536 - budget);
            }
            expect(largest <= 65536 && calls > 8, "calls " + String(calls) + ", largest step " + String(largest));
            expectEquals(line.read(500.0f), 0.0f);

        }

        beginTest("BFX4: switching a module on or off through the rack leaves no click");
        {
            const size_t total = 24000, at = 9600;
            const double on = clickTest(rackSwitch(false, at, total), 1000.0, kRate, 2000).stepDb;
            const double off = clickTest(rackSwitch(true, at, total), 1000.0, kRate, 2000).stepDb;
            logMessage("on/off through the rack: largest step on " + String(on, 1) + " dB, off " + String(off, 1) + " dB");
            expect(on < -60.0, "switch on: " + String(on) + " dB");
            expect(off < -60.0, "switch off: " + String(off) + " dB");
            // The module really is on afterwards and really is off.
            const Signal after = rackSwitch(false, at, total), before = rackSwitch(true, at, total);
            const auto level = [](const Signal& x, size_t from, size_t to)
            { return toneAmplitude(x.data() + from, to - from, 1000.0, kRate); };
            expect(std::abs(level(after, 20000, 24000) - 0.25) < 0.01,
                   "on: mono sum level " + String(level(after, 20000, 24000)));
            expect(std::abs(level(before, 20000, 24000) - 0.5) < 0.01,
                   "off: dry level " + String(level(before, 20000, 24000)));
            // Pieces of any size give the same result as blocks of 32 (the fade is per sample).
            ModuleRack rack(ModuleRegistry::instance());
            ModuleContext ctx;
            ctx.sampleRate = kRate;
            ctx.maxBlockSize = 64;
            rack.prepare(ctx);
            auto v = ParameterMap::defaultValues();
            std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
            for (int i = 0; i < rack.size(); ++i)
                v[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 0.0f;
            v[static_cast<size_t>(ParameterMap::indexOf("fx_width_amt"))] = 0.0f;
            std::vector<float> l(1000), r(1000, 0.0f);
            for (size_t i = 0; i < l.size(); ++i)
                l[i] = static_cast<float>(0.5 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
            rack.process(l.data(), r.data(), 1000, v.data(), legacy.data(), nullptr, 120.0);
            v[static_cast<size_t>(ParameterMap::indexOf("fx_width_on"))] = 1.0f;
            for (size_t i = 0; i < l.size(); ++i)
                l[i] = static_cast<float>(0.5 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
            std::fill(r.begin(), r.end(), 0.0f);
            rack.process(l.data(), r.data(), 1000, v.data(), legacy.data(), nullptr, 120.0);
            bool finite = true;
            for (const float s : l)
                finite = finite && std::isfinite(s);
            expect(finite && std::abs(l[999]) <= 0.5f, "a block larger than the rack's block capacity fades in pieces");
        }

        beginTest("BFX5: changing the limiter's lookahead does not drop out");
        {
            FxLimiter lim;
            lim.prepare(kRate);
            FxLimiterParams p;
            p.on = true;
            p.lookaheadMs = 1.5f;
            const size_t n = static_cast<size_t>(1.0 * kRate), change = n / 2;
            std::vector<float> in(n), l, r;
            for (size_t i = 0; i < n; ++i)
                in[i] = static_cast<float>(0.4 * std::sin(2.0 * kPi * 1000.0 * static_cast<double>(i) / kRate));
            l = in;
            r = in;
            for (size_t pos = 0; pos < n; pos += 32)
            {
                if (pos == change - change % 32)
                    p.lookaheadMs = 3.0f;
                lim.process(l.data() + pos, r.data() + pos, 32, p);
            }
            const ClickResult c = clickTest(Signal(l.begin(), l.end()), 1000.0, kRate, 4000);
            logMessage("limiter lookahead 1.5 -> 3 ms: largest step " + String(c.stepDb, 1) + " dB");
            expect(c.stepDb < -90.0, "step " + String(c.stepDb) + " dB");
            // A sine of 72 samples per period (the length of the change) keeps its level while the taps cross over.
            std::vector<float> in2(n), l2, r2;
            for (size_t i = 0; i < n; ++i)
                in2[i] = static_cast<float>(0.4 * std::sin(2.0 * kPi * (kRate / 72.0) * static_cast<double>(i) / kRate));
            l2 = in2;
            r2 = in2;
            FxLimiter lim2;
            lim2.prepare(kRate);
            p.lookaheadMs = 1.5f;
            for (size_t pos = 0; pos < n; pos += 32)
            {
                if (pos == change - change % 32)
                    p.lookaheadMs = 3.0f;
                lim2.process(l2.data() + pos, r2.data() + pos, 32, p);
            }
            double lowest = 1.0;
            for (size_t i = change; i < change + 2400; i += 72)
            {
                double window = 0.0;
                for (size_t j = i; j < i + 72; ++j)
                    window = std::max(window, static_cast<double>(std::abs(l2[j])));
                lowest = std::min(lowest, window);
            }
            expect(lowest > 0.38, "level dips to " + String(lowest) + " during the change");
            // Afterwards the output is the input delayed by 3 ms.
            const size_t delay = 144;
            double err = 0.0;
            for (size_t i = change + 2400; i < n; ++i)
                err = std::max(err, static_cast<double>(std::abs(l[i] - in[i - delay])));
            expect(err < 1.0e-6, "settled delay differs by " + String(err));
        }

        beginTest("BFX6: automating a crossover leaves no step above -90 dB");
        {
            const double lowStep = crossoverAutomationStepDb(false, 400.0f, 800.0f, 500.0);
            const double highStep = crossoverAutomationStepDb(true, 2000.0f, 4000.0f, 1500.0);
            logMessage("crossover 400 -> 800 Hz: " + String(lowStep, 1) + " dB, 2000 -> 4000 Hz: " + String(highStep, 1) +
                       " dB");
            expect(lowStep < -90.0, "low crossover step " + String(lowStep) + " dB");
            expect(highStep < -90.0, "high crossover step " + String(highStep) + " dB");
        }

        beginTest("BFX7: the frequency shifter's image rejection at 30 Hz does not depend on the sample rate");
        {
            const double at48 = shifterImageDb(48000.0);
            logMessage("shifter image at 48 kHz: " + String(at48, 1) + " dB");
            expect(at48 < -40.0, "48 kHz: " + String(at48) + " dB");
            for (const double rate : {22050.0, 32000.0, 44100.0, 88200.0, 96000.0, 176400.0, 192000.0, 384000.0})
            {
                const double image = shifterImageDb(rate);
                logMessage("shifter image at " + String(rate) + " Hz: " + String(image, 1) + " dB");
                // At least as good as 48 kHz, or within 3 dB of it.
                expect(image < at48 + 3.0, String(rate) + " Hz: " + String(image) + " dB against " + String(at48));
            }
        }
    }
};

static FxFixTests fxFixTests;
