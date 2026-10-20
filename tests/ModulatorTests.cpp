// SPDX-License-Identifier: MIT
// P12: the modulator bank (transfer functions, lag time constants, S&H and quantise, Turing, chaos, note gate, follower,
// pitch tracker, LFO table), scopes, evaluation order, and modulation of modulation (route amounts as targets).

#include "AnalogMeasure.h"
#include "TestSupport.h"

#include <crucible/Modulation.h>
#include <crucible/Modulators.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace crucible;
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
constexpr double kFs = 48000.0;
constexpr int kMacro1 = static_cast<int>(ModSource::Macro1);
constexpr int kMacro2 = static_cast<int>(ModSource::Macro2);
constexpr int kLfo1 = static_cast<int>(ModSource::Lfo1);

// One bank evaluated chunk by chunk with hand-set input sources.
struct Rig
{
    ModulatorSet set;
    std::array<ModulatorState, kNumBankModulators> state{};
    std::array<float, kNumSourcesExt> sources{};
    ModulatorContext ctx;
    std::uint32_t used = 1;

    Rig()
    {
        for (auto& c : set.mod)
            for (size_t k = 0; k < 16; ++k)
                c.curve[k] = static_cast<float>(k) / 15.0f;
        ctx.sampleRate = kFs;
        ctx.voice = true;
    }
    ModulatorConfig& cfg(int n = 0) { return set.mod[static_cast<size_t>(n)]; }
    void finalize() { set.finalize(used); }
    float out(int n = 0) const { return sources[static_cast<size_t>(kExtModulatorFirst + n)]; }
    float step(int samples = 32)
    {
        ctx.numSamples = samples;
        evaluateModulators(set, state.data(), ctx, sources.data());
        ctx.noteOn = false;
        return out();
    }
    void src(int source, float v) { sources[static_cast<size_t>(source)] = v; }
};

std::string num(double v, int decimals = 3)
{
    return String(v, decimals).str();
}
} // namespace

class ModulatorTests : public UnitTest
{
public:
    ModulatorTests() : UnitTest("Modulator bank", "Modulation") {}

    void runTest() override
    {
        combine();
        remap();
        lag();
        quantise();
        sampleHold();
        turing();
        binary();
        chaos();
        noteGate();
        follower();
        pitchTracker();
        lfoTable();
        order();
        engineLevel();
        routeAmounts();
    }

private:
    void combine()
    {
        beginTest("Combine: add, multiply, min, max, crossfade and difference are exact");
        const float a = 0.3f, b = 0.8f;
        struct Case
        {
            CombineMode mode;
            float expected;
        };
        const float mix = 0.25f;
        for (const Case& c : {Case{CombineMode::Add, a + b}, Case{CombineMode::Multiply, a * b}, Case{CombineMode::Min, a},
                              Case{CombineMode::Max, b}, Case{CombineMode::Crossfade, a + (b - a) * mix},
                              Case{CombineMode::Difference, a - b}})
        {
            Rig rig;
            rig.cfg().type = ModulatorType::Combine;
            rig.cfg().inA = kMacro1;
            rig.cfg().inB = kMacro2;
            rig.cfg().mode = static_cast<int>(c.mode);
            rig.cfg().amount = mix;
            rig.finalize();
            rig.src(kMacro1, a);
            rig.src(kMacro2, b);
            expectWithinAbsoluteError(rig.step(), c.expected, 1.0e-6, "combine mode " + String(static_cast<int>(c.mode)));
        }
        // Polarity follows the inputs: a difference is bipolar, an add of unipolar sources is unipolar.
        Rig rig;
        rig.cfg().type = ModulatorType::Combine;
        rig.cfg().inA = kMacro1;
        rig.cfg().inB = kMacro2;
        rig.cfg().mode = static_cast<int>(CombineMode::Add);
        rig.finalize();
        expect(!rig.set.bipolar[0]);
        rig.cfg().inA = kLfo1;
        rig.finalize();
        expect(rig.set.bipolar[0], "an LFO input makes the sum bipolar");
    }

    void remap()
    {
        beginTest(
            "Remap: the 16-point curve is exact at its points and linear between them, for unipolar and bipolar inputs");
        Rig rig;
        auto& c = rig.cfg();
        c.type = ModulatorType::Remap;
        c.inA = kMacro1;
        std::vector<float> points = {-0.9f, 0.4f, 0.1f,  0.7f,  -0.3f, 0.5f, 0.2f,  -0.6f,
                                     0.9f,  0.0f, 0.35f, -1.0f, 1.0f,  0.8f, -0.2f, 0.6f};
        for (size_t k = 0; k < 16; ++k)
            c.curve[k] = points[k];
        rig.finalize();
        double worst = 0.0;
        for (int k = 0; k < 16; ++k)
        {
            rig.src(kMacro1, static_cast<float>(k) / 15.0f);
            worst = std::max(worst, std::abs(static_cast<double>(rig.step()) - points[static_cast<size_t>(k)]));
        }
        for (int k = 0; k < 15; ++k)
            for (const float f : {0.25f, 0.5f, 0.9f})
            {
                rig.src(kMacro1, (static_cast<float>(k) + f) / 15.0f);
                const double expected = points[static_cast<size_t>(k)] +
                                        (points[static_cast<size_t>(k) + 1] - points[static_cast<size_t>(k)]) * f;
                worst = std::max(worst, std::abs(static_cast<double>(rig.step()) - expected));
            }
        expect(worst < 2.0e-6, "worst deviation " + num(worst, 8));
        // A bipolar input spans the same 16 points over -1 .. 1.
        c.inA = kLfo1;
        rig.finalize();
        rig.src(kLfo1, -1.0f);
        expectWithinAbsoluteError(rig.step(), points[0], 1.0e-6);
        rig.src(kLfo1, 1.0f);
        expectWithinAbsoluteError(rig.step(), points[15], 1.0e-6);
        rig.src(kLfo1, 0.0f);
        const double mid = points[7] + (points[8] - points[7]) * 0.5;
        expectWithinAbsoluteError(rig.step(), mid, 2.0e-6);
        expect(rig.set.bipolar[0], "a curve with negative points is bipolar");
    }

    void lag()
    {
        beginTest("Lag: rise and fall time constants are exact (63.2 % in one time constant, within 2 %)");
        for (const bool rising : {true, false})
        {
            Rig rig;
            auto& c = rig.cfg();
            c.type = ModulatorType::Lag;
            c.inA = kMacro1;
            c.rise = 0.040f;
            c.fall = 0.200f;
            rig.finalize();
            rig.src(kMacro1, rising ? 0.0f : 1.0f);
            rig.step(); // starts at the input
            rig.src(kMacro1, rising ? 1.0f : 0.0f);
            const double tau = rising ? 0.040 : 0.200;
            double t = 0.0;
            double atTau = 0.0;
            while (t < 5.0 * tau)
            {
                const float v = rig.step();
                t += 32.0 / kFs;
                if (std::abs(t - tau) < 16.0 / kFs)
                    atTau = rising ? v : 1.0 - v;
            }
            expectWithinAbsoluteError(atTau, 1.0 - std::exp(-1.0), 0.02 * 0.632,
                                      std::string(rising ? "rise" : "fall") + " at one time constant");
            const float end = rig.out();
            expect(std::abs((rising ? end : 1.0f - end) - (1.0 - std::exp(-5.0))) < 1.0e-3, "after five time constants");
        }
        Rig rig;
        rig.cfg().type = ModulatorType::Lag;
        rig.cfg().inA = kMacro1;
        rig.finalize();
        rig.src(kMacro1, 0.7f);
        rig.step();
        rig.src(kMacro1, 0.2f);
        expectWithinAbsoluteError(rig.step(), 0.2f, 1.0e-6, "zero times follow the input at once");

        beginTest("Lag: one non-finite input does not latch the modulator");
        {
            Rig nan;
            nan.cfg().type = ModulatorType::Lag;
            nan.cfg().inA = kMacro1;
            nan.cfg().rise = 0.010f;
            nan.cfg().fall = 0.010f;
            nan.finalize();
            nan.src(kMacro1, 0.5f);
            nan.step();
            nan.src(kMacro1, std::numeric_limits<float>::quiet_NaN());
            nan.step();
            nan.src(kMacro1, 0.9f);
            float v = 0.0f;
            for (int i = 0; i < 200; ++i)
                v = nan.step();
            expectWithinAbsoluteError(v, 0.9f, 1.0e-3, "follows the input again");
        }
    }

    void quantise()
    {
        beginTest("Quantise: values land on the steps (nearest, down, up) and the ends map to the ends");
        for (const QuantiseMode mode : {QuantiseMode::Nearest, QuantiseMode::Down, QuantiseMode::Up})
        {
            Rig rig;
            auto& c = rig.cfg();
            c.type = ModulatorType::Quantise;
            c.inA = kMacro1;
            c.steps = 5;
            c.mode = static_cast<int>(mode);
            rig.finalize();
            std::set<int> seen;
            bool exact = true, ordered = true;
            float last = -1.0f;
            for (int i = 0; i <= 1000; ++i)
            {
                const float x = static_cast<float>(i) / 1000.0f;
                rig.src(kMacro1, x);
                const float y = rig.step();
                const float scaled = y * 4.0f;
                if (std::abs(scaled - std::round(scaled)) > 1.0e-5f)
                    exact = false;
                seen.insert(static_cast<int>(std::lround(scaled)));
                const float expected = mode == QuantiseMode::Nearest ? std::floor(x * 4.0f + 0.5f) / 4.0f
                                       : mode == QuantiseMode::Down  ? std::floor(x * 4.0f + 1.0e-5f) / 4.0f
                                                                     : std::ceil(x * 4.0f - 1.0e-5f) / 4.0f;
                if (std::abs(y - expected) > 1.0e-6f && std::abs(x * 4.0f - std::round(x * 4.0f)) > 1.0e-3f)
                    exact = false;
                if (y < last - 1.0e-6f)
                    ordered = false;
                last = y;
            }
            expect(exact && ordered, "mode " + String(static_cast<int>(mode)));
            expectEquals(static_cast<int>(seen.size()), 5);
        }
        // A bipolar input is quantised across -1 .. 1.
        Rig rig;
        rig.cfg().type = ModulatorType::Quantise;
        rig.cfg().inA = kLfo1;
        rig.cfg().steps = 3;
        rig.finalize();
        rig.src(kLfo1, 0.2f);
        expectEquals(rig.step(), 0.0f);
        rig.src(kLfo1, 0.6f);
        expectEquals(rig.step(), 1.0f);
        rig.src(kLfo1, -0.8f);
        expectEquals(rig.step(), -1.0f);
    }

    void sampleHold()
    {
        beginTest("S&H: holds between clocks, takes the input at the clock, and clocks at the rate");
        {
            Rig rig;
            auto& c = rig.cfg();
            c.type = ModulatorType::SampleHold;
            c.inA = kMacro1;
            c.mode = static_cast<int>(ClockMode::Rate);
            c.rate = 10.0f;
            rig.finalize();
            int changes = 0;
            float held = -1.0f;
            bool followsInput = true;
            const int chunks = static_cast<int>(2.0 * kFs / 32.0);
            for (int i = 0; i < chunks; ++i)
            {
                const float input = 0.5f + 0.5f * std::sin(0.01f * static_cast<float>(i));
                rig.src(kMacro1, input);
                const float v = rig.step();
                if (v != held)
                {
                    ++changes;
                    held = v;
                    followsInput = followsInput && std::abs(v - input) < 1.0e-6f;
                }
            }
            expect(changes >= 20 && changes <= 22, "changes in 2 s at 10 Hz: " + String(changes));
            expect(followsInput, "the held value is the input at the clock");
        }
        {
            Rig rig;
            auto& c = rig.cfg();
            c.type = ModulatorType::SampleHold;
            c.inA = kMacro1;
            c.inB = kMacro2;
            c.mode = static_cast<int>(ClockMode::Input);
            rig.finalize();
            rig.src(kMacro2, 0.0f);
            rig.src(kMacro1, 0.25f);
            rig.step(); // the first evaluation samples once
            rig.src(kMacro1, 0.6f);
            expectEquals(rig.step(), 0.25f, "no edge: holds");
            rig.src(kMacro2, 1.0f);
            expectEquals(rig.step(), 0.6f, "rising edge of B samples A");
            rig.src(kMacro1, 0.9f);
            expectEquals(rig.step(), 0.6f, "B stays high: holds");
            rig.src(kMacro2, 0.0f);
            rig.step();
            rig.src(kMacro2, 1.0f);
            expectEquals(rig.step(), 0.9f, "next edge");
        }
        {
            // Without an input it samples noise, the same sequence for the same seed.
            auto run = [](std::uint32_t seed)
            {
                Rig rig;
                auto& c = rig.cfg();
                c.type = ModulatorType::SampleHold;
                c.mode = static_cast<int>(ClockMode::Rate);
                c.rate = 100.0f;
                c.seed = seed;
                rig.finalize();
                std::vector<float> v;
                for (int i = 0; i < 300; ++i)
                    v.push_back(rig.step());
                return v;
            };
            expect(run(5) == run(5) && run(5) != run(6), "seeded noise");
            const auto v = run(5);
            expect(*std::min_element(v.begin(), v.end()) >= -1.0f && *std::max_element(v.begin(), v.end()) <= 1.0f);
        }
        {
            Rig rig;
            rig.cfg().type = ModulatorType::SampleHold;
            rig.cfg().inA = kMacro1;
            rig.cfg().mode = static_cast<int>(ClockMode::NoteOn);
            rig.finalize();
            rig.src(kMacro1, 0.3f);
            rig.step();
            rig.src(kMacro1, 0.7f);
            expectEquals(rig.step(), 0.3f);
            rig.ctx.noteOn = true;
            expectEquals(rig.step(), 0.7f, "a note-on samples");
        }
    }

    void turing()
    {
        beginTest("Turing: reproducible per seed, locked loops repeat with the register length, randomness breaks the loop");
        const auto run = [](std::uint32_t seed, int length, float randomness, int steps)
        {
            Rig rig;
            auto& c = rig.cfg();
            c.type = ModulatorType::Turing;
            c.mode = static_cast<int>(ClockMode::NoteOn); // one clock per evaluation
            c.steps = length;
            c.amount = randomness;
            c.seed = seed;
            rig.finalize();
            std::vector<float> v;
            for (int i = 0; i < steps; ++i)
            {
                rig.ctx.noteOn = true;
                v.push_back(rig.step());
            }
            return v;
        };
        expect(run(7, 8, 0.0f, 100) == run(7, 8, 0.0f, 100), "same seed, same sequence");
        expect(run(7, 8, 0.0f, 100) != run(8, 8, 0.0f, 100), "another seed, another sequence");
        expect(run(7, 8, 0.5f, 100) == run(7, 8, 0.5f, 100), "reproducible with randomness too");
        for (const int length : {4, 8, 16})
        {
            const auto v = run(3, length, 0.0f, 200);
            bool periodic = true;
            for (size_t i = 8; i + static_cast<size_t>(length) < v.size(); ++i)
                periodic = periodic && v[i] == v[i + static_cast<size_t>(length)];
            expect(periodic, "locked loop of length " + String(length));
            std::set<float> distinct(v.begin() + 8, v.end());
            expect(distinct.size() > 1, "the loop is not constant");
        }
        const auto free = run(3, 8, 1.0f, 400);
        bool periodic = true;
        for (size_t i = 8; i + 8 < free.size(); ++i)
            periodic = periodic && free[i] == free[i + 8];
        expect(!periodic, "full randomness does not repeat");
        expect(*std::min_element(free.begin(), free.end()) >= -1.0f && *std::max_element(free.begin(), free.end()) <= 1.0f);
    }

    void binary()
    {
        beginTest("Binary: outputs 0 or 1 with the set probability");
        for (const float p : {0.1f, 0.5f, 0.9f})
        {
            Rig rig;
            auto& c = rig.cfg();
            c.type = ModulatorType::Binary;
            c.mode = static_cast<int>(ClockMode::NoteOn);
            c.amount = p;
            c.seed = 11;
            rig.finalize();
            int ones = 0, total = 4000;
            bool binaryValued = true;
            for (int i = 0; i < total; ++i)
            {
                rig.ctx.noteOn = true;
                const float v = rig.step();
                binaryValued = binaryValued && (v == 0.0f || v == 1.0f);
                ones += v == 1.0f ? 1 : 0;
            }
            expect(binaryValued, "only 0 and 1");
            expectWithinAbsoluteError(static_cast<double>(ones) / total, p, 0.03, "frequency of ones for p = " + num(p, 1));
        }
    }

    void chaos()
    {
        beginTest("Chaos: logistic and Lorenz stay bounded, never settle, and repeat per seed");
        for (const ChaosMap map : {ChaosMap::Logistic, ChaosMap::Lorenz})
        {
            const auto run = [&](std::uint32_t seed, int steps, float amount)
            {
                Rig rig;
                auto& c = rig.cfg();
                c.type = ModulatorType::Chaos;
                c.mode = static_cast<int>(map);
                c.rate = map == ChaosMap::Logistic ? 400.0f : 8.0f;
                c.amount = amount;
                c.seed = seed;
                rig.finalize();
                std::vector<float> v;
                for (int i = 0; i < steps; ++i)
                    v.push_back(rig.step());
                return v;
            };
            const auto v = run(2, 300000, 0.8f);
            const auto [lo, hi] = std::minmax_element(v.begin(), v.end());
            expect(*lo >= -1.0f && *hi <= 1.0f, "bounded");
            double mean = 0.0, var = 0.0;
            for (const float x : v)
                mean += x;
            mean /= static_cast<double>(v.size());
            for (const float x : v)
                var += (x - mean) * (x - mean);
            var /= static_cast<double>(v.size());
            expect(var > 0.02, std::string(map == ChaosMap::Logistic ? "logistic" : "Lorenz") + " variance " + num(var, 4));
            expect(*hi - *lo > 0.8f, "range used " + num(*hi - *lo));
            expect(run(2, 2000, 0.8f) == std::vector<float>(v.begin(), v.begin() + 2000), "deterministic");
            expect(run(3, 2000, 0.8f) != std::vector<float>(v.begin(), v.begin() + 2000), "a seed changes the orbit");
        }
        // Even at the edge of the logistic map's range (r = 4) the iterate stays inside 0 .. 1.
        Rig rig;
        rig.cfg().type = ModulatorType::Chaos;
        rig.cfg().mode = 0;
        rig.cfg().rate = 1500.0f;
        rig.cfg().amount = 1.0f;
        rig.finalize();
        float lo = 2.0f, hi = -2.0f;
        for (int i = 0; i < 200000; ++i)
        {
            const float v = rig.step();
            lo = std::min(lo, v);
            hi = std::max(hi, v);
            if (!std::isfinite(v))
                expect(false, "finite");
        }
        expect(lo >= -1.0f && hi <= 1.0f, "r = 4: [" + num(lo) + ", " + num(hi) + "]");
    }

    void noteGate()
    {
        beginTest("Note gate: follows the key per voice, scales with velocity, or pulses at note-on");
        Rig held, free;
        for (Rig* r : {&held, &free})
        {
            r->cfg().type = ModulatorType::NoteGate;
            r->finalize();
        }
        held.ctx.gate = true;
        free.ctx.gate = false;
        expectEquals(held.step(), 1.0f);
        expectEquals(free.step(), 0.0f, "two voices, two gates");
        held.ctx.gate = false;
        expectEquals(held.step(), 0.0f, "released");
        Rig vel;
        vel.cfg().type = ModulatorType::NoteGate;
        vel.cfg().mode = static_cast<int>(GateMode::Velocity);
        vel.finalize();
        vel.ctx.gate = true;
        vel.ctx.velocity = 0.4f;
        expectEquals(vel.step(), 0.4f);
        Rig pulse;
        pulse.cfg().type = ModulatorType::NoteGate;
        pulse.cfg().mode = static_cast<int>(GateMode::Trigger);
        pulse.cfg().rise = 0.01f; // 10 ms
        pulse.finalize();
        pulse.ctx.noteOn = true;
        pulse.ctx.gate = true;
        int high = 0;
        for (int i = 0; i < 40; ++i)
        {
            if (pulse.step() > 0.5f)
                ++high;
            pulse.ctx.noteOn = false;
        }
        expect(high >= 14 && high <= 16, "10 ms pulse is " + String(high) + " chunks of 0.67 ms");
    }

    void follower()
    {
        beginTest("Follower: attack and release time constants, and the mean of a rectified sine");
        const auto stepWith = [](Rig& rig, const std::vector<float>& audio)
        {
            rig.ctx.audio = audio.data();
            rig.ctx.audioCount = static_cast<int>(audio.size());
            return rig.step();
        };
        {
            Rig rig;
            rig.cfg().type = ModulatorType::Follower;
            rig.cfg().rise = 0.005f;
            rig.cfg().fall = 0.050f;
            rig.finalize();
            const std::vector<float> dc(32, 0.5f), silence(32, 0.0f);
            double t = 0.0, atTau = 0.0;
            while (t < 0.005)
            {
                atTau = stepWith(rig, dc);
                t += 32.0 / kFs;
            }
            expectWithinAbsoluteError(atTau / 0.5, 1.0 - std::exp(-t / 0.005), 0.003, "attack follows 1 - exp(-t / 5 ms)");
            for (int i = 0; i < 400; ++i)
                stepWith(rig, dc);
            const float start = rig.out();
            expectWithinAbsoluteError(start, 0.5f, 1.0e-3, "settles at the input level");
            t = 0.0;
            float v = start;
            while (t < 0.050)
            {
                v = stepWith(rig, silence);
                t += 32.0 / kFs;
            }
            expectWithinAbsoluteError(v / start, std::exp(-1.0), 0.02, "released to 1/e in one time constant");
        }
        {
            Rig rig;
            rig.cfg().type = ModulatorType::Follower;
            rig.cfg().rise = 0.020f;
            rig.cfg().fall = 0.020f;
            rig.finalize();
            std::vector<float> audio(32);
            double phase = 0.0;
            for (int c = 0; c < 600; ++c)
            {
                for (int i = 0; i < 32; ++i)
                {
                    audio[static_cast<size_t>(i)] = static_cast<float>(0.5 * std::sin(phase));
                    phase += 2.0 * 3.14159265358979 * 440.0 / kFs;
                }
                stepWith(rig, audio);
            }
            expectWithinAbsoluteError(rig.out(), 0.5 * 2.0 / 3.14159265358979, 0.01, "mean of |sin|");
        }
        Rig control;
        control.cfg().type = ModulatorType::Follower;
        control.cfg().mode = static_cast<int>(FollowMode::Input);
        control.cfg().inA = kLfo1;
        control.finalize();
        control.src(kLfo1, -0.6f);
        expectWithinAbsoluteError(control.step(), 0.6f, 1.0e-6, "input mode takes |A|");

        beginTest("Follower: a non-finite input or audio sample does not latch the envelope");
        {
            Rig audioRig;
            audioRig.cfg().type = ModulatorType::Follower;
            audioRig.cfg().rise = 0.005f;
            audioRig.cfg().fall = 0.020f;
            audioRig.finalize();
            std::vector<float> bad(32, 0.4f), good(32, 0.4f);
            bad[5] = std::numeric_limits<float>::quiet_NaN();
            bad[9] = std::numeric_limits<float>::infinity();
            stepWith(audioRig, bad);
            float v = 0.0f;
            for (int i = 0; i < 100; ++i)
                v = stepWith(audioRig, good);
            expectWithinAbsoluteError(v, 0.4f, 1.0e-3, "audio mode recovers");

            Rig inputRig;
            inputRig.cfg().type = ModulatorType::Follower;
            inputRig.cfg().mode = static_cast<int>(FollowMode::Input);
            inputRig.cfg().inA = kMacro1;
            inputRig.cfg().rise = 0.005f;
            inputRig.cfg().fall = 0.005f;
            inputRig.finalize();
            inputRig.src(kMacro1, std::numeric_limits<float>::quiet_NaN());
            inputRig.step();
            inputRig.src(kMacro1, 0.7f);
            for (int i = 0; i < 100; ++i)
                v = inputRig.step();
            expectWithinAbsoluteError(v, 0.7f, 1.0e-3, "input mode recovers");
        }
    }

    void pitchTracker()
    {
        beginTest("Pitch tracker: sine and saw within 3 cents");
        for (const double hz : {110.0, 220.0, 330.0, 880.0})
            for (const bool saw : {false, true})
            {
                Rig rig;
                rig.cfg().type = ModulatorType::PitchTracker;
                rig.cfg().rise = 0.02f;
                rig.finalize();
                std::vector<float> audio(32);
                double phase = 0.0;
                for (int c = 0; c < 400; ++c)
                {
                    for (int i = 0; i < 32; ++i)
                    {
                        const double w = phase - std::floor(phase);
                        audio[static_cast<size_t>(i)] =
                            static_cast<float>(0.4 * (saw ? 2.0 * w - 1.0 : std::sin(2.0 * 3.14159265358979 * phase)));
                        phase += hz / kFs;
                    }
                    rig.ctx.audio = audio.data();
                    rig.ctx.audioCount = 32;
                    rig.step();
                }
                const double note = 60.0 * rig.out() + 60.0;
                const double expected = 69.0 + 12.0 * std::log2(hz / 440.0);
                expectWithinAbsoluteError(note, expected, 0.03, String(hz, 0).str() + " Hz " + (saw ? "saw" : "sine"));
            }
    }

    void lfoTable()
    {
        beginTest("LFO table: sine at position 0, rate exact, morph between frames");
        Rig rig;
        auto& c = rig.cfg();
        c.type = ModulatorType::LfoTable;
        c.rate = 6.0f;
        c.amount = 0.0f;
        rig.finalize();
        double worst = 0.0;
        double phase = 0.0;
        for (int i = 0; i < 3000; ++i)
        {
            const float v = rig.step();
            phase += 6.0 * 32.0 / kFs;
            worst = std::max(worst, std::abs(static_cast<double>(v) - std::sin(2.0 * 3.14159265358979 * phase)));
        }
        expect(worst < 1.5e-3, "worst deviation from sin " + num(worst, 6));
        // Position 1/7 is the triangle frame: peak value 1 at a quarter cycle... check the morph midway differs from both.
        Rig a, b, mid;
        for (Rig* r : {&a, &b, &mid})
        {
            r->cfg().type = ModulatorType::LfoTable;
            r->cfg().rate = 6.0f;
        }
        a.cfg().amount = 0.0f;
        b.cfg().amount = 1.0f / 7.0f;
        mid.cfg().amount = 0.5f / 7.0f;
        a.finalize();
        b.finalize();
        mid.finalize();
        for (int i = 0; i < 100; ++i)
        {
            const float va = a.step(), vb = b.step(), vm = mid.step();
            expectWithinAbsoluteError(vm, 0.5 * (va + vb), 1.0e-5);
        }
    }

    void order()
    {
        beginTest("Evaluation order: inputs only from earlier modulators; later or own indices read as none");
        Rig rig;
        rig.cfg(0).type = ModulatorType::Combine;
        rig.cfg(0).inA = kMacro1;
        rig.cfg(0).inB = kNumModSources + 1; // modulator 2: later, ignored
        rig.cfg(1).type = ModulatorType::Combine;
        rig.cfg(1).inA = kNumModSources + 0; // modulator 1: earlier, valid
        rig.cfg(1).inB = kMacro2;
        rig.cfg(2).type = ModulatorType::Combine;
        rig.cfg(2).inA = kNumModSources + 2; // itself: ignored
        rig.cfg(2).inB = kNumModSources + 1;
        rig.used = 1U << 2;
        rig.finalize();
        expect(ModulatorSet::validInput(kNumModSources + 0, 1) && !ModulatorSet::validInput(kNumModSources + 1, 1) &&
                   !ModulatorSet::validInput(kNumModSources + 2, 2),
               "validInput");
        expectEquals(static_cast<int>(rig.set.needed), 0b111, "a route to modulator 3 pulls in the chain it reads");
        rig.src(kMacro1, 0.2f);
        rig.src(kMacro2, 0.5f);
        rig.step();
        expectEquals(rig.out(0), 0.2f, "modulator 1 ignores its later input");
        expectEquals(rig.out(1), 0.7f, "modulator 2 adds modulator 1 and macro 2");
        expectEquals(rig.out(2), 0.7f, "modulator 3 reads modulator 2 only");
        // Not needed, not run.
        Rig idle;
        idle.cfg(0).type = ModulatorType::Combine;
        idle.cfg(0).inA = kMacro1;
        idle.used = 0;
        idle.finalize();
        idle.src(kMacro1, 0.9f);
        idle.step();
        expectEquals(idle.out(0), 0.0f);
        // Scopes: a shared evaluation leaves voice modulators at zero, a voice evaluation keeps the shared values.
        Rig scoped;
        scoped.cfg(0).type = ModulatorType::Combine;
        scoped.cfg(0).inA = kMacro1;
        scoped.cfg(0).scope = ModulatorScope::Global;
        scoped.cfg(1).type = ModulatorType::Combine;
        scoped.cfg(1).inA = kNumModSources + 0;
        scoped.cfg(1).inB = kMacro2;
        scoped.cfg(1).scope = ModulatorScope::Voice;
        scoped.used = 0b11;
        scoped.finalize();
        scoped.src(kMacro1, 0.4f);
        scoped.src(kMacro2, 0.1f);
        scoped.ctx.voice = false;
        scoped.step();
        expectEquals(scoped.out(0), 0.4f);
        expectEquals(scoped.out(1), 0.0f, "voice modulators do not run in the shared evaluation");
        scoped.ctx.voice = true;
        scoped.step();
        expectEquals(scoped.out(0), 0.4f, "the voice finds the shared value in place");
        expectEquals(scoped.out(1), 0.5f, "and builds on it");
    }

    // Engine-level helpers.
    static std::vector<float> basePatch()
    {
        auto v = ParameterMap::defaultValues();
        setValue(v, "amp_attack", 0.001f);
        setValue(v, "amp_sustain", 1.0f);
        setValue(v, "master_gain", 0.0f);
        setValue(v, "osc1_level", 0.5f);
        setValue(v, "filter_cutoff", 500.0f);
        setValue(v, "filter_res", 0.0f);
        setValue(v, "filter_env", 0.0f);
        setValue(v, "filter_key", 0.0f);
        return v;
    }

    void engineLevel()
    {
        beginTest("the bank in the engine: parameters appended, sources 4000 + n, scope voice / global, route to a target");
        {
            const int type = ParameterMap::indexOf("bank1_type");
            expect(type >= 0 && ParameterMap::indexOf("bank8_pt16") > type, "parameters present");
            expectEquals(ParameterMap::params()[static_cast<size_t>(type)].module, std::string("mod.bank"));
            expectEquals(kSourceModulator, 4000);
            expect(isRouteSource(kSourceModulator) && isRouteSource(kSourceModulator + 7) &&
                   !isRouteSource(kSourceModulator + 8));
            expectEquals(routeSourceName(kSourceModulator + 2), std::string("Modulator 3"));
        }
        for (const bool global : {false, true})
        {
            SynthEngine engine;
            auto v = basePatch();
            setValue(v, "bank1_type", static_cast<float>(ModulatorType::Combine));
            setValue(v, "bank1_scope", global ? 2.0f : 0.0f);
            setValue(v, "bank1_in_a", static_cast<float>(kMacro1));
            setValue(v, "bank1_mode", static_cast<float>(CombineMode::Add));
            setValue(v, "macro1", 0.6f);
            prepare(engine, v);
            expect(engine.modRoutes().add({kSourceModulator, "filter_cutoff", 0.5f, 0, 0.0f, 0.0f}));
            engine.noteOn(60, 1.0f);
            renderBlocks(engine, 2048);
            expectWithinAbsoluteError(engine.modulatorValue(0), 0.6f, 1.0e-6, global ? "global" : "voice");
            const int cut = ParameterMap::indexOf("filter_cutoff");
            // 500 Hz x 2^(5 x 0.5 x 0.6)
            expectWithinAbsoluteError(engine.voiceModulatedValue(cut) / (500.0f * std::exp2(5.0f * 0.5f * 0.6f)), 1.0,
                                      2.0e-3);
        }
        beginTest("a modulator nobody reads costs nothing and does not move the sound");
        {
            const auto render = [](bool withBank)
            {
                SynthEngine engine;
                auto v = basePatch();
                if (withBank)
                {
                    setValue(v, "bank1_type", static_cast<float>(ModulatorType::Chaos));
                    setValue(v, "bank2_type", static_cast<float>(ModulatorType::Turing));
                }
                prepare(engine, v);
                engine.noteOn(57, 1.0f);
                return renderBlocks(engine, 8192, 512, true).left;
            };
            expect(render(false) == render(true), "bit-identical with unused modulators");
        }
    }

    void routeAmounts()
    {
        beginTest("route amounts as targets: depth follows another source (mod of mod), cycles are refused");
        {
            SynthEngine engine;
            auto v = basePatch();
            setValue(v, "macro1", 1.0f);
            setValue(v, "macro2", 1.0f);
            prepare(engine, v);
            const int cut = ParameterMap::indexOf("filter_cutoff");
            expect(engine.modRoutes().add({static_cast<int>(ModSource::Macro1), "filter_cutoff", 0.4f, 0, 0.0f, 0.0f}));
            engine.noteOn(60, 1.0f);
            renderBlocks(engine, 2048);
            expectWithinAbsoluteError(engine.voiceModulatedValue(cut) / (500.0f * std::exp2(5.0f * 0.4f)), 1.0, 2.0e-3,
                                      "route 0 alone");
            // Route 1 moves the amount of route 0 by 0.3 x macro 2.
            expect(
                engine.modRoutes().add({static_cast<int>(ModSource::Macro2), routeAmountTargetId(0), 0.3f, 0, 0.0f, 0.0f}));
            renderBlocks(engine, 2048);
            expectWithinAbsoluteError(engine.voiceModulatedValue(cut) / (500.0f * std::exp2(5.0f * 0.7f)), 1.0, 2.0e-3,
                                      "amount 0.4 + 0.3 x 1.0");
            // Turn macro 2 down: depth follows.
            auto v2 = v;
            setValue(v2, "macro2", 0.5f);
            engine.setParameterValues(v2.data(), static_cast<int>(v2.size()));
            renderBlocks(engine, 8192);
            expectWithinAbsoluteError(engine.voiceModulatedValue(cut) / (500.0f * std::exp2(5.0f * 0.55f)), 1.0, 3.0e-3,
                                      "amount 0.4 + 0.3 x 0.5");
            // A third route moves the amount of route 1 (modulation of modulation of modulation).
            expect(
                engine.modRoutes().add({static_cast<int>(ModSource::Macro1), routeAmountTargetId(1), -0.5f, 0, 0.0f, 0.0f}));
            renderBlocks(engine, 8192);
            // route 1 amount 0.3 - 0.5 = -0.2: route 0 amount = 0.4 + (-0.2 x 0.5) = 0.3
            expectWithinAbsoluteError(engine.voiceModulatedValue(cut) / (500.0f * std::exp2(5.0f * 0.3f)), 1.0, 3.0e-3,
                                      "three deep");
        }
        {
            ModRouteStore store;
            const int lfo = static_cast<int>(ModSource::Lfo1);
            expect(store.add({lfo, "filter_cutoff", 0.5f}));
            expect(store.add({lfo, routeAmountTargetId(0), 0.5f}), "route 1 moves route 0");
            expect(!store.add({lfo, routeAmountTargetId(2), 0.5f}), "route 2 cannot move its own amount");
        }
        {
            ModRouteStore store;
            const int lfo = static_cast<int>(ModSource::Lfo1), env = static_cast<int>(ModSource::ModEnv);
            expect(store.add({lfo, routeAmountTargetId(1), 0.5f})); // route 0 moves route 1
            expect(!store.add({env, routeAmountTargetId(0), 0.5f}), "route 1 moving route 0 would close a cycle");
            expect(store.add({env, "filter_cutoff", 0.5f}), "an ordinary route still fits");
            ModRouteStore self;
            expect(!self.add({lfo, routeAmountTargetId(0), 0.5f}), "a route cannot target its own amount");
            // Removing a route shifts the positions; the set still compiles (a cycle that appears is dropped).
            ModRouteStore shifting;
            expect(shifting.add({lfo, "filter_cutoff", 0.5f}));
            expect(shifting.add({lfo, routeAmountTargetId(0), 0.5f}));
            expect(shifting.add({env, routeAmountTargetId(1), 0.5f}));
            expect(shifting.remove(lfo, "filter_cutoff"));
            const RouteSet* set = shifting.acquire();
            expect(set != nullptr);
            shifting.release();
        }
        {
            // A route-amount target at or past kMaxRoutes would index past the audio thread's delta arrays.
            ModRouteStore store;
            const int lfo = static_cast<int>(ModSource::Lfo1);
            expect(!store.accepts(routeAmountTargetId(kMaxRoutes)), "route index kMaxRoutes is rejected");
            expect(!store.accepts(routeAmountTargetId(500)), "route index 500 is rejected");
            expect(store.accepts(routeAmountTargetId(kMaxRoutes - 1)), "last valid route index is accepted");
            expect(!store.add({lfo, routeAmountTargetId(kMaxRoutes), 0.5f}));
            expect(!store.add({lfo, routeAmountTargetId(500), 0.5f}));
            // A target past the end of the list is inert and never reaches the amount modulation list.
            expect(store.add({lfo, routeAmountTargetId(5), 0.5f}));
            const RouteSet* set = store.acquire();
            expect(set != nullptr);
            bool allInList = true;
            for (const auto& m : set->amountMods)
                allInList = allInList && m.target < static_cast<int>(set->routes.size());
            expect(allInList, "amount mods only target routes inside the list");
            expect(set->amountMods.empty(), "target past the list is dropped");
            store.release();
        }
    }
};

static ModulatorTests modulatorTests;
