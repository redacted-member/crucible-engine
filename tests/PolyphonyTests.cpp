// SPDX-License-Identifier: MIT
// Polyphony: notes never cancel each other, retriggers and steals are click free, note-offs and the sustain pedal
// act on their own voices, and a full 16-note, 8-unison, 3-oscillator load stays well inside the block budget.

#include "GraphSupport.h"

#include <chrono>

using namespace crucible;
using namespace crucible::testing;

namespace
{
std::vector<float> sinePatch()
{
    auto v = graphPatch();
    setValue(v, "osc1_pos", 0.0f); // sine
    setValue(v, "osc1_level", 1.0f);
    setValue(v, "amp_release", 0.05f);
    return v;
}

double rms(const std::vector<float>& s, size_t from, size_t to)
{
    double e = 0.0;
    for (size_t i = from; i < to; ++i)
        e += static_cast<double>(s[i]) * s[i];
    return std::sqrt(e / static_cast<double>(to - from));
}

double db(double ratio)
{
    return 20.0 * std::log10(std::max(ratio, 1.0e-12));
}

// Plays `first` at sample 0 and `second` at sample `offset`; returns the left channel.
std::vector<float> playTwo(const std::vector<float>& v, int note, int offset, int total, int secondNote = -1)
{
    CRUCIBLE_HEAP(SynthEngine, engine);
    prepare(engine, v);
    std::vector<float> out;
    std::vector<float> l(32), r(32);
    engine.noteOn(note, 1.0f);
    for (int pos = 0; pos < total; pos += 32)
    {
        if (offset >= pos && offset < pos + 32)
            engine.noteOn(secondNote < 0 ? note : secondNote, 1.0f);
        engine.render(l.data(), r.data(), 32);
        out.insert(out.end(), l.begin(), l.end());
    }
    return out;
}

double magnitudeAt(const std::vector<float>& s, size_t from, size_t to, double hz)
{
    double re = 0.0, im = 0.0;
    for (size_t i = from; i < to; ++i)
    {
        const double ph = 6.283185307179586 * hz * static_cast<double>(i) / kRate;
        const double w =
            0.5 - 0.5 * std::cos(6.283185307179586 * static_cast<double>(i - from) / static_cast<double>(to - from));
        re += w * s[i] * std::cos(ph);
        im += w * s[i] * std::sin(ph);
    }
    return 2.0 * std::sqrt(re * re + im * im) / static_cast<double>(to - from);
}
} // namespace

class PolyphonyTests : public UnitTest
{
public:
    PolyphonyTests() : UnitTest("Polyphony", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("two notes of the same pitch never cancel, whatever the retrigger offset");
        {
            for (const char* name : {"sine", "saw", "default"})
            {
                auto v = sinePatch();
                if (std::string(name) == "saw")
                    setValue(v, "osc1_pos", 1.0f);
                if (std::string(name) == "default")
                    v = ParameterMap::defaultValues();
                const int total = 48000 + 4000;
                const auto single = playTwo(v, 60, 1 << 30, total);
                const double level = rms(single, 40000, 52000);
                double worst = 1.0e9;
                // Offsets sweep several periods (C4 is about 183 samples) in steps that are not a divisor of it.
                for (int step = 0; step < 24; ++step)
                {
                    const int offset = 9600 + step * 71;
                    const auto pair = playTwo(v, 60, offset, total);
                    worst = std::min(worst, db(rms(pair, 40000, 52000) / level));
                }
                std::printf("    %s: worst overlapped pair %.2f dB against a single note\n", name, worst);
                expect(worst > -1.0, std::string(name) + " pair level " + std::to_string(worst));
            }
            // Two identical notes struck together.
            auto v = sinePatch();
            const auto single = playTwo(v, 60, 1 << 30, 52000);
            const auto together = playTwo(v, 60, 0, 52000);
            expect(db(rms(together, 40000, 52000) / rms(single, 40000, 52000)) > -1.0);
        }

        beginTest("a retrigger fades the older voice out without a click");
        {
            auto v = sinePatch();
            const auto base = playTwo(v, 69, 1 << 30, 30000);
            double calm = 0.0;
            for (size_t i = 20001; i < 24000; ++i)
                calm = std::max(calm, static_cast<double>(std::abs(base[i] - base[i - 1])));
            for (const int offset : {10000, 10037, 10090, 10133})
            {
                const auto two = playTwo(v, 69, offset, 30000);
                double worst = 0.0;
                for (size_t i = static_cast<size_t>(offset) - 64; i < static_cast<size_t>(offset) + 1000; ++i)
                    worst = std::max(worst, static_cast<double>(std::abs(two[i] - two[i - 1])));
                expect(worst < 3.2 * calm, "retrigger at " + std::to_string(offset) + ": step " + std::to_string(worst) +
                                               " against " + std::to_string(calm));
            }
        }

        beginTest("16 distinct notes sound at once; the 17th steals the oldest without a click");
        {
            auto v = sinePatch();
            setValue(v, "osc1_level", 0.4f);
            setValue(v, "amp_sustain", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, v);
            std::vector<float> l(32), r(32), out;
            for (int n = 0; n < 16; ++n)
                engine.noteOn(36 + 3 * n, 1.0f);
            for (int b = 0; b < 600; ++b)
            {
                engine.render(l.data(), r.data(), 32);
                out.insert(out.end(), l.begin(), l.end());
            }
            expectEquals(engine.activeVoiceCount(), 16);
            const size_t stealAt = out.size();
            engine.noteOn(36 + 3 * 16 + 1, 1.0f);
            expectEquals(engine.activeVoiceCount(), 16);
            for (int b = 0; b < 600; ++b)
            {
                engine.render(l.data(), r.data(), 32);
                out.insert(out.end(), l.begin(), l.end());
            }
            expectEquals(engine.activeVoiceCount(), 16);
            double before = 0.0, around = 0.0;
            for (size_t i = stealAt - 4000; i < stealAt - 1; ++i)
                before = std::max(before, static_cast<double>(std::abs(out[i + 1] - out[i])));
            for (size_t i = stealAt; i < stealAt + 1500; ++i)
                around = std::max(around, static_cast<double>(std::abs(out[i + 1] - out[i])));
            expect(around < 1.5 * before + 0.02,
                   "steal step " + std::to_string(around) + " against " + std::to_string(before));
            // The oldest note (36) is gone, the newest is present, the others continue.
            const size_t a = stealAt + 6000, z = out.size();
            expect(magnitudeAt(out, a, z, midiToHz(36)) < 0.02 * magnitudeAt(out, a, z, midiToHz(39)), "oldest gone");
            expect(magnitudeAt(out, a, z, midiToHz(36 + 3 * 16 + 1)) > 0.5 * magnitudeAt(out, a, z, midiToHz(39)),
                   "newest present");
            expect(magnitudeAt(out, a, z, midiToHz(39)) > 0.5 * magnitudeAt(out, 3000, 8000, midiToHz(39)),
                   "others continue");
        }

        beginTest("each note-off only affects its own voice");
        {
            auto v = sinePatch();
            setValue(v, "osc1_level", 0.3f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, v);
            std::vector<float> l(32), r(32), out;
            const int notes[] = {48, 52, 55, 60, 64, 67, 72, 76};
            for (const int n : notes)
                engine.noteOn(n, 1.0f);
            const auto run = [&](int blocks)
            {
                for (int b = 0; b < blocks; ++b)
                {
                    engine.render(l.data(), r.data(), 32);
                    out.insert(out.end(), l.begin(), l.end());
                }
            };
            run(500);
            const size_t cut = out.size();
            engine.noteOff(55);
            engine.noteOff(55); // a second off for the same key does nothing further
            engine.noteOff(61); // a key that was never pressed
            run(900);
            expectEquals(engine.activeVoiceCount(), 7);
            for (const int n : notes)
            {
                const double before = magnitudeAt(out, cut - 8000, cut, midiToHz(n));
                const double after = magnitudeAt(out, out.size() - 8000, out.size(), midiToHz(n));
                if (n == 55)
                    expect(after < 0.02 * before,
                           "released note gone " + std::to_string(after) + " / " + std::to_string(before));
                else
                    expect(after > 0.8 * before && after < 1.2 * before, "note " + std::to_string(n));
            }
        }

        beginTest("sustain pedal holds released notes and releases them with the pedal");
        {
            auto v = sinePatch();
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, v);
            std::vector<float> l(32), r(32), out;
            const auto run = [&](int blocks)
            {
                for (int b = 0; b < blocks; ++b)
                {
                    engine.render(l.data(), r.data(), 32);
                    out.insert(out.end(), l.begin(), l.end());
                }
            };
            engine.setSustainPedal(true);
            engine.noteOn(60, 1.0f);
            engine.noteOn(64, 1.0f);
            run(200);
            engine.noteOff(60);
            run(800);
            expectEquals(engine.activeVoiceCount(), 2);
            expect(magnitudeAt(out, out.size() - 6000, out.size(), midiToHz(60)) > 0.1, "held by pedal");
            engine.setSustainPedal(false);
            run(800);
            expectEquals(engine.activeVoiceCount(), 1);
            expect(magnitudeAt(out, out.size() - 6000, out.size(), midiToHz(60)) < 0.01, "released with pedal");
            expect(magnitudeAt(out, out.size() - 6000, out.size(), midiToHz(64)) > 0.1, "other still held");
            // Re-pressing a pedal-held key does not leave two coherent voices.
            engine.setSustainPedal(true);
            engine.noteOn(64, 1.0f);
            engine.noteOff(64);
            run(100);
            engine.noteOn(64, 1.0f);
            run(800);
            expectEquals(engine.activeVoiceCount(), 1);
        }

        beginTest("a full load of 16 notes x 8 unison x 3 oscillators renders well inside the block budget");
        {
            auto v = ParameterMap::defaultValues();
            for (const char* o : {"osc1", "osc2", "osc3"})
            {
                setValue(v, std::string(o) + "_level", 0.5f);
                setValue(v, std::string(o) + "_uni_voices", 8.0f);
                setValue(v, std::string(o) + "_pos", 1.0f);
            }
            setValue(v, "filter2_on", 1.0f);
            EngineConfig cfg;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            prepare(engine, v);
            for (int n = 0; n < 16; ++n)
                engine.noteOn(40 + 2 * n, 0.9f);
            std::vector<float> l(512), r(512);
            engine.render(l.data(), r.data(), 512); // warm-up
            double worst = 0.0, total = 0.0;
            const int blocks = 200;
            for (int b = 0; b < blocks; ++b)
            {
                const auto t0 = std::chrono::steady_clock::now();
                engine.render(l.data(), r.data(), 512);
                const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                worst = std::max(worst, ms);
                total += ms;
            }
            const double budget = 512.0 / kRate * 1000.0;
            std::printf(
                "    16 notes x 8 unison x 3 osc: mean %.2f ms, worst %.2f ms per 512-sample block (budget %.1f ms)\n",
                total / blocks, worst, budget);
            expect(std::isfinite(l[0]));
#if !defined(__SANITIZE_ADDRESS__) && !defined(__SANITIZE_THREAD__)
#if defined(__has_feature)
#if !__has_feature(address_sanitizer) && !__has_feature(thread_sanitizer)
            expect(total / blocks < budget, "mean block time exceeds the real-time budget");
#endif
#else
            expect(total / blocks < budget, "mean block time exceeds the real-time budget");
#endif
#endif
        }
    }
};

static PolyphonyTests polyphonyTests;
