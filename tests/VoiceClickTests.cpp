// SPDX-License-Identifier: MIT
// Click-free voice handling: a mono / legato retrigger of a still-sounding voice keeps the waveform continuous, a
// stolen voice fades instead of being cut, and kill-all fades every voice out quickly and silences the engine.

#include "AllocCounter.h"
#include "GraphSupport.h"

#include <crucible/Engine.h>

using namespace crucible;
using namespace crucible::testing;

namespace
{
std::vector<float> sinePatch(float release)
{
    auto v = graphPatch();
    setValue(v, "osc1_pos", 0.0f); // sine
    setValue(v, "osc1_level", 0.5f);
    setValue(v, "amp_sustain", 1.0f);
    setValue(v, "amp_release", release);
    return v;
}

struct Renderer
{
    std::vector<float> l = std::vector<float>(8), r = std::vector<float>(8), out;
    SynthEngine& engine;
    explicit Renderer(SynthEngine& e) : engine(e) { out.reserve(1 << 20); }
    void run(int samples)
    {
        for (int done = 0; done < samples; done += 8)
        {
            engine.render(l.data(), r.data(), 8);
            out.insert(out.end(), l.begin(), l.end());
        }
    }
};

double maxStep(const std::vector<float>& s, size_t from, size_t to)
{
    double m = 0.0;
    for (size_t i = std::max<size_t>(from, 1); i < std::min(to, s.size()); ++i)
        m = std::max(m, static_cast<double>(std::abs(s[i] - s[i - 1])));
    return m;
}

double maxSecondDiff(const std::vector<float>& s, size_t from, size_t to)
{
    double m = 0.0;
    for (size_t i = std::max<size_t>(from, 2); i < std::min(to, s.size()); ++i)
        m = std::max(m, static_cast<double>(std::abs(s[i] - 2.0f * s[i - 1] + s[i - 2])));
    return m;
}

// Same key struck again and again while its previous strike is still in a long release.
double fastRepeatStep(float voiceMode, int gapSamples)
{
    auto v = sinePatch(2.0f);
    setValue(v, "voice_mode", voiceMode);
    CRUCIBLE_HEAP(SynthEngine, engine);
    prepare(engine, v);
    Renderer rd(engine);
    engine.noteOn(69, 1.0f);
    rd.run(8000);
    const double calm = maxStep(rd.out, 4000, 8000);
    const size_t start = rd.out.size();
    for (int i = 0; i < 12; ++i)
    {
        engine.noteOff(69);
        rd.run(gapSamples);
        engine.noteOn(69, 1.0f);
        rd.run(gapSamples);
    }
    return maxStep(rd.out, start, rd.out.size()) / calm;
}

// `count` distinct notes one block apart on a 16-voice engine; peak second difference against a 64-voice reference.
double stealRatio(int count, int spacing, bool release)
{
    auto v = sinePatch(3.0f);
    setValue(v, "osc1_level", 0.1f);
    const auto render = [&](int voices)
    {
        EngineConfig cfg;
        cfg.maxVoices = voices;
        CRUCIBLE_HEAP(SynthEngine, engine, cfg);
        prepare(engine, v);
        Renderer rd(engine);
        for (int n = 0; n < count; ++n)
        {
            engine.noteOn(36 + n * 2 % 40, 1.0f);
            rd.run(spacing / 2);
            if (release)
                engine.noteOff(36 + n * 2 % 40);
            rd.run(spacing - spacing / 2);
        }
        rd.run(2000);
        return rd.out;
    };
    const auto steal = render(16);
    const auto ref = render(64);
    return maxSecondDiff(steal, 0, steal.size()) / maxSecondDiff(ref, 0, ref.size());
}
} // namespace

class VoiceClickTests : public UnitTest
{
public:
    VoiceClickTests() : UnitTest("Voice clicks", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("fast same-key repeats on a mono or legato patch keep the waveform continuous");
        for (const float mode : {1.0f, 2.0f})
            for (const int gap : {240, 600, 1000})
            {
                const double ratio = fastRepeatStep(mode, gap);
                std::printf("    mode %.0f gap %d: worst step %.2f x the steady slope\n", mode, gap, ratio);
                expect(ratio < 3.0,
                       "mode " + std::to_string(mode) + " gap " + std::to_string(gap) + ": " + std::to_string(ratio));
            }

        beginTest("20 overlapping released notes on 16 voices steal without a click");
        {
            const double ratio = stealRatio(20, 100, true);
            std::printf("    20 notes: peak second difference %.2f x the unstolen render\n", ratio);
            expect(ratio < 3.0, "ratio " + std::to_string(ratio));
        }

        beginTest("a burst of 40 notes exhausts the spare voices; the hard steals still fade");
        for (const bool rel : {false, true})
        {
            const double ratio = stealRatio(40, 8, rel);
            std::printf("    burst of 40 (released %d): peak second difference %.2f x the unstolen render\n", rel ? 1 : 0,
                        ratio);
            expect(ratio < 3.0, "ratio " + std::to_string(ratio));
        }

        beginTest("kill-all fades every voice out in a few ms, clears sustain and the held stack, and allocates nothing");
        {
            auto v = sinePatch(3.0f);
            setValue(v, "osc1_level", 0.1f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, v);
            Renderer rd(engine);
            for (const int n : {48, 55, 60, 64, 69})
                engine.noteOn(n, 1.0f);
            rd.run(4000);
            engine.setSustainPedal(true);
            engine.noteOff(48);
            engine.noteOff(55);
            rd.run(4000);
            const double calm = maxSecondDiff(rd.out, 2000, 4000);
            const size_t at = rd.out.size();
            gAllocations = 0;
            gTrackAllocations = true;
            engine.killAllVoices();
            rd.run(2000);
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "kill-all allocations");
            expectEquals(engine.activeVoiceCount(), 0);
            double tail = 0.0;
            for (size_t i = at + 300; i < rd.out.size(); ++i)
                tail = std::max(tail, static_cast<double>(std::abs(rd.out[i])));
            expect(tail == 0.0, "silent 300 samples (6.3 ms) after kill-all: " + std::to_string(tail));
            expect(maxSecondDiff(rd.out, at, rd.out.size()) < 3.0 * calm, "kill-all fade is click free");
            // Pedal and held keys are gone: a new note sounds and its release is not held by the old pedal.
            engine.noteOn(60, 1.0f);
            rd.run(2000);
            engine.noteOff(60);
            rd.run(160000);
            expectEquals(engine.activeVoiceCount(), 0);
        }

        beginTest("Engine::killAllVoices is queued like note input and allocates nothing in process()");
        {
            Engine e;
            e.prepare(48000.0, 256);
            std::vector<float> l(256), r(256);
            e.noteOn(60, 1.0f);
            e.process(l.data(), r.data(), 256);
            gAllocations = 0;
            gTrackAllocations = true;
            const bool queued = e.killAllVoices();
            e.process(l.data(), r.data(), 256);
            e.process(l.data(), r.data(), 256);
            gTrackAllocations = false;
            expect(queued);
            expectEquals(static_cast<int>(gAllocations), 0, "process allocations");
            expectEquals(e.activeVoiceCount(), 0);
        }
    }
};

static VoiceClickTests voiceClickTests;
