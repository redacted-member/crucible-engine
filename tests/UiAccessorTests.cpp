// SPDX-License-Identifier: MIT
#include <crucible/Clips.h>
#include <crucible/Cloud.h>
#include <crucible/Engine.h>
#include <crucible/Patch.h>
#include <crucible/Sample.h>
#include <crucible/UserWavetables.h>
#include "TestSupport.h"

#include <atomic>
#include <thread>

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr double kTwoPi = 6.283185307179586;

AudioData sineAudio(int frames, double freq, double rate = 48000.0)
{
    AudioData a;
    a.sampleRate = rate;
    a.channels.assign(1, std::vector<float>(static_cast<size_t>(frames)));
    for (int i = 0; i < frames; ++i)
        a.channels[0][static_cast<size_t>(i)] = static_cast<float>(0.8 * std::sin(kTwoPi * freq * i / rate));
    return a;
}

Clip rampClip(ClipTrigger trigger)
{
    Clip c;
    c.name = "ramp";
    c.length = 4.0;
    c.loopEnd = 4.0;
    c.trigger = trigger;
    ClipLane l;
    l.points = {{0.0, 0.0f, 0.0f, SegmentShape::Curve}, {4.0, 1.0f, 0.0f, SegmentShape::Curve}};
    c.lanes.push_back(l);
    c.sanitise();
    return c;
}

void process(EngineCore& core, const std::vector<float>& v, int blocks, const MidiInput* midi = nullptr, int numMidi = 0)
{
    std::vector<float> l(512), r(512);
    for (int b = 0; b < blocks; ++b)
        core.process(l.data(), r.data(), 512, v.data(), b == 0 ? midi : nullptr, b == 0 ? numMidi : 0, {});
}

} // namespace

class UiAccessorTests : public UnitTest
{
public:
    UiAccessorTests() : UnitTest("UI accessors for editors", "Engine") {}

    void runTest() override
    {
        beginTest("E2: clip playheads are published and readable from another thread");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            ClipBank bank;
            bank.clips = {rampClip(ClipTrigger::GlobalFree), rampClip(ClipTrigger::Voice)};
            core.engine().clips().replace(bank);
            auto v = ParameterMap::defaultValues();
            process(core, v, 4);
            std::array<double, kMaxClips> pos{};
            core.engine().clipPositions(pos);
            expect(pos[0] > 0.0, "free clip advances " + String(pos[0]));
            std::vector<double> voicePos;
            core.engine().voiceClipPositions(1, voicePos);
            expect(voicePos.size() >= static_cast<size_t>(kNumVoices));
            expect(std::all_of(voicePos.begin(), voicePos.end(), [](double p) { return p < 0.0; }),
                   "no note, no voice head");
            MidiInput on{0, 0x90, 60, 100};
            process(core, v, 6, &on, 1);
            core.engine().voiceClipPositions(1, voicePos);
            expect(std::count_if(voicePos.begin(), voicePos.end(), [](double p) { return p >= 0.0; }) >= 1,
                   "voice head runs");

            // A reader thread polls every accessor while the audio thread renders (ThreadSanitizer checks the races).
            std::atomic<bool> stop{false};
            std::thread reader(
                [&]
                {
                    std::array<double, kMaxClips> p{};
                    std::vector<double> vp;
                    std::array<std::uint64_t, 2> notes{};
                    while (!stop.load())
                    {
                        core.engine().clipPositions(p);
                        core.engine().voiceClipPositions(1, vp);
                        core.engine().voiceModulatedValue(ParameterMap::indexOf("osc1_pos"));
                        core.soundingNotes(notes);
                    }
                });
            process(core, v, 200);
            stop = true;
            reader.join();
        }

        beginTest("E3: the voice modulated value follows an LFO route that modulatedValue leaves alone");
        {
            Engine e;
            e.prepare(kRate, 512);
            e.setParam("lfo1_rate", 8.0f);
            e.setParam("osc1_pos", 0.5f);
            expect(e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "osc1_pos", 0.4f, 0, 0.0f, 0.0f}));
            const int idx = ParameterMap::indexOf("osc1_pos");
            expect(e.coreEngine().engine().voiceModulatedValue(idx) == e.coreEngine().engine().modulatedValue(idx), "idle");
            e.noteOn(60, 1.0f);
            std::vector<float> l(256), r(256);
            float lo = 9.0f, hi = -9.0f;
            for (int b = 0; b < 400; ++b)
            {
                e.process(l.data(), r.data(), 256);
                const float m = e.coreEngine().engine().voiceModulatedValue(idx);
                lo = std::min(lo, m);
                hi = std::max(hi, m);
                expect(std::abs(e.coreEngine().engine().modulatedValue(idx) - 0.5f) < 1.0e-4f);
            }
            expect(hi - lo > 0.3f, "swing " + String(hi - lo, 3));
            expect(lo >= 0.0f && hi <= 1.0f);
            // A parameter without voice modulation reads back unchanged.
            const int cut = ParameterMap::indexOf("filter_cutoff");
            expect(e.coreEngine().engine().voiceModulatedValue(cut) == e.coreEngine().engine().modulatedValue(cut));
        }

        beginTest("E4: frame detection reasons and cycle length estimate");
        {
            AudioData a = sineAudio(1024, 100.0);
            a.cycleLength = 256;
            auto d = detectWavetableFrames(a, {});
            expect(d.reason == FrameDetection::Reason::ClmMarker && d.frameSize == 256 && d.frames == 4);
            d = detectWavetableFrames(sineAudio(4096, 100.0), {});
            expect(d.reason == FrameDetection::Reason::Multiple2048 && d.frameSize == 2048 && d.frames == 2);
            d = detectWavetableFrames(sineAudio(1000, 100.0), {});
            expect(d.reason == FrameDetection::Reason::SingleCycle && d.frameSize == 1000 && d.frames == 1);
            d = detectWavetableFrames(sineAudio(10000, 100.0), {});
            expect(d.reason == FrameDetection::Reason::Fallback && d.frameSize == 2048 && d.frames == 4);
            WavetableImport forced;
            forced.frameSize = 512;
            d = detectWavetableFrames(a, forced);
            expect(d.reason == FrameDetection::Reason::Forced && d.frameSize == 512 && d.frames == 2);
            for (const auto& audio : {a, sineAudio(10000, 100.0), sineAudio(1000, 100.0)})
                expectEquals(static_cast<int>(sliceCycles(audio, {}).size()), detectWavetableFrames(audio, {}).frames);
            const int len = estimateCycleLength(sineAudio(16384, 220.0));
            expect(std::abs(len - 218) <= 2, "cycle " + String(len));
            AudioData silent;
            silent.channels.assign(1, std::vector<float>(16384, 0.0f));
            expectEquals(estimateCycleLength(silent), 0);
        }

        beginTest("E5/E6: capturePatch(EngineCore) equals capturePatch(Engine); content round trips");
        {
            Engine e;
            e.prepare(kRate, 512);
            e.setParam("filter_cutoff", 900.0f);
            ClipBank clips;
            clips.clips = {rampClip(ClipTrigger::Voice)};
            e.clips().replace(clips);
            e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "osc1_pos", 0.4f, 0, 0.0f, 0.0f});
            e.arpPattern().edit([](ArpPattern& p) { p.steps[2].on = false; });
            CRUCIBLE_HEAP(SeqBank, seq);
            seq.pattern[0].steps[3] = {true, 50, 100, 1, 0.5f, 1.0f, {}};
            e.sequencer().replace(seq);
            CloudModel cloud = CloudModel::random(40, 7);
            cloud.name = "mine";
            e.setCloud(2, cloud);
            std::string error;
            expect(e.loadSample(1, sineAudio(5000, 300.0), "sine", error), error);
            e.userWavetables().loadCycles(0, {std::vector<float>(256, 0.25f)}, WavetableImport::Normalise::Table, "flat");
            e.storeScene(0);

            PatchMeta meta;
            meta.name = "x";
            Patch viaEngine = capturePatch(e, meta);
            viaEngine.ccMap.clear();
            Patch viaCore = capturePatch(e.coreEngine(), e.getAllParams(), &e.scenes(), meta);
            expect(patchToJson(viaEngine) == patchToJson(viaCore), "same document");
            expect(viaCore.assets.size() == 2 && viaCore.clips.clips.size() == 1 && !viaCore.userWaves.empty());

            CRUCIBLE_HEAP(EngineCore, other);
            other.prepare(kRate);
            applyPatchContent(other, viaCore);
            const Patch again = capturePatch(other, e.getAllParams(), &e.scenes(), meta);
            expect(patchToJson(again) == patchToJson(viaCore), "apply then capture is identical");

            // Applying an empty patch clears the content again.
            applyPatchContent(other, Patch{});
            const Patch cleared = capturePatch(other, e.getAllParams(), nullptr, meta);
            expect(cleared.assets.empty() && cleared.clips.clips.empty() && cleared.userWaves.empty() &&
                   cleared.routes.empty());

            // Bank level helpers used by the plugin state.
            CloudBank clouds;
            clouds.set(5, cloud);
            const auto records = clouds.serialise();
            expect(records.size() == 1 && records[0].slot == 5);
            CloudBank restored;
            restored.set(1, cloud);
            restored.deserialise(records);
            expect(restored.isUser(5) && !restored.isUser(1) && restored.model(5)->points.size() == 40);
            SampleBank samples;
            expect(samples.load(3, sineAudio(3000, 200.0), "s3", error), error);
            const auto sr = samples.serialise();
            expect(sr.size() == 1 && sr[0].slot == 3 && sr[0].frames.size() == 3000 && sr[0].name == "s3");
            SampleBank samples2;
            samples2.deserialise(sr);
            expect(samples2.isLoaded(3) && samples2.sample(3)->frames == 3000);
            std::vector<float> f = {0.5f, -1.25f, 3.0f};
            std::vector<float> back;
            expect(json::base64ToFloats(json::floatsToBase64(f), back) && back == f);
        }

        beginTest("E7: sounding notes include arp and sequencer output after Key Lock");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            auto v = ParameterMap::defaultValues();
            setValue(v, "arp_on", 1.0f);
            setValue(v, "arp_sync", 0.0f);
            setValue(v, "arp_mode", 0.0f);
            std::array<std::uint64_t, 2> notes{};
            core.soundingNotes(notes);
            expect(notes[0] == 0 && notes[1] == 0);
            MidiInput midi[2] = {{0, 0x90, 60, 100}, {0, 0x90, 67, 100}};
            std::uint64_t seen0 = 0, seen1 = 0;
            std::vector<float> l(512), r(512);
            for (int b = 0; b < 60; ++b)
            {
                core.process(l.data(), r.data(), 512, v.data(), b == 0 ? midi : nullptr, b == 0 ? 2 : 0, {});
                core.soundingNotes(notes);
                seen0 |= notes[0];
                seen1 |= notes[1];
            }
            expect((seen0 & (std::uint64_t{1} << 60)) != 0 && (seen1 & (std::uint64_t{1} << 3)) != 0, "arp notes 60 and 67");
            // Key Lock to C major moves the sequencer's C#3 to the note that actually sounds.
            setValue(v, "arp_on", 0.0f);
            setValue(v, "key_mode", 1.0f);
            setValue(v, "key_root", 0.0f);
            setValue(v, "key_scale", 1.0f);
            setValue(v, "key_tie", 0.0f);
            setValue(v, "seq_run", 1.0f);
            setValue(v, "seq_sync", 0.0f);
            CRUCIBLE_HEAP(SeqBank, bank);
            bank.pattern[0].steps[0] = {true, 49, 100, 1, 0.5f, 1.0f, {}};
            core.store().replace(bank);
            std::uint64_t seq1 = 0;
            for (int b = 0; b < 200; ++b)
            {
                core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
                core.soundingNotes(notes);
                seq1 |= notes[0];
            }
            expect((seq1 & (std::uint64_t{1} << 49)) == 0 && (seq1 & (std::uint64_t{1} << 48)) != 0,
                   "quantised sequencer note sounds, not the raw one");
        }

        beginTest("E9: zero crossings and loop points on a sine");
        {
            const double freq = 440.0;
            const int frames = 48000;
            const auto sample = SampleData::build(sineAudio(frames, freq), "sine");
            // Phase (in cycles) at a normalised position; a rising zero crossing has an integer phase.
            const auto phaseError = [&](double pos01)
            {
                const double ph = pos01 * frames * freq / 48000.0;
                return std::abs(ph - std::round(ph));
            };
            for (const double p : {0.1234, 0.5, 0.777})
            {
                const double any = nearestZeroCrossing(*sample, p, 0);
                const double before = nearestZeroCrossing(*sample, p, -1);
                const double after = nearestZeroCrossing(*sample, p, 1);
                expect(phaseError(any) < 0.01 && phaseError(before) < 0.01 && phaseError(after) < 0.01, "on a crossing");
                expect(before <= p && after >= p && std::abs(any - p) <= std::abs(after - before) / 2.0 + 1.0e-9);
                expect(after - before < 1.2 / freq * 48000.0 / frames + 1.0e-9, "one period apart");
            }
            double a = 0.0, b = 0.0;
            expect(findLoopPoints(*sample, 0.2013, 0.7011, a, b));
            expect(phaseError(a) < 0.01 && phaseError(b) < 0.01, "loop points on crossings");
            expect(std::abs(a - 0.2013) < 0.05 * 0.5 && std::abs(b - 0.7011) < 0.05 * 0.5 && b > a);
            const double periods = (b - a) * frames * freq / 48000.0;
            expect(std::abs(periods - std::round(periods)) < 0.02, "whole number of periods");
            expect(!findLoopPoints(*sample, 0.5, 0.5, a, b));
        }
    }
};

static UiAccessorTests uiAccessorTests;
