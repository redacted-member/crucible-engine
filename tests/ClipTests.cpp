// SPDX-License-Identifier: MIT
#include <crucible/Clips.h>
#include <crucible/Engine.h>
#include <crucible/MotionRecorder.h>
#include <crucible/Patch.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
ClipLane rampLane(double length)
{
    ClipLane l;
    l.points = {{0.0, 0.0f, 0.0f, SegmentShape::Curve}, {length, 1.0f, 0.0f, SegmentShape::Curve}};
    return l;
}

Clip rampClip(ClipTrigger trigger, double beats = 4.0, ClipPlayMode mode = ClipPlayMode::Loop)
{
    Clip c;
    c.name = "ramp";
    c.length = beats;
    c.loopEnd = beats;
    c.trigger = trigger;
    c.mode = mode;
    c.lanes.push_back(rampLane(beats));
    c.sanitise();
    return c;
}

// Runs a player for `chunks` control chunks at a constant tempo.
void run(ClipPlayer& p, const ClipBank& bank, int chunks, double bpm, bool held = false)
{
    for (int i = 0; i < chunks; ++i)
    {
        p.beginChunk(&bank, 32, bpm, held);
        p.endChunk();
    }
}

float rmsOf(const std::vector<float>& x, size_t from, size_t to)
{
    double sum = 0.0;
    for (size_t i = from; i < to; ++i)
        sum += static_cast<double>(x[i]) * x[i];
    return static_cast<float>(std::sqrt(sum / static_cast<double>(to - from)));
}
} // namespace

class ClipTests : public UnitTest
{
public:
    ClipTests() : UnitTest("Automation clips and motion recorder", "Engine") {}

    void runTest() override
    {
        beginTest("lanes: segment shapes, steps, edges");
        {
            Clip c = rampClip(ClipTrigger::GlobalFree);
            const auto& lane = c.lanes[0];
            expectWithinAbsoluteError(evaluateLane(c, lane, 1.0, 0), 0.25f, 1.0e-6f, "linear");
            expectWithinAbsoluteError(evaluateLane(c, lane, -1.0, 0), 0.0f, 1.0e-6f, "before start");
            expectWithinAbsoluteError(evaluateLane(c, lane, 9.0, 0), 1.0f, 1.0e-6f, "after end");
            ClipLane shaped;
            shaped.points = {{0.0, 0.0f, 0.0f, SegmentShape::Hold},
                             {1.0, 1.0f, 0.0f, SegmentShape::SCurve},
                             {2.0, 0.0f, 0.5f, SegmentShape::Curve},
                             {3.0, 1.0f, 0.0f, SegmentShape::Stairs},
                             {4.0, 0.0f, 0.0f, SegmentShape::Curve}};
            expectWithinAbsoluteError(evaluateLane(c, shaped, 0.9, 0), 0.0f, 1.0e-6f, "hold keeps the start value");
            expectWithinAbsoluteError(evaluateLane(c, shaped, 1.5, 0), 0.5f, 1.0e-4f, "s-curve midpoint");
            expect(evaluateLane(c, shaped, 1.25, 0) > 0.75f, "s-curve eases in");
            const float bent = evaluateLane(c, shaped, 2.5, 0);
            expect(bent >= 0.0f && bent <= 1.0f && bent != 0.5f, "curve evaluates");
            expect(segmentShape(SegmentShape::Curve, 0.8f, 0.5f) < 0.5f &&
                       segmentShape(SegmentShape::Curve, -0.8f, 0.5f) > 0.5f,
                   "curve bends both ways");
            // Stairs: values come in discrete levels.
            std::set<int> levels;
            for (int i = 0; i < 100; ++i)
                levels.insert(static_cast<int>(std::lround(evaluateLane(c, shaped, 3.0 + i * 0.0099, 0) * 1000.0f)));
            expect(levels.size() >= 2 && levels.size() <= 17,
                   "stairs quantise the segment: " + String(static_cast<int>(levels.size())));

            ClipLane steps;
            steps.kind = ClipLaneKind::Steps;
            steps.steps = {0.0f, 0.25f, 0.5f, 1.0f};
            c.length = 4.0;
            expectWithinAbsoluteError(evaluateLane(c, steps, 0.5, 0), 0.0f, 1.0e-6f);
            expectWithinAbsoluteError(evaluateLane(c, steps, 1.5, 0), 0.25f, 1.0e-6f);
            expectWithinAbsoluteError(evaluateLane(c, steps, 3.99, 0), 1.0f, 1.0e-6f);
            steps.stepGlide = 1.0f;
            expectWithinAbsoluteError(evaluateLane(c, steps, 1.5, 0), 0.375f, 1.0e-5f, "glide halfway into the next step");
        }

        beginTest("lanes: humanise and randomise are deterministic, bounded and vary per cycle");
        {
            Clip c = rampClip(ClipTrigger::GlobalFree);
            ClipLane l = c.lanes[0];
            l.randomise = 0.5f;
            l.humanise = 1.0f;
            l.seed = 7;
            const float a = evaluateLane(c, l, 2.0, 0);
            expectEquals(a, evaluateLane(c, l, 2.0, 0), "same cycle, same value");
            bool differs = false;
            for (std::uint32_t cyc = 1; cyc < 8; ++cyc)
                differs = differs || evaluateLane(c, l, 2.0, cyc) != a;
            expect(differs, "later cycles differ");
            bool inRange = true;
            for (int i = 0; i < 400; ++i)
            {
                const float v = evaluateLane(c, l, i * 0.01, static_cast<std::uint32_t>(i % 5));
                inRange = inRange && v >= 0.0f && v <= 1.0f && std::isfinite(v);
            }
            expect(inRange, "stays within 0..1");
            ClipLane plain = c.lanes[0];
            expectEquals(evaluateLane(c, plain, 2.0, 0), evaluateLane(c, plain, 2.0, 9), "off means no variation");
        }

        beginTest("timeline: one-shot, loop wrap, ping-pong, sustain loop");
        {
            std::uint32_t cyc = 0;
            Clip c = rampClip(ClipTrigger::Voice, 4.0, ClipPlayMode::OneShot);
            expectWithinAbsoluteError(clipPosition(c, 9.0, -1.0, cyc), 4.0, 1.0e-12, "one-shot holds the end");
            c.mode = ClipPlayMode::Loop;
            c.loopStart = 1.0;
            c.loopEnd = 3.0;
            expectWithinAbsoluteError(clipPosition(c, 2.5, -1.0, cyc), 2.5, 1.0e-12, "lead-in and first pass are linear");
            expectWithinAbsoluteError(clipPosition(c, 3.5, -1.0, cyc), 1.5, 1.0e-12, "wraps to the loop start");
            expectEquals(static_cast<int>(cyc), 1, "first repeat is cycle 1");
            expectWithinAbsoluteError(clipPosition(c, 2.5, -1.0, cyc), 2.5, 1.0e-12);
            expectEquals(static_cast<int>(cyc), 0, "lead-in and first pass are cycle 0");
            expectWithinAbsoluteError(clipPosition(c, 5.5, -1.0, cyc), 1.5, 1.0e-12);
            expectEquals(static_cast<int>(cyc), 2, "second repeat is cycle 2");
            c.mode = ClipPlayMode::PingPong;
            expectWithinAbsoluteError(clipPosition(c, 3.5, -1.0, cyc), 2.5, 1.0e-12, "ping-pong reverses");
            expectWithinAbsoluteError(clipPosition(c, 5.5, -1.0, cyc), 1.5, 1.0e-12, "and forwards again");
            // Loop wrap is exact even after a very long time.
            c.mode = ClipPlayMode::Loop;
            const double far = 3600.0 * 24.0 * 2.0;
            const double pos = clipPosition(c, 1.0 + far * 2.0 + 0.25, -1.0, cyc);
            expectWithinAbsoluteError(pos, 1.25, 1.0e-6, "exact after 2 days of beats");

            c = makeMseg({{0.0, 0.0f}, {1.0, 1.0f}, {2.0, 0.5f}, {4.0, 0.0f}}, 4.0, true, 1.0, 2.0);
            expect(c.sustainLoop, "mseg sustain loop");
            expectWithinAbsoluteError(clipPosition(c, 10.25, -1.0, cyc), 1.25, 1.0e-12, "loops while held");
            expectWithinAbsoluteError(clipPosition(c, 10.25, 10.0, cyc), 1.0 + 0.25 + 0.0 + 0.0, 0.75 + 1.0e-9,
                                      "after release it plays on");
            expect(clipPosition(c, 10.0 + 5.0, 10.0, cyc) >= 3.99, "and ends at the clip end");
        }

        beginTest("playhead: tempo changes alter speed without jumps; wraps stay exact");
        {
            ClipBank bank;
            bank.clips.push_back(rampClip(ClipTrigger::GlobalFree, 4.0));
            ClipPlayer p;
            p.prepare(kRate, 2);
            run(p, bank, 750, 120.0); // 24000 samples at 120 bpm = 1 beat
            expectWithinAbsoluteError(p.globalPosition(0), 1.0, 1.0e-9, "one beat at 120");
            run(p, bank, 375, 240.0); // 12000 samples at 240 bpm = 1 beat
            expectWithinAbsoluteError(p.globalPosition(0), 2.0, 1.0e-9, "another beat at 240");
            run(p, bank, 1, 120.0);
            const double step = p.globalPosition(0) - 2.0;
            expectWithinAbsoluteError(step, 32.0 / kRate * 2.0, 1.0e-9, "no jump at the change");
            // Free clip counts seconds.
            bank.clips[0].sync = false;
            bank.clips[0].length = bank.clips[0].loopEnd = 2.0;
            bank.clips[0].lanes[0] = rampLane(2.0);
            ClipPlayer q;
            q.prepare(kRate, 1);
            run(q, bank, 1500, 77.0); // 1 second
            expectWithinAbsoluteError(q.globalPosition(0), 1.0, 1.0e-9, "seconds ignore the tempo");
            run(q, bank, 2250, 200.0);
            expectWithinAbsoluteError(q.globalPosition(0), 0.5, 1.0e-6, "wrapped: 2.5 s in a 2 s loop");
            expect(q.globalPosition(0) >= 0.0 && q.globalPosition(0) < 2.0, "inside the loop");
        }

        beginTest("transport clips follow the host position and tempo, and free-run while stopped");
        {
            ClipBank bank;
            bank.clips.push_back(rampClip(ClipTrigger::Transport, 4.0));
            ClipPlayer p;
            p.prepare(kRate, 1);
            p.setHostTransport(true, true, 6.5);
            p.beginChunk(&bank, 32, 120.0, false);
            expectWithinAbsoluteError(p.globalPosition(0), 2.5, 1.0e-9, "6.5 beats into a 4 beat loop");
            p.endChunk();
            p.beginChunk(&bank, 32, 120.0, false);
            expectWithinAbsoluteError(p.globalPosition(0), 2.5 + 32.0 / kRate * 2.0, 1.0e-9, "advances with the tempo");
            p.endChunk();
            p.setHostTransport(true, true, 1.0); // the host jumped back
            p.beginChunk(&bank, 32, 120.0, false);
            expectWithinAbsoluteError(p.globalPosition(0), 1.0, 1.0e-9, "relocates with the host");
            p.endChunk();
            p.setHostTransport(true, false, 1.0);
            const double stopped = p.globalPosition(0);
            run(p, bank, 10, 120.0);
            expect(p.globalPosition(0) > stopped, "keeps running while the host is stopped");
            // Over a long run the position stays locked to the host clock.
            p.setHostTransport(true, true, 0.0);
            double hostPpq = 0.0;
            for (int i = 0; i < 4000; ++i)
            {
                p.setHostTransport(true, true, hostPpq);
                p.beginChunk(&bank, 32, 133.0, false);
                p.endChunk();
                hostPpq += 32.0 / kRate * 133.0 / 60.0;
            }
            p.setHostTransport(true, true, hostPpq);
            p.beginChunk(&bank, 32, 133.0, false);
            expectWithinAbsoluteError(p.globalPosition(0), std::fmod(hostPpq, 4.0), 1.0e-9, "locked to the host");
        }

        beginTest("voice playheads: each voice restarts on its own note; release is per voice");
        {
            ClipBank bank;
            bank.clips.push_back(makeMseg({{0.0, 0.0f}, {2.0, 1.0f}, {4.0, 0.0f}}, 4.0, true, 1.0, 3.0));
            ClipPlayer p;
            p.prepare(kRate, 2);
            p.beginChunk(&bank, 32, 120.0, true);
            p.beginVoice(0, 1, true, false);
            p.beginVoice(1, 0, false, false);
            p.endChunk();
            for (int i = 0; i < 900; ++i)
            {
                p.beginChunk(&bank, 32, 120.0, true);
                p.beginVoice(0, 1, true, false);
                p.beginVoice(1, 0, false, false);
                p.endChunk();
            }
            p.beginChunk(&bank, 32, 120.0, true);
            p.beginVoice(0, 1, true, false);
            p.beginVoice(1, 2, true, false); // second note, new age
            const int src = clipLaneSource(0, 0);
            const float v0 = p.voiceValue(0, src);
            const float v1 = p.voiceValue(1, src);
            expect(v0 > 0.4f, "first voice has advanced");
            expectWithinAbsoluteError(v1, 0.0f, 1.0e-6f, "second voice starts at the beginning");
            p.endChunk();
            p.beginChunk(&bank, 32, 120.0, true);
            p.beginVoice(0, 3, true, false); // voice 0 is stolen by a new note
            p.beginVoice(1, 2, true, false);
            expectWithinAbsoluteError(p.voiceValue(0, src), 0.0f, 1.0e-6f, "a retriggered voice restarts");
            expect(p.voiceValue(1, src) > 0.0f, "other voice unaffected");
            p.endChunk();
            // Release: the sustain loop keeps looping while held and ends on release.
            for (int i = 0; i < 3000; ++i)
            {
                p.beginChunk(&bank, 32, 120.0, true);
                p.beginVoice(1, 2, true, false);
                p.endChunk();
            }
            p.beginChunk(&bank, 32, 120.0, true);
            p.beginVoice(1, 2, true, false);
            const float held = p.voiceValue(1, src);
            p.endChunk();
            for (int i = 0; i < 3000; ++i)
            {
                p.beginChunk(&bank, 32, 120.0, false);
                p.beginVoice(1, 2, true, true);
                p.endChunk();
            }
            p.beginChunk(&bank, 32, 120.0, false);
            p.beginVoice(1, 2, true, true);
            expectWithinAbsoluteError(p.voiceValue(1, src), 0.0f, 1.0e-4f, "plays on to the end after release");
            expect(held >= 0.0f && held <= 1.0f);
        }

        beginTest("lane smoothing slews the output");
        {
            ClipBank bank;
            Clip c;
            c.trigger = ClipTrigger::GlobalFree;
            c.length = c.loopEnd = 4.0;
            ClipLane l;
            l.kind = ClipLaneKind::Steps;
            l.steps = {0.0f, 1.0f};
            l.smoothingMs = 50.0f;
            c.lanes.push_back(l);
            bank.clips.push_back(c);
            ClipPlayer p;
            p.prepare(kRate, 1);
            const int src = clipLaneSource(0, 0);
            float prev = 0.0f;
            bool monotone = true;
            float atStep = 0.0f;
            for (int i = 0; i < 2500; ++i) // the step edge (beat 2 = 1 s) is at chunk 1500
            {
                p.beginChunk(&bank, 32, 120.0, false);
                const float v = p.globalValue(src);
                if (i == 1501)
                    atStep = v;
                monotone = monotone && v >= prev - 1.0e-6f;
                prev = v;
                p.endChunk();
            }
            expect(monotone && atStep > 0.0f && atStep < 0.5f, "rises smoothly after the step edge: " + String(atStep));
            expect(prev > 0.99f, "and settles");
        }

        beginTest("storage: sanitising, JSON and patch round trips");
        {
            ClipBank bank;
            Clip c = rampClip(ClipTrigger::Transport, 8.0, ClipPlayMode::PingPong);
            c.name = "wobble";
            c.sustainLoop = true;
            c.loopStart = 2.0;
            c.loopEnd = 6.0;
            c.rate = 2.0;
            c.lanes[0].name = "cutoff";
            c.lanes[0].points[0].shape = SegmentShape::Stairs;
            c.lanes[0].points[0].curve = 0.3f;
            c.lanes[0].smoothingMs = 12.0f;
            c.lanes[0].humanise = 0.2f;
            c.lanes[0].randomise = 0.1f;
            c.lanes[0].seed = 99;
            ClipLane s;
            s.kind = ClipLaneKind::Steps;
            s.name = "gate";
            s.steps = {1, 0, 1, 1, 0, 0, 1, 0};
            s.stepGlide = 0.25f;
            c.lanes.push_back(s);
            bank.clips.push_back(c);
            auto text = clipBankToJson(bank).dump();
            json::Value parsed;
            std::string error;
            expect(json::parse(text, parsed, error), error);
            ClipBank back;
            expect(clipBankFromJson(parsed, back));
            expect(clipBankToJson(back).dump() == text, "JSON survives a round trip unchanged");
            expectEquals(static_cast<int>(back.clips[0].lanes.size()), 2);
            expect(back.clips[0].trigger == ClipTrigger::Transport && back.clips[0].mode == ClipPlayMode::PingPong);

            Clip bad;
            bad.length = std::nan("");
            bad.rate = 1.0e9;
            bad.grid = -4.0;
            bad.loopStart = 100.0;
            bad.loopEnd = -3.0;
            ClipLane bl;
            bl.points = {{5.0e9, 7.0f, 4.0f, SegmentShape::Curve}, {-2.0, -1.0f, std::nanf(""), SegmentShape::Curve}};
            bl.smoothingMs = -5.0f;
            bad.lanes.assign(20, bl);
            bad.sanitise();
            expect(bad.lanes.size() == static_cast<size_t>(kMaxClipLanes) && bad.length > 0.0 && bad.rate <= 64.0, "limits");
            expect(bad.lanes[0].points[0].x <= bad.lanes[0].points[1].x && bad.lanes[0].points[1].y <= 1.0f,
                   "sorted and clamped");

            Engine e;
            e.prepare(kRate, 128);
            e.clips().replace(bank);
            e.modRoutes().add({clipLaneSource(0, 0), "fx_ra_size", 0.5f, 2, 0.1f, 10.0f});
            const Patch patch = capturePatch(e);
            const auto doc = patchToJson(patch);
            Patch loaded;
            expect(patchFromJson(doc, loaded, error), error);
            Engine f;
            f.prepare(kRate, 128);
            applyPatch(f, loaded);
            expect(clipBankToJson(f.clips().snapshot()).dump() == text, "clips restored from a patch");
            expect(f.modRoutes().routesFor("fx_ra_size").size() == 1 &&
                       f.modRoutes().routesFor("fx_ra_size")[0].source == clipLaneSource(0, 0),
                   "clip route restored");
            Patch plain;
            expect(patchFromJson(patchToJson(capturePatch(f)), plain, error) && plain.clips.clips.size() == 1);
            Engine g;
            g.prepare(kRate, 128);
            Patch noClips = capturePatch(g);
            expect(patchToJson(noClips).find("\"clips\"") == std::string::npos, "patches without clips stay unchanged");
        }

        beginTest("routes: clip lanes drive any parameter with amount, polarity, curve, smoothing; listed per parameter");
        {
            Engine e;
            e.prepare(kRate, 128);
            ClipBank bank;
            Clip c;
            c.trigger = ClipTrigger::GlobalFree;
            c.length = c.loopEnd = 4.0;
            c.mode = ClipPlayMode::OneShot;
            ClipLane flat;
            flat.points = {{0.0, 0.75f, 0.0f, SegmentShape::Curve}, {4.0, 0.75f, 0.0f, SegmentShape::Curve}};
            c.lanes.push_back(flat);
            bank.clips.push_back(c);
            e.clips().replace(bank);
            const int src = clipLaneSource(0, 0);
            expect(!e.modRoutes().add({clipLaneSource(0, 0) + 5000, "fx_ra_size", 1.0f}), "invalid source rejected");
            expect(e.modRoutes().add({src, "fx_ra_size", 0.4f}));
            const int idx = e.model().indexOf("fx_ra_size");
            const auto& def = ParameterMap::params()[static_cast<size_t>(idx)];
            e.setParam("fx_ra_size", def.min);
            std::vector<float> l(128), r(128);
            for (int i = 0; i < 20; ++i)
                e.process(l.data(), r.data(), 128);
            const float range = def.max - def.min;
            expectWithinAbsoluteError(e.modulatedValue("fx_ra_size"), def.min + 0.75f * 0.4f * range, range * 1.0e-3f,
                                      "unipolar: amount x lane");
            e.modRoutes().add({src, "fx_ra_size", 0.4f, static_cast<int>(ModPolarity::Bipolar), 0.0f, 0.0f});
            for (int i = 0; i < 20; ++i)
                e.process(l.data(), r.data(), 128);
            expectWithinAbsoluteError(e.modulatedValue("fx_ra_size"), def.min + (2 * 0.75f - 1) * 0.4f * range,
                                      range * 1.0e-3f, "bipolar: (2x - 1) x amount");
            expect(e.modRoutes().routesFor("fx_ra_size").size() == 1, "listed for the parameter");
            expectEquals(routeSourceName(src), std::string("Clip 1 Lane 1"));
            expect(isRouteSource(src) && !isRouteSource(3999) && !isRouteSource(kSourceModulator + 8));
            {
                const auto list = routeSourceList(bank);
                bool hasClip = false, hasCc = false, hasLfo = false, velVoiceOnly = false;
                for (const auto& s : list)
                {
                    hasClip = hasClip || s.id == src;
                    hasCc = hasCc || s.id == kSourceCC + 74;
                    hasLfo = hasLfo || (s.id == static_cast<int>(ModSource::Lfo1) && s.bipolar);
                    velVoiceOnly = velVoiceOnly || (s.id == static_cast<int>(ModSource::Velocity) && s.voiceOnly);
                }
                expect(hasClip && hasCc && hasLfo && velVoiceOnly,
                       "source list covers clips, CCs, LFOs and flags voice-only sources");
            }
            bank.clips[0].name = "Sweep";
            bank.clips[0].lanes[0].name = "Size";
            expectEquals(routeSourceName(src, bank), std::string("Sweep / Size"));
            // The automatable parameter is untouched by the modulation.
            expectWithinAbsoluteError(e.getParam("fx_ra_size"), def.min, 1.0e-6f, "host-visible value unchanged");
        }

        beginTest("per-voice clips: a note-triggered lane restarts with every note");
        {
            Engine e;
            e.prepare(kRate, 128);
            auto v = ParameterMap::defaultValues();
            setValue(v, "osc1_pos", 0.0f);
            setValue(v, "osc1_level", 0.0f);
            setValue(v, "osc2_level", 0.0f);
            setValue(v, "osc3_level", 0.0f);
            setValue(v, "amp_sustain", 1.0f);
            setValue(v, "amp_attack", 0.001f);
            setValue(v, "filter_cutoff", 20000.0f);
            setValue(v, "master_gain", 0.0f);
            for (size_t i = 0; i < v.size(); ++i)
                e.setParam(static_cast<int>(i), v[i]);
            ClipBank bank;
            Clip c = rampClip(ClipTrigger::Voice, 2.0, ClipPlayMode::OneShot);
            bank.clips.push_back(c);
            e.clips().replace(bank);
            e.modRoutes().add({clipLaneSource(0, 0), "osc1_level", 1.0f});
            SeqHostTime host;
            host.hasBpm = true;
            host.bpm = 120.0;
            std::vector<float> out, l(128), r(128);
            const auto block = [&](const MidiInput* m, int nm)
            {
                e.process(l.data(), r.data(), 128, m, nm, &host);
                out.insert(out.end(), l.begin(), l.end());
            };
            MidiInput on{0, 0x90, 57, 120};
            MidiInput off{0, 0x80, 57, 0};
            block(&on, 1);
            for (int i = 0; i < 374; ++i) // the ramp is 2 beats = 1 s = 48000 samples = 375 blocks
                block(nullptr, 0);
            const size_t firstEnd = out.size();
            block(&off, 1);
            for (int i = 0; i < 400; ++i)
                block(nullptr, 0);
            const size_t secondStart = out.size();
            block(&on, 1);
            for (int i = 0; i < 374; ++i)
                block(nullptr, 0);
            const float earlyA = rmsOf(out, 2000, 6000);
            const float lateA = rmsOf(out, firstEnd - 6000, firstEnd - 2000);
            const float earlyB = rmsOf(out, secondStart + 2000, secondStart + 6000);
            const float lateB = rmsOf(out, out.size() - 6000, out.size() - 2000);
            expect(lateA > earlyA * 5.0f,
                   "the lane opens the oscillator during the first note: " + String(earlyA) + " -> " + String(lateA));
            expect(earlyB < lateB * 0.3f, "and starts over on the second note: " + String(earlyB));
            expectWithinAbsoluteError(lateB / lateA, 1.0f, 0.15f, "both notes follow the same curve");
        }

        beginTest("motion recorder: records a move, thins it, plays it back; overdub, quantise, length detect");
        {
            Engine e;
            e.prepare(kRate, 128);
            SeqHostTime host;
            host.hasBpm = true;
            host.bpm = 120.0;
            host.hasPpq = true;
            host.playing = true;
            double ppq = 0.0;
            std::vector<float> l(128), r(128);
            const auto proc = [&](int blocks)
            {
                for (int i = 0; i < blocks; ++i)
                {
                    host.ppq = ppq;
                    e.process(l.data(), r.data(), 128, nullptr, 0, &host);
                    ppq += 128.0 / kRate * 2.0;
                    e.motion().drain();
                }
            };
            const int idx = e.model().indexOf("fx_ra_size");
            const auto& def = ParameterMap::params()[static_cast<size_t>(idx)];
            e.setParamNormalised(idx, 0.2f);
            proc(20);
            MotionSettings ms;
            ms.clip = 1;
            ms.lane = 2;
            ms.param = "fx_ra_size";
            expect(e.recordMotion(ms), "arms");
            expect(e.motion().armed());
            const auto routes = e.modRoutes().routesFor("fx_ra_size");
            expect(routes.size() == 1 && routes[0].source == clipLaneSource(1, 2) &&
                       routes[0].polarity == static_cast<int>(ModPolarity::Bipolar),
                   "clip, lane and route are created");
            // Hold still for a bit, then move from 0.2 to 0.8 over about two beats, then rest.
            proc(150);
            for (int step = 1; step <= 100; ++step)
            {
                e.setParamNormalised(idx, 0.2f + 0.6f * static_cast<float>(step) / 100.0f);
                proc(3);
            }
            proc(40);
            e.motion().stop();
            e.motion().drain();
            auto bank = e.clips().snapshot();
            expect(bank.clips.size() == 2 && bank.clips[1].lanes.size() == 3, "clip and lane were created");
            const auto& pts = bank.clips[1].lanes[2].points;
            expect(pts.size() >= 3 && pts.size() < 60,
                   "thinned to a sensible count: " + String(static_cast<int>(pts.size())));
            bool sorted = true;
            for (size_t i = 1; i < pts.size(); ++i)
                sorted = sorted && pts[i].x >= pts[i - 1].x;
            expect(sorted, "sorted by time");
            expect(pts.front().y < 0.55f && pts.back().y > 0.75f && pts.back().y <= 1.0f,
                   "relative values: " + String(pts.front().y) + " .. " + String(pts.back().y));
            expectWithinAbsoluteError(pts.back().y, 0.5f + 0.5f * 0.6f, 0.02f, "end offset is +0.6 of the range");
            const double startX = pts.front().x;
            expect(startX >= 0.0 && startX < bank.clips[1].length, "timestamps are in clip units");
            expectEquals(e.motion().dropped(), 0);

            // Playback on top of the parameter: put the parameter back to the base and watch the modulated value.
            e.setParamNormalised(idx, 0.2f);
            bank.clips[1].trigger = ClipTrigger::Transport;
            e.clips().replace(bank);
            const double clipLen = bank.clips[1].length;
            double maxErr = 0.0;
            for (int i = 0; i < 700; ++i)
            {
                host.ppq = ppq;
                e.process(l.data(), r.data(), 128, nullptr, 0, &host);
                const double pos = std::fmod(ppq + 96.0 / kRate * 2.0, clipLen); // the last chunk of the block
                const float lane = evaluateLane(bank.clips[1], bank.clips[1].lanes[2], pos, 0);
                const float expected = ParameterMap::fromNormalised(def, std::clamp(0.2f + (2 * lane - 1), 0.0f, 1.0f));
                if (i > 60 && pos > 0.05 && pos < clipLen - 0.05)
                {
                    const double err = std::abs(ParameterMap::toNormalised(def, e.modulatedValue("fx_ra_size")) -
                                                ParameterMap::toNormalised(def, expected));
                    maxErr = std::max(maxErr, err);
                }
                ppq += 128.0 / kRate * 2.0;
            }
            expect(maxErr < 0.03, "playback reproduces the recorded move (max error " + String(maxErr) + ")");

            // Overdub keeps the untouched region and replaces the touched one.
            const auto before = bank.clips[1].lanes[2].points;
            MotionSettings od = ms;
            od.overdub = true;
            od.quantise = true;
            e.setParamNormalised(idx, 0.2f);
            proc(10);
            expect(e.recordMotion(od));
            proc(100);
            e.setParamNormalised(idx, 0.9f);
            proc(60);
            e.motion().stop();
            e.motion().drain();
            const auto after = e.clips().snapshot().clips[1].lanes[2].points;
            expect(after != before && !after.empty(), "overdub changed the lane");
            expect(after.size() >= 2, "still has points");
            const double g = bank.clips[1].grid;
            int onGrid = 0;
            for (const auto& p : after)
                if (std::abs(p.x / g - std::round(p.x / g)) < 1.0e-9)
                    ++onGrid;
            expect(g > 0.0 && onGrid >= 1, "quantised points sit on the grid");
        }

        beginTest("motion recorder: loop length detection, quantise helper, bad input");
        {
            Clip c;
            c.grid = 0.25;
            expectWithinAbsoluteError(detectClipLength(c, 3.8), 3.75, 1.0e-12);
            expectWithinAbsoluteError(detectClipLength(c, 0.01), 0.25, 1.0e-12, "at least one grid step");
            c.grid = 0.0;
            expectWithinAbsoluteError(detectClipLength(c, 7.6), 8.0, 1.0e-12, "whole beats without a grid");
            c.grid = 0.5;
            ClipLane lane;
            lane.points = {{0.1, 0.0f}, {0.2, 1.0f}, {0.9, 0.5f}};
            quantiseLane(c, lane);
            expectEquals(static_cast<int>(lane.points.size()), 2, "points that land together merge");
            expectWithinAbsoluteError(lane.points[0].x, 0.0, 1.0e-12);
            expectWithinAbsoluteError(lane.points[1].x, 1.0, 1.0e-12);

            Engine e;
            e.prepare(kRate, 128);
            MotionSettings ms;
            ms.param = "nope";
            expect(!e.recordMotion(ms), "unknown parameter");
            ms.param = "osc1_table";
            ms.clip = 99;
            expect(!e.recordMotion(ms), "clip out of range");

            // Detect the length of a free take.
            ms = MotionSettings{};
            ms.param = "fx_ra_size";
            ms.detectLength = true;
            ms.quantise = true;
            SeqHostTime host;
            host.hasBpm = true;
            host.bpm = 120.0;
            std::vector<float> l(128), r(128);
            const int idx = e.model().indexOf("fx_ra_size");
            expect(e.recordMotion(ms));
            for (int i = 0; i < 700; ++i) // 700 x 128 samples = 1.867 s = 3.73 beats
            {
                if (i == 100)
                    e.setParamNormalised(idx, 0.9f);
                if (i == 300)
                    e.setParamNormalised(idx, 0.1f);
                e.process(l.data(), r.data(), 128, nullptr, 0, &host);
                e.motion().drain();
            }
            e.motion().stop();
            const auto bank = e.clips().snapshot();
            expectWithinAbsoluteError(bank.clips[0].length, 3.75, 0.5001, "length from the take");
            expect(std::fmod(bank.clips[0].length, bank.clips[0].grid) < 1.0e-9, "on the grid");
            expect(bank.clips[0].lanes[0].points.size() >= 2);
        }
    }
};

static ClipTests clipTests;
