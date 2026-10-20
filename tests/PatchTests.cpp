// SPDX-License-Identifier: MIT
#include <crucible/ArpPatternIO.h>
#include <crucible/Engine.h>
#include <crucible/Json.h>
#include <crucible/MidiFile.h>
#include <crucible/Patch.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
std::vector<float> render(Engine& e, int notes = 1)
{
    std::vector<float> out, l(256), r(256);
    MidiInput midi[3] = {{0, 0x90, 48, 100}, {0, 0x90, 55, 90}, {0, 0x90, 60, 80}};
    for (int b = 0; b < 40; ++b)
    {
        e.process(l.data(), r.data(), 256, b == 0 ? midi : nullptr, b == 0 ? notes : 0);
        out.insert(out.end(), l.begin(), l.end());
        out.insert(out.end(), r.begin(), r.end());
    }
    return out;
}

void put32(std::vector<std::uint8_t>& v, std::uint32_t x)
{
    for (int s = 24; s >= 0; s -= 8)
        v.push_back(static_cast<std::uint8_t>((x >> s) & 0xffU));
}
} // namespace

class PatchTests : public UnitTest
{
public:
    PatchTests() : UnitTest("Patches, JSON and MIDI files", "Engine") {}

    void runTest() override
    {
        beginTest("JSON: parse, dump, round trip, escapes, errors, base64");
        {
            json::Value v;
            std::string error;
            expect(json::parse(R"({"a": [1, 2.5, -3e2, true, null], "s": "q\"\\é😀", "o": {}})", v, error), error);
            expect(v["a"].size() == 5 && v["a"][1].asNumber() == 2.5 && v["a"][2].asNumber() == -300.0 &&
                   v["a"][3].asBool());
            expectEquals(v["s"].asString(), std::string("q\"\\\xc3\xa9\xf0\x9f\x98\x80"));
            json::Value back;
            expect(json::parse(v.dump(2), back, error) && back.dump(-1) == v.dump(-1));
            expect(!json::parse("{\"a\": }", back, error) && !error.empty());
            expect(!json::parse("[1, 2", back, error));
            expect(!json::parse("{} x", back, error));
            const std::uint8_t bytes[] = {0, 1, 2, 250, 251, 252, 253};
            std::vector<std::uint8_t> decoded;
            expect(json::base64Decode(json::base64Encode(bytes, sizeof(bytes)), decoded) &&
                   decoded == std::vector<std::uint8_t>(bytes, bytes + sizeof(bytes)));
        }

        beginTest("patches carry samples and clouds of the oscillator slots");
        {
            Engine a;
            a.prepare(kRate, 256);
            AudioData audio;
            audio.sampleRate = 44100.0;
            audio.channels.assign(2, std::vector<float>(8000, 0.0f));
            for (int i = 0; i < 8000; ++i)
            {
                audio.channels[0][static_cast<size_t>(i)] = 0.4f * std::sin(0.07f * static_cast<float>(i));
                audio.channels[1][static_cast<size_t>(i)] = 0.3f * std::sin(0.11f * static_cast<float>(i));
            }
            std::string error;
            expect(a.loadSample(1, audio, "loop", error), error);
            a.setCloud(2, CloudModel::random(12, 3));
            a.setParam("osc2_type", 1.0f);
            a.setParam("osc2_level", 0.6f);
            a.setParam("osc2_smp_loop", 1.0f);
            a.setParam("osc3_type", 2.0f);
            a.setParam("osc3_level", 0.4f);
            const auto reference = render(a);
            const auto patch = capturePatch(a);
            expectEquals(patch.assets.size(), size_t(2)); // one sample, one cloud
            Patch parsed;
            expect(patchFromJson(patchToJson(patch), parsed, error), error);
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, parsed);
            // Clouds are stored as text with six decimals, so compare with a tolerance.
            const auto restored = render(b);
            double worst = 0.0;
            for (size_t i = 0; i < reference.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(restored[i] - reference[i])));
            expectEquals(restored.size(), reference.size());
            expect(worst < 1.0e-3, "restored patch renders the same sound");
            // Applying a patch without them clears the slots.
            applyPatch(b, Patch{});
            expect(!b.samples().isLoaded(1));
        }

        beginTest("patch round trip restores the sound and every kind of state");
        {
            // High quality: the convolution waits for its kernel builder, so both renders do not depend on thread timing.
            EngineConfig offline;
            offline.quality = Quality::High;
            Engine a(offline);
            a.prepare(kRate, 256);
            a.setParam("osc1_pos", 0.7f);
            a.setParam("filter_cutoff", 1500.0f);
            a.setParam("fx_ra_on", 1.0f);
            a.setParam("fx_cx_on", 1.0f);
            a.setParam("osc2_level", 0.5f);
            a.setParam("osc2_table", static_cast<float>(WavetableBank::numTables + 1));
            a.storeScene(3);
            a.sequencer().edit([](SeqBank& b) { b.pattern[2].steps[5].on = true; });
            a.arpPattern().edit([](ArpPattern& p) { p.steps[3].octave = 2; });
            a.modRoutes().add({static_cast<int>(ModSource::Lfo1), "fx_ra_mix", 0.2f});
            a.loadCloudWavetable(1, CloudModel::harmonicMorph(16));
            ImpulseResponse ir;
            ir.sampleRate = kRate;
            ir.channels = {std::vector<float>(2400, 0.0f)};
            ir.channels[0][0] = 1.0f;
            ir.channels[0][1200] = 0.5f;
            a.loadConvolutionIr(ir, "two taps");
            PatchMeta meta;
            meta.name = "Test";
            meta.tags = {"bass", "dark"};
            meta.rating = 4;
            meta.favourite = true;
            const auto text = patchToJson(capturePatch(a, meta));

            Patch p;
            std::string error;
            expect(patchFromJson(text, p, error), error);
            expect(p.meta.name == "Test" && p.meta.tags.size() == 2 && p.meta.rating == 4 && p.meta.favourite);
            expect(p.moduleVersions.count("fx.saturator") == 1);
            Engine b(offline);
            b.prepare(kRate, 256);
            b.setParam("master_gain", -30.0f);
            applyPatch(b, p);
            expectWithinAbsoluteError(b.getParam("filter_cutoff"), 1500.0f, 1.0e-3f);
            expectEquals(b.getParam("master_gain"), a.getParam("master_gain"));
            expect(b.scenes().isFilled(3));
            expect(b.sequencer().snapshotPattern(2).steps[5].on);
            expect(b.arpPattern().snapshot().steps[3].octave == 2);
            expectEquals(static_cast<int>(b.modRoutes().routes().size()), 1);
            expect(b.userWavetables().isLoaded(1));
            Patch again;
            expect(patchFromJson(patchToJson(capturePatch(b, meta)), again, error));
            expect(again.convolutionIrs.size() == 1 && again.convolutionIrs[0].channels[0].size() == 2400 &&
                   again.convolutionIrs[0].name == "two taps");
            const auto ra = render(a, 3);
            const auto rb = render(b, 3);
            float diff = 0.0f;
            for (size_t i = 0; i < ra.size(); ++i)
                diff = std::max(diff, std::abs(ra[i] - rb[i]));
            expect(diff < 1.0e-4f, "reloaded patch renders the same, max diff " + String(diff));
        }

        beginTest("loading tolerates unknown and missing parameters and rejects other formats or newer versions");
        {
            Patch p;
            std::string error;
            expect(patchFromJson(
                       R"({"format": "crucible-patch", "version": 1, "params": {"filter_cutoff": 900, "gone_param": 3}})", p,
                       error),
                   error);
            Engine e;
            e.setParam("osc1_level", 0.1f);
            applyPatch(e, p);
            expectWithinAbsoluteError(e.getParam("filter_cutoff"), 900.0f, 1.0e-3f);
            expectEquals(e.getParam("osc1_level"),
                         e.model().params()[static_cast<size_t>(e.model().indexOf("osc1_level"))].defaultValue);
            expect(!patchFromJson(R"({"format": "crucible-patch", "version": 99})", p, error));
            expect(!patchFromJson(R"({"format": "something"})", p, error));
            expect(!patchFromJson("not json", p, error));
        }

        beginTest("standard MIDI files: format 1, running status, tempo changes");
        {
            std::vector<std::uint8_t> f = {'M', 'T', 'h', 'd'};
            put32(f, 6);
            f.insert(f.end(), {0, 1, 0, 2, 0x01, 0xE0}); // format 1, 2 tracks, 480 ticks per quarter
            // Track 1: tempo 120 at 0, tempo 60 at tick 960.
            std::vector<std::uint8_t> t1 = {0x00, 0xFF, 0x51, 0x03, 0x07, 0xA1, 0x20, 0x87, 0x40, 0xFF,
                                            0x51, 0x03, 0x0F, 0x42, 0x40, 0x00, 0xFF, 0x2F, 0x00};
            // Track 2: note on at 0, running-status note on at 480, note off at 1440.
            std::vector<std::uint8_t> t2 = {0x00, 0x90, 60, 100, 0x83, 0x60, 64,   90,  0x87,
                                            0x40, 0x80, 60, 0,   0x00, 0xFF, 0x2F, 0x00};
            for (auto* t : {&t1, &t2})
            {
                f.insert(f.end(), {'M', 'T', 'r', 'k'});
                put32(f, static_cast<std::uint32_t>(t->size()));
                f.insert(f.end(), t->begin(), t->end());
            }
            std::vector<TimedMidi> ev;
            MidiFileInfo info;
            std::string error;
            expect(readMidiFile(f.data(), f.size(), ev, info, error), error);
            expect(info.format == 1 && info.tracks == 2 && info.ticksPerQuarter == 480);
            expectWithinAbsoluteError(info.firstTempoBpm, 120.0, 1.0e-6);
            expect(ev.size() == 3);
            if (ev.size() == 3)
            {
                expectWithinAbsoluteError(ev[0].seconds, 0.0, 1.0e-9);
                expectWithinAbsoluteError(ev[1].seconds, 0.5, 1.0e-9); // tick 480 at 120 BPM
                expectWithinAbsoluteError(ev[2].seconds, 2.0, 1.0e-9); // 1 s to tick 960, then 1 s at 60 BPM
                expect(ev[1].status == 0x90 && ev[1].data1 == 64 && ev[2].status == 0x80);
            }
            expect(!readMidiFile(f.data(), 10, ev, info, error));
        }
    }
};

static PatchTests patchTests;
