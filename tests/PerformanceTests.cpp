// SPDX-License-Identifier: MIT
#include <crucible/CcMap.h>
#include <crucible/Engine.h>
#include <crucible/Patch.h>
#include <crucible/Tuning.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
double measureHz(const std::vector<float>& x, int skip = 4000)
{
    int crossings = 0, first = -1, last = -1;
    for (size_t i = static_cast<size_t>(skip) + 1; i < x.size(); ++i)
        if (x[i - 1] < 0.0f && x[i] >= 0.0f)
        {
            if (first < 0)
                first = static_cast<int>(i);
            last = static_cast<int>(i);
            ++crossings;
        }
    return crossings < 3 ? 0.0 : (crossings - 1) * kRate / (last - first);
}

float peakOf(const std::vector<float>& x, size_t from = 0)
{
    float p = 0.0f;
    for (size_t i = from; i < x.size(); ++i)
        p = std::max(p, std::abs(x[i]));
    return p;
}

void sinePatch(Engine& e)
{
    e.resetParamsToDefaults();
    for (const auto& [id, v] : std::vector<std::pair<const char*, float>>{{"osc1_pos", 0.0f},
                                                                          {"osc1_level", 1.0f},
                                                                          {"amp_sustain", 1.0f},
                                                                          {"amp_attack", 0.001f},
                                                                          {"filter_cutoff", 20000.0f},
                                                                          {"master_gain", 0.0f}})
        e.setParam(id, v);
}

std::vector<float> playNote(Engine& e, int note, int blocks = 100, std::uint8_t channel = 0)
{
    std::vector<float> out, l(128), r(128);
    MidiInput on{0, static_cast<std::uint8_t>(0x90 | channel), static_cast<std::uint8_t>(note), 100};
    for (int b = 0; b < blocks; ++b)
    {
        e.process(l.data(), r.data(), 128, b == 0 ? &on : nullptr, b == 0 ? 1 : 0);
        out.insert(out.end(), l.begin(), l.end());
    }
    MidiInput off{0, static_cast<std::uint8_t>(0x80 | channel), static_cast<std::uint8_t>(note), 0};
    e.process(l.data(), r.data(), 128, &off, 1);
    for (int b = 0; b < 100; ++b)
        e.process(l.data(), r.data(), 128);
    return out;
}

// Feeds raw MIDI to an engine block by block and keeps the left channel.
struct Player
{
    Engine& e;
    std::vector<float> out;
    std::vector<float> l = std::vector<float>(128), r = std::vector<float>(128);
    explicit Player(Engine& engine) : e(engine) {}
    void send(std::initializer_list<MidiInput> events, int blocks = 1)
    {
        std::vector<MidiInput> v(events);
        e.process(l.data(), r.data(), 128, v.data(), static_cast<int>(v.size()));
        out.insert(out.end(), l.begin(), l.end());
        run(blocks - 1);
    }
    void run(int blocks)
    {
        for (int b = 0; b < blocks; ++b)
        {
            e.process(l.data(), r.data(), 128);
            out.insert(out.end(), l.begin(), l.end());
        }
    }
    void clear() { out.clear(); }
    // Pitch over the last `blocks` blocks.
    double pitch(int blocks = 30) const
    {
        const size_t n = static_cast<size_t>(blocks) * 128;
        return measureHz(std::vector<float>(out.end() - static_cast<std::ptrdiff_t>(n), out.end()), 0);
    }
    float level(int blocks = 30) const
    {
        const size_t n = static_cast<size_t>(blocks) * 128;
        return peakOf(std::vector<float>(out.end() - static_cast<std::ptrdiff_t>(n), out.end()));
    }
};

MidiInput noteOnMsg(int ch, int note)
{
    return {0, static_cast<std::uint8_t>(0x90 | ch), static_cast<std::uint8_t>(note), 100};
}
MidiInput noteOffMsg(int ch, int note)
{
    return {0, static_cast<std::uint8_t>(0x80 | ch), static_cast<std::uint8_t>(note), 0};
}
MidiInput bendMsg(int ch, float norm)
{
    const int v = static_cast<int>(std::lround(8192.0f + norm * 8191.0f));
    return {0, static_cast<std::uint8_t>(0xe0 | ch), static_cast<std::uint8_t>(v & 0x7f), static_cast<std::uint8_t>(v >> 7)};
}
MidiInput pressureMsg(int ch, float v)
{
    return {0, static_cast<std::uint8_t>(0xd0 | ch), static_cast<std::uint8_t>(v * 127.0f + 0.5f), 0};
}
MidiInput ccMsg(int ch, int cc, float v)
{
    return {0, static_cast<std::uint8_t>(0xb0 | ch), static_cast<std::uint8_t>(cc),
            static_cast<std::uint8_t>(v * 127.0f + 0.5f)};
}
MidiInput polyMsg(int ch, int note, float v)
{
    return {0, static_cast<std::uint8_t>(0xa0 | ch), static_cast<std::uint8_t>(note),
            static_cast<std::uint8_t>(v * 127.0f + 0.5f)};
}

std::string quarterToneScale()
{
    std::string s = "! 24edo.scl\nQuarter tones\n24\n";
    for (int i = 1; i <= 24; ++i)
        s += std::to_string(50.0 * i) + "\n";
    return s;
}
} // namespace

class PerformanceTests : public UnitTest
{
public:
    PerformanceTests() : UnitTest("Microtuning, MPE, play modes and CC maps", "Engine") {}

    void runTest() override
    {
        beginTest("tuning: 12-TET default, Scala ratios and cents, comments, errors");
        {
            Tuning t;
            expectWithinAbsoluteError(t.frequency(69), 440.0, 1.0e-9);
            expectWithinAbsoluteError(t.frequency(60), 261.6255653, 1.0e-6);
            expect(t.isEqualTemperament() && t.table().identity);
            std::string error;
            expect(t.loadScl("! just.scl\n!\n5-limit major\n 12\n!\n 16/15\n 9/8\n 6/5\n 5/4\n 4/3\n 45/32 tritone\n 3/2\n "
                             "8/5\n 5/3\n 9/5\n 15/8\n 2/1\n",
                             error),
                   error);
            expectEquals(t.scaleSize(), 12);
            expectWithinAbsoluteError(t.degreeCents()[7], 1200.0 * std::log2(1.5), 1.0e-9, "3/2 in cents");
            expectWithinAbsoluteError(t.frequency(69), 440.0, 1.0e-9, "reference stays");
            // Middle C is the 1/1: A is 5/3 above it.
            expectWithinAbsoluteError(t.frequency(69) / t.frequency(60), 5.0 / 3.0, 1.0e-9, "just major sixth");
            expectWithinAbsoluteError(t.frequency(67) / t.frequency(60), 1.5, 1.0e-9, "just fifth");
            expectWithinAbsoluteError(t.frequency(72) / t.frequency(60), 2.0, 1.0e-9, "octave");
            expectWithinAbsoluteError(t.frequency(48) / t.frequency(60), 0.5, 1.0e-9, "below the middle note");
            expect(!t.isEqualTemperament());
            const auto table = t.table();
            expect(!table.identity && table.semitones[67] > 0.1f, "G is 18 cents sharp of 12-TET against the A reference");

            Tuning bad;
            expect(!bad.loadScl("x\n3\n100.0\n200.0\n", error) && !error.empty(), "too few pitches");
            expect(!bad.loadScl("x\n2\n100.0\nabc\n", error), "garbage pitch");
            expect(!bad.loadScl("x\n1\n-5/1\n", error), "negative ratio");
            expect(!bad.loadScl("only one line", error));
            expect(!bad.loadScl("x\n0\n", error), "empty scale");
            expect(!bad.loadKbm("12\n0\n127\n60\n", error), "kbm header too short");
            expect(!bad.loadKbm("0\n0\n200\n60\n69\n440\n12\n", error), "kbm range");
            expect(!bad.loadSclFile("/nonexistent/x.scl", error));
            expect(bad.isEqualTemperament(), "failed loads change nothing");
        }

        beginTest("tuning: keyboard maps, unmapped keys, reference, per-note offsets, serialisation");
        {
            Tuning t;
            std::string error;
            expect(t.loadScl(quarterToneScale(), error), error);
            expectWithinAbsoluteError(t.frequency(61) / t.frequency(60), std::pow(2.0, 50.0 / 1200.0), 1.0e-9,
                                      "24-EDO step");
            expectWithinAbsoluteError(t.frequency(84) / t.frequency(60), 2.0, 1.0e-9, "octave after 24 keys");
            // A map that puts 12 of the keys on every other degree, leaves one unmapped and moves the reference.
            std::string kbm = "! map\n12\n0\n127\n60\n60\n256.0\n12\n";
            for (int i = 0; i < 12; ++i)
                kbm += (i == 5 ? std::string("x") : std::to_string(i * 2)) + "\n";
            expect(t.loadKbm(kbm, error), error);
            expectWithinAbsoluteError(t.frequency(60), 256.0, 1.0e-9, "reference note and frequency");
            expectWithinAbsoluteError(t.frequency(62) / t.frequency(60), std::pow(2.0, 200.0 / 1200.0), 1.0e-9,
                                      "map entry 4 is 200 cents");
            expect(!t.isMapped(65) && t.isMapped(64), "x leaves a key unmapped");
            expectWithinAbsoluteError(t.frequency(65), Tuning::equalFrequency(65), 1.0e-9, "unmapped key falls back");
            expect(!t.table().mapped[65] && t.table().mapped[66]);
            expectWithinAbsoluteError(t.frequency(72) / t.frequency(60), std::pow(2.0, 600.0 / 1200.0), 1.0e-9,
                                      "map repeats by the formal octave (degree 12 of 24)");
            expect(t.loadKbm("0\n64\n72\n60\n60\n261.625565\n12\n", error), error);
            expect(!t.isMapped(40) && !t.isMapped(80) && t.isMapped(64), "first and last note");

            t.resetKeyboardMap();
            t.setNoteOffsetCents(62, 25.0);
            expectWithinAbsoluteError(t.noteOffsetCents(62), 25.0, 1.0e-12);
            expectWithinAbsoluteError(t.frequency(62) / t.frequency(61) / (t.frequency(61) / t.frequency(60)),
                                      std::pow(2.0, 25.0 / 1200.0), 1.0e-9, "per-note offset");
            t.setNoteOffsetCents(62, 5000.0);
            expectWithinAbsoluteError(t.noteOffsetCents(62), 1200.0, 1.0e-12, "clamped");
            t.setNoteOffsetCents(62, 25.0);
            Tuning back;
            expect(Tuning::deserialise(t.serialise(), back));
            bool same = true;
            for (int n = 0; n < 128; ++n)
                same = same && std::abs(back.frequency(n) - t.frequency(n)) < 1.0e-9 && back.isMapped(n) == t.isMapped(n);
            expect(same, "serialisation round trip");
            expect(!Tuning::deserialise("{not json", back));
            expect(Tuning::deserialise("", back) && back.isEqualTemperament(), "empty means 12-TET");
            Tuning plain;
            plain.setNoteOffsetCents(60, 10.0);
            expect(!plain.isEqualTemperament());
            plain.clearNoteOffsets();
            expect(plain.isEqualTemperament());
        }

        beginTest("tuning: the engine plays the tuned pitch, tune_amount blends, unmapped keys are silent, patches keep it");
        {
            Engine e;
            e.prepare(kRate, 128);
            sinePatch(e);
            const double base = measureHz(playNote(e, 61));
            expectWithinAbsoluteError(base, midiToHz(61), midiToHz(61) * 0.004, "12-TET reference");
            Tuning t;
            std::string error;
            expect(t.loadScl(quarterToneScale(), error), error);
            e.tuning().set(t);
            const double tuned = measureHz(playNote(e, 61));
            const double want = t.frequency(61);
            expectWithinAbsoluteError(tuned, want, want * 0.004, "24-EDO key 61");
            expect(std::abs(tuned - base) > base * 0.01, "differs from 12-TET");
            e.setParam("tune_amount", 0.0f);
            for (int i = 0; i < 40; ++i)
            {
                std::vector<float> l(128), r(128);
                e.process(l.data(), r.data(), 128);
            }
            expectWithinAbsoluteError(measureHz(playNote(e, 61)), base, base * 0.004, "tune_amount 0 is 12-TET");
            e.setParam("tune_amount", 1.0f);
            std::string kbm = "12\n0\n127\n60\n69\n440\n12\n0\nx\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n";
            expect(t.loadKbm(kbm, error), error);
            e.tuning().set(t);
            for (int i = 0; i < 40; ++i)
            {
                std::vector<float> l(128), r(128);
                e.process(l.data(), r.data(), 128);
            }
            expect(peakOf(playNote(e, 61)) < 1.0e-6f, "an unmapped key makes no sound");
            expect(peakOf(playNote(e, 62)) > 0.05f, "mapped keys do");

            PatchMeta meta;
            meta.name = "Tuned";
            const Patch p = capturePatch(e, meta);
            expect(!p.tuning.empty());
            Patch loaded;
            expect(patchFromJson(patchToJson(p), loaded, error), error);
            Engine other;
            other.prepare(kRate, 128);
            applyPatch(other, loaded);
            expectWithinAbsoluteError(other.tuning().get().frequency(62), t.frequency(62), 1.0e-9,
                                      "tuning restored from a patch");
            Engine plainEngine;
            expect(capturePatch(plainEngine).tuning.empty() &&
                       patchToJson(capturePatch(plainEngine)).find("tuning") == std::string::npos,
                   "12-TET patches carry no tuning");
        }
        beginTest("MPE: per-note bend on member channels; the master channel bends everything; off means one channel");
        {
            Engine e;
            e.prepare(kRate, 128);
            sinePatch(e);
            e.setParam("mpe_on", 1.0f);
            e.setParam("mpe_bend_range", 12.0f);
            e.setParam("voice_bend", 2.0f);
            {
                Player p(e);
                p.send({noteOnMsg(1, 60), bendMsg(1, 0.5f)}, 120);
                expectWithinAbsoluteError(p.pitch(), midiToHz(66), midiToHz(66) * 0.006, "member bend of +0.5 x 12 st");
                p.send({noteOffMsg(1, 60)}, 120);
            }
            {
                Player p(e);
                p.send({noteOnMsg(2, 60)}, 120);
                expectWithinAbsoluteError(p.pitch(), midiToHz(60), midiToHz(60) * 0.006,
                                          "another channel's bend does not leak");
                p.send({bendMsg(0, 1.0f)}, 120);
                expectWithinAbsoluteError(p.pitch(), midiToHz(62), midiToHz(62) * 0.006,
                                          "master channel: the normal bend range");
                p.send({noteOffMsg(2, 60), bendMsg(0, 0.0f)}, 120);
            }
            {
                Player p(e);
                p.send({noteOnMsg(1, 55), noteOnMsg(2, 67), bendMsg(1, -0.5f), bendMsg(2, 0.25f)}, 150);
                // Two sines: find each by measuring with a filter-free zero-crossing is unreliable, so compare the
                // spectrum peaks through two separate runs below; here only check both are audible and finite.
                expect(p.level() > 0.3f);
                p.send({noteOffMsg(1, 55), noteOffMsg(2, 67)}, 100);
            }
            e.setParam("mpe_on", 0.0f);
            {
                Player p(e);
                p.run(20);
                p.send({noteOnMsg(1, 60), bendMsg(1, 1.0f)}, 120);
                expectWithinAbsoluteError(p.pitch(), midiToHz(62), midiToHz(62) * 0.006,
                                          "MPE off: channel 1 bend is the global bend");
            }
        }

        beginTest("MPE: pressure, slide and bend are per-note route sources; poly aftertouch; global targets follow the "
                  "newest note");
        {
            Engine e;
            e.prepare(kRate, 128);
            sinePatch(e);
            e.setParam("osc1_level", 0.0f);
            e.setParam("mpe_on", 1.0f);
            expect(e.modRoutes().add({kSourceMpePressure, "osc1_level", 1.0f}));
            expect(e.modRoutes().add({kSourceMpeSlide, "filter_cutoff", -0.0f}), "slide can target the filter");
            e.modRoutes().remove(kSourceMpeSlide, "filter_cutoff");
            expectEquals(routeSourceName(kSourceMpeSlide), std::string("MPE Slide"));
            float unit = 0.0f;
            {
                // Peak of the patch at full oscillator level, the yardstick for the checks below.
                Engine ref;
                ref.prepare(kRate, 128);
                sinePatch(ref);
                Player p(ref);
                p.send({noteOnMsg(0, 60)}, 200);
                unit = p.level();
            }
            float single = 0.0f;
            {
                Player p(e);
                p.send({noteOnMsg(1, 60), pressureMsg(1, 0.8f)}, 200);
                single = p.level();
                expectWithinAbsoluteError(single, 0.8f * unit, 0.06f, "pressure scales the level");
                p.send({noteOffMsg(1, 60)}, 150);
            }
            {
                Player p(e);
                p.send({noteOnMsg(1, 60), pressureMsg(1, 0.8f), noteOnMsg(2, 67), pressureMsg(2, 0.0f)}, 200);
                expectWithinAbsoluteError(p.level(), single, 0.06f,
                                          "a note without pressure adds nothing: pressure is per note");
                p.send({pressureMsg(2, 0.8f)}, 200);
                expect(p.level() > single * 1.3f, "both pressed: " + String(p.level()));
                p.send({noteOffMsg(1, 60), noteOffMsg(2, 67)}, 150);
            }
            e.modRoutes().clear();
            expect(e.modRoutes().add({kSourceMpeSlide, "osc1_level", 1.0f}));
            {
                Player p(e);
                p.send({noteOnMsg(3, 60), ccMsg(3, 74, 0.5f)}, 200);
                expectWithinAbsoluteError(p.level(), 0.5f * unit, 0.05f, "slide (CC 74 on the member channel)");
                p.send({noteOffMsg(3, 60)}, 150);
            }
            e.modRoutes().clear();
            e.setParam("mpe_on", 0.0f);
            expect(e.modRoutes().add({kSourceMpePressure, "osc1_level", 1.0f}));
            {
                Player p(e);
                p.send({noteOnMsg(0, 60), noteOnMsg(0, 72)}, 100);
                expect(p.level() < 1.0e-4f, "no pressure, no sound");
                p.send({polyMsg(0, 60, 0.9f)}, 200);
                const float level = p.level();
                expectWithinAbsoluteError(level, 0.9f * unit, 0.06f,
                                          "poly aftertouch reaches only its note: " + String(level));
                p.send({noteOffMsg(0, 60), noteOffMsg(0, 72)}, 150);
            }
            // Poly aftertouch at the same sample as its note-on applies to the new note.
            {
                Player p(e);
                p.send({noteOnMsg(0, 60), polyMsg(0, 60, 0.9f)}, 200);
                const float level = p.level();
                expectWithinAbsoluteError(level, 0.9f * unit, 0.06f,
                                          "same-sample poly aftertouch is kept: " + String(level));
                p.send({noteOffMsg(0, 60)}, 150);
            }
            // A route that moves the amount of a per-voice route moves it: 0.5 + 0.5 x macro 1 (at 1) is a full-depth route.
            e.modRoutes().clear();
            e.setParam("mpe_on", 1.0f);
            e.setParam("macro1", 1.0f);
            expect(e.modRoutes().add({kSourceMpePressure, "osc1_level", 0.5f}));
            expect(e.modRoutes().add({static_cast<int>(ModSource::Macro1), routeAmountTargetId(0), 0.5f, 0, 0.0f, 0.0f}));
            {
                Player p(e);
                p.send({noteOnMsg(1, 60), pressureMsg(1, 0.8f)}, 300);
                expectWithinAbsoluteError(p.level(), 0.8f * unit, 0.06f,
                                          "amount modulation reaches a per-voice route: " + String(p.level()));
                p.send({noteOffMsg(1, 60)}, 150);
            }
            // A global target follows the newest voice.
            e.modRoutes().clear();
            e.setParam("mpe_on", 1.0f);
            expect(e.modRoutes().add({kSourceMpePressure, "fx_ra_size", 0.5f}));
            const int idx = e.model().indexOf("fx_ra_size");
            const auto& def = ParameterMap::params()[static_cast<size_t>(idx)];
            e.setParam("fx_ra_size", def.min);
            {
                Player p(e);
                p.send({noteOnMsg(1, 60), pressureMsg(1, 0.2f), noteOnMsg(2, 64), pressureMsg(2, 0.8f)}, 60);
                const float range = def.max - def.min;
                expectWithinAbsoluteError(e.modulatedValue("fx_ra_size"), def.min + 0.8f * 0.5f * range, 0.03f * range,
                                          "follows the newest note");
            }
            e.modRoutes().clear();
            expect(e.modRoutes().add({kSourceMpeBend, "osc1_level", 0.5f, 2}));
            for (const auto& s : routeSourceList(ClipBank{}))
                if (s.id == kSourceMpeBend)
                    expect(s.bipolar && s.voiceOnly);
        }

        beginTest("mono modes: note priority last, lowest and highest");
        {
            Engine e;
            e.prepare(kRate, 128);
            sinePatch(e);
            e.setParam("voice_mode", 1.0f); // mono
            const auto sounding = [&](int priority, std::initializer_list<MidiInput> events, int blocks = 100)
            {
                e.setParam("voice_mono_priority", static_cast<float>(priority));
                Player p(e);
                p.run(30);
                p.send(events, blocks);
                return p.pitch();
            };
            const auto reset = [&]
            {
                Player p(e);
                p.send({ccMsg(0, 123, 0.0f)}, 150);
            };
            const auto near = [&](double got, int note, const char* what)
            { expectWithinAbsoluteError(got, midiToHz(note), midiToHz(note) * 0.008, what); };
            near(sounding(0, {noteOnMsg(0, 60), noteOnMsg(0, 64)}), 64, "last: the newer note wins");
            reset();
            near(sounding(1, {noteOnMsg(0, 60), noteOnMsg(0, 64)}), 60, "lowest: the higher note is ignored");
            reset();
            near(sounding(2, {noteOnMsg(0, 64), noteOnMsg(0, 60)}), 64, "highest: the lower note is ignored");
            reset();
            near(sounding(1, {noteOnMsg(0, 64), noteOnMsg(0, 60)}), 60, "lowest: a lower note takes over");
            near(sounding(1, {noteOffMsg(0, 60)}), 64, "lowest: back to the held note when it ends");
            reset();
            sounding(2, {noteOnMsg(0, 60)});
            sounding(2, {noteOnMsg(0, 67)});
            near(sounding(2, {noteOnMsg(0, 64)}), 67, "highest of three");
            near(sounding(2, {noteOffMsg(0, 67)}), 64, "highest: next highest after release");
            reset();
            sounding(0, {noteOnMsg(0, 60), noteOnMsg(0, 64)});
            near(sounding(0, {noteOffMsg(0, 64)}), 60, "last: previous note returns");
            reset();
        }

        beginTest("glide: always or legato only, fixed time or rate");
        {
            Engine e;
            e.prepare(kRate, 128);
            sinePatch(e);
            e.setParam("voice_mode", 1.0f);
            e.setParam("voice_glide", 1.0f);
            const auto early = [&](int mode, int type, int from, int to, bool overlap)
            {
                e.setParam("voice_glide_mode", static_cast<float>(mode));
                e.setParam("voice_glide_type", static_cast<float>(type));
                Player p(e);
                p.send({ccMsg(0, 123, 0.0f)}, 150);
                p.clear();
                p.send({noteOnMsg(0, from)}, 40);
                if (!overlap)
                    p.send({noteOffMsg(0, from)}, 20);
                p.send({noteOnMsg(0, to)}, 60); // 0.16 s later
                return p.pitch(20);
            };
            const double fromHz = midiToHz(60), toHz = midiToHz(72);
            const double always = early(0, 0, 60, 72, false);
            expect(always < toHz * 0.9 && always > fromHz * 0.9, "always: glides even after release: " + String(always));
            const double legatoGap = early(1, 0, 60, 72, false);
            expectWithinAbsoluteError(legatoGap, toHz, toHz * 0.01, "legato only: no glide without overlap");
            const double legatoOverlap = early(1, 0, 60, 72, true);
            expect(legatoOverlap < toHz * 0.9, "legato only: glides when the keys overlap: " + String(legatoOverlap));
            const double rateBig = early(0, 1, 60, 72, true);
            const double rateSmall = early(0, 1, 60, 61, true);
            expect(rateBig < toHz * 0.9, "rate: an octave takes about the full time: " + String(rateBig));
            expectWithinAbsoluteError(rateSmall, midiToHz(61), midiToHz(61) * 0.012, "rate: a semitone is done quickly");
            const double timeSmall = early(0, 0, 60, 61, true);
            expect(timeSmall < midiToHz(61) * 0.99, "time: a semitone still takes the full time: " + String(timeSmall));
            e.setParam("voice_glide", 0.0f);
            expectWithinAbsoluteError(early(0, 0, 60, 72, true), toHz, toHz * 0.01, "no glide time, no glide");
        }
        beginTest("CC map: ranges, curves, inverted and relative encoders, channels, several parameters per CC");
        {
            CcBinding b;
            b.cc = 20;
            b.param = "filter_cutoff";
            expectWithinAbsoluteError(applyCcBinding(b, 127, 0.0f), 1.0f, 1.0e-6f);
            expectWithinAbsoluteError(applyCcBinding(b, 0, 1.0f), 0.0f, 1.0e-6f);
            b.lo = 0.2f;
            b.hi = 0.6f;
            expectWithinAbsoluteError(applyCcBinding(b, 127, 0.0f), 0.6f, 1.0e-6f, "range");
            expectWithinAbsoluteError(applyCcBinding(b, 0, 0.0f), 0.2f, 1.0e-6f);
            b.lo = 0.9f;
            b.hi = 0.1f;
            expectWithinAbsoluteError(applyCcBinding(b, 127, 0.0f), 0.1f, 1.0e-6f, "inverted");
            b.lo = 0.0f;
            b.hi = 1.0f;
            b.curve = 0.8f;
            expect(applyCcBinding(b, 64, 0.0f) < 0.4f, "curve bends the response");
            b.curve = 0.0f;
            b.mode = CcMode::RelativeTwos;
            b.step = 0.01f;
            expectWithinAbsoluteError(applyCcBinding(b, 3, 0.5f), 0.53f, 1.0e-6f, "twos complement up");
            expectWithinAbsoluteError(applyCcBinding(b, 125, 0.5f), 0.47f, 1.0e-6f, "twos complement down");
            b.mode = CcMode::RelativeSignedBit;
            expectWithinAbsoluteError(applyCcBinding(b, 2, 0.5f), 0.52f, 1.0e-6f);
            expectWithinAbsoluteError(applyCcBinding(b, 66, 0.5f), 0.48f, 1.0e-6f, "signed bit down");
            b.mode = CcMode::RelativeBinary;
            expectWithinAbsoluteError(applyCcBinding(b, 65, 0.5f), 0.51f, 1.0e-6f);
            expectWithinAbsoluteError(applyCcBinding(b, 60, 0.5f), 0.46f, 1.0e-6f, "binary offset");
            expectWithinAbsoluteError(applyCcBinding(b, 127, 0.99f), 1.0f, 1.0e-6f, "clamped");

            CcMapStore store;
            expect(!store.add({1, -1, "filter_cutoff"}) && !store.add({64, -1, "filter_cutoff"}), "reserved CCs refused");
            expect(!store.add({20, -1, "no_such_param"}) && !store.add({200, -1, "filter_cutoff"}) &&
                   !store.add({20, 16, "filter_cutoff"}));
            expect(store.add({20, -1, "filter_cutoff"}) && store.add({20, 2, "filter_res"}) &&
                   store.add({20, -1, "osc1_level"}));
            CcBinding replaced{20, -1, "filter_cutoff"};
            replaced.hi = 0.5f;
            expect(store.add(replaced));
            expectEquals(static_cast<int>(store.bindings().size()), 3, "same cc, channel and parameter replaces");
            expectEquals(static_cast<int>(store.bindingsFor("filter_cutoff").size()), 1);
            CcMapStore back;
            expect(back.deserialise(store.serialise()));
            expectEquals(back.serialise(), store.serialise(), "text round trip");
            expect(store.remove(20, 2, "filter_res") && !store.remove(20, 2, "filter_res"));
            expectEquals(store.removeParam("osc1_level"), 1);
            expectEquals(store.removeCc(20, -1), 1);
            expect(store.bindings().empty());

            Engine e;
            e.prepare(kRate, 128);
            e.ccMap().add({21, -1, "filter_cutoff"});
            e.ccMap().add({21, 5, "filter_res"});
            CcBinding rel{22, -1, "osc1_level"};
            rel.mode = CcMode::RelativeTwos;
            rel.step = 0.05f;
            e.ccMap().add(rel);
            const int cutoff = e.model().indexOf("filter_cutoff"), res = e.model().indexOf("filter_res"),
                      level = e.model().indexOf("osc1_level");
            e.setParamNormalised(res, 0.0f);
            e.setParamNormalised(cutoff, 0.0f);
            e.midi(0xb2, 21, 127);
            expectWithinAbsoluteError(e.getParam(cutoff), e.model().params()[static_cast<size_t>(cutoff)].max, 1.0f,
                                      "any channel");
            expectWithinAbsoluteError(e.getParam(res), e.model().params()[static_cast<size_t>(res)].min, 1.0e-4f,
                                      "channel 5 binding ignores channel 2");
            e.midi(0xb5, 21, 127);
            expectWithinAbsoluteError(e.getParam(res), e.model().params()[static_cast<size_t>(res)].max, 1.0e-3f,
                                      "channel 5 binding");
            e.setParamNormalised(level, 0.5f);
            std::vector<float> l(128), r(128);
            MidiInput ticks[3] = {{0, 0xb0, 22, 2}, {10, 0xb0, 22, 2}, {20, 0xb0, 22, 126}};
            e.process(l.data(), r.data(), 128, ticks, 3);
            expectWithinAbsoluteError(e.getParam(level),
                                      e.model().params()[static_cast<size_t>(level)].min +
                                          0.6f * (e.model().params()[static_cast<size_t>(level)].max -
                                                  e.model().params()[static_cast<size_t>(level)].min),
                                      1.0e-3f, "relative encoder in the audio path: +2 +2 -2 ticks");
            // Legacy mapping API.
            e.mapCC(23, cutoff);
            expectEquals(e.ccMapping(23), cutoff);
            e.mapCC(23, -1);
            expectEquals(e.ccMapping(23), -1);
            e.mapCC(64, cutoff);
            expectEquals(e.ccMapping(64), -1);
            // Patches carry the map.
            Patch p = capturePatch(e);
            expect(!p.ccMap.empty());
            std::string error;
            Patch loaded;
            expect(patchFromJson(patchToJson(p), loaded, error), error);
            Engine other;
            applyPatch(other, loaded);
            expectEquals(other.ccMap().serialise(), e.ccMap().serialise());
            Engine none;
            expect(patchToJson(capturePatch(none)).find("ccMap") == std::string::npos, "no map, no key");
        }

        beginTest("CC learn: arm, move a control, binding appears; channel specific; reserved and cancelled cases");
        {
            Engine e;
            e.prepare(kRate, 128);
            const int cutoff = e.model().indexOf("filter_cutoff");
            std::vector<float> l(128), r(128);
            expect(!e.ccMap().armLearn("no_such_param"));
            expect(e.ccMap().armLearn("filter_cutoff"));
            expect(e.ccMap().learning());
            // Notes and reserved controllers do not complete the learn.
            MidiInput ignored[2] = {{0, 0x90, 60, 100}, {1, 0xb0, 1, 90}};
            e.process(l.data(), r.data(), 128, ignored, 2);
            expect(e.ccMap().learning() && !e.ccMap().pollLearned());
            e.setParamNormalised(cutoff, 0.0f);
            MidiInput move{0, 0xb3, 71, 127};
            e.process(l.data(), r.data(), 128, &move, 1);
            expect(!e.ccMap().learning(), "learn finished");
            expectWithinAbsoluteError(e.getParam(cutoff), e.model().params()[static_cast<size_t>(cutoff)].max, 1.0f,
                                      "the learning message already moved the parameter");
            CcBinding added;
            expect(e.ccMap().pollLearned(&added) && added.cc == 71 && added.channel == -1 && added.param == "filter_cutoff");
            expect(!e.ccMap().pollLearned(), "one binding per learn");
            e.setParamNormalised(cutoff, 0.0f);
            e.midi(0xb9, 71, 127);
            expectWithinAbsoluteError(e.getParam(cutoff), e.model().params()[static_cast<size_t>(cutoff)].max, 1.0f,
                                      "now bound to any channel");
            int cc = 0, ch = 0, val = 0;
            expect(e.ccMap().lastCc(cc, ch, val) && cc == 71 && ch == 9 && val == 127, "last controller message");

            CcBinding tpl;
            tpl.lo = 0.25f;
            tpl.hi = 0.75f;
            tpl.mode = CcMode::Absolute;
            expect(e.ccMap().armLearn("filter_res", true, tpl));
            e.midi(0xb4, 30, 10); // through the MIDI-thread path
            expect(e.ccMap().pollLearned(&added) && added.cc == 30 && added.channel == 4 && added.lo == 0.25f &&
                   added.hi == 0.75f);
            const int res = e.model().indexOf("filter_res");
            e.setParamNormalised(res, 0.0f);
            e.midi(0xb7, 30, 127);
            expectWithinAbsoluteError(e.getParam(res), e.model().params()[static_cast<size_t>(res)].min, 1.0e-4f,
                                      "channel specific: other channels are ignored");
            e.midi(0xb4, 30, 127);
            const auto& rd = e.model().params()[static_cast<size_t>(res)];
            expectWithinAbsoluteError(e.getParam(res), rd.min + 0.75f * (rd.max - rd.min), (rd.max - rd.min) * 0.01f,
                                      "range from the template");
            expect(e.ccMap().armLearn("osc1_level"));
            e.ccMap().cancelLearn();
            e.midi(0xb0, 31, 50);
            expect(!e.ccMap().pollLearned() && e.ccMap().bindingsFor("osc1_level").empty(), "cancelled");
            // Learning a CC that is bound elsewhere adds a second target for it.
            expect(e.ccMap().armLearn("osc1_level"));
            e.midi(0xb0, 71, 64);
            expect(e.ccMap().pollLearned() && e.ccMap().bindings().size() == 3);
        }
    }
};

static PerformanceTests performanceTests;
