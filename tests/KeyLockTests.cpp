// SPDX-License-Identifier: MIT
#include <crucible/Engine.h>
#include <crucible/KeyBus.h>
#include <crucible/KeyDetect.h>
#include <crucible/KeyLock.h>
#include <crucible/MidiFile.h>
#include <crucible/Patch.h>
#include "TestSupport.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <cstring>
#include <fstream>
#include <thread>

using namespace crucible;
using namespace crucible::testing;
namespace fs = std::filesystem;

namespace
{
KeyLockSettings settings(KeyMode mode, int root, int scale, bool tieUp = false)
{
    KeyLockSettings s;
    s.mode = mode;
    s.root = root;
    s.mask = scale == kKeyScaleCustom ? static_cast<std::uint16_t>(0x0AB5) : keyScaleMask(scale);
    s.tieUp = tieUp;
    return s;
}

int popcount(unsigned v)
{
    int n = 0;
    for (; v != 0; v >>= 1)
        n += static_cast<int>(v & 1U);
    return n;
}

struct Rig
{
    Engine e;
    std::vector<float> out, l = std::vector<float>(128), r = std::vector<float>(128);
    Rig()
    {
        e.prepare(kRate, 128);
        e.resetParamsToDefaults();
        for (const auto& [id, v] : std::vector<std::pair<const char*, float>>{{"osc1_pos", 0.0f},
                                                                              {"osc1_level", 1.0f},
                                                                              {"amp_sustain", 1.0f},
                                                                              {"amp_attack", 0.001f},
                                                                              {"filter_cutoff", 20000.0f},
                                                                              {"master_gain", 0.0f}})
            e.setParam(id, v);
    }
    void block(const MidiInput* m = nullptr, int n = 0, const SeqHostTime* host = nullptr)
    {
        e.process(l.data(), r.data(), 128, m, n, host);
        out.insert(out.end(), l.begin(), l.end());
    }
    void run(int blocks)
    {
        for (int i = 0; i < blocks; ++i)
            block();
    }
    float peakLast(int blocks) const
    {
        float p = 0.0f;
        for (size_t i = out.size() - static_cast<size_t>(blocks) * 128; i < out.size(); ++i)
            p = std::max(p, std::abs(out[i]));
        return p;
    }
    double pitch(int blocks = 30) const
    {
        int crossings = 0, first = -1, last = -1;
        for (size_t i = out.size() - static_cast<size_t>(blocks) * 128 + 1; i < out.size(); ++i)
            if (out[i - 1] < 0.0f && out[i] >= 0.0f)
            {
                if (first < 0)
                    first = static_cast<int>(i);
                last = static_cast<int>(i);
                ++crossings;
            }
        return crossings < 3 ? 0.0 : (crossings - 1) * kRate / (last - first);
    }
};

MidiInput on(int note)
{
    return {0, 0x90, static_cast<std::uint8_t>(note), 100};
}
MidiInput off(int note)
{
    return {0, 0x80, static_cast<std::uint8_t>(note), 0};
}
} // namespace

class KeyLockTests : public UnitTest
{
public:
    KeyLockTests() : UnitTest("Key lock, key detection and key bus", "Engine") {}

    void runTest() override
    {
        beginTest("scales: the built-in list, sizes and names");
        {
            expectEquals(popcount(keyScaleMask(0)), 12);
            const int sizes[] = {12, 7, 7, 7, 7, 7, 7, 7, 7, 7, 5, 5, 6, 6, 8, 8};
            for (int i = 0; i < kKeyScaleCustom; ++i)
            {
                expectEquals(popcount(keyScaleMask(i)), sizes[i], String(keyScaleName(i)));
                expect((keyScaleMask(i) & 1U) != 0 || i == 0, "every scale contains its root");
            }
            expectEquals(std::string(keyScaleName(kKeyScaleCustom)), std::string("Custom"));
            expectEquals(static_cast<int>(keyScaleMask(kKeyScaleCustom)), 0);
            // Modes are rotations of the major scale.
            const unsigned major = keyScaleMask(1);
            for (const auto& [mode, degree] :
                 std::vector<std::pair<int, int>>{{5, 2}, {6, 4}, {7, 5}, {8, 7}, {2, 9}, {9, 11}})
            {
                const unsigned rotated = ((major >> degree) | (major << (12 - degree))) & 0xFFFU;
                expectEquals(static_cast<int>(keyScaleMask(mode)), static_cast<int>(rotated), String(keyScaleName(mode)));
            }
            Tuning t;
            std::string error;
            expect(t.loadScl("just\n7\n9/8\n5/4\n4/3\n3/2\n5/3\n15/8\n2/1\n", error), error);
            expectEquals(static_cast<int>(scaleMaskFromTuning(t)), static_cast<int>(keyScaleMask(1)), "Scala scale -> mask");
        }

        beginTest("quantiser: every scale, every root, every mode");
        {
            int checked = 0;
            for (int scale = 0; scale < kKeyScaleCustom + 1; ++scale)
                for (int root = 0; root < 12; ++root)
                {
                    const auto snapDown = settings(KeyMode::Snap, root, scale, false);
                    const auto snapUp = settings(KeyMode::Snap, root, scale, true);
                    const auto block = settings(KeyMode::Block, root, scale);
                    const auto remap = settings(KeyMode::Remap, root, scale);
                    const auto offS = settings(KeyMode::Off, root, scale);
                    for (int n = 0; n < 128; ++n)
                    {
                        ++checked;
                        const int d = quantiseNote(snapDown, n), u = quantiseNote(snapUp, n);
                        if (!(inKey(snapDown, d) && inKey(snapUp, u)))
                            expect(false, "snap out of key " + String(scale) + "/" + String(root) + "/" + String(n));
                        if (inKey(snapDown, n))
                        {
                            if (d != n || u != n || quantiseNote(block, n) != n)
                                expect(false, "in-key note must be unchanged");
                        }
                        else
                        {
                            if (quantiseNote(block, n) != -1)
                                expect(false, "block must mute");
                            // Nearest: nothing in key is closer.
                            const int dist = std::abs(d - n);
                            bool closer = false;
                            for (int m = std::max(0, n - dist + 1); m <= std::min(127, n + dist - 1); ++m)
                                closer = closer || (inKey(snapDown, m) && std::abs(m - n) < dist);
                            if (closer || std::abs(u - n) != dist)
                                expect(false, "snap must be nearest, tie only by direction");
                            if (u != d && !(d < n && u > n))
                                expect(false, "tie break goes down / up");
                        }
                        if (quantiseNote(offS, n) != n)
                            expect(false, "off is identity");
                        const int rm = quantiseNote(remap, n);
                        if (rm >= 0 && !inKey(remap, rm))
                            expect(false, "remap out of key " + String(scale) + "/" + String(root) + "/" + String(n));
                    }
                }
            expect(checked > 20000);
            // Tie-break example: C# in C major sits between C and D.
            expectEquals(quantiseNote(settings(KeyMode::Snap, 0, 1, false), 61), 60);
            expectEquals(quantiseNote(settings(KeyMode::Snap, 0, 1, true), 61), 62);
            // Remap: the white keys become the scale. C major is the identity on white keys.
            const auto cmaj = settings(KeyMode::Remap, 0, 1);
            for (const int n : {60, 62, 64, 65, 67, 69, 71, 72, 48, 36, 84})
                expectEquals(quantiseNote(cmaj, n), n, "white keys stay in C major");
            expectEquals(quantiseNote(cmaj, 61), 60, "a black key plays the white key below");
            expectEquals(quantiseNote(cmaj, 70), 69);
            const auto amin = settings(KeyMode::Remap, 9, 2); // A natural minor: A B C D E F G
            const int expected[] = {69, 71, 72, 74, 76, 77, 79, 81};
            const int whites[] = {60, 62, 64, 65, 67, 69, 71, 72};
            for (int i = 0; i < 8; ++i)
                expectEquals(quantiseNote(amin, whites[i]), expected[i], "white keys play A minor degrees");
            const auto penta = settings(KeyMode::Remap, 0, 10); // C D E G A
            expectEquals(quantiseNote(penta, 67), 69, "pentatonic: the fifth white key is degree 4 (A)");
            expectEquals(quantiseNote(penta, 69), 72, "the sixth white key wraps into the next octave");
            expectEquals(quantiseNote(settings(KeyMode::Snap, 0, 0), 61), 61, "chromatic changes nothing");
            auto empty = settings(KeyMode::Block, 0, 1);
            empty.mask = 0;
            expectEquals(quantiseNote(empty, 61), 61, "an empty mask means chromatic");
            expectEquals(quantiseNote(settings(KeyMode::Snap, 0, 1), -1), -1);
            expectEquals(quantiseNote(settings(KeyMode::Snap, 0, 1), 200), -1);
            const auto view = makeScaleView(settings(KeyMode::Snap, 2, 1));
            expect(view.inKey[2] && view.inKey[4] && view.inKey[6] && !view.inKey[0] && view.root == 2, "view of D major");
        }

        beginTest("quantiser: note-offs follow the started note; shared notes; repeats; reset");
        {
            NoteQuantiser q;
            q.configure(settings(KeyMode::Snap, 0, 1));
            auto a = q.noteOn(61);
            expect(a.out == 60 && a.stop == -1);
            q.configure(settings(KeyMode::Snap, 1, 1)); // C# major: 61 is in key now
            auto b = q.noteOff(61);
            expect(b.known && b.out == 60 && b.release, "the off releases what was started, not what 61 would be now");
            expect(!q.noteOff(61).known, "second off is unknown");
            q.configure(settings(KeyMode::Snap, 0, 1));
            // Two keys onto one note.
            expect(q.noteOn(61).out == 60 && q.noteOn(60).out == 60);
            expect(!q.noteOff(61).release, "still held by the other key");
            expect(q.noteOff(60).release, "released with the last one");
            // Repeated press of the same key: same note retriggers, a different note releases the old one.
            expect(q.noteOn(61).out == 60);
            auto again = q.noteOn(61);
            expect(again.out == 60 && again.stop == -1);
            q.configure(settings(KeyMode::Snap, 1, 1));
            auto moved = q.noteOn(61);
            expect(moved.out == 61 && moved.stop == 60, "a re-press under a new scale releases the old note");
            expect(q.noteOff(61).release);
            q.configure(settings(KeyMode::Block, 0, 1));
            auto muted = q.noteOn(61);
            expect(muted.out == -1 && !q.noteOff(61).release, "a muted key has nothing to release");
            q.noteOn(60);
            q.reset();
            expect(!q.noteOff(60).known);
        }

        beginTest("engine: Snap, Block, Remap, Off through the voices; mid-note scale change leaves no stuck note");
        {
            Rig rig;
            rig.e.setParam("key_scale", 1.0f);
            rig.e.setParam("key_root", 0.0f);
            const auto sound = [&](int mode, int note)
            {
                rig.e.setParam("key_mode", static_cast<float>(mode));
                rig.out.clear();
                const MidiInput m = on(note);
                rig.block(&m, 1);
                rig.run(120);
                const double hz = rig.pitch();
                const float level = rig.peakLast(30);
                const MidiInput o = off(note);
                rig.block(&o, 1);
                rig.run(150);
                return std::make_pair(hz, level);
            };
            expectWithinAbsoluteError(sound(0, 61).first, midiToHz(61), midiToHz(61) * 0.006, "off");
            expectWithinAbsoluteError(sound(1, 61).first, midiToHz(60), midiToHz(60) * 0.006, "snap down");
            rig.e.setParam("key_tie", 1.0f);
            expectWithinAbsoluteError(sound(1, 61).first, midiToHz(62), midiToHz(62) * 0.006, "snap up");
            expect(sound(2, 61).second < 1.0e-6f, "block: silence");
            expect(sound(2, 62).second > 0.1f, "block: in-key note plays");
            expectWithinAbsoluteError(sound(3, 61).first, midiToHz(60), midiToHz(60) * 0.006, "remap: black key");
            rig.e.setParam("key_root", 9.0f);
            rig.e.setParam("key_scale", 2.0f); // A minor
            expectWithinAbsoluteError(sound(3, 60).first, midiToHz(69), midiToHz(69) * 0.006, "remap: C plays A");

            // Scale change in the middle of a note.
            rig.e.setParam("key_root", 0.0f);
            rig.e.setParam("key_scale", 1.0f);
            rig.e.setParam("key_tie", 0.0f);
            rig.e.setParam("key_mode", 1.0f);
            rig.out.clear();
            MidiInput m = on(61);
            rig.block(&m, 1);
            rig.run(60);
            rig.e.setParam("key_root", 1.0f); // C# major
            rig.run(10);
            m = off(61);
            rig.block(&m, 1);
            rig.run(200);
            expect(rig.peakLast(20) < 1.0e-5f, "no stuck note after the scale changed mid-note");
            expectEquals(rig.e.activeVoiceCount(), 0);
            // Pitch bend is not quantised.
            rig.e.setParam("key_root", 0.0f);
            rig.out.clear();
            MidiInput bend[2] = {on(60), {0, 0xe0, 0x7f, 0x7f}};
            rig.block(bend, 2);
            rig.run(150);
            expectWithinAbsoluteError(rig.pitch(), midiToHz(62), midiToHz(62) * 0.01,
                                      "bend after quantising still bends by its own range");
        }

        beginTest("engine: the arpeggiator and the sequencer play in key");
        {
            const auto render = [](bool keyOn, const std::vector<int>& notes, bool arp)
            {
                Rig rig;
                if (keyOn)
                {
                    rig.e.setParam("key_mode", 1.0f);
                    rig.e.setParam("key_scale", 1.0f);
                }
                SeqHostTime host;
                host.hasBpm = true;
                host.bpm = 120.0;
                if (arp)
                {
                    rig.e.setParam("arp_on", 1.0f);
                    rig.e.setParam("arp_mode", 0.0f);
                    rig.e.setParam("arp_octaves", 2.0f);
                    std::vector<MidiInput> held;
                    for (const int n : notes)
                        held.push_back(on(n));
                    rig.block(held.data(), static_cast<int>(held.size()), &host);
                }
                else
                {
                    CRUCIBLE_HEAP(SeqBank, bank);
                    for (auto& p : bank.pattern)
                    {
                        p.lengthBars = 1;
                        for (size_t i = 0; i < p.steps.size(); ++i)
                        {
                            p.steps[i].on = true;
                            p.steps[i].note = static_cast<std::uint8_t>(notes[i % notes.size()]);
                            p.steps[i].gate = 0.5f;
                        }
                    }
                    rig.e.sequencer().replace(bank);
                    rig.e.setParam("seq_run", 1.0f);
                    rig.e.setParam("seq_tempo", 120.0f);
                    rig.block(nullptr, 0, &host);
                }
                for (int i = 0; i < 500; ++i)
                    rig.block(nullptr, 0, &host);
                return rig.out;
            };
            const std::vector<int> outOfKey = {61, 66, 70, 63};
            const std::vector<int> inKeyNotes = {60, 65, 69, 62}; // tie down: C#->C, F#->F, A#->A, D#->D
            for (const bool arp : {true, false})
            {
                const auto snapped = render(true, outOfKey, arp);
                const auto reference = render(false, inKeyNotes, arp);
                expectEquals(snapped.size(), reference.size());
                bool same = snapped.size() == reference.size();
                float peak = 0.0f;
                for (size_t i = 0; same && i < snapped.size(); ++i)
                {
                    same = snapped[i] == reference[i];
                    peak = std::max(peak, std::abs(snapped[i]));
                }
                expect(same && peak > 0.05f,
                       String(arp ? "arpeggiator" : "sequencer") + " output equals the pre-snapped notes bit for bit");
                const auto raw = render(false, outOfKey, arp);
                expect(raw != reference, "and the unquantised notes really differ");
            }
        }

        beginTest("engine: scale view for UIs, parameters and patches");
        {
            Rig rig;
            rig.e.setParam("key_mode", 1.0f);
            rig.e.setParam("key_root", 2.0f);
            rig.e.setParam("key_scale", 1.0f);
            rig.run(2);
            auto v = rig.e.keyScale();
            expect(v.mode == KeyMode::Snap && v.root == 2 && v.inKey[2] && v.inKey[1] && !v.inKey[3] && !v.following,
                   "D major view");
            rig.e.setParam("key_scale", static_cast<float>(kKeyScaleCustom));
            rig.e.setParam("key_mask", 145.0f); // root, major third, fifth
            rig.run(2);
            v = rig.e.keyScale();
            expect(v.inKey[2] && v.inKey[6] && v.inKey[9] && !v.inKey[4], "custom mask relative to the root");
            Tuning t;
            std::string error;
            t.loadScl("pent\n5\n9/8\n5/4\n3/2\n5/3\n2/1\n", error);
            rig.e.tuning().set(t);
            rig.e.setParam("key_scale", static_cast<float>(kKeyScaleTuning));
            rig.run(2);
            v = rig.e.keyScale();
            expect(v.inKey[2] && v.inKey[4] && v.inKey[6] && v.inKey[9] && v.inKey[11] && !v.inKey[3],
                   "scale taken from the loaded Scala file");
            expect(rig.e.model().indexOf("key_sync") >= 0 &&
                   rig.e.model().params()[static_cast<size_t>(rig.e.model().indexOf("key_scale"))].choices.size() ==
                       static_cast<size_t>(kNumKeyScales));
            const Patch p = capturePatch(rig.e);
            expectEquals(p.param("key_root", -1.0f), 2.0f);
            Engine other;
            applyPatch(other, p);
            expectEquals(other.getParam("key_mode"), 1.0f);
            expectEquals(other.getParam("key_scale"), static_cast<float>(kKeyScaleTuning));
        }

        beginTest("key detection: profiles, evidence, confidence, MIDI files, engine feed");
        {
            const auto histogram = [](std::initializer_list<std::pair<int, float>> w)
            {
                std::array<float, 12> h{};
                for (const auto& [pc, v] : w)
                    h[static_cast<size_t>(pc)] += v;
                return h;
            };
            auto c = KeyDetector::analyse(histogram({{0, 4}, {2, 2}, {4, 3}, {5, 2}, {7, 3.5f}, {9, 2}, {11, 1.5f}}));
            expect(c[0].root == 0 && !c[0].minor && c[0].name == "C major" && c[0].scale == 1, "C major");
            expect(c[0].confidence > 0.5f, "clear key: " + String(c[0].confidence));
            auto m = KeyDetector::analyse(
                histogram({{9, 5}, {11, 2}, {0, 3.5f}, {2, 2}, {4, 4}, {5, 1.5f}, {8, 1.5f}, {7, 0.5f}}));
            expect(m[0].root == 9 && m[0].minor && m[0].scale == 2 && m[0].name == "A minor", "A minor");
            auto flat = KeyDetector::analyse(histogram(
                {{0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 1}, {8, 1}, {9, 1}, {10, 1}, {11, 1}}));
            expect(flat[0].confidence < 0.05f, "chromatic cluster: no confidence");
            auto none = KeyDetector::analyse(histogram({}));
            expectEquals(none[0].confidence, 0.0f);
            expectEquals(static_cast<int>(c.size()), 24);
            auto few = KeyDetector::analyse(histogram({{0, 0.05f}, {4, 0.05f}, {7, 0.05f}}));
            expect(few[0].confidence < c[0].confidence, "little evidence, little confidence");
            for (size_t i = 1; i < c.size(); ++i)
                expect(c[i - 1].correlation >= c[i].correlation, "candidates sorted");

            KeyDetector d;
            for (const int n : {62, 64, 66, 67, 69, 71, 73, 74, 62, 66, 69, 74})
            {
                d.noteOn(n, 0.8f);
                d.tick(0.3f);
                d.noteOff(n);
            }
            const auto p = d.propose();
            expect(p.root == 2 && !p.minor, "D major from played notes: " + p.name);
            d.reset();
            expectEquals(d.propose().confidence, 0.0f);
            std::vector<TimedMidi> events;
            double t = 0.0;
            for (const int n : {57, 59, 60, 62, 64, 65, 64, 62, 60, 59, 57, 57, 64, 57})
            {
                events.push_back({t, 0x90, static_cast<std::uint8_t>(n), 90});
                events.push_back({t + 0.4, 0x80, static_cast<std::uint8_t>(n), 0});
                t += 0.5;
            }
            d.addMidiFile(events);
            const auto fp = d.propose();
            expect(fp.root == 9 && fp.minor, "A minor from a MIDI melody: " + fp.name);

            Rig rig;
            for (const int n : {60, 64, 67, 62, 65, 69, 71, 60, 67, 60})
            {
                const MidiInput mi = on(n);
                rig.block(&mi, 1);
                rig.run(20);
                const MidiInput mo = off(n);
                rig.block(&mo, 1);
            }
            const auto ep = rig.e.keyDetector().propose();
            expect(ep.root == 0 && !ep.minor && ep.confidence > 0.2f, "engine feeds the detector: " + ep.name);
        }

        beginTest("key bus: publish, last writer wins, generations, threads");
        {
            auto bus = KeyBus::createPrivate();
            expectEquals(bus->read().generation, 0u);
            expectEquals(bus->publish(7, 0x0AD5, true, 11), 1u);
            auto s = bus->read();
            expect(s.root == 7 && s.mask == 0x0AD5 && s.tieUp && s.generation == 1 && s.writer == 11);
            expectEquals(bus->publish(14, 0xFFFF, false, 12), 2u);
            s = bus->read();
            expect(s.root == 2 && s.mask == 0x0FFF && !s.tieUp && s.writer == 12, "last writer wins, values are normalised");
            expect(KeyBus::local() == KeyBus::local());

            // Reader never sees a torn state while two writers race.
            auto race = KeyBus::createPrivate();
            std::atomic<bool> stop{false};
            std::atomic<int> bad{0};
            const auto writer = [&](int base)
            {
                for (int i = 0; !stop.load(); ++i)
                {
                    const int root = (base + i) % 12;
                    race->publish(root, static_cast<std::uint16_t>(root * 100 + 1), root % 2 == 0);
                }
            };
            std::thread w1(writer, 0), w2(writer, 5);
            std::uint32_t last = 0;
            for (int i = 0; i < 200000; ++i)
            {
                const auto st = race->read();
                if (st.generation != 0 && (st.mask != st.root * 100 + 1 || st.tieUp != (st.root % 2 == 0)))
                    ++bad;
                if (st.generation < last)
                    ++bad;
                last = st.generation;
            }
            stop = true;
            w1.join();
            w2.join();
            expectEquals(bad.load(), 0, "consistent and monotonic under contention");
        }

        beginTest("key sync: leaders publish, followers adopt, last writer wins, local key kept without a bus");
        {
            auto bus = KeyBus::createPrivate();
            Rig leader, follower, other;
            leader.e.attachKeyBus(bus);
            follower.e.attachKeyBus(bus);
            other.e.attachKeyBus(bus);
            for (Rig* r : {&leader, &follower, &other})
            {
                r->e.setParam("key_mode", 1.0f);
                r->e.setParam("key_scale", 1.0f);
            }
            follower.e.setParam("key_sync", 1.0f);
            follower.run(2);
            expect(follower.e.keyScale().root == 0 && !follower.e.keyScale().following, "nothing published yet: local key");
            leader.e.setParam("key_sync", 2.0f);
            leader.e.setParam("key_root", 7.0f);
            leader.run(2);
            expectEquals(bus->read().root, 7);
            follower.run(2);
            auto v = follower.e.keyScale();
            expect(v.following && v.root == 7 && v.inKey[7] && v.inKey[1] == false && v.inKey[6], "follower adopts G major");
            // A follower really quantises with the bus key: F# is the leading tone of G major.
            follower.out.clear();
            MidiInput m = on(66);
            follower.block(&m, 1);
            follower.run(120);
            expectWithinAbsoluteError(follower.pitch(), midiToHz(66), midiToHz(66) * 0.006, "F# is in G major");
            m = off(66);
            follower.block(&m, 1);
            follower.run(100);
            leader.e.setParam("key_root", 0.0f);
            leader.run(2);
            follower.run(2);
            expect(follower.e.keyScale().root == 0 && follower.e.keyScale().busGeneration == bus->read().generation,
                   "follows the change");
            // Last writer wins.
            other.e.setParam("key_sync", 2.0f);
            other.e.setParam("key_root", 3.0f);
            other.run(2);
            follower.run(2);
            expectEquals(follower.e.keyScale().root, 3);
            leader.run(2); // the leader's key did not change, so it does not overwrite
            follower.run(2);
            expectEquals(follower.e.keyScale().root, 3);
            leader.e.setParam("key_root", 5.0f);
            leader.run(2);
            follower.run(2);
            expectEquals(follower.e.keyScale().root, 5);
            // Turning sync off returns to the local key; a follower never publishes.
            follower.e.setParam("key_sync", 0.0f);
            follower.run(2);
            expect(!follower.e.keyScale().following && follower.e.keyScale().root == 0);
            const auto gen = bus->read().generation;
            follower.e.setParam("key_root", 9.0f);
            follower.e.setParam("key_sync", 1.0f);
            follower.run(4);
            expectEquals(bus->read().generation, gen);
            // Without a bus attached, Follow keeps the local key.
            Rig lone;
            lone.e.setParam("key_mode", 1.0f);
            lone.e.setParam("key_sync", 1.0f);
            lone.e.setParam("key_root", 4.0f);
            lone.run(2);
            expect(lone.e.keyScale().root == 4 && !lone.e.keyScale().following);
        }
    }
};

static KeyLockTests keyLockTests;
