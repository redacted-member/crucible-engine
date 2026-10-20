// SPDX-License-Identifier: MIT
#include <crucible/ArpPatternIO.h>
#include <crucible/Arpeggiator.h>
#include <crucible/EngineCore.h>
#include "TestSupport.h"

#include <filesystem>

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct Ev
{
    long long at;
    bool on;
    int note;
    float velocity;
};

struct Key
{
    long long at;
    bool on;
    int note;
    float velocity = 0.8f;
};

std::vector<Ev> runArp(Arpeggiator& arp, const ArpSettings& s, const std::vector<Key>& keys, long long total,
                       int block = 512, const ArpPattern* grid = nullptr, const SeqHostTime* hostIn = nullptr)
{
    std::vector<Ev> out;
    std::vector<ArpOutput> buf(Arpeggiator::maxEvents);
    for (long long pos = 0; pos < total; pos += block)
    {
        const int n = static_cast<int>(std::min<long long>(block, total - pos));
        std::vector<ArpInput> in;
        for (const auto& k : keys)
            if (k.at >= pos && k.at < pos + n)
                in.push_back({static_cast<int>(k.at - pos), k.on, k.note, k.velocity});
        SeqHostTime host;
        if (hostIn != nullptr)
        {
            host = *hostIn;
            host.ppq = hostIn->ppq + static_cast<double>(pos) / kRate * hostIn->bpm / 60.0;
        }
        const int c =
            arp.process(in.data(), static_cast<int>(in.size()), s, grid, host, n, buf.data(), static_cast<int>(buf.size()));
        for (int i = 0; i < c; ++i)
            out.push_back({pos + buf[static_cast<size_t>(i)].offset, buf[static_cast<size_t>(i)].on,
                           buf[static_cast<size_t>(i)].note, buf[static_cast<size_t>(i)].velocity});
    }
    return out;
}

std::vector<Ev> ons(const std::vector<Ev>& e)
{
    std::vector<Ev> o;
    for (const auto& x : e)
        if (x.on)
            o.push_back(x);
    return o;
}

std::vector<int> notesOf(const std::vector<Ev>& e, size_t count)
{
    std::vector<int> n;
    for (const auto& x : ons(e))
        if (n.size() < count)
            n.push_back(x.note);
    return n;
}

ArpSettings basic(int mode = 0)
{
    ArpSettings s;
    s.on = true;
    s.mode = mode;
    s.rate = 10; // 1/16
    s.hostSync = false;
    s.tempo = 120.0f; // a 16th is 6000 samples at 48 kHz
    s.gate = 0.5f;
    return s;
}

std::vector<Key> chord(std::initializer_list<int> notes, long long at = 0)
{
    std::vector<Key> k;
    for (const int n : notes)
        k.push_back({at, true, n});
    return k;
}
} // namespace

class ArpTests : public UnitTest
{
public:
    ArpTests() : UnitTest("Arpeggiator", "Engine") {}

    void runTest() override
    {
        beginTest("up mode: sample-accurate steps and gates at the internal tempo");
        {
            Arpeggiator arp;
            arp.prepare(kRate);
            const auto ev = runArp(arp, basic(), chord({60, 64, 67}, 100), 6 * 6000);
            const auto on = ons(ev);
            expect(on.size() >= 5);
            for (size_t i = 0; i < 5 && i < on.size(); ++i)
                expectEquals(on[i].at, static_cast<long long>(100 + 6000 * static_cast<long long>(i)));
            expect(notesOf(ev, 5) == std::vector<int>({60, 64, 67, 60, 64}));
            for (const auto& e : ev)
                if (!e.on && e.note == 60)
                {
                    expectEquals(e.at, 100LL + 3000LL);
                    break;
                }
        }

        beginTest("modes: down, up-down, down-up, as played, converge, chord, random (deterministic)");
        {
            const auto seq = [&](int mode, const std::vector<Key>& keys, size_t n)
            {
                Arpeggiator arp;
                arp.prepare(kRate);
                return notesOf(runArp(arp, basic(mode), keys, 6000LL * static_cast<long long>(n + 1)), n);
            };
            const auto c3 = chord({64, 60, 67});
            expect(seq(static_cast<int>(ArpMode::Down), c3, 4) == std::vector<int>({67, 64, 60, 67}));
            expect(seq(static_cast<int>(ArpMode::UpDown), c3, 6) == std::vector<int>({60, 64, 67, 64, 60, 64}));
            expect(seq(static_cast<int>(ArpMode::DownUp), c3, 5) == std::vector<int>({67, 64, 60, 64, 67}));
            std::vector<Key> played = {{0, true, 67}, {0, true, 60}, {0, true, 64}};
            expect(seq(static_cast<int>(ArpMode::AsPlayed), played, 4) == std::vector<int>({67, 60, 64, 67}));
            expect(seq(static_cast<int>(ArpMode::Converge), chord({60, 62, 64, 65}), 4) ==
                   std::vector<int>({60, 65, 62, 64}));
            {
                Arpeggiator arp;
                arp.prepare(kRate);
                const auto ev = runArp(arp, basic(static_cast<int>(ArpMode::Chord)), c3, 12500);
                const auto on = ons(ev);
                expectEquals(static_cast<int>(on.size()), 9); // three steps (0, 6000, 12000) x three notes
                expect(on[0].at == on[2].at);
            }
            const auto r1 = seq(static_cast<int>(ArpMode::Random), c3, 32);
            const auto r2 = seq(static_cast<int>(ArpMode::Random), c3, 32);
            expect(r1 == r2, "random mode is deterministic");
            std::set<int> seen(r1.begin(), r1.end());
            expectEquals(static_cast<int>(seen.size()), 3);
            auto s = basic();
            s.octaves = 2;
            Arpeggiator arp;
            arp.prepare(kRate);
            expect(notesOf(runArp(arp, s, chord({60, 64, 67}), 6000LL * 7), 6) ==
                   std::vector<int>({60, 64, 67, 72, 76, 79}));
        }

        beginTest("rates: dotted, triplet and free; swing delays odd steps; results do not depend on block size");
        {
            const auto times = [&](ArpSettings s, int block)
            {
                Arpeggiator arp;
                arp.prepare(kRate);
                std::vector<long long> t;
                for (const auto& e : ons(runArp(arp, s, chord({60}), 48000, block)))
                    t.push_back(e.at);
                return t;
            };
            auto s = basic();
            s.rate = 9; // 1/8T = 1/3 beat = 8000 samples
            auto t = times(s, 512);
            expect(t.size() > 2 && t[1] == 8000 && t[2] == 16000);
            s.rate = 8; // 1/8. = 0.75 beat = 18000
            t = times(s, 512);
            expect(t.size() > 1 && t[1] == 18000);
            s.rate = 15; // free
            s.freeRateHz = 10.0f;
            t = times(s, 512);
            expect(t.size() > 2 && t[1] == 4800 && t[2] == 9600);
            s = basic();
            s.swingPercent = 50.0f;
            t = times(s, 512);
            expect(t.size() > 3 && t[1] == 6000 + 3000 && t[2] == 12000 && t[3] == 18000 + 3000, "swing");
            expect(times(s, 1) == times(s, 512) && times(s, 37) == times(s, 512), "block size independent");
        }

        beginTest("host clock: steps land on the host's grid lines; host tempo sets the rate");
        {
            auto s = basic();
            s.hostSync = true;
            SeqHostTime host;
            host.hasBpm = true;
            host.bpm = 150.0;
            host.hasPpq = true;
            host.playing = true;
            host.ppq = 0.1; // just after a grid line: the first step waits for the next 16th
            Arpeggiator arp;
            arp.prepare(kRate);
            const auto on = ons(runArp(arp, s, chord({60}), 48000, 256, nullptr, &host));
            const double samplesPerBeat = kRate * 60.0 / 150.0;
            expect(!on.empty());
            const double firstBeat = 0.1 + static_cast<double>(on[0].at) / samplesPerBeat;
            expectWithinAbsoluteError(firstBeat, 0.25, 1.0 / samplesPerBeat + 1.0e-9);
            expect(on.size() > 1 && std::llabs(on[1].at - on[0].at - static_cast<long long>(0.25 * samplesPerBeat)) <= 1);
        }

        beginTest("gate above one overlaps; a repeated note is ended exactly where it restarts");
        {
            auto s = basic();
            s.gate = 1.4f;
            Arpeggiator arp;
            arp.prepare(kRate);
            const auto ev = runArp(arp, s, chord({60}), 20000);
            // Expected: on 0, off 6000, on 6000, off 12000, on 12000 ... with the off before the on.
            bool ordered = true;
            int sounding = 0;
            for (const auto& e : ev)
            {
                sounding += e.on ? 1 : -1;
                ordered = ordered && sounding <= 1 && sounding >= 0;
            }
            expect(ordered, "never two copies of the same note");
            auto s2 = basic();
            s2.gate = 1.4f;
            Arpeggiator arp2;
            arp2.prepare(kRate);
            const auto ev2 = runArp(arp2, s2, chord({60, 67}), 20000);
            long long off60 = -1;
            for (const auto& e : ev2)
                if (!e.on && e.note == 60)
                {
                    off60 = e.at;
                    break;
                }
            expectEquals(off60, static_cast<long long>(6000 * 1.4)); // a different note overlaps legato
        }

        beginTest("latch keeps the chord after release; a new press replaces it; without latch the arp stops");
        {
            auto s = basic();
            s.latch = true;
            std::vector<Key> keys = {{0, true, 60},     {0, true, 64},     {1000, false, 60},
                                     {1000, false, 64}, {30000, true, 70}, {30100, false, 70}};
            Arpeggiator arp;
            arp.prepare(kRate);
            const auto ev = runArp(arp, s, keys, 60000);
            std::vector<int> before, after;
            for (const auto& e : ons(ev))
                (e.at < 30000 ? before : after).push_back(e.note);
            expect(before.size() == 5 && before[2] == 60 && before[3] == 64, "latched steps keep playing");
            expect(!after.empty() && std::all_of(after.begin(), after.end(), [](int n) { return n == 70; }));
            s.latch = false;
            Arpeggiator arp2;
            arp2.prepare(kRate);
            const auto ev2 = runArp(arp2, s, keys, 29000);
            expectEquals(static_cast<int>(ons(ev2).size()), 1);
            expectEquals(arp2.heldCount(), 0);
        }

        beginTest("velocity modes and ratchets");
        {
            auto s = basic();
            std::vector<Key> keys = {{0, true, 60, 0.3f}};
            Arpeggiator a1;
            a1.prepare(kRate);
            expectWithinAbsoluteError(ons(runArp(a1, s, keys, 100))[0].velocity, 0.3f, 1.0e-6f);
            s.velocityMode = static_cast<int>(ArpVelocity::Fixed);
            s.fixedVelocity = 127;
            Arpeggiator a2;
            a2.prepare(kRate);
            expectWithinAbsoluteError(ons(runArp(a2, s, keys, 100))[0].velocity, 1.0f, 1.0e-6f);
            s.velocityMode = static_cast<int>(ArpVelocity::Accent);
            s.accent = 0.5f;
            Arpeggiator a3;
            a3.prepare(kRate);
            const auto acc = ons(runArp(a3, s, keys, 6000 * 5));
            expect(acc.size() == 5 && acc[0].velocity == 1.0f && acc[1].velocity == 0.5f && acc[4].velocity == 1.0f);
            s = basic();
            s.ratchet = 3;
            Arpeggiator a4;
            a4.prepare(kRate);
            const auto r = ons(runArp(a4, s, keys, 5999));
            expect(r.size() == 3 && r[1].at == 2000 && r[2].at == 4000);
        }

        beginTest("pattern grid: rests, ties, octave, transpose, probability, per-step ratchet, direction");
        {
            ArpPattern g;
            g.length = 4;
            g.steps[1].tie = true;
            g.steps[2].on = false;
            g.steps[3].octave = 1;
            g.steps[3].transpose = 2;
            auto s = basic();
            s.useGrid = true;
            Arpeggiator arp;
            arp.prepare(kRate);
            const auto ev = runArp(arp, s, chord({60, 64}), 6000LL * 4, 512, &g);
            const auto on = ons(ev);
            // step 0: 60 tied through step 1, step 2: rest, step 3: 64 + 12 + 2
            expect(on.size() == 2 && on[0].note == 60 && on[1].note == 78 && on[1].at == 18000);
            long long off60 = -1;
            for (const auto& e : ev)
                if (!e.on && e.note == 60)
                    off60 = e.at;
            expectEquals(off60, 6000LL + 3000LL);

            ArpPattern never;
            never.length = 2;
            never.steps[0].probability = 0.0f;
            never.steps[1].ratchet = 2;
            Arpeggiator arp2;
            arp2.prepare(kRate);
            const auto o2 = ons(runArp(arp2, s, chord({60}), 6000LL * 2, 512, &never));
            expect(o2.size() == 2 && o2[0].at == 6000 && o2[1].at == 9000);

            ArpPattern rev;
            rev.length = 3;
            rev.direction = 1;
            rev.steps[0].transpose = 0;
            rev.steps[1].transpose = 1;
            rev.steps[2].transpose = 2;
            Arpeggiator arp3;
            arp3.prepare(kRate);
            expect(notesOf(runArp(arp3, s, chord({60}), 6000LL * 4, 512, &rev), 3) == std::vector<int>({62, 61, 60}));

            ArpPattern half;
            half.length = 16;
            for (auto& st : half.steps)
                st.probability = 0.5f;
            Arpeggiator a4, a5;
            a4.prepare(kRate);
            a5.prepare(kRate);
            const auto p1 = ons(runArp(a4, s, chord({60}), 6000LL * 200, 512, &half));
            const auto p2 = ons(runArp(a5, s, chord({60}), 6000LL * 200, 512, &half));
            expect(p1.size() == p2.size() && p1.size() > 70 && p1.size() < 130,
                   "probability " + String(static_cast<int>(p1.size())));

            ArpPattern ser = g;
            ser.direction = 2;
            ser.steps[9].velocity = 90;
            ser.steps[9].probability = 0.25f;
            ArpPattern back;
            expect(arpPatternDeserialise(arpPatternSerialise(ser), back));
            expect(back.length == 4 && back.direction == 2 && back.steps[1].tie && !back.steps[2].on &&
                   back.steps[3].octave == 1 && back.steps[9].velocity == 90 &&
                   std::abs(back.steps[9].probability - 0.25f) < 1.0e-4f);
            expect(!arpPatternDeserialise("", back));
        }

        beginTest("engine: the arp drives the voices from MIDI, coexists with the sequencer, releases when switched off");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            auto v = ParameterMap::defaultValues();
            setValue(v, "arp_on", 1.0f);
            setValue(v, "arp_sync", 0.0f);
            setValue(v, "arp_mode", 0.0f);
            setValue(v, "amp_release", 0.01f);
            core.arpPattern().edit([](ArpPattern& p) { p.steps[1].on = false; });
            std::vector<float> l(512), r(512);
            MidiInput midi[2] = {{10, 0x90, 60, 100}, {10, 0x90, 67, 100}};
            core.process(l.data(), r.data(), 512, v.data(), midi, 2, {});
            float peak = 0.0f;
            for (int b = 0; b < 30; ++b)
            {
                core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
                for (const float x : l)
                    peak = std::max(peak, std::abs(x));
            }
            expect(peak > 0.01f, "arp plays");
            expect(core.arpeggiator().heldCount() == 2);
            // The sequencer still reaches the voices directly while the arp runs.
            setValue(v, "seq_run", 1.0f);
            CRUCIBLE_HEAP(SeqBank, bank);
            bank.pattern[0].steps[0] = {true, 48, 110, 1, 0.5f, 1.0f, {}};
            core.store().replace(bank);
            core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
            expect(core.sequencerRunning());
            setValue(v, "seq_run", 0.0f);
            setValue(v, "arp_on", 0.0f);
            for (int b = 0; b < 40; ++b)
                core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
            expectEquals(core.engine().activeVoiceCount(), 0);
        }
        beginTest("modes x octaves: every deterministic mode against a reference model");
        {
            const std::vector<int> base = {60, 64, 67, 71};
            for (int octaves = 1; octaves <= 4; ++octaves)
            {
                std::vector<int> pool;
                for (int o = 0; o < octaves; ++o)
                    for (const int n : base)
                        pool.push_back(n + 12 * o);
                const int count = static_cast<int>(pool.size());
                const auto model = [&](int mode, int i)
                {
                    switch (mode)
                    {
                    case static_cast<int>(ArpMode::Up):
                        return pool[static_cast<size_t>(i % count)];
                    case static_cast<int>(ArpMode::Down):
                        return pool[static_cast<size_t>(count - 1 - i % count)];
                    case static_cast<int>(ArpMode::UpDown):
                    {
                        const int period = std::max(1, 2 * (count - 1));
                        const int m = i % period;
                        return pool[static_cast<size_t>(m < count ? m : period - m)];
                    }
                    case static_cast<int>(ArpMode::DownUp):
                    {
                        const int period = std::max(1, 2 * (count - 1));
                        const int m = i % period;
                        return pool[static_cast<size_t>(count - 1 - (m < count ? m : period - m))];
                    }
                    default:
                    {
                        const int m = i % count;
                        return pool[static_cast<size_t>(m % 2 == 0 ? m / 2 : count - 1 - m / 2)];
                    }
                    }
                };
                for (const int mode :
                     {static_cast<int>(ArpMode::Up), static_cast<int>(ArpMode::Down), static_cast<int>(ArpMode::UpDown),
                      static_cast<int>(ArpMode::DownUp), static_cast<int>(ArpMode::Converge)})
                {
                    auto st = basic(mode);
                    st.octaves = octaves;
                    Arpeggiator arp;
                    arp.prepare(kRate);
                    const size_t n = static_cast<size_t>(count) * 3;
                    const auto got =
                        notesOf(runArp(arp, st, chord({60, 64, 67, 71}), 6000LL * static_cast<long long>(n + 1)), n);
                    bool ok = got.size() == n;
                    for (size_t i = 0; ok && i < n; ++i)
                        ok = got[i] == model(mode, static_cast<int>(i));
                    expect(ok, "mode " + String(mode) + " octaves " + String(octaves));
                }
            }
            expectEquals(static_cast<int>(arpModeNames().size()), static_cast<int>(ArpMode::Count));
        }

        beginTest("ratchets divide the step exactly; a host tempo change changes the step length at once");
        {
            for (int ratchet = 2; ratchet <= 4; ++ratchet)
            {
                auto st = basic();
                st.ratchet = ratchet;
                Arpeggiator arp;
                arp.prepare(kRate);
                const auto on = ons(runArp(arp, st, chord({60}), 6000 * 3, 777));
                bool ok = static_cast<int>(on.size()) >= ratchet * 2;
                for (size_t i = 0; ok && i < on.size(); ++i)
                {
                    const long long step = static_cast<long long>(i) / ratchet, r = static_cast<long long>(i) % ratchet;
                    ok = std::llabs(on[i].at - (step * 6000 + std::llround(6000.0 / ratchet * static_cast<double>(r)))) <= 1;
                }
                expect(ok, "ratchet " + String(ratchet));
            }
            auto st = basic();
            st.hostSync = true;
            Arpeggiator arp;
            arp.prepare(kRate);
            std::vector<ArpOutput> buf(Arpeggiator::maxEvents);
            std::vector<ArpInput> keys = {{0, true, 60, 0.8f}};
            SeqHostTime host;
            host.hasBpm = host.hasPpq = host.playing = true;
            host.bpm = 120.0;
            std::vector<long long> starts;
            long long sample = 0;
            double ppq = 0.0;
            for (int block = 0; block < 300; ++block)
            {
                host.bpm = block < 60 ? 120.0 : 60.0;
                host.ppq = ppq;
                const int c = arp.process(block == 0 ? keys.data() : nullptr, block == 0 ? 1 : 0, st, nullptr, host, 512,
                                          buf.data(), static_cast<int>(buf.size()));
                for (int i = 0; i < c; ++i)
                    if (buf[static_cast<size_t>(i)].on)
                        starts.push_back(sample + buf[static_cast<size_t>(i)].offset);
                sample += 512;
                ppq += 512.0 / kRate * host.bpm / 60.0;
            }
            bool fast = true, slow = true;
            for (size_t i = 1; i < starts.size(); ++i)
            {
                const long long d = starts[i] - starts[i - 1];
                if (starts[i] < 60 * 512 - 6000)
                    fast = fast && std::llabs(d - 6000) <= 1;
                else if (starts[i - 1] > 60 * 512 + 12000)
                    slow = slow && std::llabs(d - 12000) <= 1;
            }
            expect(starts.size() > 10 && fast && slow, "6000 samples per 1/16 at 120 bpm, 12000 at 60 bpm");
        }

        beginTest("probability groups: exactly one alternative per pass; weights; deterministic");
        {
            ArpPattern p;
            p.length = 8;
            for (int i = 0; i < 8; ++i)
            {
                p.steps[static_cast<size_t>(i)].on = true;
                p.steps[static_cast<size_t>(i)].transpose = static_cast<std::int8_t>(i);
            }
            for (int i = 0; i < 4; ++i)
            {
                p.steps[static_cast<size_t>(i)].group = 1;
                p.steps[static_cast<size_t>(i)].probability = 0.25f;
            }
            auto st = basic();
            st.useGrid = true;
            const auto play = [&](const ArpPattern& pat, std::uint32_t seed)
            {
                Arpeggiator arp;
                arp.prepare(kRate);
                arp.setSeed(seed);
                return ons(runArp(arp, st, chord({60}), 6000LL * 8 * 200, 4096, &pat));
            };
            auto e = play(p, 7);
            std::array<int, 8> hits{};
            std::vector<int> perPass(200, 0);
            for (const auto& x : e)
            {
                const int i = x.note - 60;
                ++hits[static_cast<size_t>(i)];
                if (i < 4)
                    ++perPass[static_cast<size_t>(x.at / (6000 * 8))];
            }
            bool exactlyOne = true;
            for (const int c : perPass)
                exactlyOne = exactlyOne && c == 1;
            expect(exactlyOne, "weights adding up to 1: exactly one of the four per pass");
            for (int i = 4; i < 8; ++i)
                expectEquals(hits[static_cast<size_t>(i)], 200, "steps outside the group always play");
            for (int i = 0; i < 4; ++i)
                expect(hits[static_cast<size_t>(i)] > 25 && hits[static_cast<size_t>(i)] < 80,
                       "balanced: " + String(hits[static_cast<size_t>(i)]));
            const auto again = play(p, 7);
            bool same = again.size() == e.size();
            for (size_t i = 0; same && i < e.size(); ++i)
                same = e[i].at == again[i].at && e[i].note == again[i].note;
            expect(same, "deterministic for a seed");
            const auto other = play(p, 99);
            bool differs = other.size() != e.size();
            for (size_t i = 0; !differs && i < e.size(); ++i)
                differs = e[i].note != other[i].note;
            expect(differs, "another seed makes other choices");
            // Weights below one leave room for silence.
            for (int i = 0; i < 4; ++i)
                p.steps[static_cast<size_t>(i)].probability = (i < 2) ? 0.25f : 0.0f;
            const auto some = play(p, 7);
            int groupHits = 0;
            for (const auto& x : some)
                groupHits += (x.note - 60) < 4 ? 1 : 0;
            expect(groupHits > 60 && groupHits < 140, "total weight 0.5: about half of the passes: " + String(groupHits));
            // Two groups are independent; a rest or a tie never wins.
            p.steps[0].group = 2;
            p.steps[0].probability = 1.0f;
            p.steps[1].group = 2;
            p.steps[1].on = false;
            p.steps[1].probability = 1.0f;
            int zeros = 0;
            for (const auto& x : play(p, 7))
                zeros += x.note == 60 ? 1 : 0;
            expectEquals(zeros, 200, "an inactive member does not take part: the single playable step always wins");
            // Text form keeps the group; older text without it loads with none.
            ArpPattern back;
            expect(arpPatternDeserialise(arpPatternSerialise(p), back));
            for (int i = 0; i < 8; ++i)
                expectEquals(static_cast<int>(back.steps[static_cast<size_t>(i)].group),
                             static_cast<int>(p.steps[static_cast<size_t>(i)].group));
            expect(arpPatternDeserialise("8,0;2,1,0,0,1.0000,0,0.5000,1,0", back) && back.steps[2].group == 0 &&
                   back.steps[2].probability == 0.5f);
        }

        beginTest("pattern import / export: JSON files and MIDI clips");
        {
            ArpPattern p;
            p.length = 8;
            p.direction = 2;
            for (int i = 0; i < 8; ++i)
            {
                auto& s = p.steps[static_cast<size_t>(i)];
                s.on = i != 1 && i != 6;
                s.velocity = static_cast<std::uint8_t>(60 + i * 5);
                s.gate = 0.5f;
                s.group = static_cast<std::uint8_t>(i % 3);
                s.probability = 0.5f + 0.05f * static_cast<float>(i);
            }
            p.steps[2].transpose = 7;
            p.steps[2].ratchet = 2;
            p.steps[3].octave = 1;
            p.steps[3].gate = 1.0f;
            p.steps[4].tie = true;
            p.steps[5].transpose = -5;
            p.steps[5].ratchet = 3;
            ArpPattern back;
            std::string error;
            expect(arpPatternFromJson(arpPatternToJson(p), back, error), error);
            expectEquals(arpPatternSerialise(back), arpPatternSerialise(p));
            bool allSame = back.length == p.length && back.direction == p.direction;
            for (int i = 0; i < kArpMaxSteps; ++i)
                allSame = allSame && arpPatternSerialise(back) == arpPatternSerialise(p);
            expect(allSame);
            expect(!arpPatternFromJson("{\"format\": \"other\"}", back, error) &&
                   !arpPatternFromJson("nonsense", back, error));
            expect(!arpPatternFromJson("{\"format\": \"crucible-arp-pattern\", \"version\": 7}", back, error),
                   "newer version refused");
            const std::string path = (std::filesystem::temp_directory_path() / "crucible_arp_pattern_test.json").string();
            expect(saveArpPatternFile(path, p) && loadArpPatternFile(path, back, error) &&
                   arpPatternSerialise(back) == arpPatternSerialise(p));
            std::filesystem::remove(path);
            expect(!loadArpPatternFile("/nonexistent/arp.json", back, error));

            // MIDI: export, read it back with the file reader, import.
            const auto bytes = arpPatternToMidi(p, 60, 120.0, 4);
            std::vector<TimedMidi> events;
            MidiFileInfo info;
            expect(readMidiFile(bytes.data(), bytes.size(), events, info, error), error);
            expectWithinAbsoluteError(info.firstTempoBpm, 120.0, 0.01);
            ArpPattern imported;
            expect(arpPatternFromMidi(events, info.firstTempoBpm, 4, 60, imported, error), error);
            expectEquals(imported.length, 8);
            for (int i = 0; i < 8; ++i)
            {
                const auto& a = p.steps[static_cast<size_t>(i)];
                const auto& b = imported.steps[static_cast<size_t>(i)];
                expect(a.on == b.on, "on of step " + String(i));
                if (!a.on)
                    continue;
                expectEquals(static_cast<int>(a.transpose) + 12 * a.octave, static_cast<int>(b.transpose) + 12 * b.octave,
                             "pitch of step " + String(i));
                if (!a.tie)
                    expectEquals(static_cast<int>(a.velocity), static_cast<int>(b.velocity),
                                 "velocity of step " + String(i));
                expectEquals(static_cast<int>(a.ratchet), static_cast<int>(b.ratchet), "ratchet of step " + String(i));
                expectEquals(a.tie, b.tie, "tie of step " + String(i));
                if (!a.tie && a.gate <= 1.0f && !(i == 3))
                    expectWithinAbsoluteError(b.gate, a.gate, 0.06f, "gate of step " + String(i));
            }
            expect(imported.steps[4].tie && imported.steps[3].on, "the long note comes back as a tie");
            // The imported pattern plays the same notes as the exported one (probability aside).
            ArpPattern deterministic = p;
            for (auto& s : deterministic.steps)
            {
                s.probability = 1.0f;
                s.group = 0;
            }
            auto st = basic();
            st.useGrid = true;
            deterministic.direction = 0;
            imported.direction = 0;
            const auto play = [&](const ArpPattern& pat)
            {
                Arpeggiator arp;
                arp.prepare(kRate);
                std::vector<std::pair<long long, int>> v;
                for (const auto& x : ons(runArp(arp, st, chord({60}), 6000LL * 17, 4096, &pat)))
                    v.emplace_back(x.at, x.note);
                return v;
            };
            expect(play(deterministic) == play(imported), "same notes at the same samples");

            // Chords keep the lowest note; a base note of -1 uses the lowest note of the clip.
            std::vector<TimedMidi> clip = {{0.0, 0x90, 67, 100}, {0.0, 0x90, 64, 100},  {0.2, 0x80, 67, 0},
                                           {0.2, 0x80, 64, 0},   {0.125, 0x90, 72, 90}, {0.225, 0x80, 72, 0}};
            ArpPattern fromClip;
            expect(arpPatternFromMidi(clip, 120.0, 4, -1, fromClip, error), error);
            expect(fromClip.steps[0].on && fromClip.steps[0].transpose == 0 && fromClip.steps[1].on &&
                       fromClip.steps[1].transpose == 8,
                   "chord -> lowest, relative to the lowest note");
            expectEquals(fromClip.length, 2);
            expect(!arpPatternFromMidi({}, 120.0, 4, 60, fromClip, error) && !error.empty(), "an empty clip is refused");
            expect(!arpPatternFromMidiFile("/nonexistent/clip.mid", 4, 60, fromClip, error));
        }
    }
};

static ArpTests arpTests;
