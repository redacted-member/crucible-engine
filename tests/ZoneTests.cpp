// SPDX-License-Identifier: MIT
// Multisample zones: selection by key and velocity, equal power crossfades, round robin, the zone path of the sample
// oscillator, patch round trip with old patches, importers, the C API and the memory caps.

#include "GraphSupport.h"

#include <crucible/Engine.h>
#include <crucible/Patch.h>
#include <crucible/PresetLibrary.h>
#include <crucible/Zones.h>
#include <crucible/crucible.h>

#include <filesystem>
#include <fstream>

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr double kTau = 6.283185307179586;

AudioData tone(double hz, double seconds = 1.0, bool stereo = false)
{
    AudioData a;
    a.sampleRate = 48000.0;
    a.channels.resize(stereo ? 2 : 1);
    for (auto& ch : a.channels)
        for (int i = 0; i < static_cast<int>(seconds * 48000.0); ++i)
            ch.push_back(0.5f * static_cast<float>(std::sin(kTau * hz * i / 48000.0)));
    return a;
}

double amplitudeAt(const std::vector<float>& s, double hz, size_t from, size_t to)
{
    double re = 0.0, im = 0.0;
    for (size_t i = from; i < to && i < s.size(); ++i)
    {
        re += s[i] * std::cos(kTau * hz * static_cast<double>(i) / 48000.0);
        im += s[i] * std::sin(kTau * hz * static_cast<double>(i) / 48000.0);
    }
    return 2.0 * std::sqrt(re * re + im * im) / static_cast<double>(to - from);
}

Zone zone(int id, int keyLo, int keyHi, float root, int velLo = 0, int velHi = 127)
{
    Zone z;
    z.sampleId = id;
    z.keyLo = static_cast<std::uint8_t>(keyLo);
    z.keyHi = static_cast<std::uint8_t>(keyHi);
    z.velLo = static_cast<std::uint8_t>(velLo);
    z.velHi = static_cast<std::uint8_t>(velHi);
    z.root = root;
    return z;
}

ZoneSet setOf(std::vector<Zone> zones, int samples = 0)
{
    ZoneSet s;
    s.zones = std::move(zones);
    s.samples.resize(static_cast<size_t>(samples));
    return s;
}

std::vector<int> picked(const ZoneSet& set, int key, int vel, const ZoneSelect& o = {})
{
    std::array<ZoneLayer, kMaxZoneLayers> out;
    const int n = selectZones(set, key, vel, o, out);
    std::vector<int> r;
    for (int i = 0; i < n; ++i)
        r.push_back(out[static_cast<size_t>(i)].zone);
    return r;
}

std::vector<float> zonePatch(const std::vector<std::pair<std::string, float>>& extra = {})
{
    auto v = graphPatch();
    setValue(v, "osc1_type", 1.0f);
    setValue(v, "osc1_level", 1.0f);
    setValue(v, "osc1_smp_zone_mode", 1.0f);
    setValue(v, "amp_release", 0.05f);
    for (const auto& [id, value] : extra)
        setValue(v, id, value);
    return v;
}

// Sample 0 = 220 Hz (own sample), 1 = 440 Hz, 2 = 880 Hz; roots 57, 69, 81 (A3, A4, A5 at 220, 440, 880).
void threeSamples(SynthEngine& e)
{
    std::string err;
    e.sampleBank().load(0, tone(220.0), "a3", err);
    e.zoneBank().addSample(0, tone(440.0), "a4", err);
    e.zoneBank().addSample(0, tone(880.0), "a5", err);
}

std::vector<Zone> threeZones()
{
    return {zone(0, 0, 59, 57.0f), zone(1, 60, 71, 69.0f), zone(2, 72, 127, 81.0f)};
}

Rendered play(const std::function<void(SynthEngine&)>& setup, const std::vector<float>& values, int note, float velocity,
              int samples, int block = 512)
{
    SynthEngine engine;
    setup(engine);
    prepare(engine, values);
    engine.noteOn(note, velocity);
    return renderBlocks(engine, samples, block, true);
}

double freqOf(int note, int root, double sampleHz)
{
    return sampleHz * std::exp2((note - root) / 12.0);
}
} // namespace

class ZoneTests : public UnitTest
{
public:
    ZoneTests() : UnitTest("Multisample zones", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("selection by key and velocity (hard edges)");
        {
            const auto set = setOf({zone(0, 0, 59, 50), zone(0, 60, 127, 70, 0, 63), zone(0, 60, 127, 70, 64, 127)});
            expect(picked(set, 10, 100) == std::vector<int>({0}));
            expect(picked(set, 59, 0) == std::vector<int>({0}));
            expect(picked(set, 60, 63) == std::vector<int>({1}));
            expect(picked(set, 60, 64) == std::vector<int>({2}));
            expect(picked(set, 127, 127) == std::vector<int>({2}));
            expect(picked(setOf({zone(0, 20, 30, 25)}), 31, 64).empty(), "no match, no zone");
            const auto layered = setOf({zone(0, 0, 127, 60), zone(0, 0, 127, 60), zone(0, 40, 80, 60)});
            expect(picked(layered, 50, 64) == std::vector<int>({0, 1, 2}),
                   "round robin off: every match layers, lowest index first");
            std::vector<Zone> many(8, zone(0, 0, 127, 60));
            expectEquals(static_cast<int>(picked(setOf(many), 60, 64).size()), kMaxZoneLayers, "at most four layers");
        }

        beginTest("edge crossfades are equal power");
        {
            const auto set = setOf({zone(0, 0, 59, 50), zone(0, 60, 127, 70)});
            ZoneSelect o;
            o.xfadeKey = 6.0f;
            double worst = 0.0;
            int both = 0;
            for (int key = 0; key < 128; ++key)
            {
                std::array<ZoneLayer, kMaxZoneLayers> out;
                const int n = selectZones(set, key, 64, o, out);
                double sumSq = 0.0;
                for (int i = 0; i < n; ++i)
                    sumSq += static_cast<double>(out[static_cast<size_t>(i)].weight) * out[static_cast<size_t>(i)].weight;
                worst = std::max(worst, std::abs(sumSq - 1.0));
                both += n == 2 ? 1 : 0;
            }
            expect(worst < 1.0e-5, "sum of squares " + String(worst));
            expectEquals(both, 6, "six keys inside the fade");
            // Velocity edges too.
            const auto vset = setOf({zone(0, 0, 127, 60, 0, 63), zone(0, 0, 127, 60, 64, 127)});
            ZoneSelect ov;
            ov.xfadeVel = 0.25f;
            worst = 0.0;
            for (int vel = 0; vel < 128; ++vel)
            {
                std::array<ZoneLayer, kMaxZoneLayers> out;
                const int n = selectZones(vset, 60, vel, ov, out);
                double sumSq = 0.0;
                for (int i = 0; i < n; ++i)
                    sumSq += static_cast<double>(out[static_cast<size_t>(i)].weight) * out[static_cast<size_t>(i)].weight;
                worst = std::max(worst, std::abs(sumSq - 1.0));
            }
            expect(worst < 1.0e-5, "velocity sum of squares " + String(worst));
            // A zone that spans the whole range is never attenuated by the fade setting.
            const auto whole = setOf({zone(0, 0, 127, 60)});
            std::array<ZoneLayer, kMaxZoneLayers> out;
            expectEquals(selectZones(whole, 0, 0, o, out), 1);
            expect(out[0].weight == 1.0f);
            expect(zoneEdgeWeight(40.0f, 30, 50, 0.0f) == 1.0f && zoneEdgeWeight(51.0f, 30, 50, 0.0f) == 0.0f);
        }

        beginTest("round robin: cycle and random are deterministic, alternatives are exclusive, groups layer");
        {
            auto set = setOf({zone(0, 0, 127, 60), zone(0, 0, 127, 60), zone(0, 0, 127, 60)});
            ZoneSelect o;
            o.roundRobin = ZoneRoundRobin::Cycle;
            std::vector<int> seq;
            for (int i = 0; i < 7; ++i)
                seq.push_back(picked(set, 60, 64, o)[0]);
            expect(seq == std::vector<int>({0, 1, 2, 0, 1, 2, 0}), "cycle");
            auto again = setOf({zone(0, 0, 127, 60), zone(0, 0, 127, 60), zone(0, 0, 127, 60)});
            seq.clear();
            for (int i = 0; i < 4; ++i)
                seq.push_back(picked(again, 60, 64, o)[0]);
            expect(seq == std::vector<int>({0, 1, 2, 0}), "a new set starts from the first");
            ZoneSelect r;
            r.roundRobin = ZoneRoundRobin::Random;
            std::array<int, 3> hits{};
            for (std::uint32_t seed = 1; seed < 400; ++seed)
            {
                r.seed = seed;
                const auto a = picked(set, 60, 64, r);
                expectEquals(static_cast<int>(a.size()), 1);
                ++hits[static_cast<size_t>(a[0])];
                expect(picked(set, 60, 64, r) == a, "same seed, same zone");
            }
            expect(hits[0] > 60 && hits[1] > 60 && hits[2] > 60, "all alternatives used");
            // Two groups layer; inside a group the zones alternate.
            auto grouped = setOf({zone(0, 0, 127, 60), zone(0, 0, 127, 60), zone(0, 0, 127, 60), zone(0, 0, 127, 60)});
            grouped.zones[2].group = 1;
            grouped.zones[3].group = 1;
            const auto g = picked(grouped, 60, 64, o);
            expectEquals(static_cast<int>(g.size()), 2);
            expect(g[0] < 2 && g[1] >= 2, "one per group");
            // Crossfade neighbours of one group are not alternatives of each other.
            auto neighbours = setOf({zone(0, 0, 59, 50), zone(0, 60, 127, 70)});
            ZoneSelect x = o;
            x.xfadeKey = 4.0f;
            expectEquals(static_cast<int>(picked(neighbours, 59, 64, x).size()), 2);
        }

        beginTest("a note plays the zone of its key, at the zone's root");
        {
            for (const int note : {40, 59, 62, 71, 80})
            {
                const auto r = play(
                    [](SynthEngine& e)
                    {
                        threeSamples(e);
                        e.zoneBank().setZones(0, threeZones());
                    },
                    zonePatch(), note, 1.0f, 24000);
                const int id = note < 60 ? 0 : (note < 72 ? 1 : 2);
                const int root = id == 0 ? 57 : (id == 1 ? 69 : 81);
                const double hz = id == 0 ? 220.0 : (id == 1 ? 440.0 : 880.0);
                const double expected = freqOf(note, root, hz);
                expect(amplitudeAt(r.left, expected, 4000, 20000) > 0.3,
                       "note " + std::to_string(note) + " amplitude at " + String(expected));
                for (int other = 0; other < 3; ++other)
                    if (other != id)
                    {
                        const double ohz = other == 0 ? 220.0 : (other == 1 ? 440.0 : 880.0);
                        const int oroot = other == 0 ? 57 : (other == 1 ? 69 : 81);
                        expect(amplitudeAt(r.left, freqOf(note, oroot, ohz), 4000, 20000) < 0.03 ||
                                   std::abs(freqOf(note, oroot, ohz) - expected) < 20.0,
                               "note " + std::to_string(note) + " leaks zone " + std::to_string(other));
                    }
            }
            const auto none = play(
                [](SynthEngine& e)
                {
                    threeSamples(e);
                    e.zoneBank().setZones(0, {zone(1, 60, 71, 69.0f)});
                },
                zonePatch(), 30, 1.0f, 12000);
            expect(energy(none.left) == 0.0, "no zone, silent");
        }

        beginTest("velocity layers pick different samples");
        {
            const auto setup = [](SynthEngine& e)
            {
                threeSamples(e);
                e.zoneBank().setZones(0, {zone(1, 0, 127, 69.0f, 0, 63), zone(2, 0, 127, 69.0f, 64, 127)});
            };
            const auto soft = play(setup, zonePatch(), 69, 0.3f, 24000);
            const auto hard = play(setup, zonePatch(), 69, 0.9f, 24000);
            expect(amplitudeAt(soft.left, 440.0, 4000, 20000) > 0.05 && amplitudeAt(soft.left, 880.0, 4000, 20000) < 0.02,
                   "the soft layer is the 440 Hz sample");
            expect(amplitudeAt(hard.left, 880.0, 4000, 20000) > 0.3 && amplitudeAt(hard.left, 440.0, 4000, 20000) < 0.02,
                   "the hard layer is the 880 Hz sample");
        }

        beginTest("crossfade at a zone edge plays both samples; layers add up");
        {
            const auto setup = [](SynthEngine& e)
            {
                threeSamples(e);
                e.zoneBank().setZones(0, {zone(1, 0, 59, 69.0f), zone(2, 60, 127, 57.0f)});
            };
            const auto edge = play(setup, zonePatch({{"osc1_smp_zone_xfade_key", 4.0f}}), 59, 1.0f, 24000);
            const double low = amplitudeAt(edge.left, freqOf(59, 69, 440.0), 4000, 20000);
            const double high = amplitudeAt(edge.left, freqOf(59, 57, 880.0), 4000, 20000);
            expect(low > 0.1 && high > 0.05 && low > high,
                   "both samples sound, the nearer one louder " + String(low) + " " + String(high));
            const auto hard = play(setup, zonePatch(), 59, 1.0f, 24000);
            expect(amplitudeAt(hard.left, freqOf(59, 57, 880.0), 4000, 20000) < 0.02, "hard edge: only one");

            // Two layers equal the sum of two single-zone renders.
            const auto both = play(
                [](SynthEngine& e)
                {
                    threeSamples(e);
                    e.zoneBank().setZones(0, {zone(1, 0, 127, 69.0f), zone(2, 0, 127, 81.0f)});
                },
                zonePatch(), 64, 1.0f, 12000);
            const auto one = play(
                [](SynthEngine& e)
                {
                    threeSamples(e);
                    e.zoneBank().setZones(0, {zone(1, 0, 127, 69.0f)});
                },
                zonePatch(), 64, 1.0f, 12000);
            const auto two = play(
                [](SynthEngine& e)
                {
                    threeSamples(e);
                    e.zoneBank().setZones(0, {zone(2, 0, 127, 81.0f)});
                },
                zonePatch(), 64, 1.0f, 12000);
            double worst = 0.0;
            for (size_t i = 0; i < both.left.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(both.left[i] - one.left[i] - two.left[i])));
            expect(worst < 1.0e-4, "layers are the sum " + String(worst));
        }

        beginTest("zone gain, tuning and loop points");
        {
            const auto base = [](SynthEngine& e)
            {
                threeSamples(e);
                e.zoneBank().setZones(0, {zone(1, 0, 127, 69.0f)});
            };
            const auto plain = play(base, zonePatch(), 69, 1.0f, 12000);
            auto g = zone(1, 0, 127, 69.0f);
            g.gainDb = -6.0206f;
            const auto quiet = play(
                [&](SynthEngine& e)
                {
                    threeSamples(e);
                    e.zoneBank().setZones(0, {g});
                },
                zonePatch(), 69, 1.0f, 12000);
            const double ratio = std::sqrt(energy(quiet.left, 3000) / energy(plain.left, 3000));
            expect(std::abs(ratio - 0.5) < 0.01, "-6 dB halves the amplitude " + String(ratio));
            auto t = zone(1, 0, 127, 69.0f);
            t.tuneCents = 100.0f;
            const auto tuned = play(
                [&](SynthEngine& e)
                {
                    threeSamples(e);
                    e.zoneBank().setZones(0, {t});
                },
                zonePatch(), 69, 1.0f, 24000);
            expect(amplitudeAt(tuned.left, 440.0 * std::exp2(1.0 / 12.0), 4000, 20000) > 0.3, "+100 cents");
            // A 0.5 s tone one-shots; a zone loop keeps it going although the slot loop is off.
            const auto shortSample = [](SynthEngine& e)
            {
                std::string err;
                e.sampleBank().load(0, tone(440.0, 0.25), "s", err);
            };
            const auto oneShot = play(
                [&](SynthEngine& e)
                {
                    shortSample(e);
                    e.zoneBank().setZones(0, {zone(0, 0, 127, 69.0f)});
                },
                zonePatch(), 69, 1.0f, 24000);
            expect(energy(oneShot.left, 16000) < 1.0e-6, "no loop: the sample ends");
            auto looped = zone(0, 0, 127, 69.0f);
            looped.loopMode = 1;
            const auto sustained = play(
                [&](SynthEngine& e)
                {
                    shortSample(e);
                    e.zoneBank().setZones(0, {looped});
                },
                zonePatch(), 69, 1.0f, 24000);
            expect(energy(sustained.left, 16000) > 1.0, "zone loop sustains");
            auto slotLoop = zone(0, 0, 127, 69.0f);
            const auto bySlot = play(
                [&](SynthEngine& e)
                {
                    shortSample(e);
                    e.zoneBank().setZones(0, {slotLoop});
                },
                zonePatch({{"osc1_smp_loop", 1.0f}}), 69, 1.0f, 24000);
            expect(energy(bySlot.left, 16000) > 1.0, "loopMode -1 follows the slot parameter");
        }

        beginTest(
            "slots without zones are untouched: zone mode on or zones present but off give the old render bit for bit");
        {
            auto plainValues = graphPatch();
            setValue(plainValues, "osc1_type", 1.0f);
            setValue(plainValues, "osc1_level", 1.0f);
            setValue(plainValues, "osc1_smp_loop", 1.0f);
            setValue(plainValues, "osc1_smp_uni_voices", 3.0f);
            const auto own = [](SynthEngine& e)
            {
                std::string err;
                e.sampleBank().load(0, tone(330.0, 0.5, true), "own", err);
            };
            const auto reference = play(own, plainValues, 64, 0.8f, 24000);
            auto zoneOnNoSet = plainValues;
            setValue(zoneOnNoSet, "osc1_smp_zone_mode", 1.0f);
            expect(play(own, zoneOnNoSet, 64, 0.8f, 24000).left == reference.left, "zone mode with no ZoneSet");
            const auto withZones = [&](SynthEngine& e)
            {
                own(e);
                std::string err;
                e.zoneBank().addSample(0, tone(440.0), "a4", err);
                e.zoneBank().addSample(0, tone(880.0), "a5", err);
                e.zoneBank().setZones(0, threeZones());
            };
            const auto off = play(withZones, plainValues, 64, 0.8f, 24000);
            expect(off.left == reference.left && off.right == reference.right, "zones present, mode off");
            const auto granular = [&](const std::vector<float>& v)
            {
                auto g = v;
                setValue(g, "osc1_smp_mode", 1.0f);
                return play(own, g, 64, 0.8f, 24000).left;
            };
            expect(granular(plainValues) == granular(zoneOnNoSet), "granular too");
        }

        beginTest("granular mode reads the first active zone; unison and filters apply");
        {
            const auto r = play(
                [](SynthEngine& e)
                {
                    threeSamples(e);
                    e.zoneBank().setZones(0, {zone(1, 0, 127, 69.0f)});
                },
                zonePatch({{"osc1_smp_mode", 1.0f}, {"osc1_grain_density", 60.0f}, {"osc1_grain_spray", 0.0f}}), 69, 1.0f,
                24000);
            expect(r.finite && amplitudeAt(r.left, 440.0, 4000, 20000) > 0.0,
                   "grains of the zone sample " + String(amplitudeAt(r.left, 440.0, 4000, 20000)));
            const auto uni = play(
                [](SynthEngine& e)
                {
                    threeSamples(e);
                    e.zoneBank().setZones(0, {zone(1, 0, 127, 69.0f)});
                },
                zonePatch({{"osc1_smp_uni_voices", 4.0f}, {"osc1_smp_uni_detune", 40.0f}}), 69, 1.0f, 24000);
            expect(uni.finite && amplitudeAt(uni.left, 440.0 * std::exp2(40.0 / 1200.0), 4000, 20000) > 0.05,
                   "unison detuned lines");
        }

        beginTest("determinism: the same render for every block size and run, round robin included");
        {
            const auto setup = [](SynthEngine& e)
            {
                threeSamples(e);
                e.zoneBank().setZones(0, {zone(1, 0, 127, 69.0f), zone(2, 0, 127, 81.0f), zone(0, 0, 127, 57.0f)});
            };
            const auto values =
                zonePatch({{"osc1_smp_zone_rr", 2.0f}, {"osc1_smp_uni_voices", 3.0f}, {"osc1_smp_zone_xfade_key", 3.0f}});
            const auto a = play(setup, values, 66, 0.7f, 20000, 512);
            expect(a.left == play(setup, values, 66, 0.7f, 20000, 64).left, "block 64");
            expect(a.left == play(setup, values, 66, 0.7f, 20000, 37).left, "block 37");
            expect(a.left == play(setup, values, 66, 0.7f, 20000, 512).left, "second run");
            auto cycle = values;
            setValue(cycle, "osc1_smp_zone_rr", 1.0f);
            expect(play(setup, cycle, 66, 0.7f, 20000, 512).left == play(setup, cycle, 66, 0.7f, 20000, 100).left, "cycle");
        }

        beginTest("note off and retrigger behave as for plain samples; the sample view reports zones and the active zone");
        {
            SynthEngine engine;
            threeSamples(engine);
            engine.zoneBank().setZones(0, threeZones());
            auto values = zonePatch({{"osc1_smp_loop", 1.0f}});
            prepare(engine, values);
            engine.noteOn(62, 1.0f);
            renderBlocks(engine, 4800, 256);
            SynthEngine::SampleView view;
            expect(engine.sampleView(0, view));
            expectEquals(static_cast<int>(view.zones.size()), 3);
            expect(view.visual.zones == std::vector<int>({1}), "the voice plays zone 1");
            engine.noteOn(62, 1.0f); // retrigger
            const auto r = renderBlocks(engine, 4800, 256, true);
            expect(r.finite && r.peak < 2.0f);
            engine.noteOff(62);
            const auto tail = renderBlocks(engine, 24000, 256, true);
            expect(energy(tail.left, 20000) < 1.0e-6, "released");
        }

        beginTest("caps: 64 zones, 63 extra samples, memory per slot");
        {
            ZoneBank bank;
            std::string err;
            std::vector<Zone> many(100, zone(0, 0, 127, 60.0f));
            expect(bank.setZones(0, many));
            expectEquals(static_cast<int>(bank.zones(0).size()), kMaxZones);
            expect(!bank.setZones(8, {}));
            int added = 0;
            for (int i = 0; i < 70; ++i)
                added += bank.addSample(1, tone(300.0, 0.05), "s", err) > 0 ? 1 : 0;
            expectEquals(added, kMaxSamplesPerSlot - 1);
            expectEquals(bank.sampleCount(1), kMaxSamplesPerSlot - 1);
            expect(bank.addSample(1, tone(300.0, 0.05), "s", err) == 0 && !err.empty());
            AudioData tiny;
            tiny.channels.assign(1, std::vector<float>(1, 0.0f));
            expect(bank.addSample(2, tiny, "tiny", err) == 0, "too short");
            // Fields are clamped.
            Zone odd = zone(99, 100, 20, 300.0f, 90, 10);
            odd.gainDb = 1000.0f;
            odd.loopStart = 0.9f;
            odd.loopEnd = 0.1f;
            odd.loopMode = 7;
            bank.addSample(3, tone(300.0, 0.05), "s", err);
            bank.setZones(3, {odd});
            const Zone c = bank.zones(3)[0];
            expect(c.sampleId == 1 && c.keyLo == 20 && c.keyHi == 100 && c.velLo == 10 && c.velHi == 90 &&
                       c.root == 127.0f && c.gainDb == 24.0f && c.loopStart == 0.1f && c.loopEnd == 0.9f && c.loopMode == 2,
                   "sanitised");
            bank.clear(3);
            expect(!bank.hasZones(3) && bank.sampleCount(3) == 0);
        }

        beginTest("file names and automatic key and velocity ranges");
        {
            expect(parseZoneFileName("Piano_C3_v80.wav").root == 60.0f &&
                   parseZoneFileName("Piano_C3_v80.wav").velocity == 80);
            expect(parseZoneFileName("/x/Piano_F#2.wav").root == 54.0f &&
                   parseZoneFileName("/x/Piano_F#2.wav").velocity == -1);
            expect(parseZoneFileName("Pad_Bb1_v127.WAV").root == 46.0f);
            expect(parseZoneFileName("Kick_36.wav").root == 36.0f);
            expect(parseZoneFileName("plain.wav").root < 0.0f);
            std::vector<Zone> zs = {zone(1, 0, 127, 48), zone(2, 0, 127, 60), zone(3, 0, 127, 72), zone(4, 0, 127, 60),
                                    zone(5, 0, 127, 48)};
            assignAutomaticRanges(zs, {40, 40, 40, 100, 100});
            expect(zs[0].keyLo == 0 && zs[0].keyHi == 54 && zs[1].keyLo == 55 && zs[1].keyHi == 66 && zs[2].keyLo == 67 &&
                       zs[2].keyHi == 127,
                   "keys split at midpoints");
            expect(zs[0].velLo == 0 && zs[0].velHi == 70 && zs[3].velLo == 71 && zs[3].velHi == 127,
                   "velocity layers split at the midpoint");
            expect(zs[3].keyLo == 55 && zs[3].keyHi == 127 && zs[4].keyLo == 0 && zs[4].keyHi == 54);
        }

        beginTest("importers: a folder of WAVs and SFZ-lite");
        {
            namespace fs = std::filesystem;
            const fs::path dir = fs::temp_directory_path() / "crucible_zone_import_test";
            fs::remove_all(dir);
            fs::create_directories(dir);
            writeWavFile((dir / "Keys_C3_v40.wav").string(), tone(261.6, 0.2));
            writeWavFile((dir / "Keys_C4_v40.wav").string(), tone(523.2, 0.2));
            writeWavFile((dir / "Keys_C3_v110.wav").string(), tone(261.6, 0.2));
            std::ofstream(dir / "notes.txt") << "not audio";
            ZoneBank bank;
            std::string err;
            expectEquals(bank.importFolder(0, dir.string(), err), 3);
            expectEquals(bank.sampleCount(0), 3);
            const auto zs = bank.zones(0);
            // Sorted file names: C3_v110, C3_v40, C4_v40.
            expect(zs[0].velLo == 76 && zs[0].velHi == 127 && zs[0].keyLo == 0 && zs[0].keyHi == 127 && zs[0].root == 60.0f,
                   "loud layer, one root");
            expect(zs[1].velHi == 75 && zs[1].keyHi == 66 && zs[2].keyLo == 67 && zs[2].root == 72.0f,
                   "soft layer split between C3 and C4");
            expectEquals(bank.importFolder(1, (dir / "missing").string(), err), 0);
            expect(!err.empty());

            std::ofstream sfz(dir / "map.sfz");
            sfz << "// comment\n<group> loop_mode=loop_continuous\n<region> sample=Keys_C3_v40.wav lokey=c3 hikey=b3 "
                   "pitch_keycenter=c3\n"
                   "<region> sample=Keys_C4_v40.wav key=72 lovel=64 hivel=127 volume=-3 tune=12 loop_start=100 "
                   "loop_end=1000\n"
                   "<region> sample=gone.wav key=10\n";
            sfz.close();
            ZoneBank sb;
            expectEquals(sb.importSfz(0, (dir / "map.sfz").string(), err), 2);
            const auto sz = sb.zones(0);
            expect(sz[0].keyLo == 60 && sz[0].keyHi == 71 && sz[0].root == 60.0f && sz[0].loopMode == 1,
                   "region 1 (group loop mode inherited)");
            expect(sz[1].keyLo == 72 && sz[1].keyHi == 72 && sz[1].root == 72.0f && sz[1].velLo == 64 &&
                       sz[1].gainDb == -3.0f && sz[1].tuneCents == 12.0f && sz[1].loopMode == 1 && sz[1].loopStart > 0.0f &&
                       sz[1].loopEnd > sz[1].loopStart,
                   "region 2");
            expectEquals(sb.sampleCount(0), 2);
            fs::remove_all(dir);
        }

        beginTest("patch round trip with three samples; old patches load and render as before");
        {
            Engine a;
            a.prepare(kRate, 256);
            std::string err;
            expect(a.loadSample(0, tone(220.0, 0.3), "own", err), err);
            expect(a.addZoneSample(0, tone(440.0, 0.3, true), "two", err) == 1);
            expect(a.addZoneSample(0, tone(880.0, 0.3), "three", err) == 2);
            Zone z1 = zone(1, 60, 71, 69.0f, 0, 100);
            z1.gainDb = -2.5f;
            z1.tuneCents = 7.0f;
            z1.loopMode = 2;
            z1.loopStart = 0.2f;
            z1.loopEnd = 0.7f;
            z1.group = 3;
            expect(a.setZones(0, {zone(0, 0, 59, 57.0f), z1, zone(2, 72, 127, 81.0f)}));
            a.setParam("osc1_type", 1.0f);
            a.setParam("osc1_level", 1.0f);
            a.setParam("osc1_smp_zone_mode", 1.0f);
            a.setParam("osc1_smp_zone_xfade_key", 2.0f);
            const auto renderOf = [](Engine& e, int note)
            {
                std::vector<float> out, l(256), r(256);
                MidiInput on{0, 0x90, static_cast<std::uint8_t>(note), 100};
                e.process(l.data(), r.data(), 256, &on, 1);
                out.insert(out.end(), l.begin(), l.end());
                for (int i = 0; i < 40; ++i)
                {
                    e.process(l.data(), r.data(), 256);
                    out.insert(out.end(), l.begin(), l.end());
                }
                return out;
            };
            const auto patch = capturePatch(a);
            int samples = 0, zonesAssets = 0, extra = 0;
            for (const auto& as : patch.assets)
            {
                samples += as.kind == "sample" ? 1 : 0;
                extra += as.kind == "sample" && as.index > 0 ? 1 : 0;
                zonesAssets += as.kind == "zones" ? 1 : 0;
            }
            expectEquals(samples, 3);
            expectEquals(extra, 2);
            expectEquals(zonesAssets, 1);
            Patch parsed;
            expect(patchFromJson(patchToJson(patch), parsed, err), err);
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, parsed);
            expect(b.zoneList(0) == a.zoneList(0), "zones restored exactly");
            expectEquals(b.zoneSampleCount(0), 2);
            for (const int note : {40, 65, 90})
                expect(renderOf(a, note) == renderOf(b, note), "bit-identical render of note " + std::to_string(note));
            // Applying a patch without zones clears them.
            applyPatch(b, Patch{});
            expect(!b.zones().hasZones(0) && b.zoneSampleCount(0) == 0);

            // A patch of an engine without zones carries none of the new fields, so its text is what the old engine wrote.
            Engine plain;
            plain.prepare(kRate, 256);
            expect(plain.loadSample(0, tone(330.0, 0.3), "own", err), err);
            const std::string text = patchToJson(capturePatch(plain));
            expect(text.find("\"index\"") == std::string::npos && text.find("zones") == std::string::npos,
                   "no zone fields for plain slots");
            // An old file lacks the zone parameters entirely: it loads with zones off and renders like the engine did.
            Patch old;
            expect(patchFromJson(text, old, err), err);
            old.params.erase(std::remove_if(old.params.begin(), old.params.end(),
                                            [](const auto& kv) { return kv.first.find("smp_zone_") != std::string::npos; }),
                             old.params.end());
            old.moduleVersions["osc.sample"] = 1;
            Engine c;
            c.prepare(kRate, 256);
            applyPatch(c, old);
            expect(c.getParam("osc1_smp_zone_mode") == 0.0f);
            plain.setParam("osc1_type", 1.0f);
            plain.setParam("osc1_level", 1.0f);
            c.setParam("osc1_type", 1.0f);
            c.setParam("osc1_level", 1.0f);
            expect(renderOf(plain, 64) == renderOf(c, 64), "old patch, same samples");

            // Content kept in an asset store keeps its index.
            const std::string store = (std::filesystem::temp_directory_path() / "crucible_zone_assets").string();
            std::filesystem::remove_all(store);
            Patch ext = patch;
            expect(externaliseAssets(ext, AssetStore(store)));
            Patch ext2;
            expect(patchFromJson(patchToJson(ext), ext2, err), err);
            std::vector<std::string> missing;
            AssetStore as(store);
            resolveAssets(ext2, {&as}, missing);
            expect(missing.empty());
            Engine d;
            d.prepare(kRate, 256);
            applyPatch(d, ext2);
            expect(d.zoneList(0) == a.zoneList(0) && d.zoneSampleCount(0) == 2, "asset store round trip");
            std::filesystem::remove_all(store);
        }

        beginTest("C API: add samples, set and get zones, play, import");
        {
            CrucibleEngine* e = crucible_create(nullptr);
            crucible_prepare(e, 48000.0, 256);
            std::vector<float> s(4800);
            for (size_t i = 0; i < s.size(); ++i)
                s[i] = 0.5f * static_cast<float>(std::sin(kTau * 440.0 * static_cast<double>(i) / 48000.0));
            expect(crucible_add_zone_sample(e, 0, s.data(), 4800, 1, 48000.0, "a") == 1);
            expect(crucible_add_zone_sample(e, 0, nullptr, 4800, 1, 48000.0, "a") == 0);
            expectEquals(crucible_zone_sample_count(e, 0), 1);
            CrucibleZone z{1, 0, 127, 0, 127, 69.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1, 0};
            expect(crucible_set_zones(e, 0, &z, 1) == 1);
            CrucibleZone back[2];
            expectEquals(crucible_get_zones(e, 0, back, 2), 1);
            expect(back[0].sample_id == 1 && back[0].root == 69.0f && back[0].loop_mode == 1);
            crucible_set_param_id(e, "osc1_type", 1.0f);
            crucible_set_param_id(e, "osc1_level", 1.0f);
            crucible_set_param_id(e, "osc1_smp_zone_mode", 1.0f);
            CrucibleMidiEvent note = {0, 0x90, 69, 100};
            std::vector<float> l(256), r(256);
            float loud = 0.0f;
            for (int b = 0; b < 30; ++b)
            {
                crucible_process(e, l.data(), r.data(), 256, b == 0 ? &note : nullptr, b == 0 ? 1 : 0);
                if (b > 10)
                    for (const float v : l)
                        loud = std::max(loud, std::abs(v));
            }
            expect(loud > 0.05f, "zone plays through the C API");
            crucible_clear_zones(e, 0);
            expectEquals(crucible_get_zones(e, 0, back, 2), 0);
            expectEquals(crucible_import_zone_folder(e, 0, "/nonexistent/folder"), 0);
            crucible_destroy(e);
        }
    }
};

static ZoneTests zoneTests;
