// SPDX-License-Identifier: MIT
// Sample oscillator: unison (detuned, panned voices) and slicing (markers, one slice per note or per key range),
// the slice helpers of SampleBank, patch and C API plumbing.

#include "GraphSupport.h"

#include <crucible/Engine.h>
#include <crucible/Patch.h>
#include <crucible/Sample.h>
#include <crucible/crucible.h>

#include <atomic>
#include <thread>

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr double kTau = 6.283185307179586;

// Four tones of a quarter of a second each (220, 330, 440, 660 Hz).
AudioData fourTones(double rate = 48000.0)
{
    AudioData a;
    a.sampleRate = rate;
    a.channels.resize(1);
    const double hz[4] = {220.0, 330.0, 440.0, 660.0};
    const int quarter = static_cast<int>(rate * 0.25);
    for (int t = 0; t < 4; ++t)
        for (int i = 0; i < quarter; ++i)
            a.channels[0].push_back(0.5f * static_cast<float>(std::sin(kTau * hz[t] * i / rate)));
    return a;
}

AudioData sine(double hz, double seconds, bool stereo = false)
{
    AudioData a;
    a.sampleRate = 48000.0;
    a.channels.resize(stereo ? 2 : 1);
    for (auto& ch : a.channels)
        for (int i = 0; i < static_cast<int>(seconds * 48000.0); ++i)
            ch.push_back(0.5f * static_cast<float>(std::sin(kTau * hz * i / 48000.0)));
    return a;
}

// Decaying noise bursts at the given positions (seconds) in a one second mono sample.
AudioData bursts(const std::vector<double>& at)
{
    AudioData a;
    a.sampleRate = 48000.0;
    a.channels.assign(1, std::vector<float>(48000, 0.0f));
    Rng rng;
    for (const double t : at)
        for (int i = 0; i < 4800; ++i)
        {
            const size_t k = static_cast<size_t>(t * 48000.0) + static_cast<size_t>(i);
            if (k < a.channels[0].size())
                a.channels[0][k] += 0.8f * rng.bipolar() * std::exp(-static_cast<float>(i) / 600.0f);
        }
    return a;
}

// Amplitude of the component at hz over [from, to) (single-bin DFT).
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

std::vector<float> patch(const std::vector<std::pair<std::string, float>>& extra = {})
{
    auto v = graphPatch();
    setValue(v, "osc1_type", 1.0f);
    setValue(v, "osc1_level", 1.0f);
    setValue(v, "osc1_smp_root", 69.0f);
    setValue(v, "amp_release", 0.05f);
    for (const auto& [id, value] : extra)
        setValue(v, id, value);
    return v;
}

Rendered play(const AudioData& audio, const std::vector<float>& values, int note, int samples,
              const std::function<void(SampleBank&)>& prepareBank = {})
{
    SynthEngine engine;
    std::string err;
    engine.sampleBank().load(0, audio, "test", err);
    if (prepareBank)
        prepareBank(engine.sampleBank());
    prepare(engine, values);
    engine.noteOn(note, 1.0f);
    return renderBlocks(engine, samples, 512, true);
}

double correlation(const std::vector<float>& a, const std::vector<float>& b, size_t from)
{
    double ab = 0.0, aa = 0.0, bb = 0.0;
    for (size_t i = from; i < a.size(); ++i)
    {
        ab += static_cast<double>(a[i]) * b[i];
        aa += static_cast<double>(a[i]) * a[i];
        bb += static_cast<double>(b[i]) * b[i];
    }
    return ab / std::sqrt(std::max(aa * bb, 1.0e-20));
}
} // namespace

class SampleSliceTests : public UnitTest
{
public:
    SampleSliceTests() : UnitTest("Sample unison and slices", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        const auto tone = sine(440.0, 1.0);

        beginTest("unison: one voice is the old sound, several voices add detuned lines at equal power");
        {
            const auto single = play(tone, patch(), 69, 48000);
            const auto explicitOne =
                play(tone, patch({{"osc1_smp_uni_voices", 1.0f}, {"osc1_smp_uni_detune", 80.0f}}), 69, 48000);
            expect(single.left == explicitOne.left, "detune is ignored with one voice");

            const auto three = play(
                tone, patch({{"osc1_smp_uni_voices", 3.0f}, {"osc1_smp_uni_detune", 50.0f}, {"osc1_smp_uni_spread", 0.0f}}),
                69, 48000);
            const double centre = amplitudeAt(three.left, 440.0, 12000, 40000);
            const double up = amplitudeAt(three.left, 440.0 * std::exp2(50.0 / 1200.0), 12000, 40000);
            const double down = amplitudeAt(three.left, 440.0 * std::exp2(-50.0 / 1200.0), 12000, 40000);
            expect(centre > 0.05 && up > 0.05 && down > 0.05,
                   "three lines " + String(centre) + " " + String(up) + " " + String(down));
            const double oneLine = amplitudeAt(single.left, 440.0 * std::exp2(50.0 / 1200.0), 12000, 40000);
            expect(oneLine < 0.03, "the single voice has no side line " + String(oneLine));
            const double ratio = std::sqrt(energy(three.left, 12000) / energy(single.left, 12000));
            expect(ratio > 0.6 && ratio < 1.6, "level of three voices against one " + String(ratio));
        }

        beginTest("unison: spread pans the voices, a stereo sample keeps working, eight voices are finite");
        {
            const auto narrow = play(
                tone, patch({{"osc1_smp_uni_voices", 4.0f}, {"osc1_smp_uni_spread", 0.0f}, {"osc1_smp_uni_detune", 20.0f}}),
                69, 30000);
            const auto wide = play(
                tone, patch({{"osc1_smp_uni_voices", 4.0f}, {"osc1_smp_uni_spread", 1.0f}, {"osc1_smp_uni_detune", 20.0f}}),
                69, 30000);
            expect(correlation(narrow.left, narrow.right, 6000) > 0.99, "no spread keeps the channels equal");
            expect(correlation(wide.left, wide.right, 6000) < 0.8,
                   "spread decorrelates " + String(correlation(wide.left, wide.right, 6000)));
            const auto stereo =
                play(sine(330.0, 0.5, true), patch({{"osc1_smp_uni_voices", 8.0f}, {"osc1_smp_root", 64.0f}}), 64, 12000);
            expect(stereo.finite && stereo.peak > 0.1f && stereo.peak < 2.0f, "stereo sample, eight voices");
            expect(play(tone, patch({{"osc1_smp_uni_voices", 5.0f}, {"osc1_smp_loop", 2.0f}}), 69, 24000).left ==
                       play(tone, patch({{"osc1_smp_uni_voices", 5.0f}, {"osc1_smp_loop", 2.0f}}), 69, 24000).left,
                   "deterministic");
        }

        beginTest("unison: the voices end together at the end of a one-shot, granular unison thickens the cloud");
        {
            const auto r =
                play(sine(440.0, 0.1), patch({{"osc1_smp_uni_voices", 4.0f}, {"osc1_smp_uni_start", 1.0f}}), 69, 24000);
            expect(energy(r.left, 8000) < 1.0e-6, "silent after the sample ended");
            const auto grains = patch({{"osc1_smp_mode", 1.0f},
                                       {"osc1_grain_density", 60.0f},
                                       {"osc1_grain_spray", 0.0f},
                                       {"osc1_grain_spread", 0.0f}});
            auto stacked = grains;
            setValue(stacked, "osc1_smp_uni_voices", 4.0f);
            setValue(stacked, "osc1_smp_uni_detune", 60.0f);
            const auto plain = play(tone, grains, 69, 36000);
            const auto thick = play(tone, stacked, 69, 36000);
            expect(thick.finite && plain.finite);
            expect(amplitudeAt(thick.left, 440.0 * std::exp2(60.0 / 1200.0), 12000, 36000) >
                       3.0 * amplitudeAt(plain.left, 440.0 * std::exp2(60.0 / 1200.0), 12000, 36000),
                   "detuned grains appear");
        }

        beginTest("slice markers: sanitised, published, removed when the sample is replaced");
        {
            SampleBank bank;
            std::string err;
            expect(!bank.setSlices(0, {0.5f}), "no sample, no markers");
            expect(bank.load(0, fourTones(), "tones", err));
            expect(bank.setSlices(0, {0.75f, 0.25f, 0.25f, -1.0f, 2.0f, 0.5f, std::nanf("")}));
            const auto m = bank.slices(0);
            expectEquals(static_cast<int>(m.size()), 3);
            expect(m[0] == 0.25f && m[1] == 0.5f && m[2] == 0.75f, "sorted and unique");
            expectEquals(bank.sliceEqually(0, 8), 8);
            expectEquals(static_cast<int>(bank.slices(0).size()), 7);
            expectEquals(bank.sliceEqually(0, 1000), kMaxSlices);
            expectEquals(static_cast<int>(bank.slices(0).size()), kMaxSlices - 1);
            expectEquals(bank.sliceEqually(3, 4), 0);
            bank.clearSlices(0);
            expect(bank.slices(0).empty());
            bank.sliceEqually(0, 4);
            expect(bank.load(0, fourTones(), "again", err));
            expect(bank.slices(0).empty(), "loading a sample drops the markers");
            bank.sliceEqually(0, 4);
            bank.clear(0);
            expect(bank.slices(0).empty(), "clearing too");
        }

        beginTest("transient slicing finds the bursts");
        {
            SampleBank bank;
            std::string err;
            expect(bank.load(0, bursts({0.0, 0.25, 0.5, 0.75}), "drums", err));
            expectEquals(bank.sliceByTransients(0), 4);
            const auto m = bank.slices(0);
            expect(m.size() == 3);
            for (size_t i = 0; i < m.size(); ++i)
                expect(std::abs(m[i] - 0.25f * static_cast<float>(i + 1)) < 0.006f,
                       "marker " + std::to_string(i) + " at " + std::to_string(m[i]));
            expectEquals(bank.sliceByTransients(0, 2), 2);
            expectEquals(bank.sliceByTransients(5), 0);
            AudioData silence;
            silence.sampleRate = 48000.0;
            silence.channels.assign(1, std::vector<float>(48000, 0.0f));
            expect(bank.load(1, silence, "silence", err));
            expectEquals(bank.sliceByTransients(1), 1);
        }

        const auto sliced = [](SampleBank& b) { b.sliceEqually(0, 4); };
        const double hz[4] = {220.0, 330.0, 440.0, 660.0};

        beginTest("slices per note: note base + n plays slice n at its original pitch, other notes are silent");
        {
            for (int s = 0; s < 4; ++s)
            {
                const auto r = play(fourTones(), patch({{"osc1_smp_slice_mode", 1.0f}, {"osc1_smp_slice_base", 36.0f}}),
                                    36 + s, 12000, sliced);
                for (int t = 0; t < 4; ++t)
                {
                    const double a = amplitudeAt(r.left, hz[t], 1000, 10000);
                    if (t == s)
                        expect(a > 0.3, "slice " + std::to_string(s) + " tone " + std::to_string(a));
                    else
                        expect(a < 0.02,
                               "slice " + std::to_string(s) + " leaks tone " + std::to_string(t) + ": " + std::to_string(a));
                }
            }
            for (const int note : {35, 40, 69})
                expect(energy(play(fourTones(), patch({{"osc1_smp_slice_mode", 1.0f}, {"osc1_smp_slice_base", 36.0f}}), note,
                                   12000, sliced)
                                  .left) == 0.0,
                       "note " + std::to_string(note) + " has no slice");
            // One shot: the slice ends after a quarter of a second.
            const auto r =
                play(fourTones(), patch({{"osc1_smp_slice_mode", 1.0f}, {"osc1_smp_slice_base", 36.0f}}), 36, 24000, sliced);
            expect(energy(r.left, 13500) < 1.0e-6, "silent after the slice");
        }
        beginTest(
            "slices per key range: the keys are shared out evenly, the slice sounds at its original pitch in the middle");
        {
            const auto range = [&](int note, float key = 1.0f)
            {
                return play(fourTones(),
                            patch({{"osc1_smp_slice_mode", 2.0f},
                                   {"osc1_smp_slice_base", 48.0f},
                                   {"osc1_smp_slice_top", 59.0f},
                                   {"osc1_smp_key", key}}),
                            note, 12000, sliced);
            };
            // 12 keys, 4 slices: 48-50, 51-53, 54-56, 57-59. Slice 0 is centred on 49, slice 1 on 52.
            expect(amplitudeAt(range(49).left, 220.0, 1000, 10000) > 0.3, "key 49 plays slice 0 unchanged");
            expect(amplitudeAt(range(52).left, 330.0, 1000, 10000) > 0.3, "key 52 plays slice 1 unchanged");
            expect(amplitudeAt(range(50).left, 220.0 * std::exp2(1.0 / 12.0), 1000, 9000) > 0.3,
                   "key 50 is one semitone up");
            expect(amplitudeAt(range(48).left, 220.0 * std::exp2(-1.0 / 12.0), 1000, 11000) > 0.3,
                   "key 48 is one semitone down");
            expect(amplitudeAt(range(58).left, 660.0, 1000, 10000) > 0.3, "key 58 plays slice 3");
            expect(amplitudeAt(range(51, 0.0f).left, 330.0, 1000, 10000) > 0.3, "keytrack off: original pitch on every key");
            expect(amplitudeAt(range(53, 0.0f).left, 330.0, 1000, 10000) > 0.3);
            expect(energy(range(47).left) == 0.0 && energy(range(60).left) == 0.0, "outside the range is silent");
        }

        beginTest("slices without markers are one slice; granular mode reads inside the slice; reverse and loops stay in "
                  "the slice");
        {
            const auto whole =
                play(fourTones(), patch({{"osc1_smp_slice_mode", 1.0f}, {"osc1_smp_slice_base", 36.0f}}), 36, 24000);
            expect(amplitudeAt(whole.left, 220.0, 1000, 10000) > 0.3, "no markers: slice 0 is the sample");
            expect(amplitudeAt(whole.left, 660.0, 36000 - 12000, 24000) < 0.02 || whole.left.size() >= 24000);
            expect(
                energy(play(fourTones(), patch({{"osc1_smp_slice_mode", 1.0f}, {"osc1_smp_slice_base", 36.0f}}), 37, 12000)
                           .left) == 0.0,
                "only one slice");

            const auto granular = play(fourTones(),
                                       patch({{"osc1_smp_mode", 1.0f},
                                              {"osc1_smp_slice_mode", 1.0f},
                                              {"osc1_smp_slice_base", 36.0f},
                                              {"osc1_grain_density", 80.0f},
                                              {"osc1_grain_size", 30.0f},
                                              {"osc1_grain_spray", 0.3f}}),
                                       37, 30000, sliced);
            expect(amplitudeAt(granular.left, 330.0, 4000, 30000) > 0.01 &&
                       amplitudeAt(granular.left, 220.0, 4000, 30000) <
                           0.1 * amplitudeAt(granular.left, 330.0, 4000, 30000) &&
                       amplitudeAt(granular.left, 440.0, 4000, 30000) < 0.1 * amplitudeAt(granular.left, 330.0, 4000, 30000),
                   "grains only from slice 1: " + String(amplitudeAt(granular.left, 330.0, 4000, 30000)) + " " +
                       String(amplitudeAt(granular.left, 220.0, 4000, 30000)) + " " +
                       String(amplitudeAt(granular.left, 440.0, 4000, 30000)));

            const auto looped = play(
                fourTones(), patch({{"osc1_smp_slice_mode", 1.0f}, {"osc1_smp_slice_base", 36.0f}, {"osc1_smp_loop", 1.0f}}),
                38, 36000, sliced);
            expect(amplitudeAt(looped.left, 440.0, 1000, 36000) > 0.3 && amplitudeAt(looped.left, 660.0, 1000, 36000) < 0.03,
                   "loop stays in the slice");
            const auto reversed =
                play(fourTones(),
                     patch({{"osc1_smp_slice_mode", 1.0f}, {"osc1_smp_slice_base", 36.0f}, {"osc1_smp_reverse", 1.0f}}), 39,
                     12000, sliced);
            expect(amplitudeAt(reversed.left, 660.0, 1000, 10000) > 0.3, "reverse plays the slice backwards");
        }

        beginTest("markers travel with a patch and through the C API");
        {
            Engine a;
            a.prepare(kRate, 512);
            std::string err;
            expect(a.loadSample(1, fourTones(), "tones", err), err);
            expect(a.setSampleSlices(1, {0.2f, 0.4f, 0.9f}));
            const Patch p = capturePatch(a);
            Engine b;
            b.prepare(kRate, 512);
            applyPatch(b, p);
            const auto got = b.sampleSlices(1);
            expectEquals(static_cast<int>(got.size()), 3);
            expect(got[0] == 0.2f && got[2] == 0.9f);
            // Serialised and read back.
            Patch q;
            std::string perr;
            expect(patchFromJson(patchToJson(p), q, perr), perr);
            Engine c;
            c.prepare(kRate, 512);
            applyPatch(c, q);
            expectEquals(static_cast<int>(c.sampleSlices(1).size()), 3);
            applyPatch(c, Patch{});
            expect(c.sampleSlices(1).empty(), "a patch without a sample clears the markers");

            CrucibleEngine* ce = crucible_create(nullptr);
            crucible_prepare(ce, kRate, 512);
            const auto audio = fourTones();
            expectEquals(crucible_load_sample(ce, 0, audio.channels[0].data(), static_cast<int>(audio.channels[0].size()), 1,
                                              48000.0, "tones"),
                         1);
            const float markers[2] = {0.6f, 0.3f};
            expectEquals(crucible_set_sample_slices(ce, 0, markers, 2), 1);
            float out[4] = {};
            expectEquals(crucible_get_sample_slices(ce, 0, out, 4), 2);
            expect(out[0] == 0.3f && out[1] == 0.6f);
            expectEquals(crucible_slice_sample_equally(ce, 0, 5), 5);
            expectEquals(crucible_get_sample_slices(ce, 0, nullptr, 0), 4);
            expectEquals(crucible_slice_sample_equally(ce, 3, 5), 0);
            expectEquals(crucible_set_sample_slices(ce, 3, markers, 2), 0);
            crucible_destroy(ce);
        }

        beginTest("editing markers while notes sound is safe and finite");
        {
            Engine e;
            e.prepare(kRate, 256);
            std::string err;
            e.loadSample(0, fourTones(), "tones", err);
            e.setParam("osc1_type", 1.0f);
            e.setParam("osc1_smp_slice_mode", 2.0f);
            e.setParam("osc1_smp_slice_base", 36.0f);
            e.setParam("osc1_smp_slice_top", 84.0f);
            e.setParam("osc1_smp_uni_voices", 3.0f);
            std::atomic<bool> stop{false};
            std::thread editor(
                [&]
                {
                    for (int i = 0; !stop.load(); ++i)
                    {
                        e.sliceSampleEqually(0, 1 + i % 20);
                        if (i % 7 == 0)
                            e.sliceSampleByTransients(0);
                        e.samples().collectGarbage();
                    }
                });
            bool finite = true;
            std::vector<float> l(256), r(256);
            for (int block = 0; block < 400; ++block)
            {
                if (block % 10 == 0)
                    e.noteOn(36 + (block / 10) % 48, 0.8f);
                e.process(l.data(), r.data(), 256);
                for (int i = 0; i < 256; ++i)
                    finite = finite && std::isfinite(l[static_cast<size_t>(i)]) && std::isfinite(r[static_cast<size_t>(i)]);
            }
            stop = true;
            editor.join();
            expect(finite);
        }

        beginTest("absurd finite sample values are clamped at import");
        for (const float huge : {1.0e9f, std::numeric_limits<float>::max()})
        {
            AudioData audio = sine(220.0, 0.25, true);
            audio.channels[0][1105] = huge;
            audio.channels[1][2000] = -huge;
            for (int type : {1, 8})
            {
                Engine e;
                e.prepare(kRate, 256);
                std::string err;
                expect(e.loadSample(0, audio, "huge", err), err);
                e.setParam("osc1_type", static_cast<float>(type));
                std::vector<float> l(256), r(256);
                float peak = 0.0f;
                for (int block = 0; block < 100; ++block)
                {
                    if (block % 5 == 0)
                        e.noteOn(48 + block % 24, 0.8f);
                    e.process(l.data(), r.data(), 256);
                    for (int i = 0; i < 256; ++i)
                        peak = std::max({peak, std::fabs(l[static_cast<size_t>(i)]), std::fabs(r[static_cast<size_t>(i)])});
                }
                expect(peak < 64.0f, "peak stays bounded");
            }
        }
    }
};

static SampleSliceTests sampleSliceTests;
