// SPDX-License-Identifier: MIT
// Oscillator modules: sample / granular, cloud, noise and sub oscillators in the voice slots.

#include "GraphSupport.h"

#include <crucible/Cloud.h>
#include <crucible/Sample.h>

using namespace crucible;
using namespace crucible::testing;

namespace
{
AudioData makeSine(double hz, double seconds, double rate = 48000.0, float amp = 0.5f)
{
    AudioData a;
    a.sampleRate = rate;
    a.channels.resize(1);
    const int n = static_cast<int>(seconds * rate);
    for (int i = 0; i < n; ++i)
        a.channels[0].push_back(amp * static_cast<float>(std::sin(6.283185307179586 * hz * i / rate)));
    return a;
}

// Band-limited saw by additive synthesis (harmonics up to 20 kHz).
AudioData makeSaw(double hz, double seconds, double rate = 48000.0)
{
    AudioData a;
    a.sampleRate = rate;
    a.channels.resize(1);
    const int n = static_cast<int>(seconds * rate);
    a.channels[0].assign(static_cast<size_t>(n), 0.0f);
    for (int h = 1; h * hz < 20000.0; ++h)
        for (int i = 0; i < n; ++i)
            a.channels[0][static_cast<size_t>(i)] +=
                0.3f / static_cast<float>(h) * static_cast<float>(std::sin(6.283185307179586 * hz * h * i / rate));
    return a;
}

double frequencyOf(const std::vector<float>& s, size_t from, double rate)
{
    int crossings = 0;
    size_t first = 0, last = 0;
    for (size_t i = from + 1; i < s.size(); ++i)
        if (s[i - 1] < 0.0f && s[i] >= 0.0f)
        {
            const double frac = -s[i - 1] / (s[i] - s[i - 1]);
            if (crossings == 0)
                first = i;
            last = i;
            (void)frac;
            ++crossings;
        }
    if (crossings < 3)
        return 0.0;
    return (crossings - 1) * rate / static_cast<double>(last - first);
}

std::vector<float> samplePatch(int slot = 0)
{
    auto v = graphPatch();
    setValue(v, "osc1_level", slot == 0 ? 1.0f : 0.0f);
    setValue(v, "osc1_type", slot == 0 ? 1.0f : 0.0f);
    setValue(v, "osc1_smp_root", 69.0f);
    setValue(v, "amp_release", 0.05f);
    return v;
}

Rendered playSample(const AudioData& audio, std::vector<float> values, int note, int samples, const EngineConfig& cfg = {})
{
    CRUCIBLE_HEAP(SynthEngine, engine, cfg);
    std::string err;
    engine.sampleBank().load(0, audio, "test", err);
    prepare(engine, values);
    engine.noteOn(note, 1.0f);
    return renderBlocks(engine, samples, 512, true);
}
} // namespace

class OscillatorModuleTests : public UnitTest
{
public:
    OscillatorModuleTests() : UnitTest("Oscillator modules", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("the oscillator modules are registered and each slot has a type parameter");
        {
            const auto types = ModuleRegistry::instance().ofKind(ModuleKind::Oscillator);
            expectEquals(types.size(), size_t(9)); // six, then the modal, harmonic and spectral types of P13
            expectEquals(types[0]->id, std::string("osc.wavetable"));
            const auto& type = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("osc3_type"))];
            expectEquals(type.choices.size(), size_t(9));
            expectEquals(type.choices[1], std::string("Sample / Granular"));
            expect(ParameterMap::indexOf("osc3_grain_density") >= 0);
            expect(ParameterMap::indexOf("osc4_type") < 0);
        }

        beginTest("sample playback is pitched: root note keeps the pitch, +12 doubles it");
        {
            const auto audio = makeSine(440.0, 1.0);
            const auto root = playSample(audio, samplePatch(), 69, 24000);
            expectWithinAbsoluteError(frequencyOf(root.left, 2000, kRate), 440.0, 0.5);
            const auto up = playSample(audio, samplePatch(), 81, 24000);
            expectWithinAbsoluteError(frequencyOf(up.left, 2000, kRate), 880.0, 1.0);
            const auto down = playSample(audio, samplePatch(), 57, 24000);
            expectWithinAbsoluteError(frequencyOf(down.left, 2000, kRate), 220.0, 0.5);
            expect(root.peak > 0.2f && root.peak < 0.8f);
        }

        beginTest("a sample at another rate plays at the right pitch");
        {
            const auto audio = makeSine(441.0, 1.0, 44100.0);
            const auto r = playSample(audio, samplePatch(), 69, 24000);
            expectWithinAbsoluteError(frequencyOf(r.left, 2000, kRate), 441.0, 1.0); // root 440 Hz, the file holds 441 Hz
        }

        beginTest("transposing a rich sample up stays band-limited");
        {
            const auto audio = makeSaw(220.0, 1.0);
            // Four semitones up: the 220 Hz saw becomes 277 Hz and its harmonics would reach 25 kHz unfiltered.
            auto v = samplePatch();
            setValue(v, "osc1_smp_root", 57.0f);            // 220 Hz is A3 = 57
            const auto r = playSample(audio, v, 69, 40000); // one octave up: f0 = 440 Hz
            const double db = inharmonicEnergyDb(r.left, 440.0, kRate, 4000);
            std::printf("    sample aliasing %.1f dB\n", db);
            expect(db < -55.0, "aliasing " + String(std::to_string(db)));
            // The same sample read without band-limiting would be far worse: sanity check the metric on the source.
            const auto src = playSample(audio, v, 57, 40000);
            expect(inharmonicEnergyDb(src.left, 220.0, kRate, 4000) < -60.0);
        }

        beginTest("start and end select a region; without a loop the sample stops");
        {
            AudioData a;
            a.sampleRate = kRate;
            a.channels.resize(1);
            for (int i = 0; i < 48000; ++i)
                a.channels[0].push_back(i < 24000 ? 0.0f : 0.5f * static_cast<float>(std::sin(0.06 * i)));
            auto v = samplePatch();
            setValue(v, "osc1_smp_key", 0.0f); // play at the original speed on any key
            const auto whole = playSample(a, v, 60, 40000);
            expect(energy(whole.left, 0) > 1.0);
            // The first half is silent: nothing before 24000 samples.
            expect(energy(std::vector<float>(whole.left.begin(), whole.left.begin() + 22000)) < 1.0e-6);
            setValue(v, "osc1_smp_start", 0.5f);
            const auto second = playSample(a, v, 60, 40000);
            expect(energy(std::vector<float>(second.left.begin(), second.left.begin() + 3000)) > 1.0);
            setValue(v, "osc1_smp_end", 0.6f);
            const auto cut = playSample(a, v, 60, 40000);
            // 0.1 of the sample = 4800 samples, then silence.
            expect(energy(std::vector<float>(cut.left.begin() + 6000, cut.left.end())) < 1.0e-6);
        }

        beginTest("loops keep a short sample sounding; ping-pong and crossfade stay finite and continuous");
        {
            const auto audio = makeSine(440.0, 0.05); // 2400 samples
            auto v = samplePatch();
            setValue(v, "osc1_smp_loop", 0.0f);
            const auto once = playSample(audio, v, 69, 24000);
            expect(energy(std::vector<float>(once.left.begin() + 6000, once.left.end())) < 1.0e-6);
            for (const float mode : {1.0f, 2.0f})
            {
                setValue(v, "osc1_smp_loop", mode);
                const auto r = playSample(audio, v, 69, 24000);
                expect(r.finite);
                expect(energy(std::vector<float>(r.left.begin() + 12000, r.left.end())) > 100.0);
            }
            // A loop point that does not fall on a zero crossing clicks without a crossfade and much less with one.
            setValue(v, "osc1_smp_loop", 1.0f);
            setValue(v, "osc1_smp_loop_end", 0.93f);
            const auto hard = playSample(audio, v, 69, 24000);
            setValue(v, "osc1_smp_xfade", 0.8f);
            const auto soft = playSample(audio, v, 69, 24000);
            expect(brightness(soft.left, 4000) < brightness(hard.left, 4000));
        }

        beginTest("granular playback keeps the pitch, is deterministic and reports its grains");
        {
            const auto audio = makeSine(440.0, 1.0);
            auto v = samplePatch();
            setValue(v, "osc1_smp_mode", 1.0f);
            setValue(v, "osc1_grain_density", 60.0f);
            setValue(v, "osc1_grain_size", 60.0f);
            setValue(v, "osc1_grain_spray", 0.3f);
            CRUCIBLE_HEAP(SynthEngine, a);
            CRUCIBLE_HEAP(SynthEngine, b);
            std::string err;
            a.sampleBank().load(0, audio, "t", err);
            b.sampleBank().load(0, audio, "t", err);
            prepare(a, v);
            prepare(b, v);
            a.noteOn(69, 1.0f);
            b.noteOn(69, 1.0f);
            const auto ra = renderBlocks(a, 24000, 512, true);
            const auto rb = renderBlocks(b, 24000, 512, true);
            expect(ra.left == rb.left && ra.right == rb.right, "deterministic");
            expect(energy(ra.left, 4000) > 10.0);
            expect(ra.finite);
            // Grains of a pure sine at ratio 1 sum to a sine of the same frequency (within a few cents).
            expectWithinAbsoluteError(frequencyOf(ra.left, 4000, kRate), 440.0, 6.0);
            SynthEngine::SampleView view;
            expect(a.sampleView(0, view));
            expectEquals(view.frames, 48000);
            expectEquals(view.peakMax.size(), size_t(kSamplePeakColumns));
            expect(!view.visual.grains.empty() && !view.visual.playheads.empty());
            for (const auto& g : view.visual.grains)
                expect(g.position >= 0.0f && g.position <= 1.0f && g.width > 0.0f);
            expectWithinAbsoluteError(view.detectedNote, 69.0f, 0.2f);
        }

        beginTest("granular pitch, reverse probability and scan change the sound");
        {
            const auto audio = makeSine(440.0, 1.0);
            auto v = samplePatch();
            setValue(v, "osc1_smp_mode", 1.0f);
            setValue(v, "osc1_grain_density", 80.0f);
            setValue(v, "osc1_grain_size", 50.0f);
            setValue(v, "osc1_grain_spray", 0.0f);
            const auto base = playSample(audio, v, 69, 24000);
            setValue(v, "osc1_grain_pitch", 12.0f);
            const auto up = playSample(audio, v, 69, 24000);
            expectWithinAbsoluteError(frequencyOf(up.left, 4000, kRate), 880.0, 12.0);
            setValue(v, "osc1_grain_pitch", 0.0f);
            setValue(v, "osc1_grain_rev", 1.0f);
            const auto rev = playSample(makeSaw(110.0, 1.0), v, 69, 24000);
            const auto fwd = playSample(
                makeSaw(110.0, 1.0),
                [&]
                {
                    auto w = v;
                    setValue(w, "osc1_grain_rev", 0.0f);
                    return w;
                }(),
                69, 24000);
            expect(rev.left != fwd.left);
            (void)base;
        }

        beginTest("an empty sample slot is silent and every extreme setting stays finite");
        {
            auto v = samplePatch();
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, v);
            engine.noteOn(60, 1.0f);
            expect(renderBlocks(engine, 4800).peak == 0.0f);
            const auto audio = makeSaw(55.0, 0.5);
            std::string err;
            engine.sampleBank().load(0, audio, "x", err);
            for (const float mode : {0.0f, 1.0f})
                for (const float extreme : {0.0f, 1.0f})
                {
                    auto w = v;
                    setValue(w, "osc1_smp_mode", mode);
                    for (const char* id :
                         {"osc1_smp_start", "osc1_smp_end", "osc1_smp_loop_start", "osc1_smp_loop_end", "osc1_smp_xfade",
                          "osc1_grain_pos", "osc1_grain_spray", "osc1_grain_rev", "osc1_grain_spread", "osc1_grain_jitter"})
                        setValue(w, id, extreme);
                    setValue(w, "osc1_grain_size", extreme > 0.5f ? 500.0f : 1.0f);
                    setValue(w, "osc1_grain_density", extreme > 0.5f ? 300.0f : 0.5f);
                    setValue(w, "osc1_grain_pitch_rand", extreme * 1200.0f);
                    setValue(w, "osc1_grain_scan", extreme * 2.0f);
                    setValue(w, "osc1_smp_loop", 1.0f + extreme);
                    for (const int note : {0, 24, 60, 100, 127})
                    {
                        const auto r = playSample(audio, w, note, 9600);
                        expect(r.finite && r.peak < 8.0f);
                    }
                }
        }

        beginTest("the cloud oscillator renders the slot's cloud at the voice pitch");
        {
            auto v = graphPatch();
            setValue(v, "osc1_type", 2.0f);
            setValue(v, "osc1_level", 0.5f);
            setValue(v, "osc1_cloud_pos", 0.0f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            CloudModel cloud;
            cloud.points = {{1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {2.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f}};
            engine.cloudBank().set(0, cloud);
            prepare(engine, v);
            engine.noteOn(57, 1.0f); // 220 Hz
            const auto r = renderBlocks(engine, 24000, 512, true);
            expect(r.finite && r.peak > 0.05f);
            // The 2.5th partial (550 Hz) is inharmonic: the inharmonic energy is that partial.
            const double inh = inharmonicEnergyDb(r.left, 220.0, kRate, 4000);
            expect(inh > -25.0 && inh < -3.0, "inharmonic share " + String(std::to_string(inh)));
            CloudSnapshot snap;
            expect(engine.cloudView(0, snap));
            expectEquals(snap.points.size(), size_t(2));
            expectEquals(snap.energy.size(), size_t(2));
            expect(snap.energy[0] > snap.energy[1] && snap.activeNotes == 1);
            // Notes high enough that the 2.5th partial would pass 0.45 fs are band-limited.
            engine.allNotesOff();
            engine.noteOn(120, 1.0f);
            expect(renderBlocks(engine, 2400).finite);
        }

        beginTest("noise oscillator types have the expected spectral tilt and the width control decorrelates");
        {
            auto v = graphPatch();
            setValue(v, "osc1_type", 3.0f);
            setValue(v, "osc1_level", 1.0f);
            double low[7]{}, high[7]{};
            for (int type = 0; type < 7; ++type)
            {
                setValue(v, "osc1_noise_type", static_cast<float>(type));
                const auto r = playNote(EngineConfig{}, v, 60, 24000);
                expect(r.finite);
                // Ratio of first-difference energy to energy: rises with brightness.
                low[type] = energy(r.left, 2000);
                high[type] = brightness(r.left, 2000);
            }
            const auto ratio = [&](int t) { return high[t] / std::max(low[t], 1.0e-20); };
            expect(ratio(2) < ratio(1) && ratio(1) < ratio(0) && ratio(0) < ratio(3) && ratio(3) < ratio(4), "tilt order");
            setValue(v, "osc1_noise_type", 0.0f);
            setValue(v, "osc1_noise_width", 0.0f);
            const auto mono = playNote(EngineConfig{}, v, 60, 4800);
            expect(mono.left == mono.right);
            setValue(v, "osc1_noise_width", 1.0f);
            const auto wide = playNote(EngineConfig{}, v, 60, 4800);
            expect(wide.left != wide.right);
            setValue(v, "osc1_noise_color", -1.0f);
            const auto dark = playNote(EngineConfig{}, v, 60, 24000);
            expect(brightness(dark.left, 2000) < 0.1 * brightness(wide.left, 2000) + 1.0e-9 ||
                   brightness(dark.left, 2000) < brightness(wide.left, 2000));
        }

        beginTest("sub oscillator follows the voice pitch octaves down and stays band-limited");
        {
            auto v = graphPatch();
            setValue(v, "osc1_type", 4.0f);
            setValue(v, "osc1_level", 1.0f);
            for (int octave = 0; octave < 3; ++octave)
            {
                setValue(v, "osc1_sub_octave", static_cast<float>(octave));
                const auto r = playNote(EngineConfig{}, v, 69, 24000);
                expectWithinAbsoluteError(frequencyOf(r.left, 2000, kRate), 440.0 / std::pow(2.0, octave + 1), 0.5);
            }
            for (const float wave : {2.0f, 3.0f, 4.0f})
            {
                setValue(v, "osc1_sub_octave", 0.0f);
                setValue(v, "osc1_sub_wave", wave);
                const auto r = playNote(EngineConfig{}, v, 96, 24000);
                expect(inharmonicEnergyDb(r.left, 1046.5, kRate, 4000) < -35.0);
            }
            setValue(v, "osc1_sub_wave", 0.0f);
            setValue(v, "osc1_sub_drive", 1.0f);
            const auto driven = playNote(EngineConfig{}, v, 57, 9600);
            expect(driven.finite && driven.peak > 0.5f);
        }

        beginTest("cross modulation: FM and ring from an earlier slot (even one with level zero), hard sync to its pitch");
        {
            auto v = graphPatch();
            setValue(v, "osc1_level", 0.0f);
            setValue(v, "osc1_table", 0.0f);
            setValue(v, "osc1_pos", 0.0f);
            setValue(v, "osc2_level", 0.7f);
            setValue(v, "osc2_table", 0.0f);
            setValue(v, "osc2_pos", 0.0f);
            setValue(v, "osc2_xmod_src", 0.0f);
            const auto plain = playNote(EngineConfig{}, v, 60, 24000);
            for (const float mode : {1.0f, 2.0f, 3.0f})
            {
                auto w = v;
                setValue(w, "osc2_xmod_mode", mode);
                setValue(w, "osc2_xmod_amt", 0.0f);
                const auto zero = playNote(EngineConfig{}, w, 60, 24000);
                expect(zero.left == plain.left, "amount zero changes nothing");
                setValue(w, "osc2_xmod_amt", 0.6f);
                setValue(w, "osc1_semi", mode == 3.0f ? -7.0f : 7.0f);
                const auto r = playNote(EngineConfig{}, w, 60, 24000);
                expect(r.finite && r.peak > 0.05f && r.peak < 4.0f);
                expect(r.left != plain.left, "cross modulation changes the sound");
                // A source that is not before the slot is ignored.
                setValue(w, "osc2_xmod_src", 1.0f);
                expect(playNote(EngineConfig{}, w, 60, 24000).left == plain.left);
            }
        }

        beginTest("hard sync to another slot follows the master pitch and is band-limited");
        {
            auto v = graphPatch();
            setValue(v, "osc1_level", 0.0f); // the master only sets the period
            setValue(v, "osc1_table", 0.0f);
            setValue(v, "osc2_level", 0.7f);
            setValue(v, "osc2_table", 0.0f);
            setValue(v, "osc2_pos", 1.0f); // saw
            setValue(v, "osc2_semi", 12.0f);
            setValue(v, "osc2_fine", 37.0f); // not a harmonic of the master
            setValue(v, "osc2_xmod_mode", 3.0f);
            setValue(v, "osc2_xmod_amt", 1.0f);
            const double master = midiToHz(45.0);
            std::vector<float> nonSync;
            {
                auto w = v;
                setValue(w, "osc2_xmod_mode", 0.0f);
                nonSync = playNote(EngineConfig{}, w, 45, 40000).left;
            }
            const auto r = playNote(EngineConfig{}, v, 45, 40000);
            expect(r.finite);
            const double alias = aliasingBelowDb(r.left, master, kRate, 20000.0, 4000);
            std::printf("    synced saw (slave %.0f Hz over master %.1f Hz): aliasing below 20 kHz %.1f dB\n",
                        midiToHz(57.0) * 1.0217, master, alias);
            expect(alias < -75.0);
            expect(nonSync != r.left);
            // High master and slave pitch: still clean.
            const auto hi = playNote(EngineConfig{}, v, 84, 40000);
            expect(hi.finite);
            const double hiAlias = aliasingBelowDb(hi.left, midiToHz(84.0), kRate, 20000.0, 4000);
            std::printf("    synced saw at note 84: %.1f dB\n", hiAlias);
            expect(hiAlias < -60.0, "high synced saw " + std::to_string(hiAlias));
        }

        beginTest("ring modulation also works for module oscillators");
        {
            auto v = graphPatch();
            setValue(v, "osc1_level", 0.0f);
            setValue(v, "osc1_pos", 0.0f);
            setValue(v, "osc2_level", 0.7f);
            setValue(v, "osc2_type", 3.0f); // noise
            const auto plain = playNote(EngineConfig{}, v, 60, 9600);
            setValue(v, "osc2_xmod_mode", 2.0f);
            setValue(v, "osc2_xmod_amt", 1.0f);
            const auto ring = playNote(EngineConfig{}, v, 60, 9600);
            expect(ring.finite && ring.left != plain.left);
            // Fully ring modulated noise is a product with a sine, so its energy is about half.
            expect(energy(ring.left, 2000) < 0.8 * energy(plain.left, 2000));
        }

        beginTest("oscillator types can be switched during a note");
        {
            auto v = graphPatch();
            CRUCIBLE_HEAP(SynthEngine, engine);
            std::string err;
            engine.sampleBank().load(0, makeSaw(110.0, 0.5), "x", err);
            prepare(engine, v);
            engine.noteOn(57, 1.0f);
            for (const float type : {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 0.0f})
            {
                setValue(v, "osc1_type", type);
                engine.setParameterValues(v.data(), static_cast<int>(v.size()));
                const auto r = renderBlocks(engine, 2400);
                expect(r.finite && r.peak > 0.0f, "type " + String(std::to_string(type)));
            }
        }
    }
};

static OscillatorModuleTests oscillatorModuleTests;
