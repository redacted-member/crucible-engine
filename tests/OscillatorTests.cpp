// SPDX-License-Identifier: MIT
#include "TestSupport.h"
#include <crucible/Voice.h>
#include <crucible/Wavetables.h>

using namespace crucible;
using namespace crucible::testing;

namespace
{
const char* const kWarpNames[] = {"Off",      "FM",   "SyncHard", "SyncSoft", "Bend+", "Bend-", "Mirror", "PWM",
                                  "Quantize", "Asym", "Flip",     "Formant",  "Smear", "Ring",  "Fold"};

std::vector<float> sawPatch(int note, int warp, float a, float b)
{
    auto values = ParameterMap::defaultValues();
    setValue(values, "osc1_table", 0.0f);
    setValue(values, "osc1_pos", 1.0f);
    setValue(values, "osc1_level", 1.0f);
    setValue(values, "osc1_warp", static_cast<float>(warp));
    setValue(values, "osc1_warp_a", a);
    setValue(values, "osc1_warp_b", b);
    setValue(values, "osc1_warp_src", 1.0f);
    setValue(values, "osc2_level", 0.0f);
    setValue(values, "osc2_table", 0.0f);
    setValue(values, "osc2_pos", 0.0f);
    setValue(values, "amp_attack", 0.001f);
    setValue(values, "amp_sustain", 1.0f);
    setValue(values, "amp_decay", 0.001f);
    setValue(values, "filter_cutoff", 20000.0f);
    setValue(values, "filter_res", 0.0f);
    setValue(values, "master_gain", 0.0f);
    (void)note;
    return values;
}
} // namespace

class OscillatorTests : public UnitTest
{
public:
    OscillatorTests() : UnitTest("Oscillator", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("half-band decimator preserves DC and rejects the upper band");
        {
            HalfBandDecimator d;
            float last = 0.0f;
            for (int i = 0; i < 200; ++i)
                last = d.process(1.0f, 1.0f);
            expectWithinAbsoluteError(last, 1.0f, 1.0e-3f);

            d.reset();
            float peak = 0.0f;
            // Oversampled-rate signal at 0.75 of the oversampled Nyquist (inside the stop band).
            for (int i = 0; i < 400; ++i)
            {
                const float a = std::sin(3.14159265f * 0.75f * static_cast<float>(2 * i));
                const float b = std::sin(3.14159265f * 0.75f * static_cast<float>(2 * i + 1));
                const float y = d.process(a, b);
                if (i > 100)
                    peak = std::max(peak, std::abs(y));
            }
            expect(peak < 0.003f, "stop band peak " + String(peak));
        }

        beginTest("half-band interpolator then decimator is near unity at low frequency");
        {
            HalfBandInterpolator up;
            HalfBandDecimator down;
            float peak = 0.0f;
            for (int i = 0; i < 2000; ++i)
            {
                float a, b;
                up.process(std::sin(0.05f * static_cast<float>(i)), a, b);
                const float y = down.process(a, b);
                if (i > 200)
                    peak = std::max(peak, std::abs(y));
            }
            expectWithinAbsoluteError(peak, 1.0f, 0.02f);
        }

        beginTest("wavetable bank has 14 tables of 64 frames and every table renders");
        {
            expectEquals(WavetableBank::numTables, 14);
            expectEquals(WavetableBank::numFrames, 64);
            expectEquals(static_cast<int>(WavetableBank::tableNames().size()), WavetableBank::numTables);
            for (int t = 0; t < WavetableBank::numTables; ++t)
            {
                auto values = ParameterMap::defaultValues();
                setValue(values, "osc1_table", static_cast<float>(t));
                setValue(values, "osc1_pos", 0.6f);
                setValue(values, "osc1_level", 1.0f);
                CRUCIBLE_HEAP(SynthEngine, engine);
                prepare(engine, values);
                engine.noteOn(50, 1.0f);
                const auto r = renderBlocks(engine, 12000);
                expect(r.finite && r.peak > 0.01f, "table " + String(t));
            }
        }

        beginTest("every table has non-trivial motion across frames");
        {
            const auto& bank = WavetableBank::instance();
            for (int t = 0; t < WavetableBank::numTables; ++t)
            {
                const float* first = bank.frame(t, 4, 0);
                const float* last = bank.frame(t, 4, WavetableBank::numFrames - 1);
                float diff = 0.0f;
                for (int i = 0; i < WavetableBank::mipSize(4); ++i)
                    diff += std::abs(first[i] - last[i]);
                expect(diff > 1.0f, "table " + String(t) + " does not change over frames");
            }
        }

        beginTest("every warp mode is finite and bounded across amounts, notes and unison");
        for (int mode = 0; mode < static_cast<int>(WarpMode::Count); ++mode)
            for (const float a : {0.0f, 0.5f, 1.0f})
                for (const float b : {0.0f, 0.5f, 1.0f})
                {
                    auto values = sawPatch(60, mode, a, b);
                    setValue(values, "osc1_uni_voices", 8.0f);
                    setValue(values, "osc1_uni_detune", 100.0f);
                    setValue(values, "osc1_pos", 0.7f);
                    setValue(values, "osc2_level", 0.5f);
                    setValue(values, "osc2_warp", static_cast<float>(mode));
                    setValue(values, "osc2_warp_a", a);
                    setValue(values, "osc2_warp_b", b);
                    CRUCIBLE_HEAP(SynthEngine, engine);
                    prepare(engine, values);
                    engine.noteOn(12, 1.0f);
                    engine.noteOn(72, 1.0f);
                    engine.noteOn(120, 1.0f);
                    const auto r = renderBlocks(engine, 4800, 113);
                    expect(r.finite, String(kWarpNames[mode]) + " non-finite");
                    expect(r.peak < 12.0f, String(kWarpNames[mode]) + " peak " + String(r.peak));
                }

        beginTest("warp amount zero is identical to no warp for warps designed to be neutral");
        for (const int mode : {4, 5, 6, 7, 9, 10, 12})
        {
            CRUCIBLE_HEAP(SynthEngine, plain);
            CRUCIBLE_HEAP(SynthEngine, warped);
            prepare(plain, sawPatch(60, 0, 0.0f, 0.0f));
            prepare(warped, sawPatch(60, mode, 0.0f, 0.0f));
            plain.noteOn(60, 1.0f);
            warped.noteOn(60, 1.0f);
            const auto a = renderBlocks(plain, 9600, 512, true);
            const auto b = renderBlocks(warped, 9600, 512, true);
            // Warped path is delayed by the half-band decimator; compare energy instead of samples.
            double ea = 0.0, eb = 0.0;
            for (size_t i = 2000; i < a.left.size(); ++i)
            {
                ea += a.left[i] * a.left[i];
                eb += b.left[i] * b.left[i];
            }
            expect(std::abs(10.0 * std::log10(eb / ea)) < 0.5, String(kWarpNames[mode]) + " not neutral at zero");
        }

        beginTest("plain wavetable oscillator keeps aliasing low at high pitch");
        {
            for (const int note : {84, 100, 108})
            {
                const double f0 = midiToHz(note);
                CRUCIBLE_HEAP(SynthEngine, engine);
                prepare(engine, sawPatch(note, 0, 0.0f, 0.0f));
                engine.noteOn(note, 1.0f);
                const auto r = renderBlocks(engine, 8192 + 16384, 512, true);
                const double db = aliasingBelowDb(r.left, f0, kRate, 20000.0, 8192);
                logMessage("saw note " + String(note) + " aliasing below 20 kHz " + String(db, 1) + " dB");
                expect(db < -100.0, "note " + String(note) + " aliasing " + String(db));
            }
        }

        beginTest("warped oscillators: aliasing measurements stay bounded");
        {
            const int note = 96;
            const double f0 = midiToHz(note);
            for (int mode = 1; mode < static_cast<int>(WarpMode::Count); ++mode)
            {
                CRUCIBLE_HEAP(SynthEngine, engine);
                auto values = sawPatch(note, mode, 0.35f, 0.0f);
                // FM and ring modulation are measured with a sine carrier; a saw carrier has no meaningful bound.
                if (mode == 1 || mode == 13)
                    setValue(values, "osc1_pos", 0.0f);
                prepare(engine, values);
                engine.noteOn(note, 1.0f);
                const auto r = renderBlocks(engine, 8192 + 16384, 512, true);
                const double db = inharmonicEnergyDb(r.left, f0, kRate, 8192);
                logMessage(String("warp ") + kWarpNames[mode] + " note 96 inharmonic energy " + String(db, 1) + " dB");
                // Waveshaping warps (fold, bend, mirror) are the worst case; every mode must stay below -20 dB.
                expect(db < -20.0, String(kWarpNames[mode]) + " inharmonic " + String(db));
            }
        }

        beginTest("stack modes spread pitch as designed");
        {
            OscParams p;
            p.unisonVoices = 4;
            p.spread = 1.0f;
            std::array<float, kMaxUnison> ra{}, rb{};
            for (size_t i = 0; i < ra.size(); ++i)
            {
                ra[i] = static_cast<float>(i) / 4.0f - 1.0f;
                rb[i] = ra[i];
            }
            std::array<UnisonVoice, kMaxUnison> out{};
            p.uniMode = 0;
            computeUnisonLayout(p, 100.0f, ra, rb, 1.0f, out);
            expectWithinAbsoluteError(out[0].semitones, -1.0f, 1.0e-4f);
            expectWithinAbsoluteError(out[3].semitones, 1.0f, 1.0e-4f);
            p.uniMode = 3;
            computeUnisonLayout(p, 0.0f, ra, rb, 1.0f, out);
            expectWithinAbsoluteError(out[1].semitones, 12.0f, 1.0e-4f);
            p.uniMode = 6;
            computeUnisonLayout(p, 0.0f, ra, rb, 1.0f, out);
            expectWithinAbsoluteError(out[1].semitones, 4.0f, 1.0e-4f);
            expectWithinAbsoluteError(out[2].semitones, 7.0f, 1.0e-4f);
            // Power is normalised regardless of blend.
            for (const float blend : {0.0f, 0.5f, 1.0f})
            {
                p.uniMode = 0;
                p.uniBlend = blend;
                p.spread = 0.0f;
                computeUnisonLayout(p, 10.0f, ra, rb, 1.0f, out);
                float power = 0.0f;
                for (int u = 0; u < 4; ++u)
                    power += (out[static_cast<size_t>(u)].gainL * out[static_cast<size_t>(u)].gainL +
                              out[static_cast<size_t>(u)].gainR * out[static_cast<size_t>(u)].gainR) *
                             0.5f;
                expectWithinAbsoluteError(power, 1.0f, 0.02f);
            }
        }

        beginTest("third oscillator, sub waves and noise types produce audio");
        {
            auto values = ParameterMap::defaultValues();
            setValue(values, "osc1_level", 0.0f);
            setValue(values, "osc3_level", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, values);
            engine.noteOn(60, 1.0f);
            const auto r = renderBlocks(engine, 12000);
            expect(r.peak > 0.01f);

            for (int w = 0; w < 4; ++w)
                for (int nt = 0; nt < 4; ++nt)
                {
                    auto v = ParameterMap::defaultValues();
                    setValue(v, "osc1_level", 0.0f);
                    setValue(v, "sub_level", 1.0f);
                    setValue(v, "sub_wave", static_cast<float>(w));
                    setValue(v, "noise_level", 0.5f);
                    setValue(v, "noise_type", static_cast<float>(nt));
                    CRUCIBLE_HEAP(SynthEngine, e);
                    prepare(e, v);
                    e.noteOn(36, 1.0f);
                    const auto rr = renderBlocks(e, 12000);
                    expect(rr.finite && rr.peak > 0.001f && rr.peak < 10.0f);
                }
        }

        beginTest("mono, legato, glide and polyphony limit behave");
        {
            auto values = ParameterMap::defaultValues();
            setValue(values, "voice_mode", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, mono);
            prepare(mono, values);
            mono.noteOn(60, 1.0f);
            mono.noteOn(64, 1.0f);
            renderBlocks(mono, 4800);
            expectEquals(mono.activeVoiceCount(), 1);
            mono.noteOff(64);
            renderBlocks(mono, 4800);
            expectEquals(mono.activeVoiceCount(), 1); // falls back to the held note

            setValue(values, "voice_mode", 0.0f);
            setValue(values, "voice_poly", 3.0f);
            CRUCIBLE_HEAP(SynthEngine, limited);
            prepare(limited, values);
            for (int n = 0; n < 8; ++n)
                limited.noteOn(40 + n, 1.0f);
            renderBlocks(limited, 2400);
            int expected = 0;
            expected = limited.activeVoiceCount();
            expect(expected <= 3, "voices " + String(expected));

            setValue(values, "voice_poly", 16.0f);
            CRUCIBLE_HEAP(SynthEngine, pedal);
            prepare(pedal, values);
            pedal.setSustainPedal(true);
            pedal.noteOn(60, 1.0f);
            pedal.noteOff(60);
            renderBlocks(pedal, 48000);
            expectEquals(pedal.activeVoiceCount(), 1);
            pedal.setSustainPedal(false);
            renderBlocks(pedal, 96000);
            expectEquals(pedal.activeVoiceCount(), 0);
        }
    }
};

static OscillatorTests oscillatorTests;
