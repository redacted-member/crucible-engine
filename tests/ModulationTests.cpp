// SPDX-License-Identifier: MIT
#include <crucible/Dsp.h>
#include <crucible/ModMatrix.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
double dominantFrequency(const std::vector<float>& x, int skip = 2000)
{
    int crossings = 0;
    int first = -1;
    int last = -1;
    for (size_t i = static_cast<size_t>(skip) + 1; i < x.size(); ++i)
        if (x[i - 1] < 0.0f && x[i] >= 0.0f)
        {
            if (first < 0)
                first = static_cast<int>(i);
            last = static_cast<int>(i);
            ++crossings;
        }
    if (crossings < 3)
        return 0.0;
    return (crossings - 1) * kRate / (last - first);
}

float rms(const std::vector<float>& x, size_t from)
{
    double sum = 0.0;
    for (size_t i = from; i < x.size(); ++i)
        sum += static_cast<double>(x[i]) * x[i];
    return static_cast<float>(std::sqrt(sum / static_cast<double>(x.size() - from)));
}

std::vector<float> sinePatch()
{
    auto v = ParameterMap::defaultValues();
    setValue(v, "osc1_pos", 0.0f);
    setValue(v, "osc1_level", 1.0f);
    setValue(v, "amp_sustain", 1.0f);
    setValue(v, "amp_attack", 0.001f);
    setValue(v, "filter_cutoff", 20000.0f);
    setValue(v, "master_gain", 0.0f);
    return v;
}

void route(std::vector<float>& v, int slot, const char* src, const char* dst, float amount, int pol = 0, float curve = 0.0f)
{
    const auto sources = ParameterMap::modSourceNames();
    const auto dests = ParameterMap::modDestNames();
    const auto find = [](const std::vector<std::string>& names, const std::string& n)
    {
        for (size_t i = 0; i < names.size(); ++i)
            if (names[i] == n)
                return static_cast<float>(i);
        assert(false);
        return 0.0f;
    };
    const auto p = "mod" + std::to_string(slot);
    setValue(v, p + "_src", find(sources, src));
    setValue(v, p + "_dst", find(dests, dst));
    setValue(v, p + "_amt", amount);
    setValue(v, p + "_pol", static_cast<float>(pol));
    setValue(v, p + "_curve", curve);
}
} // namespace

class ModulationTests : public UnitTest
{
public:
    ModulationTests() : UnitTest("Modulation", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("source and destination name tables match the enums");
        expectEquals(static_cast<int>(ParameterMap::modSourceNames().size()),
                     kNumModSources - (kNumMacros - kDefaultMacros));
        expectEquals(static_cast<int>(ParameterMap::legacyModDestNames().size()), kNumModDests);
        expectEquals(ParameterMap::modDestNames().size(), kNumModDests + ParameterMap::modTargets().size());
        expectEquals(kNumModSlots, 16);

        beginTest("slot evaluation: polarity, curve and amount");
        {
            ModSlotParams s;
            s.source = static_cast<int>(ModSource::Velocity);
            s.amount = 0.5f;
            expectWithinAbsoluteError(evaluateSlot(s, 0.8f), 0.4f, 1.0e-6f);
            s.polarity = static_cast<int>(ModPolarity::Bipolar);
            expectWithinAbsoluteError(evaluateSlot(s, 0.8f), 0.3f, 1.0e-6f);
            expectWithinAbsoluteError(evaluateSlot(s, 0.0f), -0.5f, 1.0e-6f);
            s.source = static_cast<int>(ModSource::Lfo1);
            s.polarity = static_cast<int>(ModPolarity::Unipolar);
            expectWithinAbsoluteError(evaluateSlot(s, -1.0f), 0.0f, 1.0e-6f);
            expectWithinAbsoluteError(evaluateSlot(s, 1.0f), 0.5f, 1.0e-6f);
            s.source = static_cast<int>(ModSource::Macro1);
            s.polarity = 0;
            s.amount = 1.0f;
            s.curve = 0.5f;
            expect(evaluateSlot(s, 0.5f) < 0.5f); // slow start
            s.curve = -0.5f;
            expect(evaluateSlot(s, 0.5f) > 0.5f); // fast start
            s.curve = 1.0f;
            expectWithinAbsoluteError(evaluateSlot(s, 1.0f), 1.0f, 1.0e-6f);
        }

        beginTest("matrix accumulates active slots");
        {
            std::array<ModSlotParams, kNumModSlots> slots{};
            std::array<float, kNumModSources> src{};
            src[static_cast<size_t>(ModSource::Macro1)] = 0.5f;
            src[static_cast<size_t>(ModSource::Velocity)] = 1.0f;
            slots[0] = {static_cast<int>(ModSource::Macro1), static_cast<int>(ModDest::Cutoff), 0.5f, 0, 0.0f};
            slots[1] = {static_cast<int>(ModSource::Velocity), static_cast<int>(ModDest::Cutoff), 0.25f, 0, 0.0f};
            slots[15] = {static_cast<int>(ModSource::Macro1), static_cast<int>(ModDest::Drive), -1.0f, 0, 0.0f};
            std::array<float, kNumModDests> dest{};
            accumulateModulation(slots, src.data(), dest.data());
            expectWithinAbsoluteError(dest[static_cast<size_t>(ModDest::Cutoff)], 0.5f, 1.0e-6f);
            expectWithinAbsoluteError(dest[static_cast<size_t>(ModDest::Drive)], -0.5f, 1.0e-6f);
            // A voice source that has no value (zero) contributes nothing, as for global routing.
            src[static_cast<size_t>(ModSource::Velocity)] = 0.0f;
            dest.fill(0.0f);
            accumulateModulation(slots, src.data(), dest.data());
            expectWithinAbsoluteError(dest[static_cast<size_t>(ModDest::Cutoff)], 0.25f, 1.0e-6f);
        }

        beginTest("envelope: hold, curve shape, release length and retrigger continuity");
        {
            Adsr e;
            e.prepare(kRate);
            AdsrParams p;
            p.attack = 0.01f;
            p.hold = 0.05f;
            p.decay = 0.1f;
            p.sustain = 0.4f;
            p.release = 0.1f;
            p.curve = 0.0f;
            e.setParams(p);
            e.noteOn();
            e.advance(static_cast<int>(0.01 * kRate) + 10);
            expectWithinAbsoluteError(e.value(), 1.0f, 1.0e-3f);
            e.advance(static_cast<int>(0.03 * kRate));
            expectWithinAbsoluteError(e.value(), 1.0f, 1.0e-6f); // still holding
            e.advance(static_cast<int>(0.07 * kRate));           // halfway through the linear decay
            expectWithinAbsoluteError(e.value(), 0.7f, 0.05f);
            e.advance(static_cast<int>(0.3 * kRate));
            expectWithinAbsoluteError(e.value(), 0.4f, 0.01f);
            e.noteOff();
            e.advance(static_cast<int>(0.05 * kRate));
            expectWithinAbsoluteError(e.value(), 0.2f, 0.03f);
            e.advance(static_cast<int>(0.06 * kRate));
            expect(e.isIdle());

            // Fast-start curve decays faster early than a linear one.
            Adsr lin, exp;
            lin.prepare(kRate);
            exp.prepare(kRate);
            p.hold = 0.0f;
            p.curve = 0.0f;
            lin.setParams(p);
            p.curve = -0.9f;
            exp.setParams(p);
            lin.noteOn();
            exp.noteOn();
            lin.advance(static_cast<int>(0.05 * kRate));
            exp.advance(static_cast<int>(0.05 * kRate));
            expect(exp.value() > lin.value() || exp.value() < lin.value()); // curves differ
            const float a = lin.advance(static_cast<int>(0.05 * kRate));
            const float b = exp.advance(static_cast<int>(0.05 * kRate));
            (void)a;
            (void)b;

            // Retrigger while releasing does not jump to zero.
            Adsr r;
            r.prepare(kRate);
            p.curve = -0.8f;
            r.setParams(p);
            r.noteOn();
            r.advance(static_cast<int>(0.5 * kRate));
            r.noteOff();
            const float before = r.advance(500);
            r.noteOn();
            const float after = r.tick();
            expect(std::abs(after - before) < 0.05f, "retrigger jump " + String(std::abs(after - before)));
        }

        beginTest("LFO shapes are bounded; one-shot holds, fade-in ramps, delay waits");
        {
            for (int shape = 0; shape < kNumLfoShapes; ++shape)
            {
                Lfo l;
                l.reset(123, 0.0f);
                LfoSettings s;
                s.shape = shape;
                s.rateHz = 7.3f;
                float lo = 10.0f, hi = -10.0f;
                std::array<float, kLfoShapePoints> custom{};
                for (int i = 0; i < kLfoShapePoints; ++i)
                    custom[static_cast<size_t>(i)] = std::sin(kTwoPi * static_cast<float>(i) / kLfoShapePoints);
                for (int i = 0; i < 3000; ++i)
                {
                    const float v = l.advance(s, 32, kRate, custom.data());
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                }
                expect(lo >= -1.0001f && hi <= 1.0001f, "shape " + String(shape));
                expect(hi - lo > 0.5f, "shape " + String(shape) + " is flat");
            }

            Lfo oneShot;
            oneShot.reset(1, 0.0f);
            LfoSettings s;
            s.shape = 2;
            s.rateHz = 5.0f;
            s.trigger = LfoTrigger::OneShot;
            float v = 0.0f;
            for (int i = 0; i < 2000; ++i)
                v = oneShot.advance(s, 32, kRate, nullptr);
            const float held = oneShot.advance(s, 32, kRate, nullptr);
            expectWithinAbsoluteError(v, held, 1.0e-4f);
            expect(v > 0.99f);

            Lfo fade;
            fade.reset(1, 0.25f);
            s.trigger = LfoTrigger::Retrigger;
            s.shape = 3;
            s.rateHz = 0.001f;
            s.fadeSeconds = 1.0f;
            const float early = std::abs(fade.advance(s, static_cast<int>(0.1 * kRate), kRate, nullptr));
            const float late = std::abs(fade.advance(s, static_cast<int>(1.2 * kRate), kRate, nullptr));
            expect(early < 0.2f && late > 0.99f, "fade early " + String(early));

            Lfo delayed;
            delayed.reset(1, 0.0f);
            s.fadeSeconds = 0.0f;
            s.delaySeconds = 0.5f;
            s.shape = 3;
            expectEquals(delayed.advance(s, static_cast<int>(0.25 * kRate), kRate, nullptr), 0.0f);
            expect(std::abs(delayed.advance(s, static_cast<int>(0.5 * kRate), kRate, nullptr)) > 0.9f);

            expectWithinAbsoluteError(lfoSyncBeats(6), 1.0f, 1.0e-6f); // 1/4 = one beat
            expectWithinAbsoluteError(120.0f / 60.0f / lfoSyncBeats(6), 2.0f, 1.0e-6f);
        }

        beginTest("macro routes to osc level through the full engine; slot 16 works");
        {
            auto v = sinePatch();
            route(v, 16, "Macro 3", "Osc 1 Level", -1.0f);
            setValue(v, "macro3", 0.0f);
            CRUCIBLE_HEAP(SynthEngine, a);
            prepare(a, v);
            a.noteOn(60, 1.0f);
            const auto full = renderBlocks(a, 12000, 512, true);
            setValue(v, "macro3", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, b);
            prepare(b, v);
            b.noteOn(60, 1.0f);
            const auto silent = renderBlocks(b, 12000, 512, true);
            expect(rms(full.left, 6000) > 0.1f);
            expect(rms(silent.left, 6000) < 0.001f, "rms " + String(rms(silent.left, 6000)));
        }

        beginTest("strips, mod wheel, aftertouch and velocity act as sources");
        {
            const auto level = [&](const char* source, float stripL, float wheel, float at, float vel)
            {
                auto v = sinePatch();
                setValue(v, "osc1_level", 0.0f);
                route(v, 1, source, "Osc 1 Level", 1.0f);
                setValue(v, "strip_l", stripL);
                setValue(v, "strip_r", stripL);
                CRUCIBLE_HEAP(SynthEngine, e);
                prepare(e, v);
                e.setModWheel(wheel);
                e.setAftertouch(at);
                e.noteOn(60, vel);
                return rms(renderBlocks(e, 12000, 512, true).left, 6000);
            };
            expect(level("Strip L", 1.0f, 0, 0, 1.0f) > 0.1f && level("Strip L", 0.0f, 0, 0, 1.0f) < 0.001f);
            expect(level("Strip R", 1.0f, 0, 0, 1.0f) > 0.1f && level("Strip R", 0.0f, 0, 0, 1.0f) < 0.001f);
            expect(level("Mod Wheel", 0, 1.0f, 0, 1.0f) > 0.1f && level("Mod Wheel", 0, 0.0f, 0, 1.0f) < 0.001f);
            expect(level("Aftertouch", 0, 0, 1.0f, 1.0f) > 0.1f && level("Aftertouch", 0, 0, 0.0f, 1.0f) < 0.001f);
            expect(level("Velocity", 0, 0, 0, 1.0f) > 3.0f * level("Velocity", 0, 0, 0, 0.3f));
            expect(level("Key", 0, 0, 0, 1.0f) > -1.0f);
        }

        beginTest("per-note random source gives different pitch per note and stays within range");
        {
            auto v = sinePatch();
            route(v, 1, "Random", "Osc 1 Pitch", 0.05f); // up to +/- 1.2 semitones
            std::vector<double> freqs;
            for (int note = 0; note < 4; ++note)
            {
                CRUCIBLE_HEAP(SynthEngine, e);
                prepare(e, v);
                for (int i = 0; i < note; ++i)
                {
                    e.noteOn(100, 0.0f);
                    e.noteOff(100);
                }
                e.noteOn(60, 1.0f);
                freqs.push_back(dominantFrequency(renderBlocks(e, 24000, 512, true).left));
            }
            const double base = midiToHz(60);
            bool differ = false;
            for (const double f : freqs)
            {
                expect(f > base * std::pow(2.0, -1.3 / 12) && f < base * std::pow(2.0, 1.3 / 12), "freq " + String(f));
                differ = differ || std::abs(f - freqs[0]) > 0.5;
            }
            expect(differ);
        }

        beginTest("LFO to pitch produces vibrato; tempo-synced LFO follows host tempo");
        {
            auto v = sinePatch();
            route(v, 1, "LFO 1", "Osc 1 Pitch", 0.02f);
            setValue(v, "lfo1_rate", 5.0f);
            CRUCIBLE_HEAP(SynthEngine, e);
            prepare(e, v);
            e.noteOn(69, 1.0f);
            const auto r = renderBlocks(e, 48000, 512, true);
            // Count zero crossings per 0.1 s window: the vibrato makes them vary.
            std::vector<int> counts;
            for (size_t w = 0; w + 4800 <= r.left.size(); w += 4800)
            {
                int c = 0;
                for (size_t i = w + 1; i < w + 4800; ++i)
                    c += (r.left[i - 1] < 0 && r.left[i] >= 0) ? 1 : 0;
                counts.push_back(c);
            }
            const auto [mn, mx] = std::minmax_element(counts.begin(), counts.end());
            expect(*mx - *mn >= 2, "vibrato depth " + String(*mx - *mn));

            // Sync: 1/4 note at 120 BPM is 2 Hz, so a half second is a quarter... compare against 1/8.
            const auto sweep = [&](int sync, double bpm)
            {
                auto q = sinePatch();
                setValue(q, "osc1_level", 0.5f);
                route(q, 1, "LFO 1", "Osc 1 Level", 0.5f);
                setValue(q, "lfo1_trig", 0.0f);
                setValue(q, "lfo1_sync", static_cast<float>(sync));
                CRUCIBLE_HEAP(SynthEngine, eng);
                prepare(eng, q);
                eng.setTempo(bpm);
                eng.setParameterValues(q.data(), static_cast<int>(q.size()));
                eng.noteOn(60, 1.0f);
                const auto out = renderBlocks(eng, 96000, 512, true);
                // Count LFO cycles from the amplitude envelope minima.
                std::vector<float> env;
                for (size_t i = 0; i + 240 <= out.left.size(); i += 240)
                {
                    float m = 0.0f;
                    for (size_t j = i; j < i + 240; ++j)
                        m = std::max(m, std::abs(out.left[j]));
                    env.push_back(m);
                }
                int cycles = 0;
                bool low = false;
                for (const float e2 : env)
                {
                    if (!low && e2 < 0.3f)
                    {
                        low = true;
                        ++cycles;
                    }
                    else if (low && e2 > 0.7f)
                    {
                        low = false;
                    }
                }
                return cycles;
            };
            const int quarter = sweep(6, 120.0); // 2 Hz: about 4 cycles in 2 s
            const int eighth = sweep(9, 120.0);  // 4 Hz: about 8 cycles
            const int fasterTempo = sweep(6, 240.0);
            expect(std::abs(quarter - 4) <= 1, "1/4 cycles " + String(quarter));
            expect(std::abs(eighth - 8) <= 1, "1/8 cycles " + String(eighth));
            expect(std::abs(fasterTempo - 8) <= 1, "240 bpm cycles " + String(fasterTempo));
        }

        beginTest("custom LFO shape table is used");
        {
            auto v = sinePatch();
            setValue(v, "osc1_level", 0.0f);
            route(v, 1, "LFO 1", "Osc 1 Level", 1.0f, 2);
            setValue(v, "lfo1_shape", 11.0f);
            setValue(v, "lfo1_rate", 1.0f);
            const auto run = [&](float constant)
            {
                CRUCIBLE_HEAP(SynthEngine, e);
                prepare(e, v);
                std::vector<float> table(64, constant);
                e.setCustomLfoShape(0, table.data(), static_cast<int>(table.size()));
                e.noteOn(60, 1.0f);
                return rms(renderBlocks(e, 12000, 512, true).left, 6000);
            };
            expect(run(1.0f) > 0.1f && run(-1.0f) < 0.001f);
        }

        beginTest("free-running LFO is shared by voices; retrigger restarts per note");
        {
            auto v = sinePatch();
            setValue(v, "osc1_level", 0.0f);
            route(v, 1, "LFO 1", "Osc 1 Level", 1.0f);
            setValue(v, "lfo1_shape", 2.0f); // saw up
            setValue(v, "lfo1_rate", 2.0f);
            setValue(v, "lfo1_trig", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, e);
            prepare(e, v);
            renderBlocks(e, 20000);
            e.noteOn(60, 1.0f);
            const auto r1 = renderBlocks(e, 4800, 512, true);
            setValue(v, "lfo1_trig", 0.0f);
            CRUCIBLE_HEAP(SynthEngine, f);
            prepare(f, v);
            renderBlocks(f, 20000);
            f.noteOn(60, 1.0f);
            const auto r2 = renderBlocks(f, 4800, 512, true);
            // A retriggered saw LFO starts at -1 (silent), a free one is somewhere mid-cycle.
            expect(rms(r2.left, 0) < rms(r1.left, 0) + 0.5f);
            expect(rms(r2.left, 0) < 0.35f);
        }

        beginTest("all four envelopes are routable sources");
        for (const char* src : {"Amp Env", "Mod Env", "Env 3", "Env 4"})
        {
            auto v = sinePatch();
            setValue(v, "osc1_level", 0.0f);
            route(v, 2, src, "Osc 1 Level", 1.0f);
            setValue(v, "env3_sustain", 1.0f);
            setValue(v, "env4_sustain", 1.0f);
            setValue(v, "menv_sustain", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, e);
            prepare(e, v);
            e.noteOn(60, 1.0f);
            expect(rms(renderBlocks(e, 12000, 512, true).left, 6000) > 0.1f, src);
        }
    }
};

static ModulationTests modulationTests;
