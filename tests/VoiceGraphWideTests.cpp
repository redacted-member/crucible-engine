// SPDX-License-Identifier: MIT
// A wider voice graph: 5 oscillators and 4 filters (the shape is set in GraphMain.cpp).

#include "GraphSupport.h"

using namespace crucible;
using namespace crucible::testing;

class VoiceGraphWideTests : public UnitTest
{
public:
    VoiceGraphWideTests() : UnitTest("Voice graph (5 osc, 4 filters)", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("parameters exist for every slot and the route lists follow the filter count");
        {
            for (const char* id : {"osc4_table", "osc5_level", "osc5_uni_voices", "osc4_warp_src", "filter3_cutoff",
                                   "filter4_mix", "filter4_on"})
                expect(ParameterMap::indexOf(id) >= 0, id);
            expect(ParameterMap::indexOf("osc6_level") < 0);
            expect(ParameterMap::indexOf("filter5_cutoff") < 0);
            const auto& route = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("osc4_route"))];
            expectEquals(route.choices.size(), size_t(6)); // 4 filters, all, dry
            expectEquals(route.choices[4], std::string("All"));
            const auto& src = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("osc1_warp_src"))];
            expectEquals(src.choices.size(), size_t(5));
            for (size_t page = 0; page < ParameterMap::pages().size(); ++page)
                expect(ParameterMap::paramsOnPage(static_cast<int>(page)).size() <= 6);
            CRUCIBLE_HEAP(SynthEngine, engine);
            expectEquals(engine.oscillatorSlots(), 5);
            expectEquals(engine.filterSlots(), 4);
        }

        beginTest("oscillators 4 and 5 sound");
        {
            auto v = graphPatch();
            setValue(v, "osc1_level", 0.0f);
            const auto silent = playNote(EngineConfig{}, v, 48, 4800);
            expect(silent.peak < 1.0e-6f);
            for (const char* id : {"osc4_level", "osc5_level"})
            {
                auto w = v;
                setValue(w, id, 0.8f);
                setValue(w, "osc4_pos", 1.0f);
                setValue(w, "osc5_pos", 1.0f);
                const auto r = playNote(EngineConfig{}, w, 48, 4800);
                expect(r.peak > 0.05f, id);
                expect(r.finite);
            }
            // The pan of a higher slot places it.
            auto w = v;
            setValue(w, "osc5_level", 0.8f);
            setValue(w, "osc5_pan", -1.0f);
            const auto l = playNote(EngineConfig{}, w, 48, 4800);
            expect(energy(l.left) > 100.0 * energy(l.right));
        }

        beginTest("EngineConfig limits the active slots below the model");
        {
            auto v = graphPatch();
            setValue(v, "osc4_level", 0.8f);
            EngineConfig three;
            three.numOscillators = 3;
            const auto off = playNote(three, v, 48, 4800);
            const auto on = playNote(EngineConfig{}, v, 48, 4800);
            expect(off.left != on.left);
        }

        beginTest("four filters in series filter more than two");
        {
            auto v = graphPatch();
            setValue(v, "filter_route", 0.0f);
            for (const char* t : {"filter_type", "filter2_type", "filter3_type", "filter4_type"})
                setValue(v, t, 5.0f);
            for (const char* c : {"filter_cutoff", "filter2_cutoff", "filter3_cutoff", "filter4_cutoff"})
                setValue(v, c, 1200.0f);
            setValue(v, "filter2_on", 1.0f);
            const auto two = playNote(EngineConfig{}, v, 52, 9600);
            setValue(v, "filter3_on", 1.0f);
            setValue(v, "filter4_on", 1.0f);
            const auto four = playNote(EngineConfig{}, v, 52, 9600);
            expect(brightness(four.left, 2000) < 0.5 * brightness(two.left, 2000));
            expect(four.finite);
        }

        beginTest("split routing reaches a higher filter and the dry path");
        {
            auto v = graphPatch();
            setValue(v, "filter_route", 2.0f);
            setValue(v, "filter3_cutoff", 200.0f);
            setValue(v, "filter3_on", 1.0f);
            setValue(v, "filter3_type", 5.0f);
            setValue(v, "osc1_route", 5.0f); // dry (4 filters: 0..3 filters, 4 all, 5 dry)
            const auto dry = playNote(EngineConfig{}, v, 48, 9600);
            setValue(v, "osc1_route", 2.0f); // filter 3
            const auto f3 = playNote(EngineConfig{}, v, 48, 9600);
            expect(brightness(dry.left, 2000) > 10.0 * brightness(f3.left, 2000));
            setValue(v, "osc1_route", 4.0f); // all: filter 3 closes, the others are open
            const auto all = playNote(EngineConfig{}, v, 48, 9600);
            expect(brightness(all.left, 2000) > 2.0 * brightness(f3.left, 2000));
        }

        beginTest("slots beyond the fixed destinations are modulated per voice by matrix slots and routes");
        {
            const auto& names = ParameterMap::modDestNames();
            const auto it = std::find(names.begin(), names.end(), std::string("Osc 5 Level"));
            expect(it != names.end());
            const float dest = static_cast<float>(it - names.begin());
            auto v = graphPatch();
            setValue(v, "osc1_level", 0.0f);
            setValue(v, "osc5_pos", 1.0f);
            setValue(v, "macro1", 1.0f);
            const auto silent = playNote(EngineConfig{}, v, 48, 4800);
            expect(silent.peak < 1.0e-6f);
            setValue(v, "mod1_src", 14.0f); // Macro 1
            setValue(v, "mod1_dst", dest);
            setValue(v, "mod1_amt", 1.0f);
            expect(playNote(EngineConfig{}, v, 48, 4800).peak > 0.05f);

            // The same through an assignable route, which also gives a per-voice source (velocity).
            auto w = graphPatch();
            setValue(w, "osc1_level", 0.0f);
            setValue(w, "osc5_pos", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, w);
            ModRoute route;
            route.source = static_cast<int>(ModSource::Velocity);
            route.target = "osc5_level";
            route.amount = 1.0f;
            expect(engine.modRoutes().add(route));
            engine.noteOn(48, 1.0f);
            const auto loud = renderBlocks(engine, 4800, 512, true);
            expect(loud.peak > 0.05f);
            CRUCIBLE_HEAP(SynthEngine, soft);
            prepare(soft, w);
            soft.modRoutes().add(route);
            soft.noteOn(48, 0.1f);
            const auto quiet = renderBlocks(soft, 4800, 512, true);
            expect(quiet.peak < 0.5f * loud.peak);

            // A filter 4 cutoff route darkens only through that filter.
            auto f = graphPatch();
            setValue(f, "filter_route", 0.0f);
            setValue(f, "filter4_on", 1.0f);
            setValue(f, "filter4_type", 5.0f);
            setValue(f, "filter4_cutoff", 12000.0f);
            CRUCIBLE_HEAP(SynthEngine, plain);
            CRUCIBLE_HEAP(SynthEngine, swept);
            prepare(plain, f);
            prepare(swept, f);
            ModRoute down;
            down.source = static_cast<int>(ModSource::Velocity);
            down.target = "filter4_cutoff";
            down.amount = -0.8f;
            expect(swept.modRoutes().add(down));
            plain.noteOn(48, 1.0f);
            swept.noteOn(48, 1.0f);
            const auto a = renderBlocks(plain, 9600, 512, true);
            const auto b = renderBlocks(swept, 9600, 512, true);
            expect(brightness(b.left, 2000) < 0.2 * brightness(a.left, 2000));
        }

        beginTest("an oscillator can take its warp source from a higher slot");
        {
            auto v = graphPatch();
            setValue(v, "osc1_warp", 13.0f); // ring
            setValue(v, "osc1_warp_a", 1.0f);
            setValue(v, "osc1_warp_src", 4.0f);
            setValue(v, "osc5_level", 0.0f);
            const auto r = playNote(EngineConfig{}, v, 48, 4800);
            expect(r.finite && r.peak > 0.01f);
        }
    }
};

static VoiceGraphWideTests voiceGraphWideTests;
