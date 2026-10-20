// SPDX-License-Identifier: MIT
// The voice graph in the default shape (3 oscillators, 2 filters): slot limits from EngineConfig and
// the routing modes. The wide shape is covered by crucible_engine_graph_tests (VoiceGraphWideTests.cpp).

#include "GraphSupport.h"

using namespace crucible;
using namespace crucible::testing;

class VoiceGraphTests : public UnitTest
{
public:
    VoiceGraphTests() : UnitTest("Voice graph", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("the default shape is three oscillators and two filters, and cannot change once used");
        {
            const auto shape = ModuleRegistry::instance().layout().voice;
            expectEquals(shape.oscillators, 3);
            expectEquals(shape.filters, 2);
            expect(!ModuleRegistry::configureVoiceGraph({5, 4}));
            SynthEngine engine;
            expectEquals(engine.oscillatorSlots(), 3);
            expectEquals(engine.filterSlots(), 2);
        }

        beginTest("the default product has six macros and no macro 7 or 8; configureProduct is too late");
        {
            expectEquals(ModuleRegistry::instance().layout().macros, 6);
            expect(ParameterMap::indexOf("macro6") >= 0);
            expect(ParameterMap::indexOf("macro7") < 0);
            expectEquals(static_cast<int>(ParameterMap::modSourceNames().size()), 24);
            expectEquals(ParameterMap::modSourceNames().back(), std::string("Pitch Bend"));
            expect(!ModuleRegistry::configureProduct(ProductLayout{}));
            expect(!ModuleRegistry::configuredProduct().has_value());
        }

        beginTest("EngineConfig can switch higher oscillator slots off");
        {
            auto v = graphPatch();
            setValue(v, "osc2_level", 0.6f);
            setValue(v, "osc3_level", 0.6f);
            setValue(v, "osc2_table", 2.0f);
            EngineConfig one;
            one.numOscillators = 1;
            auto muted = v;
            setValue(muted, "osc2_level", 0.0f);
            setValue(muted, "osc3_level", 0.0f);
            const auto a = playNote(one, v, 48, 9600);
            const auto b = playNote(EngineConfig{}, muted, 48, 9600);
            const auto c = playNote(EngineConfig{}, v, 48, 9600);
            expect(a.left == b.left && a.right == b.right, "one slot equals the other slots muted");
            expect(a.left != c.left, "all slots differ");
        }

        beginTest("EngineConfig can switch filter 2 off");
        {
            auto v = graphPatch();
            setValue(v, "filter_route", 1.0f); // parallel
            setValue(v, "filter2_on", 1.0f);
            setValue(v, "filter2_cutoff", 300.0f);
            EngineConfig one;
            one.numFilters = 1;
            auto off = v;
            setValue(off, "filter2_on", 0.0f);
            const auto a = playNote(one, v, 48, 9600);
            // Parallel with filter 2 off still routes the signal past filter 2 (summed), unlike one slot.
            const auto c = playNote(EngineConfig{}, v, 48, 9600);
            expect(energy(a.left) > 0.0);
            expect(a.left != c.left);
            expect(a.finite && c.finite);
        }

        beginTest("split routing: dry bypasses both filters, filter routes are independent");
        {
            auto v = graphPatch();
            setValue(v, "filter_route", 2.0f);
            setValue(v, "filter_cutoff", 200.0f);
            setValue(v, "filter2_on", 1.0f);
            setValue(v, "filter2_cutoff", 20000.0f);
            setValue(v, "osc1_route", 3.0f); // dry
            const auto dry = playNote(EngineConfig{}, v, 48, 9600);
            setValue(v, "osc1_route", 0.0f); // filter 1 (200 Hz)
            const auto f1 = playNote(EngineConfig{}, v, 48, 9600);
            setValue(v, "osc1_route", 1.0f); // filter 2 (open)
            const auto f2 = playNote(EngineConfig{}, v, 48, 9600);
            expect(brightness(dry.left, 2000) > 10.0 * brightness(f1.left, 2000));
            expect(brightness(f2.left, 2000) > 0.5 * brightness(dry.left, 2000));
        }

        beginTest("serial routing runs the filters one after the other");
        {
            auto v = graphPatch();
            setValue(v, "filter_route", 0.0f);
            setValue(v, "filter_type", 5.0f); // low pass 24
            setValue(v, "filter_cutoff", 1500.0f);
            setValue(v, "filter2_type", 5.0f);
            setValue(v, "filter2_cutoff", 1500.0f);
            const auto one = playNote(EngineConfig{}, v, 48, 9600);
            setValue(v, "filter2_on", 1.0f);
            const auto two = playNote(EngineConfig{}, v, 48, 9600);
            expect(brightness(two.left, 2000) < 0.5 * brightness(one.left, 2000));
        }
    }
};

static VoiceGraphTests voiceGraphTests;
