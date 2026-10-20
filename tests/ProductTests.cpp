// SPDX-License-Identifier: MIT
// A product chosen per engine instance (4 oscillators, 2 filters, 8 macros, a layout of its own), next to the
// default product in the same process.

#include "GraphSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
ProductLayout testProduct()
{
    ProductLayout product;
    product.id = "test-product";
    product.sections = {{"sound", "Sound"}, {"fx", "FX"}};
    product.voice = {4, 2};
    product.macros = 8;
    return product;
}

EngineConfig productConfig()
{
    EngineConfig c;
    c.product = testProduct();
    return c;
}

void set(const ParameterModel& model, std::vector<float>& values, const std::string& id, float v)
{
    const int i = model.indexOf(id);
    if (i >= 0)
        values[static_cast<size_t>(i)] = v;
}

std::vector<float> productPatch(const ParameterModel& model)
{
    auto v = model.defaultValues();
    set(model, v, "amp_attack", 0.001f);
    set(model, v, "amp_sustain", 1.0f);
    set(model, v, "master_gain", 0.0f);
    set(model, v, "osc1_table", 0.0f);
    set(model, v, "osc1_pos", 1.0f);
    set(model, v, "filter_cutoff", 20000.0f);
    set(model, v, "filter_res", 0.0f);
    return v;
}

Rendered play(const EngineConfig& config, const std::vector<float>& values, int note, int samples)
{
    return playNote(config, values, note, samples);
}
} // namespace

class ProductTests : public UnitTest
{
public:
    ProductTests() : UnitTest("Product configuration (4 osc, 2 filters, 8 macros)", "Product") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("the layout replaces the default one for this engine only");
        {
            CRUCIBLE_HEAP(SynthEngine, engine, productConfig());
            const auto& layout = engine.registry().layout();
            expectEquals(layout.id, std::string("test-product"));
            expectEquals(layout.voice.oscillators, 4);
            expectEquals(layout.voice.filters, 2);
            expectEquals(layout.macros, 8);
            expectEquals(engine.model().sections().size(), size_t(2));
            expectEquals(engine.oscillatorSlots(), 4);
            expectEquals(engine.filterSlots(), 2);
            // The process default product is untouched, and it is too late to choose it differently.
            CRUCIBLE_HEAP(SynthEngine, plain);
            expectEquals(plain.registry().layout().id, std::string("crucible.default"));
            expectEquals(plain.oscillatorSlots(), 3);
            expectEquals(ModuleRegistry::instance().layout().id, std::string("crucible.default"));
            expect(!ModuleRegistry::configureProduct(ProductLayout{}));
            expect(!ModuleRegistry::configureVoiceGraph({3, 2}));
        }

        beginTest("parameter ids: eight macros, four oscillator slots, two filters");
        {
            CRUCIBLE_HEAP(SynthEngine, engine, productConfig());
            const auto& model = engine.model();
            for (int m = 1; m <= 8; ++m)
                expect(model.indexOf("macro" + std::to_string(m)) >= 0);
            expect(model.indexOf("macro9") < 0);
            expect(model.indexOf("osc4_level") >= 0);
            expect(model.indexOf("osc5_level") < 0);
            expect(model.indexOf("filter2_cutoff") >= 0);
            expect(model.indexOf("filter3_cutoff") < 0);
            // Macros 7 and 8 come after every id the six-macro product has, so none of those moved.
            expect(model.indexOf("macro7") > model.indexOf("strip_r"));
            expectEquals(model.indexOf("macro8"), model.indexOf("macro7") + 1);
            for (size_t page = 0; page < model.pages().size(); ++page)
                expect(model.paramsOnPage(static_cast<int>(page)).size() <= 6);
            const auto& names = model.modSourceNames();
            expectEquals(static_cast<int>(names.size()), kNumModSources);
            expectEquals(names[static_cast<size_t>(ModSource::PitchBend)], std::string("Pitch Bend"));
            expectEquals(names[static_cast<size_t>(ModSource::Macro7)], std::string("Macro 7"));
            expectEquals(names[static_cast<size_t>(ModSource::Macro8)], std::string("Macro 8"));
            // The default product in the same process still has six macros and three slots.
            CRUCIBLE_HEAP(SynthEngine, plain);
            expect(plain.model().indexOf("macro7") < 0);
            expect(plain.model().indexOf("osc4_level") < 0);
            expectEquals(static_cast<int>(plain.model().modSourceNames().size()), kNumModSources - 2);
        }

        beginTest("macros 7 and 8 route through matrix slots and assignable routes");
        {
            CRUCIBLE_HEAP(SynthEngine, probe, productConfig());
            const auto& model = probe.model();
            const auto& names = model.modSourceNames();
            const auto& dests = model.modDestNames();
            const float dest =
                static_cast<float>(std::find(dests.begin(), dests.end(), std::string("Osc 1 Level")) - dests.begin());
            for (const char* which : {"Macro 7", "Macro 8"})
            {
                const float src =
                    static_cast<float>(std::find(names.begin(), names.end(), std::string(which)) - names.begin());
                auto v = productPatch(model);
                set(model, v, "osc1_level", 0.0f);
                set(model, v, "mod1_src", src);
                set(model, v, "mod1_dst", dest);
                set(model, v, "mod1_amt", 1.0f);
                expect(play(productConfig(), v, 48, 4800).peak < 1.0e-6f, "macro at zero stays silent");
                set(model, v, std::string("macro") + which[6], 1.0f);
                expect(play(productConfig(), v, 48, 4800).peak > 0.05f, which);
            }

            auto w = productPatch(model);
            set(model, w, "osc1_level", 0.0f);
            set(model, w, "macro8", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, engine, productConfig());
            prepare(engine, w);
            ModRoute route;
            route.source = static_cast<int>(ModSource::Macro8);
            route.target = "osc1_level";
            route.amount = 1.0f;
            expect(engine.modRoutes().add(route));
            engine.noteOn(48, 1.0f);
            expect(renderBlocks(engine, 4800, 512, true).peak > 0.05f);
            expectEquals(routeSourceName(static_cast<int>(ModSource::Macro8), engine.model()), std::string("Macro 8"));
        }

        beginTest("rendering is deterministic");
        {
            CRUCIBLE_HEAP(SynthEngine, probe, productConfig());
            auto v = productPatch(probe.model());
            set(probe.model(), v, "macro7", 0.4f);
            const auto a = play(productConfig(), v, 52, 9600);
            const auto b = play(productConfig(), v, 52, 9600);
            expect(a.left == b.left && a.right == b.right);
        }
    }
};

static ProductTests productTests;
