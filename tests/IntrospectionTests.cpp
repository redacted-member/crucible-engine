// SPDX-License-Identifier: MIT
// P3: instance-scoped registry and parameter model, self-describing metadata, display text, describeJSON, the C
// introspection API, per-module patch migrations and two products in one process.

#include "TestSupport.h"

#include <crucible/Engine.h>
#include <algorithm>
#include <crucible/Introspection.h>
#include <crucible/ParamText.h>
#include <crucible/Patch.h>
#include <crucible/crucible.h>
#include <crucible/module/ParamBuilder.h>

#include <cctype>
#include <cstddef>
#include <set>
#include <thread>

using namespace crucible;
using namespace crucible::testing;

namespace
{
ProductLayout wideProduct()
{
    ProductLayout p;
    p.id = "wide-product";
    p.sections = {{"sound", "Sound"}, {"fx", "FX"}};
    p.voice = {4, 3};
    p.macros = 8;
    return p;
}

EngineConfig configFor(const ProductLayout* product)
{
    EngineConfig c;
    c.maxBlockSize = 256;
    if (product != nullptr)
        c.product = *product;
    return c;
}

// One scripted performance on the facade, advanced a block at a time so that two engines can be interleaved.
struct Performance
{
    explicit Performance(const ProductLayout* product) : engine(configFor(product))
    {
        engine.prepare(kRate, 256);
        engine.setParam("filter_cutoff", 1800.0f);
        engine.setParam("filter_res", 0.4f);
        engine.setParam("osc1_uni_voices", 3.0f);
        engine.setParam("osc1_uni_detune", 0.3f);
        engine.setParam("fx_ensemble_on", 1.0f);
        engine.setParam("fx_ra_on", 1.0f);
        engine.setParam("fx_ra_mix", 0.3f);
        engine.setParam("amp_release", 0.2f);
    }

    void block()
    {
        if (step == 0)
        {
            engine.noteOn(52, 0.9f);
            engine.noteOn(59, 0.7f);
        }
        if (step == 20)
            engine.noteOff(52);
        if (step == 30)
            engine.noteOn(64, 0.8f);
        if (step == 45)
        {
            engine.noteOff(59);
            engine.noteOff(64);
        }
        std::vector<float> l(256), r(256);
        engine.process(l.data(), r.data(), 256);
        left.insert(left.end(), l.begin(), l.end());
        right.insert(right.end(), r.begin(), r.end());
        ++step;
    }

    Engine engine;
    int step = 0;
    std::vector<float> left, right;
};

constexpr int kBlocks = 70;

struct Alone
{
    std::vector<float> left, right;
    int params = 0;
};

Alone renderAlone(const ProductLayout* product)
{
    Performance p(product);
    for (int i = 0; i < kBlocks; ++i)
        p.block();
    return {p.left, p.right, p.engine.model().size()};
}

float peakOf(const std::vector<float>& v)
{
    float m = 0.0f;
    for (const float s : v)
        m = std::max(m, std::abs(s));
    return m;
}

bool isAscii(const std::string& s)
{
    for (const char c : s)
        if (static_cast<unsigned char>(c) >= 128 || !std::isprint(static_cast<unsigned char>(c)))
            return false;
    return true;
}

ModuleDescriptor migratingModule()
{
    ModuleDescriptor d;
    d.id = "test.migrating";
    d.name = "Migrating Test Module";
    d.kind = ModuleKind::Performance;
    d.category = "Test";
    d.version = 3;
    d.params.push_back(params::floating("tm_level", "TM Level", "tm", 0.0f, 1.0f, 0.5f));
    d.params.push_back(params::floating("tm_gain_db", "TM Gain", "tm", -24.0f, 24.0f, 0.0f, "dB"));
    ModuleMigration one;
    one.fromVersion = 1;
    one.toVersion = 2;
    one.renames = {{"tm_volume", "tm_level"}};
    one.remaps = {{"tm_level", [](float v) { return v * 0.5f; }}};
    d.migrations.push_back(one);
    ModuleMigration two;
    two.fromVersion = 2;
    two.toVersion = 3;
    two.custom = [](PatchValues& values)
    {
        values.erase(std::remove_if(values.begin(), values.end(), [](const auto& e) { return e.first == "tm_obsolete"; }),
                     values.end());
        values.emplace_back("tm_gain_db", 6.0f);
    };
    d.migrations.push_back(two);
    return d;
}
} // namespace

class IntrospectionTests : public UnitTest
{
public:
    IntrospectionTests() : UnitTest("Instance-scoped registry and introspection", "Introspection") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        const ProductLayout wide = wideProduct();

        beginTest("two engines with different products in one process render as they do alone");
        {
            const Alone aloneDefault = renderAlone(nullptr);
            const Alone aloneWide = renderAlone(&wide);
            expect(peakOf(aloneDefault.left) > 0.01f && peakOf(aloneWide.left) > 0.01f, "both products make sound");
            expect(aloneDefault.params < aloneWide.params);

            // Interleaved block by block on one thread.
            Performance a(nullptr), b(&wide);
            for (int i = 0; i < kBlocks; ++i)
            {
                a.block();
                b.block();
            }
            expect(a.left == aloneDefault.left && a.right == aloneDefault.right, "default product, interleaved");
            expect(b.left == aloneWide.left && b.right == aloneWide.right, "wide product, interleaved");

            // Created in the opposite order, and running on two threads at once.
            Performance c(&wide), d(nullptr);
            std::thread t1(
                [&]
                {
                    for (int i = 0; i < kBlocks; ++i)
                        c.block();
                });
            std::thread t2(
                [&]
                {
                    for (int i = 0; i < kBlocks; ++i)
                        d.block();
                });
            t1.join();
            t2.join();
            expect(c.left == aloneWide.left && c.right == aloneWide.right, "wide product, threaded");
            expect(d.left == aloneDefault.left && d.right == aloneDefault.right, "default product, threaded");
        }

        beginTest("each engine owns its model");
        {
            Engine plain;
            Engine other(configFor(&wide));
            expect(&plain.model() != &other.model());
            expect(&plain.registry() != &other.registry());
            expect(other.model().indexOf("osc4_level") >= 0 && plain.model().indexOf("osc4_level") < 0);
            expectEquals(other.model().macros(), 8);
            expectEquals(plain.model().macros(), 6);
            // Engines can share one finished registry.
            const auto registry = ModuleRegistry::build(wide);
            EngineConfig c;
            c.registry = registry;
            Engine e1(c), e2(c);
            expect(&e1.registry() == registry.get() && &e2.model() == &registry->parameters());
        }

        beginTest("choice indices are global: an omitted oscillator type keeps its number and reports unavailable");
        {
            ProductLayout lean = ModuleRegistry::neutralLayout();
            lean.omit.push_back("osc.modal");
            Engine plain;
            Engine omitted(configFor(&lean));
            const int typePlain = plain.model().indexOf("osc1_type");
            const int typeLean = omitted.model().indexOf("osc1_type");
            expect(typePlain >= 0 && typeLean >= 0);
            const auto& a = plain.model().params()[static_cast<size_t>(typePlain)];
            const auto& b = omitted.model().params()[static_cast<size_t>(typeLean)];
            expect(a.choices == b.choices && a.choices.size() == 9);
            expect(a.unavailable.empty() && b.unavailable == std::vector<int>{6});
            expect(plain.choiceAvailable(typePlain, 6) && !omitted.choiceAvailable(typeLean, 6));
            for (int k = 0; k < 9; ++k)
                expect(k == 6 || omitted.choiceAvailable(typeLean, k), "choice " + std::to_string(k));
            expect(!omitted.choiceAvailable(typeLean, 9) && !omitted.choiceAvailable(typeLean, -1) &&
                   !omitted.choiceAvailable(-1, 0) && !omitted.choiceAvailable(omitted.model().indexOf("osc1_level"), 0));
            float harmonic = -1.0f, modal = -1.0f;
            expect(omitted.textToValue(typeLean, b.choices[7], harmonic) && harmonic == 7.0f);
            expect(omitted.textToValue(typeLean, b.choices[6], modal) && modal == 6.0f);
            // A patch can name the type as text; the number it loads to is the same in both products.
            for (Engine* e : {&plain, &omitted})
            {
                Patch patch;
                std::string error;
                const std::string doc = std::string(R"({"format":"crucible-patch","version":)") +
                                        std::to_string(kPatchFormatVersion) + R"(,"params":{"osc1_type":")" + a.choices[5] +
                                        R"(","osc2_type":)" + "7" + "}}";
                expect(patchFromJson(doc, patch, error), error);
                applyPatch(*e, patch);
                expectEquals(e->getParam("osc1_type"), 5.0f);
                expectEquals(e->getParam("osc2_type"), 7.0f);
                // Saved patches keep what was written.
                Patch back;
                expect(patchFromJson(patchToJson(patch), back, error) && back.paramTexts.size() == 1, error);
            }
            // A slot on the omitted type is silent; the same slot on a built-in type is audible. Only osc1 is left on.
            const auto renderOsc1 = [&lean](float type)
            {
                Engine e(configFor(&lean));
                e.prepare(kRate, 256);
                for (const ParamDef& p : e.model().params())
                    if (p.id.size() > 6 && p.id.compare(p.id.size() - 6, 6, "_level") == 0)
                        e.setParam(p.id, 0.0f);
                e.setParam("osc1_level", 1.0f);
                e.setParam("osc1_type", type);
                e.noteOn(60, 0.9f);
                float peak = 0.0f;
                std::vector<float> l(256), r(256);
                for (int i = 0; i < 20; ++i)
                {
                    e.process(l.data(), r.data(), 256);
                    peak = std::max(peak, peakOf(l));
                }
                return peak;
            };
            expect(renderOsc1(0.0f) > 0.01f, "built-in type is audible");
            expect(renderOsc1(6.0f) < 1e-6f, "omitted type is silent");
            omitted.setParam("osc1_type", 6.0f);
            expectEquals(omitted.getParam("osc1_type"), 6.0f);
        }

        beginTest("metadata is complete for every parameter");
        {
            for (const ProductLayout* product : {static_cast<const ProductLayout*>(nullptr), &wide})
            {
                Engine engine(configFor(product));
                const auto& model = engine.model();
                for (const auto& p : model.params())
                {
                    expect(!p.shortName.empty() && p.shortName.size() <= 6 && isAscii(p.shortName),
                           p.id + " short name '" + p.shortName + "'");
                    expect(!p.group.empty() && p.group.find('/') != std::string::npos, p.id + " group");
                    expect(p.uiHint != UiHint::Auto && p.polarity != Polarity::Auto && p.textFormat != TextFormat::Auto,
                           p.id + " resolved");
                    expect(!p.name.empty() && !p.module.empty(), p.id);
                    expect(engine.registry().find(p.module) != nullptr, p.id + " module");
                    if (p.type == ParamType::Choice)
                        expect(p.steps == static_cast<int>(p.choices.size()), p.id + " steps");
                    if (p.type == ParamType::Bool)
                        expect(p.uiHint == UiHint::Switch && p.steps == 2, p.id);
                    if (p.min < 0.0f && p.max > 0.0f)
                        expect(p.polarity == Polarity::Bipolar, p.id + " polarity");
                }
                for (const auto& m : engine.registry().modules())
                {
                    expect(!m.outputs.empty() || !m.inputs.empty(), m.id + " ports");
                    if (m.kind == ModuleKind::Effect)
                        expect(m.inputs.size() >= 1 && m.outputs.size() == 1, m.id + " effect ports");
                }
            }
            Engine engine;
            const auto& model = engine.model();
            expect(model.params()[static_cast<size_t>(model.indexOf("vector_x"))].uiHint == UiHint::Xy, "xy");
            expect(model.params()[static_cast<size_t>(model.indexOf("osc1_cloud_band1"))].uiHint == UiHint::Drawbar,
                   "drawbar");
            expect(model.params()[static_cast<size_t>(model.indexOf("fx_saturator_on"))].uiHint == UiHint::Switch, "switch");
            expect(model.params()[static_cast<size_t>(model.indexOf("filter_cutoff"))].routable, "routable");
        }

        beginTest("valueToText and textToValue are exact for every parameter");
        {
            for (const ProductLayout* product : {static_cast<const ProductLayout*>(nullptr), &wide})
            {
                Engine engine(configFor(product));
                const auto& model = engine.model();
                int checked = 0;
                for (int i = 0; i < model.size(); ++i)
                {
                    const auto& p = model.params()[static_cast<size_t>(i)];
                    std::vector<float> values = {p.min, p.max, p.defaultValue};
                    if (p.type == ParamType::Float)
                        for (int k = 0; k <= 40; ++k)
                            values.push_back(fromNormalised(p, static_cast<float>(k) / 40.0f));
                    else
                        for (float v = p.min; v <= p.max && values.size() < 400; v += 1.0f)
                            values.push_back(v);
                    for (const float v : values)
                    {
                        const std::string text = model.valueToText(i, v);
                        float back = 0.0f;
                        const bool ok = model.textToValue(i, text, back);
                        expect(ok && !text.empty(), p.id + " reads '" + text + "'");
                        if (!ok)
                            continue;
                        // The text is a fixed point of the round trip, and the value comes back exactly (floats
                        // print six significant digits, which every bound and default fits).
                        expect(model.valueToText(i, back) == text, p.id + " text '" + text + "' is stable");
                        const bool listed = v == p.min || v == p.max || v == p.defaultValue || p.type != ParamType::Float;
                        if (listed)
                            expect(back == v, p.id + " '" + text + "' gives " + String(back) + " not " + String(v));
                        else
                            expectWithinAbsoluteError(back, v, 6.0e-6f * std::max(1.0f, std::abs(v)) + 1.0e-9f,
                                                      p.id + " '" + text + "'");
                        ++checked;
                    }
                }
                expect(checked > model.size() * 3);
            }
            Engine engine;
            const auto& m = engine.model();
            float v = 0.0f;
            expect(m.textToValue(m.indexOf("filter_cutoff"), "1.5k", v) && v == 1500.0f);
            expect(m.textToValue(m.indexOf("filter_cutoff"), "440hz", v) && v == 440.0f);
            expect(m.textToValue(m.indexOf("amp_attack"), "20 ms", v) && v == 0.02f);
            expectEquals(m.valueToText(m.indexOf("filter_cutoff"), 1800.0f), std::string("1800 Hz"));
            expectEquals(m.valueToText(m.indexOf("fx_saturator_on"), 1.0f), std::string("On"));
            expectEquals(m.valueToText(m.indexOf("osc1_smp_root"), 60.0f), std::string("C4"));
            expect(m.textToValue(m.indexOf("osc1_smp_root"), "a#3", v) && v == 58.0f);
            expectEquals(m.valueToText(m.indexOf("fx_cc_ratio"), 4.0f), std::string("4:1"));
            expect(!m.textToValue(m.indexOf("filter_cutoff"), "loud", v));
            expect(!m.textToValue(-1, "1", v) && m.valueToText(m.size(), 0.0f).empty());

            const int type = m.indexOf("osc1_type");
            float exact = -1.0f;
            expect(m.textToValue(type, "Analog (VA)", exact));
            expect(m.textToValue(type, "Analog", v) && v == exact);
            expect(m.textToValue(type, "analog", v) && v == exact);
            expect(m.textToValue(type, "ANALOG (va)", v) && v == exact);
            expect(!m.textToValue(type, "Analo", v));
            const int scale = m.indexOf("key_scale");
            expect(m.textToValue(scale, "natural minor", v) && m.valueToText(scale, v) == "Natural Minor (Aeolian)");
            expect(!m.textToValue(scale, "Diminished", v)); // fits two scales
            expect(m.textToValue(scale, "Diminished (Half-Whole)", v));
            expect(m.indexOfHostId(hostIdOf("filter_cutoff")) == m.indexOf("filter_cutoff") &&
                   m.indexOfHostId(hostIdOf("nope")) == -1);
            expect(engine.paramIndexByHostId(hostIdOf("osc1_type")) == type);
            const int cutoff = m.indexOf("filter_cutoff");
            expect(engine.setParamText("filter_cutoff", "2k") && engine.getParam(cutoff) == 2000.0f);
            expect(!engine.setParamText("filter_cutoff", "loud") && engine.getParam(cutoff) == 2000.0f);
            expect(!engine.setParamText("nope", "1") && engine.setParamText(type, "analog"));
        }

        beginTest("module status: the legacy mechanism is in the description, and no built-in module is legacy");
        {
            Engine engine;
            const std::string text = engine.describeJSON();
            expect(text.find("\"status\": \"current\"") != std::string::npos ||
                       text.find("\"status\":\"current\"") != std::string::npos,
                   "describeJSON carries status");
            expect(text.find("superseded_by") != std::string::npos, "describeJSON carries superseded_by");
            // The first-generation effects were removed before v1; a retirement after v1 sets Legacy and supersededBy.
            for (const auto& m : engine.registry().modules())
            {
                expect(m.status == ModuleStatus::Current, m.id + " is current");
                expect(m.supersededBy.empty(), m.id + " names no replacement");
            }
            for (const char* removed : {"fx.drive", "fx.chorus", "fx.flanger", "fx.phaser", "fx.delay", "fx.reverb", "fx.comp",
                                        "fx.eq", "fx.peq", "fx.dimension", "fx.gate", "fx.conv"})
                expect(engine.registry().find(removed) == nullptr, std::string(removed) + " is gone");
            ProductDescription parsed;
            std::string error;
            expect(ProductDescription::fromJson(text, parsed, error), error);
            for (const auto& m : parsed.modules)
                expect(m.status == "current" && m.supersededBy.empty(), m.id);
        }

        beginTest("describeJSON round-trips");
        {
            for (const ProductLayout* product : {static_cast<const ProductLayout*>(nullptr), &wide})
            {
                Engine engine(configFor(product));
                const std::string text = engine.describeJSON();
                ProductDescription parsed;
                std::string error;
                expect(ProductDescription::fromJson(text, parsed, error), error);
                expectEquals(parsed.toJson(), text);
                expectEquals(static_cast<int>(parsed.params.size()), engine.model().size());
                expectEquals(parsed.modules.size(), engine.registry().modules().size());
                expectEquals(parsed.macros, engine.model().macros());
                expectEquals(parsed.oscillators, engine.registry().layout().voice.oscillators);
                bool equal = true;
                for (size_t i = 0; i < parsed.params.size(); ++i)
                {
                    const auto& a = parsed.params[i];
                    const auto& b = engine.model().params()[i];
                    equal = equal && a.id == b.id && a.min == b.min && a.max == b.max && a.defaultValue == b.defaultValue &&
                            a.skew == b.skew && a.choices == b.choices && a.shortName == b.shortName && a.group == b.group &&
                            a.textFormat == b.textFormat && a.uiHint == b.uiHint && a.polarity == b.polarity &&
                            a.modulatable == b.modulatable && a.voiceMod == b.voiceMod && a.steps == b.steps;
                }
                expect(equal, "parsed parameters equal the live model");
                // Instances: oscillator modules appear once per slot.
                for (const auto& m : parsed.modules)
                    if (m.id == "osc.sample")
                    {
                        expectEquals(static_cast<int>(m.instances.size()), engine.registry().layout().voice.oscillators);
                        expect(!m.instances[0].params.empty());
                    }
                // Live capabilities.
                bool found = false;
                for (const auto& m : parsed.modules)
                    if (m.id == "fx.convolution")
                        for (const auto& c : m.capabilities)
                            found = found || c == "convolution_ir";
                expect(found, "convolution advertises impulse responses");
                // Views: the hub's views are listed with their payload size and handed to the module that owns them.
                const auto view = [&](const std::string& instance, const std::string& id) -> const ViewDescription*
                {
                    for (const auto& v : parsed.views)
                        if (v.instance == instance && v.id == id)
                            return &v;
                    return nullptr;
                };
                const auto* response = view("filter", "response");
                expect(response != nullptr && response->kind == "vector" && response->maxElements == 256 &&
                           !response->live && !response->module.empty(),
                       "filter response view listed with its size");
                const auto* meter = view("master", "meter");
                expect(meter != nullptr && meter->maxElements == 3 && meter->live && meter->module.empty(),
                       "engine-wide view");
                expect(view("params", "filter_cutoff.mod") == nullptr, "per-parameter views are not listed one by one");
                bool owned = false, eqViews = false, sampleViews = false;
                for (const auto& m : parsed.modules)
                {
                    if (response != nullptr && m.id == response->module)
                        owned = std::find(m.views.begin(), m.views.end(), "response") != m.views.end();
                    if (m.id == "fx.eq_pro")
                        eqViews = std::find(m.views.begin(), m.views.end(), "spectrum") != m.views.end();
                    if (m.id == "osc.sample")
                        sampleViews = std::find(m.views.begin(), m.views.end(), "sample_info") != m.views.end() &&
                                      std::find(m.views.begin(), m.views.end(), "cloud_points") == m.views.end();
                }
                expect(owned, "the voice module lists the filter views");
                expect(eqViews, "an effect lists its views");
                expect(sampleViews, "an oscillator module lists the views of its family");
                for (const auto& v : parsed.views)
                    if (v.live && v.instance == "fx.eq_pro")
                        expect(v.maxElements > 0 && v.module == "fx.eq_pro", "effect views name their module");
            }
            ProductDescription bad;
            std::string error;
            expect(!ProductDescription::fromJson("{\"format\":\"nope\"}", bad, error));
            ProductLayout layout;
            expect(productLayoutFromJson(productLayoutToJson(wide), layout, error));
            expectEquals(productLayoutToJson(layout), productLayoutToJson(wide));
        }

        beginTest("the C API describes and creates products");
        {
            char error[200] = {0};
            CrucibleConfig config{};
            config.struct_size = sizeof(config);
            const char* wideJson = "{\"format\":\"crucible-product\",\"version\":1,\"id\":\"wide-product\","
                                   "\"layout\":{\"oscillators\":4,\"filters\":3,\"macros\":8}}";
            CrucibleEngine* wideEngine = crucible_create_from_product(&config, wideJson, nullptr, error, sizeof(error));
            CrucibleEngine* plainEngine = crucible_create(nullptr);
            expect(wideEngine != nullptr && plainEngine != nullptr, error);
            if (wideEngine != nullptr && plainEngine != nullptr)
            {
                expect(crucible_param_count(wideEngine) > crucible_param_count(plainEngine), "count");
                expect(crucible_param_index(wideEngine, "osc4_level") >= 0);
                expect(crucible_param_index(plainEngine, "osc4_level") < 0);
                const int cutoff = crucible_param_index(plainEngine, "filter_cutoff");
                CrucibleParamMeta meta{};
                meta.struct_size = sizeof(meta);
                expect(crucible_param_meta(plainEngine, cutoff, &meta) == 1);
                expectEquals(std::string(meta.id), std::string("filter_cutoff"));
                expect(meta.routable == 1 && std::strlen(meta.short_name) <= 6 && meta.group != nullptr, "meta");
                char text[64];
                expectEquals(crucible_value_to_text(plainEngine, cutoff, 1800.0f, text, sizeof(text)), 7);
                expectEquals(std::string(text), std::string("1800 Hz"));
                float value = 0.0f;
                expect(crucible_text_to_value(plainEngine, cutoff, "2k", &value) == 1 && value == 2000.0f);
                expect(crucible_text_to_value(plainEngine, cutoff, "x", &value) == 0);
                expect(crucible_param_meta(plainEngine, -1, &meta) == 0);
                CrucibleParamMeta unsized{};
                expect(crucible_param_meta(plainEngine, cutoff, &unsized) == 0 && unsized.id == nullptr,
                       "struct_size 0 fills nothing");
                CrucibleParamMeta shortMeta{};
                shortMeta.struct_size = offsetof(CrucibleParamMeta, name);
                expect(crucible_param_meta(plainEngine, cutoff, &shortMeta) == 1 && shortMeta.id != nullptr &&
                           shortMeta.name == nullptr && shortMeta.struct_size == offsetof(CrucibleParamMeta, name),
                       "only the fields that fit are filled");
                const int shape = crucible_param_index(plainEngine, "osc1_type");
                expect(crucible_param_choice(plainEngine, shape, 0) != nullptr);

                const int modules = crucible_module_count(plainEngine);
                expect(modules > 20, "modules");
                int paramsSeen = 0;
                bool sawEffect = false;
                for (int m = 0; m < modules; ++m)
                {
                    CrucibleModuleMeta mm{};
                    mm.struct_size = sizeof(mm);
                    expect(crucible_module_meta(plainEngine, m, &mm) == 1);
                    paramsSeen += mm.num_params;
                    expect(mm.superseded_by != nullptr);
                    expect((mm.legacy == 1) == (std::string(mm.superseded_by) != ""));
                    expect(mm.legacy == 0, std::string("C meta: ") + mm.id + " is current");
                    for (int k = 0; k < mm.num_params; ++k)
                        expect(crucible_module_param(plainEngine, m, k) >= 0);
                    if (std::string(mm.kind) == "effect")
                    {
                        sawEffect = true;
                        CruciblePortMeta port{};
                        expect(crucible_module_input(plainEngine, m, 0, &port) == 1 &&
                               std::string(port.type) == "audio.stereo");
                        expect(crucible_module_output(plainEngine, m, 0, &port) == 1);
                    }
                }
                expect(sawEffect, "effect");
                expectEquals(paramsSeen, crucible_param_count(plainEngine));

                const int jsonSize = crucible_describe_json(plainEngine, nullptr, 0);
                expect(jsonSize > 0);
                std::string jsonText(static_cast<size_t>(std::max(jsonSize, 0)) + 1, '\0');
                expectEquals(crucible_describe_json(plainEngine, jsonText.data(), jsonText.size()), jsonSize);
                jsonText.resize(static_cast<size_t>(std::max(jsonSize, 0)));
                const char* json = jsonText.c_str();
                ProductDescription parsed;
                std::string err;
                expect(json != nullptr && ProductDescription::fromJson(json, parsed, err), err);
                expectEquals(parsed.toJson(), std::string(json != nullptr ? json : ""));
                expect(json != nullptr && std::string(json) == Engine().describeJSON(), "same json");
                // The two engines process independently in one process.
                crucible_prepare(wideEngine, 48000.0, 256);
                crucible_prepare(plainEngine, 48000.0, 256);
                std::vector<float> l(256), r(256);
                crucible_note_on(wideEngine, 60, 0.8f);
                crucible_note_on(plainEngine, 60, 0.8f);
                crucible_process(wideEngine, l.data(), r.data(), 256, nullptr, 0);
                crucible_process(plainEngine, l.data(), r.data(), 256, nullptr, 0);
            }
            crucible_destroy(wideEngine);
            crucible_destroy(plainEngine);
            expect(crucible_create_from_product(nullptr, nullptr, nullptr, error, sizeof(error)) == nullptr);
            std::string cppError;
            expect(Engine::fromProduct("{not json", cppError) == nullptr && !cppError.empty());
            auto cppEngine = Engine::fromProduct(wideJson, cppError);
            expect(cppEngine != nullptr, cppError);
            if (cppEngine != nullptr)
                expect(cppEngine->model().indexOf("osc4_level") >= 0);
            expect(crucible_create_from_product(nullptr, "{not json", nullptr, error, sizeof(error)) == nullptr &&
                   error[0] != '\0');
        }

        beginTest("a product with the documented maximum of eight oscillators is created, and a failure never throws");
        {
            const char* json = "{\"format\":\"crucible-product\",\"version\":1,\"id\":\"eight-osc\",\"voice\":\"classic\","
                               "\"layout\":{\"oscillators\":8,\"filters\":2,\"macros\":6}}";
            std::string cppError;
            std::unique_ptr<Engine> cppEngine;
            bool threw = false;
            try
            {
                cppEngine = Engine::fromProduct(json, cppError);
            }
            catch (const std::exception& ex)
            {
                threw = true;
                cppError = ex.what();
            }
            expect(!threw, "Engine::fromProduct threw: " + cppError);
            expect(cppEngine != nullptr, cppError);
            if (cppEngine != nullptr)
                expect(cppEngine->model().indexOf("osc8_level") >= 0);
            char error[400] = {0};
            CrucibleEngine* engine = crucible_create_from_product(nullptr, json, nullptr, error, sizeof(error));
            expect(engine != nullptr, error);
            crucible_destroy(engine);
        }

        beginTest("a product the model cannot hold returns an error from Engine::fromProduct and a failing one from the C "
                  "API, neither throws");
        {
            // A node type whose 700 per-voice parameters exceed the limit of the parameter model.
            graph::NodeRegistry nodes = graph::NodeRegistry::builtIn();
            graph::NodeType wide = *nodes.find("amp.vca");
            wide.type = "test.wide";
            for (int i = 0; i < 700; ++i)
                wide.params.push_back(
                    params::voiceModulatable(params::floating("p" + std::to_string(i), "P", "test", 0.0f, 1.0f, 0.0f)));
            nodes.add(wide);
            const char* json = "{\"format\":\"crucible-product\",\"version\":1,\"id\":\"too-many-targets\","
                               "\"voice\":{\"nodes\":[{\"id\":\"o\",\"type\":\"osc.sub\",\"slot\":1},"
                               "{\"id\":\"w\",\"type\":\"test.wide\"},{\"id\":\"m\",\"type\":\"mix.out\"}],"
                               "\"edges\":[{\"from\":\"o.out\",\"to\":\"w.in\"},{\"from\":\"w.out\",\"to\":\"m.in\"}]}}";
            std::string cppError;
            std::unique_ptr<Engine> cppEngine;
            bool threw = false;
            try
            {
                cppEngine = Engine::fromProduct(json, cppError, {}, &nodes);
            }
            catch (...)
            {
                threw = true;
            }
            expect(!threw, "Engine::fromProduct threw");
            expect(cppEngine == nullptr, "no engine for a product beyond the limit");
            expect(!cppError.empty(), "error text");
            expect(cppError.find("640") != std::string::npos, cppError);
            // The C entry point catches std::exception around the same construction; the plain-C module registry cannot add
            // node types, so only the compile-failure path (unknown node type) is reachable from here.
            bool cThrew = false;
            char error[400] = {0};
            CrucibleEngine* engine = nullptr;
            try
            {
                engine = crucible_create_from_product(nullptr,
                                                      "{\"format\":\"crucible-product\",\"version\":1,\"id\":\"bad\","
                                                      "\"voice\":{\"nodes\":[{\"id\":\"w\",\"type\":\"test.wide\"}]}}",
                                                      nullptr, error, sizeof(error));
            }
            catch (...)
            {
                cThrew = true;
            }
            expect(!cThrew, "crucible_create_from_product threw");
            expect(engine == nullptr && error[0] != '\0', error);
            crucible_destroy(engine);
        }

        beginTest("host ids are stable hashes of the parameter ids");
        {
            expectEquals(static_cast<int>(hostIdOf("")),
                         static_cast<int>(2166136261u & 0x7fffffffu)); // FNV-1a 32, top bit cleared
            expectEquals(static_cast<int>(hostIdOf("a")),
                         static_cast<int>(0x640c292cu)); // published FNV-1a 32 value 0xe40c292c, top bit cleared
            const char* wideJson = "{\"format\":\"crucible-product\",\"version\":1,\"id\":\"wide-product\","
                                   "\"layout\":{\"oscillators\":4,\"filters\":3,\"macros\":8}}";
            char error[400] = {0};
            CrucibleEngine* wideEngine = crucible_create_from_product(nullptr, wideJson, nullptr, error, sizeof(error));
            CrucibleEngine* plainEngine = crucible_create(nullptr);
            expect(wideEngine != nullptr && plainEngine != nullptr, error);
            if (wideEngine != nullptr && plainEngine != nullptr)
            {
                std::set<uint32_t> seen;
                int shared = 0;
                for (int i = 0; i < crucible_param_count(plainEngine); ++i)
                {
                    CrucibleParamMeta m{}, w{};
                    m.struct_size = w.struct_size = sizeof(m);
                    expect(crucible_param_meta(plainEngine, i, &m) == 1);
                    expect(seen.insert(m.host_id).second, std::string("host id collision at ") + m.id);
                    expect(m.host_id != 0 && m.host_id < 0x80000000u, std::string("host id in the 31-bit range: ") + m.id);
                    const int j = crucible_param_index(wideEngine, m.id);
                    if (j >= 0 && crucible_param_meta(wideEngine, j, &w) == 1)
                    {
                        ++shared;
                        expect(w.host_id == m.host_id, std::string("same host id in both products: ") + m.id);
                    }
                }
                expect(shared > 300, "the products share their parameters");
            }
            crucible_destroy(wideEngine);
            crucible_destroy(plainEngine);

            // "zz_mx5kacs3" and "filter_cutoff" hash to the same 32-bit value.
            const char* clash =
                "{\"format\":\"crucible-product\",\"version\":1,\"id\":\"clash\","
                "\"fx\":{\"instances\":[{\"id\":\"a\",\"type\":\"fx.saturator\"},{\"id\":\"b\",\"type\":\"fx.saturator\"}],"
                "\"lanes\":[{\"id\":\"main\",\"order\":\"listed\",\"input\":[\"voices\"],\"chain\":[\"a\",\"b\"],\"master\":"
                "true}]},"
                "\"idMap\":{\"fx2_saturator_mix\":\"zz_mx5kacs3\"}}";
            expectEquals(static_cast<int>(hostIdOf("zz_mx5kacs3")), static_cast<int>(hostIdOf("filter_cutoff")));
            std::string clashError;
            expect(Engine::fromProduct(clash, clashError) == nullptr);
            expect(clashError.find("zz_mx5kacs3") != std::string::npos &&
                       clashError.find("filter_cutoff") != std::string::npos &&
                       clashError.find("host id") != std::string::npos,
                   clashError);
        }

        beginTest("per-module migrations bring old patches up to the module versions");
        {
            auto registry = ModuleRegistry::builder(ModuleRegistry::neutralLayout());
            registry->add(migratingModule());
            registry->finalize();
            expect(!registry->finalized() == false);
            bool refused = false;
            try
            {
                registry->add(migratingModule());
            }
            catch (const std::logic_error&)
            {
                refused = true;
            }
            expect(refused, "no module can be added once the registry is finalized");

            EngineConfig c;
            c.registry = registry;
            Engine engine(c);
            // A product with an extra module is its own product: the default engine does not see it.
            expect(engine.model().indexOf("tm_level") >= 0 && Engine().model().indexOf("tm_level") < 0);

            Patch saved = capturePatch(engine);
            expectEquals(saved.moduleVersions["test.migrating"], 3);

            Patch old;
            old.moduleVersions["test.migrating"] = 1;
            old.params = {{"tm_volume", 0.8f}, {"tm_obsolete", 1.0f}, {"filter_cutoff", 900.0f}};
            applyPatch(engine, old);
            expectWithinAbsoluteError(engine.getParam("tm_level"), 0.4f, 1.0e-6f);
            expectEquals(engine.getParam("tm_gain_db"), 6.0f);
            expectEquals(engine.getParam("filter_cutoff"), 900.0f);

            // A parameter given as text under its old id is renamed like a number, then parsed against the new one.
            Patch oldText;
            oldText.moduleVersions["test.migrating"] = 1;
            oldText.paramTexts = {{"tm_volume", "0.8"}, {"tm_obsolete", "1"}, {"filter_cutoff", "900"}};
            applyPatch(engine, oldText);
            expectWithinAbsoluteError(engine.getParam("tm_level"), 0.8f, 1.0e-6f);
            expectEquals(engine.getParam("tm_gain_db"), 6.0f);
            expectEquals(engine.getParam("filter_cutoff"), 900.0f);

            Patch mid;
            mid.moduleVersions["test.migrating"] = 2;
            mid.params = {{"tm_level", 0.8f}, {"tm_obsolete", 1.0f}};
            applyPatch(engine, mid);
            expectWithinAbsoluteError(engine.getParam("tm_level"), 0.8f, 1.0e-6f);
            expectEquals(engine.getParam("tm_gain_db"), 6.0f);

            Patch current;
            current.moduleVersions["test.migrating"] = 3;
            current.params = {{"tm_level", 0.8f}, {"tm_gain_db", -3.0f}};
            applyPatch(engine, current);
            expectEquals(engine.getParam("tm_gain_db"), -3.0f);
            expectWithinAbsoluteError(engine.getParam("tm_level"), 0.8f, 1.0e-6f);

            // A patch without module versions counts as version 1 for every module; modules without migrations
            // (all built-ins) leave the values alone.
            Patch bare;
            bare.params = {{"filter_cutoff", 777.0f}};
            applyPatch(engine, bare);
            expectEquals(engine.getParam("filter_cutoff"), 777.0f);
        }

        beginTest("scenes, routes and CC maps of an engine follow its own model");
        {
            Engine engine(configFor(&wide));
            engine.setParam("osc4_level", 0.6f);
            engine.storeScene(2);
            engine.setParam("osc4_level", 0.0f);
            expect(engine.recallScene(2));
            expectWithinAbsoluteError(engine.getParam("osc4_level"), 0.6f, 1.0e-6f);
            ModRoute route;
            route.source = static_cast<int>(ModSource::Macro8);
            route.target = "osc4_level";
            route.amount = 0.5f;
            expect(engine.modRoutes().add(route));
            Engine plain;
            expect(!plain.modRoutes().add(route), "the default product has neither macro 8 nor osc4");
            CcBinding binding;
            binding.cc = 20;
            binding.param = "osc4_level";
            expect(engine.ccMap().add(binding));
            expect(!plain.ccMap().add(binding));
            const Patch p = capturePatch(engine);
            expect(p.param("osc4_level", -1.0f) >= 0.0f);
            Engine again(configFor(&wide));
            applyPatch(again, p);
            expectWithinAbsoluteError(again.getParam("osc4_level"), engine.getParam("osc4_level"), 1.0e-6f);
        }
    }
};

static IntrospectionTests introspectionTests;
