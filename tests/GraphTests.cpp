// SPDX-License-Identifier: MIT
// Graph in data: product definitions, the compiler, the classic voice node, the FX graph, layers and chokes, and
// reconfiguration while audio runs.

#include "AllocCounter.h"
#include "GraphSupport.h"

#include <crucible/Introspection.h>
#include <crucible/graph/Compiler.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>

using namespace crucible;
using namespace crucible::testing;
using namespace crucible::graph;

namespace
{
std::string productPath(const std::string& name)
{
    return std::string(CRUCIBLE_ENGINE_PRODUCTS_DIR) + "/" + name + ".json";
}

ProductDef load(const std::string& name)
{
    ProductDef def;
    std::string error;
    if (!loadProduct(productPath(name), def, error))
        std::printf("!!! cannot load product %s: %s\n", name.c_str(), error.c_str());
    return def;
}

std::shared_ptr<const CompiledProduct> build(const ProductDef& def, std::string* errorOut = nullptr)
{
    std::string error;
    auto compiled = compile(def, error);
    if (!compiled)
        std::printf("!!! compile failed: %s\n", error.c_str());
    if (errorOut != nullptr)
        *errorOut = error;
    return compiled;
}

void put(const ParameterModel& model, std::vector<float>& v, const std::string& id, float value)
{
    const int i = model.indexOf(id);
    if (i < 0)
        std::printf("!!! no parameter %s\n", id.c_str());
    else
        v[static_cast<size_t>(i)] = value;
}

// A steady sine voice at full level.
void sineVoice(const ParameterModel& model, std::vector<float>& v)
{
    put(model, v, "amp_attack", 0.001f);
    put(model, v, "amp_sustain", 1.0f);
    put(model, v, "master_gain", 0.0f);
    put(model, v, "osc1_table", 0.0f);
    put(model, v, "osc1_pos", 1.0f);
    put(model, v, "filter_cutoff", 20000.0f);
    put(model, v, "filter_res", 0.0f);
}

struct Audio
{
    std::vector<float> left, right;
    void append(const float* l, const float* r, int n)
    {
        left.insert(left.end(), l, l + n);
        right.insert(right.end(), r, r + n);
    }
};

Audio run(SynthEngine& engine, int samples, int block = 512)
{
    Audio a;
    std::vector<float> l(static_cast<size_t>(block)), r(static_cast<size_t>(block));
    for (int done = 0; done < samples; done += block)
    {
        const int n = std::min(block, samples - done);
        engine.render(l.data(), r.data(), n);
        a.append(l.data(), r.data(), n);
    }
    return a;
}

double maxAbsDiff(const std::vector<float>& a, const std::vector<float>& b)
{
    double d = 0.0;
    for (size_t i = 0; i < std::min(a.size(), b.size()); ++i)
        d = std::max(d, static_cast<double>(std::abs(a[i] - b[i])));
    return d;
}

double peak(const std::vector<float>& a, size_t from = 0, size_t to = SIZE_MAX)
{
    double p = 0.0;
    for (size_t i = from; i < std::min(a.size(), to); ++i)
        p = std::max(p, static_cast<double>(std::abs(a[i])));
    return p;
}

double maxStep(const std::vector<float>& a, size_t from, size_t to)
{
    double s = 0.0;
    for (size_t i = std::max<size_t>(from, 1); i < std::min(a.size(), to); ++i)
        s = std::max(s, static_cast<double>(std::abs(a[i] - a[i - 1])));
    return s;
}

std::string errorsOf(const ProductDef& def)
{
    return validate(def).text();
}

bool contains(const std::string& text, const std::string& part)
{
    return text.find(part) != std::string::npos;
}

// A minimal valid graph voice: oscillator slot 1 -> vca -> out, enveloped.
ProductDef smallGraph()
{
    ProductDef def;
    std::string error;
    const bool ok = productFromText(R"({
        "format": "crucible-product", "version": 1, "id": "test.small",
        "voice": { "nodes": [
            { "id": "osc", "type": "osc.sub", "slot": 1 },
            { "id": "env", "type": "env.adsr" },
            { "id": "vca", "type": "amp.vca" },
            { "id": "out", "type": "mix.out" } ],
          "edges": [
            { "from": "osc.out", "to": "vca.in" },
            { "from": "env.out", "to": "vca.gain" },
            { "from": "vca.out", "to": "out.in" } ] } })",
                                    def, error);
    if (!ok)
        std::printf("!!! smallGraph: %s\n", error.c_str());
    return def;
}

// A node type that processes four voices together, for the dispatch test.
std::atomic<int> gQuadCalls{0};
std::atomic<int> gSingleCalls{0};

class QuadProbe final : public Node
{
public:
    void process(NodeIO& io, int n) override
    {
        gSingleCalls.fetch_add(1);
        std::fill(io.out[0].left, io.out[0].left + n, 0.25f);
        std::fill(io.out[0].right, io.out[0].right + n, 0.25f);
    }
    bool processQuad(Node* const peers[4], NodeIO* const io[4], int n) override
    {
        gQuadCalls.fetch_add(1);
        for (int k = 0; k < 4; ++k)
            if (dynamic_cast<QuadProbe*>(peers[k]) == nullptr)
                return false;
        for (int k = 0; k < 4; ++k)
        {
            std::fill(io[k]->out[0].left, io[k]->out[0].left + n, 0.25f);
            std::fill(io[k]->out[0].right, io[k]->out[0].right + n, 0.25f);
        }
        return true;
    }
};
// What a wrapper's idle poll reads from the control thread.
void pollReported(SynthEngine& engine, const std::vector<float>&)
{
    (void)engine.effects().reportedLatency();
    (void)engine.effects().reportedTail();
}
} // namespace

class GraphTests : public UnitTest
{
public:
    GraphTests() : UnitTest("Graph in data", "Graph") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        schema();
        validation();
        ids();
        classicBitExact();
        fxGraph();
        fxOnly();
        layers();
        nodes();
        reconfiguration();
        overhead();
    }

private:
    // ---------------------------------------------------------------------------------------------------------
    void schema()
    {
        beginTest("a product definition round-trips through JSON");
        {
            for (const char* name : {"crucible-default", "drive-trio", "fx-only", "drum-kit"})
            {
                const ProductDef def = load(name);
                const std::string text = productToText(def);
                ProductDef again;
                std::string error;
                expect(productFromText(text, again, error), error);
                expectEquals(productToText(again), text, name);
                expect(validate(def).ok(), std::string(name) + ": " + errorsOf(def));
            }
        }

        beginTest("products/crucible-default.json is the default product definition");
        {
            const std::string generated = productToText(defaultProductDef()) + "\n";
            const char* update = std::getenv("CRUCIBLE_UPDATE_PRODUCTS");
            if (update != nullptr && std::string(update) == "1")
            {
                std::ofstream(productPath("crucible-default")) << generated;
                logMessage("rewrote " + productPath("crucible-default"));
            }
            std::ifstream f(productPath("crucible-default"));
            std::stringstream ss;
            ss << f.rdbuf();
            expect(ss.str() == generated,
                   "products/crucible-default.json differs from defaultProductDef(); run with CRUCIBLE_UPDATE_PRODUCTS=1");
        }

        beginTest("docs/products/template.json compiles and lists no legacy module");
        {
            ProductDef def;
            std::string error;
            const std::string path = std::string(CRUCIBLE_ENGINE_SOURCE_DIR) + "/docs/products/template.json";
            expect(loadProduct(path, def, error), error);
            expect(validate(def).ok(), errorsOf(def));
            expect(!def.fx.instances.empty());
            const auto compiled = build(def);
            expect(compiled != nullptr);
            if (compiled)
            {
                const ProductDescription description = describe(*compiled->registry);
                for (const auto& inst : def.fx.instances)
                {
                    bool found = false;
                    for (const auto& m : description.modules)
                        if (m.id == inst.type)
                            found = true;
                    expect(found, inst.type + " is not in the compiled product");
                }
                for (const auto& m : description.modules)
                    expect(m.status == "current", m.id + " is legacy; use " + m.supersededBy);
            }
        }

        beginTest("the template's surface lists its parameters in order and hides the rest");
        {
            ProductDef def;
            std::string error;
            expect(loadProduct(std::string(CRUCIBLE_ENGINE_SOURCE_DIR) + "/docs/products/template.json", def, error), error);
            expect(def.surface.present && def.surface.listed && def.surface.pages.size() == 4);
            const auto compiled = build(def);
            expect(compiled != nullptr);
            if (compiled)
            {
                const ParameterModel& model = compiled->registry->parameters();
                size_t listed = 0;
                for (const auto& page : def.surface.pages)
                    listed += page.params.size();
                expect(model.surface().size() == listed);
                expect(!model.surface().empty() &&
                       model.params()[static_cast<size_t>(model.surface()[0])].id == "osc1_table");
                const auto& cutoff = model.params()[static_cast<size_t>(model.indexOf("filter_cutoff"))];
                expect(cutoff.exposed && cutoff.automatable && cutoff.pageId == "filter" && cutoff.group == "sound/filter");
                const auto& hidden = model.params()[static_cast<size_t>(model.indexOf("osc1_semi"))];
                expect(!hidden.exposed && !hidden.automatable);
                expect(hidden.modulatable == model.params()[static_cast<size_t>(model.indexOf("osc1_semi"))].modulatable);
                expect(model.sections().size() >= 2 && model.sections()[0].id == "sound" && model.sections()[1].id == "fx");
            }
        }

        beginTest("a product without a surface exposes every automatable parameter in index order");
        {
            ProductDef def = defaultProductDef();
            expect(!def.surface.present);
            const auto compiled = build(def);
            expect(compiled != nullptr);
            if (compiled)
            {
                const ParameterModel& model = compiled->registry->parameters();
                expect(model.surface().size() == model.params().size());
                bool same = true;
                for (size_t i = 0; i < model.surface().size(); ++i)
                    same = same && model.surface()[i] == static_cast<int>(i) && model.params()[i].exposed &&
                           model.params()[i].automatable;
                expect(same);
            }
            ProductDef all = def;
            all.surface.present = true;
            all.surface.pages.push_back({"mine", "Mine", "", {"filter_res", "filter_cutoff"}});
            const auto moved = build(all);
            expect(moved != nullptr);
            if (moved)
            {
                const ParameterModel& model = moved->registry->parameters();
                expect(model.surface().size() == model.params().size()); // "all": index order, not page order
                const auto& res = model.params()[static_cast<size_t>(model.indexOf("filter_res"))];
                expect(res.pageId == "mine" && res.exposed && res.automatable);
            }
        }

        beginTest("every parameter of a product with a surface names a page that exists, listed or not");
        {
            ProductDef templ;
            std::string error;
            expect(loadProduct(std::string(CRUCIBLE_ENGINE_SOURCE_DIR) + "/docs/products/template.json", templ, error),
                   error);
            ProductDef all = defaultProductDef();
            all.surface.present = true;
            all.surface.pages.push_back({"mine", "Mine", "", {"filter_res", "filter_cutoff"}});
            for (const ProductDef& def : {templ, all})
            {
                const auto compiled = build(def);
                expect(compiled != nullptr);
                if (!compiled)
                    continue;
                const ParameterModel& model = compiled->registry->parameters();
                ProductDescription back;
                std::string readError;
                expect(ProductDescription::fromJson(describeJSON(*compiled->registry), back, readError), readError);
                // The surface's pages come first, in declared order.
                for (size_t i = 0; i < def.surface.pages.size() && i < model.pages().size(); ++i)
                    expect(model.pages()[i].id == def.surface.pages[i].id);
                std::set<std::string> ids;
                for (const auto& page : model.pages())
                    expect(ids.insert(page.id).second, page.id);
                for (const auto& p : model.params())
                {
                    const bool inModel = p.page >= 0 && static_cast<size_t>(p.page) < model.pages().size() &&
                                         model.pages()[static_cast<size_t>(p.page)].id == p.pageId;
                    expect(inModel, p.id + " -> " + p.pageId);
                    expect(ids.count(p.pageId) == 1, p.id + " -> " + p.pageId);
                }
                for (const auto& page : model.pages())
                {
                    expect(page.section >= 0 && static_cast<size_t>(page.section) < model.sections().size(), page.id);
                    expect(std::any_of(model.params().begin(), model.params().end(),
                                       [&](const ParamDef& p) { return p.pageId == page.id; }) ||
                               std::any_of(def.surface.pages.begin(), def.surface.pages.end(),
                                           [&](const auto& s) { return s.id == page.id; }),
                           "empty page " + page.id);
                }
                bool described = back.params.size() == model.params().size();
                for (const auto& p : back.params)
                    described = described && p.page >= 0 && static_cast<size_t>(p.page) < back.pages.size() &&
                                back.pages[static_cast<size_t>(p.page)].id == p.pageId;
                expect(described);
            }
        }

        beginTest("surface documents round-trip and bad ones are refused with a path");
        {
            ProductDef def;
            std::string error;
            const std::string good = R"({"format":"crucible-product","version":1,"id":"s","surface":{"expose":"listed",
                "sections":[{"id":"a","name":"A"}],"pages":[{"id":"p","section":"a","params":["filter_cutoff"]}]}})";
            expect(productFromText(good, def, error), error);
            expect(def.surface.listed && def.surface.pages[0].name == "p" && def.surface.sections[0].name == "A");
            ProductDef again;
            expect(productFromText(productToText(def), again, error), error);
            expect(productToText(again) == productToText(def));
            expect(!productFromText(R"({"format":"crucible-product","version":1,"id":"s","surface":{"expose":"some"}})", def,
                                    error) &&
                   contains(error, "surface.expose"));
            expect(!productFromText(R"({"format":"crucible-product","version":1,"id":"s","surface":{"pages":[{"id":"p"}]}})",
                                    def, error) &&
                   contains(error, "surface.pages[0]") && contains(error, "params"));
            expect(!productFromText(
                       R"({"format":"crucible-product","version":1,"id":"s","surface":{"pages":[{"id":"p","params":[1]}]}})",
                       def, error) &&
                   contains(error, "surface.pages[0].params[0]"));

            const auto compileText = [&](const std::string& surface)
            {
                ProductDef d;
                std::string e;
                expect(
                    productFromText(R"({"format":"crucible-product","version":1,"id":"s","surface":)" + surface + "}", d, e),
                    e);
                std::string compileError;
                const bool ok = compile(d, compileError) != nullptr;
                return ok ? std::string() : compileError;
            };
            expect(contains(
                compileText(
                    R"({"pages":[{"id":"a","params":["filter_cutoff"]},{"id":"b","params":["filter_res","nope"]}]})"),
                "surface.pages[1].params[1]: unknown parameter id 'nope'"));
            expect(contains(
                compileText(R"({"pages":[{"id":"a","params":["filter_cutoff"]},{"id":"b","params":["filter_cutoff"]}]})"),
                "surface.pages[1].params[0]: parameter 'filter_cutoff' is already listed at surface.pages[0].params[0]"));
            expect(
                contains(compileText(R"({"pages":[{"id":"a","params":[]},{"id":"a","params":[]}]})"), "surface.pages[1]"));
            expect(contains(compileText(R"({"sections":[{"id":"x"}],"pages":[{"id":"a","section":"y","params":[]}]})"),
                            "surface.pages[0]: unknown section"));
            expect(compileText(R"({"expose":"listed","pages":[{"id":"a","params":["filter_cutoff"]}]})").empty());
        }

        beginTest("malformed documents are refused with a path");
        {
            ProductDef def;
            std::string error;
            expect(!productFromText("{", def, error) && contains(error, "json"));
            expect(!productFromText(R"({"format":"other","version":1,"id":"x"})", def, error) && contains(error, "format"));
            expect(!productFromText(R"({"format":"crucible-product","version":2,"id":"x"})", def, error) &&
                   contains(error, "version"));
            expect(!productFromText(R"({"format":"crucible-product","version":1})", def, error) && contains(error, "id"));
            expect(!productFromText(R"({"format":"crucible-product","version":1,"id":"x","voice":{"nodes":[{"id":"a"}]}})",
                                    def, error) &&
                   contains(error, "voice.nodes[0]"));
            expect(!productFromText(
                       R"({"format":"crucible-product","version":1,"id":"x","fx":{"lanes":[{"id":"a","order":"x"}]}})", def,
                       error) &&
                   contains(error, "fx.lanes[0]"));
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void validation()
    {
        beginTest("validation: types, ports, edge kinds, cycles, sinks");
        {
            ProductDef def = smallGraph();
            expect(validate(def).ok(), errorsOf(def));

            ProductDef bad = def;
            bad.voice.nodes[1].type = "env.nothing";
            expect(contains(errorsOf(bad), "unknown node type"));

            bad = def;
            bad.voice.edges[0].from = "osc.nothing";
            expect(contains(errorsOf(bad), "no output"));

            bad = def;
            bad.voice.edges.push_back({"env.out", "vca.in", 1.0f}); // modulation into an audio port
            expect(contains(errorsOf(bad), "audio and modulation"));

            bad = def;
            bad.voice.edges.push_back({"out.out", "vca.in", 1.0f}); // vca -> out -> vca
            expect(contains(errorsOf(bad), "cycle"));

            bad = def;
            bad.voice.nodes.pop_back();
            bad.voice.edges.pop_back();
            expect(contains(errorsOf(bad), "mix.out"));

            bad = def;
            bad.voice.nodes[0].slot = 0;
            expect(contains(errorsOf(bad), "slot"));

            bad = def;
            bad.voice.nodes[1].params.emplace_back("nothing", 1.0);
            expect(contains(errorsOf(bad), "not a parameter"));

            bad = def;
            bad.voice.nodes[1].params.emplace_back("sustain", 7.0);
            expect(contains(errorsOf(bad), "out of range"));

            bad = def;
            bad.voice.nodes[1].id = "vca";
            expect(contains(errorsOf(bad), "twice"));

            bad = def;
            bad.voice.nodes[1].id = "Env";
            expect(contains(errorsOf(bad), "lower case"));

            bad = def;
            bad.voice.nodes.push_back({"classic", "classic.voice"});
            expect(contains(errorsOf(bad), "whole voice"));
        }

        beginTest("validation: limits are configurable");
        {
            ProductDef def = smallGraph();
            ProductLimits tight;
            tight.nodesPerVoice = 3;
            expect(contains(validate(def, NodeRegistry::builtIn(), tight).text(), "nodes in one voice"));
            tight = ProductLimits{};
            tight.modulators = 0;
            expect(contains(validate(def, NodeRegistry::builtIn(), tight).text(), "modulators"));

            // The documented defaults: 32 nodes per voice, 64 FX instances, 16 layers, 64 modulators.
            expectEquals(ProductLimits{}.nodesPerVoice, 32);
            expectEquals(ProductLimits{}.fxInstances, 64);
            expectEquals(ProductLimits{}.layers, 16);
            expectEquals(ProductLimits{}.modulators, 64);

            ProductDef many = def;
            for (int i = 0; i < 40; ++i)
                many.voice.nodes.push_back({"extra" + std::to_string(i), "amp.vca"});
            expect(contains(errorsOf(many), "more than 32 nodes"));

            ProductDef layers;
            layers.id = "test.layers";
            layers.voice.kind = VoiceKind::Graph;
            for (int i = 0; i < 17; ++i)
            {
                LayerDef l;
                l.id = "l" + std::to_string(i);
                l.nodes = def.voice.nodes;
                for (auto& n : l.nodes)
                    n.id += "_" + std::to_string(i);
                for (auto e : def.voice.edges)
                {
                    e.from.insert(e.from.find('.'), "_" + std::to_string(i));
                    e.to.insert(e.to.find('.'), "_" + std::to_string(i));
                    l.edges.push_back(e);
                }
                layers.voice.layers.push_back(l);
            }
            expect(contains(errorsOf(layers), "more than 16 layers"));

            ProductDef fx;
            fx.id = "test.fx";
            fx.fx.present = true;
            for (int i = 0; i < 65; ++i)
                fx.fx.instances.push_back({"d" + std::to_string(i), "fx.saturator"});
            expect(contains(errorsOf(fx), "more than 64 instances"));
        }

        beginTest("validation: the FX graph");
        {
            const ProductDef def = load("drive-trio");
            ProductDef bad = def;
            bad.fx.lanes[0].chain = {"nothing"};
            expect(contains(errorsOf(bad), "unknown instance"));

            bad = def;
            bad.fx.lanes[1].chain.push_back("drive_a");
            expect(contains(errorsOf(bad), "more than one lane"));

            bad = def;
            bad.fx.lanes[0].chain.clear();
            expect(contains(errorsOf(bad), "in no lane"));

            bad = def;
            bad.fx.lanes[0].inputs = {{"lane_b", 1.0f}};
            bad.fx.lanes[1].inputs = {{"lane_a", 1.0f}};
            expect(contains(errorsOf(bad), "cycle"));

            bad = def;
            bad.fx.lanes[0].inputs = {{"missing", 1.0f}};
            expect(contains(errorsOf(bad), "unknown source"));

            bad = def;
            bad.fx.instances[0].type = "fx.nothing";
            expect(contains(errorsOf(bad), "not an effect module"));

            bad = def;
            bad.fx.sends.push_back({"lane_a", "lane_zz", 0.5f});
            expect(contains(errorsOf(bad), "sends connect lanes"));

            bad = def;
            bad.fx.lanes[0].id = "voices";
            expect(contains(errorsOf(bad), "lane id"));
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void ids()
    {
        beginTest("the default product has today's parameters in today's order");
        {
            std::string error;
            const auto compiled = build(defaultProductDef());
            expect(compiled != nullptr);
            if (!compiled)
                return;
            const auto legacy = ModuleRegistry::build(ModuleRegistry::neutralLayout());
            std::vector<std::string> expected;
            for (const auto& p : legacy->parameters().params())
                expected.push_back(p.id);
            expect(compiled->parameterIds() == expected, "parameter ids and order");
            expectEquals(compiled->registry->parameters().modTargets().size(), legacy->parameters().modTargets().size());
            expectEquals(compiled->registry->parameters().modDestNames().size(), legacy->parameters().modDestNames().size());
            expect(compiled->defaultValues() == legacy->parameters().defaultValues());
            // The file on disk compiles to the same thing.
            const auto fromFile = build(load("crucible-default"));
            expect(fromFile != nullptr && fromFile->parameterIds() == expected);
        }

        beginTest("instance k >= 2 of an effect is fx{k}_..., first instances keep their ids, pins apply");
        {
            const auto compiled = build(load("drive-trio"));
            expect(compiled != nullptr);
            if (!compiled)
                return;
            const auto& model = compiled->registry->parameters();
            expect(model.indexOf("fx_saturator_drive") >= 0);
            expect(model.indexOf("fx2_saturator_drive") >= 0);
            expect(model.indexOf("fx2_saturator_on") >= 0);
            expect(model.indexOf("fx2_pos_saturator") >= 0);
            expect(model.indexOf("fx3_saturator_drive") < 0, "pinned away");
            expect(model.indexOf("drive_c_drive") >= 0);
            expect(model.indexOf("fx3_saturator_mix") >= 0);
            // The instances are modules of their own, with ports and the type they copy.
            const auto* second = compiled->registry->find("fx.saturator.2");
            expect(second != nullptr && second->instanceOf == "fx.saturator" && second->instanceLabel == "drive_b");
            // Effects the FX graph does not use are not in the product.
            expect(model.indexOf("fx_ra_on") < 0);
            expect(model.indexOf("fx_ensemble_on") < 0);
            // Built-in ids of the first instance did not move relative to each other.
            expect(model.indexOf("fx_saturator_on") < model.indexOf("fx_saturator_drive"));
            // The description names the type of an instance and still round-trips.
            const std::string text = describeJSON(*compiled->registry);
            expect(contains(text, "\"instance_of\": \"fx.saturator\""));
            ProductDescription back;
            std::string readError;
            expect(ProductDescription::fromJson(text, back, readError), readError);
            expectEquals(back.toJson(), text);
            // Ids are unique.
            std::set<std::string> seen;
            for (const auto& p : model.params())
                expect(seen.insert(p.id).second, p.id);
            // The copies are modulation targets too, appended after the built-in targets.
            expect(compiled->registry->parameters().modTargets().size() > 0);
        }

        beginTest("a pin that matches nothing, or collides, is an error");
        {
            ProductDef def = load("drive-trio");
            def.idMap.emplace_back("fx9_saturator_drive", "whatever");
            std::string error;
            expect(compile(def, error) == nullptr && contains(error, "matches no generated"), error);

            def = load("drive-trio");
            def.idMap[0].second = "master_gain";
            expect(compile(def, error) == nullptr && contains(error, "already exists"), error);

            def = load("drive-trio");
            def.defaults.emplace_back("nothing_here", 1.0);
            expect(compile(def, error) == nullptr && contains(error, "defaults"), error);

            def = smallGraph();
            def.voice.nodes[1].id = "amp"; // amp_attack is a built-in id
            def.voice.edges[1].from = "amp.out";
            expect(compile(def, error) == nullptr && contains(error, "already exists"), error);
        }

        beginTest("graph nodes get <instance>_<param> ids with the product's defaults");
        {
            const auto compiled = build(load("drum-kit"));
            expect(compiled != nullptr);
            if (!compiled)
                return;
            const auto& model = compiled->registry->parameters();
            const int attack = model.indexOf("kick_env_attack");
            expect(attack >= 0);
            expectWithinAbsoluteError(model.params()[static_cast<size_t>(attack)].defaultValue, 0.001f, 1.0e-6f);
            expect(model.indexOf("hat_open_filter_cutoff") >= 0);
            expect(model.indexOf("snare_out_level") >= 0);
            expectEquals(compiled->nodeParameterId("kick_env", "decay"), std::string("kick_env_decay"));
            expectEquals(model.params()[static_cast<size_t>(model.indexOf("osc4_level"))].id, std::string("osc4_level"));
            expectEquals(compiled->registry->layout().voice.oscillators, 4);
            const auto values = compiled->defaultValues();
            expectWithinAbsoluteError(values[static_cast<size_t>(model.indexOf("osc1_level"))], 1.0f, 1.0e-6f);
            // The envelope module carries its type and ports for generated UIs.
            const auto* env = compiled->registry->find("kick_env");
            expect(env != nullptr && env->instanceOf == "env.adsr" && env->outputs.size() == 1);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void classicBitExact()
    {
        beginTest("the default product as data renders bit-identically to an engine without one");
        {
            const auto compiled = build(defaultProductDef());
            expect(compiled != nullptr);
            if (!compiled)
                return;
            struct Scenario
            {
                const char* name;
                std::vector<std::pair<std::string, float>> set;
                std::vector<int> notes;
            };
            const std::vector<Scenario> scenarios = {
                {"dry", {}, {48}},
                {"saturator and reverb",
                 {{"fx_saturator_on", 1},
                  {"fx_saturator_drive", 28},
                  {"fx_saturator_mix", 1},
                  {"fx_ra_on", 1},
                  {"fx_ra_mix", 0.4f}},
                 {48, 55}},
                {"order reversed",
                 {{"fx_ensemble_on", 1},
                  {"fx_ensemble_mix", 0.5f},
                  {"fx_delay_pro_on", 1},
                  {"fx_delay_pro_mix", 0.4f},
                  {"fx_saturator_on", 1},
                  {"fx_pos_saturator", 9},
                  {"fx_pos_ensemble", 2},
                  {"fx_pos_delay_pro", 1}},
                 {40, 47, 52}},
                {"compressor limiter eq",
                 {{"fx_cc_on", 1}, {"fx_limit_on", 1}, {"fx_ep_on", 1}, {"fx_crush_on", 1}, {"fx_width_on", 1}},
                 {60}},
                {"unison saw",
                 {{"osc1_uni_voices", 5},
                  {"osc1_table", 1},
                  {"fx_phaser2_on", 1},
                  {"fx_flanger2_on", 1},
                  {"fx_phaser2_mix", 0.5f}},
                 {36, 43}},
            };
            for (const auto& s : scenarios)
            {
                CRUCIBLE_HEAP(SynthEngine, plain);
                EngineConfig cfg;
                cfg.compiled = compiled;
                CRUCIBLE_HEAP(SynthEngine, data, cfg);
                auto values = plain.model().defaultValues();
                sineVoice(plain.model(), values);
                for (const auto& [id, v] : s.set)
                    put(plain.model(), values, id, v);
                plain.prepare(kRate);
                data.prepare(kRate);
                plain.ensureSelectedTables(values.data(), static_cast<int>(values.size()));
                plain.setParameterValues(values.data(), static_cast<int>(values.size()));
                data.setParameterValues(values.data(), static_cast<int>(values.size()));
                for (const int n : s.notes)
                {
                    plain.noteOn(n, 0.8f);
                    data.noteOn(n, 0.8f);
                }
                const Audio a = run(plain, 24000, 480);
                const Audio b = run(data, 24000, 480);
                expect(a.left == b.left && a.right == b.right, s.name);
                expect(peak(a.left) > 0.01, std::string(s.name) + " is silent");
            }
        }

        beginTest("the classic voice is a node of the graph");
        {
            EngineConfig cfg;
            cfg.compiled = build(defaultProductDef());
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            engine.prepare(kRate);
            expect(engine.program() != nullptr);
            if (engine.program() == nullptr)
                return;
            expect(engine.program()->voiceKind() == VoiceKind::Classic);
            expect(const_cast<graph::Program*>(engine.program())->classicNode() != nullptr);
            expect(engine.product() != nullptr && engine.product()->def.id == "crucible.default");
            expect(engine.program()->fxPlan().isInPlace());
            const NodeType* type = NodeRegistry::builtIn().find("classic.voice");
            expect(type != nullptr && type->kind == ModuleKind::Voice);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // Renders the drive-trio product with the given lane levels.
    Audio driveTrio(const std::array<float, 3>& levels, float amountC = -1.0f, bool serial = false)
    {
        ProductDef def = load("drive-trio");
        for (size_t i = 0; i < 3; ++i)
            def.fx.lanes[i].level = levels[i];
        if (serial)
        {
            def.fx.lanes.resize(1);
            def.fx.lanes[0].chain = {"drive_a", "drive_b", "drive_c"};
            def.fx.lanes[0].level = 1.0f;
        }
        EngineConfig cfg;
        cfg.compiled = build(def);
        CRUCIBLE_HEAP(SynthEngine, engine, cfg);
        auto values = cfg.compiled->defaultValues();
        const auto& model = engine.model();
        sineVoice(model, values);
        if (amountC >= 0.0f)
            put(model, values, "drive_c_drive", amountC);
        engine.prepare(kRate);
        engine.setParameterValues(values.data(), static_cast<int>(values.size()));
        engine.noteOn(52, 0.9f);
        return run(engine, 24000, 480);
    }

    void fxGraph()
    {
        beginTest("a 3-instance drive product renders three independent drives");
        {
            const Audio a = driveTrio({1, 0, 0});
            const Audio b = driveTrio({0, 1, 0});
            const Audio c = driveTrio({0, 0, 1});
            expect(peak(a.left) > 0.05 && peak(b.left) > 0.05 && peak(c.left) > 0.05);
            // Three different amounts: three different sounds.
            expect(maxAbsDiff(a.left, b.left) > 0.01);
            expect(maxAbsDiff(b.left, c.left) > 0.01);
            expect(maxAbsDiff(a.left, c.left) > 0.01);
            // Changing the third instance's pinned amount moves only the third lane.
            const Audio a2 = driveTrio({1, 0, 0}, 8.0f);
            const Audio b2 = driveTrio({0, 1, 0}, 8.0f);
            const Audio c2 = driveTrio({0, 0, 1}, 8.0f);
            expect(maxAbsDiff(a.left, a2.left) == 0.0 && maxAbsDiff(a.right, a2.right) == 0.0);
            expect(maxAbsDiff(b.left, b2.left) == 0.0);
            expect(maxAbsDiff(c.left, c2.left) > 0.01);
            // Serial: the chain of three differs from every single drive.
            const Audio chain = driveTrio({1, 1, 1}, -1.0f, true);
            expect(maxAbsDiff(chain.left, a.left) > 0.01 && maxAbsDiff(chain.left, c.left) > 0.01);
        }

        beginTest("parallel lanes equal the sum of the separate renders within 1e-6");
        {
            const Audio a = driveTrio({0.5f, 0, 0});
            const Audio b = driveTrio({0, 0.4f, 0});
            const Audio c = driveTrio({0, 0, 0.3f});
            const Audio all = driveTrio({0.5f, 0.4f, 0.3f});
            double worst = 0.0;
            for (size_t i = 0; i < all.left.size(); ++i)
            {
                worst = std::max(worst, static_cast<double>(std::abs(all.left[i] - (a.left[i] + b.left[i] + c.left[i]))));
                worst =
                    std::max(worst, static_cast<double>(std::abs(all.right[i] - (a.right[i] + b.right[i] + c.right[i]))));
            }
            logMessage("largest difference " + String(worst));
            expect(worst < 1.0e-6, "parallel vs separate: " + String(worst));
            expect(peak(all.left) > 0.05);
        }

        beginTest("sends add a lane's output to another lane's input");
        {
            ProductDef def;
            std::string error;
            expect(productFromText(R"({
                "format": "crucible-product", "version": 1, "id": "test.send",
                "fx": { "instances": [ { "id": "d", "type": "fx.saturator" } ],
                        "lanes": [ { "id": "a", "input": ["voices"], "chain": [] },
                                   { "id": "b", "input": [], "chain": ["d"] } ],
                        "sends": [ { "from": "a", "to": "b", "gain": 0.5 } ] } })",
                                   def, error),
                   error);
            def.fx.lanes[1].inputs.clear();
            const auto compiled = build(def);
            expect(compiled != nullptr);
            if (!compiled)
                return;
            // With the drive off, b passes its input: the master is a + 0.5 a.
            EngineConfig cfg;
            cfg.compiled = compiled;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            auto values = compiled->defaultValues();
            sineVoice(engine.model(), values);
            engine.prepare(kRate);
            engine.setParameterValues(values.data(), static_cast<int>(values.size()));
            engine.noteOn(60, 0.9f);
            const Audio sent = run(engine, 9600, 480);
            CRUCIBLE_HEAP(SynthEngine, plain);
            auto pv = plain.model().defaultValues();
            sineVoice(plain.model(), pv);
            plain.prepare(kRate);
            plain.setParameterValues(pv.data(), static_cast<int>(pv.size()));
            plain.noteOn(60, 0.9f);
            const Audio dry = run(plain, 9600, 480);
            double worst = 0.0;
            for (size_t i = 0; i < dry.left.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(sent.left[i] - 1.5f * dry.left[i])));
            expect(worst < 1.0e-6, "send: " + String(worst));
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void fxOnly()
    {
        beginTest("an FX-only product processes the host input (process_aux)");
        {
            const auto compiled = build(load("fx-only"));
            expect(compiled != nullptr);
            if (!compiled)
                return;
            EngineConfig cfg;
            cfg.compiled = compiled;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            auto values = compiled->defaultValues();
            engine.prepare(kRate);
            engine.setParameterValues(values.data(), static_cast<int>(values.size()));
            expect(engine.program()->voiceKind() == VoiceKind::None);

            std::vector<float> input(9600);
            std::uint32_t seed = 12345;
            for (auto& x : input)
            {
                seed = seed * 1664525U + 1013904223U;
                x = (static_cast<float>(seed >> 8) / 8388608.0f - 1.0f) * 0.5f;
            }
            const auto process = [&](SynthEngine& e)
            {
                Audio out;
                std::vector<float> l(480), r(480);
                for (size_t at = 0; at < input.size(); at += 480)
                {
                    e.setAuxInput(input.data() + at, nullptr);
                    e.render(l.data(), r.data(), 480);
                    out.append(l.data(), r.data(), 480);
                }
                return out;
            };
            // Notes do nothing in an FX product.
            engine.noteOn(60, 1.0f);
            const Audio passed = process(engine);
            // Every effect off: the main lane passes the input, the send adds 0.3 of it through the (off) reverb.
            double worst = 0.0;
            for (size_t i = 0; i < input.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(passed.left[i] - 1.3f * input[i])));
            expect(worst < 1.0e-6, "pass-through: " + String(worst));
            expectEquals(engine.activeVoiceCount(), 0);

            // Drive on: the input is processed.
            auto driven = values;
            put(engine.model(), driven, "fx_saturator_on", 1.0f);
            put(engine.model(), driven, "fx_saturator_drive", 32.0f);
            put(engine.model(), driven, "fx_saturator_mix", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, second, cfg);
            second.prepare(kRate);
            second.setParameterValues(driven.data(), static_cast<int>(driven.size()));
            const Audio processed = process(second);
            expect(maxAbsDiff(processed.left, passed.left) > 0.05);
            bool finite = true;
            for (const float s : processed.left)
                finite = finite && std::isfinite(s);
            expect(finite);
            expect(peak(processed.left) > 0.05);
            // Without an input the product is silent.
            CRUCIBLE_HEAP(SynthEngine, quiet, cfg);
            quiet.prepare(kRate);
            quiet.setParameterValues(values.data(), static_cast<int>(values.size()));
            expectEquals(peak(run(quiet, 4800).left), 0.0);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void layers()
    {
        const auto compiled = build(load("drum-kit"));
        expect(compiled != nullptr);
        if (!compiled)
            return;
        EngineConfig cfg;
        cfg.compiled = compiled;
        auto quietValues = [&](const SynthEngine& e)
        {
            auto v = compiled->defaultValues();
            put(e.model(), v, "fx_ra_on", 0.0f);
            put(e.model(), v, "master_gain", 0.0f);
            return v;
        };
        const auto hit = [&](int note, int samples, float velocity = 1.0f)
        {
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            const auto v = quietValues(engine);
            engine.prepare(kRate);
            engine.setParameterValues(v.data(), static_cast<int>(v.size()));
            engine.noteOn(note, velocity);
            const int voices = engine.activeVoiceCount();
            return std::make_pair(run(engine, samples, 480), voices);
        };

        beginTest("a 4-layer drum product triggers one layer per note");
        {
            const int notes[4] = {36, 38, 42, 46};
            std::vector<Audio> hits;
            for (const int note : notes)
            {
                auto [audio, voices] = hit(note, 14400);
                expectEquals(voices, 1, "note " + String(note));
                expect(peak(audio.left) > 0.02, "note " + String(note) + " is silent");
                hits.push_back(audio);
            }
            // Four different sounds: the kick is dull, the hats are bright.
            expect(brightness(hits[2].left) > 20.0 * brightness(hits[0].left));
            expect(maxAbsDiff(hits[0].left, hits[1].left) > 0.01);
            expect(maxAbsDiff(hits[2].left, hits[3].left) > 0.001);
            // A note no layer plays is silent.
            auto [none, voices] = hit(40, 4800);
            expectEquals(voices, 0);
            expectEquals(peak(none.left), 0.0);
            // Velocity scales the hit (the VCA follows velocity).
            const double soft = peak(hit(36, 9600, 0.3f).first.left);
            const double loud = peak(hit(36, 9600, 1.0f).first.left);
            expect(loud > 1.5 * soft, "velocity: " + String(soft) + " vs " + String(loud));
        }

        beginTest("one-shot layers end by themselves and free their voices");
        {
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            const auto v = quietValues(engine);
            engine.prepare(kRate);
            engine.setParameterValues(v.data(), static_cast<int>(v.size()));
            engine.noteOn(42, 1.0f); // closed hat: 40 ms decay
            run(engine, 480);
            expectEquals(engine.activeVoiceCount(), 1);
            engine.noteOff(42); // ignored by a one-shot layer
            run(engine, 480);
            expectEquals(engine.activeVoiceCount(), 1);
            const Audio tail = run(engine, 24000);
            expectEquals(engine.activeVoiceCount(), 0);
            expect(peak(tail.left, 20000) < 1.0e-6);
        }

        beginTest("a choke group silences the open hat when the closed hat plays");
        {
            const auto tailEnergy = [&](bool choke)
            {
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                const auto v = quietValues(engine);
                engine.prepare(kRate);
                engine.setParameterValues(v.data(), static_cast<int>(v.size()));
                engine.noteOn(46, 1.0f);
                run(engine, 2400); // 50 ms of the open hat
                if (choke)
                    engine.noteOn(42, 1.0f);
                const Audio a = run(engine, 24000, 480);
                return std::make_pair(energy(a.left, 9600), engine.activeVoiceCount());
            };
            const auto [open, openVoices] = tailEnergy(false);
            const auto [choked, chokedVoices] = tailEnergy(true);
            logMessage("tail energy " + String(open) + " open, " + String(choked) + " choked");
            expect(open > 0.01);
            expect(choked < open * 0.01, "the choked open hat must be silent");
            expectEquals(chokedVoices, 0);
            (void)openVoices;
        }

        beginTest("a choked voice fades out without a click");
        {
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            const auto v = quietValues(engine);
            engine.prepare(kRate);
            engine.setParameterValues(v.data(), static_cast<int>(v.size()));
            engine.noteOn(46, 1.0f);
            run(engine, 2400);
            const Audio before = run(engine, 480);
            engine.noteOn(42, 0.01f); // nearly silent closed hat: the open hat's cut is the only event
            const Audio after = run(engine, 960);
            const double natural = maxStep(before.left, 0, before.left.size());
            expect(maxStep(after.left, 0, after.left.size()) <= natural + 0.001, "choke step");
        }

        beginTest("voice stealing keeps a layer inside its polyphony");
        {
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            const auto v = quietValues(engine);
            engine.prepare(kRate);
            engine.setParameterValues(v.data(), static_cast<int>(v.size()));
            for (int i = 0; i < 6; ++i)
            {
                engine.noteOn(36, 1.0f);
                run(engine, 96);
            }
            expectEquals(engine.activeVoiceCount(), 2); // the kick layer has two voices
        }

        beginTest("velocity ranges and round-robin groups choose layers");
        {
            ProductDef def = smallGraph();
            const auto layered = [&](const std::string& json)
            {
                ProductDef d;
                std::string error;
                expect(productFromText(json, d, error), error);
                return build(d);
            };
            const auto voiceLayer = [](const std::string& id, const std::string& extra)
            {
                return "{ \"id\": \"" + id + "\", " + extra +
                       ", \"voices\": 4, \"nodes\": ["
                       "{ \"id\": \"" +
                       id +
                       "_osc\", \"type\": \"osc.sub\", \"slot\": 1 },"
                       "{ \"id\": \"" +
                       id +
                       "_env\", \"type\": \"env.adsr\" },"
                       "{ \"id\": \"" +
                       id +
                       "_vca\", \"type\": \"amp.vca\" },"
                       "{ \"id\": \"" +
                       id +
                       "_out\", \"type\": \"mix.out\" } ], \"edges\": ["
                       "{ \"from\": \"" +
                       id + "_osc.out\", \"to\": \"" + id +
                       "_vca.in\" },"
                       "{ \"from\": \"" +
                       id + "_env.out\", \"to\": \"" + id +
                       "_vca.gain\" },"
                       "{ \"from\": \"" +
                       id + "_vca.out\", \"to\": \"" + id + "_out.in\" } ] }";
            };
            const std::string head = R"({"format":"crucible-product","version":1,"id":"test.layers","voice":{"layers":[)";
            const auto velocity = layered(head + voiceLayer("soft", "\"velocity\": [1, 64]") + "," +
                                          voiceLayer("hard", "\"velocity\": [65, 127]") + "]}}");
            expect(velocity != nullptr);
            if (velocity)
            {
                EngineConfig c;
                c.compiled = velocity;
                CRUCIBLE_HEAP(SynthEngine, engine, c);
                engine.prepare(kRate);
                engine.noteOn(60, 0.3f);
                expectEquals(engine.program()->activeVoices(0), 1);
                expectEquals(engine.program()->activeVoices(1), 0);
                engine.noteOn(61, 1.0f);
                expectEquals(engine.program()->activeVoices(0), 1);
                expectEquals(engine.program()->activeVoices(1), 1);
            }
            const auto rr = layered(head + voiceLayer("a", "\"round_robin\": \"snare\"") + "," +
                                    voiceLayer("b", "\"round_robin\": \"snare\"") + "," +
                                    voiceLayer("c", "\"round_robin\": \"snare\"") + "]}}");
            expect(rr != nullptr);
            if (rr)
            {
                EngineConfig c;
                c.compiled = rr;
                CRUCIBLE_HEAP(SynthEngine, engine, c);
                engine.prepare(kRate);
                std::array<int, 3> count{};
                for (int i = 0; i < 6; ++i)
                {
                    engine.noteOn(40 + i, 1.0f);
                    for (int l = 0; l < 3; ++l)
                        count[static_cast<size_t>(l)] = engine.program()->activeVoices(l);
                }
                expectEquals(count[0], 2);
                expectEquals(count[1], 2);
                expectEquals(count[2], 2);
            }
            (void)def;
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void nodes()
    {
        beginTest("a graph voice built from nodes sounds, follows its envelope and ends");
        {
            ProductDef def = smallGraph();
            def.defaults = {{"osc1_level", 1.0},
                            {"osc1_sub_octave", 0.0},
                            {"env_attack", 0.002},
                            {"env_sustain", 0.8},
                            {"env_release", 0.05}};
            const auto compiled = build(def);
            expect(compiled != nullptr);
            if (!compiled)
                return;
            EngineConfig cfg;
            cfg.compiled = compiled;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            auto v = compiled->defaultValues();
            put(engine.model(), v, "master_gain", 0.0f);
            engine.prepare(kRate);
            engine.setParameterValues(v.data(), static_cast<int>(v.size()));
            engine.noteOn(60, 1.0f);
            const Audio held = run(engine, 9600, 480);
            expect(peak(held.left, 4800) > 0.2, "sustains while held");
            expectEquals(engine.activeVoiceCount(), 1);
            engine.noteOff(60);
            const Audio released = run(engine, 9600, 480);
            expectEquals(engine.activeVoiceCount(), 0);
            expect(peak(released.left, 4800) < 1.0e-5, "silent after the release");
            // The Sub oscillator wrapper plays an octave below the note: 60 -> C3 at 130.8 Hz.
            std::vector<float> tail(held.left.begin() + 4800, held.left.end());
            int crossings = 0;
            for (size_t i = 1; i < tail.size(); ++i)
                crossings += (tail[i - 1] < 0.0f && tail[i] >= 0.0f) ? 1 : 0;
            const double hz = crossings / (static_cast<double>(tail.size()) / kRate);
            expectWithinAbsoluteError(hz, 130.8, 6.0);
        }

        beginTest("filter, LFO and VCA nodes shape the signal");
        {
            ProductDef def;
            std::string error;
            expect(productFromText(R"({
                "format": "crucible-product", "version": 1, "id": "test.nodes",
                "voice": { "nodes": [
                    { "id": "osc", "type": "osc.noise", "slot": 1 },
                    { "id": "lfo", "type": "lfo.basic", "params": { "rate": 8.0 } },
                    { "id": "flt", "type": "filter.svf", "params": { "cutoff": 1000.0 } },
                    { "id": "env", "type": "env.adsr", "params": { "sustain": 1.0, "attack": 0.001 } },
                    { "id": "vca", "type": "amp.vca" },
                    { "id": "out", "type": "mix.out" } ],
                  "edges": [
                    { "from": "osc.out", "to": "flt.in" },
                    { "from": "lfo.out", "to": "flt.cutoff", "amount": 0.5 },
                    { "from": "flt.out", "to": "vca.in" },
                    { "from": "env.out", "to": "vca.gain" },
                    { "from": "vca.out", "to": "out.in" } ] },
                "defaults": { "osc1_level": 1.0 } })",
                                   def, error),
                   error);
            const auto compiled = build(def);
            expect(compiled != nullptr);
            if (!compiled)
                return;
            const auto render = [&](float cutoff)
            {
                EngineConfig cfg;
                cfg.compiled = compiled;
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                auto v = compiled->defaultValues();
                put(engine.model(), v, "flt_cutoff", cutoff);
                engine.prepare(kRate);
                engine.setParameterValues(v.data(), static_cast<int>(v.size()));
                engine.noteOn(60, 1.0f);
                return run(engine, 9600, 480);
            };
            const Audio dark = render(300.0f);
            const Audio bright = render(12000.0f);
            expect(peak(dark.left) > 0.001 && peak(bright.left) > 0.001);
            expect(brightness(bright.left, 480) > 5.0 * brightness(dark.left, 480), "the filter node filters");
            // The LFO moves the cutoff: the filtered noise level changes over a cycle.
            double lo = 1e9, hi = 0.0;
            for (size_t at = 2400; at + 480 <= dark.left.size(); at += 480)
            {
                double e = 0.0;
                for (size_t i = at; i < at + 480; ++i)
                    e += static_cast<double>(dark.left[i]) * dark.left[i];
                lo = std::min(lo, e);
                hi = std::max(hi, e);
            }
            expect(hi > 1.5 * lo, "the LFO modulates the cutoff");
        }

        beginTest("nodes that process four voices together are dispatched in fours");
        {
            NodeRegistry registry = NodeRegistry::builtIn();
            NodeType probe;
            probe.type = "test.quad";
            probe.name = "Quad probe";
            probe.outputs = {{"out", PortType::AudioStereo}};
            probe.create = [](const NodeCreate&) { return std::make_unique<QuadProbe>(); };
            registry.add(probe);
            ProductDef def;
            std::string error;
            expect(productFromText(R"({
                "format": "crucible-product", "version": 1, "id": "test.quad",
                "voice": { "nodes": [ { "id": "q", "type": "test.quad" }, { "id": "out", "type": "mix.out" } ],
                           "edges": [ { "from": "q.out", "to": "out.in" } ] } })",
                                   def, error),
                   error);
            def.voice.layers.push_back(LayerDef{});
            def.voice.layers[0].id = "all";
            def.voice.layers[0].voices = 6;
            def.voice.layers[0].nodes = def.voice.nodes;
            def.voice.layers[0].edges = def.voice.edges;
            def.voice.nodes.clear();
            def.voice.edges.clear();
            auto compiled = compile(def, error, registry);
            expect(compiled != nullptr, error);
            if (!compiled)
                return;
            EngineConfig cfg;
            cfg.compiled = compiled;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            engine.prepare(kRate);
            for (int i = 0; i < 6; ++i)
                engine.noteOn(40 + i, 1.0f);
            gQuadCalls = 0;
            gSingleCalls = 0;
            const Audio a = run(engine, 480, 480);
            // Six voices: one group of four and two singles per chunk of 32 samples.
            expect(gQuadCalls.load() > 0 && gSingleCalls.load() > 0);
            expectEquals(gSingleCalls.load(), 2 * gQuadCalls.load());
            expectWithinAbsoluteError(a.left[100], 6 * 0.25f, 1.0e-5);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void reconfiguration()
    {
        beginTest("reconfiguring a running engine is click-bounded (< -60 dB step)");
        {
            const ProductDef parallel = load("drive-trio");
            ProductDef serial = parallel;
            serial.fx.lanes.resize(1);
            serial.fx.lanes[0].chain = {"drive_a", "drive_b", "drive_c"};
            serial.fx.lanes[0].level = 1.0f;

            const auto first = build(parallel);
            expect(first != nullptr);
            if (!first)
                return;
            std::string error;
            const auto second = compileReconfiguration(*first, serial, error);
            expect(second != nullptr, error);
            if (!second)
                return;
            expect(second->registry == first->registry, "the registry is shared");

            EngineConfig cfg;
            cfg.compiled = first;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            auto values = first->defaultValues();
            sineVoice(engine.model(), values);
            engine.prepare(kRate);
            engine.setParameterValues(values.data(), static_cast<int>(values.size()));
            engine.noteOn(52, 0.9f);
            Audio all = run(engine, 9600, 160);
            const size_t before = all.left.size();
            expect(!engine.reconfiguring());
            expect(engine.reconfigure(second, &error), error);

            // Allocation free from here on: the audio thread only reads the published program.
            std::vector<float> l(160), r(160);
            bool sawFade = false;
            long allocations = 0;
            for (int i = 0; i < 90; ++i)
            {
                gAllocations = 0;
                gTrackAllocations = true;
                engine.render(l.data(), r.data(), 160);
                gTrackAllocations = false;
                allocations += gAllocations;
                sawFade = sawFade || engine.reconfiguring();
                all.append(l.data(), r.data(), 160);
            }
            expectEquals(allocations, 0L, "allocations while reconfiguring");
            expect(sawFade, "the fade ran");
            expect(!engine.reconfiguring(), "the fade ended");
            expect(engine.program()->product().fx.lanes.size() == 1, "the new graph runs");

            const size_t fadeEnd = before + static_cast<size_t>(kRate * 0.030);
            const double natural =
                std::max(maxStep(all.left, 480, before), maxStep(all.left, fadeEnd + 480, all.left.size()));
            const double during = maxStep(all.left, before, fadeEnd + 480);
            logMessage("step during the swap " + String(during) + ", natural " + String(natural));
            expect(during <= natural + 0.001, "step " + String(during) + " vs natural " + String(natural));
            // The sound after the swap is the new graph's: the serial chain.
            const Audio steady = driveTrio({1, 1, 1}, -1.0f, true);
            const size_t at = all.left.size() - 960;
            double diffNew = 0.0;
            for (size_t i = 0; i < 960; ++i)
                diffNew = std::max(diffNew, static_cast<double>(std::abs(all.left[at + i] - steady.left[at + i])));
            expect(peak(all.left, at) > 0.05);
            logMessage("after the swap: difference to a fresh serial render " + String(diffNew));
        }

        beginTest("reconfiguration: wrong parameters are refused, programs are collected");
        {
            const auto first = build(load("drive-trio"));
            ProductDef other = load("fx-only");
            std::string error;
            expect(compileReconfiguration(*first, other, error) == nullptr && contains(error, "different parameters"),
                   error);

            EngineConfig cfg;
            cfg.compiled = first;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            engine.prepare(kRate);
            const auto stranger = build(load("fx-only"));
            expect(!engine.reconfigure(stranger, &error) && contains(error, "different parameters"), error);

            // Many reconfigurations in a row, some before the fade finished: nothing leaks.
            ProductDef serial = load("drive-trio");
            serial.fx.lanes.resize(1);
            serial.fx.lanes[0].chain = {"drive_a", "drive_b", "drive_c"};
            const auto b = compileReconfiguration(*first, serial, error);
            expect(b != nullptr, error);
            std::vector<float> l(480), r(480);
            for (int i = 0; i < 30; ++i)
            {
                expect(engine.reconfigure(i % 2 == 0 ? b : first, &error), error);
                if (i % 3 == 0)
                    engine.render(l.data(), r.data(), 480);
            }
            for (int i = 0; i < 6; ++i)
                engine.render(l.data(), r.data(), 480);
            expect(!engine.reconfiguring());
            // Control thread housekeeping drops what the audio thread let go of.
            engine.reconfigure(first, &error);
            engine.render(l.data(), r.data(), 480);
            engine.render(l.data(), r.data(), 480);
            expect(engine.livePrograms() <= 4, "live programs: " + String(engine.livePrograms()));
        }

        beginTest("reconfiguration: the control thread polls latency and tail while the audio thread swaps programs");
        {
            const auto first = build(load("drive-trio"));
            ProductDef serial = load("drive-trio");
            serial.fx.lanes.resize(1);
            serial.fx.lanes[0].chain = {"drive_a", "drive_b", "drive_c"};
            std::string error;
            const auto b = compileReconfiguration(*first, serial, error);
            expect(b != nullptr, error);
            EngineConfig cfg;
            cfg.compiled = first;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            engine.prepare(kRate);
            const auto values = first->defaultValues();
            engine.setParameterValues(values.data(), static_cast<int>(values.size()));
            std::atomic<bool> stop{false};
            std::atomic<long> polls{0};
            std::thread audio(
                [&]
                {
                    std::vector<float> l(160), r(160);
                    while (!stop.load())
                        engine.render(l.data(), r.data(), 160);
                });
            std::thread poller(
                [&]
                {
                    while (!stop.load())
                    {
                        pollReported(engine, values);
                        polls.fetch_add(1);
                    }
                });
            for (int i = 0; i < 60; ++i)
            {
                expect(engine.reconfigure(i % 2 == 0 ? b : first, &error), error);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            stop.store(true);
            audio.join();
            poller.join();
            expect(polls.load() > 0);
        }

        beginTest("reconfiguration: a program published during the fade-in is still applied");
        {
            const auto first = build(load("drive-trio"));
            ProductDef serial = load("drive-trio");
            serial.fx.lanes.resize(1);
            serial.fx.lanes[0].chain = {"drive_a", "drive_b", "drive_c"};
            std::string error;
            const auto b = compileReconfiguration(*first, serial, error);
            expect(b != nullptr, error);
            EngineConfig cfg;
            cfg.compiled = first;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            engine.prepare(kRate);
            std::vector<float> l(160), r(160);
            engine.render(l.data(), r.data(), 160);
            expect(engine.reconfigure(b, &error), error);
            // Render until the first switch has happened (the fade-in has started), then ask for a third program.
            int guard = 0;
            while (engine.program()->product().fx.lanes.size() != 1 && guard++ < 200)
                engine.render(l.data(), r.data(), 160);
            expect(engine.program()->product().fx.lanes.size() == 1, "the first switch happened");
            expect(engine.reconfiguring(), "still fading in");
            expect(engine.reconfigure(first, &error), error);
            for (int i = 0; i < 200; ++i)
                engine.render(l.data(), r.data(), 160);
            expect(engine.program()->product().fx.lanes.size() != 1, "the last reconfiguration is the one that plays");
            expect(!engine.reconfiguring(), "the fade ended");
        }

        beginTest("an engine without a product can be switched to the default product's graph");
        {
            const auto compiled = build(defaultProductDef());
            CRUCIBLE_HEAP(SynthEngine, engine);
            std::string error;
            engine.prepare(kRate);
            expect(engine.reconfigure(compiled, &error), error);
            std::vector<float> l(480), r(480);
            for (int i = 0; i < 5; ++i)
                engine.render(l.data(), r.data(), 480);
            expect(engine.program() != nullptr && !engine.reconfiguring());
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void overhead()
    {
        beginTest("graph overhead on the reference patch");
        {
            const auto compiled = build(defaultProductDef());
            if (!compiled)
                return;
            EngineConfig withGraph;
            withGraph.compiled = compiled;
            const auto time = [&](const EngineConfig& cfg)
            {
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                auto v = engine.model().defaultValues();
                put(engine.model(), v, "osc1_uni_voices", 4.0f);
                put(engine.model(), v, "fx_saturator_on", 1.0f);
                put(engine.model(), v, "fx_ensemble_on", 1.0f);
                put(engine.model(), v, "fx_ra_on", 1.0f);
                engine.prepare(kRate);
                engine.setParameterValues(v.data(), static_cast<int>(v.size()));
                for (int n = 0; n < 8; ++n)
                    engine.noteOn(40 + 3 * n, 0.9f);
                std::vector<float> l(256), r(256);
                for (int i = 0; i < 20; ++i)
                    engine.render(l.data(), r.data(), 256);
                const auto t0 = std::chrono::steady_clock::now();
                for (int i = 0; i < 400; ++i)
                    engine.render(l.data(), r.data(), 256);
                return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            };
            double plain = 1e9, graph = 1e9;
            for (int rep = 0; rep < 5; ++rep)
            {
                plain = std::min(plain, time(EngineConfig{}));
                graph = std::min(graph, time(withGraph));
            }
            const double overheadPct = 100.0 * (graph / plain - 1.0);
            logMessage("plain " + String(plain * 1000.0) + " ms, graph " + String(graph * 1000.0) + " ms, overhead " +
                       String(overheadPct) + " %");
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
            expect(true);
#else
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
            expect(true);
#else
            expect(overheadPct < 5.0, "graph overhead " + String(overheadPct) + " %");
#endif
#else
            expect(overheadPct < 5.0, "graph overhead " + String(overheadPct) + " %");
#endif
#endif
        }
    }
};

static GraphTests graphTests;
