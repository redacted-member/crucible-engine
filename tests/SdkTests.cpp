// SPDX-License-Identifier: MIT
// The module SDK: the C ABI (validation, version negotiation, registration, library loading), modules inside product
// graphs (FX graph and voice graph), the real-time harness behind crucible-module-check, the analog filter node, and
// the real-time contract of the built-in modules.

#include "AllocCounter.h"
#include "TestSupport.h"

#include <cstddef>
#include <crucible/Introspection.h>
#include <crucible/crucible.h>
#include <crucible/crucible_module.h>
#include <crucible/module/ParamBuilder.h>
#include <crucible/graph/Compiler.h>
#include <crucible/sdk/Check.h>
#include <crucible/sdk/Host.h>
#include <crucible/sdk/Module.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <map>
#include <set>
#include <cstring>

using namespace crucible;
using namespace crucible::testing;

namespace
{
std::string registryError(const crucible_module_registry* reg)
{
    std::string text(static_cast<size_t>(crucible_module_registry_error(reg, nullptr, 0)), '\0');
    crucible_module_registry_error(reg, text.data(), text.size() + 1);
    return text;
}
} // namespace

namespace
{
using crucible::testing::gAllocations;
using crucible::testing::gTrackAllocations;

volatile float gSink = 0.0f;
int gInstances = 0;

// ---- Modules written with the C++ layer, used in-process -------------------------------------------------------
sdk::Info effectInfo(const char* id, std::vector<sdk::Param> params)
{
    sdk::Info i;
    i.id = id;
    i.name = id;
    i.kind = sdk::Kind::Effect;
    i.params = std::move(params);
    return i;
}

class TestGain
{
public:
    static sdk::Info info() { return effectInfo("test.gain", {sdk::Param::floating("level", "Level", 0.0f, 4.0f, 1.0f)}); }
    void process(sdk::Process& p)
    {
        for (int i = 0; i < p.frames(); ++i)
        {
            p.left()[i] *= p.param(0);
            p.right()[i] *= p.param(0);
        }
    }
};

// Breaks the contract on purpose, one way per class.
class AllocatingModule
{
public:
    static sdk::Info info()
    {
        return effectInfo("test.allocates", {sdk::Param::floating("level", "Level", 0.0f, 1.0f, 0.5f)});
    }
    void process(sdk::Process&)
    {
        void* p = ::operator new(16);
        gSink = static_cast<float*>(p)[0] * 0.0f;
        ::operator delete(p);
    }
};

class NanModule
{
public:
    static sdk::Info info() { return effectInfo("test.nan", {sdk::Param::floating("level", "Level", 0.0f, 10.0f, 1.0f)}); }
    void process(sdk::Process& p)
    {
        const float bad = p.param(0) > 9.0f ? std::nanf("") : 1.0f;
        for (int i = 0; i < p.frames(); ++i)
        {
            p.left()[i] *= bad;
            p.right()[i] *= bad;
        }
    }
};

class LoudModule
{
public:
    static sdk::Info info() { return effectInfo("test.loud", {sdk::Param::floating("gain", "Gain", 0.0f, 1.0e6f, 1.0f)}); }
    void process(sdk::Process& p)
    {
        for (int i = 0; i < p.frames(); ++i)
        {
            p.left()[i] *= p.param(0);
            p.right()[i] *= p.param(0);
        }
    }
};

class DriftingModule
{
public:
    static sdk::Info info() { return effectInfo("test.drifts", {sdk::Param::floating("level", "Level", 0.0f, 1.0f, 0.5f)}); }
    DriftingModule() : offset(0.001f * static_cast<float>(++gInstances)) {}
    void process(sdk::Process& p)
    {
        for (int i = 0; i < p.frames(); ++i)
        {
            p.left()[i] += offset;
            p.right()[i] += offset;
        }
    }

private:
    float offset;
};

// Records the seed each instance is prepared with: nodes are prepared once per voice.
std::vector<std::uint32_t> gProbeSeeds;

class SeedProbe
{
public:
    static sdk::Info info()
    {
        sdk::Info i;
        i.id = "test.seedprobe";
        i.name = "Seed probe";
        i.kind = sdk::Kind::Generator;
        i.params = {sdk::Param::floating("level", "Level", 0.0f, 1.0f, 0.5f)};
        i.outputs = {{"out", sdk::PortKind::AudioStereo}};
        return i;
    }
    void prepare(const sdk::Prepare& p) { gProbeSeeds.push_back(p.seed); }
    void process(sdk::Process& p)
    {
        for (int i = 0; i < p.frames(); ++i)
            p.left()[i] = p.right()[i] = 0.0f;
    }
};

// ---- A descriptor written by hand, as a C author would -----------------------------------------------------------
struct RawModule
{
    crucible_param_spec param{sizeof(crucible_param_spec),
                              CRUCIBLE_MODULE_ABI_VERSION,
                              "level",
                              "Level",
                              nullptr,
                              nullptr,
                              CRUCIBLE_PARAM_FLOAT,
                              0,
                              0.0f,
                              1.0f,
                              0.5f,
                              0.0f,
                              1.0f,
                              0,
                              nullptr};
    const crucible_param_spec* table[1] = {&param};
    crucible_module_descriptor desc{};

    static void* create(const crucible_module_descriptor*, const crucible_module_host*) { return new int(0); }
    static void destroy(void* p) { delete static_cast<int*>(p); }
    static void process(void*, const crucible_module_process*) {}

    explicit RawModule(const char* id)
    {
        desc.size = sizeof(desc);
        desc.abi_version = CRUCIBLE_MODULE_ABI_VERSION;
        desc.id = id;
        desc.name = "Raw";
        desc.kind = CRUCIBLE_MODULE_KIND_EFFECT;
        desc.version = 1;
        desc.num_params = 1;
        desc.params = table;
        desc.create = &create;
        desc.destroy = &destroy;
        desc.process = &process;
    }
};

// ---- Helpers --------------------------------------------------------------------------------------------------
sdk::check::Options quickOptions()
{
    sdk::check::Options o;
    o.seconds = 0.25;
    o.corners = 4;
    o.enforceCpu = false;
    o.probe.track = [](bool on)
    {
        gTrackAllocations = on;
        if (on)
            gAllocations = 0;
    };
    o.probe.count = [] { return gAllocations; };
    return o;
}

const sdk::check::CheckResult* find(const sdk::check::Report& r, const std::string& name)
{
    for (const auto& c : r.checks)
        if (c.name == name)
            return &c;
    return nullptr;
}

std::string failedText(const sdk::check::Report& r)
{
    std::string text;
    for (const auto& c : r.checks)
        if (c.status == sdk::check::Status::Fail)
            text += " [" + c.name + ": " + c.detail + "]";
    return text;
}

struct Audio
{
    std::vector<float> left, right;
};

std::shared_ptr<const graph::CompiledProduct> build(const std::string& json, const graph::NodeRegistry& nodes,
                                                    std::string& error)
{
    graph::ProductDef def;
    if (!graph::productFromText(json, def, error))
        return nullptr;
    return graph::compile(def, error, nodes);
}

// Plays notes on an engine built from the product and returns the render; counts allocations after the first block.
Audio play(const std::shared_ptr<const graph::CompiledProduct>& product,
           const std::vector<std::pair<std::string, float>>& set, const std::vector<int>& notes, int samples,
           long* allocations = nullptr)
{
    EngineConfig c;
    c.compiled = product;
    SynthEngine engine(c);
    engine.prepare(kRate);
    auto values = product->defaultValues();
    for (const auto& [id, v] : set)
    {
        const int i = engine.model().indexOf(id);
        if (i >= 0)
            values[static_cast<size_t>(i)] = v;
    }
    engine.setParameterValues(values.data(), static_cast<int>(values.size()));
    for (const int n : notes)
        engine.noteOn(n, 0.8f);
    Audio a;
    std::vector<float> l(160), r(160);
    long counted = 0;
    for (int done = 0; done < samples; done += 160)
    {
        if (done > 0)
        {
            gAllocations = 0;
            gTrackAllocations = true;
        }
        engine.render(l.data(), r.data(), 160);
        gTrackAllocations = false;
        counted += gAllocations;
        a.left.insert(a.left.end(), l.begin(), l.end());
        a.right.insert(a.right.end(), r.begin(), r.end());
    }
    if (allocations != nullptr)
        *allocations = counted;
    return a;
}

double peakOf(const std::vector<float>& v, size_t from = 0)
{
    double p = 0.0;
    for (size_t i = from; i < v.size(); ++i)
        p = std::max(p, static_cast<double>(std::fabs(v[i])));
    return p;
}

bool finiteAll(const std::vector<float>& v)
{
    for (const float x : v)
        if (!std::isfinite(x))
            return false;
    return true;
}

double maxDiff(const std::vector<float>& a, const std::vector<float>& b)
{
    double d = 0.0;
    for (size_t i = 0; i < std::min(a.size(), b.size()); ++i)
        d = std::max(d, static_cast<double>(std::fabs(a[i] - b[i])));
    return a.size() == b.size() ? d : 1.0e9;
}

int risingCrossings(const std::vector<float>& v, size_t from, size_t to)
{
    int n = 0;
    for (size_t i = std::max<size_t>(from, 1); i < std::min(to, v.size()); ++i)
        if (v[i - 1] < 0.0f && v[i] >= 0.0f)
            ++n;
    return n;
}

bool contains(const std::string& text, const std::string& part)
{
    return text.find(part) != std::string::npos;
}

const char* kSaturatorFx = R"({
    "format": "crucible-product", "version": 1, "id": "test.sdkfx",
    "fx": { "instances": [ { "id": "sat", "type": "acme.c_saturator" }, { "id": "sat_b", "type": "acme.c_saturator" } ],
            "lanes": [ { "id": "main", "order": "listed", "input": ["voices"], "chain": ["sat", "sat_b"] } ] } })";

const char* kSdkVoice = R"({
    "format": "crucible-product", "version": 1, "id": "test.sdkvoice",
    "voice": { "nodes": [
        { "id": "osc", "type": "acme.cpp_sine", "params": { "level": 0.5 } },
        { "id": "lp", "type": "acme.cpp_onepole", "params": { "cutoff": 20000 } },
        { "id": "env", "type": "env.adsr", "params": { "attack": 0.001, "sustain": 1.0 } },
        { "id": "vca", "type": "amp.vca" },
        { "id": "out", "type": "mix.out" } ],
      "edges": [
        { "from": "osc.out", "to": "lp.in" },
        { "from": "lp.out", "to": "vca.in" },
        { "from": "env.out", "to": "vca.gain" },
        { "from": "vca.out", "to": "out.in" } ] } })";

std::string filterProduct(const char* node)
{
    return std::string(R"({
    "format": "crucible-product", "version": 1, "id": "test.filters",
    "voice": { "nodes": [
        { "id": "osc", "type": "osc.sub", "slot": 1 },
        { "id": "env", "type": "env.adsr", "params": { "attack": 0.001, "sustain": 1.0 } },
        )") +
           node + R"(,
        { "id": "vca", "type": "amp.vca" },
        { "id": "out", "type": "mix.out" } ],
      "edges": [
        { "from": "osc.out", "to": "flt.in" },
        { "from": "flt.out", "to": "vca.in" },
        { "from": "env.out", "to": "vca.gain" },
        { "from": "vca.out", "to": "out.in" } ] } })";
}
} // namespace

class SdkTests : public UnitTest
{
public:
    SdkTests() : UnitTest("Module SDK and C ABI", "Sdk") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        validation();
        registry();
#ifdef CRUCIBLE_TEST_C_MODULE
        libraries();
        fxGraph();
        voiceGraph();
        cApi();
        cApiExceptions();
        checkExamples();
#endif
        harness();
        analogFilterNode();
        builtIns();
    }

private:
    // ---------------------------------------------------------------------------------------------------------
    void validation()
    {
        beginTest("descriptors that break a rule are refused with a reason, not a crash");
        std::string error;
        RawModule ok("acme.fine");
        expectEquals(sdk::validateDescriptor(&ok.desc, error), CRUCIBLE_MODULE_OK, error);

        const auto code = [&](auto mutate)
        {
            RawModule m("acme.bad");
            mutate(m);
            error.clear();
            const int r = sdk::validateDescriptor(&m.desc, error);
            expect(r == CRUCIBLE_MODULE_OK || !error.empty(), "a refusal explains itself");
            return r;
        };
        expectEquals(sdk::validateDescriptor(nullptr, error), CRUCIBLE_MODULE_ERR_ARGUMENT);
        expectEquals(code([](RawModule& m) { m.desc.abi_version = CRUCIBLE_MODULE_ABI_VERSION + 1; }),
                     CRUCIBLE_MODULE_ERR_ABI_VERSION);
        expect(contains(error, "ABI version"), error);
        expectEquals(code([](RawModule& m) { m.desc.abi_version = 0; }), CRUCIBLE_MODULE_ERR_ABI_VERSION);
        expectEquals(code([](RawModule& m) { m.desc.size = 16; }), CRUCIBLE_MODULE_ERR_INVALID);
        expectEquals(code([](RawModule& m) { m.desc.id = "tremolo"; }), CRUCIBLE_MODULE_ERR_INVALID, "one segment");
        expectEquals(code([](RawModule& m) { m.desc.id = "Acme.Tremolo"; }), CRUCIBLE_MODULE_ERR_INVALID, "upper case");
        expectEquals(code([](RawModule& m) { m.desc.id = "fx.mine"; }), CRUCIBLE_MODULE_ERR_INVALID, "reserved vendor");
        expectEquals(code([](RawModule& m) { m.desc.id = "crucible.mine"; }), CRUCIBLE_MODULE_ERR_INVALID);
        expectEquals(code([](RawModule& m) { m.desc.process = nullptr; }), CRUCIBLE_MODULE_ERR_INVALID);
        expectEquals(code([](RawModule& m) { m.desc.kind = 99; }), CRUCIBLE_MODULE_ERR_INVALID);
        expectEquals(code([](RawModule& m) { m.desc.num_params = 65; }), CRUCIBLE_MODULE_ERR_INVALID);
        expectEquals(code([](RawModule& m) { m.param.default_value = 2.0f; }), CRUCIBLE_MODULE_ERR_INVALID,
                     "default out of range");
        expectEquals(code(
                         [](RawModule& m)
                         {
                             m.param.min = 1.0f;
                             m.param.max = 0.0f;
                             m.param.default_value = 0.5f;
                         }),
                     CRUCIBLE_MODULE_ERR_INVALID);
        expectEquals(code([](RawModule& m) { m.param.id = "Bad Id"; }), CRUCIBLE_MODULE_ERR_INVALID);
        expectEquals(code([](RawModule& m) { m.param.abi_version = 7; }), CRUCIBLE_MODULE_ERR_ABI_VERSION);
        expectEquals(code([](RawModule& m) { m.table[0] = nullptr; }), CRUCIBLE_MODULE_ERR_INVALID);
        expectEquals(code([](RawModule& m) { m.param.type = CRUCIBLE_PARAM_CHOICE; }), CRUCIBLE_MODULE_ERR_INVALID,
                     "no choices");
        crucible_port_spec port{sizeof(crucible_port_spec), CRUCIBLE_MODULE_ABI_VERSION, "in", CRUCIBLE_PORT_AUDIO_STEREO};
        const crucible_port_spec* ports[1] = {&port};
        expectEquals(code(
                         [&](RawModule& m)
                         {
                             m.desc.num_inputs = 1;
                             m.desc.inputs = ports;
                         }),
                     CRUCIBLE_MODULE_ERR_INVALID, "an effect declares no ports");
        expectEquals(code(
                         [&](RawModule& m)
                         {
                             m.desc.kind = CRUCIBLE_MODULE_KIND_FILTER;
                             m.desc.num_inputs = 1;
                             m.desc.inputs = ports;
                         }),
                     CRUCIBLE_MODULE_ERR_INVALID, "a node needs an output");

        beginTest("parameter ids may not collide: the bypass switch, a module's own, or another effect's");
        {
            sdk::ModuleSet set;
            RawModule onModule("acme.gate");
            onModule.param.id = "on"; // flattens to acme_gate_on, the host's bypass switch
            expectEquals(set.add(&onModule.desc), CRUCIBLE_MODULE_ERR_DUPLICATE);
            expect(contains(set.error(), "acme_gate_on"), set.error());
            expectEquals(set.count(), 0);

            RawModule a("acme.b");
            a.param.id = "c_d"; // acme_b_c_d
            RawModule b("acme.b_c");
            b.param.id = "d"; // acme_b_c_d as well
            expectEquals(set.add(&a.desc), CRUCIBLE_MODULE_OK, set.error());
            expectEquals(set.add(&b.desc), CRUCIBLE_MODULE_ERR_DUPLICATE);
            expect(contains(set.error(), "acme_b_c_d"), set.error());
            expectEquals(set.count(), 1);
            RawModule fine("acme.b_c");
            fine.param.id = "e";
            expectEquals(set.add(&fine.desc), CRUCIBLE_MODULE_OK, set.error());

            // The registry refuses the same thing for descriptors added directly.
            auto registry = ModuleRegistry::builder(ModuleRegistry::neutralLayout());
            const size_t before = registry->modules().size();
            ModuleDescriptor d1;
            d1.id = "test.one";
            d1.params.push_back(params::toggle("test_shared", "Shared", "test", false));
            registry->add(d1);
            ModuleDescriptor d2;
            d2.id = "test.two";
            d2.params.push_back(params::toggle("test_shared", "Shared", "test", false));
            bool threw = false;
            try
            {
                registry->add(d2);
            }
            catch (const std::logic_error&)
            {
                threw = true;
            }
            expect(threw, "a duplicate parameter id across modules is refused");
            expect(registry->find("test.two") == nullptr && registry->modules().size() == before + 1,
                   "and nothing is added");
            ModuleDescriptor d3;
            d3.id = "test.three";
            d3.params.push_back(params::toggle("test_twice", "A", "test", false));
            d3.params.push_back(params::toggle("test_twice", "B", "test", false));
            threw = false;
            try
            {
                registry->add(d3);
            }
            catch (const std::logic_error&)
            {
                threw = true;
            }
            expect(threw, "a duplicate inside one module is refused");
        }

        beginTest("the version 1 size floor is the end of the last version 1 field, not sizeof of today's struct");
        {
            // A module built against version 1 reports the size of its structs; a later version appends fields and the
            // host must still accept the shorter v1 structs.
            RawModule m("acme.v1only");
            m.param.size = offsetof(crucible_param_spec, choices) + sizeof(m.param.choices);
            crucible_port_spec out{0, CRUCIBLE_MODULE_ABI_VERSION, "out", CRUCIBLE_PORT_AUDIO_STEREO};
            out.size = offsetof(crucible_port_spec, type) + sizeof(out.type);
            const crucible_port_spec* outs[1] = {&out};
            m.desc.kind = CRUCIBLE_MODULE_KIND_FILTER;
            m.desc.num_outputs = 1;
            m.desc.outputs = outs;
            m.desc.size = offsetof(crucible_module_descriptor, holds_voice) + sizeof(m.desc.holds_voice);
            error.clear();
            expectEquals(sdk::validateDescriptor(&m.desc, error), CRUCIBLE_MODULE_OK, error);
            m.desc.size -= 1;
            expectEquals(sdk::validateDescriptor(&m.desc, error), CRUCIBLE_MODULE_ERR_INVALID, "one byte short of v1");
        }

        beginTest("an older module (smaller abi_version) is accepted: structs are append-only");
        // Version 1 is the oldest and the newest today; the rule is [MIN, VERSION].
        expect(CRUCIBLE_MODULE_ABI_MIN <= CRUCIBLE_MODULE_ABI_VERSION);
        expectEquals(sdk::validateDescriptor(&ok.desc, error), CRUCIBLE_MODULE_OK);
    }

    // ---------------------------------------------------------------------------------------------------------
    void registry()
    {
        beginTest("the C registry registers, counts and refuses duplicates");
        crucible_module_registry* reg = crucible_module_registry_create();
        expect(reg != nullptr);
        RawModule a("acme.alpha"), b("acme.beta");
        expectEquals(crucible_register_module(reg, &a.desc), CRUCIBLE_MODULE_OK);
        expectEquals(crucible_register_module(reg, &b.desc), CRUCIBLE_MODULE_OK);
        expectEquals(crucible_module_registry_count(reg), 2);
        expectEquals(std::string(crucible_module_registry_id(reg, 1)), std::string("acme.beta"));
        expect(crucible_module_registry_id(reg, 2) == nullptr);
        expectEquals(crucible_register_module(reg, &a.desc), CRUCIBLE_MODULE_ERR_DUPLICATE);
        expect(contains(registryError(reg), "already registered"), registryError(reg));
        expectEquals(crucible_module_registry_count(reg), 2);
        RawModule future("acme.future");
        future.desc.abi_version = 2;
        expectEquals(crucible_register_module(reg, &future.desc), CRUCIBLE_MODULE_ERR_ABI_VERSION);
        expectEquals(crucible_module_registry_count(reg), 2, "nothing registered");
        expectEquals(crucible_register_module(nullptr, &a.desc), CRUCIBLE_MODULE_ERR_ARGUMENT);
        expectEquals(crucible_register_module(reg, nullptr), CRUCIBLE_MODULE_ERR_ARGUMENT);

        beginTest("libraries are not loaded unless the host allows it, and bad paths fail cleanly");
        expectEquals(crucible_load_module_library(reg, "/does/not/exist.so"), CRUCIBLE_MODULE_ERR_DISABLED);
        crucible_module_registry_allow_libraries(reg, 1);
#ifndef CRUCIBLE_NO_MODULE_LOADING
        expectEquals(crucible_load_module_library(reg, "/does/not/exist.so"), CRUCIBLE_MODULE_ERR_LOAD);
        expect(contains(registryError(reg), "exist.so"), registryError(reg));
#endif
        expectEquals(crucible_load_module_library(reg, nullptr), CRUCIBLE_MODULE_ERR_ARGUMENT);
        crucible_module_registry_destroy(reg);

        beginTest("registerModule is the path every built-in takes, and it refuses a duplicate id");
        auto builder = ModuleRegistry::builder(ModuleRegistry::neutralLayout());
        ModuleDescriptor d;
        d.id = "fx.saturator";
        bool threw = false;
        try
        {
            sdk::registerModule(*builder, d);
        }
        catch (const std::logic_error&)
        {
            threw = true;
        }
        expect(threw, "fx.saturator is registered already");
    }

#ifdef CRUCIBLE_TEST_C_MODULE
    // ---------------------------------------------------------------------------------------------------------
    void libraries()
    {
        beginTest("a C module and a C++ module load from shared libraries");
        sdk::ModuleSet set;
        set.allowLibraries(true);
        expectEquals(set.loadLibrary(CRUCIBLE_TEST_C_MODULE), CRUCIBLE_MODULE_OK, set.error());
        expectEquals(set.loadLibrary(CRUCIBLE_TEST_CPP_MODULE), CRUCIBLE_MODULE_OK, set.error());
        expectEquals(set.count(), 3);
        const auto ids = set.ids();
        expect(std::find(ids.begin(), ids.end(), "acme.c_saturator") != ids.end());
        expect(std::find(ids.begin(), ids.end(), "acme.cpp_sine") != ids.end());
        expect(std::find(ids.begin(), ids.end(), "acme.cpp_onepole") != ids.end());
        expectEquals(set.loadLibrary(CRUCIBLE_TEST_C_MODULE), CRUCIBLE_MODULE_ERR_DUPLICATE, "loaded twice");
        expectEquals(set.count(), 3);

        beginTest("a library built for another ABI version fails cleanly and registers nothing");
        sdk::ModuleSet other;
        other.allowLibraries(true);
        expectEquals(other.loadLibrary(CRUCIBLE_TEST_FUTURE_MODULE), CRUCIBLE_MODULE_ERR_ABI_VERSION, other.error());
        expect(contains(other.error(), "ABI version 999"), other.error());
        expectEquals(other.count(), 0);
        expectEquals(other.loadLibrary(CRUCIBLE_TEST_C_MODULE), CRUCIBLE_MODULE_OK, "the set is still usable");
    }

    // ---------------------------------------------------------------------------------------------------------
    graph::NodeRegistry nodesWithExamples(sdk::ModuleSet& set)
    {
        set.allowLibraries(true);
        set.loadLibrary(CRUCIBLE_TEST_C_MODULE);
        set.loadLibrary(CRUCIBLE_TEST_CPP_MODULE);
        graph::NodeRegistry nodes = graph::NodeRegistry::builtIn();
        set.addTo(nodes);
        return nodes;
    }

    void fxGraph()
    {
        beginTest("an external C effect renders inside the FX graph and shows up in introspection");
        graph::NodeRegistry nodes;
        std::shared_ptr<const graph::CompiledProduct> product;
        std::string error;
        {
            sdk::ModuleSet set;
            nodes = nodesWithExamples(set);
            product = build(kSaturatorFx, nodes, error);
        } // the set is gone; the modules live on in the product
        expect(product != nullptr, error);
        if (!product)
            return;
        const auto& model = product->registry->parameters();
        expect(model.indexOf("acme_c_saturator_drive") >= 0, "first instance keeps the module's ids");
        expect(model.indexOf("fx2_acme_c_saturator_drive") >= 0, "second instance is a copy with its own ids");
        expect(model.indexOf("acme_c_saturator_on") >= 0, "the host adds a bypass switch");

        const std::string json = describeJSON(*product->registry);
        expect(contains(json, "\"acme.c_saturator\""), "describeJSON lists the module");
        expect(contains(json, "\"acme_c_saturator_drive\""));
        ProductDescription desc;
        expect(ProductDescription::fromJson(json, desc, error), error);
        bool found = false;
        for (const auto& m : desc.modules)
            if (m.id == "acme.c_saturator")
            {
                found = true;
                expectEquals(m.kind, std::string("effect"));
                expectEquals(m.category, std::string("Distortion"));
                expectEquals(m.params.size(), size_t(4));
                expectEquals(m.inputs.size(), size_t(1));
            }
        expect(found, "module entry in the description");

        const std::vector<std::pair<std::string, float>> loud = {
            {"amp_attack", 0.001f}, {"amp_sustain", 1.0f},       {"master_gain", 0.0f}, {"osc1_table", 0.0f},
            {"osc1_pos", 1.0f},     {"filter_cutoff", 20000.0f}, {"filter_res", 0.0f}};
        auto bypassed = loud;
        bypassed.push_back({"acme_c_saturator_on", 0.0f});
        bypassed.push_back({"fx2_acme_c_saturator_on", 0.0f});
        auto driven = loud;
        driven.push_back({"acme_c_saturator_drive", 36.0f});
        driven.push_back({"acme_c_saturator_mix", 1.0f});
        long allocations = -1;
        const Audio dry = play(product, bypassed, {57}, 9600);
        const Audio wet = play(product, driven, {57}, 9600, &allocations);
        expect(peakOf(dry.left) > 0.05, "the voice sounds");
        expect(finiteAll(wet.left) && finiteAll(wet.right));
        expect(maxDiff(dry.left, wet.left) > 0.01, "the saturator changes the sound");
        expectEquals(allocations, 0L, "allocations on the audio thread");

        beginTest("an effect type that is not registered is an error that names it");
        graph::ProductDef def;
        expect(graph::productFromText(kSaturatorFx, def, error), error);
        const auto missing = graph::validate(def);
        expect(!missing.ok());
        expect(contains(missing.text(), "acme.c_saturator"), missing.text());
    }

    // ---------------------------------------------------------------------------------------------------------
    void voiceGraph()
    {
        beginTest("external C++ nodes render inside a voice graph");
        sdk::ModuleSet set;
        auto nodes = nodesWithExamples(set);
        std::string error;
        auto product = build(kSdkVoice, nodes, error);
        expect(product != nullptr, error);
        if (!product)
            return;
        long allocations = -1;
        const Audio a = play(product, {{"master_gain", 0.0f}}, {69}, 48000, &allocations);
        const size_t from = 4800, to = 48000;
        const int crossings = risingCrossings(a.left, from, to);
        const double expected = 440.0 * static_cast<double>(to - from) / kRate;
        expectWithinAbsoluteError(crossings, expected, 3.0, "440 Hz sine for note 69");
        const double peak = peakOf(a.left, from);
        expect(peak > 0.3 && peak < 0.5, "level 0.5 at velocity 0.8: peak " + String(peak, 3));
        expectEquals(allocations, 0L, "allocations on the audio thread");

        beginTest("the node parameters and instances appear in introspection");
        const std::string json = describeJSON(*product->registry);
        expect(contains(json, "\"osc_level\""));
        expect(contains(json, "\"lp_cutoff\""));
        ProductDescription desc;
        expect(ProductDescription::fromJson(json, desc, error), error);
        bool found = false;
        for (const auto& m : desc.modules)
            if (m.id == "osc")
            {
                found = true;
                expectEquals(m.instanceOf, std::string("acme.cpp_sine"));
                expectEquals(m.kind, std::string("oscillator"));
                expectEquals(m.outputs.size(), size_t(1));
            }
        expect(found);

        beginTest("voice nodes are prepared with a seed of their own, not a constant");
        {
            sdk::ModuleSet probeSet;
            expectEquals(probeSet.add(sdk::Module<SeedProbe>::descriptor()), CRUCIBLE_MODULE_OK, probeSet.error());
            graph::NodeRegistry probeNodes = graph::NodeRegistry::builtIn();
            probeSet.addTo(probeNodes);
            const char* probeProduct = R"({
                "format": "crucible-product", "version": 1, "id": "test.seedprobe",
                "voice": { "nodes": [
                    { "id": "gen", "type": "test.seedprobe" },
                    { "id": "out", "type": "mix.out" } ],
                  "edges": [ { "from": "gen.out", "to": "out.in" } ] } })";
            auto compiled = build(probeProduct, probeNodes, error);
            expect(compiled != nullptr, error);
            if (compiled)
            {
                gProbeSeeds.clear();
                play(compiled, {}, {60}, 960);
                std::set<std::uint32_t> distinct(gProbeSeeds.begin(), gProbeSeeds.end());
                expect(gProbeSeeds.size() >= 2, "instances prepared: " + String(static_cast<int>(gProbeSeeds.size())));
                expectEquals(distinct.size(), gProbeSeeds.size(), "every instance has its own seed");
                expect(distinct.count(1U) == 0, "not the constant 1");
                // The same product and engine seed give the same seeds again.
                const auto first = gProbeSeeds;
                gProbeSeeds.clear();
                play(compiled, {}, {60}, 960);
                expect(first == gProbeSeeds, "deterministic");
            }
        }

        beginTest("four voices go through the module's quad entry and match single voices");
        const Audio chord = play(product, {{"master_gain", 0.0f}}, {60, 64, 67, 72}, 9600);
        Audio sum;
        sum.left.assign(9600, 0.0f);
        for (const int n : {60, 64, 67, 72})
        {
            const Audio one = play(product, {{"master_gain", 0.0f}}, {n}, 9600);
            for (size_t i = 0; i < sum.left.size(); ++i)
                sum.left[i] += one.left[i];
        }
        expectWithinAbsoluteError(maxDiff(chord.left, sum.left), 0.0, 1.0e-5, "chord equals the sum of its notes");
        expect(peakOf(chord.left) > 0.5, "the chord sounds");
    }

    // ---------------------------------------------------------------------------------------------------------
    void cApi()
    {
        beginTest("a host builds an engine from a product and a module registry through the C API");
        crucible_module_registry* reg = crucible_module_registry_create();
        crucible_module_registry_allow_libraries(reg, 1);
        expectEquals(crucible_load_module_library(reg, CRUCIBLE_TEST_C_MODULE), CRUCIBLE_MODULE_OK, registryError(reg));
        expectEquals(crucible_load_module_library(reg, CRUCIBLE_TEST_FUTURE_MODULE), CRUCIBLE_MODULE_ERR_ABI_VERSION);
        expect(contains(registryError(reg), "999"), registryError(reg));
        expectEquals(crucible_module_registry_count(reg), 1);

        char error[512] = {0};
        CrucibleEngine* none = crucible_create_from_product(nullptr, kSaturatorFx, nullptr, error, sizeof(error));
        expect(none == nullptr && std::strlen(error) > 0, "without the registry the product does not compile");

        CrucibleEngine* e = crucible_create_from_product(nullptr, kSaturatorFx, reg, error, sizeof(error));
        expect(e != nullptr, error);
        if (e == nullptr)
        {
            crucible_module_registry_destroy(reg);
            return;
        }
        // The registry can go while the engine plays: the modules stay loaded as long as they are in use.
        crucible_module_registry_destroy(reg);
        {
            std::string description(static_cast<size_t>(std::max(crucible_describe_json(e, nullptr, 0), 0)) + 1, '\0');
            crucible_describe_json(e, description.data(), description.size());
            expect(contains(description.c_str(), "acme.c_saturator"));
        }
        const int drive = crucible_param_index(e, "acme_c_saturator_drive");
        expect(drive >= 0);
        crucible_prepare(e, kRate, 512);
        crucible_set_param_id(e, "amp_sustain", 1.0f);
        crucible_note_on(e, 57, 0.8f);
        std::vector<float> l(512), r(512);
        double peak = 0.0;
        bool finite = true;
        for (int b = 0; b < 20; ++b)
        {
            crucible_process(e, l.data(), r.data(), 512, nullptr, 0);
            for (const float x : l)
            {
                finite = finite && std::isfinite(x);
                peak = std::max(peak, static_cast<double>(std::fabs(x)));
            }
        }
        expect(finite && peak > 0.01, "the engine renders through the external effect");
        crucible_destroy(e);
    }

    // ---------------------------------------------------------------------------------------------------------
    void cApiExceptions()
    {
        // A module that throws something that is not a std::exception from create / prepare: the C API turns it into the
        // documented failure value, and nothing crosses the boundary.
        static const auto throwingCreate = [](const crucible_module_descriptor*, const crucible_module_host*) -> void*
        { throw 7; };
        static const auto throwingPrepare = [](void*, const crucible_module_prepare*) -> int32_t { throw 7; };
        const char* product = R"({
            "format": "crucible-product", "version": 1, "id": "test.throws",
            "fx": { "instances": [ { "id": "t", "type": "acme.raw_throws" } ],
                    "lanes": [ { "id": "main", "order": "listed", "input": ["voices"], "chain": ["t"] } ] } })";

        beginTest("a module that throws from create makes crucible_create_from_product return NULL with a message");
        {
            crucible_module_registry* reg = crucible_module_registry_create();
            RawModule m("acme.raw_throws");
            m.desc.create = throwingCreate;
            expectEquals(crucible_register_module(reg, &m.desc), CRUCIBLE_MODULE_OK);
            char error[256] = {0};
            CrucibleEngine* e = crucible_create_from_product(nullptr, product, reg, error, sizeof(error));
            expect(e == nullptr);
            expect(std::strlen(error) > 0, "an error text");
            crucible_destroy(e);
            crucible_module_registry_destroy(reg);
        }

        beginTest("a module that throws from prepare does not escape crucible_prepare, and the engine still answers");
        {
            crucible_module_registry* reg = crucible_module_registry_create();
            RawModule m("acme.raw_throws");
            m.desc.prepare = throwingPrepare;
            expectEquals(crucible_register_module(reg, &m.desc), CRUCIBLE_MODULE_OK);
            char error[256] = {0};
            CrucibleEngine* e = crucible_create_from_product(nullptr, product, reg, error, sizeof(error));
            expect(e != nullptr, error);
            if (e != nullptr)
            {
                crucible_prepare(e, kRate, 512);
                expectEquals(crucible_set_param_id(e, "filter_cutoff", 1000.0f), 1);
                crucible_destroy(e);
            }
            crucible_module_registry_destroy(reg);
        }

        beginTest("the registry error is copied into a caller buffer and its size returned");
        {
            crucible_module_registry* reg = crucible_module_registry_create();
            expectEquals(crucible_module_registry_error(reg, nullptr, 0), 0);
            RawModule a("acme.alpha");
            expectEquals(crucible_register_module(reg, &a.desc), CRUCIBLE_MODULE_OK);
            expectEquals(crucible_register_module(reg, &a.desc), CRUCIBLE_MODULE_ERR_DUPLICATE);
            const int need = crucible_module_registry_error(reg, nullptr, 0);
            expect(need > 0);
            char small[8];
            expectEquals(crucible_module_registry_error(reg, small, sizeof small), need);
            expectEquals(static_cast<int>(std::strlen(small)), 7);
            expectEquals(static_cast<int>(registryError(reg).size()), need);
            expectEquals(crucible_module_registry_error(nullptr, small, sizeof small), 0);
            crucible_module_registry_destroy(reg);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void checkExamples()
    {
        beginTest("the example modules pass the real-time harness, allocation counter included");
        sdk::ModuleSet set;
        auto nodes = nodesWithExamples(set);
        const auto subjects = sdk::check::sdkSubjects(nodes);
        expectEquals(subjects.size(), size_t(3));
        for (const auto& s : subjects)
        {
            const auto report = sdk::check::run(s, quickOptions());
            expect(report.passed(), s.id + failedText(report));
            const auto* alloc = find(report, "allocation");
            expect(alloc != nullptr && alloc->status == sdk::check::Status::Pass, s.id + " allocation");
        }
    }
#endif

    // ---------------------------------------------------------------------------------------------------------
    void harness()
    {
        beginTest("the harness passes a well-behaved C++ module");
        sdk::ModuleSet set;
        expectEquals(set.add(sdk::Module<TestGain>::descriptor()), CRUCIBLE_MODULE_OK, set.error());
        expectEquals(set.add(sdk::Module<AllocatingModule>::descriptor()), CRUCIBLE_MODULE_OK);
        expectEquals(set.add(sdk::Module<NanModule>::descriptor()), CRUCIBLE_MODULE_OK);
        expectEquals(set.add(sdk::Module<LoudModule>::descriptor()), CRUCIBLE_MODULE_OK);
        expectEquals(set.add(sdk::Module<DriftingModule>::descriptor()), CRUCIBLE_MODULE_OK);
        graph::NodeRegistry nodes;
        set.addTo(nodes);
        std::map<std::string, sdk::check::Report> reports;
        for (const auto& s : sdk::check::sdkSubjects(nodes))
            reports[s.id] = sdk::check::run(s, quickOptions());
        expectEquals(reports.size(), size_t(5));
        expect(reports["test.gain"].passed(), failedText(reports["test.gain"]));

        beginTest("and catches a module that breaks one rule of the contract");
        const auto failedCheck = [&](const char* id, const char* check)
        {
            const auto* c = find(reports[id], check);
            expect(c != nullptr && c->status == sdk::check::Status::Fail, std::string(id) + " should fail " + check);
        };
#if CRUCIBLE_ALLOC_COUNTER_AVAILABLE
        failedCheck("test.allocates", "allocation");
#else
        logMessage("allocation counter unavailable under a sanitizer on this platform: allocation check not exercised");
#endif
        failedCheck("test.nan", "finite");
        failedCheck("test.loud", "extremes");
        failedCheck("test.drifts", "determinism");
        // Only the broken rule fails: everything else about these modules is fine.
#if CRUCIBLE_ALLOC_COUNTER_AVAILABLE
        expectEquals(reports["test.allocates"].failures(), 1, failedText(reports["test.allocates"]));
#endif
        expectEquals(reports["test.nan"].failures(), 1, failedText(reports["test.nan"]));
        expectEquals(reports["test.loud"].failures(), 1, failedText(reports["test.loud"]));
        expectEquals(reports["test.drifts"].failures(), 1, failedText(reports["test.drifts"]));

        beginTest("the check is repeatable for a seed");
        const auto subjects = sdk::check::sdkSubjects(nodes);
        auto a = sdk::check::run(subjects[0], quickOptions());
        auto b = sdk::check::run(subjects[0], quickOptions());
        expectEquals(a.checks.size(), b.checks.size());
        for (size_t i = 0; i < a.checks.size() && i < b.checks.size(); ++i)
            expect(a.checks[i].status == b.checks[i].status &&
                       (a.checks[i].name == "cpu" || a.checks[i].name == "denormals" ||
                        a.checks[i].detail == b.checks[i].detail),
                   a.checks[i].name);
    }

    // ---------------------------------------------------------------------------------------------------------
    void analogFilterNode()
    {
        beginTest("filter.analog plays the analog models and matches the legacy node on the same model");
        std::string error;
        const auto& builtIn = graph::NodeRegistry::builtIn();
        const auto analog = build(filterProduct(R"({ "id": "flt", "type": "filter.analog",
                                    "params": { "model": 1, "cutoff": 900, "res": 0.4, "drive": 0.2 } })"),
                                  builtIn, error);
        const auto legacy = build(filterProduct(R"({ "id": "flt", "type": "filter.svf",
                                    "params": { "type": 18, "cutoff": 900, "res": 0.4, "drive": 0.2 } })"),
                                  builtIn, error);
        expect(analog != nullptr && legacy != nullptr, error);
        if (!analog || !legacy)
            return;
        long allocations = -1;
        const Audio a = play(analog, {}, {45}, 9600, &allocations);
        const Audio l = play(legacy, {}, {45}, 9600);
        expect(finiteAll(a.left) && peakOf(a.left) > 0.01, "the analog node sounds");
        expectWithinAbsoluteError(maxDiff(a.left, l.left), 0.0, 1.0e-6, "same model, same settings");
        expectEquals(allocations, 0L);

        beginTest("all three models are selectable without allocating on the audio thread");
        for (int model = 0; model < 3; ++model)
        {
            long n = -1;
            const std::string node = R"({ "id": "flt", "type": "filter.analog" })";
            const auto p = build(filterProduct(node.c_str()), graph::NodeRegistry::builtIn(), error);
            expect(p != nullptr, error);
            if (!p)
                return;
            const Audio out = play(p, {{"flt_model", static_cast<float>(model)}, {"flt_cutoff", 1500.0f}, {"flt_res", 0.5f}},
                                   {52}, 9600, &n);
            expect(finiteAll(out.left) && peakOf(out.left) > 0.01, "model " + String(model));
            expectEquals(n, 0L, "model " + String(model));
        }

        beginTest("selecting an analog model in filter.svf does not allocate either");
        const auto svf = build(filterProduct(R"({ "id": "flt", "type": "filter.svf" })"), builtIn, error);
        expect(svf != nullptr, error);
        if (svf)
        {
            long n = -1;
            const Audio out = play(svf, {{"flt_type", 19.0f}, {"flt_cutoff", 1500.0f}}, {52}, 9600, &n);
            expect(finiteAll(out.left) && peakOf(out.left) > 0.01);
            expectEquals(n, 0L);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    void builtIns()
    {
        beginTest("every built-in module passes the module check");
        auto options = quickOptions();
        options.seconds = 0.3;
        options.corners = 3;
        int modules = 0, processors = 0;
        std::string problems;
        for (const auto& s : sdk::check::builtInSubjects())
        {
            const auto report = sdk::check::run(s, options);
            ++modules;
            processors += s.make ? 1 : 0;
            if (!report.passed())
                problems += "\n    " + s.id + failedText(report);
        }
        expect(problems.empty(), "failing built-ins:" + problems);
        expect(modules > 40, "the neutral product has more than 40 modules");
        expect(processors > 25, "and most of them process audio: " + String(processors));
    }
};

static SdkTests sdkTests;
