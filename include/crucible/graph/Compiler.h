// SPDX-License-Identifier: MIT
#pragma once

// The graph compiler: validates a ProductDef (Product.h) and turns it into a CompiledProduct, the immutable result an
// engine plays. Compilation runs on the control thread and may allocate.
//
//  * The parameter registry of the product is built: the built-in modules (minus the effects an FX graph leaves out),
//    one copy of an effect module per further instance (parameter ids "fx{k}_..."), and the parameters of every graph
//    node ("<instance>_<param>"), each with the product's pinned ids applied. Built-in ids and indices never move.
//  * The voice graph (or every layer's) is topologically sorted into a schedule of nodes with their parameter
//    indices and connections.
//  * The FX lanes are sorted so that a lane only reads earlier lanes.
//
// A CompiledProduct can be handed to an engine (EngineConfig::compiled) or swapped into a running one
// (SynthEngine::reconfigure) if it has the same parameters as the one that engine plays.

#include <crucible/graph/FxPlan.h>
#include <crucible/graph/Node.h>
#include <crucible/graph/Product.h>

#include <memory>
#include <string>
#include <vector>

namespace crucible
{
class ModuleRack;
}

namespace crucible::graph
{

struct ProductLimits
{
    int nodesPerVoice = kMaxNodesPerVoice;
    int fxInstances = kMaxFxInstances;
    int layers = kMaxLayers;
    int modulators = kMaxModulators;
};

struct Diagnostic
{
    std::string path;
    std::string message;
};

struct ValidationResult
{
    std::vector<Diagnostic> errors;
    bool ok() const { return errors.empty(); }
    // One line per error: "path: message".
    std::string text() const;
};

// Checks everything that can be checked without building the registry: ids, types, ports, edge kinds, cycles, limits,
// ranges, lanes and their sources.
ValidationResult validate(const ProductDef& product, const NodeRegistry& nodes = NodeRegistry::builtIn(),
                          const ProductLimits& limits = {});

// A connection into a node: the producing node (index into LayerPlan::nodes) and which of its compacted audio or
// modulation outputs.
struct Source
{
    int node = 0;
    int port = 0;
    float amount = 1.0f;
};

struct NodePlan
{
    std::string id;
    const NodeType* type = nullptr; // points into CompiledProduct::nodeTypes
    int slot = -1;                  // 0-based oscillator slot of the wrappers
    const ModuleDescriptor* oscillator = nullptr;
    std::vector<int> params;                  // global parameter index per local parameter
    std::vector<std::vector<Source>> audioIn; // per audio input port
    std::vector<std::vector<Source>> modIn;   // per modulation input port
    std::vector<std::vector<Source>> rateIn;  // per audio-rate input port (the sources are audio outputs)
    int numAudioOut = 0;
    int numModOut = 0;
};

struct LayerPlan
{
    std::string id;
    int noteLo = 0, noteHi = 127, velLo = 1, velHi = 127;
    int chokeGroup = 0; // 0: none
    int roundRobin = 0; // 0: none; layers with the same number alternate
    int voices = 4;
    bool oneShot = false;
    std::vector<NodePlan> nodes; // topological order
    int sink = -1;               // the node whose first audio output is the layer's output
};

// The FX graph with module ids instead of rack slots (the rack of an engine resolves them).
struct FxLaneSpec
{
    std::string id;
    std::vector<FxSource> inputs;
    std::vector<std::string> modules;
    bool byPosition = false;
    bool toMaster = true;
    float level = 1.0f;
};

struct FxPlanSpec
{
    bool classic = true; // no fx section: every effect, ordered by position
    std::vector<FxLaneSpec> lanes;
    float masterLevel = 1.0f;
};

struct FxInstanceInfo
{
    std::string id;
    std::string type;
    std::string module; // descriptor id in the registry ("fx.saturator", "fx.drive.2")
    int ordinal = 1;    // k: 1 for the first instance of the type
};

class CompiledProduct
{
public:
    ProductDef def;
    std::shared_ptr<const ModuleRegistry> registry;
    NodeRegistry nodeTypes; // the types the plans point into
    std::vector<LayerPlan> layers;
    std::vector<FxInstanceInfo> fxInstances;
    FxPlanSpec fx;

    VoiceKind voiceKind() const { return def.voice.kind; }
    // The model's defaults with the product's `defaults` applied (index like registry->parameters().params()).
    std::vector<float> defaultValues() const;
    // The parameter ids of the product, in table order.
    std::vector<std::string> parameterIds() const;
    // True when `other` has the same parameters (ids and order), so an engine playing this product can switch to it.
    bool sameParameters(const CompiledProduct& other) const;
    // The generated parameter id of a node's local parameter (idMap applied), or empty.
    std::string nodeParameterId(const std::string& node, const std::string& local) const;
    // Builds the plan the rack runs. Returns false when a module is not in the rack.
    bool buildFxPlan(const ModuleRack& rack, FxPlan& out) const;

    std::vector<std::pair<std::string, std::string>> nodeParamIds; // "node/local" -> final id
};

// Compiles a definition; nullptr with `error` set on failure (validation errors, one per line, or a build failure).
std::shared_ptr<const CompiledProduct> compile(const ProductDef& product, std::string& error,
                                               const NodeRegistry& nodes = NodeRegistry::builtIn(),
                                               const ProductLimits& limits = {});

// Compiles a definition that only changes topology (FX lanes and chains, layer ranges, chokes, edges) of a product
// an engine already plays. Fails with an explanation when the parameters would differ (another module set needs a new
// engine). The result shares the base's registry.
std::shared_ptr<const CompiledProduct> compileReconfiguration(const CompiledProduct& base, const ProductDef& product,
                                                              std::string& error);

// The product definition of the engine's default product (all built-in effects in the classic chain, the classic
// voice): the same data as products/crucible-default.json.
ProductDef defaultProductDef();

} // namespace crucible::graph
