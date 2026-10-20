// SPDX-License-Identifier: MIT
#pragma once

// Product definitions: the instrument as data. A `crucible-product` JSON document (format version 1) names the module
// instances, the voice graph (nodes and edges, optionally split into layers), the FX graph (instances, lanes, sends)
// and the parameter ids it pins. The compiler (Compiler.h) validates a definition and builds the parameter model and
// the schedules the audio thread runs.
//
//   {
//     "format": "crucible-product", "version": 1,
//     "id": "my.product", "name": "My Product",
//     "layout": { "oscillators": 3, "filters": 2, "macros": 6 },   // optional; the voice shape of classic.voice / slots
//     "voice": "classic" | "none" | {                              // default: "classic"
//        "nodes":  [ { "id": "env", "type": "env.adsr", "params": { "attack": 0.002 } }, ... ],
//        "edges":  [ { "from": "osc.out", "to": "vca.in" },                       // audio
//                    { "from": "env.out", "to": "vca.gain", "amount": 1.0 } ],      // modulation (port types decide)
//        "layers": [ { "id": "kick", "notes": [36, 36], "velocity": [1, 127], "choke": "hats", "voices": 2,
//                      "oneShot": true, "round_robin": "snare", "nodes": [...], "edges": [...] } ]
//     },
//     "fx": {                                                      // absent: the classic chain of every built-in effect
//        "instances": [ { "id": "drive_a", "type": "fx.saturator" }, ... ],
//        "lanes":  [ { "id": "main", "order": "listed" | "position", "input": ["voices"], "chain": ["drive_a"],
//                      "level": 1.0, "master": true } ],
//        "sends":  [ { "from": "main", "to": "verb", "gain": 0.3 } ],
//        "master": { "level": 1.0 } },
//     "surface":  {                                                // optional: what a plug-in host sees
//        "expose":   "all" | "listed",                             // default "all": every automatable parameter
//        "sections": [ { "id": "osc", "name": "Oscillators" } ],   // optional; replaces the neutral sections
//        "pages":    [ { "id": "osc1", "name": "Osc 1", "section": "osc", "params": [ "osc1_type", "osc1_level" ] } ] },
//     "idMap":    { "<generated id>": "<pinned id>" },
//     "defaults": { "<parameter id>": value }
//   }
//
// Parameter ids. The first instance of a built-in effect type keeps its id ("fx_saturator_drive"); instance k >= 2 of a type
// is "fx{k}_saturator_drive". The parameters of a graph node are "<instance>_<local>" ("kick_env_attack"). `idMap` pins a
// generated id to another one (ids of the first instances are never renamed).
//
// Surface. The pages of the surface become the product's pages (describeJSON: page, pageId, group) and a listed
// parameter moves onto its page; the order of the lists is the order of Engine::surface() / crucible_surface_param().
// With "expose": "listed" only the listed parameters are exposed (exposed = 1); the others get automatable = 0 but
// still load from patches and stay modulation targets. A parameter is listed at most once. Without "surface", or with
// "expose": "all", every automatable parameter is exposed in index order and the pages are the modules' own.

#include <crucible/Json.h>

#include <string>
#include <utility>
#include <vector>

namespace crucible::graph
{

constexpr int kProductFormatVersion = 1;
constexpr const char* kProductFormat = "crucible-product";

struct NodeDef
{
    std::string id;
    std::string type;
    int slot = 0;                // 1-based oscillator slot for the oscillator wrappers
    std::string scope = "voice"; // "voice" or "layer" (a modulator per layer instead of per voice is not built yet)
    std::vector<std::pair<std::string, double>> params; // local id -> default value
};

struct EdgeDef
{
    std::string from; // "node.port"
    std::string to;
    float amount = 1.0f; // modulation edges
};

struct LayerDef
{
    std::string id;
    int noteLo = 0, noteHi = 127;
    int velLo = 1, velHi = 127;
    std::string chokeGroup; // a new note of the group silences the other layers' voices of the group
    std::string roundRobin; // layers with the same name alternate (among those that match the note)
    int voices = 4;
    bool oneShot = false; // note-off is ignored; the voice ends when its envelopes finish
    std::vector<NodeDef> nodes;
    std::vector<EdgeDef> edges;
};

enum class VoiceKind
{
    Classic, // today's whole voice (classic.voice)
    None,    // no voices (an FX product)
    Graph
};

struct VoiceDef
{
    VoiceKind kind = VoiceKind::Classic;
    std::vector<NodeDef> nodes; // a graph without layers is one layer over every note
    std::vector<EdgeDef> edges;
    std::vector<LayerDef> layers;
};

struct FxInstanceDef
{
    std::string id;
    std::string type; // an Effect module id: "fx.saturator"
};

struct FxSourceDef
{
    std::string source; // "voices", "aux" or the id of an earlier lane
    float gain = 1.0f;
};

struct FxLaneDef
{
    std::string id;
    bool byPosition = false; // "position": ordered by the instances' position parameters
    std::vector<FxSourceDef> inputs;
    std::vector<std::string> chain;
    float level = 1.0f;
    bool toMaster = true;
};

struct FxSendDef
{
    std::string from, to;
    float gain = 1.0f;
};

struct FxDef
{
    bool present = false; // false: the classic chain of every built-in effect
    std::vector<FxInstanceDef> instances;
    std::vector<FxLaneDef> lanes;
    std::vector<FxSendDef> sends;
    float masterLevel = 1.0f;
};

struct SurfaceSectionDef
{
    std::string id;
    std::string name;
};

struct SurfacePageDef
{
    std::string id;
    std::string name;
    std::string section; // empty: the first section
    std::vector<std::string> params;
};

struct SurfaceDef
{
    bool present = false;
    bool listed = false; // "expose": "listed"
    std::vector<SurfaceSectionDef> sections;
    std::vector<SurfacePageDef> pages;
};

struct ProductDef
{
    int version = kProductFormatVersion;
    std::string id;
    std::string name;
    std::string description;
    int oscillators = 0; // 0: the neutral product's counts
    int filters = 0;
    int macros = 0;
    VoiceDef voice;
    FxDef fx;
    SurfaceDef surface;
    std::vector<std::pair<std::string, std::string>> idMap;
    std::vector<std::pair<std::string, double>> defaults;
};

// Reads a definition. Returns false and fills `error` (a path such as "voice.layers[1].edges[0]: ..." and a reason)
// when the document is not a valid crucible-product of a known version. Semantic checks are validate()'s job.
bool productFromJson(const json::Value& root, ProductDef& out, std::string& error);
bool productFromText(const std::string& text, ProductDef& out, std::string& error);
json::Value productToJson(const ProductDef& product);
std::string productToText(const ProductDef& product, int indent = 2);

// Reads a definition from a file; false with `error` on failure.
bool loadProduct(const std::string& path, ProductDef& out, std::string& error);

} // namespace crucible::graph
