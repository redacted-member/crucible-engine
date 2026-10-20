// SPDX-License-Identifier: MIT
#pragma once

// Introspection: everything a UI, a host or a tool needs to know about a product, read from the live registry of an
// engine instance rather than from documentation. describe() collects it into plain structs; toJson() writes the
// same as the "describeJSON" document (schema 2) and fromJson() reads it back, so
// parse(describe().toJson()).toJson() reproduces the text exactly.
//
// The document lists, in order: product (layout id, voice graph shape, macros), sections, pages, modules (id, name,
// kind, category, version, cpu, description, ports, views, capabilities, parameter indices and instances), the views
// (read-back streams, see Views.h) and all parameters in model order, each with id, names, group, page, type, range,
// default, interval, skew, unit, steps, choices, smoothing, polarity, ui hint, display format and the modulatable /
// per-voice / automatable / scene flags. Parameter indices are those of ParameterModel::params() of the same product.
//
// Views: a live description (one made from an engine) lists every view of the engine's hub with its instance, id,
// kind ("scalar", "vector", "points", "mask"), payload capacity in 32-bit elements (what crucible_view_read needs as
// capacity), stride, whether it is live (published by the audio thread) or pulled, and the module that owns it
// ("" for engine-wide ones such as "master" / "meter"). A module's own "views" lists the ids of its views. The
// per-parameter views of instance "params" ("<parameter id>.mod", ".voice", ".band", ".voice@<n>") are not listed one
// by one; they exist for every parameter.

#include <crucible/module/Module.h>
#include <crucible/module/ModuleRack.h>

#include <string>
#include <vector>

namespace crucible
{

class ViewHub;

// One occurrence of a module in the product: a module that exists once per oscillator slot has an instance per
// slot ("osc1", "osc2"...), every other module has a single instance named after itself.
struct ModuleInstanceInfo
{
    std::string id;
    std::string name;
    std::vector<int> params; // indices into ProductDescription::params
};

struct ModuleInfo
{
    std::string id;
    std::string name;
    std::string kind;
    std::string category;
    int version = 1;
    std::string cpu; // "light" / "medium" / "heavy"
    std::string description;
    std::string status = "current"; // "current" / "legacy"
    std::string supersededBy;       // id of the replacing module; empty unless legacy
    std::vector<PortDef> inputs;
    std::vector<PortDef> outputs;
    std::vector<std::string> views;
    std::vector<std::string> capabilities;
    std::vector<int> params; // indices into ProductDescription::params, in declaration order
    std::vector<ModuleInstanceInfo> instances;
    // Graph products: the module type this is another instance of ("fx.saturator" for the second saturator, "env.adsr" for a
    // graph node). Empty for the built-in modules; written to the JSON only when set.
    std::string instanceOf;
};

// One view of a live engine (see ViewHub::info).
struct ViewDescription
{
    std::string instance;
    std::string id;
    std::string description;
    std::string kind; // "scalar" / "vector" / "points" / "mask"
    int maxElements = 0;
    int stride = 1;
    bool live = true;
    std::string module; // id of the module that owns the view, "" for engine-wide views
};

struct ProductDescription
{
    int schema = 2;
    std::string engine;  // library version
    std::string product; // layout id
    int oscillators = 0;
    int filters = 0;
    int macros = 0;
    std::vector<SectionDef> sections;
    std::vector<PageDef> pages;
    std::vector<ModuleInfo> modules;
    std::vector<ViewDescription> views; // empty unless described from a live engine's hub
    std::vector<ParamDef> params;       // resolved definitions (shortName, group, polarity, uiHint, textFormat, steps set)

    std::string toJson(int indent = 2) const;
    static bool fromJson(const std::string& text, ProductDescription& out, std::string& error);
};

// A product layout (sections, page order, voice graph shape, macro count) as JSON, and back:
// {"id", "oscillators", "filters", "macros", "sections": [{"id", "name"}],
//  "pages": [{"id", "name", "section": index, "title", "description"}]}. Missing fields keep the neutral defaults.
std::string productLayoutToJson(const ProductLayout& layout);
bool productLayoutFromJson(const std::string& text, ProductLayout& out, std::string& error);

// live: the rack of a running engine; its instances add the capabilities they implement (user impulse responses,
// gain-reduction meters, step positions). hub: the engine's views; they are listed in the description and in the
// "views" of the module that owns them.
ProductDescription describe(const ModuleRegistry& registry, ModuleRack* live = nullptr, const ViewHub* hub = nullptr);
std::string describeJSON(const ModuleRegistry& registry, ModuleRack* live = nullptr, const ViewHub* hub = nullptr);

} // namespace crucible
