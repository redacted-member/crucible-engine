// SPDX-License-Identifier: MIT
#pragma once

// Host side of the module SDK: the modules a program has registered (C ABI descriptors from crucible_module.h, in this
// process or in shared libraries), turned into what the engine plays. A module of kind effect becomes an Effect module
// of the product (an FX graph instance type); every other kind becomes a node type of the voice graph. Hand the result to
// the graph compiler:
//
//   crucible::sdk::ModuleSet modules;
//   modules.allowLibraries(true);
//   modules.loadLibrary("/path/to/libacme_tremolo.so");
//   crucible::graph::NodeRegistry nodes = crucible::graph::NodeRegistry::builtIn();
//   modules.addTo(nodes);
//   auto product = crucible::graph::compile(definition, error, nodes);
//
// The built-in modules reach the engine through the same registration function (registerModule), so a built-in and a
// third-party module are both just descriptors in a registry. See docs/MODULE_GUIDE.md.

#include <crucible/crucible_module.h>
#include <crucible/graph/Node.h>
#include <crucible/module/Module.h>

#include <memory>
#include <string>
#include <vector>

namespace crucible::sdk
{

// Checks a descriptor against the rules of the ABI (versions, sizes, ids, parameters, ports, required functions).
// Returns CRUCIBLE_MODULE_OK or the matching CRUCIBLE_MODULE_ERR_* code with `error` set.
int validateDescriptor(const crucible_module_descriptor* descriptor, std::string& error);

// The one path every module takes into a ModuleRegistry, built-in or not. Throws std::logic_error as
// ModuleRegistry::add does (missing or duplicate id, registry already finalized).
void registerModule(ModuleRegistry& registry, ModuleDescriptor descriptor);

// Modules registered with a host. Control thread only. Modules stay alive for as long as an engine, node or effect made
// from them exists, even after the set is destroyed; loaded libraries are closed after the last of them.
class ModuleSet
{
public:
    ModuleSet();
    ~ModuleSet();
    ModuleSet(const ModuleSet&) = delete;
    ModuleSet& operator=(const ModuleSet&) = delete;

    // Shared libraries are only opened when the host allows it (default: not).
    void allowLibraries(bool allow);
    bool librariesAllowed() const;

    // Registers one module; the descriptor and what it points to must outlive the set (static storage).
    int add(const crucible_module_descriptor* descriptor);
    // Opens a shared library, looks up crucible_get_modules and registers all of its modules or none.
    int loadLibrary(const std::string& path);
    // The reason of the last failed call ("" if none).
    const std::string& error() const;

    int count() const;
    std::vector<std::string> ids() const;
    // Id of the k-th registered module (valid for the life of the set).
    const std::string& idOf(int k) const;

    // Adds the node types (voice graph) and effect modules of every registered module to a node registry.
    void addTo(graph::NodeRegistry& nodes) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

// An effect module that offers views (the optional get_view of the ABI); found with ModuleRack::findCapability.
class ViewSource
{
public:
    virtual ~ViewSource() = default;
    // Control thread. JSON text of the view, or null; valid until the next call.
    virtual const char* view(const char* viewId) = 0;
};

} // namespace crucible::sdk
