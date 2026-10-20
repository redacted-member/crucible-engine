// SPDX-License-Identifier: MIT
#pragma once

// Helpers shared by the oscillator modules: per-slot parameter ids and pages.

#include <crucible/module/Module.h>
#include <crucible/module/ParamBuilder.h>

#include <string>

namespace crucible::modules::osc
{

inline std::string slotId(int slot, const std::string& suffix)
{
    return "osc" + std::to_string(slot + 1) + "_" + suffix;
}

inline std::string slotLabel(int slot)
{
    return "Osc " + std::to_string(slot + 1);
}

// Adds a page for the slot to the module (hardware section follows the slot: Osc 1, or Osc 2 / 3 and above).
inline std::string addSlotPage(ModuleDescriptor& d, int slot, const std::string& suffix, const std::string& name,
                               const std::string& description)
{
    const auto id = slotId(slot, suffix);
    d.pages.push_back({id, slotLabel(slot) + " " + name, slot == 0 ? "osc1" : "osc2",
                       "Oscillator " + std::to_string(slot + 1) + " " + name, description});
    return id;
}

inline ParamDef voiceMod(ParamDef p)
{
    return params::voiceModulatable(std::move(p));
}

} // namespace crucible::modules::osc
