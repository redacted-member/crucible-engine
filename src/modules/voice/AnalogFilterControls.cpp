// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/module/ParamBuilder.h>

#include <string>

// The voicing controls of the nonlinear filter models (FilterModel 17 and up) for every filter slot: `character`
// (gain compensation of the ladders and OTA filters, clipper hardness of the Sallen-Key, low-pass / notch / high-pass
// morph of the OTA state-variable filter) and `hp`, the passive high-pass step ahead of the model. The filter slots,
// their type, cutoff and resonance stay where the wavetable voice declares them; the legacy models ignore both. The module
// is registered after everything else so that the indices of the existing parameters do not move.

namespace crucible::modules
{

ModuleDescriptor analogFilterControls(const VoiceGraphShape& shape)
{
    ModuleDescriptor d;
    d.id = "voice.filter_analog";
    d.name = "Analog Filter Controls";
    d.kind = ModuleKind::Voice;
    d.category = "Filter";
    d.cpu = CpuClass::Light;
    d.description = "Character and passive high-pass step of the nonlinear filter models: Sallen-Key with diode feedback "
                    "clipping, OTA state-variable morph, four-pole OTA with partial compensation, the transistor ladder, "
                    "OTA cascade and diode ladder, and the passive high-pass.";
    for (int slot = 1; slot <= shape.filters; ++slot)
    {
        const auto prefix = slot == 1 ? std::string("filter") : "filter" + std::to_string(slot);
        const auto label = "Filter " + std::to_string(slot);
        // The filter's own page already holds six knobs: the voicing gets a page of its own.
        const auto page = prefix + "_voicing";
        d.pages.push_back({page, label + " Voicing", "filter", label + " Voicing",
                           "Character of the nonlinear filter models and the passive high-pass step ahead of them."});
        d.params.push_back(params::floating(prefix + "_character", label + " Character", page, 0.0f, 1.0f, 0.0f));
        d.params.push_back(
            params::choice(prefix + "_hp", label + " High-pass", page, {"Off", "60 Hz", "180 Hz", "540 Hz"}, 0));
    }
    return d;
}

} // namespace crucible::modules
