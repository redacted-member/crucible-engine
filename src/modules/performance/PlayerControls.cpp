// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/module/ParamBuilder.h>

namespace crucible::modules
{

// How the keyboard is played: glide behaviour, mono note priority, MPE expression and the microtuning amount. The
// tuning table itself (Scala files, per-note offsets) is data stored in the patch, see Tuning.h.
ModuleDescriptor playerControls()
{
    ModuleDescriptor d;
    d.id = "perf.player";
    d.name = "Player Modes";
    d.kind = ModuleKind::Performance;
    d.category = "Performance";
    auto& p = d.params;
    const auto add = [&p](ParamDef def)
    {
        def.modulatable = false;
        p.push_back(std::move(def));
    };

    add(params::choice("voice_glide_mode", "Glide Mode", "player", {"Always", "Legato"}, 0));
    add(params::choice("voice_glide_type", "Glide Type", "player", {"Time", "Rate"}, 0));
    add(params::choice("voice_mono_priority", "Mono Priority", "player", {"Last", "Lowest", "Highest"}, 0));
    add(params::floating("tune_amount", "Tuning Amount", "tuning", 0.0f, 1.0f, 1.0f, "", 1.0f, Smoothing::None));
    add(params::toggle("mpe_on", "MPE", "tuning", false));
    add(params::integer("mpe_bend_range", "MPE Bend Range", "tuning", 1, 96, 48, "st"));

    return d;
}

} // namespace crucible::modules
