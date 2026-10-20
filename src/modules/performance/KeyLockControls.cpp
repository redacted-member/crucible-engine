// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/KeyLock.h>
#include <crucible/module/ParamBuilder.h>

namespace crucible::modules
{

// Scale lock: root, scale and mode of the note quantiser (see KeyLock.h) and the Key Sync role on the key bus.
ModuleDescriptor keyLockControls()
{
    ModuleDescriptor d;
    d.id = "perf.keylock";
    d.name = "Key Lock";
    d.kind = ModuleKind::Performance;
    d.category = "Performance";
    auto& p = d.params;
    const auto add = [&p](ParamDef def)
    {
        def.scene = false;
        p.push_back(std::move(def));
    };

    std::vector<std::string> scales;
    for (int i = 0; i < kNumKeyScales; ++i)
        scales.push_back(keyScaleName(i));
    add(params::choice("key_mode", "Key Lock", "keylock", {"Off", "Snap", "Block", "Remap"}, 0));
    add(params::choice("key_root", "Key Root", "keylock", {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"},
                       0));
    add(params::choice("key_scale", "Key Scale", "keylock", scales, 1));
    add(params::integer("key_mask", "Custom Scale", "keylock", 0, 4095, 0x0AB5));
    add(params::choice("key_tie", "Snap Tie", "keylock", {"Down", "Up"}, 0));
    add(params::choice("key_sync", "Key Sync", "keylock", {"Off", "Follow", "Lead"}, 0));

    return d;
}

} // namespace crucible::modules
