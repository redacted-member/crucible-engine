// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/ModMatrix.h>
#include <crucible/module/ParamBuilder.h>

#include <algorithm>

namespace crucible::modules
{

// Macros, the touch strips (modulation sources) and the clip-indicator hold time.
ModuleDescriptor performanceControls(int macros)
{
    ModuleDescriptor d;
    d.id = "perf.controls";
    d.name = "Performance Controls";
    d.kind = ModuleKind::Performance;
    d.category = "Performance";
    auto& p = d.params;

    for (int m = 1; m <= std::min(macros, kDefaultMacros); ++m)
        p.push_back(params::floating("macro" + std::to_string(m), "Macro " + std::to_string(m), "macros", 0.0f, 1.0f, 0.0f));
    {
        auto hold =
            params::floating("meter_hold", "Clip Hold", "controllers", 0.0f, 10.0f, 2.0f, "s", 1.0f, Smoothing::None);
        hold.scene = false;
        p.push_back(hold);
        auto strip = params::floating("strip_l", "Strip L", "controllers", 0.0f, 1.0f, 0.0f);
        strip.scene = false;
        p.push_back(strip);
        strip = params::floating("strip_r", "Strip R", "controllers", 0.0f, 1.0f, 0.0f);
        strip.scene = false;
        p.push_back(strip);
    }
    // Macros 7 and 8 come last so that the ids and the order of everything before them never change.
    for (int m = kDefaultMacros + 1; m <= macros; ++m)
        p.push_back(
            params::floating("macro" + std::to_string(m), "Macro " + std::to_string(m), "macros_2", 0.0f, 1.0f, 0.0f));
    if (macros > kDefaultMacros)
        d.pages.push_back(
            {"macros_2", "Macros 2", "mod", "Macros 7-8", "LCD shows macro positions 7 and 8; strips set them."});

    return d;
}

// Scene slots, the A / B morph and the step sequencer's transport and pattern settings.
ModuleDescriptor scenesAndSequencer()
{
    ModuleDescriptor d;
    d.id = "seq.step";
    d.name = "Scenes and Step Sequencer";
    d.kind = ModuleKind::Sequencer;
    d.category = "Sequencer";
    auto& p = d.params;

    {
        const std::vector<std::string> slotNames = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"};
        const auto add = [&p](ParamDef d)
        {
            d.scene = false;
            p.push_back(std::move(d));
        };
        add(params::choice("scene_a", "Scene A", "scenes", slotNames, 0));
        add(params::choice("scene_b", "Scene B", "scenes", slotNames, 1));
        auto morph = params::floating("scene_morph", "Scene Morph", "scenes", 0.0f, 1.0f, 0.0f);
        add(morph);
        add(params::choice("scene_slot", "Scene Slot", "scenes", slotNames, 0));

        add(params::toggle("seq_run", "Seq Run", "seq_1", false));
        add(params::choice("seq_sync", "Seq Sync", "seq_1", {"Internal", "Host"}, 0));
        add(params::floating("seq_tempo", "Seq Tempo", "seq_1", 20.0f, 300.0f, 120.0f, "BPM", 1.0f, Smoothing::None));
        add(params::choice("seq_rate", "Seq Rate", "seq_1", {"1/16", "1/8", "1/32", "1/16T", "1/8T", "1/32T"}, 0));
        add(params::choice("seq_dir", "Seq Direction", "seq_1", {"Forward", "Reverse", "Ping-Pong", "Random"}, 0));
        add(params::floating("seq_swing", "Seq Swing", "seq_1", 0.0f, 75.0f, 0.0f, "%", 1.0f, Smoothing::None));
        add(params::integer("seq_transpose", "Seq Transpose", "seq_2", -24, 24, 0, "st"));
        add(params::choice("seq_pattern", "Seq Pattern", "seq_2", slotNames, 0));
        add(params::integer("seq_length", "Seq Length", "seq_2", 1, 16, 1, "bars"));
        add(params::choice("seq_chain", "Seq Chain", "seq_2", {"Off", "1-2", "1-3", "1-4", "1-6", "1-8", "1-12"}, 0));
        add(params::toggle("seq_rec", "Seq Record", "seq_2", false));
        // Version 2: Keys plays the pattern only while keys are held (from step 1 at the first key, transposed by the
        // newest held key relative to C4); the keys themselves do not sound. Appended (sequence 4): the frozen order holds.
        auto trigger = params::choice("seq_trigger", "Seq Trigger", "seq_2", {"Free", "Keys"}, 0);
        trigger.since = 2;
        trigger.appended = 4;
        add(trigger);
    }
    d.version = 2;

    return d;
}

} // namespace crucible::modules
