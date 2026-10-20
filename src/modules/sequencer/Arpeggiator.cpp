// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/Arpeggiator.h>
#include <crucible/module/ParamBuilder.h>

namespace crucible::modules
{

// Arpeggiator settings (the DSP is Arpeggiator, run by EngineCore in front of the voices).
ModuleDescriptor arpeggiator()
{
    ModuleDescriptor d;
    d.id = "seq.arp";
    d.name = "Arpeggiator";
    d.kind = ModuleKind::Sequencer;
    d.category = "Sequencer";
    d.description = "Eight modes, 1-4 octaves, synced or free rates, swing, latch, velocity modes, ratchets and a "
                    "32-step pattern grid.";
    auto& p = d.params;

    p.push_back(params::toggle("arp_on", "Arp On", "arp", false));
    p.push_back(params::choice("arp_mode", "Arp Mode", "arp", arpModeNames(), 0));
    p.push_back(params::choice("arp_rate", "Arp Rate", "arp", arpRateNames(), 10));
    p.push_back(params::integer("arp_octaves", "Arp Octaves", "arp", 1, 4, 1));
    p.push_back(params::floating("arp_gate", "Arp Gate", "arp", 0.05f, 1.5f, 0.5f, {}, 1.0f, Smoothing::None));
    p.push_back(params::toggle("arp_latch", "Arp Latch", "arp", false));
    p.push_back(params::choice("arp_sync", "Arp Clock", "arp_2", {"Internal", "Host"}, 1));
    p.push_back(params::floating("arp_tempo", "Arp Tempo", "arp_2", 20.0f, 300.0f, 120.0f, "BPM", 1.0f, Smoothing::None));
    p.push_back(params::floating("arp_free_rate", "Arp Free Rate", "arp_2", 0.1f, 50.0f, 8.0f, "Hz", 0.4f, Smoothing::None));
    p.push_back(params::floating("arp_swing", "Arp Swing", "arp_2", 0.0f, 75.0f, 0.0f, "%", 1.0f, Smoothing::None));
    p.push_back(params::integer("arp_ratchet", "Arp Ratchet", "arp_2", 1, 4, 1));
    p.push_back(params::integer("arp_transpose", "Arp Transpose", "arp_2", -24, 24, 0, "st"));
    p.push_back(params::choice("arp_vel_mode", "Arp Velocity", "arp_3", {"As Played", "Fixed", "Accent"}, 0));
    p.push_back(params::integer("arp_vel", "Arp Fixed Velocity", "arp_3", 1, 127, 100));
    p.push_back(params::floating("arp_accent", "Arp Accent", "arp_3", 0.0f, 1.0f, 0.3f, {}, 1.0f, Smoothing::None));
    p.push_back(params::toggle("arp_grid", "Arp Use Grid", "arp_3", false));
    p.push_back(params::integer("arp_grid_length", "Arp Grid Length", "arp_3", 1, kArpMaxSteps, 8));
    p.push_back(
        params::choice("arp_grid_dir", "Arp Grid Direction", "arp_3", {"Forward", "Reverse", "Ping-Pong", "Random"}, 0));
    return d;
}

} // namespace crucible::modules
