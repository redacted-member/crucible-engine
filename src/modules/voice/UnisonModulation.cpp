// SPDX-License-Identifier: MIT
#include "../Modules.h"

// Makes the continuous unison parameters of every oscillator slot per-voice modulation targets. The parameters stay
// where they are declared (the wavetable voice, the sample oscillator); this module only lists them, and the
// registry numbers them after all other targets so that the destinations of earlier parameters keep their numbers.

namespace crucible::modules
{

ModuleDescriptor unisonModulation(const VoiceGraphShape& shape)
{
    ModuleDescriptor d;
    d.id = "voice.unison_mod";
    d.name = "Unison Modulation";
    d.kind = ModuleKind::Voice;
    d.category = "Oscillator";
    d.cpu = CpuClass::Light;
    d.description = "Detune, spread, blend and scatter of the unison stacks as per-voice modulation targets.";
    d.perOscillator = true;
    for (int slot = 1; slot <= shape.oscillators; ++slot)
    {
        const auto p = "osc" + std::to_string(slot);
        // Stack detune of slot 1 already has a fixed matrix destination (ModDest::UnisonDetune).
        if (slot > 1)
            d.lateVoiceMod.push_back(p + "_uni_detune");
        for (const char* suffix : {"_uni_spread", "_uni_blend", "_uni_scatter", "_smp_uni_detune", "_smp_uni_spread"})
            d.lateVoiceMod.push_back(p + suffix);
    }
    return d;
}

} // namespace crucible::modules
