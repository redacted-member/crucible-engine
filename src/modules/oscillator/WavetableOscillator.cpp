// SPDX-License-Identifier: MIT
#include "../Modules.h"

// The wavetable oscillator is part of the voice (its parameters are declared by voice.wavetable and its DSP is
// Voice::renderOscillator). It is registered as the first oscillator type so type 0 of every oscN_type parameter
// has a module of its own in listings.

namespace crucible::modules
{

ModuleDescriptor wavetableOscillator()
{
    ModuleDescriptor d;
    d.id = "osc.wavetable";
    d.name = "Wavetable";
    d.kind = ModuleKind::Oscillator;
    d.category = "Wavetable";
    d.cpu = CpuClass::Heavy;
    d.description = "Band-limited wavetable oscillator with 15 warp modes and unison stacks (built into the voice).";
    return d;
}

} // namespace crucible::modules
