// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "../oscillator/OscSupport.h"

#include <crucible/Spectral.h>

// Opt-in spectral processing of the wavetable oscillator (P13): oscN_wt_interp selects how neighbouring frames of the
// table are blended (the legacy linear crossfade, or a sweep that moves the spectral peaks), oscN_wt_spec a spectral
// warp and oscN_wt_spec_amt its amount. The defaults (Crossfade, Off, 0) leave the legacy code path in charge. The
// parameters live in a module of their own, registered after all others, so that no parameter index moves; the voice
// reads them by id (see SynthEngine and Voice::spectralFrame).

namespace crucible::modules
{

ModuleDescriptor spectralWavetable(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "voice.spectral_wt";
    d.name = "Spectral Wavetable";
    d.kind = ModuleKind::Voice;
    d.category = "Oscillator";
    d.cpu = CpuClass::Light;
    d.description = "Spectral frame interpolation (peaks sweep in frequency instead of crossfading) and spectral warps "
                    "for the wavetable oscillator; opt in per oscillator.";
    d.perOscillator = true;
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto label = slotLabel(slot);
        const auto page =
            addSlotPage(d, slot, "wt_spec", "Wavetable Spectral", "LCD shows the spectrum of the current frame.");
        std::vector<std::string> warps;
        for (int w = 0; w < static_cast<int>(spectral::Warp::Count); ++w)
            warps.push_back(spectral::warpName(static_cast<spectral::Warp>(w)));
        d.params.push_back(params::choice(slotId(slot, "wt_interp"), label + " Wavetable Interpolation", page,
                                          {"Crossfade", "Spectral Sweep"}, 0));
        d.params.push_back(params::choice(slotId(slot, "wt_spec"), label + " Wavetable Spectral Warp", page, warps, 0));
        d.params.push_back(voiceMod(params::floating(slotId(slot, "wt_spec_amt"), label + " Wavetable Spectral Warp Amount",
                                                     page, -1.0f, 1.0f, 0.0f)));
    }
    return d;
}

} // namespace crucible::modules
