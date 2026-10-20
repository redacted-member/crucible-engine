// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "../oscillator/OscSupport.h"

#include <algorithm>
#include <string>

// Per-slot oscillator selection: oscN_type chooses which registered oscillator module drives the slot (0 =
// wavetable). Registered after the oscillator modules so the choice list can name them.

namespace crucible::modules
{

const std::vector<const char*>& builtInOscillatorIds()
{
    static const std::vector<const char*> ids = {"osc.wavetable", "osc.sample", "osc.cloud",    "osc.noise",   "osc.sub",
                                                 "osc.analog",    "osc.modal",  "osc.harmonic", "osc.spectral"};
    return ids;
}

ModuleDescriptor oscillatorSlots(const ModuleRegistry& registry, const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "voice.oscillators";
    d.name = "Oscillator Slots";
    d.kind = ModuleKind::Voice;
    d.category = "Voice";
    d.cpu = CpuClass::Light;
    d.description = "Oscillator type per slot.";
    d.perOscillator = true;
    // Type indices are global: every built-in type keeps its number in every product. A type the product omits stays in the
    // list, flagged unavailable (ParamDef::unavailable); a slot set to it is silent.
    const std::vector<std::string> names = {
        wavetableOscillator().name,  sampleOscillator(shape).name, cloudOscillator(shape).name,
        noiseOscillator(shape).name, subOscillator(shape).name,    analogOscillatorName(),
        modalOscillatorName(),       harmonicOscillatorName(),     spectralOscillatorName()};
    const auto& omit = registry.layout().omit;
    std::vector<int> unavailable;
    const auto& ids = builtInOscillatorIds();
    for (size_t t = 0; t < ids.size(); ++t)
        if (std::find(omit.begin(), omit.end(), std::string(ids[t])) != omit.end())
            unavailable.push_back(static_cast<int>(t));
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto page = addSlotPage(d, slot, "type", "Type / Cross Mod",
                                      "LCD shows the oscillator type and the cross modulation of the slot.");
        const auto label = slotLabel(slot);
        d.params.push_back(params::choice(slotId(slot, "type"), label + " Type", page, names, 0));
        d.params.back().unavailable = unavailable;
        // Cross modulation from an earlier slot: FM and ring modulation by its output, or hard sync to its
        // fundamental. Slots can only listen to slots before them.
        d.params.push_back(
            params::choice(slotId(slot, "xmod_mode"), label + " Cross Mod", page, {"Off", "FM", "Ring", "Sync"}, 0));
        std::vector<std::string> sources;
        for (int o = 1; o <= shape.oscillators; ++o)
            sources.push_back("Osc " + std::to_string(o));
        d.params.push_back(params::choice(slotId(slot, "xmod_src"), label + " Cross Mod Source", page, sources, 0));
        d.params.push_back(
            voiceMod(params::floating(slotId(slot, "xmod_amt"), label + " Cross Mod Amount", page, 0.0f, 1.0f, 0.0f)));
    }
    return d;
}

} // namespace crucible::modules
