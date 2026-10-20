// SPDX-License-Identifier: MIT
#include "Modules.h"

#include <crucible/sdk/Host.h>

#include <algorithm>

namespace crucible
{

// The built-in modules, one line each. Order matters: it is the order of the parameter table and, for effects,
// the tie-break order of the FX chain. Append new modules at the end of their group.
void registerBuiltInModules(ModuleRegistry& r)
{
    using namespace modules;
    const ProductLayout layout = r.layout();
    // Registers a module unless the product leaves it out (graph products drop the effects they do not use). Every
    // module, built-in or third-party, enters the registry through sdk::registerModule.
    const auto add = [&r, &layout](ModuleDescriptor d)
    {
        // Retirements: a module that a newer one replaces stays registered for existing patches, marked Legacy with the
        // id of its successor (empty at present: the first-generation effects were removed before v1).
        static const std::vector<std::pair<std::string, std::string>> kSuperseded;
        for (const auto& l : kSuperseded)
            if (d.id == l.first)
            {
                d.status = ModuleStatus::Legacy;
                d.supersededBy = l.second;
            }
        if (std::find(layout.omit.begin(), layout.omit.end(), d.id) == layout.omit.end())
            sdk::registerModule(r, std::move(d));
    };

    add(wavetableVoice(layout.voice, layout.macros));
    add(performanceControls(layout.macros));
    add(scenesAndSequencer());

    add(fx::limiterEffect());
    add(fx::crusherEffect());
    add(fx::widthEffect());
    add(fx::filterFxEffect());
    add(fx::multibandEffect());
    add(fx::tranceGateEffect());
    add(fx::tremoloEffect());
    add(fx::ringShiftEffect());

    add(arpeggiator());
    add(playerControls());
    add(keyLockControls());

    add(wavetableOscillator());
    add(sampleOscillator(layout.voice));
    add(cloudOscillator(layout.voice));
    add(noiseOscillator(layout.voice));
    add(subOscillator(layout.voice));
    add(oscillatorSlots(r, layout.voice));

    // Registered after everything else so that the parameter indices (and the generic modulation destination
    // numbers derived from them) of existing patches do not move. Its default chain slot is the last one.
    add(fx::vocoderEffect());
    add(cloudBands(layout.voice));
    add(vectorPad());
    add(unisonModulation(layout.voice));
    add(analogOscillator(layout.voice));
    add(fx::compCleanEffect());
    add(fx::compVcaEffect());
    add(fx::gate2Effect());
    add(analogFilterControls(layout.voice));
    add(modalOscillator(layout.voice));
    add(harmonicOscillator(layout.voice));
    add(spectralOscillator(layout.voice));
    add(spectralWavetable(layout.voice));
    add(analogVoice(layout.voice));
    add(fx::maximizerEffect());
    add(fx::clipperEffect());
    add(fx::meterEffect());
    add(modulatorBank(layout.voice));
    add(audioRateModulation(layout.voice));
    add(fx::eqProEffect());
    add(fx::reverbAlgoEffect());
    add(fx::convolutionEffect());
    add(fx::delayProEffect());
    add(fx::transientEffect());
    add(fx::deEsserEffect());
    add(fx::ensembleEffect());
    add(fx::phaser2Effect());
    add(fx::flanger2Effect());
    add(fx::exciterEffect());
    add(fx::saturatorEffect());
    add(fx::shaperEffect());
    add(fx::multibandDriveEffect());
    add(fx::cabinetEffect());
}

} // namespace crucible
