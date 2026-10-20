// SPDX-License-Identifier: MIT
#pragma once

// Factory functions of the built-in modules. Each lives in its own file under src/modules/<kind>/ and is
// registered by one line in Modules.cpp.

#include <crucible/module/Module.h>

namespace crucible::modules
{

ProductLayout defaultLayout();

ModuleDescriptor wavetableVoice(const VoiceGraphShape& shape, int macros);
ModuleDescriptor performanceControls(int macros);
ModuleDescriptor playerControls();
ModuleDescriptor keyLockControls();
ModuleDescriptor vectorPad();
ModuleDescriptor scenesAndSequencer();
ModuleDescriptor arpeggiator();

// Oscillator types (src/modules/oscillator) and the per-slot type selection (src/modules/voice).
ModuleDescriptor wavetableOscillator();
ModuleDescriptor sampleOscillator(const VoiceGraphShape& shape);
ModuleDescriptor cloudOscillator(const VoiceGraphShape& shape);
ModuleDescriptor cloudBands(const VoiceGraphShape& shape);
ModuleDescriptor unisonModulation(const VoiceGraphShape& shape);
ModuleDescriptor noiseOscillator(const VoiceGraphShape& shape);
ModuleDescriptor subOscillator(const VoiceGraphShape& shape);
ModuleDescriptor analogOscillator(const VoiceGraphShape& shape);
ModuleDescriptor analogFilterControls(const VoiceGraphShape& shape);
ModuleDescriptor analogVoice(const VoiceGraphShape& shape);
ModuleDescriptor modulatorBank(const VoiceGraphShape& shape);
ModuleDescriptor audioRateModulation(const VoiceGraphShape& shape);
// Name of the analog oscillator. It is registered after the other modules (so that no parameter index moves) but is an
// oscillator type like the others: the type choice of every slot lists it last.
const char* analogOscillatorName();

// P13 oscillator types, registered after the analog one in this order (type indices 6, 7 and 8); the type choice of every
// slot lists them after the analog oscillator.
ModuleDescriptor modalOscillator(const VoiceGraphShape& shape);
ModuleDescriptor harmonicOscillator(const VoiceGraphShape& shape);
ModuleDescriptor spectralOscillator(const VoiceGraphShape& shape);
ModuleDescriptor spectralWavetable(const VoiceGraphShape& shape);
const char* modalOscillatorName();
const char* harmonicOscillatorName();
const char* spectralOscillatorName();
ModuleDescriptor oscillatorSlots(const ModuleRegistry& registry, const VoiceGraphShape& shape);
// Module ids of the built-in oscillator types in type-index order (the type choice of every slot lists them in this order,
// whether or not a product omits them).
const std::vector<const char*>& builtInOscillatorIds();

namespace fx
{
ModuleDescriptor limiterEffect();
ModuleDescriptor crusherEffect();
ModuleDescriptor widthEffect();
ModuleDescriptor filterFxEffect();
ModuleDescriptor multibandEffect();
ModuleDescriptor tranceGateEffect();
ModuleDescriptor tremoloEffect();
ModuleDescriptor ringShiftEffect();
ModuleDescriptor vocoderEffect();   // module-only: no FxRack counterpart, registered last
ModuleDescriptor compCleanEffect(); // FX3 dynamics, module-only, registered after everything else
ModuleDescriptor compVcaEffect();
ModuleDescriptor gate2Effect();
ModuleDescriptor maximizerEffect(); // FX4: true-peak maximiser, clipper and meter, module-only, registered last
ModuleDescriptor clipperEffect();
ModuleDescriptor meterEffect();
ModuleDescriptor eqProEffect();       // FX5 pro EQ, module-only, registered after everything else
ModuleDescriptor reverbAlgoEffect();  // FX6 algorithmic reverbs, module-only, registered after everything else
ModuleDescriptor convolutionEffect(); // FX7 convolution with true IR import, module-only, registered after the reverb
ModuleDescriptor delayProEffect();    // FX10 digital / analog delay, module-only, registered after the convolution
ModuleDescriptor transientEffect();   // FX13 transient shaper, module-only, registered after the delay
ModuleDescriptor deEsserEffect();     // FX13 de-esser, module-only, registered after the transient shaper
ModuleDescriptor ensembleEffect();    // FX11 BBD chorus / ensemble, module-only, registered after the de-esser
ModuleDescriptor phaser2Effect();     // FX11 all-pass phaser, module-only, registered after the ensemble
ModuleDescriptor flanger2Effect();    // FX11 through-zero flanger, module-only, registered after the phaser
ModuleDescriptor exciterEffect();     // FX13 harmonic exciter and bass enhancer, module-only, registered after the flanger
ModuleDescriptor saturatorEffect();   // FX13 saturator, module-only, registered after the flanger
ModuleDescriptor shaperEffect();      // drawn-curve waveshaper, module-only, registered last
ModuleDescriptor multibandDriveEffect(); // module-only, registered after the saturator
ModuleDescriptor cabinetEffect();     // procedural guitar / bass cabinet, module-only, registered last
} // namespace fx

} // namespace crucible::modules
