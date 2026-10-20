// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/module/Module.h>

#include <cstdint>
#include <memory>
#include <optional>

namespace crucible
{

namespace graph
{
class CompiledProduct;
}

// Construction-time settings of an engine instance.
struct EngineConfig
{
    // Voice pool size, 1..64. The voice_poly parameter limits polyphony inside the pool; its maximum (16) means
    // "the whole pool".
    int maxVoices = 16;
    // Largest block Engine::process() hands to the core at once (larger blocks are split).
    int maxBlockSize = 512;
    // The product this engine instance plays. `compiled` is the one product input: the product as data
    // (crucible/graph/Compiler.h) with its registry, voice graph and FX graph. Most code never fills it by hand:
    // Engine::fromProduct(json, error, base) parses, compiles and sets it. A product with the classic voice and the
    // default FX chain sounds exactly like an engine built without one. Every engine owns what it reads: two engines
    // with different products in one process do not influence each other.
    std::shared_ptr<const graph::CompiledProduct> compiled;
    // Advanced, used only when `compiled` is empty: a finished module registry (ModuleRegistry::build(); several
    // engines may share one), or just a layout from which the engine builds its own registry of the built-in modules.
    // With neither, the engine builds the process default product (the layout chosen with the deprecated
    // ModuleRegistry::configureProduct, else the neutral one).
    std::shared_ptr<const ModuleRegistry> registry;
    std::optional<ProductLayout> product;
    // Active oscillator and filter slots, 0 = every slot of the product's voice graph shape (3 oscillators and 2
    // filters in the neutral product). Smaller values switch the higher slots off to save CPU.
    int numOscillators = 0;
    int numFilters = 0;
    // Quality / oversampling policy handed to modules (ModuleContext::quality).
    Quality quality = Quality::Standard;
    // Filter family of the oversamplers: Draft / Standard / High pick 2x / 4x / 8x with 23 / 47 / 95 FIR taps or
    // 4 / 6 / 8 all-pass sections in the drive, and 2x / unchanged / one stage more in the warp modes.
    OversamplingFilter oversampling = OversamplingFilter::HalfBand;
    // Mixed into every per-note random seed; 0 keeps the default sequence. Rendering is deterministic for a
    // given seed, configuration and input.
    std::uint32_t seed = 0;
    // Generate all factory wavetables (about 130 MB) when the engine is created, so that no table is ever generated
    // on the audio thread. By default tables are generated when an oscillator is first pointed at them (see
    // WavetableBank).
    bool preloadWavetables = false;
    // Run the nonlinear filter models (FilterModel 17 and up) of all sounding voices four lanes at a time (FilterBank.h).
    // The result equals the voice-by-voice path to float rounding; false renders every voice alone (reference for the
    // equivalence test, and a fallback).
    bool batchAnalogFilters = true;
};

} // namespace crucible
