// SPDX-License-Identifier: MIT
#pragma once

// Runs the analog filter models (FilterModel 17 and up) of many voices four lanes at a time. A voice has two filter
// channels (left and right), so a pair of voices fills one four-lane object: voice 2p owns lanes 0 and 1 of quad p,
// voice 2p + 1 owns lanes 2 and 3. The assignment is fixed, so a lane keeps its state from chunk to chunk and nothing is
// copied in or out; a voice that starts (or whose filter model changes) resets just its own two lanes.
//
// Per control chunk and filter slot, SynthEngine drives the bank in this order, for all sounding voices:
//   beginChunk(); each voice: resetVoice() when needed, configure(), then writes its filter input into lane();
//   process(); each voice reads its filtered signal back from lane().
// A voice that does not sound leaves its lanes on silence. Threading: prepare() allocates and belongs to the control
// thread; everything else is allocation free and runs on the audio thread.

#include <crucible/Filters.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace crucible
{

class FilterBank
{
public:
    static constexpr int kModels = FilterChannel::kAnalogModels;
    static constexpr int kBlock = 32; // longest block passed to process() (one voice chunk)

    // Control thread. voices: the size of the voice pool; slots: filter slots per voice.
    void prepare(double sampleRate, Quality quality, int voices, int slots, std::uint32_t seed);
    bool ready() const { return slotCount > 0; }
    int slots() const { return slotCount; }

    // Audio thread. model is the analog index: FilterModel value minus FilterModel::TransistorLadder.
    void beginChunk();
    // The state a fresh voice (or a model change) gives the two lanes of the voice.
    void resetVoice(int slot, int model, int voice);
    // Component draws of the voice's two lanes: with a card seed, the values FilterChannel gives a mono filter of this
    // model under that seed (channel c: seed + 16 c + model); without, the draws of the quad seed given to prepare().
    void seedVoice(int slot, int model, int voice, bool useCard, std::uint32_t cardSeed);
    // Controls of the voice's lanes for this chunk, same arguments and clamping as FilterChannel::configure; marks the
    // voice as feeding the slot.
    void configure(int slot, int model, int voice, const FilterSettings& s, float drive);
    // Input and output block (in place) of one channel of a voice; kBlock floats.
    float* lane(int slot, int voice, int channel)
    {
        return buffers.data() + (static_cast<size_t>(slot) * paddedVoices + static_cast<size_t>(voice)) * 2 * kBlock +
               static_cast<size_t>(channel) * kBlock;
    }
    // Audio-rate cutoff modulation of a voice's lanes for this chunk: n values in octaves, applied on top of the cutoff of
    // configure() (see AnalogFilter::process). Call after configure(); a voice without it runs as before.
    void setCutoffMod(int slot, int voice, const float* octaves, int n);
    // Filters n samples of every quad that has a voice feeding the slot.
    void process(int slot, int model, int n);
    // Quad blocks filtered since prepare(); read after rendering on the rendering thread (tests and benchmarks use it to
    // see that the batched path ran).
    std::uint64_t processedQuadBlocks() const { return quadBlocks; }

private:
    size_t quadIndex(int slot, int model, int pair) const
    {
        return (static_cast<size_t>(slot) * kModels + static_cast<size_t>(model)) * static_cast<size_t>(pairs) +
               static_cast<size_t>(pair);
    }

    double rate = 48000.0;
    std::uint32_t baseSeed = 0;
    int slotCount = 0;
    int voiceCount = 0;
    int pairs = 0;
    int paddedVoices = 0;
    std::uint64_t quadBlocks = 0;
    std::vector<std::unique_ptr<AnalogFilter>> quads;
    std::vector<int> highpassStep;     // last high-pass step handed to each quad
    std::vector<float> buffers;        // [slot][voice][channel][kBlock]
    std::vector<unsigned char> fed;    // [slot][voice]: the voice configured the slot in this chunk
    std::vector<float> cutoffMod;      // [slot][voice][kBlock]: per-sample cutoff modulation in octaves
    std::vector<unsigned char> modFed; // [slot][voice]: cutoffMod holds this chunk's values
};

} // namespace crucible
