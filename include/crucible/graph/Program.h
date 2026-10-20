// SPDX-License-Identifier: MIT
#pragma once

// The runtime of a compiled product: a Program owns the node objects of every layer voice, the buffers they use and
// the FX plan, and runs them on the audio thread. Programs are built on the control thread and handed to the audio
// thread through ProgramHost (RtShared underneath), so a product can be reconfigured while audio runs without locks
// or allocation on the audio side. See crucible/graph/Compiler.h for how a product is compiled.

#include <crucible/RtShared.h>
#include <crucible/graph/Compiler.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace crucible::graph
{

// What the classic voice node calls back into: the engine's own voice pool (allocation, glide, mono modes, MPE and
// the Voice objects stay where they are; the node is the graph's handle on them).
class ClassicVoiceHost
{
public:
    virtual ~ClassicVoiceHost() = default;
    // Adds one chunk (n <= kChunkSize) of every voice to left / right.
    virtual void renderClassicChunk(float* left, float* right, int n) = 0;
};

// classic.voice: today's whole Voice (wavetable slots with warps and unison, the module oscillators, the filters, the
// four envelopes, the four LFOs, the matrix) as one node. No DSP of its own.
class ClassicVoiceNode final : public Node
{
public:
    void bind(ClassicVoiceHost* h) { host = h; }
    void process(NodeIO& io, int numSamples) override
    {
        if (host != nullptr && io.numOut > 0)
            host->renderClassicChunk(io.out[0].left, io.out[0].right, numSamples);
    }

private:
    ClassicVoiceHost* host = nullptr;
};

class Program
{
public:
    // Control thread. Builds the node objects for every layer voice and resolves the FX plan against the rack.
    // ok() is false (and error() says why) when the product does not fit the engine's rack.
    Program(std::shared_ptr<const CompiledProduct> product, const ModuleRack& rack, const ModuleContext& context,
            std::uint32_t seed, ClassicVoiceHost* classicHost);
    ~Program();
    Program(const Program&) = delete;
    Program& operator=(const Program&) = delete;

    bool ok() const { return failure.empty(); }
    const std::string& error() const { return failure; }
    const CompiledProduct& product() const { return *compiled; }
    VoiceKind voiceKind() const { return compiled->voiceKind(); }
    const FxPlan& fxPlan() const { return plan; }
    // The classic voice node (null unless the product's voice is classic).
    ClassicVoiceNode* classicNode() { return classic; }

    // ---- Audio thread.
    void noteOn(int note, float velocity);
    void noteOff(int note);
    void allNotesOff();
    // Stops every voice at once (the output is silent at this point).
    void killAll();
    // Adds n (<= kChunkSize) samples of every layer voice to left / right. values: the product's parameter values.
    // shared: pinned samples and clouds of the engine (may be null). bend: pitch bend in semitones.
    void render(float* left, float* right, int n, const float* values, const VoiceParams* shared, float bend);
    // Amounts of the voice cards' tolerances (voice.analog); all zero: the nodes run without tolerances. Audio thread.
    void setCardAmounts(const CardAmounts& amounts) { cardAmounts = amounts; }
    // Pairs of voices whose filter.analog nodes ran as one four-lane block since the program was built; audio thread.
    std::uint64_t quadFilterBlocks() const { return pairBlocks; }
    int activeVoices() const;
    // Voices sounding in one layer (by index in the product's layer list).
    int activeVoices(int layer) const;
    int numLayers() const { return static_cast<int>(layerRt.size()); }

private:
    struct NodeRt
    {
        std::unique_ptr<Node> node;
        const NodePlan* plan = nullptr;
        std::vector<AudioBuffer> in, out;
        std::vector<float> modIn, modOut;
        std::vector<float> scratch; // sums of inputs with several sources
        std::vector<float*> rate;   // one mono buffer per audio-rate input port (sums of the wired audio outputs)
        std::vector<const float*> rateView;
        NodeIO io;
    };
    struct VoiceRt
    {
        std::vector<NodeRt> nodes;
        std::vector<float> arena;
        bool active = false;
        bool released = false;
        bool canHold = false;
        bool choking = false;
        int chokePos = 0;
        int note = 0;
        float velocity = 0.0f;
        std::uint64_t age = 0;
        VoiceCard card; // seed + layer + voice index; fixed for the life of the program
    };
    struct LayerRt
    {
        const LayerPlan* plan = nullptr;
        std::vector<VoiceRt> voices;
    };

    void build(const ModuleContext& context, std::uint32_t seed);
    void triggerLayer(LayerRt& layer, int note, float velocity);
    void renderLayer(LayerRt& layer, float* left, float* right, int n, const float* values, const VoiceParams* shared,
                     float bend);

    std::shared_ptr<const CompiledProduct> compiled;
    std::string failure;
    FxPlan plan;
    ClassicVoiceNode* classic = nullptr;
    std::uint64_t pairBlocks = 0;
    std::unique_ptr<Node> classicOwner;
    std::vector<LayerRt> layerRt;
    std::vector<float> zeros;
    std::array<std::uint64_t, 16> roundRobinCount{};
    std::uint64_t counter = 0;
    std::uint32_t baseSeed = 1;
    CardAmounts cardAmounts;
    int chokeSamples = 240;
    double rate = 48000.0;
};

// Publishes programs to one audio thread and keeps them alive for as long as it can use them (the active program,
// the one being faded to, and any it has just picked up). The control thread calls publish() and collect(); the audio
// thread calls poll() and tells the host what it holds. Nothing here locks or allocates on the audio side.
class ProgramHost
{
public:
    ProgramHost();
    ~ProgramHost();
    ProgramHost(const ProgramHost&) = delete;
    ProgramHost& operator=(const ProgramHost&) = delete;

    // Control thread. Takes ownership; the returned pointer stays valid until collect() finds it unused.
    Program* publish(std::unique_ptr<Program> program);
    // Control thread: frees the programs that are neither the newest nor held by the audio thread.
    void collect();
    // Control thread: the newest published program (or null).
    Program* newest() const;
    int liveCount() const;

    // Audio thread: the newest published program. It is protected until the caller records it with hold().
    Program* poll() noexcept;
    // Audio thread: records the programs in use: slot 0 the active one, slot 1 the one being faded to. Also ends the
    // protection poll() gave.
    void hold(int slot, Program* program) noexcept;
    void endPoll() noexcept { candidate.store(nullptr, std::memory_order_seq_cst); }

private:
    struct Handle
    {
        Program* program = nullptr;
    };
    RtShared<Handle> shared;
    mutable std::mutex mutex;
    std::vector<std::unique_ptr<Program>> programs;
    Program* latest = nullptr;
    std::atomic<Program*> held[2];
    std::atomic<Program*> candidate{nullptr};
};

} // namespace crucible::graph
