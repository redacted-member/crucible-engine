// SPDX-License-Identifier: MIT
#pragma once

// The node interface of the voice graph and the registry of node types. A product defines its voice as nodes and
// edges (crucible/graph/Product.h); the compiler turns the definition into a schedule and the executor runs one
// node object per node and voice, a chunk of at most kChunkSize samples at a time, on the audio thread.
//
// Audio travels as stereo buffers (AudioBuffer; a mono port has no right channel). Modulation travels as one value
// per chunk and port. Parameters arrive as local indices: the node reads them from the product's parameter table
// through NodeIO::param(), already smoothed.

#include <crucible/Voice.h>
#include <crucible/VoiceCard.h>
#include <crucible/module/Module.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace crucible::graph
{

// Limits of a product definition (configurable by the host through ProductLimits).
constexpr int kMaxNodesPerVoice = 32;
constexpr int kMaxFxInstances = 64;
constexpr int kMaxLayers = 16;
constexpr int kMaxModulators = 64;
constexpr int kMaxNodePorts = 8;

struct AudioBuffer
{
    float* left = nullptr;
    float* right = nullptr; // null for a mono port
};

// Everything a node needs for one chunk.
struct NodeIO
{
    const AudioBuffer* in = nullptr;
    int numIn = 0;
    AudioBuffer* out = nullptr;
    int numOut = 0;
    const float* modIn = nullptr; // one summed value per modulation input port
    int numModIn = 0;
    float* modOut = nullptr; // one value per modulation output port, written by the node
    int numModOut = 0;
    // The product's parameter values (natural units, smoothed) and this node's local -> global index table.
    const float* values = nullptr;
    const int* paramIndex = nullptr;
    float note = 60.0f;    // pitch of the voice in MIDI note units, bend included
    float velocity = 1.0f; // 0..1
    unsigned modUsed = 0;  // bit p set: modulation input port p has at least one edge
    // Audio-rate modulation inputs (PortType::AudioRate), one mono buffer of numSamples per port: the sum of the audio
    // outputs wired to it (each mixed down to mono and scaled by its edge amount). Zero when nothing is wired.
    const float* const* rateIn = nullptr;
    int numRateIn = 0;
    unsigned rateUsed = 0; // bit p set: audio-rate input port p has at least one edge
    // The engine's shared render data (pinned samples, clouds, zones per oscillator slot); may be null.
    const VoiceParams* shared = nullptr;
    // The voice card of the voice this node belongs to and the amounts of its tolerances; null when the product runs
    // without voice cards (every node then behaves as before). See VoiceCard.h.
    const VoiceCard* card = nullptr;
    CardAmounts cardAmounts;
    float param(int local) const { return values[paramIndex[local]]; }
    float mod(int port) const { return port < numModIn ? modIn[port] : 0.0f; }
    bool modConnected(int port) const { return (modUsed >> port & 1U) != 0U; }
    // Audio-rate input `port` (null when the node has no such port).
    const float* rate(int port) const { return port < numRateIn ? rateIn[port] : nullptr; }
    bool rateConnected(int port) const { return (rateUsed >> port & 1U) != 0U; }
};

struct NodePrepare
{
    double sampleRate = 48000.0;
    int maxBlock = kChunkSize;
    Quality quality = Quality::Standard;
    OversamplingFilter oversampling = OversamplingFilter::HalfBand;
    // Differs per node instance (layer, voice and node position mixed into the product's seed); a node that draws
    // randomness at prepare time (component tolerances) seeds from it. Same product and seed give the same values.
    std::uint32_t seed = 1;
    // Position of the node in its layer's node list: a node that wants its own entry of the voice card (envelopes,
    // filters) takes it from here.
    int index = 0;
    // The card of the voice the node object belongs to (valid for the node's life); null when the product has none.
    const VoiceCard* card = nullptr;
    // The program batches the node across voice pairs (pairsVoices / processPair): voice 2p is the lead of the pair
    // (2p, 2p + 1) and owns the shared state.
    bool batchPairs = false;
    bool pairLead = false;
};

struct NodeNote
{
    float note = 60.0f;
    float velocity = 1.0f;
    std::uint32_t seed = 1;
    // The layer ignores note-off: envelopes finish by themselves (a drum hit).
    bool oneShot = false;
    // The voice's card and the amounts at note-on (null / zero: no tolerances).
    const VoiceCard* card = nullptr;
    CardAmounts cardAmounts;
};

class Node
{
public:
    virtual ~Node() = default;
    // Control thread: allocate everything the node needs.
    virtual void prepare(const NodePrepare&) {}
    virtual void reset() {}
    // Audio thread from here on; never allocate, lock or wait.
    virtual void noteOn(const NodeNote&) {}
    virtual void noteOff() {}
    // True while the node keeps the voice alive (an envelope that has not finished). A voice ends when none does.
    virtual bool holdsVoice() const { return false; }
    // Renders numSamples (<= kChunkSize).
    virtual void process(NodeIO& io, int numSamples) = 0;
    // Optional: processes the same node of four voices together (SIMD). peers[i] and io[i] belong to voice i. Return
    // false to have the executor call process() for each voice.
    virtual bool processQuad(Node* const peers[4], NodeIO* const io[4], int numSamples)
    {
        (void)peers;
        (void)io;
        (void)numSamples;
        return false;
    }
    // Optional: the node filters the voices 2p and 2p + 1 of a layer together. True when the executor should call
    // processPair() on the node of voice 2p instead of process() on each voice.
    virtual bool pairsVoices() const { return false; }
    // Called on the node of voice 2p. ioA belongs to that voice and is null when the voice does not sound; nodeB and ioB are
    // the node and io of voice 2p + 1 (null when it does not sound or the layer has an odd number of voices). Return
    // false to have the executor call process() for each sounding voice.
    virtual bool processPair(NodeIO* ioA, Node* nodeB, NodeIO* ioB, int numSamples)
    {
        (void)ioA;
        (void)nodeB;
        (void)ioB;
        (void)numSamples;
        return false;
    }
    // Samples of delay the node adds to its audio path.
    virtual int latency() const { return 0; }
};

// What the factory of a node type is given.
struct NodeCreate
{
    const ParameterModel* model = nullptr;
    std::string instanceId;
    int slot = -1;                                // oscillator slot (0-based) for the oscillator wrappers
    const ModuleDescriptor* oscillator = nullptr; // the wrapped oscillator module
    std::vector<int> paramIndex;                  // global parameter index per local parameter
};

// A node type: parameters with local ids (the product turns them into "<instance>_<local>"), ports and a factory.
struct NodeType
{
    std::string type; // "env.adsr"
    std::string name;
    ModuleKind kind = ModuleKind::Modulator;
    std::string category;
    std::string description;
    int version = 1;
    std::vector<ParamDef> params; // ids are local ("attack")
    std::vector<PortDef> inputs;
    std::vector<PortDef> outputs;
    // Wrappers of an oscillator slot: the node uses the existing "oscN_..." parameters of the product's voice shape.
    bool usesSlot = false;
    // Modulators: where the node's values live. "voice" (per voice) and "layer" are supported.
    bool voiceScope = true;
    std::function<std::unique_ptr<Node>(const NodeCreate&)> create;
};

// The node types of a build. The built-in types are available through builtIn(); hosts that add types make their own
// registry (copy of the built-ins plus add()) and pass it to the compiler.
class NodeRegistry
{
public:
    static const NodeRegistry& builtIn();

    void add(NodeType type);
    const NodeType* find(const std::string& type) const;
    const std::vector<NodeType>& types() const { return list; }

    // Effect modules that products can instantiate in their FX graph next to the built-in effects (the module SDK adds
    // its effects here). The descriptor's parameter ids are final ("acme_tremolo_rate") and it carries its own
    // createEffect factory; a product's first instance of the type registers it as it is, further instances as copies.
    void addEffect(ModuleDescriptor effect);
    const ModuleDescriptor* findEffect(const std::string& id) const;
    const std::vector<ModuleDescriptor>& effects() const { return effectList; }

private:
    std::vector<NodeType> list;
    std::vector<ModuleDescriptor> effectList;
};

// Index of a port by name, or -1.
int portIndex(const std::vector<PortDef>& ports, const std::string& name);

} // namespace crucible::graph
