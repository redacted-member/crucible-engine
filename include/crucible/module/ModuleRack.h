// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/graph/FxPlan.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/Module.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace crucible
{

// The FX graph executor of one product. Every registered Effect module is a slot (the product's instances: the
// first instance of a type is the built-in module, further instances are registered by the graph compiler as
// copies with their own parameter ids). A graph::FxPlan decides how the slots are wired: lanes (serial chains, in
// the order of their position parameters or as listed) that read the voice bus, the host input or other lanes, and
// sum into the master. Without a plan the rack runs the classic chain: one lane over all slots, ordered by their
// position parameters (ties keep registration order), each switched by its on parameter. Parameter indices are
// resolved once at construction; process() never allocates.
class ModuleRack
{
public:
    // The effects of the registry product; the registry must outlive the rack.
    explicit ModuleRack(const ModuleRegistry& registry);
    ~ModuleRack();
    ModuleRack(const ModuleRack&) = delete;
    ModuleRack& operator=(const ModuleRack&) = delete;

    // Control thread. Allocates the lane buffers (kMaxLanes of up to context.maxBlockSize samples) and prepares the
    // effects.
    void prepare(const ModuleContext& context);
    void reset();

    // Most lanes a plan may have.
    static constexpr int kMaxLanes = 16;
    // Wires the slots with a plan (audio thread, or control thread while no audio runs); nullptr restores the classic
    // chain. The plan must outlive its use. A plan is valid for this rack when every slot index exists, every
    // slot is in at most one lane and there are at most kMaxLanes lanes.
    bool validPlan(const graph::FxPlan& plan) const;
    void setPlan(const graph::FxPlan* plan) { active = plan; }
    const graph::FxPlan* plan() const { return active; }
    // Slot of a module (effect instance) by its descriptor id, or -1.
    int slotOf(const std::string& moduleId) const;

    // current: natural values indexed like ParameterMap::params(); legacyDests: kNumModDests entries;
    // genericDest: one entry per ParameterMap::modTargets() entry (may be null).
    // auxLeft / auxRight (both or neither; a mono input passes the same pointer twice) is the sidechain input that
    // effects such as the vocoder read through EffectInputs::auxLeft().
    // The note that effects read through EffectInputs::note() (MIDI note units; negative = none). Set by the engine
    // before process(); audio thread, no allocation.
    void setNote(float midiNote) { note = midiNote; }

    void process(float* left, float* right, int numSamples, const float* current, const float* legacyDests,
                 const float* genericDest, double bpm, const float* auxLeft = nullptr, const float* auxRight = nullptr);

    // Latency of the rack in samples. current: natural parameter values (indexed like ParameterMap::params()), which
    // decide which modules are on. A lane adds the LatencyReporter latencies of its enabled modules to the largest
    // latency among the lanes it reads (the voice bus and the host input add none); the rack takes the largest over
    // the lanes that feed the master. Control thread.
    int latencySamples(const float* current, const graph::FxPlan* plan = nullptr) const;
    // Longest tail in seconds among the enabled modules that implement LatencyReporter; HUGE_VAL when one has an
    // infinite tail (freeze). Audio thread, or control thread while no audio runs.
    double tailSeconds(const float* current) const;
    // The latency and tail of the last block (or of publishReported()), readable from any thread without touching the
    // plan, which the audio thread swaps.
    int reportedLatency() const { return pubLatency.load(std::memory_order_relaxed); }
    double reportedTail() const { return pubTail.load(std::memory_order_relaxed); }
    // Computes and publishes both for `current`. process() does it at the end of every block; call it from the control
    // thread only while no audio runs (after construction, prepare or reconfigure). `plan` measures a plan that is not
    // active yet (null: the active one).
    void publishReported(const float* current, const graph::FxPlan* plan = nullptr);
    // Lets every LatencyReporter adopt the latency-relevant parameters of `current` (LatencyReporter::adoptLatencyParams);
    // control thread, while no audio runs. Engine::prepare calls it before publishReported().
    void adoptLatencyParams(const float* current);

    int size() const { return static_cast<int>(slots.size()); }
    const ModuleDescriptor& descriptor(int i) const { return *slots[static_cast<size_t>(i)].descriptor; }
    EffectModule* module(int i) { return slots[static_cast<size_t>(i)].instance.get(); }
    EffectModule* find(const std::string& moduleId);

    // First instance implementing a capability interface (ConvolutionIr, GainReductionMeter...).
    template <typename T> T* findCapability()
    {
        for (auto& s : slots)
            if (auto* t = dynamic_cast<T*>(s.instance.get()))
                return t;
        return nullptr;
    }
    template <typename T> T* findCapability(const std::string& moduleId) { return dynamic_cast<T*>(find(moduleId)); }

private:
    struct Slot
    {
        const ModuleDescriptor* descriptor = nullptr;
        std::unique_ptr<EffectModule> instance;
        std::vector<int> globalIndex;
        std::vector<int> genericTarget;
        int onIndex = -1;
        int positionIndex = -1;
        bool copy = false; // a further instance: the classic modulation destinations of the first one do not reach it
        // On / off crossfade: 1 while the module is on, 0 while it is off, in between during the fade. The module
        // runs while the weight is above 0 and is deactivated (state cleared) once a fade-out has finished.
        float fade = 1.0f;
        const LatencyReporter* reporter = nullptr; // instance, when it reports latency (set once at construction)
        bool seeded = false; // the first block after prepare() or reset() adopts the on state without a fade
    };
    struct Context
    {
        const float* current;
        const float* legacyDests;
        const float* genericDest;
        double bpm;
        const float* auxL;
        const float* auxR;
        float note;
    };
    static bool slotOn(const Slot& s, const float* current);
    void processSegment(float* left, float* right, int n, const Context& c);
    void runLane(const graph::FxLane& lane, float* left, float* right, int n, const Context& c);

    const ParameterModel* model;
    std::vector<Slot> slots;
    std::vector<int> order;
    const graph::FxPlan* active = nullptr;
    std::atomic<int> pubLatency{0};
    std::atomic<double> pubTail{0.0};
    graph::FxPlan classicPlan;
    int blockCapacity = 0;
    float note = -1.0f; // EffectInputs::note(); see setNote()
    std::vector<float> laneMemory;  // kMaxLanes stereo lane buffers followed by a copy of the voice bus
    std::vector<float> zeros;       // legacy destinations of the copies
    std::vector<float> fadeMemory;  // the dry input of a module that is fading (left, then right)
    float fadeStep = 1.0f / 480.0f; // on / off crossfade of 10 ms
};

} // namespace crucible
