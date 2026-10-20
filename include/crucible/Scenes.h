// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/ParameterMap.h>
#include <crucible/RtShared.h>

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace crucible
{

constexpr int kNumScenes = 12;

// Plain serialised form of one stored scene: "id=value;id=value;..." in natural units (six decimals).
// Hosts store these records however they like (the plugin wraps them in its state tree).
struct SceneRecord
{
    int slot = -1;
    std::string values;
};

// Immutable copy of the stored scenes for the audio thread (the vector field reads it): natural values and the
// same values in normalised (skewed) space, per scene slot. Empty slots have filled = false.
struct SceneSnapshot
{
    std::uint64_t generation = 0; // grows with every publish
    std::array<bool, kNumScenes> filled{};
    std::array<std::vector<float>, kNumScenes> natural;
    std::array<std::vector<float>, kNumScenes> normalised;
};

// ---- Vector field: a four-corner morph between scenes (perf.vector, applied on the audio thread by SynthEngine).
//
// The square has corner A at the top left (x 0, y 1), B top right (1, 1), C bottom left (0, 0), D bottom right (1, 0).
// Bilinear weights: A (1-x) y, B x y, C (1-x)(1-y), D x (1-y); they sum to 1 and the centre weighs 0.25 each.
struct VectorWeights
{
    std::array<float, 4> w{};
};
VectorWeights vectorWeights(float x, float y);

// The morphed value of parameter `index` for the given weights. slots[k] is the scene slot at corner k, or a
// negative number / an empty slot for "use hostValue" (the parameter's unmorphed value). Floats blend in
// normalised (skewed) space, choices, ints and bools take the heaviest corner (ties go to the earlier corner); a
// weight of exactly 1 returns that corner's stored value, and parameters equal at all four corners return that
// value untouched. Allocation free.
float vectorMorphValue(const ParamDef& def, size_t index, const SceneSnapshot& scenes, const std::array<int, 4>& slots,
                       const VectorWeights& weights, float hostValue);

// Twelve full-parameter snapshots (natural units, indexed like ParameterMap::params()) and morphing between
// any two. All methods run on the message thread; the audio thread never touches scenes, it only sees the
// resulting parameter values. Parameters flagged scene=false are neither stored nor morphed.
class SceneBank
{
public:
    // model: parameter model of the product the scenes belong to (must outlive the bank); null = process default
    // product (deprecated singleton shim).
    explicit SceneBank(const ParameterModel* model = nullptr);

    void store(int slot, const std::vector<float>& values);
    bool isFilled(int slot) const;
    void clear(int slot);

    // Copies the stored values of scene-managed parameters into out (other entries are left alone).
    bool recall(int slot, std::vector<float>& out) const;

    // t = 0 gives scene a, t = 1 scene b. Float parameters interpolate in normalised (skewed) space,
    // choices, ints and bools switch at t = 0.5. Returns false when either scene is empty.
    bool morph(int a, int b, float t, std::vector<float>& out) const;

    // Control-thread reference of the audio-thread vector overlay: writes the morph of the four corner scenes at
    // (x, y) into out for every scene-managed parameter (other entries are left alone; empty corners keep the
    // value already in out). Returns false when no corner scene is filled.
    bool vectorMorph(const std::array<int, 4>& slots, float x, float y, std::vector<float>& out) const;

    // Filled scenes only. Unknown ids are ignored on load, missing ids take their default value, so states
    // survive parameter additions.
    std::vector<SceneRecord> serialise() const;
    void deserialise(const std::vector<SceneRecord>& records);

    // Control thread: `hook` runs with the values of every scene that store / deserialise has just filled, before the
    // audio thread can see them (the engine uses it to build the heavy state of oscillator types a scene selects).
    void setStoredHook(std::function<void(const std::vector<float>&)> hook);

    // Audio thread: pin the snapshot published by the last store / clear / deserialise (null before the first),
    // use it, release it. Lock and allocation free. The control thread frees superseded snapshots.
    const SceneSnapshot* acquire() noexcept { return shared.acquire(); }
    void release() noexcept { shared.release(); }

private:
    void publishLocked();

    const ParameterModel* params;
    mutable std::mutex mutex;
    std::uint64_t generation = 0;
    std::array<std::vector<float>, kNumScenes> data;
    RtShared<SceneSnapshot> shared;
    std::function<void(const std::vector<float>&)> storedHook;
};

} // namespace crucible
