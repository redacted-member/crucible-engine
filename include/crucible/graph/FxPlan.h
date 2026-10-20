// SPDX-License-Identifier: MIT
#pragma once

// The topology of an FX graph as the audio thread runs it: lanes (serial chains of effect slots) that read the voice
// bus, the host input or other lanes, and sum into the master. A plan is plain data (indices into the ModuleRack), so
// switching plans never touches an effect's state. See ModuleRack and docs/ARCHITECTURE.md ("Graph in data").

#include <vector>

namespace crucible::graph
{

// Where a lane takes its input from.
enum class FxSourceKind
{
    Voices, // the synth output (the bus the effects process today)
    Aux,    // the host audio input (process_aux)
    Lane    // the output of another lane (a send when the gain is below one)
};

struct FxSource
{
    FxSourceKind kind = FxSourceKind::Voices;
    int lane = -1; // index into FxPlan::lanes for FxSourceKind::Lane (always earlier than the reader)
    float gain = 1.0f;
};

struct FxLane
{
    std::vector<FxSource> inputs;
    // Rack slot indices in processing order. With byPosition the order is decided at run time by the slots'
    // position parameters (ties keep the listed order), which is how the classic chain is ordered.
    std::vector<int> slots;
    bool byPosition = false;
    bool toMaster = true; // false: the lane only feeds other lanes
    float level = 1.0f;   // gain of the lane's output into the master sum
};

struct FxPlan
{
    // Topologically sorted: a lane only reads earlier lanes.
    std::vector<FxLane> lanes;
    float masterLevel = 1.0f;

    // One position-ordered lane over `slotCount` slots reading the voice bus: the classic chain.
    static FxPlan classic(int slotCount);
    // True for the shape the rack runs in place on the bus (one lane, voices in at unity, to master at unity).
    bool isInPlace() const;
};

} // namespace crucible::graph
