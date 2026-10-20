// SPDX-License-Identifier: MIT
#pragma once

// Clip modulators: drawable, multi-lane automation that the engine plays by itself (independent of the DAW).
//
// A Clip has one timeline (length, loop region, play mode, trigger, tempo sync) and up to kMaxClipLanes lanes.
// A lane is either a curve (breakpoints with a shape per segment) or a row of steps, and outputs 0..1. Every lane
// is a modulation source (clipLaneSource()) that assignable routes (Modulation.h) send to any parameter with
// amount, polarity, curve and smoothing. An MSEG is a clip with one curve lane, per-voice trigger and a sustain
// loop (makeMseg()).
//
// Time: synced clips count in beats (quarter notes) and follow the tempo, free clips count in seconds. A playhead
// keeps the elapsed time as a double and derives the position from it, so loop wraps are exact and tempo changes
// change the speed without jumps. Transport-triggered clips derive their position from the host's PPQ position.
//
// Threads: ClipStore is edited on a control thread and published to the audio thread as an immutable ClipBank
// (RtShared); evaluation (ClipPlayhead, evaluateLane) never allocates.

#include <crucible/Json.h>
#include <crucible/RtShared.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace crucible
{

constexpr int kMaxClips = 8;
constexpr int kMaxClipLanes = 8;
constexpr int kMaxClipPoints = 1024; // per curve lane
constexpr int kMaxClipSteps = 256;   // per step lane

enum class ClipLaneKind : std::uint8_t
{
    Curve,
    Steps
};

// Shape of the segment that starts at a point.
enum class SegmentShape : std::uint8_t
{
    Curve,  // power curve; curve -1..1 bends it (0 = linear)
    Hold,   // keeps the start value until the next point
    SCurve, // ease in and out; curve sets the steepness
    Stairs  // 2..16 equal steps; curve selects the count
};

enum class ClipPlayMode : std::uint8_t
{
    OneShot, // plays once, then holds the last value
    Loop,    // plays the lead-in once, then repeats the loop region
    PingPong // like Loop, but alternates direction inside the loop region
};

enum class ClipTrigger : std::uint8_t
{
    Voice,           // one playhead per voice, restarted by each note it plays
    GlobalRetrigger, // one playhead, restarted by every note-on
    GlobalFree,      // one playhead that runs from prepare() / restartClips()
    Transport        // one playhead locked to the host position (free-running while the host is stopped)
};

struct ClipPoint
{
    double x = 0.0;     // position in clip units (beats when synced, seconds when free)
    float y = 0.0f;     // 0..1
    float curve = 0.0f; // -1..1, shape parameter of the segment starting here
    SegmentShape shape = SegmentShape::Curve;
    bool operator==(const ClipPoint&) const = default;
};

struct ClipLane
{
    std::string name;
    ClipLaneKind kind = ClipLaneKind::Curve;
    std::vector<ClipPoint> points; // curve lanes, sorted by x
    std::vector<float> steps;      // step lanes, values 0..1 spread evenly over the clip length
    float stepGlide = 0.0f;        // step lanes: share of each step (0..1) spent gliding into the next value
    float smoothingMs = 0.0f;      // one-pole smoothing of the lane output
    float humanise = 0.0f;         // 0..1: each segment starts up to half a segment late, varying per cycle
    float randomise = 0.0f;        // 0..1: random value offset per point / step, varying per cycle
    std::uint32_t seed = 1;
};

struct Clip
{
    std::string name;
    bool sync = true;    // beats (true) or seconds
    double length = 4.0; // clip units
    double grid = 0.25;  // snap grid in clip units (0 = off); also the step size of "length in steps"
    double loopStart = 0.0;
    double loopEnd = 4.0;
    ClipPlayMode mode = ClipPlayMode::Loop;
    ClipTrigger trigger = ClipTrigger::Voice;
    double rate = 1.0;        // speed multiplier
    bool sustainLoop = false; // MSEG style: loop only while the note is held, then play on to the end
    std::vector<ClipLane> lanes;

    // Length expressed in grid steps (0 when the grid is off).
    int lengthSteps() const;
    void setLengthSteps(int steps); // needs a grid > 0
    double snap(double x) const;    // to the grid (unchanged when the grid is off)
    // Clamps everything into range, sorts points, limits counts. Called by ClipStore before publishing.
    void sanitise();
};

// Route source id of lane `lane` of clip `clip` (see Modulation.h).
constexpr int kSourceClip = 2000;
constexpr int clipLaneSource(int clip, int lane)
{
    return kSourceClip + clip * kMaxClipLanes + lane;
}
constexpr bool isClipSource(int source)
{
    return source >= kSourceClip && source < kSourceClip + kMaxClips * kMaxClipLanes;
}

// A one-lane MSEG: per-voice, optional sustain loop between loopStart and loopEnd.
Clip makeMseg(const std::vector<ClipPoint>& points, double lengthBeats, bool sync = true, double loopStart = -1.0,
              double loopEnd = -1.0);

// ---- Evaluation (audio thread safe, no allocation).

// Position of the clip timeline after `elapsed` clip units of play. cycle receives the loop count (0 during the
// lead-in and the first pass). releasedAt >= 0: the note was released at that elapsed time (sustain loop exits).
double clipPosition(const Clip& clip, double elapsed, double releasedAt, std::uint32_t& cycle);
// Lane value 0..1 at a position (humanise / randomise use the cycle).
float evaluateLane(const Clip& clip, const ClipLane& lane, double position, std::uint32_t cycle);
// Shape function of one segment: f 0..1 -> 0..1.
float segmentShape(SegmentShape shape, float curve, float f);

struct ClipPlayhead
{
    double elapsed = 0.0;
    double releasedAt = -1.0;
    bool running = false;
    std::uint32_t cycle = 0;
    double position = 0.0;

    void start()
    {
        elapsed = 0.0;
        releasedAt = -1.0;
        running = true;
        cycle = 0;
        position = 0.0;
    }
    void release()
    {
        if (running && releasedAt < 0.0)
            releasedAt = elapsed;
    }
};

// Clip units that pass in `seconds` at `bpm`.
inline double clipUnits(const Clip& clip, double seconds, double bpm)
{
    return seconds * clip.rate * (clip.sync ? bpm / 60.0 : 1.0);
}

// ---- Playback runtime (audio thread, owned by SynthEngine).
//
// One set of playheads for the global view of every clip and one set per voice for ClipTrigger::Voice clips.
// Per control chunk: beginChunk(), beginVoice() / voiceValue() per voice, globalValue() per route, endChunk().
// Positions come from accumulated elapsed time (double), so loop wraps are exact and a tempo change changes the
// speed without a jump; Transport clips take the host position at the start of every block and advance with the
// tempo inside it. Nothing here allocates after prepare().
struct ClipBank;

class ClipPlayer
{
public:
    void prepare(double sampleRate, int voiceCount);
    void restart();
    // Host transport at the start of a block (before any chunk of it is rendered).
    void setHostTransport(bool hasPpq, bool playing, double ppq);
    void noteOn(); // restarts the retrigger heads
    // Clip transport at the start of a chunk: bit k of `wanted` = clip k plays, bit k of `restart` = start clip k
    // from its beginning. A clip that is not wanted fades its routes out over 10 ms and then parks its playheads.
    void applyTransport(std::uint32_t wanted, std::uint32_t restart, int numSamples);
    // Gain applied to the modulation of clip `clip`'s lanes: 1 while playing, ramping to 0 after a stop.
    float gate(int clip) const { return clip >= 0 && clip < kMaxClips ? gains[static_cast<size_t>(clip)] : 1.0f; }
    bool parked(int clip) const
    {
        return clip >= 0 && clip < kMaxClips && !wantedPlaying[static_cast<size_t>(clip)] &&
               gains[static_cast<size_t>(clip)] <= 0.0f;
    }
    void beginChunk(const ClipBank* bank, int numSamples, double bpm, bool anyNoteHeld);
    void beginVoice(int voice, std::uint64_t age, bool active, bool releasing);
    // Lane value 0..1 (lane smoothing applied) of a route source; 0 when the lane does not exist.
    float globalValue(int source);
    float voiceValue(int voice, int source);
    void endChunk();

    // Read-back on the audio thread (position in clip units of the global head).
    double globalPosition(int clip) const;
    // Copies the playhead positions into atomics; called by the engine once at the end of each render call, while
    // the clip bank is still pinned. Allocation and lock free.
    void publishPositions();
    // UI-safe read-back of the last published positions (any thread). Global: clip units of the global head.
    // Voice: clip units of the voice's head, or -1 when that voice has no running head for the clip.
    double publishedGlobalPosition(int clip) const;
    double publishedVoicePosition(int voice, int clip) const;
    int voiceCount() const { return voices; }
    double hostPpq() const { return ppq; }

private:
    float evaluate(const ClipPlayhead& head, int source, float& smoothState);
    static constexpr int laneSlots = kMaxClips * kMaxClipLanes;

    double sampleRate = 48000.0;
    int voices = 0;
    const ClipBank* bank = nullptr;
    int chunk = 32;
    double tempo = 120.0;
    bool hostPlaying = false;
    bool hostHasPpq = false;
    bool pendingNoteOn = false;
    double ppq = 0.0;
    std::array<ClipPlayhead, kMaxClips> heads{};
    std::array<float, laneSlots> smooth{};
    std::array<float, kMaxClips> gains = []
    {
        std::array<float, kMaxClips> g{};
        g.fill(1.0f);
        return g;
    }();
    std::array<bool, kMaxClips> wantedPlaying = []
    {
        std::array<bool, kMaxClips> p{};
        p.fill(true);
        return p;
    }();
    std::vector<ClipPlayhead> voiceHeads; // voices x kMaxClips
    std::vector<float> voiceSmooth;       // voices x laneSlots
    std::vector<std::uint64_t> voiceAge;
    std::array<std::atomic<double>, kMaxClips> publishedGlobal{};
    std::unique_ptr<std::atomic<double>[]> publishedVoice; // voices x kMaxClips
};

// ---- Storage.

struct ClipBank
{
    std::vector<Clip> clips; // at most kMaxClips
    int laneCount(int clip) const
    {
        return clip >= 0 && clip < static_cast<int>(clips.size())
                   ? static_cast<int>(clips[static_cast<size_t>(clip)].lanes.size())
                   : 0;
    }
    const ClipLane* lane(int source) const;
    const Clip* clipOf(int source) const;
};

class ClipStore
{
public:
    ClipStore();

    // Control thread.
    void edit(const std::function<void(ClipBank&)>& change);
    void replace(const ClipBank& bank);
    ClipBank snapshot() const;
    void clear();
    void collectGarbage() { shared.collectGarbage(); }
    // Bumps on every publish (lets UIs and the recorder notice changes).
    std::uint64_t revision() const { return rev.load(std::memory_order_acquire); }

    // Audio thread.
    const ClipBank* acquire() noexcept { return shared.acquire(); }
    void release() noexcept { shared.release(); }

private:
    void publishLocked();
    mutable std::mutex mutex;
    ClipBank bank;
    RtShared<ClipBank> shared;
    std::atomic<std::uint64_t> rev{0};
};

// JSON form used by patches: [{"name", "sync", "length", "grid", "loop": [start, end], "mode", "trigger", "rate",
// "sustainLoop", "lanes": [{"name", "kind", "points": [[x, y, curve, shape], ...], "steps": [...], "stepGlide",
// "smoothingMs", "humanise", "randomise", "seed"}]}]. Mode, trigger, kind and shape are written as names.
json::Value clipBankToJson(const ClipBank& bank);
bool clipBankFromJson(const json::Value& value, ClipBank& out);

} // namespace crucible
