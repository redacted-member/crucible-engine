// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/RtShared.h>
#include <crucible/Sequencer.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace crucible
{

// Arpeggiator. Sits between the keyboard input (MIDI, Engine::noteOn) and the voice allocator; the step sequencer
// bypasses it and drives the voices directly, so both can run together. Timing is sample accurate (note-ons,
// gates, ratchets and swing land on exact samples), the clock is the host tempo / transport or an internal BPM,
// and everything is deterministic for a given seed.

enum class ArpMode
{
    Up,
    Down,
    UpDown, // ends are not repeated
    DownUp,
    Random,
    AsPlayed,
    Chord,    // all held notes on every step
    Converge, // outside in: lowest, highest, second lowest, second highest, ...
    Count
};

const std::vector<std::string>& arpModeNames();
const std::vector<std::string>& arpRateNames(); // "1/1" .. "1/32T", "Free"
// Quarter notes per step for a rate choice; 0 for "Free".
double arpRateBeats(int rate);

constexpr int kArpMaxSteps = 32;

// One step of the editable pattern grid (used when ArpSettings::useGrid is set).
struct ArpStep
{
    bool on = true;
    bool tie = false;          // hold the previous step's note through this step instead of playing a new one
    std::uint8_t velocity = 0; // 1..127 overrides the computed velocity; 0 keeps it
    float gate = 1.0f;         // multiplies the global gate, 0.05..2
    std::int8_t octave = 0;    // -3..3
    float probability = 1.0f;  // 0..1, deterministic per step counter
    std::uint8_t ratchet = 1;  // 1..4 notes in the step
    std::int8_t transpose = 0; // -24..24 semitones
    // Probability group 1..8 (0 = none). The steps of a group are alternatives: once per pass through the pattern
    // exactly one of its playable steps fires, drawn with the steps' probabilities as weights. When the weights add
    // up to less than 1 the remainder is "nothing fires" (so two steps at 0.25 each play one of them half the time).
    std::uint8_t group = 0;
};

struct ArpPattern
{
    int length = 8;    // 1..kArpMaxSteps
    int direction = 0; // 0 forward, 1 reverse, 2 ping-pong, 3 random
    std::array<ArpStep, kArpMaxSteps> steps{};
};

// Plain-text form: "length,direction;index,on,tie,velocity,gate,octave,probability,ratchet,transpose[,group];..."
// (only non-default steps are listed; the group field is optional, older text loads unchanged).
std::string arpPatternSerialise(const ArpPattern& p);
bool arpPatternDeserialise(const std::string& text, ArpPattern& out);

// Pattern shared between a control thread (edits) and the audio thread (reads), lock free on the audio side.
class ArpPatternStore
{
public:
    ArpPatternStore();
    void edit(const std::function<void(ArpPattern&)>& change); // control thread
    void replace(const ArpPattern& p);
    ArpPattern snapshot() const;
    void collectGarbage() { shared.collectGarbage(); }
    const ArpPattern* acquire() noexcept { return shared.acquire(); } // audio thread
    void release() noexcept { shared.release(); }

private:
    mutable std::mutex mutex;
    RtShared<ArpPattern> shared;
};

enum class ArpVelocity
{
    AsPlayed,
    Fixed,
    Accent // fixed velocity, accented steps (every fourth, or grid steps with velocity >= 100) at full level
};

struct ArpSettings
{
    bool on = false;
    int mode = 0;    // ArpMode
    int octaves = 1; // 1..4
    int rate = 10;   // arpRateNames() index ("1/16")
    float freeRateHz = 8.0f;
    float gate = 0.5f;         // fraction of a step, 0.05..1.5 (> 1 overlaps the next step)
    float swingPercent = 0.0f; // 0..75, delays odd steps
    bool latch = false;
    int velocityMode = 0; // ArpVelocity
    int fixedVelocity = 100;
    float accent = 0.3f; // level drop of non-accented steps in Accent mode
    int ratchet = 1;     // 1..4 (grid steps carry their own)
    int transpose = 0;   // semitones
    bool useGrid = false;
    bool hostSync = true; // host tempo / transport when the host provides it, else the internal tempo
    float tempo = 120.0f; // internal BPM
};

struct ArpInput
{
    int position = 0;
    bool on = true;
    int note = 60;
    float velocity = 0.8f; // 0..1
    bool panic = false;    // all-notes-off: forgets every held (and latched) key; note and on are ignored
};

struct ArpOutput
{
    int offset = 0;
    bool on = true;
    int note = 60;
    float velocity = 0.8f;
};

class Arpeggiator
{
public:
    static constexpr int maxEvents = 512;
    static constexpr int maxHeld = 32;

    void prepare(double sampleRate);
    void reset(); // forgets held notes and pending events
    void setSeed(std::uint32_t s) { seed = s == 0 ? 1U : s; }

    // inputs sorted by position. grid may be null (then useGrid is ignored). Returns the number of events written.
    int process(const ArpInput* inputs, int numInputs, const ArpSettings& s, const ArpPattern* grid, const SeqHostTime& host,
                int numSamples, ArpOutput* out, int cap);

    // Note-offs for everything sounding (used when the arp is switched off). Returns the count written.
    int flush(ArpOutput* out, int cap);

    int heldCount() const { return held; }
    // Grid step (or plain step counter modulo 32) of the most recent step, for a UI playhead.
    int currentStep() const { return lastStep.load(std::memory_order_relaxed); }

private:
    struct Pending
    {
        std::int64_t at = 0;
        bool on = false;
        int note = 0;
        float velocity = 0.0f;
    };

    void noteInput(const ArpInput& in, const ArpSettings& s);
    void triggerStep(std::int64_t k, std::int64_t now, double stepSamples, const ArpSettings& s, const ArpPattern* grid);
    void schedule(const Pending& p);
    int buildSequence(const ArpSettings& s, int* outNotes, float* outVel) const;

    double sampleRate = 48000.0;
    std::uint32_t seed = 0x2545f491U;
    std::int64_t now = 0;      // absolute sample counter
    double beat = 0.0;         // internal clock (beats, or steps in free mode)
    std::int64_t nextStep = 0; // index of the next step to trigger
    bool running = false;
    std::array<int, maxHeld> heldNotes{};
    std::array<float, maxHeld> heldVel{};
    std::array<std::uint32_t, maxHeld> heldOrder{};
    std::array<bool, maxHeld> physicallyDown{};
    int held = 0;
    std::uint32_t arrival = 0;
    bool latchArmed = false; // latch: the next key press after a full release starts a new chord
    int seqIndex = 0;        // position in the note sequence
    int lastNote = -1;       // last note started (for ties)
    std::array<Pending, 256> pending{};
    int pendingCount = 0;
    std::atomic<int> lastStep{0};
};

} // namespace crucible
