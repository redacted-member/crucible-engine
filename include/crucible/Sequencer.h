// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/ParameterMap.h>
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

constexpr int kSeqSlots = 12;
constexpr int kSeqStepsPerBar = 16;
constexpr int kSeqMaxBars = 16;
constexpr int kSeqMaxSteps = kSeqStepsPerBar * kSeqMaxBars;
constexpr int kSeqLocks = 4;

struct SeqLock
{
    std::int16_t param = -1; // index into ParameterMap::params(), -1 = unused
    float value = 0.0f;      // natural units
};

struct SeqStep
{
    bool on = false;
    std::uint8_t note = 60;
    std::uint8_t velocity = 100;
    std::uint8_t ratchet = 1;
    float gate = 0.5f;        // fraction of a step; > 1 ties into the next step
    float probability = 1.0f; // 0..1
    std::array<SeqLock, kSeqLocks> locks;
};

struct SeqPattern
{
    int lengthBars = 1;
    std::array<SeqStep, kSeqMaxSteps> steps;
    int lengthSteps() const { return lengthBars * kSeqStepsPerBar; }
};

struct SeqBank
{
    std::array<SeqPattern, kSeqSlots> pattern;
};

// Plain serialised form of one pattern slot. steps is a ';'-separated list of non-default steps, each
// "index,on,note,velocity,gate,probability,ratchet[,paramId=value...]". Hosts store these records however they
// like (the plugin wraps them in its state tree).
struct PatternRecord
{
    int slot = -1;
    int bars = 1;
    std::string steps;
};

std::vector<PatternRecord> seqBankSerialise(const SeqBank& bank, const ParameterModel* model = nullptr);
// Resets bank, then loads every valid record. Out-of-range fields are clamped; unknown lock ids are skipped.
void seqBankDeserialise(const std::vector<PatternRecord>& records, SeqBank& bank, const ParameterModel* model = nullptr);

// Pattern data shared between the message thread (edits, UI snapshots, state) and the audio thread
// (read only). Edits copy the bank and publish it atomically; the audio thread pins the published
// bank with a hazard pointer, so reading never locks or allocates.
class SeqStore
{
public:
    SeqStore();

    // Message thread.
    void edit(const std::function<void(SeqBank&)>& change);
    void replace(const SeqBank& bank);
    SeqBank snapshot() const;
    SeqPattern snapshotPattern(int slot) const;
    void collectGarbage();

    // Audio thread.
    const SeqBank* acquire() noexcept;
    void release() noexcept { inUse.store(nullptr, std::memory_order_release); }

private:
    void publish(std::unique_ptr<SeqBank> bank);

    mutable std::mutex mutex;
    std::vector<std::unique_ptr<SeqBank>> banks;
    std::atomic<SeqBank*> published{nullptr};
    std::atomic<SeqBank*> inUse{nullptr};
};

struct SeqSettings
{
    bool run = false;
    bool hostSync = false;
    float tempo = 120.0f;
    int rate = 0;      // see seqRateBeats()
    int direction = 0; // 0 forward, 1 reverse, 2 ping-pong, 3 random
    float swingPercent = 0.0f;
    int transpose = 0;
    int pattern = 0;
    int chain = 0; // see seqChainSlots()
    // Keys trigger (seq_trigger = Keys): the pattern runs only while keysHeld, restarting from step 1 each time the first
    // key goes down; keyTranspose (the newest held key - 60) adds to transpose.
    bool keyTrigger = false;
    bool keysHeld = false;
    int keyTranspose = 0;
};

// Beats (quarter notes) per step for the rate choices 1/16, 1/8, 1/32, 1/16T, 1/8T, 1/32T.
double seqRateBeats(int rate);
// Number of slots played in order for each chain choice (0 = chaining off).
int seqChainSlots(int chain);

struct SeqHostTime
{
    bool hasPpq = false;
    double ppq = 0.0;
    bool playing = false;
    bool hasBpm = false;
    double bpm = 120.0;
};

struct SeqEvent
{
    enum class Type
    {
        NoteOff,
        LockClear,
        LockSet,
        Position,
        NoteOn
    };
    Type type = Type::NoteOn;
    int offset = 0;     // sample offset inside the block
    int a = 0;          // note, parameter index or step index
    float value = 0.0f; // velocity (0..1) or lock value
};

class Sequencer
{
public:
    static constexpr int maxEvents = 512;

    void prepare(double sampleRate);
    void reset();
    void setSeed(std::uint32_t s) { seed = s; }

    // Produces the events of one block, ordered by offset. Returns the count (<= cap).
    int process(const SeqBank& bank, const SeqSettings& settings, const SeqHostTime& host, int numSamples, SeqEvent* out,
                int cap);

    // Probability gate, exposed for tests: deterministic in (seed, step counter).
    static float random01(std::uint32_t seed, std::int64_t stepCounter);

    // Which pattern slot and step index the step counter k maps to.
    struct Position
    {
        int slot = 0;
        int index = 0;
    };
    static Position positionFor(const SeqBank& bank, const SeqSettings& settings, std::int64_t k, std::uint32_t seed);

    bool isActive() const { return active; }
    Position lastPosition() const { return last; }

private:
    struct Pending
    {
        double beat = 0.0;
        SeqEvent::Type type = SeqEvent::Type::NoteOn;
        int a = 0;
        float value = 0.0f;
        bool tied = false;
    };

    void push(const Pending& p);
    void handleStep(const SeqBank& bank, const SeqSettings& s, std::int64_t k, double beat, double stepBeats);
    int emit(double p0, double p1, double bps, int numSamples, SeqEvent* out, int cap);
    int flushAll(SeqEvent* out, int cap);

    double sampleRate = 48000.0;
    std::uint32_t seed = 0x1badb002U;
    bool active = false;
    double internalBeat = 0.0;
    double lastEnd = -1.0;
    std::int64_t lastHandled = -1;
    std::array<Pending, 512> pending{};
    int pendingCount = 0;
    std::array<int, 64> activeLocks{};
    int activeLockCount = 0;
    Position last;
};

} // namespace crucible
