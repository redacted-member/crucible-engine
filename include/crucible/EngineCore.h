// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/Arpeggiator.h>
#include <crucible/KeyBus.h>
#include <crucible/KeyDetect.h>
#include <crucible/KeyLock.h>
#include <crucible/MasterAnalyzer.h>
#include <crucible/Views.h>
#include <crucible/Meter.h>
#include <crucible/Sequencer.h>
#include <crucible/SpscRing.h>
#include <crucible/SynthEngine.h>

#include <array>
#include <atomic>
#include <vector>

namespace crucible
{

// Event sent from a non-audio thread (OSC keyboard, UI keyboard) to the audio thread.
struct ExternalEvent
{
    enum class Type : std::uint8_t
    {
        NoteOn,
        NoteOff,
        PitchBend,
        ModWheel,
        Sustain,
        AllNotesOff,
        KillAll,
        Aftertouch,
        Controller,  // note = controller number, value 0..1
        PolyPressure // note = key, value 0..1
    };
    Type type = Type::NoteOn;
    std::uint8_t channel = 0; // MIDI channel 0..15 (MPE)
    std::uint8_t note = 0;
    float value = 0.0f; // velocity 0..1, bend -1..1, wheel 0..1, sustain 0/1, aftertouch 0..1
};

// A note played while step-record is armed. step < 0 means "use the record cursor".
struct RecordEvent
{
    int step = -1;
    int slot = 0;
    int note = 60;
    int velocity = 100;
};

struct MidiInput
{
    int position = 0;
    std::uint8_t status = 0;
    std::uint8_t data1 = 0;
    std::uint8_t data2 = 0;
};

// Everything that happens on the audio thread, independent of any plugin wrapper so it can be tested
// headless: MIDI, external events, the step sequencer, the synth engine, parameter locks and metering.
// See crucible/Engine.h for the simpler embedding facade built on top of this class.
class EngineCore
{
    friend struct EngineViews;

public:
    explicit EngineCore(const EngineConfig& config = {});

    void prepare(double sampleRate);
    // Clears voices, effect states, held notes, the arpeggiator and queued performance events without re-preparing or
    // allocating. Not while process() runs.
    void reset();

    // values: natural-unit parameter values, model().params().size() entries.
    // auxLeft / auxRight: optional sidechain input of numSamples samples (right may be null for mono).
    void process(float* left, float* right, int numSamples, const float* values, const MidiInput* midi, int numMidi,
                 const SeqHostTime& host, const float* auxLeft = nullptr, const float* auxRight = nullptr);

    // Any thread (single producer).
    bool pushExternal(const ExternalEvent& e);
    // Message thread (single consumer).
    bool popRecord(RecordEvent& e);

    SynthEngine& engine() { return synth; }
    const ParameterModel& model() const { return synth.model(); }
    // Scenes the vector field morphs between (see SynthEngine::attachScenes); the bank must outlive the core.
    void attachScenes(SceneBank* bank) { synth.attachScenes(bank); }
    // Parameters the product keeps out of the vector field (one flag per parameter, ParameterMap order; null for none).
    // The core ORs them with its sequencer locks. The mask must outlive the core.
    void setProductExclusions(const char* mask);
    // Key lock. The bus is optional (Key Sync Follow / Lead need one); the pointer must outlive the core.
    void attachKeyBus(KeyBus* bus) { keyBus.store(bus, std::memory_order_release); }
    KeyDetector& keyDetector() { return detector; }
    // The settings the quantiser used in the last block (local or from the bus), any thread.
    KeyLockSettings keySettings() const;
    bool keyFollowing() const { return following.load(std::memory_order_relaxed); }
    SeqStore& store() { return seqStore; }
    OutputMeter& meter() { return outputMeter; }
    const OutputMeter& meter() const { return outputMeter; }
    MasterAnalyzer& analyzer() { return masterAnalyzer; }
    const MasterAnalyzer& analyzer() const { return masterAnalyzer; }
    Sequencer& sequencer() { return seq; }
    ArpPatternStore& arpPattern() { return arpStore; }
    const Arpeggiator& arpeggiator() const { return arp; }
    // GUI-agnostic read-back (crucible/Views.h): every live value of the engine as a subscribable view. process() publishes
    // the subscribed ones at its end.
    ViewHub& views() { return viewHub; }
    const ViewHub& views() const { return viewHub; }
    std::uint64_t blockCount() const { return blocks.load(std::memory_order_relaxed); }
    // Notes the arpeggiator and the step sequencer started (after Key Lock, so the keys that actually sound), as a
    // bitmask over MIDI notes 0..127 (word 0 = notes 0..63). Holds the notes sounding at the end of the last block
    // plus those started and ended inside it. Any thread.
    void soundingNotes(std::array<std::uint64_t, 2>& out) const
    {
        out[0] = sounding[0].load(std::memory_order_relaxed);
        out[1] = sounding[1].load(std::memory_order_relaxed);
    }
    int sequencerStep() const { return seqStep.load(std::memory_order_relaxed); }
    bool sequencerRunning() const { return seqRunning.load(std::memory_order_relaxed); }

private:
    struct Event
    {
        int position = 0;
        int order = 0;
        enum class Kind : std::uint8_t
        {
            NoteOn,
            NoteOff,
            PitchBend,
            ModWheel,
            Aftertouch,
            Sustain,
            AllNotesOff,
            KillAll,
            SeqNoteOn,
            SeqNoteOff,
            LockSet,
            LockClear,
            Position,
            Controller,
            PolyPressure,
            Slide
        } kind = Kind::NoteOn;
        int a = 0;
        float f = 0.0f;
        int channel = -1;
    };

    void apply(const Event& e, bool& dirty);
    void recordNote(int note, int velocity);
    static int priority(Event::Kind k);

    SynthEngine synth;
    Sequencer seq;
    SeqStore seqStore;
    OutputMeter outputMeter;
    MasterAnalyzer masterAnalyzer;
    ViewHub viewHub;
    int viewScope = -1; // the master scope view, which the spectrum views are computed from
    std::vector<float> working;
    std::vector<float> locked;
    std::vector<char> lockActive;
    std::vector<char> exclusions; // what the synth reads: lockActive OR the product's mask
    const char* productExclude = nullptr;
    void refreshExclusions(size_t index);
    std::array<Event, 2048> events{};
    std::array<SeqEvent, Sequencer::maxEvents> seqEvents{};
    Arpeggiator arp;
    ArpPatternStore arpStore;
    std::array<ArpInput, 512> arpIn{};
    std::array<ArpOutput, Arpeggiator::maxEvents> arpOut{};
    bool arpWasOn = false;
    SpscRing<ExternalEvent, 512> externalRing;
    SpscRing<RecordEvent, 64> recordRing;
    std::atomic<std::uint64_t> blocks{0};
    std::atomic<int> seqStep{0};
    std::array<std::atomic<std::uint64_t>, 2> sounding{};
    std::array<std::uint64_t, 2> soundingNow{}, soundingTouched{};
    void markSounding(int note, bool on);
    std::atomic<bool> seqRunning{false};

    struct Indices
    {
        int run, sync, tempo, rate, dir, swing, transpose, pattern, chain, rec, hold;
        int arpOn, arpMode, arpRate, arpOctaves, arpGate, arpLatch, arpSync, arpTempo, arpFree, arpSwing, arpRatchet,
            arpTranspose, arpVelMode, arpVel, arpAccent, arpGrid, arpGridLength, arpGridDir;
        int mpeOn;
        int keyMode, keyRoot, keyScale, keyMask, keyTie, keySync;
        int seqTrigger;
    } idx{};
    int startKey(NoteQuantiser& q, int note, float velocity, int channel);
    int stopKey(NoteQuantiser& q, int note);
    void updateKeyLock(const float* values);

    NoteQuantiser quantiser, arpQuantiser;
    KeyDetector detector;
    std::atomic<KeyBus*> keyBus{nullptr};
    std::atomic<std::uint64_t> keyPacked{0};
    std::atomic<bool> following{false};
    std::uint64_t writerId = 0;
    double rate = 48000.0;
    bool haveLead = false;
    KeyLockSettings leadSent;
    bool recording = false;
    bool mpeActive = false;
    bool seqWasActive = false;
    // Keys held for the sequencer's Keys trigger, oldest first (the newest sets the transposition); audio thread only.
    std::array<std::uint8_t, 16> seqKeys{};
    int seqKeyCount = 0;
};

} // namespace crucible
