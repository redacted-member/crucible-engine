// SPDX-License-Identifier: MIT
#pragma once

// Oscillator modules. An oscillator slot of the voice (osc1 .. oscN) has a type parameter (oscN_type) that picks
// one of the registered Oscillator modules: type 0 is the built-in wavetable oscillator (it lives in the voice),
// the others are OscillatorInstance objects created once per voice and slot when the engine is prepared. An
// instance renders unit-level stereo audio at the pitch the voice computed; the voice applies level, pan,
// routing and cross modulation. See engine/docs/MODULE_GUIDE.md.

#include <crucible/ParameterMap.h>
#include <crucible/VoiceCard.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace crucible
{

struct SampleData;
struct SliceSet;
struct ZoneSet;
struct CloudModel;

// Parameters of one slot, resolved by suffix: bind("smp_start") finds "osc2_smp_start" for slot index 1.
class SlotParams
{
public:
    // model: the parameter model of the product the oscillator belongs to; it must outlive the SlotParams.
    SlotParams(int slotIndex, const ParameterModel& parameterModel) : slot(slotIndex), model(&parameterModel) {}

    // Returns the local index to pass to value(). Throws std::logic_error when the parameter does not exist.
    int bind(const std::string& suffix);
    int slotIndex() const { return slot; }

    // Value of local parameter `local` after per-voice modulation. values are all parameter values; targetMod
    // the per-voice modulation (ParameterModel::voiceTargets()) or null.
    float value(const float* values, const float* targetMod, int local) const;
    int globalIndex(int local) const { return global[static_cast<size_t>(local)]; }

private:
    int slot;
    const ParameterModel* model;
    std::vector<int> global;
    std::vector<int> voiceTarget;
};

struct OscNoteInfo
{
    float note = 60.0f;
    float velocity = 1.0f;
    std::uint32_t seed = 1;
    // True when the oscillator is (re)started in the middle of a note, e.g. after its type was switched.
    bool restart = false;
    // The voice card's draws for the unison voices of this slot (kCardUnison entries) and their amount, 0..1; null when
    // the voice has no card (the oscillator then draws its own from the seed). See VoiceCard.h.
    const OscCard* card = nullptr;
    float cardAmount = 0.0f;
};

// Everything an oscillator needs for one chunk (at most kChunkSize samples).
struct OscRender
{
    const float* values = nullptr;    // all parameter values, natural units
    const float* targetMod = nullptr; // per-voice modulation of voiceMod parameters, or null
    float pitch = 60.0f;              // pitch in MIDI note units, including transpose, slot tuning, bend, modulation
    double sampleRate = 48000.0;
    int numSamples = 0;
    float* left = nullptr; // zeroed; the oscillator adds its output
    float* right = nullptr;
    // Data shared with the control thread, pinned for the render call; null when nothing is loaded.
    const SampleData* sample = nullptr;
    const SliceSet* slices = nullptr; // slice markers of the sample, or null
    const CloudModel* cloud = nullptr;
    // Seconds since the note started (for scan-style movement).
    double noteTime = 0.0;
    const ZoneSet* zones = nullptr; // multisample zones of the slot (see Zones.h), or null
    // Audio-rate modulation (P12), one value per sample, or null. Only an oscillator whose supportsAudioRate() is true gets
    // them; the others are given the chunk mean through pitch and the parameters instead.
    //   pitchMod: semitones added to `pitch`, per sample;
    //   fmAmountMod: added to the cross-modulation amount (the same unit as the parameter);
    //   widthMod: added to the pulse width (the same unit as the parameter).
    const float* pitchMod = nullptr;
    const float* fmAmountMod = nullptr;
    const float* widthMod = nullptr;
};

// What a UI draws: playheads and grains of the active voices, in 0..1 positions along the source.
// Most grains one oscillator instance reports in OscVisual::grains; the read-back views reserve for it up front.
constexpr int kOscVisualMaxGrains = 32;

struct OscVisual
{
    struct Grain
    {
        float position = 0.0f; // 0..1 along the source
        float width = 0.0f;    // 0..1
        float amplitude = 0.0f;
        float pan = 0.0f;
    };
    std::vector<float> playheads;
    std::vector<Grain> grains;
    std::vector<int> zones; // zone indices sounding in the active voices (multisample slots)
};

class OscillatorInstance
{
public:
    virtual ~OscillatorInstance() = default;
    virtual void prepare(double sampleRate, int quality) {}
    // Control thread: allocate the state that only a slot playing this type needs. Every voice owns an instance of every
    // type, so a heavy type keeps its large buffers out of prepare() and builds them here instead, when the type is first
    // selected (Engine::setParam on the type parameter, Engine::prepare for the current selection, a graph node on
    // prepare). Idempotent; the audio thread may be rendering other types, and this one stays silent until it is armed.
    virtual void arm() {}
    virtual void noteOn(const OscNoteInfo& info) = 0;
    virtual void noteOff() {}
    // True when the oscillator applies OscNoteInfo::card itself; the voice then leaves out its own tuning tolerance.
    virtual bool usesVoiceCard() const { return false; }
    // True when render() honours OscRender::pitchMod, fmAmountMod and widthMod sample by sample.
    virtual bool supportsAudioRate() const { return false; }
    // True when the waveform may simply run on when a voice that still sounds is struck again (mono / legato retrigger):
    // the voice then skips noteOn() and the note continues without a step. False (the default) for oscillators that need
    // a fresh start, such as an exciter or a sample playhead: the voice fades out over about 5 ms, then starts the note.
    virtual bool continuesOnRetrigger() const { return false; }
    virtual void render(const OscRender& r) = 0;
    // Any thread: appends this instance's visual state (reads atomics written by the audio thread).
    virtual void visual(OscVisual& out) const {}
};

// slot is the zero-based slot index; instances resolve their parameters with SlotParams(slot, model).
using OscillatorFactory = std::function<std::unique_ptr<OscillatorInstance>(int slot, const ParameterModel& model)>;

} // namespace crucible
