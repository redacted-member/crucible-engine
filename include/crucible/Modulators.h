// SPDX-License-Identifier: MIT
#pragma once

// The modulator bank (docs/plans/ENGINE-COMMUNITY-PLAN.md, section 2.10). Eight modulators, route sources
// kSourceModulator + n, each one of: Combine (add, multiply, min, max, crossfade, difference), Remap (16-point curve),
// Lag (rise / fall), Quantise, S&H, Turing, Binary, Chaos (logistic, Lorenz), Note gate, Follower, Pitch tracker, LFO
// table. A modulator reads up to two inputs (a ModSource or an earlier modulator) and is evaluated once per control chunk
// in index order, so the inputs of modulator n can only be modulators 0 .. n - 1 and no cycle can form.
//
// Scope. A voice modulator has its own state in every voice (restarted by the voice's note-on); a layer or global
// modulator has one state in the engine, shared by all voices (the classic voice is one layer, so the two scopes meet
// there). Voice modulators see every source; layer and global modulators see the sources that exist without a voice (macros,
// wheel, free LFOs, other shared modulators) and read zero from voice sources, like a global route does.
//
// Source numbering used by the per-voice evaluation ("extended source index"): 0 .. kNumModSources - 1 are the ModSource
// values, then the eight modulators, then the audio-rate sources of Voice.h (a chunk mean for targets that are not
// audio-rate).
//
// Everything here is allocation free and lock free; the state structs are plain data.

#include <crucible/ModMatrix.h>
#include <crucible/ParameterMap.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace crucible
{

constexpr int kSourceModulator = 4000; // route source id of modulator n (0-based) is kSourceModulator + n
constexpr int kNumBankModulators = 8;
constexpr bool isModulatorSource(int source)
{
    return source >= kSourceModulator && source < kSourceModulator + kNumBankModulators;
}

// Audio-rate route sources (see Voice.h): the mono outputs of the voice's audio bus.
constexpr int kSourceAudio = 5000; // route source id of audio source a is kSourceAudio + a
constexpr int kAudioLfoFirst = 0;  // lfoN_audio, N = 1 .. kNumLfos
constexpr int kAudioOscFirst = 4;  // output of oscillator slot s (0-based) = kAudioOscFirst + s, s < 8
constexpr int kAudioEnvFirst = 12; // per-sample envelopes: mod env, env 3, env 4
constexpr int kNumAudioEnvs = 3;
constexpr int kNumAudioSources = 16;
constexpr int kAudioBusBuffers = 8; // distinct audio sources the routes may use at once (the store refuses a ninth)
constexpr bool isAudioSource(int source)
{
    return source >= kSourceAudio && source < kSourceAudio + kNumAudioSources;
}

constexpr int kExtModulatorFirst = kNumModSources;
constexpr int kExtAudioFirst = kNumModSources + kNumBankModulators;
constexpr int kNumSourcesExt = kNumModSources + kNumBankModulators + kNumAudioSources;

// Extended index of a route source id, or -1 (clip, CC and MPE sources have none).
constexpr int extSourceIndex(int source)
{
    if (source > 0 && source < kNumModSources)
        return source;
    if (isModulatorSource(source))
        return kExtModulatorFirst + (source - kSourceModulator);
    if (isAudioSource(source))
        return kExtAudioFirst + (source - kSourceAudio);
    return -1;
}

// Whether an extended source is naturally bipolar (-1 .. 1). set supplies the types of the modulators.
struct ModulatorSet;

enum class ModulatorType
{
    Off,
    Combine,
    Remap,
    Lag,
    Quantise,
    SampleHold,
    Turing,
    Binary,
    Chaos,
    NoteGate,
    Follower,
    PitchTracker,
    LfoTable,
    Count
};

enum class ModulatorScope
{
    Voice,
    Layer,
    Global
};

// `mode` of Combine.
enum class CombineMode
{
    Add,
    Multiply,
    Min,
    Max,
    Crossfade,
    Difference
};

// `mode` of Quantise: how a value is mapped to one of the steps.
enum class QuantiseMode
{
    Nearest,
    Down,
    Up
};

// `mode` of the clocked types (S&H, Turing, Binary): where the clock comes from.
enum class ClockMode
{
    Rate,  // `rate` Hz
    Input, // rising edge (through 0.5) of input B
    NoteOn // every note-on (the voice's own for a voice modulator, any key for the shared ones)
};

// `mode` of Chaos.
enum class ChaosMap
{
    Logistic,
    Lorenz
};

// `mode` of Note gate.
enum class GateMode
{
    Gate,     // 1 while a key is held
    Velocity, // the velocity while a key is held
    Trigger   // a pulse of `rise` seconds at each note-on
};

// `mode` of Follower.
enum class FollowMode
{
    Audio, // the voice's oscillator mix (the previous chunk)
    Input  // |input A|
};

struct ModulatorConfig
{
    ModulatorType type = ModulatorType::Off;
    ModulatorScope scope = ModulatorScope::Voice;
    // Input selectors: 0 none, 1 .. kNumModSources - 1 a ModSource, kNumModSources + n modulator n (must be below this one).
    int inA = 0, inB = 0;
    int mode = 0;
    float rate = 4.0f;   // Hz: clock rate, LFO rate, Chaos iteration rate
    float rise = 0.0f;   // seconds: Lag rise, Follower attack, Pitch tracker smoothing, Trigger length
    float fall = 0.0f;   // seconds: Lag fall, Follower release
    int steps = 8;       // Quantise steps, Turing length
    float amount = 0.5f; // Crossfade mix, Turing randomness, Binary probability, Chaos r, LFO table position
    std::uint32_t seed = 1;
    std::array<float, 16> curve{}; // Remap points; default is the identity of a unipolar input
};

struct ModulatorSet
{
    std::array<ModulatorConfig, kNumBankModulators> mod;
    std::uint32_t active = 0;                       // bit n: modulator n has a type
    std::uint32_t needed = 0;                       // bit n: modulator n must run (a route or a needed modulator reads it)
    std::array<bool, kNumBankModulators> bipolar{}; // natural range -1 .. 1 (else 0 .. 1)
    bool needsAudio = false;                        // a needed Follower or Pitch tracker reads the voice's audio
    bool anyVoice = false;                          // a needed modulator has voice scope
    bool anyShared = false;                         // a needed modulator has layer or global scope

    // Works out needed, bipolar and the flags. usedByRoutes: bit n set when a route (or the audio bus) uses modulator n.
    void finalize(std::uint32_t usedByRoutes);
    // True when modulator `index` reads its inputs correctly: input selectors that name the modulator itself or a later
    // one are ignored (read as none), which is what keeps the bank acyclic.
    static bool validInput(int selector, int index);
    bool neededVoice(int n) const
    {
        return (needed >> n & 1U) != 0U && mod[static_cast<size_t>(n)].scope == ModulatorScope::Voice;
    }
};

// Natural range of an extended source: ModSource by its kind, modulators by type, audio-rate sources bipolar except
// envelopes.
inline bool extSourceBipolar(const ModulatorSet* set, int ext);

// Per-instance state of one modulator.
struct ModulatorState
{
    bool started = false;
    float value = 0.0f;
    float held = 0.0f;
    float phase = 0.0f;
    float previous = 0.0f; // Chaos: the iterate before `value`
    std::uint32_t rng = 1;
    std::uint32_t reg = 0;
    bool wasHigh = false;
    float env = 0.0f;
    float gateTime = 0.0f;
    double lx = 1.0, ly = 1.0, lz = 1.0;
    // Pitch tracker.
    double samples = 0.0;
    double lastCross = -1.0;
    float lastSample = 0.0f;
    bool armed = false;
    float logHz = 0.0f;
};

struct ModulatorContext
{
    int numSamples = 32;
    double sampleRate = 48000.0;
    bool voice = false; // evaluating for one voice (true) or the shared modulators of the engine (false)
    bool gate = false;  // a key is held (this voice's, or any key for the shared modulators)
    float velocity = 1.0f;
    float note = 60.0f;
    bool noteOn = false;          // a note started since the last evaluation
    const float* audio = nullptr; // mono mix of the previous chunk, for Follower and Pitch tracker
    int audioCount = 0;
};

inline bool extSourceBipolar(const ModulatorSet* set, int ext)
{
    if (ext < kNumModSources)
        return isBipolarSource(static_cast<ModSource>(ext));
    if (ext < kExtAudioFirst)
        return set != nullptr && set->bipolar[static_cast<size_t>(ext - kExtModulatorFirst)];
    return ext - kExtAudioFirst < kAudioEnvFirst;
}

// Evaluates the needed modulators of the right scope in index order. sources has kNumSourcesExt entries: the ModSource
// values on input, and the modulator outputs written to [kExtModulatorFirst + n]. A modulator that does not belong to
// this evaluation (voice scope in a shared evaluation, shared scope in a voice evaluation) is left as it is: the engine
// puts the shared values there before the voices run.
void evaluateModulators(const ModulatorSet& set, ModulatorState* states, const ModulatorContext& ctx, float* sources);

// Restarts the states of the modulators that a note-on restarts (all voice modulators; none for a shared evaluation).
void restartVoiceModulators(const ModulatorSet& set, ModulatorState* states);

// Display names: "Modulator 3".
const char* modulatorTypeName(ModulatorType type);

} // namespace crucible
