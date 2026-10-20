// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/AnalogEnvelope.h>
#include <crucible/AudioLfo.h>
#include <crucible/Dsp.h>
#include <crucible/FilterBank.h>
#include <crucible/Filters.h>
#include <crucible/HalfBand.h>
#include <crucible/Cloud.h>
#include <crucible/MinBlep.h>
#include <crucible/ModMatrix.h>
#include <crucible/Modulators.h>
#include <crucible/module/Module.h>
#include <crucible/ParameterMap.h>
#include <crucible/Sample.h>
#include <crucible/UserWavetables.h>
#include <crucible/VoiceCard.h>
#include <crucible/Wavetables.h>

#include <array>
#include <memory>
#include <vector>

namespace crucible
{

namespace spectral
{
class SpectralSynth;
class SpectralTable;
} // namespace spectral

struct RouteSet;

constexpr int kMaxUnison = 8;
constexpr int kChunkSize = 32;
constexpr int kNumOscs = 3;    // oscillator slots with fixed matrix destinations (the default layout)
constexpr int kNumFilters = 2; // filter slots with fixed matrix destinations
// Mono buffers of a voice's audio bus: the routes' sources (at most kAudioBusBuffers) plus one for each filter's FM source.
constexpr int kAudioBusSlots = kAudioBusBuffers + kNumFilters;

enum class WarpMode
{
    Off,
    Fm,
    SyncHard,
    SyncSoft,
    BendPlus,
    BendMinus,
    Mirror,
    Pwm,
    Quantize,
    Asym,
    Flip,
    Formant,
    Smear,
    Ring,
    Fold,
    Count
};

struct OscParams
{
    int type = 0; // oscillator module (0 = wavetable, see module/Oscillator.h)
    // Cross modulation from an earlier slot: 0 off, 1 FM, 2 ring, 3 hard sync (see Voice.cpp).
    int xmode = 0;
    int xsrc = 0;
    float xamt = 0.0f;
    int xamtTarget = -1; // dense per-voice target of the amount
    int table = 0;
    float position = 0.0f;
    float semitones = 0.0f;
    float fineCents = 0.0f;
    float level = 0.0f;
    float phaseDegrees = 0.0f;
    int unisonVoices = 1;
    float detuneCents = 0.0f;
    float spread = 0.0f;
    int warp = 0;
    float warpA = 0.0f;
    float warpB = 0.0f;
    float pan = 0.0f;
    int route = 0;
    int warpSource = 0;
    // Dense per-voice modulation targets (position, pitch, level, warp A, warp B, pan) for slots beyond the fixed
    // matrix destinations; -1 when the slot uses ModDest.
    std::array<int, 6> targets{-1, -1, -1, -1, -1, -1};
    int uniMode = 0;
    float uniBlend = 1.0f;
    float uniRand = 1.0f;
    float uniScatter = 0.0f;
    // Spectral wavetable path (parameters of the spectral wavetable module): 0 keeps the legacy crossfade between frames,
    // 1 sweeps the spectral peaks; specWarp is a spectral::Warp, specAmount -1 .. 1. See Spectral.h.
    int wtInterp = 0;
    int specWarp = 0;
    float specAmount = 0.0f;
    int specAmountTarget = -1; // dense per-voice target of the amount
    // Dense per-voice targets of detune, spread, blend and scatter (-1: none).
    std::array<int, 4> uniTargets{-1, -1, -1, -1};
};

struct LfoParams
{
    LfoSettings settings; // rateHz already resolved against tempo sync
    float amount = 1.0f;
    // Audio-rate mode (lfoN_audio): 0 off, 1 fixed frequency, 2 ratio of the note's frequency. The LFO's shape is then
    // also played as a band-limited oscillator on the voice's audio bus.
    int audioMode = 0;
    float audioHz = 440.0f;
    float audioRatio = 1.0f;
};

// Control values shared by every voice for one chunk.
struct GlobalMod
{
    std::array<float, kNumMacros> macro{};
    std::array<float, 2> strip{};
    float modWheel = 0.0f;
    float aftertouch = 0.0f;
    float morph = 0.0f;
    float pitchBend = 0.0f;
    std::array<float, kNumLfos> freeLfo{}; // values of free-running LFOs, amount applied
    std::array<const float*, kNumLfos> customShape{};
    // Values of the layer and global modulators of the bank (Modulators.h), evaluated once for all voices.
    std::array<float, kNumBankModulators> bank{};
};

struct FilterParams
{
    bool on = true;
    int model = 0;
    float cutoff = 8000.0f;
    float resonance = 0.0f;
    float drive = 0.0f;
    float envAmount = 0.0f;
    float keytrack = 0.0f;
    float pan = 0.0f;
    float mix = 1.0f;
    // Voicing of the nonlinear models (index 17 and up): see AnalogFilter::setControls and setHighpassStep.
    float character = 0.0f;
    int highpass = 0;
    // Dense per-voice modulation targets (cutoff, resonance, drive, mix, pan) for filters beyond the fixed
    // matrix destinations; -1 otherwise.
    std::array<int, 5> targets{-1, -1, -1, -1, -1};
    // Audio-rate cutoff modulation (filterN_fm_src / filterN_fm_amt): audio source index + 1 (0 = off) and the amount,
    // in units of 5 octaves per unit of signal.
    int fmSource = 0;
    float fmAmount = 0.0f;
};

// Snapshot of every parameter a voice needs for one control chunk.
struct VoiceParams
{
    // Active slots: the first numOsc oscillators and numFilter filters of the arrays are used.
    int numOsc = kNumOscs;
    int numFilter = kNumFilters;
    int modelFilters = kNumFilters; // filters in the parameter model: the split-route "All" index
    std::array<OscParams, kMaxOscSlots> osc;
    float subLevel = 0.0f;
    int subOctave = 0;
    int subWave = 0;
    float noiseLevel = 0.0f;
    int noiseType = 0;
    std::array<FilterParams, kMaxFilterSlots> filter;
    int filterRoute = 0;
    int subRoute = 2;
    int noiseRoute = 2;
    std::array<AdsrParams, kNumEnvs> env; // 0 amp, 1 mod env, 2 and 3 free
    // Envelope circuit, retrigger mode and time scaling per envelope (the defaults are the legacy digital envelope), and
    // the amounts of the voice card's tolerances (all zero: no card, the legacy voice).
    std::array<EnvShape, kNumEnvs> envShape;
    CardAmounts card;
    std::array<LfoParams, kNumLfos> lfo;
    std::array<ModSlotParams, kNumModSlots> slots;
    float glide = 0.0f;
    float driftAmount = 0.0f;
    float transpose = 0.0f;
    // User wavetables pinned for the current render call (tables >= WavetableBank::numTables); may be null.
    const UserWavetableBank::Pinned* userTables = nullptr;
    // User vowel tables of the Formant filter model per filter slot, pinned for the current render call (null = built-in).
    std::array<const FormantTable*, kMaxFilterSlots> formants{};
    // Assignable modulation routes to voice destinations (evaluated like extra matrix slots) and, when not null,
    // a per-destination offset computed globally (routes driven by MIDI CC). Set for the current render call.
    const ModSlotParams* routes = nullptr;
    int numRoutes = 0;
    const float* globalExtra = nullptr;
    // Per-voice modulation of module parameters (ParameterMap::voiceTargets()): the current parameter values, the
    // assignable routes aimed at such parameters and the per-target offset of routes driven by MIDI CC. Set for
    // the current render call; null when unused.
    const float* values = nullptr;
    const ModSlotParams* targetRoutes = nullptr;
    int numTargetRoutes = 0;
    const float* globalExtraTargets = nullptr;
    // Data of the oscillator modules per slot (pinned for the current render call; null when empty).
    std::array<const SampleData*, kMaxOscSlots> sample{};
    std::array<const SliceSet*, kMaxOscSlots> slices{};
    std::array<const ZoneSet*, kMaxOscSlots> zones{};
    std::array<const CloudModel*, kMaxOscSlots> cloud{};
    // Engine seed mixed into per-note randomness (EngineConfig::seed).
    std::uint32_t seed = 0;
    // Synthesiser of the spectral wavetable path, owned by the engine; null when the path is not available.
    spectral::SpectralSynth* spectralSynth = nullptr;
    // The modulator bank and the route set (for amount modulation and the audio bus), pinned for the current render call;
    // null when no modulator runs / no route needs them.
    const ModulatorSet* modulators = nullptr;
    const RouteSet* routeSet = nullptr;
};

// Per-unison-voice layout derived from the stack mode.
struct UnisonVoice
{
    float semitones = 0.0f;
    float gainL = 0.0f;
    float gainR = 0.0f;
};

void computeUnisonLayout(const OscParams& p, float detuneCents, const std::array<float, kMaxUnison>& randA,
                         const std::array<float, kMaxUnison>& randB, float voiceGain,
                         std::array<UnisonVoice, kMaxUnison>& out);

// Modulation offsets of one oscillator slot (chunk rate).
struct SlotMod
{
    float position = 0.0f, pitch = 0.0f, level = 0.0f, warpA = 0.0f, warpB = 0.0f, pan = 0.0f, xamt = 0.0f;
    float uniDetune = 0.0f, uniSpread = 0.0f, uniBlend = 0.0f, uniScatter = 0.0f; // natural units
    float specAmount = 0.0f;
};

class Voice
{
public:
    // Sizes the per-slot state (control thread; before prepare()).
    void configure(int oscillators, int filterSlots, const ModuleRegistry& registry);
    void prepare(double rate, int quality = 1);
    // The voice's place in the engine's pool; with the engine seed it fixes the voice card (control thread).
    void setIndex(int index) { voiceIndex = index; }
    // detuneSemitones: static offset of this voice (the engine detunes a note that sounds on top of another voice at
    // the same pitch, so the two beat slowly instead of holding a fixed phase relation).
    void noteOn(int midiNote, float velocity, const VoiceParams& params, std::uint64_t noteAge, float startNote,
                float detuneSemitones = 0.0f);
    // Starts a note on a voice that may still be sounding (a steal when every voice is busy): the old sound fades out
    // over about 5 ms and the new note begins when the fade has finished. On a silent voice it is noteOn. `params` must
    // outlive the fade (the engine's parameter block does).
    void noteOnAfterFade(int midiNote, float velocity, const VoiceParams& params, std::uint64_t noteAge, float startNote,
                         float detuneSemitones = 0.0f);
    // True while a note waits for this voice's fade-out to end.
    bool hasPendingStart() const { return pending.on; }
    void glideTo(int midiNote);
    void noteOff();
    void kill();

    // The oscillator module instance of a slot and type (type >= 1), for UI snapshots; null if absent.
    OscillatorInstance* oscillatorInstance(int slot, int type) const;
    // Control thread: OscillatorInstance::arm() for the instance of a slot and type (see there).
    void armOscillator(int slot, int type) const
    {
        if (auto* inst = oscillatorInstance(slot, type))
            inst->arm();
    }

    bool isActive() const { return active; }
    bool isReleasing() const { return active && (fading || env[0].isReleasing()); }
    bool isFading() const { return active && fading; }
    // Ends the voice with a short cosine fade (about 5 ms) instead of cutting it: used when the voice is stolen or
    // when the same key is struck again.
    void fadeOut();
    // fadeOut() that also drops a note waiting for this voice (panic).
    void fadeOutNow()
    {
        pending.on = false;
        fadeOut();
    }
    int note() const { return pending.on ? pending.note : midiNote; }
    std::uint64_t age() const { return noteAge; }

    // Matrix and route modulation of the last chunk, for read-back: per fixed destination (ModDest) and per
    // per-voice parameter target (ParameterMap::voiceTargets(), valid up to the target count).
    const std::array<float, 64>& lastDestMod() const { return readoutDest; }
    const std::array<float, ParameterMap::kMaxVoiceTargets>& lastTargetMod() const { return readoutTarget; }
    // Outputs of the modulator bank as of the last chunk (voice modulators; zero for those that did not run).
    const std::array<float, kNumBankModulators>& lastModulators() const { return readoutBank; }

    // Adds numSamples (<= kChunkSize) of audio to left/right.
    void renderChunk(const VoiceParams& params, const GlobalMod& global, float pitchBendSemitones, int numSamples,
                     float* left, float* right);

    // The same work in stages, so that the nonlinear filter models of several voices can run together in a FilterBank
    // (see SynthEngine::renderClassicVoices). beginChunk() does everything ahead of the filters and returns false when
    // the voice does not sound. Then, for every filter slot in order, filterIn(slot), the bank's process() for the slot,
    // filterOut(slot); and finally endChunk(). renderChunk() is these calls with the voice's own filters. bank may be null;
    // bankVoice is the voice's index in the bank.
    bool beginChunk(const VoiceParams& params, const GlobalMod& global, float pitchBendSemitones, int numSamples,
                    FilterBank* bank, int bankVoice);
    void filterIn(int slot);
    void filterOut(int slot);
    void endChunk(float* left, float* right);

private:
    std::array<float, 64> readoutDest{};
    std::array<float, kNumBankModulators> readoutBank{};
    bool readoutBankLive = false;
    std::array<float, ParameterMap::kMaxVoiceTargets> readoutTarget{};
    struct OscState
    {
        std::array<float, kMaxUnison> phase{};
        std::array<float, kMaxUnison> prev{};
        std::array<float, kMaxUnison> randA{};
        std::array<float, kMaxUnison> randB{};
        float modPhase = 0.0f;
        float lastPosition = -1.0f; // table position at the end of the previous chunk (the morph ramps from it)
        std::array<MinBlepBuffer, kMaxUnison> blep; // hard sync to another slot
        HalfBandInterpolator up1, up2;              // cross modulation signal to the oversampled rate
        std::array<HalfBandDecimator, 2> decimator;
        std::array<HalfBandDecimator, 2> decimator2;
        // Cache of cycles synthesised by the spectral wavetable path: kSpecCache frames of the largest mip (plus guards),
        // keyed by what they were made from. Allocated by configure(); empty slots simply never use the path. The
        // vector path resolves the cycles of one group of four unison voices before rendering it, so the cache holds at
        // least four entries: no cycle a pending group reads is evicted.
        static constexpr int kSpecCache = 4;
        struct SpecKey
        {
            std::uint64_t analysis = 0; // SpectralTable::id(): a replaced table is a different key
            int table = -1, mip = -1, position = -1, interp = 0, warp = 0, amount = 0;
            bool operator==(const SpecKey&) const = default;
        };
        std::array<SpecKey, kSpecCache> specKey{};
        std::array<std::uint32_t, kSpecCache> specUsed{};
        std::uint32_t specTick = 0;
        std::vector<float> specData;
    };

    // monoOut (optional) receives the unit-level, pre-pan mono signal for cross modulation; levelOverride > 0
    // renders at that level when the slot's own level is zero (a slot used only as a modulator).
    void renderOscillator(int index, const VoiceParams& vp, const std::array<float, 64>& mod,
                          const std::array<SlotMod, kMaxOscSlots>& slotMod, float pitchSemis, int numSamples, float* outL,
                          float* outR, float* monoOut, float levelOverride);
    float noiseSample(int channel, int type);
    float bandLimit(int oversampling) const;
    // The cycle of the spectral wavetable path (Spectral.h) for the table position of this chunk and the mip level, from the
    // slot's cache (synthesised on a miss: arithmetic and one inverse transform, no analysis). Returns the frame with its
    // guard samples, or null when the path cannot serve this mip level.
    const float* spectralFrame(OscState& st, const spectral::SpectralTable& table, spectral::SpectralSynth& synth,
                               const OscParams& p, float amount, float position, float previousPosition, int mip);
    // Per-sample modulation of an oscillator slot from the audio bus (null pointers: none).
    struct OscAudio
    {
        const float* pitch = nullptr; // semitones
        const float* fm = nullptr;    // added to the cross-modulation amount
        const float* pw = nullptr;    // added to the pulse width
    };
    void renderModule(int index, const VoiceParams& vp, const std::array<SlotMod, kMaxOscSlots>& slotMod,
                      const float* targetMod, float pitchSemis, int numSamples, float* outL, float* outR, float* monoOut,
                      float levelOverride, const OscAudio* audio = nullptr);

    struct FilterRun
    {
        bool on = false;
        float mix = 1.0f, panL = 1.0f, panR = 1.0f;
    };
    // The chunk in flight between beginChunk() and endChunk().
    struct Chunk
    {
        int samples = 0;
        int numFilters = 0;
        bool serial = false;
        float ampMod = 1.0f;
        FilterBank* bank = nullptr;
        int bankVoice = 0;
        std::array<FilterRun, kMaxFilterSlots> run{};
        std::array<bool, kMaxFilterSlots> banked{}; // the slot's filter runs in the bank
        // Audio-rate modulation of the chunk: per-sample cutoff offsets (octaves) and amplifier gain; see Voice::beginChunk.
        std::array<bool, kMaxFilterSlots> hasCutoffMod{};
        bool hasAmpMod = false;
        float bus[kMaxFilterSlots][2][kChunkSize]; // one input per filter
        float in[kMaxFilterSlots][2][kChunkSize];  // what the filter of the slot received
        float dry[2][kChunkSize];
        float carry[2][kChunkSize]; // serial routing: output of the last filter stage
        float sum[2][kChunkSize];   // parallel and split routing: sum of the stage outputs
    };
    void emitStage(int slot, const float* wetL, const float* wetR);
    void finishChunk();
    // Builds the voice card when the parameters ask for one and it is missing or belongs to another seed.
    void ensureCard(const VoiceParams& p);
    void setEnvelopes(const VoiceParams& p);

    double sampleRate = 44100.0;
    int quality = 1; // Quality as int: 0 draft, 1 standard, 2 high
    bool active = false;
    struct Pending
    {
        bool on = false;
        bool released = false;
        int note = 60;
        float velocity = 1.0f;
        const VoiceParams* params = nullptr;
        std::uint64_t age = 0;
        float startNote = 60.0f;
        float detune = 0.0f;
    } pending;
    int midiNote = 60;
    float curNote = 60.0f;
    float velocity = 1.0f;
    std::uint64_t noteAge = 0;
    std::array<OscState, kMaxOscSlots> osc;
    // Oscillator module instances, [slot * oscTypes + type]; type 0 (wavetable) has none.
    int oscTypes = 1;
    const ParameterModel* model = nullptr; // set by configure()
    std::vector<std::unique_ptr<OscillatorInstance>> instances;
    std::array<int, kMaxOscSlots> startedType{};
    // Cross modulation: the unit-level mono signal of slots that other slots listen to, and the reference phase
    // (at the slot's centre pitch) that hard sync follows.
    std::array<std::array<float, kChunkSize>, kMaxOscSlots> modSignal{};
    struct Wraps
    {
        int count = 0;
        std::array<float, 64> time{}; // in samples from the start of the chunk, ascending
    };
    std::array<Wraps, kMaxOscSlots> wraps{};
    std::array<double, kMaxOscSlots> refPhase{};
    std::uint32_t noteSeed = 1;
    float staticDetune = 0.0f;
    bool fading = false;
    int fadeLength = 240;
    int fadeDone = 0;
    double noteTime = 0.0;
    float subPhase = 0.0f;
    float drift = 0.0f;
    int driftClock = 0;
    std::array<float, 2> pink0{}, pink1{}, pink2{};
    std::array<float, 2> brown{};
    std::array<float, 2> crackle{};
    std::array<AnalogEnvelope, kNumEnvs> env;
    std::array<Lfo, kNumLfos> lfos;
    std::array<float, 4> lfoRateMod{};
    float noteRandom = 0.0f;
    std::vector<FilterUnit> filters = std::vector<FilterUnit>(kNumFilters);
    // The voice card (seed + voice index); cardOn is false while all amounts are zero, and then nothing of it applies.
    VoiceCard card;
    bool cardBuilt = false;
    bool cardOn = false;
    CardAmounts cardAmounts;
    float cardTune = 0.0f; // semitones, for the oscillators that do not draw their own tolerance
    float cardGain = 1.0f;
    int voiceIndex = 0;
    // The modulator bank and the audio bus (P12). Their state lives in a block of its own, allocated by configure() and
    // touched only by a patch that uses them, so the voice's hot members stay where they were.
    struct Extra
    {
        std::array<ModulatorState, kNumBankModulators> bankState{};
        std::array<AudioLfo, kNumLfos> audioLfos;
        struct AudioBus
        {
            std::array<std::array<float, kChunkSize>, kAudioBusSlots> buf{};
            std::array<signed char, kNumAudioSources> slotOf{}; // buffer of an audio source in this chunk, -1 = none
            std::array<bool, kNumAudioSources> filled{};
            std::array<float, kNumAudioSources> lastMean{}; // mean of an oscillator source in the previous chunk
            int used = 0;
        } audio;
        // Work space of beginChunk() that only audio-rate routes and amount modulation touch.
        struct Scratch
        {
            std::array<float, kMaxRoutes> amountDelta;
            std::array<std::array<float, kChunkSize>, 5>
                osc; // per-sample pitch, FM amount, cross amount, pulse width, position
        } scratch;
        std::array<float, kChunkSize> mixNow{},
            mixPrev{};                                // mono oscillator mix of this and the previous chunk (follower, pitch)
        float cutoffMod[kMaxFilterSlots][kChunkSize]; // per-sample cutoff offsets (octaves) of the chunk, see beginChunk()
        float ampBuf[kChunkSize];                     // per-sample amplifier gain of the chunk
    };
    std::unique_ptr<Extra> extra;
    bool bankRestart = true; // a note started: the voice modulators restart at the next chunk, and note-on clocks fire
    int mixPrevCount = 0;
    std::array<int, kMaxFilterSlots> bankModel{}; // analog model the voice's bank lanes hold per slot, -1 = none
    Chunk chunk;
    Rng rng;
};

} // namespace crucible
