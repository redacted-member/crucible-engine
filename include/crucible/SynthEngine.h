// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/Clips.h>
#include <crucible/EngineConfig.h>
#include <crucible/Fx.h>
#include <crucible/graph/Program.h>
#include <crucible/Modulation.h>
#include <crucible/Modulators.h>
#include <crucible/Scenes.h>
#include <crucible/MotionRecorder.h>
#include <crucible/module/ModuleRack.h>
#include <crucible/SmoothedValue.h>
#include <crucible/Spectral.h>
#include <crucible/Tuning.h>
#include <crucible/Voice.h>
#include <crucible/Zones.h>

#include <array>
#include <limits>
#include <memory>
#include <atomic>
#include <mutex>
#include <crucible/RtShared.h>
#include <string>
#include <vector>

namespace crucible
{

constexpr int kNumVoices = 16;
// Extra voices beyond the configured pool that only serve to let stolen and retriggered voices fade out for about
// 5 ms while the new note starts on a free voice.
constexpr int kStealSpareVoices = 8;
// Largest configurable voice pool, and the size of the per-voice state arrays (pool plus the steal spares).
constexpr int kMaxConfigVoices = 64;
constexpr int kMaxVoiceSlots = kMaxConfigVoices + kStealSpareVoices;

class SynthEngine
{
    friend struct EngineViews;

public:
    explicit SynthEngine(const EngineConfig& config = {});
    const EngineConfig& config() const { return cfg; }
    // The product this engine plays: its module registry and parameter model (owned by, or shared with, this
    // engine; never process-wide). Indices of values passed to setParameterValues() refer to model().params().
    const ModuleRegistry& registry() const { return *reg; }
    const ParameterModel& model() const { return *params_; }
    std::shared_ptr<const ModuleRegistry> sharedRegistry() const { return reg; }

    void prepare(double sampleRate);

    // Values are indexed like model().params(), in natural units.
    // Control thread: generates the factory wavetables the oscillators of `values` select. setParameterValues() never
    // does (it runs on the audio thread); a bare SynthEngine user calls this first with the values it is about to set.
    void ensureSelectedTables(const float* values, int count);
    void setParameterValues(const float* values, int count);

    // channel: the MIDI channel (0..15) of the note when MPE is on, otherwise -1.
    void noteOn(int note, float velocity, int channel = -1);
    void noteOff(int note);
    void allNotesOff();
    // Fades every voice out in about 5 ms and clears the held stack and the sustain pedal (audio thread, no allocation).
    void killAllVoices();
    // Silences everything at once: voices, held notes and the sustain pedal, and every effect's state (delay lines,
    // reverb tanks, convolution history, filters). No fade, no allocation; not while rendering.
    void reset();
    // The host renders offline (CLAP render mode, VST3 kOffline): effects with background builds wait for them on the
    // audio thread at any quality. Any thread; takes effect from the next block.
    void setRenderOffline(bool offline) { renderOffline.store(offline, std::memory_order_relaxed); }
    bool isRenderOffline() const { return renderOffline.load(std::memory_order_relaxed); }
    void setSustainPedal(bool down);
    void setPitchBendNormalised(float bend) { pitchBendNorm = bend; }
    // MPE member channels and poly aftertouch (audio thread).
    void setChannelBend(int channel, float normalised)
    {
        if (validChannel(channel))
            chBend[static_cast<size_t>(channel)] = std::clamp(normalised, -1.0f, 1.0f);
    }
    void setChannelPressure(int channel, float v)
    {
        if (validChannel(channel))
            chPressure[static_cast<size_t>(channel)] = std::clamp(v, 0.0f, 1.0f);
    }
    void setChannelSlide(int channel, float v)
    {
        if (validChannel(channel))
            chSlide[static_cast<size_t>(channel)] = std::clamp(v, 0.0f, 1.0f);
    }
    void setNotePressure(int note, float v);
    void setModWheel(float v) { modWheel = std::clamp(v, 0.0f, 1.0f); }
    void setAftertouch(float v) { aftertouch = std::clamp(v, 0.0f, 1.0f); }
    void setTempo(double bpm) { tempoBpm = std::clamp(bpm, 20.0, 400.0); }
    // MIDI CC value 0..1 (audio thread), a source for assignable routes.
    void setController(int cc, float value)
    {
        if (cc >= 0 && cc < 128)
            controllers[static_cast<size_t>(cc)] = std::clamp(value, 0.0f, 1.0f);
    }

    // User LFO shape: count points over one cycle (any count, resampled to kLfoShapePoints), values -1..1.
    // Safe to call from the message thread while audio is running.
    void setCustomLfoShape(int lfo, const float* points, int count);

    // Overwrites left/right with numSamples of audio.
    void render(float* left, float* right, int numSamples);
    // Sidechain input for the next render() call: numSamples samples per pointer (right may be null for mono), read
    // by the effects (vocoder). The pointers are forgotten when render() returns.
    void setAuxInput(const float* left, const float* right)
    {
        auxLeft = left;
        auxRight = right != nullptr ? right : left;
    }

    int activeVoiceCount() const;
    // Quad blocks the analog filter bank has filtered since prepare() (zero when batching is off or unused); rendering
    // thread. Includes the filter.analog nodes of a graph product, whose voice pairs run as quads.
    std::uint64_t batchedFilterBlocks() const;
    // Active slots (model slots limited by the EngineConfig).
    int oscillatorSlots() const { return numOsc; }
    int filterSlots() const { return numFilt; }

    // User wavetable slots (tables WavetableBank::numTables.. of the osc*_table parameters). Control thread.
    UserWavetableBank& userWavetables() { return userTables; }
    const UserWavetableBank& userWavetables() const { return userTables; }

    // User vowel table of the Formant filter model for a filter slot, used by every voice on that slot (see
    // makeFormantTable for the rules). A null table restores the built-in vowels. Control thread; the audio thread picks
    // the change up at the next render call. False with a reason in error on a bad slot.
    bool setFormantTable(int slot, const FormantTable* table, std::string& error);
    // Copy of the slot's user table, or false when the slot uses the built-in vowels.
    bool formantTable(int slot, FormantTable& out) const;
    // Sets FormantTable::saturate of the slot's user table; false when there is none.
    bool setFormantSaturation(int slot, bool on, std::string& error);

    // Audio and clouds for the sample / granular and cloud oscillators, one per oscillator slot. Control thread.
    SampleBank& sampleBank() { return samples; }
    const SampleBank& sampleBank() const { return samples; }
    // Multisample zones (extra samples and zone lists) of the sample oscillator slots. Control thread.
    ZoneBank& zoneBank() { return zonesBank; }
    const ZoneBank& zoneBank() const { return zonesBank; }
    CloudBank& cloudBank() { return clouds; }
    const CloudBank& cloudBank() const { return clouds; }
    // What a UI draws for a slot: the sample overview with playheads and grains of all voices, and the cloud with
    // the energy of each point. Control thread (reads atomics the audio thread writes).
    struct SampleView
    {
        std::string name;
        double sampleRate = 0.0;
        int frames = 0;
        float detectedNote = -1.0f;
        std::vector<float> peakMin, peakMax;
        std::vector<float> slices; // slice markers, 0..1
        std::vector<Zone> zones;   // multisample zones of the slot (empty = none)
        OscVisual visual;
    };
    bool sampleView(int slot, SampleView& out) const;
    // Everything in sampleView() except the voices' playheads and grains (`visual` stays empty): reads only published
    // data, so it works from any thread except the audio thread. The pulled views use this one.
    bool sampleSnapshot(int slot, SampleView& out) const;
    // Control thread: builds the lazily allocated state of oscillator type `type` in slot `slot` (0-based) of every voice.
    void armOscillator(int slot, int type) const
    {
        for (const auto& v : voices)
            v.armOscillator(slot, type);
    }
    bool cloudView(int slot, CloudSnapshot& out) const;

    // ---- Vector field (perf.vector). The morph is an overlay applied at every 32-sample control chunk: while
    // vector_on is set (and a corner scene is stored) the scene-managed parameters follow the four corner scenes
    // instead of the values the host passes, so it renders identically offline and never produces automation.
    // The bank supplies the scenes; attach it on the control thread before audio starts, it must outlive rendering.
    void attachScenes(SceneBank* bank) { sceneSource.store(bank, std::memory_order_release); }
    // Parameters flagged here (one entry per parameter, non-zero = excluded) keep the host value while the vector
    // field is on, e.g. sequencer parameter locks. The array must outlive the engine.
    void setVectorExclusions(const char* mask) { vectorExclude = mask; }
    // The mask in force (null: the field owns every parameter).
    const char* vectorExclusions() const { return vectorExclude; }
    // The puck as the morph uses it (after smoothing, orbit and drift), 0..1; any thread.
    void vectorPosition(float& x, float& y) const
    {
        x = shownX.load(std::memory_order_relaxed);
        y = shownY.load(std::memory_order_relaxed);
    }
    bool vectorActive() const { return overlayLive.load(std::memory_order_relaxed); }

    // ---- Clip transport (control thread). Every clip plays from the start (all clips are running by default,
    // which is the behaviour without these calls). Stopping fades the clip's modulation out over 10 ms and parks
    // its playheads; starting restarts the clip from its beginning. Patches store which clips are stopped.
    void startClip(int clip);
    void stopClip(int clip);
    bool isClipPlaying(int clip) const;
    // Bit k set = clip k is wanted running (the state of startClip / stopClip, whether or not clip k exists).
    std::uint32_t clipRunMask() const { return clipWanted.load(std::memory_order_acquire); }
    // Sets the whole run state at once; a clip that goes from stopped to running restarts from its beginning,
    // running clips that stay running are not disturbed.
    void setClipRunMask(std::uint32_t mask);

    // Host transport at the start of a block (clip playback locked to the host position).
    void setHostTransport(bool hasPpq, bool playing, double ppq) { clipPlayer.setHostTransport(hasPpq, playing, ppq); }
    // Automation clips (control thread edits, see crucible/Clips.h); their lanes are route sources.
    ClipStore& clips() { return clipStore; }
    const ClipPlayer& clipPlayback() const { return clipPlayer; }
    MotionRecorder& motion() { return motionRecorder; }
    // Microtuning (control thread), see crucible/Tuning.h. tune_amount blends it with 12-TET.
    TuningStore& tuning() { return tuningStore; }

    // Assignable modulation routes (control thread edits, see crucible/Modulation.h).
    ModRouteStore& modRoutes() { return routes; }
    // Value of a parameter after global modulation (generic targets and routes), as of the last render call.
    // For per-voice destinations this is the unmodulated value. Any thread.
    float modulatedValue(int paramIndex) const;
    // Value of a parameter before modulation, as of the last render call (what the engine was given, smoothed). Any thread.
    float baseValue(int paramIndex) const;
    // Output of modulator n of the bank (0-based) as of the last render call: the shared value for layer and global
    // modulators, the newest active voice's for voice modulators (0 when none runs). Any thread.
    float modulatorValue(int n) const;
    // Like modulatedValue(), but for per-voice destinations (oscillator, filter and noise / sub parameters with a
    // fixed matrix destination, and ParameterMap::voiceTargets()) it returns the value of the newest active voice
    // as of the last render call, with matrix slots and assignable routes applied. Equals modulatedValue() when
    // no voice is active or the parameter is not modulated per voice. Any thread.
    float voiceModulatedValue(int paramIndex) const;
    // Clip playheads for UIs, published at the end of each render call (any thread). Positions are in clip units.
    void clipPositions(std::array<double, kMaxClips>& out) const;
    // One entry per voice slot: the voice's head position, or -1 when that voice does not play the clip.
    void voiceClipPositions(int clip, std::vector<double>& out) const;

    // The effect chain (registered Effect modules), for control-thread access to capabilities such as user
    // impulse responses and gain-reduction meters.
    ModuleRack& effects() { return fx; }

    // ---- Graph products (crucible/graph/Compiler.h).
    // The product this engine was configured with or last switched to (null for an engine built from a plain layout).
    const graph::CompiledProduct* product() const { return productNow.get(); }
    // The program the audio thread runs (null for an engine without a compiled product that was never reconfigured);
    // for read-back and tests. Valid until the next reconfiguration completes.
    const graph::Program* program() const { return activeProgram; }
    // Switches to another compiled product with the same parameters (compileReconfiguration()): another FX topology,
    // other layers, chokes or edges. Control thread; allocates; the audio thread picks the new program up at the next
    // control cell without locks or allocation. The output fades down over 10 ms, the new graph starts, and the output
    // fades back up over 10 ms. Returns false (and sets *error) when the product does not fit this engine.
    bool reconfigure(std::shared_ptr<const graph::CompiledProduct> product, std::string* error = nullptr);
    // True while a reconfiguration is fading (audio thread state, readable from any thread).
    bool reconfiguring() const { return fadeState.load(std::memory_order_relaxed) != 0; }
    // Programs the host still keeps alive (the active one and anything not yet collected); control thread.
    int livePrograms() const { return programHost.liveCount(); }

private:
    struct Indices
    {
        struct Osc
        {
            int table, pos, semi, fine, level, phase, voices, detune, spread;
            int warp, warpA, warpB, pan, route, warpSrc;
            int uniMode, uniBlend, uniRand, uniScatter;
            int type = 0, xmode = 0, xsrc = 0, xamt = 0;
            int wtInterp = -1, specWarp = -1, specAmt = -1; // spectral wavetable module, -1 when the product omits it
        };
        std::array<Osc, kMaxOscSlots> osc;
        int subLevel, subOctave, subWave, noiseLevel, noiseType, masterGain;
        struct Filt
        {
            int on, type, cutoff, res, drive, env, key, pan, mix;
            int character = -1, highpass = -1; // parameters of the analog filter controls module, -1 when absent
            int fmSrc = -1, fmAmt = -1;        // audio-rate modulation module (filterN_fm_*), -1 when absent
        };
        std::array<Filt, kMaxFilterSlots> filter;
        int filterRoute, subRoute, noiseRoute;
        std::array<std::array<int, 7>, kNumEnvs> env;
        struct Lfo
        {
            int rate, shape, amount, trig, sync, fade, delay, phase, smooth;
            int audio = -1, audioHz = -1, audioRatio = -1; // lfoN_audio, -1 when the product leaves the module out
        };
        std::array<Lfo, kNumLfos> lfo;
        struct Slot
        {
            int src, dst, amount, pol, curve;
        };
        std::array<Slot, kNumModSlots> slot;
        std::array<int, kNumMacros> macro;
        int stripL, stripR, morph;
        int voiceMode, glide, bendRange, poly, drift, transpose;
        int tuneAmount = -1, glideMode = -1, glideType = -1, monoPriority = -1, mpeOn = -1, mpeRange = -1;
        // The analog voice (voice.analog, registered last; -1 when a product leaves it out): envelope circuit per envelope,
        // voice card amounts, noise floor and hum.
        struct EnvShapeIndex
        {
            int mode = -1, retrigger = -1, overshoot = -1, velocityTime = -1, keyTime = -1;
        };
        std::array<EnvShapeIndex, kNumEnvs> envShape;
        int cardOsc = -1, cardFilter = -1, cardEnv = -1, cardLevel = -1;
        int noiseOn = -1, noiseDb = -1, humOn = -1, humDb = -1, humHz = -1;
        // The modulator bank (mod.bank, registered last; -1 when a product leaves it out).
        struct Bank
        {
            int type = -1, scope = -1, inA = -1, inB = -1, mode = -1, amount = -1, rate = -1, rise = -1, fall = -1,
                steps = -1, seed = -1;
            std::array<int, 16> pt{};
        };
        std::array<Bank, kNumBankModulators> bank;
    };

    struct ClassicHost final : graph::ClassicVoiceHost
    {
        SynthEngine* engine = nullptr;
        void renderClassicChunk(float* left, float* right, int n) override;
    };

    void renderClassicVoices(float* left, float* right, int n, float bend);
    void renderProgramVoices(float* left, float* right, int n, float bend);
    void pollProgram();
    void swapProgram();
    void applyFade(float* left, float* right, int n);
    void installProgram();
    bool graphVoices() const { return activeProgram != nullptr && activeProgram->voiceKind() != graph::VoiceKind::Classic; }
    void stepSmoothers();
    void updateVector(int numSamples);
    void updateGlobalMod(int numSamples);
    void buildModulators();
    const float* voiceRouteExtra(size_t voice, const Voice& v, int numSamples);
    const float* baseExtraTargets = nullptr;
    const float* voiceTargetExtraPtr = nullptr;
    std::array<float, ParameterMap::kMaxVoiceTargets> voiceTargetExtra{};
    size_t newestVoice() const;
    float applyVoiceMod(int paramIndex, float base, int dest, float m) const;
    void accumulateRoutesGlobal(const RouteSet& set, const float* sources, int numSamples);
    void buildParams();
    Voice* allocateVoice();
    void startNote(int note, float velocity, bool overlap);
    static bool validChannel(int c) { return c >= 0 && c < 16; }

public:
    struct Expression
    {
        float pressure = 0.0f, slide = 0.0f, bend = 0.0f;
    };

private:
    Expression expressionOf(size_t voice) const;
    int pickMonoNote(int exceptNote) const;

    EngineConfig cfg;
    std::shared_ptr<const ModuleRegistry> reg;
    const ParameterModel* params_;
    double sampleRate = 44100.0;
    Indices idx{};
    std::vector<LinearSmoothedValue> smoothers;
    std::vector<int> smoothedParams; // indices of the parameters with Smoothing::Linear or Log, stepped every control cell
    std::vector<ParamDef> const* table = nullptr;
    std::vector<float> current;
    bool initialised = false;
    VoiceParams params;
    spectral::SpectralSynth spectralSynth; // renders the cycles of the spectral wavetable path (Voice::spectralFrame)
    float masterGain = 1.0f;
    float pitchBendNorm = 0.0f;
    float modWheel = 0.0f;
    float aftertouch = 0.0f;
    double tempoBpm = 120.0;
    GlobalMod globalMod;
    // The modulator bank (Modulators.h): the snapshot of its parameters for this chunk, the state of the layer and global
    // modulators, and what they need of the notes and the audio. Audio thread, except the published values.
    ModulatorSet bankSet;
    std::array<ModulatorState, kNumBankModulators> sharedBank{};
    std::array<std::atomic<float>, kNumBankModulators> bankPublished{};
    std::uint32_t noteOnCount = 0, noteOnSeen = 0;
    float sharedVelocity = 1.0f, sharedNote = 60.0f;
    std::array<float, kChunkSize> sharedMix{}, sharedMixPrev{};
    int sharedMixPrevCount = 0;
    std::array<float, kMaxRoutes> routeDelta{};
    ModuleRack fx;
    // Graph products: programs published by the control thread, the one the audio thread runs and the one it fades to.
    graph::ProgramHost programHost;
    std::shared_ptr<const graph::CompiledProduct> productNow;
    graph::Program* activeProgram = nullptr;
    graph::Program* incomingProgram = nullptr;
    ClassicHost classicHost;
    float chunkBend = 0.0f;
    NoiseHum noiseHum; // the optional noise floor and mains hum of the analog stage
    CardAmounts cardAmountsNow() const;
    enum class Fade
    {
        Idle,
        Out,
        In
    };
    Fade fade = Fade::Idle;
    int fadePos = 0;
    int fadeLength = 480;
    std::atomic<int> fadeState{0};
    std::atomic<int> publishedVoiceCount{0};
    void publishVoiceCount();
    struct VoiceCountPublisher
    {
        SynthEngine& engine;
        ~VoiceCountPublisher() { engine.publishVoiceCount(); }
    };
    const float* auxLeft = nullptr;
    const float* auxRight = nullptr;
    std::array<float, kNumModDests> globalDest{};
    std::vector<float> genericDest;
    std::array<float, kNumModDests> routeVoiceGlobal{};
    std::array<float, kMaxRoutes> routeSmooth{};
    std::array<float, ParameterMap::kMaxVoiceTargets> routeTargetGlobal{};
    std::array<float, 128> controllers{};
    ModRouteStore routes;
    const RouteSet* pinnedRoutes = nullptr;
    TuningStore tuningStore;
    const TuningTable* pinnedTuning = nullptr;
    float tuneAmount = 1.0f;
    int glideMode = 0, glideType = 0, monoPriority = 0;
    bool mpeOn = false;
    float mpeRange = 48.0f;
    int pendingChannel = -1;
    std::array<int, kMaxVoiceSlots> voiceChannel{};
    std::array<float, kMaxVoiceSlots> voiceNotePressure{};
    std::array<float, kMaxVoiceSlots> voiceGlideScale{};
    std::array<float, 16> chBend{}, chPressure{}, chSlide{};
    ClipStore clipStore;
    ClipPlayer clipPlayer;
    MotionRecorder motionRecorder{clipStore, routes};
    // Vector field state (audio thread, except the atomics).
    struct VectorIndices
    {
        int on = -1, x = -1, y = -1, smooth = -1, drift = -1, driftRate = -1, orbitRate = -1, orbitDepth = -1,
            orbitShape = -1;
        std::array<int, 4> corner{-1, -1, -1, -1};
        int targetX = -1, targetY = -1; // positions in ParameterMap::modTargets()
    } vec;
    std::atomic<SceneBank*> sceneSource{nullptr};
    SceneBank* pinnedBank = nullptr;
    const SceneSnapshot* pinnedScenes = nullptr;
    const char* vectorExclude = nullptr;
    std::vector<float> hostValues; // the values the host last passed, natural units
    // Last argument and result of the log2 / exp2 that map a log-smoothed parameter to its smoothing domain and back. The
    // values hardly ever move between blocks, so the transcendental calls are skipped when the argument repeats.
    struct Memo
    {
        float key = std::numeric_limits<float>::quiet_NaN(), value = 0.0f;
    };
    std::vector<Memo> logMemo, expMemo;
    static float memoised(Memo& m, float x, float (*f)(float))
    {
        if (!(m.key == x))
        {
            m.key = x;
            m.value = f(x);
        }
        return m.value;
    }
    std::vector<int> sceneManaged; // indices of scene-managed parameters
    // Per overlay: the parameters that differ between the four corner scenes, with the corner values in
    // normalised space, so a chunk only blends what can change. Rebuilt when the scenes or the corner slots change.
    struct VectorParam
    {
        int index = 0;
        bool isFloat = true;
        float min = 0.0f, range = 1.0f, invSkew = 1.0f;
        bool linear = true;
        float lo = 0.0f, hi = 1.0f; // parameter range, kept here so the blend never touches the ParamDef
        Smoothing smoothing = Smoothing::Linear;
        std::array<float, 4> natural{}, norm{};
    };
    std::vector<float> varyingValue;  // scratch of the blend, one value per varying parameter
    std::vector<VectorParam> varying; // all four corners stored, values differ
    std::vector<int> hostDependent;   // at least one corner empty: blended against the host value every chunk
    std::uint64_t cachedGeneration = 0;
    std::array<int, 4> cachedSlots{{-1, -1, -1, -1}};
    bool cacheValid = false;
    float lastWeightX = -1.0f, lastWeightY = -1.0f;
    std::vector<char> lastExclude;
    bool lastExcludeUsed = false;
    std::vector<float> overlayExact; // log-smoothed parameters: the overlay's natural target, or -1
    bool vecWasOn = false;
    float vecX = 0.5f, vecY = 0.5f;
    double orbitPhase = 0.0;
    float driftX = 0.0f, driftY = 0.0f, driftTargetX = 0.0f, driftTargetY = 0.0f;
    double driftTimer = 0.0;
    std::uint32_t vecRng = 1;
    std::atomic<bool> overlayLive{false};
    std::atomic<bool> renderOffline{false}; // ModuleContext::renderOffline points here
    std::atomic<float> shownX{0.5f}, shownY{0.5f};
    bool lastLive = false;
    int controlPhase = 0; // samples rendered into the current 32-sample control cell
    // Clip transport: wanted state per clip (bit set = playing) and restart requests, written by any thread.
    std::atomic<std::uint32_t> clipWanted{0xffffffffU};
    std::atomic<std::uint32_t> clipRestart{0};
    const ClipBank* pinnedClips = nullptr;
    const float* baseExtra = nullptr;
    std::array<float, kNumModDests> voiceExtra{};
    std::vector<float> voiceRouteSmooth; // voices x kMaxRoutes
    std::unique_ptr<std::atomic<float>[]> modulated;
    std::unique_ptr<std::atomic<float>[]> published; // the unmodulated values, for baseValue()
    std::array<std::atomic<float>, 64> voiceDestMod{};
    std::array<std::atomic<float>, ParameterMap::kMaxVoiceTargets> voiceTargetMod{};
    std::vector<int> paramDest; // per parameter: fixed matrix destination (ModDest as int) or 0
    std::array<Lfo, kNumLfos> freeLfos;
    std::array<std::array<std::array<float, kLfoShapePoints>, 2>, kNumLfos> customShapes{};
    std::array<std::atomic<int>, kNumLfos> customActive{};
    int numOsc = kNumOscs;
    int numFilt = kNumFilters;
    int modelFilters = kNumFilters;
    int voiceMode = 0;
    int polyLimit = 16;
    float bendRange = 2.0f;
    std::uint64_t noteCounter = 0;
    float lastNote = -1.0f; // the note started last (fed to the effects as EffectInputs::note())
    bool sustainDown = false;
    struct HeldNote
    {
        int note;
        float velocity;
        bool released;
        int channel;
    };
    std::array<HeldNote, 64> held{};
    int heldCount = 0;
    std::vector<Voice> voices; // heap allocated: each voice carries filter delay lines
    // The nonlinear filter models of the sounding voices run four lanes at a time (FilterBank.h); renderClassicVoices
    // drives it. stagedVoice marks the voices that took part in the chunk.
    FilterBank filterBank;
    std::vector<unsigned char> stagedVoice;
    // Scratch of the live sample and cloud views (crucible/Views.h), reserved up front so publishing never allocates.
    OscVisual viewVisual;
    std::vector<float> viewEnergy;
    UserWavetableBank userTables;
    struct FormantSet
    {
        std::array<FormantTable, kMaxFilterSlots> table;
        std::array<bool, kMaxFilterSlots> used{};
    };
    RtShared<FormantSet> formantSets;
    std::mutex formantMutex;
    const FormantSet* pinnedFormants = nullptr;
    SampleBank samples;
    ZoneBank zonesBank;
    CloudBank clouds;
    SampleBank::Pinned pinnedSamples{};
    SampleBank::PinnedSlices pinnedSlices{};
    ZoneBank::Pinned pinnedZones{};
    CloudBank::Pinned pinnedClouds{};
    UserWavetableBank::Pinned pinnedTables{};
};

} // namespace crucible
