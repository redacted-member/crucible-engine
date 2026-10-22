// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/AudioFile.h>
#include <crucible/CcMap.h>
#include <crucible/KeyBus.h>
#include <crucible/KeyDetect.h>
#include <crucible/KeyLock.h>
#include <crucible/Cloud.h>
#include <crucible/EngineCore.h>
#include <crucible/fx/IrImport.h>
#include <crucible/fx/ShaperCurve.h>
#include <crucible/Meter.h>
#include <crucible/ParameterMap.h>
#include <crucible/Scenes.h>
#include <crucible/Sequencer.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace crucible
{

// CrucibleEngine embedding facade.
//
// Threads:
//  - control thread (UI, OSC, preset loading): parameters, scenes, sequencer pattern edits;
//  - input thread (a MIDI callback, or the control thread): noteOn/noteOff/pitchBend/... and midi(). These calls
//    form a single-producer queue, so call them from one thread at a time;
//  - audio thread: process() only.
// Parameter setters store into atomics and take effect at the start of the next process() call; performance
// input is applied at the start of the next block. For sample-accurate MIDI pass a MidiInput array to process().
// process() never allocates, locks or blocks.
namespace graph
{
class NodeRegistry;
}

class Engine
{
public:
    explicit Engine(const EngineConfig& config = {});
    // The one way to build an engine from a product definition (JSON, format "crucible-product", see
    // crucible/graph/Product.h). `base` carries the other construction settings (voices, block size, quality, ...);
    // its registry / product / compiled are replaced by the compiled product. `nodes` adds module-provided node and
    // effect types to the built-in ones (nullptr: built-ins only). Returns nullptr and sets `error` (the validation
    // errors, one per line) when the definition does not parse or compile.
    static std::unique_ptr<Engine> fromProduct(const std::string& json, std::string& error, EngineConfig base = {},
                                               const graph::NodeRegistry* nodes = nullptr);
    ~Engine();
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // Not real-time safe. Call before the first process() and whenever the sample rate changes.
    // maxBlockSize is the largest block process() will be asked for; larger blocks are split internally.
    void prepare(double sampleRate, int maxBlockSize);
    double sampleRate() const { return rate; }

    // ---- The product this instance plays. Registry and parameter model belong to the instance (or are shared with
    // the engines that were given the same EngineConfig::registry); there is no process-wide state, so engines with
    // different products can run side by side. Self-description for UIs: describeJSON() (below) and
    // crucible/Introspection.h.
    const ModuleRegistry& registry() const { return core->engine().registry(); }
    const ParameterModel& model() const { return core->engine().model(); }
    // The whole product as JSON (modules, instances, ports, parameters with ranges, units, groups, display
    // formats and flags). parseProductDescription() reads it back; see crucible/Introspection.h.
    std::string describeJSON() const;
    // The parameters a plug-in host lists (indices into model().params(), in the product's declared order): the product's
    // "surface", or every automatable parameter in index order when it declares none. Hidden parameters still load
    // from patches and stay modulation targets.
    const std::vector<int>& surface() const { return model().surface(); }
    // Display text of a natural value ("440 Hz", "-6 dB", "C4", "On") and back; see ParameterModel::valueToText.
    // Whether choice k of a choice parameter can be used in this product (see ParameterModel::choiceAvailable).
    bool choiceAvailable(int index, int choice) const { return model().choiceAvailable(index, choice); }
    std::string valueToText(int index, float natural) const { return model().valueToText(index, natural); }
    bool textToValue(int index, const std::string& text, float& natural) const
    {
        return model().textToValue(index, text, natural);
    }

    // ---- Who changed a parameter, as reported by paramChanges().
    enum class ParamSource : int
    {
        Host = 0,    // setParam* / setAllParams / setParamRt called by the embedding host
        Midi = 1,    // a mapped MIDI CC or CC learn
        Internal = 2 // patch load, scene recall / morph, reset to defaults, sequencer length / pattern coupling
    };
    // Marks the parameters set through the control-thread setters inside its lifetime with `source` (the engine's own
    // loaders use it; hosts rarely need it). Control thread only.
    class SourceScope
    {
    public:
        SourceScope(Engine& e, ParamSource source) : engine(e), previous(e.mainSource)
        {
            e.mainSource = static_cast<int>(source);
        }
        ~SourceScope() { engine.mainSource = previous; }
        SourceScope(const SourceScope&) = delete;
        SourceScope& operator=(const SourceScope&) = delete;

    private:
        Engine& engine;
        int previous;
    };
    // Pseudo indices in paramChanges(): the engine latency / the engine tail changed since the previous poll.
    static constexpr int kLatencyChanged = -1;
    static constexpr int kTailChanged = -2;
    // The modulation route list changed since the previous poll (add, remove, patch load, scene), by any source.
    static constexpr int kRoutesChanged = -3;
    // Control thread. Drains the set of exposed parameters changed since the last call, in index order, into indices[] and
    // sources[] (ParamSource of the latest change; sources may be null). Hidden parameters (ParamDef::exposed false) are
    // never reported: a change to one, internal or not, is dropped. Lock free, allocation free; writers on the
    // audio thread (setParamRt, mapped CCs) and on the control thread mark it. If the engine latency
    // (latencySamples()) changed since the previous call, the first entry is kLatencyChanged with source Internal; a changed
    // tailSeconds() follows it as kTailChanged, and a changed route list (modRoutes().generation()) as kRoutesChanged, both
    // with source Internal. Returns the number of entries written (at most cap); the rest stay marked for the next call.
    int paramChanges(int* indices, int* sources, int cap);
    // Samples between an input sample and the output sample that carries it: the longest latency among the master
    // paths of the FX graph, a path being the sum of the latencies reported (LatencyReporter) by its enabled modules;
    // parallel lanes take the larger one. Voices add none. Published by the audio thread after each block (and by prepare),
    // so a change shows from the next block on; one atomic load, any thread.
    int latencySamples() const;
    // Longest tail in seconds among the enabled modules that report one (reverb, delay feedback, convolution); HUGE_VAL
    // when one is infinite (freeze). Voice release time is not included. Published like latencySamples().
    double tailSeconds() const;
    // Clears delay, reverb, convolution and other effect states, voices, held notes and queued performance events
    // without re-preparing, allocating or locking. Audio thread (between process calls) or a control thread while no process
    // call runs; never concurrently with process().
    void reset();
    // Tells the engine the host renders offline (CLAP render mode, VST3 kOffline), or in real time again. While set,
    // fx.convolution and fx.cabinet wait on the audio thread for their background builds at any quality (Quality::High
    // always does), so an offline render right after loadImpulseResponse or a cabinet change does not depend on thread
    // timing. Never set it for live playback: the audio thread may then block for a build. Any thread; takes effect from
    // the next block.
    void setRenderOffline(bool offline) { core->engine().setRenderOffline(offline); }
    bool isRenderOffline() const { return core->engine().isRenderOffline(); }

    // ---- Parameters (any control thread). Values are natural units (Hz, seconds, dB, choice index...).
    bool setParam(int index, float natural);           // clamped to the range; false for a bad index
    bool setParam(std::string_view id, float natural); // no allocation for a literal
    // Sets a parameter from display text ("1.2k", "Analog"); false for an unknown id or text that does not parse.
    bool setParamText(std::string_view id, const std::string& text);
    bool setParamText(int index, const std::string& text);
    // Index of the parameter with this host id (ParamInfo::hostId), or -1.
    int paramIndexByHostId(std::uint32_t hostId) const { return model().indexOfHostId(hostId); }
    bool setParamNormalised(int index, float normalised01);
    // Audio-thread form for plug-in hosts (CLAP / VST3 deliver parameter changes inside process()): clamps, stores
    // atomically and, for parameters with heavy side effects (osc*_type, osc*_table, seq_pattern, seq_length), marks
    // them for service(). Never allocates or locks. Until service() has run, such a change is stored but its heavy
    // state (wavetables, oscillator arming, sequencer pattern sync) is not built yet.
    // `source` (ParamSource) is recorded for paramChanges(); the default marks it as a host edit.
    bool setParamRt(int index, float natural, int source = static_cast<int>(ParamSource::Host));
    // Control thread (a timer, or the host's main thread): performs the work setParamRt() deferred. Cheap when
    // nothing is pending. Offline tools that only use setParam() never need it.
    void service();
    // Any thread: true when setParamRt() or a mapped CC left work for service(). A wrapper that runs process_events
    // offline checks it after the block and calls service() (or asks its host for a main-thread callback).
    bool servicePending() const;
    // Mapped MIDI CCs reaching process() use setParamRt(), so call service() regularly when CCs may drive such parameters.
    float getParam(int index) const;
    float getParam(std::string_view id) const;
    void resetParamsToDefaults();
    std::vector<float> getAllParams() const;
    void setAllParams(const std::vector<float>& naturalValues);

    // ---- Performance input (single producer). Notes are MIDI numbers 0..127, velocity 0..1.
    bool noteOn(int note, float velocity);
    bool noteOff(int note);
    bool pitchBend(float normalised); // -1..1, scaled by the voice_bend range
    bool modWheel(float amount01);
    bool aftertouch(float amount01); // channel pressure
    bool sustain(bool down);
    bool allNotesOff();
    // Panic: every voice fades out within about 5 ms (no release tail), and the held-note stack, the sustain pedal, the
    // arpeggiator's held keys and the sequencer's sounding notes are cleared. Audio thread safe, no allocation.
    bool killAllVoices();
    // Raw MIDI channel-voice message (any channel). Notes, pitch bend, channel pressure, CC 1 (mod wheel), CC 64
    // (sustain), CC 120/123 (all off) are handled; any other CC drives the parameter mapped with mapCC().
    bool midi(std::uint8_t status, std::uint8_t data1, std::uint8_t data2);
    // Maps a MIDI CC number (0..127) on any channel to a parameter index; the 0..127 value is applied as a
    // normalised position. paramIndex -1 removes the mapping. CC 1, 64, 120 and 123 are reserved. This is the simple
    // form of ccMap(), which holds full bindings (range, curve, relative encoders, channel) and CC learn.
    void mapCC(int cc, int paramIndex);
    int ccMapping(int cc) const;
    CcMapStore& ccMap() { return ccStore; }

    // ---- Audio thread.
    // Renders numSamples into left / right (overwritten). midi, if given, must be sorted by position and is applied
    // sample accurately; host, if given, provides tempo and transport for tempo sync and the sequencer.
    // auxLeft / auxRight: optional sidechain input (numSamples samples, right may be null for mono) for effects
    // that listen to an external signal, such as the vocoder's modulator. Not modified.
    void process(float* left, float* right, int numSamples, const MidiInput* midiEvents = nullptr, int numMidi = 0,
                 const SeqHostTime* host = nullptr, const float* auxLeft = nullptr, const float* auxRight = nullptr);

    // ---- Meter (any thread).
    OutputMeter::Snapshot meter() const { return core->meter().snapshot(); }
    void resetClip() { core->meter().resetClip(); }
    // Master loudness (LUFS momentary / short-term), true peak, correlation, scope and spectrum (see
    // MasterAnalyzer); disable with analyzer().setEnabled(false) to save its CPU.
    MasterAnalyzer& analyzer() { return core->analyzer(); }
    LoudnessSnapshot loudness() const { return core->analyzer().loudness(); }

    // ---- Views (crucible/Views.h): the generic read-back API. Every accessor on this class that reads live state
    // (meter, loudness, scope and spectrum, modulatedValue, voiceModulatedValue, clipPositions, sampleView, cloudView,
    // the sounding notes) is also a view; subscribe on the control thread, read from any thread.
    ViewHub& views() { return core->views(); }
    const ViewHub& views() const { return core->views(); }

    // ---- Scenes (control thread). Slots 0..11.
    SceneBank& scenes() { return sceneBank; }
    void storeScene(int slot);
    bool recallScene(int slot);
    bool morphScenes(int slotA, int slotB, float t); // writes the morphed values into the parameters
    // Vector field: with vector_on set, the scene-managed parameters follow the scenes at the corners vector_a..d
    // (slots) as the puck vector_x / vector_y moves, as an overlay on the audio thread (the parameters themselves
    // keep the values you set; modulatedValue() reports the morphed ones). Scenes come from scenes().
    void vectorPosition(float& x, float& y) const { core->engine().vectorPosition(x, y); }
    // Clip transport: clips play from the start by default; stop fades one out, start restarts it.
    void startClip(int clip) { core->engine().startClip(clip); }
    void stopClip(int clip) { core->engine().stopClip(clip); }
    bool isClipPlaying(int clip) const { return core->engine().isClipPlaying(clip); }
    // Playhead of every clip in clip units, as of the last process() call (any thread).
    void clipPositions(std::array<double, kMaxClips>& out) const { core->engine().clipPositions(out); }

    // ---- Sequencer (control thread for edits; position getters are safe from any thread).
    SeqStore& sequencer() { return core->store(); }
    int sequencerStep() const { return core->sequencerStep(); }
    bool sequencerRunning() const { return core->sequencerRunning(); }
    bool popRecordedNote(RecordEvent& e) { return core->popRecord(e); }

    // ---- Assignable modulation (control thread), see crucible/Modulation.h.
    ModRouteStore& modRoutes() { return core->engine().modRoutes(); }
    // ---- Automation clips (control thread), see crucible/Clips.h. Lanes are route sources: clipLaneSource(clip, lane).
    ClipStore& clips() { return core->engine().clips(); }
    // ---- Microtuning (control thread), see crucible/Tuning.h.
    TuningStore& tuning() { return core->engine().tuning(); }
    // ---- Key lock (scale quantiser), see crucible/KeyLock.h. The key_* parameters set it.
    // Joins a key bus (KeyBus::local() inside one process; across processes a host supplies a KeyBus subclass over memory it
    // maps, see KeyBus.h); key_sync then follows or leads it. Control thread, before or while processing.
    void attachKeyBus(std::shared_ptr<KeyBus> bus);
    // Read-only view for UIs: absolute in-key pitch classes plus the root, as used in the last processed block
    // (with Key Sync = Follow this is the bus's key).
    KeyScaleView keyScale() const;
    // "Learn key": listens to the notes played; propose() returns root, scale and confidence.
    KeyDetector& keyDetector() { return core->keyDetector(); }
    // Motion recorder: arm with the parameter's current value as the reference, stop(), drain() from a UI timer.
    MotionRecorder& motion() { return core->engine().motion(); }
    bool recordMotion(const MotionSettings& settings);
    // A parameter's value after global modulation, as of the last process() call (any thread).
    float modulatedValue(std::string_view id) const { return core->engine().modulatedValue(model().indexOf(id)); }

    // ---- Arpeggiator pattern grid (control thread edits; the arp_* parameters switch and shape the arp).
    ArpPatternStore& arpPattern() { return core->arpPattern(); }
    int arpStep() const { return core->arpeggiator().currentStep(); }

    // ---- User content (control thread). Slots 0..7 are selected by osc*_table = WavetableBank::numTables + slot.
    UserWavetableBank& userWavetables() { return core->engine().userWavetables(); }
    bool loadWavetable(int slot, const AudioData& audio, const WavetableImport& options, const std::string& name,
                       std::string& error);
    bool loadWavetableFile(int slot, const std::string& wavPath, const WavetableImport& options, std::string& error);
    // User vowel table of the Formant filter model for filter slot 0.. (all voices); bands null restores the built-in vowels.
    // vowels x bandsPerVowel entries, row-major per vowel; rules in makeFormantTable(). Control thread.
    bool setFormantTable(int filterSlot, const FormantBand* bands, int vowels, int bandsPerVowel, float outputGain,
                         std::string& error);
    // The slot's user table (false when it uses the built-in vowels).
    bool formantTable(int filterSlot, FormantTable& out) const;
    // Switches the Hardware-style state saturation of the user table on filterSlot (FormantTable::saturate). False with a reason
    // when the slot is out of range or has no user table (the built-in vowels are never saturated). Setting a table again
    // turns it off. Control thread.
    bool setFormantSaturation(int filterSlot, bool on, std::string& error);
    // Renders the cloud into a user wavetable slot (64 frames along z).
    bool loadCloudWavetable(int slot, const CloudModel& cloud);
    // Sample / granular oscillator: audio for an oscillator slot (0-based; osc*_type = Sample / Granular), and the
    // cloud the cloud oscillator renders in that slot (starts as the harmonic morph model).
    SampleBank& samples() { return core->engine().sampleBank(); }
    bool loadSample(int slot, const AudioData& audio, const std::string& name, std::string& error);
    bool loadSampleFile(int slot, const std::string& wavPath, std::string& error);
    void clearSample(int slot) { samples().clear(slot); }
    // Slice markers (0..1 along the sample) of a slot's sample for the oscillator's slice modes (osc*_smp_slice_mode).
    // Loading or clearing a sample removes its markers. Control thread.
    bool setSampleSlices(int slot, std::vector<float> markers) { return samples().setSlices(slot, std::move(markers)); }
    std::vector<float> sampleSlices(int slot) const { return core->engine().sampleBank().slices(slot); }
    int sliceSampleEqually(int slot, int count) { return samples().sliceEqually(slot, count); }
    int sliceSampleByTransients(int slot, int maxSlices = 32, float sensitivity = 0.5f)
    {
        return samples().sliceByTransients(slot, maxSlices, sensitivity);
    }
    // Multisample zones of a slot (osc*_smp_zone_mode = Zones; design in docs/MULTISAMPLE_DESIGN.md). Zone sample id 0 is
    // the slot's own sample (loadSample), ids 1.. are added here. Control thread.
    ZoneBank& zones() { return core->engine().zoneBank(); }
    const ZoneBank& zones() const { return core->engine().zoneBank(); }
    int addZoneSample(int slot, const AudioData& audio, const std::string& name, std::string& error)
    {
        return zones().addSample(slot, audio, name, error);
    }
    bool setZones(int slot, std::vector<Zone> list) { return zones().setZones(slot, std::move(list)); }
    std::vector<Zone> zoneList(int slot) const { return zones().zones(slot); }
    int zoneSampleCount(int slot) const { return zones().sampleCount(slot); }
    void clearZones(int slot) { zones().clear(slot); }
    int importZoneFolder(int slot, const std::string& directory, std::string& error)
    {
        return zones().importFolder(slot, directory, error);
    }
    int importZoneSfz(int slot, const std::string& sfzPath, std::string& error)
    {
        return zones().importSfz(slot, sfzPath, error);
    }
    void setCloud(int slot, const CloudModel& cloud) { core->engine().cloudBank().set(slot, cloud); }
    void resetCloud(int slot) { core->engine().cloudBank().reset(slot); }
    // Waveform overview, playheads and grains / cloud points with their energy, for UIs (control thread).
    bool sampleView(int slot, SynthEngine::SampleView& out) const { return core->engine().sampleView(slot, out); }
    bool cloudView(int slot, CloudSnapshot& out) const { return core->engine().cloudView(slot, out); }

    // Impulse response of the convolution module (fx.convolution): WAV or AIFF file with 1, 2 or 4 (true-stereo) channels,
    // up to 30 s, any sample rate. The file is decoded here (control thread); the kernel is built off the audio thread.
    // Return false (error set) when the file is invalid or the module is not in the rack.
    bool loadConvolutionIrFile(const std::string& path, std::string& error);
    bool loadConvolutionIr(const ImpulseResponse& ir, const std::string& name);
    // Loads an impulse response file into one module instance, named by its module id ("fx.convolution", "fx.convolution.2"
    // for a second instance in a graph product). Control thread. Returns false (error set) for an
    // unknown instance, a module that takes no impulse response, or an invalid file.
    bool loadIr(const std::string& instanceId, const std::string& path, std::string& error);
    // Removes the impulse response of one module instance (same instance ids as loadIr). Returns false (error set) for an
    // unknown instance or a module whose response cannot be cleared.
    bool clearIr(const std::string& instanceId, std::string& error);

    // Transfer curve of one waveshaper instance (fx.shaper, fx.shaper.2, ...; see fx/ShaperCurve.h for the rules): 2..64
    // points, optionally symmetric (the positive half is drawn and mirrored). Published to the audio thread as an immutable
    // table and cross-faded in; saved in patches. Control thread. Returns false (error set) for an unknown instance, a module
    // without a curve or an invalid curve. setShaperCurve with no points (nullptr, 0) restores the default identity curve.
    bool setShaperCurve(const std::string& instanceId, const ShaperPoint* points, int count, bool symmetric,
                        std::string& error);
    // The curve last set, as given; false for an unknown instance or while it is the default identity.
    bool shaperCurve(const std::string& instanceId, std::vector<ShaperPoint>& out, bool& symmetric) const;

    // Direct access for hosts that need more (custom LFO shapes, voice counts...).
    EngineCore& coreEngine() { return *core; }
    int activeVoiceCount() const { return core->engine().activeVoiceCount(); }

private:
    void armSceneOscillators(const std::vector<float>& values);
    const ParamDef& info(int index) const;
    void syncSequencerParam(int index, float value);
    bool push(ExternalEvent::Type type, int note, float value, int channel = 0);
    // Applies a controller message to the mapped parameters (and learn); returns true when one was changed.
    // realtime: audio thread (process()), parameters go through setParamRt and wait for service().
    bool applyCc(int channel, int cc, int data2, bool realtime);
    bool setParamNormalisedRt(int index, float normalised01, int source);
    void markChanged(int index, int source);
    bool storeParam(int index, float value, int source);

    std::unique_ptr<EngineCore> core;
    SceneBank sceneBank;
    int seqLengthIndex = -1;
    int seqPatternIndex = -1;
    std::unique_ptr<std::atomic<float>[]> params;
    static constexpr size_t kMidiSliceCapacity = 1024; // events per chunk after the first one of a block
    std::vector<MidiInput> midiSlice;                  // audio thread: re-based events of a later chunk
    std::vector<float> snapshot;                       // audio thread copy of params
    std::unique_ptr<std::atomic<bool>[]> pending;      // setParamRt() changes waiting for service()
    std::vector<bool> needsService;
    std::atomic<bool> anyPending{false};
    std::unique_ptr<std::atomic<std::uint8_t>[]> changedBy; // 0: unchanged, else ParamSource + 1 of the latest change
    std::atomic<bool> anyChanged{false};
    int mainSource = 0; // ParamSource stamped by the control-thread setters (SourceScope)
    int reportedLatency = 0;
    double reportedTail = 0.0;
    std::uint32_t reportedRoutes = 0;
    CcMapStore ccStore;
    std::shared_ptr<KeyBus> bus;
    double rate = 48000.0;
    int maxBlock = 512;
};

} // namespace crucible
