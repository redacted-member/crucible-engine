// SPDX-License-Identifier: MIT
#pragma once

// Module system. Every DSP building block that a product can expose (effects today; oscillators, filters,
// modulators and performance pieces as they are converted) is described by a ModuleDescriptor: a stable id, a
// display name, a kind and category, a version, a CPU class, its parameters (with page placement) and, for
// processing modules, a factory. A ModuleRegistry holds the descriptors of one product in registration order; the
// parameter model (ParameterModel) and the runtime racks are built from it. Registries are ordinary objects owned
// by the engine instances that use them, so several products can live in one process. See
// engine/docs/MODULE_GUIDE.md.

#include <crucible/ParameterMap.h>
#include <crucible/module/Oscillator.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace crucible
{

enum class ModuleKind
{
    Voice, // a complete voice architecture (oscillators, filters, envelopes, matrix)
    Oscillator,
    Filter,
    Effect,      // stereo bus processor in the FX chain
    Modulator,   // modulation source (LFO, envelope, clip, MSEG...)
    Performance, // macros, controllers, scenes
    Sequencer    // step sequencer, arpeggiator
};

const char* moduleKindName(ModuleKind kind);

// Whether a module is the one to use in a new product. A legacy module stays registered so that existing patches
// render bit-exactly; supersededBy names the module that replaces it.
enum class ModuleStatus
{
    Current,
    Legacy
};

const char* moduleStatusName(ModuleStatus status);

enum class CpuClass
{
    Light,  // a few operations per sample
    Medium, // filters, modulated delays
    Heavy   // oversampling, convolution, large FDNs
};

// Oversampling / quality policy handed to modules; each module documents what it does with it.
enum class Quality
{
    Draft,    // lowest CPU, for previews
    Standard, // the default, real-time
    High      // offline rendering
};

// Filter family of the oversamplers (see crucible/Oversampler.h).
enum class OversamplingFilter
{
    HalfBand, // linear-phase FIR half-band (23 / 47 / 95 taps)
    Polyphase // all-pass pair IIR half-band (4 / 6 / 8 coefficients), steep and minimum latency
};

struct ModuleContext
{
    double sampleRate = 48000.0;
    int maxBlockSize = 512;
    Quality quality = Quality::Standard;
    OversamplingFilter oversampling = OversamplingFilter::HalfBand;
    std::uint32_t seed = 1;
    // Graph voices run the analog filters of voice pairs as one four-lane object (see EngineConfig::batchAnalogFilters).
    bool batchAnalogFilters = true;
};

// What an effect reads each control block: its own parameters by local index (the order of
// ModuleDescriptor::params), already smoothed and with modulation applied, plus shared context.
class EffectInputs
{
public:
    float value(int local) const
    {
        const int g = globalIndex[local];
        const float v = current[g];
        if (genericTarget == nullptr || genericTarget[local] < 0)
            return v;
        const auto& d = defs[g];
        const float m = v + genericDest[genericTarget[local]] * (d.max - d.min);
        return m < d.min ? d.min : (m > d.max ? d.max : m);
    }
    bool on(int local) const { return value(local) > 0.5f; }
    // Truncating conversion, as the classic effects read their choices.
    int choice(int local) const { return static_cast<int>(value(local)); }
    // Rounded conversion for integer-valued parameters.
    int rounded(int local) const;
    // Output of a fixed modulation destination (the classic effect destinations of the matrix).
    float legacyDest(ModDest d) const { return legacyDests[static_cast<int>(d)]; }
    double bpm() const { return tempo; }
    // Auxiliary (sidechain) audio input for this block, one sample per bus sample, or null when the host supplies
    // none. A mono input has auxRight() equal to auxLeft().
    const float* auxLeft() const { return auxL; }
    const float* auxRight() const { return auxR; }
    bool hasAux() const { return auxL != nullptr; }
    // The note the engine played last, in MIDI note units (fractional notes of a tuning included), or a negative
    // value while no note has been played since the engine was reset. Any effect may follow it (note tracking); it
    // changes only at block boundaries, so a reader that needs a continuous value smooths it itself.
    float note() const { return noteValue; }
    bool hasNote() const { return noteValue >= 0.0f; }

    // Set up by the rack.
    const float* current = nullptr;
    const int* globalIndex = nullptr;
    const int* genericTarget = nullptr; // per local parameter: index into genericDest or -1
    const float* genericDest = nullptr;
    const ParamDef* defs = nullptr;
    const float* legacyDests = nullptr;
    double tempo = 120.0;
    const float* auxL = nullptr;
    const float* auxR = nullptr;
    float noteValue = -1.0f;
};

class EffectModule
{
public:
    virtual ~EffectModule() = default;
    // Control thread: allocate everything the effect needs.
    virtual void prepare(const ModuleContext& context) = 0;
    virtual void reset() = 0;
    // Audio thread: in place, never allocates, locks or blocks.
    virtual void process(float* left, float* right, int numSamples, const EffectInputs& inputs) = 0;
    // Called instead of process() while the effect is bypassed.
    virtual void deactivate() {}
};

// Optional capabilities a module instance can implement; hosts find them with ModuleRack::findCapability<T>().
class GainReductionMeter
{
public:
    virtual ~GainReductionMeter() = default;
    virtual int meterBands() const = 0;
    virtual float gainReductionDb(int band) const = 0;
};

// An effect that can draw its own frequency response and spectrum (equalisers). Both views are filled on the audio thread
// when somebody subscribes (the engine publishes them as "response" and "spectrum" views of the module).
class ResponseView
{
public:
    virtual ~ResponseView() = default;
    // dB of the module's current magnitude response (including its gain stages) at responsePoints() frequencies spaced
    // logarithmically from 20 Hz to 20 kHz.
    virtual int responsePoints() const = 0;
    virtual void fillResponse(float* out) = 0;
    // dB spectrum of the module's output (bins from 0 Hz to Nyquist), or 0 bins when the module has none.
    virtual int spectrumBins() const { return 0; }
    virtual void fillSpectrum(float*) {}
};

class StepPosition
{
public:
    virtual ~StepPosition() = default;
    virtual int currentStep() const = 0;
};

// How many oscillator and filter slots the voice has. The parameter model declares the parameters of every slot,
// so the shape is fixed for the process (see ModuleRegistry::configureVoiceGraph); EngineConfig can switch
// higher slots off to save CPU.
struct VoiceGraphShape
{
    int oscillators = 3;
    int filters = 2;
};
constexpr int kMaxOscSlots = 8;
constexpr int kMaxFilterSlots = 4;

// A page that a module brings along (pages of slots the product layout does not list).
struct ModulePage
{
    std::string id;
    std::string name;
    std::string sectionId;
    std::string title;
    std::string description;
};

// A connection point of a module, for graph editors and generated UIs.
enum class PortType
{
    AudioMono,
    AudioStereo,
    Modulation,
    Trigger,
    // An input only: a mono signal per sample that moves a parameter of the node (cutoff, pitch, pulse width, amplitude) at
    // audio rate. An edge into it must come from an audio output, which makes audio-rate modulation explicit in the graph;
    // the edge's amount scales the signal. Appended after the ports above, which keep their numbers.
    AudioRate
};

const char* portTypeName(PortType type);
bool portTypeFromName(const std::string& name, PortType& out);

struct PortDef
{
    std::string name;
    PortType type = PortType::AudioStereo;
};

// Parameter values by id, as a patch holds them.
using PatchValues = std::vector<std::pair<std::string, float>>;

// Brings the stored parameters of a patch that was saved with module version `fromVersion` to `toVersion`: ids are
// renamed first, then the values are remapped (by their new id), then `custom` may do anything else (split or merge
// parameters, drop obsolete ones). Parameters the patch does not hold are left alone, so added parameters need no
// migration: they load as their defaults.
struct ModuleMigration
{
    int fromVersion = 1;
    int toVersion = 2;
    std::vector<std::pair<std::string, std::string>> renames;                // old id -> new id
    std::vector<std::pair<std::string, std::function<float(float)>>> remaps; // new id -> value mapping
    std::function<void(PatchValues&)> custom;
};

struct ModuleDescriptor
{
    std::string id;   // stable, dotted: "fx.saturator", "voice.wavetable"
    std::string name; // display name
    ModuleKind kind = ModuleKind::Effect;
    std::string category; // free text for browsers: "Distortion", "Reverb", ...
    int version = 1;      // bumped when parameters are added or their meaning changes
    CpuClass cpu = CpuClass::Light;
    std::string description;
    // Legacy modules are kept for existing patches; supersededBy is the id of the current module to use instead.
    ModuleStatus status = ModuleStatus::Current;
    std::string supersededBy;
    // Section (layout) for pages that the product layout does not place explicitly.
    std::string sectionHint;
    // Parameters in declaration order; each sets pageId (and module is filled in by the registry).
    std::vector<ParamDef> params;
    // Ids of parameters of other modules that become per-voice modulation targets (ParamDef::voiceMod). They are
    // numbered after every target declared by the parameters themselves, so the generic destination numbers of
    // earlier parameters (stored in patches) do not move.
    std::vector<std::string> lateVoiceMod;
    // Effects: ids of the bypass switch and the chain-position parameter.
    std::string onParam;
    std::string positionParam;
    std::function<std::unique_ptr<EffectModule>()> createEffect;
    // Oscillators: factory of the per-voice, per-slot instance (null for the built-in wavetable oscillator).
    OscillatorFactory createOscillator;
    // Pages this module needs that the product layout does not list.
    std::vector<ModulePage> pages;

    // ---- Self-description for UIs and graph editors (all optional; ports are filled in by kind when empty).
    // The module exists once per oscillator slot ("osc1_..." parameters): introspection lists one instance per slot.
    bool perOscillator = false;
    std::vector<PortDef> inputs;
    std::vector<PortDef> outputs;
    // Views the module offers to read-back (see the view API); free-form ids.
    std::vector<std::string> views;
    // Capability ids ("gain_reduction", "step_position"...). Live engines add the ones
    // their instances implement.
    std::vector<std::string> capabilities;
    // One step per version bump that changed parameter ids or meanings; see ModuleMigration.
    std::vector<ModuleMigration> migrations;

    // ---- Instances (graph products, see crucible/graph/Product.h). A descriptor that is another instance of a module
    // type names that type here ("fx.saturator" for the second saturator); its parameter ids carry the instance prefix.
    // Empty for the first instance and for every module the built-in registration list adds.
    std::string instanceOf;
    // Display label of the instance in the product definition.
    std::string instanceLabel;
};

// Product layout: the sections (one per hardware section button) and the explicit page order with titles.
// Pages that modules use but the layout does not list are appended in module order.
struct ProductLayout
{
    std::string id;
    std::vector<SectionDef> sections;
    std::vector<PageDef> pages;
    VoiceGraphShape voice;
    int macros = kDefaultMacros; // 6 (default) or 8
    // Built-in modules (ids such as "fx.saturator") that the registry leaves out. Graph products list the effects their
    // FX graph does not use; the parameters of an omitted module do not exist in the product.
    std::vector<std::string> omit;
    // Host-parameter surface (product "surface"). pageOf puts a parameter on a page of `pages` (parameter id -> page id);
    // `surface` lists the parameter ids a host sees, in order. With exposeListed only those are exposed (the others get
    // automatable = false); without it every automatable parameter is, and `surface` is ignored.
    std::vector<std::pair<std::string, std::string>> pageOf;
    std::vector<std::string> surface;
    bool exposeListed = false;
    // Set when `pages` lists the surface's pages first (keepPages of them, in `keepSections` surface sections) and the
    // neutral ones after: the parameter model then drops the other pages and sections that end up holding nothing.
    bool dropEmptyPages = false;
    size_t keepPages = 0;
    size_t keepSections = 0;
};

class ModuleRegistry
{
public:
    // An empty registry for a product. Fill it with registerBuiltInModules() and / or add(), then finalize(); or use
    // build() / builder(). The layout is clamped to the engine limits.
    ModuleRegistry();
    explicit ModuleRegistry(ProductLayout layout);
    ~ModuleRegistry();
    ModuleRegistry(const ModuleRegistry&) = delete;
    ModuleRegistry& operator=(const ModuleRegistry&) = delete;

    // The built-in modules arranged as the given product, finished and ready to share between engines.
    static std::shared_ptr<const ModuleRegistry> build(ProductLayout layout);
    // The same, but still open: add() third-party modules, then finalize().
    static std::shared_ptr<ModuleRegistry> builder(ProductLayout layout);
    // The neutral product ("crucible.default": generic sections, 3 oscillators, 2 filters, 6 macros).
    static ProductLayout neutralLayout();

    // Registers a module. Throws std::logic_error for a missing or duplicate id, for a parameter id that another
    // parameter of this module or of an earlier module already uses, and once the registry is finalized. A refused
    // module leaves the registry unchanged.
    void add(ModuleDescriptor descriptor);
    // Builds the parameter model; no module can be added afterwards. Idempotent; returns *this.
    ModuleRegistry& finalize();
    bool finalized() const { return model != nullptr; }
    // The parameter model of this product. Throws std::logic_error before finalize().
    const ParameterModel& parameters() const;

    const std::vector<ModuleDescriptor>& modules() const { return list; }
    const ModuleDescriptor* find(const std::string& id) const;
    std::vector<const ModuleDescriptor*> ofKind(ModuleKind kind) const;
    // The oscillator types by type index (the value of oscN_type): the built-in types in their fixed order, then any other
    // registered oscillator. A built-in type the product omits is a null entry, so the index of later types does not move.
    std::vector<const ModuleDescriptor*> oscillatorTypes() const;
    const ProductLayout& layout() const { return productLayout; }

    // Brings parameter values saved by older module versions up to this registry's versions. savedVersions maps
    // module id -> the version the patch was saved with (missing = 1). Modules whose version did not change, and
    // versions newer than the registry's, are left alone. See ModuleMigration.
    void migrate(const std::map<std::string, int>& savedVersions, PatchValues& values) const;

    // ---- DEPRECATED process-wide default product (singleton shim). The hosts that were written when the product was
    // fixed per process keep using these; engines built without an explicit registry or product use the same layout
    // (processLayout()), each with a registry of its own. New code passes EngineConfig::registry or
    // EngineConfig::product and never needs them.
    //
    // The built-in modules arranged as the process default product, built on first use.
    static const ModuleRegistry& instance();
    // Chooses the voice graph shape for the process (clamped to kMaxOscSlots / kMaxFilterSlots). Must be called
    // before the process product is first used; returns false when it is too late.
    static bool configureVoiceGraph(VoiceGraphShape shape);
    // The configured shape, or the given default when none was configured.
    static VoiceGraphShape resolveVoiceGraph(VoiceGraphShape productDefault);
    // Chooses the whole process default product: sections, page order, voice graph shape and macro count replace the
    // neutral layout. Must be called before the process product is first used; returns false when it is too late.
    static bool configureProduct(ProductLayout layout);
    // The configured product, if any.
    static std::optional<ProductLayout> configuredProduct();
    // Macro count of the process product (configured, else the given default). Reading it fixes the configuration.
    static int resolveMacros(int productDefault);
    // The layout of the process default product (configured, else neutral). Reading it fixes the configuration.
    static ProductLayout processLayout();

private:
    std::vector<ModuleDescriptor> list;
    std::unordered_set<std::string> paramIds; // every parameter id of every registered module
    ProductLayout productLayout;
    std::unique_ptr<ParameterModel> model;
};

// Registration list of the built-in modules (one line per module).
// The registry's layout decides the voice graph shape and the macro count the modules declare.
void registerBuiltInModules(ModuleRegistry& registry);

} // namespace crucible
