// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace crucible
{

enum class ParamType
{
    Float,
    Int,
    Choice,
    Bool
};

enum class Smoothing
{
    None,
    Linear,
    Log
};

enum class ModSource
{
    None,
    Lfo1,
    Lfo2,
    ModEnv,
    Velocity,
    Key,
    AmpEnv,
    Env3,
    Env4,
    Lfo3,
    Lfo4,
    ModWheel,
    Aftertouch,
    Random,
    Macro1,
    Macro2,
    Macro3,
    Macro4,
    Macro5,
    Macro6,
    StripL,
    StripR,
    SceneMorph,
    PitchBend,
    // Appended for products with eight macros (never renumber the ones above).
    Macro7,
    Macro8,
    Count
};

enum class ModDest
{
    None,
    Osc1Position,
    Osc2Position,
    Osc1Pitch,
    Osc2Pitch,
    Osc1Level,
    Osc2Level,
    Cutoff,
    Resonance,
    Drive,
    SubLevel,
    NoiseLevel,
    UnisonDetune,
    AmpLevel,
    Osc3Position,
    Osc3Pitch,
    Osc3Level,
    Osc1WarpA,
    Osc1WarpB,
    Osc2WarpA,
    Osc2WarpB,
    Osc3WarpA,
    Osc3WarpB,
    Osc1Pan,
    Osc2Pan,
    Osc3Pan,
    Cutoff2,
    Resonance2,
    Drive2,
    Filter1Mix,
    Filter2Mix,
    Filter1Pan,
    Filter2Pan,
    Pitch,
    Lfo1Rate,
    Lfo2Rate,
    Lfo3Rate,
    Lfo4Rate,
    MasterGain,
    Count
};

constexpr int kNumModSlots = 16;
constexpr int kNumLfos = 4;
constexpr int kNumEnvs = 4;
constexpr int kNumMacros = 8; // most a product can have; ModuleRegistry::macroCount() says how many exist
constexpr int kDefaultMacros = 6;

// Route source of macro m (0-based): macros 1-6 sit next to each other, 7 and 8 were appended after the others.
constexpr ModSource macroSource(int m)
{
    return m < kDefaultMacros ? static_cast<ModSource>(static_cast<int>(ModSource::Macro1) + m)
                              : static_cast<ModSource>(static_cast<int>(ModSource::Macro7) + (m - kDefaultMacros));
}

// How a UI draws a parameter. Auto is resolved when the parameter model is built (floats and ints: Knob, choices:
// Menu, switches: Switch).
enum class UiHint
{
    Auto,
    Knob,
    Slider,
    Switch,
    Menu,
    Xy,
    Drawbar
};

enum class Polarity
{
    Auto, // Bipolar when the range spans zero, otherwise Unipolar
    Unipolar,
    Bipolar
};

// The standard display-text functions (src/ParamText.cpp). The format of a parameter turns a natural value into
// text and back (ParameterModel::valueToText / textToValue). Auto is resolved from the unit and the type when the
// model is built.
enum class TextFormat
{
    Auto,
    Number,       // plain number, trailing zeros trimmed
    Hertz,        // "440 Hz"; the parser also reads "1.5 kHz"
    Seconds,      // "0.25 s"
    Milliseconds, // "20 ms"
    Decibels,     // "-6 dB"
    Percent,      // "50 %" for a unit-less 0..1 style value stored in percent already
    Semitones,    // "+7 st"
    Cents,        // "-12 ct"
    NoteName,     // MIDI note number as "C4" (middle C = 60)
    Ratio,        // "4:1"
    Bpm,          // "120 BPM"
    Degrees,      // "90 deg"
    Octaves,      // "2 oct"
    Bits,         // "8 bit"
    Bars,         // "4 bars"
    Choice,       // the name of the choice
    Toggle        // "Off" / "On"
};

const char* textFormatName(TextFormat format);
bool textFormatFromName(const std::string& name, TextFormat& out);
const char* uiHintName(UiHint hint);
bool uiHintFromName(const std::string& name, UiHint& out);
const char* polarityName(Polarity polarity);
bool polarityFromName(const std::string& name, Polarity& out);

// The stable number a plug-in host (CLAP param id, VST3 ParamID, AU address) gets for a parameter: the 32-bit FNV-1a hash
// of its id with the top bit cleared (31 bits), so it never falls in VST3's reserved range and is never 0xFFFFFFFF. It
// depends on the id only, never on the parameter's index, so it survives products that add, omit or reorder modules.
// Ids that collide (after the masking) make the product fail to compile.
constexpr std::uint32_t hostIdOf(const std::string& id)
{
    std::uint32_t h = 2166136261u;
    for (const char c : id)
        h = (h ^ static_cast<std::uint8_t>(c)) * 16777619u;
    return h & 0x7fffffffu;
}

struct ParamDef
{
    std::string id; // stable: never renamed once released (patches and host automation refer to it)
    std::string name;
    int page = 0; // index into ParameterMap::pages(), resolved from pageId when the map is built
    std::string pageId;
    std::string module; // id of the module that declares it
    ParamType type = ParamType::Float;
    float min = 0.0f;
    float max = 1.0f;
    float defaultValue = 0.0f;
    float interval = 0.0f;
    float skew = 1.0f;
    std::string unit;
    std::vector<std::string> choices;
    // Choices (indices into `choices`) that exist in the list but that this product cannot use, ascending; empty for most
    // parameters. See ParameterModel::choiceAvailable.
    std::vector<int> unavailable;
    Smoothing smoothing = Smoothing::Linear;
    // False for performance controls and the scene system itself (not stored in scenes).
    bool scene = true;
    // Accepts routes from the generic modulation system (see ParameterMap::modTargets()).
    bool modulatable = false;
    // With modulatable: the voice that reads the parameter evaluates the routes per voice (matrix slots and
    // assignable routes, so per-voice sources work) instead of the global generic path. Log-smoothed parameters
    // are scaled in octaves (5 octaves per unit of modulation), the others by their range.
    bool voiceMod = false;
    // Module version that introduced the parameter (patch migration).
    int since = 1;
    // Append sequence: 0 for a regular parameter. A parameter with sequence n >= 1 is listed after every module's regular
    // parameters (and numbered after every generic modulation target), ordered by (sequence, module registration order).
    // This is how a parameter is added to a module that is already frozen in tests/golden/param_order.txt without moving
    // any existing index. Rule after v1: a new parameter on an existing module (or a new module whose parameters are all
    // appended) takes the NEXT unused sequence; never reuse a sequence and never renumber one. Sequences in use:
    //   1 fx.crusher note tracking, 2 fx.saturator pre / post filter, 3 fx.cabinet, 4 seq.step seq_trigger.
    // Use together with `since` and a module version bump.
    int appended = 0;
    // A matrix destination selector: its choices are filled with modDestNames() when the map is built.
    bool destinationChoice = false;

    // ---- Self-description. Everything below is optional: the parameter model fills the defaults in when it is
    // built, so a model's ParamDef always carries the resolved values.
    // Up to six ASCII characters, for hardware LCDs. Default: derived from name.
    std::string shortName;
    // Tree path of the parameter in a generated UI, "section/page". Default: the section and page ids.
    std::string group;
    UiHint uiHint = UiHint::Auto;
    Polarity polarity = Polarity::Auto;
    TextFormat textFormat = TextFormat::Auto;
    // False for parameters a host must not expose to automation (none of the built-ins today).
    bool automatable = true;
    // A host sees it: the parameter is on the product's surface (see ParameterModel::surface()). Resolved when the
    // model is built; true for every automatable parameter unless the product lists its surface.
    bool exposed = true;
    // Accepts assignable modulation routes (ModRouteStore): the generic targets (modulatable) plus the parameters
    // with a fixed voice destination (filter cutoff, oscillator pitch...). Resolved when the model is built.
    bool routable = false;
    // Number of distinct values (ints, choices, switches, floats with an interval); 0 = continuous. Resolved.
    int steps = 0;
};

// A page holds at most ParameterMap::knobsPerPage parameters, in knob order.
// Pages are grouped into sections; each section maps to one round button.
struct PageDef
{
    std::string id;
    std::string name;
    int section = 0;
    std::string layoutTitle;
    std::string layoutDescription;
};

struct SectionDef
{
    std::string id;
    std::string name;
};

// Parameters modulated per voice (ParamDef::voiceMod), numbered densely in parameter order.
struct VoiceTarget
{
    int param = 0;
    float range = 1.0f; // max - min
    bool octaves = false;
};

// Optional host-specific blocks inserted into the JSON export (the plugin adds its hardware layout here).
struct JsonExtras
{
    std::string pluginName = "Crucible";
    std::string blockKey = "hardware";
    // Called per page index with the indentation depth; returns a JSON value, or nothing when unset.
    std::function<std::string(int page, int indent)> pageBlock;
    std::function<std::string(int indent)> globalBlock;
};

class ModuleRegistry;

// Named modulation sources of a product with the given macro count (six macros end at Pitch Bend, eight append two).
std::vector<std::string> modSourceNamesFor(int macros);

// The parameter model of one product: sections, pages, parameters in registration order, the generic modulation
// targets and the per-voice targets, built from a ModuleRegistry (the registry owns its model, see
// ModuleRegistry::parameters()). Immutable once built, so it is safe to read from any thread. Two products in one
// process have two models; nothing here is process-wide.
class ParameterModel
{
public:
    explicit ParameterModel(const ModuleRegistry& registry);

    const std::vector<PageDef>& pages() const { return pageList; }
    const std::vector<SectionDef>& sections() const { return sectionList; }
    const std::vector<ParamDef>& params() const { return paramList; }
    // The parameters a host builds its list from, as indices into params(), in the product's declared order. A product
    // that does not list a surface exposes every automatable parameter in index order.
    const std::vector<int>& surface() const { return surfaceList; }
    // Whether choice `choice` of parameter `paramIndex` can be used in this product. Choice numbers are the same in every
    // product: an entry a product leaves out stays in the list and reports false here. False for a bad index, a parameter
    // that is not a choice, or a choice outside the list.
    bool choiceAvailable(int paramIndex, int choice) const;
    int size() const { return static_cast<int>(paramList.size()); }
    const std::vector<std::string>& modSourceNames() const { return sourceNames; }
    // Matrix destinations: the fixed voice destinations (ModDest) followed by one entry per modTargets() parameter.
    const std::vector<std::string>& modDestNames() const { return destNames; }
    // Macros of the product (6 or 8).
    int macros() const { return macroCount; }

    // Index into params(), or -1 when the id is unknown.
    int indexOf(std::string_view id) const; // no allocation: a literal or a view never builds a std::string
    // Index into params() of the parameter whose hostIdOf(id) is hostId, or -1. Binary search over a table built with the
    // map.
    int indexOfHostId(std::uint32_t hostId) const;
    // Parameter ids of one page, in knob order.
    std::vector<int> paramsOnPage(int page) const;
    // Indices of the parameters that accept generic modulation routes, in a stable order (the generic part of the
    // matrix destination list follows the legacy ModDest entries in this order).
    const std::vector<int>& modTargets() const { return targets; }
    const std::vector<VoiceTarget>& voiceTargets() const { return voiceTargetList; }
    // Dense voice target of a parameter index, or -1.
    int voiceTargetOfParam(int paramIndex) const;
    // Dense voice target of a generic matrix destination (destination - kNumModDests), or -1.
    int voiceTargetOfModTarget(int modTarget) const;
    // Default values in natural units (choice = index).
    std::vector<float> defaultValues() const;

    // Display text of a natural value, and the value of a text. valueToText is the canonical form; textToValue also
    // accepts the usual variants (see ParamText.h). Both clamp to the parameter's range; textToValue returns false
    // for text it cannot read, and for an index out of range. For every text t that valueToText produced,
    // valueToText(textToValue(t)) == t, and textToValue(valueToText(v)) == v for each value v that has at most six
    // significant decimal digits (every default, bound, integer, choice and switch value of a parameter does).
    std::string valueToText(int paramIndex, float value) const;
    bool textToValue(int paramIndex, const std::string& text, float& out) const;

    // The model as JSON, schema 1 (sections, pages, parameters with ranges, units and choices): the form the
    // hardware editor and the plug-in export use. describeJSON() (crucible/Introspection.h) is the full view.
    std::string toJson(const JsonExtras& extras = {}) const;

private:
    std::vector<SectionDef> sectionList;
    std::vector<PageDef> pageList;
    std::vector<ParamDef> paramList;
    std::vector<int> surfaceList;
    std::vector<int> targets;
    std::vector<std::string> destNames;
    std::vector<std::string> sourceNames;
    std::vector<VoiceTarget> voiceTargetList;
    std::vector<int> voiceOfParam;
    std::vector<int> voiceOfModTarget;
    std::vector<std::pair<std::string, int>> sortedIds;       // id -> index, sorted by id
    std::vector<std::pair<std::uint32_t, int>> sortedHostIds; // host id -> index, sorted by host id
    int macroCount = kDefaultMacros;
};

// Natural value <-> normalised 0..1 position, honouring skew (natural = min + (max - min) * n ^ (1 / skew)) and,
// for fromNormalised, the interval and the integer nature of int / choice / bool parameters.
float toNormalised(const ParamDef& def, float natural);
float fromNormalised(const ParamDef& def, float normalised);

// The legacy matrix destinations (ModDest order), the same for every product.
const std::vector<std::string>& legacyModDestNames();

// DEPRECATED singleton shim: the same queries answered by the process default product (the one chosen with
// ModuleRegistry::configureProduct, else the neutral default), i.e. ModuleRegistry::instance().parameters().
// New code takes the model of its engine (Engine::model(), SynthEngine::model()) instead; these functions exist so
// that hosts written against the process-wide model keep working. Using any of them fixes the process product.
namespace ParameterMap
{
constexpr int knobsPerPage = 6;
// Per-voice modulation targets of one product. 256 sufficed until the voice graph of five oscillator slots met the P13
// oscillators (about 60 per-voice parameters per slot), and 512 until eight slots (557 targets) met it; the arrays sized
// by it are a few kilobytes.
constexpr int kMaxVoiceTargets = 640;
using VoiceTarget = crucible::VoiceTarget;
using JsonExtras = crucible::JsonExtras;

const std::vector<PageDef>& pages();
const std::vector<SectionDef>& sections();
const std::vector<ParamDef>& params();
const std::vector<std::string>& modSourceNames();
const std::vector<std::string>& modDestNames();
const std::vector<std::string>& legacyModDestNames();
int indexOf(std::string_view id);
std::vector<int> paramsOnPage(int page);
const std::vector<int>& modTargets();
const std::vector<VoiceTarget>& voiceTargets();
int voiceTargetOfParam(int paramIndex);
int voiceTargetOfModTarget(int modTarget);
std::vector<float> defaultValues();
using crucible::fromNormalised;
using crucible::toNormalised;
std::string toJson(const JsonExtras& extras = {});
} // namespace ParameterMap

} // namespace crucible
