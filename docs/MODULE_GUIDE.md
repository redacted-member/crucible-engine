# Adding a module

This guide adds an effect, the most common case. It takes three steps and touches only the new file plus one line
in the registration list (and one declaration). Use `src/modules/fx/Tremolo.cpp` as a template.

## 1. Write the DSP and the module in one file

`src/modules/fx/MyEcho.cpp`:

```cpp
#include "FxModuleSupport.h"

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_myecho_mix")), time(localIndex(d, "fx_myecho_time")) {}

    void prepare(const ModuleContext& c) override { sr = c.sampleRate; line.allocate(static_cast<int>(sr)); }
    void reset() override { line.clear(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        const float wet = in.value(mix);                    // smoothed and modulated, natural units
        const float delay = in.value(time) * 0.001f * static_cast<float>(sr);
        for (int i = 0; i < n; ++i)
        {
            line.write(0.5f * (l[i] + r[i]));
            const float e = line.read(delay);
            l[i] += (e - l[i]) * wet;
            r[i] += (e - r[i]) * wet;
        }
    }

private:
    const int mix, time;
    DelayLine line;
    double sr = 48000.0;
};
} // namespace

ModuleDescriptor myEchoEffect()
{
    auto d = effectDescriptor("fx.myecho", "My Echo", "Delay", CpuClass::Light, "A mono slap-back echo.");
    auto& p = d.params;
    addOn(p, "myecho", "My Echo", "fx_myecho", 0.3f);                       // fx_myecho_on, fx_myecho_mix
    addF(p, "myecho_time", "My Echo Time", "fx_myecho", 1.0f, 500.0f, 80.0f, "ms", 0.5f);
    addPosition(p, "myecho", "My Echo", "fx_order_4", 8 );                   // fx_pos_myecho
    d.onParam = "fx_myecho_on";
    d.positionParam = "fx_pos_myecho";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.myecho")); };
    return d;
}

} // namespace crucible::modules::fx
```

Rules:

* `process()` must not allocate, lock or block. Allocate in `prepare()`.
* Read parameters only through `EffectInputs` (`value`, `on`, `choice` / `rounded`, `bpm`). Values arrive
  smoothed (per the parameter's `Smoothing`) and modulated.
* Ids: `fx_<id>_...`, lower case, stable forever. Continuous parameters added with `addF` / `addHz` are
  modulation targets automatically; use `params::floating(...)` directly for parameters that must not be.
* Pages hold at most six parameters (the hardware has six knobs). Use a new page id; the product layout appends it
  under the FX section unless it lists it explicitly.
* Optional capabilities: implement `ConvolutionIr` (loadable impulse responses), `GainReductionMeter` (only for dynamics processors that
  compute gain reduction anyway), `LatencyReporter` (lookahead, oversampling or linear phase: whole samples) `StepPosition` or `ResponseView` (an equaliser draws its own curve and spectrum; declare the
  `response_view` capability and the engine publishes the `response` and `spectrum` views) if a UI needs them. Modules do not provide their own level meters; the synth has one meter set on the master bus. The
  exceptions are modules whose job is measuring (`util.meter`) or whose loudness is part of their function (`fx.maximizer`):
  they implement `MeterReadout` (`crucible/module/MeterReadout.h`) on top of `crucible/fx/Loudness.h` and list the
  `loudness` / `levels` capabilities, and the engine publishes `<module id>/loudness` and `<module id>/levels` views.
* Honour `ModuleContext::quality` if the module has an oversampling choice.

## 2. Register it

`src/modules/Modules.h`: declare `ModuleDescriptor myEchoEffect();` in `namespace fx`.
`src/modules/Modules.cpp`: add `add(fx::myEchoEffect());` at the end of the effect group (`add` is `sdk::registerModule`, the
one path every module takes into a registry, built-in or third-party). The file itself is
compiled automatically (CMake globs `src/modules`).

## 3. Test and document

* Add a test class (copy the pattern of `tests/FxExtraTests.cpp`): finite and bounded at parameter extremes (the
  generic `ModuleTests` and the FX extreme sweep already cover every registered effect), plus checks of what the
  effect is supposed to do (levels, timing, spectrum).
* Run `CRUCIBLE_UPDATE_GOLDEN=1 build/crucible_engine_tests Golden` only if an existing sound intentionally changed.
* Regenerate `docs/PARAMETERS.md` with `build/crucible-render --params --markdown > docs/PARAMETERS.md` and, for the
  plugin, rebuild so `synth/parameters.json` is exported.
* Bump the module's `version` whenever an existing module gains or changes parameters, and add a migration step in
  `Patch.cpp` if old patches need translating.

## Other module kinds

* **Performance / sequencer / voice modules** contribute parameters the same way (see
  `src/modules/sequencer/Arpeggiator.cpp`); their DSP is wired in `EngineCore` or `SynthEngine`.
* **Modulators** expose their outputs as route sources (`Modulation.h`: extend the source id space like `kSourceCC`
  and `kSourceClip`) so they can drive any target through routes. The automation clips (`Clips.h`) are the worked
  example: a lane is source `clipLaneSource(clip, lane)`; `SynthEngine` evaluates global clips once per control
  chunk and voice-triggered clips per voice (`ClipPlayer`), and applies each route's amount, polarity, curve and
  smoothing. Clips and the motion recorder (`MotionRecorder.h`) hold no module parameters: their data lives in the
  patch (`"clips"`), not in the parameter list.
* **Oscillators** are slot types: see below. Filters are still models of the voice filter (`Filters.h`).

## Adding an oscillator type

An oscillator module is one file under `src/modules/oscillator/` (`NoiseOscillator.cpp` is the smallest example):

```cpp
class Instance final : public OscillatorInstance          // one per voice and slot, created at prepare time
{
public:
    explicit Instance(int slot) : sp(slot) { amount = sp.bind("mine_amount"); }   // resolves "osc{slot+1}_mine_amount"
    void prepare(double rate, int quality) override { ... }   // control thread: allocate here
    void noteOn(const OscNoteInfo& n) override { ... }        // seed, note, velocity
    void render(const OscRender& r) override                  // add unit-level stereo into r.left / r.right
    {
        const float a = sp.value(r.values, r.targetMod, amount);   // natural units, per-voice modulation applied
        const float hz = 440.0f * std::exp2((r.pitch - 69.0f) / 12.0f);
        ...
    }
};

ModuleDescriptor myOscillator(const VoiceGraphShape& shape)   // declares the parameters of every slot
{
    ModuleDescriptor d;  d.id = "osc.mine"; d.kind = ModuleKind::Oscillator; ...
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto page = osc::addSlotPage(d, slot, "mine", "Mine", "LCD description");   // pages hold six knobs
        d.params.push_back(osc::voiceMod(params::floating(osc::slotId(slot, "mine_amount"), ..., page, 0, 1, 0)));
    }
    d.createOscillator = [](int slot) { return std::make_unique<Instance>(slot); };
    return d;
}
```

Register it in `Modules.cpp` before `oscillatorSlots(...)` (which builds the type choice list from the registered
oscillator modules; new types are appended, so existing type indices stay valid). Rules: no allocation in `render()`;
share data with the control thread through `RtShared` (see `SampleBank`); `voiceMod` parameters are modulated per
voice; honour the sample rate and keep the output band-limited (partials above `rate - 20 kHz` fold into the audible
band). Add a test (finite and bounded at extremes, pitch, aliasing) and an entry in `tests/PlatformTests.cpp`'s
allocation test if the oscillator owns buffers.

Every voice owns an instance of every oscillator type (a voice can switch type mid-note), so an oscillator with large buffers
should not allocate them in `prepare()`: override `OscillatorInstance::arm()` (control thread, idempotent) and build them there,
publishing the pointer through an atomic that `render()` reads (silence while it is null). `Engine::setParam` calls `arm()` on
every voice's instance when a slot selects the type, `Engine::prepare` for the types selected at that moment, a graph node in its
`prepare`, and `crucible-module-check` after `prepare`. `osc.spectral` (the 0.8 MB stretcher) and the string and bow buffers of
`osc.modal` do this.

## Products with their own configuration

A product that wants its own page order, shape or macro count passes a `ProductLayout` to the engine
(`EngineConfig::product`) or builds a registry with it (`ModuleRegistry::build(layout)`): the layout's sections and
pages replace the default ones, `layout.voice` sets the oscillator and filter slot counts and `layout.macros` (6 or 8)
the number of macros. Anything that depends on those choices (a module that declares per-slot or per-macro
parameters) takes them from the layout it is registered with (`registerBuiltInModules` passes `registry.layout()`
values to the descriptor factories), never from a static or a process-wide setting. Every engine owns its product, so
tests use several products in one executable (`tests/ProductTests.cpp`, `tests/IntrospectionTests.cpp`).

A third party adds a module to one product with `auto r = ModuleRegistry::builder(layout); r->add(descriptor);
r->finalize();` and `EngineConfig::registry = r`. A module that renames or re-scales parameters between versions bumps
`ModuleDescriptor::version` and lists a `ModuleMigration` per step; the optional descriptor fields `inputs`,
`outputs`, `views`, `capabilities` describe it to UIs, and the optional `ParamDef` fields `shortName`, `group`,
`uiHint`, `polarity`, `textFormat`, `automatable` refine how its parameters are shown. An oscillator factory
receives the product's `ParameterModel` (`SlotParams(slot, model)`).

Parameters that are added to an existing product must not move the generic modulation destination numbers of earlier
parameters: register such a module after the existing ones (see `cloudBands` and `vectorPad` at the end of
`Modules.cpp`; `CloudDrawbarTests` checks the order).
To make parameters of an existing module per-voice targets afterwards, list their ids in `ModuleDescriptor::lateVoiceMod`
of a module registered last (see `unisonModulation`).

Audio-rate modulation (P12). A route from an audio-rate source (an oscillator's output, `lfoN_audio`, a per-sample envelope) to the pitch,
FM amount, pulse width, wavetable position, filter cutoff or amplifier of a voice runs per sample. An oscillator opts in by returning true
from `supportsAudioRate()` and reading `OscRender::pitchMod` (semitones added to `pitch`), `fmAmountMod` and `widthMod` (added to the
parameters of that name, same units), each one value per sample or null; the voice gives every other oscillator the chunk mean through its
ordinary pitch and parameters. `osc.analog` does (an edge that moves per sample is placed on a smooth curve through the widths, four pieces
per sample). A filter model that should follow a per-sample cutoff takes it in `AnalogFilter::process(io, n, cutoffOctMod)` (octaves per
sample); `LadderBase` samples it every `kFmChunk` (4) samples and ramps the integrator gain between the points.

## Graph nodes and product definitions

A voice built from data uses node types (`crucible/graph/Node.h`). A `NodeType` has local parameter ids (`attack`),
ports (audio ports carry stereo buffers, modulation ports one value per 32-sample chunk) and a factory returning a `Node`:
`prepare` and `reset` on the control thread, `noteOn`, `noteOff`, `process(NodeIO&, n)` on the audio thread (read
parameters with `io.param(local)`, never allocate), optionally `processQuad` for four voices of the node together, and
`holdsVoice()` for nodes that keep a voice alive (envelopes). Register the type in a `NodeRegistry` (a copy of
`NodeRegistry::builtIn()` plus `add()`) and pass it to `graph::compile()`. The compiler registers a module per node
instance and generates the ids `<instance>_<local>`, so a type never names its own global ids. A wrapper of an existing
DSP class keeps that class unchanged (`src/graph/Nodes.cpp` wraps `Adsr`, `Lfo`, `FilterUnit` and the oscillator modules).

A node input of type `PortType::AudioRate` takes a mono signal per sample (`io.rate(port)`, `io.rateConnected(port)`): the audio outputs wired to
it, mixed down to mono and scaled by the edge's `amount`. Only an audio output can be wired to it, which makes audio-rate modulation explicit in
the product. Built-ins: `filter.analog.cutoff_fm` and `filter.svf.cutoff_fm` (5 octaves per unit), `amp.vca.am` (gain `1 + signal`), and
`pitch_fm` (24 semitones per unit), `width_mod` and `fm_mod` of the oscillator nodes; `lfo.audio` is the band-limited audio-rate LFO node.

A node that wants the part-to-part tolerances of the voice (P11) reads `NodeIO::card` (a `VoiceCard`, null when the product runs
without cards) and `NodeIO::cardAmounts`; `NodeNote` and `NodePrepare` carry the same pointer, and `NodePrepare::index` is the node's
position in its layer, which a node uses to pick its own entry (`env.analog` takes `card->envTimeScale(index % 4, amount)`). Null means the
legacy behaviour, so a node must treat it as "no tolerance". An oscillator module receives the draws of its slot in `OscNoteInfo::card` and
says so with `usesVoiceCard()`.

A product definition (`products/*.json`, schema in `crucible/graph/Product.h`) names the instances, the voice graph, the FX
graph and the parameter ids it pins. `crucible-render --product FILE` plays one. Test products must keep their
parameter ids: a pin that matches no generated id is a compile error.

## Third-party modules: the module SDK

A module that does not live in this repository is a shared library that implements the C ABI of
`crucible/crucible_module.h` and links against nothing of the engine. The host loads it at run time; the engine never
scans directories, the host names every path and decides whether libraries may be loaded at all.

**Kinds.** A module is an *effect* (an instance type of the FX graph: one stereo bus in, the same buffers out, processed in
place) or a *node* of the voice graph: a *generator*, *filter* or *modulator* with the audio and modulation ports it
declares (`CRUCIBLE_PORT_AUDIO_MONO | AUDIO_STEREO | MODULATION | TRIGGER`). The ids are dotted lower case, vendor first
(`acme.tremolo`); `crucible`, `fx`, `osc`, `voice`, `filter`, `env`, `lfo`, `amp`, `mix` and `classic` are the engine's.

**Parameters.** The descriptor lists them (float, int, choice, bool; range, default, skew, unit, a short name, flags for
generic modulation, log smoothing and automation) and `process` receives them in that order, in natural units, already
smoothed. Ids are local (`rate`); the host makes them global: a node instance `lfo1` of the type gets `lfo1_rate`, an effect
gets `acme_tremolo_rate` (and `fx2_acme_tremolo_rate` for a second instance). The host adds a bypass switch
`<prefix>_on` to every effect; the module never sees it.

**Entry points.** `create` / `destroy` / `prepare` / `get_view` run on a control thread and may allocate. `process`,
`process_quad`, `note_on`, `note_off`, `reset`, `latency` and `holds_voice` run on the audio thread. The real-time
contract: no allocation, lock or wait in them, finite and bounded output at every parameter value in range, no reliance on
flush-to-zero, output a function of seed, parameters and input only. `prepare` receives the seed; `note_on` a per-note
seed. Optional `process_quad` handles the same node of four voices in one call and returns 0 to decline.

**Versions.** Every struct starts with `size` and `abi_version`. Structs only grow at the end, so a host accepts every
module whose `abi_version` is between `CRUCIBLE_MODULE_ABI_MIN` and `CRUCIBLE_MODULE_ABI_VERSION` and reads a struct only as
far as its `size` says. A module built for a newer version is refused with `CRUCIBLE_MODULE_ERR_ABI_VERSION` and a message;
nothing of it is registered. A module bumps its own `version` when parameters are added or change meaning.

**In C** (`examples/third_party_module/c_module.c`): fill static `crucible_param_spec` and `crucible_module_descriptor`
structs, export `crucible_get_modules` returning a `crucible_module_library`.

**In C++** (`examples/third_party_module/cpp_module.cpp`): write a class with `static sdk::Info info()` and
`void process(sdk::Process&)`, plus any of `prepare`, `reset`, `noteOn`, `noteOff`, `holdsVoice`, `latency`,
`static bool processQuad(...)` and `view`, and end the file with `CRUCIBLE_SDK_EXPORT_MODULES(MyModule, Other)`.

```cpp
class Tremolo
{
public:
    static crucible::sdk::Info info()
    {
        crucible::sdk::Info i;
        i.id = "acme.tremolo";
        i.name = "Tremolo";
        i.kind = crucible::sdk::Kind::Effect;
        i.params = {crucible::sdk::Param::floating("rate", "Rate", 0.1f, 20.0f, 4.0f).unit("Hz").skew(0.4f),
                    crucible::sdk::Param::floating("depth", "Depth", 0.0f, 1.0f, 0.5f)};
        return i;
    }
    bool prepare(const crucible::sdk::Prepare& p) { rate = p.sampleRate; return true; }
    void reset() { phase = 0.0; }
    void process(crucible::sdk::Process& p) { /* p.left(), p.right(), p.param(i), p.frames() */ }
private:
    double rate = 48000.0, phase = 0.0;
};
CRUCIBLE_SDK_EXPORT_MODULES(Tremolo)
```

**Using it from a host.**

```cpp
crucible::sdk::ModuleSet modules;
modules.allowLibraries(true);                         // the host flag; off by default
if (modules.loadLibrary(path) != CRUCIBLE_MODULE_OK)  // all or nothing; modules.error() says why
    return fail(modules.error());
auto nodes = crucible::graph::NodeRegistry::builtIn();
modules.addTo(nodes);                                 // effects and node types
auto product = crucible::graph::compile(definition, error, nodes);
crucible::EngineConfig config;
config.compiled = product;                            // the product JSON names "acme.tremolo" as an fx instance type
```

The same in C: `crucible_module_registry_create`, `crucible_module_registry_allow_libraries`,
`crucible_load_module_library` or `crucible_register_module`, then `crucible_create_from_product(config, product_json,
registry, error, size)`. The registry can be destroyed while engines made from it run: libraries stay open as long as a
module of them is in use. Modules cannot be added to a running engine (a new module set needs a new engine, as in
`compileReconfiguration`). Building with `-DCRUCIBLE_ENGINE_MODULE_LOADING=OFF` removes library loading altogether;
`crucible_register_module` still works for modules linked into the host.

**Check it.** `crucible-module-check libacme_tremolo.so` runs the real-time harness: metadata, allocation counter on the
audio calls (including parameter changes every block and note events), an extremes sweep (every parameter at its minimum
and maximum, all minima, all maxima and seeded corners, with loud noise and with silence; output finite and below +40
dBFS), NaN / infinity and denormal inputs, independence from flush-to-zero, determinism (two fresh instances with one
seed and an instance after `reset()` render the same bits) and CPU (a budget by `cpu_class`: 4 %, 10 %, 40 % of one core
per second of audio; `--no-cpu-limit` reports without failing). `--builtin` checks every built-in module the same way.
Exit status 0: all passed, 1: a check failed, 2: a library could not be loaded or the command line was wrong. The
allocation check counts C++ `operator new` (the tool replaces it); a C module that calls `malloc` in `process` is not
seen by it, so review those by hand. The same harness is a library (`crucible/sdk/Check.h`) for module test suites.

**Built-ins.** Every built-in module is registered through the same function third-party descriptors end up in
(`sdk::registerModule`, which is `ModuleRegistry::add`) and passes the same check (`tests/SdkTests.cpp`); they keep C++
factories rather than going through the C ABI, so the sound of the existing modules is untouched.
