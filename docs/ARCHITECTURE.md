# CrucibleEngine architecture

Crucible Engine is a polyphonic synthesis engine that hosts embed to build instruments. It is a framework-free C++20 static library with a C API. This document describes how it is put
together and which parts are finished; `MODULE_GUIDE.md` shows how to add a module, `PARAMETERS.md` lists every
parameter, `ROADMAP.md` lists what is planned.

## Layers

```
 host (plugin, CLI, Rust / Swift / Python through crucible.h)
   │
   ├── Engine (Engine.h)         facade: parameters by id, notes / MIDI, process(), patches, user content,
   │                             modulation routes, meters
   ├── C API (crucible.h)           the same, as plain C with opaque handles
   │
   └── EngineCore                audio-thread core: event merge (MIDI, queued input, sequencer, arpeggiator),
         │                       sequencer parameter locks, sample-accurate segmentation, master meters
         ├── Arpeggiator, Sequencer (+ ArpPatternStore, SeqStore: lock-free pattern hand-over)
         ├── SynthEngine         parameter smoothing (32-sample control rate), global modulation, voice
         │     │                 allocation, user wavetable pinning, assignable routes
         │     ├── Voice × N     oscillator slots (wavetable / sample / cloud / noise / sub), filter slots,
         │     │                 envelopes, LFOs, matrix; N = pool + 8 spare voices for fades
         │     └── ModuleRack    the FX graph executor: every registered Effect module, wired by an FxPlan (the classic
         │                       chain is ordered by position parameters); graph::Program runs layer voices of a product
         ├── OutputMeter         peak L/R and clip latch
         └── MasterAnalyzer      loudness, true peak, correlation, scope / spectrum source
```

## Module system

`include/crucible/module/Module.h`. A module is described by a `ModuleDescriptor`:

| field | meaning |
| --- | --- |
| `id` | stable dotted id (`fx.saturator`, `voice.wavetable`); never renamed |
| `name`, `category`, `description` | display data for browsers and docs |
| `kind` | Voice, Oscillator, Filter, Effect, Modulator, Performance, Sequencer |
| `version` | bumped when the module's parameters or their meaning change (patch migration) |
| `cpu` | Light / Medium / Heavy, for budgets and UI hints |
| `params` | `ParamDef`s with stable ids, ranges, skew, unit, choices, smoothing, page placement and the `modulatable` flag |
| `createEffect`, `onParam`, `positionParam` | for effects: factory, bypass switch and chain position |

A `ModuleRegistry` object holds the modules of one product in registration order, and it owns that product's
`ParameterModel`. There are no process-wide singletons: an engine gets its registry from `EngineConfig::registry`
(built with `ModuleRegistry::build(layout)`, or `builder(layout)` + `add()` for third-party modules + `finalize()`;
several engines may share one) or builds one from `EngineConfig::product`. Two engines with different products in
one process do not influence each other. `ModuleRegistry::instance()` and the static `ParameterMap::` functions
remain as a deprecated shim for the process default product (`configureProduct`, else the neutral layout); default
engines build the same layout, each with a registry of its own. Registration is one line per module
in `src/modules/Modules.cpp`, through `sdk::registerModule` (explicit, so the order is deterministic and nothing depends on static-initialiser
tricks in a static library). Module sources live one per file under `src/modules/<kind>/` and are picked up by the
build automatically.

**Parameter model.** `ParameterModel` (`ParameterMap.h`) is assembled from the registry: every module's parameters in registration
order, placed on pages by id. The page order, page titles and the eight sections come from a *product layout*
(the neutral default is `src/modules/layout/DefaultLayout.cpp`); pages a module uses that the layout does not list are appended
automatically. Ids are the contract with patches, plugin state and host automation; a product exposes every
parameter to the host unless its `"surface"` lists a subset (`crucible_surface_count`), and new parameters are only ever added.

**Parameter threading.** `Engine::setParam` does the work at once on the control thread; `Engine::setParamRt` is the audio-thread variant that clamps, stores atomically and leaves setup work (wavetables, oscillator arming, sequencer length) pending until the control thread calls `Engine::service()`.

**Self-description.** A model's `ParamDef` is complete: id, name, `shortName` (at most six ASCII characters for
hardware LCDs), `group` (`section/page` tree path), type, range, default, `interval`, `skew`, unit, `steps`, choices,
smoothing, `polarity`, `uiHint` (knob, slider, switch, menu, xy, drawbar), `textFormat`, and the flags `modulatable`,
`voiceMod`, `routable`, `automatable`, `scene`, plus `since` (the module version that introduced it). Everything
except id, name, range and default is optional in a module's declaration and filled in when the model is built.
`ParameterModel::valueToText` / `textToValue` (`src/ParamText.cpp`) use the standard formatters (Hz, s, ms, dB, %,
semitones, cents, note name, ratio, BPM, degrees, octaves, bits, bars, choice, on/off). Floats print six significant
digits, so every bound, default and integer round-trips exactly; any text that was printed is a fixed point of the
round trip, and the parser also reads `1.5k`, `20ms`, `C#4`, `on`. Modules carry ports (audio mono / stereo, mod,
trigger), views and capability ids; a live engine adds the capabilities its instances implement.
`Engine::describeJSON()` / `crucible_describe_json()` write the whole product (schema 2, `Introspection.h`) and
`ProductDescription::fromJson` reads it back, text-identical. The older `ParameterModel::toJson` (schema 1, used by
hardware editors) is unchanged.

**Patch migrations.** `ModuleDescriptor::version` is recorded per module in every patch. When a patch saved with an
older version loads, `ModuleDescriptor::migrations` (`fromVersion -> toVersion`: id renames, value remaps, a custom
step) run in order (`ModuleRegistry::migrate`, used by `applyPatch` and `Patch::migratedParams`). Added parameters
need no migration; they load as their defaults.

**Effects at runtime.** `ModuleRack` instantiates every Effect module, resolves its parameter indices once and
per control block runs the enabled ones in position order. An `EffectModule` gets `EffectInputs`: its own
parameters by local index, already smoothed and modulated. Control-thread features are reached through
capability interfaces that a module may implement (`ConvolutionIr`, `GainReductionMeter`, `StepPosition`),
found with `ModuleRack::findCapability<T>()`.

**Status.** Effects, performance controls, scenes / sequencer, arpeggiator, the wavetable voice and the oscillator
types are modules (see "Voice graph" below).

## Modulation

* **Matrix.** Sixteen slots (parameters `modN_src/dst/amt/pol/curve`). Destinations are the fixed voice
  destinations (`ModDest`) followed by one entry for every parameter flagged `modulatable` (generic targets,
  currently all continuous effect parameters).
* **Assignable routes** (`Modulation.h`). Unlimited (128) drag-to-assign routes: any `ModSource` or MIDI CC to
  any voice destination or generic target, with amount, polarity, curve and smoothing; listed per parameter for
  modulation rings; stored in patches.
* **Modulator bank** (`Modulators.h`, module `mod.bank`). Eight modulators, route sources `kSourceModulator + n` (4000 + n), each a
  Combine (add, multiply, min, max, crossfade, difference), Remap (16-point curve), Lag (rise / fall), Quantise, S&H, Turing, Binary,
  Chaos (logistic, Lorenz), Note gate, Follower, Pitch tracker or LFO table. They run once per 32-sample control chunk in index order
  and read only ModSources and *earlier* modulators (the choice list of an input offers nothing else), so no cycle exists. Scope voice:
  state per voice, restarted by its note-on; layer or global: one state in the engine, shared by all voices (the classic voice is one
  layer), blind to voice sources. Only modulators that a route (or a needed modulator) reads are evaluated; with every type Off the bank
  costs nothing. Voices see the bank through an extended source index (ModSource, then the modulators, then the audio sources).
* **Modulation of modulation.** A route to the pseudo target `route:<n>:amount` adds its output to the amount of route *n* (clamped
  to -1 .. 1), to any depth; `ModRouteStore::add()` refuses a cycle, and the compiled `RouteSet::amountMods` list is in dependency order.
* **Audio-rate modulation.** Each voice has an audio bus of up to eight mono buffers (`kAudioBusBuffers`): oscillator outputs
  (`kSourceAudio + 4 + slot`), audio-rate LFOs (`lfoN_audio`, band-limited with the fine minBLEP of `FineBlep.h`, `AudioLfo.h`) and
  per-sample modulation and extra envelopes. A route from an audio source to oscillator pitch (`oscN_semi`), FM amount, pulse width
  (`oscN_an_pw`), wavetable position, a filter cutoff or `voice:amp` runs per sample: `osc.analog` takes `OscRender::pitchMod`,
  `fmAmountMod` and `widthMod` (exact edge placement on a smooth width curve), the wavetable oscillator takes groups of four samples,
  `AnalogFilter::process(io, n, cutoffOctMod)` samples the cutoff every four samples while it is modulated (also through the
  `FilterBank` quad path), the other filter models re-derive their coefficients every four samples, and the amplifier multiplies per
  sample. Any other target, and oscillators without per-sample input, get the chunk mean. In graph products `PortType::AudioRate`
  inputs (`filter.analog.cutoff_fm`, `filter.svf.cutoff_fm`, `amp.vca.am`, `pitch_fm` / `width_mod` / `fm_mod` of the oscillator
  nodes) accept only audio outputs, which makes the edge explicit; `lfo.audio` is the audio-rate LFO node.
* **Automation clips** (`Clips.h`). Up to 8 clips of up to 8 lanes (curve lanes with per-segment shapes, step
  lanes), loop region, one-shot / loop / ping-pong, sustain loop (MSEG), length in beats or seconds, snap grid,
  per-lane smoothing, humanise and randomise. A clip is triggered per voice, by every note, free-running, or locked
  to the host transport (position derived from the host PPQ at each block, advanced with the tempo inside it, so
  loop wraps are exact and tempo changes never jump). Each lane is a route source (`clipLaneSource`). Clips are
  published to the audio thread as immutable banks; playheads are plain doubles in `ClipPlayer`, which publishes
  copies of them through atomics once per render call (`SynthEngine::clipPositions`, `voiceClipPositions`) for UIs.
* **Motion recorder** (`MotionRecorder.h`). The audio thread timestamps the moving value of one parameter at the
  control rate; the control thread thins it, optionally quantises, and merges it into a lane (overdub or replace),
  detecting the clip length on request. Takes are stored relative to the value at arming and played back through a
  bipolar route, so host automation of the parameter is never disturbed.
* **Evaluation.** Voice destinations are modulated per voice, generic targets globally (`value + depth × range`,
  clamped). `Engine::modulatedValue(id)` reads back the modulated value without touching the automatable
  parameter, so a UI can draw live modulation while DAW automation stays intact. For per-voice destinations
  `SynthEngine::voiceModulatedValue(index)` returns the newest active voice's value (matrix slots and routes
  applied; each `Voice` keeps its last modulation arrays and the engine copies the newest voice's into atomics at
  the end of the render call).
* **UI read-back.** Everything a UI polls is an atomic written once per render call, with no locks or allocation
  on the audio thread: meters and analyzer, modulated values, clip playheads, sample / cloud views, and
  `EngineCore::soundingNotes()` (a 128-bit mask of the notes the arp and sequencer started, after Key Lock, which
  includes notes that began and ended inside the last block).

## Performance layer

* **Vector field** (`perf.vector`, `Scenes.h`, `SynthEngine::updateVector`). Four scene slots (`vector_a..d`) sit at the
  corners of a square (A top left, B top right, C bottom left, D bottom right); the puck `vector_x/y` blends them
  bilinearly. `SceneBank` publishes an immutable `SceneSnapshot` (natural and normalised values, via `RtShared`) at
  every store / clear / load; the engine pins it per render call (`attachScenes`; `Engine` attaches its own bank).
  At every 32-sample chunk, before the smoothers step, the overlay computes orbit (`vector_orbit_*`), drift
  (`vector_drift*`, seeded random walk) and one-pole smoothing (`vector_smooth`) of the puck, then sets the smoother
  targets of every scene-managed parameter: floats blend in normalised space, choices / ints / bools take the heaviest
  corner (ties: the earlier corner), a corner is the exact stored value, parameters equal at all corners are set once,
  an empty corner uses the host value, and nothing happens when no corner scene exists. While live, the host values of
  scene-managed parameters are ignored (they are kept and return when `vector_on` goes off); sequencer parameter locks
  stay with the host. `modulatedValue()` reports the morphed values. The puck is also a modulation target (routes,
  recorded paths). Everything runs at the control rate on the audio thread: deterministic, identical across block
  sizes that are multiples of 32, no allocation. Cost: nothing extra with the puck standing still; a puck that moves
  every chunk over every scene-managed float (397 varying parameters in the bench) costs about 2 % of a 24-voice render
  (`crucible_engine_bench "vector field"`; measured inside `updateVector`: 3.2 % before the 2026-10 pass, 2.2 % after). The blend
  runs in two passes over cached per-parameter records (range and smoothing are copied into them, the `ParamDef` is not
  touched) and parameter locks are filtered when the list is built.
* **Clip transport.** `SynthEngine::startClip/stopClip/isClipPlaying` (also on `Engine`): all clips run by default; a
  stopped clip fades its lane modulation out over 10 ms (`ClipPlayer::gate`) and parks its playheads, start restarts it
  from the beginning. Patches store the stopped clips (`clipsStopped`, a bit mask, written only when non-zero; an additive
  field, so the format version stays 2 and older patches load with every clip running); loading a patch applies it with
  `SynthEngine::setClipRunMask`, which restarts only clips that go from stopped to running.
* **Two-lane motion recording.** `MotionSettings::param2/lane2` records a second parameter into a second lane of the same
  clip on the same clock (a vector path is `vector_x` + `vector_y`).

* **Key lock** (`KeyLock.h`). `EngineCore` runs every note (keyboard, sequencer, arpeggiator input and output)
  through a `NoteQuantiser` before the voice allocator. Modes Off / Snap / Block / Remap; the scale is a 12-bit mask
  (18 choices: chromatic, major, three minors, the other five modes, two pentatonics, blues, whole tone, two
  diminished, custom mask, mask of the loaded Scala scale). Note-offs follow the note that was started, so changing
  the scale mid-note never leaves a stuck note; bend, MPE and controllers are untouched. With the mode Off the
  engine renders bit-identically to one without the feature. `Engine::keyScale()` returns the read-only view (12
  in-key flags plus root) for UIs; `KeyDetector` proposes root and scale from played notes or a MIDI file.
* **Key bus** (`KeyBus.h`). Key Sync = Lead publishes the engine's key, Follow adopts the bus key (last writer wins,
  generation counter, layout version). The core provides the in-process bus; across processes the host supplies a `KeyBus` subclass over memory it maps
  (the product adapters use a 64-byte memory-mapped file in the per-user temp directory, `$CRUCIBLE_KEYBUS` overrides),
  chosen over loopback UDP because it needs no port or listener thread, is not reachable from the network, and costs
  one atomic load on the audio thread. Hosts attach it with `Engine::attachKeyBus()`
  (`KeyBus::local()` inside one process, a host-supplied `KeyBus` across processes); the Rust daemon can map the same file
  (layout in the header) or use the C API (`crucible_keybus_*`).
* **Tuning / MPE / modes.** `Tuning.h` publishes a per-note semitone table the voices add to their pitch;
  `tune_amount` blends it. With `mpe_on`, member channels carry per-note bend (applied per voice), pressure and slide
  (route sources 3000..3002); poly aftertouch reaches its note. `voice_glide_mode/type` and `voice_mono_priority`
  extend the Mono / Legato modes. `CcMap.h` replaces the single CC table.

## Real-time rules

* `process()` never allocates, locks or blocks. Tests count `operator new` calls while the engine runs with
  voices, sequencer, arpeggiator, every effect, user tables and routes, including an impulse-response swap.
* Data that the control thread changes while audio runs (sequencer patterns, arp grid, routes, user wavetables,
  impulse responses, cloud models) is published as immutable objects through `RtShared` / hazard pointers; the
  audio thread pins and releases them without locks, the control thread frees superseded copies.
* Performance input from other threads goes through a lock-free SPSC ring.
* Parameters are smoothed at control rate (linear or log per parameter); denormals are flushed in
  `Engine::process()`.

## Quality

* Wavetables are stored at 27 band-limited levels (1023 ... 1 partials, 128 to 4096 samples per cycle) and read with
  6-point Lagrange interpolation; the level keeps every partial below (rate - 20 kHz), so aliasing lands above 20 kHz.
  The frame morph ramps per sample. Hard sync uses minBLEP / minBLAMP (`MinBlep.h`). Warps run 2x / 4x oversampled
  through half-band filters; the cloud oscillator skips partials above 0.45 fs.
* `EngineConfig::quality` (Draft / Standard / High) and `EngineConfig::oversampling` (linear-phase FIR half-band or
  polyphase all-pass IIR) are handed to modules as an oversampling policy (`ModuleContext`). The drive uses
  `Oversampler` (2x / 4x / 8x); warped oscillators cap at 2x in Draft and run 2x modes at 4x in High. New modules
  should honour the policy.
* Nonlinear filters (`filters/FilterModel.h`, `src/filters/`): `AnalogFilter` models integrate their capacitor
  equations with the trapezoidal rule and solve the tanh terms of one sample by Newton's method in closed form
  (4x4 cyclic or tridiagonal systems). The tier follows the quality policy: Draft 1x, Standard 2x, High 4x with
  Newton to 1e-6 (the other tiers take one step from the linear ladder). Each model owns up to four lanes, so one
  object runs four voices in lock step (`processQuad`, `simd::Float4`). A drive shaper runs at twice the filter rate,
  the self-oscillation frequency is corrected by a table measured on the model at first use, and `tolerance` draws
  a seeded part-to-part spread. The models are entries 17 and up of `FilterModel`; `FilterChannel` preallocates all
  of them in `prepare()` so selecting one never allocates on the audio thread. The legacy models 0 to 16 are untouched.
* Voices and the nonlinear filters (`FilterBank.h`, `Voice::beginChunk` / `filterIn` / `filterOut` / `endChunk`):
  `SynthEngine::renderClassicVoices` renders a chunk in stages when a filter slot holds a model of index 17 or up. Every
  sounding voice runs its pre-filter part (`beginChunk`), then for each slot the voices hand their filter input to the bank
  (`filterIn`), the bank filters four lanes at a time (`FilterBank::process`), and the voices pick up the result
  (`filterOut`) and finish the post-filter part (`endChunk`: mix, pan, routing, amplitude). A voice has two filter channels,
  so voice 2p and 2p + 1 own the four lanes of one `AnalogFilter` object per slot and model; the assignment is fixed, a
  voice that starts or whose model changes resets only its two lanes (`AnalogFilter::resetLane`), and nothing is copied
  in or out. Serial, parallel and split routing, mix, pan and per-voice cutoff modulation are the voice's, unchanged.
  `EngineConfig::batchAnalogFilters = false` renders every voice alone (the reference of `tests/VoiceQuadTests.cpp`).
  `Voice::renderChunk` is the same stages with the voice's own filters; patches without a nonlinear model take it.
* Denormals are flushed in `Engine::process`, `EngineCore::process` and `SynthEngine::render`.
* Measurements without a plugin host: `crucible-measure` renders a job file (parameter settings by name, one WAV per
  render) so the analysis suite in `tools/reference-bench` can score the engine (`tools/refbench.sh`).
* Wavetable memory: a factory table takes 9.0 MB (all fourteen 121 MB), so tables are generated the first time an
  oscillator is pointed at them: `Engine::prepare` and `Engine::setParam` do it on the control thread (11 ms per
  table); the audio thread never generates one: a table switched with `setParamRt` (CC, host automation) is silent
  until `service()` has generated it. `EngineConfig::preloadWavetables` generates everything at
  construction for hosts that want no first-use work on the audio thread. A default patch holds 9 MB of tables.
* `simd::Float4` / `simd::Int4` (`Simd.h`) wrap NEON / SSE (AVX2 gathers with `-DCRUCIBLE_X86_AVX2=ON`) / scalar. The
  unison oscillators (plain, FM, ring, hard sync, fold) run four voices per vector (`runGroup4` in `Voice.cpp`), the
  half-band decimators work on blocks, the convolver uses `Float4`; other code is written for compiler
  vectorisation. `-DCRUCIBLE_FORCE_SCALAR=ON` compiles the scalar paths only; `simd::setKernelsEnabled(false)` switches
  the vector kernels off at run time (tests compare both in one binary, `tests/SimdTests.cpp`).

## Determinism and parity

* Same configuration, seed and input give the same samples (`EngineConfig::seed` mixes into per-note random
  state). Golden fingerprints in `tests/golden/renders.txt` guard the sound (0.05 dB tolerance).
* Blocks that are multiples of the 32-sample control block render bit-identically when events fall on that grid;
  offline (`crucible-render`) and real-time use the same code path; the C API renders bit-identically to the C++
  facade.

## Interfaces

* C++: `Engine.h` (facade), `EngineCore.h` (hosts that own parameter storage, like the plugin), the module and
  DSP headers for advanced use.
* C: `crucible.h`, versioned by `CRUCIBLE_API_VERSION`; functions are only added.
* Patch: versioned JSON (`Patch.h`, format 2), migrated step by step on load (1 to 2 adds preset id, modified date,
  description and product); unknown parameters are ignored and missing ones take their defaults. Hosts that keep the
  parameters themselves (the plugin's parameter tree) use `capturePatch(EngineCore&, values, scenes)` and
  `applyPatchContent(EngineCore&, patch)`, which cover everything else; the CC map stays with `Engine`.
  `SampleBank`, `CloudBank` and `UserWavetableBank` expose `serialise()` / `deserialise()` records and `Json.h`
  has float32 base64 helpers for host state.
* Import helpers: `detectWavetableFrames` reports the frame size and why (clm marker, 2048 multiple, single cycle,
  fallback, forced), `estimateCycleLength` uses the sample pitch detector, and `nearestZeroCrossing` /
  `findLoopPoints` (`Sample.h`) snap sample markers to rising zero crossings.
* Presets: `PresetLibrary.h` keeps patches in factory (read-only) and user banks on disk, indexes them (with a cache
  keyed by size and modification time), searches by text, tags, category, rating and favourites, and stores large
  content (wavetables, impulse responses, clouds, samples) either inside the patch or in a shared asset store.
* CLI: `crucible-render` renders a patch with a MIDI file, a text score or one note to WAV, and lists modules and
  parameters.

## Threads and multi-core

Voices render on the audio thread. A multithreaded voice renderer behind a flag is not implemented (see ROADMAP):
the voice loop is self-contained per voice, so a split into workers that sum in voice order would be deterministic,
but a real-time safe hand-over (no locks, bounded waits, priority inheritance for workers) needs host-specific
thread support first. Single-thread cost is low: 16 notes x 8 unison x 1 oscillator uses 4 % of a core.

## Voice graph

The voice is a graph of slots (`Voice.h`, `modules/voice/`):

```
 oscillator slot 1..N ──(type: wavetable | sample / granular | cloud | noise | sub)──┐  level, pan
   cross modulation from an earlier slot: FM, ring, hard sync                         │  route
                                                                                      ▼
   serial   : all sources → filter 1 → filter 2 → ... → filter M
   parallel : all sources → every filter, outputs summed
   split    : each source names a filter, "all" or "dry" (past the filters)         → amp envelope × velocity
```

* **Counts.** The parameter model declares the parameters of every slot, so the shape (`VoiceGraphShape`: oscillators
  and filters) is fixed per product, i.e. per engine instance: `EngineConfig::product` (a `ProductLayout`: sections,
  page order, shape and macro count) or `ModuleRegistry::build(layout)`; the neutral layout has 3 and 2. The
  deprecated `ModuleRegistry::configureProduct()` / `configureVoiceGraph()` choose the layout of the process default
  product (they return false once it has been used); a product with eight macros adds `macro7`, `macro8` (page `macros_2`) and
  the route sources `Macro 7` / `Macro 8` (appended to `ModSource`, so no id moved); the default keeps six. `EngineConfig::numOscillators` / `numFilters` switch higher
  slots off. Up to 8 oscillators and 4 filters. Slot 1-3 / filter 1-2 keep their original ids and matrix destinations;
  slots beyond have the same parameter layout (`osc4_...`, `filter3_...`) and are modulated through the per-voice
  generic targets below.
* **Oscillator types.** `oscN_type` picks a registered `Oscillator` module. Type 0 is the wavetable oscillator that
  lives in the voice (unison stacks, 15 warps, user tables). The others are `OscillatorInstance` objects created per
  voice and slot at prepare time (`module/Oscillator.h`); they render unit-level stereo at the pitch the voice
  computed, and the voice applies level, pan and routing. Sample audio (`SampleBank`) and clouds (`CloudBank`) are
  published per slot through `RtShared` and pinned for each render call; both travel in patches as assets.
  * `osc.sample`: pitched playback with start / end / loop (forward, ping-pong, crossfade) or a granular mode (size,
    density, position, spray, pitch, window, reverse probability, spread, scan, jitter). Two band-limited and
    decimated copies per octave and a windowed-sinc read keep transposition below the 20 kHz folding limit.
    `SynthEngine::sampleView()` returns the waveform overview, playheads and grains for a UI.
  * `osc.cloud`: the additive 3D cloud, `SynthEngine::cloudView()` returns points and per-point energy.
    Spectral drawbars: `oscN_cloud_band1..8` (0..2, default 1, log smoothing, per-voice modulatable) scale the partials in
    eight octave bands by partial number (1, 2, 3-4, 5-8, 9-16, 17-32, 33-64, 65+; `cloudBandOf`) before the output
    normalisation, so all ones render exactly as before. Log-smoothed parameters are floored at 1e-4 (-80 dB) so a
    drawbar can sit at zero. Module version 2; older patches simply lack the ids and take the default.
  * `osc.noise`, `osc.sub`: seven noise colours with tilt and width; sub in five waves, three octaves, with drive.
  * `osc.analog`: virtual analog oscillator (`src/modules/oscillator/AnalogOscillator.cpp`), registered last so no parameter index moves; its name is listed last in the type choice by `voice.oscillators`.
  * `osc.modal`, `osc.harmonic`, `osc.spectral` (P13, types 6, 7, 8, appended in that order): `ModalOscillator.cpp` is a bank of up to 64
    zero-delay-feedback state-variable modes whose poles are placed exactly from the T60 and the frequency (so both are exact at any
    sample rate) or a comb string whose loop delay and gain are corrected at the fundamental, played by a strike, pluck, particle, blow,
    noise or bow exciter (designs after Mutable Instruments Rings and Elements, MIT, `third_party/mutable/`); `HarmonicOscillator.cpp`
    builds a partial law (count, odd / even, stretch, tilt, a formant bell fixed in Hz, per-partial decay) into a `CloudModel` inside the
    instance and plays it with the cloud renderer; `SpectralOscillator.cpp` plays the slot's sample through Signalsmith Stretch (MIT) with
    independent speed (0.25x to 4x), pitch and formants, primed at note start so the output has no latency. All three are also graph node
    types. Every voice owns an instance of every type, so the heavy state (the stretcher, 0.8 MB; the string and bow buffers of `osc.modal`) is
    built by `OscillatorInstance::arm()` on the control thread when a slot first selects the type (`Engine::setParam` on `oscN_type`,
    `Engine::prepare` for the current selection, a graph node on `prepare`; `SynthEngine::armOscillator`); an unarmed instance is silent.
    A product that does not want the type lists it in `ProductLayout::omit`.
  * Spectral wavetable path (`include/crucible/Spectral.h`, module `voice.spectral_wt`, registered last): `oscN_wt_interp` (Crossfade, the
    legacy default, or Spectral Sweep), `oscN_wt_spec` (Off, Shift, Stretch, Blur, Comb, Dispersion, Tilt) and `oscN_wt_spec_amt`. The peaks of
    neighbouring frames are matched when a table is analysed (control thread: factory tables on first use by `Engine`, user tables with
    the table), the frequency axes are warped so matched peaks meet, and the voice synthesises the cycle for its mip level with one
    inverse transform, cached per slot and quantised position (no analysis, allocation or locks on the audio thread). With the defaults
    the voice runs the legacy code exactly.
  * `crucible/Fft.h` backends: Accelerate on Apple, pffft (`third_party/pffft/`, SIMD, about 4x to 9x the portable code, benchmark
    `crucible_engine_fft_bench`) elsewhere, the double-precision radix-2 code as the reference (`-DCRUCIBLE_ENGINE_FFT_BACKEND=PFFFT|PORTABLE|ACCELERATE`).
* **Cross modulation.** `oscN_xmod_mode / src / amt`: a slot listens to an earlier slot (which is rendered even at
  level zero). FM and ring use the other slot's output upsampled to the oscillator's rate; hard sync restarts the
  carrier at the master's wraps with a minBLEP (step) and minBLAMP (slope).
* **Per-voice generic modulation.** Parameters flagged `voiceMod` (`params::voiceModulatable`) are modulated inside
  the voice by matrix slots and assignable routes (including CC, clip lanes and MPE sources) with the voice's own
  sources, without a `ModDest` entry: value + depth x range, or in octaves for log-smoothed parameters.
  Unison detune, spread, blend and scatter of every slot (and the sample oscillator's unison detune and spread) are
  per-voice targets since `voice.unison_mod`: the parameters stay in their modules, which list them in
  `ModuleDescriptor::lateVoiceMod` so the registry numbers them after every other target. Slot 1's stack detune keeps
  `ModDest::UnisonDetune` (which offsets every slot). Voice count, stack mode and random phase are not modulatable.

## Graph in data

A product can be a document instead of C++: `crucible-product` JSON (`graph/Product.h`, `products/*.json`). The compiler
(`graph/Compiler.h`) validates it and returns a `CompiledProduct`: the product's `ModuleRegistry` (so its parameter
model), the schedules of the voice graph and the FX graph. `EngineConfig::compiled` plays it. Everything is built on the
control thread; the audio thread only runs prebuilt tables.

* **Instances and ids.** The registry is the built-in modules (minus the effects an `fx` section leaves out), one copy of
  an effect module per further instance, and one module per graph node that has parameters. The first instance of an
  effect keeps its flat ids (`fx_saturator_drive`); instance k >= 2 of a type is `fx{k}_saturator_drive` (module `fx.saturator.2`, see
  `ModuleDescriptor::instanceOf`); node parameters are `<instance>_<param>` (`kick_env_attack`). `idMap` pins a
  generated id to another one, and compilation fails when a pin matches nothing or collides. Copies are registered after
  every built-in module, so no existing parameter index, generic modulation destination number or patch changes.
  `products/crucible-default.json` (generated by `defaultProductDef()`, checked by a test) lists every built-in effect in
  one position-ordered lane; its registry has exactly today's parameters in today's order.
* **Voice.** `"voice": "classic"` is `classic.voice`: today's whole `Voice` pool (the allocation, glide, mono and MPE
  logic stays in `SynthEngine`); the node is the graph's handle on it, so the classic product's sound is untouched
  (tests compare it sample for sample). `"voice": "none"` is an FX product. A graph voice is nodes and edges: audio edges
  carry stereo buffers (several edges into a port are summed), modulation edges carry one value per 32-sample chunk
  (scaled by the edge's amount). Node types (`graph/Node.h`): `osc.noise|sub|sample|cloud` wrap the oscillator modules
  and use the slot's `oscN_...` parameters, `env.adsr` (the voice envelope), `lfo.basic`, `filter.svf` (the legacy filter
  models), `filter.analog` (the nonlinear ZDF models 17-19 with drive, character and tolerance), `amp.vca`, `mix.out`
  (the layer output) and `classic.voice`. Hosts add types with `NodeRegistry::add()`; the module SDK adds the types of
  third-party modules.
  A node implements `prepare`, `reset`, `noteOn`, `noteOff`, `process(NodeIO&, n)`, optionally `processQuad` (four
  voices of one node together; the executor tries it first) and `latency`. Envelope nodes keep the voice alive
  (`holdsVoice`); a voice ends when they finish.
* **Layers.** `voice.layers` (up to 16): each has a note range, a velocity range, `voices` (polyphony, oldest stolen
  first), `oneShot` (note-off ignored; envelopes release at their sustain stage), a `choke` group (a new note silences
  the group's other voices with a 5 ms fade) and a `round_robin` group (matching layers of a group take turns).
* **FX graph.** `ModuleRack` executes an `FxPlan`: lanes (a serial chain of rack slots, ordered by their position
  parameters or as listed) that read the voice bus, the host input (`aux`, `process_aux`) or earlier lanes (sends), and
  sum into the master with a level. A plan of one lane over the voice bus runs in place on the bus, which is the classic
  chain exactly; other plans use preallocated lane buffers (at most 16 lanes). Copies of an effect do not receive the
  legacy matrix destinations of the first instance. Lanes are sorted so that a lane only reads earlier lanes; cycles
  are refused.
* **Reconfiguration.** `compileReconfiguration(base, newDef)` compiles a definition with the same parameters (another FX
  topology, other layers, chokes, edges; adding modules needs a new engine). `SynthEngine::reconfigure()` builds a
  `graph::Program` and publishes it through `ProgramHost` (an `RtShared` of handles plus a hazard slot protocol for the
  programs the audio thread still uses). At the next control cell the audio thread starts a 10 ms linear fade of the
  output, swaps the program at silence (killing the old graph voices), and fades back up over 10 ms; it allocates and
  locks nothing. Superseded programs are freed by the control thread.
* **Limits** (`ProductLimits`): 32 nodes per voice, 64 FX instances, 16 layers, 64 modulators; 16 FX lanes.
* **Not yet:** per-voice modulation of node parameters (node parameters read the smoothed product values, so they are
  not routable per voice), feedback nodes (cycles are refused), modulators with scope layer or global, and adding modules
  while an engine runs.

## Views

`Views.h` is the read-back API for GUIs, hosts and scripts: nothing in it knows a toolkit. A **view** is a read-only window
onto the running engine, named `{instance, id}` and carrying a typed payload of 32-bit elements: a scalar, a vector, points
(tuples of `stride` floats) or a mask (words of bits). `Engine::views()` (or `EngineCore::views()`) gives the `ViewHub`;
`crucible_view_*` is the same from C.

* **Publication.** Live views are produced by the audio thread at the end of `EngineCore::process`, at most once per call and
  no faster than the fastest subscriber's rate limit (the remainder carries over, so the cadence does not depend on the block
  size). A scalar is one atomic 64-bit word (sequence number and value). Other payloads use four slots per view: the writer
  picks a slot that is not the newest and not being read, marks it `writing`, checks again that no reader holds it, fills it and
  makes it the newest; a reader announces itself on the newest slot, checks that it is not being written, copies and leaves
  (the two checks are seq_cst mirror images, so either the reader retries or the writer takes another slot). The writer never
  waits: if every spare slot is held by a reader, that publication is skipped (`ViewHub::dropped()`). Readers on any thread get
  a consistent snapshot and its sequence number; a payload with the same sequence number is the same snapshot.
* **Cost.** Nothing is allocated for a view until the first subscription, and a view nobody subscribed to is never produced.
  `ViewHub::publish()` with no subscription at all is one relaxed atomic load (about 1.3 ns per render call, measured in
  `ViewTests`). Producers allocate nothing and lock nothing (the sample and cloud scratch vectors are reserved when the engine is
  built; `kOscVisualMaxGrains` bounds the grains one voice reports).
* **Subscriptions** belong to the control thread: `subscribe(view, maxHz)` returns an id, `unsubscribe(id)` ends it. Several
  subscribers of a view share one publication at the fastest rate; a derived view (a spectrum) subscribes its source (the scope).
* **Pulled views** are computed by the reading thread from control-side data or from another view: the sample overview, slices
  and zones, the cloud points, and the spectra (an FFT of the published scope). Their sequence number advances when the content
  changes. They are safe to read from any thread: the control-side objects (`SampleBank`, `ZoneBank`, `CloudBank`) are read
  through `RtShared::withLatest()`, which holds off `publish()` and `collectGarbage()` while the reader copies, so a sample or
  cloud replaced on the control thread cannot be freed under a reader (the audio thread never takes that lock). Pulled views
  never touch the voices; playheads and grains are live views.
* **Catalog** (`crucible_view_count` / `_info`, or `ViewHub::info()`): `master/meter` (peak L dB, peak R dB, clip),
  `master/loudness` (momentary, short-term, true peak, max true peak, correlation), `master/scope` (4096 left then 4096 right),
  `master/spectrum_256..4096`; `engine/sounding_notes` (mask, word k = notes 32k..32k+31), `engine/active_voices`,
  `engine/vector_position` (x, y, active); `sequencer/step`, `sequencer/running`; `clips/positions` (kMaxClips, as floats); for
  every parameter `params/<id>.mod` (value after global modulation), `.voice` (newest voice), `.band` (lowest, highest, number of
  active voices) and `.voice@<n>` (voice n, made on demand: `find()` resolves it); for every oscillator slot `oscN/sample_info`,
  `sample_peaks`, `sample_slices`, `sample_zones`, `sample_playheads`, `sample_grains`, `sample_zones_active`, `cloud_points`,
  `cloud_energy`, `cloud_state`.
  Effect modules that declare the `gain_reduction` capability publish `<module id>/gain_reduction` (scalar, dB, positive when the
  signal is turned down): `fx.comp_clean`, `fx.comp_vca`, `fx.gate2`. Effect modules that implement `ResponseView` and declare the
  `response_view` capability publish `<module id>/response` (the current magnitude response in dB at 256 points spaced logarithmically
  from 20 Hz to 20 kHz, output gain included) and `<module id>/spectrum` (the output spectrum in dB, 1025 bins from 0 Hz to Nyquist):
  `fx.eq_pro`. Both are computed on the audio thread, only while somebody is subscribed.
  Effect modules that implement `MeterReadout` (`crucible/module/MeterReadout.h`) and declare the `loudness` capability publish
  `<module id>/loudness` (vector of 8: momentary, short-term and integrated LUFS, loudness range LU, peak dB, largest momentary LUFS,
  range low and high LUFS); with the `levels` capability also `<module id>/levels` (vector of 11: sample peak, decaying true peak, RMS,
  PPM and VU in dB per channel, correlation): `fx.maximizer` (loudness, of its output) and `util.meter` (both).
  The voice filters publish pulled views `filter/response` (filter 1) and `filter2/response`, `filter3/response`... (the
  parameter prefixes): the small-signal magnitude response in dB at 256 points spaced logarithmically from 20 Hz to 20 kHz
  (`filterResponseDb()` in `Filters.h`), computed on the reading thread from the type, cutoff, resonance, drive, character and
  high-pass parameter values as of the last render call (`SynthEngine::baseValue()`); `filter/response_mod` does the same from the
  `.mod` values (`modulatedValue()`). The linear models (state variable, ladder, comb, formant, phaser) are exact to the filter's
  own coefficients (measured within 0.02 dB of stepped sines); the nonlinear models use the small-signal limit of their circuit
  equations through `AnalogFilter::responseDb()` (measured within 0.05 dB). A loop that self-oscillates is drawn at 99.5 % of its
  critical gain. The view reads no voice state; the prepared analog models behind it are guarded by a lock the audio thread
  never takes.
* **Discovery.** `describeJSON` lists every view of the engine with its instance, id, kind, payload capacity in 32-bit elements,
  stride, live / pulled and owning module (the per-parameter `params` views are not listed one by one), and each module's
  `views` holds the ids it owns. `crucible_view_size(e, view)` returns the payload capacity from C.
* **The old accessors stay.** `meter()`, `loudness()`, `MasterAnalyzer::scope()/spectrum()`, `modulatedValue()`,
  `voiceModulatedValue()`, `clipPositions()`, `sampleView()`, `cloudView()` and `soundingNotes()` read the same sources, and
  `ViewTests` checks that every view carries the same values (bit for bit). `voiceModulatedValue()` and the band share
  `SynthEngine::applyVoiceMod()`. Strings (a sample's name) stay on `sampleView()`.
* **Not yet:** views of graph-node parameters and of modules outside the built-in set (the SDK, P5, will register its own through
  `ViewHub::add()`) and double-precision payloads.

## Module SDK and C ABI

Third-party modules (`include/crucible/crucible_module.h`, `include/crucible/sdk/`, `src/sdk/`) are shared libraries with a
C ABI. The conventions are CLAP's: every struct starts with `size` and `abi_version` and only grows at the end, strings are
UTF-8, the host hands a table of callbacks (`crucible_module_host`), and a library exports one symbol,
`crucible_get_modules`, that lists its descriptors in static storage. A descriptor carries identity (dotted id, vendor
first; the engine's own first segments are reserved), kind, version, CPU class, a parameter table, ports and the function
pointers `create`, `destroy`, `prepare`, `reset`, `process`, `note_on`, `note_off` and the optional `process_quad`,
`get_view`, `latency`, `holds_voice`.

* **Host side.** `sdk::ModuleSet` (C handle `crucible_module_registry`) validates a descriptor (`validateDescriptor`: ABI
  range, sizes, id syntax, parameter ranges and defaults, unique ids, ports, required functions), keeps a host-owned copy
  and, for `addTo(NodeRegistry&)`, wraps each module as an adapter. An effect module becomes a `ModuleDescriptor` whose
  factory builds an `EffectModule` that calls `process` on the bus in place (parameters via `EffectInputs::value`, so
  smoothing and generic modulation apply; a bypass switch is added by the host) and is added to the `NodeRegistry` as an
  effect type; the graph compiler registers it like a built-in effect (first instance with its ids, further ones as
  `fx{k}_...` copies). Every other kind becomes a `graph::NodeType` whose nodes call `process` / `process_quad` with the
  `NodeIO` buffers translated to `crucible_module_process` on the stack. Nothing allocates after `prepare`; mono ports
  are presented with a null right channel and mirrored afterwards.
* **Lifetime.** Adapters hold a `shared_ptr` to the module copy, which holds the `SharedLibrary`; a library is closed after
  the last engine, node or effect made from it is gone, whatever happens to the set.
* **Versions.** A host accepts `abi_version` in `[CRUCIBLE_MODULE_ABI_MIN, CRUCIBLE_MODULE_ABI_VERSION]` and reads a struct
  up to its `size`; a library, descriptor, parameter or port from a newer ABI is refused before anything is called, with
  `CRUCIBLE_MODULE_ERR_ABI_VERSION`. Library loading (`dlopen` / `LoadLibrary` behind `sdk/SharedLibrary`) is allowed per
  registry by the host and compiled out with `CRUCIBLE_ENGINE_MODULE_LOADING=OFF`; the engine never scans directories.
* **One registration path.** The built-in list in `src/modules/Modules.cpp` and the compiler's registration of effect copies
  and node instances go through `sdk::registerModule`, so a built-in is a descriptor in a registry like any other.
* **Check.** `sdk::check` (`Check.h`, `tools/crucible-module-check`) drives any module block by block through a `Runner`
  (effect, oscillator or node adapters): allocation counter (hooks supplied by the program), extremes sweep, finite and
  denormal checks, determinism per seed and after reset, CPU by class. The tests run it over the examples, deliberately
  broken modules and every built-in.

## Testing and CI

`tests/` (built-in framework, `TestFramework.h`): DSP behaviour per module, aliasing measured by FFT, effects at
parameter extremes, sequencer / arp timing, patches, C API (from C), golden renders, parity, fuzzing (random
parameters and MIDI), master analyzer accuracy. `tools/ci.sh` builds and runs everything in Release, ASan + UBSan,
TSan, the scalar fallback and (via Rosetta, or natively on x86_64) x86_64 builds with SSE and AVX2. `crucible_engine_bench` reports CPU per configuration and per effect module.
