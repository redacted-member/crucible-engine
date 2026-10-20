# Plan: Crucible Engine, an open community synth engine

Status: the design plan written before implementation; where it and the code differ, the code and the reference
docs (docs/ARCHITECTURE.md, docs/MEASUREMENTS.md) are current.

Goal: release the engine as an open engine "that allows people to create synths like Arturia Pigments,
Serum 2, u-he Diva, Kilohearts Phase Plant, etc. The closer it gets to the quality of real hardware, the better."

## Maintainer decisions (fixed)

* Do not develop demo products (P14 removed); demos are built separately later. Product definitions in data stay part
  of the platform. This plan covers the engine only.

* **Name**: Crucible Engine.
  * Namespace `crucible`.
  * Macro prefix `CRUCIBLE_`, C API prefix `crucible_`.
  * CMake target `crucible::engine`.
* **Licence**: MIT. The project owns the full product, so the whole community gets access.
* **No GPL dependencies, ever.** Surge XT and Vital are studied clean-room: public docs, architecture, design
  choices and known pitfalls. Their code and tables are never copied. Permissive dependencies (MIT / BSD / zlib /
  Apache-2.0 / ISC) are allowed where they are clearly better than what we would write.
* **Extraction**: a repository of its own with git history. Nothing is pushed.
* **Order**: the rename and isolation land before the big feature packages, right after the verification fixes.

## Top-level design principle: Crucible is a platform for many different products

Crucible must be dynamic enough that developers can build many unique-sounding products and GUIs on it without
forking it. Examples:

* a Diva-like VA;
* a Pigments-like hybrid;
* a Phase Plant-like modular;
* a drum synth;
* a granular instrument;
* an FX plug-in;
* hardware-controller instruments built by host applications.

Every design decision in this plan is checked against six rules:

1. **Graph in data, not code.**
   * A product is a *product definition*, a JSON data pack containing:
     * its voice graph: generators, filters, shapers, mixers, envelopes and VCAs as composable module instances,
       in any count, with any routing, and with lanes in series or parallel;
     * layers: note / velocity ranges that select a subgraph, so a drum kit is several layers;
     * its modulators, its FX graph (any number of instances, in serial and parallel lanes), macros, pages,
       presets and default patch.
   * Hosts load or change a product at runtime. Each change goes through the control thread, which compiles the
     graph and swaps it in with a short fade.
2. **Everything self-describing.**
   * Each module and parameter carries its id, name, range, skew, unit, display-text function, steps or choices,
     group, page hint, modulatable and per-voice flags, default and version.
   * Any GUI can be generated from this metadata, or hand-built, without engine changes.
3. **GUI-agnostic read-back.**
   * Meters, scopes, spectra, modulated values (global and per voice), envelope / LFO / MSEG playheads, and
     wavetable, sample, grain and cloud views.
   * Published lock-free (atomics, triple buffers, `RtShared`).
   * Polled by any thread, in any UI toolkit.
4. **Extensible by third parties.**
   * New modules are registered through a plugin-style module SDK with a stable, versioned C ABI, so modules can
     come from other languages or shared libraries.
5. **No product assumptions in the core.**
   * Host applications' products are definitions in data. The core ships a neutral default.
6. **Bit-exact continuity.**
   * The existing product, defined as data, renders exactly what the hard-wired engine rendered.
   * Existing patches load and sound identical.

Further ground rules:

* **Product knowledge.** Only from public documentation. Reference plugins are measured only as black boxes with
  `tools/reference-bench`: no disassembly, no preset or resource extraction.
* **Names.** Module ids and names are generic, never product or trademark names.
* **Adopted code.** Vendored under `third_party/<name>/` with its licence file and an entry in
  `THIRD_PARTY.md`.
* **Paths.** Paths in this plan are relative to the repository root of the engine (`include/crucible/...`).
* **New parameters.** Appended; ids are never renamed. Legacy code paths stay byte-identical. Intended sound changes
  are recorded in `docs/SOUND_CHANGES.md`.

---

## 0. The niche Crucible fills

| Existing open source | Licence | What it is | Why it does not cover the goal |
| --- | --- | --- | --- |
| Surge XT (+ sst-filters, sst-waveshapers, sst-effects, sst-basic-blocks) | GPL-3 (a few headers MIT) | Complete hybrid synth with an excellent filter and waveshaper zoo | GPL. A synth with one fixed architecture (2 scenes, fixed slots), not an embeddable, data-defined engine |
| Vital | GPL-3 | Wavetable synth: spectral warps, audio-rate modulation everywhere | GPL. One fixed product, not an embeddable engine |
| VCV Rack, Odin 2, Dexed, ZynAddSubFX, Helm | GPL | Complete synths and a modular host | GPL |
| Mutable Instruments code (Plaits, Rings, Elements, Clouds, Marbles, stmlib) | MIT | Superb single-voice Eurorack DSP | No polyphony, patch format, modulation system or FX rack |
| DaisySP | MIT | Embedded DSP blocks (its LGPL parts are split into a separate repository) | Blocks only; embedded-grade quality |
| Soundpipe, STK, Maximilian, Gamma | MIT / STK / MIT-like | DSP blocks and physical models | Blocks only; mixed quality; no VA alias floors near -100 dB |
| Airwindows | MIT | Mix effects and saturation | Effects only |
| chowdsp_utils | mostly BSD-3, some modules GPL-3 | JUCE DSP utilities | JUCE coupling; not an engine |

**The niche.** As far as public information shows, nothing combines all of the following under a permissive
licence:

* a complete polyphonic engine;
* framework-free and real-time-safe;
* defined by data;
* self-describing for any GUI;
* extensible by third parties through a C ABI;
* hardware-grade nonlinear filters and VA oscillators;
* wavetable, sample, granular, modal and spectral sources;
* deep modulation, an FX graph, and versioned patches.

Every complete engine of that calibre is GPL. Every permissive project is a set of components.

**Build vs adopt.**

* **BUILD** the identity: the graph and product system, metadata, SDK, UI data API, modulation, the nonlinear
  filters and VA oscillator, the FX graph.
* **ADOPT** permissive components where they match or beat what we would write:
  * Mutable resonators;
  * Signalsmith Stretch;
  * pffft (if faster);
  * Surge's MIT tuning-library, if ours disagrees with it.

---

## 1. Gap analysis

* **Status**: **have** (implemented and tested), **partial** (narrower than the reference; the gap is stated),
  **missing**.
* **Ref** (who has the feature): P = Pigments, S = Serum 2, D = Diva, PP = Phase Plant.
* **Prior art / adopt vs build**: the best open implementation and its licence, then the recommendation:
  * **ADOPT**: a permissive implementation is as good as or better than ours;
  * **BUILD**: no permissive equivalent of that quality exists (GPL projects only inform the design, clean-room);
  * **KEEP**: ours is already good;
  * **SKIP**: do not build.
* **Pkg**: the work package that closes the gap.

### 1.1 Platform (top-level principle)

| Feature | Ref | Engine today (file / function) | Status | Prior art / adopt vs build | Pkg |
| --- | --- | --- | --- | --- | --- |
| Voice graph defined in data, any count, any routing, runtime reconfiguration | PP (fully modular), P / S / D (fixed) | `VoiceGraphShape` (up to 8 osc / 4 filters, fixed per process, `ModuleRegistry::configureVoiceGraph` before first use); routing serial / parallel / split (`VoiceParams::filterRoute`); the voice is hand-coded in `Voice.cpp` (1526 lines) | partial | Phase Plant shows the target, VCV Rack the extreme (a GPL modular host). BUILD | P4 |
| Several products per process / per engine instance | — | `ModuleRegistry::instance()` and the static `ParameterMap` are process singletons; a product is fixed per process (`tests/ProductMain.cpp` needs its own executable) | missing | BUILD (instance-scoped registry) | P3 |
| FX graph: multiple instances, parallel lanes, sends | S PP P | one instance per effect, one serial chain ordered by `fx_pos_*` (`ModuleRack`); ROADMAP: "needs instance-indexed parameter ids" | partial | BUILD (part of the graph) | P4 |
| Layers / key-velocity split to different subgraphs (drum kits, splits) | P S | none (zones exist inside `osc.sample`) | missing | BUILD | P4 |
| Self-describing parameters (range, unit, display text, groups, flags) | all | `ParamDef` (id, range, skew, unit, choices, smoothing, page, `modulatable`, `voiceMod`) and `ParameterMap` JSON export with host extras | partial (no display-text / parse functions, no groups beyond pages, no per-instance metadata) | BUILD | P3 |
| Module SDK for third parties, stable C ABI | — | `ModuleDescriptor` (C++ only, `std::function` factories), registration only in `Modules.cpp`; the C API covers hosting, not modules | missing | CLAP's plugin ABI (MIT) is a good model for versioned C structs and extension ids. BUILD (design inspired by CLAP; no dependency) | P5 |
| GUI-agnostic read-back (meters, scopes, spectra, mod values, views) | all | `MasterAnalyzer`, `OutputMeter`, `modulatedValue`, `voiceModulatedValue`, `clipPositions`, `sampleView`, `cloudView`, `soundingNotes`: good but ad hoc, one function per view, not generic, not in the C API in full | partial | BUILD (generalise) | P6 |
| Product presets / layouts / content as data packs | — | `PresetLibrary.h`, the content-addressed asset store; layouts were C++ in the core | partial | BUILD | P4 |
| No product assumptions in the core | — | a product layout was the default; a wire codec, a key-bus file layout and product strings lived in the core | missing | BUILD (move to adapters) | P2 |

### 1.2 Sound sources

| Feature family | Ref | Engine today | Status | Prior art / adopt vs build | Pkg |
| --- | --- | --- | --- | --- | --- |
| Wavetable osc: mip-mapped, morph, unison | P S PP | `Voice::renderOscillator`: 27 band-limited levels, 6-point Lagrange, per-sample morph | have | KEEP: already on a par (refbench 39/39) | — |
| Warps / frame modes | P S | 15 `WarpMode`s; no spectral warps, no phase-distortion family, no second warp slot | partial | Vital's spectral warps are a public idea (code is GPL). BUILD | P13 |
| Spectral frame interpolation | S P | linear crossfade between frames | missing | BUILD on `Fft.h` | P13 |
| VA oscillator (saw / pulse / PWM / tri / sine, sync, analog shapes) | P D PP | no dedicated module; analog shapes are wavetables; PWM warp at -50 dB inharmonic energy; sub osc PolyBLEP | partial | Plaits VA / stmlib (MIT, 2-point polyBLEP: weaker floor); DaisySP (MIT, polyBLEP). Surge's Classic (BLIT) and Modern (DPW) oscillators are publicly documented ideas. BUILD on our minBLEP / minBLAMP (`MinBlep.h`) | P9 |
| Classic oscillator characters (saw-core / tri-core VCO, DCO) | D | none | missing | BUILD | P9 |
| Drift / per-voice tolerance | D P | one chunk-rate random walk per voice, pitch only (`voice_drift`) | partial | BUILD | P9, P11 |
| Cross-mod: FM, ring, sync | all | `oscN_xmod_*` with minBLEP sync; carrier must be a wavetable; no soft sync, no through-zero FM on analog shapes | partial | BUILD | P9 |
| Sample playback | P S PP | `osc.sample` | have | KEEP | — |
| Multisample / SFZ | S P | `Zones.h` (SFZ-lite) | have (no time-stretch) | sfizz (BSD-2): ADOPT its parser only if full SFZ becomes a goal; do not grow SFZ-lite ourselves | backlog |
| Granular | S P | `osc.sample` granular mode | have | KEEP | — |
| Spectral oscillator / time-stretch | S | none | missing | Signalsmith Stretch (MIT): ADOPT | P13 |
| Harmonic / additive | P | `osc.cloud`, 8 octave drawbars | partial | Plaits harmonic engine (MIT): optional port. Light BUILD on the cloud renderer | P13 |
| Modal / physical | P | none | missing | Mutable Rings / Elements (MIT): ADOPT. STK (permissive) as a fallback | P13 |
| Noise | all | `osc.noise`, 7 colours | have | KEEP | — |
| Distortion / filter generators in the voice path | PP | filter drive only | partial | BUILD as graph nodes | P4, P10 |

### 1.3 Filters

| Feature family | Ref | Engine today | Status | Prior art / adopt vs build | Pkg |
| --- | --- | --- | --- | --- | --- |
| Multimode ZDF SVF 12 / 24 | P S PP | `FilterChannel::svf`, `PeakTable` | have | Cytomic papers (free). KEEP | — |
| Filter model extension point | all | closed 17-entry `FilterModel` enum (Filters.h) | missing | BUILD | P7 |
| Transistor ladder, per-stage nonlinear | D P | `LadderLowpass`: linear stages, one input clip, fixed `ladderComp`; 1x | partial | Surge vintage ladder (public description: per-stage nonlinearity, oversampled); DaisySP ladder (MIT, below target). Papers: Huovilainen 2004, D'Angelo & Välimäki 2013, Zavalishin. BUILD | P7 |
| OTA 4-pole cascade (Jupiter / Juno-style) | D P | none | missing | BUILD | P7 |
| Diode ladder (TB-303-style) | D | `DiodeLowpass`: ladder plus clips; no stage coupling | partial | Papers: Stinchcombe, Zavalishin. BUILD | P7 |
| Sallen-Key with feedback clipping (MS-20 / K35-style) | D P | none | missing | Surge's K35 is a public idea; Stinchcombe / Pirkle notes. BUILD | P8 |
| Nonlinear OTA SVF (SEM-style morph) | D P | `DirtyLowpass` (LP only) | partial | BUILD | P8 |
| 4-pole OTA with partial compensation (Prophet-style) | D | none | missing | BUILD | P8 |
| Passive HPF stage | D | via filter 2 only | partial | BUILD | P8 |
| Comb, formant, phaser | P S | have | have | KEEP | — |
| Audio-rate cutoff / filter FM | S PP P | coefficients per 32-sample chunk | missing | BUILD | P12 |
| Oversampled nonlinear filters | D S | ladder internals at 1x | missing | KEEP `Oversampler` (HIIR (WTFPL) and r8brain (MIT) are no better) | P7, P8 |
| Accuracy tiers | D | quality honoured by drive and warps only | partial | BUILD the solver tiers | P7, P8 |

### 1.4 Envelopes and analog behaviour

| Feature | Ref | Engine today | Status | Prior art / adopt vs build | Pkg |
| --- | --- | --- | --- | --- | --- |
| AHDSR with curves | all | `Adsr` (Dsp.h) | have | KEEP | — |
| Analog RC envelope, overshoot target | D | none | missing | BUILD | P11 |
| Retrigger modes (from level / reset / legato / ADR) | D P | attack from the current level only | partial | BUILD | P11 |
| Envelope time scaling and tolerance | D P | matrix only | partial | BUILD | P11 |
| MSEG / function generators | P S PP | clips with sustain loop, per-voice trigger (`Clips.h`) | have (no UI) | KEEP | — |
| Voice-to-voice variance (voice cards) | D | none beyond pitch drift | missing | BUILD | P11 |
| Noise floor / hum (optional) | D | none | missing | BUILD | P11 |

### 1.5 Modulation

| Feature | Ref | Engine today | Status | Prior art / adopt vs build | Pkg |
| --- | --- | --- | --- | --- | --- |
| LFOs | all | 4 per voice, 12 shapes plus a custom table | have (fixed count) | count becomes data in P4 | P4 |
| LFO tables (position morph) | PP S | one custom table | partial | BUILD | P12 |
| Random: S&H, smooth, per note | all | have | have | KEEP | — |
| Turing / binary / chaos | P S | none | missing | Mutable Marbles (MIT) for ideas. BUILD | P12 |
| Combinators, remap, lag, quantise | P PP | per-route curve only | missing | BUILD | P12 |
| Note gate, follower, pitch tracker | PP | none | missing | BUILD | P12 |
| Modulation of modulation | S P PP | none | missing | BUILD | P12 |
| Matrix plus drag-to-assign, per-voice targets | all | 16 slots + 128 routes, `voiceMod` targets | have | KEEP | — |
| Audio-rate modulation | PP S | oscillator xmod only | missing | BUILD (fixed audio-rate target set; see the Vital lesson in 1.8) | P12 |
| Macros | all | 6 / 8 (`ProductLayout::macros`) | have | count becomes data in P4 | P4 |
| Microtuning, MPE | P S | `Tuning.h`, MPE sources | have | Surge tuning-library (MIT) as the oracle; ADOPT on any disagreement. MTS-ESP client: check its licence before offering it | P1 |

### 1.6 Effects

| Feature | Ref | Engine today | Status | Prior art / adopt vs build | Pkg |
| --- | --- | --- | --- | --- | --- |
| Distortion family | all | `fx.drive`: 9 shapers + tape + bitcrush, 4x | have | KEEP | — |
| Physical saturation (triode, tape hysteresis, transformer) | D P | static curves | partial | Airwindows (MIT): optional ADOPT of 1-2 algorithms. Papers: Holters & Zölzer, Koren. BUILD the core | P10 |
| Modulation FX, delay, reverb, dynamics, EQ, vocoder, etc. | all | have (21 modules) | have | KEEP | — |
| BBD ensemble, tape echo, spring reverb | D | none | missing | chowdsp BBD: ADOPT only if confirmed BSD. Spring from papers | backlog |
| Multiple instances, parallel lanes, sends | S PP P | none | missing | BUILD | P4 |

### 1.7 Infrastructure (adopt vs keep)

| Component | Today | Best permissive option | Recommendation |
| --- | --- | --- | --- |
| FFT | `Fft.h`: Accelerate on Apple, our own portable FFT | pffft (BSD-like), KissFFT (BSD-3) | ADOPT pffft as the portable backend if it is at least 1.5x faster on Linux / Windows (P13) |
| JSON | `Json.h` | nlohmann/json (MIT) | KEEP. Ours works, has no exceptions and builds quickly; replacing it buys nothing |
| Tuning | `Tuning.h` | Surge tuning-library (MIT) | Use as the oracle (P1); ADOPT on any disagreement |
| Oversampling | `Oversampler.h` | HIIR (WTFPL), r8brain (MIT) | KEEP (measured 65-122 dB rejection) |
| WAV / MIDI | `AudioFile.h`, `MidiFile.h` | dr_wav (public domain / MIT-0) | KEEP; adopt dr_wav only if more formats are needed |
| Plugin-style ABI | none | CLAP (MIT) | Use as a design model for P5. Hosts wrap Crucible in CLAP / VST3 / AU themselves |
| Tests | `TestFramework.h` | doctest / Catch2 | KEEP |

### 1.8 Phase Plant mapping

| Phase Plant feature | Status | Pkg |
| --- | --- | --- |
| Analog generator | partial | P9 |
| Wavetable / sampler / noise generators | have | — |
| Distortion and filter generators in the path, per-lane routing, groups | partial | P4 (graph), P7 / P8 / P10 (nodes) |
| LFO tables, envelopes, random, curve, remap, combinators, note gate, follower | partial | P12 |
| Audio-rate modulation | missing | P12 |
| Snapins: many reorderable effects, multiple instances, parallel lanes | partial | P4 |
| Macros | have (count becomes data) | P4 |

---

## 1.9 Lessons from Surge XT and Vital (clean-room: public docs, talks, issue trackers and design discussions only)

These are design lessons drawn from public material. No code, tables or coefficients are taken.

### What Surge does well, and what Crucible adopts as design ideas

| Area | What Surge does (public) | Crucible decision |
| --- | --- | --- |
| Filter zoo breadth | Dozens of filter types, each with documented per-model resonance and self-oscillation behaviour; split out as a reusable library | Same breadth as the goal: filters as registered models (P7), every model documented with its measured self-oscillation and bass-loss curves (3.1) |
| SIMD across voices | Filters and waveshapers process four voices at once in one SIMD register ("quad" filter chains) | Adopt the idea: analog filter and shaper nodes expose `processQuad` over four voices (`simd::Float4`), grouping voices with the same model. Our unison kernels already do this for oscillators (`runGroup4`) |
| Portable SIMD | The SSE-centric code had to be made portable for Apple Silicon through a translation layer | Already avoided: `Simd.h` wraps NEON / SSE / AVX2 / scalar, and CI tests all of them |
| Oversampling | The whole synth runs internally at 2x and is decimated at the end: simple, uniform, and costly for cheap paths | Per-module policy instead (2.7): oscillators with BLEPs run at 1x, nonlinear nodes at 2x / 4x by quality tier |
| Waveshaper set | A large, separately packaged waveshaper set, many with ADAA | Saturation library (P10) with ADAA where an antiderivative exists, otherwise oversampling, plus physical models |
| Modulation architecture | A clear split between voice modulators (per voice) and scene / global modulators; modulation visible on every control; formula modulators | Already have per-voice vs global evaluation. P4 makes the scope (voice / layer / global) an explicit property of every modulator node in the product definition. Formula modulators are SKIP for now (a scripting runtime in the audio thread is a big real-time and security surface) |
| FX ordering | Fixed FX slot topology (insert per scene, sends, global) that grew in later versions | Crucible's FX graph is data (P4): products choose slots, lanes and sends, and the topology does not need to be extended in code later |
| Tuning / MPE | A standalone MIT tuning library; MPE and MTS-ESP support | Use the MIT tuning library as the oracle or replacement; MPE exists; MTS-ESP optional after a licence check |
| Testing and CI | Headless test target, tests that load every factory patch, CI on all three OSes, nightly builds | P1 / P2: factory-patch load-and-render test for every example product, three-OS CI, nightly artifacts (workflow only; nothing is pushed from here) |
| Community process | Contributing guide, enforced clang-format, labelled first issues, active chat, stable releases plus nightlies | P2: CONTRIBUTING, clang-format checked in CI, issue templates, CHANGELOG, a documented release cadence (proposal: minor release per quarter, nightly from main) |
| Library extraction | Over several years, reusable DSP was pulled out of the monolith into separate libraries | Crucible starts modular: modules behind the SDK (P5), no singletons (P3) |
| Adopting permissive code | Surge itself adopted Mutable Instruments (MIT) oscillators and Airwindows (MIT) effects | Validates our ADOPT choices (Mutable resonators P13, optional Airwindows P10) |

### Common pitfalls of long-lived synth code bases, and how Crucible avoids them

| Pitfall | Crucible avoidance |
| --- | --- |
| **Legacy patch compatibility.** Every patch-format revision adds conditional loading paths that are never removed, and old sounds must be kept exact | One migration step per format version in `Patch.cpp` (already in place); module versions with per-module migrations (P3); bit-exact golden and hash tests in CI; legacy behaviour in isolated "classic" nodes (P4), so new code never needs `if (revision < N)` branches inside DSP |
| **Monolithic code and global state.** Central storage and synth objects reach everywhere, so features touch many files | Instance-scoped registry and parameter model (P3), module SDK boundaries (P5), the voice as a compiled graph of nodes (P4) instead of a growing `Voice.cpp` |
| **UI coupled to the engine.** Changing GUI framework meant a very large rewrite | Crucible ships no GUI: metadata (P3) and read-back APIs (P6) are toolkit-free, and the example products include a generated GUI and a hand-built one, to prove the separation |
| **Fixed architecture.** Fixed counts (scenes, oscillators, LFOs, FX slots) are hard to change once patches depend on them | Counts live in product definitions (P4); parameter ids are generated per instance with stable naming rules |
| **Parameters streamed by index.** Index-based state breaks on reordering | Ids are the contract (already the case); indices are per product and never stored in patches |
| **Bus factor.** A project that depends on one maintainer can stall | Contributor docs, module SDK (others extend without core changes), CI that a newcomer can trust, maintainer rotation noted in CONTRIBUTING |
| **Code licence vs brand / content.** Unclear trademark and content terms lead to confusion for forks | Clear from the start: code MIT; factory content (example wavetables and presets) under CC0 or MIT; the name policy stated in README (forks rename) |
| **Audio-rate modulation everywhere costs CPU.** Smooth, but every destination costs per-sample work | A fixed audio-rate target set plus per-chunk ramps elsewhere (2.10) |
| **Build system churn.** Moving between build systems cost the community time | CMake only, with a `find_package` consumer test (P2) |

---

## 2. Design

### 2.1 Product definitions and the graph in data (P4)

* **Product definition** (`crucible-product` JSON, versioned) contains:
  * `modules`: named instances (`{ "id": "osc1", "type": "osc.analog" }`) of any count;
  * `voice`: the voice graph as nodes and edges (audio and modulation edges kept separate), plus lanes (named
    serial chains that sum or feed each other) and `layers` (note / velocity / round-robin ranges that select a
    subgraph);
  * `modulators`: LFOs, envelopes, MSEGs and the modulator bank, each with a scope of voice / layer / global;
  * `fx`: the FX graph (instances, lanes, sends, master);
  * `macros`, `pages` / `sections` (for hardware and GUIs), `defaultPatch` and preset banks (data packs).
* **Parameter ids are generated** as `<instance>_<param>`. A per-product `idMap` can pin legacy ids. The existing
  product definition maps its instances onto today's flat ids (`osc1_pos`, `filter_cutoff`, `fx_drive_amount`) in today's
  order, so its parameter indices, generic destination numbers, patches and host automation are unchanged.
  Instance k >= 2 of an effect follows the rule `fx{k}_...`.
* **Compilation.** On the control thread, the graph is checked: types exist, no audio cycles except through
  explicit one-sample feedback nodes, and the counts fit the limits. It is then topologically sorted into a
  per-voice schedule, buffers are preallocated, and the result is published through `RtShared`.
  * Runtime reconfiguration means a new compiled graph. Voices fade out over 10 ms and the new graph starts.
    Patches are re-applied by id.
  * No allocation or locking happens on the audio thread.
* **Classic voice node (bit-exact).** Today's whole `Voice` (wavetable slots with warps and unison, the module
  oscillators, the filter slots, the four envelopes, the four LFOs, the matrix) is wrapped as one composite node,
  `classic.voice`. The existing products use it, so their sound is byte-identical with no DSP rewrite. New
  products compose fine-grained nodes instead:
  * generator nodes: wrap `OscillatorInstance`, the wavetable oscillator extracted as a node, and `osc.analog`;
  * filter nodes: legacy SVF models and `AnalogFilter` models;
  * shaper, mixer, VCA, envelope and LFO nodes.
* **Node interface** (C++, mirrored in the C ABI of P5):
  * `prepare(ctx)`, `reset`, `noteOn` / `noteOff`;
  * `process(NodeIO&, n)`, where audio is in and out as mono or stereo buffers, modulation inputs arrive as
    per-chunk values or per-sample buffers, and parameters come as local indices;
  * optional `processQuad` for four voices in SIMD;
  * `latency()`.
* **FX graph.** `ModuleRack` becomes the FX graph executor: any number of instances, lanes in series or parallel,
  sends. With the existing product definition it reproduces today's position-ordered chain exactly.
* **Other product shapes:**
  * **FX product**: no voices. Host audio input through `process_aux` (it exists for the vocoder) feeds the FX
    graph.
  * **Drum product**: layers per note, each with its own small graph and choke groups.
* **Limits** (configurable): 32 nodes per voice, 64 FX instances, 16 layers, 64 modulators.

### 2.2 Instance-scoped registry and self-describing metadata (P3)

* **No singletons.** `ModuleRegistry` and `ParameterMap` become objects owned by an engine instance (built from its
  product definition). `ModuleRegistry::instance()` stays as a deprecated shim for the default product.
* **Metadata per parameter** (added to `ParamDef`, all optional with defaults):
  * `displayName` / `shortName` (for 6-character hardware LCDs);
  * `group` (a tree path such as `osc1/unison`);
  * `toText(value)` / `fromText(text)` function ids, from a standard set: Hz, ms / s, dB, %, semitones, cents,
    note name, ratio, choice;
  * `steps`, `defaultValue`, `polarity`, `modulatable`, `voiceMod`;
  * `automatable` (host);
  * `uiHint` (knob / slider / switch / menu / xy / drawbar);
  * `introducedIn` (module version).
* **Metadata per module**: `id`, `name`, `kind`, `category`, `version`, `cpu`, `description`, `inputs` / `outputs`
  (port names and types: audio mono / stereo, mod, trigger), `views` offered (P6), `capabilities`.
* **Introspection API** (C++ and C):
  * enumerate modules, instances, parameters, ports and views;
  * `describeJSON()` for the whole product;
  * `valueToText` / `textToValue`.
  * The existing JSON export becomes a view of this.
* **Module versions and migrations.** Per-module migration functions (`fromVersion -> toVersion`, id renames,
  value remaps) run when a patch made with an older module version loads. The patch records each module's version
  (an additive field).

### 2.3 Module SDK and stable C ABI (P5)

* **`crucible_module.h`**: a C ABI modelled on CLAP's conventions (versioned structs, extension ids, host callbacks).
  * `crucible_module_descriptor`: abi version, id, name, kind, version, parameter table, ports, function pointers
    `create`, `destroy`, `prepare`, `reset`, `process`, `note_on`, `note_off`, optional `process_quad`, optional
    `get_view`.
  * Strings are UTF-8, structs append-only, and every struct starts with `size` and `abi_version`.
* **Registration.** `crucible_register_module(engine_or_registry, &desc)` before a product loads. A C++ convenience
  layer (`crucible::sdk::Module<T>`) generates the descriptor.
* **Shared-library loading** is optional, behind a host flag. `crucible_load_module_library(path)` looks up
  `crucible_get_modules`. The engine never scans directories by itself; the host decides what is trusted.
* **Built-in modules** register through the same SDK path. This dogfoods it: every built-in is an SDK module.
* **Real-time contract** (documented and tested by a harness that third parties can run):
  * no allocation, locking or blocking in `process`;
  * bounded output at parameter extremes;
  * denormal-safe;
  * deterministic per seed.
* **`crucible-module-check` CLI**: loads a module and runs the harness: allocation counter, extremes sweep, NaN /
  infinity, determinism, CPU.

### 2.4 GUI-agnostic read-back API (P6)

* **Generic views.** A module or engine view is identified by `{instance, viewId}`, with a typed payload:
  * scalar: meter, gain reduction, playhead;
  * vector: scope, spectrum, envelope or LFO shape, wavetable frame;
  * points: grains, cloud points;
  * mask: sounding notes.
* **Publication.** The audio thread writes into lock-free triple buffers (or atomics for scalars) at most once per
  render call. A reader on any thread calls `crucible_view_read(engine, view, buffer, capacity, &sequence)` and
  gets a consistent snapshot with a sequence number.
* **Subscriptions** are made on the control thread with a rate limit. Views nobody subscribes to cost nothing.
* **Existing accessors** (`meter`, `MasterAnalyzer`, `modulatedValue`, `voiceModulatedValue`, `clipPositions`,
  `sampleView`, `cloudView`, `soundingNotes`) are re-expressed as views. The old functions stay as wrappers.
* **Per-voice modulated values**: for the newest voice, a chosen voice, or a min / max band across voices (for
  modulation rings).

### 2.5 VA oscillator (`osc.analog`, P9)

* **Waveforms**: saw, pulse with PWM, triangle, sine, saw + pulse mix, and a continuous shape morph.
* **Band-limiting**:
  * double-precision phase;
  * 32-zero-crossing minBLEP for steps and minBLAMP for corners, at exact sub-sample positions;
  * PWM uses the instantaneous pulse-width position;
  * above fs/4, a half-octave crossfade into the mip-mapped table of the same shape;
  * a 5 Hz DC blocker on the pulse.
* **Characters**, each derived from circuit principles:
  * `Ideal` (default);
  * `Saw-core VCO`: ramp curvature, a finite reset time (BLAMP-corrected), a comparator pulse with a width offset,
    a soft-shaped triangle;
  * `Triangle-core VCO`: a switched saw, a tanh sine;
  * `DCO`: exact pitch, saw level that falls with pitch unless compensated, a divider sub.
* **Sync / cross-mod**: hard and soft sync (via `Wraps`), through-zero linear FM, exponential FM, ring, PWM from
  another slot.
* **Drift and tolerance** (default 0):
  * Ornstein-Uhlenbeck drift per unison voice (tau 0.5-5 s, std `drift * 6 cents`);
  * voice-card tolerance on pitch, pulse width, level and curvature;
  * optional warm-up drift.
* **CPU**: at most 0.05 % of one core per voice-oscillator for a saw, at most 0.12 % with PWM + sync
  (48 kHz, Apple M-class core).

### 2.6 Nonlinear ZDF filters (P7, P8)

* **Interface.** `AnalogFilter` (`include/crucible/filters/FilterModel.h`) has `prepare(rate, os)`, `reset`,
  `setControls(cutoff, res, drive, character, tolerance)`, `process(io, n, cutoffOctMod)` and `processQuad`.
* **Filter types and the legacy enum.**
  * Legacy models 0-16 are untouched.
  * New models are appended to the filter type list (17+), so legacy `FilterChannel` slots can use them too.
  * Each new model is also a graph node type.
* **Solver tiers**:
  * Draft: linearised at the previous state;
  * Standard: plus one Newton step;
  * High: Newton to 1e-6, at most 4 iterations.
* **Per-model behaviour**:

| Model | Topology | Behaviour to reproduce |
| --- | --- | --- |
| `transistor_ladder` | 4 TPT stages `y' = g(tanh x - tanh y)`, feedback `-k tanh(y4)`, k from 0 to 4.2 | self-oscillation at k of about 4; DC gain `1/(1+k)` (bass loss); `character` blends in compensation |
| `ota_cascade` | 4 OTA integrators with tanh inputs, resonance VCA, optional 4-step HPF ahead | less bass loss, earlier resonance, clean self-oscillation |
| `diode_ladder` | 4 coupled stages (tridiagonal, first capacitor doubled), diode curves, closed-form 4x4 solve | about 18 dB/oct, lower peak, squelch with level |
| `sallen_key_35` LP / HP | 2-pole Sallen-Key, asymmetric diode clipper in the feedback | clipping in the loop; even + odd harmonics |
| `svf_ota` | 2-pole SVF with tanh integrator inputs, LP-notch-HP morph | no self-oscillation, no bass loss |
| `ota_4pole_comp` | 4 OTA stages, resonance VCA, partial compensation | bass loss between the ladder and the OTA cascade |
| `passive_hp` | 1 / 2-pole passive HPF | exact slopes |

* **Cutoff accuracy.** A per-model pitch-correction table built at load: within +-5 cents from 40 Hz to 8 kHz in
  High, +-15 cents in Draft.
* **Tolerance**: cutoff +-4 %, feedback +-3 %, capacitor mismatch +-1 % per stage.
* **Self-oscillation start**: an optional seeded noise injection at -120 dB, off by default.
* **CPU** (per voice, mono, Standard = 2x): ladders at most 0.12 %, diode ladder at most 0.18 %, 2-pole models at
  most 0.08 %. `processQuad` should be about 3x faster than four scalar calls.

### 2.7 Oversampling policy

| Module | Draft | Standard | High |
| --- | --- | --- | --- |
| `osc.analog` | 1x | 1x | 1x |
| Nonlinear filters | 1x, linearised | 2x + Newton step | 4x + Newton |
| Legacy filters | 1x | 1x | 1x |
| Tube / transformer | ADAA 1x | ADAA 2x | 4x |
| Tape | 2x | 2x | 4x |
| Shaper node | 2x | 4x | 8x |
| Modal / harmonic / spectral | 1x | 1x | 1x |

Rule: through its default settings, no module may alias above -90 dB (Standard) or -100 dB (High) below 20 kHz.

### 2.8 Saturation library (P10)

* **Library**: `include/crucible/Saturation.h`, scalar and `Float4`.
* **Triode**: a Koren-fitted closed form with bias (H2), grid-current clipping, and a level-dependent bias shift
  for sag.
* **Tape**: Jiles-Atherton hysteresis (RK2, oversampled), emphasis EQ, head bump, HF loss, wow and flutter.
* **Transformer**: flux saturation (integrate, then tanh, then differentiate), so H3 rises at low frequency.
* **Diode / rail clippers.**
* **Where it is used**: `fx.drive` (modes appended), the shaper node, and the filter drive stages.

### 2.9 Analog envelopes and voice cards (P11)

* **Envelope mode** per envelope: `Digital` (default, bit-identical) or `Analog RC`.
  * The attack charges toward `1+o` and reaches the peak at the attack time.
  * Decay and release are exponential.
  * Sustain moves through the RC.
* **Retrigger modes**: from level, reset (0.5 ms de-click), legato, ADR.
* **Velocity / key time scaling.**
* **Voice cards** (`voice.analog`) come from the seed and the voice index: tolerances for oscillators, filters and
  envelopes. A null pointer means legacy behaviour.
* **Optional noise floor and hum** (-120 to -60 dBFS, seeded, off by default).

### 2.10 Modulator bank and audio-rate modulation (P12)

* **Modulator bank.**
  * Sources at `kSourceModulator = 4000 + n`.
  * Types: Combine (add, multiply, min, max, crossfade, difference), Remap (16-point curve), Lag (rise / fall),
    Quantise, S&H, Turing, Binary, Chaos (logistic, Lorenz), Note gate, Follower, Pitch tracker, LFO table.
  * Scope: voice / layer / global.
  * Evaluated in index order, inputs only from lower indices, so no cycles.
  * Route amounts are targets (`route:<n>:amount`), which gives modulation of modulation.
* **Audio-rate modulation.**
  * A per-voice audio bus of up to 8 mono buffers: oscillator outputs, per-sample envelopes, audio-rate LFOs
    (`lfoN_audio`, BLEP band-limited).
  * Audio-rate targets: pitch, FM / PM amount, filter cutoff (per-sample buffer into `AnalogFilter`), amp, pulse
    width, wavetable position.
  * Other targets get the chunk mean.
  * In the graph, audio-rate modulation edges are explicit.

### 2.11 Adopted and new engines (P13)

1. **Modal / physical: ADOPT.** Port the Mutable Rings / Elements resonators, strings and exciters (MIT) under
   `third_party/mutable/` as `osc.modal`, adding a band-limit check and voice-card tolerance.
2. **Harmonic layer: light BUILD** on the cloud renderer: count, odd / even, stretch, tilt, formants, partial decay.
   Optionally port Plaits' harmonic engine (MIT).
3. **Spectral.**
   * Time-stretch and pitch: Signalsmith Stretch (MIT), ADOPT.
   * Spectral warps and spectral frame interpolation: BUILD on `Fft.h`.
   * pffft is adopted if it benchmarks at least 1.5x faster.

### 2.12 CPU targets (48 kHz, 128-sample blocks, Apple M-class core)

| Configuration | Target |
| --- | --- |
| 16 voices: 2 `osc.analog`, ladder at 2x, analog envelopes, voice cards | at most 6 % |
| The same at High | at most 12 % |
| 16 x 8 unison through `classic.voice` in the graph | unchanged 4 % (graph overhead below 2 % relative) |
| Graph of 32 fine-grained nodes per voice, 16 voices | at most 10 % |
| Each extra FX instance | at most 1 % |

The bench fails CI at the target + 20 %.

### 2.13 Compatibility rules

* New choices go at the end of their lists.
* New parameter modules register last, or come through product definitions with pinned ids.
* Defaults select the legacy code path through an early branch.
* Module versions come with migrations.
* The patch format stays at 2 for additive fields. The product definition has its own format version.
* Patches store ids, never indices.

---

## 3. Verification without listening

Nothing is judged by ear; every claim is a number.

### 3.1 Analytic tests

P1 provides a measurement header:

* Blackman-Harris spectrum, harmonics and inharmonic energy below 20 kHz;
* frequency estimation;
* THD per harmonic;
* sweep impulse responses;
* statistics: std, autocorrelation time, PSD slope.

| Property | Test | Pass rule |
| --- | --- | --- |
| VA aliasing | saw, 10 % pulse, PWM sweep, hard sync ratio 2.37, through-zero FM; C2-C8 at 44.1 / 48 / 96 kHz | <= -100 dB re fundamental (saw / pulse / tri), <= -90 dB (PWM / sync / FM), below 20 kHz |
| VA shape | harmonics | `Ideal` saw follows 1/n within 0.1 dB to 10 kHz; triangle odd 1/n^2; DC < -100 dB |
| Self-oscillation tracking | max resonance, noise seed on, 40 Hz-8 kHz | error <= 5 cents (High), <= 15 cents (Draft); amplitude varies by < 3 dB |
| Self-oscillation threshold | resonance sweep | ladder onset k in 4.0-4.3; others inside their documented band; SVF never self-oscillates |
| Bass loss | -40 dBFS sweep, resonance 0-0.95 | ladder within 0.5 dB of `-20 log10(1+k)`; compensated within 0.5 dB of 0 |
| Slopes | 2-4 octaves above cutoff | 24 +-1.5 (ladder / OTA), 18 +-3 (diode), 12 +-1 (2-pole) dB/oct |
| Level dependence | -40 / -12 / 0 / +6 dBFS | monotonic peak compression for nonlinear models; legacy unchanged |
| Saturator spectra | 1 kHz, stepped drive | symmetric: even < -90 dB; triode: H2 > H3 at low drive; transformer: H3 at 50 Hz >= H3 at 1 kHz + 10 dB; tape: loop area > 0; THD monotonic |
| Nonlinear aliasing | 5 kHz, full drive | <= -90 dB (Standard), <= -100 dB (High) |
| RC envelope | exponential fit | residual < 0.5 %; peak time within 2 %; retrigger continuous |
| Drift | 200 notes x 10 s | std within 10 %, tau within 20 %, PSD slope -2 +-0.3, identical per seed |
| Tolerance | 64 voice cards | bounded, mean about 0, identical per seed and voice |
| Modal (adopted) | mode frequencies, T60 | within 1 cent / 5 %; nothing above 0.45 fs |
| Tuning oracle | the tuning-library test set | within 0.01 cent |
| Graph equivalence | the existing product as data vs the pre-P4 engine | bit-identical hashes and golden renders |
| Graph algebra | parallel lanes vs separate renders | equal within 1e-6 for linear nodes |
| SDK contract | `crucible-module-check` on every built-in | zero allocations, bounded, deterministic |
| Views | reader-thread stress (TSan) | no tearing (sequence check), no locks on the audio side |
| Bit-exactness, real time, CPU | existing suites, allocation counter, bench | unchanged / zero / target + 20 % |

### 3.2 Reference-bench black-box comparisons

* **New analysis test types**: `selfosc_track`, `res_bassloss`, `res_level_dependence`, `sat_harmonics`,
  `drift_stats`, `env_rc_fit`, `voice_variance`.
* **References.** Pigments and Serum 2 as today. Diva and Phase Plant only if the maintainers license them. Targets are
  bands around measured behaviour, never sample matches. Without a licensed reference, targets are `analytic`, from
  published circuit papers.
* **Licence boundary.** The JUCE-based bench host is not part of this repository. Crucible ships only
  `crucible-measure`, which renders the plan to WAV.

---

## 4. Work packages

**Rules for every package.**

* **Worker.** A worker in its own worktree, touching only the listed files plus the shared append-only files:
  * `Modules.cpp` / `Modules.h` (a line each), or the built-in module list after P5;
  * regenerated `PARAMETERS.md`;
  * a row in `ROADMAP.md`;
  * an entry in `CHANGELOG.md`;
  * `THIRD_PARTY.md`.
* **Merging.** The coordinator merges packages one at a time.
* **Acceptance for every package:**
  * all tests pass;
  * bit-exact hashes and golden renders unchanged;
  * zero allocations in render;
  * `tools/ci.sh` green;
  * CPU targets met;
  * the engine-side refbench renders still 39/39.
* **Code rules.** C++20, clang-format, SPDX header (after P2), no AI authorship in comments, adopted code only under
  `third_party/`.

### P1. Verification harness (after the in-flight `fix/engine-verify` merges)

* **Scope.**
  * `tests/AnalogMeasure.h` and `AnalogMeasureTests.cpp`.
  * Bench row skeletons (2.12) in `tests/Bench.cpp`.
  * Reference-bench analysis types (`suite.py`, `measure.py`, `dsp.py`, `targets.json` marked `pending`).
  * The tuning-library (MIT) oracle test, with the library vendored for tests.
  * Fixes for any failures the harness exposes.
* **Acceptance.**
  * Self-tests recover synthetic truth: THD within 0.1 dB, slope within 0.2 dB/oct, pitch within 0.1 cent, OU std
    within 5 %, clean-sine floor below -130 dB.
  * The tuning oracle passes, or the failing cases are recorded with an "adopt tuning-library" follow-up.
* **Step 1, alone.**

### P2. Crucible isolation, extraction and rename

* **Depends on:** P1. **Runs alone**, because it touches nearly every engine file. Every later package branches
  from it.
* **Approach.** Rename inside the engine, keeping every commit green; the engine then lives in a repository of its
  own with its history. Nothing is pushed.
* **Scope:**
  1. **Remove product coupling** from the core: product layouts, wire codecs, key-bus file layouts and product strings
     belong to host applications. The core gets a neutral `crucible.default` layout and keeps an abstract `KeyBus`
     plus the in-process `local()`; the reference-bench dependency of `refbench.sh` is carried by the engine.
  2. **Rename** to `crucible`:
     * namespace and include directory, `crucible.h`, `crucible_*`, `CRUCIBLE_*` (including the
       `CRUCIBLE_UPDATE_GOLDEN` and `CRUCIBLE_UPDATE_BITEXACT` environment variables);
     * targets `crucible_engine` (alias `crucible::engine`), `crucible-render`, `crucible-measure`;
     * patch `format` written as `crucible-patch`.
  3. **Install / export.** `install(EXPORT CrucibleEngineTargets)`, `CrucibleEngineConfig.cmake` and
     `ConfigVersion.cmake`, installed headers, and a `find_package(CrucibleEngine)` consumer test.
  4. **Single-sourced version.** `project(CrucibleEngine VERSION 0.1.0)` drives a generated `crucible/Version.h`,
     `crucible_version()` and the CLI `--version`. `CRUCIBLE_API_VERSION` stays the separate ABI counter.
  5. **Licence.** `LICENSE` (MIT), an SPDX header on every file (checked by a CI script), `THIRD_PARTY.md`, and the
     content licence for example presets and tables (CC0).
  6. **Community files.**
     * `README.md`: the niche, the platform principle, build, embed, C API.
     * `CONTRIBUTING.md`: real-time rules, bit-exact policy, clean-room rule (Surge / Vital: docs only, never
       code), module guide, DCO sign-off, clang-format, release cadence.
     * `CHANGELOG.md`, `CODE_OF_CONDUCT.md`, issue templates.
     * `PROVENANCE.md`, the clean-room note: designs come from papers and public docs; GPL projects were studied
       through public material only; commercial products were only measured as black boxes; the list of papers.
  7. **CI.** `.github/workflows/ci.yml` (written, not run here):
     * macOS arm64, Ubuntu gcc + clang, Windows MSVC;
     * Release + ctest on every platform;
     * ASan / UBSan, TSan, the scalar fallback;
     * the `find_package` consumer, a clang-format check, the SPDX check;
     * nightly artifacts.
  8. **C API example.** `examples/c_api/render_c.c` (create, set parameters, play a note, write a WAV), built in
     CI, plus the renamed `examples/render_wav.cpp`.
  9. **Docs** renamed.
* **Acceptance.**
  * Hashes and goldens identical before and after the rename.
  * Host applications build and pass through their adapters.
  * No name from the engine's pre-extraction history appears in the tree.
  * The repository builds offline from a clean clone, and its ctest passes.
  * The consumer builds, and the C example renders a non-silent WAV.
  * SPDX on every file; history preserved.
* **Step 2, alone.**

### P3. Instance-scoped registry, parameter model and self-describing metadata

* **Depends on:** P2.
* **Scope** (2.2):
  * `include/crucible/module/Module.h`, `ParameterMap.h` / `.cpp`, `src/modules/Registry.cpp`;
  * engine ownership in `Engine` / `EngineCore` / `SynthEngine`;
  * metadata fields and the standard text formatters (`src/ParamText.cpp`);
  * the introspection API in C++ and C (`crucible.h` additions);
  * per-module migrations in `Patch.cpp`;
  * a deprecated singleton shim;
  * tests `tests/IntrospectionTests.cpp`;
  * removal of the separate `ProductMain` executable (two products in one process).
* **Prior art.** BUILD (Surge lesson: global state).
* **Acceptance.**
  * Two engines with different products run in one process, and each matches its single-process render.
  * `describeJSON` round-trips.
  * `valueToText` / `textToValue` are exact for every parameter.
  * Bit-exact unchanged.
* **Wave A.**

### P4. Graph in data: product definitions, graph compiler, classic voice node, FX graph, layers

* **Depends on:** P3.
* **Scope** (2.1):
  * `include/crucible/graph/*.h`, `src/graph/*.cpp`: schema, validation, compiler, executor, node interface;
  * `classic.voice` wrapping today's `Voice` (no DSP changes);
  * node wrappers for `OscillatorInstance`, the legacy filters and the existing envelopes and LFOs;
  * the `ModuleRack` -> FX graph executor (instances, lanes, sends);
  * layers and choke groups;
  * runtime reconfiguration with a fade;
  * example product definitions in `products/` (with pinned ids);
  * `products/crucible-default.json`;
  * tests `tests/GraphTests.cpp`.
* **Prior art.** Phase Plant shows the target behaviour; VCV Rack (GPL) informs the public design only. BUILD.
* **Acceptance.**
  * The existing products as data are bit-identical to the pre-P4 engine (hashes, goldens, refbench).
  * A 3-instance drive product renders three independent drives.
  * Parallel lanes equal the separate renders within 1e-6.
  * An FX-only product processes host input.
  * A 4-layer drum test product triggers per note with chokes.
  * Reconfiguration is click-bounded (< -60 dB step) and allocation-free on the audio thread.
  * Graph overhead below 2 % on the reference patch.
* **Wave B.** Mostly alone. Only new-file DSP packages run in parallel.

### P5. Module SDK and stable C ABI

* **Depends on:** P4.
* **Scope** (2.3):
  * `include/crucible/crucible_module.h`, the `include/crucible/sdk/Module.h` C++ layer, `src/sdk/*.cpp`;
  * built-ins migrated to SDK registration (mechanical);
  * optional shared-library loading;
  * the `tools/crucible-module-check` CLI;
  * `examples/third_party_module/` (a C module and a C++ module, built as shared libraries);
  * tests `tests/SdkTests.cpp`.
* **Prior art.** CLAP (MIT) as the design model. BUILD.
* **Acceptance.**
  * An external C module loads at runtime, appears in introspection, renders inside a product graph, passes
    `crucible-module-check`, and survives an ABI-version mismatch with a clean error.
  * Every built-in passes the module check.
  * Bit-exact unchanged.
* **Wave C.**

### P6. GUI-agnostic read-back (view) API

* **Depends on:** P3 (it can run beside P5).
* **Scope** (2.4):
  * `include/crucible/Views.h`, `src/Views.cpp`;
  * existing accessors re-expressed as views, keeping wrappers;
  * C API `crucible_view_*`;
  * per-voice modulation bands;
  * tests `tests/ViewTests.cpp` (TSan stress).
* **Prior art.** BUILD.
* **Acceptance.**
  * Every existing accessor is reachable as a view, with identical values.
  * The reader stress test shows no tearing, and the TSan job is clean.
  * Unsubscribed views cost nothing in the bench.
  * Zero allocations.
* **Wave C.**

### P7. Analog filter interface and ladder family

* **Depends on:** P1 (it runs beside P4, because it touches only `Filters.*` and new files).
* **Scope** (2.6, rows 1-3):
  * `include/crucible/filters/FilterModel.h`;
  * `src/filters/{TransistorLadder,OtaCascade,DiodeLadder}.cpp`;
  * `src/filters/NonlinearSolve.h`;
  * appended entries in `Filters.cpp` / `Filters.h`, with every model preallocated per channel (no audio-thread
    allocation);
  * the `processQuad` path;
  * `tests/AnalogFilterLadderTests.cpp`.
  * Graph node registration is a one-line follow-up after P4.
* **Prior art.** Surge's public filter descriptions inform the design; DaisySP is below target; papers. BUILD.
* **Acceptance.**
  * The 3.1 rows for self-oscillation, threshold, bass loss, slopes, level dependence and nonlinear aliasing.
  * The CPU targets; `processQuad` at least 2.5x faster than four scalar calls.
  * 60 s random-modulation stability.
  * Legacy indices unchanged.
* **Wave B.**

### P8. Sallen-Key, OTA SVF, OTA 4-pole, passive HPF

* **Depends on:** P7.
* **Scope** (2.6, rows 4-7): `src/filters/{SallenKey35,SvfOta,Ota4Pole,PassiveHp}.cpp`, the `filterN_hp` and
  `filterN_character` parameters (module registered last), `tests/AnalogFilterSkTests.cpp`.
* **Prior art.** BUILD.
* **Acceptance.**
  * Same rows as P7.
  * Even and odd harmonics in Sallen-Key self-oscillation.
  * The SVF never self-oscillates.
  * HPF slopes within 0.5 dB.
* **Wave C.**

### P9. VA oscillator `osc.analog`

* **Depends on:** P2 (it uses the existing `OscillatorInstance` interface, so it runs beside P3).
* **Scope** (2.5): `src/modules/oscillator/AnalogOscillator.cpp`, `tests/AnalogOscillatorTests.cpp`.
* **Prior art.** Plaits / stmlib (MIT) can be borrowed from with attribution; our minBLEP beats permissive
  polyBLEPs. BUILD.
* **Acceptance.**
  * The VA aliasing and shape rows.
  * Sync / soft sync / through-zero FM band-limited.
  * The drift statistics.
  * Type indices unchanged.
  * The CPU targets.
* **Wave A.**

### P10. Saturation library, shaper node and drive modes

* **Depends on:** P2.
* **Scope** (2.8):
  * `include/crucible/Saturation.h`, `src/Saturation.cpp`;
  * modes appended in `src/modules/fx/Drive.cpp`;
  * `src/modules/voice/Shaper.cpp` (the per-voice shaper, usable as a node after P4);
  * `tests/SaturationTests.cpp`;
  * a `CHANGELOG.md` evaluation of 1-2 Airwindows (MIT) algorithms, vendored if they add a measurably distinct
    character.
* **Prior art.** Papers; Airwindows optional ADOPT. BUILD the core.
* **Acceptance.**
  * The saturator rows.
  * Existing drive modes bit-identical.
  * Tape stable for 60 s.
  * Unity small-signal gain within 0.2 dB.
  * At most 1 % per instance.
* **Wave A.**

### P11. Analog envelopes, voice cards, noise floor and hum

* **Depends on:** P4 (nodes), P9.
* **Scope** (2.9):
  * `include/crucible/AnalogEnvelope.h`;
  * `src/modules/voice/AnalogVoice.cpp`;
  * the `VoiceCard` in the graph context and in `classic.voice` (null = legacy);
  * envelope node types;
  * `tests/AnalogVoiceTests.cpp`.
* **Prior art.** BUILD.
* **Acceptance.**
  * The RC envelope, tolerance and noise rows.
  * `Digital` is the default and bit-identical.
  * Retrigger click below -83 dB.
* **Wave D.**

### P12. Modulator bank, modulation of modulation, audio-rate bus and filter FM

* **Depends on:** P4, P7.
* **Scope** (2.10):
  * `include/crucible/Modulators.h`, `src/Modulators.cpp`;
  * route amount targets in `Modulation.*`;
  * modulator nodes with scopes;
  * audio-rate modulation edges in the graph executor;
  * `lfoN_audio`;
  * `filterN_fm_*`;
  * tests `tests/ModulatorTests.cpp`, `tests/AudioRateModTests.cpp`.
* **Prior art.** Mutable Marbles (MIT) for ideas. BUILD.
* **Acceptance.**
  * Each modulator type matches its definition (Turing reproducible, lag within 2 %, chaos bounded, remap exact,
    note gate per voice).
  * Cycles rejected.
  * Ladder FM sidebands within 0.5 dB of an offline double-precision model.
  * Audio-rate PWM and FM alias <= -90 dB.
  * Bit-exact when no audio-rate edges exist.
* **Wave D.**

### P13. Adopted and new engines: modal, harmonic, spectral, FFT backend

* **Depends on:** P2 (it uses the `OscillatorInstance` interface; registered as nodes after P4).
* **Scope** (2.11):
  * `third_party/mutable/` plus `src/modules/oscillator/ModalOscillator.cpp`;
  * `HarmonicOscillator.cpp`;
  * `third_party/signalsmith-stretch/` plus `SpectralOscillator.cpp`;
  * `include/crucible/Spectral.h`, `src/Spectral.cpp` (warps, frame interpolation, opt-in `oscN_wt_interp`);
  * the pffft benchmark and conditional adoption behind `Fft.h`;
  * `tests/EnginesTests.cpp`.
* **Prior art.** ADOPT Mutable (MIT), Signalsmith (MIT) and possibly pffft (BSD-like). Light BUILD of the harmonic
  layer and the warps.
* **Acceptance.**
  * Modal: mode frequencies within 1 cent, T60 within 5 % at 44.1 / 48 / 96 kHz, nothing above 0.45 fs, bounded
    for 60 s, at most 0.3 % per voice, licence notices present.
  * Harmonic partial law within 0.1 dB.
  * Time-stretch from 0.5x to 2x within 2 cents.
  * Spectral interpolation sweeps the frequency (not a crossfade).
  * No analysis on the audio thread.
  * FFT backend results equal within 1e-6.
  * Legacy paths bit-identical.
* **Wave B** (new files only; beside P4 and P7).

### Schedule (at most three in parallel)

| Step | Packages | Why |
| --- | --- | --- |
| 0 | `fix/engine-verify` (in flight) | the verification fixes land first |
| 1 | P1 alone | the measurement foundation |
| 2 | P2 alone | rename and isolation before any feature work |
| A | P3, P9, P10 | the platform core starts; VA and saturation use existing interfaces and new files |
| B | P4, P7, P13 | the graph (core files) beside filter files and new-engine files |
| C | P5, P6, P8 | SDK, views, second filter family |
| D | P11, P12 | analog voice behaviour and modulation on the graph |

**Backlog** (not packaged):

* BBD ensemble (adopt chowdsp's only if confirmed BSD), tape echo (P10 tape), spring / plate reverb;
* second warp slot and phase-distortion warps;
* sfizz adoption if full SFZ becomes a goal;
* MTS-ESP after a licence check;
* multithreaded voices;
* formula modulators (needs a real-time-safe scripting design).

**SKIP** (honest list):

* any GPL dependency;
* replacing `Json.h`;
* a hand-written time-stretcher or modal resonator bank;
* a full SFZ parser of our own;
* sample-level "clones" of commercial products;
* a scripting runtime in the audio thread for now.
