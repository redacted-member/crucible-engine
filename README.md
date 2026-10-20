# Crucible Engine

A polyphonic synthesis engine as a C++20 library with a plain-C API: no GUI, no plug-in framework, no allocation on the
audio thread, MIT licensed. You build the instrument; Crucible makes the sound.

New here? Read [`docs/GETTING_STARTED.md`](docs/GETTING_STARTED.md): the plug-in lifecycle, threads, host parameters and
state in one page, with a runnable skeleton in `examples/instrument_skeleton/`.

## The niche

Complete polyphonic engines with hardware-grade nonlinear filters, band-limited oscillators and a deep modulation
system exist, but the ones of that calibre are GPL, and fixed to one product. The permissive projects in this space
are collections of DSP blocks: no voices, no patches, no modulation system, no effects rack. Crucible aims to be the
permissively licensed middle: a whole engine you can embed, that describes itself so any UI can be generated from it.

What is in it today: wavetable (with opt-in spectral frame interpolation and warps), sample, multisample, granular, cloud (additive), virtual analog, modal / physical, harmonic, spectral time-stretch and noise sources with up to eight
oscillator slots and unison; 25 filter models (three of them nonlinear ladders, 1x / 2x / 4x oversampled); four envelopes, four LFOs, a 16-slot modulation matrix and assignable
modulation routes; automation clips; 30 effect and utility modules in an orderable rack; arpeggiator, step sequencer, scenes,
microtuning, MPE, key lock; versioned JSON patches and a preset library; meters, loudness and analyzers.

## The platform principle

Crucible is a platform for many different instruments (a virtual analog synth, a hybrid, a modular, a drum synth,
an FX plug-in), so every design choice follows six rules: the instrument is data rather than code, everything
describes itself (range, unit, group, page, flags), reading state back is toolkit-free and lock-free, third parties
can extend it, the core assumes no product, and existing sound stays bit-exact. Some of this is done, some is the
roadmap: `docs/ARCHITECTURE.md` and `docs/ROADMAP.md` say which is which.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
ctest --test-dir build --output-on-failure              # crucible_engine_tests, the C API test, ... 
build/render_wav demo.wav                               # offline render in C++
build/render_c demo_c.wav                               # the same through the C API
build/crucible-render --note 48 --length 2 -o out.wav   # offline CLI (patches, MIDI files, scores); --version
build/crucible-module-check --builtin                   # real-time contract of every module (or a shared library of yours)
build/crucible_engine_bench                             # CPU per configuration and per effect module
tools/ci.sh                                             # Release + ASan/UBSan + TSan + scalar builds and tests
tools/check-spdx.sh                                     # licence headers
tools/refbench.sh build                                 # score the engine against the reference targets (39/39; needs python3 + numpy)
```

Requirements: CMake 3.22+, a C++20 compiler (Apple clang, gcc 11+, clang 14+, MSVC 2022), threads. On Apple platforms
the system Accelerate framework is used for the FFT; elsewhere a portable FFT is used. The library has no third-party
code; `third_party/` holds code used by tests only (`THIRD_PARTY.md`). The build needs no network access. Windows: see `docs/BUILDING-WINDOWS.md`.

Golden-render fingerprints live in `tests/golden/`; regenerate with `CRUCIBLE_UPDATE_GOLDEN=1
build/crucible_engine_tests Golden` (and `CRUCIBLE_UPDATE_BITEXACT=1` for the bit-exact hashes) after an intended
change of sound, and record it in `docs/SOUND_CHANGES.md`. `crucible_engine_tests [filter]` runs only the test classes
whose name contains `filter`.

CMake options: `CRUCIBLE_ENGINE_BUILD_TESTS`, `CRUCIBLE_ENGINE_BUILD_EXAMPLES`, `CRUCIBLE_ENGINE_BUILD_TOOLS` (ON when
this is the top-level project, OFF under `add_subdirectory`), `CRUCIBLE_ENGINE_INSTALL`, `CRUCIBLE_ENGINE_SHARED` (shared library for FFI, off by default),
`CRUCIBLE_ENGINE_SANITIZE` (`address;undefined` or `thread`), `CRUCIBLE_FORCE_SCALAR`, `CRUCIBLE_X86_AVX2`.

## Python

`python/crucible_engine` renders the engine offline from Python over the plain C API (ctypes plus NumPy; no compiled extension).
Build the shared library in its own build directory (only the `crucible_*` functions are exported), then point Python at it:

```sh
cmake -S . -B build-shared -DCMAKE_BUILD_TYPE=Release -DCRUCIBLE_ENGINE_SHARED=ON
cmake --build build-shared --target crucible_shared -j8
export PYTHONPATH=python                                 # or: pip install -e .  (after copying the library into python/crucible_engine)
python3 python/examples/render_patch.py out.wav          # a 3 second note to a WAV
python3 -m unittest discover -s python/tests             # the package tests
```

The library is found through `CRUCIBLE_LIBRARY` (a path), the package directory, or a `build*` directory of a source
checkout. `Engine()` plays the default product, `Engine.from_product(json)` builds another; `set_param`, `set_param_text`,
`note_on`, `render(seconds)` (float32, shape `(n, 2)`), `save_patch`, `load_patch`, `describe`, `reset`, `latency_samples`
and `tail_seconds` cover the rest. The package will be published on PyPI as `crucible-engine` (`pip install crucible-engine`, then `import crucible_engine`) at the public launch; until then, build it locally as above.

## Embedding

Either as a subdirectory:

```cmake
add_subdirectory(path/to/crucible-engine)
target_link_libraries(my_synth PRIVATE crucible::engine)
```

or from an install:

```sh
cmake --install build --prefix /opt/crucible
```
```cmake
find_package(CrucibleEngine 0.1 REQUIRED)
target_link_libraries(my_synth PRIVATE crucible::engine)
```

```cpp
#include <crucible/Engine.h>

crucible::Engine engine;
engine.prepare(48000.0, 512);                 // sample rate, largest block
engine.setParam("filter_cutoff", 1800.0f);    // natural units; clamped to the range
engine.mapCC(74, engine.model().indexOf("filter_cutoff"));

// MIDI callback thread (one producer):
engine.midi(status, data1, data2);            // or noteOn / noteOff / pitchBend / modWheel / aftertouch / sustain

// Audio thread:
engine.process(left, right, numSamples);      // optional: sorted MidiInput array for sample-accurate MIDI, host tempo
auto m = engine.meter();                      // peak dB per channel, clip flag (any thread)
```

`examples/render_wav.cpp` is a complete C++ program (phrase in, 16-bit WAV out).

## C API

`<crucible/crucible.h>` is plain C with opaque handles; `CRUCIBLE_API_VERSION` is the ABI counter (functions are only
added; structs that start with `struct_size` may grow by appended fields, every other layout is frozen per version). `crucible_version()` returns the library version string (the one in
`project(CrucibleEngine VERSION ...)`). Use it from Rust, Swift, Python or C#:

```c
#include <crucible/crucible.h>
CrucibleEngine* e = crucible_create(NULL);   /* the default product; or crucible_create_from_product(NULL, json, NULL, ...) */
crucible_prepare(e, 48000.0, 512);
crucible_set_param_id(e, "filter_cutoff", 1800.0f);
crucible_note_on(e, 60, 0.8f);
crucible_process(e, left, right, 512, NULL, 0);
crucible_destroy(e);
```

Text and arrays come back through caller-owned buffers (`crucible_save_patch(e, name, buf, cap)` and
`crucible_describe_json(e, buf, cap)` return the size they need, snprintf style; pass `NULL` / 0 to ask first). Structs the engine
fills or reads (`CrucibleConfig`, `CrucibleParamMeta`, `CrucibleModuleMeta`, `CrucibleViewInfo`, ...) take `struct_size = sizeof` first;
`crucible_config_init` sets it for the config. Every function is tagged [main], [audio] or [any] in the header, which also has the
thread table.

To play your own instrument, pass a `crucible-product` JSON document (see `docs/products/template.json`) to
`crucible_create_from_product(config, json, modules, error, size)`; `modules` is NULL for the built-in modules. In C++
the same is `Engine::fromProduct(json, error)`.

`examples/c_api/render_c.c` is a complete program that creates an engine, sets parameters, plays a note and writes a
WAV file.

## Versioning

The CMake project version is the single source: it generates `crucible/Version.h`, `crucible_version()` and
`--version` of the tools. Patch files carry their own format version and migrate forward on load.

## Public API (`include/crucible/`)

| Header | What it is |
| --- | --- |
| `Engine.h` | **The embedding facade.** `Engine(EngineConfig)`, `prepare(sampleRate, maxBlock)`, parameters by id / index / normalised value, performance input (`noteOn`, `noteOff`, `pitchBend`, `modWheel`, `aftertouch`, `sustain`, `allNotesOff`, raw `midi()`), CC-to-parameter mapping, `process(L, R, n [, midi, numMidi, host])`, meters and loudness, scenes, sequencer, arp grid, user wavetables and impulse responses, modulation routes and `modulatedValue()`. |
| `crucible.h` | The same as a plain-C API with opaque handles (stable ABI, `CRUCIBLE_API_VERSION`). |
| `EngineConfig.h` | Voice pool size, block size, quality policy, seed. |
| `Patch.h`, `Json.h` | Versioned JSON patch format (meta, parameters, scenes, sequencer, arp, routes, clips, user waves, IR, extra assets) with migration; the small JSON library it uses. |
| `PresetLibrary.h` | Preset library on disk: factory and user banks, index cache, search / filter / sort by text, tags, category, rating, favourites, load / save / rename / move / remove, referenced assets in a content-addressed store. |
| `crucible_module.h`, `sdk/Module.h`, `sdk/Host.h`, `sdk/Check.h` | The module SDK: the C ABI of third-party modules (shared libraries), the C++ layer that generates descriptors, the host-side registry and loader, the real-time harness behind `crucible-module-check`. |
| `module/Module.h`, `module/ModuleRack.h`, `module/ParamBuilder.h` | The module system: descriptors, registry, effect interface, capabilities, the effect chain, parameter helpers. |
| `ParameterMap.h` | The parameter model assembled from the registry: ids, ranges, pages, sections, matrix sources / destinations, generic modulation targets, normalisation, JSON export with host extras. |
| `Modulation.h` | Assignable modulation routes (drag-to-assign: LFOs, envelopes, macros, MIDI CCs, velocity, key, random, clip lanes, bank modulators and audio-rate sources to any modulatable parameter, or to the amount of another route) and their store. |
| `Modulators.h`, `AudioLfo.h`, `FineBlep.h` | The modulator bank (combine, remap, lag, quantise, S&H, Turing, binary, chaos, note gate, follower, pitch tracker, LFO table; voice / layer / global scope), the band-limited audio-rate LFO and the fine minBLEP corrections it shares with `osc.analog`. |
| `KeyLock.h`, `KeyDetect.h`, `KeyBus.h` | Scale lock (Snap / Block / Remap quantiser in the note path, 18 scales + custom mask + Scala), "learn key" detection, and the key bus that lets engines follow one key (Key Sync): in-process out of the box, cross-process through a transport the host supplies. |
| `Tuning.h`, `CcMap.h` | Microtuning (Scala .scl / .kbm, per-note offsets) and the MIDI CC map with learn (ranges, curves, relative encoders, channels). MPE per-note bend / pressure / slide are route sources (`Modulation.h`). |
| `Clips.h`, `MotionRecorder.h` | Automation clips (curve / step lanes, loop, transport sync, MSEG) with their audio-thread player, and the motion recorder that writes live parameter moves into a lane. |
| `EngineCore.h` | The audio-thread core the facade wraps (for hosts that own parameter storage). |
| `SynthEngine.h`, `Voice.h`, `Wavetables.h`, `UserWavetables.h`, `Filters.h`, `Fx.h`, `FxExtra.h`, `ModMatrix.h`, `Dsp.h`, `HalfBand.h`, `DelayLine.h` | DSP building blocks. |
| `Spectral.h` | Spectral wavetable tools: peak-sweeping frame interpolation, six spectral warps, an allocation-free cycle synthesiser (opt-in `oscN_wt_interp`, `oscN_wt_spec`). |
| `Cloud.h` | The 3D cloud model, cloud-to-wavetable, the additive `CloudOscillator` and its snapshot API, `CloudBank` (a cloud per oscillator slot). |
| `Sample.h`, `module/Oscillator.h` | Sample data with band-limited copies and the windowed-sinc kernel, `SampleBank`; the oscillator module interface (`OscillatorInstance`, `SlotParams`). |
| `MinBlep.h`, `Oversampler.h` | minBLEP / minBLAMP corrections; 2x / 4x / 8x oversampling with FIR half-band or polyphase IIR stages. |
| `Arpeggiator.h`, `ArpPatternIO.h`, `Sequencer.h`, `Scenes.h` | Arpeggiator with pattern grid (ties, ratchets, probability and probability groups), pattern import / export (JSON, MIDI clips), step sequencer, scenes; plain-text serialisation of each. |
| `Meter.h`, `MasterAnalyzer.h` | Master peak / clip meter; loudness, true peak, correlation, scope and spectrum. |
| `AudioFile.h`, `MidiFile.h` | WAV read / write, raw float import; standard MIDI file reader. |
| `RtShared.h`, `SpscRing.h`, `SmoothedValue.h`, `Fft.h`, `Simd.h`, `Denormals.h` | Real-time utilities (`Fft.h`: Accelerate on Apple, pffft elsewhere). |

Values are always natural units (Hz, seconds, dB, semitones, choice index). `model().size()` and the ids are the
contract; ids never get renamed once released, new parameters are appended.

## Threading and real-time rules

- `process()` / `EngineCore::process()` / `SynthEngine::render()` never allocate, lock or block (a test counts
  `operator new` calls while the engine runs with 16 voices, the sequencer and every effect).
- Parameter setters write atomics, read once per block.
- Performance input goes through a 511-slot SPSC ring (single producer: call the input methods from one thread).
- Sequencer patterns are edited on a control thread by copy-and-publish; the audio side pins the current bank
  with a hazard pointer.
- Scenes, serialisation, `prepare()`, wavetable bank creation and JSON export are control-thread only.
- `Engine::process()` flushes denormals for its duration.

## Architecture

See `docs/ARCHITECTURE.md`. In short: `Engine` (facade) → `EngineCore` (event merge, sequencer, arpeggiator, meters)
→ `SynthEngine` (smoothing, modulation, voices) → `ModuleRack` (registered effect modules in position order).

## Which module do I use

The first-generation effects (`fx.drive`, `fx.chorus`, `fx.flanger`, `fx.phaser`, `fx.delay`, `fx.reverb`, `fx.comp`,
`fx.eq`, `fx.peq`, `fx.dimension`, `fx.gate`, `fx.conv`) were removed before v1; their successors are in the table. `describeJSON` and `crucible_module_meta` carry `status` and
`supersededBy` for every module: a module retired after v1 stays registered, marked `legacy`, with the id that replaces it.

| I want | Use | Replaced (removed) |
| --- | --- | --- |
| Reverb, algorithmic (FDN, size, damping, modulation) | `fx.reverb_algo` | `fx.reverb` |
| Reverb or cabinet from a recorded impulse response (load a file per instance with `crucible_load_ir`; no built-in responses) | `fx.convolution` | `fx.conv` |
| Compressor, transparent | `fx.comp_clean` | `fx.comp` |
| Compressor, VCA bus glue | `fx.comp_vca` | `fx.comp` |
| Noise gate, expander or ducker | `fx.gate2` | `fx.gate` |
| Equaliser (up to 16 bands, any band type) | `fx.eq_pro` | `fx.eq`, `fx.peq` |
| Delay, digital or analog (BBD) tape-style echo with saturation, filters, stereo modes, modulation, ducking, freeze | `fx.delay_pro` | `fx.delay` |
| Chorus or ensemble: bucket-brigade (BBD) chorus, 1 to 4 lines, Dimensional modes, hyper chorus | `fx.ensemble` | `fx.chorus`, `fx.dimension` |
| Phaser: 2 to 12 all-pass stages, OTA / JFET stages, barber-pole, envelope or sidechain sweep | `fx.phaser2` | `fx.phaser` |
| Flanger: through-zero, digital or BBD, envelope sweep (reports a fixed 249-sample latency) | `fx.flanger2` | `fx.flanger` |
| Loudness maximiser with a true-peak ceiling | `fx.maximizer` | none (`fx.limiter` stays current for plain limiting) |
| Attack and sustain shaping that does not depend on input level | `fx.transient` | none |
| Sibilance reduction on a voice or bus (split-band or wide-band) | `fx.deesser` | none |
| Harmonic exciter (High: brightness above 1 to 10 kHz) or bass enhancer (Low: below 40 to 250 Hz), anti-aliased | `fx.exciter` | none |
| Saturator: ten models (soft, hard, rail, diode, wavefolder, three tubes, tape, transformer), tilt, auto gain, oversampled and anti-aliased | `fx.saturator` | `fx.drive` |
| Multiband Drive: three bands (LR4 crossovers, minimum phase, flat sum), per band one of the ten saturator models, drive 0 to 36 dB, output, mix, solo, bypass; anti-aliased (alias at most -90 dB Standard / -100 dB High, tape -85 / -97), latency reported (26 / 49 / 55 samples); 24 parameters | `fx.multiband_drive` | current |
| Guitar / bass cabinet: eight generated (no recorded IR) cabinets, mic position and distance, low and high cut, zero latency | `fx.cabinet` | none |
| Level and loudness readout | `util.meter` | none |

Every other effect module is current. Start a new product from `docs/products/template.json` (classic voice, 2
oscillators, a short chain of current modules); copy it, change the id, and add or remove instances.

## Effects

All modules are registered in `src/modules/Modules.cpp`.

| Effect | Module | Status |
| --- | --- | --- |
| Delay, digital or analog BBD: 26 parameters, saturating feedback, stereo / ping-pong / cross, diffusion, ducking, freeze | `fx.delay_pro` | current |
| Limiter (lookahead) | `fx.limiter` | current |
| Bitcrusher | `fx.crusher` | current |
| Stereo width | `fx.width` | current |
| Filter FX: the 20 voice filter models with drive and synced LFO sweep | `fx.filter` | current |
| Multiband compressor (3 bands, LR4) with gain-reduction readout | `fx.multiband` | current |
| Trance gate (16 steps, synced) | `fx.trance` | current |
| Tremolo / auto-pan | `fx.tremolo` | current |
| Ring modulator / frequency shifter | `fx.ring` | current |
| Vocoder | `fx.vocoder` | current |
| Clean compressor | `fx.comp_clean` | current |
| VCA bus compressor | `fx.comp_vca` | current |
| Gate, expander and ducker | `fx.gate2` | current |
| Maximizer: lookahead true-peak limiter | `fx.maximizer` | current |
| Clipper | `fx.clipper` | current |
| Meter (utility) | `util.meter` | current |
| Pro EQ | `fx.eq_pro` | current |
| Algorithmic reverb | `fx.reverb_algo` | current |
| Convolution: partitioned, WAV or AIFF IR with 1, 2 or 4 channels (true stereo), up to 30 s, per-instance IR | `fx.convolution` | current |
| Transient shaper: level-independent attack and sustain (plus or minus 15 dB), lookahead up to 5 ms | `fx.transient` | current |
| De-esser: band or high-pass detector, split-band (linear-phase FIR) or wide-band reduction, lookahead up to 2 ms | `fx.deesser` | current |
| Ensemble: BBD chorus, 1 to 4 lines, Dimensional modes with crossfeed, compander and noise; 14 parameters | `fx.ensemble` | current |
| Phaser: 2 to 12 all-pass stages, linear / OTA / JFET stages, barber-pole, envelope or sidechain sweep; 18 parameters | `fx.phaser2` | current |
| Flanger: through-zero (digital or BBD), feedback of either sign, envelope sweep, 249-sample latency; 16 parameters | `fx.flanger2` | current |
| Exciter: High or Low mode, anti-aliased rail saturator with asymmetry, 0 to 9 dB drive, 2x oversampling (Standard and High), latency reported; 9 parameters | `fx.exciter` | current |
| Saturator: ten models (soft, hard, rail, diode, wavefolder, triode, push-pull, pentode, tape hysteresis, transformer), 2nd-order ADAA with oversampling chosen by quality and drive (alias at most -90 dB Standard / -100 dB High; tape -85 / -97 over the whole drive range), tilt pair, loudness-matched auto gain, optional resonant pre / post filter, 14 parameters | `fx.saturator` | current |

Every effect has an on switch, its own chain position (`fx_pos_*`) and, where it makes sense, a mix.

## State serialisation

`crucible_save_patch` / `crucible_load_patch` are the state container. A
plug-in stores the patch JSON as its state chunk and loads it back on restore; `crucible_load_patch_file` reads one
from disk. Patches are versioned JSON. Ids that the engine does not know are ignored and missing ids take their
defaults, so states survive parameter additions. Choices are stored as numbers, and a hand-written patch may use the
choice text instead (`"osc1_type": "Analog"`). `SceneBank::serialise()` and `seqBankSerialise()` return plain
records (`slot` plus a compact `id=value;...` or step-row string, locale independent) for the scene and sequencer
banks. See `docs/GETTING_STARTED.md` for the save / load recipe.

## Tests

`tests/` uses a tiny built-in framework (`TestFramework.h`, no dependencies). Suites: parameter model, wavetables,
oscillators and warps (aliasing measured by FFT), filters, modulation, FX, sequencer, core event merging, scenes,
meter, the `Engine` facade, the SPSC ring, the smoother and the FFT.

Further suites: voice graph (`crucible_engine_graph_tests` runs the 5-oscillator, 4-filter shape in its own process), oscillator modules and cross modulation, minBLEP, oversampler, polyphony (retrigger, steal, pedal, 16 x 8 x 3 load), quality (aliasing, THD+N, bandwidth). Additional suites cover the module registry, modulation routes, patches / JSON / MIDI files, the arpeggiator, user
wavetables and the cloud, the second-generation effects, golden renders, parity, fuzzing and the master analyzer.

CPU-budget expectations need a quiet machine; set `CRUCIBLE_SKIP_CPU_TESTS=1` to log them as skips on shared runners (see `CONTRIBUTING.md`).

Bit-exactness is protected by fingerprints of whole renders (a plain patch, warps and filters, every FX, the
sequencer with locks and swing) and of all wavetables; see `CONTRIBUTING.md`.
