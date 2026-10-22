# Intentional sound changes

Changes that alter the rendered samples of existing patches. Every entry names the commit that regenerated the
golden fingerprints (`tests/golden/renders.txt`) and the bit-exact hashes (`tests/golden/bitexact.txt`).

## `scene_morph` morphs between scene A and scene B (maintainer decision)

* **What changed.** With `vector_on` off, `scene_morph` above 0 and the slots of `scene_a` and `scene_b` both stored, the
  scene-managed parameters now follow the morph from scene A to scene B (the vector field's overlay with corners B, A, B, A;
  `docs/README-features.md`, "Scenes"). Before, `scene_morph` was only the `SceneMorph` modulation source.
* **Who hears it.** Only patches that store scenes and leave `scene_morph` above 0. `scene_morph` defaults to 0, and no
  golden scenario stores scenes with a non-zero morph: `tests/golden/renders.txt` and `tests/golden/bitexact.txt` are
  unchanged (not regenerated).
* **Tests.** `Vector field and clip transport` ("scene_morph morphs scene A towards scene B").

## The twelve first-generation effects are removed (maintainer decision

* **What changed.** `fx.drive`, `fx.chorus`, `fx.flanger`, `fx.phaser`, `fx.delay`, `fx.reverb`, `fx.comp`, `fx.eq`, `fx.peq`,
  `fx.dimension`, `fx.gate` and `fx.conv` no longer exist; the default product's effects chain is the current modules
  (`fx.saturator`, `fx.ensemble`, `fx.flanger2`, `fx.phaser2`, `fx.delay_pro`, `fx.reverb_algo`, `fx.comp_clean`, `fx.eq_pro`,
  `fx.gate2`, `fx.convolution`). This is the one allowed shrink of the frozen parameter order before v1 (never after).
  The five fixed modulation destinations FX Drive Amount, FX Chorus Mix, FX Delay
  Mix, FX Reverb Mix and FX Reverb Size are gone with them (route any modulatable parameter instead).
* **Measured** (Release, arm64; chord E3 + B3, 0.5 s held, 1.2 s render, mono mix; level over the held part, tail over
  0.6 .. 0.7 s after release; `tests/LegacyMapMeasure.cpp`, removed with the modules). Dry level -14.95 dB, peak -6.77 dB.
  Old settings = the module defaults of the old effect; new = the closest settings of the successor:

| Effect (old -> new) | level old / new (dB) | peak old / new (dB) | tail old / new (dB) | note |
|---|---|---|---|---|
| drive Soft, amount 0.3 -> saturator Soft Clip, drive 12 dB, auto gain off, output -4.2 dB | -9.61 / -9.64 | -4.51 / -4.44 | -162 / -177 | drive dB = 40 x amount, output = -14 x amount dB |
| drive Soft, amount 0.8 -> drive 32 dB, output -11.2 dB | -11.73 / -11.74 | -10.17 / -9.53 | -133 / -171 | |
| drive Tube 0.5 -> Tube Triode, drive 20 dB, output -7 dB | -9.16 / -7.25 | -4.98 / -0.23 | -88 / -55 | different tube model (Koren triode), louder |
| chorus defaults -> ensemble (rate 0.4, depth 15 ct, delay 12 ms, no noise / compander) | -20.05 / -19.31 | -9.55 / -9.78 | silent | |
| dimension defaults -> ensemble Dimensional II | -19.57 / -20.71 | -9.33 / -10.90 | silent | |
| flanger defaults -> flanger2 (depth 3 ms, manual 2 ms, feedback 0.5) | -21.36 / -19.72 | -12.07 / -8.77 | -200 / -154 | |
| phaser defaults -> phaser2 (8 stages, range 1.8 oct) | -21.48 / -21.14 | -11.74 / -11.03 | silent | |
| delay defaults -> delay_pro Digital, saturation 0 | -18.04 / -18.03 | -9.53 / -9.44 | -28.8 / -28.7 | band levels equal within 0.1 dB |
| reverb defaults -> reverb_algo Plate, mix 0.25 | -14.42 / -17.29 | -6.43 / -9.26 | -35.6 / -36.3 | old adds the wet on top of the full dry, the new one cross-fades: body 2.9 dB lower, tail equal |
| comp defaults -> comp_clean defaults (RMS) | -19.10 / -19.25 | -8.18 / -6.77 | silent | |
| eq +6 dB on three bands -> eq_pro bands 1-3 (low shelf, bell Q 0.7, high shelf) | -12.60 / -13.17 | -3.30 / -3.44 | | |
| peq band 4 +6 dB -> eq_pro bell 800 Hz Q 0.707 | -12.66 / -12.66 | -1.90 / -1.90 | | identical |
| gate, flat eq | unchanged | | | |
| conv Room (built-in impulse response) -> convolution | -17.68 / -14.95 | -9.51 / -6.77 | -61 / silent | no built-in responses any more: the new module is dry until a response is loaded |

* **Goldens.** `tests/golden/renders.txt` and `bitexact.txt`: only the two scenarios that switch the old effects on changed,
  `classic_fx` (drive, chorus, phaser, delay, reverb, comp, eq on) and `new_fx` (parametric EQ, dimension and convolver on);
  both now use the mapped successor settings (tests/QaTests.cpp, tests/BitExactTests.cpp; the convolver is not in `new_fx`
  any more: no built-in response). Mean level over the second: `classic_fx` -24.22 -> -26.77 dB (loudest 50 ms window -19.95 ->
  -23.44 dB), `new_fx` -31.36 -> -28.46 dB (loudest window -23.98 -> -22.10 dB; the tail after the release is -65 -> -120 dB
  because the convolver's room tail is gone). Hashes: `classic_fx` 4e846bf4397e3968 -> 7ef467793071ae5e, `new_fx`
  78b59d73c7118dfd -> 9b71e5f4f3f9212e. Every other scenario (default chord, warps / filters / matrix, split routes, serial
  filters, arpeggiator and sequencer, ...) is unchanged bit for bit. `tests/golden/param_order.txt` was regenerated once (1625
  -> fewer parameters; indices and the generic modulation destination numbers moved down).
* **Approval.** Maintainer decision.

## Pro EQ: bell centre gain near Nyquist

* **What changed.** The magnitude fit of a bell (`fx_ep_bN_type` Bell) whose centre lies above half of Nyquist now weights
  its centre frequency up to three times as much (weight 1 -> 3 over 0.5 ... 0.8 of Nyquist, smooth, bells only). The fit used
  to give up part of the centre gain to follow the cramped skirts: a +9 dB, Q 2 bell at 17640 Hz (44.1 kHz) measured
  +9.115 dB. Bells below half of Nyquist, every other band type and every other module are bit-identical.
* **Measured** (+9 dB, Q 2; set value 9.000): minimum phase 44.1 kHz at 17640 Hz 9.115 -> 9.032 dB (acceptance bound
  +-0.10); 48 kHz at 19200 Hz 9.103 -> 9.029 dB. Linear phase (the FIR follows the same design): 44.1 kHz 9.115 dB and
  48 kHz (18000 Hz) 9.065 dB, both now inside the +-0.05 dB bound. The worst whole-curve error of the bells
  (`EqPro` tests, 44.1 kHz, |gain| <= 12 dB) moved 0.405 -> 0.391 dB.
* **Effect on the goldens.** None: `tests/golden/renders.txt`, `bitexact.txt` and the fx baseline contain no bell above half
  of Nyquist, so no entry changed. Three `fx.eq_pro` bell entries left `tests/OlderFxKnownFail.txt`.

## Voices: random start phase, retrigger and steal fades

* **What changed.** Every unison voice, the first included, now starts at a random phase scaled by the oscillator's
  *Random Phase* parameter (`oscN_uni_rand`, default 1). Before, only the second and later voices of a unison stack
  did, so two notes of the same pitch held a fixed phase relation (measured: two same-pitch voices could sum 2.2 dB
  *below* one note). `Random Phase` = 0 resets every voice to the *Phase* knob, as before for the first voice.
  The phases come from the per-note seed (note age, key, `EngineConfig::seed`), so renders stay reproducible.
* **Retrigger.** A held key struck again fades its older voice out over about 5 ms (cosine) instead of running
  beside the new one; voices stolen at the polyphony limit fade the same way while the new note starts on one of
  eight spare voices (`kStealSpareVoices`, beyond `EngineConfig::maxVoices`). A note that starts on top of a
  *releasing* voice of the same pitch is detuned by +3, -3, +6 ... cents so the two beat slowly.
* **Effect on the goldens.** Level and spectrum fingerprints of chords move by up to a few dB (the phases of
  stacked partials are different), all bit-exact hashes change.
* **Tests.** `Polyphony`: same-pitch pairs across retrigger offsets stay within 1 dB of a single note, click-free
  retrigger and steal, note-offs and the sustain pedal act on their own voices, 16 x 8 x 3 load timing.

## Oscillator anti-aliasing: finer band-limited levels, 6-point reads, smooth morph, minBLEP sync

Measured with `tools/refbench.sh` against the recorded targets.

* **Wavetable levels.** Every frame is stored at 27 levels of 1023 ... 1 partials (was 11 octave steps). The voice
  picks the level with the most partials that stays below (sample rate - 20 kHz) / f0, so partials that fold over
  Nyquist land above 20 kHz and the audible bandwidth is at least 20 kHz at every pitch from C2 up (was 12.5 kHz at
  the worst note). Neighbouring levels differ by less than 1.4 in partial count. Tables are stored with 128 to 4096
  samples per cycle (more for the high levels), memory grew from about 37 MB to about 130 MB for the factory tables.
  Notes below 65 Hz use the four top levels, which trade interpolation accuracy for memory.
* **Interpolation.** 6-point, 5th-order Lagrange with weights shared by both morph frames (was cubic Catmull-Rom).
  Interpolation images are below -120 dB at every level (were -75 dB at high notes). The unison inner loop runs four
  voices per NEON register, so 16 notes x 8 unison costs less CPU than before (4.2 % against 5.3 % on the reference
  machine).
* **Oversampled (warped) oscillators** keep the Nyquist of their own rate as the limit (partials of warps are
  generated after the table).
* **Morphing.** The frame position ramps per sample from the previous chunk's end instead of stepping every 32
  samples (sweep aliasing p90 at C7 from -80 to below -107 dB).
* **Hard sync.** The restart of the slave is corrected with a 32-zero-crossing minimum-phase minBLEP and a minBLAMP
  for the slope break (was a two-sample polynomial). Synced saws alias about -100 dB below 20 kHz at low pitch.
* **Tests.** Reference-bench targets for saw, square, sine, bandwidth, unison and sweep now pass; unit tests measure
  aliasing below 20 kHz.

## Filters, envelope attack, unison width, drive oversampler

* **Resonant low-pass peaks sit on the cutoff dial.** A state-variable low-pass peaks below its cutoff parameter; the
  12 dB and 24 dB filters now divide the cutoff by the (analytic / numerically found) peak ratio from resonance 0.6
  on, fully from 0.85. Reference bench: peak error 50 -> 9 cents (24 dB, 1-12 kHz), 25 -> 4 cents (12 dB, 1 kHz).
* **Top of the resonance dial.** Damping of the state-variable sections is unchanged up to resonance 0.9 (Q about 4)
  and falls exponentially to 0.006 at 1.0 (Q about 330): the 24 dB filter reaches a 38.7 dB peak (was 24.6 dB).
* **Attack.** The attack reaches the peak at the attack time (0.97x measured, was 0.70x): the attack no longer uses the
  decay/release curve. It is linear with rounded corners and has its own `*_attack_curve` parameter per envelope
  (`amp_attack_curve`, `menv_attack_curve`, `env3_attack_curve`, `env4_attack_curve`, default 0). Onset click of a zero
  attack stays below -83 dB.
* **Unison width.** `oscN_uni_spread` defaults to 1 (was 0.5): the 8-voice stack correlates 0.54 between left and
  right (was 0.87); mono sum loss stays at 1.1 dB.
* **Drive oversampler.** The drive uses the new `Oversampler`: a proper cascade of half-band stages (the previous
  per-phase split was equivalent to a different filter structure). Quality picks 2x / 4x / 8x
  (`EngineConfig::quality`), `EngineConfig::oversampling` picks linear-phase FIR half-band (23 / 47 / 95 taps) or
  all-pass polyphase IIR (4 / 6 / 8 coefficients, 65 / 96 / 122 dB of stop-band rejection). Warped oscillators cap at
  2x in Draft and run 2x modes at 4x in High.
* **Denormals.** `SynthEngine::render` and `EngineCore::process` flush denormals themselves (as `Engine::process`
  already did), so hosts calling the lower layers are protected too.

## Vector kernels for the warps (rounding-level)

* **What changed.** FM, ring, hard sync and fold oscillators, and unison stacks of 2, 3, 5, 6 or 7 voices of every
  warp that has a kernel, now run in four-lane vector kernels (NEON, SSE, AVX2 gathers, or the scalar `Float4` when
  `CRUCIBLE_FORCE_SCALAR` is set) instead of one voice at a time, and the half-band decimators of the oversampled
  oscillators work on blocks. The arithmetic is the same 6-point Lagrange interpolation; only the order of some
  additions differs, so samples move by rounding error (about -140 dB). The level and spectrum fingerprints in
  `tests/golden/renders.txt` and all reference-bench targets (39 of 39) are unchanged.
* **Effect on the goldens.** Only the bit-exact hashes (`tests/golden/bitexact.txt`) of scenarios with warps and
  unison stacks changed; they were regenerated in the commit that follows the kernels.
* **CPU.** 16 voices x 3 oscillators x 4 unison with FM + hard sync + fold: 37.7 % to 15.4 % of one core.

## Control rate on an absolute grid: smoothing and drift independent of block size (audit fix)

* **What changed.** `SynthEngine::render` cut its audio into 32-sample control chunks starting at the start of every call, and
  stepped the parameter smoothers once per chunk. A host block (or a segment between two MIDI events) that ended inside a
  chunk therefore stepped them for a short chunk, and `setParameterValues` overwrote the smoothed values with the targets on every
  call. A 30 ms parameter ramp took 1440 samples at block sizes that are multiples of 32, but about 100 samples at a block
  size of 17, 7 samples at 7 and 2 samples at 1 (zipper noise on knobs with hosts that use small blocks). The control rate now
  runs on an absolute 32-sample grid: short chunks end on the grid, the smoothers step only when a grid cell begins, and
  parameters that are still gliding keep their smoothed value across calls. The voice's analog drift (one random-walk step per
  call) steps once per 32 samples of the voice's own time.
* **Effect on the goldens.** Blocks that are multiples of 32 with events on the 32-sample grid render bit-identically to
  before. Renders with events off the grid (a note-off that falls inside a cell, as in the bit-exact scenarios) move because
  the chunks after the event now end on the grid instead of 32 samples after the event: seven bit-exact hashes changed
  (`warps_filters_matrix`, `split_routes`, `serial_two_filters`, `parallel_filter1_off`, `mono_legato_glide`,
  `matrix_voice_dests`, `drift_stacks_sub`); the level / spectrum fingerprints (0.05 dB) and all reference-bench targets are
  unchanged.
* **Tests.** `FeatureAuditTests`: parameter smoothing takes the same time (1409 to 1472 samples) at blocks of 64, 32, 17, 7
  and 1; renders at blocks 32 and 512 are bit-identical with events on the grid.

## Wavetable band limit above 48 kHz (measurement harness fix)

* **What changed.** `Voice::bandLimit` picks the wavetable level with the most partials below (rate - 20 kHz). At 96 kHz that is
  76 kHz, so a low note read table partials whose 6-point interpolation images folded back into the audible band: a saw at C2
  measured -50.0 dB of aliasing below 20 kHz at 96 kHz and -64.3 dB at 192 kHz (-115 to -118 dB at 44.1 and 48 kHz). The limit
  is now also capped at 30 kHz (never below 0.2 cycles per sample): C2 reads -120.4 dB at 96 kHz and -119.3 dB at 192 kHz.
* **Effect on the goldens.** None. The cap is above the old limit at 44.1 and 48 kHz (30 kHz / 48 kHz = 0.625 against
  0.583), so renders at those rates are bit-identical; the golden and bit-exact files are rendered at 48 kHz. Only renders at
  88.2 kHz and above change.
* **Tests.** `MeasuredBaseline`: saw aliasing below -100 dB at C2, C4, C6 at 44.1, 48, 96 and 192 kHz. Numbers in `MEASUREMENTS.md`.

## Legacy effect fixes BFX1-BFX7 (FX2)

Seven bugs of the legacy effects, found by the FX1 baseline (`docs/plans/ENGINE-FX-PLAN.md`, section 2.3). Numbers are
measured with `tests/FxMeasure.h`; each has a test in `tests/FxFixTests.cpp` that fails on the old code.

* **BFX1, drive DC blocker.** The wet-path blocker had a fixed pole of 0.995, so its corner was 38 Hz at 48 kHz and moved
  with the rate. The pole is now `exp(-2 pi 5 Hz / rate)`. Gain at 40 Hz (amount 0, tone 1): -2.47 / -2.80 / -6.67 /
  -11.93 dB at 44.1 / 48 / 96 / 192 kHz before, -0.06 / -0.06 / -0.07 / -0.07 dB after (the rest is the tone filter).
  Every drive patch keeps more bass.
* **BFX2, multiband band sum.** The low band now runs through the high crossover's all-pass (its low-pass and high-pass,
  summed), so low + mid + high is the all-pass of both crossovers. Worst deviation of the sum, dynamics neutral:
  7.19 dB (500 / 750 Hz), 0.76 dB (300 / 1200 Hz), 0.05 dB (defaults) before; below 0.001 dB after. Compressed bands
  see the low band with the extra phase shift of the all-pass.
* **BFX3, stale tails.** `deactivate()` of the delay, reverb and convolver clears their lines, filters and buffers once
  (the rack calls it every block while the effect is off). Output after switch-on with silent input: -10.96 dBFS
  (replayed echoes) before, digital silence after.
* **BFX4, on / off click.** The rack crossfades each module between its input and its output over 10 ms (smoothstep)
  when its on parameter changes, runs it while the fade lasts and deactivates it when a fade-out has ended; the first
  block after `prepare()` / `reset()` adopts the on state without a fade. Largest step of a 1 kHz sine through the
  width module: -35.7 dB before, -133 dB after. Modules with latency crossfade against the unaligned input (the legacy
  modules report no latency).
* **BFX5, limiter lookahead.** The audio delay is an `fx::Lookahead` (two taps, 10 ms quintic crossfade) and the
  detector keeps its history, so a change no longer clears the buffers. Step of a 1 kHz sine when the lookahead goes from
  1.5 to 3 ms: -31.6 dB (and a dropout of `look` samples) before, -120.6 dB after. With a constant lookahead the output is
  unchanged.
* **BFX6, multiband crossovers.** The crossover frequencies glide through two one-pole stages of 12 ms and the filters are
  redesigned per sample while they move. Step while a crossover goes 400 -> 800 Hz (500 Hz carrier): -38.1 dB before,
  -110 dB after; 2000 -> 4000 Hz (1.5 kHz carrier): -20.4 dB before, -106 dB after. Settled filters are the same as before.
* **BFX7, frequency shifter.** The Hilbert filters are designed in `prepare()`: at 48 kHz the classic coefficient set is
  kept (bit-identical); at other rates the pole frequencies of that set are extended and refined by a damped Gauss-Newton
  fit of the 90 degree difference from 20 Hz to min(20 kHz, 0.45 rate) with 10 sections per path pair (12 above 200 kHz).
  Image of a 30 Hz tone shifted up by 100 Hz against the wanted tone: -44.3 dB at 48 kHz both before and after; at 192 kHz
  -11.3 dB before, -61.8 dB after (96 kHz: -24.3 before, -71.8 after; 44.1 kHz: -78.2 after).
  Nothing is allocated on the audio thread.

**Effect on the goldens.** Two scenarios changed and no other:
* `classic_fx` (level fingerprint up to 0.10 dB, bit-exact hash): the drive is on in it; BFX1 only. With the old pole
  restored it matches the old hash.
* `new_fx` (level fingerprint up to 0.06 dB, bit-exact hash): the multiband is on in it; BFX2 only. With the old low band
  restored it matches the old hash.

All other fingerprints and hashes are unchanged, and the reference bench stays at 39 of 39.

## `osc.analog`: an edge exactly on a sample boundary (P12)

* **What changed.** The band-limited corrections of the analog oscillator (`FineBlep.h`, moved out of `AnalogOscillator.cpp` for the
  audio-rate LFO) treated an edge that falls exactly on a sample boundary as arriving one sample early, so it was cancelled
  one sample off. That happens whenever the frequency is a rational share of the sample rate whose phase lands on the boundary
  (440 Hz at 48 kHz: every eleventh cycle), and left the output with a non-harmonic residue of -22 dB on a 440 Hz pulse
  and saw (-13 dB at 1760 Hz). The edge now stays in its own sample; the same pulse measures -119.6 dB. Every other
  frequency has no such edge and renders the same bits as before.
* **Effect on the goldens.** None: no recorded scenario plays `osc.analog` at such a frequency (the default
  patches, `tests/golden/renders.txt` and `tests/golden/bitexact.txt`, are unchanged).
* **Tests.** `Audio-rate modulation`: the 440 Hz pulse with and without PWM, audio-rate LFOs at 440 and 1760 Hz.

## Analog filters run blockwise in graph nodes and the non-bank classic voice

* **What changed.** `filter.analog` nodes and the classic voice's filter path when the quad `FilterBank` is not used
  (analog models 17-24) now pass the whole chunk to `FilterUnit::processBlock` instead of calling `process()` once per
  sample, so the resampling chain, shaper and filter core run on blocks of up to the control chunk rather than on single
  samples. Legacy models keep their per-sample loop (`processBlock` with a null cutoff modulation is the same loop).
* **Why.** Each per-sample call ran the full up/down resampling chain for one sample.
* **Size of the difference.** None in the samples at constant controls: the per-sample and block paths agree bit for bit
  (max abs error 0 on a 4 s saw plus noise render, drive 0.2, resonance 0.7, tolerance 0.3, all eight models, Standard
  and High; all goldens and bit-exact hashes unchanged). With the cutoff moving from chunk to chunk the 16-sample
  control ramps advance in different steps in the two paths, which differs by at most 2e-6 (rounding level; the
  `VoiceQuad` test asserts both). Control updates were already every chunk in both paths, so modulation rates are
  unchanged. Cost, one stereo filter, % of one core, per-sample then block: Standard 0.52-1.16 then 0.20-0.92
  (transistor ladder 1.02 then 0.75), High 1.37-4.19 then 0.48-3.51 (transistor ladder 3.22 then 2.70). Measured on a
  loaded machine.
* **Date.** 2026-10-16, approved on the condition that every golden stays bit-identical.

## filter.analog nodes of voice pairs run as four-lane quads

* **What changed.** In a graph product the `filter.analog` nodes of voices 2p and 2p + 1 of a layer (by pool position)
  share one four-lane `AnalogFilter` per model, the way the classic voice's `FilterBank` pairs voices: left and right of
  one voice fill two lanes. A lane keeps its voice for the life of the program. A voice that does not sound leaves its
  lanes on silence. `EngineConfig::batchAnalogFilters = false` keeps the per-voice path of the previous entry, bit for
  bit. `SynthEngine::batchedFilterBlocks()` counts these quad blocks too.
* **Why.** Each voice ran its own scalar filter and resampling chain.
* **Size of the difference.** Batched against single differs by float rounding (vector lane against scalar, ramps
  advanced in different steps): at most 5e-7 absolute (peak about 1) over all eight models, with and without an
  audio-rate cutoff, six voices with staggered notes and note-offs; the existing FilterBank bound (1e-5) applies, and
  self-oscillation at full resonance and drive is chaotic, as for the classic voice. Goldens and bit-exact hashes are
  unchanged (no recorded scenario runs `filter.analog`). Cost, % of one core per stereo voice, single then batched,
  4 and 8 voices (loaded machine): transistor ladder 0.82 then 0.28, coupled diode ladder 1.0 then 0.33, Sallen-Key
  LP 0.47 then 0.17 (before step 3: 1.13, 1.31, 0.92).
* **Date.** 2026-10-16, approved: batched lanes differ from single by the FilterBank bound, goldens
  unchanged.

## `Bbd` reconstruction filter: four complex modes instead of a 20-tick FIR (FX11)

* **What changed.** The reconstruction filter of the bucket-brigade core (8th-order Butterworth on the held staircase) was a 20-tick
  FIR with a table lookup per tick per sample. It is now the filter's four complex modes (partial fractions of the step response):
  per sample the modes advance by exp(p x) from a table, and a tick adds the held step times its residue. It is exact at a constant
  clock (the FIR cut the tail at 20 clock periods, about -74 dB) and a moving clock ages old steps at the current clock.
* **Effect.** `fx.delay_pro` in Analog mode (and the new BBD modes of `fx.ensemble` and `fx.flanger2`) differ from the FIR output by
  -78 dB rms. `fx.delay_pro` is unreleased, so no patch is affected; no golden or bit-exact hash changed. Bandwidth stays within
  0.1 % of before and the clock images measure -70 to -102 dB (were -69 to -100).
* **Cost.** BBD delay of 10 ms 0.198 to 0.128 % per channel, `fx.delay_pro` Analog with all features 0.511 to 0.352 %.
* **Date.** 2026-10-17.


## Noise oscillator: Noise Tilt is monotonic for positive values

* **What changed.** For `oscN_noise_color` > 0 the complementary high-pass corner now rises from 20 Hz to 2.56 kHz
  (20 Hz * 2^(7 |tilt|)). Before, it used the low-pass law and fell from 20 kHz to 160 Hz, so brightness peaked near
  +0.3 .. +0.5 and +1 was almost flat (a 160 Hz high-pass). Colour 0 and every negative colour are bit-identical.
* **Measured** (energy above 8 kHz minus below 250 Hz, dB; 65536 samples, `tests/NoiseTiltTests.cpp`):

| Noise | Colour | Tilt before | Tilt after | RMS before (dBFS) | RMS after (dBFS) |
|-------|-------:|------------:|-----------:|------------------:|-----------------:|
| White | -1   | -18.2 | -18.2 | -30.6 | -30.6 |
| White | -0.5 | 0.5 | 0.5 | -20.1 | -20.1 |
| White | 0    | 16.5 | 16.5 | -10.8 | -10.8 |
| White | 0.5  | 38.0 | 22.2 | -12.3 | -11.0 |
| White | 1    | 20.4 | 41.1 | -10.9 | -13.0 |
| Pink  | -1   | -39.7 | -39.7 | -16.8 | -16.8 |
| Pink  | -0.5 | -20.3 | -20.3 | -14.7 | -14.7 |
| Pink  | 0    | -4.7 | -4.7 | -13.2 | -13.2 |
| Pink  | 0.5  | 18.5 | 2.5 | -19.7 | -16.2 |
| Pink  | 1    | 0.5 | 21.6 | -15.8 | -20.8 |
| Brown | -1   | -60.6 | -60.6 | -5.5 | -5.5 |
| Brown | -0.5 | -40.6 | -40.6 | -5.2 | -5.2 |
| Brown | 0    | -25.1 | -25.1 | -5.1 | -5.1 |
| Brown | 0.5  | 0.7 | -15.7 | -26.9 | -17.3 |
| Brown | 1    | -18.0 | 3.7 | -15.8 | -29.0 |

  ("Before" tilt peaks at about +0.3 (white 45.9 dB) and falls to 20.4 dB at +1.)
  Brown at +1 is about 24 dB quieter than at 0 because a high-pass removes most of its energy; at +1 pink is 5 dB and brown 13 dB
  quieter than before, white 2 dB quieter.
* **Goldens.** No golden changed (none renders positive noise colour).
* **Approval.** Pending a maintainer decision.

## Analog RC envelope: the attack curve now applies

* **What changed.** In the Analog RC envelope mode (`*_mode` = Analog RC, and the `env.analog` node) the attack curve
  (`amp_attack_curve`, `menv_attack_curve`, `env3_attack_curve`, `env4_attack_curve`, node `attack_curve`) used to have no effect.
  It now bends the RC charge: level = expm1(k u) / expm1(k) of the RC state u, k = 8 x curve. Negative starts faster, positive
  slower; the peak is still reached at the attack time (the charge crosses 1 there) and a retrigger from a level continues from it.
  Curve 0 (default), the Digital mode and every other stage are bit-identical (golden and hash checked). Patches with RC mode AND a
  non-zero attack curve change.
* **Measured** (attack 50 ms, overshoot 0.35, level at the time after note-on; `tests/RcAttackCurveTests.cpp`):

| Curve | 5 ms | 10 ms | 15 ms |
|------:|-----:|------:|------:|
| -1   | 0.7445 | 0.9226 | 0.9729 |
| -0.5 | 0.5035 | 0.7347 | 0.8499 |
| 0 (unchanged) | 0.1704 | 0.3194 | 0.4495 |
| 0.5  | 0.0182 | 0.0482 | 0.0940 |
| 1    | 0.0009 | 0.0039 | 0.0119 |

* **Goldens.** No golden changed.
* **Approval.** Maintainer decision ("make curve apply in RC").

## fx.clipper below 48 kHz: 4x with the High filter at Standard

* **What changed.** At sample rates below 48 kHz the clipper's Standard 2x setting runs 4x with the High (95-tap)
  half-band filters. The 5th harmonic of a 5 kHz tone (25 kHz) sat in the transition band of the 2x half-band at
  44.1 kHz and folded to 19.1 kHz. Measured alias power for a 5 kHz sine 6 dB over the ceiling (default shape,
  ADAA 2): -53.5 dB before, -116.0 dB after (bound -90). 48 kHz and above are bit-identical; Draft (1x) and 4x /
  8x are unchanged. Reported latency follows the stage actually run.
* **Cost.** About 0.34 % of a core before, about 1.06 to 1.09 % after at 44.1 kHz (stereo, 32-sample blocks, continuous
  clipping; 4x costs about that much at 48 kHz too).
* **Goldens.** None changed (they render at 48 kHz).
