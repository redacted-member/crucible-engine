# Changelog

All notable changes are listed here. The format follows Keep a Changelog; versions follow semantic versioning
(`project(CrucibleEngine VERSION ...)` is the single source). `CRUCIBLE_API_VERSION` is the separate C ABI counter.

## [Unreleased]

### Added
- C API gaps found by building a plug-in on the plain C API (all additive, `CRUCIBLE_API_VERSION` unchanged):
  `crucible_process_events_aux` takes MIDI events, parameter events, the transport and a sidechain in one call (a plug-in
  with a sidechain bus no longer needs a zero-length `crucible_process_events` before `crucible_process_aux`);
  `crucible_store_scene` / `_recall_scene` / `_morph_scenes` / `_clear_scene` / `_scene_filled` and `CRUCIBLE_SCENES`
  expose the scene slots; `crucible_set_render_offline` / `crucible_render_offline` (C++ `Engine::setRenderOffline`,
  `ModuleContext::renderOffline`) let a host that renders offline make `fx.convolution` and `fx.cabinet` wait for their
  background builds at any quality, so a Standard-quality render right after `crucible_load_ir` or a cabinet change no
  longer depends on thread timing (default off: real time unchanged). Tests: `crucible_capi_test`, `Vocoder`,
  `Convolution`, `Vector field and clip transport`.
- `SECURITY.md` (private vulnerability reporting), `.gitattributes` (vendored and generated files), pyproject metadata (SPDX licence, classifiers, project URLs).

### Fixed
- Tests: two MSVC-only failures after the effect removal: the saturator-mode allocation test built a parameter id longer than MSVC's small-string buffer inside the counted window, and the patch round trip rendered `fx.convolution` below High quality, where the audio thread does not wait for the kernel builder (both renders now run at High quality). No engine change.
- Pro EQ (`fx.eq_pro`): the auto gain makeup is limited to +-30 dB. With many bands at their extremes the mean of the curve could be far below -30 dB, the makeup went past +100 dB and the noise floor of a silent input came out as a constant at the output clamp (found by the feature audit's random extreme patterns; about 13 % of random patterns). Settings inside the range of normal use are unchanged bit for bit.

### Removed
- The twelve first-generation effects (maintainer decision, before the v1 freeze): `fx.drive`, `fx.chorus`, `fx.flanger`, `fx.phaser`, `fx.delay`, `fx.reverb`, `fx.comp`, `fx.eq`, `fx.peq`, `fx.dimension`, `fx.gate` and `fx.conv`, with their DSP classes, the `FxRack` / `FxParams` test rack, the built-in convolver rooms, the legacy single impulse response (`UserImpulseResponse`, `Engine::loadImpulseResponse*`, the patch field `impulseResponse`) and the five fixed modulation destinations FX Drive Amount, FX Chorus Mix, FX Delay Mix, FX Reverb Mix and FX Reverb Size. The successors are `fx.saturator`, `fx.ensemble`, `fx.flanger2`, `fx.phaser2`, `fx.delay_pro`, `fx.reverb_algo`, `fx.comp_clean`, `fx.eq_pro`, `fx.gate2` and `fx.convolution`. `tests/golden/param_order.txt` shrank once (allowed before v1, never after); the surviving rack effects were renumbered (`fx_pos_*` slots 1 to 8, pages `fx_order_1` to `fx_order_3`); products and the skeleton use the successors; the two default-product scenarios `classic_fx` and `new_fx` were re-recorded (`docs/SOUND_CHANGES.md`). The generic `status` / `supersededBy` mechanism (`ModuleStatus::Legacy`, `crucible_module_meta` `legacy` / `superseded_by`) stays for retirements after v1; no built-in module is legacy now.

### Changed
- `scene_morph` now morphs (maintainer decision): with `vector_on` off, `scene_morph` above 0 and the slots of `scene_a` and
  `scene_b` both stored, the scene-managed parameters follow the morph from scene A to scene B on the audio thread, through
  the vector field's overlay (no parameter writes, no automation; floats in normalised space, choices switch at 0.5, as
  `crucible_morph_scenes`). It used to be only the `SceneMorph` modulation source, which it still is. A patch with stored
  scenes and `scene_morph` above 0 now sounds different (docs/SOUND_CHANGES.md); golden renders are unchanged.
- The fuzz harness hardware profile is selected with `--hardware-preset`; documentation and comments use neutral wording for the public release.
- Older-fx acceptance, `fx_mx_gain` zipper: judged against the same maximizer fed the same gain change smoothly per sample on its input (module's own gain smoothing), so the intended gain-reduction motion cancels; -90 dB unchanged. The maximizer gain smoothing is now 3 x 20 ms (was 3 x 8 ms; a 512-sample block step was at -89.7 / -88.4 dB, 12 ms still -86 dB on the square), so `fx_mx_gain` passes at blocks 1, 7 and 512 and its known-fail entry is gone; constant gain is bit-identical. The known-fail list is now empty, and an empty list no longer inverts the known-fail ctest.
- Smoothed effect controls now always end exactly on their target: a float one-pole step that rounds to nothing (before the relative-epsilon snap fired) left dynamics threshold / mix / makeup, maximizer ceiling / gain / clip share, reverb mix / width / size / density / mod depth and the Pro EQ output / mix / range stuck a few ulps short, so the moving per-sample paths ran forever and constant settings never returned to the bit-identical static path (the maximizer could exceed its ceiling by about 7e-6). `onePoleStepSnap` (fx/Smoothing.h) lands on the target when a step makes no progress; the reverb and Pro EQ smoothers keep double state so that landing is never a step. Tests: `Smoother settle`.
- fx.maximizer: ceiling, clip share and gain now move per sample (gain through three one-poles; ceiling and clip share over at least the lookahead, the output clamp uses the ceiling the sample was detected at, so the ceiling guarantee holds while it moves; the clip stage fades in and out). Constant settings are bit-identical. `fx_mx_ceiling` and `fx_mx_clip` zipper entries removed from the known-fail list; `fx_mx_gain` stays (limiter's own gain-reduction onset, about -86 dB).
- fx.clipper below 48 kHz: Standard 2x runs 4x with the High filter, alias at 44.1 kHz -53.5 dB to -116.0 dB; 48 kHz and above bit-identical (docs/SOUND_CHANGES.md).
- C API nits before the v1 freeze: `crucible_param_index` is now tagged [any] (it already looked the id up by string_view without allocating; a test counts allocations), and `crucible_latency_samples` / `crucible_tail_seconds` take `const CrucibleEngine*` (source and ABI compatible).
- Pro EQ (`fx.eq_pro`): no zipper noise on any continuous parameter (band gain / frequency glide, dynamic range / threshold / ratio / attack / release, output, mix; linear-phase kernels crossfade over 20 ms); bell centre gain near Nyquist now within 0.05 dB (docs/SOUND_CHANGES.md).
- Analog RC envelope: the attack curve (`amp_attack_curve`, `menv_`, `env3_`, `env4_`, node `attack_curve`) now bends the RC attack (negative faster, positive slower, peak still at the attack time); curve 0 and the Digital mode are bit-identical (docs/SOUND_CHANGES.md).
- Noise oscillator: Noise Tilt above 0 now raises a high-pass corner (20 Hz to 2.56 kHz) so brightness rises steadily to +1; it used to fold back above about +0.4. Colour 0 and below are unchanged (docs/SOUND_CHANGES.md).

### Added
- fx.saturator resonant pre / post filter (maintainer decision, distortion gap 4; opt-in, default Off and bit-identical): `fx_saturator_filter_mode` (Off / Pre / Post), `fx_saturator_filter_type` (Low-pass / Band-pass / High-pass), `fx_saturator_filter_cutoff` (20 Hz .. 20 kHz, log) and `fx_saturator_filter_res` (0 .. 1, Q 0.7 .. 21), on their own page `fx_saturator_filter`. A zero-delay-feedback state-variable filter at the base rate: Pre feeds the oversampler (cannot alias, dry path unfiltered), Post filters the wet path after downsampling; cutoff glides geometrically and resonance linearly over the 128-sample grid chunk and land exactly on the target. The four parameters append to `param_order.txt` (saturator module version 2). Measured on the output: post LP / BP / HP against the analytic state-variable response, worst deviation 0.000 dB (limit 0.5), LP peak at resonance 0 / .2 / .4 / .6 / .8 / 1 = -3.0 / 2.9 / 8.8 / 14.7 / 20.6 / 26.5 dB (monotonic); a pre LP at 1.5 kHz cuts the products above 10 kHz of a 2 + 9 kHz pair by 45 dB; the glide is 75x smoother (third difference) than a hard switch and the module matches the reference glide to 7e-9; CPU 0.23 % (off), 0.33 % (pre / post) of a core at 48 kHz stereo, Standard. Tests: `Saturator filter`.
- `fx.shaper`, a user-drawn waveshaper (maintainer request, distortion gap 3; new module, additive: existing sound, goldens and bit-exact renders unchanged, `param_order.txt` only appended `fx_shaper_on / mix / drive / output / dc / os` and `fx_pos_shaper`; `kNumModuleOnlyFx` 19). The transfer curve is 2..64 points (x in -1..1 ascending, y in -1..1) with a curvature per segment (`y = y0 + (y1 - y0) t^(4^curve)`) and an optional symmetric (odd, mirrored, positive half drawn) mode. API: `crucible_set_shaper_curve(engine, instance_id, points, count, symmetric, error, size)` / `crucible_get_shaper_curve` / `CrucibleShaperPoint { x, y, curve }` (C), `Engine::setShaperCurve` / `shaperCurve`, `ShaperPoint`, `makeShaperCurveTable` (C++); NULL points with count 0 restore the identity. Published to the audio thread as an immutable table (copied in at a block boundary, 10 ms cross-fade, no allocation), pulled view `fx.shaper/curve` (257 points), saved in patches as `"shaperCurves"` (additive, no format version bump, save -> load -> save bit-stable). Identity curve at 0 dB drive is bit-exact pass-through (delayed by the reported latency). Drive -24..+24 dB, output, mix, DC block, 1x..8x oversampling (latency reported). Measured: hard-clip curve against `std::clamp` 3e-8, 64-point curvature curve against the analytic shape 9e-8, curve view 7e-8; latency measured = reported within 0.5 samples at every factor and quality; zipper on drive / output / mix at most -95.9 dB at blocks 1, 7, 512; curve change step -130 dB; alias of a 5 kHz hard clip at +12 dB drive -14 / -29 / -39 / -61 dB at 1x / 2x / 4x / 8x; CPU 0.22 % (Draft 2x), 0.61 % (Standard 4x), 2.2 % (High 8x) of a core with a 64-point curved curve. Tests: `FxShaper`, `FxShaperRace`, `crucible_capi_test`.
- fx.cabinet (maintainer decision, distortion gap 6; new module, opt-in, default off, existing sound bit-identical; no recorded impulse
  responses): guitar / bass speaker-cabinet simulation. Eight cabinets (1x12 open back, 2x12 open back, 4x12 closed, 1x10 combo, 2x10
  open back, 1x15 bass, 4x10 bass, 8x10 bass), each generated from a parametric model (`include/crucible/fx/CabinetModel.h`): box
  resonance (second-order high-pass with Q, ported boxes add a lower corner), cone breakup peaks and dips, high roll-off, back-wall comb
  notches for open backs, microphone position (centre to edge: high-frequency tilt and a dust-cap notch that moves down) and distance
  (proximity bass, one early room reflection). The impulse response is the minimum-phase filter of that magnitude (cepstral folding),
  at most 2048 taps at 48 kHz (scaled with the rate, at most 8192), run through the zero-latency partitioned convolver (latency 0,
  reported). Built on a per-instance builder thread (like `fx.convolution`), primed on the live input and crossfaded over 40 ms; offline
  (High) renders wait for the builder and are bit-identical run to run. Controls: cabinet, mic position, mic distance, low cut, high
  cut (12 dB / octave, ramped), output dB, mix. Nine parameters flagged `appended` (after `fx_crush_fine`): `tests/golden/param_order.txt`
  gains nine lines, nothing moves; the chain has 39 slots (`kNumModuleOnlyFx` 19). Tests: `Cabinet` (model features, rendered features,
  zipper, reset, bypass, determinism, allocations, CPU).
- Optional state saturation for user formant tables (maintainer request; additive, off = bit-identical): `crucible_set_formant_saturation(engine, filter_slot, on, error, size)` (C), `Engine::setFormantSaturation`, `FormantTable::saturate` (C++), saved in patches as `"saturate": true` on the table entry. On, each formant's band-pass state is soft-clipped (softClip(x) = clamp(x, -3, 3) (27 + x^2) / (27 + 9 x^2)) and the low-pass state clamped to +-8, like a hardware-style vowel filter; the output stays the unity-peak band-pass sum. Needs a user table (the built-in vowels are never saturated, decided to keep them frozen); setting a table again turns it off; the `filterN/response` view stays the small-signal response. Rendered level of a hardware-style 5x4 table (+ 4.5) on a saw within 0.5 dB of the in-test reference at notes 36 / 48 / 60, cutoff 110 / 160 / 632, res .35 / .55 (FilterChannel and engine voice); note 60 @632 res .35 / .55 = -0.1 / -4.5 dB, note 36 @110 res .35 = -11.3 dB (legacy plug-in: +0.4 / -4.0 / -9.8). Tests: `FormantSaturation`, `crucible_capi_test`.
- fx.crusher note tracking (maintainer decision, distortion gap 5; opt-in, default off and bit-identical): `fx_crush_track` makes the hold rate note frequency x `fx_crush_ratio` (0.25 .. 16) x 2^(`fx_crush_fine` / 1200) (fine -100 .. 100 ct), gliding about 10 ms on note changes; no note yet leaves the fixed rate. The three parameters append to `param_order.txt` (crusher module version 2; new `ParamDef::appended` lists them after every regular parameter, and each graph instance such as `fx.crusher.2` owns its own). Generic note input for effects: `EffectInputs::note()` / `hasNote()` (MIDI note units of the engine's last started note, -1 before any), set with `ModuleRack::setNote`; the module SDK C ABI is unchanged. Measured on the output: tracked hold rate within 0.001 % of note x ratio (notes 48 / 60 / 72, ratios 8 .. 16, fine +-100 ct). Tests: `Crusher track`.
- Six shaper curves in the saturation library (maintainer decision, distortion gap 2; additive, appended at the end of every model list so
  existing choice numbers, patches, goldens and the fx baseline are unchanged): `Model::RectHalf`, `RectFull`, `SineFold`,
  `LinFold` (triangle fold with its first fold at |x| = 0.5), `ZeroSquare` (tanh(8x)) and `Asym` (tanh above zero, 0.4 tanh(x / 0.4)
  below), library indices 10 to 15. All have closed-form antiderivatives and run with the first and second order ADAA like Soft;
  the rectifiers and Asym make DC, removed by the model's DC blocker. `fx.saturator` model choices 10 to 15 (Rectify Half, Rectify
  Full, Sine Fold, Linear Fold, Zero-Square, Asymmetric) with their own drive ranges and oversampling-switch drives, aliasing at most
  -90 dB (Standard) / -100 dB (High) over drive 3 to 36 dB at 5 and 7 kHz; `fx.drive` modes 19 to 24, second order ADAA, one
  octave more oversampling. No new parameters (`tests/golden/param_order.txt` unchanged). Tests: `Saturation shapes`.
- `fx.multiband_drive` (maintainer request, distortion gap 1; additive, every existing sound, golden, bit-exact and fx baseline unchanged):
  three-band distortion on the existing LR4 crossover (`fx/Crossover.h`, minimum phase, flat sum) with, per band, the ten saturation models of
  `fx.saturator`, drive 0 to 36 dB, output -24 to +6 dB, mix, solo and bypass (`fx_mbdrive_{low,mid,high}_{model,drive,output,mix,solo,bypass}`),
  and the global `fx_mbdrive_on / _mix / _xlow / _xhigh / _output` (24 parameters with the chain slot `fx_pos_mbdrive`). At 0 dB drive the
  bands are not shaped: the output is the crossover's all-pass sum delayed by the reported latency (null -148.7 dB against the crossover
  alone, magnitude flat within 0.0000 dB, 20 Hz to 20 kHz); a 100 Hz tone driven only in the low band reads -9.3 dB THD against
  -147.4 dB with only the mid and high bands driven. Anti-aliased on the saturator's own path (half-band oversampling, second-order
  ADAA, flattening filter; 4x Standard / 8x High, the high band one stage more): alias -92.5 dB Standard and -113.8 dB High over the models
  (tape -86.2 / -97.8, held to the saturator's -85 / -97 exception). The wavefolder's drive range is a third of the control
  (the saturator's is a half). Latency 26 / 49 / 55 samples (Draft / Standard / High), the same for every setting, reported. Controls glide per
  sample (two 10 ms one-poles, exact landing as in `fx/Smoothing.h`), the crossover frequencies move over about 50 ms; zipper at most -90 dB
  (or the held floor + 3 dB) at host blocks of 1, 7 and 512; output independent of the host block size to -112 dB (128-sample grid; the crossover's SIMD tails differ by an ulp). CPU, stereo 48 kHz,
  three bands at 12 dB: 0.93 % Draft, 2.0 % Standard, 3.6 % High of a core (budget 1.3 / 2.5 / 4.5 %; the crossover alone is 0.33 %, idle bands cost nothing).
  Registered last (`kNumModuleOnlyFx` 19 to 20 after fx.shaper; the chain-slot range of the module-only effects is now 1..40); `tests/golden/param_order.txt`
  appended only (+24 lines). Shared pieces of the saturator path moved verbatim to `src/modules/fx/SaturatorParts.h`. Tests: `MultibandDrive`;
  numbers in `docs/MEASUREMENTS.md` section 25.
- User vowel tables for the Formant filter (maintainer decision; additive, the built-in vowels and old patches are bit-identical):
  `crucible_set_formant_table` / `CrucibleFormant { freq_hz, bandwidth_hz, gain_db }` (C), `Engine::setFormantTable` / `formantTable`,
  `FormantTable` / `makeFormantTable` (C++). 2..8 vowels x 1..4 formants per filter slot, shared by all voices; NULL restores the
  built-in vowels. The cutoff moves along the vowels as before (spread over vowels - 1 segments); frequency and bandwidth blend
  geometrically, gain linearly in dB. Each formant is a unity-peak two-pole band-pass summed in parallel, scaled by 10^(gain_db/20)
  and the table's `output_gain`; with a bandwidth Q = max(freq / bandwidth * 0.5 * 16^resonance, 0.3), with 0 the built-in
  resonance-derived damping. The table is published to the audio thread like user wavetables (no allocation or lock) and the
  `filterN/response` view follows it. Tests: `FormantTable`, `FormantTableEngine`.
- Per-mip user wavetable zones (approved; additive, one-zone tables and old patches are bit-identical):
  `crucible_load_wavetable_zones` / `CrucibleWavetableZone` (C), `UserWavetable::fromZones`, `UserWavetableBank::loadZones` (C++).
  A table takes 1..16 zones, each a full set of source frames plus `max_hz` (strictly ascending); each of the 27 mip levels is
  band-limited from the zone containing the level's representative fundamental (geometric centre of the notes that read it), so
  pitch-dependent formants survive across the keyboard at no audio-thread cost. One gain normalises the whole table. Zoned
  tables are rebuilt on the control thread when the engine is prepared at another sample rate. The `oscN/wavetable` view and
  the spectral view show the zone of the lowest mip level.
  An oversampled read (FM, ring, sync, warps, Quality High) of a zoned table never uses a finer mip than the plain read of
  the same note, so it keeps the note's zone instead of landing on a lower one (it used to brighten the tone and drop its level).
  Factory and one-zone tables are unchanged.
- `crucible_kill_all_voices` / `Engine::killAllVoices` (panic, [any], queued with the note input): every voice fades out in
  about 5 ms; held-note stack, sustain pedal, arpeggiator keys and sequencer note state are cleared. Python `kill_all_voices`.
- GUI read-back (approved generic API additions; no DSP change, goldens bit-exact, `param_order` untouched):
  - E1: pulled view `oscN/wavetable` (slots 1 to 8): the frame count, then 64 frames of 256 points, frame-major, of the table the
    slot selects (factory or user; empty for an empty user slot). Size via `crucible_view_size`. Spectral tables are not covered.
    The live frame position is the existing `params/oscN_pos.mod` (also `.voice` and `.band`), so no new position view.
  - E2: `crucible_route_count` / `crucible_route_get` read the modulation route list (`CrucibleRoute`). Routes are indexed in
    creation order; `crucible_remove_route` shifts later routes down, and the pair (source, target) is the stable key. Changes
    are reported as `CRUCIBLE_CHANGE_ROUTES` (-3) in `crucible_param_changes`; `Engine::kRoutesChanged` and
    `ModRouteStore::generation` back it in C++. Python: `Engine.routes()`.
  - Python package 0.2.0 with the new declarations.
- Python package `python/crucible` (ctypes over the C API, NumPy): offline rendering, parameters by id or text, patches, products,
  `reset`, latency and tail; `CRUCIBLE_ENGINE_SHARED` builds `libcrucible` exporting only the `CRUCIBLE_API` functions; tests,
  `python/examples/render_patch.py`, `pyproject.toml` (name is a placeholder, nothing published).
- FX10 delay: `FractionalDelay` (Kaiser-windowed sinc fractional read, polyphase table built at prepare, modulatable) and `Bbd`
  (bucket-brigade core: 256-4096 stages, clock tracking, anti-alias and reconstruction filters, compander with noise, soft clip).
- `fx.delay_pro`: digital and analog (BBD) delay with 26 parameters (`fx_delay_pro_*`): tempo sync, saturating feedback, TPT low and
  high cut in the loop, stereo / ping-pong / cross feedback, modulation, diffusion, ducking from the input, freeze, and time change by
  glide or crossfade. Latency 0. Registered last, so existing parameter ids and goldens do not move; `tests/golden/param_order.txt`
  is appended only.
- FX13 dynamics (part 2): `fx.transient` (level-independent attack and sustain, plus or minus 15 dB, lookahead up to 5 ms, clip guard)
  and `fx.deesser` (band or high-pass detector 2-16 kHz, relative or absolute threshold, split-band through a linear-phase FIR or
  wide-band reduction, lookahead up to 2 ms, listen, link). Both registered last; `tests/golden/param_order.txt` appended only.
  Numbers in `docs/MEASUREMENTS.md` section 20. 
- FX13 saturator: `fx.saturator` (ten models: soft, hard, rail, diode, wavefolder, triode, push-pull, pentode, tape, transformer; drive,
  bias, tilt pair, loudness-matched auto gain, mix, output; second-order ADAA with half-band oversampling by quality and drive; 10
  parameters). `fx.drive` is now legacy (`supersededBy` `fx.saturator`), unchanged and bit-exact. `Saturation.h` gains an optional
  `adaaOrder` (default 1, so every earlier user, including `fx.exciter`, is bit-exact). Alias rows -90 / -100 dB (Standard / High) for
  every model but tape; maintainer decision: tape is held to -85 / -97 dB across the whole drive range (meets -90 / -100 at
  full drive). Output independent of the host block size (grid of 128 samples). Registered last; `tests/golden/param_order.txt` appended only. Numbers in `docs/MEASUREMENTS.md` section 24.
- FX13 exciter: `fx.exciter` (harmonic exciter, High mode above 1 to 10 kHz, Low bass enhancer below 40 to 250 Hz; anti-aliased
  rail saturator with asymmetry, 0 to 9 dB drive, 2x oversampling in Standard and High, latency reported; 9 parameters). Registered
  last; `tests/golden/param_order.txt` appended only. Numbers in `docs/MEASUREMENTS.md` section 22.
- FX11 modulation: `fx.ensemble` (BBD chorus / ensemble, 1 to 4 lines, Dimensional modes, 14 parameters), `fx.phaser2` (2 to 12 all-pass
  stages, linear / OTA / JFET, barber-pole, envelope or sidechain sweep, 18 parameters) and `fx.flanger2` (through-zero, digital or
  BBD, feedback of either sign, 16 parameters, fixed latency of 249 samples at 48 kHz, see `docs/MEASUREMENTS.md` section 21.3).
  Registered after `fx.deesser`; `tests/golden/param_order.txt` appended only. `fx.chorus`, `fx.dimension`, `fx.phaser` and `fx.flanger`
  are now legacy (superseded by the new modules, bit-exact as before); `docs/products/template.json` uses `fx.ensemble`.

### API
- Review fixes, waves G-H (pre-v1, breaking where noted):
  - `host_id` is the 32-bit FNV-1a of the parameter id with the top bit cleared (31 bits): never in VST3's reserved range, never
    `0xFFFFFFFF`; the compiler's collision check runs on the masked value (breaking: ids change).
  - `struct_size` is the first field of `CrucibleParamMeta`, `CrucibleModuleMeta` and `CrucibleViewInfo`: callers set it, the engine
    fills only the fields that fit, and a size of 0 fails the call (breaking). A `CrucibleTransport` with `struct_size` 0 means "no
    transport", without error. The ABI paragraph lists every `struct_size` struct.
  - `crucible_create_product_modules` is removed (use `crucible_create_from_product`).
  - `crucible_reset` is tagged [audio]: it allocates and locks nothing and may run between process calls. `crucible_tail_seconds` is a
    cached [any] read; a tail change is reported by `crucible_param_changes` as `CRUCIBLE_CHANGE_TAIL` (-2) after the latency entry.
  - `setParamRt` / `crucible_process_events` round int, choice and bool values to the nearest whole number, as `setParam` now does.
  - `crucible_view_read` is [any] for pushed views and [main] for pulled ones (the pulled views are listed). `CrucibleParamEvent`
    events must be sorted by position, and a position is clamped to the block.
  - Examples, `docs/GETTING_STARTED.md`, the skeleton and `tools/wasm-probe` follow the new signatures.
- V1 API package (pre-v1, breaking where noted):
  - `crucible_save_patch` and `crucible_describe_json` write into a caller buffer `(char* buf, size_t cap)` and return the needed
    size, snprintf style (`buf` may be NULL); no engine-owned strings remain. Every C function is tagged [main] / [audio] / [any]
    in `crucible.h`, with a thread table at the top; the MIDI sort requirement and `CRUCIBLE_MAX_EVENTS_PER_BLOCK` are documented.
  - Host plumbing: `crucible_param_changes` (lock-free change list with `CRUCIBLE_SOURCE_HOST` / `_MIDI` / `_INTERNAL`, and the
    `CRUCIBLE_CHANGE_LATENCY` pseudo entry), `crucible_latency_samples`, `crucible_tail_seconds`, `crucible_reset`, with C++
    equivalents on `Engine`.
  - `CRUCIBLE_API` export macro on all C functions (`CRUCIBLE_STATIC`, `CRUCIBLE_BUILD_SHARED`).
  - `crucible_load_wavetable` (wavetable from memory), `CrucibleWavetableInfo` (`frames_in`, `frames_used`, `decimated`, `frame_size`) on both
    wavetable loads, slot errors; `crucible_add_route_ex` with `CrucibleRoute` (polarity, curve, smoothing).
  - `Engine::setParam` / `getParam` / `setParamText` / `modulatedValue` and `ParameterModel::indexOf` take `std::string_view`.
  - `examples/instrument_skeleton` uses all of the above; `docs/GETTING_STARTED.md` lists what a wrapper still does itself.
- Patch format version 3: the response of every `fx.convolution` instance is saved (`convolutionIrs`, keyed by instance id, e.g.
  `fx.convolution.2`); version 2 patches with a single `convolutionIr` migrate to the instance `fx.convolution`. `Patch::convolutionIr`
  becomes `Patch::convolutionIrs`. Presets saved with `referenceAssets` keep every instance's channels in the asset store
  (`assetRefs` entries of kind `ir` gain `instance`).
- Pulled views `filter/response` and `filterN/response` (256 log-spaced points, 20 Hz to 20 kHz, dB) and `.../response_mod` (from the
  `.mod` values): the small-signal magnitude of the selected filter model for the current parameters; `filterResponseDb()` in
  `Filters.h`, `AnalogFilter::responseDb()`, `SynthEngine::baseValue()`.
- `crucible_view_size(e, view)` returns a view's payload capacity in 32-bit elements. `describeJSON` lists the views (a top-level
  `views` array with kind, size, stride, live and owning module) and each module's `views`.
- Product surface (G1b): `crucible_create_from_product(config, json, modules, error, size)` and `Engine::fromProduct(json, error)`
  build an engine from `crucible-product` JSON. The layout-only `crucible_create_product` is removed;
  `crucible_create_product_modules` was an alias of the new call and is removed (see the review fixes below).
- A product declares its host surface in `"surface"`: `sections`, `pages` (each with `params` ids) and `expose`
  (`"listed"` or `"all"`, default `"all"`). With `"listed"`, only the listed parameters are exposed to a host; the others still
  load from patches and remain modulation targets. `crucible_surface_count` / `crucible_surface_param` (and `Engine::surface()`)
  walk the exposed parameters in declared order. `CrucibleParamMeta` gains `exposed`.
- Every parameter has a stable `host_id` (32-bit FNV-1a of its id) in `CrucibleParamMeta`; a collision between two ids is a
  compile-time error that names both.
- Choice indices are global: `oscN_type` keeps every built-in type at its number in every product. Types a product omits are
  reported by `crucible_choice_available(e, index, k)` (and `Engine::choiceAvailable`) as unavailable; a slot set to an omitted
  type is silent, as before. Patches keep the numbers and also accept the choice text (`"osc1_type": "Analog"`).
- Modules carry a status: `describeJSON` and `CrucibleModuleMeta` report `status` (`current` or `legacy`) and `supersededBy`.
  Legacy modules stay registered and bit-exact for existing patches: `fx.reverb` -> `fx.reverb_algo`, `fx.conv` -> `fx.convolution`,
  `fx.comp` -> `fx.comp_clean`, `fx.gate` -> `fx.gate2`, `fx.eq` and `fx.peq` -> `fx.eq_pro`, `fx.delay` -> `fx.delay_pro`.
- `crucible_load_ir(e, instance_id, path, err, err_size)` and `crucible_clear_ir(e, instance_id)` load and clear an impulse
  response per module instance, so a second `fx.convolution` instance can take its own IR. They replace
  `crucible_load_impulse_response_file`, `crucible_load_convolution_ir_file` and `crucible_clear_convolution_ir`, which are removed.
- `docs/products/template.json` is a lean starter product of current modules only, and the README has a "which module do I use"
  table with the module status.
- `crucible_set_param_text(e, id, text)` sets a parameter from its display text (C; C++ equivalent on `Engine`).
  `crucible_param_index_by_host_id(e, host_id)` returns the parameter index for a host id, or -1 if none.
- Choice text matches case-insensitively and also without a trailing parenthetical: `"analog"` and `"Analog"` both match
  `"Analog (VA)"`. An ambiguous short form is refused, not guessed.
- `examples/instrument_skeleton/` is a C example with no GUI toolkit and no audio device. It walks the whole plug-in lifecycle
  (create from a product, build a host parameter list, process events and transport, service on the main thread, print
  views, save and reload a patch, load an impulse response) and runs as the ctest `instrument_skeleton` (exit 0).
- `docs/GETTING_STARTED.md` is a one-page guide to the lifecycle, the thread table, the state recipe and the skeleton. The
  README "State serialisation" section is rewritten (`crucible_save_patch` is the container) and links the guide.

### Tooling
- `tools/ci.sh` runs every requested configuration even after a failure and ends with a pass / fail summary; the exit
  status stays non-zero if any configuration failed.

### Changed
- `Bbd` reconstruction filter: four complex modes instead of a 20-tick FIR (exact at constant clock, about -78 dB from the old output).
  `fx.delay_pro` Analog output changes by that amount; the module is unreleased. See `docs/SOUND_CHANGES.md`.

### Performance
- CPU gap tuning (G3): analog filters in graph `filter.analog` nodes and the classic voice's non-bank fallback run the block path
  instead of the per-sample one (bit-identical at constant controls; see `docs/SOUND_CHANGES.md`). Graph voice pairs share a
  four-lane filter, so a stereo graph filter costs 0.28 % (transistor ladder), 0.33 % (diode ladder) and 0.17 % (Sallen-Key LP)
  per voice, down from 1.13, 1.31 and 0.92 %. Batched and single-lane output agree to 5e-7.
- The CPU checks (convolver test, `crucible-measure`) measure thread CPU time at user-interactive QoS, so they stop failing when
  other applications load the machine; the limits are unchanged. The `crucible-measure` CPU window covers only `Engine::process`.
- Measurements are recorded in `docs/MEASUREMENTS.md` section 18 (profile of "1 note, saw" and of the low-CPU convolver).

### Fixed
- Review fixes: (1) the analog RC envelope no longer clicks when the attack curve is changed or modulated during the attack (the charge is re-derived from the current level; curve 0 is bit-identical); (2) `crucible_load_wavetable_zones` steps the zone array by the caller's `struct_size`, so a caller built against a larger zone struct reads every zone right; (3) a saved zoned user wavetable with more than 2^20 frames in a zone now reloads (the unpack limit is 2^24, the most the C API loads); (4) the linear-phase convolver no longer leaks a kernel set when the mailbox's retired slots are full during a length-class change (the new set waits a block instead).
- fx.convolution (mix, out) and fx.clipper (mix, ceiling, drive, shape) no longer zipper at host blocks of 1, 7 and 512: the controls are smoothed per sample (three one-poles of about 4 ms; the clipper guard ceiling follows a moving ceiling with 3 dB of margin). Constant-parameter output is bit-identical.
- Fuzz findings F3, F5, F6: `crucible_save_patch` applies pending `seq_length` / `seq_pattern` work (as `crucible_service` would), so a patch saved after parameter events from the audio thread is consistent; the `osc1_warp_src` default is clamped to the oscillator count (one-oscillator layouts reported 1 for a 0..0 choice); user wavetables restore their saved frames verbatim, so save / load / save is bit-exact (patch format unchanged).
- Algorithmic reverb (fx.reverb_algo): every continuous parameter (size, predelay, density, decay, mod depth, mix, width, duck, early, the shelves and cuts) now glides instead of stepping per block or chunk (cascaded smoothing, per-sample early-reflection delay / weight and all-pass gain glides, interpolated line reads while a delay moves), so fx.reverb_algo meets the -90 dB zipper limit; constant-parameter output is bit-identical.
- Dynamics core and VCA comp console drive: threshold, ratio, knee, range, makeup, mix, the sidechain filter corners and the console drive now glide (about 6 to 20 ms) instead of stepping per block, so fx.comp_clean, fx.comp_vca and fx.gate2 meet the -90 dB zipper limit; constant-parameter output is bit-identical.
- `max_voices` 57..64 no longer overruns the per-voice arrays (crash with MPE or poly pressure once the steal spares were used).
- An infinite or huge host tempo or position (`crucible_process_host` / `_events` / `_aux`) no longer hangs the sequencer: tempo is clamped to 1..1000 BPM (non-finite means unknown), a position beyond 1e12 beats or non-finite is unknown, and a block visits at most 4096 steps.
- Fuzz findings: `crucible_add_route` refuses a non-finite amount and `ModRouteStore::add` stores non-finite amount / curve /
  smoothing as 0 (F4); imported sample values (`crucible_load_sample`, `crucible_add_zone_sample`, file loaders) are clamped to
  -16..16 (F7, ordinary audio unchanged); the half-band decimator and oversampler even / odd stores are written as if / else
  because GCC 13.3 with UBSan miscompiled the ternary-array form (T1, no change in results).
- Clicks at note start: a mono / legato retrigger of a still-sounding voice no longer re-randomises the oscillator phases
  (the waveform stays continuous under the envelope's From Level restart), and a voice stolen when every voice is busy
  fades out (about 5 ms) before the new note starts instead of being cut.
- Deferred items of the waves D-F review: quad control rate no longer leaks between voices sharing a filter quad (A6); per-voice dynamic routes apply amount modulation (A8); a filter FM source keeps its bus buffer (A9); reverb re-enable during the spread-out clear no longer zeroes all memory in one callback (C6).
- Review of waves G-H, effects (slice B). These modules are unreleased, so their
  output may change; goldens and other modules stay bit-exact, `param_order.txt` is untouched.
  - `fx.phaser2`: barber-pole pair weights sum to at most unity gain; the JFET and OTA stages are passive one-ports (output
    `a - 2y` with `y` the saturated current, JFET bias replaced by a 1 : 2 asymmetric saturation), so a stage never exceeds unity
    gain and the loop is bounded for every stage count, type, barber-pole setting and feedback of +-0.95.
  - `fx.deesser` and `fx.transient` publish the lookahead latency from `prepare` / `reset` (constant across enable / disable);
    the de-esser FIR moves to a new design through two 3 ms poles per tap.
  - Zipper-free parameter changes (below -90 dB at blocks of 1, 7 and 512): exciter gain and mode change, ensemble spread /
    crossfeed / norm / mix, BBD drive gain (ensemble, flanger2, delay_pro) and transient output gain.
  - `fx.delay_pro` freeze is bounded for 10 minutes of audio (loop gain guard, inactive on a loop that holds or decays).
  - The threadless (WASM) pump never runs inside `process()` (`fx.convolution`, `fx.eq_pro`); native behaviour unchanged.
  - Tests at 44.1 / 48 / 96 / 192 kHz for every new effect, and zipper tests on every automatable parameter.

- A factory wavetable switched with `setParamRt` (host automation or a mapped CC) before `service()` is no longer generated on the
  audio thread: the oscillator is silent until `service()` has generated the table (C13). `SynthEngine::prepare` generates the
  tables the oscillators start on, so the first `process()` of a prepared engine allocates nothing.
- Poly aftertouch at the same sample as its note-on is applied to the new note (it was ordered before the note-on and lost) (C7).
- A product with `layout.oscillators = 8` is created (C11): `kMaxVoiceTargets` is raised from 512 to 640 because 8 oscillators
  need 557 per-voice targets and 8 is the documented maximum; parameter order and goldens are unchanged. `Engine::fromProduct`
  reports a parse, compile or model failure through `error` instead of throwing; the message names the count and the limit.
- Effects-side fixes from the wave D-F review; renders and goldens are bit-identical.
  * EqPro Linear phase never started in real time (C1): `PartitionedConvolver::adopt()` takes the posted kernel before the path
    switch. A Linear Resolution change now ducks through the switch, and the dry delay and reported latency follow the wet path
    (C3). Offline posts are never lost (`LatestRequestSlot`), and `prepare()` drops the old-rate kernel (C4, C5). Because live
    Linear mode now depends on design-thread timing, `fx_ep_phase` is excluded from the audit's sound comparison, like `fx_conv_on`.
  * The reverb's bypassed low cut tracks the wet signal instead of freezing its state, which left a DC offset (C2).
  * `DynamicsProcessor` keeps its latency-known flag across `reset()`, so a latency change made after a reset is still announced (B3).
  * `fx.clipper`: the second-order ADAA fallback threshold scales with the input level; the difference quotient of F2 cancelled
    (errors up to 6 % on the plateau at a 1000x crest) with the old absolute threshold (B5).
  * Checked, not defects: the maximiser skip bound (B4) and the clipper's dry-path alignment at 2x (B6; the HalfBand oversampler's
    x.5 latency makes the total whole).
- CPU regression since FX4 (the `1 note, saw (all threads)` row of `tools/refbench.sh` read 0.679 % against a limit of 0.662 %):
  the master analyzer's `LoudnessMeter` ran the K-weighting per sample with a three-deep dependency chain and its 4x true-peak
  interpolator waited on a 12-step accumulator and four divisions per sample, and `SynthEngine::setParameterValues` and
  `stepSmoothers` called `log2` / `exp2` for every log-smoothed parameter in every block. The K-weighting now runs a block at a
  time with both channels side by side and a one-step recursion, `TruePeakDetector` (4x) unrolls its default filter and
  evaluates the parabolic refinement of the four points in one vector step, and the transcendental calls repeat only when their
  argument changes. Same results: renders and goldens are bit-identical, `TruePeakDetector` output is bit-identical, the loudness
  readings agree with the earlier ones to double rounding. The row reads 0.59 %.
- Generic modulation destination numbers are assigned module by module: the per-voice unison targets that a module adds
  after the fact (`lateVoiceMod`) follow that module's own targets instead of the end of the table, so a module
  registered later no longer moves them. Numbers of modules added since the last snapshot changed once (no release
  has shipped); `tests/golden/param_order.txt` freezes the result.
- `Engine::modulatedValue(id)` resolves the id in the engine's own product (it used the process default model, so a product
  that omits an effect read the wrong macro).

### Added
- Modulator bank, modulation of modulation, audio-rate bus and filter FM (P12). Parameters are appended by two modules registered last
  (`mod.bank`: `bankN_type`, `_scope`, `_in_a`, `_in_b`, `_mode`, `_amount`, `_rate`, `_rise`, `_fall`, `_steps`, `_seed`, `_pt01` .. `_pt16` for N = 1..8;
  `mod.audio_rate`: `lfoN_audio`, `_audio_hz`, `_audio_ratio`, `filterN_fm_src`, `filterN_fm_amt`); existing ids, indices and generic destination numbers
  are unchanged (frozen in `tests/golden/param_order.txt`) and the goldens are bit-identical.
  * `Modulators.h`: eight modulators with route sources `kSourceModulator + n` (4000 + n): Combine (add, multiply, min, max, crossfade, difference), Remap
    (16-point curve over the input range), Lag (rise and fall time constants), Quantise, S&H, Turing (register of 1-16 steps, seeded, reproducible), Binary,
    Chaos (logistic map, Lorenz system), Note gate (gate, velocity, trigger), Follower (the voice's oscillator mix or an input), Pitch tracker, LFO table
    (eight morphing frames). Evaluated once per control chunk in index order; an input can only name a ModSource or an *earlier* modulator, so no cycle can
    form. Scope voice (state per voice, restarted by its note-on), layer or global (one state in the engine, shared; the classic voice is one layer).
    A modulator no route reads is not evaluated, so an idle bank costs nothing. `ModSlotParams` sources of voice routes are now extended indices
    (`extSourceIndex`), `evaluateSlot()` has an overload that takes the source's natural polarity.
  * Modulation of modulation (`Modulation.h`): a route to `route:<n>:amount` (`routeAmountTargetId(n)`) adds its output to the amount of route *n* (clamped to
    -1 .. 1), to any depth; `ModRouteStore::add()` refuses a route that closes a cycle (or targets itself), the compiled `RouteSet::amountMods` is in
    dependency order and drops a cycle that only appears after a removal shifted the positions. New pseudo target `voice:amp`.
  * Audio-rate bus (`Voice.h`): up to eight mono buffers per voice (`kAudioBusBuffers`) carrying oscillator outputs (`kSourceAudio + 4 + slot`), the audio-rate
    LFOs `lfoN_audio` (`AudioLfo.h`: sine, triangle, saw, square, saw down, 25 % pulse, band-limited with the fine minBLEP of the new `FineBlep.h`, which
    `osc.analog` now shares) and per-sample modulation, mod and extra envelopes. Routes from these to oscillator pitch (`oscN_semi`, `voice_transpose`),
    `oscN_an_xmod_amt`, `oscN_an_pw`, `oscN_xmod_amt`, `oscN_pos`, a filter cutoff or `voice:amp` run per sample; any other target gets the chunk mean (an
    oscillator without per-sample input too). `OscRender::pitchMod` / `fmAmountMod` / `widthMod` and `OscillatorInstance::supportsAudioRate()`:
    `osc.analog` takes them (PWM and FM from another slot, closing P9's gap); the wavetable oscillator takes pitch, cross amount and position in groups of four
    samples. `AnalogFilter::process(io, n, cutoffOctMod)` now samples the cutoff every four samples (was 16) while it is modulated, also through `FilterBank`
    (`setCutoffMod`); the other filter models re-derive their coefficients every four samples (`FilterUnit::processBlock`); `filterN_fm_src` / `_fm_amt` are
    the same without a route. At most eight distinct audio sources are in use at a time; `add()` refuses a ninth.
  * Graph: `PortType::AudioRate` inputs accept only audio outputs (explicit audio-rate edges, the edge amount scales the signal). New inputs
    `filter.analog.cutoff_fm`, `filter.svf.cutoff_fm`, `amp.vca.am`, and `pitch_fm` / `width_mod` / `fm_mod` of the oscillator nodes; new node types
    `lfo.audio` and `osc.analog`.
  * Fixed with it (`docs/SOUND_CHANGES.md`): an edge of `osc.analog` that falls exactly on a sample boundary (a rational frequency, 440 Hz at 48 kHz) was
    cancelled one sample off; a 440 Hz pulse measured -22 dB of non-harmonic energy and now -119.6 dB. No recorded scenario is affected.
  * CPU: an unused bank costs nothing (a modulator no route reads is not evaluated); one modulator on the plan's 16-voice analog voice adds 0.15 % of a core, eight add 0.45 %;
    an audio-rate LFO on the ladder cutoff adds 1.5 %, per-sample PWM from another slot 2.5 %, per-sample pitch FM 5.7 % (`crucible_engine_bench mod`). The control-cell step now visits only
    the smoothed parameters, so an idle engine is faster than before even with 232 more parameters (1.4 against 1.8 microseconds per 32 samples).
  * Tests `tests/ModulatorTests.cpp`, `tests/AudioRateModTests.cpp`, `tests/ParamOrderTests.cpp`; `crucible_engine_bench mod`; numbers in `docs/MEASUREMENTS.md` section 15.
- Analog envelopes, voice cards, noise floor and hum (P11): the module `voice.analog` (registered last, parameters appended) adds per envelope
  `amp_` / `menv_` / `env3_` / `env4_` `mode` (Digital, the default and bit-identical, or Analog RC), `retrig` (From Level, Reset, Legato, ADR),
  `overshoot`, `vel_time` and `key_time`. `AnalogEnvelope.h`: the RC attack charges toward 1 + overshoot and reaches the peak at the attack time (to one
  sample), decay and release are exponential (the time set is the time to fall within 1 %), a sustain change moves through the RC; Reset fades the old
  level out over 0.5 ms (click -84.8 dBFS), Legato ignores a note-on while the key is held, ADR has no sustain; velocity and key scale the times. `VoiceCard.h`: a
  voice card is a function of the engine seed and the voice index (unit-variance draws, bounded) and carries part-to-part tolerances for the oscillators (`osc.analog` now
  takes its tolerance draws from it, and keeps drawing from the note seed when the voice has no card), the analog filter models (`FilterSettings::tolerance` and their
  seed), the envelope times and the amplifier gain: `voice_card_osc`, `_filter`, `_env`, `_level` (0 = no card). `OscNoteInfo::card`, `NodeIO::card`,
  `NodeNote::card` and `NodePrepare::card` carry it through the voice and the graph (null = legacy); `OscillatorInstance::usesVoiceCard()`. An optional seeded noise floor and mains
  hum (`voice_noise_on` / `_db`, `voice_hum_on` / `_db` / `_hz`, -120 to -60 dBFS RMS, off by default) is added to the output after the master gain. Graph node types `env.analog` and `env.adr`;
  the `amp.vca` and `filter.analog` nodes and the oscillator wrappers use the card. `tests/AnalogVoiceTests.cpp`, `crucible_engine_measure envelope` (analog table), `crucible_engine_bench plan`
  (the analog-voice rows are now measured), `docs/MEASUREMENTS.md` section 13. Existing parameter indices, goldens and the voice-quad batching are unchanged.
- Algorithmic reverb suite (FX6): module `fx.reverb_algo` ("Algorithmic Reverb", appended after every existing module, so no earlier
  parameter id, index or modulation destination moved; the legacy `fx.reverb` is untouched) with the algorithms Room, Hall, Chamber, Plate
  and Ambience, built on new cores in `include/crucible/fx/Fdn.h`:
  * `Fdn`: Jot's feedback delay network with 16 lines (32 at High quality), prime base lengths spread geometrically and scaled by size,
    a Hadamard (Hall, Chamber) or Householder (Room, Ambience) feedback matrix, and in every line a two-section first-order shelving
    absorption filter designed from the line's own length so that the reverberation time is the set one at DC, in the middle band and at
    Nyquist (low / high decay multipliers with their crossover frequencies); Kaiser-windowed 8-tap sinc reads (table scaled so no phase
    has a gain above one), smooth random modulation of every line length (excursion proportional to the line, so the pitch smear does not
    depend on the algorithm), and an energy guard that holds a frozen network at the level it had.
  * `EarlyReflections`: image-source shoebox taps with velvet-noise polarity for Room and Chamber, jittered velvet-noise taps for Hall and
    Ambience, none for Plate; the tap tables are built in `prepare()` and played back with chunked linear reads.
  * `PlateTank`: Dattorro's figure-eight all-pass tank (his published lengths, two cross-coupled branches with modulated all-passes) with
    the same absorption filter; its round trip shortens automatically so that it stays within a fifth of the decay time.
  * `AlgorithmicReverb`: pre-delay, an 8-stage all-pass input diffuser whose output also forms the dense onset of the tail, density,
    early level, size, decay with low / high multipliers and crossovers, modulation depth / rate, width, wet low / high cut, ducking, freeze,
    mix. Changing the algorithm fades the wet signal out over 20 ms, clears the state and fades the new algorithm in (no click above -60 dB).
    `deactivate()` clears the state 65536 samples per call (the review-fix pattern); a clear that is not finished when the effect is
    switched on again is completed first. Tail length and "infinite while frozen" are reported through `LatencyReporter`.
  Parameter ids `fx_ra_*`, 21 parameters on four pages, default chain slot after the FX3 / FX4 modules. Measurements in
  `docs/MEASUREMENTS.md` section 15; tests in `tests/ReverbAlgoTests.cpp`. Existing registration-order checks follow the appended module
  (`CloudDrawbarTests`).
- Convolution v2 with true IR import (FX7). `fx.convolution` ("Convolution", registered last; no existing id or index moves, the legacy
  `fx.conv` is untouched and stays bit-identical): non-uniform partitioned convolution in `include/crucible/fx/PartitionedConvolver.h`
  (`NonUniformConvolver`, `ConvolverPlan`): zero-latency mode with a 64-sample direct head and 32 / 128 / 1024 / 8192-sample partitions,
  or low-CPU mode with 128 samples of reported latency and 64 / 512 / 4096-sample partitions; large partitions are computed across blocks
  so each block does even work. The kernel changes crossfade (no tail cut). Both modes reach the same accuracy against a direct double-precision convolution.
  * IR import (`include/crucible/fx/IrImport.h`, `src/fx/IrImport.cpp`): WAV (`AudioFile`) and AIFF / AIFC (own reader) of any rate,
    up to 30 s, with 1, 2 or 4 channels (mono, stereo, or true stereo as LL, LR, RL, RR). Resampling is a Kaiser-windowed sinc with
    140 dB stopband, exact polyphase tables for integer rate ratios.
  * IR edit (`include/crucible/fx/IrEdit.h`, `src/fx/IrEdit.cpp`): trim start, fade, length, reverse, stretch by resampling, early / late
    balance, low / high cut, pre-delay and width. The module receives the imported response through the new capability
    `ConvolutionIr` (`include/crucible/module/ConvolutionIr.h`). Edits and kernel builds run on a per-instance builder thread; the audio
    thread does not allocate.
  * Engine and C API: `Engine::loadConvolutionIrFile`, `Engine::loadConvolutionIr`, `Engine::clearConvolutionIr`, and
    `crucible_load_convolution_ir_file` / `crucible_clear_convolution_ir` in the C API. Patches save the response as the optional
    `convolutionIr` field (name, rate, layout, base64 channels); patches without it load as before, and the legacy `impulseResponse`
    field is unchanged.
  * Vendored FFTConvolver helpers (`third_party/fftconvolver/`, MIT, pinned to `f2cdeb04`): `AudioFFT.h` and `Utilities.{h,cpp}`. The
    adapter `src/fx/AudioFftAdapter.cpp` implements `audiofft::AudioFFT` on `crucible::Fft`; the bundled Ooura / FFTW / Accelerate FFT
    is not compiled. The FFTConvolver and TwoStage classes are not vendored: their one-shot tail blocks cannot spread work, so the
    scheduler is ours.
  16 parameters (`fx_cx_*`, `fx_pos_cx`) are appended to `tests/golden/param_order.txt`; `fx.convolution` is in `products/crucible-default.json`.
  Measurements in `docs/MEASUREMENTS.md` section 17; tests in `tests/ConvolutionTests.cpp`, `tests/PartitionedConvolverTests.cpp`,
  `tests/IrImportTests.cpp`, and the engine IR load test in the engine suite.
- Voice quad path (`FilterBank.h`): voices whose filter slot holds a nonlinear model (`FilterModel` 17-24) now run their filters four lanes at a
  time through `AnalogFilter::processQuad` instead of one sample per call per voice. `Voice::renderChunk` is split into `beginChunk` /
  `filterIn` / `filterOut` / `endChunk` (the voice's own filters give the old `renderChunk`); `SynthEngine::renderClassicVoices` drives the stages and
  the bank (voice 2p and 2p + 1 share one four-lane object per slot and model, fixed lanes, no copying). New `AnalogFilter::resetLane`,
  `EngineConfig::batchAnalogFilters` (default on; off renders every voice alone). Per mono lane at Standard, 8 and 16 voices: 0.43-0.60 % before,
  0.045-0.154 % after (plan targets met within 20 %); a stereo voice runs two lanes. Batched equals voice by voice within 1e-5 of the peak
  (`tests/VoiceQuadTests.cpp`); `./crucible_engine_bench analog` prints the table. Legacy models 0-16 and all goldens are bit-identical.
- Adopted and new engines (P13). Oscillator types 6, 7 and 8 (appended; types 0 to 5 and every existing parameter keep their
  indices and ids), also graph node types:
  * `osc.modal` ("Modal"): a bank of up to 64 state-variable modes (poles placed exactly from the frequency and the T60, so both are
    exact at 44.1, 48 and 96 kHz) or a comb string (loop delay and gain corrected at the fundamental), played by a strike, pluck,
    particles, blow, noise or bow exciter; geometry (stiffness), brightness, pickup position, decay, exciter contour / timbre /
    sustain, stereo spread, mode count and voice-card tolerance. Designs after Mutable Instruments Rings and Elements (MIT,
    `third_party/mutable/`, THIRD_PARTY.md); modes stay below 0.40 fs and the exciters are band limited.
  * `osc.harmonic` ("Harmonic"): a harmonic layer on the cloud renderer: partial count (1 to 128), odd / even balance, stretch, tilt,
    a formant bell fixed in Hz and per-partial decay.
  * `osc.spectral` ("Spectral"): the slot's sample through Signalsmith Stretch (MIT, `third_party/signalsmith-stretch/`): speed 0.25x to
    4x independent of the pitch, keytracked or fixed transposition, formant shift, start and loop; primed at note start so the output has
    no latency. Heavy state is built on the control thread by the new `OscillatorInstance::arm()` when a slot first selects the type (0.8 MB per
    voice and slot for the stretcher; `Engine` arms on `oscN_type` and in `prepare`), so engines that never use it pay nothing; an unarmed
    instance is silent. `ProductLayout::omit` can still drop the type.
- Pro EQ and crossovers (FX5). `fx.eq_pro` ("Pro EQ", registered last; no existing id or index moves, the legacy `fx.eq` and `fx.peq` are
  untouched): sixteen bands (bell, low / high shelf with 6 or 12 dB per octave, tilt, band-pass, notch, all-pass, low / high cut at 6, 12,
  18, 24, 36, 48 and 96 dB per octave as Butterworth, Bessel, Linkwitz-Riley or resonant), each on stereo, left, right, mid or side, with
  optional dynamics (gain follows the key level above or below a threshold, on the gain computer of the FX3 dynamics core, internal key
  or the host's auxiliary input), three phase modes (minimum, natural, linear), automatic output gain, parallel mix, a `response` view
  (the analytic curve) and a `spectrum` view (output spectrum) through P6, and latency reported through `LatencyReporter`. New library
  headers under `crucible/fx/`:
  * `AnalogMatchedFilters.h`: second-order sections whose magnitude follows the analog prototype to 20 kHz at every sample rate. Matched-z
    poles, numerator from the magnitude identity of Vicanek's matched second-order filters (DC, centre and Nyquist), then a few
    Levenberg-Marquardt steps against the prototype between 20 Hz and 20 kHz (a "smooth" variant with the Nyquist gain pinned while a
    parameter moves, and a phase-weighted "natural" variant); cuts start from the inverted boost design; double-precision coefficients and
    direct-form-1 states (`BiquadChain`, coefficients move linearly across a block); a TPT state-variable filter for audio-rate modulation.
    The legacy pre-warped bilinear bell loses 8 dB of a +12 dB, Q 1 bell at 16 kHz at 44.1 kHz; the matched one 0.2 dB.
  * `LinearPhase.h`: symmetric FIR kernels from a target magnitude (frequency sampling on a 4x finer grid, raised-cosine taper), uniformly
    partitioned stereo-input matrix convolution on `Fft.h` (kernels of 1k to 32k taps, latency centre + block reported), kernel sets handed
    from a worker thread to the audio thread without locks or allocation (`KernelMailbox`; the offline High quality waits for the worker
    so renders are deterministic).
  * `Crossover.h`: Linkwitz-Riley trees of 2 to 8 bands at 6, 12, 24, 36 and 48 dB per octave with an all-pass compensation of every band
    (flat sum for any spacing), and a linear-phase crossover whose bands telescope to the delayed input.
  `EffectModule`s can implement the new `ResponseView` capability (`crucible/module/Module.h`); `kNumModuleOnlyFx` is 5.
  Tests: `tests/EqProTests.cpp`, `tests/CrossoverTests.cpp`; numbers in `docs/MEASUREMENTS.md` section 8.7. Registration-order checks in
  `tests/DynamicsTests.cpp` follow the appended module.
- Spectral tools (`crucible/Spectral.h`): spectral frame interpolation that sweeps the peaks in frequency instead of crossfading (peak
  matching at analysis time on a control thread, frequency-axis warping and a level match at render time), six spectral warps (shift, stretch,
  blur, comb, dispersion, tilt) and an allocation-free cycle synthesiser. Opt-in per oscillator slot through `oscN_wt_interp`,
  `oscN_wt_spec` and `oscN_wt_spec_amt` (module `voice.spectral_wt`, registered last); with the defaults the legacy wavetable code runs
  unchanged. User wavetables carry their analysis; factory tables are analysed on first use by `Engine`.
- FFT backend behind `crucible/Fft.h`: pffft (`third_party/pffft/`, BSD-like licence) is the backend on platforms without Accelerate
  (Apple keeps Accelerate, so recorded renders are unchanged); `-DCRUCIBLE_ENGINE_FFT_BACKEND=PFFFT|PORTABLE|ACCELERATE`;
  `Fft::forwardPffft`, `Fft::backendName`; benchmark `crucible_engine_fft_bench [--check]` (adoption rule: at least 1.5x the portable code).
- `ParameterMap::kMaxVoiceTargets` is 512 (was 256): five oscillator slots with the new per-voice parameters exceed 256.
- Tests: `tests/EnginesTests.cpp`; measurements in `docs/MEASUREMENTS.md` section 12. Existing tests that counted oscillator types or assumed
  the last registered module were updated to follow the appended modules (`OscillatorModuleTests`, `AnalogOscillatorTests`,
  `AnalogFilterSkTests`, `CloudDrawbarTests`, `DynamicsTests`).
- Maximiser, clipper, loudness and true-peak metering (FX4), appended after every existing module so no parameter id or index moves
  (`kNumModuleOnlyFx` is now 7; the legacy `fx.limiter` is unchanged). `crucible/fx/Loudness.h` (`LoudnessMeter`): ITU-R BS.1770-4
  K-weighting for any sample rate (the printed 48 kHz coefficients to 1e-7), momentary, short-term and integrated loudness with the
  absolute and relative gates, loudness range (EBU Tech 3342), sample and 4x true peak per channel (evaluated only where a sample
  can reach the decaying peak), plus `PpmMeter` (types I and II), `VuMeter` and `CorrelationMeter`; it is now also what
  `MasterAnalyzer` runs. `fx.maximizer` (`fx_mx_*`, `crucible/fx/Maximizer.h`): lookahead 0.5 to 10 ms true-peak limiter whose detector is a
  32-tap 4x interpolator on the sidechain, ceiling in dBTP held by construction (running maximum over the hold window, then a box
  average as long as the lookahead), hold adapted to the period of the bass, fixed, dual-stage or crest-adaptive release, stereo link
  and a mid / side bias, an optional ADAA soft-clip stage that takes a share of the peak reduction, an optional 4x oversampled audio
  path, TPDF or noise-shaped dither to 16, 20 or 24 bits, a loudness readout with a target-loudness gain suggestion, latency and gain
  reduction reported. `fx.clipper` (`fx_clp_*`, `crucible/fx/Clipper.h`, `ClipperEffect.h`): one curve that morphs from a hard clip to
  tanh, first- or second-order antiderivative anti-aliasing with a droop equaliser behind it, 1x to 8x oversampling (Auto follows the
  quality policy), a true-peak guard and a latency-aligned parallel mix. `util.meter` (`fx_mtr_*`): passes the audio through unchanged
  and measures it. New capability `MeterReadout` and engine views `<module>/loudness` and `<module>/levels`. `TruePeakDetector` takes a
  tap count (`configure(factor, refine, tapsPerPhase)`), `skip()` and `gainBound()`. libebur128 (MIT) is vendored under
  `third_party/libebur128/` for tests only, as the oracle of `tests/LoudnessTests.cpp`. `tests/MaximizerTests.cpp`; numbers in
  `docs/MEASUREMENTS.md` sections 8.5 and 8.6.
- Second filter family (P8): `FilterModel` 20 to 24 (`Sallen-Key LP`, `Sallen-Key HP`, `OTA SVF`, `OTA 4-Pole`, `Passive HP`; indices 0 to 19
  unchanged). Sallen-Key with a diode clipper in the feedback (asymmetric: the limit cycle carries even and odd harmonics, unity pass band,
  oscillation from Ka = 3, `character` = clipper hardness); OTA state-variable filter (Q 0.5 to 20, never oscillates, `character` morphs low-pass /
  notch / high-pass); four-pole OTA with a saturating resonance VCA and 15 % built-in compensation; passive high-pass of one or two exact RC sections.
  Same interface, solver tiers, oversampling, pitch correction (the Sallen-Key table is measured at both ends of `character`), tolerance and
  `processQuad` as P7. New parameters `filter_character`, `filter_hp` (and `filterN_...`), registered last by the module `voice.filter_analog`, a
  passive high-pass step (60 / 180 / 540 Hz) ahead of the Sallen-Key LP, SVF and 4-pole; `FilterSettings::highpass`; `filter.analog` graph node
  lists the new models and gains `hp`. `tests/AnalogFilterSkTests.cpp`, measurements in `docs/MEASUREMENTS.md` section 11.
- Instance-scoped products: `ModuleRegistry` and `ParameterModel` are owned by the engine instance
  (`EngineConfig::registry` / `EngineConfig::product`), so several products with different layouts run in one
  process. `ModuleRegistry::instance()` and the static `ParameterMap::` functions remain as a deprecated shim.
- Self-describing parameters (`shortName`, `group`, `polarity`, `uiHint`, `textFormat`, `steps`, `routable`,
  `automatable`), module ports, views and capabilities; `valueToText` / `textToValue` with the standard formatters
  (`ParamText.h`); `describeJSON` (`Introspection.h`); C API `crucible_create_product`, `crucible_engine_param_*`,
  `crucible_value_to_text`, `crucible_text_to_value`, `crucible_module_*`, `crucible_describe_json`.
- Per-module patch migrations (`ModuleMigration`, `ModuleRegistry::migrate`, `Patch::migratedParams`).
- Graph in data (P4): product definitions as `crucible-product` JSON (format version 1, `crucible/graph/Product.h`),
  a graph compiler with validation and configurable limits (`crucible/graph/Compiler.h`: 32 nodes per voice, 64 FX
  instances, 16 layers, 64 modulators), a node interface (`crucible/graph/Node.h`: prepare / reset / noteOn / noteOff /
  process / optional processQuad / latency) and node types that wrap the existing DSP unchanged (`classic.voice`, the
  oscillator modules, `env.adsr`, `lfo.basic`, `filter.svf`, `amp.vca`, `mix.out`).
  `EngineConfig::compiled` plays a compiled product; `products/crucible-default.json` is today's product as data and
  renders bit-identically to an engine without one. Example products: `drive-trio`, `fx-only`, `drum-kit`.
- FX graph: `ModuleRack` runs any number of effect instances in lanes (serial, in position order or as listed), in
  parallel, with sends, from the voice bus or the host input (`process_aux`); further instances of an effect get the ids
  `fx{k}_...`, and a product can pin ids with `idMap`. The default product reproduces today's position-ordered chain.
- Layers: note and velocity ranges, round-robin groups and choke groups select small per-layer graphs (drum kits, splits).
- Runtime reconfiguration: `SynthEngine::reconfigure()` swaps in a program compiled with `compileReconfiguration()` while
  audio runs (published through `RtShared`, no allocation or locks on the audio thread, 10 ms fade down and up).
- Saturation library (`crucible/Saturation.h`): soft, hard, rail, diode and foldback curves with first-order
  antiderivative anti-aliasing, tube stages from Koren's equations (triode, push-pull, pentode), Jiles-Atherton tape
  hysteresis and a flux-saturating transformer; `fx.drive` gains ten appended modes (the nine existing ones are
  unchanged); a per-voice shaper stage.
- Install and export: `find_package(CrucibleEngine)` with the `crucible::engine` target, version file and installed
  headers; a consumer test project.
- `crucible/Version.h`, `crucible_version()` and `--version` on the command line tools, generated from the CMake version.
- MIT licence, SPDX headers, `THIRD_PARTY.md`, `PROVENANCE.md`, contribution and conduct documents, issue templates.
- Continuous integration workflow (macOS arm64, Linux gcc and clang, Windows MSVC, sanitizers, scalar fallback).
- `examples/c_api/render_c.c`: a complete C program that renders a note to a WAV file.
- `osc.analog`, the virtual analog oscillator (oscillator type 5 of every slot, `oscN_an_*` parameters; the existing types
  and parameter ids and indices do not move): saw, pulse with PWM, triangle, sine, saw + pulse mix and a continuous shape
  morph; double-precision phase; minBLEP steps and minBLAMP corners at exact sub-sample positions (wraps, moving pulse
  edges, triangle corners, sync resets, through-zero FM crossings) on a 256-per-sample table that is lag-aligned so edges
  land on their exact positions and carry no DC; a half-octave crossfade into the one-harmonic table above fs/4; a 5 Hz DC
  blocker on the pulse; characters Ideal, Saw-core VCO (curved ramp, finite reset, comparator pulse, soft triangle),
  Triangle-core VCO (switched saw, tanh sine) and DCO (exact pitch, saw level falling with pitch, divider sub); hard and soft
  (reversing) sync, through-zero linear FM, exponential FM and ring modulation against an internal modulator at a pitch
  ratio; up to eight unison voices; Ornstein-Uhlenbeck drift per voice (time constant 0.5 to 5 s, standard deviation
  drift x 6 cents, default 0) and voice-card tolerance. `tests/AnalogOscillatorTests.cpp`.
- Dynamics core and three new effect modules (FX3), appended after every existing module so no parameter id or index moves
  (`kNumModuleOnlyFx` is now 4; the vocoder keeps its default slot): `crucible/fx/Dynamics.h` (`DynamicsProcessor`: the
  quadratic soft-knee gain computers of Giannoulis, Massberg and Reiss for downward and upward compression, expansion,
  gate and ducker; peak, RMS and hybrid detectors with an optional 4x true-peak reading; sidechain high-pass, low-pass and
  bell; attack and release as times to 63 % of a step of gain, hold, a dual-stage program-dependent release; lookahead up to
  10 ms through `Lookahead` and reported as latency; feed-forward or feedback; stereo link and every `ChannelMode`; parallel
  mix, makeup and auto makeup), `fx.comp_clean` (`fx_cc_*`), `fx.comp_vca` (`fx_cv_*`: ratios 1.5 / 2 / 4 / 10, attacks
  0.1 to 30 ms, releases 0.1 to 1.2 s or auto, a gain cell with a slight third harmonic, optional transformer output stage)
  and `fx.gate2` (`fx_g2_*`: gate, expander and ducker with hysteresis, hold and lookahead). The external key is the host's
  auxiliary input (`process_aux`); without one the internal key is used. Each module implements `LatencyReporter` and
  `GainReductionMeter`, and the engine publishes a `gain_reduction` scalar view per module. The legacy `fx.comp` and
  `fx.gate` are unchanged. `TruePeakDetector` now evaluates its phases together (readings equal to float rounding, no refinement branch
  misprediction). `tests/DynamicsTests.cpp`.
- Analog filter interface and the ladder family (P7): `crucible/filters/FilterModel.h` (`AnalogFilter`: `prepare`,
  `reset`, `setControls(cutoff, resonance, drive, character, tolerance)`, `process(io, n, cutoffOctMod)`, `processQuad`
  for four voices in lock step) and three nonlinear zero-delay-feedback models: a transistor ladder (feedback 0 to 4.2,
  self-oscillation at 4, bass loss `1 / (1 + k)`, `character` gives it back), an OTA cascade (keeps more bass, clean
  self-oscillation, optional four-step input high-pass) and a coupled diode ladder (tridiagonal stage coupling, about
  18 dB/octave). Solver tiers follow the quality policy (Draft 1x, Standard 2x, High 4x with Newton to 1e-6), a drive
  shaper runs at twice the filter rate, a per-model pitch-correction table is measured on first use, and `tolerance`
  adds a seeded component spread. The models are appended to `FilterModel` (17 Transistor Ladder, 18 OTA Cascade,
  19 Coupled Diode Ladder), so `filter_type`, `filter2_type`, `fx_filter_model` and the other filter choices list them;
  indices 0 to 16 and every existing render are unchanged. `FilterChannel` preallocates all three models (call
  `prepare(rate, quality)`; the voices do), `FilterSettings` gains `drive`, `character` and `tolerance`.
- GUI-agnostic read-back (P6, `crucible/Views.h`): generic views identified by `{instance, id}` with scalar, vector,
  points and mask payloads. The audio thread publishes subscribed views at most once per render call into a lock-free
  multi-reader buffer (an atomic word for scalars), readers on any thread copy a consistent snapshot with its sequence
  number, subscriptions are made on the control thread with a rate limit, and a view nobody subscribed to costs one atomic
  load per render call. Built-in views: `master/meter`, `loudness`, `scope`, `spectrum_<n>`; `engine/sounding_notes`,
  `active_voices`, `vector_position`; `sequencer/step`, `running`; `clips/positions`; per parameter
  `params/<id>.mod`, `.voice` (newest voice), `.band` (lowest, highest, active voices) and `.voice@<n>` (a chosen voice);
  per oscillator slot `sample_*` and `cloud_*`. The old accessors (`meter`, `loudness`, `MasterAnalyzer`,
  `modulatedValue`, `voiceModulatedValue`, `clipPositions`, `sampleView`, `cloudView`, `soundingNotes`) stay and return the
  same values as their views. `Engine::views()`; C API `crucible_view_count`, `_info`, `_find`, `_subscribe`,
  `_unsubscribe`, `_read`, `_read_mask`. `MasterAnalyzer::copyScope`, `fftLength`, `spectrumOf`.
  `tests/ViewTests.cpp`.
- Module SDK and a stable C ABI (P5). `crucible/crucible_module.h` is plain C modelled on CLAP's conventions: versioned,
  append-only structs that start with `size` and `abi_version`, UTF-8 strings, a descriptor with id / name / kind /
  version / CPU class, a parameter table and ports, and `create`, `destroy`, `prepare`, `reset`, `process`, `note_on`,
  `note_off`, optional `process_quad`, `get_view`, `latency` and `holds_voice`; a library exports `crucible_get_modules`.
  A module is an effect (an FX graph instance type, always in place on the stereo bus) or a generator, filter or
  modulator node of the voice graph. The host side registers modules (`crucible_register_module`,
  `crucible_load_module_library`; `crucible::sdk::ModuleSet` in C++), refuses a module built for another ABI version
  with a clean error and nothing registered, and loads shared libraries only when the host allows it and only from paths
  it names (the engine never scans directories). `crucible_create_product_modules` plays a product JSON with the
  registered modules. External modules appear in `describeJSON` and introspection like built-in ones.
  `crucible/sdk/Module.h` is the C++ layer (`crucible::sdk::Module<T>` generates the descriptor,
  `CRUCIBLE_SDK_EXPORT_MODULES` the entry point). Every built-in module now registers through the same function,
  `sdk::registerModule`. `NodeRegistry::addEffect()` lets a product instantiate SDK effects in its FX graph.
- `crucible-module-check`: the real-time harness for modules (`crucible/sdk/Check.h`): allocation counter, extremes
  sweep over parameter minima, maxima and seeded corners, NaN / infinity and denormal input, flush-to-zero independence,
  determinism per seed and after `reset()`, CPU budget by class. `--builtin` checks every built-in module; the exit code
  distinguishes a failed check (1) from a library that cannot be loaded (2).
- `examples/third_party_module/`: a C effect and two C++ nodes (generator, filter) built as shared libraries with no
  engine dependency, plus a build of the C module that declares a future ABI version for the loading tests.
- `filter.analog`, a graph node for the analog filter models (transistor ladder, OTA cascade, coupled diode ladder) with
  drive, character, tolerance and keytrack (follow-up of P7).
- Real-time parameter path (G1a): `Engine::setParamRt(index, value)` writes a parameter from the audio thread (clamped, atomic
  store, no allocation or lock); `Engine::service()` runs the control-thread work that such writes leave pending (wavetable
  generation, oscillator arming, sequencer length sync). `setParam` still does the work at once, so offline tools need no
  `service()`. Mapped MIDI CCs in `Engine::process()` go through `setParamRt`, so they no longer allocate on the audio thread.
- C API: `crucible_process_events(e, L, R, n, midi, nmidi, params, nparams, const CrucibleTransport*)` applies
  `CrucibleParamEvent {position, index, value}` at block start and takes the host transport; `crucible_service(e)` is the
  control-thread call. `CrucibleConfig` starts with `size_t struct_size` (0 or smaller means the defaults) and adds
  `preload_wavetables` and `oversampling_filter`; `crucible_config_init` fills the defaults.

### Changed
- Sallen-Key and OTA SVF: the four-sample coefficient block restarts when new controls are set (`LadderBase::onRestart`), so controls apply at once
  and a lane joining a running quad finds a valid coefficient set. Only affects blocks of odd length between control changes.
- Oscillator factories and `SlotParams` take the product's `ParameterModel`; `ModuleRack`, `ModRouteStore`, `SceneBank`
  and `CcMapStore` take the model / registry of their product (null selects the process default product).
- The separate product-test executable is gone: product tests run in `crucible_engine_tests`.
- The project started as the engine of two hardware-controlled instruments and was extracted into its own repository
  under the name Crucible Engine: namespace `crucible`, include directory `<crucible/...>`, C API `crucible.h` with
  `crucible_*` symbols and `CRUCIBLE_*` macros, targets `crucible_engine` / `crucible::engine`, `crucible-render`,
  `crucible-measure`. The environment variables are `CRUCIBLE_UPDATE_GOLDEN` and `CRUCIBLE_UPDATE_BITEXACT`.
- Patches are written with format `crucible-patch`, arpeggiator patterns as `crucible-arp-pattern` and preset files
  with the extension `.crucible-patch`. Files written under earlier project names are not read.
- The core ships a neutral product layout (`crucible.default`). It changes page order only; parameter ids and indices
  are unchanged, and every golden render and bit-exact hash is identical to the engine before the rename.
- `KeyBus` is an abstract bus with an in-process `local()` instance; cross-process transports are the host's job.

### Fixed
- Legacy effects (FX2, bugs BFX1-BFX7 of the FX plan; sound changes in `docs/SOUND_CHANGES.md`): the drive's DC blocker
  corner is 5 Hz at every sample rate (it cut 40 Hz by 2.5 to 12 dB); the multiband's low band passes the high crossover's
  all-pass, so the bands add up flat; delay, reverb and convolver start empty when switched on again; effects crossfade
  over 10 ms when switched on or off (`ModuleRack`, every module); the limiter's lookahead changes through
  `fx::Lookahead` without a dropout; multiband crossovers glide instead of jumping; the frequency shifter's Hilbert
  filters are designed for the sample rate in `prepare()`.
- `filter.svf` allocated the analog filter models on the audio thread when one of the types 17 to 19 was selected; the
  node now prepares them with the other nodes.
- `fx.crusher` did not restart its jitter / dither noise on `reset()`, so a reset instance rendered differently from a
  new one (found by the module check). Renders after `prepare()` are unchanged.
- Review fixes after waves B and C (no default sound changes; goldens unchanged):
  - `reconfigure()` while an earlier switch was still fading in could leave the newest program held and never applied;
    a pending program now always gets its own fade.
  - Pulled views (sample info / peaks / slices / zones, cloud points) read control-thread objects from the caller's
    thread; they now read through `RtShared::withLatest()` (safe from any thread, no voices touched), and
    `SynthEngine::sampleSnapshot()` is the thread-safe part of `sampleView()`.
  - `SynthEngine::activeVoiceCount()` read audio-thread state from any thread; it returns a count the audio thread
    publishes (after each render and note event).
  - Module SDK: the "version 1 minimum" struct sizes are fixed numbers (end of the last version 1 field), so structs that
    gain fields in a later ABI version still accept version 1 modules. Parameter id collisions (a parameter named `on`
    against the host's bypass switch, two ids of one module, or ids that flatten to the same name in two effects) are
    refused at registration with `CRUCIBLE_MODULE_ERR_DUPLICATE`; `ModuleRegistry::add` refuses duplicate parameter ids.
    SDK voice nodes are prepared with a seed per instance instead of the constant 1 (`NodePrepare::seed`).
  - Windows: module libraries load with `LoadLibraryExW` from an absolute path, searching only the library's own
    directory and the default system directories (untested on Windows).
  - `osc.analog` kept NaN in its phase for good after a NaN pitch; non-finite pitches are now clamped.
  - `fx.delay` and `fx.reverb` cleared several megabytes inside one audio block when switched off (192 kHz); the clear is
    spread over blocks (256 KiB per call), and completed at once if the effect is switched on again before it ends.
  - `fx.multiband` redesigned twelve biquads per sample while a crossover moved; it redesigns every 16 samples and
    moves the coefficients linearly in between (automated crossovers: 29.2 -> 23.3 us per 256-sample block at 48 kHz,
    static 17.8; crossover step still -124 / -114 dB).
- Review fixes after waves D to F, synth side (no default sound changes; goldens unchanged):
  - `route:<n>:amount` targets with `n` past `kMaxRoutes` or past the route list are refused (and dropped on publish)
    instead of indexing past the list.
  - The spectral oscillator's frame cache holds four frames, so a pending SIMD unison run no longer loses frames it still
    reads; oscillator types and tables that only stored scenes or the vector pad select are armed.
  - `osc.analog` kept a `thread_local` scratch that allocated on the first audio-rate render of a thread; the scratch lives
    in the instance. Hard / soft sync crossings use the modulated master step (phase and steps clamped), and FM / ring
    modulation scale the master step by the mean pitch multiplier under audio-rate pitch.
  - Batched filter lanes draw their component values from the voice card's filter seed, so `voice_card_filter` sounds the
    same through the filter bank, the single path and `filter.analog` (`AnalogFilter::setLaneSeed`, `FilterBank::seedVoice`).
  - A non-finite input no longer latches the Lag and Follower modulators at 0.
  - The modal oscillator keeps its prepare rate in a member the audio thread never writes (data race with `arm()`).
- C parameter calls that take the engine are renamed without aliases: `crucible_engine_param_*` is now `crucible_param_*`.

### Removed
- Product-blind parameter helpers: the C calls without an engine argument (`crucible_param_count()`,
  `crucible_param_index(id)`, `crucible_param_info()`, `crucible_param_choice()`) and `CrucibleParamInfo`; C++ static
  `Engine::paramCount`, `Engine::paramIndex` and `Engine::paramInfo`. Ask the instance instead
  (`crucible_param_*(engine, ...)`, `Engine::model()`).
- Product-specific code left the core: the hardware page layout, a wire codec, the
  shared-file key bus and the plug-in host of the reference bench. They live with the applications that need them.
