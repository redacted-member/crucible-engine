# Plan: Crucible FX, effects at professional mixing and mastering quality

Status: the design plan written before implementation; where it and the code differ, the code and the reference
docs (docs/ARCHITECTURE.md, docs/MEASUREMENTS.md) are current.

Goal: the Crucible Engine's effects must reach the level of the best commercial mixing and mastering
plug-ins (the Waves, UAD, FabFilter, Soundtoys and Valhalla class of products).

This plan extends `ENGINE-COMMUNITY-PLAN.md` (read first; its maintainer decisions, ground rules, verification
method and package rules all apply here unchanged). Section and package references such as "2.7" or "P4" point
into that plan; FX packages are numbered FX1 to FX17.

**Paths.** Code is cited relative to the engine's repository root (`src/Fx.cpp`, `include/crucible/...`, namespace
`crucible`). New files in this plan are written with their final paths. Line numbers refer to commit `24f3bf9`.

---


> **Scope for v1 (maintainer decision):** FX1-FX7 are done; v1 adds only FX10 (delays / BBD), FX11 (modulation) and
> FX13 (saturator / exciter / transient / de-esser). FX8, FX9, FX12, FX14, FX15, FX16, FX17 and FXR are deferred past
> v1: the designs below stay as the reference for later work or community modules.

## 0. Ground rules for effects (in addition to the community plan's)

1. **Generic devices, never clones.** Modules model *circuit and device types* (a FET compressor, an optical
   compressor, a passive tube EQ, a tape machine, a bucket-brigade delay). Module ids, names, parameter names,
   preset names and UI strings never contain a manufacturer, product or trademark name, and never claim to be a
   model of a specific commercial unit. Standards are not trademarks and may be named (NAB / IEC tape EQ, ITU-R
   BS.1770, EBU R128, IEC 60268 meter ballistics, IEC 60386 wow and flutter).
2. **Clean-room.** Designs come from published papers, textbooks, public circuit analysis and standards. GPL code
   (ChowTapeModel, Surge's effects, Dragonfly, zita-rev1, the x42 meters, Vital) is studied only through its public
   papers, documentation and talks, never copied. Commercial plug-ins are never disassembled, and no presets,
   impulse responses, tables or resources are extracted from them. They may be **measured as black boxes** by a
   maintainer, on licensed copies, with the reference bench (section 5.3): the result is a band a target must fall into,
   never a sample match.
3. **Adoption.** Permissive code (MIT / BSD / zlib / Apache-2.0 / ISC / WTFPL / public domain) may be vendored under
   `third_party/<name>/` with its licence file and a `THIRD_PARTY.md` row, after the worker has read its
   `LICENSE` in the exact revision vendored. Licences named below are as publicly stated at the time of writing and
   are re-checked at adoption.
4. **Legacy effects stay bit-exact.** The 21 existing effect modules keep their ids, parameters and sound. New
   behaviour comes as new modules or as appended parameters whose default selects the legacy path. The only
   exception is the bug-fix package FX2, whose sound changes are recorded in `docs/SOUND_CHANGES.md` and
   whose golden updates the coordinator approves one by one.
5. **Every claim is a number.** Every quality statement in this plan has an acceptance test in the measurement
   harness (section 5). Nothing is judged by ear (`feedback-testing-without-sound`).

---

## 1. The quality bar: what makes professional effects sound professional

Each item is a property a measurement can check. Section 2 scores the current effects against it; section 3 gives
each new module its numeric targets.

| # | Property | What the best products do | How it is measured (section 5) |
| --- | --- | --- | --- |
| Q1 | **Correct harmonic spectra** | Each analog-modelled device produces the harmonic signature of its circuit type: single-ended triode (H2 dominant, H3 below it), push-pull and transformer (odd dominant, H3 rising at low frequency), tape (odd dominant, hysteresis memory), FET gain cell (H2 rising with gain reduction), VCA (very low THD) | THD and per-harmonic curves against level and frequency; even / odd split |
| Q2 | **Program-dependent behaviour** | Release that depends on how long and how hard the device was driven (optical cells, auto release), attack that depends on crest factor, saturation that depends on frequency (transformers, tape) and on history (hysteresis, thermal memory) | step-response families: release time against hold time and level; THD at 50 Hz against 1 kHz; hysteresis loop area |
| Q3 | **Anti-aliasing** | Oversampling sized to the nonlinearity, plus antiderivative anti-aliasing (ADAA) where an antiderivative exists, so even high drive leaves no inharmonic tones in the audible band | aliasing power and worst spur below 20 kHz for 5 kHz and 7 kHz tones at full drive; IMD (SMPTE 60 Hz + 7 kHz, CCIF 19 + 20 kHz) |
| Q4 | **Analog-matched linear filters** | EQ curves that do not cramp near Nyquist (matched or oversampled designs), proportional-Q and interactive curves where the circuit type has them | magnitude error against the analog prototype up to 20 kHz at 44.1 / 48 / 96 kHz |
| Q5 | **Smooth parameter response** | No zipper noise on any automated or modulated parameter; filter structures that stay stable and quiet under fast modulation; crossfades on discrete switches | zipper test (ramp and 2 Hz square automation on a sine; residual outside the expected sidebands); click test on choice changes |
| Q6 | **Clean on / off and bypass** | Latency-compensated crossfades for bypass, on / off and preset changes; tails that ring out or are cleared on purpose, never replayed | step at the switch; stale-buffer test |
| Q7 | **Low noise floor** | Silence in gives silence out unless a noise feature is switched on; optional modelled hiss is shaped, seeded and off by default; dither only where word length is reduced | idle output (exact zero or below -150 dBFS); shaped-noise spectrum when on |
| Q8 | **Latency reporting** | Every module reports its latency (oversampling filters, lookahead, linear phase, block convolution) and its tail length; the host compensates | measured cross-correlation delay equals the reported latency to one sample |
| Q9 | **True peak** | Limiters, clippers and meters work on inter-sample peaks (ITU-R BS.1770 true peak) | output true peak against the ceiling on an inter-sample-peak torture set |
| Q10 | **Oversampled detectors** | Detectors that see the true waveform (oversampled peak, true RMS with a correct time constant), not the sample values | detector reading against the analytic value for fs/4 sines at all phases |
| Q11 | **Stereo, mid / side, linking** | Every stereo processor offers linked / unlinked / mid / side / M-S dual operation and link amount | per-channel response tests; M/S null tests |
| Q12 | **Sidechain** | External and internal sidechain input, sidechain filters, listen mode | key-driven gain reduction tests |
| Q13 | **Lookahead** | Lookahead where it removes overshoot or onset clicks (limiters, gates, de-essers), reported as latency | overshoot and onset-residual tests |
| Q14 | **Gain staging and calibration** | A defined analog reference level (for example -18 dBFS = 0 VU), unity small-signal gain, auto gain (loudness-matched) for honest comparison | small-signal gain; loudness change across the drive range |
| Q15 | **Metering** | Gain reduction, input / output peak, RMS, LUFS (momentary / short term / integrated, range), true peak, correlation, spectrum, published lock-free | EBU Tech 3341 / 3342 test signals; true-peak reference |
| Q16 | **CPU** | Professional quality at a cost that allows dozens of instances | bench rows per module (section 3.10) |
| Q17 | **Accurate controls** | Labelled times, ratios, frequencies and decay times mean what they say (attack time to 63 % of the gain-reduction step, RT60 per octave band) | attack / release accuracy; RT60 (T30) against the set decay; static curve against the gain computer |

---

## 2. Assessment of the current effects

### 2.1 How the numbers were produced

A scratch program (not committed) linked `libcrucible_engine.a` (Release, Apple M-class,
2026-10-11) and drove the effect classes of `src/Fx.cpp` and `src/FxExtra.cpp` directly with test
signals, measuring with the P1 harness `tests/AnalogMeasure.h` (`harmonicProfile`, `measureAliasing`,
`steppedGainDb`, `responseFromImpulse`, `truePeak`, `rmsEnvelope`). 48 kHz unless stated, 32-sample blocks.
FX1 turns this into the permanent `crucible-measure fx` baseline with regression guards, so every number below
will be reproducible from the repository.

### 2.2 Headline

**The effects are clean, real-time safe and robust (the feature audit found no faults: finite, bounded,
allocation-free, denormal-free at every extreme), and several cores are good starting points. They are
"synth-effect" grade, not "mix and master" grade.** Measured against the bar in section 1:

* **Q3 aliasing fails at high drive.** `fx.drive` at Standard (4x half-band FIR) leaves -28 dB of alias power for a
  5 kHz tone at full drive with the default Soft model, -27 dB with Hard (and -56 dB already at amount 0.3);
  Fold and Sine reach +7 dB (aliases louder than the fundamental). The plan's rule is -90 dB (Standard).
* **Q9 true peak fails.** The limiter holds sample peaks at the ceiling (-0.30 dBFS) but lets an fs/4 sine at 45
  degrees out at **+2.71 dBTP**.
* **Q4 cramping.** The RBJ bells of `fx.peq` / `fx.eq` lose 6.2 dB of a +12 dB, Q 1 bell at 16 kHz (3.42 dB at
  20 kHz against 9.65 dB analog), 4.5 dB for a 12 kHz bell.
* **Four real bugs**, listed in 2.3: the drive's DC blocker cuts bass by an amount that depends on the sample rate;
  the multiband's band sum has a notch of up to -7.2 dB; switching a delay or reverb back on replays stale audio;
  on / off is a hard cut.
* **Q2 / Q1 character is missing.** No module has program-dependent release, hysteresis, frequency-dependent
  saturation or a modelled gain cell. Saturation curves are static functions. The reverb is one 8-line FDN; there
  is no plate, spring, BBD, tape or console model.
* **Q8 latency is never reported.** `EffectModule` (`include/crucible/module/Module.h:107-118`) has no latency
  or tail call. The drive aligns its own dry path (`Fx.cpp:118`, `:205-206`), but the limiter's lookahead (up to 5
  ms), the drive's oversampler on the wet path and the convolver's 128-sample block are invisible to a host.
* **Q16 CPU is fine except convolution.** Per instance (stereo, 48 kHz): drive 0.61 % (High 1.96 %), reverb 0.48 %,
  limiter 0.33 %, multiband 0.18 %, chorus 0.18 %, compressor 0.10 %, param EQ 0.10 %, delay 0.05 %; the
  convolver 3.2 % (built-in 3 s hall) and 4.3 % (4 s user IR), four times the community plan's 1 % per instance.
* **Q11, Q12, Q15 platform gaps.** No mid / side mode on any processor, one sidechain bus (the vocoder's
  `process_aux`), no external key for the compressor or gate, gain reduction published only by the multiband
  (`GainReductionMeter`), no loudness or true-peak meter.

### 2.3 Bugs found (fixed by FX2)

| Id | Bug (evidence) | Effect | Where |
| --- | --- | --- | --- |
| BFX1 | The drive's wet-path DC blocker has a fixed pole of 0.995, so its corner moves with the sample rate: at amount 0, tone 1 the drive cuts 40 Hz by 2.47 dB (44.1 kHz), 2.80 dB (48), 6.67 dB (96), 11.93 dB (192); 20 Hz by 6.1 to 17.7 dB | bass loss on every drive patch, worse at high rates | `src/Fx.cpp:225` |
| BFX2 | Multiband crossover: the low band is not passed through the high crossover's all-pass, so the band sum is not flat: -7.19 dB at 570 Hz with crossovers 500 / 750 Hz, -0.76 dB at 392 Hz with 300 / 1200; -0.05 dB with the defaults 200 / 3000 | a notch whenever the crossovers are less than about 3 octaves apart | `src/FxExtra.cpp:466-469` |
| BFX3 | Re-enabling a time-based effect replays the audio from when it was switched off: the rack calls `deactivate()` (`src/ModuleRack.cpp:93-97`), which only clears `primed` (`include/crucible/Fx.h:209` and siblings); delay lines, FDN lines and convolver buffers keep their contents | a burst of old echoes / reverb tail on switch-on | `Fx.cpp:410-416`, `:515-520`, `FxExtra.cpp:1001-1009` |
| BFX4 | On / off is a hard cut (no crossfade) | click at every switch while audio plays | `ModuleRack.cpp:93-101` |
| BFX5 | Changing the limiter's lookahead clears its buffers | a dropout of `look` samples | `Fx.cpp:770-779` |
| BFX6 | The multiband's crossover coefficients jump when the crossover parameter changes (no smoothing) | clicks when a crossover is automated | `FxExtra.cpp:430-449` |
| BFX7 | The frequency shifter's Hilbert all-pass coefficients are fixed (designed for 48 kHz, per the comment); their 90-degree band edges scale with the rate, so the low edge (about 20 Hz at 48 kHz) moves to about 40 / 80 Hz at 96 / 192 kHz (analytic, not measured) | image leakage for bass content at high rates | `FxExtra.cpp:667-668` |

Status: all seven fixed by FX2 (branch `feat/fx2-legacy-fixes`); `tests/FxFixTests.cpp` guards each, the numbers before and
after are in `docs/SOUND_CHANGES.md` and `docs/MEASUREMENTS.md`.

### 2.4 Per-effect assessment

Verdicts: **KEEP** (already at the bar for its role), **IMPROVE** (keep the module, fix or extend it), **REPLACE**
(keep the legacy module bit-exact for old patches, and supersede it with a new module that new products use).

| Module (file:line) | What it is | Measured / observed against the bar | Verdict |
| --- | --- | --- | --- |
| `fx.drive` (`Fx.cpp:111-233`, shapes `:142-166`) | 9 static shapers, 2x / 4x / 8x (`:115`), pre high-pass, one-pole tone, DC blocker, mix with an aligned dry path | Good: oversampled, dry path aligned to whole samples, mix smoothed per sample. Tube gives H2 -20.9 dB at amount 0.3 (a real even-order signature). Fails Q3 at high drive (above); no ADAA; static curves (no Q2); tone at maximum still -1.27 dB at 20 kHz (`:178`, `:223`); BFX1; model switch is instant (click); the gain compensation `gain^-0.35` (`:177`) is a heuristic, so loudness changes with drive | IMPROVE: FX2 fixes BFX1; P10 appends physical models; the new saturator (FX13) is the mix-grade module |
| `fx.comp` (`Fx.cpp:590-645`, module `modules/fx/Compressor.cpp`) | Feed-forward, sample-peak detector on a one-pole high-passed sidechain, soft knee, dB-domain attack / release | Static curve close to the computer (gain at 0 dBFS in: -12.4 dB, ideal -13.5 dB; peak vs RMS reading). Attack labelled 10 ms reaches 63 % of the gain-reduction step after **18.5 ms** (the peak detector only attacks near waveform peaks). With 1 ms / 50 ms the distortion of a 50 Hz sine is **-34 dB** THD (100 Hz -39 dB, 1 kHz -59 dB): no RMS option, no hold. No lookahead, no external key, no stereo-link control, no M/S, no gain-reduction meter, makeup stepped per block | REPLACE by the dynamics core (FX3): the legacy module stays as is |
| `fx.limiter` (`Fx.cpp:737-819`) | Lookahead min-hold plus box-car smoothing of the required gain, one-pole release, hard clamp | Sound core design (the min-hold / moving-average structure used by good limiters); sample peaks never exceed the ceiling. Fails Q9 (+2.71 dBTP); 60 Hz sine 6 dB over: -39 dB THD (fixed release, no hold or adaptive release); the per-sample min search is O(lookahead) (`:799-800`); lookahead capped at 511 samples (`:764`, 2.7 ms at 192 kHz); BFX5; latency unreported | REPLACE by the maximiser (FX4) |
| `fx.eq` (`Fx.cpp:648-734`) | 3-band RBJ low shelf / bell / high shelf | Cramping (Q4); shelf slope fixed | KEEP as a legacy simple EQ; FX5 is the pro EQ |
| `fx.peq` (`FxExtra.cpp:161-283`) | 8-band RBJ (double design), per-32-sample log-frequency glide, analytic `magnitudeDb` | Good: smooth sweeps, an analytic curve for GUIs. Cramping (6.2 dB at 20 kHz for a 16 kHz bell); cuts only 12 / 24 dB/oct; no dynamic bands, M/S, linear phase, analyser | KEEP (legacy); its magnitude API is the model for FX5 |
| `fx.multiband` (`FxExtra.cpp:403-491`) | 3 bands, LR4 crossovers, shared attack / release / ratio | BFX2, BFX6; no knee, no per-band times, no upward mode, no linear phase, no lookahead | IMPROVE (FX2 fixes BFX2 / BFX6), REPLACE by FX14 |
| `fx.gate` (`FxExtra.cpp:494-538`) | Peak gate, 6 dB hysteresis, hold, range | No lookahead (onset click with fast attack), no sidechain filter or key input, no expander ratio | REPLACE by the expander / gate of FX3 |
| `fx.reverb` (`Fx.cpp:469-587`) | 8-line FDN (Householder-type mix), 4 series all-pass diffusers per side, one-pole damping, sine modulation | Decorrelated output (L/R correlation 0.00 to 0.02) and size-independent decay. RT60 is **0.86x** of the setting (1.0 s gives 0.86 s, 2.5 s gives 2.16 s, 6 s gives 5.2 s). The damping coefficient is the same on every line (`:526`, `:559`) rather than scaled by line length, so high-frequency decay is not controlled per line (uneven HF decay, metallic tails at high damping); 8 lines give a low echo density for halls; hard clamp in the loop (`:568`); no early reflections, no plate / room / chamber algorithms, no per-band decay | REPLACE by the FDN suite (FX6); legacy stays |
| `fx.conv` (`FxExtra.cpp:750-1056`) | Uniform-partitioned FFT convolution (128 / 256), SIMD multiply-accumulate, 4 s maximum, procedural rooms | Correct overlap-save, real-time-safe kernel publication. CPU 3.2 % / 4.3 % per instance; 128-sample latency unreported; the IR is resampled by linear interpolation (`:805-822`, images in the IR); stereo only (no true-stereo four-channel IRs); switching IRs clears the tail abruptly | REPLACE the engine (FX7: non-uniform partitions, true IR import) |
| `fx.delay` (`Fx.cpp:392-466`) | Stereo / ping-pong / mono, 60 ms time glide, one-pole filters in the loop | Good glide behaviour. Cubic (Catmull-Rom) fractional read (`Fx.cpp:93-108`) loses **-3.25 dB at 16 kHz and -8.4 dB at 20 kHz** at half-sample positions, so modulated or gliding delays dull and amplitude-modulate the top octave; hard clamp at ±4 in the loop (`:431`); no modulation, saturation, ducking, diffusion; BFX3 | KEEP (legacy clean delay, FX2 fixes BFX3); FX10 is the pro delay |
| `fx.chorus` (`Fx.cpp:236-287`), `fx.flanger` (`:290-335`), `fx.dimension` (`FxExtra.cpp:338-400`) | Sine-modulated delay lines with cubic reads | Same top-octave loss; flanger delay floor clamped at 2 samples (`Fx.cpp:95`, so no through-zero); no BBD bandwidth, companding or clock behaviour | KEEP (legacy); FX11 adds the ensemble / BBD chorus and through-zero flanger |
| `fx.phaser` (`Fx.cpp:338-389`) | 8 first-order TPT all-passes, feedback | Clean linear core (modulation-safe TPT structure); `tan` per stage per sample; fixed 8 stages; no stage nonlinearity | KEEP (legacy); FX11 adds stage counts and OTA / JFET stage models |
| `fx.width` (`Fx.cpp:870-909`) | M/S width, bass mono by a one-pole on the side, pan | Bass-mono filter is 6 dB/oct and phase-shifting; no correlation meter | KEEP; FX15 imager |
| `fx.crusher` (`Fx.cpp:822-867`) | Sample-and-hold, quantiser, TPDF dither, jitter | Aliasing is the intended effect | KEEP |
| `fx.filter` (`FxExtra.cpp:286-335`) | The voice `FilterUnit` with an LFO | Inherits the P7 / P8 filter models when they land | KEEP |
| `fx.trance`, `fx.tremolo` (`FxExtra.cpp:541-655`) | Step gate, LFO tremolo | Edges smoothed (2 ms), tempo-synced | KEEP |
| `fx.ring` (`FxExtra.cpp:658-747`) | Ring modulator, Hilbert frequency shifter | BFX7 | IMPROVE (FX2) |
| `fx.vocoder` (`modules/fx/Vocoder.cpp`) | 16-32 band channel vocoder with flattening | Outside the mix / master scope; covered by `VocoderTests` | KEEP |
| Filter drive (`Filters.cpp:366-380`, `Filters.h:43`) | First-order ADAA tanh at 1x | ADAA already exists in the engine (reuse the pattern); THD -14 dB at drive 0.5 on a 0.5 sine | KEEP (P7 / P8 own it) |
| `Oversampler` (`Oversampler.cpp`) | Half-band Kaiser FIR (23 / 47 / 95 taps) or polyphase all-pass IIR (65 / 96 / 122 dB) per stage, measured latency | Good; the community plan keeps it. Gaps for FX: the IIR path is not linear phase, so a wet / dry mix against a delayed dry signal combs; the same coefficients serve every stage (later stages could be cheaper) | KEEP; FX1 wraps it (`OversampledBlock`) |

---

## 3. Target suite

Every module below is a new module type (an FX graph node after P4) with a generic id. "Design" names the public
techniques; "Accept" lists the numbers its tests must reach with the harness of section 5; "CPU" is per instance,
stereo, 48 kHz, Apple M-class core, Standard quality; "Prior art" ends with the verdict.

Common acceptance for **every** new module (not repeated below): the community plan's package acceptance
(tests, bit-exact legacy, zero allocations, `ci.sh`, refbench 39/39); silence in gives silence out (exact zero, or
below -150 dBFS with noise features off); 60 s at random parameter extremes stays finite and bounded; zipper
test below -90 dB and choice-change click test below -60 dB (4.7); measured latency equals the reported latency to
one sample; unity small-signal gain within 0.2 dB at neutral settings; 44.1 / 48 / 96 / 192 kHz all pass.

### 3.1 Shared cores (libraries, no parameters of their own)

| Core | Design | Used by | Package |
| --- | --- | --- | --- |
| `fx/Smoothing.h` | Per-sample linear ramp across each control block from the engine's control-rate values; log-domain ramps for frequencies; one-pole for times; a twin-state crossfader for discrete switches | all | FX1 |
| `fx/ChannelMode.h` | L/R, linked, unlinked, mid, side, M-S dual, left, right; encode / decode around any stereo kernel; link amount for detectors | all stereo modules | FX1 |
| `fx/OversampledBlock.h` | Wraps `Oversampler`: up, per-sample kernel, down, reported latency, dry path aligned with an all-pass-matched or FIR-delayed copy so wet / dry mixing never combs | all nonlinear modules | FX1 |
| `fx/Lookahead.h` | Delay buffer whose length can change without a dropout (two taps crossfaded over 10 ms; the reported latency follows) | limiter, gate, de-esser, dynamics | FX1 |
| `fx/Detectors.h` | Sample peak, oversampled true peak (4x polyphase per BS.1770 Annex 2, 8x option), true RMS (exponential and sliding window), peak-hold with a running min / max (monotonic deque, O(1)), box sum | dynamics, limiter, meters | FX1 |
| `fx/Dynamics.h` | Gain computers (down / up compression, expansion, gating, soft knee of any width), ballistics (attack, release, hold, dual-stage program-dependent release), feed-forward and feedback topologies, sidechain filters, stereo link | all dynamics | FX3 |
| `fx/GainCells.h` | Nonlinear gain elements: VCA (clean, slight level-dependent THD), FET (gate-voltage-dependent resistance, H2 rising with gain reduction), optical (electroluminescent panel plus light-dependent resistor with two-time-constant memory), variable-mu (remote-cutoff tube transfer, bias-dependent gain and distortion) | character compressors | FX8 |
| `fx/AnalogMatchedFilters.h` | Second-order sections matched to the analog prototype: Vicanek's matched second-order designs (magnitude-matched at DC, the centre and Nyquist), Orfanidis' prescribed-Nyquist-gain bells and shelves, plus a TPT state-variable form for audio-rate modulation; double-precision coefficients | EQs, crossovers, console curves | FX5 |
| `fx/LinearPhase.h` | FFT-based FIR built from a target magnitude (frequency sampling with a window, or minimum / mixed phase from the cepstrum for "natural phase"); partitioned execution; latency reported | linear-phase EQ, crossovers, imager | FX5 |
| `fx/Crossover.h` | Linkwitz-Riley 12 / 24 / 48 dB/oct with all-pass compensation of every band (flat sum for any band count), linear-phase FIR crossovers | multiband, imager, de-esser, exciter | FX5 |
| `Saturation.h` (P10) | Triode (Koren fit, bias, grid current, sag), Jiles-Atherton tape hysteresis, transformer flux saturation, diode and rail clippers, with ADAA where an antiderivative exists | saturator, tape, console, character compressors, passive EQ makeup stage | P10 (shared, not duplicated) |
| `fx/TapeMachine.h` | Record / playback emphasis, bias, hysteresis (P10), head-gap / spacing / thickness losses, head bump, wow / flutter / scrape flutter, hiss | tape module, tape echo | FX9 |
| `fx/FractionalDelay.h` | Kaiser-windowed sinc interpolation (8 / 16 / 32 taps, tables by quality), Thiran all-pass for fixed fractional delays, cubic kept for legacy | delays, choruses, flangers, reverb modulation | FX10 |
| `fx/Bbd.h` | Bucket-brigade delay: clock-rate-dependent input anti-aliasing and output reconstruction filters, sample-and-hold at the clock with the clock as the delay control, compander noise and breathing, soft clipping, clock feed-through (optional) | analog delay, chorus / ensemble, flanger | FX10 |
| `fx/Fdn.h` | Feedback delay network (8 / 16 / 32 lines), fast Hadamard or Householder mixing, per-line three-band absorption scaled by line length (exact RT60 per band), smooth random modulation through sinc-interpolated reads, input diffusion, early-reflection tap generator, freeze | algorithmic reverbs | FX6 |
| `fx/PartitionedConvolver.h` | Non-uniform partitioned convolution (direct head, small FFT partitions, then larger ones; Gardner / Garcia / Wefers scheduling), SIMD multiply-accumulate (the existing kernel), zero-latency and low-CPU modes, true-stereo | convolution, linear-phase filters with long kernels | FX7 |
| `fx/Loudness.h` | ITU-R BS.1770-4 K-weighting and gating, momentary / short-term / integrated loudness, loudness range (EBU Tech 3342), true-peak meter, PPM (IEC 60268-10) and VU (IEC 60268-17) ballistics, correlation | meter module, maximiser, auto gain | FX4 |
| `fx/PitchEngine.h` | Wraps Signalsmith Stretch (MIT, vendored once for P13 and FX16) for high-quality shifting with formant control; a low-latency granular (PSOLA-style) shifter for monitoring | pitch, harmonizer, doubler, shimmer | FX16 |

### 3.2 Dynamics

#### `fx.comp_clean`: clean digital compressor (FX3)

* **Design.** Dynamics core: detector choice (peak / RMS / hybrid crest-aware), feed-forward or feedback, knee 0-24
  dB, ratio 1-inf plus negative ratios, upward compression, attack 0.02-300 ms, release 5 ms-5 s plus auto
  (dual-stage program-dependent), hold, lookahead 0-10 ms, sidechain HP / LP / bell filters, external key, listen,
  channel mode and link, range, mix, auto makeup. Gain smoothing in the log domain (Giannoulis, Massberg and Reiss,
  "Digital Dynamic Range Compressor Design: A Tutorial and Analysis", JAES 2012).
* **Accept.** Static curve within 0.1 dB of the gain computer (stepped 1 kHz, RMS detector, -60 to +6 dBFS); attack
  and release times within 10 % of the label (time to 63 % of a 20 dB level step's gain-reduction change, peak and
  RMS modes); with RMS detection and 10 ms / 100 ms the THD of a 50 Hz sine 10 dB over threshold at most -60 dB;
  lookahead removes overshoot (output peak at most 0.1 dB above the steady value on a 20 dB step); external key:
  gain reduction follows the key only; gain-reduction view published.
* **CPU** at most 0.15 % (0.3 % with lookahead and true-peak detection).
* **Prior art.** Signalsmith DSP (MIT) envelope helpers (peak-hold, box filters) are a clean reference; DaisySP's
  compressor (MIT) is below target. BUILD on FX1 detectors.

#### `fx.comp_vca`: VCA bus compressor (FX3)

* **Design.** Feed-forward VCA topology with a smoothed-RMS detector, fixed ratios 1.5 / 2 / 4 / 10, attack 0.1 /
  0.3 / 1 / 3 / 10 / 30 ms, release 0.1 / 0.3 / 0.6 / 1.2 s and auto (program-dependent: fast for transients, slow
  for sustained material), sidechain high-pass, mix (parallel), a VCA gain cell (FX8's VCA element; available in
  FX3 as a clean cell) with a slight level-dependent THD, optional console output stage (P10 transformer, off by
  default).
* **Accept.** Ratios and thresholds as the clean compressor; auto release: release time to 63 % shortens by at least
  2x for a 50 ms burst against a 2 s burst (program dependence measured, documented curve); THD at 4 dB of gain
  reduction at most -70 dB without the output stage.
* **CPU** at most 0.2 %. **Prior art.** Public circuit descriptions of VCA bus compressors; BUILD.

#### `fx.comp_fet`, `fx.comp_opto`, `fx.comp_varimu`: character compressors (FX8)

| Module | Design | Parameters | Accept |
| --- | --- | --- | --- |
| `fx.comp_fet` (FET compressor) | Feedback topology; FET gain cell as a voltage-controlled resistor in a divider (gain-dependent distortion, H2 dominant, rising with gain reduction); peak detector with program-dependent attack 20 µs-0.8 ms and release 50 ms-1.1 s; ratios 4 / 8 / 12 / 20 / "all ratios" (all buttons: the knee and ratio rise and the attack / release change, as in published circuit analyses); input transformer and class-A output stage saturation (P10) at 2x oversampling | input, output, attack, release, ratio, mix, sidechain HP | THD vs gain-reduction curve monotonic with H2 > H3 at 10 dB GR; attack and release ranges hit within 10 %; "all ratios" shows a documented overshoot on a step; alias at most -90 dB at 5 kHz full input |
| `fx.comp_opto` (optical compressor) | Electroluminescent panel driven by the detector, light-dependent resistor with two release components (fast about 60 ms for roughly half the recovery, then slow 0.5-5 s depending on how long and how deep the compression was: a state variable for cell memory); soft, level-dependent ratio; tube makeup stage (P10 triode) | peak reduction, gain, compress / limit, mix, emphasis (sidechain HF) | release family: 63 % recovery after a 0.2 s burst at most 1/3 of that after a 5 s burst; attack about 10 ms within 20 %; ratio rises from about 2:1 to 4:1 over 20 dB (documented curve) |
| `fx.comp_varimu` (variable-mu compressor) | Remote-cutoff tube gain stage whose gain falls smoothly with bias (ratio rises with level: a very wide natural knee); slow attack (0.2-80 ms), release 0.3-10 s; single-ended (H2) or push-pull (H3, even harmonics cancelled) character; input / output transformers (P10) | input, threshold, attack, release (6 positions + auto), character, link, mix | static curve shows the rising ratio (from about 1.5:1 to over 6:1 across 30 dB, documented); push-pull even harmonics at least 30 dB below single-ended at equal GR |

* **CPU** at most 0.6 % each (2x oversampled gain cells). **Prior art.** Published circuit analyses and textbooks;
  DAFx papers on optical cell modelling and on wave digital filter models of compressor circuits; chowdsp_wdf
  (BSD-3, re-check) is an optional ADOPT for any WDF sub-circuit. BUILD.

#### `fx.gate2`: expander / gate / ducker (FX3)

* **Design.** Dynamics core in expansion mode: ratio 1:1 to 1:inf, range, hysteresis, hold, lookahead 0-10 ms,
  sidechain filters and external key, ducking mode (gain falls when the key rises), channel mode.
* **Accept.** With 2 ms lookahead the opening of a 1 kHz burst leaves an onset residual (output minus ideal gated
  signal) at most -60 dB; hysteresis and hold times within 5 %; ducking depth equals the range within 0.2 dB.
* **CPU** at most 0.1 %. BUILD.

#### `fx.deesser`: de-esser (FX13)

* **Design.** Detector band 2-16 kHz (band-pass or high-pass) with relative (band level against full-band level)
  or absolute threshold; split-band mode (only the band is reduced, through a complementary crossover from
  `Crossover.h`) or wide-band; lookahead 1-2 ms; listen; channel mode.
* **Accept.** Split-band: outside the band the output nulls against the input (minus latency) to -100 dB when no
  reduction happens, and changes by at most 0.1 dB when it does; reduction tracks the threshold within 0.5 dB on a
  synthetic sibilant (band-limited noise bursts over a vowel-like harmonic tone).
* **CPU** at most 0.2 %. BUILD.

#### `fx.transient`: transient shaper (FX13)

* **Design.** Difference of a fast and a slow envelope (level independent, works on their ratio in dB), attack and
  sustain gains ±15 dB, optional lookahead, output clip guard, mix.
* **Accept.** The same gain curve at -40 and -6 dBFS input within 0.5 dB; on a synthetic hit (noise burst with an
  exponential decay) the peak change equals the attack setting within 1 dB; a steady sine changes by less than 0.1
  dB with THD at most -80 dB.
* **CPU** at most 0.1 %. BUILD.

#### `fx.multiband2`: multiband dynamics (FX14)

* **Design.** 2-5 bands; crossovers from `Crossover.h`: minimum phase (LR4 / LR8 with all-pass compensation) or
  linear phase (FIR, latency reported); per band the full dynamics core (down / upward compression, expansion,
  threshold, ratio, knee, attack, release, makeup, solo, bypass, channel mode); lookahead; "upward plus downward"
  preset shapes for loud-and-dense processing; gain-reduction view per band.
* **Accept.** With dynamics neutral the band sum is flat within 0.01 dB (minimum phase, magnitude) and nulls against
  the delayed input to -100 dB (linear phase); slopes 24 / 48 ±1 dB/oct; per-band static curves within 0.1 dB;
  automating a crossover produces no step above -90 dB.
* **CPU** 5 bands: at most 1.2 % (minimum phase), 2.5 % (linear phase). **Prior art.** BUILD (the legacy multiband
  shows the bug class to test against: BFX2).

#### `fx.maximizer`: limiter / maximiser, and `fx.clipper` (FX4)

* **Design (maximiser).** Optional first stage: soft clipper (ADAA, oversampled) taking a set share of the peak
  reduction. Main stage: lookahead (0.5-10 ms) limiter whose detector is the 4x true-peak detector on the
  sidechain only (the audio path stays at 1x and linear phase) plus an optional 4x oversampled audio path for
  ISP-safe processing of extreme material; gain from a running minimum (O(1) deque) of the required gain, smoothed
  by a box filter as long as the lookahead (no overshoot by construction); release with two stages (fast for
  transients, slow for sustained gain reduction) and an adaptive mode keyed by crest factor; stereo link 0-100 %
  and M/S option; output ceiling in dBTP; loudness readout (FX4 meter core) with an optional target-LUFS gain
  assistant (control-thread suggestion, never automatic on the audio thread); TPDF / noise-shaped dither for 16 /
  20 / 24-bit output.
* **Design (clipper).** Soft / hard / curve-morph clipper with first-order ADAA at 2x / 4x / 8x, ceiling in dBTP
  with a true-peak guard.
* **Accept.** On an inter-sample-peak torture set (fs/4 sines at 0 / 22.5 / 45 degrees, square-like and clipped
  masters, the EBU true-peak test signals) output true peak at most ceiling + 0.1 dB (4x sidechain mode) and at
  most ceiling + 0.02 dB (oversampled audio mode); sample peaks never exceed the ceiling; 60 Hz sine with 6 dB of
  gain reduction: THD at most -60 dB in the transparent mode (hold and release adapted to the period); latency =
  lookahead + oversampling, reported. Clipper: 5 kHz at 6 dB over leaves alias power at most -90 dB (Standard) /
  -100 dB (High).
* **CPU** maximiser at most 0.6 % (4x true-peak sidechain), 1.2 % (oversampled audio); clipper at most 0.3 %.
* **Prior art.** Signalsmith's published limiter design notes and DSP library (MIT) describe the min-hold plus box
  filter structure the legacy limiter already uses: reference, optional ADOPT of the box / peak-hold helpers.
  BUILD the rest.

### 3.3 EQ and channel processing

#### `fx.eq_pro`: precision parametric / dynamic EQ (FX5)

* **Design.** Up to 16 bands from `AnalogMatchedFilters.h`: bell, low / high shelf (with slope and an optional
  resonant "bump"), tilt, band-pass, notch, all-pass, low / high cut 6 / 12 / 18 / 24 / 36 / 48 / 96 dB/oct (Butterworth,
  Bessel, Linkwitz-Riley, or a resonant Q). Phase modes: minimum (matched), natural (matched magnitude with
  analog-like phase), linear (FIR from `LinearPhase.h`, latency reported). Dynamic bands: each band's gain follows
  the dynamics core (threshold, ratio, attack, release, internal or external key), plus a spectral-dynamic mode
  later (backlog). Per-band channel mode (stereo / L / R / M / S). Auto gain. Analytic magnitude curve for GUIs
  (extending `FxParamEq::magnitudeDb`), analyser view through P6.
* **Accept.** Minimum / natural phase: magnitude within 0.1 dB of the analog prototype up to 20 kHz at 44.1, 48 and
  96 kHz for bells and shelves with centre frequencies up to 18 kHz (the legacy RBJ bell is 6.2 dB off at 20 kHz);
  linear phase: magnitude within 0.05 dB of the target and the impulse response symmetric to -120 dB; a full-range
  0-to-1 sweep of any band's frequency at 2 Hz leaves no zipper above -90 dB; dynamic band: static curve within 0.2
  dB, times within 10 %; 20 Hz bells at 192 kHz keep their accuracy (double-precision states where needed).
* **CPU** 8 active bands: at most 0.25 % (minimum), 0.8 % (linear phase).
* **Prior art.** Vicanek, "Matched Second Order Digital Filters" (2016, free paper); Orfanidis, "Digital
  Parametric Equalizer Design with Prescribed Nyquist-Frequency Gain" (JAES 1997); Cytomic (Simper) SVF technical
  papers (free); Zölzer, "DAFX". BUILD.

#### Console EQs (FX12)

| Module | Design | Accept |
| --- | --- | --- |
| `fx.eq_console_br` (British console EQ) | Two curve families modelled from public circuit analyses of classic British console channel EQs: a "fixed-Q" family (constant bandwidth, shelves with optional bell shape) and a "proportional-Q" family (bandwidth narrows as the gain rises); 18 dB/oct high-pass, 12 dB/oct low-pass; inductor-style mid character; analog prototypes discretised with the FX5 matched designs (or at 2x where a stage is nonlinear) | magnitude within 0.3 dB of the analog prototype (the transfer function derived from the circuit topology), proportional-Q bandwidth law within 5 %; console saturation THD curve as documented |
| `fx.eq_console_us` (American console EQ) | Proportional-Q three / four-band EQ with stepped frequencies and the characteristic shelf / peak curves of discrete op-amp American consoles (public circuit analyses); optional discrete op-amp stage saturation | as above |
| `fx.eq_passive` (passive tube EQ) | Passive LC network in which simultaneous low boost and low cut at the same frequency interact (a dip above the boost, the familiar shelf-with-dip curve), high boost with bandwidth and high attenuation shelf; modelled from the network's analog transfer function (or as a wave digital filter) and discretised at 2x; tube makeup stage (P10 triode) and output transformer (P10) | small-signal magnitude within 0.3 dB of the network's analytic response for all boost / cut combinations; the boost-and-cut interaction curve reproduced (dip depth and frequency within 10 % of the analytic network); makeup stage THD H2-dominant, documented |
| `fx.channel_strip` (console channel strip) | Composite of shared cores, no duplicated DSP: high / low-pass, expander / gate (FX3), compressor (VCA or FET cell, FX3 / FX8), console EQ (one of the above), console saturation (op-amp rail plus transformer, P10), order switch (dynamics before / after EQ), sidechain from the EQ, in / out trim | each section nulls against the same standalone module to -120 dB; console saturation THD vs level documented, H3 at 40 Hz at least 10 dB above H3 at 1 kHz (transformer) |

* **CPU** console EQs at most 0.4 %, passive EQ at most 0.6 %, channel strip at most 1.0 %.
* **Prior art.** Public circuit analyses, textbooks (Self, "Small Signal Audio Design"), and wave digital filter
  literature (Fettweis 1986; Werner et al., R-type adaptors, 2015); chowdsp_wdf (BSD-3, re-check) as an optional
  ADOPT for the passive network. Airwindows (MIT) console-summing algorithms are a candidate ADOPT for an extra
  console-character mode only if they measure as distinct (P10's evaluation rule). BUILD.

### 3.4 Saturation and colour

#### `fx.saturator` (FX13)

* **Design.** P10 models (triode single-ended, push-pull, tape hysteresis, transformer, diode, rail / op-amp,
  soft, hard, fold) with ADAA where an antiderivative exists, otherwise oversampling by quality (4.4); drive, bias /
  asymmetry, emphasis / de-emphasis pair around the shaper (tilt), optional 3-band mode (FX5 crossovers) with
  per-band drive, auto gain (loudness-matched through the FX4 meter core), mix, output.
* **Accept.** Harmonic signatures per model as P10's saturator rows (triode H2 > H3 at low drive, symmetric models
  even harmonics below -90 dB, transformer H3 at 50 Hz at least 10 dB above 1 kHz, hysteresis loop area > 0); THD
  monotonic with drive; 5 kHz at full drive alias at most -90 dB (Standard) / -100 dB (High), where the legacy drive
  measures -28 dB; auto gain keeps short-term loudness within 0.5 LU across the drive range on pink noise; drive
  at minimum nulls against the dry signal to -100 dB.
  **Exception (maintainer decision:** the tape (hysteresis) model is held to -85 dB (Standard) / -97 dB (High)
  across the whole drive range, because the loop's own switch aliases at low drive; it meets -90 / -100 at full drive.
  Every other model keeps -90 / -100 (`tests/ColourDynamicsTests.cpp`, `kTapeException*`).
* **CPU** at most 0.5 % (Standard), 1.5 % (High). **Prior art.** P10; Airwindows (MIT) optional ADOPT after
  measurement. BUILD.

#### `fx.tape`: tape machine (FX9)

* **Design.** Record path: input level against a calibration (tape "operating level"), record pre-emphasis per
  speed (NAB / IEC curves), bias amount (narrows or widens the hysteresis and changes the HF drive), Jiles-Atherton
  hysteresis (P10; RK2 at 2x for Standard, RK4 at 4x for High, as in the public DAFx 2019 tape-modelling paper);
  playback path: head-gap loss, spacing loss and thickness loss (Wallace / Bertram formulas) giving a
  speed-dependent HF roll-off, head bump (low-frequency ripple from the playback head contour, peaks scaling with
  speed), playback de-emphasis; transport: wow (0.3-3 Hz), flutter (5-30 Hz) and scrape flutter (filtered noise
  around 100 Hz-2 kHz) through a sinc-interpolated modulated delay, optional dropouts; hiss (shaped, seeded, off
  by default); speeds 7.5 / 15 / 30 ips; latency reported.
* **Accept.** Small-signal response at -20 dB below operating level matches the model's analytic curve within 0.5
  dB (head-bump peak frequency proportional to speed within 10 %, HF -3 dB point proportional to speed within 10
  %); 1 kHz at operating level: THD 1 % ±0.3 %, H3 dominant; level of 3 % THD documented; loop area > 0; weighted
  wow and flutter (IEC 60386 method on 3.15 kHz) within 10 % of the setting; alias at most -90 dB (Standard); stable
  60 s at extremes.
* **CPU** at most 1.0 % (Standard), 2.5 % (High).
* **Prior art.** Jiles and Atherton (1986); Chowdhury, "Real-time Physical Modelling for Analog Tape Machines" (DAFx
  2019, public paper; the ChowTapeModel code is GPL-3: clean-room only); Holters and Zölzer (DAFx 2016) on
  Jiles-Atherton in circuit simulation; Bertram, "Theory of Magnetic Recording". BUILD on P10.

#### `fx.exciter`: harmonic exciter and bass enhancer (FX13)

* **Design.** High mode: band above a set frequency (1-10 kHz) through an asymmetric or symmetric saturator,
  band-limited, mixed back; low mode: the band below a frequency generates harmonics of the bass (perceived
  fundamental), high-passed so the added energy sits above the original band.
* **Accept.** Below the exciter's frequency the output changes by at most 0.5 dB; generated harmonics follow the
  model's documented law; alias at most -90 dB. **CPU** at most 0.3 %. BUILD.

### 3.5 Time-based effects

#### `fx.delay_pro`: digital / analog / tape delay (FX10)

* **Design.** Modes: digital (sinc-interpolated reads from `FractionalDelay.h`, flat when modulated), analog
  (`Bbd.h`: stages 256-4096, clock = stages / (2 x delay), clock-dependent bandwidth, compander noise and breathing,
  soft clip), tape echo (FX9 tape core: 1-3 playback heads, wow and flutter, saturation and HF loss compounding per
  repeat). Common: tempo sync, feedback with saturation in the loop (no hard clamp), TPT filters in the loop, ping
  pong / stereo / cross feedback, modulation, diffusion (all-pass chain), ducking from the input or a sidechain,
  freeze, time glide (pitch, as now) or crossfade (digital, no pitch) on time changes.
* **Accept.** Digital: delay time exact to 0.01 sample at fixed times; magnitude flat within 0.1 dB to 18 kHz at
  every fractional position, also while modulated (the legacy cubic read loses 3.25 dB at 16 kHz); BBD: -3 dB
  bandwidth tracks the clock as documented within 10 %, imaging below -60 dB; tape: per-repeat HF loss and wow /
  flutter as FX9; feedback 0.99 for 60 s stays bounded with saturation; ducking depth within 0.2 dB.
* **CPU** at most 0.3 % (digital), 0.6 % (BBD), 1.0 % (tape).
* **Prior art.** Raffel and Smith, "Practical Modeling of Bucket-Brigade Device Circuits" (DAFx 2010) and
  Chowdhury's public BBD emulation write-up (2021) for the BBD; the chowdsp_utils BBD code is used only if its
  module is confirmed BSD (otherwise design reading only). Laakso et al., "Splitting the unit delay" (IEEE SP
  Magazine 1996) for fractional delays. Signalsmith DSP (MIT) `delay.h` Kaiser-sinc interpolators: ADOPT if they
  meet the flatness target, else BUILD. BUILD the BBD.

#### Algorithmic reverbs: `fx.reverb_algo` (FX6) and `fx.reverb_spring` (FX17)

* **Design (`fx.reverb_algo`, algorithm choice Room / Hall / Chamber / Plate / Ambience).**
  * FDN core (`Fdn.h`): 16 lines (32 at High), co-prime lengths spread by size, Hadamard or Householder mixing;
    per-line three-band absorption (low / mid / high shelving gains scaled by each line's length, Jot's method,
    with the improved accuracy design of Prawda, Schlecht and Välimäki) so RT60 is exact per band; low and high
    crossover frequencies and multipliers as parameters.
  * Modulation: per-line smooth random (interpolated noise) of a few samples through sinc reads, depth and rate
    parameters; chorusing that never produces audible pitch wobble at default settings.
  * Input: predelay, input diffusion (all-pass chain or velvet noise), early reflections per algorithm (a
    tapped delay from an image-source shoebox model for Room / Chamber, sparse for Hall, none for Plate).
  * Plate: a Dattorro-type figure-eight all-pass tank (Dattorro, "Effect Design Part 1", JAES 1997) instead of
    the FDN, for its fast build-up and bright density.
  * Output: width, decorrelated taps, low / high cut, freeze, ducking, mix; tail length reported for hosts.
* **Design (`fx.reverb_spring`).** Dispersive all-pass chains producing the chirp, low-pass at a few kHz,
  feedback with tension / length / drip (Välimäki, Parker and Abel, "Parametric spring reverberation effect",
  JAES 2010; Parker's later spring papers), optional tank shaking transient; plus `fx.reverb_special` (gated and
  reverse shapes; shimmer when FX16 exists).
* **Accept.** RT60 (T30 from the Schroeder energy decay curve) within ±10 % of the target per octave band 125 Hz-8
  kHz (the legacy reverb is 14 % short broadband); energy decay curve straight within 1 dB from -5 to -35 dB
  (no double slope unless designed); normalised echo density reaches 1 within 80 ms (Hall) and 20 ms (Plate);
  late-tail magnitude spectrum (1/3-octave-normalised) standard deviation at most 2 dB (no metallic ringing);
  interaural correlation at most 0.3 at full width; mono sum within ±3 dB of the stereo energy per band;
  modulation pitch spread on a sine at default settings at most 3 cents RMS; freeze for 60 s holds level within 0.5
  dB and stays bounded. Spring: group delay rises with frequency across the chirp band (dispersion slope sign and
  range documented); bandwidth -3 dB in the documented range.
* **CPU** at most 1.0 % (16 lines), 2.0 % (32 lines); spring at most 0.8 %.
* **Prior art.** Jot and Chaigne (1991); Schlecht and Habets, "On lossless feedback delay networks" (2017) and
  "Time-varying feedback matrices in FDNs" (2015); Välimäki et al., "Fifty years of artificial reverberation"
  (2012); Dattorro (1997). CloudSeed (MIT) is a strong permissive algorithmic reverb: optional ADOPT later as an
  extra algorithm after measurement (backlog). Freeverb (public domain) is below target. Dragonfly and zita-rev1
  (GPL) only through their public descriptions. Valhalla's public blog posts on reverb design are design reading,
  not code. BUILD the core.

#### `fx.convolution`: convolution with true IR import (FX7)

* **Design.** `PartitionedConvolver.h`: non-uniform partitions (direct-form head of 32-64 samples for the
  zero-latency mode, then 128 / 1024 / 8192-sample partitions; the larger ones computed incrementally across
  blocks so the audio thread does even work per block), the existing split-complex SIMD multiply-accumulate.
  Import: WAV / AIFF of any rate and length up to 30 s (control thread), mono, stereo or true-stereo (4 channels:
  LL, LR, RL, RR), resampled offline by a Kaiser-windowed sinc (stopband at least 120 dB); IR edit (trim start,
  fade, length, reverse, stretch by resampling, early / late balance, low / high cut, predelay, width); kernel
  changes crossfade (no tail cut); latency 0 (zero-latency mode) or one small block (low-CPU mode), reported.
  The legacy `fx.conv` keeps its engine unless a maintainer accepts a rounding-level golden change.
* **Accept.** Output equals a double-precision direct convolution to -120 dB relative (any partition plan); 4 s
  stereo IR at most 0.6 % CPU (low-CPU mode) and 1.2 % (zero latency) against today's 4.3 %; per-block CPU
  spread (max / mean) at most 2 (no spikes from large partitions); resampled IR images below -120 dB; true-stereo
  matrix verified by channel-impulse tests; kernel switch has no step above -60 dB.
* **Prior art.** Gardner, "Efficient convolution without input-output delay" (JAES 1995); Wefers, "Partitioned
  convolution algorithms for real-time auralization" (2015); FFTConvolver (MIT, HiFi-LoFi): ADOPT its two-stage
  scheduling design (port onto `Fft.h` with attribution) or BUILD to the same design; WDL's convolution engine
  (zlib-style licence, re-check) as a second reference; r8brain-free-src (MIT) only if our offline resampler
  misses the 120 dB images target. ADOPT / BUILD as stated.

### 3.6 Modulation (FX11)

| Module | Design | Accept | CPU |
| --- | --- | --- | --- |
| `fx.ensemble` (BBD chorus / ensemble) | 1-3 BBD lines (`Bbd.h`) with LFOs 120 degrees apart (string-ensemble style) or 2 lines with crossfeed in four depth / rate modes (the "dimensional" chorus type), triangle / sine / smooth-random LFOs, BBD noise optional, stereo spread | modulation pitch deviation equals the set depth (cents) within 5 %; HF response per the BBD bandwidth law; mono sum within ±1 dB of each channel's energy at default; no zipper on rate changes | at most 0.5 % |
| `fx.phaser2` | 2 / 4 / 6 / 8 / 12 TPT all-pass stages with optional OTA or JFET stage nonlinearity (tanh-shaped stage input, solved per sample), feedback with saturation, spread, LFO / envelope follower / sidechain modulation, optional barber-pole (endless) mode | notch frequencies equal the analytic all-pass prediction within 1 % (linear mode); notch depth at least 40 dB at mix 0.5; sweep range within 2 % | at most 0.3 % |
| `fx.flanger2` | Through-zero flanger (two sinc-read delays crossing at zero, as tape flanging), BBD mode, positive / negative feedback with saturation, envelope-follower mode | comb notches at the analytic frequencies within 0.5 %; passing through zero produces no step above -60 dB; feedback 0.95 bounded | at most 0.3 % |

Prior art: public BBD and phaser circuit analyses; DAFx papers on virtual-analog phasers; Signalsmith DSP (MIT)
references. Tremolo / auto-pan, ring modulator and frequency shifter: the legacy modules are kept (FX2 fixes
BFX7). Rotary speaker: backlog. BUILD.

### 3.7 Pitch (FX16)

| Module | Design | Accept | CPU |
| --- | --- | --- | --- |
| `fx.pitch` | `PitchEngine.h`: high-quality mode (Signalsmith Stretch) ±24 semitones with formant shift; low-latency granular mode (about 15 ms) for monitoring | pitch of a shifted sine within 1 cent; THD+N of a shifted sine at most -60 dB (high-quality mode); formant preservation: spectral-envelope error of a synthetic vowel at most 2 dB; reported latency equals measured | at most 2.0 % |
| `fx.harmonizer` | 2-4 voices with intervals in a key and scale (from the engine's key detection / key lock, `KeyDetect.h`, `KeyLock.h`), per-voice level / pan / delay / detune / formant | each voice's interval correct for every scale degree; voices independent (null tests per voice) | at most 4.0 % (4 voices) |
| `fx.doubler` | Micro-pitch shift (0-50 cents) plus short modulated delays per side | pitch offsets within 0.5 cent; mono compatibility within ±1.5 dB | at most 0.8 % |

Prior art: Signalsmith Stretch (MIT): ADOPT (vendored once, shared with P13). Rubber Band and SoundTouch are GPL /
LGPL: not used. BUILD the granular mode and the harmonizer logic.

### 3.8 Stereo and utility (FX15; meters in FX4)

| Module | Design | Accept | CPU |
| --- | --- | --- | --- |
| `fx.imager` | 1-4 bands (FX5 crossovers, minimum or linear phase), width 0-200 % per band, M/S balance, mono below a frequency (linear phase), stereoize (decorrelation by all-pass networks or velvet noise, mono-compatible), correlation and goniometer views | width 100 % and neutral settings bit-transparent; stereoize mono sum flat within ±1 dB; band sum as FX14 | at most 0.4 % (minimum), 1.0 % (linear) |
| `util.utility` | Gain / trim (ramped), phase invert per channel, swap, M/S encode / decode, mono, sample-rate-independent DC blocker (corner set in Hz), channel delay | exact arithmetic tests; DC blocker corner within 1 % at every rate | at most 0.05 % |
| `util.meter` (FX4) | `Loudness.h`: LUFS momentary / short-term / integrated, loudness range, true peak, sample peak, RMS, PPM / VU ballistics, correlation, published as views (P6) | EBU Tech 3341 and 3342 test signals (synthesised from their published definitions) within ±0.1 LU; true peak within ±0.2 dB of a 16x reference; libebur128 (MIT) vendored for tests only as the oracle, like the tuning-library in P1 | at most 0.2 % |

### 3.9 Oversampling and anti-aliasing per module

The community plan's 2.7 rule (no module aliases above -90 dB at Standard / -100 dB at High below 20 kHz through
its default settings) is extended to "at its maximum drive as well" for every new module.

| Module class | Draft | Standard | High |
| --- | --- | --- | --- |
| Saturator, clipper (ADAA models) | ADAA 1x | ADAA 2x | ADAA 4x |
| Saturator (no antiderivative: hysteresis, fold) | 2x | 4x | 8x |
| Tape | 2x RK2 | 2x RK2 | 4x RK4 |
| Character compressor gain cells, console stages, passive EQ makeup | 1x ADAA | 2x | 4x |
| Maximiser detector / audio | 4x detector, 1x audio | 4x detector, 1x audio | 8x detector, 4x audio option |
| Linear EQ, linear dynamics (clean, VCA cell clean), delays, reverbs, modulation | 1x | 1x | 1x |
| BBD (clock domain is its own sample-and-hold) | 1x | 1x | 2x |

Wet / dry mixing on an oversampled module always uses `OversampledBlock`'s aligned dry path: FIR filters give a
pure delay; the IIR path is allowed only with an all-pass-matched dry path (null test at mix 0 to -120 dB).

### 3.10 CPU summary (bench rows added by each package; the bench fails at target + 20 %)

| Module | Target | Module | Target |
| --- | --- | --- | --- |
| `fx.comp_clean` | 0.15 % (0.3 % lookahead + TP) | `fx.reverb_algo` | 1.0 % (16 lines), 2.0 % (32) |
| `fx.comp_vca` | 0.2 % | `fx.reverb_spring` | 0.8 % |
| `fx.comp_fet / opto / varimu` | 0.6 % each | `fx.convolution` (4 s stereo) | 0.6 % low-CPU, 1.2 % zero latency |
| `fx.gate2`, `fx.transient` | 0.1 % | `fx.delay_pro` | 0.3 / 0.6 / 1.0 % (digital / BBD / tape) |
| `fx.deesser` | 0.2 % | `fx.ensemble` | 0.5 % |
| `fx.multiband2` (5 bands) | 1.2 % (min phase), 2.5 % (linear) | `fx.phaser2`, `fx.flanger2` | 0.3 % |
| `fx.maximizer` | 0.6 % (1.2 % OS audio) | `fx.pitch`, `fx.harmonizer` | 2.0 %, 4.0 % |
| `fx.clipper` | 0.3 % | `fx.doubler` | 0.8 % |
| `fx.eq_pro` (8 bands) | 0.25 % (min), 0.8 % (linear) | `fx.imager` | 0.4 % / 1.0 % |
| console EQs / passive EQ / strip | 0.4 / 0.6 / 1.0 % | `util.utility`, `util.meter` | 0.05 %, 0.2 % |
| `fx.saturator` | 0.5 % (1.5 % High) | `fx.tape` | 1.0 % (2.5 % High) |

The community plan's "each extra FX instance at most 1 %" row stays the default; the modules above 1 % (linear
phase, 32-line reverb, zero-latency convolution, pitch, High-quality tape) are listed as named exceptions.

---

## 4. Platform needs

These are requirements on the community plan's platform packages. Each item names the package that carries it;
the FX packages only consume them.

### 4.1 Multiple instances and FX graph lanes (P4)

* Any number of instances of any effect (limit 64), with ids `fx{k}_<param>` for instance k >= 2 (P4's rule).
* Lanes in series and parallel, sends and returns, per-lane wet / dry, a master lane.
* FX-only products (no voices; host input through `process_aux` today, multiple host buses after 4.3).
* **Plugin delay compensation inside the graph**: the compiler sums reported latencies along every path and inserts
  delays on the shorter parallel paths and on dry / send paths, so parallel compression with a lookahead
  compressor, or a linear-phase EQ in one lane, stays phase-aligned. Acceptance: two parallel lanes, one with a
  10 ms-lookahead limiter in neutral and one dry, sum to exactly 2x the delayed input.
* **Bypass and on / off crossfades** (fixes BFX4 for every module, legacy included): the executor crossfades over
  10 ms between the module's output and its latency-aligned input; the module's state is reset (or kept, per its
  `bypassPolicy`: reset for time-based effects, kept for dynamics) at the end of the fade. Acceptance: switching any
  module on or off on a 1 kHz sine leaves no step above -60 dB.
* Module and quality changes at runtime: the new instance is prepared on the control thread and crossfaded in.

### 4.2 Latency and tail reporting API (P4 node interface, P5 ABI, C API)

* `int latencySamples() const` and `double tailSeconds() const` (infinite flag for freeze / infinite feedback) on
  every effect node; defaults 0.
* Latency may change only on the control thread (prepare, quality change, lookahead or phase-mode parameter);
  a change sets a flag that the engine reports.
* Engine level: `crucible_latency_samples(engine)` and `crucible_tail_seconds(engine)` plus a
  `latency_changed` poll (modelled on CLAP's latency extension; no dependency). Hosts wrap it as their plugin
  format's latency report.
* Until P4, FX1 adds a non-virtual-breaking hook (a `LatencyReporter` capability interface found with
  `ModuleRack::find<T>()`, like `GainReductionMeter`) so FX modules report latency from day one.

### 4.3 Sidechain buses (P4 graph, P5 ABI)

* A `sidechain` port type (stereo) on modules; each module declares 0-2 sidechain inputs in its metadata.
* Sources: host auxiliary buses (generalising the one `process_aux` input to N named buses in the C API), the
  output of any lane or node, a voice layer's output (a kick layer ducking a pad layer in one product), and the
  dry input of the lane.
* The compiler orders sidechain sources before their consumers; a cycle is rejected unless the edge is marked
  "one block late".
* Sidechain filters and listen are module features (FX3 dynamics core), not graph features.

### 4.4 Oversampling policy (community 2.7 plus 3.9 above)

* The quality tier comes from the engine (`ModuleContext::quality`) with an optional per-instance override
  parameter `<instance>_quality` (Engine default / Draft / Standard / High).
* Oversampling factor and filter family are fixed per prepare; changes re-prepare and crossfade (4.1).
* `OversampledBlock` (FX1) is the only way a module oversamples: it reports latency, aligns dry paths, and
  allocates in prepare only.

### 4.5 Lookahead

* Lookahead is latency (4.2). Lookahead changes go through `Lookahead.h` (two taps, 10 ms crossfade), never a
  buffer clear (BFX5), and raise the latency-changed flag.
* Products that need constant latency (live use) can pin the lookahead (metadata `latencyPolicy: fixed-max`, the
  buffer always delays by the maximum and the lookahead only moves the detector).

### 4.6 Mid / side and channel modes

* Standard parameter `<instance>_channels`: Stereo linked, Stereo unlinked, Mid, Side, Mid / Side (dual), Left,
  Right; plus `<instance>_link` (0-100 %) for dynamics. Implemented once in `ChannelMode.h` (FX1) and offered by
  every stereo module where it is meaningful.
* Per-band channel modes inside EQs and multiband modules.

### 4.7 Parameter smoothing standard (every new module; P3 metadata carries the class)

| Class | Examples | Rule |
| --- | --- | --- |
| Gain | levels, mix, makeup, width | per-sample linear ramp across each control block between the engine's smoothed values (no stepping every 32 samples) |
| Frequency | cutoffs, crossovers, EQ centres | log-domain ramp; coefficients updated at least every 32 samples (every sample for audio-rate modulation) in a modulation-safe structure (TPT / SVF; direct-form biquads only with per-sample coefficient interpolation) |
| Time | delay time | glide (pitch, "tape") or crossfade (no pitch, "digital"), selectable |
| Discrete | model, algorithm, stage count, ratio steps | 20 ms crossfade between the old and new kernel states (preallocated twin) |
| Structural | quality, phase mode, lookahead | re-prepare off the audio thread, crossfade (4.1 / 4.5) |
| Switch | on / off, bypass | 10 ms latency-aligned crossfade by the executor (4.1) |

The **zipper test** (section 5): each continuous parameter is automated with a 0-to-1 ramp over 50 ms and with a 2 Hz
square wave while a 1 kHz sine plays; after removing the expected modulation sidebands, the residual must stay at
most -90 dB. The **click test**: each discrete change on a sine leaves no step above -60 dB.

### 4.8 Gain staging and calibration

* Reference: -18 dBFS RMS sine = 0 VU = analog operating level (EBU R68 convention), adjustable per analog-modelled
  module (`<instance>_cal`, -24 to -12 dBFS) so the documented THD figures hold at the documented level.
* Every nonlinear module has unity small-signal gain at neutral settings (within 0.2 dB) and an auto-gain option
  that compensates its static loudness change (an offline-measured curve, so the audio thread does no metering).

### 4.9 Noise, precision and denormals

* Silence in gives silence out unless a noise feature is on; modelled hiss is seeded per instance (`ModuleContext::seed`)
  and off by default.
* Double precision for low-frequency IIR states where the float error would exceed the accuracy targets (EQ bells
  below 40 Hz at 192 kHz, DC blockers); float elsewhere; flush-to-zero as today.
* Stereo kernels may use `processQuad` (`simd::Float4` across L / R and two bands) where it measures at least 1.5x
  faster.

### 4.10 Metering and views (P6)

Gain reduction (per band), input / output peak, RMS, LUFS, true peak, correlation, EQ curve (analytic), spectrum,
reverb decay, BBD / tape transport state, all as P6 views with no cost when unsubscribed. The legacy
`GainReductionMeter` becomes a view wrapper.

### 4.11 How this ties into P3-P6, P10 and P13

| Community package | What the FX plan needs from it |
| --- | --- |
| P3 metadata | display text for ratios (`4:1`), dB / ms / Hz / % / LUFS / dBTP units, the smoothing class (4.7), `latencyPolicy`, sidechain port declarations, per-instance parameter groups |
| P4 graph | instances, lanes, sends, sidechain edges, PDC, bypass crossfades, runtime module changes with fades, FX-only products; new FX modules register as graph nodes |
| P5 SDK | `latency_samples`, `tail_seconds`, sidechain buffers in `process`, `bypass_policy`; `crucible-module-check` adds the zipper, click, latency and silence checks for effects |
| P6 views | meter, gain reduction, loudness, EQ curve, scope, goniometer, spectrum views |
| P10 saturation | the single home of triode, tape hysteresis, transformer and clipper models (FX8, FX9, FX12, FX13 call it) |
| P13 engines | vendors Signalsmith Stretch once; FX16 reuses it (whichever lands first vendors it) |

---

## 5. Verification

### 5.1 FX measurement harness (`tests/FxMeasure.h`, FX1)

Built on `AnalogMeasure.h` (P1), header only, double precision, tests and offline tools only. Each primitive gets
a self-test with a synthetic truth, like P1.

| Primitive | Measures | Self-test truth |
| --- | --- | --- |
| `staticCurve` | gain against input level (stepped sine, RMS) | an ideal gain computer within 0.01 dB |
| `stepTimes` | attack / release time to 63 % and 10-90 % of a gain-reduction change, from level steps | an analytic one-pole within 1 % |
| `releaseFamily` | release time against burst length and depth (program dependence) | a two-state model within 2 % |
| `thdCurve`, `harmonicCurve` | THD and H2-H5 against level and frequency | P1 `harmonicProfile` truths |
| `imd` | SMPTE (60 Hz + 7 kHz, 4:1) and CCIF (19 + 20 kHz) intermodulation | a known polynomial within 0.1 dB |
| `aliasAtDrive` | alias power and worst spur at 5 / 7 / 10 kHz | P1 `measureAliasing` truths |
| `nullTest` | residual of output against a reference after latency alignment | a pure delay: below -150 dB |
| `measureLatency` | delay by cross-correlation with sub-sample refinement | fractional delays within 0.01 sample |
| `zipperTest`, `clickTest` | residual outside expected sidebands under automation; largest step at a switch | a stepped gain at a known level |
| `energyDecay` | Schroeder curve, EDT / T20 / T30 per octave band, curvature, normalised echo density, IACC, tail spectral flatness | a synthetic exponential noise decay per band within 2 % |
| `modulationDepth` | pitch deviation (cents) and rate of a modulated sine | a synthetic FM sine within 0.1 cent |
| `wowFlutter` | weighted wow and flutter of a 3.15 kHz tone (IEC 60386 method) | a synthetic FM tone within 2 % |
| `loudness` | thin wrapper over `Loudness.h` (FX4) for tests, checked against vendored libebur128 | EBU Tech 3341 / 3342 cases |
| `stereoStats` | correlation, mono-sum energy per band, M/S energy | constructed signals |

### 5.2 Baseline and guards

`crucible-measure fx` (FX1) prints the section 2 numbers for every legacy effect (aliasing at drive, true peak,
static curve, attack time, band-sum flatness, RT60, fractional-read response, cramping, CPU) into
`docs/MEASUREMENTS.md` ("Effects" section). FX2 turns each bug into a regression test; later packages add
their acceptance rows as guards.

### 5.3 Reference bench, effect mode (FXR)

`tools/reference-bench` gains an effect mode: its JUCE host (not part of this repository) loads an
licensed effect plug-in, runs the section 5.1 signal set through it at fixed settings and writes the same
measurements (THD curves, static curves, attack / release, frequency responses, RT60 bands, true peak) into
`targets.json` as bands, marked `pending` until an engine module is scored against them. Black box only: no
disassembly, no preset, IR or resource extraction, no sample matching. Without a licensed reference, targets are
`analytic` (published circuit analyses, standards).

---

## 6. Work packages

**Rules.** The community plan's package rules apply (one worker per package in its own worktree; listed
files plus the shared append-only files: the built-in module list / `Modules.cpp` registration lines, the source
list in `CMakeLists.txt`, regenerated `PARAMETERS.md`, a `ROADMAP.md` row, a `CHANGELOG.md` entry,
`THIRD_PARTY.md`, and an appended section in `docs/MEASUREMENTS.md`; coordinator merges one at a time). In
addition: every new module meets the common acceptance of section 3; every package adds its bench rows (3.10);
legacy effects stay bit-exact except in FX2. **All FX packages start after P2 has merged.** New effect *modules*
start after P4 (they register as graph nodes); FX1 (libraries and harness) and the library halves of later
packages do not need P4.

Ordered by value:

### FX1. FX core library, FX measurement harness, effect baseline

* **Depends on:** P2. New files only (plus a test-executable line in CMake).
* **Scope:** `include/crucible/fx/{Smoothing,ChannelMode,OversampledBlock,Lookahead,Detectors}.h`,
  `src/fx/FxCore.cpp`; the `LatencyReporter` capability interface (new header, 4.2);
  `tests/FxMeasure.h`, `tests/FxMeasureTests.cpp`, `tests/FxBaseline.cpp` (`crucible-measure fx`);
  the "Effects" section of `MEASUREMENTS.md` reproducing section 2.
* **Accept:** every 5.1 primitive recovers its synthetic truth; detectors: true-peak reading of fs/4 sines at all
  phases within 0.1 dB, O(1) running min / max equals brute force; `OversampledBlock` null at mix 0 below -120 dB
  for FIR and IIR families; `Lookahead` length change without any step above -90 dB; the section 2 numbers
  reproduced within 0.5 dB / 5 %.
* **Wave:** runs beside the community wave A (new files only).

### FX2. Legacy effect fixes (BFX1-BFX7)

* **Depends on:** FX1 (tests), P10 (P10 appends drive modes in `Fx.cpp`; FX2 goes after it to avoid conflicts),
  P4 for BFX4 (the crossfade lives in the graph executor; if P4 has merged, FX2 wires the executor's crossfade to
  the legacy modules; otherwise BFX4 is left to P4).
* **Scope:** `src/Fx.cpp`, `src/FxExtra.cpp`, `tests/FxFixTests.cpp`, `SOUND_CHANGES.md`. BFX1 corner in Hz
  (about 5 Hz, rate-independent); BFX2 all-pass compensation of the low band; BFX3 clear delay / FDN / convolver
  state on re-activation; BFX5 lookahead change through `Lookahead.h`; BFX6 smoothed crossovers; BFX7 Hilbert
  coefficients designed per sample rate at prepare.
* **Accept:** each bug has a failing-before / passing-after test (drive 40 Hz loss below 0.1 dB at every rate;
  multiband sum within 0.01 dB for 500 / 750 Hz; no stale audio after re-enable; no dropout on lookahead change;
  crossover automation step below -90 dB; shifter image rejection at 30 Hz equal at 48 and 192 kHz); goldens
  changed only where a fix applies, each change approved and recorded.
* **Wave:** step C (see the schedule).

### FX3. Dynamics core, clean compressor, VCA bus compressor, expander / gate / ducker

* **Depends on:** FX1; P4 for module registration.
* **Scope:** `include/crucible/fx/Dynamics.h`, `src/fx/Dynamics.cpp`,
  `src/modules/fx/{CompClean,CompVca,Gate2}.cpp`, `tests/DynamicsTests.cpp`.
* **Accept:** 3.2 rows for `fx.comp_clean`, `fx.comp_vca`, `fx.gate2`; the legacy `fx.comp` and `fx.gate`
  untouched.

### FX4. Maximiser, clipper, loudness and true-peak metering

* **Depends on:** FX1; P4.
* **Scope:** `include/crucible/fx/Loudness.h`, `src/fx/Loudness.cpp`,
  `src/modules/fx/{Maximizer,Clipper,Meter}.cpp`, `tests/MaximizerTests.cpp`, `tests/LoudnessTests.cpp`,
  `third_party/libebur128/` (tests only).
* **Accept:** 3.2 maximiser / clipper rows; 3.8 meter row.

### FX5. Pro EQ: analog-matched filters, linear-phase engine, crossovers, `fx.eq_pro`

* **Depends on:** FX3 (dynamic bands); P4.
* **Scope:** `include/crucible/fx/{AnalogMatchedFilters,LinearPhase,Crossover}.h`,
  `src/fx/{AnalogMatchedFilters,LinearPhase,Crossover}.cpp`, `src/modules/fx/EqPro.cpp`, `tests/EqProTests.cpp`,
  `tests/CrossoverTests.cpp`.
* **Accept:** 3.3 `fx.eq_pro` row; crossovers: flat sums (min phase 0.01 dB for 2-5 bands at any spacing, linear
  phase null -100 dB).

### FX6. Algorithmic reverb suite

* **Depends on:** FX1; P4.
* **Scope:** `include/crucible/fx/Fdn.h`, `src/fx/{Fdn,EarlyReflections,PlateTank}.cpp`,
  `src/modules/fx/ReverbAlgo.cpp`, `tests/ReverbAlgoTests.cpp`.
* **Accept:** 3.5 `fx.reverb_algo` rows for Room, Hall, Chamber, Plate, Ambience.

### FX7. Convolution v2 with true IR import

* **Depends on:** FX1; P4.
* **Scope:** `include/crucible/fx/PartitionedConvolver.h`, `src/fx/{PartitionedConvolver,IrImport}.cpp`,
  `src/modules/fx/Convolution.cpp`, `tests/ConvolutionTests.cpp`; optional `third_party/fftconvolver/`.
* **Accept:** 3.5 `fx.convolution` rows.

### FX8. Character compressors: FET, optical, variable-mu

* **Depends on:** FX3, P10.
* **Scope:** `include/crucible/fx/GainCells.h`, `src/fx/GainCells.cpp`,
  `src/modules/fx/{CompFet,CompOpto,CompVariMu}.cpp`, `tests/CharacterCompTests.cpp`.
* **Accept:** 3.2 character-compressor table.

### FX9. Tape machine

* **Depends on:** P10 (hysteresis), FX1.
* **Scope:** `include/crucible/fx/TapeMachine.h`, `src/fx/TapeMachine.cpp`, `src/modules/fx/Tape.cpp`,
  `tests/TapeTests.cpp`.
* **Accept:** 3.4 `fx.tape` rows.

### FX10. Fractional delay, BBD core, pro delay

* **Depends on:** FX1; FX9 for the tape-echo mode (if FX9 has not merged, the mode is appended in a follow-up).
* **Scope:** `include/crucible/fx/{FractionalDelay,Bbd}.h`, `src/fx/{FractionalDelay,Bbd}.cpp`,
  `src/modules/fx/DelayPro.cpp`, `tests/DelayProTests.cpp`.
* **Accept:** 3.5 `fx.delay_pro` rows; `FractionalDelay` flat within 0.1 dB to 18 kHz at every fraction.

### FX11. Modulation suite: ensemble, phaser, through-zero flanger

* **Depends on:** FX10.
* **Scope:** `src/modules/fx/{Ensemble,Phaser2,Flanger2}.cpp`, `tests/ModulationFxTests.cpp`.
* **Accept:** 3.6 table.

### FX12. Console EQs, passive tube EQ, channel strip

* **Depends on:** FX5, FX8, P10.
* **Scope:** `include/crucible/fx/ConsoleCurves.h`, `src/fx/ConsoleCurves.cpp`,
  `src/modules/fx/{EqConsoleBritish,EqConsoleAmerican,EqPassiveTube,ChannelStrip}.cpp`, `tests/ConsoleTests.cpp`;
  optional `third_party/chowdsp_wdf/` (after the licence check).
* **Accept:** 3.3 console table.

### FX13. Saturator, exciter, transient shaper, de-esser

* **Depends on:** P10, FX3, FX5.
* **Scope:** `src/modules/fx/{Saturator,Exciter,TransientShaper,DeEsser}.cpp`, `tests/ColourDynamicsTests.cpp`;
  optional Airwindows (MIT) evaluation recorded in `CHANGELOG.md` (vendored only if measurably distinct, under
  `third_party/airwindows/`).
* **Accept:** 3.4 saturator / exciter rows; 3.2 de-esser / transient rows.

### FX14. Multiband dynamics v2

* **Depends on:** FX3, FX5.
* **Scope:** `src/modules/fx/Multiband2.cpp`, `tests/Multiband2Tests.cpp`.
* **Accept:** 3.2 `fx.multiband2` rows.

### FX15. Stereo imager and utility

* **Depends on:** FX5.
* **Scope:** `src/modules/fx/{Imager,Utility}.cpp`, `tests/ImagerTests.cpp`.
* **Accept:** 3.8 imager and utility rows.

### FX16. Pitch shifter, harmonizer, doubler

* **Depends on:** P13 (Signalsmith Stretch vendored; if FX16 runs first it vendors it, and P13 reuses it).
* **Scope:** `include/crucible/fx/PitchEngine.h`, `src/fx/PitchEngine.cpp`,
  `src/modules/fx/{PitchShift,Harmonizer,Doubler}.cpp`, `tests/PitchFxTests.cpp`.
* **Accept:** 3.7 table.

### FX17. Spring and special reverbs

* **Depends on:** FX6 (shares `Fdn.h` utilities), FX16 for shimmer (otherwise shimmer follows later).
* **Scope:** `src/fx/SpringTank.cpp`, `src/modules/fx/{ReverbSpring,ReverbSpecial}.cpp`, `tests/SpringTests.cpp`.
* **Accept:** 3.5 spring rows; gated / reverse shapes match their envelopes within 1 dB.

### FXR. Reference bench effect mode (analysis side only; the host is not part of this repository)

* **Depends on:** nothing in the engine; any time a worker slot is free.
* **Scope:** `tools/reference-bench/` (effect host mode, signal set, analysers, `targets.json` bands marked
  `pending`).
* **Accept:** `refbench.py selftest` passes the new analysers on synthetic renders; the engine-side 39/39 is
  unchanged.

### Schedule (at most three workers at once, merged with the community plan)

Community dependencies are unchanged; the only change to the community order is that P9 moves from wave A to B
and P13 from B to D to make room for FX1 and FX3 (neither P9 nor P13 has a dependent before wave D / E).

| Step | Packages | Why |
| --- | --- | --- |
| 2 | P2 alone | rename and isolation; every FX package starts after it |
| A | P3, P10, **FX1** | FX1 is new files only; P10's saturation library is the base of FX8 / FX9 / FX12 / FX13 |
| B | P4, P7, P9 | the graph lands; FX modules can register as nodes from here |
| C | P5, P6, **FX2** | legacy fixes after P10 (drive) and P4 (bypass crossfade) |
| D | P8, P13, **FX3** | dynamics core, the most used mixing tool |
| E | P11, P12, **FX4** | maximiser and meters (mastering) |
| F | **FX5, FX6, FX7** | EQ, reverb, convolution |
| G | **FX8, FX9, FX16** | character compressors, tape, pitch (P13 done) |
| H | **FX10, FX13, FX14** | delays and BBD, colour and dynamics tools, multiband |
| I | **FX11, FX12, FX15** | modulation, console EQs and strip, imager |
| J | **FX17** (+ FXR whenever a slot is free) | special reverbs |

If effects are wanted sooner, FX4 and FX5 can swap places with P11 / P12 (analog envelopes and the modulator
bank), which no FX package depends on. Packages that can run alongside community waves: FX1 (wave A), FX2 (C),
FX3 (D), FX4 (E); everything from F on runs after the community plan, or beside its backlog.

---

## 7. Backlog and SKIP

**Backlog:** spectral dynamics (per-bin dynamic EQ and resonance suppression), CloudSeed (MIT) as an extra reverb
algorithm, rotary speaker, vocal chain presets as product definitions, analog console summing across graph lanes
(per-channel nonlinearity and crosstalk in a summing node), stereo tape crosstalk, noise reduction / restoration,
amp and cabinet simulation (needs its own plan), match EQ (spectrum matching from a reference file, control
thread), MS-aware dither shaping, a GPU-free convolution reverb with multiple IR morphing.

**SKIP:** any GPL or LGPL dependency (Rubber Band, SoundTouch, ChowTapeModel, Surge effects, Dragonfly, zita-rev1,
LSP, x42, JUCE's DSP module); copying commercial IRs, presets or curves; product or trademark names in modules,
parameters or presets; sample-level "matches" of commercial plug-ins; analysis on the audio thread (IR import,
auto-gain curves and loudness targets are control-thread work).
