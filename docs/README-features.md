<!-- SPDX-License-Identifier: MIT -->

# Crucible Engine: features and how to use them

Crucible Engine is a polyphonic synthesis and effects engine written as a C++20 library with a plain-C API. It has no
GUI, no plug-in framework and no audio device of its own. You build the instrument, either in C or C++ or as data (a
product JSON file), and Crucible makes the sound. The engine is real-time safe: the audio path never allocates, locks
or blocks, so it can sit inside a plug-in, a game, a hardware firmware or an offline renderer without changes.

This guide has two halves. The first covers what the engine sounds like and what each part is for. The second covers
how to build it, drive it from the C API, design a product, make sounds and read the meters. Facts here come from the
code and from `README.md`, `docs/PARAMETERS.md`, `docs/MODULE_GUIDE.md`, `docs/ARCHITECTURE.md` and
`include/crucible/crucible.h`. Where a parameter or choice list is long, the full list lives in
`docs/PARAMETERS.md`, which is generated from the code.

## Contents

Part one: what it sounds like

1. [What is in the box](#1-what-is-in-the-box)
2. [Voices and oscillators](#2-voices-and-oscillators)
3. [Filters](#3-filters)
4. [Modulation, performance and sequencing](#4-modulation-performance-and-sequencing)
5. [Effects](#5-effects)

Part two: how to use it

6. [Building and linking](#6-building-and-linking)
7. [The C API, step by step](#7-the-c-api-step-by-step)
8. [Parameters](#8-parameters)
9. [Designing a product](#9-designing-a-product)
10. [Sound design workflow](#10-sound-design-workflow)
11. [Metering, views and GUIs](#11-metering-views-and-guis)
12. [Command-line tools](#12-command-line-tools)
13. [The Python package](#13-the-python-package)
14. [Third-party modules](#14-third-party-modules)
15. [Module status](#15-module-status)

---

# Part one: what it sounds like

## 1. What is in the box

* **Voices.** A polyphonic voice pool (1 to 64 voices, 16 by default) with poly, mono and legato modes, glide, and
  voice-level modulation. Each voice has up to eight oscillator slots, a product-chosen number of filter slots, four
  envelopes and four LFOs.
* **Oscillators.** Nine oscillator types, from band-limited wavetables and virtual analog to samples, granular, physical
  modelling and spectral time stretch. Slots can be cross-modulated with FM, ring modulation or hard and soft sync.
* **Filters.** Twenty-five models, including three nonlinear transistor, OTA and diode ladders with oversampled solvers,
  Sallen-Key and state-variable designs, and a passive high-pass.
* **Modulation.** Envelopes, LFOs, six or eight macros, a sixteen-slot modulation matrix, drag-to-assign routes, an
  eight-slot modulator bank, MPE per-note expression, automation clips and a scene morph.
* **Sequencing and performance.** An arpeggiator with a pattern grid, a step sequencer with per-step locks, twelve
  scenes, microtuning (Scala), scale lock with key detection, and a shared key bus for keeping several engines in one key.
* **Effects.** About thirty modules in an orderable rack: saturation, distortion, chorus and ensemble, flanger, phaser,
  delay, reverbs, convolution, dynamics, EQ, vocoder, ring modulation, metering and more.
* **Product system.** A product is a JSON document that names the voice shape, the effect instances, the lanes and sends,
  the host-facing surface and the default values. Patches are versioned JSON with migration, and a preset library sits on
  top.
* **Host side.** A stable C API with opaque handles, a parameter model that describes itself (ranges, units, pages,
  host ids), generic views for meters and scopes, loudness and spectrum analysis, and a versioned module SDK for
  third-party effects.

---

## 2. Voices and oscillators

A voice is built from oscillator slots. The product decides how many slots exist (the layout gives the count, for
example three oscillators in the default product), and each slot picks one of nine oscillator types. Every voice owns an
instance of every type, so a slot can switch type mid-note.

### 2.1 The nine oscillator types

| Type (choice name) | What it is | Sound and use |
| --- | --- | --- |
| Wavetable | Band-limited wavetable with 15 warp modes and unison stacks. Built into the classic voice. | The workhorse. Smooth morphing between tables, clean at any pitch, and bright or hollow depending on the warp. Good for pads, leads and basses. |
| Sample / Granular | Pitched sample playback with loop points, or a cloud of grains read from the same audio. | Samples, vocals, found sounds and textures. Granular mode smears time: clouds, shimmer and stretched pads. |
| Cloud | Additive rendering of a three-dimensional point cloud of partials, morphed along the cloud's z axis. Inharmonic spectra are allowed. | Bells, glassy and metallic timbres, and spectra that do not follow the harmonic series. |
| Noise | White, pink, brown, blue, violet, velvet and crackle noise, with a tilt filter and stereo width. | Hats, wind, breath, risers and percussion layers. |
| Sub | A sine, triangle, square, saw or pulse one to three octaves below the pitch. | Weight under a bass or a kick. |
| Analog (VA) | Virtual analog: saw, pulse with PWM, triangle, sine, saw and pulse mixed, and a shape morph. Band-limited with minBLEP and minBLAMP at exact sub-sample positions. | Classic subtractive sounds. Hard and soft sync, through-zero and exponential FM, ring modulation, drift and part-to-part tolerance, with saw-core, triangle-core and DCO characters. |
| Modal | A modal resonator (up to 64 modes with exact frequency and decay) or a comb string, excited by strike, pluck, particles, blow, noise or bow. | Physical-model percussion, bells, plucked and bowed strings, and resonant objects. Designs follow the Mutable Instruments Rings and Elements modules (MIT). |
| Harmonic | An additive layer on the cloud renderer: partial count, odd and even balance, stretch, spectral tilt, a formant bell fixed in hertz, and per-partial decay over the note. | Organ-like and bell-like spectra you shape directly. Useful for tone design rather than for realistic instruments. |
| Spectral | The slot's sample through a phase-vocoder time stretcher. Speed (0.25x to 4x) and pitch are independent, with formant shift and a forward loop. | Stretched and frozen recordings, pitch-shifted sources that keep their formants, and slow ambient textures. |

### 2.2 Wavetable voice details

The classic voice holds three wavetable oscillators, a sub, a noise source, two filters, four envelopes and four LFOs.

* **Tables.** 22 choices: fourteen factory tables (Sine-Saw, PWM, Formant, Spectral, Analog Saw, Analog Square,
  Analog Tri, Vocal, FM Bells, Additive, Noise Texture, Digital Harsh, Organ, Sync Saw) and eight user tables loaded from
  WAV files. Position morphs through the frames of a table.
* **Warps (15).** Off, FM, Sync Hard, Sync Soft, Bend +, Bend -, Mirror, PWM, Quantize, Asym, Flip, Formant, Smear, Ring
  and Fold. Each warp bends the read position or the shape of a cycle. The character ranges from clean bends to
  metallic FM and hard sync, and each warp can take its modulation from another oscillator (the warp source).
* **Route.** Each oscillator goes to Filter 1, Filter 2, Both, or the dry path past the filters.
* **Unison stacks (8 modes).** Linear, Super, Exp, Octaves, Fifths, Minor, Major and Random. Up to eight detuned voices
  per oscillator, with spread, blend, random phase and voice scatter. Detune, spread, blend and scatter are per-voice
  modulation targets.
* **Sub** (Sine, Triangle, Square, Saw) with one or two octaves down. **Noise** (White, Pink, Brown, Crackle).

### 2.3 Cross modulation and sync

Each slot after the first can take another slot as a cross-modulation source. The modes are **Off**, **FM** (frequency
modulation, bright and bell-like, metallic when the ratio is inharmonic), **Ring** (ring modulation, sum and difference
sidebands, clangorous), and **Sync** (hard sync to the fundamental of the source, the classic sweeping lead). Slots can
only listen to slots before them. The engine's notes say FM and sync need a wavetable carrier in the current build.

### 2.4 Spectral wavetable options

Each wavetable oscillator can opt into spectral processing. **Wavetable Interpolation** is Crossfade (the usual) or
Spectral Sweep, where the spectral peaks slide in frequency instead of fading. **Spectral warps** are Shift, Stretch,
Blur, Comb, Dispersion and Tilt. These give vocal, glassy and smeared timbres that a plain crossfade cannot reach.

### 2.5 Sample and granular, in more detail

* **Mode.** Sample (pitched playback) or Granular (a cloud of grains).
* **Loops.** Off, Forward or Ping-Pong.
* **Grain windows.** Hann, Gauss, Triangle, Trapezoid or Blackman. Softer windows give smoother grains, harder ones give
  more edge.
* **Unison.** Up to eight detuned, panned voices.
* **Slicing.** Off, Per Note (one slice per key) or Key Range (the slices spread across a key range). Good for breakbeats
  and loop-based instruments.
* **Zones.** Off or Zones: several samples mapped by key and velocity, with round robin (Off, Cycle or Random). This is the
  route to drum kits and multisample instruments. SFZ files and folders of samples can be imported.

### 2.6 Analog (VA) details

* **Waves.** Saw, Pulse, Triangle, Sine, Saw + Pulse, or Morph (shape morph between waves).
* **Character.** Ideal, Saw-core VCO, Triangle-core VCO or DCO. These change how the core behaves and how it drifts. The
  VCO characters add a little of the imperfection of real oscillators.
* **Cross mod.** Off, Hard Sync, Soft Sync, Linear FM (through-zero), Exponential FM, or Ring.

### 2.7 Modal details

* **Model.** Resonator (a set of resonant modes) or String (a comb-style string).
* **Exciter.** Strike, Pluck, Particles, Blow, Noise or Bow. The exciter sets the attack and the character of the
  excitation, and the resonator or string rings on afterwards. Use it for percussion and resonant tones that would be
  hard to build from oscillators.

### 2.8 Voice modes, glide and voice cards

* **Voice mode.** Poly (the usual), Mono, or Legato. Mono and Legato have glide and a mono priority (which note wins when
  several are held).
* **Voice cards** (the analog voice). Each voice gets a seeded set of part-to-part tolerances for oscillators, filters,
  envelopes and the amplifier, so a chord sounds like a set of real instruments rather than identical copies. An optional
  noise floor and mains hum (50 or 60 Hz) can be added.

---

## 3. Filters

Each voice has up to two filters. The filter model is chosen per filter (25 choices), and the two filters can be routed
Serial, Parallel or Split. Cutoff, resonance, drive, filter envelope amount and keytrack are the usual controls, and the
nonlinear models add a character control and a component-tolerance control.

### 3.1 The models

Models are listed with the character of what they are, so you can pick one for the sound you want.

| Model | Character and use |
| --- | --- |
| Lowpass 12, Highpass 12, Bandpass 12 | Plain two-pole state-variable filters. Clean and predictable. The default family for gentle shaping. |
| Notch | A two-pole notch: removes a band and leaves the rest. Useful for hollow, phasey tones. |
| Peak | A two-pole peaking filter: boosts a band at the cutoff. Good for formant-like emphasis. |
| Lowpass 24, Highpass 24, Bandpass 24 | Four-pole versions of the state-variable family. Steeper slopes, a more focused bass or treble cut. |
| Ladder LP, Ladder HP, Ladder BP | A four-pole ladder in a linear, zero-delay form with bass compensation at resonance. Smooth, musical and well behaved. |
| Diode LP | A lowpass with a diode-ladder flavour: a softer resonance with a gentle roll-off. |
| Comb + and Comb - | Feedback comb filters tuned to the cutoff, positive or negative. Metallic, pitched resonances, flanger-like and resonant tuned tones. |
| Formant | A bank of three resonant bands that morph through five vowel positions as the cutoff moves. Vocal and talking tones. |
| Phaser | A phaser-style set of notches with feedback, set by the cutoff. Swooshy, comb-like tones from the filter itself. |
| Dirty LP | A two-pole lowpass whose integrators saturate softly. Gets grittier as it is driven, without a separate drive stage. |
| Transistor Ladder | Four tanh-saturating stages with resonance feedback and the classic bass loss of the circuit, which the character control can cancel. Warm, saturating and very ladder-like. The one to reach for on classic bass and lead sounds. |
| OTA Cascade | Four OTA integrators. Keeps more bass than the transistor ladder, its resonance reaches self-oscillation sooner, and its differential pairs saturate softly. Smooth and round. |
| Coupled Diode Ladder | An eighteen-decibel-per-octave ladder of the TB-303 family. The resonance peak is lower, the roll-off onset is gradual, and it gives the squelchy, acid character. |
| Sallen-Key LP, Sallen-Key HP | A two-pole Sallen-Key with a diode clipper in the feedback path, the topology of the MS-20 family. Aggressive, growly and gritty when pushed. |
| OTA SVF | A two-pole state-variable filter built from OTA integrators, morphing between lowpass, notch and highpass. It never self-oscillates, so it stays clean at high resonance. |
| Ota 4-Pole | A four-pole OTA filter with a resonance VCA and partial compensation, in the style of the 1980s polysynth filters. Fat, rounded and synth-like. |
| Passive HP | A passive RC highpass, six or twelve decibels per octave. Linear at zero drive. Used for thinning a sound, and as the fixed highpass step that the nonlinear models can take ahead of them. |

The nonlinear models (the three ladders from the transistor, OTA and diode families, the Sallen-Key, the OTA state-variable
filter and the four-pole OTA) run at up to four times the host rate in the High quality tier, so their saturation does
not alias. The Draft tier runs them at the host rate.

### 3.2 Routing and the highpass step

* **Filter Routing.** Serial (filter 1 into filter 2), Parallel (both filters take the oscillators), or Split (each source
  chooses which filter it reaches).
* **High-pass.** The nonlinear models take a passive high-pass step ahead of them: Off, 60 Hz, 180 Hz or 540 Hz. Useful for
  clearing sub-bass from a resonant ladder.
* **Filter FX.** The same filter models are also available as an effect on the bus (see section 5), with drive and a
  synced LFO sweep.

---

## 4. Modulation, performance and sequencing

### 4.1 Envelopes

The classic voice has four envelopes: Amp, Mod Env, Env 3 and Env 4. Each has attack, decay, sustain, release, hold and
curve controls, and a separate attack curve. The analog voice adds an analog RC mode, where the envelope reshapes as a
real capacitor charges and discharges, with four retrigger behaviours (From Level, Reset, Legato and ADR) and velocity
and key time scaling. Use the analog mode when you want envelopes that feel less perfect and more played.

### 4.2 LFOs

Four LFOs, each with a rate, a shape and an amount. The twelve shapes are Sine, Triangle, Saw, Square, Sample and Hold,
Saw Down, Smooth Random, Pulse 25%, Stairs, Rect Sine, Exp Decay and Custom. Each LFO can retrigger on each note, run
free, or run once (One-Shot), and can sync to the tempo over a range of note values.

### 4.3 Macros and scenes

* **Macros.** Six macros in the default layout (eight in a product that asks for them). A macro is a performance knob that
  you map to several parameters at once through the modulation matrix. A single macro can open a filter, bring in a
  delay and widen a stereo image at the same time.
* **Scenes.** Twelve scene slots store complete snapshots of the sound. A scene morph moves between two scenes (Scene A
  and Scene B) with a morph control, and a scene can be recalled into a slot.
* **Vector field.** A four-corner morph between stored scenes, with smoothing, an orbit and drift. Good for evolving pads
  that move across several sounds.

### 4.4 Modulation matrix and routes

* **Matrix.** The classic voice has a sixteen-slot modulation matrix, each slot with a source, a destination and a
  polarity (Native, Unipolar or Bipolar).
* **Routes.** Any modulatable parameter can take a route from the list of sources: the LFOs, envelopes, macros, MIDI
  controllers (CC), velocity, key, random, the mod wheel, aftertouch, pitch bend, the strips, the scene morph, clip lanes,
  modulator bank outputs and audio-rate sources. A route has an amount (-1 to 1), a polarity, a curve and a smoothing
  time. A route can also target the amount of another route, so modulation can modulate modulation.
* **Modulator bank.** Eight modulators evaluated in order, each reading the sources or an earlier modulator: combine, remap
  curve, lag, quantise, sample and hold, Turing, binary, chaos, note gate, follower, pitch tracker and LFO table. They run
  per voice, per layer or globally. Use them for stepped, random or evolving modulation that a single LFO cannot make.
* **Audio-rate modulation.** An oscillator or an audio-rate LFO can drive pitch, FM amount, pulse width, wavetable position,
  filter cutoff or amplifier per sample, for FM-like and ring-like behaviour from any source.

### 4.5 MPE and expression

Per-note pitch bend, pressure and slide from an MPE controller are route sources, so each note can respond to its own
expression. The mod wheel, aftertouch and pitch bend are also available as sources. Glide and mono modes are in the voice
settings.

### 4.6 Automation clips and the motion recorder

Clips are multi-lane automation: curve and step lanes, a loop, transport sync and an MSEG shape. A lane is a route
source, so a clip can drive any parameter. The motion recorder writes live parameter moves into a lane, so a knob turned
during a performance becomes an automation curve.

### 4.7 Arpeggiator

Eight modes: Up, Down, Up-Down, Down-Up, Random, As Played, Chord and Converge. Sixteen rates from whole notes to thirty-
second-note triplets, plus Free. The clock is the host tempo or an internal one. Velocity is As Played, Fixed or Accent.
The pattern grid takes ties (hold the previous note), ratchets (one to four notes per step), probability, probability
groups (alternatives that fire one of their members per pass), transpose and a grid direction (Forward, Reverse,
Ping-Pong or Random). The grid is 32 steps long, which lets you build rolling, stuttering and gated arpeggios that
still feel played.

### 4.8 Step sequencer

Twelve pattern slots. Each step has a note, a velocity, a gate length, an octave, a probability, ties and ratchets.
Patterns run from sixteen steps per bar to sixteen bars, so a pattern can be as long as 256 steps. Four parameter locks
per step let a step change a sound (a filter setting, a level) while it plays. Patterns can chain, run forward or in
other directions, and sync to the host or an internal clock with a swing setting.

### 4.9 Key lock and scales

Key lock keeps the notes you play in a scale. There are three behaviours: **Snap** (an out-of-key note plays the nearest
in-key note), **Block** (out-of-key notes are muted) and **Remap** (the white keys play the scale degrees, so every key is
in key). The root is one of twelve pitch classes, the scale is one of eighteen (Major, Natural Minor, Harmonic
Minor, Melodic Minor, Dorian, Phrygian, Lydian, Mixolydian, Locrian, Major and Minor Pentatonic, Blues, Whole Tone, both
Diminished scales and more) or a custom mask. Snap Tie (Down or Up) chooses which way a note on the edge snaps. Key detection
proposes a key from the notes played so far.

Key Sync (Off, Follow or Lead) lets several engines follow one key through a key bus. Inside one process the bus is
built in. Between processes the host supplies the transport.

### 4.10 Microtuning

Scala `.scl` and `.kbm` files are supported, with per-note offsets. This gives just intonation, historical temperaments and
any tuning you can describe in Scala.

---

## 5. Effects

Effects sit in a rack in an order you choose (the product gives a position to each instance). Each has an on switch and,
where it makes sense, a mix. The first-generation effects (drive, chorus, dimension, flanger, phaser, delay, reverb, convolver, compressor, gate,
EQ and parametric EQ) were removed before v1; their successors are described below.

### 5.1 Saturation and distortion

**Saturator** (`fx.saturator`, current). Sixteen models: Soft Clip, Hard Clip, Op-Amp Rail, Diode, Wavefolder, Tube
Triode, Tube Push-Pull, Tube Pentode, Tape and Transformer, then six shapers added at the end of the list (patches keep
their choice numbers): Rectify Half, Rectify Full (octave-up, even harmonics only), Sine Fold, Linear Fold (triangle fold,
first fold at half the level of Wavefolder), Zero-Square (a near-square wave from a small signal) and Asymmetric (the
negative half clips early, even harmonics). The rectifiers and Asymmetric make DC, which the module's DC blocker takes
out; the folding and steep shapers have a shorter usable drive range, so the Drive knob does less at the top of its scale
for them. Anti-aliased with antiderivative anti-aliasing and oversampling
chosen by quality and drive, with a tilt control, loudness-matched auto gain, bias, output and mix. The models differ in
character: the tube models are rounded and add harmonic thickness, tape gives a soft, hysteresis-style compression, and
the rail and hard clips are the bright, aggressive end. Use it as the general-purpose colour stage.
An optional resonant filter (page "Saturator Filter": Filter Off / Pre / Post, Low-pass / Band-pass / High-pass, cutoff
20 Hz to 20 kHz, resonance 0 to 1, Q from 0.7 to 21) sits before the shaper, ahead of the oversampler so it cannot
alias, to shape what gets distorted (a low-pass keeps the highs from generating harmonics), or after it on the wet path
to tame the result. Cutoff and resonance glide without zipper noise. Off is the default and bit-identical to earlier
versions; the auto-gain does not compensate the filter, so it acts as a tone control.

**Shaper** (`fx.shaper`, current). A waveshaper whose transfer curve you draw: 2 to 64 points (x and y within -1..1, x
ascending) with a curvature per segment (`y = y0 + (y1 - y0) * t^(4^curve)`, 0 is a straight line, positive bows towards the
start value, negative towards the end value), optionally symmetric (draw the positive half, it is mirrored, odd). Set with
`crucible_set_shaper_curve` (C) or `Engine::setShaperCurve` (C++; instance ids as for `crucible_load_ir`: `fx.shaper`,
`fx.shaper.2`); the curve is published to the audio thread as an immutable table, cross-faded in over 10 ms, saved in
patches (`"shaperCurves"`) and readable for drawing as the pulled view `fx.shaper/curve` (257 values at x = -1 .. +1). Drive
(pre gain, -24 to +24 dB), output, mix, an optional DC block and 1x to 8x oversampling (Auto: 2x / 4x / 8x by quality; the
latency follows and is reported). The default curve is the identity: at 0 dB drive the module passes the input through bit
for bit (delayed by the reported latency), and the identity is the unbounded straight line, so it never clips. Any other
curve holds its end values beyond the first and last point, so a curve that ends at +-0.5 is a hard clip at +-0.5. There is
no antiderivative anti-aliasing: hard corners rely on the oversampler (5 kHz, -6 dBFS, +12 dB drive into a hard clip:
-14 / -29 / -39 / -61 dB alias power at 1x / 2x / 4x / 8x).

**Multiband Drive** (`fx.multiband_drive`, current). A three-band distortion: two Linkwitz-Riley crossovers (24 dB per octave,
minimum phase, 40 to 1000 Hz and 800 to 12000 Hz; the sum of the bands is flat and phase-coherent with the crossover's
all-pass) split the input into low, mid and high bands. Every band has its own model (the ten of the Saturator), drive
(0 to 36 dB), output (-24 to +6 dB), mix, solo and bypass; the global controls are the two crossover frequencies, a mix against
the delayed input and an output trim (-24 to +3 dB). With every drive at 0 dB the module is transparent (flat within 0.1 dB
from 20 Hz to 20 kHz); a band at 0 dB drive, bypassed or muted by another band's solo is not processed at all. Typical use:
heavy drive on the low band with a clean top, or a bright fold on the high band only. The bands run the same anti-aliased
models as the Saturator (4x oversampling in Standard, the high band 8x), so it costs about one Saturator per driven band
(Standard about 2 % of a core, High about 3.6 %, idle bands cost nothing but the crossover, 0.5 %). Latency (reported) is the
oversampling filters; changing a model dips that band's wet share briefly while the new model takes over.

**Clipper** (`fx.clipper`, current). A clipper with a ceiling in dBTP. The curve morphs from a hard clip to tanh, with
first- or second-order anti-aliasing and oversampling at 1x, 2x, 4x or 8x (Auto follows the quality policy), and a true-peak
guard that trims inter-sample overs. Use it for loud, controlled clipping before a limiter.

**Crusher** (`fx.crusher`, current). Bit-depth and sample-rate reduction with jitter and dither. Lo-fi, digital grit and
sample-rate aliasing. Use it for character, not transparency. Track makes the hold rate follow the last played note (note
frequency x ratio, plus a fine control in cents), so the aliasing stays tuned to what you play; off, the rate is fixed.

**Exciter** (`fx.exciter`, current). A harmonic exciter and bass enhancer. High mode takes the band above a frequency
(1 to 10 kHz) and adds brightness and air. Low mode takes the band below a bass frequency (40 to 250 Hz) and adds
low-end presence that is audible on small speakers. Anti-aliased and oversampled; latency is reported.

**Cabinet** (`fx.cabinet`, current). A guitar and bass speaker cabinet with no recorded impulse responses: the response of each
of eight cabinets (1x12, 2x12, 1x10 and 2x10 open back or combo, closed 4x12, bass 1x15, 4x10 and 8x10) is generated from a model
of the box resonance, the cone's breakup peaks and dips, the high roll-off and, for open backs, the back-wall comb. Mic position
(centre to edge) tilts the top end and moves a notch; mic distance adds proximity bass and a room reflection. Low cut, high cut,
output level and mix follow. Put it after an amp-style saturator; latency is 0. Changing the cabinet or the mic crossfades to the
new response over 40 ms.

### 5.2 Modulation effects

**Ensemble** (`fx.ensemble`, current). A chorus of bucket-brigade devices. String ensemble runs one to three lines whose
oscillators are spread around the cycle and panned across the stereo field. Dimensional I to IV run two lines, one per
channel, at four fixed rate and depth steps, with crossfeed between them. Depth is in cents of pitch deviation, the
oscillator is a triangle, a sine or smooth random, and a darkness control shapes the wet signal. Warm, lush and slightly
analogue: the sound of a vintage string machine.

**Flanger 2** (`fx.flanger2`, current). A through-zero flanger. A fixed reference path and a modulated path cross at zero
offset and go negative, as between two tape machines, which gives the classic full-cancellation sweep. Digital or BBD
mode, LFO or envelope source, sine or triangle shape, 256 or 512 BBD stages. It reports a fixed 249-sample latency.

**Phaser 2** (`fx.phaser2`, current). Two, four, six, eight or twelve first-order all-pass stages, swept by an LFO, the
input envelope or the sidechain. Linear, OTA or JFET stage characters. Barber-pole mode (off, up or down) for the
endless-sweep illusion. The sweep is smooth and the stage count sets how dense the notches are.

**Tremolo** (`fx.tremolo`, current). Tremolo and auto-pan with five shapes (Sine, Triangle, Square, Saw Up, Saw Down) and
tempo sync. The square shape gives gated, rhythmic chops; the sine shape gives a smooth pulse.

**Ring / Shift** (`fx.ring`, current). A ring modulator (Ring mode) and a single-sideband frequency shifter (Shift Up and
Shift Down). Ring mode gives bell-like, clangorous, inharmonic tones. The shifter moves every partial by the same number
of hertz, which makes an unusual, detuned, alien sound.

**Vocoder** (`fx.vocoder`, current). A channel vocoder of 16 to 32 bands. The spectrum of the sidechain input shapes the
synth bus, with a carrier of synth, saw, noise or saw plus noise. Formant shift moves the carrier bands, and sibilance
passes the modulator's highs so consonants stay clear. It needs a sidechain input (see section 11.4 on process_aux).

### 5.3 Delay

**Delay Pro** (`fx.delay_pro`, current). A stereo delay with two engines. **Digital** reads the line with
Kaiser-windowed sinc interpolation, so the time is exact and the response is flat to 18 kHz at any fractional time, even
while modulated. **Analog** is a bucket-brigade device: its bandwidth falls as the time grows, with companding noise, a
little breathing and a soft clip, the sound of a tape or BBD echo. Options include tempo sync, stereo modes (Stereo,
Ping-pong and Cross), time changes that glide or crossfade, analog stage counts (256 to 4096), a ducking mode driven by the
input or the sidechain, a right-channel time ratio from 0.25 to 2, feedback with saturation, and a freeze. Use Digital for
clean, precise echoes and Analog for warm, dark, slightly unstable repeats.

### 5.4 Reverb and convolution

**Algorithmic Reverb** (`fx.reverb_algo`, current). Five reverbs: **Room**, **Hall**, **Chamber**, **Plate** and
**Ambience**. Room, Hall, Chamber and Ambience are a 16-line feedback delay network (32 lines at High quality) whose decay
time is set separately at low, middle and high frequencies, behind an all-pass input diffuser and early reflections.
Plate has its own character. Controls are decay (0.1 to 60 seconds, 2.5 by default), size, predelay, density, early
level, and low and high multipliers that shape the decay across the spectrum. Use Room and Chamber for rooms and studio
spaces, Hall for large spaces, Plate for bright, dense, metallic-smooth reverb, and Ambience for soft, diffuse space
that sits behind a mix.

**Convolution** (`fx.convolution`, current). Convolution with true impulse-response import: mono, stereo or true-stereo
(four kernels, left and right in, left and right out) responses at any sample rate, up to 30 seconds. Choose Zero Latency
mode (a direct head for a low-latency start) or Low CPU mode. Use it to put a real room, a real cabinet or a real spring
under your sound.

### 5.5 Dynamics

**Clean Compressor** (`fx.comp_clean`, current). A transparent digital compressor on the shared dynamics core. Peak, RMS or
hybrid detection (optionally a 4x true-peak detector), feed-forward or feedback topology, a knee up to 24 dB, a ratio up to
infinity, downward or upward operation, and an internal or external key. Use it to even out a sound without changing its
character.

**VCA Bus Compressor** (`fx.comp_vca`, current). A feed-forward bus compressor with a smoothed-RMS detector. Ratios of 1.5,
2, 4 and 10 to 1, attack from fast to slow, and releases from 0.1 seconds to Auto. The glue-bus sound: it holds a mix
together and adds a little punch.

**Gate Expander Ducker** (`fx.gate2`, current). Expander, gate or ducker on the shared dynamics core. Ratios from 1:1 to
1:infinity, range, hysteresis, hold, a peak, RMS or hybrid detector and an internal or external key. Use the gate to clean
up noise, the expander for a gentler version, and the ducker to make one sound step aside for another (a pad that ducks
for a kick, for example).

**Multiband Compressor** (`fx.multiband`, current). Three bands with LR4 crossovers and gain-reduction meters. Tames the
bass without dulling the top, or controls a mix band by band.

**Trance Gate** (`fx.trance`, current). A tempo-synced sixteen-step gate with shaped edges, so a sound chops in rhythm. Rates
from quarter notes to sixteenth-note triplets and dotted values.

**Transient Shaper** (`fx.transient`, current). A level-independent transient shaper. A fast and a slow follower compare the
onset with the decay, so the attack and sustain can be boosted or cut by up to 15 dB without depending on how loud the input
is. Lookahead up to 5 ms. Use it to add snap to drums or to tame a sharp pluck.

**De-esser** (`fx.deesser`, current). Reduces sibilance on a voice or a bus. Split-band mode lowers only the sibilant band,
cut by a linear-phase filter so it stays in phase; wide-band mode turns the whole signal down. The detector reads a band of
two to sixteen kilohertz, and the threshold is relative to the band's share of the level or absolute. Lookahead up to 2 ms.

**Maximizer** (`fx.maximizer`, current). A lookahead true-peak limiter and maximiser with a ceiling in dBTP that is never
exceeded. A 4x sidechain or the whole path at 4x, lookahead from 0.5 to 10 ms (reported as latency), a hold that adapts to
the bass period, fixed, dual-stage or crest-adaptive release, stereo link or a mid/side bias, an optional soft-clip stage,
and dither (Off, TPDF or Shaped) at 16, 20 or 24 bits. Use it as the last stage to make a master loud without overs.

**Limiter** (`fx.limiter`, current). A lookahead peak limiter for plain limiting and protection.

### 5.6 EQ and filtering

**Pro EQ** (`fx.eq_pro`, current). A sixteen-band precision parametric equaliser: bell, shelves, tilt, band-pass, notch,
all-pass and low and high cuts up to 96 dB per octave, in Butterworth, Bessel, Linkwitz-Riley or resonant alignments. It
draws its own response curve and spectrum for a GUI. The general-purpose EQ for mixing and mastering.

**Filter** (`fx.filter`, current). Any of the voice filter models (the same 25 choices as section 3) on the bus, with drive
and a synced LFO sweep. The filter description in the code still says 17 models; the parameter list offers all 25. Use it
to put a Moog-style ladder or a Sallen-Key across a whole mix or a drum bus.

**Width** (`fx.width`, current). Mid and side width, a bass-mono option and an output pan. Widen the top without spreading
the bass.

**Meter** (`util.meter`, current). Loudness and level metering that passes audio through unchanged. ITU-R BS.1770-4 and EBU
R128 momentary and short-term loudness, true peak and peak programme meters (Type I or II). Put it anywhere in the chain to
read a level.

---

# Part two: how to use it

## 6. Building and linking

### 6.1 Requirements

CMake 3.22 or newer, a C++20 compiler (Apple clang, gcc 11 or newer, clang 14 or newer, or MSVC 2022), and threads. On
Apple platforms the system Accelerate framework provides the FFT; elsewhere a portable or pffft backend is used. The build
needs no network access. The library itself has no third-party code; `third_party/` holds code used by tests only.

### 6.2 Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

`tools/ci.sh` runs the full matrix: Release, ASan and UBSan, TSan and a scalar build. `tools/check-spdx.sh` checks licence
headers.

### 6.3 CMake options

| Option | Default | What it does |
| --- | --- | --- |
| `CRUCIBLE_ENGINE_BUILD_TESTS` | on at top level | Builds `crucible_engine_tests` and the other test executables. |
| `CRUCIBLE_ENGINE_BUILD_EXAMPLES` | on at top level | Builds the example programs (`render_wav`, `render_c`, the instrument skeleton, the module examples). |
| `CRUCIBLE_ENGINE_BUILD_TOOLS` | on at top level | Builds `crucible-render`, `crucible-measure` and `crucible-module-check`. |
| `CRUCIBLE_ENGINE_INSTALL` | on at top level | Generates install rules and the `CrucibleEngine` package files. |
| `CRUCIBLE_ENGINE_SHARED` | off | Builds the C API as a shared library (`libcrucible`) for FFI hosts such as the Python package. |
| `CRUCIBLE_ENGINE_MODULE_LOADING` | on | Compiles support for loading third-party module shared libraries. Off removes library loading; `crucible_register_module` still works. |
| `CRUCIBLE_ENGINE_SANITIZE` | empty | Semicolon-separated sanitizers: `address`, `undefined`, `thread`. Needs gcc or clang; MSVC is refused. |
| `CRUCIBLE_ENGINE_FFT_BACKEND` | `AUTO` | `AUTO`, `ACCELERATE` (Apple only), `PFFFT` or `PORTABLE`. |
| `CRUCIBLE_FORCE_SCALAR` | off | Disables the SIMD kernels and uses scalar code everywhere. Use it to test the fallback that platforms without NEON or SSE take. |
| `CRUCIBLE_X86_AVX2` | off | Compiles x86 builds for AVX2 (gathers) and FMA. |

`CRUCIBLE_ENGINE_BUILD_TESTS`, `BUILD_EXAMPLES`, `BUILD_TOOLS` and `INSTALL` default to on only when the engine is the
top-level project. Under `add_subdirectory` they default to off, so your project does not build the engine's tests
by accident.

Example: a sanitized debug build on the scalar code path.

```sh
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug \
      -DCRUCIBLE_ENGINE_SANITIZE="address;undefined" -DCRUCIBLE_FORCE_SCALAR=ON
```

### 6.4 Consuming the engine

As a subdirectory:

```cmake
add_subdirectory(path/to/crucible-engine)
target_link_libraries(my_synth PRIVATE crucible::engine)
```

From an install:

```sh
cmake --install build --prefix /opt/crucible
```

```cmake
find_package(CrucibleEngine 0.1 REQUIRED)
target_link_libraries(my_synth PRIVATE crucible::engine)
```

The package file is generated from `cmake/CrucibleEngineConfig.cmake.in` and needs Threads. `examples/consumer/` is a
complete project that does this, and `tools/test-consumer.sh <build-dir> [scratch-dir]` installs a build tree into a
scratch prefix, configures and builds the consumer against it, and runs it. It reads the sanitizer setting from the build
tree's cache so the consumer links the same runtime.

For C, include `<crucible/crucible.h>` and link `crucible::engine` the same way. The CMake target defines `CRUCIBLE_STATIC`
for the static library; a shared build is described in `crucible_export.h`.

---

## 7. The C API, step by step

The C API is in `include/crucible/crucible.h`. It is plain C with opaque handles and no exceptions. The header tags every
function with the thread that may call it:

* **[audio]**: the audio thread. Only `crucible_process`, `crucible_process_host`, `crucible_process_aux`,
  `crucible_process_events`, and `crucible_reset` (which may also run on the control thread while the audio thread is
  outside a process call).
* **[main]**: the host's main thread or a timer that you serialise with it. May allocate, lock and read files.
* **[any]**: any thread, including audio. Lock-free, no allocation. Immutable metadata and atomics.
* **[main, not concurrent with process]**: `crucible_prepare` and `crucible_destroy` only.

The note input calls (`crucible_note_on`, `_note_off`, `_pitch_bend`, `_mod_wheel`, `_sustain`, `_all_notes_off`) are
[any] but feed a single-producer queue: call them from one thread at a time. `crucible_midi` shares that queue but is [main].

### 7.1 Create

```c
#include <crucible/crucible.h>

CrucibleConfig config;
crucible_config_init(&config);          /* fills every default and sets struct_size */
config.max_voices = 8;                  /* 1..64, 0 = 16 */
config.quality = 1;                     /* 0 draft, 1 standard, 2 high */

/* [main] NULL config means defaults; product_json is required (crucible_create(NULL) plays the default product). */
char error[1024] = "";
CrucibleEngine* engine =
    crucible_create_from_product(&config, product_json, NULL, error, sizeof error);
if (engine == NULL) { fprintf(stderr, "product rejected:\n%s\n", error); return 1; }
```

`crucible_create(const CrucibleConfig*)` builds the process default product instead. The product JSON format is in section 9.
The `modules` argument is a registry of third-party modules (section 14), or NULL for the built-in ones. Every engine owns its
product, so several engines with different products can live in one process.

### 7.2 Prepare

```c
crucible_prepare(engine, 48000.0, 512);   /* [main, not concurrent with process] */
```

Call it before the first process call and whenever the sample rate changes. It allocates the voices and effects.

### 7.3 Resolve parameters once

Parameter ids are strings, and looking them up allocates. Look them up once and keep the indices.

```c
int cutoff = crucible_param_index(engine, "filter_cutoff");   /* [any] -1 when unknown */
crucible_set_param(engine, cutoff, 1800.0f);                  /* [main] natural units, clamped */
```

### 7.4 The process loop

On the audio thread, `crucible_process_events` takes everything a plug-in host delivers in one call: MIDI events, parameter
changes and the host transport.

```c
/* [audio] One block of 512 frames. */
CrucibleMidiEvent midi[2] = {
    {0,   0x90, 60, 100},   /* note on C4, position 0 */
    {256, 0x80, 60, 0},     /* note off, position 256 */
};
CrucibleParamEvent changes[1] = {
    {0, cutoff, 3000.0f},   /* applied at the start of the block */
};
CrucibleTransport transport;
transport.struct_size = sizeof transport;
transport.bpm = 120.0;
transport.ppq = -1.0;       /* < 0: no position */
transport.playing = 1;

crucible_process_events(engine, left, right, 512, midi, 2, changes, 1, &transport);
```

Rules from the header: MIDI events must be sorted by position, and at most `CRUCIBLE_MAX_EVENTS_PER_BLOCK` (1024) are used per
call. Parameter events must also be sorted, and their `position` is reserved: today every change applies at the start of the
block, so 0 is fine. Int, choice and bool values are rounded, as `crucible_set_param` does. `transport` may be NULL, which
means no tempo, no position and stopped.

For a simpler host, `crucible_process(engine, left, right, n, events, num_events)` renders with MIDI only, and
`crucible_process_host` adds tempo and transport. `crucible_process_aux` adds a sidechain input (section 11.4).

A block that has no notes, no changes and no tail is silent and cheap.

### 7.5 Service

Some parameter changes need setup work that the audio thread must not do: a changed oscillator type, a new wavetable, a
sequencer pattern. The audio thread only marks them. The control thread finishes them:

```c
/* [main] from a timer, or after a block when pending */
if (crucible_service_pending(engine))   /* [any] one atomic load */
    crucible_service(engine);           /* [main] */
```

Call `crucible_service` regularly. An offline bounce has no timer, so it calls `crucible_service_pending` between blocks.

### 7.6 Report changes to the host

The engine changes parameters on its own (a patch load, a mapped CC, a scene recall). Tell the host about them:

```c
int indices[64], sources[64];
int count;
while ((count = crucible_param_changes(engine, indices, sources, 64)) > 0)   /* [main] */
{
    for (int k = 0; k < count; ++k)
    {
        if (indices[k] == CRUCIBLE_CHANGE_LATENCY) { /* ask for crucible_latency_samples */ }
        else if (indices[k] == CRUCIBLE_CHANGE_TAIL) { /* ask for crucible_tail_seconds */ }
        else if (sources[k] != CRUCIBLE_SOURCE_HOST) { /* send a parameter value to the host */ }
    }
}
```

`sources[k]` says who made the change: `CRUCIBLE_SOURCE_HOST` (the host already knows), `_MIDI` (a mapped CC) or `_INTERNAL`
(patch load, scene, reset, latency). Only exposed parameters are reported.

### 7.7 Latency, tail and reset

```c
int latency = crucible_latency_samples(engine);   /* [main] lookahead, linear-phase, oversampling: whole samples */
double tail = crucible_tail_seconds(engine);      /* [any] cached; HUGE_VAL when a freeze never decays */
crucible_reset(engine);                           /* [audio] clear voices, notes and effect state */
```

Report the latency to the host so it can compensate, and report the tail so it keeps the sound alive after notes end. Call
`crucible_reset` on activate, on a transport jump and on bypass. It clears voices, held notes and every effect's state
(delay lines, reverb tanks, convolution history, filters), but keeps parameters, patches, scenes and loaded content.

### 7.8 Destroy

```c
crucible_destroy(engine);   /* [main, not concurrent with process] */
```

### 7.9 A complete offline render

`examples/c_api/render_c.c` is a full program that does all of this to write a WAV file. The core of it:

```c
CrucibleEngine* engine = crucible_create(NULL);
crucible_prepare(engine, 48000, 256);
crucible_set_param_id(engine, "filter_cutoff", 2400.0f);
crucible_note_on(engine, 57, 0.8f);
for (uint32_t pos = 0; pos < frames; pos += 256) {
    const int n = frames - pos < 256 ? (int)(frames - pos) : 256;
    if (pos == 72192) crucible_note_off(engine, 57);   /* release after about 1.5 s */
    crucible_process(engine, left + pos, right + pos, n, NULL, 0);
}
crucible_destroy(engine);
```

### 7.10 A plug-in skeleton

`examples/instrument_skeleton/main.c` is a complete plug-in lifecycle with a fake host and no GUI toolkit: create from a
product, build the host parameter list from the surface, activate, process, tick on the main thread, read views for a GUI
and save and load state. Copy the functions named after host callbacks into your wrapper, and run `ctest` to see it pass.

### 7.11 Thread summary

| Thread | What it may call |
| --- | --- |
| Audio | `crucible_process*`, `crucible_reset`, and every [any] call. |
| Main | every [main] call and every [any] call, one main thread at a time. |
| Note input | the six note and bend calls, from one thread at a time. |
| Any | reads of values and meters, pushed views, the key bus, `crucible_service_pending`, `crucible_tail_seconds`. |

---

## 8. Parameters

### 8.1 Finding and reading them

The parameter list is the same for every instance of one product and never changes after creation.

* **By id.** `crucible_param_index(engine, "filter_cutoff")` (any thread, no allocation; a binary search). Ids are stable across releases:
  renamed ids are never reused, and new parameters are appended.
* **By host id.** Each parameter has a 31-bit `host_id` (FNV-1a of the id, top bit cleared), the number a CLAP, VST3 or AU
  host stores. `crucible_param_index_by_host_id` finds the index from a host event with a table lookup, safe on any thread.
* **Metadata.** `crucible_param_meta(engine, index, &meta)` fills a `CrucibleParamMeta`: id, name, short name (six
  characters or fewer), unit, module, group, page, type (float, int, choice or bool), range, default, skew, steps,
  smoothing, polarity, UI hint (knob, slider, switch, menu, XY or drawbar), modulatable, voice-modulated, automatable,
  routable, exposed and host id. Set `meta.struct_size = sizeof meta` first.
* **Choices.** `crucible_param_choice(engine, index, k)` gives the name of choice k. The number of choices is in the
  metadata. Choice numbers stay the same in every product: `crucible_choice_available` says whether a product includes
  that choice (for example an oscillator type it leaves out).

### 8.2 Natural units, normalised values and text

* **Natural units.** The usual calls take natural values: hertz, seconds, dB, semitones and the choice index. Setters clamp to
  the range.
* **Normalised.** `crucible_set_param_normalised(engine, index, value01)` takes a 0 to 1 value, for hosts that work that way.
* **Text.** `crucible_value_to_text(engine, index, value, buffer, size)` gives the display text ("1.5k", "-6dB", "C#4").
  `crucible_text_to_value` parses text back to a value. `crucible_set_param_text(engine, "osc1_type", "analog")` sets a choice
  from its name in any letter case, or without its trailing bracket ("analog" for "Analog (VA)"). A short form that fits two
  choices is refused.
* **Modulated.** `crucible_get_param` is the stored value. `crucible_modulated_value` is the value after global modulation as
  of the last process call: what a knob should show when an LFO is moving it.

### 8.3 The surface

A product can declare a surface: the parameters a plug-in host lists, in the order and on the pages you choose.
`crucible_surface_count` and `crucible_surface_param(engine, k)` walk it. Without a surface, every automatable parameter is
exposed in index order. Hidden parameters still load from patches and remain modulation targets. `exposed` in the metadata
tells you which are on the surface.

### 8.4 Changing parameters

| Call | Thread | Use |
| --- | --- | --- |
| `crucible_set_param` / `_id` / `_text` / `_normalised` | main | Setup and UI. May build wavetables or oscillators, so not on the audio thread. |
| `crucible_process_events` parameter list | audio | Real-time changes from the host, applied at the start of the block. |
| `crucible_midi` with a mapped CC | main | Raw MIDI that may apply a mapped controller. |
| `crucible_map_cc(engine, cc, index)` | main | Maps a CC to a parameter. `-1` removes the mapping. |

Use indices on the fast path. Resolve ids once at setup.

### 8.5 MIDI controllers

`crucible_map_cc` maps a controller number to any parameter. For CC learn and the full mapping (ranges, curves, relative
encoders and channels), the CC map in `CcMap.h` provides it. A CC can also be a route source: source value `1000 + CC
number` in `crucible_add_route` (section 10.2). Note-based controllers (pitch bend, mod wheel, aftertouch, sustain) are
separate calls.

---

## 9. Designing a product

A product is a `crucible-product` JSON document. It describes the instrument as data: the voice shape, the effect
instances, how they are wired, what a host sees, and default values. The engine validates it and reports every error at
once, so a broken product fails cleanly at creation.

### 9.1 The template

`docs/products/template.json` is the starting point. Here is its shape, annotated.

```json
{
  "format": "crucible-product",
  "version": 1,
  "id": "my.product",
  "name": "My Product",
  "layout": { "oscillators": 2, "filters": 2, "macros": 6 },
  "voice": "classic",
  "fx": {
    "instances": [
      { "id": "saturator", "type": "fx.saturator" },
      { "id": "ensemble",  "type": "fx.ensemble" },
      { "id": "delay",     "type": "fx.delay_pro" },
      { "id": "reverb",    "type": "fx.reverb_algo" },
      { "id": "comp",      "type": "fx.comp_clean" },
      { "id": "eq",        "type": "fx.eq_pro" },
      { "id": "limiter",   "type": "fx.limiter" }
    ],
    "lanes": [
      { "id": "main", "order": "listed", "input": ["voices"],
        "chain": ["saturator", "ensemble", "delay", "reverb", "comp", "eq", "limiter"] }
    ]
  },
  "surface": {
    "expose": "listed",
    "sections": [ { "id": "sound", "name": "Sound" }, { "id": "fx", "name": "Effects" } ],
    "pages": [
      { "id": "osc1", "name": "Osc 1", "section": "sound", "params": ["osc1_table", "osc1_pos", "osc1_level"] },
      { "id": "filter", "name": "Filter", "section": "sound", "params": ["filter_cutoff", "filter_res"] },
      { "id": "amp", "name": "Amp", "section": "sound",
        "params": ["amp_attack", "amp_decay", "amp_sustain", "amp_release", "master_gain"] },
      { "id": "saturator", "name": "Saturator", "section": "fx", "params": ["fx_saturator_drive"] }
    ]
  }
}
```

Walk through it:

* **`layout`** sets the shape of the voice. `oscillators` and `filters` are the slot counts, and `macros` is 6 or 8. Anything
  that depends on these counts comes from the layout.
* **`voice`** is `"classic"` (the wavetable voice with its slots), `"none"` (no voices, an effects-only product), or a graph
  of nodes and edges (advanced; see the drum-kit product below).
* **`fx.instances`** names each effect instance by a local id and a module type. The id is used by lanes and sends. Two
  instances of the same type are allowed; the second and later ones get their own parameter ids (`fx2_...`).
* **`fx.lanes`** are the signal paths. Each lane takes an `input` (`voices`, or `aux` for the host's sidechain input) and a
  `chain` of instance ids, run in the order listed (`"order": "listed"`) or by each instance's position (`"position"`). A
  lane has a `level`, and `"master": false` keeps a lane off the master bus (lanes feed the master by default).
* **`fx.sends`** feed one lane into another with a gain, for a reverb send, for example.
* **`fx.master`** is an optional object, `{"level": ...}`, that sets the master level.
* **`surface`** is what a plug-in host lists. `expose` is `"all"` (every automatable parameter, the default) or `"listed"` (only
  the parameters you list). Sections and pages organise them. A parameter appears at most once.

### 9.2 The other sample products

`products/` holds four sample products. They are small and useful for seeing each feature in use.

* **`crucible-default.json`** (`crucible.default`): the neutral product. Three oscillators, two filters, six macros and every
  built-in effect in one position-ordered chain, with the flat parameter ids of every release. It is what you get with the
  process default. Read it when you want to see every effect's id.
* **`fx-only.json`** (`crucible.fx-only`): no voices. The host's sidechain input (`"input": ["aux"]`) runs through a saturator and a
  delay in series, with a reverb on a parallel lane fed by a send of 0.3. Use this shape for a pure effects plug-in.
* **`drive-trio.json`** (`crucible.drive-trio`): three independent saturator instances on three parallel lanes, each at level
  0.33. It shows several instances of one type. Instance 1 keeps the flat ids (`fx_saturator_*`), instances 2 and 3 are
  `fx2_saturator_*` and `fx3_saturator_*`. Its `idMap` pins the third instance's drive to a readable id (`"fx3_saturator_drive":
  "drive_c_drive"`), and its `defaults` set the starting values.
* **`drum-kit.json`** (`crucible.drum-kit-test`): a voice made from a graph. Four layers on four notes, each with its own
  small graph of nodes (`osc.sub`, `env.adsr`, `amp.vca`, `mix.out`, `filter.svf`, `osc.noise`), with a choke group on the
  hats. A test product for the graph executor, but the pattern is how a drum instrument is built.

### 9.3 The other keys

* **`idMap`** pins a generated parameter id to another name: `{ "<generated id>": "<pinned id>" }`. Use it to give a
  readable name to a parameter of a second instance or a graph node. Ids of the first instances are never renamed.
* **`defaults`** sets starting values for parameters by id: `{ "fx_saturator_on": 1, "fx_saturator_drive": 8 }`. Values are natural
  units. A test product must keep its parameter ids: a pin that matches no generated id is a compile error.

### 9.4 Parameter names of instances

The first instance of a built-in effect keeps its id (`fx_saturator_drive`, `fx_delay_pro_mode`). Instance k of a type is
`fx{k}_...`. A graph node's parameters are `<instance>_<local>`, such as `kick_env_attack`. Read `docs/PARAMETERS.md` for
the ids of every built-in module.

### 9.5 Checking a product

`crucible_create_from_product` validates it and returns the errors, one per line, in the error buffer. The command line
tool reads a product too: `crucible-render --product FILE` plays one.

---

## 10. Sound design workflow

### 10.1 Patches: save and load

A patch is versioned JSON: meta, parameters, scenes, sequencer, arpeggiator, routes, clips, user waves, impulse responses and
extra assets. Patches migrate forward on load.

```c
char buffer[1 << 20];
int size = crucible_save_patch(engine, "Warm Lead", buffer, sizeof buffer);   /* [main] */
/* size > sizeof buffer - 1: call again with a larger buffer */

char error[512] = "";
crucible_load_patch(engine, buffer, error, sizeof error);                      /* [main] */
crucible_load_patch_file(engine, "lead.json", error, sizeof error);            /* from disk */
```

Unknown ids are ignored and missing ids take their defaults, so a patch survives parameter additions. Choices are stored as
numbers, but a hand-written patch can name the choice text (`"osc1_type": "Analog"`). A plug-in stores the patch JSON as its
state chunk and loads it back on restore. Load patches between notes when you can, because the sound changes mid-block.

### 10.2 Modulation routes

Routes connect a source to any modulatable parameter.

```c
/* [main] source is a ModSource value (ParameterMap.h), or 1000 + the MIDI CC number */
crucible_add_route(engine, 1000 + 74, "filter_cutoff", 0.6f);       /* CC 74 to cutoff, amount 0.6 */

CrucibleRoute route;
route.struct_size = sizeof route;
route.source = 1000 + 1;          /* MIDI CC 1 */
route.target_id = "filter_res";
route.amount = -0.3f;             /* -1 .. 1 */
route.polarity = 2;               /* 0 native, 1 unipolar, 2 bipolar */
route.curve = 0.0f;               /* -1 .. 1, 0 linear */
route.smoothing_ms = 40.0f;       /* 0 .. 2000 */
crucible_add_route_ex(engine, &route);

crucible_remove_route(engine, 1000 + 74, "filter_cutoff");
```

A route with the same source and target replaces the old one. The call returns 0 when refused: an unknown source or target, a cycle,
or too many routes. `crucible_add_route_ex` sets polarity, curve and smoothing. The sources include the LFOs, the envelopes,
the six or eight macros, velocity, key, the mod wheel, aftertouch, pitch bend, random, the strips and the scene morph (all
named in `ModSource` in `ParameterMap.h`). The classic voice's own matrix is also available as parameters
(`mod1_src`, `mod1_dst`, `mod1_pol`, through `mod16_*`), so a patch can use either.

### 10.3 Macros

A macro is a single 0 to 1 control (`macro1` to `macro6`, or up to `macro8`) that you route to several destinations. Set the
macro with `crucible_set_param_id(engine, "macro1", 0.5f)`, or from a MIDI controller with `crucible_map_cc`. Give each route
from the macro a different amount and polarity to build one knob that sweeps a whole sound.

### 10.4 Scenes

There are twelve scene slots. A scene stores a snapshot of the sound and can be recalled into a slot. `scene_a` and `scene_b`
choose the two scenes, and the scene morph (`SceneMorph` source) moves the sound between them.
`SceneBank::serialise()` returns each scene as a plain text record, locale independent, for saving alongside a patch.

### 10.5 Sequencer and arpeggiator

The sequencer and arpeggiator are parameters and data. Patterns are edited on the control thread and published atomically, so the
audio thread never waits for an edit. The step format and arpeggiator pattern format are plain text (`length,direction;index,on,tie,velocity,gate,octave,probability,ratchet,transpose[,group];...`)
and are also importable as JSON or MIDI clips. Choose a pattern with `seq_pattern`, set `seq_rate` and `seq_dir`, and sync with
`seq_sync` (internal or host). For the arpeggiator, `arp_mode`, `arp_rate`, `arp_sync` and `arp_vel_mode` are the main controls.

### 10.6 User wavetables

Load a WAV file into one of eight user slots, then select it from a wavetable oscillator's table choice (table index 14 plus the
slot number).

```c
CrucibleWavetableInfo info;
info.struct_size = sizeof info;
char error[256] = "";
crucible_load_wavetable_file(engine, 0, "my_table.wav", &info, error, sizeof error);   /* [main] */
/* or from memory: frames of frame_size mono samples, one after another */
crucible_load_wavetable(engine, 1, frames, 16, 2048, "Harsh", &info, error, sizeof error);
```

For pitch-dependent timbre (formants fixed in Hz) give a table several zones, each a full set of frames plus the highest
fundamental it serves; each mip level is built from the zone that contains its representative pitch:

```c
CrucibleWavetableZone zones[2] = {
    { sizeof zones[0], low_frames, 16, 2048, 262.0 },    /* up to about C4 */
    { sizeof zones[1], high_frames, 16, 2048, 4000.0 }   /* everything above */
};
crucible_load_wavetable_zones(engine, 1, zones, 2, "Voice", &info, error, sizeof error);   /* [main] */
```

`max_hz` must be finite, positive and strictly ascending (1 to 16 zones); the last zone also covers everything above its
`max_hz`. One gain normalises the whole table, so zones keep their relative level.

Each user table is 64 frames of 2048 samples. Fewer source frames are crossfaded to fill it, and more are decimated. `info`
reports the frame count and whether decimation happened.

### 10.7 Samples, zones and slices

```c
crucible_load_sample_file(engine, 0, "vox.wav");                          /* slot 0 (0-based) */
crucible_load_sample(engine, 1, interleaved, frames, 2, 48000.0, "pad");  /* from memory */
crucible_slice_sample_transients(engine, 0, 16, 0.5f);                    /* 16 slices, sensitivity 0.5 */
crucible_slice_sample_equally(engine, 0, 8);                              /* or 8 equal slices */
float markers[32];
int count = crucible_get_sample_slices(engine, 0, markers, 32);           /* read them back */
```

For zones (multisamples and drum kits), add samples with `crucible_add_zone_sample` and describe the mapping with
`crucible_set_zones`: each `CrucibleZone` gives a key range, a velocity range, a root key, a gain, a tuning, a loop mode and a
round-robin group. A whole folder can be imported with `crucible_import_zone_folder`, and an SFZ file with
`crucible_import_zone_sfz`. Set the oscillator to Zones mode (`osc*_smp_zone_mode`) to play them.

Cloud oscillators take a cloud description with `crucible_set_cloud` (the text form of `CloudModel::serialise()`).

### 10.8 Impulse responses

```c
char error[256] = "";
crucible_load_ir(engine, "fx.convolution", "hall.wav", error, sizeof error);   /* per instance */
crucible_clear_ir(engine, "fx.convolution");
```

`crucible_load_ir` takes the module id of the instance: `fx.convolution` (WAV or AIFF, mono, stereo or four-channel true stereo,
up to 30 seconds), `fx.convolution.2` for a second instance in a graph product. Its
impulse response is saved in patches.

### 10.9 Oscillator and modulation tips

* Start from a wavetable with a warp, then add unison and a filter route. Most sounds need only three decisions.
* Use Cross Mod Sync for lead tones, FM for bells, and Ring for metallic or clangorous tones.
* Use a macro for the sweep you will perform, and a route from an LFO for the motion you want without thinking about it.
* Put Transistor Ladder or OTA Cascade on the bass and the lead, Sallen-Key on acid and growl, and OTA SVF when resonance must
  never ring out.

---

## 11. Metering, views and GUIs

### 11.1 Views: read back without the audio thread

A view is `{instance, id}` with a typed payload of 32-bit numbers. Views are how a GUI reads the engine.

* **Subscribe.** `crucible_view_find(engine, "master", "meter")` gives the view, and `crucible_view_subscribe(engine, view, 30.0f)`
  starts publication at up to 30 times a second. A view nobody subscribes to costs nothing.
* **Read.** `crucible_view_read(engine, view, buffer, capacity, &sequence)` copies the newest snapshot (`sequence` changes whenever
  the payload may have changed). `crucible_view_size` gives the capacity to allocate.
* **Describe.** `crucible_view_count` and `crucible_view_info` list the views, with their kind (scalar, vector, points or mask),
  their instance and id, and whether the audio thread publishes them (live) or the reader computes them.
* **Parameter views.** `crucible_view_find(engine, "params", "filter_cutoff.mod")` gives the modulated value of a parameter, so a
  knob can move with the envelope.
* **Pushed or pulled.** Pushed views (the meter, the modulated values) are read from any thread. Pulled views (the spectrum of
  the master, sample peaks, the filter response) are computed by the reader, so read them from the main thread.

The GUI example in `examples/instrument_skeleton/main.c` reads the master meter, the modulated cutoff and the filter response
curve on a timer.

### 11.2 Meter and clip

`crucible_meter(engine)` returns the master peak in dB per channel and a clip flag, from any thread. `crucible_reset_clip` clears
the clip flag.

### 11.3 Loudness, scope and spectrum

* **Loudness.** `crucible_loudness(engine)` gives momentary and short-term LUFS, true peak, the maximum true peak and stereo
  correlation, from any thread.
* **Scope.** `crucible_scope(engine, left, right, count)` returns the most recent samples per channel (up to 4096), on the
  main thread.
* **Spectrum.** `crucible_spectrum(engine, fft_size, bins, capacity)` returns the magnitude spectrum in dB of the most recent
  `fft_size` samples (a power of two from 256 to 4096), on the main thread.
* **Analyzers on or off.** `crucible_set_analyzer_enabled` turns the analysers off when you do not need them.

### 11.4 Sidechain input

`crucible_process_aux` gives the engine a second, sidechain input for the vocoder, the key of a compressor or gate, and any
module that listens to an external signal. Pass `aux_left` and `aux_right` (NULL for no input, and `aux_right` NULL for mono).

### 11.5 Measuring a module

Each effect module reports its own level and gain reduction where that is part of its job. The multiband compressor and the
maximiser show gain reduction and loudness; the Meter module (`util.meter`) measures and passes audio through. The engine has
one meter set on the master bus.

---

## 12. Command-line tools

All tools print their version with `--version`.

### 12.1 `crucible-render`

Renders offline to a WAV file, plays a patch or a product, and lists the modules and parameters.

```sh
crucible-render --note 48 --length 2 -o out.wav                 # one note through the default product
crucible-render --patch lead.json --midi take.mid -o take.wav    # a patch and a MIDI file
crucible-render --patch lead.json --score score.txt -o score.wav # a score file with tempo
crucible-render --patch lead.json --set filter_cutoff=1800 --note 60 --length 3 -o cut.wav
crucible-render --product products/drive-trio.json --note 60 -o trio.wav
crucible-render --patch lead.json --save-patch saved.json        # write a patch
crucible-render --list-modules                                    # every module with its status
crucible-render --params --markdown > docs/PARAMETERS.md          # the parameter reference
```

The options are `--patch`, `--product`, `--midi`, `--score`, `-o` (or `--out`), `--save-patch`, `--note`, `--length`, `--velocity`,
`--rate`, `--block`, `--tail`, `--bpm`, `--seed`, `--voices`, `--bits` (16 or 32) and `--set id=value` (repeatable). `--params`
lists parameters, and `--markdown` with it writes the reference.

### 12.2 `crucible-measure`

Renders a measurement job file so an external analysis suite can score the engine without a plug-in host.

```sh
crucible-measure --job job.json --out-dir renders/ [--voices N] [--quality 0|1|2] [--seed N]
```

The job addresses parameters by display name or id, with normalised numbers and display-text strings. It writes one 32-bit
float stereo WAV per render, and a `manifest.json` with timing (wall time of `process()`, CPU time and the worst block).

### 12.3 `crucible-module-check`

Runs the real-time contract on a module: metadata, an allocation counter on the audio calls, an extremes sweep (every parameter at
its minimum and maximum, with loud noise and with silence), NaN, infinity and denormal inputs, flush-to-zero independence,
determinism and CPU against a budget by class.

```sh
crucible-module-check --builtin                   # every built-in module
crucible-module-check libacme_tremolo.so          # your shared library
crucible-module-check --module fx.saturator --seconds 4 --block 256 --rate 48000
crucible-module-check --list
```

Options are `--builtin`, `--module`, `--seed`, `--seconds`, `--block`, `--rate`, `--no-cpu-limit`, `--list`, `--quiet` and `--version`.
Exit status 0 means every check passed, 1 means a check failed, and 2 means a library could not be loaded or the command line was
wrong.

### 12.4 `render_wav` and the examples

`examples/render_wav.cpp` is a complete C++ program: a phrase in, a 16-bit WAV out, through `Engine`. `examples/c_api/render_c.c`
does the same through the C API.

---

## 13. The Python package

`python/crucible_engine` renders the engine offline from Python over the C API, with ctypes and NumPy. There is no compiled
extension. The package is not yet on PyPI (it is planned for the public launch); until then, build it locally.

```sh
cmake -S . -B build-shared -DCMAKE_BUILD_TYPE=Release -DCRUCIBLE_ENGINE_SHARED=ON
cmake --build build-shared --target crucible_shared -j8
export PYTHONPATH=python
python3 python/examples/render_patch.py out.wav
python3 -m unittest discover -s python/tests
```

The library is found through the `CRUCIBLE_LIBRARY` environment variable (a path), the package directory, or a `build*`
directory of a source checkout. `pyproject.toml` describes the package (setuptools, `package-dir` `python`, NumPy as the only
dependency); it expects the shared library copied next to the package before a wheel is built.

```python
from crucible_engine import Engine

with Engine(max_voices=8, quality=1) as engine:          # the default product
    engine.prepare(48000.0)
    engine.set_param("filter_cutoff", 1800.0)
    engine.set_param_text("filter_type", "Transistor Ladder")
    engine.note_on(60, 0.8)
    audio = engine.render(2.0)                            # float32, shape (n, 2)
    engine.note_off(60)
    patch = engine.save_patch("lead")
    print(engine.latency_samples(), engine.tail_seconds())

product = Engine.from_product(open("products/drive-trio.json").read())   # a product as JSON text or a dict
```

The methods are `prepare`, `param_count`, `param_index`, `set_param`, `set_param_text`, `get_param`, `param_text`, `note_on`,
`note_off`, `all_notes_off`, `pitch_bend`, `mod_wheel`, `sustain`, `midi`, `render` (seconds or samples), `describe`,
`save_patch`, `load_patch` (text, bytes or a dict), `reset`, `latency_samples` and `tail_seconds`. `Engine.from_product` builds
from a product, and errors raise `CrucibleError` with the engine's reason.

---

## 14. Third-party modules

A module that is not in this repository is a shared library that implements the C ABI in `include/crucible/crucible_module.h`.
It links against nothing from the engine. The host loads it at run time, and the host names every path and decides whether
libraries may be loaded at all. The engine never scans directories.

### 14.1 What a module is

A module is either an **effect** (a stereo bus in and out, processed in place) or a **node** of the voice graph: a generator, a
filter or a modulator, with the audio and modulation ports it declares. Ids are dotted, lower case and vendor first (`acme.tremolo`).
The ids `crucible`, `fx`, `osc`, `voice`, `filter`, `env`, `lfo`, `amp`, `mix` and `classic` belong to the engine.

Parameters are declared in the descriptor (float, int, choice or bool, with range, default, skew, unit, a short name and flags for
modulation, smoothing and automation). `process` receives them in that order, in natural units, already smoothed. The host gives
each one a global id, so a tremolo's `rate` becomes `acme_tremolo_rate`. The host adds a bypass switch to every effect.

### 14.2 The entry points

`create`, `destroy`, `prepare` and `get_view` run on a control thread and may allocate. `process`, `process_quad`, `note_on`,
`note_off`, `reset`, `latency` and `holds_voice` run on the audio thread and must not allocate, lock or wait. The output must be
finite and bounded at every parameter value, must not depend on flush-to-zero, and must be a function of the seed, the parameters
and the input only.

### 14.3 Writing one in C++

The C++ layer generates the descriptor from a class. From `examples/third_party_module/cpp_module.cpp`, condensed:

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

Optional methods are `noteOn`, `noteOff`, `holdsVoice`, `latency`, `processQuad` (four voices in one call) and `view`.

### 14.4 Writing one in C

`examples/third_party_module/c_module.c` fills static `crucible_param_spec` and `crucible_module_descriptor` structs and exports
`crucible_get_modules`, which returns a `crucible_module_library`. Every struct starts with `size` and `abi_version`, and structs
only grow at the end. A host accepts any module whose ABI version is in the supported range and refuses newer ones with
`CRUCIBLE_MODULE_ERR_ABI_VERSION`.

### 14.5 Loading it in a host

```cpp
crucible::sdk::ModuleSet modules;
modules.allowLibraries(true);                           /* off by default */
if (modules.loadLibrary(path) != CRUCIBLE_MODULE_OK)
    return fail(modules.error());
auto nodes = crucible::graph::NodeRegistry::builtIn();
modules.addTo(nodes);
auto product = crucible::graph::compile(definition, error, nodes);
crucible::EngineConfig config;
config.compiled = product;                              /* the product names "acme.tremolo" as an fx instance type */
```

In C the same is `crucible_module_registry_create`, `crucible_module_registry_allow_libraries`, `crucible_load_module_library` (or
`crucible_register_module` for a module linked into the host), and then `crucible_create_from_product(config, json, registry, error, size)`.
Modules cannot be added to a running engine; a new module set needs a new engine.

### 14.6 Checking it

`crucible-module-check libacme_tremolo.so` runs the real-time harness on it (section 12.3). Run it in your own tests too: the same
harness is a library, `crucible/sdk/Check.h`. Its allocation counter sees C++ `operator new` only, so review C modules that call
`malloc` in `process` by hand.

---

## 15. Module status

Every module is registered in `src/modules/Modules.cpp`. A module retired after v1 stays registered so that existing patches render
bit-exactly, marked legacy. `describeJSON` and `crucible_module_meta` carry a `legacy` flag and a `superseded_by` id for each module; none is legacy at present (the first-generation effects were removed before v1).

| Module | Id | Status | Use this instead |
| --- | --- | --- | --- |
| Saturator | `fx.saturator` | current | |
| Multiband Drive | `fx.multiband_drive` | current | |
| Clipper | `fx.clipper` | current | |
| Crusher | `fx.crusher` | current | |
| Ensemble | `fx.ensemble` | current | |
| Flanger 2 | `fx.flanger2` | current | |
| Phaser 2 | `fx.phaser2` | current | |
| Tremolo | `fx.tremolo` | current | |
| Ring / Shift | `fx.ring` | current | |
| Filter | `fx.filter` | current | |
| Multiband | `fx.multiband` | current | |
| Trance Gate | `fx.trance` | current | |
| Vocoder | `fx.vocoder` | current | |
| Clean Compressor | `fx.comp_clean` | current | |
| VCA Bus Compressor | `fx.comp_vca` | current | |
| Gate Expander Ducker | `fx.gate2` | current | |
| Maximizer | `fx.maximizer` | current | |
| Limiter | `fx.limiter` | current | |
| Width | `fx.width` | current | |
| Pro EQ | `fx.eq_pro` | current | |
| Algorithmic Reverb | `fx.reverb_algo` | current | |
| Convolution | `fx.convolution` | current | |
| Delay Pro | `fx.delay_pro` | current | |
| Transient Shaper | `fx.transient` | current | |
| De-esser | `fx.deesser` | current | |
| Exciter | `fx.exciter` | current | |
| Cabinet | `fx.cabinet` | current | |
| Meter | `util.meter` | current | |

Voice and control modules (the wavetable voice, the oscillator slots, the oscillator types, the analog voice and its filter
controls, the modulator bank, the arpeggiator, the step sequencer, the scenes, key lock, the performance controls and the vector
field) are current. The module list and every parameter are in `docs/PARAMETERS.md`, and `crucible-render --list-modules` prints
the same registry with status.

---

*Where the code and this guide differ, the code is right. The parameter reference and `crucible.h` are generated or maintained
alongside it.*
