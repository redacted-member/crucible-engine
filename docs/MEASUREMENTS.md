# Measurements: the baseline of the current filters and oscillators

Everything in this file is a number produced by the measurement harness (`tests/AnalogMeasure.h`), not a listening
impression. It is the before-picture for the analog-filter, VA-oscillator, saturation and envelope packages (P7 to P12 of
`docs/plans/ENGINE-COMMUNITY-PLAN.md`): each of those packages has to beat these numbers or meet the plan's targets.

Measured on an Apple M-class Mac (Release build, 2026-10-11) with:

```sh
cmake -S engine -B engine/build -DCMAKE_BUILD_TYPE=Release && cmake --build engine/build -j8
engine/build/crucible_engine_measure [filters|oscillators|envelope|drift|all]   # prints the tables below (about 25 s)
engine/build/crucible_engine_tests AnalogMeasure        # self-tests of the primitives
engine/build/crucible_engine_tests MeasuredBaseline     # guards for the facts below
engine/build/crucible_engine_tests TuningOracle         # Tuning against the vendored tuning-library
engine/build/crucible_engine_bench plan                 # plan CPU rows
```

## 1. The harness and what it recovers

`tests/AnalogMeasure.h` (header only, double precision, tests and offline tools only): spectra (Blackman-Harris and a
7-term window), frequency estimation (spectral peak plus a golden-section search of the windowed DTFT), harmonic profile
with THD, THD+N and even / odd shares, aliasing and worst spur outside the harmonics, band RMS, DC (plain and windowed),
true peak (4x windowed sinc), stepped-sine gain and response, exponential-sweep impulse response, slope, -3 dB point,
resonance peak, self-oscillation (pitch, level, steadiness), Ornstein-Uhlenbeck statistics (std, autocorrelation time,
PSD slope, pooled over runs), pitch trace, exponential envelope fits, RMS envelopes. `tests/AnalogMeasureTests.cpp`
gives every primitive a synthetic signal with a known answer:

| quantity | plan tolerance | recovered (worst case in the tests) |
| --- | --- | --- |
| THD and harmonic levels | within 0.1 dB | 0.00001 dB (harmonics at -60 to -100 dB, 44.1 / 48 / 96 kHz, off-bin frequencies) |
| filter slope | within 0.2 dB/oct | 0.0000 dB/oct against the filter's exact transfer function (24.14 and 24.10 dB/oct at 48 / 96 kHz; the analog 24 is bent by the bilinear transform) |
| filter magnitude response | - | stepped sines within 0.01 dB, sweep impulse response within 0.00001 dB of a Butterworth |
| pitch | within 0.1 cent | 0.0002 cent (40 Hz to 8 kHz, three rates, noise at -60 dB) |
| self-oscillation tracking | - | 0.1 cent bound, level steady to 0.01 dB (rotation oscillator, 40 Hz to 8 kHz) |
| Ornstein-Uhlenbeck std | within 5 % | 0.5 % (5.969 against 6); tau 1.982 against 2; PSD slope -1.96 |
| clean-sine floor | below -130 dB | -161.3 dB (12 frequency / rate combinations) |
| inharmonic spur at -80 dB | - | read as -80 dB within 0.1 dB |
| true peak (fs/4 sine at 45 degrees) | - | 0 dBTP within 0.1 dB while the samples read -3.01 dB |
| exponential fits | - | tau within 0.01 % (noise-free), 1 % with noise; audio envelope tau within 1 % |

The reference-bench Python side has the same discipline: `python3 tools/reference-bench/analysis/refbench.py selftest`
runs the seven new analysers (section 5) on synthetic renders (20 of 20 checks).

## 2. Headline numbers (before P7 to P12)

* **Wavetable oscillators are accurate and clean at 44.1 / 48 kHz.** Tuning error at most 0.01 cent (0.026 cent at C2 at 96 kHz, the size to expect from a float phase
  accumulator: its 6e-8 resolution against an increment of 6.8e-4); the saw follows 1/n to 0.00 dB up to 10 kHz; aliasing below 20 kHz relative to the
  fundamental is -108.5 to -141 dB for sine, saw and square at C2 to C8 (the plan wants -100 dB for the VA oscillator, so the wavetable
  engine already meets that on the saw). The sine table is not a perfect sine: THD+N -92 to -133 dB depending on rate and note
  (C2 at 96 kHz is the worst, -92.4 dB); at note 60 and 48 kHz H2 is -123.9 dB.
* **Warps alias a lot.** At C6 and 48 kHz the alias power under 20 kHz relative to the fundamental is -47.7 / -39.7 / -36.0 dB for PWM
  at amount 0.2 / 0.5 / 0.8, -31.8 to -41 dB for hard sync, -62.7 to -21.8 dB for soft sync, and between -28 and +1 dB for
  fold (they run oversampled, 2x/4x; the plan's `osc.analog` has to reach -90 dB for PWM / sync / FM).
* **The state-variable filters are textbook.** LP12 slope -12.0 dB/oct, LP24 -24.2 (250 Hz cutoff; at 1 kHz the bilinear warping
  bends the measured slope to -15 and -30 dB/oct between 4 and 16 kHz, a property of the topology at 48 kHz), no bass loss
  with resonance (+0.04 dB at 50 Hz), no level dependence (peak gain identical from -40 to +6 dBFS), never self-oscillate (the
  resonance-1 peak is +35.5 dB for LP12 and +32.3 dB for LP24, rings and decays), THD at drive 0 below -143 dB.
* **The ladder is a marginal oscillator, not a self-oscillator.** Resonance 1 sets the feedback to exactly 4.0, the linear
  threshold. It rings at the cutoff to 0.0 cent from 1 kHz up (steady, -53 dBFS RMS) but does not sustain at 40, 100 and 250 Hz
  within 3 s. Bass loss with resonance is only -2.7 dB (0.25) to -5.0 dB (1.0), where an ideal ladder loses
  `-20 log10(1+k)` = -6 to -14 dB; the input stage compresses hard (peak gain at resonance 0.9: +19.0 dB at -40 dBFS, -4.0 dB at 0 dBFS).
  The plan wants onset k in 4.0 to 4.3 (a margin) and bass loss within 0.5 dB of the law.
* **The diode ladder** (3.6 x resonance) cannot reach self-oscillation (decays), slope 18.6 dB/oct, peak +16.8 dB at resonance 1.
* **Drift is a fast random walk, not Ornstein-Uhlenbeck.** At `voice_drift` = 1: std 7.8 cent, autocorrelation time 73 ms (the
  plan's drift is tau 0.5 to 5 s), lag-1 (20 ms) correlation 0.82, PSD slope -0.89 between 0.5 and 10 Hz (the corner is
  at 2.2 Hz). 24 notes of 10 s.
* **Envelopes are digital curves, not RC** (the Digital mode; P11 adds the Analog RC mode, section 13). Amp attack of 100 ms peaks at 96 ms and does not fit a single exponential (residual
  2.66 % of the range, tau 0.115 s with a target of 2.09); decay residual 0.77 %, release 0.21 %. The plan wants a residual
  below 0.5 % and the peak time within 2 %.
* **Exact silence.** An idle engine and a held voice with oscillator level 0 output exact zeros.
* **CPU:** `crucible_engine_bench plan`: the reference row (16 voices x 8 unison) costs 4.4 % of one core, the plan's 4 % target
  with its +20 % margin (4.8 %) holds; the other rows wait for P4, P7, P9, P11.

### Bug found and fixed by the harness

**Wavetable aliasing at high sample rates.** Saw at C2: -50.0 dB at 96 kHz and -64.3 dB at 192 kHz against -115 to -118 dB at
44.1 and 48 kHz. The level picker kept partials up to (rate - 20 kHz), which is 76 kHz at 96 kHz; the highest partials of a low note
sit close to the table's read resolution, and the 6-point interpolation's images folded into the audible band as
a forest of lines at -70 dB and above. `Voice::bandLimit` now also caps the tables at 30 kHz (never less than 0.2 cycles per
sample) above 48 kHz: saw C2 is now -120.4 dB at 96 kHz and -119.3 dB at 192 kHz, 44.1 and 48 kHz are untouched (the cap is
above the old limit there), hashes and goldens are unchanged. Regression test: `MeasuredBaseline`, "wavetable saw keeps
its aliasing below -100 dB ...". Recorded in `SOUND_CHANGES.md`.

### Findings that are not bugs (kept for the packages that follow)

* The "Analog Square" table is not an ideal square: odd harmonics deviate from 1/n by 26 dB at C2 and 4 dB at C4 (up to 10 kHz), 0.02 dB from C6 on.
  It is a shaped table, so the shape row of the plan applies to `osc.analog`, not to this table.
* Pitch accuracy at low notes and high sample rates (0.026 cent at C2 / 96 kHz) matches the float phase accumulator of the voice
  (`Voice.h`, `std::array<float, kMaxUnison> phase`); the plan's VA oscillator uses a double phase.
* DC offset of every oscillator measurement is below -94 dBFS (windowed estimate; the plain mean of a partial period is
  biased by the tone itself, which is why `dcOffsetWindowed` exists).

## 3. Filters (48 kHz, FilterChannel, stepped sines at -40 dBFS unless noted)

### 3.1 Filters at 1 kHz, resonance 0 (stepped sines at -40 dBFS, 48 kHz)

Slope: least-squares dB per octave between 2 and 4 octaves above (low-pass) or below (high-pass) the cutoff; for the low-passes also at a 250 Hz cutoff, where the bilinear transform's frequency warping does not yet bend the response.

| model | gain 100 Hz (dB) | gain 10 kHz (dB) | -3 dB point (Hz) | slope, 1 kHz cutoff (dB/oct) | slope, 250 Hz cutoff (dB/oct) |
|---|---|---|---|---|---|
| Lowpass 12 | -0.09 | -42.80 | 655 | -15.1 | -12.0 |
| Highpass 12 | -40.11 | -0.06 | 1613 | 11.8 | - |
| Lowpass 24 | -0.09 | -85.54 | 579 | -30.4 | -24.2 |
| Highpass 24 | -80.14 | -0.07 | 1810 | 23.9 | - |
| Ladder LP | -0.17 | -85.60 | 449 | -30.2 | -24.0 |
| Ladder HP | -80.23 | -0.13 | 2281 | 23.7 | - |
| Diode LP | -0.28 | -73.21 | 457 | -23.3 | -18.6 |
| Dirty LP | -0.09 | -42.80 | 655 | -15.1 | -12.0 |

### 3.2 Resonance peak and bass loss, cutoff 1 kHz

Peak: highest point of the response between 100 Hz and 8 kHz (stepped sines at -40 dBFS, 24 points per octave). Bass loss: gain at 50 Hz against the same filter at resonance 0.

| model | resonance | peak (dB) | peak at (Hz) | bass loss at 50 Hz (dB) |
|---|---|---|---|---|
| Lowpass 12 | 0.25 | -0.01 | 100 | 0.02 |
| Lowpass 12 | 0.50 | 1.14 | 693 | 0.03 |
| Lowpass 12 | 0.75 | 5.83 | 974 | 0.04 |
| Lowpass 12 | 0.90 | 12.60 | 997 | 0.04 |
| Lowpass 12 | 1.00 | 35.52 | 1004 | 0.04 |
| Lowpass 24 | 0.25 | -0.02 | 100 | 0.02 |
| Lowpass 24 | 0.50 | 0.48 | 502 | 0.03 |
| Lowpass 24 | 0.75 | 3.50 | 946 | 0.04 |
| Lowpass 24 | 0.90 | 9.56 | 995 | 0.04 |
| Lowpass 24 | 1.00 | 32.27 | 1004 | 0.04 |
| Ladder LP | 0.25 | -0.49 | 615 | -2.73 |
| Ladder LP | 0.50 | 3.83 | 818 | -3.90 |
| Ladder LP | 0.75 | 10.89 | 926 | -4.55 |
| Ladder LP | 0.90 | 18.84 | 973 | -4.83 |
| Ladder LP | 1.00 | 24.84 | 996 | -4.97 |
| Diode LP | 0.25 | -0.73 | 587 | -2.51 |
| Diode LP | 0.50 | 2.94 | 789 | -3.65 |
| Diode LP | 0.75 | 8.34 | 895 | -4.31 |
| Diode LP | 0.90 | 12.98 | 941 | -4.59 |
| Diode LP | 1.00 | 16.84 | 962 | -4.74 |
| Dirty LP | 0.25 | -0.01 | 100 | 0.02 |
| Dirty LP | 0.50 | 1.14 | 692 | 0.03 |
| Dirty LP | 0.75 | 5.81 | 926 | 0.04 |
| Dirty LP | 0.90 | 12.39 | 984 | 0.04 |
| Dirty LP | 1.00 | 22.91 | 998 | 0.04 |

### 3.3 Self-oscillation (resonance 1, cutoff 1 kHz, impulse start, 2 s)

| model | oscillates | frequency (Hz) | error against cutoff (cent) | level (dBFS rms) | level variation (dB) |
|---|---|---|---|---|---|
| Lowpass 12 | no | - | - | decays to nothing | - |
| Highpass 12 | no | - | - | decays to nothing | - |
| Bandpass 12 | no | - | - | decays to nothing | - |
| Notch | no | - | - | decays to nothing | - |
| Peak | no | - | - | decays to nothing | - |
| Lowpass 24 | no | - | - | decays to nothing | - |
| Highpass 24 | no | - | - | decays to nothing | - |
| Bandpass 24 | no | - | - | decays to nothing | - |
| Ladder LP | yes | 1000.0 | -0.0 | -53.0 | 0.08 |
| Ladder HP | yes | 1000.0 | -0.0 | -53.0 | 0.08 |
| Ladder BP | yes | 1000.0 | -0.0 | -40.0 | 0.08 |
| Diode LP | no | - | - | decays to nothing | - |
| Comb + | no | - | - | decays to nothing | - |
| Comb - | no | - | - | decays to nothing | - |
| Formant | no | - | - | decays to nothing | - |
| Phaser | no | - | - | decays to nothing | - |
| Dirty LP | no | - | - | decays to nothing | - |

### 3.4 Self-oscillation pitch tracking (error in cents against the cutoff setting)

| model | 40 Hz | 100 Hz | 250 Hz | 1000 Hz | 3000 Hz | 8000 Hz | amplitude spread (dB) |
|---|---|---|---|---|---|---|---|
| Ladder LP | - | - | - | -0.0 | -0.0 | -0.0 | 5.3 |
| Ladder HP | - | - | - | -0.0 | -0.0 | -0.0 | 5.3 |
| Ladder BP | - | 0.0 | -0.0 | -0.0 | -0.0 | -0.0 | 25.0 |

### 3.5 Level dependence (gain at the resonance peak, resonance 0.9, cutoff 1 kHz; input level in dBFS)

| model | -40 | -12 | 0 | +6 |
|---|---|---|---|---|
| Lowpass 12 | 12.60 | 12.60 | 12.60 | 12.60 |
| Lowpass 24 | 9.56 | 9.56 | 9.56 | 9.56 |
| Ladder LP | 18.98 | 5.78 | -4.03 | -9.67 |
| Diode LP | 13.00 | -0.55 | -11.08 | -16.90 |
| Dirty LP | 12.40 | 3.48 | -3.46 | -7.02 |

### 3.6 Filter drive: THD of a 1 kHz sine at 0.5 amplitude (cutoff 8 kHz, resonance 0)

| model | drive 0 | drive 0.5 | drive 1 |
|---|---|---|---|
| Lowpass 12 | -146.4 dB | -14.4 dB | -11.2 dB |
| Lowpass 24 | -143.5 dB | -14.5 dB | -11.4 dB |
| Ladder LP | -48.5 dB | -15.2 dB | -12.3 dB |
| Diode LP | -26.0 dB | -14.1 dB | -11.9 dB |
| Dirty LP | -87.6 dB | -14.4 dB | -11.2 dB |

## 4. Oscillators, envelope, drift (SynthEngine, one voice, filter off)

### 4.1 Wavetable oscillator: tuning, aliasing and shape (one voice, filter off)

Aliasing: power outside the harmonics below 20 kHz against the fundamental. Spur: strongest single non-harmonic line. Shape: worst deviation of the harmonic levels from the ideal law (1/n saw, odd 1/n square) up to 10 kHz. THD+N for the sine.

| wave | rate | note | f0 (Hz) | tuning (cent) | alias (dB) | worst spur (dB) | shape (dB) | THD+N (dB) | DC (dBFS) |
|---|---|---|---|---|---|---|---|---|---|
| sine | 44100 | 36 | 65.407 | 0.004 | -137.6 | -143.9 | - | -122.5 | -133.7 |
| sine | 44100 | 60 | 261.626 | 0.002 | -141.1 | -156.6 | - | -113.1 | -118.4 |
| sine | 44100 | 84 | 1046.502 | 0.000 | -137.5 | -143.8 | - | -133.0 | -141.1 |
| sine | 44100 | 96 | 2093.005 | 0.000 | -140.5 | -152.9 | - | -130.0 | -136.0 |
| sine | 44100 | 108 | 4186.009 | -0.000 | -137.1 | -144.2 | - | -132.4 | -140.5 |
| sine | 48000 | 36 | 65.406 | -0.010 | -137.8 | -146.3 | - | -109.2 | -115.4 |
| sine | 48000 | 60 | 261.626 | 0.003 | -129.5 | -133.7 | - | -120.0 | -127.3 |
| sine | 48000 | 84 | 1046.502 | -0.000 | -124.0 | -129.4 | - | -122.5 | -160.8 |
| sine | 48000 | 96 | 2093.004 | -0.000 | -118.3 | -123.0 | - | -116.9 | -162.9 |
| sine | 48000 | 108 | 4186.009 | -0.000 | -114.5 | -117.0 | - | -112.6 | -157.0 |
| sine | 96000 | 36 | 65.407 | 0.026 | -139.7 | -145.6 | - | -92.4 | -97.4 |
| sine | 96000 | 60 | 261.625 | -0.006 | -136.0 | -141.2 | - | -104.4 | -109.5 |
| sine | 96000 | 84 | 1046.504 | 0.002 | -127.0 | -132.1 | - | -115.8 | -121.3 |
| sine | 96000 | 96 | 2093.004 | -0.000 | -122.9 | -127.8 | - | -122.3 | -161.0 |
| sine | 96000 | 108 | 4186.009 | -0.000 | -117.0 | -121.7 | - | -116.9 | -162.7 |
| saw | 44100 | 36 | 65.407 | 0.004 | -115.0 | -139.0 | 0.00 | - | -128.9 |
| saw | 44100 | 60 | 261.626 | 0.002 | -133.0 | -139.5 | 0.00 | - | -123.2 |
| saw | 44100 | 84 | 1046.502 | 0.000 | -126.7 | -141.5 | 0.00 | - | -142.3 |
| saw | 44100 | 96 | 2093.005 | 0.000 | -133.5 | -135.9 | 0.00 | - | -141.4 |
| saw | 44100 | 108 | 4186.009 | -0.000 | -132.1 | -135.6 | 0.00 | - | -143.9 |
| saw | 48000 | 36 | 65.406 | -0.010 | -117.5 | -137.4 | 0.00 | - | -115.9 |
| saw | 48000 | 60 | 261.626 | 0.003 | -110.8 | -133.5 | 0.00 | - | -127.4 |
| saw | 48000 | 84 | 1046.502 | -0.000 | -111.7 | -129.3 | 0.00 | - | -162.1 |
| saw | 48000 | 96 | 2093.004 | -0.000 | -108.8 | -123.3 | 0.00 | - | -164.5 |
| saw | 48000 | 108 | 4186.009 | -0.000 | -108.5 | -117.2 | 0.00 | - | -153.5 |
| saw | 96000 | 36 | 65.407 | 0.026 | -120.4 | -140.7 | 0.00 | - | -101.6 |
| saw | 96000 | 60 | 261.625 | -0.006 | -119.2 | -141.0 | 0.00 | - | -113.7 |
| saw | 96000 | 84 | 1046.504 | 0.002 | -114.2 | -132.2 | 0.00 | - | -125.0 |
| saw | 96000 | 96 | 2093.004 | -0.000 | -113.5 | -127.7 | 0.00 | - | -184.1 |
| saw | 96000 | 108 | 4186.009 | -0.000 | -111.0 | -121.8 | 0.00 | - | -190.4 |
| square | 44100 | 36 | 65.407 | 0.004 | -126.8 | -144.1 | 26.10 | - | -126.8 |
| square | 44100 | 60 | 261.626 | 0.002 | -135.0 | -151.7 | 3.95 | - | -114.8 |
| square | 44100 | 84 | 1046.502 | 0.000 | -129.7 | -142.8 | 0.02 | - | -138.4 |
| square | 44100 | 96 | 2093.005 | 0.000 | -132.5 | -134.3 | 0.00 | - | -132.8 |
| square | 44100 | 108 | 4186.009 | -0.000 | -133.1 | -139.7 | 0.00 | - | -137.2 |
| square | 48000 | 36 | 65.406 | -0.010 | -127.6 | -147.1 | 26.10 | - | -112.4 |
| square | 48000 | 60 | 261.626 | 0.003 | -116.9 | -133.8 | 3.95 | - | -123.9 |
| square | 48000 | 84 | 1046.502 | -0.000 | -114.5 | -129.5 | 0.02 | - | -175.3 |
| square | 48000 | 96 | 2093.004 | -0.000 | -111.3 | -123.2 | 0.00 | - | -159.6 |
| square | 48000 | 108 | 4186.009 | -0.000 | -111.5 | -117.4 | 0.00 | - | -151.0 |
| square | 96000 | 36 | 65.407 | 0.026 | -128.8 | -146.8 | 26.11 | - | -94.2 |
| square | 96000 | 60 | 261.625 | -0.006 | -124.0 | -141.1 | 3.95 | - | -106.3 |
| square | 96000 | 84 | 1046.504 | 0.002 | -117.3 | -132.2 | 0.02 | - | -118.0 |
| square | 96000 | 96 | 2093.004 | -0.000 | -116.0 | -127.7 | 0.00 | - | -163.2 |
| square | 96000 | 108 | 4186.009 | -0.000 | -114.0 | -121.8 | 0.00 | - | -182.1 |

### 4.2 Sine harmonic profile (note 60, 48 kHz): level of harmonics 2..6 against the fundamental

| THD (dB) | H2 | H3 | H4 | H5 | H6 |
|---|---|---|---|---|---|
| -123.4 | -123.9 | -133.1 | -143.6 | -150.0 | -149.9 |

### 4.3 Warp aliasing (saw table, note 84, 48 kHz): alias power below 20 kHz against the fundamental

| warp | amount 0.2 | amount 0.5 | amount 0.8 |
|---|---|---|---|
| PWM | -47.7 dB | -39.7 dB | -36.0 dB |
| Hard sync | -31.8 dB | -40.3 dB | -41.0 dB |
| Soft sync | -62.7 dB | -31.0 dB | -21.8 dB |
| Mirror | -32.3 dB | -28.6 dB | -17.8 dB |
| Asym | -44.3 dB | -36.8 dB | -33.3 dB |
| Fold | -28.3 dB | -6.3 dB | 1.0 dB |

### 4.4 Noise floor and idle output

| case | RMS (dBFS) | DC (dBFS) |
|---|---|---|
| voice held, oscillator level 0 | exact zero | exact zero |
| no notes | exact zero | exact zero |

### 4.5 Amplitude envelope (sine A4, attack 100 ms, decay 300 ms, sustain 50 %, release 400 ms; RMS envelope, 9-period window)

| segment | time to peak / constant | single-exponential fit residual (rms, % of range) | note |
|---|---|---|---|
| attack | peak at 0.096 s (10-90 %: 0.062 s) | 2.66 | RC fit tau 0.115 s, target level 2.09 |
| decay | tau 0.049 s | 0.77 | settles at 0.516 of peak |
| release | tau 0.063 s | 0.21 | falls to -0.0003 |

### 4.6 Voice drift (voice_drift = 1, sine A4, pitch measured every 20 ms from 42.7 ms windows)

| quantity | value |
|---|---|
| runs | 24 notes x 10 s |
| mean (cent) | 0.158 |
| standard deviation (cent) | 7.831 |
| autocorrelation time (s) | 0.073 |
| lag-1 autocorrelation (20 ms) | 0.819 |
| PSD slope 0.5-10 Hz (one 180 s run) | -0.89 |

## 5. Tuning against the tuning-library (MIT, Surge Synth Team)

Surge's standalone tuning-library (MIT, verified from its `LICENSE.md`; unmodified headers in `third_party/tuning-library/`,
tests only) is the oracle. `tests/TuningOracleTests.cpp` compares the frequency of all 128 keys and the mapped flag
for 52 scales and mappings (12 hand-written: 12-TET, 19-EDO, Pythagorean, quarter-comma meantone, Bohlen-Pierce with a
tritave, 7-note just scale on 7 and 12 keys with unmapped keys, a 3/2-period scale, a restricted key range, a formal-octave degree;
and 40 generated). Result: **36 agree to 0.01 cent on every key (the pass rule is 0.01 cent), 7 are rejected by the library (reference key unmapped),
9 differ by convention** (below). None of the differences is a fault of `Tuning` against the Scala file definitions.

| difference | what each side does | verdict |
| --- | --- | --- |
| first / last key of the .kbm | the library ignores them and tunes every key; `Tuning` leaves keys outside the range unmapped (they play 12-TET) | convention; ours follows the Scala file definition |
| default anchor without a .kbm | the library anchors middle C (note 60) at 261.6256 Hz; `Tuning` anchors note 69 = 440 Hz, Scala's default mapping. Identical for 12 notes per octave; 19-EDO differs by a constant 331.6 cent | convention; explicit .kbm files agree |
| reference key one or more map repeats away from the middle note, map size different from the scale size | the library places the reference key in the first repeat, so the whole scale is a period off (701.955 cent for a 3/2 period) and two keys one repeat apart share a pitch; `Tuning` keeps one pitch per key and the reference key exactly on its frequency | the library is inconsistent here; not adopted |
| formal-octave degree different from the scale size, with a key map | the library takes the scale's last degree as the period; `Tuning` repeats the map at the formal-octave degree | convention; ours follows the .kbm definition |

The follow-up the plan names ("adopt tuning-library") is **not needed**: the only fully in-spec disagreements are cases where
the library is the odd one. If the maintainers prefer Surge's conventions, the switch is one include and four adapter functions
(`Tuning::frequency`, `isMapped`, `loadScl`, `loadKbm`).

## 6. Reference bench: new analysis types

`tools/reference-bench/analysis/refbench_analysis/measure_analog.py` adds `selfosc_track`, `res_bassloss`,
`res_level_dependence`, `sat_harmonics`, `drift_stats`, `env_rc_fit` and `voice_variance`; their tests are in
`suite_analog.py` (`refbench.py plan --include-pending`, 37 tests) and 13 targets in `targets.json` carry `"status": "pending"`
(shown as `pending` in the report; they are neither pass nor fail, so `tools/refbench.sh` stays at 39 of 39). They run on the engine
once P7 to P11 exist, and on Diva / Phase Plant only if the maintainers license them.

## 7. Plan CPU rows

`engine/build/crucible_engine_bench plan` (the full bench also prints the effect table and then the same rows): rows with a package
name are skeletons; the reference row (hard-wired voice today) is the reference for the graph-overhead budget of P4.

| row | target | fails at | today |
| --- | --- | --- | --- |
| 16 voices: 2 osc.analog, ladder at 2x, analog envelopes, voice cards | 6 % | 7.2 % | 7.1 % (P11, section 13; the ladder is about 3.1 of it) |
| the same at High quality | 12 % | 14.4 % | 15.9 % (over: the 4x ladder, section 13) |
| Reference patch 16 x 8 unison | 4 % | 4.8 % | 4.4 % |
| graph of 32 fine-grained nodes per voice, 16 voices | 10 % | 12 % | pending P4 |
| each extra FX instance | 1 % | 1.2 % | pending P4 (heaviest single module is measured in the full run) |

## 8. Effects

The before-picture for the FX packages (`docs/plans/ENGINE-FX-PLAN.md`, section 2), produced by FX1. Three pieces:

* `tests/FxMeasure.h`: the effect primitives, built on `AnalogMeasure.h` (header only, double precision, offline).
  `tests/FxMeasureTests.cpp` gives each one a synthetic truth.
* `include/crucible/fx/`: the core library (`Smoothing`, `ChannelMode`, `Detectors`, `Lookahead`, `OversampledBlock`,
  out-of-line parts in `src/fx/FxCore.cpp`) and `module/LatencyReporter.h`, found with
  `ModuleRack::findCapability<LatencyReporter>()`. Tested in the same file.
* `tests/FxBaseline.cpp`: the rack effects that remain (limiter, multiband, ring / shift, EQ cramping, the delay line) measured with those primitives. The tables in 8.3 are its output as of 2026-10-11, when it also measured the first-generation effects.

```sh
engine/build/crucible_engine_tests FxMeasure        # primitives against their truths
engine/build/crucible_engine_tests FxCore           # library acceptance (detectors, lookahead, oversampled block)
engine/build/crucible_engine_fx_baseline [all|peak|eq|dynamics|bugs|reverb|cpu]   # prints the tables (about 3 s)
engine/build/crucible_engine_fx_baseline check      # CTest guard: exit 1 when a row leaves 0.5 dB / 5 % of the plan
```

### 8.1 What each primitive recovers (worst case in the tests)

| primitive | synthetic truth | recovered |
| --- | --- | --- |
| `staticCurve` | ideal gain computer (-20 dB threshold, 4:1), -60 to +6 dBFS | 0.00000 dB (plan 0.01) |
| `stepTimes` | one-pole in dB, 5 / 12 / 150 ms, at 44.1 and 48 kHz | t63 and 10-90 % within 1 % (plan 1 %) |
| `releaseFamily` | two-state model (fast 60 ms, slow 1.2 s charged in 5 ms / 0.8 s), bursts of 0.2 and 2 s | 0.008 % (plan 2 %) |
| `thdCurve`, `harmonicCurve` | polynomial y = x + a2 x^2 + a3 x^3, closed-form H2, H3, THD | 0.00000 dB (plan: P1 truths) |
| `imdSmpte`, `imdCcif` | the same polynomial, closed-form lines (a2 A1 A2, 3 a3 A1^2 A2 / 4, ...) | 0.0000 dB (plan 0.1) |
| `aliasAtDrive` | an inharmonic spur at -80 dB; harmonic distortion alone | -80.0 dB to 0.1 dB; floor -162 dB |
| `nullTest` | pure integer and fractional delay (periodic multisine) | below -150 dB (exact zero) |
| `measureLatency` | multisine delayed by 0 to 100.01 samples | 0.00012 sample (plan 0.01) |
| `clickTest` | gain steps of -40, -60, -80 dB; phase step; smooth 3 Hz modulation | step read within 0.5 dB of the value jump it causes; modulation reads below -120 dB |
| `zipperTest` | 32-sample staircase against smooth automation | staircase -45.0 dB (analytic -44.1), smooth ramp and square below -190 dB |
| `energyDecay` | seven tones with T60 of 0.4 to 1.5 s (EDT, T20, T30) | 0.12 % (plan 2 %), curvature 0.01 % |
| echo density, IACC, tail flatness | Gaussian noise, sparse impulses, delayed copies, white tail with / without a resonance | density 1.00 +- 0.05 against 0.00; IACC 1.000 / 0.99 / below 0.15; flatness 0.42 dB against 8.9 dB |
| `modulationDepth` | FM sine, 25 cent at 5 Hz | 25.0000 cent, 5.0000 Hz (plan 0.1 cent) |
| `wowFlutter` | FM tone at 3.15 kHz, 0.2 % at 0.5 / 4 / 20 Hz | weighted and unweighted peak and RMS within 1 % (plan 2 %) |
| `measureLoudness` | K-weighting at 48 kHz against the printed coefficients (1e-6); -23 dBFS stereo tone; gating; two levels 10 LU apart | -23.00 LUFS within 0.05; range 10.000 LU |
| `stereoStats`, `monoSumPerBand` | identical, inverted and independent noise | correlation +1 / -1 / below 0.02; mono sum 0 dB / below -120 dB / -3 dB |

Notes. `clickTest` reads the jump of the sample value (a gain step e at carrier phase p jumps A e sin p, so click tests
should look at a crest or take the worst of several phases); a step at a zero crossing changes only the slope and reads
about 2 pi f / fs lower. The wow and flutter weighting is a first-order high-pass at 1 Hz and a second-order low-pass at
8 Hz scaled to 0 dB at 4 Hz, an approximation of the shape of the IEC 60386 curve (the standard's table is not
reproduced). `measureLoudness` is an offline BS.1770-4 implementation in the harness; FX4's `Loudness.h` replaces it
with a wrapper and the libebur128 oracle.

### 8.2 Library acceptance

| item | acceptance (plan FX1) | measured |
| --- | --- | --- |
| true-peak reading of an fs/4 sine at 90 phases | within 0.1 dB | 4x with parabolic refinement 0.014 dB, 8x 0.003 dB; 4x without refinement 0.147 dB; the sample peak reads down to -2.86 dB |
| true peak of a broadband signal against the 8x harness reference | - | 0.03 dB |
| running min / max against brute force | equal | exact, windows 1 to 1500, shrinking window, monotone and constant input |
| `Lookahead` length change (1 kHz sine, four changes including queued ones) | no step above -90 dB | -118.9 dB; a cleared buffer (the legacy behaviour) reads above -40 dB |
| `OversampledBlock` null at mix 0, Delay path, FIR and IIR, 2x and 4x | below -120 dB | -152 dB (float precision) |
| `OversampledBlock` Matched path, identity kernel, mix 0 / 0.5 against mix 1 | no comb | exact zero for both families |
| `OversampledBlock` reported latency against the measured delay | within 1 sample | FIR 2x 23 (22.5), 4x 34 (33.75); IIR 2x 4 (3.31), 4x 5 (4.91) |

The FIR half-band latency is not a whole number of samples (22.5 at 2x), so the Delay path is half a sample off the
wet path; the Matched path has no such error and is the one to use when the mix is not at an end. The IIR family is
not linear phase, so only the Matched path keeps a mixed dry / wet signal free of combing.

### 8.3 Baseline of the first-generation effects (48 kHz, Release, Apple M-class, 2026-10-11)

Historical record. The first-generation drive, chorus, flanger, phaser, delay, reverb, compressor, EQ, parametric EQ, dimension,
gate and convolver were removed before v1; the rows that measure them below (drive, compressor,
reverb, the BFX3 delay row, the CPU rows of the removed effects) are no longer produced by `crucible_engine_fx_baseline`.

"plan" is the number printed in section 2 of the plan; `check` fails when a row differs by more than 0.5 dB (levels)
or 5 % (times). Settings are the module defaults unless the row says otherwise: the drive at tone 1, the
compressor at -18 dB / 4:1 / knee 6 dB, the limiter at a -0.3 dB ceiling, 80 ms release and 1.5 ms lookahead,
the reverb at mix 1. Hard at amount 0.3, Fold and Sine alias chaotically (10 dB or more for a 1 dB change of input level),
so their rows are guarded by a bound; the other numbers reproduce at the quoted settings. The multiband "at" row is
on a 24-per-octave grid.

**Drive aliasing (Standard, 5 kHz, -6 dBFS, amount 1)**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| Soft | dB | -28.00 | -27.87 |
| Hard | dB | -27.00 | -26.89 |
| Hard, amount 0.3 (guard: at most -50 dB) | dB | -56.00 | -54.53 |
| Fold (guard: louder than the fundamental) | dB | 7.00 | 13.23 |
| Sine (guard: louder than the fundamental) | dB | 7.00 | 6.65 |

**Drive character**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| Tube, amount 0.3: H2 against the fundamental (1 kHz, -6 dBFS) | dB | -20.90 | -20.89 |
| tone at maximum: gain at 20 kHz | dB | -1.27 | -1.27 |

**Drive DC blocker, amount 0, tone 1 (BFX1)**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| 44.1 kHz: gain at 40 Hz | dB | -0.06 | -0.06 |
| 44.1 kHz: gain at 20 Hz | dB | -0.26 | -0.26 |
| 48.0 kHz: gain at 40 Hz | dB | -0.06 | -0.06 |
| 48.0 kHz: gain at 20 Hz | dB | - | -0.26 |
| 96.0 kHz: gain at 40 Hz | dB | -0.07 | -0.07 |
| 96.0 kHz: gain at 20 Hz | dB | - | -0.26 |
| 192.0 kHz: gain at 40 Hz | dB | -0.07 | -0.07 |
| 192.0 kHz: gain at 20 Hz | dB | -0.26 | -0.26 |

**Bugs found by the baseline**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| BFX3 delay: output after switch-on with silent input (the fix needs -150 dBFS or less) | dBFS | - | -6000.00 |
| BFX5 limiter lookahead 1.5 -> 3 ms: largest step | dB | - | -120.63 |
| BFX6 multiband crossover 400 -> 800 Hz: largest step | dB | - | -110.14 |
| BFX7 frequency shifter, 30 Hz up by 100 Hz at 48 kHz: image against the wanted tone | dB | - | -44.25 |
| BFX7 frequency shifter, 30 Hz up by 100 Hz at 192 kHz: image against the wanted tone | dB | - | -61.81 |

**Limiter (ceiling -0.3 dB, fs/4 sine at 45 degrees)**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| sample peak | dBFS | -0.30 | -0.30 |
| true peak | dBTP | 2.71 | 2.71 |

**Limiter THD**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| 60 Hz sine at +6 dBFS (ceiling -0.3 dB) | dB | -39.00 | -39.49 |

**EQ cramping (RBJ bells, +12 dB, Q 1, 48 kHz): response at 20 kHz**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| 16 kHz bell: digital minus analog | dB | -6.20 | -6.22 |
| 16 kHz bell: digital | dB | 3.42 | 3.42 |
| 16 kHz bell: analog prototype | dB | 9.65 | 9.65 |
| 12 kHz bell: digital minus analog | dB | -4.50 | -4.51 |

**Compressor (threshold -18 dB, ratio 4:1, knee 6 dB)**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| gain at 0 dBFS in (ideal -13.5 dB) | dB | -12.40 | -12.40 |
| attack labelled 10 ms: time to 63 % of the step | ms | 18.50 | 18.32 |
| THD at -6 dBFS, 1 ms attack, 50 ms release: 50 Hz | dB | -34.00 | -33.97 |
| THD at -6 dBFS, 1 ms attack, 50 ms release: 100 Hz | dB | -39.00 | -39.07 |
| THD at -6 dBFS, 1 ms attack, 50 ms release: 1000 Hz | dB | -59.00 | -58.70 |

**Multiband band sum with the dynamics neutral (BFX2): worst deviation**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| crossovers 500 / 750 Hz | dB | 0.00 | -0.00 |
| crossovers 500 / 750 Hz: at | Hz | - | 109.05 |
| crossovers 300 / 1200 Hz | dB | 0.00 | 0.00 |
| crossovers 300 / 1200 Hz: at | Hz | - | 86.55 |
| defaults 200 / 3000 Hz | dB | 0.00 | -0.00 |

**Reverb**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| RT60 set to 1.0 s: measured T30 of the left channel | s | 0.86 | 0.84 |
| RT60 set to 2.5 s: measured T30 of the left channel | s | 2.15 | 2.15 |
| left / right correlation of the tail (plan: 0.00 to 0.02) |  | 0.01 | -0.00 |
| RT60 set to 6.0 s: measured T30 of the left channel | s | 5.16 | 5.19 |

**Delay line**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| cubic fractional read at half a sample: gain at 16 kHz | dB | -3.25 | -3.25 |
| cubic fractional read at half a sample: gain at 20 kHz | dB | -8.40 | -8.41 |

**CPU per instance (percent of one core, stereo, 48 kHz; machine dependent, not checked)**

| quantity | unit | plan | measured |
| --- | --- | --- | --- |
| drive, Standard | % | 0.61 | 0.65 |
| drive, High | % | 1.96 | 2.09 |
| reverb | % | 0.48 | 0.49 |
| limiter | % | 0.33 | 0.34 |
| multiband | % | 0.18 | 0.18 |
| chorus | % | 0.18 | 0.18 |
| compressor | % | 0.10 | 0.12 |
| parametric EQ | % | 0.10 | 0.10 |
| delay | % | 0.05 | 0.05 |
| convolver, built-in hall (3 s) | % | 3.20 | 3.24 |
| convolver, user IR (4 s) | % | 4.30 | 4.35 |

BFX1-BFX7 are fixed (FX2; the numbers before the fixes are in `SOUND_CHANGES.md`): the rows above are the values after
the fixes, and `tests/FxFixTests.cpp` guards each. BFX4 (the hard on / off cut) is a property of the rack and is measured
there (largest step -133 dB, was -36 dB). The shifter's image rejection of a 30 Hz tone shifted up by 100 Hz was -44 dB at
48 kHz and -11 dB at 192 kHz, and is now -44 dB and -62 dB.
The CPU rows depend on the machine; they agree with the plan's figures within about 10 %.

### 8.4 Dynamics (FX3, `tests/DynamicsTests.cpp`, 48 kHz, Release, Apple M-class, 2026-10-11)

`fx.comp_clean`, `fx.comp_vca` and `fx.gate2` on the shared core `fx/Dynamics.h`. The test prints the numbers below.
Run: `build/crucible_engine_tests Dynamics`.

| quantity | plan | measured |
| --- | --- | --- |
| static curve, RMS detector, 1 kHz, -60 to +6 dBFS (five curves: 4:1 knee 6, inf:1 hard, 2:1 knee 24, upward, expander) | within 0.1 dB | 0.005 dB; through the module and rack 0.0006 dB |
| attack and release, time to 63 % of a 20 dB step's gain change, peak detector (1 ms to 2 s) | within 10 % | 4.2 % worst |
| the same, RMS detector with a 10 ms window (labels from 10 ms attack, 50 ms release) | within 10 % | 4.4 % worst; an attack of 1 ms with a 10 ms window reads 4.0 ms (the window limits it) |
| THD, 50 Hz, 10 dB over the threshold, RMS, 10 ms / 100 ms | at most -60 dB | -72.0 dB (20 ms window, module default), -63.2 dB (10 ms window); peak detector -47.4 dB (legacy -34) |
| alias floor, 5 and 7.3 kHz, -12 to 0 dBFS, peak and RMS | - | -103.4 dB |
| overshoot of a 20 dB step, 10 ms lookahead, 1 ms attack | at most 0.1 dB | 0.001 dB (16.9 dB without lookahead) |
| output true peak, fs/4 sine at 45 degrees, threshold -10 dB | - | -10.02 dBTP with the 4x true-peak detector (no parabolic refinement), -6.99 dBTP with sample peaks |
| external key: gain reduction follows the key only | - | 0.000 dB with a silent key; 15.00 dB (ideal 15.00) with the key loud and the audio quiet |
| sidechain filters | - | 1 kHz high-pass keeps a -10 dB 50 Hz tone out (0.000 dB against 16.05); +18 dB bell lifts a 5 kHz tone to 6.00 dB of reduction (ideal 6.00); listen = filtered key within 0.03 dB |
| stereo: linked / unlinked / mid-side lanes | - | 15.00 / 15.00 dB linked; 15.00 / 0.000 unlinked; 15.00 / 0.000 mid / side |
| neutral settings against the input, every channel mode | - | -147.5 dB |
| VCA auto release: t63 after a 50 ms burst against a 2 s burst | at least 2x | 59 ms against 1359 ms (23x); clean compressor 59 against 1104 ms (18.7x) |
| VCA THD at 4 dB of gain reduction | at most -70 dB | -88.7 dB (1 kHz), -83.4 dB (100 Hz) |
| gate hysteresis (6 and 12 dB) | within 5 % | 6.05 and 12.05 dB |
| gate hold (10, 50, 200 ms) | within 5 % | 0.63 % worst |
| gate onset residual, 2 ms lookahead | at most -60 dB | -152 dB (-30 dB without lookahead) |
| ducking depth (6, 20, 60 dB) | within 0.2 dB | 0.000 dB error |
| latency, measured against reported, lookahead 0 to 10 ms with and without true peak | one sample | 0.0002 sample |
| lookahead change while running | no step above -80 dB | -148 dB |
| on / off through the rack, lookahead 0 and 5 ms | no step above -60 dB | -127.6 dB |
| CPU per stereo instance, white noise, blocks of 32 | 0.15 % (0.3 % lookahead + true peak) / 0.2 % / 0.1 % | 0.10 % (0.21 % with lookahead, true peak, hybrid) / 0.10 % / 0.10 % |

Notes. The ballistics run on the dB-domain gain after the gain computer; the detector delays what they see (the RMS average
rises quickly in dB but falls slowly), so the one-pole times are shortened by the delay measured on 20 dB steps. Labels
shorter than about the RMS window cannot be met. Peak detection is a hard window of `peakHoldMs` (a sliding maximum,
`HoldMax`), ripple free for frequencies above one over twice the window. True-peak detection drops the parabolic
refinement of `TruePeakDetector` to stay inside the CPU target (the 4x reading can be 0.15 dB low at worst). Not done:
negative ratios, and aligning the dry path of the rack's 10 ms on / off fade with a module's lookahead (switching on is
click free because the delay line is primed first; the fade-out of a lookahead module crossfades against an undelayed dry
signal).

### 8.5 Loudness and metering (FX4, `tests/LoudnessTests.cpp`, `fx/Loudness.h`, 48 kHz unless noted, Release, Apple M-class, 2026-10-12)

`LoudnessMeter` (K-weighting, momentary / short-term / integrated loudness with both gates, loudness range, sample and 4x true
peak) and the programme meters. Oracle: libebur128 1.2.6 (MIT, `third_party/libebur128/`, tests only, exact block lists). The EBU
signals are synthesised from the published definitions (1 kHz stereo tones at the stated loudness; the levels and durations are
our reading of Tech 3341 cases 1 to 5 and Tech 3342 cases 1 to 4 and are listed in the test). Run: `build/crucible_engine_tests Loudness`.

| quantity | plan | measured |
| --- | --- | --- |
| K-weighting at 48 kHz against the coefficients printed in BS.1770-4 | - | identical to 1e-7; response at 44.1 / 88.2 / 96 / 192 kHz within 0.04 dB of the 48 kHz one (50 Hz to 10 kHz) |
| Tech 3341 cases 1 to 5, integrated loudness (-23.0, -33.0, -23.0 with the relative gate, -23.0 with the absolute gate, -23.0) | within 0.1 LU | 0.000, 0.000, 0.021, 0.021, 0.007 LU error; momentary and short term read the same on the steady tones |
| 400 ms burst (momentary maximum) and 3 s burst (short-term maximum) at -23 LUFS | within 0.1 LU | -23.000 and -23.000 |
| five channels (L, R, C at -28, Ls, Rs at -30 dBFS), surround weight 1.41, LFE weight 0 | - | -24.210 LUFS against -24.210 analytic |
| Tech 3342 cases 1 to 4, loudness range (10, 5, 20, 15 LU) | within 1 LU | 10.000, 4.990, 20.010, 15.010 LU; libebur128 10.000, 5.000, 20.000, 15.000 |
| 100 s of program material (loud, quiet, a very quiet passage, bursts) at 44.1 / 48 / 96 / 192 kHz against libebur128 | within 0.1 LU | integrated 0.0000 LU, short-term maximum 0.007, range 0.015, momentary maximum 0.031 (ours is sampled every 10 ms, libebur128's here every 100 ms), sample peak 0.0000 dB, true peak 0.006 dB; the offline reference of `FxMeasure.h` agrees to 0.001 LU |
| true peak against a 16x, 64-tap windowed-sinc reference: fs/4 sines at four phases, ++-- square (+3.01 dBTP), tones from 997 Hz to 21 kHz, a clipped and low-passed sine, noise low-passed at 16 kHz | within 0.2 dB | -0.000 to +0.014 dB (the 12-tap, 4x interpolator of BS.1770 Annex 2 with parabolic refinement) |
| the same for noise with energy up to Nyquist (outside the design band of any 48-tap interpolator) | - | -0.33 dB (reads low) |
| VU: time to 99 % of a step, overshoot, a sine at -6.02 dBFS | 300 ms, 1 to 1.5 % | 300.0 ms, 1.52 %, -6.033 dB |
| PPM: 10 ms burst of 5 kHz against a steady tone, return time type I / II | - | -1.06 dB; 20 dB in 1.5 s and 24 dB in 2.8 s to 0.01 dB; a steady sine reads its peak |
| correlation: mono, inverted, independent noise | - | +1.0000, -1.0000, -0.0100 |
| block size independence (1, 777, 4800 samples) and non-finite input | - | bit-identical results; NaN and infinity ignored |
| CPU, two channels, blocks of 32 | `util.meter` at most 0.2 % | `LoudnessMeter` 0.030 % without and 0.067 % with true peak (0.058 % and 0.083 % before the block-wise K-weighting and the vectorised true-peak refinement); `util.meter` (adds sample peak, RMS, PPM, VU and correlation) 0.195 % |

Engine cost: with the master analyzer on (the default), `tools/refbench.sh` row `1 note, saw (all threads)` reads 0.586 % (limit 0.662 %,
the best reference); it read 0.679 % after FX4 because the analyzer's meter and the per-block parameter update were slower (the K-weighting
runs a block at a time with the output off the recursion, the 4x true-peak interpolator unrolls its filter and refines the four points
at once, and `log2` / `exp2` of the log-smoothed parameters repeat only when the value moves).

Notes. Gating blocks are 400 ms with 75 % overlap taken from 10 ms sub-blocks; gated statistics keep a histogram in 0.01 LU bins
with the exact energy of each bin, so a gate decision is off by at most one bin and the means are exact. The true-peak
interpolator is only evaluated where the newest samples times its gain bound can reach the decaying peak, which makes it
several times cheaper on program material and changes no reading above that level. Wide-band, hard-clipped material has no true
peak that a short interpolator can fix at 0.1 dB: against a 64-tap reference the 12-tap reading is up to 0.35 dB low on noise with
energy up to Nyquist, and the 16x reference moves with its own length there.

### 8.6 Maximiser, clipper and meter module (FX4, `tests/MaximizerTests.cpp`, 48 kHz, Release, Apple M-class, 2026-10-12)

`fx.maximizer`, `fx.clipper`, `util.meter` on `fx/Maximizer.h`, `fx/Clipper.h` and `fx/Loudness.h`. The torture set is eleven
signals that exceed the ceiling by 4 to 12 dB: fs/4 sines at 0, 22.5 and 45 degrees, a ++-- square, hard-clipped wide-band noise,
1 kHz at +6 dBFS, 11.2 and 19 kHz at +3 dBFS, half-sample-shifted band-limited pulses, a sweep, bass with 7 and 8 kHz tones.
True peak is measured with the 16x, 64-tap reference of section 8.5. Run: `build/crucible_engine_tests Maximizer`.

| quantity | plan | measured |
| --- | --- | --- |
| output true peak minus ceiling, sidechain mode (32-tap 4x detector), ceilings -0.3 / -1 / -6 dBTP, ten band-limited and tonal signals | at most +0.1 dB | -0.030 dB (never above the ceiling) |
| the same on the hard-clipped wide-band noise | at most +0.1 dB | +0.110 dB |
| output true peak minus ceiling, 4x oversampled audio mode, ten signals | at most +0.02 dB | -0.007 dB |
| the same on the hard-clipped wide-band noise | at most +0.02 dB | +0.255 dB (the filter that brings the audio back down rings on a clipped plateau) |
| sample peak minus ceiling, both modes, 33 runs | never above | -0.050 dB and -0.107 dB (the ceiling is clamped last) |
| a signal under the ceiling | - | null against the delayed input -151.9 dB, no gain reduction |
| THD of 60 Hz with 6.07 dB of reduction, adaptive release with auto hold (transparent mode) | at most -60 dB | -167.3 dB; 40 Hz -171.2, 100 Hz -170.1 dB; without auto hold -140.7 dB (60 Hz) and -52.0 dB (40 Hz); legacy `fx.limiter` -61.7 dB |
| fixed release of 200 ms, time to 63 % of the recovery | label | 202.0 ms |
| dual release of 200 ms after a 50 ms burst and after a 3 s burst | fast, then slow | 202.0 ms and 1168.7 ms |
| stereo: both channels reduced alike with link 100 %, quiet channel untouched with link 0 % | - | -6.58 / -6.58 dB, -0.00 dB; link switched every 100 ms: -2.992 dBTP at a -3 ceiling |
| mid / side bias (side turned down more): ceiling kept, side RMS | - | -5.12 dBTP at a -2 ceiling; side -25.9 dB against -16.9 dB |
| soft-clip stage at 100 % on peaky material: limiter reduction | takes a share | 7.56 dB without, 1.05 dB with; true peak -1.024 dBTP at a -1 ceiling |
| dither: TPDF at 16 bits, shaped, TPDF at 24 bits (RMS of the error); power below 3 kHz TPDF against shaped | - | -96.4, -91.3, -144.3 dBFS; -19.1 against -37.0 dB |
| latency, reported against measured, lookahead 0.5 to 10 ms; 4x path; clipper at 1x to 8x with and without the guard | one sample | sidechain 0.0002 sample; 4x path 0.25 sample (the rounding of the filters' latency); clipper 0.0003 sample |
| clipper alias power of 5 kHz at 6 dB over the ceiling, default shape (40 %), second-order ADAA: 2x (Standard) / 4x (High) / 8x | -90 dB / -100 dB | -93.0 / -121.3 / -132.2 dB |
| clipper alias, hard clip, ADAA 2: 1x / 2x / 4x / 8x; ADAA 1: 2x / 4x / 8x; no ADAA at 1x | - | -39.4 / -76.6 / -106.5 / -122.5 dB; -64.2 / -81.5 / -97.2 dB; -29.2 dB |
| clipper small-signal response below the knee, 1 to 20 kHz, ADAA 1 and 2 at 2x, 4x, 8x (with the droop equaliser) | - | within 0.23 dB at 2x (second order), 0.10 dB elsewhere; without the equaliser ADAA 2 at 2x loses 6 dB at 20 kHz |
| clipper with the guard, ceiling -3 dBTP: sample peak, true peak (band-limited signals) | never above | -0.050 dB, +0.057 dB; hard-clipped wide-band noise +0.145 dB; without the guard the band-limited result rises up to 3.0 dB (samples) and 4.3 dB (true peak) above the ceiling |
| CPU per stereo instance, noise at 0.5, blocks of 32: `fx.maximizer` at +10 dB gain | 0.6 % | 0.58 % (0.72 % with the clip stage and shaped dither) |
| `fx.maximizer` with the 4x audio path | 1.2 % | 0.98 % |
| `fx.clipper`, Standard 2x without the guard (continuously clipping noise / signal below the ceiling) | 0.3 % | 0.38 % / 0.32 %; with the guard 0.92 %; 4x without the guard 0.82 % |
| `util.meter` | 0.2 % | 0.195 % |
| allocations on the audio thread with parameter changes every block (all three modules, link, oversampling, guard, ADAA order) | 0 | 0 |

Notes. The limiter holds its ceiling by construction (the hold window is at least as long as the box average over the lookahead, so
every tap of the average contains the overshoot of a peak that is `lookahead - 1` samples ahead of the audio; the release only raises
the held value) and clamps to the ceiling last, so sample peaks are exact. True peak is the part that depends on the detector: a
32-tap 4x interpolator (the BS.1770 filter has 12 taps per phase) keeps 0.05 dB of headroom and is evaluated only where the newest
samples can reach the ceiling; hard-clipped wide-band material has inter-sample peaks that no short interpolator sees (a 12-tap
detector is up to 1.8 dB low on it, 32 taps 0.2 dB, and the 64-tap reference itself keeps growing with its length), which is the
0.11 dB over the ceiling in the sidechain mode and 0.26 dB in the 4x mode. The 4x audio path keeps the pass-band gain of its up and
down filters (measured at `prepare`) plus 0.06 dB below the ceiling. The clipper's guard is a 1 ms lookahead instance of the same
limiter; it costs about 0.55 % on continuously clipping material because the signal then sits at the ceiling and every sample is
interpolated. Second-order ADAA low-passes the linear part with (1 + 2 cos w) / 3; a 9-tap symmetric FIR fitted by least squares to its
inverse restores the response and is part of the reported latency. A hard clip needs 4x for -100 dB at Standard quality; the default
shape needs 2x. ADAA runs as first order at 1x (the second order has a zero at 16 kHz there).

### 8.7 Pro EQ and crossovers (FX5, `tests/EqProTests.cpp`, `tests/CrossoverTests.cpp`, Release, Apple M-class, 2026-10-12)

`fx.eq_pro` on `fx/AnalogMatchedFilters.h`, `fx/LinearPhase.h` and `fx/Crossover.h`. Run: `build/crucible_engine_tests EqPro`
and `build/crucible_engine_tests Crossover`; the tests print the numbers below.

Magnitude against the analog prototype (cookbook analog bell and shelves, the transfer function in `s`), 20 Hz to 20 kHz,
centres 20 Hz to 18 kHz, gains +-3 to +-24 dB, Q 0.5 to 5 (shelves to 2), second-order shelves. The legacy pre-warped bilinear
(RBJ) design is the column on the right.

| fs | matched, \|gain\| <= 12 dB | matched, \|gain\| = 24 dB | bilinear (RBJ), \|gain\| <= 12 dB |
| --- | --- | --- | --- |
| 44.1 kHz | 0.343 dB | 0.624 dB | 8.43 dB |
| 48 kHz | 0.247 dB | 0.407 dB | 7.07 dB |
| 96 kHz | 0.082 dB | 0.183 dB | 1.54 dB |
| 192 kHz | 0.022 dB | 0.032 dB | 0.37 dB |

The plan's 0.1 dB holds from 96 kHz up (for gains up to 12 dB). At 44.1 and 48 kHz a second-order section cannot do better than
about 0.12 to 0.2 dB (a minimax fit over the whole grid gives 0.12 dB for a +12 dB, Q 0.7 bell at 8 kHz at 44.1 kHz; ours is a
least-squares fit with analytic Jacobian at 6 to 8 microseconds per design and reads 0.3 dB); the digital response has to leave
Nyquist flat while the analog one does not. The plan's case, a +12 dB, Q 1 bell: 16 kHz at 44.1 kHz 8.0 dB (bilinear) against
0.21 dB, 12 kHz 5.2 against 0.24 dB. Other shapes, worst error at 44.1 / 48 / 96 kHz: tilt 0.25 / 0.17 / 0.03, band-pass 0.55 /
0.37 / 0.09, notch 0.82 / 0.55 / 0.07, all-pass 0 (unit magnitude), low cut 1.56 / 1.01 / 0.09, high cut 1.35 / 0.75 / 0.15 dB
(all seven slopes and four shapes, compared down to -60 dB; the worst cases sit at corners above 12 kHz). A 20 Hz bell at 192 kHz:
0.0000 dB, a 20 Hz tone through the double-precision filter reads +12.000 dB. Cut slopes 6 to 96 dB per octave within 0.08 dB per
octave, Butterworth -3.01 dB, Linkwitz-Riley -6.02 dB, Bessel -2.99 dB at the corner. TPT state-variable bell: 0.044 dB to
6 kHz, bounded under audio-rate modulation.

| quantity | plan | measured |
| --- | --- | --- |
| module impulse response against the design (rack, blocks of 32) | - | 0.0000 dB; against the analog prototype 0.196 / 0.096 / 0.054 dB (four bands, 44.1 / 48 / 96 kHz) |
| natural phase against minimum phase: phase error to the analog prototype up to 16 kHz, +-12 dB bells | - | 16.9 deg against 18.1 deg at 44.1 kHz (magnitude 0.45 against 0.34 dB); 14.7 against 16.0 deg at 48 kHz: a biquad cannot follow the analog phase near Nyquist, natural phase trades 0.1 dB for 1 deg |
| linear phase, magnitude against the target | within 0.05 dB | 0.0000 dB (3 bands, 2k to 8k taps) |
| linear phase, impulse response symmetry | -120 dB | -144 to -149 dB |
| linear phase, measured delay against the reported latency (1k, 4k, 16k taps) | one sample | 767 / 2303 / 9215 against 767 / 2303 / 9215 (exact) |
| linear phase, mix 0 against the input delayed by the latency | - | -152 dB |
| linear phase, resolution: 100 Hz, Q 4, +10 dB bell, error by kernel length | - | 1023 taps 4.67 dB, 2047 2.34, 4095 0.79, 8191 0.11, 16383 0.004, 32767 0.000 dB |
| linear phase, kernels swapped every 256 samples while a gain moves 24 dB in 0.7 s | no step above -90 dB | -116.7 dB |
| frequency sweep 20 Hz to 20 kHz, bell +12 dB Q 1, 50 ms ramp / 2 Hz square with 20 ms edges, minimum / natural phase | no zipper above -90 dB | -93.7 / -96.8 dB, -93.6 / -96.8 dB |
| gain sweep -24 to +24 dB, same automation | no zipper above -90 dB | -106.4 / -99.7 dB |
| dynamic band, static curve (four settings, above and below the threshold, -60 to 0 dBFS) | within 0.2 dB | 0.028 dB |
| dynamic band, attack 20 / 50 / 150 ms and release 100 / 300 / 800 ms (time to 63 %) | within 10 % | 2.3 % worst |
| channel modes: mid band on a mono signal, on pure side; side band; left; right | - | +9.00 / 0.00; 0.00 on mono, +9.00 on side; left only; right only (setting 9 dB) |
| external key: mid band closed by a loud key / left open by a silent one | - | -12.00 dB / -0.00 dB |
| response view against the designed curve plus the output gain | - | 0.0000 dB (256 points); through the engine and `ViewHub` a 12 dB bell reads 11.99 dB |
| flat module against the input | - | bit-identical |
| CPU per stereo instance, eight active bands, blocks of 32 | 0.25 % minimum, 0.8 % linear | 0.240 % minimum, 0.239 % natural, 0.236 % linear (4k taps); 0.528 % with two dynamic bands |
| allocations on the audio thread, parameters and phase mode changing every block | 0 | 0 |

Crossovers (minimum phase): the sum of 2 to 5 bands is an all-pass, so flat within 1e-10 dB analytically and 1e-5 dB through
`process()`, over 240 settings (five slopes, 2 to 5 bands, random, tightly packed 1.02 to 1.1 apart and wide spacings; plan 0.01 dB).
Bands are -6.021 dB with 0.000 degrees between them at the corner (12, 24, 36, 48 dB per octave; odd orders invert the high
band, the sum is still flat). Linear-phase crossover (4095 taps, Linkwitz-Riley orders 2, 4, 8, 2 to 5 bands): the sum against the
input delayed by the kernel centre is -133.8 dB worst (plan -100 dB); bands are -6.02 dB at the corner (300 and 3000 Hz) and the low band is 96 dB down four octaves above its corner. CPU, five bands: minimum phase 0.64 % (12 and 24 dB/oct), 1.22 % (48 dB/oct); linear phase 0.46 %.

Notes. Design of a section: matched-z poles, a closed-form numerator from the squared-magnitude identity, then up to five
Levenberg-Marquardt steps on about 30 grid points (the correction fades in between 0.004 and 0.02 dB of closed-form error). The
unconstrained fit has local minima, and along a sweep the optimum jumps between them by up to 0.4 in coefficients (0.016 dB at
1 kHz); the pro EQ therefore uses the "smooth" design (Nyquist gain pinned, three parameters, one basin, 0.6 dB worst at 44.1 kHz
for +-12 dB) while a band moves, with up to four designs per control block for fast moves and a 512-sample lag, and swaps in the
fitted design over 4096 samples once the band has rested for eight blocks. Resonant shelves (Q above 2) fall back to the fit.
Dynamic bands in linear-phase mode stay minimum phase behind the FIR. A mode switch ducks the wet path over 10 ms. Offline
(Quality::High) the audio thread waits for the kernel worker so that renders are deterministic.

## 9. VA oscillator `osc.analog` (P9; `tests/AnalogOscillatorTests.cpp`)

Measured on the oscillator instance with the harness of section 1 (Harris-7 window, non-harmonic power below 20 kHz re the
fundamental, C2 to C8 at 44.1, 48 and 96 kHz unless noted). The test bounds are the figures of the plan's 2.7 row (-90 dB);
the plan's table asks for -100 dB on saw / pulse / triangle.

| quantity | plan | measured |
| --- | --- | --- |
| saw aliasing, worst of 21 cases (C8 at 44.1 kHz) | -100 dB | -104 dB |
| pulse 50 % | -100 dB | -108 dB |
| pulse 10 % (its fundamental is 14 dB below a saw's) | -100 dB | -96 dB |
| triangle | -100 dB | -120 dB |
| static pulse widths 2 to 97 %, C2 to C7, re a saw-sized fundamental | -90 dB | -109 dB |
| pitches above fs/4 (notes 126 to 134), crossfade into the one-harmonic table | -90 dB | -101 dB |
| hard sync (ratio 2.37), C2 to C6 | -90 dB | -99 dB |
| soft sync (ratio 2.37), C2 to C6 (period of two master cycles) | -90 dB | -90.4 dB |
| hard / soft sync at C7 (5 kHz slave) | -90 dB | -88 dB |
| through-zero linear FM (ratio 2, index 2), C2 to C6 | -90 dB | -105 dB |
| exponential FM (ratio 2, 0.5 octave), C2 to C6 | -90 dB | -111 dB |
| saw harmonics against 1/n to 10 kHz | 0.1 dB | 0.008 dB |
| triangle odd harmonics against 1/n^2 to 10 kHz | 0.1 dB | 0.008 dB |
| DC of saw / triangle / sine | -100 dBFS | -146 / -327 / -324 dBFS |
| drift, drift = 1: standard deviation (tau 0.5 / 1 / 3 s) | 6 cents | 6.10 / 6.26 / 6.09 cents |
| drift, drift = 1: autocorrelation time (tau 0.5 / 1 / 3 s) | 0.5 / 1 / 3 s | 0.51 / 1.06 / 3.13 s |
| voice-card tolerance 1: pitch standard deviation | 5 cents set | 4.3 cents over 24 cards |
| CPU, saw (percent of one core per voice-oscillator, 48 kHz) | 0.05 % | 0.027 % |
| CPU, pulse with hard sync | 0.12 % | 0.078 % |

Notes on the design that the numbers depend on:
- The table of `MinBlep.h` (32 entries per sample, linear interpolation) limits a saw above 1 kHz to -85 dB and its
  integrated minBLAMP table limits a triangle to -87 dB at C8; `osc.analog` resamples both once to 256 entries per sample
  (cubic) and integrates the corner table from the step table.
- The min-phase step has a centroid delay of 3.2 samples; computing the naive waveform four samples late and stretching the
  table to a centroid of exactly four makes every edge land on its exact position and removes the DC (a plain minBLEP saw
  carries 2 x 3.2 x f0 / fs, -30 dBFS at 220 Hz). The cost is a fixed latency of four samples and a kernel whose
  pass band ends at 0.8 of Nyquist.
- Through-zero FM follows the phase along the quartic through five samples of the instantaneous frequency and splits the
  sample where the speed reverses, so a boundary is never crossed twice inside one piece; the alias floor rises with
  the FM deviation relative to the sample rate (index 2.4 at ratio 3 reaches -93 dB at C5, a deviation of 5 kHz).
- The old drift (`voice_drift`, section 4.6) is unchanged; `an_drift` is the Ornstein-Uhlenbeck process of the plan.

## 10. Analog filters (P7: transistor ladder, OTA cascade, coupled diode ladder)

Measured with `tests/AnalogFilterLadderTests.cpp` (48 kHz, Release, Apple M1 Max, 2026-10-11; `crucible_engine_tests AnalogFilterLadder`).
The models are `FilterModel` 17 to 19 and the interface is `AnalogFilter` (`include/crucible/filters/FilterModel.h`); the legacy ladder
(index 8) keeps the baseline of section 3. All rows use the model's own `dcGain()` / `selfOscillationGain()` for the reference value.

| row (section 3.1 of the plan) | pass rule | transistor ladder | OTA cascade | coupled diode ladder |
| --- | --- | --- | --- | --- |
| Self-oscillation tracking, resonance 1, start-up noise on, 40 Hz to 8 kHz, Standard | <= 5 cents, level within 3 dB | -0.03 cent worst | -0.76 cent worst | -0.09 cent worst |
| the same at High (100 Hz to 8 kHz) | <= 5 cents | -0.03 cent | -0.73 cent | -0.06 cent |
| the same at Draft (100 Hz to 8 kHz) | <= 15 cents | -0.03 cent | -0.99 cent | -0.03 cent |
| Self-oscillation level (limit cycle, RMS) | - | -22 dBFS | -5 dBFS | -30 dBFS |
| Threshold: first k that builds up (linear limit) | transistor and OTA 4.0 to 4.3 | 4.05 (4.00) | 4.05 (4.00) | 16.60 (16.60); resonance 1 is k = 17.4 |
| Bass loss, -40 dBFS, resonance 0 to 0.95 | within 0.5 dB of `-20 log10(1 + k)` | 0.00 dB (-13.96 dB at k = 3.99) | 0.00 dB from its own law (built-in 35 % compensation: -6.46 dB at k = 4.18) | 0.01 dB (-24.88 dB at k = 16.5) |
| with `character` = 1 | within 0.5 dB of 0 | 0.00 dB | 0.00 dB | 0.01 dB |
| Slope, 2 to 4 octaves above the cutoff | 24 +-1.5 (ladder, OTA), 18 +-3 (diode) | -23.72 dB/oct (Standard), -23.66 (High) | -23.72, -23.66 | -17.44, -17.39 |
| Level dependence, peak gain at resonance 0.9 for -40 / -12 / 0 / +6 dBFS | monotonic compression | 10.4, 0.3, -6.9, -11.7 dB | 16.1, 9.2, 2.5, -3.5 dB | 3.1, -8.5, -15.0, -20.9 dB |
| Aliasing, 5 kHz, drive 1, -6 dBFS, Standard | <= -90 dB | -111.7 dB | -101.5 dB | -107.2 dB |
| the same at High, 0 dBFS | <= -100 dB | -120.9 dB | -107.9 dB | -122.2 dB |
| Standard, drive 0, 0 dBFS (default settings) | <= -90 dB | -136.1 dB | -127.0 dB | -96.8 dB |
| 60 s of random cutoff, resonance, drive, tolerance and audio, peak of the output (Draft / Standard / High / four lanes) | finite | 2.1 / 2.3 / 2.5 / 2.1 | 44 / 10 / 4.3 / 31 | 3.3 / 3.9 / 16 / 3.1 |

The old ladder's gaps of section 2 are closed by the new model: resonance 1 self-oscillates at every cutoff from 40 Hz (from the -120 dB start-up noise alone the transistor ladder needs about 10 s at 40 Hz; the test adds a
-60 dBFS kick and 8 s; the diode ladder oscillates weakly and builds up more slowly), and the bass loss follows the law (-13.96 dB at k = 3.99 against the
legacy ladder's -5.0 dB).

What is not in the table and was measured:

* **CPU** (one voice, 48 kHz, 128-sample blocks, noise input, drive 0, resonance 0.7; percent of one core): Standard 2x: transistor 0.38 %, OTA 0.41 %, diode 0.46 %
  for one voice on its own; 0.117 %, 0.126 %, 0.142 % per voice when four voices go through `processQuad` (plan targets 0.12 %, 0.12 %, 0.18 %; the bench fails at
  target + 20 %). With drive 0.5: 0.126 %, 0.135 %, 0.150 % per voice. `processQuad` is 3.1 to 3.3 times faster than four scalar calls (target 2.5).
  **The scalar per-voice target is not met: one voice alone costs 3.2 times the plan figure** (a Newton step is a chain of dependent divisions, so one voice is
  latency bound while four lanes fill the pipeline). Draft 1x: 0.18 / 0.19 / 0.22 % (0.056 / 0.060 / 0.068 % per voice in a quad); High 4x: 1.35 / 1.49 / 1.74 %
  (0.41 / 0.44 / 0.48 % per voice in a quad). Through `FilterUnit` (stereo, one sample per call, Voice's path today) a stereo pair costs 1.1 to 1.2 % at Standard.
* **Aliasing against level** (drive 1, fc 8 kHz, resonance 0.5, 5 kHz): Standard holds -90 dB up to -6 dBFS and falls to -72 to -80 dB at 0 dBFS; High holds
  -100 dB to 0 dBFS. The drive shaper tanh(a x) runs at twice the filter rate (4x at Standard) because a hard-clipped input is what aliases; before it was added, the
  in-loop tanh alone gave -48 dB (transistor), -14 dB (OTA) and -28 dB (diode) at -6 dBFS and Standard.
* **Solver**: Standard takes one Newton step from the linear ladder, High iterates to 1e-6 (at most 4). A step linearised only at the previous sample
  (no Newton correction) held the loop below fc/fs of 1/8 and ran away above it (transistor Draft at 8 kHz: -315 cents, 7.6 dBFS), so Draft runs the Standard step at 1x.
* **Start-up**: the first `prepare()` of a model and tier measures its pitch-correction table (28 cutoffs): about 11 to 13 ms at Draft, 80 to 100 ms at High per model, in between at Standard; later `prepare()` calls reuse it.
* **Tolerance** 1 (seeds 1 to 48, resonance 1 at 1 kHz; cutoff +-4 %, feedback +-3 %, stage capacitors +-1 %): worst pitch offset 78 cents (transistor), 116 (OTA), 92 (diode), mean +8 to +10 cents; the same seed always gives the same instrument.
* Legacy models 0 to 16 and every golden render are bit-identical to before (`tests/golden/` unchanged).

## 11. Analog filters, second family (P8: Sallen-Key, OTA SVF, OTA 4-pole, passive high-pass)

Measured with `tests/AnalogFilterSkTests.cpp` (48 kHz, Release, Apple M1 Max, 2026-10-11; `crucible_engine_tests AnalogFilterSk`). Models are `FilterModel` 20 to 24
(Sallen-Key LP / HP, OTA SVF, OTA 4-pole, Passive HP); the interface and tiers are those of section 10. `character` is each model's own voicing (Sallen-Key: diode
clipper hardness; SVF: low-pass 0, notch 0.5, high-pass 1; 4-pole: remaining gain compensation; passive: none). `filterN_hp` puts a one-pole passive high-pass
(off, 60, 180, 540 Hz) ahead of the Sallen-Key LP, SVF and 4-pole.

| row | pass rule | SK LP | SK HP | OTA SVF | OTA 4-pole | passive HP |
| --- | --- | --- | --- | --- | --- | --- |
| Self-oscillation tracking, resonance 1, 40 Hz to 8 kHz, Standard / High / Draft | 5 / 5 / 15 cents | 0.13 / 0.13 / 0.03 cent | 0.07 / 0.06 / 0.01 | never oscillates | 0.11 / 0.11 / 0.11 | - |
| Limit cycle level, steadiness | within 3 dB | -18.3 dBFS, 0.00 dB | -18.3, 0.00 | - | -14.2, 0.00 | - |
| Harmonics of the limit cycle (1 kHz) | SK: even and odd | H2 -27.4, H3 -35.0, H4 -52.7 dB | same | - | H2 -170, H3 -52 (odd only) | - |
| Threshold: first loop gain that builds up | SK Ka 3.0; 4-pole 4.0 to 4.3; SVF never | 3.01 (linear 3.00) | 3.01 | Q capped at 20: rings down to -116 .. -141 dBFS (noise floor) at every cutoff and tier | 4.05 (4.00) | - |
| Bass loss, resonance 0 to 0.95, 25 Hz | SK / SVF: 0 dB; 4-pole: its law within 0.5 dB | 0.00 dB | 0.03 dB pass band (16 fc) | 0.00 dB | -7.54 dB at 0.5, -9.69 dB at 0.95 (law -9.97); character 1: 0 dB | - |
| Bass loss at k = 3.8 (resonance 0.9) | ladder < 4-pole < cascade | - | - | - | ladder -13.6 < 4-pole -9.8 < cascade -6.4 dB | - |
| Slope, 2 to 4 octaves from the cutoff (Standard / High) | 12 +-1; 24 +-1.5; HPF 6 and 12 within 0.5 | -11.86 / -11.83 | +11.82 / +11.82 | -11.86 / -11.83 (HP morph +11.82) | -23.72 / -23.66 | 1 section +5.91, 2 sections +11.82 (Draft 5.92 / 11.84) |
| Passive HP corner | exact | - | - | - | - | -3.010 dB (1 section), -6.021 dB (2); flat above |
| Level dependence, peak gain at -40 / -12 / 0 / +6 dBFS | monotonic compression | 14.3, 4.1, -0.9, -2.9 dB (res 0.85) | 14.6, 4.0, -1.8, -4.2 | 20.5, 18.1, 10.7, 4.8 (res 0.9; +20 dBFS: -9.0) | 13.5, 6.8, 0.4, -5.5 | linear |
| Aliasing 5 kHz, default settings, Standard, -6 / 0 dBFS | <= -90 dB | -130.1 / -91.4 | -134.6 / -134.4 | -134.0 / -134.8 | -138.9 / -136.6 | -134.7 / -134.7 |
| the same at High | <= -100 dB | -137.5 / -136.2 | -132.8 / -134.2 | -132.9 / -134.1 | -138.3 / -137.1 | -138.8 / -138.8 |
| Full drive, Standard, -6 / 0 dBFS | <= -90 dB at -6 dBFS | -115.0 / -101.5 | **-69.8 / -44.9** | -109.8 / -80.7 | -103.4 / -73.5 | -116.7 / -86.4 |
| Full drive, High, 0 dBFS | <= -100 dB | -123.5 | **-87.9** | -123.8 | -111.6 | -111.8 |
| 60 s random control and audio (Draft / Standard / High / four lanes), peak out for peak in | finite, < 100 (SVF: < 200 x input) | all < 100 | all < 100 | 336 / 308 / 6.3 / 1242 for input 13 | all < 100 | all < 100 |

Other rows: the SVF morph (resonance 0.3): low-pass keeps 250 Hz (+0.4 dB), high-pass rejects it (-23.7 dB), notch centre -88 dB; resonance 0.8 peaks 19 dB above resonance 0.2.
The high-pass steps give -3.01 / -3.01 / -3.05 dB at their corners and -18.1 to -18.3 dB one pole below. Tolerance 1 on the Sallen-Key (32 seeds): worst 71 cents, mean +11.
Sallen-Key tracking holds across the whole `character` dial (<= 1.6 cents): the pitch table is measured at character 0 and 1 and interpolated.
`processQuad` equals four mono filters (< 1e-4), and the output does not depend on the block size (< 1e-5, blocks of 1, 7, 128).

* **CPU** (one voice, 128-sample blocks, noise, drive 0.2, resonance 0.7; percent of one core): Standard, processQuad per voice / one voice alone:
  SK LP 0.078 / 0.22, SK HP 0.079 / 0.22, SVF 0.081-0.084 / 0.25, 4-pole 0.129-0.142 / 0.40-0.48, passive 0.042 / 0.10 (targets: 2-pole 0.08, 4-pole 0.12; fail at +20 %).
  **The single-voice target (0.08 %) is not met by the nonlinear 2-pole models: they cost 0.22 to 0.26 %, about 3 times the plan.** The resampling chain alone (a null kernel)
  costs 0.062 % for one voice at Standard (0.024 % per voice in a quad), so no kernel could reach 0.08 % without a cheaper chain; the kernels were cut from 0.34 % to 0.22 %
  (rational saturator without a clamp, one division per Newton step, coefficients refreshed every fourth oversampled sample, states folded into one multiply-add per
  coefficient). Through `FilterUnit` (one sample per call, the voice's path today) costs are higher, as in section 10.
* **Saturators**: the Sallen-Key clipper, the SVF integrators and the 4-pole use the rational `g(u) = (27 u + u^3) / (27 + 9 u^2)` (tanh-like, residual slope 1/9 instead of a flat top);
  the SVF therefore stays a resonator at overload (stress peak above), while steady signals compress strongly (+20 dBFS: -9 dB at the peak).
* **Aliasing at full drive**: as in section 10, Standard falls short at 0 dBFS (-73 to -101 dB). The high-pass types (SK HP, passive HP) fall short even at -6 dBFS or High, because the
  hard-driven input keeps its harmonics instead of having them removed by a low-pass before they fold; the rows are recorded, not asserted. Default settings pass everywhere at 0 and -6 dBFS.
* The Sallen-Key LP at default settings, Standard, 0 dBFS: -91.4 dB, a margin of 1.4 dB.
* Legacy models 0 to 19 and every golden render are bit-identical to before.

## 12. Voices and the analog filter bank (voice-quad: `tests/VoiceQuadTests.cpp`, `crucible_engine_bench analog`)

Before this package `Voice` called each model one sample at a time per channel; now the sounding voices share four-lane objects (`FilterBank.h`, docs/ARCHITECTURE.md).
Measured through the whole engine: the same patch (one oscillator, cutoff 2000 Hz, resonance 0.7, drive 0.2) with the filter on and off, Standard, 48 kHz,
512-sample blocks, best of three, Apple M1 Max, 2026-10-12, percent of one core. A stereo voice runs two lanes; the plan's targets are per mono lane.

| model | target / fails at | before, per lane (8 / 16 voices) | after, per lane (8 / 16 voices) | after, per stereo voice (8 / 16) |
| --- | --- | --- | --- | --- |
| Transistor ladder | 0.12 / 0.14 | 0.535 / 0.536 | 0.130 / 0.130 | 0.260 / 0.260 |
| OTA cascade | 0.12 / 0.14 | 0.547 / 0.547 | 0.137 / 0.137 | 0.275 / 0.274 |
| Coupled diode ladder | 0.18 / 0.22 | 0.601 / 0.607 | 0.153 / 0.154 | 0.307 / 0.307 |
| Sallen-Key LP | 0.08 / 0.10 | 0.428 / 0.427 | 0.080 / 0.080 | 0.161 / 0.160 |
| Sallen-Key HP | 0.08 / 0.10 | 0.427 / 0.428 | 0.080 / 0.081 | 0.160 / 0.161 |
| OTA SVF | 0.08 / 0.10 | 0.485 / 0.484 | 0.084 / 0.084 | 0.168 / 0.169 |
| OTA 4-pole | 0.12 / 0.14 | 0.540 / 0.537 | 0.130 / 0.130 | 0.260 / 0.259 |
| Passive HP | 0.08 / 0.10 | 0.285 / 0.283 | 0.045 / 0.045 | 0.090 / 0.090 |

All rows are within the bench's target + 20 %; the transistor ladder, OTA cascade, OTA 4-pole and OTA SVF sit between the target and +20 %. The stereo voice costs twice the per-lane
figure (left and right are separate filters); the old per-voice figures of sections 10 and 11 (0.22-0.48 %) were measured on a single mono lane through `process()` in 128-sample
blocks, while the old voice path called it one sample at a time (the per-lane figures above), which is where most of the saving comes from.

* **Equivalence** (`tests/VoiceQuadTests.cpp`, odd block lengths so voices start inside a control cell; five staggered notes with note-offs, unison with spread, envelope and key tracking
  of the cutoff, steals with a 4-voice pool, mono and legato with glide, serial / parallel / split routing with two slots, model switches and a slot switched off and on): the bank
  and voice-by-voice renders differ by at most 1.5e-6 (peak 0.004 to 1.6), bound 1e-5 of the peak. At resonance 1 and drive 1 (self-oscillation in a nonlinear loop) rounding differences of 1e-7 in the
  integrator-gain ramp grow to 3.4e-4 (OTA cascade) and 0.015 (diode ladder, 1.4 % of the peak; 3.5 % in the scalar and SSE builds; a phase and amplitude error of the same oscillation); the test bound there is 8 %. Bit-exact
  agreement is not reachable: the lanes of a vector and the scalar path round differently and the ramps advance in different step sizes. Legacy models 0-16: bit-identical (goldens unchanged, and a test compares both paths).
* **Behaviour kept**: voice stealing and fades, note-off, retrigger, mono, unison, per-voice cutoff / resonance / drive modulation, mix and pan per slot, the three routings. A new voice or a model
  change resets its two lanes only. A slot switched off freezes its lanes as it froze the voice's own filter.
* **Cost of idle lanes**: a pair of voices shares one object, so a pair with one sounding voice costs a full quad; voices are allocated lowest index first, which keeps the quads full. Memory:
  one four-lane object per slot, model and voice pair (24 voices, 2 slots: 192 objects, about 0.5 MB).
* **Not covered**: graph products (`filter.analog` node in a voice graph) still run one voice per node call; a mono voice (left equals right) still uses two lanes.

## 13. Adopted and new engines (P13: `osc.modal`, `osc.harmonic`, `osc.spectral`, spectral tools, FFT backend; `tests/EnginesTests.cpp`)

Measured on the oscillator instances with the harness of section 1 (windowed DTFT peak search for frequencies, a least-squares
slope of the Hann-windowed envelope for T60, Harris-7 spectra), Release, Apple M-class, 2026-10-12. Timing figures are the best of
five runs of 10 s of audio in 32-sample chunks on a machine that was shared with other builds.

### 12.1 Modal oscillator (`osc.modal`)

| quantity | plan | measured |
| --- | --- | --- |
| resonator mode frequencies, 316 modes (44.1 / 48 / 96 kHz, C2, C3, A4, geometry 0.25 / 0.5 / 0.8, first 12 modes) | 1 cent | worst 0.0003 cent |
| resonator T60, 48 modes (3 rates, decay 0.6 / 2.5 s, brightness 1 / 0.6, modes 1, 2, 4, 8) against the Rings / Elements decay law | 5 % | worst 0.013 % |
| string fundamental, C2 to C7 at 44.1 / 48 / 96 kHz (dispersion 0) | 1 cent | worst 0.002 cent |
| string T60 (1.5 s), same cases | 5 % | worst 0.95 % |
| level above 0.45 fs re the spectral peak, both models, six exciters, 44.1 and 96 kHz, C2 to E7, timbre 1 | nothing | worst -72.8 dB (strike and pluck: at the float noise floor, below -146 dB); modes stop at 0.40 fs |
| 60 s at the extremes (decay 30 s, geometry 0.9, brightness 1, sustain 1, tolerance 1; 24 cases) | bounded | finite, worst peak 2.04 (the output guard is at 4) |
| CPU, 64-mode resonator, noise exciter, decay 30 s (percent of one core, 48 kHz) | 0.3 % | 0.19 % |
| CPU, resonator in the scalar fallback build (`CRUCIBLE_FORCE_SCALAR`, no NEON / SSE kernels; the test is skipped there) | 0.3 % | 0.53 % (bowed 0.58 %) |
| CPU, string | 0.3 % | 0.10 to 0.13 % |
| CPU, resonator with the bowed banded waveguides | 0.3 % | 0.24 % |
| CPU, resonator while the pitch moves every chunk (vibrato; coefficients refreshed every fourth chunk) | | about 0.32 % |
| tolerance 1: pitch standard deviation over 24 cards (tolerance 0: 0.0000 cent) | | 7.1 cents |

The modes are zero-delay-feedback state-variable filters whose coefficients come from the z-plane pole (radius from the T60, angle from the
frequency) through the bilinear map, which is why frequency and T60 do not depend on the sample rate; the low-pass output of each mode is
scaled so that the impulse response peaks at 1. The string reads its delay line with a cubic Hermite interpolator; its loop delay is the period
minus the exact phase delays at the fundamental of the two-pole loss filter, the loop's DC blocker, the dispersion all-pass and the
interpolator, and its loop gain removes the filter gains at the fundamental. A loop whose requested T60 is beyond what the loss filter allows at
that pitch saturates at unity gain (the figures above stay inside that limit). A continuous exciter's level is divided by sqrt(T60 / 1.2 s) so that
the steady level does not grow with the decay time.

### 12.2 Harmonic oscillator (`osc.harmonic`)

| quantity | plan | measured |
| --- | --- | --- |
| partial levels against the documented law (count, odd / even, tilt, formant bell, per-partial decay; 5 settings) | 0.1 dB | worst 0.0001 dB |
| stretch: partials at n^(1 + 0.2 s) | | within 1 cent (s = -1, 0.5, 1) |
| level above 0.45 fs (128 partials, C7) | | below -80 dB |
| CPU, 16 partials / 128 partials | | 0.21 % / 1.3 % (module class Heavy) |

### 12.3 Spectral oscillator (`osc.spectral`, Signalsmith Stretch 1.4.0)

| quantity | plan | measured |
| --- | --- | --- |
| pitch kept while stretching 0.5x to 2x (110 Hz, 440 Hz, 1.76 kHz sines at 44.1 and 48 kHz, 5 stretch factors) | 2 cents | worst 0.035 cent |
| playhead: a linear chirp read at stretch 0.5 / 1 / 2 | | frequency within 4 % of source time / stretch |
| onset after a note starts (latency primed with `outputSeek`) | | first sample above -34 dBFS within 2 samples |
| transposition by the key or `spec_semi` (-12 to +12 semitones; not in the plan's criterion) | | worst 6.5 cents (a property of the phase-vocoder pitch shift) |
| CPU, stereo, one voice | | 0.70 % |
| first chunk of a note (primes the stretcher) | | 0.57 ms |
| memory | | 0.75 MB per voice and slot that selects the type, built on the control thread by `OscillatorInstance::arm()`; 0 before (an engine with the new modules registered grows from 47 to 58 MB at construction and `prepare`, mostly the 25 KB cloud renderer of `osc.harmonic` per voice and slot and the frame caches of the spectral wavetable path) |

### 12.4 Spectral frame interpolation and warps (`Spectral.h`, `oscN_wt_interp`, `oscN_wt_spec`)

| quantity | plan | measured |
| --- | --- | --- |
| two frames with a formant at harmonic 8 and at 24, mix 0.5: peak of a sweep / of a crossfade | sweeps | sweep: one peak at 16 (position within 0.01 harmonic of 8 + 16 x for x = 0 to 1 in steps of 1/8); crossfade: two half-height bumps at 8 and 24, nothing at 16 |
| energy of the swept spectrum against the interpolation of the two frame energies | | within 2 % |
| end points (mix 0 and 1) against the frames; two identical frames | | equal (< 1e-3) |
| through the voice, user table of alternating frames played halfway between two: amplitude at harmonics 8 / 16 / 24 | | crossfade 0.0500 / 0.0000 / 0.0499, sweep 0.0066 / 0.0976 / 0.0000 |
| a smooth factory table (4) between its frames | | sweep and crossfade differ by 0.05 % rms |
| synthesised cycle against the table frame, full band and mip levels 0, 3, 8, 12, 16 | | relative error below 1e-4 (energy ratio 1e-8) |
| warps (shift, stretch, blur, comb, dispersion, tilt) | | peak moves by the stated number of harmonics; level matched within 0.1 %; tilt slope 24 dB between harmonics 4 and 16 at +1 |
| analysis calls while rendering | none | 0 (`SpectralTable::analysisCount()` is constant in every test, including the voice) |
| allocations while rendering with the P13 oscillators and the spectral wavetable path | 0 | 0 |

The sweep matches the peaks of two frames on the control thread (a monotone alignment of up to 24 peaks per frame), so for tables whose
neighbouring frames differ little (the factory tables, 64 frames) it is nearly the crossfade; it matters for tables whose neighbouring
frames differ a lot (sparse imports). Mip levels with more than 4096 samples do not exist; every level of the bank (128 to 4096 samples)
is served.

### 12.5 FFT backends (`Fft.h`, `tests/FftBench.cpp`)

`crucible_engine_fft_bench` (best of five, microseconds per forward transform of n real samples, Apple M-class, NEON):

| n | portable (double radix-2) | pffft | Accelerate path of `Fft::forward()` | pffft vs portable | pffft vs Accelerate |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 0.36 | 0.08 | 0.11 | 4.7x | 1.42x |
| 256 | 1.79 | 0.26 | 0.46 | 6.9x | 1.80x |
| 1024 | 8.83 | 1.03 | 2.04 | 8.6x | 1.98x |
| 2048 | 18.69 | 2.19 | 4.27 | 8.6x | 1.95x |
| 4096 | 41.19 | 4.46 | 9.14 | 9.2x | 2.05x |
| 8192 | 99.06 | 10.72 | 19.19 | 9.2x | 1.79x |
| 32768 | 511.91 | 60.87 | 96.26 | 8.4x | 1.58x |

The adoption rule of the plan (at least 1.5x) is met, so pffft is the backend of `Fft.h` where there is no Accelerate. Apple keeps Accelerate
so that every recorded render stays bit-identical; `-DCRUCIBLE_ENGINE_FFT_BACKEND=PFFFT` selects pffft there too. Without NEON (pffft's
scalar code) the ratio to the portable code is 3.0x to 4.6x. Linux, Windows and x86 SSE were not measured here: run the benchmark with
`--check` on those platforms. Results of pffft against the portable reference: forward differs by at most 1.9e-7 of the largest bin
(sizes 4 to 32768), inverse and round trip by 1.8e-7 of the largest sample; the test bound is 1e-6.

## 14. Analog envelopes, voice cards, noise floor and hum (P11: `tests/AnalogVoiceTests.cpp`, `AnalogEnvelope.h`, `VoiceCard.h`)

The module `voice.analog` (registered last) adds, per envelope (`amp`, `menv`, `env3`, `env4`): `_mode` (Digital default / Analog RC), `_retrig` (From Level default / Reset / Legato / ADR),
`_overshoot` (0.35), `_vel_time` and `_key_time` (0); the voice card amounts `voice_card_osc`, `_filter`, `_env`, `_level` (0 = no card = the legacy voice); and `voice_noise_on` / `_db`, `voice_hum_on` /
`_db` / `_hz` (off). Graph nodes `env.analog` and `env.adr`. **Digital is the default and bit-identical**: the goldens and bit-exact fingerprints are unchanged, and the wrapped envelope equals a bare `Adsr`
sample for sample through note-on, note-off, retrigger and parameter changes.

* **RC envelope** (48 kHz, per-sample trace of the class; 5 attack times from 5 ms to 2 s x 3 overshoots; the time set for decay and release is the time to fall within 1 %, tau = time / ln 100):

| quantity | plan | measured |
| --- | --- | --- |
| attack reaches 1 at the attack time | within 2 % | 0.000 % (the level crosses 1 within one sample; also at 44.1 and 96 kHz) |
| attack fit toward 1 + overshoot: time constant, residual | residual < 0.5 % | tau within 0.000 % of `attack / ln((1 + o) / o)`, residual 0.0000 %, fitted target 1.35 |
| decay (sustain 0.5), release | tau within tolerance, residual < 0.5 % | tau 0.06514 / 0.08686 s against nominal 0.06514 / 0.08686, residual 0.0000 % |
| sustain moved 0.5 to 0.9 while held | moves through the RC | tau 0.06514 s (the decay's), target 0.9000, no step |
| retrigger From Level | continuous | first step 0.0003 at level 0.281; peaks after 0.0827 s, the circuit's `tau ln((1.35 - level) / 0.35)` |
| velocity / key time scaling | - | +-1: times x 0.5 / x 2 exactly (full velocity, an octave of key) |

  Read back from audio through the engine (sine A4, attack 100 ms, decay 300 ms, sustain 50 %, release 400 ms, 3-period RMS window, `crucible_engine_measure envelope`): the peak is at 0.099 s (1 %),
  attack tau 0.074 s (nominal 0.0742, target level 1.36) with a fit residual of 0.19 %, decay tau 0.065 s (0.13 %), release tau 0.087 s (0.03 %). The digital rows of section 4.5 (residual 2.66 / 0.77 / 0.21 %)
  are what the Digital mode still gives.
* **Retrigger**: Reset fades the old level to zero over 0.5 ms (quintic smoothstep) while the attack starts from zero; Legato ignores a note-on while the key is held (the trace is identical) and restarts
  from the level after a note-off; ADR ignores the sustain, ends by itself while the key is held and releases from the level. **Click** (a 1 kHz tone under the envelope, RMS of 6 to 22 kHz in
  a window of 8192 samples around the retrigger, attack 10 ms): From Level -175 dBFS at sustain 1 (nothing changes), -99 (RC) / -109 (Digital) at sustain 0.5; Reset -84.8 (RC) and -86.0 (Digital) at sustain 1,
  -89.3 / -91.6 at sustain 0.5. Plan: below -83 dB; met, 1.8 dB of margin at the worst case (a full-scale level cut to zero in 0.5 ms). A 2 ms attack puts its own corner into the band (-79.6 dBFS):
  the metric measures the retrigger, so it is taken with an attack of 10 ms.
* **Voice card** (`VoiceCard::make(seed, voice)`; unit-variance draws, a sum of twelve uniforms, so bounded to +-6): over 64 cards the draws have |mean| <= 0.3 and std 0.86 to 1.04 (pooled over
  24,576 oscillator draws: mean -0.002, std 0.993); largest draw 3.7. Identical per seed and voice, different for another seed or voice. At tolerance 1: envelope times std 5.4 % (5 % set), amp gain std 1.9 % (2 %), pitch std 5.3
  cents (5). Through the engine (16 simultaneous voices of sines, `voice_card_osc` = 1): pitch std 4.82 cents, 0.00 cents without the card; the amount scales the spread linearly. `osc.analog` takes its per-unison draws from the card when the
  voice has one (the same offset, -11.92 cents, note after note on voice 0) and from the note seed as before otherwise (6.5, 1.1, -7.5 cents on three notes); its `an_tol` and the card amount combine as the larger. Envelope time of voice 0
  follows its card (measured x1.0027 against 1.0021 in the classic voice, x1.0181 against 1.0193 for an `env.analog` node, whose entry is chosen by the node's position in the layer). The card's filter tolerance reaches the analog models
  (`FilterSettings::tolerance`, filter seed from the card): the filter's RMS moves 0.2486 to 0.2464 (bank) / 0.2477 (single) at tolerance 1. With a filter tolerance the bank and the voice's own filters draw different component values
  (the bank's lanes are seeded by their place in a quad), so only the other card parts are compared between the two paths.
* **Noise floor and hum** (added to the output after the master gain, ahead of the effects; RMS in dBFS, 2 s measured): white noise at -120, -108, ..., -60 dBFS measures -119.99 ... -59.99 dBFS (L) and -120.03 ... -60.03 (R), L/R correlation
  -0.002, flat spectrum within 0.6 dB between 100 Hz - 4 kHz and 12 - 20 kHz, identical per seed. Hum (fundamental plus harmonics 2 and 3 at 0.5 and 0.3) measures exactly the set level at -120, -96, -84 and -60 dBFS, at 50.00 and 60.00 Hz, second harmonic
  -6.0 dB, the same on both channels. Both are off by default and the idle engine is exactly silent; a level change ramps over a control chunk.
* **Voice-quad batching** stays engaged with analog envelopes, Reset retrigger, velocity scaling and the card's oscillator, envelope and amplifier parts: bank and voice-by-voice renders differ by 9.8e-7 at a peak of 1.07
  (the bank runs; the renders are not bit-identical), bound 1e-5.
* **Real time**: zero `operator new` calls with 16 voices, analog envelopes in all four slots, every retrigger mode switched while playing, cards, noise and hum, and a parameter change per block.
* **CPU** (`crucible_engine_bench plan`, 16 voices of 2 osc.analog, transistor ladder, analog RC envelopes, all four card parts; 512-sample blocks, best of two; the machine was loaded by other builds): Standard 7.1 % against the
  plan's 6 % (fails at 7.2 %); High 15.9 % against 12 % (fails at 14.4 %: over). The same patch without the envelopes and cards costs 7.10 % / 15.91 %, so P11 adds nothing measurable; the cost is the
  ladder (the same patch with the legacy SVF costs 4.0 % at both qualities). Reference patch 16 x 8 unison: 5.2 % against 5.1 % on `main` in the same minutes (the digital envelope now goes through the wrapper).
* **Not done**: the voice card does not reach the effects or the wavetable oscillators' unison (only a per-voice pitch offset); noise and hum are one stereo source for the whole engine (not per voice); the sustain stage of a one-shot RC envelope
  releases after the level has settled within 1 % of the sustain (about 4.6 time constants); `filter.svf` (legacy models) takes no tolerance.

## 15. Modulator bank, modulation of modulation, audio-rate bus and filter FM (P12: `tests/ModulatorTests.cpp`, `tests/AudioRateModTests.cpp`, `crucible_engine_bench mod`)

The modules `mod.bank` and `mod.audio_rate` are registered last (232 parameters appended, frozen with the others in `tests/golden/param_order.txt`); every default leaves the voice as it was, so the goldens and bit-exact
fingerprints are unchanged. Plan rows 2.10 / 3.1, measured on Apple M-class, 48 kHz:

| quantity | plan | measured |
| --- | --- | --- |
| Combine (add, multiply, min, max, crossfade, difference) | exact | exact to 1e-6 for every mode; polarity follows the inputs |
| Remap, 16-point curve | exact | at all 16 points and 45 points between them within 2e-6, for unipolar and bipolar inputs |
| Lag, rise and fall time constants | within 2 % | 63.2 % at one time constant (rise 40 ms, fall 200 ms) within the 2 % bound, 1 - e^-5 after five time constants within 1e-3; the chunk-wise one-pole is exact at chunk boundaries |
| Follower | - | attack follows 1 - e^(-t/5 ms) within 0.3 %, release to 1/e in one time constant within 2 %, mean of \|sin\| within 1 % |
| Quantise (nearest, down, up) | correct | all steps hit and monotone, bipolar inputs quantised across -1 .. 1 |
| S&H | correct | holds between clocks, takes the input at the clock (rate: 20 to 22 changes in 2 s at 10 Hz; input edge; note-on), seeded noise reproducible |
| Turing | reproducible | same seed, same sequence; a locked register repeats with its length (4, 8, 16); full randomness does not repeat |
| Binary | - | only 0 and 1, frequency of ones within 3 % of the probability (0.1, 0.5, 0.9) |
| Chaos (logistic, Lorenz) | bounded | -1 .. 1 over 300,000 chunks (logistic at r = 4 too), variance 0.02+, repeats per seed, a seed changes the orbit |
| Note gate | per voice | gate, velocity and 10 ms trigger; two voices, two gates |
| Pitch tracker | - | sine and saw at 110, 220, 330, 880 Hz within 0.03 note (3 cents) |
| LFO table | - | position 0 is a sine within 1.5e-3 of the ideal; the morph is the average of its frames |
| Evaluation order, cycles | cycles rejected | an input naming a later modulator reads as none; `ModRouteStore::add()` refuses a route into its own amount and a cycle of amount routes |
| Modulation of modulation, depth | correct | cutoff route amount 0.4 + 0.3 x macro: 2^(5 x 0.7) within 0.2 %; follows a macro move; three deep (0.4 + (0.3 - 0.5) x 0.5 = 0.3) within 0.3 % |
| Audio-rate LFO aliasing (saw, square, triangle, saw down, 25 % pulse; 440 Hz .. 12.3 kHz, non-harmonic power below 20 kHz) | below -90 dB | worst -91.7 dB (saw at 8.2 kHz); -110 dB or better up to 5.3 kHz, triangle -96 dB or better; fundamentals exact to 0.05 dB |
| Ladder filter FM sidebands against an offline double-precision model (4 trapezoidal stages at 2x the rate, cutoff per sample) | within 0.5 dB | 0.13 dB for depths up to 1 octave (sidebands down to -60 dB), 0.59 dB at 2 octaves (one sideband); through the voice (batched or voice by voice) 0.11 dB; through a graph product 0.11 dB |
| FilterBank quad path with FM | equals one lane | four lanes against one lane, modulated and unmodulated: 7e-8; batched against voice by voice in the engine 1.1e-8 at a peak of 0.0185 |
| Audio-rate FM stability | stable, band limited | all eight analog models at 3 octaves of depth and 2.3 kHz modulation, resonance 0.9, drive 0.3: finite, peak below 8 |
| PWM from another slot (the gap P9 left) | alias <= -90 dB | 15 sideband lines within 0.04 dB of the exact Fourier integral of the pulse train; aliasing -92.4 dB (modulator at 3x the carrier); the static 440 Hz pulse is -119.6 dB (it was -22 dB, see `SOUND_CHANGES.md`) |
| Pitch FM of an `osc.analog` slot by another slot's output | alias <= -90 dB | sine: lines within 0.14 dB of numerical FM; saw: aliasing -112.3 dB |
| Amplitude modulation (`voice:amp`) | exact | sidebands at 0.25 of the carrier within 0.1 dB |
| Wavetable oscillator, pitch and position in groups of four samples | - | pitch-FM sidebands within 0.45 dB |
| Real time | zero allocations | 0 allocations in 48,000 samples at 128 per block with four modulators, audio-rate routes, filter FM and two audio-rate LFOs |

The batched filter path stays engaged: with audio-rate cutoff modulation the bank and voice-by-voice renders differ bit for bit and agree within 1e-5 of the peak. `AnalogFilter::process(io, n, cutoffOctMod)` samples the cutoff
every four samples while it is modulated (it was every sixteen); the other filter models re-derive their coefficients every four samples.

* **CPU** (`crucible_engine_bench mod`, process CPU time; the plan's analog voice, 16 voices of two `osc.analog` slots, transistor ladder, analog RC envelopes and cards; Standard, best of three runs of 4 s):

| configuration | % of one core | added |
| --- | --- | --- |
| baseline, the bank present and every modulator off | 7.33 | - |
| 1 modulator (Lag of LFO 1) routed to the cutoff | 7.49 | +0.15 |
| 2 (+ Combine) / 4 (+ S&H, Turing) | 7.46 / 7.51 | +0.12 / +0.17 |
| 6 (+ Chaos (Lorenz), Follower) / 8 (+ LFO table, Pitch tracker) | 7.79 / 7.86 | +0.46 / +0.45 |
| audio-rate LFO (saw) to the ladder cutoff, per sample, batched | 8.94 | +1.53 |
| oscillator 1 output to the pulse width of oscillator 2, per sample | 9.92 | +2.51 |
| oscillator 1 output to the pitch of oscillator 2, per sample | 13.0 | +5.7 |
| audio-rate LFO to the amplifier, per sample | 7.96 | +0.62 |

  A modulator that no route reads is not evaluated, so an idle bank costs nothing per voice; the fixed cost of the whole package on a default voice (the extra branches in `Voice::beginChunk`, a larger `Voice`) was
  measured against the commit before P12 with the same patches in the same minutes (user CPU, best of five): 16 voices of the plain wavetable voice +0.0 %, the same with the transistor ladder -0.4 % (noise), the plan's analog voice +0.5 %,
  two `osc.analog` slots with the filter off (the cheapest voice) +1.9 %. An idle engine got faster, 1.4 against 1.8 microseconds per control chunk, because the smoothing step now visits only the smoothed parameters (it visited all of them,
  and P12 adds 232). The 16 x 8 unison reference row is unchanged (5.5 to 5.7 % in both builds).
* **Not done**: the layer scope is the engine's single voice pool (graph layers do not get an instance of their own, and graph nodes cannot read the bank); the parameters of a modulator are not modulation targets (that would let a cycle through the bank in; mod of mod is by route
  amounts); Follower and Pitch tracker listen to the voice's oscillator mix (a one-chunk delay; the shared scopes to the engine's voice sum) rather than to a chosen bus; the amp envelope is not an audio-rate source (it ticks in the output stage); the wavetable
  oscillator takes audio-rate pitch, cross amount and position in groups of four samples (a 12 kHz control rate), not per sample, and the other oscillator modules take the chunk mean; audio-rate cutoff reaches the nonlinear models at 12 kHz sampling (0.59 dB
  at two octaves of depth for one sideband, 0.13 dB up to one octave); audio-rate FM by pitch is an exponential law (24 semitones per unit of signal), the linear FM
  of `osc.analog` is its own `an_xmod_*` with an audio-rate amount; `AudioLfo` shapes beyond the six listed play as sine.

## 16. Algorithmic reverbs (FX6: `fx.reverb_algo`; `tests/ReverbAlgoTests.cpp`)

Room, Hall, Chamber, Plate and Ambience on the cores of `fx/Fdn.h`, 48 kHz, Release, Apple M-class, 2026-10-12. Impulse responses are taken
with the same impulse on both channels, mix 100 %, pre-delay 0, wet cuts off and both decay multipliers 1 ("flat"), unless a row says
otherwise; the reverberation time is T30 (least-squares line through the Schroeder curve from -5 to -35 dB, extrapolated to 60 dB) of the
zero-phase octave-band-filtered response (`FxMeasure.h`). Run: `build/crucible_engine_tests "Algorithmic reverb"`.

### 15.1 Plan rows (section 3.5)

| quantity | plan | measured |
| --- | --- | --- |
| RT60 per octave band, 125 Hz to 8 kHz, set 2 s (Ambience 0.8 s), 16 lines | within 10 % | worst band error: Room 3.0 %, Hall 4.8 %, Chamber 3.2 %, Plate 6.6 %, Ambience 3.1 % |
| the same with 32 lines (High quality) | within 10 % | Room 2.6 %, Hall 5.8 %, Chamber 2.5 %, Ambience 8.7 % |
| RT60 at set 0.3 s and 10 s (Hall, Plate); size 0 and 1 at 2 s | within 10 % | 8.6 % worst for the decay sweep, 9.6 % worst for size (Plate at size 0, 250 Hz band); Hall within 7.3 % at 0.3 s and 3.0 % at 10 s, Plate within 8.6 % and 4.1 % |
| RT60 at 44.1, 96 and 192 kHz (Hall, Plate, 2 s) | within 10 % | 6.6 % worst (the plate) |
| low / high decay multipliers (1.6x below 400 Hz, 0.35x above 3 kHz, set 3 s): each band against the shelf design | - | Hall 2.4 %, Chamber 3.9 %, Plate 6.7 % worst band; 125 Hz band 4.57 s (design 4.53), 8 kHz band 1.13 s (1.13) |
| energy decay curve against its line, -5 to -35 dB | within 1 dB | Room 0.22, Hall 0.34, Chamber 0.20, Plate 0.39, Ambience 0.25 dB |
| normalised echo density (Abel and Huang, 20 ms window) first at least 0.95 | Hall 80 ms, Plate 20 ms | Room 24.3, Hall 62.7, Chamber 36.0, Plate 11.2, Ambience 29.1 ms; first reading of at least 1.00: 24.9, 67.7, 47.6, 24.6 (Plate), 38.4 ms |
| late-tail 1/3-octave levels, 100 Hz to 10 kHz, from 0.3 s, standard deviation (L / R) | at most 2 dB | Room 0.69 / 1.35, Hall 0.64 / 0.78, Chamber 0.56 / 0.57, Plate 1.24 / 0.69, Ambience 1.58 / 1.44 dB |
| interaural cross-correlation, first 80 ms, +-1 ms, width 1 | at most 0.3 | Room 0.069, Hall 0.066, Chamber 0.081, Plate 0.065, Ambience 0.099 (width 0.5: 0.60; width 0: 1.000 and L equals R) |
| mono sum (L + R) / 2 against the mean channel energy per octave band | -3 dB +-3 | between -5.60 and -1.90 dB (independent channels give -3.01) |
| pitch spread of a 1 kHz sine at the default modulation depth (power-weighted spectral spread, cents RMS) | at most 3 | Room 2.44, Hall 1.06, Chamber 0.65, Plate 0.61, Ambience 1.76 (floor of the measurement 0.37); at depth 1: 6.63, 2.15, 1.77, 3.74, 4.93 |
| freeze for 60 s: level over five 10 s windows (both channels) | within 0.5 dB, bounded | spread 0.41 / 0.23 / 0.34 / 0.16 / 0.30 dB (16 lines), 0.32 / 0.26 / 0.38 / 0.32 dB (32 lines); peak 2.25 at most; 1.5 s after release the level is 35 to 43 dB down |
| silence in | exact zero or below -150 dBFS | below -340 dBFS (a 1e-18 offset keeps the loops out of the denormal range); no subnormal output sample in 40 s of tail; silent block time 0.87 of live |
| CPU per stereo instance, noise in, blocks of 32 (16 lines) | 1.0 % | Room 0.49, Hall 0.66, Chamber 0.64, Plate 0.40, Ambience 0.47 % |
| the same with 32 lines | 2.0 % | Room 0.79, Hall 1.05, Chamber 1.02, Ambience 0.78 % |

### 15.2 Other measurements

| quantity | measured |
| --- | --- |
| sinc kernel (Kaiser beta 6.5, 8 taps, 512 phases, no interpolation between phases) | largest magnitude of any phase 0.0000 dB, smallest DC gain -0.006 dB; worst-phase droop -0.015 dB at 0.1 fs, -0.006 at 0.2 fs, -0.31 at 0.3 fs |
| absorption design: decay time implied by the filter at DC and Nyquist against the target (lows 1.6x, highs 0.4x, 5 to 120 ms loops, 0.4 to 20 s) | 0.29 % worst |
| algorithm change (20 ordered pairs, 997 Hz sine at mix 50 %): largest step; wet level 150 to 350 ms after the change with silent input | -68.8 dBFS; below -300 dB (the old tail is gone) |
| pre-delay 10 / 50 / 200 ms: first wet sample above -80 dB | within 0.02 ms of the setting (the sinc read may lead by up to four samples, never before 0.1 ms) |
| wet low cut 500 Hz / high cut 2 kHz | -14.9 dB at 50-125 Hz and -13.6 dB at 6-12 kHz; -0.13 and -0.26 dB in the pass band |
| ducking at 100 % with a loud burst | wet level -30.0 dB during the burst, -0.3 dB 1 s after it (release 200 ms) |
| state of a 32-line reverb | 719160 samples, cleared in 11 `deactivate()` calls of at most 65536; switching on after two seconds off gives silence (-365 dBFS) |
| 12 s of random parameter extremes, blocks of 1, 7, 32, 64, 500 | finite, peak at most 7.4 |
| allocations on the audio thread: all 21 parameters changed every 64-sample block, algorithm switches, freeze, 16 and 32 lines | 0 |
| `crucible-module-check --builtin` | pass, 0.64 % of one core |

Notes. The plan's "reaches 1" for the echo density is read as 0.95: the estimator of a Gaussian tail scatters by about 0.05 (one sigma, 20 ms
window), so the first reading of at least 1.00 falls up to 15 ms later (columns above). Hall reaches 0.95 at 63 ms with the default 0.7
density; the dense onset is the output of the input diffuser (8 all-passes) added to the wet signal. The plate's round trip is 0.36 s at the nominal
size (half of Dattorro's published lengths) and is shortened automatically to a fifth of the decay time, which keeps the stepwise energy
decay of a recirculating loop within the 10 % band; with the published lengths and a 2 s decay the error was 7 to 17 %. The default `fx_ra_himult`
of 0.7 tilts the tail on purpose: the flatness figures are taken with both multipliers at 1. The power-weighted spectral spread is the width of
the line a modulated reverb makes of a steady sine; it is dominated by Room and Ambience because their lines are short and recirculate often,
so the excursion of every line is proportional to its own length.

## 17. Convolution v2 (FX7: `fx.convolution`; `tests/PartitionedConvolverTests.cpp`, `tests/IrImportTests.cpp`, `tests/ConvolutionTests.cpp`)

`NonUniformConvolver` (`fx/PartitionedConvolver.h`), IR import (`fx/IrImport.h`) and the module, 48 kHz, Release, Apple M-class, 2026-10-15,
unless a row says otherwise. The CPU rows are the minimum of five passes over a 6 s stereo run with a 4 s stereo response and 128-sample
blocks, one core, and are machine dependent. Run: `build/crucible_engine_tests "Non-uniform partitioned"`, `build/crucible_engine_tests
"Impulse response import"`, `build/crucible_engine_tests Convolution`.

### 17.1 Plan rows (section 3.5)

| quantity | plan | measured |
| --- | --- | --- |
| output against a double-precision direct convolution, every partition plan (zero-latency and low-CPU), 7 IR lengths from 1 to 20000 samples, 7 host block sizes | -120 dB | worst -125.6 dB in the test grid (-128.2 dB before the partition split for large FFTs) |
| `fx.convolution` against direct convolution, zero-latency and low-CPU | -120 dB | -133.7 dB |
| the same at 44.1, 48, 96 and 192 kHz, both modes | -120 dB (common acceptance: -100 dB) | below -100 dB in every case |
| resampled IR: tone error against the analytic tone, 12 rate pairs | - | at most -137.6 dB |
| resampled IR: stopband leakage | -120 dB | at most -144.7 dB (140 dB Kaiser sinc) |
| CPU per stereo instance, low-CPU mode, three runs | 0.6 % | 0.557, 0.576, 0.557 % (0.55 % in the step 3 run) |
| CPU per stereo instance, zero-latency mode, three runs | 1.2 % | 0.792, 0.792, 0.790 % |
| per-block CPU spread (max / mean), low-CPU, three runs | at most 2 | 1.78, 1.84, 1.79 |
| per-block CPU spread (max / mean), zero-latency, three runs | at most 2 | 1.80, 1.80, 1.79 |
| CPU of the removed first-generation `fx.conv` with a 4 s IR (section 8.3) | - | 4.35 % (the target is 0.6 % for the low-CPU mode, 1.2 % for zero latency) |
| latency, reported against measured: zero-latency mode | 0 samples | 0 samples |
| latency, reported against measured: low-CPU mode | reported | 128 samples, equal to the reported value |
| true-stereo LL / LR / RL / RR matrix | verified by channel impulses | pass (`ConvolutionTests`) |
| kernel switch while playing (largest step above the steady level) | at most -60 dB | -114 dB |

### 17.2 Common acceptance (`tests/ConvolutionTests.cpp`)

| quantity | measured |
| --- | --- |
| silence in, with and without a response | exact zero out |
| 60 s of random parameter extremes, Standard and High | finite; peak 0.56 and 0.50 on unit-L1 kernels (bound 256) |
| unity small-signal gain, both modes | 0.000 dB |
| zipper: 50 ms ramp and 2 Hz square on each of the 12 continuous parameters | all at or below -96 dB (worst: mix square -96.5 dB, output square -96.4 dB, pre-delay -105 dB) |
| click: mode change, reverse, replaced and cleared response, off | at or below -132 dB |
| offline render, three repeats with random builder timing | bit-exact |
| audio-thread allocations: parameter changes every 64 samples, response load and clear | 0 |
| feature audit (`tests/FeatureAuditTests.cpp`): every parameter at extremes and automated | 1458 parameters, none unresponsive, 0 failed (`docs/FEATURE-AUDIT.md`) |

Notes. The partition plans are a 64-sample direct head with 32 / 128 / 1024 / 8192-sample partitions (zero-latency mode) and 64 / 512 / 4096-sample
partitions behind a 128-sample delay (low-CPU mode). Large partitions are computed incrementally across blocks, which is what keeps the per-block
spread near 1.8 rather than the spikes of a single large FFT. The response is resampled offline on the control thread, and an edit or a new response
is built on the instance's builder thread; at High quality the audio thread waits for the build, as `fx.eq_pro` does, so offline renders stay
deterministic.

## 18. Gap tuning (G3, `feat/gap-tuning`)

Measured on the development Mac (Apple M-class) under other applications, so the load average was 6 to 18 during
the runs. CPU figures are thread CPU time (`CLOCK_THREAD_CPUTIME_ID`) with the measured thread at user-interactive QoS, so
they do not move with the load. Wall time is used only for the spread checks.

- Convolver CPU test (`tests/PartitionedConvolverTests.cpp`, helpers in `tests/CpuTiming.h`): low-CPU 0.56 to 0.60 % before, 0.57 %
  after (limit 0.6 %); zero-latency 0.80 to 0.82 % before and after (limit 1.2 %). The low-CPU limit sits close to the intrinsic cost
  on this host; the limits are unchanged.
- `crucible-measure` (`tools/crucible-measure/main.cpp`): the CPU window covers only the `Engine::process` calls (not the per-sample
  collection work), thread CPU time, user-interactive QoS. `cpu_1voice` (all threads) 0.585 to 0.587 % before, 0.578 to 0.587 % after.
- Analog filters in graph nodes and the classic voice's non-bank fallback run the block path for analog models (bit-identical at
  constant controls; the maximum difference over all eight models at Standard and High is 0). Per stereo filter:
  Standard 0.52 to 1.16 % becomes 0.20 to 0.92 %; High 1.37 to 4.19 % becomes 0.48 to 3.51 %.
- Graph quad batching (`crucible_engine_bench graphfilter`): voice pairs share a four-lane `LadderBase` filter. Per stereo voice:

| graph filter model | single (before) | blockwise (step 3) | quad batched |
| --- | --- | --- | --- |
| transistor ladder | 1.13 % | 0.82 % | 0.28 % |
| diode ladder | 1.31 % | 1.0 % | 0.33 % |
| Sallen-Key LP | 0.92 % | 0.47 % | 0.17 % |

  Batched against single-lane output: maximum difference 5e-7 (`VoiceQuadTests`, 16 cases, plus an allocation test with 0 audio-thread
  allocations).
- Profile of "1 note, saw" (macOS `sample`, shares of `Engine::process`): voices 36 %; parameter smoothing (`stepSmoothers`) 20 %;
  master analyzer (K-weighting, true peak) 19 %; per-block parameter copy 12 %; module rack 4 %. The per-voice work is a minority:
  the one-voice row is mostly per-block and per-cell overhead.
- Convolver, low-CPU mode, standalone 4 s stereo 128-sample loop: multiply-accumulate 66 %, FFT (vDSP) about 12 %, `process()` 7 %,
  copies 5 %. The multiply-accumulate streams the kernel spectra once per block, so it is memory-bound. Keeping two partitions per
  pass in registers was bit-identical and showed no gain (2.84 s against 2.84 s), so it was reverted.
- Open for v1 (not done in G3): event-driven smoothing (skip idle smoothers; keeps the `overlayExact` and `setParameterValues`
  interplay), dirty tracking in `setParameterValues`, a master analyzer disable switch for benchmarks.

## 19. Fractional delay, BBD core and fx.delay_pro (FX10: `FractionalDelay.h`, `Bbd.h`, `fx.delay_pro`; `tests/DelayProTests.cpp`)

Kaiser-windowed sinc reads (`FractionalDelay`) and a bucket-brigade core (`Bbd`) behind the digital and analog modes of `fx.delay_pro`.
The module is registered last; the first-generation `fx.delay` was removed before v1. Measured with the same thread-CPU helpers as section 18
(`tests/CpuTiming.h`).

### 19.1 Plan rows (section 3.5 of `docs/plans/ENGINE-FX-PLAN.md`)

- Fractional read: phase-delay error 0.0006 sample at 32 taps; magnitude flat within 0.0052 / 0.0039 / 0.0028 dB to 18 kHz at 44.1 /
  48 / 96 kHz over 64 fractional positions; 0.003 dB while modulated. 16 taps: 0.028 dB to 16 kHz; 64 taps: 0.0012 dB to 19.8 kHz.
  CPU per read 14.3 ns (32 taps), 7.4 ns (16), 30.8 ns (64).
- BBD core: -3 dB bandwidth within 0.1 / 0.8 / 3.1 % of 0.30 x clock (8 / 20 / 40 kHz clock; 4.2 % at 96 kHz with 4096 stages).
  Images at 0.1 / 0.2 x clock below -77.9 / -68.9 dB (worst of lower and upper image, 20 and 40 kHz clock, 96 kHz rate).
  Full-scale square at +24 dB drive peaks at 3.99 (limiter bound 4). Zero in gives exact zero out; tail below 1e-7.
  CPU 0.21 % per channel at the highest clock (45.6 kHz), 0.16 % at 10 kHz; stereo about 0.42 % against a 0.6 % module budget.
- `fx.delay_pro`, digital: delay error 0.0004 sample; 8 / 16 / 18 kHz within 0.005 dB of 1 kHz (24 taps at 50 kHz or below, 16 above);
  flat within 0.025 dB to 18 kHz at 44.1 / 48 / 96 kHz; latency 0.
- `fx.delay_pro`, feedback: 0.99 feedback for 60 s stays bounded in every stereo mode (peak 2.3 digital, 0.52 analog).
- Ducking: depth error below 0.001 dB.
- Time change: crossfade step -69 dBFS; glide -62 dBFS; analog -60.5 dBFS; engine switch -65 dBFS (all below the -60 dB limit).
- Analog: echo time within 0.7 % at 300 / 100 / 40 ms.
- Freeze: digital -0.02 dB over 30 s. Analog sinks 3 dB over the same span (soft-clip loss per pass, documented in the module).

### 19.2 CPU (48 kHz, stereo, quiet machine)

| mode | all features | defaults |
| --- | --- | --- |
| Digital | 0.28 % | 0.18 % |
| Analog (1024 stages, 12 ms; 256 stages, 3 ms) | 0.51 to 0.55 % | 0.31 % |

Budgets: 0.3 % digital and 0.6 % BBD (`CpuTiming.h`). The analog figure is measured with the machine quiet; the CPU test uses thread
CPU time, so it does not move with the load.

### 19.3 Design notes

- Analog runs in 16-sample chunks and its feedback lags one chunk; the delay is shortened by one chunk to compensate.
- Engine or stage change: 20 ms fade out, clear, then a smoothstep input ramp (no allocation on the audio thread; the polyphase table
  is built at prepare).
- Glide: the time moves at a rate limit of 0.5 followed by a one-pole cascade.

## 20. Transient shaper and de-esser (FX13: `fx.transient`, `fx.deesser`; `tests/TransientDeEsserTests.cpp`)

Two current dynamics modules, both registered last (`fx.transient`, then `fx.deesser`). Neither changes an existing parameter id or
golden. The de-esser's split band is a 97-tap Kaiser linear-phase FIR at 48 kHz rather than the complementary crossover in the plan:
the minimum-phase band came out of phase with the input, so the realised reduction depended strongly on frequency (1 dB at 5 kHz
against 12 dB at 8 kHz for a 12 dB setting). The FIR keeps the band in phase, so an un-reduced output is the delayed input exactly.
Measured with the thread-CPU helpers (`tests/CpuTiming.h`).

### 20.1 Plan rows (section 3.5 of `docs/plans/ENGINE-FX-PLAN.md`)

- `fx.transient`, level independence: the same gain curve at -40 and -6 dBFS input differs by 0.0000 dB.
- `fx.transient`, hit: peak change equals the attack setting within 0.03 dB (noise burst with exponential decay).
- `fx.transient`, steady tone: a sine changes by 0.0000 dB; THD -156 dB at 100 Hz, 1 kHz and 5 kHz.
- `fx.transient`, cuts: attack cuts last only while the fast follower is 1 dB above the slow one, so on decays slower than about 40 ms
  the later part of a burst is cut less than its onset (tested with tau 10 and 20 ms). Boosts hold at 80 ms. Sustain reaches its
  full setting when the fast follower is 6 dB under the slow one.
- `fx.deesser`, no reduction: output nulls against the delayed input (-inf dB) at 0, 1 and 2 ms lookahead.
- `fx.deesser`, outside the band: 250, 500 and 1000 Hz tones with 18 dB of reduction available change by at most 0.003 dB.
- `fx.deesser`, reduction: on 1/3-octave noise bursts over a vowel-like tone, reduction tracks the static curve within 0.2 dB
  (output band level and meter agree).
- `fx.deesser`, threshold: the relative threshold gives the same level (11.169 dB) at two input levels.
- `fx.deesser`, band shape: edges are about 1.4 kHz wide transitions, 6 dB down at the set edges. Latency is the FIR half length
  (1 ms) plus the lookahead (0 to 2 ms, default 1 ms).
- Allocations on the audio thread: 0 for both modules.

### 20.2 CPU (48 kHz, stereo, quiet machine)

| module | measured | plan |
| --- | --- | --- |
| `fx.transient` | 0.09 % | 0.1 % |
| `fx.deesser` | 0.18 % | 0.2 % |

The de-esser runs its gain computer every fourth sample and its detector power over 2 ms. The transient curve is evaluated every
second sample. Latency: `fx.transient` up to 5 ms (lookahead, reported through `crucible_latency_samples`); `fx.deesser` 1 ms plus
0 to 2 ms lookahead.

## 21. Modulation effects (FX11: `fx.ensemble`, `fx.phaser2`, `fx.flanger2`; `tests/ModulationFxTests.cpp`)

Three modulation effects on `FractionalDelay.h` and `Bbd.h` (section 19). They are registered after `fx.deesser`, so no earlier
parameter moves; the first-generation `fx.chorus`, `fx.dimension`, `fx.phaser` and `fx.flanger` were removed before v1.

### 21.1 Plan rows (section 3.6 of `docs/plans/ENGINE-FX-PLAN.md`)

- `fx.ensemble` (14 parameters): modulation depth within 1.29 % of the set cents (sine, 4 to 20 cents, 0.5 to 3 Hz; triangle and
  Dimensional I to IV within 1.5 %); high-frequency -3 dB within 1.3 % of 0.30 x clock at 256 / 512 / 1024 stages; mono sum -0.13 dB;
  no zipper, every change (stages included) at or below -67 dBFS; 0 allocations on the audio thread.
- `fx.phaser2` (18 parameters): 45 notches above 100 Hz (2 to 12 stages) within 0.42 % of the analytic all-pass prediction, 104 dB deep
  at the predicted frequencies (mix 0.5, no feedback); sweep range within 0.69 %; OTA odd and JFET even harmonics; small signal equals
  the linear stage within 0.007 dB; feedback of +-0.95 bounded (peak 17.9 on 0.25 rms noise, DC-blocked); envelope and sidechain sweep
  move the notch; barber-pole direction and periodicity measured; no zipper; 0 allocations.
- `fx.flanger2` (16 parameters): comb notches within 0.013 % over 86 notches (delay offset -4.8 to +4.8 ms, either sign), the first
  four of each at least 58.9 dB deep; unity at zero offset; crossing step -73 dBFS; feedback of +-0.95 bounded (peak 3.4); BBD mode
  notch within 0.22 %; no zipper (mode, source and shape changes fade through a hold); 0 allocations.

### 21.2 CPU (48 kHz, stereo, best of 4, three repeats, loaded machine; % of one core)

| mode | measured | plan |
| --- | --- | --- |
| `fx.ensemble` defaults (3 lines, 256 stages, 8 ms) | 0.31 | 0.5 |
| `fx.phaser2` linear, 4 / 6 + feedback / 12 + feedback stages | 0.08 / 0.09 / 0.13 | 0.3 |
| `fx.phaser2` barber-pole, 8 stages | 0.15 to 0.17 | 0.3 |
| `fx.phaser2` OTA, 4 stages (JFET is the same code) | 0.30 | 0.3 |
| `fx.flanger2` digital / envelope | 0.08 to 0.09 / 0.10 to 0.11 | 0.3 |
| `fx.flanger2` BBD, 256 stages | 0.28 | information |

The OTA row sits on the budget (0.297 best, 0.314 worst under load): the feedback loop is latency bound, one division per stage.
Options are a 0.35 % budget for OTA / JFET, or a cheaper stage nonlinearity (changes the harmonic balance).

### 21.3 `fx.flanger2` latency: 249 samples at 48 kHz in every mode

The through-zero offset crosses zero and goes negative (down to -5 ms), so the modulated path is read at R + offset with
R = ceil(5 ms) + 8 (half the 16-tap sinc) + 1 = 249 samples, and the dry path is delayed by the same R to stay aligned. The figure is
a constant: a host cannot renegotiate latency when the mode or the sign of the offset changes, so the BBD mode reports it too
although its device delay is separate. A non-through-zero setting (offset at or above zero only) could report 0, with the modulated
path at its minimum read delay of 7 samples (0.15 ms, first notch at most 3.4 kHz), which is normal for a classic flanger. That
needs a new switch, which the frozen parameter order does not have, and switching it would change the reported latency while running.

### 21.4 Bbd reconstruction filter

`Bbd`'s reconstruction filter (8th-order Butterworth on the held staircase) was a 20-tick FIR. It is now the filter's four complex
modes (see `docs/SOUND_CHANGES.md`). Old against new: rms difference -78 dB; `fx.delay_pro` bandwidth within 0.1 % of before, clock
images -70 to -102 dB (were -69 to -100); all section 19 rows pass. `fx.delay_pro` Analog with all features 0.511 to 0.352 %, BBD
delay of 10 ms 0.198 to 0.128 % per channel.

## 22. Harmonic exciter (FX13: `fx.exciter`; `tests/ExciterTests.cpp`)

One current module, registered last, so no earlier parameter id or golden moves. Nine parameters: on, mode (High / Low), exciter
frequency (1 to 10 kHz), bass frequency (40 to 250 Hz), drive (0 to 9 dB), asymmetry, amount, listen and chain slot. Measured with
the thread-CPU helpers (`tests/CpuTiming.h`); the suite has 144 checks, all passing.

### 22.1 Plan rows (section 3.4 of `docs/plans/ENGINE-FX-PLAN.md`, `fx.exciter`: below 0.5 dB, the documented harmonic law, alias at most -90 dB)

- Below the frequency the output changes by at most 0.5 dB: High 0.084 dB, Low 0.319 dB (below half the frequency).
- Harmonic law (rail-curve Fourier series x filters x ADAA droop): worst error 0.082 dB High, 0.075 dB asymmetry, 0.001 dB Low
  (limit 0.3 dB).
- Symmetric curve: even harmonics at -171 dB.
- Alias, worst case: -126.2 dB in Standard and High (test limits -90 and -100 dB; the plan's limit is -90 dB). The test set is 5 and 7 kHz tones at -12 and -6 dBFS,
  at maximum drive, asymmetry, amount, and frequencies of 1 and 3.5 kHz. The guarantee covers -6 dBFS tones; see 22.3.
- Amount 0: the output is the delayed input exactly (null).
- Latency: 0 / 23 / 47 samples (Draft / Standard / High), reported through `crucible_latency_samples`.
- Allocations on the audio thread: 0.

### 22.2 CPU (48 kHz, stereo, load 7, best of 8, quiet machine)

| mode | measured | plan |
| --- | --- | --- |
| `fx.exciter` Standard | 0.334 % | 0.3 % |
| `fx.exciter` High | 0.446 % | not a plan row |

Standard is 11 % over the plan row, inside the bench's +20 % margin. Table 3.10 of the plan does not list `fx.exciter`; its CPU
target is the accept row in section 3.4.

### 22.3 Deviations from the plan

- Saturator: `Model::Rail`, a cubic (x - x^3 / 6.75 inside |x| < 1.5, rail at 1) instead of tanh. Tanh with ADAA costs 24 ns per
  sample (0.45 % at 2x stereo on its own) and aliases at -81 dB at 9 dB of drive. The rail model is a cubic inside its knee, so it
  does not alias there, and costs 5 ns per sample.
- Drive is 0 to 9 dB. Asymmetry is limited to an offset of 0.1. A larger offset or a hot input pushes the
  signal into the knee and raises alias: at -6 dBFS with 12 dB of drive the tones measure -68 dB. At 0 dBFS with 9 dB of drive the
  alias figure is outside the guarantee.
- Oversampling is 2x for Standard and High (High uses its longer filters) and 1x for Draft. Table 3.9 specifies ADAA at 4x for High.
  2x gives -126 dB alias, and 4x costs 0.98 % CPU.
- Filters: Butterworth BiquadChain sections (pre 12 dB per octave, post 36 dB per octave), not `fx::Crossover` (0.43 % for two of
  them). The corner moves at most 12 % per 128-sample block: steep filters ramped across large jumps reached a peak of 12.8.


## 23. Review fixes, waves G-H effects (slice B: `fix/review-gh-fx`)

- phaser2 JFET / OTA stage: the stage output was `a - 2z` (z the resistor voltage) instead of `a - 2y` with `y = tanh(kz)/k` the
  saturated current, so in saturation a single JFET stage had a steady gain of 2.8 (OTA 1.57). With `a - 2y` the stage is passive:
  single-stage gain <= 1.000 over a grid for both types. The sweep-matrix bound is 1.05 x input peak / (1 - |fb|); worst case 4.93
  (analytic ideal all-pass loop about 5.25). Before the fix the peak was up to 50 (JFET) and 11 (OTA); barber-pole matrix had 25
  failures including Inf at 2 stages, fb 0.95.
- delay_pro freeze (10 min of audio, 8 cases): before, Digital short (40 ms, depth 6 ms, 3 Hz, glides) rose +12 dB (peak 2.8 for a
  0.7 input) and medium cases drifted +0.11 dB per 30 s. The guard (window level vs the level before the freeze, 0.01 dB trim per
  window above +1 dB, floor -3 dB) holds the short case at +1.3 dB (peak 0.68); Digital steady -0.3 dB over 10 min; the analog frozen
  loop only decays (-20 dB in 10 min).
- Zipper (second-difference estimator, ramps and 2 Hz squares at blocks 1 / 7 / 512): exciter <= -103 dB, transient output
  <= -101 dB, deesser <= -99 dB. Ensemble (-85 dB) and delay_pro analog (-54 dB at +24 dB drive on a -6 dBFS sine; -85 dB at 0 dB
  drive) sit at the engines' held-setting floor (BBD clock ripple, drive saturation), so their tests allow max(-90, held floor + 3 dB).
  Block-size match of a jump (blocks 1 / 7 / 512): exciter and transient exact, delay_pro drive 3e-5, deesser 6e-4, ensemble spread
  and mix exact, crossfeed 2.3e-3.
- Per-rate results: every new effect passes at 44.1 / 48 / 96 / 192 kHz. flanger2 reported latency equals measured: 230 (44.1),
  249 (48), 489 (96), 969 (192) samples.
- Merge with the Windows CPU work (batched de-esser FIR, delay_pro early read): both kept; the batched FIR falls back to the
  single-sample path while the taps are moving to a new design (about 3 ms after a change), so the crossfade stays exact.
## 24. Saturator (FX13: `fx.saturator`; `tests/ColourDynamicsTests.cpp`, suite "Saturator")

Ten models through the module rack, 48 kHz, 5 kHz and 7 kHz tones at -6 dBFS, alias level relative to the fundamental. Standard
oversamples 4x and moves to 8x as the drive rises, High 8x to 16x; second-order ADAA where an antiderivative exists. CPU is
thread CPU time, stereo, 512-sample blocks, minimum of 5, load average about 10.

Alias, worst over the 6 to 36 dB drive sweep: Standard -92.1 dB (hard clip, 7 kHz), High -102.2 dB (hard clip, 5 kHz) for the nine
models other than tape (plan rows -90 / -100).

**Exception (maintainer decision:** the tape (hysteresis) model is held to -85 dB at Standard and -97 dB at High across the
whole drive range. The relaxation loop's own switch (a jump in the second derivative where the magnetisation crosses the
anhysteretic curve) aliases at low drive: -82.0 dB (Standard, 6 dB, 7 kHz) and -95.5 dB (High, 9 dB, 7 kHz) over the full sweep,
-87.2 / -99.4 dB on the test's drive grid. At 36 dB drive tape meets the rows (-123 / -97 Standard, -136 / -128 High). The test
asserts tape at -85 / -97 (`kTapeExceptionStandardDb`, `kTapeExceptionHighDb`) and every other model at -90 / -100.

| model | Std 5k@18 | 5k@36 | 7k@18 | 7k@36 | High 5k@18 | 5k@36 | 7k@18 | 7k@36 | CPU % Draft / Std / High @12 | @36 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Soft | -139 | -103 | -130 | -100 | -143 | -131 | -136 | -121 | 0.17 / 0.28 / 0.53 | 0.16 / 0.47 / 0.85 |
| Hard | -127 | -99 | -113 | -96 | -127 | -122 | -114 | -125 | 0.16 / 0.28 / 0.52 | 0.16 / 0.46 / 0.85 |
| Rail | -118 | -103 | -107 | -98 | -138 | -130 | -134 | -125 | 0.17 / 0.29 / 0.51 | 0.16 / 0.45 / 0.85 |
| Diode | -107 | -103 | -100 | -101 | -135 | -131 | -133 | -119 | 0.16 / 0.28 / 0.52 | 0.16 / 0.46 / 0.86 |
| Wavefolder | -116 | -103 | -116 | -94 | -116 | -117 | -118 | -110 | 0.15 / 0.26 / 0.48 | 0.15 / 0.42 / 0.78 |
| Triode | -111 | -108 | -102 | -101 | -132 | -130 | -132 | -132 | 0.18 / 0.32 / 0.60 | 0.18 / 0.58 / 1.08 |
| Push-pull | -116 | -112 | -103 | -107 | -142 | -113 | -132 | -107 | 0.17 / 0.30 / 0.53 | 0.16 / 0.49 / 0.48 |
| Pentode | -113 | -112 | -106 | -110 | -113 | -125 | -106 | -122 | 0.18 / 0.33 / 0.60 | 0.18 / 0.57 / 1.08 |
| Tape | -108 | -123 | -102 | -97 | -116 | -136 | -113 | -128 | 0.18 / 0.55 / 1.04 | 0.18 / 0.55 / 1.04 |
| Transformer | -143 | -143 | -142 | -142 | -143 | -144 | -141 | -140 | 0.19 / 0.33 / 0.61 | 0.18 / 0.33 / 0.60 |

Other figures: auto gain within 0.48 LU on pink noise (-18 dB RMS), 0.65 LU at -12 dB; null at mix 0 / drive 0 -151.9 dB; response
+-0.11 dB to 20 kHz at Standard, +-0.02 dB at High; latency 26 / 49 / 55 samples (Draft / Standard / High), same for all models.
Host block size 1, 7 and 512 render the same sound to -129.8 dB (all ten models, Standard and High, auto gain on): parameters, path
changes and ramps run on a 128-sample grid of the instance's own time, the loudness trim learns every 512 samples.
CPU limits asserted: 0.5 / 0.6 / 1.8 % at 36 dB (Draft / Standard / High), tape 0.4 / 0.6 / 1.5 %.

## 25. Multiband Drive (`fx.multiband_drive`; `tests/MultibandDriveTests.cpp`, suite "MultibandDrive")

Three bands (defaults 200 Hz and 3 kHz, LR4, minimum phase) on the saturator's oversampled path, 48 kHz, through the module rack.
Standard oversamples the low and mid bands 4x and the high band 8x, High 8x and 16x, Draft 2x for all; second-order ADAA where an
antiderivative exists. CPU is thread CPU time, stereo, 512-sample blocks, minimum of 3. There is no row in the plan for this module,
so the acceptance rows are the common ones of section 3 of `docs/plans/ENGINE-FX-PLAN.md` and the saturator's own alias rows.

| row | result |
| --- | --- |
| unity, drives at 0 dB, mix 1, 20 Hz to 20 kHz (19 tones, all qualities) | 0.0000 dB deviation (limit 0.1 dB) |
| phase coherence: output against the crossover's band sum delayed by the latency | -148.7 dB null; one sample off reads 3.0 dB |
| latency Draft / Standard / High | 26 / 49 / 55 samples, the same for every setting; the driven high band in its linear region measures 49.00 |
| isolation: 100 Hz at -12 dBFS, Hard Clip 24 dB | low band driven -9.3 dB THD; mid and high driven -147.4 dB; none -156.5 dB |
| solo low band: 60 Hz / 8 kHz at -12 dBFS | -12.1 dB / -143.4 dB |
| all bands bypassed against all at 0 dB | identical (largest difference 0) |
| mix 0 against the delayed input | -151.6 dB |
| silence in; after reset; after an off / on cycle (Tape, 30 dB) | below -140 dBFS (limit -140) |
| host block size 1, 7, 512 (all ten models, 18 dB, mix 0.7) | largest difference 9.5e-7 on a 0.4 peak signal, about -112 dB (limit -105 dB) |
| zipper, 13 continuous controls x blocks 1, 7, 512 (ramp and 2 Hz square) | all at or under the -90 dB limit or the held floor + 3 dB; closest margin 0.5 dB |
| alias, Standard (2.5 kHz mid band, 5 and 7 kHz high band, -6 dBFS, drives 6 to 36 dB) | -92.5 dB for nine models (row -90), tape -86.2 dB (exception -85) |
| alias, High | -113.8 dB for nine models (row -100), tape -97.8 dB (exception -97) |
| allocations over 400 blocks while models, drives, solo, bypass and crossovers change | 0 |
| CPU at 12 dB on every band, Draft / Standard / High | 0.93 / 2.03 / 3.59 % of a core (budget 1.3 / 2.5 / 4.5 %) |
| CPU with every band at 0 dB (crossover and glue only) | 0.49 / 0.50 / 0.49 % |

Deviations. The wavefolder's drive is a third of the control here (a half in the saturator): at 4x a band's wavefolder at 36 dB
aliased to -68 dB in the mid band, and with a third it meets the row. The tape model keeps the saturator's exception. The CPU is
about one saturator per driven band plus the crossover (0.33 %); a band at 0 dB drive, bypassed or muted by a solo is not run (its
shaper is warmed up on the last stretch of the band when it wakes). The global mix blends the delayed input with the band sum, which
is the crossover's all-pass of the input, so a mix between 0 and 1 with every drive at 0 dB combs where the all-pass phase differs
from zero (near the two crossover frequencies); mix 0 and mix 1 are exact. A model change takes the band's wet share to zero,
switches (the shaper restarts) and brings the share back, so the band dips toward its dry path (about 100 ms down while the glide passes -80 dB, then the share rises again) instead of
cross-fading two warm models as the saturator does.

## 26. Cabinet (`fx.cabinet`; `tests/CabinetTests.cpp`, suite "Cabinet")

One current module, registered last, nine parameters (on, mix, cabinet, mic position, mic distance, low cut, high cut, output, chain
slot), flagged `appended` so no earlier parameter index or modulation destination moved. No recorded or third-party response is
used (`PROVENANCE.md`). The 250 checks of the suite pass at 48 kHz, and the rendered rows again at 44.1, 96 and 192 kHz.

### 26.1 Designed features on the rendered output (impulse through the rack, 48 kHz, default mic: centre, 8 cm)

| cabinet | resonance design / measured | roll-off corner design / measured |
| --- | --- | --- |
| 1x12 Open Back | 105 / 104.7 Hz | 5200 / 5193 Hz |
| 2x12 Open Back | 92 / 92.3 Hz | 5000 / 4979 Hz |
| 4x12 Closed | 98 / 97.9 Hz | 4800 / 4804 Hz |
| 1x10 Combo | 128 / 127.4 Hz | 5900 / 5906 Hz |
| 2x10 Open Back | 118 / 117.8 Hz | 5600 / 5596 Hz |
| 1x15 Bass | 55 / 55.3 Hz | 3200 / 3206 Hz |
| 4x10 Bass | 64 / 62.7 Hz | 4500 / 4511 Hz |
| 8x10 Bass | 50 / 51.9 Hz | 3800 / 3803 Hz |

Limits: resonance within 5 %, corner within 10 % (worst measured 3.8 % and 0.4 %). The resonance is the highest level between 25 and
400 Hz; the corner is the last frequency within 3 dB of the highest half-octave-smoothed level of the presence region (0.3 to 1 times
the design corner). The model calibrates its high-pass and low-pass corners once per cabinet so that these two readings land on the
design values; the measurement is on the finished response (reflections, bells, proximity and tilt included), not on the table.

- Mic position 0 to 100 %: the top end (3.5 to 7 kHz against 0.4 to 1 kHz) falls monotonically, 1x12 from +2.6 dB to -5.6 dB (model and
  rendered); every cabinet moves by more than 5 dB (bass cabinets about 8 dB).
- Open-back comb notch at 1 / (2 x delay): 2.4 dB (1x12, 806 Hz), 2.9 dB (2x12, 641 Hz), 1.6 dB (1x10, 1 kHz), 2.1 dB (2x10, 909 Hz) below
  the neighbouring bands. The notch is modest by design: the reflection fades above 1.5 kHz (the cone beams its top end forward).
- Proximity: 0 cm has more bass than 40 cm by more than 2 dB on every cabinet; the room reflection (delay 0.4 ms + 2 x distance / c) grows
  with distance.
- Minimum phase: 97 % of the impulse energy in the first 10 ms, tail below -50 dB; mean power 400 Hz to 2.5 kHz within 0.5 dB of 0 dB.

### 26.2 Plan acceptance rows

- Latency 0 samples, reported (`crucible_latency_samples`), and true: the response peak is within 40 samples of the impulse.
- Silence in gives exactly silence out; reset clears the tail; the response after a reset equals a fresh instance's; bypass is bit-exact.
- No zipper: worst 1 ms envelope step (output 1.21x for a 18 dB step, mix 1.01x, high cut 1.09x, cabinet change 1.01x), low cut 2.2x per
  10 ms window at 100 Hz for a 20 to 300 Hz jump (a jump becomes a sweep of at most 12 % per 128 samples).
- Offline (High) render bit-identical over three runs including a cabinet and mic change; real-time (builder thread) path checked.
- Allocations on the audio thread over 3000 blocks with every parameter changing: 0.
- CPU, stereo instance, 48 kHz, blocks of 32, Release, best of 8 (audio thread only, the builder is separate): 0.074 %
  (load average 18). IR generation and convolver preparation take a few milliseconds on the builder thread per change.
