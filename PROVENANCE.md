# Provenance

Where the code and the numbers in this repository come from. The rule behind this page is simple: every design
comes from papers, public documentation or our own measurements, and nothing is copied from code we may not
copy.

## History

The engine began inside an application repository as the sound engine of two hardware-controlled instruments. It was
extracted with its git history, renamed to Crucible Engine, and released under the MIT licence. Names from the period
before the rename do not appear in this repository.

## Clean-room rule

- **GPL synthesizers** (Surge XT, Vital, and others) were studied through public material only: documentation,
  manuals, conference talks, issue trackers and design discussions. No code, tables or coefficients were copied or
  transcribed. Contributors must not read their sources while working on the matching feature here; see
  `CONTRIBUTING.md`.
- **Commercial instruments** were only ever measured as black boxes: audio rendered through a plug-in host and
  analysed. No disassembly, no preset or resource extraction, no sample-level cloning. The analysis suite and the
  recorded summary numbers are in `tools/reference-bench`; no audio, presets or plug-in state of any commercial product
  is part of this repository.
- **Permissive code** is vendored only under `third_party/` with its licence file and an entry in `THIRD_PARTY.md`.

## Designs and the papers they follow

The topics below name the published sources the corresponding code follows. Each list is the starting point for a
reviewer, not a claim that every detail is taken from one paper.

- **Band-limited oscillators:** minBLEP and minBLAMP step and corner corrections (Brandt, Eli, "Hard sync without
  aliasing", ICMC 2001; Stilson and Smith, "Alias-free digital synthesis of classic analog waveforms", ICMC 1996);
  mip-mapped wavetables with per-level partial limits.
- **State-variable filters:** zero-delay-feedback (topology-preserving transform) structures (Zavalishin, "The Art of
  VA Filter Design"; Cytomic technical papers by Andrew Simper).
- **Ladder and diode filters:** Huovilainen 2004; Stinchcombe's analyses of the diode ladder; D'Angelo and Valimaki
  2013. The nonlinear ladder family in `src/filters/` (transistor ladder, OTA cascade, coupled diode ladder) is
  written from those papers and from Zavalishin's trapezoidal (zero-delay-feedback) treatment: the stage equations
  are circuit principles (a tanh per differential pair, a 4-node tridiagonal diode network with the first capacitor
  doubled), solved by Newton's method in closed form; the pitch-correction tables and every constant (feedback
  range, tolerances, oversampler coefficients shared with `Oversampler`) come from our own measurements.
- **Oversampling:** FIR half-band and polyphase all-pass IIR half-band filters (standard multirate DSP
  texts); the rejection of each stage is measured in `docs/MEASUREMENTS.md`.
- **Saturation and distortion:** anti-derivative anti-aliasing (Parker, Zavalishin, Le Bivic, DAFx 2016); tube and
  transformer models from the circuit-modelling literature (Koren; Holters and Zolzer).
- **Equalisers and crossovers:** matched second-order digital filters (M. Vicanek, "Matched Second Order Digital Filters", 2016: pole
  mapping and the squared-magnitude identity used to place the numerator); prescribed Nyquist gain (S. Orfanidis, "Digital Parametric
  Equalizer Design with Prescribed Nyquist-Frequency Gain", JAES 45(6), 1997); the analog bell and shelf prototypes of the Audio EQ
  Cookbook (R. Bristow-Johnson); Butterworth and Bessel pole tables computed from the polynomials; Linkwitz-Riley crossovers
  (S. Linkwitz, JAES 1976) with all-pass compensation of the split bands; linear-phase FIR by frequency sampling and uniformly
  partitioned overlap-save convolution (Oppenheim and Schafer; Wefers, "Partitioned convolution algorithms for real-time auralization").
- **Reverb and delay:** feedback delay networks (Jot and Chaigne 1991); Schroeder and Moorer structures.
- **Reverb and delay:** feedback delay networks (Jot and Chaigne 1991); Schroeder and Moorer structures. The algorithmic
  reverbs (`include/crucible/fx/Fdn.h`, `src/fx/{Fdn,EarlyReflections,PlateTank}.cpp`) follow Jot's frequency-dependent
  absorption (the per-line gain is 10^(-3 delay / RT60) at DC, in the middle and at Nyquist, realised with first-order
  shelves), orthogonal Hadamard and Householder feedback matrices, Dattorro's figure-eight plate tank (JAES 1997; the
  published delay lengths, our own output taps), the image-source method for the shoebox early reflections (Allen and
  Berkley, JASA 1979), velvet-noise taps (Karjalainen and Jarvelainen 2007) and Schroeder all-pass diffusion. No code of any
  commercial or GPL reverb was read; those products are only known through their public descriptions.
- **Speaker cabinets (`fx.cabinet`):** no measured or recorded cabinet response, and nothing taken from a commercial or GPL cabinet
  simulator, is used. The magnitude of each cabinet is written down from public acoustics (the second-order resonance of a driver
  in a closed or ported box, Thiele and Small; the rise of a cone's response from breakup; the shelf of a microphone's proximity
  effect; the comb of a reflection off a back wall) with numbers chosen by us, and the impulse response is the minimum-phase filter of
  that magnitude (real-cepstrum folding, Oppenheim and Schafer).
- **Loudness and metering:** ITU-R BS.1770 and EBU R128.
- **Microtuning:** the Scala scale and keyboard-mapping file formats, as publicly specified; checked against the
  MIT-licensed tuning-library in `third_party/`.
- **Loudness and true-peak metering:** ITU-R BS.1770-4 (K-weighting, gating, true peak by 4x oversampling, Annex 1 and 2),
  EBU R128 and EBU Tech 3341 / 3342 (momentary, short-term and integrated loudness, loudness range); checked against the
  MIT-licensed libebur128 in `third_party/` (tests only).
- **Analysis:** Blackman-Harris windowed spectra and standard THD, slope and drift estimators, as described in the
  literature on audio measurement.

## Open-source projects consulted

Every repository read while designing the engine. Each was read only (no build scripts, tests or binaries were run).
**Nothing was copied: no code, tables, constants or data.** What was taken is the idea level only, restated in our own
words; anything implemented here is written from the cited papers and our own design. The engine is MIT-licensed and
GPL code cannot be pasted into it without GPL-licensing the result, which is why the study stays at the level of ideas
and published literature.

| repository | commit | licence | read | taken |
|---|---|---|---|---|
| github.com/surge-synthesizer/surge | 1cac43d6772a6646a54020683320a84364924ef3 | GPL-3.0-or-later | `src/common/dsp/oscillators/ClassicOscillator.cpp`, `WavetableOscillator.cpp`, `ModernOscillator.cpp`, `src/common/dsp/Wavetable.cpp`, `SurgeSynthesizer.cpp` (polyphony and stealing), `SurgeVoice.cpp` (modulation sources), `globals.h` (oversampling constants) | ideas: windowed-sinc impulse (BLIT) synthesis with sub-sample placement, per-octave mip tables with a bias threshold, 2x oscillator/filter rate with half-band decimation per scene, soft voice stealing |
| github.com/surge-synthesizer/sst-filters | 343169683176273b57086a64b6d1fc0a04e6204c | GPL-3.0-or-later | `include/sst/filters/CytomicSVF.h`, `VintageLadders.h`, `DiodeLadder.h`, `K35Filter.h`, `OBXDFilter.h`, `CutoffWarp.h`, `ResonanceWarp.h`, `TriPoleFilter.h`, `QuadFilterUnit*.h`, `HalfRateFilter.h` | ideas: four voices per SIMD register, per-sample coefficient deltas across a block, the list of published filter models they adapt |
| github.com/surge-synthesizer/sst-basic-blocks | b536f32d43fa9f8782e41d46cab4cc38b1a1a0f1 | GPL-3.0-or-later (a few headers dual MIT) | `include/sst/basic-blocks/dsp/OscillatorDriftUnisonCharacter.h`, `EllipticBlepOscillators.h`, `LanczosResampler.h`, `BlockInterpolators.h` | ideas: unison detune spread and alternating pan law with 1/sqrt(N) gain, slow filtered-noise drift, block-linear parameter interpolation |
| github.com/surge-synthesizer/sst-effects | 50c32ab14616d913074d82e21a474078215a9f63 | GPL-3.0-or-later | `include/sst/effects/Reverb2.h`, `Delay.h`, `Chorus.h` | ideas: allpass-loop reverb topology with damping per block, sinc-interpolated delay reads, crossfaded delay retiming |
| github.com/mtytel/vital | 636ca0ef517a4db087a6a08a6a8a5e704e21f836 | GPL-3.0 | `src/synthesis/producers/synth_oscillator.*`, `lookups/wavetable.*`, `filters/ladder_filter.*`, `digital_svf.h`, `dirty_filter.h`, `effects/reverb.h`, `effects/compressor.h`, `framework/voice_handler.h`, `utilities/smooth_value.*`, `effects_engine/sound_engine.cpp`, `modules/modulation_connection_processor.h` | ideas: per-block spectral resynthesis with an exact last harmonic, spectral morphs, detune power curve, global oversampling scaled by host rate, 16-line FDN reverb, upward/downward multiband compression, one-pole parameter smoothing, voice priority modes |
| github.com/asb2m10/dexed | 2e182b3db85c09083ab13c8b9b00565ce7d9ff85 | GPL-3.0 (the `msfa` engine: Apache-2.0) | `Source/msfa/fm_op_kernel.cc`, `env.cc`, `sin.cc`, `exp2.cc` | ideas: per-block linear gain ramps for operators, log-domain envelopes, table sine with fixed-point phase |
| github.com/zynaddsubfx/zynaddsubfx | c379439e41b34dee5eededbf5b13e7514c4ded64 | GPL-2.0-or-later | `src/Synth/OscilGen.cpp`, `ADnote.cpp`, `src/Params/PADnoteParameters.cpp`, `src/DSP/Unison.cpp`, `Value_Smoothing_Filter.h`, `src/Effects/` (listing) | ideas: per-note spectrum truncation at the played pitch, PADsynth bandwidth-spread harmonics, unison by modulated delay |
| github.com/mtytel/helm | abdedd527e6e1cf86636f0f1e8a3e75b06ed166a | GPL-3.0 | `mopo/src/wave.h` and the `mopo/src` listing | idea: one band-limited table per harmonic count (finest possible mip granularity) |
| github.com/DISTRHO/Cardinal | 0a530b73273afc914ec71a78e9165cd28c53b599 | GPL-3.0-or-later | top level and `.gitmodules` only (module code lives in submodules, read from VCV's own repositories below) | - |
| github.com/VCVRack/Rack | 061ccf63c1758599396ac1bb10d47345d9d34076 | GPL-3.0-or-later | `include/dsp/minblep.hpp`, `src/dsp/minblep.cpp`, `include/dsp/resampler.hpp` | ideas: minimum-phase BLEP built from a windowed sinc by cepstral reconstruction, inserted at sub-sample discontinuities |
| github.com/VCVRack/Fundamental | 10dd0160c664770910e5584b7b00498cc48d9ddd | GPL-3.0-or-later | `src/VCO.cpp` | ideas: minBLEP on every waveform discontinuity including hard and soft sync |

No other open-source synthesizer was consulted. The two commercial reference instruments were only hosted and measured;
none of their files were opened, inspected or copied.

### Published sources behind the ideas

Implementations in the engine should start from these primary sources, not from the code above.

- T. Stilson, J. O. Smith, "Alias-Free Digital Synthesis of Classic Analog Waveforms", ICMC 1996 (BLIT).
- E. Brandt, "Hard Sync Without Aliasing", ICMC 2001 (minBLEP).
- V. Välimäki et al., "Alias-Suppressed Oscillators Based on Differentiated Polynomial Waveforms", IEEE TASLP 2010 (DPW).
- V. Välimäki, J. Pekonen, J. Nam, "Perceptually informed synthesis of bandlimited classical waveforms using integrated polynomial interpolation", JASA 2012 (polyBLEP/BLAMP).
- G. Luff (Signalsmith Audio), "Elliptic BLEP" (public article and reference code).
- V. Zavalishin, "The Art of VA Filter Design", rev. 2.1.2, 2020 (TPT/ZDF filters, nonlinear ladders).
- A. Simper, "Linear Trapezoidal Integrated State Variable Filter With Low Noise Optimisation", Cytomic technical paper, 2011-2016.
- A. Huovilainen, "Non-Linear Digital Implementation of the Moog Ladder Filter", DAFx 2004.
- J. Parker, V. Zavalishin, E. Le Bivic, "Reducing the Aliasing of Nonlinear Waveshaping Using Continuous-Time Convolution", DAFx 2016 (ADAA).
- J.-M. Jot, A. Chaigne, "Digital Delay Networks for Designing Artificial Reverberators", AES 1991 (FDN).
- J. Dattorro, "Effect Design Part 1: Reverberator and Other Filters", JAES 1997.
- N. O. Paul (Nasca Octavian Paul), "PADsynth algorithm" (author's public description).
- M. Schroeder, "New Method of Measuring Reverberation Time", JASA 1965 (energy decay curve, used by the bench).

## Vendored third-party code (P13)

Permissive code vendored under `third_party/` with its licence file (see `THIRD_PARTY.md` for the rows). Each repository was
cloned from its official upstream into a scratch directory outside the checkout, the commit below was pinned, the licence file
was read (MIT, or BSD-like for pffft), only the files the build needs were copied unmodified, and no upstream build script,
test or binary was run.

| repository | commit | licence | files | used for |
|---|---|---|---|---|
| github.com/pichenettes/eurorack | f4cfa7697095a28d292bff758beb51deb49b5a86 | MIT (STM32 code; the AVR code in the same repository is GPL-3.0 and was not touched) | `rings/resources.h`, `rings/resources.cc` (the lookup tables, the licence text is in each file header) | the stiffness table of the modal resonator; the designs of the Rings and Elements resonators, strings and exciters were read (`rings/dsp/resonator.cc`, `string.cc`, `elements/dsp/resonator.cc`, `exciter.cc`, `voice.cc`) and adapted in `ModalOscillator.cpp` |
| github.com/pichenettes/stmlib | e3bd7c9cc00e4364166f9905c0509b6ffd0535ec (the submodule commit pinned by the eurorack commit above) | MIT | `LICENSE`, `stmlib.h`, `dsp/dsp.h`, `dsp/cosine_oscillator.h` | helpers included by the Rings resources and used by the pickup comb |
| github.com/Signalsmith-Audio/signalsmith-stretch | a670068d9aeb64913331d5cc29337b19a457a7df | MIT | `signalsmith-stretch.h`, `LICENSE.txt`, the include forwarder | `osc.spectral` |
| github.com/Signalsmith-Audio/linear | de55e6a50ffcf6f8f43f649692d94691c7025151 | MIT | `stft.h`, `fft.h`, `LICENSE.txt`, the include forwarder | included by signalsmith-stretch 1.4.0 (it no longer uses signalsmith-dsp) |
| github.com/marton78/pffft | aa16fd3db58de4ba5dae8b0438440bb9da46b6fa | BSD-like (FFTPACK / NCAR) | `src/pffft.c`, `src/pffft_common.c`, `src/pffft_priv_impl.h`, `src/simd/pf_*float.h`, `src/simd/pf_fma_x86.h`, `include/pffft/pffft.h`, `LICENSE.txt` | the FFT backend of `Fft.h` off Apple, after the benchmark in `tests/FftBench.cpp` met the 1.5x rule |

The spectral warps and the spectral frame interpolation in `Spectral.h` and the harmonic oscillator are our own designs,
built from published ideas: spectral envelope and peak morphing (Fitz and Haken style sinusoidal track morphing, Settel and
Lippe's frequency-warped spectral interpolation) and the additive-synthesis partial laws in the documentation of the
oscillator; no source of another synthesizer was read for them.

## Verification

Claims about sound are numbers: aliasing floors, slopes, self-oscillation tracking and similar are measured by the
tests (see `docs/MEASUREMENTS.md`). Bit-exact hashes and golden renders protect existing behaviour (see
`CONTRIBUTING.md`).
