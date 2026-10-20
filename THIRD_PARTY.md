# Third-party code and content

Code that is not ours lives under `third_party/<name>/` with its licence file. Only permissive licences
(MIT, BSD, zlib, Apache-2.0, ISC) are accepted; GPL code is never vendored. Every entry needs its row here.

| Name | Licence | Where | Used by | Upstream and pinned commit |
| --- | --- | --- | --- | --- |
| tuning-library (Surge Synth Team, Paul Walker) | MIT | `third_party/tuning-library/` | tests only: `tests/TuningOracleTests.cpp` checks `Tuning` against it | https://github.com/surge-synthesizer/tuning-library at `48422e2f014fcda8dd5d1a4678bb2674faf3bb3e` |
| mutable (Emilie Gillet, Mutable Instruments: Rings resource tables, `stmlib` headers) | MIT | `third_party/mutable/` | `osc.modal` (`src/modules/oscillator/ModalOscillator.cpp`): the stiffness table and the cosine pickup; the resonator, string and exciter designs of Rings and Elements are adapted in that file | https://github.com/pichenettes/eurorack at `f4cfa7697095a28d292bff758beb51deb49b5a86` (`rings/resources.{h,cc}`), https://github.com/pichenettes/stmlib at `e3bd7c9cc00e4364166f9905c0509b6ffd0535ec` (the commit that eurorack pins; `stmlib.h`, `dsp/dsp.h`, `dsp/cosine_oscillator.h`, `LICENSE`) |
| signalsmith-stretch (Geraint Luff, Signalsmith Audio) | MIT | `third_party/signalsmith-stretch/` | `osc.spectral` (`SpectralOscillator.cpp`): time stretch and pitch shift | https://github.com/Signalsmith-Audio/signalsmith-stretch at `a670068d9aeb64913331d5cc29337b19a457a7df` (tag 1.4.0) |
| signalsmith-linear (Signalsmith Audio) | MIT | `third_party/signalsmith-linear/` | the STFT and FFT that signalsmith-stretch includes (`stft.h`, `fft.h`; its own portable FFT, no platform backend) | https://github.com/Signalsmith-Audio/linear at `de55e6a50ffcf6f8f43f649692d94691c7025151` (tag 0.6.4, the version signalsmith-stretch 1.4.0 names) |
| pffft (Julien Pommier, Hayati Ayguen, Dario Mambro) | BSD-like (FFTPACK / NCAR licence, `third_party/pffft/LICENSE.txt`) | `third_party/pffft/` | `Fft.h`: the backend on platforms without Accelerate (single precision, SIMD), benchmarked in `tests/FftBench.cpp` | https://github.com/marton78/pffft at `aa16fd3db58de4ba5dae8b0438440bb9da46b6fa` (`src/pffft.c`, `pffft_common.c`, `pffft_priv_impl.h`, `src/simd/pf_*float.h`, `src/simd/pf_fma_x86.h`, `include/pffft/pffft.h`) |
| libebur128 (Jan Kokemueller) | MIT (`queue.h`: BSD-3-Clause) | `third_party/libebur128/` | tests only: `tests/LoudnessTests.cpp` checks `crucible/fx/Loudness.h` against it | https://github.com/jiixyj/libebur128 at `67b33abe1558160ed76ada1322329b0e9e058b02` (v1.2.6; `ebur128.{c,h}`, `queue/sys/queue.h`) |
| FFTConvolver (HiFi-LoFi) | MIT (`third_party/fftconvolver/LICENSE.txt`) | `third_party/fftconvolver/` | `fx/PartitionedConvolver.h` (`NonUniformConvolver`): the split-complex buffer and multiply-accumulate helpers (`Utilities.{h,cpp}`) and the `AudioFFT` interface (`AudioFFT.h`), which `src/fx/AudioFftAdapter.cpp` implements on `crucible::Fft`; the bundled Ooura / FFTW / Accelerate FFT (`AudioFFT.cpp`) is not vendored or compiled | https://github.com/HiFi-LoFi/FFTConvolver at `f2cdeb04c42141d2caec19ca4f137398b2a76b85` (`AudioFFT.h`, `Utilities.{h,cpp}`, `COPYING.txt` as `LICENSE.txt`) |

Vendored files are unmodified copies of the files listed in the last column (only the files the build uses). The Mutable
Instruments name is a trademark of its owner: this repository names its module "Modal" and credits the Rings and Elements
designs in the source and here, as the upstream guidelines ask.

The library itself (`include/`, `src/`) contains no third-party code. The vendored code above is compiled into the library
(private: no public header includes it). On Apple platforms the FFT of `crucible/Fft.h` is the system Accelerate framework;
elsewhere it is pffft (`-DCRUCIBLE_ENGINE_FFT_BACKEND=PORTABLE` selects the double-precision radix-2 code of our own).

## Content

Example content that ships in this repository (generated wavetables, example patches, test fixtures) is released
under CC0 1.0 (public domain dedication) unless a file says otherwise. Code stays MIT.
