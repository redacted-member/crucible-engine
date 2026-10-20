# Vendored: Mutable Instruments Rings tables and stmlib helpers

Upstream, both MIT licensed, copyright Emilie Gillet:

* https://github.com/pichenettes/eurorack at `f4cfa7697095a28d292bff758beb51deb49b5a86`: `rings/resources.h`,
  `rings/resources.cc` (lookup tables; the licence text is in the header of each file).
* https://github.com/pichenettes/stmlib at `e3bd7c9cc00e4364166f9905c0509b6ffd0535ec` (the commit the eurorack repository
  pins as its `stmlib` submodule): `LICENSE` (copied as `third_party/mutable/LICENSE`), `stmlib.h`, `dsp/dsp.h`,
  `dsp/cosine_oscillator.h`.

The files are unmodified. Only what `src/modules/oscillator/ModalOscillator.cpp` uses is here: the stiffness table
(`rings::lut_stiffness`) and the cosine oscillator that generates the pickup comb. The `LICENSE` file also mentions
`third_party/STM` of the stmlib repository (code of STMicroelectronics); that directory is not vendored.

"Mutable Instruments" is a registered trademark of its owner. Upstream asks that derivative works do not use the name; the
engine's module is called "Modal" and credits the Rings and Elements designs in `ModalOscillator.cpp` and `THIRD_PARTY.md`.
