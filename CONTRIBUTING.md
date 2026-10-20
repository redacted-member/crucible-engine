# Contributing to Crucible Engine

Thank you for helping. This page is the short version of how changes get in. Read the clean-room rule and the
real-time rules before you write DSP; they are the two ways a good patch can still be rejected.

## Ground rules

1. **Real time.** Anything reachable from `process()`, `render()` or a voice never allocates, locks, waits, throws,
   logs, or calls into the operating system. `tests/PlatformTests.cpp` counts `operator new` while 16 voices, the
   sequencer and every effect run; a new module must keep that count at zero. Setup (`prepare()`, wavetable
   creation, JSON export) is control-thread code and may allocate. Parameters are atomics read once per block.
2. **Bit-exact policy.** Existing sound does not change by accident. `tests/golden/renders.txt` and
   `tests/golden/bitexact.txt` hold fingerprints of whole renders. A change that moves them is either a bug or an
   intended sound change. For an intended change, regenerate with `CRUCIBLE_UPDATE_GOLDEN=1` /
   `CRUCIBLE_UPDATE_BITEXACT=1`, explain it in the pull request and add it to `docs/SOUND_CHANGES.md`. New behaviour
   goes behind a new parameter or choice whose default selects the legacy code path through an early branch.
3. **Compatibility.** Parameter ids are never renamed or removed; new parameters and choices are appended. Patches
   store ids, never indices. Old patches must keep loading.
4. **Every claim is a number.** "Sounds better" is not evidence. Add a test with a pass rule (aliasing floor in dB,
   slope in dB/octave, error in cents, ...). Nothing is judged by ear in CI.
5. **No GPL code, ever** (see below).

## Clean-room rule

Surge XT, Vital and similar GPL synthesizers are studied through **public material only**: manuals, documentation,
talks, blog posts, issue trackers, design discussions. If you have read their source for the feature you are about to
write, do not write it here. Never copy or transcribe code, tables, coefficients, preset data or wavetables from them
or from any project with a licence we cannot ship under MIT. Commercial instruments may only be measured as black
boxes (rendered audio), never disassembled or unpacked. Write the source of each design in the pull request: a paper,
a public document, or your own measurement. Permissive code (MIT, BSD, zlib, Apache-2.0, ISC) may be vendored under
`third_party/<name>/` with its licence file and a row in `THIRD_PARTY.md`.

## Developer Certificate of Origin

Sign off every commit (`git commit -s`). The sign-off line certifies the Developer Certificate of Origin 1.1
(https://developercertificate.org): you wrote the change or have the right to submit it under the MIT licence. Pull
requests with unsigned commits cannot be merged.

## Building and testing

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
tools/ci.sh            # Release, ASan+UBSan, TSan and the scalar fallback
tools/check-spdx.sh    # every source file carries its licence header
```

Run the sanitizer builds before you open a pull request that touches threading, memory or SIMD code.

### CPU-budget tests on shared machines

Many tests check a CPU budget ("percent of one core") or a timing ratio. They need a quiet machine and are enforced by
default, in `tools/ci.sh` too. On a shared machine (the hosted GitHub Actions runners, a busy laptop) set
`CRUCIBLE_SKIP_CPU_TESTS=1`: every CPU-budget or timing expectation is then logged as `skipped CPU check ... would
pass` or `would fail` and never fails the run; all other expectations are unchanged. `crucible-module-check` honours the
same variable: it then reports, but does not enforce, the CPU budget and the denormal-tail slowdown. `.github/workflows/ci.yml` sets it
for the hosted runners.

```sh
CRUCIBLE_SKIP_CPU_TESTS=1 ctest --test-dir build --output-on-failure
```

Sanitizer builds on Linux (AddressSanitizer / ThreadSanitizer) link a runtime that defines `operator new` itself, so the
allocation counter of the tests is not compiled there and the "no allocation on the audio thread" checks are vacuous in
those builds; the Release and scalar builds carry them.

## Adding a module

`docs/MODULE_GUIDE.md` walks through it: a module is a `ModuleDescriptor` registered with one line in
`src/modules/Modules.cpp`. Append it after the existing ones so parameter indices do not move, regenerate
`docs/PARAMETERS.md`, add tests, and add a row to `docs/ROADMAP.md`. Every built-in must pass
`build/crucible-module-check --builtin` (real-time contract: no allocation, bounded and finite output, deterministic per
seed). Modules that live outside this repository use the C ABI of `crucible/crucible_module.h`; the guide has a section.

## Style

C++20, formatted with clang-format using the checked-in `.clang-format` (Allman braces, four spaces, 125 columns).
Names are descriptive, comments say why and not what. Every source file starts with an SPDX line:

```cpp
// SPDX-License-Identifier: MIT
```

(`#` comments in CMake and shell files, `/* */` in C files.) Portable code first: guard compiler-specific flags and
intrinsics, and keep a scalar fallback for every SIMD path.

## Pull requests

- Keep one logical change per pull request, with tests.
- State the verification: which tests, which sanitizer runs, which measurements.
- Update `CHANGELOG.md` under *Unreleased* for anything a user can see.
- Be patient with review; maintainers volunteer their time. Participation is covered by `CODE_OF_CONDUCT.md`.

## Release cadence

A minor release roughly every quarter (`0.x.0`), patch releases when needed, nightly builds from `main` as CI
artifacts. A release has a tag, a `CHANGELOG.md` section and a matching `project(CrucibleEngine VERSION ...)`. The C ABI
version (`CRUCIBLE_API_VERSION`) changes only when the C API breaks, which is announced one minor release ahead.

## Maintainers

The project aims for more than one maintainer at all times. If you review regularly and want to help merge, say so in
an issue; maintainers rotate and are listed in the repository settings.
