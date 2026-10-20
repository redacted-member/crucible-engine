<!-- SPDX-License-Identifier: MIT -->
# Building and testing on Windows (MSVC)

Crucible Engine builds with Visual Studio 2022 (MSVC 19.4x) and CMake 3.22 or newer. The library needs no third-party code and no network access.

## Build

From a "Developer PowerShell for VS 2022" (or any shell with CMake on the path):

```bat
cmake -S . -B build
cmake --build build --config Release -- /m:8
ctest --test-dir build -C Release --output-on-failure
```

The Visual Studio generator is multi-configuration, so the configuration goes on the build and test commands (`--config Release`, `-C Release`), not on the configure step. Use Release for tests: Debug is much slower and the CPU-budget tests assume an optimised build.

Notes:

- Build from a short path (for example `C:\src\crucible`). Under a very long path the compiler check can fail with `FTK1011` or "not able to compile a simple test program".
- `crucible_engine_tests` needs more than the default 1 MB stack because a few test functions keep large objects on the stack. The build sets `/STACK:8388608` for it; keep that if you change the link options.
- `CRUCIBLE_X86_AVX2` (see the README) compiles the x86 kernels with `/arch:AVX2`. It is off by default.
- `crucible_engine_audit_tests` is long (about 20 minutes on a desktop CPU in Release). Its ctest timeout is raised accordingly. `CRUCIBLE_AUDIT_STRIDE=n` audits only every n-th parameter, for a faster run.

## MSVC notes

- File names are converted from UTF-8 to UTF-16 (`src/Utf8Path.h`), so patches, WAVs, MIDI files, zone folders and preset libraries work in folders with non-ASCII names. The command line tools `crucible-render` and `crucible-measure` read `argv` in the ANSI code page, so pass them ASCII paths.
- MSVC's small-string buffer holds 15 characters (libc++ 22). `Engine::setParam` and `ParameterModel::indexOf` take `const std::string&`, so calling them from the audio thread with a literal longer than that allocates. Resolve parameter ids to indices before the audio thread starts.
- A clean Release build has no compiler warnings from the engine. The one line CMake prints, `D9025 overriding '/W3' with '/w'`, comes from the third-party test sources and is harmless.
- The CPU-budget tests were tuned on Apple clang and are checked on MSVC x64; on a loaded machine a timing test can fail once and pass on a re-run on a quiet machine.
