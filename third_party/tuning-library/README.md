# tuning-library (vendored, tests only)

Surge Synth Team's standalone microtuning library (Scala .scl / .kbm), MIT licence (`LICENSE.md`,
Copyright 2019-2020 Paul Walker). Upstream: https://github.com/surge-synthesizer/tuning-library,
commit 48422e2f014fcda8dd5d1a4678bb2674faf3bb3e. The three headers in `include/` are unmodified.

It is used only by `tests/TuningOracleTests.cpp`, as an independent implementation to check `crucible::Tuning`
against. It is not linked into the engine library and nothing in `src/` or `include/` includes it. The upstream
tests and the Catch2 copy in its `libs/` are not vendored.
