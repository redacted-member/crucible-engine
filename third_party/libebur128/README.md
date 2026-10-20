# libebur128 (vendored, tests only)

Jan Kokemueller's EBU R128 / ITU-R BS.1770 loudness library, MIT licence (`LICENSE`, Copyright (c) 2011 Jan Kokemueller).
Upstream: https://github.com/jiixyj/libebur128, commit 67b33abe1558160ed76ada1322329b0e9e058b02 (version 1.2.6,
2021-02-14). `ebur128.c` and `ebur128.h` are unmodified. `queue/sys/queue.h` is the BSD list macro header that upstream
ships beside them (Regents of the University of California, three-clause BSD licence in the file's own header).

It is used only by `tests/LoudnessTests.cpp`, as an independent implementation to check `crucible/fx/Loudness.h`
against (integrated, momentary and short-term loudness, loudness range, sample and true peak). It is compiled as a
separate C object library for the test executable and is not linked into the engine library; nothing in `src/` or
`include/` includes it. Upstream's build files, tests and documentation are not vendored.
