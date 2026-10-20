# Security policy

## Reporting a vulnerability

Please report suspected vulnerabilities privately through GitHub's private vulnerability reporting: open the
repository's **Security** tab and choose **Report a vulnerability**. Do not open a public issue for a security problem.

Include the engine version or commit, the platform and compiler, and the smallest input (patch, wavetable, impulse
response, sample or MIDI sequence) that reproduces the problem. A crash log or sanitizer report helps.

We aim to acknowledge a report within a few days and to agree a fix and disclosure date with the reporter.

## Supported versions

Only the latest release receives security fixes.

## What counts as a security issue

Crucible Engine is an audio library that hosts load untrusted content into. These are security issues:

- Crashes, hangs or memory-safety errors (out-of-bounds access, use after free, uninitialised reads, undefined
  behaviour, integer overflow) triggered by untrusted input: a malformed or hostile patch, product definition,
  wavetable, impulse response, sample file, tuning file or formant table, or by hostile calls through the C API.
- Unbounded memory or CPU use caused by a small input.
- Problems in the build or CI scripts that could run untrusted code.

These are ordinary bugs, not security issues: audio glitches, sound differences, wrong parameter ranges or
performance regressions with valid input. Report them as normal issues.

Third-party code under `third_party/` is reported to the upstream project where possible; tell us as well so we can
update the vendored copy.
