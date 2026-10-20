# reference-bench

The measurement suite behind the engine's quality targets: a numpy-only Python package that turns rendered 32-bit float
WAV files into numbers, charts and a pass/fail report, and compares them with recorded results of two commercial
synthesizers and a set of targets.

`tools/refbench.sh [build-dir] [--all]` runs the whole loop for the engine: it renders the plan with `crucible-measure`,
analyses the renders and compares them with `data/serum2.json` and `data/pigments.json`. It prints the failing targets
(none expected: all 39 pass).

## What is in this folder

```
analysis/refbench.py            entry point (plan, job, run, analyze, compare, selftest)
analysis/refbench_analysis/     tests (suite), DSP helpers, analysers, reports, charts
analysis/profiles/              parameter maps (serum2, pigments) and the engine profile (crucible)
analysis/targets.json           pass/fail rules
data/serum2.json, pigments.json recorded summary numbers of the two reference synthesizers
```

The folder holds analysis code and summary numbers only. No audio, presets, plug-in state or other content of any
commercial product is included; renders are produced on demand and ignored by git (see `.gitignore`). The plug-in host that produced the reference numbers is not part of this repository, so the `run` command (which drives
that host) and re-recording the references are not available here; `analyze` and `compare` work on any folder of WAVs.

Commercial plug-ins were treated as black boxes: loaded in a host, played with MIDI, parameters set through what they
publish to any host, and the audio measured. See `PROVENANCE.md`.

## Measurement suite

Python 3.10+ with numpy. matplotlib is optional (used for the PNGs when installed; otherwise a
built-in rasteriser draws them). SVG versions are always written.

```sh
cd tools/reference-bench/analysis
python3 refbench.py run --profile profiles/serum2.json   --work ../work/serum2
python3 refbench.py run --profile profiles/pigments.json --work ../work/pigments
python3 refbench.py run --profile profiles/crucible.json --work ../work/crucible
python3 refbench.py compare --result ../work/serum2/results.json --result ../work/pigments/results.json \
        --candidate ../work/crucible/results.json --out-dir ../report
```

`run` writes `job.json`, `plan.json`, `renders/` (WAVs, ignored by git), `results.json` (numbers) and
`results.plotdata.npz` (curves for charts, ignored by git). `--groups oscillator,filter` or
`--tests a,b` select a subset; `--analyze-only` re-analyses existing renders. `compare` writes
`REPORT.md`, `plots/*.png|svg`, `data/<profile>.json` and `targets_evaluated.json`.

### What is measured

| group | tests | metrics |
|---|---|---|
| oscillator | saw, square, sine at notes 36, 60, 84, 96, 103, 108 | f0 and cents error, aliasing below 20 kHz and in total (inharmonic / harmonic energy), worst spur below 20 kHz, harmonic error vs ideal, bandwidth (last harmonic within 3 dB), THD and THD+N (sine), DC |
| unison | 8 voices at C3 and C7, default and maximum detune | voice spread (cents), aliasing with a spread-aware harmonic mask, L/R correlation, side/mid, mono-sum loss |
| wavetable | position swept 0..1 over 4 s at C6 and C7 | aliasing per 43 ms frame (median, 90th percentile, max), level steps between frames |
| filter | LP12, LP24, HP12, BP12, ladder (and Pigments' default F1) at 1 kHz; LP24 at 250 Hz to 12 kHz, resonance 0 and 0.9; resonance 0.5/0.9/1.0; self-oscillation; drive 0/0.5/1 | response from the harmonic transfer of a 16.35 Hz saw (filter on / off), -3 and -6 dB points, slope, passband gain, resonant peak frequency and gain, tracking across cutoff, self-oscillation level and pitch, THD against drive |
| envelope | ADSR 100/300/50 %/400 ms, 1 s/1 s/25 %/1.5 s, zero attack | attack 10-90 % and to peak, attack curvature, decay to 63 % and 95 %, sustain error, release to -20 and -60 dB, note-on latency, onset and release clicks (energy above 2 kHz around a 440 Hz sine) |
| pitch, noise | pitch bend +/-2 st; no notes; 1-3 s after release; velocity 127..8 | bend error (cents), idle and residual level, DC, velocity curve |
| lfo | sine/triangle/saw/square at 2 Hz, sine at 9 Hz, routed to amplitude | rate error, shape correlation with the ideal shape |
| fx | reverb 1 s and 3 s, delay 250 ms / feedback 0.5, chorus, 3-band EQ +6 dB, compressor 4:1 static curve | T20/T30/EDT and RT60 ratio, band decay at 250 Hz/1 kHz/4 kHz, tail L/R correlation, echo time and feedback per repeat, chorus width and sidebands, EQ band gain error, measured ratio and threshold |
| cpu | 1 note; 8 notes; 8 x 8 unison; 16 x 8 unison; 8 x 8 with a 24 dB low-pass (each best of three runs) | CPU % of one core (wall time and all threads), real-time factor, worst block |

### Analog-behaviour types (pending)

`measure_analog.py` adds seven analysis types for the community-engine verification plan: `selfosc_track` (pitch of a
self-oscillating filter against its cutoff, level steadiness), `res_bassloss`, `res_level_dependence`, `sat_harmonics`
(H2..Hn, THD, even / odd shares), `drift_stats` (pooled std, autocorrelation time and PSD slope of the pitch of many notes),
`env_rc_fit` (exponential fits of attack, decay, release; retrigger step) and `voice_variance` (pitch and level spread over
64 notes). Their tests are in `suite_analog.py`, listed with `refbench.py plan --include-pending`; their targets in
`targets.json` carry `"status": "pending"` (reported as `pending`, neither pass nor fail) until a profile or the engine can run
them. `python3 refbench.py selftest` recovers synthetic truth with every analyser.

### Profiles and roles

A profile maps abstract roles onto one plugin's parameters: `saw`, `square`, `sine`, `wavetable`
(ramp), `osc_kill` (ramp to silence), `unison.voices`, `filter.enable|types.<t>|cutoff|resonance|drive`,
`amp_env.attack|decay|sustain|release`, `bend_range`, `lfo.*`, `fx.*`. Value roles carry a text
template (`"{ms:g} ms"`, `"{hz:g}"`, `"{db:.2f} dB"`, `"{lin:g}"`, `"{pct:g}"`, `"{s:g}"`, `"{n:g}"`)
rendered through the plugin's own display conversion, so no parameter curve has to be known. A test
whose role is missing is reported as skipped with the missing role.

### Rendering our engine

The plan is plugin-agnostic. `python3 refbench.py plan --out plan.json` lists every test with its
notes, timing, bends, ramps, role names and nominal values. A renderer (the engine's `crucible-measure`, see `tools/refbench.sh`) renders each test to `<test name>.wav` (48 kHz, stereo, 32-bit float) in
one folder, optionally with a `manifest.json` in the reference-bench format for CPU numbers, then:

```sh
python3 refbench.py analyze --renders ours/ --label "Crucible Engine" --out ours.json --plan plan.json
python3 refbench.py compare --result serum2.json --result pigments.json --candidate ours.json --out-dir out/
```

### Targets

`targets.json` holds the pass/fail rules (`best_reference` = equal to or better than the best
reference plus a margin; `absolute` and `range` = fixed bounds, used where the references cannot be
driven or where a physical definition exists; `info` = reported only). Metric paths address
`tests.<name>.<metric>` (test names may be globs, folded with `agg`: max, min, mean, max_abs) or
`derived.*`.

