"""Test plan: plugin-agnostic test definitions, turned into reference-bench jobs through a profile.

A test is a render (notes, duration, timing) plus an analysis recipe with nominal values. A profile
maps abstract roles ("saw", "filter.cutoff", "amp_env.attack", ...) onto one plugin's parameters.
Tests whose roles a profile lacks are listed as skipped with the missing role, so the report shows
exactly what could not be driven.
"""

from __future__ import annotations

import copy
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

SAMPLE_RATE = 48000
BLOCK_SIZE = 512

OSC_NOTES = [36, 60, 84, 96, 103, 108]  # C2, C4, C6, C7, G7, C8 (4186 Hz)
UNISON_VOICES = 8
CPU_REPEATS = 3  # best of three for timing tests
FILTER_NOTE = 12  # 16.35 Hz saw: harmonics every 16 Hz sample the filter response finely


class MissingRole(Exception):
    pass


@dataclass
class Test:
    name: str
    group: str
    seconds: float
    notes: list[dict]
    analysis: dict
    roles: list[str]  # role paths the test needs, in application order
    values: dict = field(default_factory=dict)  # role path -> nominal value for value specs
    ramps: list[dict] = field(default_factory=list)  # {"role": path, "start": s, "end": s}
    bends: list[dict] = field(default_factory=list)
    description: str = ""

    def spec(self) -> dict:
        """Plugin-agnostic description (what another renderer must reproduce)."""
        return {
            "name": self.name,
            "group": self.group,
            "description": self.description,
            "seconds": self.seconds,
            "sampleRate": SAMPLE_RATE,
            "notes": self.notes,
            "roles": self.roles,
            "values": self.values,
            "ramps": self.ramps,
            "bends": self.bends,
            "analysis": self.analysis,
        }


def note(pitch: int, start: float, length: float, velocity: int = 100) -> dict:
    return {"pitch": pitch, "start": start, "length": length, "velocity": velocity}


# --------------------------------------------------------------------------------------------
# Test definitions


def build_tests(include_pending: bool = False) -> list[Test]:
    t: list[Test] = []

    # Oscillators: steady single notes, analysed over a window that skips attack and release.
    for wave in ("saw", "square", "sine"):
        for n in OSC_NOTES:
            t.append(Test(
                name=f"osc_{wave}_n{n}", group="oscillator", seconds=1.6,
                notes=[note(n, 0.0, 1.5)], roles=[wave],
                analysis={"type": "osc", "wave": wave, "note": n, "window": [0.35, 1.45]},
                description=f"{wave} wave, single voice, MIDI note {n}"))

    # Unison: 8 voices at the plugin's default detune and at maximum detune.
    for n in (48, 96):
        t.append(Test(
            name=f"unison8_n{n}", group="unison", seconds=2.2, notes=[note(n, 0.0, 2.1)],
            roles=["saw", "unison", "unison.voices"], values={"unison.voices": UNISON_VOICES},
            analysis={"type": "unison", "note": n, "window": [0.5, 2.0], "voices": UNISON_VOICES},
            description=f"saw, {UNISON_VOICES} unison voices at the default detune, note {n}"))
    t.append(Test(
        name="unison8max_n48", group="unison", seconds=2.2, notes=[note(48, 0.0, 2.1)],
        roles=["saw", "unison", "unison.voices", "unison.detune_max"], values={"unison.voices": UNISON_VOICES},
        analysis={"type": "unison", "note": 48, "window": [0.5, 2.0], "voices": UNISON_VOICES},
        description=f"saw, {UNISON_VOICES} unison voices at maximum detune, note 48"))

    # Wavetable position sweep at high pitch: aliasing while morphing.
    for n in (84, 96):
        t.append(Test(
            name=f"wtsweep_n{n}", group="wavetable", seconds=4.4, notes=[note(n, 0.0, 4.3)],
            roles=["wavetable"], ramps=[{"role": "wavetable", "start": 0.2, "end": 4.2}],
            analysis={"type": "wtsweep", "note": n, "window": [0.2, 4.2]},
            description=f"wavetable position swept from start to end over 4 s, note {n}"))

    # Filters: harmonic transfer function of a low saw (32.7 Hz) against the filter-off render.
    t.append(Test(
        name="filter_ref", group="filter", seconds=2.2, notes=[note(FILTER_NOTE, 0.0, 2.1)],
        roles=["saw", "filter.open"], analysis={"type": "filter_ref", "note": FILTER_NOTE, "window": [0.4, 2.0]},
        description="filter reference: saw at 16.35 Hz with the filter bypassed or fully open"))

    def filt(name, ftype, cutoff, res, extra_roles=(), extra_values=None, analysis_extra=None, desc=""):
        values = {"filter.cutoff": cutoff, "filter.resonance": res}
        values.update(extra_values or {})
        a = {"type": "filter", "note": FILTER_NOTE, "window": [0.4, 2.0], "ref": "filter_ref", "filter": ftype,
             "cutoff": cutoff, "resonance": res}
        a.update(analysis_extra or {})
        t.append(Test(name=name, group="filter", seconds=2.2, notes=[note(FILTER_NOTE, 0.0, 2.1)],
                      roles=["saw", "filter.enable", f"filter.types.{ftype}", "filter.cutoff", "filter.resonance",
                             *extra_roles],
                      values=values, analysis=a, description=desc))

    for ftype in ("lp12", "lp24", "hp12", "bp12", "ladder", "default"):
        filt(f"filter_{ftype}_1k", ftype, 1000.0, 0.0, desc=f"{ftype} at 1 kHz, no resonance")
    for fc in (250.0, 4000.0, 8000.0, 12000.0):
        for ftype in ("lp24", "default"):
            filt(f"filter_{ftype}_{int(fc)}", ftype, fc, 0.0, desc=f"{ftype} at {fc:g} Hz (cutoff tracking)")
    for fc in (250.0, 4000.0, 8000.0, 12000.0):
        for ftype in ("lp24", "default"):
            filt(f"filter_{ftype}_{int(fc)}_r90", ftype, fc, 0.9,
                 desc=f"{ftype} at {fc:g} Hz, resonance 0.9 (resonant peak tracking)")
    for res in (0.5, 0.9, 1.0):
        for ftype in ("lp12", "lp24", "ladder", "default"):
            filt(f"filter_{ftype}_1k_r{int(res * 100)}", ftype, 1000.0, res,
                 desc=f"{ftype} at 1 kHz, resonance {res:g}")

    # Self-oscillation: a saw excites the filter, then the oscillator level drops to zero at 0.1 s;
    # a self-oscillating filter keeps ringing, a stable one decays.
    for ftype in ("lp12", "lp24", "ladder", "default"):
        t.append(Test(
            name=f"selfosc_{ftype}", group="filter", seconds=2.2, notes=[note(60, 0.0, 2.1)],
            roles=["saw", "filter.enable", f"filter.types.{ftype}", "filter.cutoff", "filter.resonance"],
            values={"filter.cutoff": 1000.0, "filter.resonance": 1.0},
            ramps=[{"role": "osc_kill", "start": 0.1, "end": 0.11}],
            analysis={"type": "selfosc", "cutoff": 1000.0, "window": [0.6, 2.0]},
            description=f"{ftype}: resonance 1, cutoff 1 kHz, oscillator faded out at 0.1 s"))

    # Drive: a sine through the low-pass well below cutoff; distortion versus drive.
    for drive in (0.0, 0.5, 1.0):
        ftype = "lp24"
        for ft in (ftype, "default"):
            t.append(Test(
                name=f"drive_{ft}_d{int(drive * 100)}", group="filter", seconds=1.6, notes=[note(48, 0.0, 1.5)],
                roles=["sine", "filter.enable", f"filter.types.{ft}", "filter.cutoff", "filter.resonance",
                       "filter.drive"],
                values={"filter.cutoff": 8000.0, "filter.resonance": 0.0, "filter.drive": drive},
                analysis={"type": "drive", "note": 48, "drive": drive, "window": [0.4, 1.4]},
                description=f"{ft} at 8 kHz, sine at note 48, drive {drive:g}"))

    # Amplitude envelope.
    env = {"amp_env.attack": 0.1, "amp_env.decay": 0.3, "amp_env.sustain": 0.5, "amp_env.release": 0.4}
    t.append(Test(
        name="env_adsr", group="envelope", seconds=2.2, notes=[note(69, 0.1, 1.0)],
        roles=["sine", "amp_env.attack", "amp_env.decay", "amp_env.sustain", "amp_env.release"], values=env,
        analysis={"type": "envelope", "note_on": 0.1, "note_off": 1.1, **{k.split(".")[1]: v for k, v in env.items()}},
        description="sine A4, attack 100 ms, decay 300 ms, sustain 50 % (-6 dB), release 400 ms"))
    env_long = {"amp_env.attack": 1.0, "amp_env.decay": 1.0, "amp_env.sustain": 0.25, "amp_env.release": 1.5}
    t.append(Test(
        name="env_long", group="envelope", seconds=6.0, notes=[note(69, 0.1, 3.0)],
        roles=["sine", "amp_env.attack", "amp_env.decay", "amp_env.sustain", "amp_env.release"], values=env_long,
        analysis={"type": "envelope", "note_on": 0.1, "note_off": 3.1,
                  **{k.split(".")[1]: v for k, v in env_long.items()}},
        description="sine A4, attack 1 s, decay 1 s, sustain 25 % (-12 dB), release 1.5 s"))
    t.append(Test(
        name="env_fast", group="envelope", seconds=1.0, notes=[note(69, 0.1, 0.3)],
        roles=["sine", "amp_env.attack", "amp_env.release"], values={"amp_env.attack": 0.0, "amp_env.release": 0.001},
        analysis={"type": "onset", "note_on": 0.1, "note_off": 0.4},
        description="sine A4 with the fastest attack and release: onset latency and clicks"))

    # Velocity response (default velocity routing of the init patch).
    vel_notes = [note(69, 0.1 + 0.5 * i, 0.3, v) for i, v in enumerate((127, 100, 64, 32, 8))]
    t.append(Test(
        name="velocity", group="envelope", seconds=2.8, notes=vel_notes, roles=["saw"],
        analysis={"type": "velocity", "notes": vel_notes},
        description="saw A4 at velocities 127, 100, 64, 32, 8"))

    # Pitch bend accuracy (bend range 2 semitones).
    t.append(Test(
        name="bend", group="pitch", seconds=2.4, notes=[note(69, 0.0, 2.3)], roles=["sine", "bend_range"],
        bends=[{"time": 0.8, "value": 1.0}, {"time": 1.5, "value": -1.0}],
        analysis={"type": "bend", "note": 69, "segments": [[0.3, 0.75, 0.0], [1.0, 1.45, 2.0], [1.7, 2.2, -2.0]]},
        description="sine A4, pitch bend +max at 0.8 s and -max at 1.5 s with a 2 semitone range"))

    # Idle output and the release tail.
    t.append(Test(name="idle", group="noise", seconds=2.0, notes=[], roles=["saw"],
                  analysis={"type": "idle", "window": [0.0, 2.0]},
                  description="no notes: output noise floor"))
    t.append(Test(name="tail", group="noise", seconds=3.0, notes=[note(60, 0.0, 0.2)], roles=["saw"],
                  analysis={"type": "idle", "window": [1.0, 3.0]},
                  description="0.2 s saw note, then the residual output 0.8 s to 2.8 s after release"))

    # LFO (needs a routing role; references do not expose one).
    for shape in ("sine", "triangle", "saw", "square"):
        t.append(Test(
            name=f"lfo_{shape}_2hz", group="lfo", seconds=3.2, notes=[note(69, 0.0, 3.1)],
            roles=["sine", "lfo", "lfo.rate", f"lfo.shape.{shape}"], values={"lfo.rate": 2.0},
            analysis={"type": "lfo", "shape": shape, "rate": 2.0, "window": [0.5, 3.0]},
            description=f"LFO {shape} at 2 Hz routed to amplitude"))
    t.append(Test(
        name="lfo_sine_9hz", group="lfo", seconds=2.2, notes=[note(69, 0.0, 2.1)],
        roles=["sine", "lfo", "lfo.rate", "lfo.shape.sine"], values={"lfo.rate": 9.0},
        analysis={"type": "lfo", "shape": "sine", "rate": 9.0, "window": [0.3, 2.0]},
        description="LFO sine at 9 Hz routed to amplitude"))

    # FX (need FX roles; references do not expose FX slots).
    pluck = [note(60, 0.1, 0.02)]
    for decay in (1.0, 3.0):
        t.append(Test(
            name=f"fx_reverb_{decay:g}s", group="fx", seconds=1.0 + 2.5 * decay, notes=pluck,
            roles=["saw", "fx.reverb", "fx.reverb.decay"], values={"fx.reverb.decay": decay},
            analysis={"type": "reverb", "excite_end": 0.14, "nominal_rt60": decay},
            description=f"reverb 100 % wet, nominal decay {decay:g} s, 20 ms saw pluck"))
    t.append(Test(
        name="fx_delay", group="fx", seconds=3.0, notes=pluck,
        roles=["saw", "fx.delay", "fx.delay.time", "fx.delay.feedback"],
        values={"fx.delay.time": 0.25, "fx.delay.feedback": 0.5},
        analysis={"type": "delay", "excite": 0.1, "time": 0.25, "feedback": 0.5},
        description="delay 250 ms, feedback 0.5, 100 % wet, 20 ms saw pluck"))
    t.append(Test(
        name="fx_chorus", group="fx", seconds=2.2, notes=[note(48, 0.0, 2.1)], roles=["saw", "fx.chorus"],
        analysis={"type": "chorus", "note": 48, "window": [0.5, 2.0]},
        description="chorus on a saw at note 48"))
    t.append(Test(
        name="fx_eq_ref", group="fx", seconds=2.2, notes=[note(FILTER_NOTE, 0.0, 2.1)], roles=["saw"],
        analysis={"type": "filter_ref", "note": FILTER_NOTE, "window": [0.4, 2.0]},
        description="EQ reference: saw at 16.35 Hz, EQ off"))
    t.append(Test(
        name="fx_eq_6db", group="fx", seconds=2.2, notes=[note(FILTER_NOTE, 0.0, 2.1)],
        roles=["saw", "fx.eq", "fx.eq.bands"], values={"fx.eq.gain_db": 6.0},
        analysis={"type": "eq", "note": FILTER_NOTE, "window": [0.4, 2.0], "ref": "fx_eq_ref", "gain_db": 6.0},
        description="EQ with every band at +6 dB at its nominal frequency"))
    for i, level in enumerate((0.03, 0.06, 0.125, 0.25, 0.5, 1.0)):
        for on in (False, True):
            name = f"fx_comp_{'on' if on else 'off'}_{i}"
            t.append(Test(
                name=name, group="fx", seconds=1.6, notes=[note(57, 0.0, 1.5)],
                roles=["sine", "fx.compressor.input"] + (["fx.compressor"] if on else []),
                values={"fx.compressor.input": level},
                analysis={"type": "comp_point", "step": i, "on": on, "window": [0.6, 1.4]},
                description=f"compressor {'on' if on else 'off'}, input level step {i}"))

    # CPU: render time against audio time.
    chord8 = [note(p, 0.0, 4.5) for p in (48, 52, 55, 59, 60, 64, 67, 71)]
    chord16 = chord8 + [note(p, 0.0, 4.5) for p in (36, 40, 43, 47, 72, 76, 79, 83)]
    t.append(Test(name="cpu_1voice", group="cpu", seconds=5.0, notes=[note(60, 0.0, 4.5)], roles=["saw"],
                  analysis={"type": "cpu", "voices": 1, "unison": 1, "repeat": CPU_REPEATS},
                  description="1 note, saw, 1 unison voice"))
    t.append(Test(name="cpu_8voices", group="cpu", seconds=5.0, notes=chord8, roles=["saw"],
                  analysis={"type": "cpu", "voices": 8, "unison": 1, "repeat": CPU_REPEATS},
                  description="8-note chord, saw"))
    t.append(Test(name="cpu_8x8unison", group="cpu", seconds=5.0, notes=chord8,
                  roles=["saw", "unison", "unison.voices"], values={"unison.voices": 8},
                  analysis={"type": "cpu", "voices": 8, "unison": 8, "repeat": CPU_REPEATS},
                  description="8-note chord, saw, 8 unison"))
    t.append(Test(name="cpu_16x8unison", group="cpu", seconds=5.0, notes=chord16,
                  roles=["saw", "unison", "unison.voices"], values={"unison.voices": 8},
                  analysis={"type": "cpu", "voices": 16, "unison": 8, "repeat": CPU_REPEATS},
                  description="16-note chord, saw, 8 unison"))
    t.append(Test(name="cpu_8x8_filter", group="cpu", seconds=5.0, notes=chord8,
                  roles=["saw", "unison", "unison.voices", "filter.enable", "filter.types.lp24", "filter.cutoff",
                         "filter.resonance"],
                  values={"unison.voices": 8, "filter.cutoff": 2000.0, "filter.resonance": 0.5},
                  analysis={"type": "cpu", "voices": 8, "unison": 8, "repeat": CPU_REPEATS},
                  description="8-note chord, saw, 8 unison, 24 dB low-pass"))
    if include_pending:  # analog-behaviour tests no reference or engine can run yet (targets marked pending)
        from .suite_analog import build_pending_tests
        t.extend(build_pending_tests())
    return t


# --------------------------------------------------------------------------------------------
# Profiles


def load_profile(path: str | Path) -> dict:
    p = json.loads(Path(path).read_text())
    plugin = os.path.expandvars(p.get("plugin", ""))
    p["plugin"] = plugin
    return p


def lookup(roles: dict, path: str):
    node = roles
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            raise MissingRole(path)
        node = node[part]
    return node


def value_fields(v: float) -> dict:
    """Every unit a text template may use for one nominal value."""
    return {
        "v": v, "n": v, "s": v, "ms": v * 1000.0, "hz": v, "lin": v, "pct": v * 100.0,
        "db": 20.0 * math.log10(v) if v > 0 else -120.0,
    }


def render_spec(spec: dict, v: float) -> dict:
    """{"param", "text"} template + nominal value -> {param: text-or-number}."""
    if "text" in spec:
        return {spec["param"]: spec["text"].format(**value_fields(v))}
    if "value" in spec:
        return {spec["param"]: spec["value"]}
    raise MissingRole(f"value spec for {spec.get('param')} has no text")


def resolve_sets(profile: dict, test: Test) -> tuple[dict, list[dict]]:
    """Parameter assignments (in order) and ramps for one test, or MissingRole.

    Role nodes are one of: a value spec {"param", "text"} (needs a nominal value from the test),
    a node with "set" (applied as is), or a plain {parameter: value} map (applied as is).
    """
    roles = profile.get("roles", {})
    sets: dict = dict(profile.get("base", {}))
    for path in test.roles:
        if path == "filter.open":  # optional: profiles whose base already bypasses the filter omit it
            sets.update(roles.get("filter", {}).get("open", {}))
            continue
        if path.startswith("lfo.shape."):
            spec = lookup(roles, "lfo.shape")
            shape = path.split(".")[-1]
            if shape not in spec.get("values", {}):
                raise MissingRole(path)
            sets[spec["param"]] = spec["values"][shape]
            continue
        node = lookup(roles, path)
        if path == "fx.eq.bands":
            gain = test.values.get("fx.eq.gain_db", 6.0)
            for band in node:
                sets.update(render_spec(band["gain"], 10 ** (gain / 20.0)))
                if "freqParam" in band:
                    sets.update(render_spec(band["freqParam"], band["freq"]))
            continue
        if path in test.values:
            sets.update(render_spec(node, test.values[path]))
        elif isinstance(node, dict) and "from" in node and "to" in node:  # ramp spec: see test.ramps
            sets.update(node.get("set", {}))
        elif isinstance(node, dict) and "set" in node:
            sets.update(node["set"])
        elif isinstance(node, dict) and "param" in node and "text" in node:
            raise MissingRole(f"{path} (needs a value)")
        elif isinstance(node, dict) and all(not isinstance(v, (dict, list)) for v in node.values()):
            sets.update(node)
    ramps = []
    for r in test.ramps:
        node = lookup(roles, r["role"])
        ramps.append({"param": node["param"], "from": node["from"], "to": node["to"], "start": r["start"],
                      "end": r["end"]})
    return sets, ramps


def make_job(profile: dict, tests: list[Test]) -> tuple[dict, dict]:
    renders, skipped = [], {}
    for t in tests:
        try:
            sets, ramps = resolve_sets(profile, t)
        except MissingRole as e:
            skipped[t.name] = f"role '{e.args[0]}' not available in profile {profile['id']}"
            continue
        r = {"name": t.name, "seconds": t.seconds, "set": sets, "notes": t.notes}
        if ramps:
            r["ramps"] = ramps
        if t.bends:
            r["bend"] = t.bends
        renders.append(r)
        # Timing tests are repeated; the analysis keeps the best run (least disturbed by other load).
        for k in range(2, t.analysis.get("repeat", 1) + 1):
            renders.append({**r, "name": f"{t.name}__r{k}"})
    job = {"sampleRate": SAMPLE_RATE, "blockSize": BLOCK_SIZE, "warmup": 0.5, "renders": renders}
    return job, skipped


def plan(tests: list[Test]) -> dict:
    return {"sampleRate": SAMPLE_RATE, "tests": [t.spec() for t in tests]}


def tests_by_name(tests: list[Test]) -> dict[str, Test]:
    return {t.name: t for t in tests}


def filter_tests(tests: list[Test], groups: list[str] | None, names: list[str] | None) -> list[Test]:
    """Selects tests by group and/or name, adding the reference renders they compare against."""
    selected = [t for t in tests if (not groups or t.group in groups) and (not names or t.name in names)]
    wanted = {t.name for t in selected} | {t.analysis["ref"] for t in selected if t.analysis.get("ref")}
    return copy.deepcopy([t for t in tests if t.name in wanted])
