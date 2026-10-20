"""Comparison report: reference plugins against a candidate (our engine), with pass/fail targets.

Targets (targets.json) are rules over metrics found in the results files:

  {"id", "label", "group", "metric" | "metrics", "agg", "better": "lower"|"higher", "abs": bool,
   "rule": "best_reference" | "reference:<id>" | "absolute" | "range" | "info",
   "margin", "threshold", "range": [lo, hi], "unit", "why"}

`metric` is a dotted path such as "tests.osc_saw_n96.alias_audible_db"; test names may use shell
globs ("tests.osc_saw_n*.alias_audible_db") and `agg` (max, min, mean, max_abs) folds the matches.
"""

from __future__ import annotations

import datetime as _dt
import fnmatch
import json
import math
from pathlib import Path

import numpy as np

from . import plot


# --------------------------------------------------------------------------------------------
# Loading


class Result:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.data = json.loads(self.path.read_text())
        npz = self.path.with_suffix(".plotdata.npz")
        self.plots = dict(np.load(npz)) if npz.exists() else {}
        self.id = self.data.get("profile") or self.path.parent.name
        self.label = self.data.get("label") or self.id

    def series(self, test: str, key: str):
        return self.plots.get(f"{test}::{key}")


def _get(d, path: str):
    node = d
    for part in path.split("."):
        if isinstance(node, dict) and part in node:
            node = node[part]
        else:
            return None
    return node


def metric_values(result: Result, target: dict) -> list[float]:
    paths = target.get("metrics") or [target["metric"]]
    vals = []
    for p in paths:
        head, _, rest = p.partition(".")
        if head == "tests" and any(c in rest.split(".")[0] for c in "*?["):
            pattern, _, key = rest.partition(".")
            for name in sorted(result.data.get("tests", {})):
                if fnmatch.fnmatch(name, pattern):
                    v = _get(result.data["tests"][name], key)
                    if isinstance(v, (int, float)) and not isinstance(v, bool):
                        vals.append(float(v))
        else:
            v = _get(result.data, p)
            if isinstance(v, bool):
                v = float(v)
            if isinstance(v, (int, float)):
                vals.append(float(v))
    return vals


def metric(result: Result, target: dict) -> float | None:
    vals = metric_values(result, target)
    if not vals:
        return None
    if target.get("abs"):
        vals = [abs(v) for v in vals]
    agg = target.get("agg", "max" if len(vals) > 1 else None)
    if agg in (None, "first"):
        return vals[0]
    if agg == "max_abs":
        return max(abs(v) for v in vals)
    if agg == "max":
        return max(vals)
    if agg == "min":
        return min(vals)
    if agg == "mean":
        return float(np.mean(vals))
    raise ValueError(f"unknown aggregate {agg}")


def evaluate(target: dict, refs: list[Result], cand: Result | None) -> dict:
    if target.get("status") == "pending":
        # Defined and analysable, but not yet scored: neither a pass nor a fail until the behaviour exists.
        cv = metric(cand, target) if cand else None
        required = float(target["threshold"]) if "threshold" in target else None
        return {"refs": {}, "required": required, "basis": "pending", "candidate": cv, "verdict": "pending"}
    lower = target.get("better", "lower") == "lower"
    rule = target.get("rule", "best_reference")
    margin = float(target.get("margin", 0.0))
    ref_vals = {r.id: metric(r, target) for r in refs}
    present = {k: v for k, v in ref_vals.items() if v is not None}
    required = None
    basis = ""
    if rule == "best_reference" and present:
        best_id = min(present, key=present.get) if lower else max(present, key=present.get)
        best = present[best_id]
        required = best + margin if lower else best - margin
        basis = f"best reference ({best_id})"
    elif rule.startswith("reference:"):
        rid = rule.split(":", 1)[1]
        if ref_vals.get(rid) is not None:
            required = ref_vals[rid] + margin if lower else ref_vals[rid] - margin
            basis = rid
    elif rule == "absolute":
        required = float(target["threshold"])
        basis = "absolute"
    elif rule == "range":
        basis = "range"
    elif rule == "info":
        basis = "informational"

    cv = metric(cand, target) if cand else None
    verdict = "n/a"
    if cv is not None:
        if rule == "range":
            lo, hi = target["range"]
            verdict = "PASS" if lo <= cv <= hi else "FAIL"
        elif rule == "info":
            verdict = "info"
        elif required is not None:
            verdict = "PASS" if ((cv <= required) if lower else (cv >= required)) else "FAIL"
        else:
            verdict = "no reference"
    elif rule != "info":
        verdict = "not measured"
    return {"refs": ref_vals, "required": required, "basis": basis, "candidate": cv, "verdict": verdict}


def _fmt(v, unit=""):
    if v is None:
        return "-"
    if isinstance(v, float):
        if abs(v) >= 1000:
            s = f"{v:,.0f}"
        elif abs(v) >= 100:
            s = f"{v:.1f}"
        elif abs(v) >= 1:
            s = f"{v:.2f}"
        else:
            s = f"{v:.3f}"
    else:
        s = str(v)
    return f"{s} {unit}".strip()


# --------------------------------------------------------------------------------------------
# Charts


def _charts(all_results: list[Result], out: Path) -> list[tuple[str, str]]:
    made: list[tuple[str, str]] = []
    pdir = out / "plots"

    def add(chart, name, caption):
        if chart.series or chart.bars:
            plot.save(chart, pdir / name)
            made.append((f"plots/{name}.png", caption))

    # Aliasing and bandwidth against pitch.
    for wave in ("saw", "square", "sine"):
        c = plot.Chart(f"{wave}: aliasing below 20 kHz vs pitch", "MIDI note", "inharmonic re harmonic energy (dB)")
        c2 = plot.Chart(f"{wave}: worst spur below 20 kHz vs pitch", "MIDI note", "dBc")
        for i, r in enumerate(all_results):
            rows = r.data.get("derived", {}).get(f"osc_{wave}") or []
            if rows:
                xs = [row["note"] for row in rows]
                c.line(xs, [_nan(row["alias_audible_db"]) for row in rows], r.label, plot.PALETTE[i], points=True)
                c2.line(xs, [_nan(row["worst_audible_spur_dbc"]) for row in rows], r.label, plot.PALETTE[i],
                        points=True)
        add(c, f"alias_{wave}", f"{wave}: aliasing below 20 kHz relative to the harmonic energy (lower is better)")
        add(c2, f"spur_{wave}", f"{wave}: largest single inharmonic component below 20 kHz (lower is better)")
    c = plot.Chart("saw: harmonic bandwidth vs pitch", "MIDI note", "last harmonic within 3 dB of ideal (Hz)",
                   ylim=(0, 24000))
    for i, r in enumerate(all_results):
        rows = r.data.get("derived", {}).get("osc_saw") or []
        if rows:
            c.line([row["note"] for row in rows], [_nan(row["bandwidth_hz"]) for row in rows], r.label,
                   plot.PALETTE[i], points=True)
    add(c, "bandwidth_saw", "saw: how far up the spectrum the harmonics stay at full level (higher is brighter)")

    # Spectra.
    for test, title in (("osc_saw_n96", "saw C7 (2093 Hz)"), ("osc_saw_n108", "saw C8 (4186 Hz)"),
                        ("osc_saw_n36", "saw C2 (65 Hz)"), ("unison8_n96", "saw C7, 8 unison voices")):
        c = plot.Chart(f"spectrum: {title}", "frequency (Hz)", "dB re fundamental", xlog=True, xlim=(20, 24000),
                       ylim=(-160, 5))
        for i, r in enumerate(all_results):
            f, y = r.series(test, "spectrum_f"), r.series(test, "spectrum_db")
            if f is not None:
                c.line(f, y, r.label, plot.PALETTE[i], width=1.0)
        add(c, f"spectrum_{test}", f"max-hold spectrum, {title}")

    # Wavetable sweep.
    for n in (84, 96):
        c = plot.Chart(f"wavetable sweep, note {n}: aliasing vs position", "table position (0..1)",
                       "inharmonic re harmonic energy (dB)", xlim=(0, 1))
        for i, r in enumerate(all_results):
            p, a = r.series(f"wtsweep_n{n}", "position"), r.series(f"wtsweep_n{n}", "alias_db")
            if p is not None:
                c.line(p, a, r.label, plot.PALETTE[i], width=1.0)
        add(c, f"wtsweep_n{n}", f"aliasing in 43 ms frames while the table position sweeps (note {n})")

    # Filters.
    for ftype, title in (("lp12", "low-pass 12 dB"), ("lp24", "low-pass 24 dB"), ("ladder", "ladder low-pass"),
                         ("hp12", "high-pass 12 dB"), ("bp12", "band-pass 12 dB"),
                         ("default", "Pigments default F1")):
        c = plot.Chart(f"filter: {title}, cutoff 1 kHz", "frequency (Hz)", "dB re passband", xlog=True,
                       xlim=(16, 20000), ylim=(-80, 50))
        for i, r in enumerate(all_results):
            for res, dash in (("", False), ("_r90", True)):
                f, y = r.series(f"filter_{ftype}_1k{res}", "freq"), r.series(f"filter_{ftype}_1k{res}", "response_db")
                if f is not None:
                    c.line(f, y, f"{r.label} res {'0.9' if res else '0'}", plot.PALETTE[i], width=1.4, dashed=dash)
        c.vlines.append((1000.0, "1 kHz"))
        add(c, f"filter_{ftype}", f"{title}: response, resonance 0 (solid) and 0.9 (dashed)")
    c = plot.Chart("resonant peak tracking (resonance 0.9)", "cutoff (Hz)", "peak frequency error (cents)",
                   xlog=True, xlim=(200, 15000))
    for i, r in enumerate(all_results):
        for ftype in ("lp24", "default"):
            rows = r.data.get("derived", {}).get(f"filter_tracking_{ftype}")
            if rows:
                c.line([row["cutoff_hz"] for row in rows], [_nan(row["peak_error_cents_r90"]) for row in rows],
                       f"{r.label} {ftype}", plot.PALETTE[i], points=True)
    c.hlines.append((0.0, ""))
    add(c, "filter_tracking", "resonant peak frequency against the nominal cutoff (0 = exact; 250 Hz is limited "
                              "by the 16 Hz harmonic spacing)")
    c = plot.Chart("resonance: peak gain vs amount (1 kHz)", "resonance (normalised)", "peak re passband (dB)")
    for i, r in enumerate(all_results):
        for ftype in ("lp24", "default", "ladder"):
            xs, ys = [], []
            for res, suffix in ((0.0, ""), (0.5, "_r50"), (0.9, "_r90"), (1.0, "_r100")):
                m = r.data.get("tests", {}).get(f"filter_{ftype}_1k{suffix}")
                if m and m.get("peak_gain_db") is not None:
                    xs.append(res)
                    ys.append(m["peak_gain_db"])
            if xs:
                c.line(xs, ys, f"{r.label} {ftype}", plot.PALETTE[i], points=True, dashed=ftype == "ladder")
    add(c, "resonance_curve", "resonant peak height against the resonance control")

    # Envelopes.
    for test, title in (("env_adsr", "A 100 ms, D 300 ms, S 50 %, R 400 ms"),
                        ("env_long", "A 1 s, D 1 s, S 25 %, R 1.5 s")):
        c = plot.Chart(f"amp envelope: {title}", "time from note-on (s)", "dB re peak", ylim=(-80, 3))
        for i, r in enumerate(all_results):
            t, e = r.series(test, "t"), r.series(test, "env_db")
            if t is not None:
                c.line(t, e, r.label, plot.PALETTE[i], width=1.2)
        add(c, test, f"amplitude envelope, {title}")

    c = plot.Chart("velocity response of the default patch", "velocity", "dB re velocity 127", ylim=(-40, 3))
    for i, r in enumerate(all_results):
        v, lv = r.series("velocity", "velocity"), r.series("velocity", "level_db")
        if v is not None:
            c.line(v, lv, r.label, plot.PALETTE[i], points=True)
    add(c, "velocity", "output level against note velocity (default patch routing)")

    tests = ["cpu_1voice", "cpu_8voices", "cpu_8x8unison", "cpu_16x8unison", "cpu_8x8_filter"]
    groups = []
    for t in tests:
        vals = [r.data.get("tests", {}).get(t, {}).get("cpu_percent_all_threads") for r in all_results]
        groups.append((t.replace("cpu_", ""), [_nan(v) for v in vals], [r.label for r in all_results]))
    c = plot.Chart("CPU (48 kHz, 512-sample blocks, best of 3)", "", "% of one core (all threads)")
    c.bars = groups
    c.ylim = (0, max([v for g in groups for v in g[1] if math.isfinite(v)] + [1]) * 1.15)
    add(c, "cpu", "process CPU time of all threads / audio time (lower is better)")

    groups = []
    for t in ("unison8_n48", "unison8_n96", "unison8max_n48"):
        vals = [r.data.get("tests", {}).get(t, {}).get("correlation") for r in all_results]
        groups.append((t, [_nan(v) for v in vals], [r.label for r in all_results]))
    c = plot.Chart("unison: L/R correlation (1 = mono)", "", "correlation", ylim=(-0.3, 1.05))
    c.bars = groups
    add(c, "unison_correlation", "stereo correlation of 8-voice unison")

    c = plot.Chart("reverb energy decay curves", "time after excitation (s)", "dB", ylim=(-80, 2))
    for i, r in enumerate(all_results):
        for test in ("fx_reverb_1s", "fx_reverb_3s"):
            t, e = r.series(test, "t"), r.series(test, "edc_db")
            if t is not None:
                c.line(t, e, f"{r.label} {test[10:]}", plot.PALETTE[i], width=1.2, dashed=test.endswith("3s"))
    add(c, "reverb_edc", "Schroeder energy decay of the reverb (nominal 1 s solid, 3 s dashed)")
    return made


def _nan(v):
    return float("nan") if v is None else float(v)


# --------------------------------------------------------------------------------------------
# Markdown


def compare(result_paths: list[str], candidate_path: str | None, targets: dict, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    refs = [Result(Path(p)) for p in result_paths]
    cand = Result(Path(candidate_path)) if candidate_path else None
    everyone = refs + ([cand] if cand else [])

    data_dir = out / "data"
    data_dir.mkdir(exist_ok=True)
    for r in everyone:
        # Committed copies keep the bundle name only, not a machine-specific path.
        d = json.loads(json.dumps(r.data))
        if isinstance(d.get("plugin"), dict) and d["plugin"].get("file"):
            d["plugin"]["file"] = Path(d["plugin"]["file"]).name
        (data_dir / f"{r.id}.json").write_text(json.dumps(d, indent=1))

    charts = _charts(everyone, out)
    evals = [(t, evaluate(t, refs, cand)) for t in targets["targets"]]
    (out / "targets_evaluated.json").write_text(json.dumps(
        [{"id": t["id"], "label": t["label"], **e} for t, e in evals], indent=1))

    L: list[str] = []
    L.append("# Reference bench: measured comparison")
    L.append("")
    L.append(f"Generated {_dt.date.today().isoformat()} by `tools/reference-bench/analysis/refbench.py compare`. "
             "All numbers come from offline renders measured as black boxes; no audio is stored here.")
    L.append("")
    L.append("| | plugin | version | format | role |")
    L.append("|---|---|---|---|---|")
    for r in everyone:
        p = r.data.get("plugin") or {}
        role = "candidate" if r is cand else "reference"
        L.append(f"| {r.label} | {p.get('name', '-')} | {p.get('version', '-')} | {p.get('format', '-')} | {role} |")
    L.append("")
    sr = everyone[0].data.get("sample_rate") if everyone else None
    L.append(f"Sample rate {sr} Hz, 512-sample blocks, realtime processing mode (not the offline flag); CPU is "
             "the wall time of `processBlock` on an Apple M1 Max.")
    L.append("")

    if cand:
        passed = sum(1 for _, e in evals if e["verdict"] == "PASS")
        failed = sum(1 for _, e in evals if e["verdict"] == "FAIL")
        other = sum(1 for _, e in evals if e["verdict"] not in ("PASS", "FAIL", "info", "pending"))
        L.append(f"**Candidate {cand.label}: {passed} pass, {failed} fail, {other} without a reference or "
                 f"measurement.**")
        L.append("")

    L.append("## Targets")
    L.append("")
    L.append("`required` is what our engine must reach: the best reference (adjusted by the margin) for "
             "`best_reference` rules, a fixed bound for `absolute` and `range` rules.")
    group = None
    head = "| target | " + " | ".join(r.label for r in refs) + " | required | rule | " + \
           (f"{cand.label} | verdict |" if cand else "")
    sep = "|---" * (len(refs) + 3 + (2 if cand else 0)) + "|"
    for t, e in evals:
        if t.get("group") != group:
            group = t.get("group")
            L.append("")
            L.append(f"### {group}")
            L.append("")
            L.append(head)
            L.append(sep)
        unit = t.get("unit", "")
        rule = t.get("rule", "best_reference")
        req = f"{t['range'][0]} .. {t['range'][1]} {unit}".strip() if rule == "range" else _fmt(e["required"], unit)
        better = "higher" if t.get("better") == "higher" else "lower"
        rule_txt = rule if rule in ("range", "info") else f"{rule}, {better} is better"
        if t.get("margin"):
            rule_txt += f", margin {t['margin']}"
        row = f"| {t['label']} | " + " | ".join(_fmt(e["refs"].get(r.id), unit) for r in refs) + \
              f" | {req} | {rule_txt} |"
        if cand:
            row += f" {_fmt(e['candidate'], unit)} | {e['verdict']} |"
        L.append(row)
    L.append("")
    whys = [(t["label"], t["why"]) for t in targets["targets"] if t.get("why")]
    if whys:
        L.append("Notes on targets:")
        L.append("")
        for label, why in whys:
            L.append(f"- *{label}*: {why}")
        L.append("")

    L.append("## Charts")
    L.append("")
    for path, caption in charts:
        L.append(f"![{caption}]({path})")
        L.append("")
        L.append(f"*{caption}*")
        L.append("")

    L.append("## Coverage")
    L.append("")
    for r in everyone:
        d = r.data
        L.append(f"### {r.label}")
        L.append("")
        for note in d.get("profile_notes", []):
            L.append(f"- {note}")
        L.append(f"- {len(d.get('tests', {}))} tests measured, {len(d.get('skipped', {}))} skipped, "
                 f"{len(d.get('errors', {}))} analysis errors.")
        reasons: dict[str, list[str]] = {}
        for name, why in d.get("skipped", {}).items():
            reasons.setdefault(why, []).append(name)
        for why, names in sorted(reasons.items()):
            L.append(f"- skipped ({why}): {', '.join(sorted(names))}")
        for name, err in d.get("errors", {}).items():
            L.append(f"- error in {name}: {err}")
        L.append("")
    L.append("Per-test numbers: `data/<plugin>.json`; evaluated targets: `targets_evaluated.json`.")
    L.append("")
    (out / "REPORT.md").write_text("\n".join(L))
    print(f"report -> {out / 'REPORT.md'} ({len(charts)} charts)")
