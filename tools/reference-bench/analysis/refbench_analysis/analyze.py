"""Runs the analysers of a plan over a folder of WAV renders."""

from __future__ import annotations

import json
import math
import traceback
from pathlib import Path

import numpy as np

from . import measure, measure_analog
from .wavio import read_wav


def analyze_dir(plan: dict, renders: Path, label: str, profile: dict | None = None,
                skipped: dict | None = None) -> tuple[dict, dict]:
    manifest = {}
    man_path = renders / "manifest.json"
    plugin = None
    if man_path.exists():
        man = json.loads(man_path.read_text())
        plugin = man.get("plugin")
        for r in man.get("renders", []):
            manifest[r["name"]] = r
    ctx: dict = {"manifest": manifest}
    if profile:
        ctx["profile_eq_bands"] = profile.get("roles", {}).get("fx", {}).get("eq", {}).get("bands")

    results, plots, errors = {}, {}, {}
    skipped = dict(skipped or {})
    for test in plan["tests"]:
        name = test["name"]
        wav = renders / f"{name}.wav"
        if name in skipped:
            continue
        if not wav.exists():
            err = manifest.get(name, {}).get("error")
            skipped[name] = f"render failed: {err}" if err else "no render"
            continue
        try:
            sr, x = read_wav(wav)
            fn = measure.ANALYSERS[test["analysis"]["type"]]
            metrics, pdata = fn(test, sr, x, ctx)
            if name in manifest:
                rm = manifest[name]
                metrics.setdefault("render_cpu_percent", round(rm.get("cpuPercent", 0.0), 3))
                metrics["non_finite_samples"] = int(rm.get("nonFiniteSamples", 0))
                metrics["readback"] = {r["name"]: r["text"] for r in rm.get("readback", [])}
            results[name] = metrics
            for k, v in pdata.items():
                plots[f"{name}::{k}"] = np.asarray(v, dtype=np.float64)
        except Exception as e:  # keep going: one broken render must not hide the rest
            errors[name] = f"{type(e).__name__}: {e}"
            traceback.print_exc()

    derived = {}
    comp = profile.get("roles", {}).get("fx", {}).get("compressor") if profile else None
    curve = measure.compressor_curve(ctx, comp.get("threshold_db") if comp else None, comp.get("ratio") if comp else None)
    if curve:
        derived["compressor"] = curve
    derived.update(summaries(results))

    out = {
        "label": label,
        "plugin": plugin,
        "profile": profile.get("id") if profile else None,
        "profile_notes": profile.get("notes", []) if profile else [],
        "sample_rate": plan.get("sampleRate"),
        "tests": results,
        "derived": derived,
        "skipped": skipped,
        "errors": errors,
    }
    return out, plots


def summaries(results: dict) -> dict:
    """Cross-test tables: aliasing against pitch, tuning, filter tracking."""
    s: dict = {}
    for wave in ("saw", "square", "sine"):
        rows = []
        for name, m in sorted(results.items()):
            if name.startswith(f"osc_{wave}_n"):
                rows.append({"note": int(name.split("_n")[1]), "alias_audible_db": m.get("alias_audible_db"),
                             "worst_audible_spur_dbc": m.get("worst_audible_spur_dbc"),
                             "cents_error": m.get("cents_error"), "bandwidth_hz": m.get("bandwidth_hz")})
        if rows:
            rows.sort(key=lambda r: r["note"])
            s[f"osc_{wave}"] = rows
    # Filter tracking: resonant peak error (cents) and -3 dB ratio drift relative to 1 kHz.
    for ftype in ("lp12", "lp24", "hp12", "bp12", "ladder", "default"):
        rows = []
        base = results.get(f"filter_{ftype}_1k", {}).get("f3db_ratio")
        for fc in (250, 1000, 4000, 8000, 12000):
            r0 = results.get(f"filter_{ftype}_{fc}" if fc != 1000 else f"filter_{ftype}_1k", {})
            r9 = results.get(f"filter_{ftype}_{fc}_r90" if fc != 1000 else f"filter_{ftype}_1k_r90", {})
            if not r0 and not r9:
                continue
            row = {"cutoff_hz": fc, "peak_error_cents_r90": r9.get("cutoff_error_cents"),
                   "peak_gain_db_r90": r9.get("peak_gain_db")}
            if base and r0.get("f3db_ratio"):
                row["f3db_drift_cents"] = round(1200 * math.log2(r0["f3db_ratio"] / base), 2)
            rows.append(row)
        if len(rows) > 1:
            s[f"filter_tracking_{ftype}"] = rows
    cents = [abs(m["cents_error"]) for n, m in results.items() if n.startswith("osc_") and m.get("cents_error") is not None]
    if cents:
        s["tuning_max_abs_cents"] = round(max(cents), 4)
    s.update(measure_analog.summaries(results))
    return s


def save(out: dict, plots: dict, out_json: Path) -> None:
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(_clean(out), indent=1, sort_keys=False))
    if plots:
        np.savez_compressed(out_json.with_suffix(".plotdata.npz"), **plots)


def _clean(v):
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, (np.floating, float)):
        f = float(v)
        return f if math.isfinite(f) else None
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, np.bool_):
        return bool(v)
    return v
