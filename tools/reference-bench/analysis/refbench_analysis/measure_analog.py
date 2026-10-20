"""Analysers for analog behaviour: filter self-oscillation tracking, resonance bass loss and level dependence,
saturation harmonics, drift statistics, RC envelope fits and voice-to-voice variance.

These are the analysis types of the community-engine verification plan (docs/plans/ENGINE-COMMUNITY-PLAN.md,
section 3.2). Their tests live in suite_analog.py and are marked pending: no reference plugin has been driven
through them yet, and the engine does not implement the behaviours they check. The analysers themselves are
complete and are exercised on synthetic renders by selftest.py. Same contract as measure.py: each function
returns (metrics, plot_data).
"""

from __future__ import annotations

import math

import numpy as np

from . import dsp
from .measure import _rd


def _rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.asarray(x, dtype=np.float64) ** 2))) if len(x) else 0.0


def _tone_amp(x: np.ndarray, sr: int, hz: float) -> float:
    """Amplitude of the sinusoid at hz (windowed DTFT, so whole periods are not required)."""
    n = len(x)
    w = np.blackman(n)
    t = np.arange(n) / sr
    c = np.sum(x * w * np.exp(-2j * math.pi * hz * t))
    return float(2.0 * abs(c) / w.sum())


# --------------------------------------------------------------------------------------------
# Self-oscillation tracking


def selfosc_track(test, sr, x, ctx):
    """A resonant filter left ringing after the oscillator is cut: pitch against the cutoff setting and steadiness."""
    a = test["analysis"]
    seg = dsp.segment(dsp.mono(x), sr, *a["window"])
    level = dsp.db(_rms(seg))
    pieces = np.array_split(seg, 8)
    levels = [dsp.db(_rms(p)) for p in pieces]
    out = {"level_dbfs": level, "amplitude_variation_db": max(levels) - min(levels)}
    out["oscillating"] = bool(level > -60.0 and abs(levels[-1] - levels[0]) < 3.0)
    if level > -100.0:
        f0 = dsp.estimate_f0(seg, sr, a["cutoff"])
        out["freq_hz"] = f0
        out["cents_error"] = dsp.cents(f0 / a["cutoff"])
    return _rd(out), {}


# --------------------------------------------------------------------------------------------
# Resonance: bass loss and level dependence


def _gain_against_ref(test, sr, x, ctx, key):
    a = test["analysis"]
    seg = dsp.segment(dsp.mono(x), sr, *a["window"])
    hz = a.get("freq_hz") or dsp.midi_to_hz(a["note"])
    amp = _tone_amp(seg, sr, hz)
    refs = ctx.setdefault(key, {})
    refs[test["name"]] = amp
    ref = refs.get(a.get("ref"))
    return amp, ref


def res_bassloss(test, sr, x, ctx):
    """Gain of a low sine through the filter against the same sine with the filter open (the reference render)."""
    a = test["analysis"]
    amp, ref = _gain_against_ref(test, sr, x, ctx, "bassloss")
    out = {"level_dbfs": dsp.db(amp)}
    if a.get("ref") is None:
        return _rd(out), {}
    if ref is None:
        raise RuntimeError(f"reference render {a['ref']} missing")
    out["gain_db"] = dsp.db(amp / ref)
    out["resonance"] = a["resonance"]
    # What a ladder with resonance feedback k predicts: -20 log10(1 + k), k = 4 x resonance.
    out["ladder_prediction_db"] = -20.0 * math.log10(1.0 + 4.0 * a["resonance"])
    return _rd(out), {}


def res_level_dependence(test, sr, x, ctx):
    """Gain at the resonant peak for a stepped input level; nonlinear filters compress as the level rises."""
    a = test["analysis"]
    amp, ref = _gain_against_ref(test, sr, x, ctx, "resdep")
    out = {"level_in_db": a["level_db"], "level_dbfs": dsp.db(amp)}
    if a.get("ref") is None:
        return _rd(out), {}
    if ref is None:
        raise RuntimeError(f"reference render {a['ref']} missing")
    out["peak_gain_db"] = dsp.db(amp / ref)
    return _rd(out), {}


# --------------------------------------------------------------------------------------------
# Saturation


def sat_harmonics(test, sr, x, ctx):
    """Harmonic levels of a sine through a saturator: H2..Hn against the fundamental, THD, even and odd shares."""
    a = test["analysis"]
    seg = dsp.segment(dsp.mono(x), sr, *a["window"])
    f0 = dsp.estimate_f0(seg, sr, dsp.midi_to_hz(a["note"]))
    amps = []
    for k in range(1, a.get("harmonics", 12) + 1):
        if k * f0 >= 0.45 * sr:
            break
        amps.append(_tone_amp(seg, sr, k * f0))
    amps = np.array(amps)
    out = {"f0_hz": f0, "drive": a.get("drive"), "level_dbfs": dsp.db(amps[0])}
    for k in range(2, len(amps) + 1):
        out[f"h{k}_db"] = dsp.db(amps[k - 1] / amps[0])
    p = amps ** 2
    out["thd_db"] = dsp.pdb(float(p[1:].sum() / p[0]))
    out["even_db"] = dsp.pdb(float(p[1::2].sum() / p[0]))
    out["odd_db"] = dsp.pdb(float(p[2::2].sum() / p[0]))
    return _rd(out), {}


# --------------------------------------------------------------------------------------------
# Drift statistics


def _pitch_cents(seg: np.ndarray, sr: int, expected: float, frame: int, hop: int) -> np.ndarray:
    _, f = dsp.pitch_track(seg, sr, expected, frame=frame, hop=hop)
    return 1200.0 * np.log2(f / expected) if len(f) else np.array([])


def drift_stats(test, sr, x, ctx):
    """Pitch drift of many notes: standard deviation, autocorrelation time and PSD slope, pooled over the notes."""
    a = test["analysis"]
    m = dsp.mono(x)
    expected = dsp.midi_to_hz(a["note"])
    frame, hop = a.get("frame", 4096), a.get("hop", 2400)
    dt = hop / sr
    runs = []
    for start in a["starts"]:
        seg = dsp.segment(m, sr, start + a.get("skip", 0.5), start + a["length"])
        c = _pitch_cents(seg, sr, expected, frame, hop)
        if len(c) > 8:
            runs.append(c)
    stats = series_stats(runs, dt, a.get("slope_band", [0.3, 3.0]))
    stats["notes"] = len(runs)
    stats["dt_s"] = dt
    if a.get("nominal_std"):
        stats["std_error_pct"] = 100.0 * (stats["std"] / a["nominal_std"] - 1.0)
    if a.get("nominal_tau"):
        stats["tau_error_pct"] = 100.0 * (stats["tau_s"] / a["nominal_tau"] - 1.0)
    return _rd(stats), {}


def series_stats(runs: list[np.ndarray], dt: float, slope_band=(0.3, 3.0)) -> dict:
    """Pooled mean, standard deviation, autocorrelation time (fit of ln rho over lags with rho > 0.3) and PSD slope."""
    allv = np.concatenate(runs)
    mean = float(allv.mean())
    max_lag = min(min(len(r) for r in runs) // 2, 400)
    cov = np.zeros(max_lag + 1)
    pairs = np.zeros(max_lag + 1)
    for r in runs:
        d = r - mean
        for lag in range(max_lag + 1):
            cov[lag] += float(np.dot(d[:len(d) - lag], d[lag:]))
            pairs[lag] += len(d) - lag
    cov /= pairs
    std = math.sqrt(max(cov[0], 0.0))
    num = den = 0.0
    for lag in range(1, max_lag + 1):
        rho = cov[lag] / cov[0]
        if rho < 0.3:
            break
        t = lag * dt
        num += t * t
        den += -t * math.log(rho)
    tau = num / den if den > 0 else float("nan")
    # Welch PSD over the runs.
    seg = 1 << int(math.floor(math.log2(max(16, min(len(r) for r in runs)))))
    win = np.hanning(seg)
    acc, count = np.zeros(seg // 2 + 1), 0
    for r in runs:
        d = r - r.mean()
        for s in range(0, len(d) - seg + 1, seg // 2):
            acc += np.abs(np.fft.rfft(d[s:s + seg] * win)) ** 2
            count += 1
    f = np.fft.rfftfreq(seg, dt)
    psd = acc / max(count, 1)
    lo, hi = slope_band
    sel = (f >= lo) & (f < hi)
    slope = float(np.polyfit(np.log10(f[sel]), np.log10(psd[sel]), 1)[0]) if sel.sum() >= 3 else float("nan")
    return {"mean": mean, "std": std, "tau_s": tau, "lag1": float(cov[1] / cov[0]), "psd_slope": slope}


# --------------------------------------------------------------------------------------------
# RC envelope fits


def fit_exponential(t: np.ndarray, y: np.ndarray) -> dict:
    """Least-squares y = c + a exp(-t / tau): scan tau, solve c and a exactly, refine; residuals as a share of the range."""
    t = np.asarray(t, dtype=np.float64) - t[0]
    y = np.asarray(y, dtype=np.float64)
    span = max(t[-1], 1e-9)
    rng = max(float(y.max() - y.min()), 1e-30)

    def solve(tau):
        e = np.exp(-t / tau)
        A = np.stack([np.ones_like(t), e], axis=1)
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        r = y - A @ coef
        return float(r @ r), coef

    taus = np.geomspace(span * 1e-4, span * 1e3, 240)
    sse = [solve(tau)[0] for tau in taus]
    i = int(np.argmin(sse))
    lo, hi = math.log(taus[max(i - 1, 0)]), math.log(taus[min(i + 1, len(taus) - 1)])
    g = 0.6180339887498949
    c1, c2 = hi - g * (hi - lo), lo + g * (hi - lo)
    f1, f2 = solve(math.exp(c1))[0], solve(math.exp(c2))[0]
    for _ in range(80):
        if f1 < f2:
            hi, c2, f2 = c2, c1, f1
            c1 = hi - g * (hi - lo)
            f1 = solve(math.exp(c1))[0]
        else:
            lo, c1, f1 = c1, c2, f2
            c2 = lo + g * (hi - lo)
            f2 = solve(math.exp(c2))[0]
    tau = math.exp(0.5 * (lo + hi))
    _, coef = solve(tau)
    resid = y - (coef[0] + coef[1] * np.exp(-t / tau))
    return {"tau": tau, "offset": float(coef[0]), "amplitude": float(coef[1]),
            "residual_rms_pct": 100.0 * float(np.sqrt(np.mean(resid ** 2))) / rng,
            "residual_max_pct": 100.0 * float(np.abs(resid).max()) / rng}


def env_rc_fit(test, sr, x, ctx):
    """Analog RC envelope: exponential fits of attack, decay and release; peak time against the attack setting."""
    a = test["analysis"]
    # Whole carrier periods per window keep the sliding RMS free of ripple; two of them barely smooth a 100 ms attack.
    window = a.get("window_s") or 2.0 / dsp.midi_to_hz(a.get("note", 69))
    t, e = dsp.rms_envelope(dsp.mono(x), sr, win_s=window, hop_s=0.002)
    on, off = a["note_on"], a["note_off"]
    i_on, i_off = int(np.searchsorted(t, on)), int(np.searchsorted(t, off))
    k = i_on + int(np.argmax(e[i_on:i_off]))
    peak, t_peak = float(e[k]), float(t[k])
    out = {"peak_time_s": t_peak - on, "peak_time_error_pct": 100.0 * ((t_peak - on) / a["attack"] - 1.0)}
    att = fit_exponential(t[i_on:k + 1], e[i_on:k + 1] / peak)
    out.update({"attack_tau_s": att["tau"], "attack_target": att["offset"],
                "attack_residual_pct": att["residual_rms_pct"]})
    j = int(np.searchsorted(t, off - 0.01))
    dec = fit_exponential(t[k:j], e[k:j] / peak)
    out.update({"decay_tau_s": dec["tau"], "decay_residual_pct": dec["residual_rms_pct"]})
    end = min(len(t), int(np.searchsorted(t, off + a["release"] * 4)))
    rel = fit_exponential(t[i_off:end], e[i_off:end] / peak)
    out.update({"release_tau_s": rel["tau"], "release_residual_pct": rel["residual_rms_pct"]})
    if a.get("retrigger"):
        # Retrigger inside the release: largest sample-to-sample step of the envelope around the new note-on.
        r0 = a["retrigger"]
        sel = (t >= r0 - 0.01) & (t <= r0 + 0.05)
        step = float(np.abs(np.diff(e[sel])).max()) / peak
        out["retrigger_step_db"] = dsp.db(step)
    return _rd(out), {"t": t - on, "env": e / peak}


# --------------------------------------------------------------------------------------------
# Voice-to-voice variance


def voice_variance(test, sr, x, ctx):
    """Notes of one pitch played in sequence: spread of pitch (cents) and level (dB) across the voices."""
    a = test["analysis"]
    m = dsp.mono(x)
    expected = dsp.midi_to_hz(a["note"])
    pitches, levels = [], []
    for i in range(a["notes"]):
        start = i * a["period"]
        seg = dsp.segment(m, sr, start + a["window"][0], start + a["window"][1])
        spec = dsp.spectrum(seg, sr, pad=4)
        f0 = dsp.estimate_f0(seg, sr, expected, spec)
        pitches.append(dsp.cents(f0 / expected))
        levels.append(dsp.db(_rms(seg)))
    p, lv = np.array(pitches), np.array(levels)
    out = {"notes": a["notes"], "pitch_mean_cents": float(p.mean()), "pitch_std_cents": float(p.std()),
           "pitch_max_abs_cents": float(np.abs(p).max()), "level_std_db": float(lv.std()),
           "level_max_dev_db": float(np.abs(lv - lv.mean()).max())}
    return _rd(out), {"pitch_cents": p, "level_db": lv}


ANALOG_ANALYSERS = {
    "selfosc_track": selfosc_track, "res_bassloss": res_bassloss, "res_level_dependence": res_level_dependence,
    "sat_harmonics": sat_harmonics, "drift_stats": drift_stats, "env_rc_fit": env_rc_fit,
    "voice_variance": voice_variance,
}


def summaries(results: dict, plan_tests: dict | None = None) -> dict:
    """Cross-test tables for the pending types: tracking spread, bass loss against resonance, compression, drift."""
    s: dict = {}
    track = {n: m for n, m in results.items() if n.startswith("selfosc_track_") and "cents_error" in m}
    if track:
        levels = [m["level_dbfs"] for m in track.values()]
        s["selfosc_track"] = {
            "max_abs_cents": max(abs(m["cents_error"]) for m in track.values()),
            "amplitude_spread_db": max(levels) - min(levels),
            "count": len(track),
        }
    bass = sorted(((m["resonance"], m["gain_db"]) for n, m in results.items()
                   if n.startswith("bassloss_") and "gain_db" in m), key=lambda r: r[0])
    if bass:
        base = bass[0][1]
        s["bassloss"] = {"rows": [{"resonance": r, "loss_db": round(g - base, 3)} for r, g in bass]}
        pred = [(r, -20.0 * math.log10(1.0 + 4.0 * r)) for r, _ in bass]
        s["bassloss"]["ladder_error_db"] = round(max(abs((g - base) - (p - pred[0][1])) for (_, g), (_, p) in zip(bass, pred)), 3)
    dep = sorted(((m["level_in_db"], m["peak_gain_db"]) for n, m in results.items()
                  if n.startswith("resdep_") and "peak_gain_db" in m), key=lambda r: r[0])
    if dep:
        gains = [g for _, g in dep]
        s["res_level_dependence"] = {"rows": [{"level_db": l, "peak_gain_db": g} for l, g in dep],
                                     "monotonic": 1 if all(b <= a + 0.05 for a, b in zip(gains, gains[1:])) else 0}
    drift = [m for n, m in results.items() if n.startswith("drift_stats") and "std" in m]
    if drift:
        s["drift_stats"] = {k: round(float(np.mean([d[k] for d in drift if k in d])), 4)
                            for k in ("std", "tau_s", "psd_slope", "std_error_pct", "tau_error_pct") if any(k in d for d in drift)}
        s["drift_stats"]["notes"] = int(sum(d["notes"] for d in drift))
    return s
