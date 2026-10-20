"""Self-test of the analog-behaviour analysers: each one gets a synthetic render whose answer is known and has to
recover it. Run with `python3 refbench.py selftest` (numpy only, about 20 seconds)."""

from __future__ import annotations

import math
import sys

import numpy as np

from refbench_analysis import measure, measure_analog  # measure first: measure_analog needs its helpers

SR = 48000
RESULTS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str) -> None:
    RESULTS.append((name, bool(ok), detail))


def tone(n: int, hz: float, amp: float = 1.0, phase: float = 0.0) -> np.ndarray:
    return amp * np.sin(2 * math.pi * hz * np.arange(n) / SR + phase)


def ou(n: int, dt: float, tau: float, sigma: float, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    a = math.exp(-dt / tau)
    b = sigma * math.sqrt(1 - a * a)
    noise = rng.standard_normal(n)
    x = np.empty(n)
    v = sigma * rng.standard_normal()
    for i in range(n):
        x[i] = v
        v = a * v + b * noise[i]
    return x


def run(name: str, spec: dict, x: np.ndarray, ctx: dict | None = None) -> dict:
    test = {"name": name, "analysis": spec}
    metrics, _ = measure.ANALYSERS[spec["type"]](test, SR, x, ctx if ctx is not None else {})
    return metrics


def main() -> int:
    # Self-oscillation tracking: a steady sine 0.64 cent above the cutoff, and a decaying one.
    x = tone(3 * SR, 1000.37, 0.1)
    m = run("selfosc_track_1000", {"type": "selfosc_track", "cutoff": 1000.0, "window": [1.2, 2.9]}, x)
    check("selfosc_track cents", abs(m["cents_error"] - 1200 * math.log2(1.00037)) < 0.02, f"{m['cents_error']}")
    check("selfosc_track steady", m["oscillating"] and m["amplitude_variation_db"] < 0.01, f"{m}")
    decay = tone(3 * SR, 1000.0, 0.1) * np.exp(-np.arange(3 * SR) / SR / 0.3)
    m = run("selfosc_track_1000", {"type": "selfosc_track", "cutoff": 1000.0, "window": [1.2, 2.9]}, decay)
    check("selfosc_track decay is not oscillation", not m["oscillating"], f"{m}")

    # Bass loss and level dependence: gain of a tone against its reference render.
    ctx: dict = {}
    ref = tone(2 * SR, 46.25, 0.01)
    run("bassloss_ref", {"type": "res_bassloss", "note": 30, "window": [0.5, 1.9]}, ref, ctx)
    m = run("bassloss_r75", {"type": "res_bassloss", "ref": "bassloss_ref", "resonance": 0.75, "note": 30,
                             "window": [0.5, 1.9]}, ref * 10 ** (-9.0 / 20), ctx)
    check("res_bassloss gain", abs(m["gain_db"] + 9.0) < 0.05, f"{m['gain_db']}")
    check("res_bassloss ladder prediction", abs(m["ladder_prediction_db"] + 20 * math.log10(4.0)) < 0.01, f"{m}")
    ctx = {}
    ref = tone(2 * SR, 987.77, 0.5)
    run("resdep_ref_l-6", {"type": "res_level_dependence", "level_db": -6.0, "freq_hz": 987.77, "window": [0.5, 1.9]}, ref, ctx)
    m = run("resdep_l-6", {"type": "res_level_dependence", "ref": "resdep_ref_l-6", "level_db": -6.0,
                           "freq_hz": 987.77, "window": [0.5, 1.9]}, ref * 10 ** (7.5 / 20), ctx)
    check("res_level_dependence gain", abs(m["peak_gain_db"] - 7.5) < 0.05, f"{m['peak_gain_db']}")
    results = {f"resdep_l{l}": {"level_in_db": l, "peak_gain_db": g} for l, g in ((-40, 19.0), (-12, 6.0), (0, -4.0), (6, -9.0))}
    check("summary: compression is monotonic", measure_analog.summaries(results)["res_level_dependence"]["monotonic"] == 1, "")
    results["resdep_l6"]["peak_gain_db"] = 30.0
    check("summary: expansion is caught", measure_analog.summaries(results)["res_level_dependence"]["monotonic"] == 0, "")

    # Saturation: harmonics at known levels.
    n = int(1.6 * SR)
    x = tone(n, 987.77, 0.5) + tone(n, 2 * 987.77, 0.5 * 10 ** (-60 / 20), 0.3) + tone(n, 3 * 987.77, 0.5 * 10 ** (-70 / 20), 0.7)
    m = run("sat_1k_d50", {"type": "sat_harmonics", "note": 83, "drive": 0.5, "window": [0.4, 1.4], "harmonics": 6}, x)
    check("sat_harmonics H2/H3", abs(m["h2_db"] + 60) < 0.1 and abs(m["h3_db"] + 70) < 0.1, f"{m['h2_db']} {m['h3_db']}")
    thd_true = 10 * math.log10(10 ** -6 + 10 ** -7)
    check("sat_harmonics THD/even/odd", abs(m["thd_db"] - thd_true) < 0.1 and abs(m["even_db"] + 60) < 0.1 and abs(m["odd_db"] + 70) < 0.1,
          f"{m['thd_db']} {m['even_db']} {m['odd_db']}")

    # Drift: the pooled statistics of OU series, then the whole path through audio.
    dt, tau, sigma = 0.05, 2.0, 6.0
    runs = [ou(2000, dt, tau, sigma, 100 + i) for i in range(40)]
    s = measure_analog.series_stats(runs, dt, (0.3, 3.0))
    check("drift std", abs(s["std"] / sigma - 1) < 0.05, f"{s['std']}")
    check("drift tau", abs(s["tau_s"] / tau - 1) < 0.05, f"{s['tau_s']}")
    check("drift PSD slope", abs(s["psd_slope"] + 2.0) < 0.3, f"{s['psd_slope']}")
    hop = 2400
    notes, length = 12, 10.0
    audio = np.zeros(int(notes * length * SR))
    starts, truth = [], []
    for i in range(notes):
        c = ou(int(length / (hop / SR)) + 2, hop / SR, tau, sigma, 7 + i)
        f_t = 440.0 * 2 ** (np.interp(np.arange(int(length * SR)) / SR, np.arange(len(c)) * hop / SR, c) / 1200)
        ph = 2 * math.pi * np.cumsum(f_t) / SR
        audio[int(i * length * SR):int((i + 1) * length * SR)] = 0.5 * np.sin(ph)
        starts.append(i * length)
        truth.append(c[int(0.5 / (hop / SR)) + 1:int(9.8 / (hop / SR)) - 1])
    m = run("drift_stats_0", {"type": "drift_stats", "note": 69, "starts": starts, "length": 9.8, "skip": 0.5, "frame": 4096,
                              "hop": hop, "slope_band": [0.3, 3.0], "nominal_std": sigma, "nominal_tau": tau}, audio)
    ts = measure_analog.series_stats(truth, hop / SR, (0.3, 3.0))
    check("drift through audio matches the realised series", abs(m["std"] / ts["std"] - 1) < 0.05 and abs(m["tau_s"] / ts["tau_s"] - 1) < 0.10,
          f"measured std {m['std']} tau {m['tau_s']}, realised std {ts['std']} tau {ts['tau_s']}")

    # RC envelope: an attack that overshoots to 1.2 and reaches 1.0 at 100 ms, an exponential decay to 0.5, a release.
    on, off, attack, release = 0.1, 1.3, 0.1, 0.4
    overshoot = 0.2
    t_att = -attack / math.log(1 - 1 / (1 + overshoot))
    t = np.arange(int(2.4 * SR)) / SR
    env = np.zeros_like(t)
    a = (t >= on) & (t < on + attack)
    env[a] = (1 + overshoot) * (1 - np.exp(-(t[a] - on) / t_att))
    d = (t >= on + attack) & (t < off)
    env[d] = 0.5 + 0.5 * np.exp(-(t[d] - on - attack) / 0.1)
    r = t >= off
    env[r] = 0.5 * np.exp(-(t[r] - off) / 0.15)
    x = env * 0.5 * np.sin(2 * math.pi * 440.0 * t)
    m = run("env_rc", {"type": "env_rc_fit", "note_on": on, "note_off": off, "attack": attack, "release": release}, x)
    check("env_rc_fit peak time", abs(m["peak_time_error_pct"]) < 5, f"{m['peak_time_error_pct']}")
    check("env_rc_fit attack tau", abs(m["attack_tau_s"] / t_att - 1) < 0.05 and m["attack_residual_pct"] < 1.0, f"{m['attack_tau_s']} vs {t_att}, {m['attack_residual_pct']}")
    check("env_rc_fit decay/release tau", abs(m["decay_tau_s"] / 0.1 - 1) < 0.05 and abs(m["release_tau_s"] / 0.15 - 1) < 0.05,
          f"{m['decay_tau_s']} {m['release_tau_s']}")
    fit = measure_analog.fit_exponential(np.arange(0, 1, 0.001), 0.1 + 0.9 * np.exp(-np.arange(0, 1, 0.001) / 0.25))
    check("fit_exponential exact", abs(fit["tau"] / 0.25 - 1) < 1e-4 and fit["residual_rms_pct"] < 1e-4, f"{fit}")

    # Voice variance: 16 notes with known pitch offsets.
    rng = np.random.default_rng(3)
    cents = rng.normal(0.0, 3.0, 16)
    levels = rng.normal(0.0, 0.2, 16)
    period = 1.2
    x = np.zeros(int(16 * period * SR))
    for i in range(16):
        n = int(1.0 * SR)
        x[int(i * period * SR):int(i * period * SR) + n] = 0.5 * 10 ** (levels[i] / 20) * tone(n, 261.626 * 2 ** (cents[i] / 1200))
    m = run("voice_variance", {"type": "voice_variance", "note": 60, "notes": 16, "period": period, "window": [0.3, 0.95]}, x)
    truth_mean = cents.mean() + 1200 * math.log2(261.626 / measure.dsp.midi_to_hz(60))
    check("voice_variance pitch", abs(m["pitch_std_cents"] - cents.std()) < 0.02 and abs(m["pitch_mean_cents"] - truth_mean) < 0.02,
          f"{m['pitch_std_cents']} vs {cents.std()}, {m['pitch_mean_cents']} vs {truth_mean}")
    check("voice_variance level", abs(m["level_std_db"] - levels.std()) < 0.02, f"{m['level_std_db']} vs {levels.std()}")

    failed = [r for r in RESULTS if not r[1]]
    for name, ok, detail in RESULTS:
        print(f"{'ok  ' if ok else 'FAIL'} {name}" + ("" if ok else f": {detail}"))
    print(f"{len(RESULTS) - len(failed)} of {len(RESULTS)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.path.insert(0, __file__.rsplit("/", 1)[0])
    sys.exit(main())
