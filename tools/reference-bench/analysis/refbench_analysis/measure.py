"""Analysers: one function per analysis type of the test plan.

Each analyser gets the test spec, the samples and a context (other renders' harmonic data for
transfer measurements, the render manifest for CPU) and returns (metrics, plot_data). Metrics are
plain JSON numbers; plot data are small arrays kept out of the committed results.
"""

from __future__ import annotations

import math

import numpy as np

from . import dsp


def _r(v, nd=3):
    if v is None:
        return None
    if isinstance(v, (float, np.floating)):
        return None if not math.isfinite(float(v)) else round(float(v), nd)
    return v


def _rd(d: dict, nd=3) -> dict:
    return {k: (_rd(v, nd) if isinstance(v, dict) else [_r(x, nd) for x in v] if isinstance(v, list) else _r(v, nd))
            for k, v in d.items()}


def spectrum_curve(spec: dsp.Spectrum, points: int = 1500, f_lo: float = 20.0) -> tuple[np.ndarray, np.ndarray]:
    """Max-hold of the amplitude spectrum on log-spaced bands (for plots)."""
    edges = np.geomspace(f_lo, spec.sr / 2, points + 1)
    idx = np.searchsorted(spec.freqs, edges)
    a = spec.amp_db()
    ys = np.array([a[i:j].max() if j > i else np.nan for i, j in zip(idx[:-1], idx[1:])])
    xs = np.sqrt(edges[:-1] * edges[1:])
    return xs, ys


# --------------------------------------------------------------------------------------------


def ideal_harmonic_db(wave: str, k: int) -> float | None:
    if wave == "saw":
        return -20 * math.log10(k)
    if wave == "square":
        return -20 * math.log10(k) if k % 2 else None
    return None


def osc(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.mono(x)
    seg = dsp.segment(m, sr, *a["window"])
    expected = dsp.midi_to_hz(a["note"])
    spec = dsp.spectrum(seg, sr)
    f0 = dsp.estimate_f0(seg, sr, expected, spec)
    h = dsp.harmonics(spec, f0)
    out = {
        "f0_hz": f0,
        "cents_error": dsp.cents(f0 / expected),
        "level_dbfs": dsp.db(float(np.sqrt(np.mean(seg ** 2)))),
        "dc_dbfs": dsp.db(abs(float(np.mean(seg)))),
        "alias_total_db": h.alias_db,
        "alias_audible_db": h.alias_audible_db,
        "worst_audible_spur_dbc": h.worst_audible_spur_dbc,
        "worst_audible_spur_hz": h.worst_audible_spur_hz,
        "harmonics": len(h.harmonics_db),
    }
    wave = a["wave"]
    if wave in ("saw", "square"):
        devs, bandwidth = [], 0.0
        for k, hdb in enumerate(h.harmonics_db, start=1):
            ideal = ideal_harmonic_db(wave, k)
            if ideal is None:
                continue
            d = hdb - ideal
            if k * f0 <= 10000:
                devs.append(d)
        # Bandwidth: highest harmonic before the first one more than 3 dB below the ideal level.
        for k, hdb in enumerate(h.harmonics_db, start=1):
            ideal = ideal_harmonic_db(wave, k)
            if ideal is None:
                continue
            if hdb - ideal < -3.0:
                break
            bandwidth = k * f0
        out["harmonic_error_db_rms"] = float(np.sqrt(np.mean(np.square(devs)))) if devs else None
        out["bandwidth_hz"] = bandwidth
        if wave == "square":
            evens = [hdb for k, hdb in enumerate(h.harmonics_db, start=1) if k % 2 == 0 and k * f0 < 10000]
            out["even_harmonics_db"] = max(evens) if evens else None
    else:
        out.update(dsp.thd(spec, f0))
    st = dsp.stereo_metrics(dsp.segment(x, sr, *a["window"]))
    out["stereo_correlation"] = st["correlation"]
    xs, ys = spectrum_curve(spec)
    a1 = dsp.amplitude_at(spec, f0)
    return _rd(out), {"spectrum_f": xs, "spectrum_db": ys - dsp.db(a1)}


def unison(test, sr, x, ctx):
    a = test["analysis"]
    seg_st = dsp.segment(x, sr, *a["window"])
    m = dsp.mono(seg_st)
    expected = dsp.midi_to_hz(a["note"])
    spec = dsp.spectrum(m, sr)
    # Spread: span of the voice cluster around harmonic k (bins within 40 dB of the cluster maximum,
    # searched within +/- 0.45 harmonic spacings). A first pass on the fundamental gives a rough
    # spread; the second pass uses the highest harmonic below 2 kHz whose cluster still cannot
    # touch its neighbours, which resolves narrow detune much better.
    lobe = spec.guard_hz()

    def cluster(k):
        fk = k * expected
        sel = (spec.freqs >= fk - 0.45 * expected) & (spec.freqs <= fk + 0.45 * expected)
        fa, aa = spec.freqs[sel], spec.amp[sel]
        strong = fa[aa > aa.max() * 10 ** (-40 / 20)]
        return fa, aa, strong.min(), strong.max()

    fa, aa, f_lo, f_hi = cluster(1)
    rough = max((f_hi - f_lo - 2 * lobe) / expected, 1e-4)
    k = 1
    while (k + 1) * expected < 2000.0 and (k + 1) * expected * rough < 0.3 * expected:
        k += 1
    fa, aa, f_lo, f_hi = cluster(k)
    spread_cents = max(0.0, dsp.cents((f_hi - lobe) / (f_lo + lobe))) if f_hi - f_lo > 2 * lobe else 0.0
    is_peak = (aa[1:-1] > aa[:-2]) & (aa[1:-1] >= aa[2:]) & (aa[1:-1] > aa.max() * 10 ** (-40 / 20))
    spread = (f_hi / f_lo) ** 0.5 - 1.0  # relative half-width including the window lobe
    # Aliasing is only defined where neighbouring harmonic clusters do not overlap.
    alias_limit = min(20000.0, expected * 0.45 / spread) if spread > 0 else 20000.0
    h = dsp.harmonics(spec, expected, spread=spread, audible_hz=alias_limit)
    st = dsp.stereo_metrics(seg_st)
    out = {
        "voice_peaks": int(is_peak.sum()),
        "spread_cents": spread_cents,
        "alias_band_limit_hz": alias_limit,
        "alias_audible_db": h.alias_audible_db if alias_limit > 4 * expected else None,
        "worst_audible_spur_dbc": h.worst_audible_spur_dbc if alias_limit > 4 * expected else None,
        "level_dbfs": dsp.db(float(np.sqrt(np.mean(m ** 2)))),
        **{key: st[key] for key in ("correlation", "side_to_mid_db", "mono_loss_db", "balance_db")},
    }
    xs, ys = spectrum_curve(spec)
    return _rd(out), {"spectrum_f": xs, "spectrum_db": ys - dsp.db(aa.max())}


def wtsweep(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.mono(x)
    f0 = dsp.midi_to_hz(a["note"])
    t0, t1 = a["window"]
    frame, hop = 2048, 1024  # short frames: the table changes little within one frame
    times, alias, levels = [], [], []
    for start in range(int(t0 * sr), int(t1 * sr) - frame, hop):
        seg = m[start:start + frame]
        spec = dsp.spectrum(seg, sr, pad=2)
        # Morphing changes the DC offset within a frame; its leakage is not aliasing.
        h = dsp.harmonics(spec, f0, dc_hz=0.5 * f0)
        times.append((start + frame / 2) / sr)
        alias.append(h.alias_audible_db)
        levels.append(dsp.db(float(np.sqrt(np.mean(seg ** 2)))))
    alias = np.array(alias)
    levels = np.array(levels)
    pos = (np.array(times) - t0) / (t1 - t0)
    out = {
        "alias_audible_db_max": float(alias.max()),
        "alias_audible_db_median": float(np.median(alias)),
        "alias_audible_db_p90": float(np.percentile(alias, 90)),
        "level_step_db_max": float(np.max(np.abs(np.diff(levels)))) if len(levels) > 1 else 0.0,
    }
    return _rd(out), {"position": pos, "alias_db": alias}


def harmonic_amplitudes(sr, x, note, window):
    m = dsp.segment(dsp.mono(x), sr, *window)
    spec = dsp.spectrum(m, sr)
    expected = dsp.midi_to_hz(note)
    f0 = dsp.estimate_f0(m, sr, expected, spec)
    kmax = int(min(20000.0, sr / 2 * 0.95) / f0)
    amps = np.array([dsp.amplitude_at(spec, k * f0, 0.3 / k if k > 1 else 0.02) for k in range(1, kmax + 1)])
    return f0, amps


def filter_ref(test, sr, x, ctx):
    a = test["analysis"]
    f0, amps = harmonic_amplitudes(sr, x, a["note"], a["window"])
    ctx.setdefault("refs", {})[test["name"]] = (f0, amps)
    return _rd({"f0_hz": f0, "level_dbfs": dsp.db(float(np.sqrt(np.mean(dsp.mono(x) ** 2))))}), {}


def _transfer(test, sr, x, ctx):
    a = test["analysis"]
    ref = ctx.get("refs", {}).get(a["ref"])
    if ref is None:
        raise RuntimeError(f"reference render {a['ref']} missing")
    f0r, ar = ref
    f0, at = harmonic_amplitudes(sr, x, a["note"], a["window"])
    n = min(len(ar), len(at))
    f = f0r * np.arange(1, n + 1)
    resp = 20 * np.log10(np.maximum(at[:n], 1e-12) / np.maximum(ar[:n], 1e-12))
    return f, resp


def _crossing(f, r, level, start, step):
    """First crossing of `level` walking from index `start` in direction `step` (log-f interpolated)."""
    i = start
    while 0 <= i < len(r) and 0 <= i + step < len(r):
        a, b = r[i], r[i + step]
        if (a - level) * (b - level) <= 0 and a != b:
            fa, fb = math.log(f[i]), math.log(f[i + step])
            return math.exp(fa + (level - a) / (b - a) * (fb - fa))
        i += step
    return float("nan")


def _peak(f, r):
    """Peak of a sampled response, refined by a parabola in log frequency."""
    i = int(np.argmax(r))
    if 0 < i < len(r) - 1:
        x = np.log(f[i - 1:i + 2])
        y = r[i - 1:i + 2]
        c = np.polyfit(x, y, 2)
        if c[0] < 0:
            xv = -c[1] / (2 * c[0])
            if x[0] <= xv <= x[2]:
                return float(math.exp(xv)), float(np.polyval(c, xv))
    return float(f[i]), float(r[i])


def filter_(test, sr, x, ctx):
    a = test["analysis"]
    f, resp = _transfer(test, sr, x, ctx)
    fc = a["cutoff"]
    ftype = a["filter"]
    out = {"nominal_cutoff_hz": fc, "resonance": a["resonance"]}
    kind = "hp" if ftype.startswith("hp") else "bp" if ftype.startswith("bp") else "lp"
    ipk = int(np.argmax(resp))
    if kind == "lp":
        pass_sel = f < fc / 4
        pass_db = float(np.median(resp[pass_sel])) if pass_sel.sum() >= 2 else float(resp[0])
        rel = resp - pass_db
        f3 = _crossing(f, rel, -3.0, 0, 1)
        slope_sel = (f >= 2 * fc) & (f <= min(8 * fc, 16000))
    elif kind == "hp":
        pass_sel = (f > fc * 4) & (f < 16000)
        pass_db = float(np.median(resp[pass_sel])) if pass_sel.sum() >= 2 else float(resp[-1])
        rel = resp - pass_db
        f3 = _crossing(f, rel, -3.0, len(rel) - 1, -1)
        slope_sel = (f <= fc / 2) & (f >= max(fc / 8, 33))
    else:
        pass_db = float(resp[ipk])
        rel = resp - pass_db
        lo = _crossing(f, rel, -3.0, ipk, -1)
        hi = _crossing(f, rel, -3.0, ipk, 1)
        f3 = float("nan")
        out["bandwidth_octaves"] = math.log2(hi / lo) if lo > 0 and hi > 0 else None
        out["q"] = f[ipk] / (hi - lo) if hi > lo else None
        slope_sel = (f >= 2 * fc) & (f <= min(8 * fc, 16000))
    out["passband_gain_db"] = pass_db
    out["f_minus3db_hz"] = f3
    if kind == "lp":
        out["f_minus6db_hz"] = _crossing(f, rel, -6.0, 0, 1)
    elif kind == "hp":
        out["f_minus6db_hz"] = _crossing(f, rel, -6.0, len(rel) - 1, -1)
    fpk, gpk = _peak(f, rel)
    out["peak_hz"] = fpk
    out["peak_gain_db"] = gpk
    if slope_sel.sum() >= 3:
        slope = np.polyfit(np.log2(f[slope_sel]), rel[slope_sel], 1)[0]
        out["slope_db_per_oct"] = float(slope)
    # Accuracy: resonant filters by peak, others by the -3 dB point.
    if a["resonance"] >= 0.5 and gpk > 3.0:
        out["cutoff_error_cents"] = dsp.cents(fpk / fc)
    if math.isfinite(f3):
        # Where the -3 dB point sits relative to the nominal cutoff depends on the filter design
        # (pole frequency vs corner), so this is compared across cutoffs, not against 1.0.
        out["f3db_ratio"] = f3 / fc
    return _rd(out), {"freq": f, "response_db": rel}


def selfosc(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.segment(dsp.mono(x), sr, *a["window"])
    rms = float(np.sqrt(np.mean(m ** 2)))
    out = {"level_dbfs": dsp.db(rms), "self_oscillates": bool(rms > 10 ** (-50 / 20))}
    if out["self_oscillates"]:
        spec = dsp.spectrum(m, sr)
        f0 = dsp.estimate_f0(m, sr, None, spec)
        out["f0_hz"] = f0
        out["pitch_error_cents"] = dsp.cents(f0 / a["cutoff"])
        out.update(dsp.thd(spec, f0))
    return _rd(out), {}


def drive(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.segment(dsp.mono(x), sr, *a["window"])
    spec = dsp.spectrum(m, sr)
    f0 = dsp.estimate_f0(m, sr, dsp.midi_to_hz(a["note"]), spec)
    out = {"drive": a["drive"], "level_dbfs": dsp.db(float(np.sqrt(np.mean(m ** 2)))), **dsp.thd(spec, f0, n=20)}
    return _rd(out), {}


def envelope(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.mono(x)
    t, e = dsp.rms_envelope(m, sr, win_s=0.005, hop_s=0.0005)
    e_db = 20 * np.log10(np.maximum(e, 1e-9))
    on, off = a["note_on"], a["note_off"]
    hold = (t >= on) & (t <= off)
    i_on = int(np.searchsorted(t, on))
    i_off = int(np.searchsorted(t, off))
    peak_i = i_on + int(np.argmax(e[i_on:i_off]))
    peak = e[peak_i]
    out = {}
    t10 = dsp.first_crossing(t, e, 0.1 * peak, i_on)
    t90 = dsp.first_crossing(t, e, 0.9 * peak, i_on)
    t99 = dsp.first_crossing(t, e, 0.99 * peak, i_on)
    out["attack_10_90_s"] = t90 - t10
    out["attack_to_peak_s"] = t99 - on
    out["attack_ratio"] = (t99 - on) / a["attack"] if a.get("attack") else None
    # Curve: level at half the time to peak (0.5 = linear in amplitude).
    if math.isfinite(t99):
        mid = on + (t99 - on) / 2
        out["attack_mid_level"] = float(np.interp(mid, t, e) / peak)
    sus_sel = (t >= off - 0.2 * (off - on)) & (t <= off - 0.01)
    sus = float(np.median(e[sus_sel])) if sus_sel.any() else float("nan")
    out["sustain_db"] = dsp.db(sus / peak)
    nominal_sus = a.get("sustain")
    if nominal_sus:
        out["sustain_error_db"] = out["sustain_db"] - dsp.db(nominal_sus)
    # Decay: time from the peak to 95 % of the way down to the sustain level (linear amplitude).
    target = sus + 0.05 * (peak - sus)
    td = dsp.first_crossing(t, e, target, peak_i, rising=False)
    out["decay_95_s"] = td - t[peak_i]
    target63 = sus + math.exp(-1) * (peak - sus)
    out["decay_63_s"] = dsp.first_crossing(t, e, target63, peak_i, rising=False) - t[peak_i]
    if a.get("decay"):
        out["decay_ratio"] = out["decay_95_s"] / a["decay"]
    # Release from the level at note-off.
    lvl_off = float(np.interp(off, t, e))
    for name, drop in (("release_to_minus20db_s", 0.1), ("release_to_minus60db_s", 0.001)):
        tr = dsp.first_crossing(t, e, lvl_off * drop, i_off, rising=False)
        out[name] = tr - off
    if a.get("release"):
        out["release_ratio_60db"] = out["release_to_minus60db_s"] / a["release"]
    dec = max(1, int(0.002 / (t[1] - t[0])))
    return _rd(out), {"t": t[::dec] - on, "env_db": e_db[::dec] - dsp.db(peak)}


def _hf_ratio(seg, sr, cutoff=2000.0):
    if len(seg) < 32 or not np.any(seg):
        return float("nan")
    X = np.abs(np.fft.rfft(seg * np.hanning(len(seg)))) ** 2
    f = np.fft.rfftfreq(len(seg), 1 / sr)
    return dsp.pdb(X[f > cutoff].sum() / max(X.sum(), 1e-300))


def onset(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.mono(x)
    on, off = a["note_on"], a["note_off"]
    seg = m[int(on * sr):]
    peak = np.abs(seg).max() if len(seg) else 0.0
    out = {}
    for rel, name in ((-60.0, "latency_minus60db_ms"), (-20.0, "latency_minus20db_ms")):
        idx = np.nonzero(np.abs(seg) >= peak * 10 ** (rel / 20))[0]
        out[name] = 1000.0 * idx[0] / sr if len(idx) else None
    # Clicks: share of energy above 2 kHz around the transitions (a 440 Hz sine has none).
    n10 = int(0.01 * sr)
    i_on = int(on * sr) + int((out["latency_minus60db_ms"] or 0) * sr / 1000)
    out["onset_hf_db"] = _hf_ratio(m[i_on:i_on + n10], sr)
    out["release_hf_db"] = _hf_ratio(m[int(off * sr) - n10 // 2:int(off * sr) + n10], sr)
    out["steady_hf_db"] = _hf_ratio(m[int((on + 0.15) * sr):int((on + 0.15) * sr) + n10], sr)
    return _rd(out), {}


def velocity(test, sr, x, ctx):
    m = dsp.mono(x)
    levels = []
    for n in test["analysis"]["notes"]:
        seg = dsp.segment(m, sr, n["start"] + 0.05, n["start"] + n["length"] - 0.02)
        levels.append(dsp.db(float(np.sqrt(np.mean(seg ** 2)))))
    ref = levels[0]
    vels = [n["velocity"] for n in test["analysis"]["notes"]]
    out = {f"vel{v}_db": lv - ref for v, lv in zip(vels, levels)}
    span = levels[0] - levels[-1]
    out["dynamic_range_db"] = span
    return _rd(out), {"velocity": np.array(vels, float), "level_db": np.array(levels) - ref}


def bend(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.mono(x)
    base = dsp.midi_to_hz(a["note"])
    out = {}
    errs = []
    for t0, t1, semis in a["segments"]:
        seg = dsp.segment(m, sr, t0, t1)
        exp = base * 2 ** (semis / 12)
        f = dsp.estimate_f0(seg, sr, exp)
        err = dsp.cents(f / exp)
        out[f"bend_{semis:+g}st_error_cents"] = err
        errs.append(abs(err))
    out["max_error_cents"] = max(errs)
    return _rd(out), {}


def idle(test, sr, x, ctx):
    a = test["analysis"]
    seg = dsp.segment(x, sr, *a["window"])
    m = dsp.mono(seg)
    return _rd({
        "rms_dbfs": dsp.db(float(np.sqrt(np.mean(seg ** 2)))) if seg.size else None,
        "peak_dbfs": dsp.db(float(np.abs(seg).max())) if seg.size else None,
        "dc_dbfs": dsp.db(abs(float(m.mean()))) if m.size else None,
        "nonzero_fraction": float(np.mean(seg != 0.0)) if seg.size else None,
    }), {}


def _shape(shape: str, phase: np.ndarray) -> np.ndarray:
    p = phase % 1.0
    if shape == "sine":
        return np.sin(2 * np.pi * p)
    if shape == "triangle":
        return 1 - 4 * np.abs(p - 0.5)
    if shape == "saw":
        return 2 * p - 1
    if shape == "square":
        return np.where(p < 0.5, 1.0, -1.0)
    return np.sin(2 * np.pi * p)


def lfo(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.segment(dsp.mono(x), sr, *a["window"])
    t, e = dsp.rms_envelope(m, sr, win_s=0.0045, hop_s=0.001)
    e = e - e.mean()
    if not np.any(e):
        return _rd({"rate_hz": None}), {}
    # Rate from rising zero crossings of the mean-removed envelope (linearly interpolated), after
    # light smoothing so ripple of the carrier cannot add crossings.
    k = max(1, int(0.02 / (t[1] - t[0])))
    sm = np.convolve(e, np.ones(k) / k, mode="same")
    idx = np.nonzero((sm[:-1] < 0) & (sm[1:] >= 0))[0]
    idx = idx[(idx > k) & (idx < len(sm) - k - 1)]
    if len(idx) < 3:
        return _rd({"rate_hz": None}), {}
    tc = t[idx] + (0 - sm[idx]) / (sm[idx + 1] - sm[idx]) * (t[idx + 1] - t[idx])
    rate = (len(tc) - 1) / (tc[-1] - tc[0])
    # Shape fidelity: best correlation with the ideal shape (both polarities, any phase).
    best = -1.0
    for ph in np.linspace(0, 1, 200, endpoint=False):
        ideal = _shape(a["shape"], rate * t + ph)
        for sign in (1, -1):
            c = np.corrcoef(e, sign * ideal)[0, 1]
            best = max(best, c)
    env = e + 1e-9
    out = {"rate_hz": rate, "rate_error_pct": 100 * (rate / a["rate"] - 1), "shape_correlation": best}
    return _rd(out), {"t": t, "env": env / np.abs(env).max()}


def reverb(test, sr, x, ctx):
    a = test["analysis"]
    tail = x[int(a["excite_end"] * sr):]
    m = dsp.mono(tail)
    tt, edc = dsp.schroeder_decay(m, sr)
    out = {
        "nominal_rt60_s": a["nominal_rt60"],
        "edt_s": dsp.decay_time(tt, edc, 0.0, -10.0),
        "t20_s": dsp.decay_time(tt, edc, -5.0, -25.0),
        "t30_s": dsp.decay_time(tt, edc, -5.0, -35.0),
    }
    if out["t30_s"] and math.isfinite(out["t30_s"]):
        out["rt60_ratio"] = out["t30_s"] / a["nominal_rt60"]
    # Band decay (FFT brick-wall bands).
    X = np.fft.rfft(m)
    f = np.fft.rfftfreq(len(m), 1 / sr)
    for fc in (250, 1000, 4000):
        band = np.where((f >= fc / math.sqrt(2)) & (f < fc * math.sqrt(2)), X, 0)
        mb = np.fft.irfft(band, len(m))
        tb, eb = dsp.schroeder_decay(mb, sr)
        out[f"t20_{fc}hz_s"] = dsp.decay_time(tb, eb, -5.0, -25.0)
    late = tail[int(0.1 * sr):int(min(len(tail), (0.1 + a["nominal_rt60"] / 2) * sr))]
    out["tail_correlation"] = dsp.stereo_metrics(late)["correlation"] if len(late) > 100 else None
    dec = max(1, len(tt) // 2000)
    return _rd(out), {"t": tt[::dec], "edc_db": edc[::dec]}


def delay(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.mono(x)
    t, e = dsp.rms_envelope(m, sr, win_s=0.01, hop_s=0.001)
    T = a["time"]
    peaks = []
    for i in range(0, 8):
        c = a["excite"] + i * T
        sel = (t > c - 0.4 * T) & (t < c + 0.4 * T)
        if not sel.any():
            break
        j = np.argmax(np.where(sel, e, 0))
        peaks.append((t[j], e[j]))
    times = np.array([p[0] for p in peaks])
    lv = np.array([dsp.db(p[1]) for p in peaks])
    out = {"nominal_time_ms": 1000 * T}
    if len(times) >= 3:
        out["time_ms"] = 1000 * float(np.median(np.diff(times[1:])))
        out["time_error_ms"] = out["time_ms"] - 1000 * T
        steps = np.diff(lv[1:])
        out["feedback_db_per_repeat"] = float(np.median(steps[:4]))
        out["feedback_error_db"] = out["feedback_db_per_repeat"] - dsp.db(a["feedback"])
    return _rd(out), {}


def chorus(test, sr, x, ctx):
    a = test["analysis"]
    seg = dsp.segment(x, sr, *a["window"])
    st = dsp.stereo_metrics(seg)
    m = dsp.mono(seg)
    spec = dsp.spectrum(m, sr)
    # Modulation sidebands: energy away from the harmonics below 5 kHz relative to the harmonics.
    h = dsp.harmonics(spec, dsp.midi_to_hz(a["note"]), audible_hz=5000.0)
    return _rd({**st, "sideband_db": h.alias_audible_db}), {}


def eq(test, sr, x, ctx):
    a = test["analysis"]
    f, resp = _transfer(test, sr, x, ctx)
    out = {"max_gain_db": float(resp.max()), "min_gain_db": float(resp.min())}
    bands = ctx.get("profile_eq_bands") or []
    for b in bands:
        fc = b["freq"]
        sel = (f >= fc / 4) & (f <= fc * 4)
        if sel.any():
            g = float(resp[sel].max())
            out[f"band_{fc}hz_gain_db"] = g
            out[f"band_{fc}hz_error_db"] = g - a["gain_db"]
    return _rd(out), {"freq": f, "response_db": resp}


def comp_point(test, sr, x, ctx):
    a = test["analysis"]
    m = dsp.segment(dsp.mono(x), sr, *a["window"])
    lvl = dsp.db(float(np.sqrt(np.mean(m ** 2))))
    ctx.setdefault("comp", {}).setdefault(a["step"], {})["on" if a["on"] else "off"] = lvl
    return _rd({"level_dbfs": lvl}), {}


def cpu(test, sr, x, ctx):
    a = test["analysis"]
    manifest = ctx.get("manifest", {})
    runs = [m for name, m in manifest.items()
            if name == test["name"] or name.startswith(test["name"] + "__r")]
    if not runs:
        return {"note": "no render manifest (CPU unknown)"}, {}
    voices = a["voices"] * a["unison"]

    def best(key):
        vals = [m[key] for m in runs if m.get(key) is not None]
        return min(vals) if vals else None

    cpu_pct = best("cpuPercent")
    return _rd({
        "runs": len(runs),
        "cpu_percent": cpu_pct,
        "cpu_percent_all_threads": best("cpuPercentAllThreads"),
        "realtime_factor": max(m.get("realtimeFactor", 0.0) for m in runs),
        "worst_block_ms": best("worstBlockMs"),
        "block_budget_ms": runs[0].get("blockBudgetMs"),
        "cpu_percent_per_note": (cpu_pct or 0.0) / max(1, a["voices"]),
        "cpu_percent_per_osc_voice": (cpu_pct or 0.0) / max(1, voices),
    }), {}


ANALYSERS = {
    "osc": osc, "unison": unison, "wtsweep": wtsweep, "filter_ref": filter_ref, "filter": filter_,
    "selfosc": selfosc, "drive": drive, "envelope": envelope, "onset": onset, "velocity": velocity,
    "bend": bend, "idle": idle, "lfo": lfo, "reverb": reverb, "delay": delay, "chorus": chorus, "eq": eq,
    "comp_point": comp_point, "cpu": cpu,
}

from .measure_analog import ANALOG_ANALYSERS  # noqa: E402  (needs _rd from this module)

ANALYSERS.update(ANALOG_ANALYSERS)


def compressor_curve(ctx: dict, threshold_db: float | None, ratio: float | None) -> dict | None:
    pts = ctx.get("comp")
    if not pts:
        return None
    xs, ys = [], []
    for step in sorted(pts):
        p = pts[step]
        if "on" in p and "off" in p:
            xs.append(p["off"])
            ys.append(p["on"])
    if len(xs) < 3:
        return None
    xs, ys = np.array(xs), np.array(ys)
    out = {"input_dbfs": [round(v, 2) for v in xs], "output_dbfs": [round(v, 2) for v in ys]}
    if threshold_db is not None:
        above = xs > threshold_db + 6
        if above.sum() >= 2:
            slope = np.polyfit(xs[above], ys[above], 1)[0]
            out["ratio_measured"] = round(float(1.0 / slope), 3) if slope > 0 else None
            out["ratio_nominal"] = ratio
        gr = xs - ys
        crossing = np.nonzero(gr > 1.0)[0]
        out["gain_reduction_onset_dbfs"] = round(float(xs[crossing[0]]), 2) if len(crossing) else None
        out["threshold_nominal_dbfs"] = threshold_db
    return out
