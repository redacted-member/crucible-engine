"""Measurement primitives: spectra, pitch, harmonics, aliasing, envelopes, decay, stereo.

Everything here works on float64 numpy arrays and needs nothing beyond numpy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

KAISER_BETA = 20.0  # sidelobes near -150 dB, enough for float renders
MAINLOBE_BINS = math.sqrt(1.0 + (KAISER_BETA / math.pi) ** 2)  # half width of the main lobe in bins


def midi_to_hz(note: float, a4: float = 440.0) -> float:
    return a4 * 2.0 ** ((note - 69.0) / 12.0)


def cents(ratio: float) -> float:
    return 1200.0 * math.log2(ratio) if ratio > 0 else float("nan")


def db(x: float, floor: float = -300.0) -> float:
    return 20.0 * math.log10(x) if x > 0 else floor


def pdb(p: float, floor: float = -300.0) -> float:
    return 10.0 * math.log10(p) if p > 0 else floor


def mono(x: np.ndarray) -> np.ndarray:
    return x.mean(axis=1) if x.ndim == 2 else x


def segment(x: np.ndarray, sr: int, start: float, end: float) -> np.ndarray:
    a = max(0, int(round(start * sr)))
    b = min(len(x), int(round(end * sr)))
    return x[a:b]


# --------------------------------------------------------------------------------------------
# Spectrum


@dataclass
class Spectrum:
    freqs: np.ndarray
    amp: np.ndarray  # sine amplitude scale: a sine of amplitude A peaks at A
    power: np.ndarray  # per-bin power on a common scale (ratios only)
    bin_hz: float
    sr: int
    n: int

    def amp_db(self) -> np.ndarray:
        return 20.0 * np.log10(np.maximum(self.amp, 1e-15))

    def guard_hz(self) -> float:
        # Main lobe plus a margin for the near-carrier skirt (phase noise, slow level changes).
        return (MAINLOBE_BINS + 3.0) * self.sr / self.n


def spectrum(x: np.ndarray, sr: int, pad: int = 2) -> Spectrum:
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    w = np.kaiser(n, KAISER_BETA)
    nfft = 1 << int(math.ceil(math.log2(max(16, n * pad))))
    X = np.fft.rfft((x - x.mean()) * w, nfft)
    amp = np.abs(X) * 2.0 / w.sum()
    power = np.abs(X) ** 2
    freqs = np.fft.rfftfreq(nfft, 1.0 / sr)
    return Spectrum(freqs, amp, power, sr / nfft, sr, n)


def interp_peak(spec: Spectrum, f_lo: float, f_hi: float) -> tuple[float, float]:
    """Largest peak in [f_lo, f_hi]: (frequency, amplitude) refined by a parabola on log magnitude."""
    i0 = max(1, int(f_lo / spec.bin_hz))
    i1 = min(len(spec.amp) - 2, int(math.ceil(f_hi / spec.bin_hz)))
    if i1 <= i0:
        return float("nan"), 0.0
    k = i0 + int(np.argmax(spec.amp[i0:i1 + 1]))
    a, b, c = (math.log(max(spec.amp[j], 1e-30)) for j in (k - 1, k, k + 1))
    denom = a - 2 * b + c
    p = 0.5 * (a - c) / denom if denom < 0 else 0.0
    p = max(-0.5, min(0.5, p))
    freq = (k + p) * spec.bin_hz
    amp = math.exp(b - 0.25 * (a - c) * p)
    return freq, amp


def estimate_f0(x: np.ndarray, sr: int, expected: float | None = None, spec: Spectrum | None = None) -> float:
    """Fundamental frequency of a steady periodic segment.

    With an expected pitch the search is limited to +/- 1.5 semitones; the estimate is refined with
    the first harmonics (weighted by amplitude) so a weak fundamental still resolves precisely.
    """
    spec = spec or spectrum(x, sr, pad=4)
    if expected is None:
        # Harmonic sum over candidate fundamentals from 20 Hz to 5 kHz.
        amp_db = spec.amp_db()
        best, best_score = None, -1e18
        for f in np.geomspace(20.0, 5000.0, 2000):
            score = 0.0
            for h in range(1, 6):
                idx = int(round(h * f / spec.bin_hz))
                if idx >= len(amp_db):
                    break
                score += max(amp_db[idx], -140.0) / h
            if score > best_score:
                best, best_score = f, score
        expected = best
    lo, hi = expected * 2 ** (-1.5 / 12), expected * 2 ** (1.5 / 12)
    f1, a1 = interp_peak(spec, lo, hi)
    if not math.isfinite(f1):
        return float("nan")
    nyq = sr / 2
    estimates, weights = [f1], [a1]
    for h in range(2, 9):
        if h * f1 > nyq * 0.9:
            break
        fh, ah = interp_peak(spec, h * f1 - 0.25 * f1, h * f1 + 0.25 * f1)
        if math.isfinite(fh) and ah > a1 * 1e-3:
            estimates.append(fh / h)
            weights.append(ah * h)  # higher harmonics resolve the fundamental more finely
    return float(np.average(estimates, weights=weights))


# --------------------------------------------------------------------------------------------
# Harmonics and aliasing


@dataclass
class HarmonicReport:
    f0: float
    harmonics_db: list[float]  # dB relative to the fundamental, index 0 = h1
    harmonic_power: float
    inharmonic_power: float
    inharmonic_power_audible: float  # below `audible_hz` (20 kHz): folded energy people can hear
    worst_spur_dbc: float
    worst_spur_hz: float
    worst_audible_spur_dbc: float
    worst_audible_spur_hz: float
    harmonic_power_audible: float = 0.0
    spur_list: list[tuple[float, float]] = field(default_factory=list)

    @property
    def alias_db(self) -> float:
        return pdb(self.inharmonic_power / max(self.harmonic_power, 1e-300))

    @property
    def alias_audible_db(self) -> float:
        return pdb(self.inharmonic_power_audible / max(self.harmonic_power, 1e-300))


def harmonic_mask(spec: Spectrum, f0: float, spread: float = 0.0, extra_hz: float = 0.0) -> np.ndarray:
    """Bins that belong to harmonic k*f0, widened by the main lobe, a relative spread (unison
    detune) and a fixed margin."""
    guard = spec.guard_hz() + extra_hz
    f = spec.freqs
    k = np.maximum(1, np.round(f / f0))
    width = guard + k * f0 * spread
    return np.abs(f - k * f0) <= width


def harmonics(spec: Spectrum, f0: float, spread: float = 0.0, max_k: int | None = None,
              audible_hz: float = 20000.0, dc_hz: float = 15.0) -> HarmonicReport:
    nyq = spec.sr / 2
    kmax = int(nyq / f0) if max_k is None else min(max_k, int(nyq / f0))
    win = spec.guard_hz() + 0.0
    hs = []
    a1 = None
    for k in range(1, kmax + 1):
        lo = k * f0 * (1 - spread) - win * 0.5
        hi = k * f0 * (1 + spread) + win * 0.5
        _, a = interp_peak(spec, max(lo, dc_hz), min(hi, nyq))
        if a1 is None:
            a1 = max(a, 1e-30)
        hs.append(db(a / a1))

    mask = harmonic_mask(spec, f0, spread)
    valid = spec.freqs > dc_hz
    harm = spec.power[mask & valid].sum()
    inh_mask = (~mask) & valid
    inh = spec.power[inh_mask].sum()
    inh_aud = spec.power[inh_mask & (spec.freqs < audible_hz)].sum()
    harm_aud = spec.power[mask & valid & (spec.freqs < audible_hz)].sum()

    # Worst inharmonic spur (local maxima only).
    amp = np.where(inh_mask, spec.amp, 0.0)
    worst_i = int(np.argmax(amp))
    worst = db(amp[worst_i] / max(a1 or 1e-30, 1e-30))
    amp_aud = np.where(spec.freqs < audible_hz, amp, 0.0)
    worst_aud_i = int(np.argmax(amp_aud))
    worst_aud = db(amp_aud[worst_aud_i] / max(a1 or 1e-30, 1e-30))
    # A short list of the largest spurs (for the report), merged within the guard width.
    order = np.argsort(amp)[::-1]
    spurs: list[tuple[float, float]] = []
    for i in order[:4000]:
        if amp[i] <= 0:
            break
        fi = float(spec.freqs[i])
        if all(abs(fi - s[0]) > 4 * win for s in spurs):
            spurs.append((fi, db(amp[i] / max(a1 or 1e-30, 1e-30))))
        if len(spurs) >= 8:
            break
    return HarmonicReport(f0, hs, float(harm), float(inh), float(inh_aud), worst, float(spec.freqs[worst_i]),
                          worst_aud, float(spec.freqs[worst_aud_i]), float(harm_aud), spurs)


def thd(spec: Spectrum, f0: float, n: int = 10) -> dict:
    """Total harmonic distortion of a (nominally) sinusoidal signal and THD+N."""
    nyq = spec.sr / 2
    _, a1 = interp_peak(spec, f0 * 0.97, f0 * 1.03)
    hp = 0.0
    for k in range(2, n + 1):
        if k * f0 >= nyq:
            break
        _, a = interp_peak(spec, k * f0 * 0.99, k * f0 * 1.01)
        hp += a * a
    guard = spec.guard_hz()
    fund = np.abs(spec.freqs - f0) <= guard
    valid = spec.freqs > 15.0
    p_f = spec.power[fund].sum()
    p_rest = spec.power[valid & ~fund].sum()
    return {
        "thd_pct": 100.0 * math.sqrt(hp) / max(a1, 1e-30),
        "thd_db": db(math.sqrt(hp) / max(a1, 1e-30)),
        "thdn_db": pdb(p_rest / max(p_f, 1e-300)),
    }


def amplitude_at(spec: Spectrum, freq: float, rel_width: float = 0.02) -> float:
    _, a = interp_peak(spec, freq * (1 - rel_width), freq * (1 + rel_width))
    return a


# --------------------------------------------------------------------------------------------
# Envelopes and time-domain


def rms_envelope(x: np.ndarray, sr: int, win_s: float = 0.004, hop_s: float = 0.001) -> tuple[np.ndarray, np.ndarray]:
    """Sliding RMS (rectangular window) sampled every hop; returns (times at window centre, rms)."""
    x = np.asarray(x, dtype=np.float64)
    win = max(1, int(win_s * sr))
    hop = max(1, int(hop_s * sr))
    c = np.concatenate([[0.0], np.cumsum(x * x)])
    starts = np.arange(0, max(1, len(x) - win), hop)
    e = np.sqrt(np.maximum(0.0, (c[starts + win] - c[starts]) / win))
    t = (starts + win / 2) / sr
    return t, e


def first_crossing(t: np.ndarray, e: np.ndarray, level: float, start: int = 0, rising: bool = True) -> float:
    """Time of the first crossing of `level`, linearly interpolated."""
    for i in range(max(1, start), len(e)):
        a, b = e[i - 1], e[i]
        if (rising and a < level <= b) or (not rising and a > level >= b):
            frac = (level - a) / (b - a) if b != a else 0.0
            return float(t[i - 1] + frac * (t[i] - t[i - 1]))
    return float("nan")


def onset_time(x: np.ndarray, sr: int, rel_db: float = -40.0) -> float:
    """First sample whose magnitude exceeds rel_db relative to the peak of the signal."""
    a = np.abs(x)
    peak = a.max()
    if peak <= 0:
        return float("nan")
    idx = np.nonzero(a >= peak * 10 ** (rel_db / 20))[0]
    return float(idx[0] / sr) if len(idx) else float("nan")


def schroeder_decay(x: np.ndarray, sr: int) -> tuple[np.ndarray, np.ndarray]:
    """Energy decay curve in dB (Schroeder backward integration)."""
    e = np.asarray(x, dtype=np.float64) ** 2
    edc = np.cumsum(e[::-1])[::-1]
    edc_db = 10 * np.log10(np.maximum(edc / max(edc[0], 1e-300), 1e-30))
    return np.arange(len(x)) / sr, edc_db


def decay_time(t: np.ndarray, edc_db: np.ndarray, top: float, bottom: float) -> float:
    """RT60 extrapolated from the straight-line fit between `top` and `bottom` dB of the EDC."""
    sel = (edc_db <= top) & (edc_db >= bottom)
    if sel.sum() < 10:
        return float("nan")
    slope, _ = np.polyfit(t[sel], edc_db[sel], 1)
    return float(-60.0 / slope) if slope < 0 else float("nan")


# --------------------------------------------------------------------------------------------
# Stereo


def stereo_metrics(x: np.ndarray) -> dict:
    if x.ndim != 2 or x.shape[1] < 2:
        return {"correlation": 1.0, "side_to_mid_db": -300.0, "mono_loss_db": 0.0}
    l, r = x[:, 0], x[:, 1]
    m, s = 0.5 * (l + r), 0.5 * (l - r)
    pl, pr = float(np.mean(l * l)), float(np.mean(r * r))
    corr = float(np.mean(l * r) / math.sqrt(max(pl * pr, 1e-300)))
    pm, ps = float(np.mean(m * m)), float(np.mean(s * s))
    return {
        "correlation": corr,
        "side_to_mid_db": pdb(ps / max(pm, 1e-300)),
        # Level of the mono sum relative to the average channel level (0 dB = fully mono-safe).
        "mono_loss_db": pdb(pm / max(0.5 * (pl + pr), 1e-300)),
        "balance_db": pdb(pl / max(pr, 1e-300)),
    }


# --------------------------------------------------------------------------------------------
# Pitch tracking (for bends and vibrato)


def pitch_track(x: np.ndarray, sr: int, expected: float, frame: int = 4096, hop: int = 512,
                harmonic: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Frame-wise frequency of the given harmonic near `expected`, returned as the fundamental."""
    times, freqs = [], []
    w = np.kaiser(frame, 12.0)
    nfft = frame * 4
    fr = np.fft.rfftfreq(nfft, 1.0 / sr)
    bin_hz = sr / nfft
    target = expected * harmonic
    lo_i = max(1, int(target * 2 ** (-3 / 12) / bin_hz))
    hi_i = min(len(fr) - 2, int(target * 2 ** (3 / 12) / bin_hz))
    for start in range(0, len(x) - frame, hop):
        seg = x[start:start + frame]
        if not np.any(seg):
            continue
        X = np.abs(np.fft.rfft((seg - seg.mean()) * w, nfft))
        k = lo_i + int(np.argmax(X[lo_i:hi_i + 1]))
        a, b, c = (math.log(max(X[j], 1e-30)) for j in (k - 1, k, k + 1))
        den = a - 2 * b + c
        p = 0.5 * (a - c) / den if den < 0 else 0.0
        times.append((start + frame / 2) / sr)
        freqs.append((k + p) * bin_hz / harmonic)
    return np.array(times), np.array(freqs)
