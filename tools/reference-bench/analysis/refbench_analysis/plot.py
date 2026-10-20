"""Line/bar charts written as PNG and SVG with numpy and the standard library only.

matplotlib is used for the PNG when it is installed; otherwise a small rasteriser with a built-in
5x7 font draws the chart (rendered at 2x and box-filtered for smooth lines). The SVG is always
written and carries the same chart with real text.
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:  # optional
    import matplotlib  # type: ignore

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # type: ignore

    HAVE_MPL = True
except Exception:  # pragma: no cover - depends on the environment
    HAVE_MPL = False

# Colour-blind safe series palette (Okabe-Ito), dark text on white.
PALETTE = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9", "#000000"]
BG = (255, 255, 255)
AXIS = (40, 40, 40)
GRID = (225, 225, 225)
TEXT = (20, 20, 20)

# 5x7 font, rows top to bottom, '#' = ink.
_F = {
    " ": ["     "] * 7,
    "0": [" ### ", "#   #", "#  ##", "# # #", "##  #", "#   #", " ### "],
    "1": ["  #  ", " ##  ", "  #  ", "  #  ", "  #  ", "  #  ", " ### "],
    "2": [" ### ", "#   #", "    #", "   # ", "  #  ", " #   ", "#####"],
    "3": ["#####", "   # ", "  #  ", "   # ", "    #", "#   #", " ### "],
    "4": ["   # ", "  ## ", " # # ", "#  # ", "#####", "   # ", "   # "],
    "5": ["#####", "#    ", "#### ", "    #", "    #", "#   #", " ### "],
    "6": ["  ## ", " #   ", "#    ", "#### ", "#   #", "#   #", " ### "],
    "7": ["#####", "    #", "   # ", "  #  ", " #   ", " #   ", " #   "],
    "8": [" ### ", "#   #", "#   #", " ### ", "#   #", "#   #", " ### "],
    "9": [" ### ", "#   #", "#   #", " ####", "    #", "   # ", " ##  "],
    "A": [" ### ", "#   #", "#   #", "#####", "#   #", "#   #", "#   #"],
    "B": ["#### ", "#   #", "#   #", "#### ", "#   #", "#   #", "#### "],
    "C": [" ### ", "#   #", "#    ", "#    ", "#    ", "#   #", " ### "],
    "D": ["#### ", "#   #", "#   #", "#   #", "#   #", "#   #", "#### "],
    "E": ["#####", "#    ", "#    ", "#### ", "#    ", "#    ", "#####"],
    "F": ["#####", "#    ", "#    ", "#### ", "#    ", "#    ", "#    "],
    "G": [" ### ", "#   #", "#    ", "# ###", "#   #", "#   #", " ####"],
    "H": ["#   #", "#   #", "#   #", "#####", "#   #", "#   #", "#   #"],
    "I": [" ### ", "  #  ", "  #  ", "  #  ", "  #  ", "  #  ", " ### "],
    "J": ["  ###", "   # ", "   # ", "   # ", "   # ", "#  # ", " ##  "],
    "K": ["#   #", "#  # ", "# #  ", "##   ", "# #  ", "#  # ", "#   #"],
    "L": ["#    ", "#    ", "#    ", "#    ", "#    ", "#    ", "#####"],
    "M": ["#   #", "## ##", "# # #", "# # #", "#   #", "#   #", "#   #"],
    "N": ["#   #", "#   #", "##  #", "# # #", "#  ##", "#   #", "#   #"],
    "O": [" ### ", "#   #", "#   #", "#   #", "#   #", "#   #", " ### "],
    "P": ["#### ", "#   #", "#   #", "#### ", "#    ", "#    ", "#    "],
    "Q": [" ### ", "#   #", "#   #", "#   #", "# # #", "#  # ", " ## #"],
    "R": ["#### ", "#   #", "#   #", "#### ", "# #  ", "#  # ", "#   #"],
    "S": [" ####", "#    ", "#    ", " ### ", "    #", "    #", "#### "],
    "T": ["#####", "  #  ", "  #  ", "  #  ", "  #  ", "  #  ", "  #  "],
    "U": ["#   #", "#   #", "#   #", "#   #", "#   #", "#   #", " ### "],
    "V": ["#   #", "#   #", "#   #", "#   #", "#   #", " # # ", "  #  "],
    "W": ["#   #", "#   #", "#   #", "# # #", "# # #", "# # #", " # # "],
    "X": ["#   #", "#   #", " # # ", "  #  ", " # # ", "#   #", "#   #"],
    "Y": ["#   #", "#   #", " # # ", "  #  ", "  #  ", "  #  ", "  #  "],
    "Z": ["#####", "    #", "   # ", "  #  ", " #   ", "#    ", "#####"],
    ".": ["     ", "     ", "     ", "     ", "     ", " ##  ", " ##  "],
    ",": ["     ", "     ", "     ", "     ", " ##  ", "  #  ", " #   "],
    "-": ["     ", "     ", "     ", "#####", "     ", "     ", "     "],
    "+": ["     ", "  #  ", "  #  ", "#####", "  #  ", "  #  ", "     "],
    ":": ["     ", " ##  ", " ##  ", "     ", " ##  ", " ##  ", "     "],
    "/": ["    #", "    #", "   # ", "  #  ", " #   ", "#    ", "#    "],
    "%": ["##  #", "##  #", "   # ", "  #  ", " #   ", "#  ##", "#  ##"],
    "(": ["   # ", "  #  ", " #   ", " #   ", " #   ", "  #  ", "   # "],
    ")": [" #   ", "  #  ", "   # ", "   # ", "   # ", "  #  ", " #   "],
    "[": [" ### ", " #   ", " #   ", " #   ", " #   ", " #   ", " ### "],
    "]": [" ### ", "   # ", "   # ", "   # ", "   # ", "   # ", " ### "],
    "=": ["     ", "     ", "#####", "     ", "#####", "     ", "     "],
    "_": ["     ", "     ", "     ", "     ", "     ", "     ", "#####"],
    "'": ["  #  ", "  #  ", " #   ", "     ", "     ", "     ", "     "],
    "<": ["   # ", "  #  ", " #   ", "#    ", " #   ", "  #  ", "   # "],
    ">": [" #   ", "  #  ", "   # ", "    #", "   # ", "  #  ", " #   "],
    "*": ["     ", "# # #", " ### ", "#####", " ### ", "# # #", "     "],
    "#": [" # # ", " # # ", "#####", " # # ", "#####", " # # ", " # # "],
    "?": [" ### ", "#   #", "    #", "   # ", "  #  ", "     ", "  #  "],
}


def _glyph(c: str) -> list[str]:
    return _F.get(c, _F.get(c.upper(), _F["?"]))


@dataclass
class Series:
    x: np.ndarray
    y: np.ndarray
    label: str
    color: str
    width: float = 1.5
    dashed: bool = False
    points: bool = False


@dataclass
class Chart:
    title: str
    xlabel: str = ""
    ylabel: str = ""
    xlog: bool = False
    xlim: tuple[float, float] | None = None
    ylim: tuple[float, float] | None = None
    series: list[Series] = field(default_factory=list)
    hlines: list[tuple[float, str]] = field(default_factory=list)
    vlines: list[tuple[float, str]] = field(default_factory=list)
    bars: list[tuple[str, list[float], list[str]]] | None = None  # (group label, values per series, labels)
    width: int = 900
    height: int = 520

    def line(self, x, y, label, color=None, **kw):
        color = color or PALETTE[len(self.series) % len(PALETTE)]
        self.series.append(Series(np.asarray(x, float), np.asarray(y, float), label, color, **kw))
        return self


def _hex(c: str) -> tuple[int, int, int]:
    c = c.lstrip("#")
    return int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16)


def _nice_ticks(lo: float, hi: float, n: int = 6) -> list[float]:
    if not (math.isfinite(lo) and math.isfinite(hi)) or hi <= lo:
        return [lo]
    span = hi - lo
    step = 10 ** math.floor(math.log10(span / n))
    for m in (1, 2, 2.5, 5, 10):
        if span / (step * m) <= n:
            step *= m
            break
    start = math.ceil(lo / step) * step
    ticks = []
    v = start
    while v <= hi + step * 1e-6:
        ticks.append(round(v, 10))
        v += step
    return ticks


def _log_ticks(lo: float, hi: float) -> list[float]:
    out = []
    for e in range(int(math.floor(math.log10(lo))), int(math.ceil(math.log10(hi))) + 1):
        for m in (1, 2, 5):
            v = m * 10 ** e
            if lo <= v <= hi:
                out.append(v)
    return out


def _fmt(v: float) -> str:
    if v == 0:
        return "0"
    a = abs(v)
    if a >= 1000:
        return f"{v / 1000:g}k"
    if a >= 1:
        return f"{v:g}"
    return f"{v:.3g}"


def _limits(chart: Chart) -> tuple[float, float, float, float]:
    xs = [s.x[np.isfinite(s.x) & np.isfinite(s.y)] for s in chart.series]
    ys = [s.y[np.isfinite(s.x) & np.isfinite(s.y)] for s in chart.series]
    xs = [a for a in xs if len(a)]
    ys = [a for a in ys if len(a)]
    if chart.xlim:
        x0, x1 = chart.xlim
    elif xs:
        x0, x1 = min(a.min() for a in xs), max(a.max() for a in xs)
    else:
        x0, x1 = 0.0, 1.0
    if chart.ylim:
        y0, y1 = chart.ylim
    elif ys:
        y0, y1 = min(a.min() for a in ys), max(a.max() for a in ys)
        pad = 0.05 * (y1 - y0 or 1.0)
        y0, y1 = y0 - pad, y1 + pad
    else:
        y0, y1 = 0.0, 1.0
    if chart.xlog:
        x0 = max(x0, 1e-9)
    if x1 <= x0:
        x1 = x0 + 1
    if y1 <= y0:
        y1 = y0 + 1
    return x0, x1, y0, y1


# --------------------------------------------------------------------------------------------
# SVG


def to_svg(chart: Chart) -> str:
    W, H = chart.width, chart.height
    L, R, T, B = 74, 20, 40, 64
    pw, ph = W - L - R, H - T - B
    x0, x1, y0, y1 = _limits(chart)

    def X(v):
        if chart.xlog:
            return L + pw * (math.log10(max(v, 1e-12)) - math.log10(x0)) / (math.log10(x1) - math.log10(x0))
        return L + pw * (v - x0) / (x1 - x0)

    def Y(v):
        return T + ph * (1 - (v - y0) / (y1 - y0))

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
           f'font-family="Helvetica, Arial, sans-serif" font-size="12">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{W / 2}" y="22" text-anchor="middle" font-size="15" font-weight="bold">{_esc(chart.title)}</text>']
    if chart.bars is None:
        xt = _log_ticks(x0, x1) if chart.xlog else _nice_ticks(x0, x1)
        for v in xt:
            out.append(f'<line x1="{X(v):.1f}" y1="{T}" x2="{X(v):.1f}" y2="{T + ph}" stroke="#e1e1e1"/>')
            out.append(f'<text x="{X(v):.1f}" y="{T + ph + 18}" text-anchor="middle">{_fmt(v)}</text>')
    for v in _nice_ticks(y0, y1):
        out.append(f'<line x1="{L}" y1="{Y(v):.1f}" x2="{L + pw}" y2="{Y(v):.1f}" stroke="#e1e1e1"/>')
        out.append(f'<text x="{L - 6}" y="{Y(v) + 4:.1f}" text-anchor="end">{_fmt(v)}</text>')
    out.append(f'<rect x="{L}" y="{T}" width="{pw}" height="{ph}" fill="none" stroke="#282828"/>')
    for v, label in chart.hlines:
        out.append(f'<line x1="{L}" y1="{Y(v):.1f}" x2="{L + pw}" y2="{Y(v):.1f}" stroke="#777" stroke-dasharray="5,4"/>')
        out.append(f'<text x="{L + pw - 4}" y="{Y(v) - 4:.1f}" text-anchor="end" fill="#555">{_esc(label)}</text>')
    for v, label in chart.vlines:
        out.append(f'<line x1="{X(v):.1f}" y1="{T}" x2="{X(v):.1f}" y2="{T + ph}" stroke="#777" stroke-dasharray="5,4"/>')
        out.append(f'<text x="{X(v) + 4:.1f}" y="{T + 14}" fill="#555">{_esc(label)}</text>')

    out.append(f'<clipPath id="plot"><rect x="{L}" y="{T}" width="{pw}" height="{ph}"/></clipPath>')
    if chart.bars is not None:
        groups = chart.bars
        n_series = max((len(g[1]) for g in groups), default=1)
        gw = pw / max(1, len(groups))
        bw = gw * 0.8 / n_series
        for gi, (glabel, values, labels) in enumerate(groups):
            for si, v in enumerate(values):
                if v is None or not math.isfinite(v):
                    continue
                bx = L + gi * gw + gw * 0.1 + si * bw
                top, base = Y(max(v, y0)), Y(max(min(0.0, y1), y0))
                out.append(f'<rect x="{bx:.1f}" y="{min(top, base):.1f}" width="{bw * 0.9:.1f}" '
                           f'height="{abs(base - top):.1f}" fill="{PALETTE[si % len(PALETTE)]}"/>')
            out.append(f'<text x="{L + gi * gw + gw / 2:.1f}" y="{T + ph + 18}" text-anchor="middle">{_esc(glabel)}</text>')
    for s in chart.series:
        ok = np.isfinite(s.x) & np.isfinite(s.y)
        pts = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in zip(s.x[ok], s.y[ok]))
        dash = ' stroke-dasharray="6,4"' if s.dashed else ""
        if s.points:
            for a, b in zip(s.x[ok], s.y[ok]):
                out.append(f'<circle cx="{X(a):.1f}" cy="{Y(b):.1f}" r="3.5" fill="{s.color}" clip-path="url(#plot)"/>')
        out.append(f'<polyline points="{pts}" fill="none" stroke="{s.color}" stroke-width="{s.width}"{dash} '
                   f'clip-path="url(#plot)"/>')
    labels = [(s.label, s.color) for s in chart.series]
    if chart.bars is not None and chart.bars:
        labels = [(lab, PALETTE[i % len(PALETTE)]) for i, lab in enumerate(chart.bars[0][2])]
    for i, (label, color) in enumerate(labels):
        out.append(f'<rect x="{L + 10}" y="{T + 8 + i * 18}" width="14" height="4" fill="{color}"/>')
        out.append(f'<text x="{L + 30}" y="{T + 14 + i * 18}">{_esc(label)}</text>')
    out.append(f'<text x="{L + pw / 2}" y="{H - 18}" text-anchor="middle">{_esc(chart.xlabel)}</text>')
    out.append(f'<text transform="translate(18 {T + ph / 2}) rotate(-90)" text-anchor="middle">{_esc(chart.ylabel)}</text>')
    out.append("</svg>")
    return "\n".join(out)


def _esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


# --------------------------------------------------------------------------------------------
# Raster


class _Canvas:
    def __init__(self, w: int, h: int, scale: int = 2):
        self.s = scale
        self.w, self.h = w * scale, h * scale
        self.px = np.empty((self.h, self.w, 3), dtype=np.uint8)
        self.px[:] = BG

    def rect(self, x0, y0, x1, y1, color):
        s = self.s
        a, b = int(round(min(x0, x1) * s)), int(round(max(x0, x1) * s))
        c, d = int(round(min(y0, y1) * s)), int(round(max(y0, y1) * s))
        a, b, c, d = max(0, a), min(self.w, b), max(0, c), min(self.h, d)
        if b > a and d > c:
            self.px[c:d, a:b] = color

    def line(self, x0, y0, x1, y1, color, width=1.0, clip=None, dash=None):
        s = self.s
        x0, y0, x1, y1 = x0 * s, y0 * s, x1 * s, y1 * s
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        r = max(1, int(round(width * s / 2)))
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, n)
        if dash:
            seg = np.arange(n) % (dash[0] + dash[1]) * s < dash[0] * s
            xs, ys = xs[seg], ys[seg]
        if clip:
            cx0, cy0, cx1, cy1 = (v * s for v in clip)
            ok = (xs >= cx0) & (xs <= cx1) & (ys >= cy0) & (ys <= cy1)
            xs, ys = xs[ok], ys[ok]
        xi, yi = np.round(xs).astype(int), np.round(ys).astype(int)
        for dx in range(-r + 1, r):
            for dy in range(-r + 1, r):
                X, Y = xi + dx, yi + dy
                ok = (X >= 0) & (X < self.w) & (Y >= 0) & (Y < self.h)
                self.px[Y[ok], X[ok]] = color

    def text(self, x, y, s_: str, color=TEXT, size=1, anchor="left", vertical=False):
        """Draws text with the 5x7 font; `size` multiplies the pixel scale; y is the baseline-top."""
        sc = self.s * size
        adv = 6 * sc
        total = len(s_) * adv
        if vertical:
            px = int(x * self.s)
            py = int(y * self.s + (total / 2 if anchor == "middle" else 0))
            for ch in s_:
                g = _glyph(ch)
                for r_, row in enumerate(g):
                    for c_, v in enumerate(row):
                        if v == "#":
                            yy = py - c_ * sc
                            xx = px + r_ * sc
                            self.px[max(0, yy - sc):max(0, yy), max(0, xx):xx + sc] = color
                py -= adv
            return
        px = int(x * self.s - (total / 2 if anchor == "middle" else total if anchor == "right" else 0))
        py = int(y * self.s)
        for ch in s_:
            g = _glyph(ch)
            for r_, row in enumerate(g):
                for c_, v in enumerate(row):
                    if v == "#":
                        yy, xx = py + r_ * sc, px + c_ * sc
                        if 0 <= yy < self.h and 0 <= xx < self.w:
                            self.px[yy:yy + sc, xx:xx + sc] = color
            px += adv

    def png(self) -> bytes:
        s = self.s
        img = self.px.reshape(self.h // s, s, self.w // s, s, 3).mean(axis=(1, 3)).astype(np.uint8)
        h, w = img.shape[:2]
        raw = b"".join(b"\x00" + img[i].tobytes() for i in range(h))

        def chunk(tag, data):
            return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

        return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def to_png_raster(chart: Chart) -> bytes:
    W, H = chart.width, chart.height
    L, R, T, B = 74, 20, 40, 64
    pw, ph = W - L - R, H - T - B
    x0, x1, y0, y1 = _limits(chart)
    c = _Canvas(W, H)

    def X(v):
        if chart.xlog:
            return L + pw * (math.log10(max(v, 1e-12)) - math.log10(x0)) / (math.log10(x1) - math.log10(x0))
        return L + pw * (v - x0) / (x1 - x0)

    def Y(v):
        return T + ph * (1 - (v - y0) / (y1 - y0))

    clip = (L, T, L + pw, T + ph)
    if chart.bars is None:
        for v in (_log_ticks(x0, x1) if chart.xlog else _nice_ticks(x0, x1)):
            c.line(X(v), T, X(v), T + ph, GRID)
            c.text(X(v), T + ph + 8, _fmt(v), anchor="middle")
    for v in _nice_ticks(y0, y1):
        c.line(L, Y(v), L + pw, Y(v), GRID)
        c.text(L - 6, Y(v) - 3.5, _fmt(v), anchor="right")
    for v, label in chart.hlines:
        c.line(L, Y(v), L + pw, Y(v), (120, 120, 120), dash=(5, 4), clip=clip)
        c.text(L + pw - 4, Y(v) - 12, label, color=(90, 90, 90), anchor="right")
    for v, label in chart.vlines:
        c.line(X(v), T, X(v), T + ph, (120, 120, 120), dash=(5, 4), clip=clip)
        c.text(X(v) + 4, T + 6, label, color=(90, 90, 90))
    if chart.bars is not None:
        groups = chart.bars
        n_series = max((len(g[1]) for g in groups), default=1)
        gw = pw / max(1, len(groups))
        bw = gw * 0.8 / n_series
        for gi, (glabel, values, _) in enumerate(groups):
            for si, v in enumerate(values):
                if v is None or not math.isfinite(v):
                    continue
                bx = L + gi * gw + gw * 0.1 + si * bw
                c.rect(bx, Y(max(v, y0)), bx + bw * 0.9, Y(max(min(0.0, y1), y0)), _hex(PALETTE[si % len(PALETTE)]))
            c.text(L + gi * gw + gw / 2, T + ph + 8, glabel, anchor="middle")
    for s in chart.series:
        ok = np.isfinite(s.x) & np.isfinite(s.y)
        xs, ys = s.x[ok], s.y[ok]
        col = _hex(s.color)
        for i in range(1, len(xs)):
            c.line(X(xs[i - 1]), Y(ys[i - 1]), X(xs[i]), Y(ys[i]), col, s.width, clip=clip,
                   dash=(6, 4) if s.dashed else None)
        if s.points:
            for a, b in zip(xs, ys):
                c.rect(X(a) - 3, Y(b) - 3, X(a) + 3, Y(b) + 3, col)
    c.line(L, T, L + pw, T, AXIS)
    c.line(L, T + ph, L + pw, T + ph, AXIS)
    c.line(L, T, L, T + ph, AXIS)
    c.line(L + pw, T, L + pw, T + ph, AXIS)
    labels = [(s.label, s.color) for s in chart.series]
    if chart.bars is not None and chart.bars:
        labels = [(lab, PALETTE[i % len(PALETTE)]) for i, lab in enumerate(chart.bars[0][2])]
    for i, (label, color) in enumerate(labels):
        c.rect(L + 10, T + 10 + i * 18, L + 24, T + 14 + i * 18, _hex(color))
        c.text(L + 30, T + 8 + i * 18, label)
    c.text(W / 2, 12, chart.title, size=2 if len(chart.title) * 12 < W - 20 else 1, anchor="middle")
    c.text(L + pw / 2, H - 24, chart.xlabel, anchor="middle")
    c.text(16, T + ph / 2, chart.ylabel, anchor="middle", vertical=True)
    return c.png()


def to_png_mpl(chart: Chart, path: Path) -> None:  # pragma: no cover - optional dependency
    fig, ax = plt.subplots(figsize=(chart.width / 100, chart.height / 100), dpi=100)
    if chart.bars is not None:
        groups = chart.bars
        n = max((len(g[1]) for g in groups), default=1)
        for si in range(n):
            xs = [gi + si * 0.8 / n for gi in range(len(groups))]
            ax.bar(xs, [g[1][si] if si < len(g[1]) else 0 for g in groups], width=0.8 / n * 0.9,
                   color=PALETTE[si % len(PALETTE)], label=groups[0][2][si])
        ax.set_xticks([gi + 0.4 - 0.4 / n for gi in range(len(groups))], [g[0] for g in groups])
    for s in chart.series:
        ax.plot(s.x, s.y, color=s.color, lw=s.width, ls="--" if s.dashed else "-", label=s.label,
                marker="o" if s.points else None)
    for v, label in chart.hlines:
        ax.axhline(v, color="#777", ls="--", lw=1)
    for v, label in chart.vlines:
        ax.axvline(v, color="#777", ls="--", lw=1)
    if chart.xlog:
        ax.set_xscale("log")
    x0, x1, y0, y1 = _limits(chart)
    if chart.bars is None:
        ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_title(chart.title)
    ax.set_xlabel(chart.xlabel)
    ax.set_ylabel(chart.ylabel)
    ax.grid(True, color="#e1e1e1")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save(chart: Chart, path_base: str | Path) -> list[Path]:
    """Writes <base>.png and <base>.svg; returns the paths."""
    base = Path(path_base)
    base.parent.mkdir(parents=True, exist_ok=True)
    png, svg = base.with_suffix(".png"), base.with_suffix(".svg")
    if HAVE_MPL:
        to_png_mpl(chart, png)
    else:
        png.write_bytes(to_png_raster(chart))
    svg.write_text(to_svg(chart))
    return [png, svg]
