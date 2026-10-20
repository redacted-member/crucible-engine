"""Minimal RIFF/WAVE reader (PCM 16/24/32 and IEEE float 32/64) using numpy only."""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np


class WavError(Exception):
    pass


def read_wav(path: str | Path) -> tuple[int, np.ndarray]:
    """Returns (sample_rate, samples[n, channels]) as float64 in [-1, 1]."""
    data = Path(path).read_bytes()
    if len(data) < 12 or data[0:4] != b"RIFF" or data[8:12] != b"WAVE":
        raise WavError(f"{path}: not a RIFF/WAVE file")

    pos = 12
    fmt = None
    pcm = None
    while pos + 8 <= len(data):
        chunk_id = data[pos:pos + 4]
        size = struct.unpack("<I", data[pos + 4:pos + 8])[0]
        body = data[pos + 8:pos + 8 + size]
        if chunk_id == b"fmt ":
            fmt = body
        elif chunk_id == b"data":
            pcm = body
        pos += 8 + size + (size & 1)

    if fmt is None or pcm is None:
        raise WavError(f"{path}: missing fmt or data chunk")

    tag, channels, rate, _, block_align, bits = struct.unpack("<HHIIHH", fmt[:16])
    if tag == 0xFFFE and len(fmt) >= 26:  # WAVE_FORMAT_EXTENSIBLE: sub-format GUID starts at 24
        tag = struct.unpack("<H", fmt[24:26])[0]

    frames = len(pcm) // block_align
    pcm = pcm[: frames * block_align]
    if tag == 3 and bits == 32:
        x = np.frombuffer(pcm, dtype="<f4").astype(np.float64)
    elif tag == 3 and bits == 64:
        x = np.frombuffer(pcm, dtype="<f8").copy()
    elif tag == 1 and bits == 16:
        x = np.frombuffer(pcm, dtype="<i2").astype(np.float64) / 32768.0
    elif tag == 1 and bits == 32:
        x = np.frombuffer(pcm, dtype="<i4").astype(np.float64) / 2147483648.0
    elif tag == 1 and bits == 24:
        b = np.frombuffer(pcm, dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
        v = np.where(v >= 1 << 23, v - (1 << 24), v)
        x = v.astype(np.float64) / 8388608.0
    else:
        raise WavError(f"{path}: unsupported format tag {tag} with {bits} bits")
    return rate, x.reshape(-1, channels)


def write_wav_float(path: str | Path, rate: int, samples: np.ndarray) -> None:
    """Writes 32-bit float WAV (used by tests and synthetic references)."""
    x = np.asarray(samples, dtype="<f4")
    if x.ndim == 1:
        x = x[:, None]
    channels = x.shape[1]
    body = x.tobytes()
    fmt = struct.pack("<HHIIHH", 3, channels, rate, rate * channels * 4, channels * 4, 32)
    riff = b"WAVE" + b"fmt " + struct.pack("<I", len(fmt)) + fmt + b"data" + struct.pack("<I", len(body)) + body
    Path(path).write_bytes(b"RIFF" + struct.pack("<I", len(riff)) + riff)
