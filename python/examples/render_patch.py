# SPDX-License-Identifier: MIT
"""Renders a note (optionally with a patch loaded) to a WAV file: python render_patch.py out.wav [patch.json]"""

import sys
import wave

import numpy as np

from crucible_engine import Engine

with Engine() as engine:
    if len(sys.argv) > 2:
        engine.load_patch(open(sys.argv[2], encoding="utf-8").read())
    engine.note_on(60, 0.8)
    audio = engine.render(2.0)
    engine.note_off(60)
    audio = np.concatenate([audio, engine.render(1.0)])
    pcm = (np.clip(audio, -1.0, 1.0) * 32767.0).astype("<i2")
    with wave.open(sys.argv[1] if len(sys.argv) > 1 else "out.wav", "wb") as out:
        out.setnchannels(2)
        out.setsampwidth(2)
        out.setframerate(int(engine.sample_rate))
        out.writeframes(pcm.tobytes())
