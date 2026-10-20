# SPDX-License-Identifier: MIT
"""The :class:`Engine` class: offline rendering of a Crucible instrument from Python."""

from __future__ import annotations

import ctypes
import json
from typing import Any, Optional

import numpy as np

from . import _lib
from .errors import CrucibleError

_ERROR_SIZE = 4096
_RENDER_BLOCK = 256


class Engine:
    """One engine instance. Use as a context manager or call :meth:`close`.

    Everything runs on the calling thread, so the control and audio thread rules of the C API are met trivially.
    ``Engine()`` plays the default product; :meth:`from_product` takes product JSON (text, bytes or a dict).
    """

    def __init__(self, *, max_voices: int = 0, quality: int = 1, seed: int = 0, _handle: Optional[int] = None,
                 sample_rate: float = 48000.0) -> None:
        self._lib = _lib.load()
        self._handle: Optional[int] = _handle
        self.sample_rate = float(sample_rate)
        if _handle is None:
            self._handle = self._lib.crucible_create(self._config(max_voices, quality, seed))
            if not self._handle:
                self._handle = None
                raise CrucibleError("crucible_create failed")
        self._prepare()

    @staticmethod
    def _config(max_voices: int, quality: int, seed: int) -> Any:
        config = _lib.CrucibleConfig()
        _lib.load().crucible_config_init(ctypes.byref(config))
        config.max_voices = max_voices
        config.quality = quality
        config.seed = seed
        return ctypes.byref(config)

    @classmethod
    def from_product(cls, product: Any, *, max_voices: int = 0, quality: int = 1, seed: int = 0,
                     sample_rate: float = 48000.0) -> "Engine":
        """Builds an engine from a product definition; raises :class:`CrucibleError` with the engine's validation errors."""
        if isinstance(product, (dict, list)):
            product = json.dumps(product)
        if isinstance(product, str):
            product = product.encode("utf-8")
        lib = _lib.load()
        error = ctypes.create_string_buffer(_ERROR_SIZE)
        handle = lib.crucible_create_from_product(cls._config(max_voices, quality, seed), product, None, error,
                                                  _ERROR_SIZE)
        if not handle:
            raise CrucibleError(error.value.decode("utf-8", "replace") or "product rejected")
        return cls(_handle=handle, sample_rate=sample_rate)

    def _prepare(self) -> None:
        self._lib.crucible_prepare(self._h(), self.sample_rate, _RENDER_BLOCK)

    def _h(self) -> int:
        if self._handle is None:
            raise CrucibleError("engine is closed")
        return self._handle

    def close(self) -> None:
        """Destroys the engine; safe to call twice."""
        if self._handle is not None:
            self._lib.crucible_destroy(self._handle)
            self._handle = None

    def __enter__(self) -> "Engine":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:  # interpreter shutdown
            pass

    def prepare(self, sample_rate: float) -> None:
        """Sets the sample rate (render block size is fixed at 256)."""
        self.sample_rate = float(sample_rate)
        self._prepare()

    @property
    def param_count(self) -> int:
        return self._lib.crucible_param_count(self._h())

    def param_index(self, param_id: str) -> int:
        """Index of a parameter id; raises ``KeyError`` when unknown."""
        index = self._lib.crucible_param_index(self._h(), param_id.encode())
        if index < 0:
            raise KeyError(param_id)
        return index

    def set_param(self, param_id: str, value: float) -> None:
        """Sets a parameter by id in natural units."""
        if not self._lib.crucible_set_param_id(self._h(), param_id.encode(), float(value)):
            raise KeyError(param_id)

    def set_param_text(self, param_id: str, text: str) -> None:
        """Sets a parameter from display text such as ``"440 Hz"`` or ``"-6dB"``."""
        if not self._lib.crucible_set_param_text(self._h(), param_id.encode(), text.encode()):
            raise CrucibleError(f"cannot set {param_id!r} from text {text!r}")

    def get_param(self, param_id: str) -> float:
        """Current value of a parameter in natural units."""
        return float(self._lib.crucible_get_param(self._h(), self.param_index(param_id)))

    def param_text(self, param_id: str) -> str:
        """The display text of a parameter's current value."""
        index = self.param_index(param_id)
        value = self._lib.crucible_get_param(self._h(), index)
        buffer = ctypes.create_string_buffer(128)
        self._lib.crucible_value_to_text(self._h(), index, value, buffer, len(buffer))
        return buffer.value.decode("utf-8", "replace")

    def note_on(self, note: int, velocity: float = 0.8) -> None:
        """Starts a note (MIDI number 0..127, velocity 0..1)."""
        self._lib.crucible_note_on(self._h(), int(note), float(velocity))

    def note_off(self, note: int) -> None:
        self._lib.crucible_note_off(self._h(), int(note))

    def all_notes_off(self) -> None:
        self._lib.crucible_all_notes_off(self._h())

    def kill_all_voices(self) -> None:
        self._lib.crucible_kill_all_voices(self._h())

    def pitch_bend(self, value: float) -> None:
        """Pitch bend, -1..1."""
        self._lib.crucible_pitch_bend(self._h(), float(value))

    def mod_wheel(self, value: float) -> None:
        self._lib.crucible_mod_wheel(self._h(), float(value))

    def sustain(self, down: bool) -> None:
        self._lib.crucible_sustain(self._h(), int(bool(down)))

    def midi(self, status: int, data1: int = 0, data2: int = 0) -> None:
        """Sends a raw MIDI message."""
        self._lib.crucible_midi(self._h(), status, data1, data2)

    def render(self, seconds: Optional[float] = None, *, samples: Optional[int] = None) -> np.ndarray:
        """Renders audio and returns a float32 array of shape (n, 2). Give ``seconds`` or ``samples``."""
        if (seconds is None) == (samples is None):
            raise ValueError("give exactly one of seconds or samples")
        total = int(samples if samples is not None else round(seconds * self.sample_rate))
        if total < 0:
            raise ValueError("length must not be negative")
        handle = self._h()
        left = np.zeros(_RENDER_BLOCK, dtype=np.float32)
        right = np.zeros(_RENDER_BLOCK, dtype=np.float32)
        out = np.zeros((total, 2), dtype=np.float32)
        lp = left.ctypes.data_as(_lib.c_float_p)
        rp = right.ctypes.data_as(_lib.c_float_p)
        done = 0
        while done < total:
            n = min(_RENDER_BLOCK, total - done)
            self._lib.crucible_service(handle)
            self._lib.crucible_process_events(handle, lp, rp, n, None, 0, None, 0, None)
            out[done:done + n, 0] = left[:n]
            out[done:done + n, 1] = right[:n]
            done += n
        return out

    def _text_call(self, call: Any, what: str) -> str:
        """Runs a C call of the form f(buffer, capacity) -> full length, growing the buffer once when it was too small."""
        capacity = 1 << 16
        for _ in range(2):
            buffer = ctypes.create_string_buffer(capacity)
            length = call(buffer, capacity)
            if length < 0:
                raise CrucibleError(f"{what} failed")
            if length < capacity:
                return buffer.value.decode("utf-8")
            capacity = length + 1
        raise CrucibleError(f"{what} failed")

    def describe(self) -> dict:
        """The product's self-description (modules, parameters) as a dict."""
        handle = self._h()
        return json.loads(self._text_call(lambda b, n: self._lib.crucible_describe_json(handle, b, n), "describe"))

    def save_patch(self, name: str = "patch") -> str:
        """The current state as patch JSON text."""
        handle = self._h()
        return self._text_call(lambda b, n: self._lib.crucible_save_patch(handle, name.encode(), b, n), "save_patch")

    def reset(self) -> None:
        """Clears voices, held notes and every effect's state (reverb tails, delay lines); parameters are kept."""
        self._lib.crucible_reset(self._h())

    @property
    def latency_samples(self) -> int:
        """Latency of the whole engine in samples (look-ahead and linear-phase effects)."""
        return int(self._lib.crucible_latency_samples(self._h()))

    @property
    def tail_seconds(self) -> float:
        """Longest effect tail in seconds; ``inf`` when one never decays."""
        return float(self._lib.crucible_tail_seconds(self._h()))

    def load_patch(self, patch: Any) -> None:
        """Loads patch JSON (text, bytes or dict); raises :class:`CrucibleError` with the engine's reason."""
        if isinstance(patch, dict):
            patch = json.dumps(patch)
        if isinstance(patch, str):
            patch = patch.encode("utf-8")
        error = ctypes.create_string_buffer(_ERROR_SIZE)
        if not self._lib.crucible_load_patch(self._h(), patch, error, _ERROR_SIZE):
            raise CrucibleError(error.value.decode("utf-8", "replace") or "patch rejected")

    def add_route(self, source: int, target_id: str, amount: float) -> bool:
        """Adds or replaces a modulation route (source: ModSource value or 1000 + MIDI CC); False when refused."""
        return bool(self._lib.crucible_add_route(self._h(), int(source), target_id.encode(), float(amount)))

    def remove_route(self, source: int, target_id: str) -> bool:
        """Removes the route with this source and target; later routes move down one position."""
        return bool(self._lib.crucible_remove_route(self._h(), int(source), target_id.encode()))

    def routes(self) -> list[dict[str, Any]]:
        """The modulation routes in position order (a new pair is appended, a replaced pair keeps its position)."""
        handle = self._h()
        found = []
        route = _lib.CrucibleRoute()
        route.struct_size = ctypes.sizeof(route)
        for k in range(self._lib.crucible_route_count(handle)):
            if self._lib.crucible_route_get(handle, k, ctypes.byref(route)):
                found.append(
                    {
                        "source": route.source,
                        "target": route.target_id.decode(),
                        "amount": route.amount,
                        "polarity": route.polarity,
                        "curve": route.curve,
                        "smoothing_ms": route.smoothing_ms,
                    }
                )
        return found
