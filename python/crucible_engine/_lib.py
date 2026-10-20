# SPDX-License-Identifier: MIT
"""Locates the Crucible shared library and declares the C API for ctypes.

Every ``argtypes`` / ``restype`` lives in :func:`_declare`, so a change to ``crucible.h`` is a change to this file only.
"""

from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path
from typing import Iterator, Optional

ENV_VAR = "CRUCIBLE_LIBRARY"

c_float_p = ctypes.POINTER(ctypes.c_float)


class CrucibleConfig(ctypes.Structure):
    """Mirror of ``CrucibleConfig`` in crucible.h."""

    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("max_voices", ctypes.c_int),
        ("max_block_size", ctypes.c_int),
        ("quality", ctypes.c_int),
        ("seed", ctypes.c_uint32),
        ("preload_wavetables", ctypes.c_int),
        ("oversampling_filter", ctypes.c_int),
    ]


class CrucibleRoute(ctypes.Structure):
    """Mirror of ``CrucibleRoute`` in crucible.h (``target_id`` from crucible_route_get lives as long as the engine)."""

    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("source", ctypes.c_int),
        ("target_id", ctypes.c_char_p),
        ("amount", ctypes.c_float),
        ("polarity", ctypes.c_int),
        ("curve", ctypes.c_float),
        ("smoothing_ms", ctypes.c_float),
    ]


CHANGE_LATENCY = -1
CHANGE_TAIL = -2
CHANGE_ROUTES = -3


def _library_names() -> list[str]:
    if sys.platform == "darwin":
        return ["libcrucible.dylib"]
    if sys.platform == "win32":
        return ["crucible.dll", "libcrucible.dll"]
    return ["libcrucible.so"]


def _candidates() -> Iterator[Path]:
    names = _library_names()
    env = os.environ.get(ENV_VAR)
    if env:
        yield Path(env)
    package_dir = Path(__file__).resolve().parent
    for name in names:
        yield package_dir / name
    # A source checkout: <repo>/python/crucible_engine/_lib.py, with CMake build directories at <repo>/build*.
    repo = package_dir.parent.parent
    for build in sorted(repo.glob("build*")):
        for name in names:
            yield build / name
            yield build / "Release" / name


def find_library() -> Path:
    """Returns the path of the shared library, or raises ``FileNotFoundError`` listing where it looked."""
    tried = []
    for path in _candidates():
        if path.is_file():
            return path
        tried.append(str(path))
    raise FileNotFoundError(
        f"Crucible shared library not found. Build it with cmake -DCRUCIBLE_ENGINE_SHARED=ON, or set {ENV_VAR} to its "
        "path. Looked in: " + ", ".join(tried)
    )


def _declare(lib: ctypes.CDLL) -> None:
    c = ctypes
    vp, ci, cf, cs = c.c_void_p, c.c_int, c.c_float, c.c_char_p
    sig = {
        "crucible_api_version": (ci, []),
        "crucible_version": (cs, []),
        "crucible_config_init": (None, [c.POINTER(CrucibleConfig)]),
        "crucible_create": (vp, [c.POINTER(CrucibleConfig)]),
        "crucible_create_from_product": (vp, [c.POINTER(CrucibleConfig), cs, vp, c.c_char_p, c.c_size_t]),
        "crucible_destroy": (None, [vp]),
        "crucible_prepare": (None, [vp, c.c_double, ci]),
        "crucible_param_count": (ci, [vp]),
        "crucible_param_index": (ci, [vp, cs]),
        "crucible_value_to_text": (ci, [vp, ci, cf, c.c_char_p, c.c_size_t]),
        "crucible_text_to_value": (ci, [vp, ci, cs, c.POINTER(cf)]),
        "crucible_describe_json": (ci, [vp, c.c_char_p, c.c_size_t]),
        "crucible_set_param": (ci, [vp, ci, cf]),
        "crucible_set_param_id": (ci, [vp, cs, cf]),
        "crucible_set_param_text": (ci, [vp, cs, cs]),
        "crucible_get_param": (cf, [vp, ci]),
        "crucible_note_on": (ci, [vp, ci, cf]),
        "crucible_note_off": (ci, [vp, ci]),
        "crucible_pitch_bend": (ci, [vp, cf]),
        "crucible_mod_wheel": (ci, [vp, cf]),
        "crucible_sustain": (ci, [vp, ci]),
        "crucible_all_notes_off": (ci, [vp]),
        "crucible_kill_all_voices": (ci, [vp]),
        "crucible_midi": (ci, [vp, c.c_uint8, c.c_uint8, c.c_uint8]),
        "crucible_process_events": (None, [vp, c_float_p, c_float_p, ci, vp, ci, vp, ci, vp]),
        "crucible_service": (None, [vp]),
        "crucible_load_patch": (ci, [vp, cs, c.c_char_p, c.c_size_t]),
        "crucible_save_patch": (ci, [vp, cs, c.c_char_p, c.c_size_t]),
        "crucible_latency_samples": (ci, [vp]),
        "crucible_tail_seconds": (c.c_double, [vp]),
        "crucible_reset": (None, [vp]),
        "crucible_param_changes": (ci, [vp, c.POINTER(ci), c.POINTER(ci), ci]),
        "crucible_add_route": (ci, [vp, ci, cs, cf]),
        "crucible_add_route_ex": (ci, [vp, c.POINTER(CrucibleRoute)]),
        "crucible_remove_route": (ci, [vp, ci, cs]),
        "crucible_route_count": (ci, [vp]),
        "crucible_route_get": (ci, [vp, ci, c.POINTER(CrucibleRoute)]),
    }
    for name, (restype, argtypes) in sig.items():
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes


_cached: Optional[ctypes.CDLL] = None


def load() -> ctypes.CDLL:
    """Loads (once) and returns the declared library."""
    global _cached
    if _cached is None:
        lib = ctypes.CDLL(str(find_library()))
        _declare(lib)
        _cached = lib
    return _cached
