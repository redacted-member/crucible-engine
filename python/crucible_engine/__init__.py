# SPDX-License-Identifier: MIT
"""Offline rendering of the Crucible synth engine from Python, over the plain C API (ctypes + NumPy)."""

from .engine import Engine
from .errors import CrucibleError

__all__ = ["Engine", "CrucibleError", "version"]


def version() -> str:
    """The engine library version."""
    from . import _lib

    return _lib.load().crucible_version().decode()
