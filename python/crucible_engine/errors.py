# SPDX-License-Identifier: MIT
"""Exceptions raised by the Crucible binding."""


class CrucibleError(Exception):
    """An engine call failed; the message is the engine's own reason where it gives one."""
