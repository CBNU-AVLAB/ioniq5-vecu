# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      manual_protocol.py
# @brief     Manual control side-channel wire format
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""Wire format for the manual-control side channel (single shared definition for both ends).

The pure (no-socket) part - host console/input.py (sender) and container
io/manual_channel.py (receiver) both import it to share the same format.

Payload (JSON):
    {"steer": <-1..1 rate>, "brake": <0..1>, "accel": <0..1>}

* steer is a "wheel-turning speed" (normalized rate). The vECU integrates it with dt
  to back-drive, so it's insensitive to message rate and naturally stops when input
  stops (staleness).
* brake/accel are pedal depression (absolute ratio).
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional

MANUAL_HOST = "127.0.0.1"
MANUAL_PORT = 47100  # manual-control UDP port (spec-independent, side channel only)


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass(frozen=True)
class ManualInput:
    """A single manual-control input sample. Ranges are clamped at construction."""

    steer: float = 0.0  # -1 (left) .. +1 (right) rate
    brake: float = 0.0  # 0 .. 1
    accel: float = 0.0  # 0 .. 1

    def clamped(self) -> "ManualInput":
        return ManualInput(
            steer=_clamp(self.steer, -1.0, 1.0),
            brake=_clamp(self.brake, 0.0, 1.0),
            accel=_clamp(self.accel, 0.0, 1.0),
        )


NEUTRAL = ManualInput()


def encode(mi: ManualInput) -> bytes:
    mi = mi.clamped()
    return json.dumps(
        {"steer": mi.steer, "brake": mi.brake, "accel": mi.accel}
    ).encode("utf-8")


def decode(data: bytes) -> Optional[ManualInput]:
    """Received bytes -> ManualInput. None (ignored) on broken JSON/format."""
    try:
        obj = json.loads(data.decode("utf-8"))
        if not isinstance(obj, dict):
            return None
        return ManualInput(
            steer=float(obj.get("steer", 0.0)),
            brake=float(obj.get("brake", 0.0)),
            accel=float(obj.get("accel", 0.0)),
        ).clamped()
    except (ValueError, TypeError, UnicodeDecodeError):
        return None
