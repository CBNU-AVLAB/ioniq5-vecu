# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      manual_protocol.py
# @brief     Manual control side-channel wire format
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
Wire format of the manual-control side channel (shared by sender and receiver).

Payload (JSON):
    {"steer": <-1..1 rate>, "brake": <0..1>, "accel": <0..1>}

* steer is a turning rate; the vECU integrates it over time.
* brake / accel are absolute pedal positions.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional

MANUAL_HOST = "127.0.0.1"
MANUAL_PORT = 47100  # manual-control UDP port


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass(frozen=True)
class ManualInput:
    """Manual input at one instant. clamped() limits the values to their ranges."""

    steer: float = 0.0  # turning rate -1..+1 (left +1 / right -1)
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
    """Bytes -> ManualInput, or None for malformed data."""
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
