# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      servo.py
# @brief     Single-axis position servo physical model
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
Single-axis position servo model (unit-agnostic, independent of CAN and the dbc).

* enabled (SON=1): moves toward the target at most max_speed per second, clamped to the limits.
* disabled: holds the position with zero velocity; set_position() moves it directly.
"""

from __future__ import annotations


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class ServoModel:
    """Position-tracking servo. All values are in physical units (deg or mm)."""

    def __init__(
        self,
        limit_min: float,
        limit_max: float,
        max_speed: float,
        position: float = 0.0,
    ) -> None:
        if limit_min > limit_max:
            raise ValueError("limit_min > limit_max")
        if max_speed <= 0:
            raise ValueError("max_speed must be > 0")
        self.limit_min = limit_min
        self.limit_max = limit_max
        self.max_speed = max_speed
        self.position = _clamp(position, limit_min, limit_max)
        self.target = self.position
        self.velocity = 0.0
        self.enabled = False

    # ── Inputs ────────────────────────────────────────────────────────────
    def set_target(self, target: float) -> None:
        """AD position command (target_pos), clamped to the limits."""
        self.target = _clamp(target, self.limit_min, self.limit_max)

    def set_enabled(self, enabled: bool) -> None:
        """SON. Disabling stops immediately (velocity 0)."""
        self.enabled = bool(enabled)
        if not self.enabled:
            self.velocity = 0.0

    def set_position(self, position: float) -> None:
        """Set the position directly (manual back-drive, zero calibration)."""
        self.position = _clamp(position, self.limit_min, self.limit_max)
        self.target = self.position
        self.velocity = 0.0

    # ── Integration ───────────────────────────────────────────────────────
    def step(self, dt: float) -> float:
        """Advance by dt seconds and return the position. Moves only while enabled."""
        if dt <= 0:
            return self.position
        if not self.enabled:
            self.velocity = 0.0
            return self.position

        err = self.target - self.position
        max_step = self.max_speed * dt
        step = _clamp(err, -max_step, max_step)
        self.position = _clamp(self.position + step, self.limit_min, self.limit_max)
        self.velocity = step / dt
        return self.position

    # ── Status ────────────────────────────────────────────────────────────
    def in_position(self, tol: float = 1e-3) -> bool:
        """True when the target is reached (INP)."""
        return abs(self.target - self.position) <= tol

    def at_zero_speed(self, tol: float = 1e-6) -> bool:
        """True when stopped (ZSP)."""
        return abs(self.velocity) <= tol
