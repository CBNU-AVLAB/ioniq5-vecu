# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      servo.py
# @brief     Single-axis position servo physical model
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""Single-axis position servo physical model (unit-agnostic, CAN/dbc-independent).

Used as-is for both steering (deg) and braking (mm) - the pure physics part (common
logic collected, units/limits injected). It knows nothing about CAN encoding or
message names; the ecus/ layer bridges dbc signals to this model.

Behavior:
* enable(SON)=True + a target -> each step tracks target at max_speed (slew-rate
  limited), clamped to limits.
* enable=False -> hold position at zero speed (control mode OFF). Manual back-drive
  pushes position directly via set_position() from outside (side channel).
"""
from __future__ import annotations


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class ServoModel:
    """Position-tracking servo. All values in physical units (deg or mm)."""

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

    # -- inputs ----------------------------------------------------------------
    def set_target(self, target: float) -> None:
        """AD position command (target_pos). Clamped to limits before storing."""
        self.target = _clamp(target, self.limit_min, self.limit_max)

    def set_enabled(self, enabled: bool) -> None:
        """SON. On falling to False, stop immediately (velocity 0)."""
        self.enabled = bool(enabled)
        if not self.enabled:
            self.velocity = 0.0

    def set_position(self, position: float) -> None:
        """External direct position set (e.g. manual back-drive via the side channel)."""
        self.position = _clamp(position, self.limit_min, self.limit_max)
        self.target = self.position
        self.velocity = 0.0

    # -- integration -----------------------------------------------------------
    def step(self, dt: float) -> float:
        """Advance time by dt seconds. Returns the updated position.

        Only when enabled, move toward target by at most max_speed*dt.
        """
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

    # -- state queries (used by status frames, etc.) ---------------------------
    def in_position(self, tol: float = 1e-3) -> bool:
        """Whether target is reached (INP)."""
        return abs(self.target - self.position) <= tol

    def at_zero_speed(self, tol: float = 1e-6) -> bool:
        """Whether stopped (ZSP)."""
        return abs(self.velocity) <= tol
