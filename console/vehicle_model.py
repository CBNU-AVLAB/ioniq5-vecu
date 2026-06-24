#!/usr/bin/env python3
# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      vehicle_model.py
# @brief     Display-side longitudinal speed model
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""Display-side speed model (host-native, display only).

Speed is not in the official CAN matrix, so the cluster derives it from acceleration
(APS_OUT_PERCENT) and braking (brake stroke) with a simple longitudinal model, for
display only. Nothing is sent on vcan0.

Physics (simplified):
  a = drive - brake - resist
    drive  = (accel% / 100) * accel_ms2
    brake  = (stroke / stroke_max) * brake_ms2
    resist = roll + aero * v        (rolling + aero drag, always decelerating)
  v += a*dt,  0 <= v <= max_speed
Coefficients are pre-measurement assumptions for display feel (to be tuned).
"""
from __future__ import annotations

KMH_PER_MS = 3.6


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class VehicleModel:
    """Longitudinal speed integrator. Internal state in m/s, exposed as km/h."""

    def __init__(
        self,
        max_kmh: float = 180.0,
        accel_ms2: float = 3.5,   # full-throttle acceleration
        brake_ms2: float = 6.0,   # full-braking deceleration
        roll: float = 0.15,       # rolling resistance (constant decel) m/s^2
        aero: float = 0.012,      # aero drag coefficient (proportional to v) 1/s
    ) -> None:
        self.max_ms = max_kmh / KMH_PER_MS
        self.accel_ms2 = accel_ms2
        self.brake_ms2 = brake_ms2
        self.roll = roll
        self.aero = aero
        self.v = 0.0  # m/s

    def reset(self) -> None:
        self.v = 0.0

    def step(self, dt: float, accel_pct: float, brake_mm: float,
             brake_max_mm: float = 60.0) -> float:
        """Advance by dt seconds. accel_pct (0-100), brake_mm (0-max). Returns km/h."""
        if dt <= 0:
            return self.v * KMH_PER_MS
        drive = _clamp(accel_pct, 0.0, 100.0) / 100.0 * self.accel_ms2
        brake = (_clamp(brake_mm, 0.0, brake_max_mm) / brake_max_mm) * self.brake_ms2
        resist = self.roll + self.aero * self.v   # always decelerating
        self.v = _clamp(self.v + (drive - brake - resist) * dt, 0.0, self.max_ms)
        return self.v * KMH_PER_MS

    @property
    def speed_kmh(self) -> float:
        return self.v * KMH_PER_MS
