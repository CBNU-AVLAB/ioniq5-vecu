#!/usr/bin/env python3
# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      vehicle_model.py
# @brief     Display-side longitudinal speed model
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#            2026-07-17 updated by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#              : make the model gear-aware (P/N/D/R) with automatic creep
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

Per-gear behavior (display only -- gear arrives only via the console-internal gear_link):
  P : hold speed at 0 (parked).
  N : ignore accel (power cut), decelerate only by braking + friction (neutral coasting).
  D/R : the base physics above + idle creep -- releasing the brake gently pushes up to
        creep_kmh (~5-6) (real automatic creep). Strong braking overcomes creep and stops.
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
        creep_kmh: float = 6.0,   # automatic creep target (D/R, brake released); ~5-6 after friction offset
        creep_ms2: float = 2.5,   # creep acceleration (pushes toward creep speed at low speed)
    ) -> None:
        self.max_ms = max_kmh / KMH_PER_MS
        self.accel_ms2 = accel_ms2
        self.brake_ms2 = brake_ms2
        self.roll = roll
        self.aero = aero
        self.creep_ms = creep_kmh / KMH_PER_MS
        self.creep_ms2 = creep_ms2
        self.v = 0.0  # m/s

    def reset(self) -> None:
        self.v = 0.0

    def step(self, dt: float, accel_pct: float, brake_mm: float,
             brake_max_mm: float = 60.0, gear: str = "D") -> float:
        """Advance by dt seconds. accel_pct (0-100), brake_mm (0-max), gear (P/R/N/D). Returns km/h."""
        if dt <= 0:
            return self.v * KMH_PER_MS
        if gear == "P":
            self.v = 0.0                        # parked: hold speed at 0
            return 0.0

        resist = self.roll + self.aero * self.v  # always decelerating
        brake = (_clamp(brake_mm, 0.0, brake_max_mm) / brake_max_mm) * self.brake_ms2
        if gear == "N":
            a = -brake - resist                 # neutral: ignore accel, decelerate by braking + friction
        else:                                   # D / R: accel + creep - brake - friction
            drive = _clamp(accel_pct, 0.0, 100.0) / 100.0 * self.accel_ms2
            creep = 0.0
            if self.v < self.creep_ms:          # only gently push at low speed (tapered)
                creep = self.creep_ms2 * (1.0 - self.v / self.creep_ms)
            a = max(drive, creep) - brake - resist
        self.v = _clamp(self.v + a * dt, 0.0, self.max_ms)
        return self.v * KMH_PER_MS

    @property
    def speed_kmh(self) -> float:
        return self.v * KMH_PER_MS
