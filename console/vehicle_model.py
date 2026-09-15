#!/usr/bin/env python3
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      vehicle_model.py
# @brief     Display-side longitudinal speed model
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#            2026-07-17 updated by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#              : make the model gear-aware (P/N/D/R) with automatic creep

"""
Display-side speed model (host).

Speed is not in the CAN matrix, so the cluster derives it from APS_OUT_PERCENT and the
brake stroke. Nothing is sent on vcan0.

  a = drive - brake - resist
    drive  = (accel% / 100) * accel_ms2
    brake  = (stroke / stroke_max) * brake_ms2
    resist = roll + aero * v
  v += a * dt,  0 <= v <= max_speed
The coefficients are assumed values.

Gears:
  P   : speed fixed at 0
  N   : no drive; brake and resistance only
  D/R : drive, plus creep up to creep_kmh when the brake is released
"""

from __future__ import annotations

KMH_PER_MS = 3.6


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class VehicleModel:
    """Longitudinal speed integrator. m/s internally, km/h outside."""

    def __init__(
        self,
        max_kmh: float = 180.0,
        accel_ms2: float = 3.5,   # full-throttle acceleration
        brake_ms2: float = 6.0,   # full-brake deceleration
        roll: float = 0.15,       # rolling resistance, m/s²
        aero: float = 0.012,      # aerodynamic drag coefficient (proportional to v), 1/s
        creep_kmh: float = 6.0,   # creep target speed (D/R, brake released)
        creep_ms2: float = 2.5,   # creep acceleration
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
             brake_max_mm: float = 170.0, gear: str = "D") -> float:
        """Advance by dt seconds. accel_pct 0~100, brake_mm 0~max, gear P/R/N/D. Returns km/h."""
        if dt <= 0:
            return self.v * KMH_PER_MS
        if gear == "P":
            self.v = 0.0                       # park: speed 0
            return 0.0

        resist = self.roll + self.aero * self.v   # always decelerating
        brake = (_clamp(brake_mm, 0.0, brake_max_mm) / brake_max_mm) * self.brake_ms2
        if gear == "N":
            a = -brake - resist                 # neutral: no drive
        else:                                   # D / R: drive or creep, minus brake and resistance
            drive = _clamp(accel_pct, 0.0, 100.0) / 100.0 * self.accel_ms2
            creep = 0.0
            if self.v < self.creep_ms:          # creep only at low speed (tapered)
                creep = self.creep_ms2 * (1.0 - self.v / self.creep_ms)
            a = max(drive, creep) - brake - resist
        self.v = _clamp(self.v + a * dt, 0.0, self.max_ms)
        return self.v * KMH_PER_MS

    @property
    def speed_kmh(self) -> float:
        return self.v * KMH_PER_MS
