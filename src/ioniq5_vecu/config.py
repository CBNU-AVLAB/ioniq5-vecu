# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      config.py
# @brief     vECU units/limits/defaults and dbc message/signal name bindings
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
vECU units, limits and defaults, and the dbc messages/signals each ECU uses.

Bit-level definitions stay in the dbc; this module only names the messages and signals.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ServoSpec:
    """Single-axis servo spec shared by the steering and brake ECUs."""

    node: str          # dbc node name (e.g. "ADA_S")
    unit: str          # physical unit (deg / mm)
    limit_min: float   # position lower limit
    limit_max: float   # position upper limit
    max_speed: float   # AD tracking speed limit (unit/s)
    manual_speed: float  # manual back-drive speed (unit/s) at steer rate = ±1

    # dbc message names
    servo_ctrl_msg: str   # RX: servo control bits (SON, ...)       0x100 / 0x200
    target_msg: str       # RX: position command (target_pos)       0x101 / 0x201
    feedback_msg: str     # TX: position feedback (encoder_pos)     0x104 / 0x204
    status_msg: str       # TX: servo status bits (SON/ALM/RD/INP/ZSP)  0x105 / 0x205
    fsm_msg: str          # TX: controller fsm_state_id                 0x106 / 0x206

    # dbc signal names (same in both servo ECUs)
    target_sig: str = "target_pos"
    encoder_sig: str = "encoder_pos"
    abs_sig: str = "servo_abs_pos"
    enable_sig: str = "SON"

    # Manual back-drive mapping (used while SON=0)
    manual_axis: str = "steer"   # ManualInput axis (steer/brake/accel)
    manual_mode: str = "rate"    # rate: integrate val*manual_speed / absolute: val(0..1) -> [limit_min, limit_max]


# ── ADA-S steering ──────────────────────────────────────────────────────────
# Position in deg (dbc resolution 1/60), limits ±480 deg. max_speed is an assumed value.
STEERING = ServoSpec(
    node="ADA_S",
    unit="deg",
    limit_min=-480.0,
    limit_max=480.0,
    max_speed=720.0,     # deg/s (AD tracking)
    manual_speed=540.0,  # deg/s (manual, at rate = ±1)
    servo_ctrl_msg="ADA_S_100",
    target_msg="ADA_S_101",
    feedback_msg="ADA_S_104",
    status_msg="ADA_S_105",
    fsm_msg="ADA_S_106",
    manual_axis="steer",
    manual_mode="rate",
)


# ── ADA-B brake ─────────────────────────────────────────────────────────────
# Position in mm of pedal stroke (dbc resolution 1/100), limits 0~170 mm.
# Manual input maps the brake pedal (0..1) directly to the stroke. max_speed is an assumed value.
BRAKE = ServoSpec(
    node="ADA_B",
    unit="mm",
    limit_min=0.0,
    limit_max=170.0,
    max_speed=240.0,     # mm/s (AD tracking)
    manual_speed=240.0,  # unused in absolute mode
    servo_ctrl_msg="ADA_B_200",
    target_msg="ADA_B_201",
    feedback_msg="ADA_B_204",
    status_msg="ADA_B_205",
    fsm_msg="ADA_B_206",
    manual_axis="brake",
    manual_mode="absolute",
)


# ── ADE-A accelerator (APS pedal voltage emulator) ──────────────────────────
# Passes the driver pedal (IN) to OUT; an AD override on 0x311 replaces OUT.
# APS1/APS2 are redundant sensors. The %<->voltage mapping and thresholds are assumed values.
@dataclass(frozen=True)
class AccelSpec:
    node: str = "ADE_A"
    ctrl_msg: str = "ADE_A_311"      # RX: override command (OVR_*, CAL_EN, values)
    status_msg: str = "ADE_A_314"    # TX: percent and flags (BRK_S, APS_IN/OUT_PERCENT, ...)
    voltage_msg: str = "ADE_A_315"   # TX: voltages (APS1/2_IN/OUT_VOLTAGE)

    out_pct_sig: str = "APS_OUT_PERCENT"  # final accel output % (shown on the cluster)

    aps1_v0: float = 0.8             # APS1 voltage at 0%
    aps1_v100: float = 4.0           # APS1 voltage at 100%
    aps2_ratio: float = 0.5          # APS2 = APS1 * ratio

    aps_driver_pct: float = 3.0      # APS_OVR_flag threshold (driver accelerating), %
    brk_driver_pct: float = 20.0     # BRK_OVR_flag threshold (driver braking), %


ACCEL = AccelSpec()
