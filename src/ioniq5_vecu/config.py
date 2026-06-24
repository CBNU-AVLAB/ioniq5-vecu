# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      config.py
# @brief     vECU units/limits/defaults and dbc message/signal name bindings
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""vECU units/limits/defaults + dbc message/signal name bindings.

Magic numbers and message names are not scattered across the code. Per-actuator
physical parameters (limits, speeds) and which dbc message/signal each uses are
collected here. The bit definitions themselves still live in the dbc (single source
of truth); this only points at "which signal of which message".
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ServoSpec:
    """Single-axis servo (shared by steering/brake) physical + mapping spec.

    Reused by changing only units/limits.
    """

    node: str          # dbc node/ECU name (e.g. "ADA_S")
    unit: str          # physical unit (deg / mm)
    limit_min: float   # position lower bound (physical)
    limit_max: float   # position upper bound (physical)
    max_speed: float   # max AD-tracking speed (physical/s) - slew rate
    manual_speed: float  # manual back-drive speed (physical/s) @ steer rate=+/-1

    # dbc message names
    servo_ctrl_msg: str   # RX: servo control bits (SON etc.)   0x100 / 0x200
    target_msg: str       # RX: position command (target_pos)   0x101 / 0x201
    feedback_msg: str     # TX: position feedback (encoder_pos)  0x104 / 0x204
    status_msg: str       # TX: servo status bits (SON/ALM/RD/INP/ZSP)  0x105 / 0x205
    fsm_msg: str          # TX: controller fsm_state_id                 0x106 / 0x206

    # dbc signal names (same naming across all three ECUs)
    target_sig: str = "target_pos"
    encoder_sig: str = "encoder_pos"
    abs_sig: str = "servo_abs_pos"
    enable_sig: str = "SON"

    # manual back-drive mapping (when SON=0, control OFF)
    manual_axis: str = "steer"   # which ManualInput axis (steer/brake/accel)
    manual_mode: str = "rate"    # rate: integrate val*manual_speed / absolute: val(0..1)->[limit_min,limit_max]


# -- ADA-S steering ----------------------------------------------------------
# Position unit deg, dbc resolution 1/60. Limits +/-480 deg.
# max_speed is a pre-measurement assumption (to be tuned).
STEERING = ServoSpec(
    node="ADA_S",
    unit="deg",
    limit_min=-480.0,
    limit_max=480.0,
    max_speed=720.0,     # deg/s (AD tracking)
    manual_speed=540.0,  # deg/s (hand-turning feel, at rate=+/-1)
    servo_ctrl_msg="ADA_S_100",
    target_msg="ADA_S_101",
    feedback_msg="ADA_S_104",
    status_msg="ADA_S_105",
    fsm_msg="ADA_S_106",
    manual_axis="steer",
    manual_mode="rate",
)


# -- ADA-B braking -----------------------------------------------------------
# Position unit mm (pedal stroke), dbc resolution 1/100. Limits 0~60mm.
# Same servo controller as ADA-S - base_servo.py engine reused with different units/limits.
# Manual input maps the brake pedal (absolute 0..1) directly to stroke.
BRAKE = ServoSpec(
    node="ADA_B",
    unit="mm",
    limit_min=0.0,
    limit_max=60.0,
    max_speed=240.0,     # mm/s (AD tracking)
    manual_speed=240.0,  # unused in absolute mode (placeholder)
    servo_ctrl_msg="ADA_B_200",
    target_msg="ADA_B_201",
    feedback_msg="ADA_B_204",
    status_msg="ADA_B_205",
    fsm_msg="ADA_B_206",
    manual_axis="brake",
    manual_mode="absolute",
)


# -- ADE-A acceleration (APS pedal voltage emulator) -------------------------
# Not a servo. Passes the driver pedal (IN) through to OUT; when an AD override
# arrives, OUT follows the command. APS is a redundant sensor (APS1/APS2; usually
# APS2 ~ APS1/2). %<->voltage mapping and thresholds are assumptions (to be tuned).
@dataclass(frozen=True)
class AccelSpec:
    node: str = "ADE_A"
    ctrl_msg: str = "ADE_A_311"      # RX: override command (OVR_*, CAL_EN, values)
    status_msg: str = "ADE_A_314"    # TX: %/flags (BRK_S, APS_IN/OUT_PERCENT etc.)
    voltage_msg: str = "ADE_A_315"   # TX: voltages (APS1/2_IN/OUT_VOLTAGE)

    out_pct_sig: str = "APS_OUT_PERCENT"  # final output accel % in status_msg (for console display)

    aps1_v0: float = 0.8             # APS1 0% voltage (assumed)
    aps1_v100: float = 4.0           # APS1 100% voltage (assumed)
    aps2_ratio: float = 0.5          # APS2 = APS1 * ratio (redundant half-value sensor, assumed)

    aps_driver_pct: float = 3.0      # APS_OVR_flag (driver-accel detect) threshold %
    brk_driver_pct: float = 20.0     # BRK_OVR_flag (driver-brake detect) threshold %


ACCEL = AccelSpec()
