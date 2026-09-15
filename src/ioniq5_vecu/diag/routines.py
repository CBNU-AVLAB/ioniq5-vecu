# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      routines.py
# @brief     UDS routine (0x31) and ECU reset (0x11) actions bound to live ECUs
#
# @date      2026-09-11 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
Routine (0x31) and ECU reset (0x11) actions on the live ECU objects.

  RID 0x0201 zero calibration
    servo  set the position to 0 (NRC 0x22 while SON=1)
    accel  turn off the AD percent override
  RID 0x0202 fault injection
    servo  ecu.fault = True -> 0x105/0x205 ALM/RD = 0, 0x106/0x206 fsm_state_id = 6; record the DTC
    accel  record the DTC
  RID 0x0203 fault clear
    servo  ecu.fault = False (the DTC stays until 0x14)
    accel  nothing to restore
  11 01 hard reset
    servo  restore limits and max_speed from the spec, SON off, restart the controller FSM;
           the position and ecu.fault are kept
    accel  clear the override command state
"""

from __future__ import annotations

from typing import Callable

from ..ecus.base_servo import ServoFsm
from .constants import (
    NRC_CONDITIONS_NOT_CORRECT,
    RID_FAULT_CLEAR,
    RID_FAULT_INJECT,
    RID_ZERO_CALIBRATION,
)
from .dtc_store import DtcStore
from .handler import NegativeResponse


# ── ADA-S / ADA-B (single-axis servo) ───────────────────────────────────────
def build_servo_routines(ecu, dtc_store: DtcStore,
                         fault_code: int) -> dict[int, Callable[[], None]]:
    """RID -> action table for a servo ECU (BaseServoEcu).

    ecu        : object with `model` and `fault`
    fault_code : DTC recorded by fault injection (must exist in dtc_store)
    """
    m = ecu.model

    def zero_calibration() -> None:
        if m.enabled:
            raise NegativeResponse(NRC_CONDITIONS_NOT_CORRECT)
        m.set_position(0.0)          # clamped to the limits

    def fault_inject() -> None:
        ecu.fault = True
        dtc_store.set_active(fault_code)

    def fault_clear() -> None:
        ecu.fault = False

    return {
        RID_ZERO_CALIBRATION: zero_calibration,
        RID_FAULT_INJECT: fault_inject,
        RID_FAULT_CLEAR: fault_clear,
    }


def build_servo_reset(ecu) -> Callable[[], None]:
    """Hard-reset action for a servo ECU (object with `model`, `spec` and `fsm`)."""

    def reset() -> None:
        m, hw = ecu.model, ecu.spec
        m.set_enabled(False)
        m.limit_min = hw.limit_min
        m.limit_max = hw.limit_max
        m.max_speed = hw.max_speed
        m.set_position(m.position)   # target = current position, velocity 0
        ecu.fsm = ServoFsm()         # restart from the boot chain

    return reset


# ── ADE-A (APS pedal voltage emulator) ──────────────────────────────────────
def build_accel_routines(ecu, dtc_store: DtcStore,
                         fault_code: int) -> dict[int, Callable[[], None]]:
    """RID -> action table for the accelerator ECU (AccelEcu).

    ecu        : object with the AD override state (`ovr_pct_on`, `ovr_pct`, ...)
    fault_code : DTC recorded by fault injection (must exist in dtc_store)
    """

    def zero_calibration() -> None:
        ecu.ovr_pct_on = False
        ecu.ovr_pct = 0.0

    def fault_inject() -> None:
        dtc_store.set_active(fault_code)

    def fault_clear() -> None:
        pass

    return {
        RID_ZERO_CALIBRATION: zero_calibration,
        RID_FAULT_INJECT: fault_inject,
        RID_FAULT_CLEAR: fault_clear,
    }


def build_accel_reset(ecu) -> Callable[[], None]:
    """Hard-reset action for the accelerator ECU."""

    def reset() -> None:
        ecu.ovr_pct_on = False
        ecu.ovr_v_on = False
        ecu.cal_en = False
        ecu.ovr_pct = 0.0
        ecu.ovr_v1 = 0.0
        ecu.ovr_v2 = 0.0

    return reset
