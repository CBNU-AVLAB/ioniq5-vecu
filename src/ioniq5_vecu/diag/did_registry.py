# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      did_registry.py
# @brief     UDS DID table bound to live ECU state
#
# @date      2026-09-11 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
DID table bound to live ECU state (0x22 ReadDataByIdentifier / 0x2E WriteDataByIdentifier).

Values are read from the ECU objects when a request arrives. A write to 0x0111
(limit_max) takes effect on the next control cycle.

DidEntry
  read()         returns exactly `length` bytes
  write(data)    called with len(data) == length; raises ValueError for an out-of-range
                 value (answered with NRC 0x31)
  min_session    minimum session for writing
  need_security  writing requires SecurityAccess
Reads are allowed in every session.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Callable, Optional

from .constants import SESSION_DEFAULT, SESSION_EXTENDED, SW_VERSION

# Positions and limits: signed 16-bit big-endian in 0.1 units (>h).
# Steering ±480.0 deg -> ±4800, brake 0~170.0 mm -> 0~1700.
_TENTHS = 10.0
_INT16_MIN, _INT16_MAX = -0x8000, 0x7FFF
_UINT16_MAX = 0xFFFF

VIN_LENGTH = 17
SW_VERSION_LENGTH = 4
SERIAL_LENGTH = 8


@dataclass(frozen=True)
class DidEntry:
    """One DID definition (see the module docstring)."""

    did: int
    name: str
    length: int                                     # data bytes in the response (without the DID)
    read: Callable[[], bytes]
    write: Optional[Callable[[bytes], None]] = None  # None: read only
    min_session: int = SESSION_DEFAULT
    need_security: bool = False


# ── Encoding helpers ────────────────────────────────────────────────────────
def _clamp(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v


def _pack_tenths(value: float) -> bytes:
    """physical -> 0.1-unit >h (rounded, clamped to the field)."""
    return struct.pack(">h", _clamp(round(value * _TENTHS), _INT16_MIN, _INT16_MAX))


def _unpack_tenths(data: bytes) -> float:
    return struct.unpack(">h", data)[0] / _TENTHS


def _pack_u16(value: float) -> bytes:
    return struct.pack(">H", _clamp(round(value), 0, _UINT16_MAX))


def _pack_flag(value: bool) -> bytes:
    return b"\x01" if value else b"\x00"


def _require_within(name: str, value: float, lo: float, hi: float) -> None:
    if not lo <= value <= hi:
        raise ValueError(f"{name}={value:g} is outside [{lo:g}, {hi:g}]")


def _require_length(name: str, value: bytes, length: int) -> None:
    if len(value) != length:
        raise ValueError(f"{name} must be {length} bytes, got {len(value)}")


# ── Identification DIDs (all ECUs) ──────────────────────────────────────────
def _identification(vin: bytes, serial: bytes, sw_version: bytes) -> dict[int, DidEntry]:
    """ISO 14229-1 identification DIDs, shared by all ECUs (read only)."""
    _require_length("vin", vin, VIN_LENGTH)
    _require_length("serial", serial, SERIAL_LENGTH)
    _require_length("sw_version", sw_version, SW_VERSION_LENGTH)
    return {
        # vehicleIdentificationNumber (17 bytes: sent as an ISO-TP multi-frame)
        0xF190: DidEntry(0xF190, "VIN", VIN_LENGTH, lambda: vin),
        # systemSupplierECUSoftwareVersionNumber
        0xF195: DidEntry(0xF195, "SW_VERSION", SW_VERSION_LENGTH, lambda: sw_version),
        # ECUSerialNumber
        0xF18C: DidEntry(0xF18C, "SERIAL", SERIAL_LENGTH, lambda: serial),
    }


# ── ADA-S / ADA-B (single-axis servo) ───────────────────────────────────────
def build_servo_registry(ecu, vin: bytes, serial: bytes,
                         sw_version: bytes = SW_VERSION) -> dict[int, DidEntry]:
    """DID table for a servo ECU (BaseServoEcu).

    ecu        : object with `model` (ServoModel), `fsm` (ServoFsm) and `spec` (ServoSpec)
    vin/serial : 17 / 8 ASCII bytes (ValueError otherwise)

    Units follow ecu.spec.unit (deg or mm). Limits can be written only within the
    spec limits and must keep limit_min <= limit_max; max_speed must be > 0.
    """
    ids = _identification(vin, serial, sw_version)
    m = ecu.model
    hw = ecu.spec

    def write_limit_min(data: bytes) -> None:
        value = _unpack_tenths(data)
        _require_within("limit_min", value, hw.limit_min, m.limit_max)
        m.limit_min = value

    def write_limit_max(data: bytes) -> None:
        value = _unpack_tenths(data)
        _require_within("limit_max", value, m.limit_min, hw.limit_max)
        m.limit_max = value

    def write_max_speed(data: bytes) -> None:
        value = float(struct.unpack(">H", data)[0])
        _require_within("max_speed", value, 1.0, _UINT16_MAX)
        m.max_speed = value

    live = {
        # current position = encoder_pos in 0x104/0x204 (0.1 units)
        0x0100: DidEntry(0x0100, "CURRENT_POSITION", 2,
                         lambda: _pack_tenths(m.position)),
        # controller state = fsm_state_id in 0x106/0x206
        # (0 start, 1 wait driver ready, 2 read abs pos, 3 free wheeling,
        #  4 wait RD on, 5 position control, 6 warning)
        0x0101: DidEntry(0x0101, "FSM_STATE", 1,
                         lambda: struct.pack(">B", ecu.fsm.state)),
        # servo on = SON from 0x100/0x200 (0 manual, 1 AD)
        0x0102: DidEntry(0x0102, "SERVO_ON", 1,
                         lambda: _pack_flag(m.enabled)),
        # soft limits (0.1 units)
        0x0110: DidEntry(0x0110, "LIMIT_MIN", 2,
                         read=lambda: _pack_tenths(m.limit_min),
                         write=write_limit_min,
                         min_session=SESSION_EXTENDED, need_security=True),
        0x0111: DidEntry(0x0111, "LIMIT_MAX", 2,
                         read=lambda: _pack_tenths(m.limit_max),
                         write=write_limit_max,
                         min_session=SESSION_EXTENDED, need_security=True),
        # AD tracking speed limit (1 unit/s)
        0x0112: DidEntry(0x0112, "MAX_SPEED", 2,
                         read=lambda: _pack_u16(m.max_speed),
                         write=write_max_speed,
                         min_session=SESSION_EXTENDED, need_security=True),
    }
    ids.update(live)
    return ids


# ── ADE-A (APS pedal voltage emulator) ──────────────────────────────────────
def build_accel_registry(ecu, vin: bytes, serial: bytes,
                         sw_version: bytes = SW_VERSION) -> dict[int, DidEntry]:
    """DID table for the accelerator ECU (AccelEcu). All DIDs are read only.

    ecu        : object with the AD override state (`ovr_pct_on`, `ovr_pct`, `cal_en`)
    vin/serial : 17 / 8 ASCII bytes (ValueError otherwise)
    """
    ids = _identification(vin, serial, sw_version)
    live = {
        # 0x311 OVR__PERCENT
        0x0120: DidEntry(0x0120, "APS_OVERRIDE_ACTIVE", 1,
                         lambda: _pack_flag(ecu.ovr_pct_on)),
        # 0x311 APS_OVR_PERCENT_VALUE (0.1 % units, >H)
        0x0121: DidEntry(0x0121, "APS_OVERRIDE_PERCENT", 2,
                         lambda: _pack_u16(ecu.ovr_pct * _TENTHS)),
        # 0x311 CAL_EN
        0x0122: DidEntry(0x0122, "CALIBRATION_ENABLE", 1,
                         lambda: _pack_flag(ecu.cal_en)),
    }
    ids.update(live)
    return ids
