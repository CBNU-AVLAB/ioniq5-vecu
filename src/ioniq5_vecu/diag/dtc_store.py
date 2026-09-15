# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      dtc_store.py
# @brief     DTC store with SAE J2012 code formatting
#
# @date      2026-09-11 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
DTC (Diagnostic Trouble Code) store for 0x19 (read), 0x14 (clear) and fault injection.

A DTC is 3 bytes. SAE J2012 text form:

  byte0 bits 7-6 : system  00=P 01=C 10=B 11=U
  byte0 bits 5-4 : first digit (0-3)
  byte0 bits 3-0 : second digit (0-F)
  byte1          : third and fourth digits
  byte2          : FTB (failure type byte), after the "-"

  e.g. 0x410100 -> "C0101-00"

Status byte: 0x00 = not recorded, 0x2F = confirmed fault.
A record stays after the fault is cleared, until 0x14 clears it.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable

from .constants import (
    DTC_STATUS_AVAILABILITY_MASK,
    DTC_STATUS_CONFIRMED,
    DTC_STATUS_NONE,
)

_SYSTEM_LETTERS = "PCBU"   # byte0 bits 7-6
DTC_CODE_MAX = 0xFFFFFF    # 3 bytes
DTC_RECORD_LENGTH = 4      # one DTC in a 19 02 response: code 3 bytes + status 1 byte


def format_dtc(code: int) -> str:
    """3-byte DTC -> SAE J2012 text (0x410100 -> "C0101-00"). ValueError if out of range."""
    if not 0 <= code <= DTC_CODE_MAX:
        raise ValueError(f"DTC code must fit in 3 bytes, got 0x{code:X}")
    b0, b1, ftb = code >> 16, (code >> 8) & 0xFF, code & 0xFF
    letter = _SYSTEM_LETTERS[b0 >> 6]
    return f"{letter}{(b0 >> 4) & 0x3}{b0 & 0xF:X}{b1:02X}-{ftb:02X}"


@dataclass
class Dtc:
    """One DTC. `status` is changed by DtcStore."""

    code: int                       # 3-byte DTC (e.g. 0x410100)
    description: str                # human-readable text (not sent over UDS)
    status: int = DTC_STATUS_NONE   # statusOfDTC

    def __post_init__(self) -> None:
        format_dtc(self.code)       # validate the 3-byte range

    @property
    def label(self) -> str:
        """SAE J2012 text of the code."""
        return format_dtc(self.code)

    def record(self) -> bytes:
        """4 bytes for a 19 02 response: code (3 bytes, big-endian) + status.

        e.g. C0101-00 confirmed -> 41 01 00 2F
        """
        return self.code.to_bytes(3, "big") + bytes([self.status])


class DtcStore:
    """DTCs one ECU can report, with their status. Results keep the definition order."""

    def __init__(self, definitions: Iterable[Dtc]) -> None:
        # Keep copies so status changes do not modify the shared definitions.
        self._dtcs: dict[int, Dtc] = {}
        for d in definitions:
            if d.code in self._dtcs:
                raise ValueError(f"duplicate DTC {d.label}")
            self._dtcs[d.code] = replace(d)

    def set_active(self, code: int) -> None:
        """Record `code` as a confirmed fault (0x2F). KeyError if the code is not defined."""
        self._dtcs[code].status = DTC_STATUS_CONFIRMED

    def clear_all(self) -> None:
        """Clear all records (14 FF FF FF)."""
        for d in self._dtcs.values():
            d.status = DTC_STATUS_NONE

    def matching(self, status_mask: int) -> list[Dtc]:
        """DTCs whose status & mask is non-zero (19 02 <mask>), in definition order."""
        return [d for d in self._dtcs.values() if d.status & status_mask]

    def active(self) -> list[Dtc]:
        """DTCs with any status bit set."""
        return self.matching(DTC_STATUS_AVAILABILITY_MASK)


# DTC recorded by the fault-injection routine (RID 0x0202), per ECU node.
FAULT_DTCS = {
    "ADA_S": Dtc(0x410100, "Steering Actuator Position Error"),   # C0101-00
    "ADA_B": Dtc(0x410200, "Brake Actuator Position Error"),      # C0102-00
    "ADE_A": Dtc(0x410300, "APS Signal Implausible"),             # C0103-00
}
