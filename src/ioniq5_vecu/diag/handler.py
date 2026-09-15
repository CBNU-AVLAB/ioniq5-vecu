# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      handler.py
# @brief     UDS request -> response byte logic (CAN / ISO-TP independent)
#
# @date      2026-09-11 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
UDS request bytes -> response bytes (independent of CAN and ISO-TP).

Responses
  positive  <SID + 0x40> <data>   22 F1 95       -> 62 F1 95 56 31 2E 30 ("V1.0")
  negative  7F <SID> <NRC>        2E 01 11 07 D0 -> 7F 2E 7F (write in the Default session)
  none      None for a positive response with the 0x80 suppress bit set

Services (other SIDs -> 7F <SID> 11)
  request            positive response            sessions
  10 <01|02|03>      50 <s> 00 32 01 F4           all
  11 01              51 01                        all          hard reset (DTCs are kept)
  14 FF FF FF        54                           all
  19 02 <mask>       59 02 FF (<DTC 3><status>)*  all
  22 <DID 2>...      62 (<DID 2><data>)*          all          several DIDs per request
  27 01              67 01 <seed 4>               non-default  seed 00 00 00 00 when unlocked
  27 02 <key 4>      67 02                        non-default  key = seed XOR 0x5A5A5A5A
  2E <DID 2> <data>  6E <DID 2>                   non-default  plus the DID's session/security rules
  31 01 <RID 2>      71 01 <RID 2>                non-default
  3E 00              7E 00                        all          3E 80 -> no response

NRC check order (the first failing check is returned)
  1. minimum length 0x13        2. sub-function 0x12       3. full length 0x13
  4. session 0x7F               5. target exists 0x31      6. target session/security 0x7F/0x33
  7. data length 0x13           8. execution 0x24/0x35/0x31/0x22

A non-default session returns to Default after S3_TIMEOUT_S (5 s) without requests,
and security locks again. A seed is valid for one key attempt.
"""

from __future__ import annotations

import random
import struct
import time
from typing import Callable, Optional

from .constants import (
    DTC_STATUS_AVAILABILITY_MASK,
    GROUP_OF_DTC_ALL,
    NEGATIVE_RESPONSE_SID,
    NRC_INCORRECT_LENGTH,
    NRC_INVALID_KEY,
    NRC_REQUEST_OUT_OF_RANGE,
    NRC_REQUEST_SEQUENCE_ERROR,
    NRC_SECURITY_ACCESS_DENIED,
    NRC_SERVICE_NOT_SUPPORTED,
    NRC_SERVICE_NOT_SUPPORTED_IN_SESSION,
    NRC_SUBFUNCTION_NOT_SUPPORTED,
    P2_SERVER_MAX_MS,
    P2_STAR_SERVER_MAX_MS,
    POSITIVE_RESPONSE_OFFSET,
    REPORT_DTC_BY_STATUS_MASK,
    RESET_HARD,
    ROUTINE_START,
    S3_TIMEOUT_S,
    SECURITY_KEY_MASK,
    SECURITY_LEVEL_1,
    SECURITY_SEED_LENGTH,
    SECURITY_SEND_KEY_1,
    SESSION_DEFAULT,
    SESSION_EXTENDED,
    SESSION_PROGRAMMING,
    SID_CLEAR_DTC,
    SID_ECU_RESET,
    SID_READ_DID,
    SID_READ_DTC,
    SID_ROUTINE_CONTROL,
    SID_SECURITY_ACCESS,
    SID_SESSION_CONTROL,
    SID_TESTER_PRESENT,
    SID_WRITE_DID,
    SUPPRESS_POS_RSP_BIT,
    TESTER_PRESENT_ZERO,
)
from .did_registry import DidEntry
from .dtc_store import DtcStore

_SUPPORTED_SESSIONS = (SESSION_DEFAULT, SESSION_PROGRAMMING, SESSION_EXTENDED)
_SEED_MAX = (1 << (8 * SECURITY_SEED_LENGTH)) - 1


class NegativeResponse(Exception):
    """Raise to answer 7F <SID> <nrc>. Used by services and injected callbacks."""

    def __init__(self, nrc: int) -> None:
        super().__init__(f"NRC 0x{nrc:02X}")
        self.nrc = nrc


def _split_subfunction(value: int) -> tuple[int, bool]:
    """Sub-function byte -> (value, suppress positive response). 0x80 -> (0x00, True)."""
    return value & ~SUPPRESS_POS_RSP_BIT & 0xFF, bool(value & SUPPRESS_POS_RSP_BIT)


def _u16(data: bytes, offset: int) -> int:
    """Read a 2-byte big-endian ID (DID, RID)."""
    return (data[offset] << 8) | data[offset + 1]


def _random_seed() -> int:
    # 0 means "already unlocked", so it is never generated
    return random.randint(1, _SEED_MAX)


class UdsHandler:
    """UDS server logic for one ECU: keeps session/security state and answers requests.

    node         ECU name for logs (e.g. "ADA_S")
    registry     DID -> DidEntry
    dtc_store    this ECU's DTC store
    routines     RID -> callable; may raise NegativeResponse(0x22)
    reset        callable for 11 01
    clock        monotonic clock in seconds
    seed_source  seed generator returning 1..0xFFFFFFFF
    """

    # SID -> service method
    _SERVICES = {
        SID_SESSION_CONTROL: "_svc_session_control",
        SID_ECU_RESET:       "_svc_ecu_reset",
        SID_CLEAR_DTC:       "_svc_clear_dtc",
        SID_READ_DTC:        "_svc_read_dtc",
        SID_READ_DID:        "_svc_read_did",
        SID_SECURITY_ACCESS: "_svc_security_access",
        SID_WRITE_DID:       "_svc_write_did",
        SID_ROUTINE_CONTROL: "_svc_routine_control",
        SID_TESTER_PRESENT:  "_svc_tester_present",
    }

    def __init__(
        self,
        node: str,
        registry: dict[int, DidEntry],
        dtc_store: DtcStore,
        routines: dict[int, Callable[[], None]],
        reset: Callable[[], None],
        *,
        clock: Callable[[], float] = time.monotonic,
        seed_source: Callable[[], int] = _random_seed,
    ) -> None:
        self.node = node
        self.registry = registry
        self.dtc = dtc_store
        self.routines = routines
        self._reset_ecu = reset
        self._clock = clock
        self._seed_source = seed_source
        self.session = SESSION_DEFAULT
        self.security_unlocked = False
        self._pending_seed: Optional[int] = None
        self._last_activity = clock()

    # ── Entry point ───────────────────────────────────────────────────────
    def handle(self, req: bytes) -> Optional[bytes]:
        """Handle one request. Returns the response bytes, or None for an empty
        request or a suppressed positive response."""
        if not req:
            return None
        self._expire_session_if_idle()
        sid = req[0]
        try:
            method = self._SERVICES.get(sid)
            if method is None:
                raise NegativeResponse(NRC_SERVICE_NOT_SUPPORTED)
            return getattr(self, method)(req)
        except NegativeResponse as neg:
            return bytes([NEGATIVE_RESPONSE_SID, sid, neg.nrc])
        finally:
            self._last_activity = self._clock()

    # ── Session / security state ──────────────────────────────────────────
    def _expire_session_if_idle(self) -> None:
        if (self.session != SESSION_DEFAULT
                and self._clock() - self._last_activity > S3_TIMEOUT_S):
            self.session = SESSION_DEFAULT
            self._lock_security()

    def _lock_security(self) -> None:
        self.security_unlocked = False
        self._pending_seed = None

    @staticmethod
    def _positive(sid: int, payload: bytes = b"",
                  suppress: bool = False) -> Optional[bytes]:
        if suppress:
            return None
        return bytes([sid + POSITIVE_RESPONSE_OFFSET]) + payload

    # ── 0x10 DiagnosticSessionControl ─────────────────────────────────────
    def _svc_session_control(self, req: bytes) -> Optional[bytes]:
        if len(req) < 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        session, suppress = _split_subfunction(req[1])
        if session not in _SUPPORTED_SESSIONS:
            raise NegativeResponse(NRC_SUBFUNCTION_NOT_SUPPORTED)
        if len(req) != 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        # Security locks on a session change or on Default; the same session keeps it.
        if session == SESSION_DEFAULT or session != self.session:
            self._lock_security()
        self.session = session
        # P2 in 1 ms, P2* in 10 ms
        timing = struct.pack(">HH", P2_SERVER_MAX_MS, P2_STAR_SERVER_MAX_MS // 10)
        return self._positive(req[0], bytes([session]) + timing, suppress)

    # ── 0x11 ECUReset ─────────────────────────────────────────────────────
    def _svc_ecu_reset(self, req: bytes) -> Optional[bytes]:
        if len(req) < 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        kind, suppress = _split_subfunction(req[1])
        if kind != RESET_HARD:
            raise NegativeResponse(NRC_SUBFUNCTION_NOT_SUPPORTED)
        if len(req) != 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        # DTCs are kept across a reset
        self._reset_ecu()
        self.session = SESSION_DEFAULT
        self._lock_security()
        return self._positive(req[0], bytes([kind]), suppress)

    # ── 0x14 ClearDiagnosticInformation ───────────────────────────────────
    def _svc_clear_dtc(self, req: bytes) -> Optional[bytes]:
        if len(req) != 4:                                   # 14 <groupOfDTC 3 bytes>
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        if int.from_bytes(req[1:4], "big") != GROUP_OF_DTC_ALL:
            raise NegativeResponse(NRC_REQUEST_OUT_OF_RANGE)   # only "all DTCs" is supported
        self.dtc.clear_all()
        return self._positive(req[0])

    # ── 0x19 ReadDTCInformation ───────────────────────────────────────────
    def _svc_read_dtc(self, req: bytes) -> Optional[bytes]:
        if len(req) < 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        if req[1] != REPORT_DTC_BY_STATUS_MASK:
            # only sub-function 0x02 (e.g. 19 0A -> 7F 19 12)
            raise NegativeResponse(NRC_SUBFUNCTION_NOT_SUPPORTED)
        if len(req) != 3:                                   # 19 02 <DTCStatusMask>
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        records = b"".join(d.record() for d in self.dtc.matching(req[2]))
        payload = bytes([REPORT_DTC_BY_STATUS_MASK, DTC_STATUS_AVAILABILITY_MASK]) + records
        return self._positive(req[0], payload)

    # ── 0x22 ReadDataByIdentifier ─────────────────────────────────────────
    def _svc_read_did(self, req: bytes) -> Optional[bytes]:
        if len(req) < 3 or (len(req) - 1) % 2:              # 22 (<DID 2 bytes>)+
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        payload = bytearray()
        for i in range(1, len(req), 2):
            entry = self.registry.get(_u16(req, i))
            if entry is None:
                # one unknown DID rejects the whole request
                raise NegativeResponse(NRC_REQUEST_OUT_OF_RANGE)
            payload += req[i:i + 2] + entry.read()
        return self._positive(req[0], bytes(payload))

    # ── 0x27 SecurityAccess ───────────────────────────────────────────────
    def _svc_security_access(self, req: bytes) -> Optional[bytes]:
        if len(req) < 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        level = req[1]
        if level not in (SECURITY_LEVEL_1, SECURITY_SEND_KEY_1):
            raise NegativeResponse(NRC_SUBFUNCTION_NOT_SUPPORTED)
        expected_len = 2 if level == SECURITY_LEVEL_1 else 2 + SECURITY_SEED_LENGTH
        if len(req) != expected_len:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        if self.session == SESSION_DEFAULT:
            raise NegativeResponse(NRC_SERVICE_NOT_SUPPORTED_IN_SESSION)

        if level == SECURITY_LEVEL_1:                       # requestSeed
            if self.security_unlocked:
                seed = 0                                    # already unlocked
            else:
                seed = self._seed_source()
                self._pending_seed = seed
            return self._positive(req[0], bytes([level])
                                  + seed.to_bytes(SECURITY_SEED_LENGTH, "big"))

        # sendKey
        if self._pending_seed is None:
            raise NegativeResponse(NRC_REQUEST_SEQUENCE_ERROR)   # key without a seed
        expected = self._pending_seed ^ SECURITY_KEY_MASK
        self._pending_seed = None                           # a seed is used once
        if int.from_bytes(req[2:], "big") != expected:
            raise NegativeResponse(NRC_INVALID_KEY)
        self.security_unlocked = True
        return self._positive(req[0], bytes([level]))

    # ── 0x2E WriteDataByIdentifier ────────────────────────────────────────
    def _svc_write_did(self, req: bytes) -> Optional[bytes]:
        if len(req) < 4:                                    # 2E <DID 2 bytes> <data>
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        if self.session == SESSION_DEFAULT:
            raise NegativeResponse(NRC_SERVICE_NOT_SUPPORTED_IN_SESSION)
        entry = self.registry.get(_u16(req, 1))
        if entry is None or entry.write is None:
            raise NegativeResponse(NRC_REQUEST_OUT_OF_RANGE)     # unknown or read-only DID
        # session order: 01 Default < 02 Programming < 03 Extended
        if self.session < entry.min_session:
            raise NegativeResponse(NRC_SERVICE_NOT_SUPPORTED_IN_SESSION)
        if entry.need_security and not self.security_unlocked:
            raise NegativeResponse(NRC_SECURITY_ACCESS_DENIED)
        data = req[3:]
        if len(data) != entry.length:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        try:
            entry.write(data)
        except ValueError:
            raise NegativeResponse(NRC_REQUEST_OUT_OF_RANGE) from None   # value out of range
        return self._positive(req[0], req[1:3])

    # ── 0x31 RoutineControl ───────────────────────────────────────────────
    def _svc_routine_control(self, req: bytes) -> Optional[bytes]:
        if len(req) < 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        control, suppress = _split_subfunction(req[1])
        if control != ROUTINE_START:
            # only startRoutine (0x01)
            raise NegativeResponse(NRC_SUBFUNCTION_NOT_SUPPORTED)
        if len(req) != 4:                                   # 31 01 <RID 2 bytes>
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        if self.session == SESSION_DEFAULT:
            raise NegativeResponse(NRC_SERVICE_NOT_SUPPORTED_IN_SESSION)
        routine = self.routines.get(_u16(req, 2))
        if routine is None:
            raise NegativeResponse(NRC_REQUEST_OUT_OF_RANGE)
        routine()
        return self._positive(req[0], bytes([control]) + req[2:4], suppress)

    # ── 0x3E TesterPresent ────────────────────────────────────────────────
    def _svc_tester_present(self, req: bytes) -> Optional[bytes]:
        if len(req) < 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        zero, suppress = _split_subfunction(req[1])
        if zero != TESTER_PRESENT_ZERO:
            raise NegativeResponse(NRC_SUBFUNCTION_NOT_SUPPORTED)
        if len(req) != 2:
            raise NegativeResponse(NRC_INCORRECT_LENGTH)
        # the S3 timer is refreshed in handle()
        return self._positive(req[0], bytes([zero]), suppress)
