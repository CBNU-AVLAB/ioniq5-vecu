# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      constants.py
# @brief     UDS diagnostic constants (CAN IDs, SIDs, NRCs, sessions, routines)
#
# @date      2026-09-11 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
UDS constants: diagnostic CAN IDs, service IDs, NRCs, sessions, security access,
routine IDs and ECU identification values (ISO 14229-1).
"""

from __future__ import annotations

# ── Diagnostic addresses (ISO-TP normal 11-bit addressing) ─────────────────
# (rxid, txid) from the ECU side: rxid = tester -> ECU request, txid = ECU -> tester response.
# A tester uses the opposite pair (txid=0x7A0, rxid=0x7A8).
DIAG_ADDRESSES = {
    "ADA_S": (0x7A0, 0x7A8),
    "ADA_B": (0x7B0, 0x7B8),
    "ADE_A": (0x7C0, 0x7C8),
}
FUNCTIONAL_ADDRESS = 0x7DF     # functional addressing (request to all ECUs)

# ── Service IDs (first request byte) ───────────────────────────────────────
SID_SESSION_CONTROL   = 0x10   # DiagnosticSessionControl
SID_ECU_RESET         = 0x11   # ECUReset
SID_CLEAR_DTC         = 0x14   # ClearDiagnosticInformation
SID_READ_DTC          = 0x19   # ReadDTCInformation
SID_READ_DID          = 0x22   # ReadDataByIdentifier
SID_SECURITY_ACCESS   = 0x27   # SecurityAccess
SID_WRITE_DID         = 0x2E   # WriteDataByIdentifier
SID_ROUTINE_CONTROL   = 0x31   # RoutineControl
SID_TESTER_PRESENT    = 0x3E   # TesterPresent

POSITIVE_RESPONSE_OFFSET = 0x40   # positive response SID = request SID + 0x40 (0x22 -> 0x62)
NEGATIVE_RESPONSE_SID    = 0x7F   # negative response: 7F <request SID> <NRC>

# Top bit of a sub-function byte: suppress the positive response (e.g. 3E 80).
SUPPRESS_POS_RSP_BIT = 0x80

# ── Negative response codes (third byte of 7F <SID> <NRC>) ─────────────────
NRC_GENERAL_REJECT                    = 0x10   # internal server error
NRC_SERVICE_NOT_SUPPORTED             = 0x11   # unknown SID
NRC_SUBFUNCTION_NOT_SUPPORTED         = 0x12   # known SID, unknown sub-function
NRC_INCORRECT_LENGTH                  = 0x13   # wrong request length or format
NRC_CONDITIONS_NOT_CORRECT            = 0x22   # cannot run in the current state
NRC_REQUEST_SEQUENCE_ERROR            = 0x24   # wrong order (key sent without a seed)
NRC_REQUEST_OUT_OF_RANGE              = 0x31   # unknown DID/RID or value out of range
NRC_SECURITY_ACCESS_DENIED            = 0x33   # protected action while locked
NRC_INVALID_KEY                       = 0x35   # SecurityAccess key mismatch
NRC_SERVICE_NOT_SUPPORTED_IN_SESSION  = 0x7F   # not allowed in the current session

# ── Diagnostic sessions (0x10 sub-function) ────────────────────────────────
SESSION_DEFAULT     = 0x01   # power-on session, read only
SESSION_PROGRAMMING = 0x02   # accepted, no programming functions
SESSION_EXTENDED    = 0x03   # enables writes and security access

# Server timing in the 0x10 positive response: 50 <session> <P2 2 bytes> <P2* 2 bytes>.
# P2 is encoded in 1 ms, P2* in 10 ms: 50 ms = 00 32, 5000 ms = 01 F4.
P2_SERVER_MAX_MS      = 50     # request -> start of response
P2_STAR_SERVER_MAX_MS = 5000   # extended limit after a response-pending NRC

# S3 timer: a non-default session with no request for this long returns to Default
# and security locks again. Testers keep the session alive with 3E 80.
S3_TIMEOUT_S = 5.0

# ── Security access (0x27) ─────────────────────────────────────────────────
# Odd sub-function = requestSeed, the next even one = sendKey. Level 1 only.
SECURITY_LEVEL_1       = 0x01   # 27 01 -> 67 01 <seed 4 bytes>
SECURITY_SEND_KEY_1    = 0x02   # 27 02 <key 4 bytes> -> 67 02
SECURITY_SEED_LENGTH   = 4      # seed/key length in bytes (big-endian)
# key = seed XOR mask
SECURITY_KEY_MASK = 0x5A5A5A5A

# ── ECU reset (0x11 sub-function) ──────────────────────────────────────────
RESET_HARD = 0x01   # 11 01 -> 51 01

# ── TesterPresent (0x3E sub-function) ──────────────────────────────────────
TESTER_PRESENT_ZERO = 0x00   # 3E 00 -> 7E 00, 3E 80 -> no response

# ── DTC (0x14 / 0x19) ──────────────────────────────────────────────────────
GROUP_OF_DTC_ALL                = 0xFFFFFF   # 14 FF FF FF: clear all DTCs
REPORT_DTC_BY_STATUS_MASK       = 0x02       # 19 02 <mask>: the only 0x19 sub-function supported
DTC_STATUS_AVAILABILITY_MASK    = 0xFF       # 59 02 <this> ...: all status bits supported
DTC_STATUS_NONE = 0x00   # not recorded
# Confirmed fault: testFailed(0x01) | testFailedThisOperationCycle(0x02)
# | pendingDTC(0x04) | confirmedDTC(0x08) | testFailedSinceLastClear(0x20)
DTC_STATUS_CONFIRMED = 0x2F

# ── Routines (0x31) ────────────────────────────────────────────────────────
ROUTINE_START = 0x01   # 31 01 <RID 2 bytes>: startRoutine (the only sub-function supported)

RID_ZERO_CALIBRATION = 0x0201   # set the position to 0 (ADE-A: clear the override)
RID_FAULT_INJECT     = 0x0202   # record a DTC; servos also report ALM/RD=0 and fsm_state_id=6
RID_FAULT_CLEAR      = 0x0203   # restore normal status frames (DTCs stay until 0x14)

# ── ECU identification DIDs (0xF190 / 0xF195 / 0xF18C) ─────────────────────
# Same VIN on every ECU (one vehicle), a different serial number per ECU.
VEHICLE_VIN = b"KMHK381BLSU000001"
SW_VERSION = b"V1.0"
ECU_SERIALS = {
    "ADA_S": b"ADAS0001",
    "ADA_B": b"ADAB0001",
    "ADE_A": b"ADEA0001",
}
