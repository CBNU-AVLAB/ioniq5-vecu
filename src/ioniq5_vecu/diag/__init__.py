# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      __init__.py
# @brief     UDS diagnostic layer (ISO 14229 over ISO-TP)
#
# @date      2026-09-11 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
UDS diagnostic layer (ISO 14229 over ISO-TP).

  constants.py     diagnostic CAN IDs, SIDs, NRCs, sessions, security, routine IDs, ECU identification
  did_registry.py  DID table bound to live ECU state
  dtc_store.py     DTC storage and SAE J2012 formatting
  handler.py       UDS request bytes -> response bytes (9 services)
  routines.py      routine (0x31) and ECU reset (0x11) actions
  server.py        ISO-TP server and per-ECU assembly

Import modules directly, e.g. `from ioniq5_vecu.diag.constants import SID_READ_DID`.
"""
