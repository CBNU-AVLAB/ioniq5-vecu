#!/usr/bin/env python3
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      03_uds_client.py
# @brief     Student stub: UDS diagnostic sequence (tester)
#
# @date      2026-09-13 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
Exercise 2 — automate a diagnostic sequence.

Sends UDS requests to the ADA-S ECU running in the container.

  [1] Identify           read the VIN and SW version
  [2] Live values        read the steering angle repeatedly
  [3] Change a setting   rejected -> raise the session -> rejected -> unlock -> accepted
  [4] Verify             while AD sweeps the steering, poll the angle and see it clipped
  [5] Faults             inject a fault, read and clear DTCs
  [6] Restore            ECU reset

Fill in 6 places (8 blanks): replace each `____` with the right value.
An unfilled `____` raises NameError on that line.

Setup
  1) Start the vECU in another terminal:
       PYTHONPATH=src .venv/bin/python -m ioniq5_vecu.vecu
  2) Watch only the diagnostic frames in another terminal:
       candump vcan0,7A0:7F8,7A8:7F8

Run
  .venv/bin/python practice/03_uds_client.py

Diagnostic addresses (tester side)
  send on 0x7A0, receive on 0x7A8 (the ECU uses the opposite pair).

API
  client.change_session(session)                  0x10
  client.read_data_by_identifier(did)             0x22
  client.write_data_by_identifier(did, value)     0x2E
  client.unlock_security_access(level)            0x27 (requests the seed and sends the key)
  client.start_routine(rid)                       0x31
  client.get_dtc_by_status_mask(mask)             0x19
  client.ecu_reset(...)                           0x11
  Rejections raise NegativeResponseException; exc.response.code holds the NRC.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import can
import isotp
import udsoncan
from udsoncan.client import Client
from udsoncan.configs import default_client_config
from udsoncan.connections import PythonIsoTpConnection
from udsoncan.exceptions import NegativeResponseException
from udsoncan.services import DiagnosticSessionControl, ECUReset

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from ioniq5_vecu.bus import CanBus  # noqa: E402

SECURITY_MASK = 0x5A5A5A5A     # key = seed XOR MASK
SWEEP_DEG = 400.0              # AD steering command amplitude (beyond the limit)
SWEEP_PERIOD_S = 2.0           # left-right cycle


# ── Provided (no changes needed) ───────────────────────────────────────────
def make_config() -> dict:
    """DID codecs and the security algorithm."""
    config = dict(default_client_config)
    config["data_identifiers"] = {
        0xF190: udsoncan.AsciiCodec(17),   # VIN
        0xF195: udsoncan.AsciiCodec(4),    # SW version
        0x0100: udsoncan.DidCodec(">h"),   # current position (0.1 deg)
        0x0110: udsoncan.DidCodec(">h"),   # limit_min (0.1 deg)
        0x0111: udsoncan.DidCodec(">h"),   # limit_max (0.1 deg)
    }
    config["security_algo"] = lambda level, seed, params: (
        int.from_bytes(seed, "big") ^ SECURITY_MASK).to_bytes(4, "big")
    return config


def read_text(client: Client, did: int) -> str:
    """Read an AsciiCodec DID as a string."""
    return client.read_data_by_identifier(did).service_data.values[did]


def read_deg(client: Client, did: int) -> float:
    """Read a 0.1-deg DID as degrees (DidCodec('>h') returns a tuple)."""
    return client.read_data_by_identifier(did).service_data.values[did][0] / 10.0


def try_write(client: Client, did: int, raw: int, label: str) -> bool:
    """Try a write. Prints the NRC and returns False when rejected."""
    try:
        client.write_data_by_identifier(did, raw)
        print(f"      {label}: accepted")
        return True
    except NegativeResponseException as exc:
        print(f"      {label}: rejected — NRC 0x{exc.response.code:02X} ({exc.response.code_name})")
        return False


def ad_sweep(stop: threading.Event) -> None:
    """Act as the AD controller: turn the servo on and sweep the position command
    on a separate CAN socket."""
    import math
    bus = CanBus(channel="vcan0")
    try:
        bus.send("ADA_S_100", {"SON": 1}, fill_defaults=True)
        t0 = time.monotonic()
        while not stop.is_set():
            elapsed = time.monotonic() - t0
            bus.send("ADA_S_101", {
                "target_pos": SWEEP_DEG * math.sin(2 * math.pi * elapsed / SWEEP_PERIOD_S)})
            stop.wait(0.05)
        bus.send("ADA_S_100", {"SON": 0}, fill_defaults=True)
    finally:
        bus.close()


# ── Fill in from here ──────────────────────────────────────────────────────
def main() -> None:
    bus = can.Bus(channel="vcan0", interface="socketcan")
    address = isotp.Address(isotp.AddressingMode.Normal_11bits, txid=0x7A0, rxid=0x7A8)
    stack = isotp.CanStack(bus=bus, address=address, params={"stmin": 0, "blocksize": 0})
    try:
        with Client(PythonIsoTpConnection(stack), config=make_config(),
                    request_timeout=2) as client:

            # [1] Identify ---------------------------------------------------
            # (1) Enter the default session.
            #     Hint: DiagnosticSessionControl.Session.defaultSession
            client.change_session(____)

            # (2) Read the VIN. At 17 bytes it does not fit in one frame, so ISO-TP splits it.
            #     Hint: find the VIN DID in the diagnostic spec
            #     -> candump shows 10.. / 30.. / 21.. / 22..
            print(f"[1] VIN={read_text(client, ____)}  SW={read_text(client, 0xF195)}")

            # [2] Live values ------------------------------------------------
            # (3) Read the current steering angle 5 times, 0.4 s apart.
            #     Hint: "current position" in the diagnostic spec
            #     -> turn the wheel with the arrow keys in console/input.py meanwhile
            print("[2] current steering angle (turn the wheel)")
            for _ in range(5):
                print(f"      {read_deg(client, ____):+7.1f} deg")
                time.sleep(0.4)

            # [3] Change a setting -------------------------------------------
            # (4) Narrow the steering limit to ±200.0 deg. limit_max is DID 0x0111
            #     in 0.1 deg units — what raw value do you write?
            new_limit_raw = ____

            print(f"[3] narrow the steering limit to ±200.0 deg "
                  f"(now {read_deg(client, 0x0110):+.1f} ~ {read_deg(client, 0x0111):+.1f} deg)")
            try_write(client, 0x0111, new_limit_raw, "default session")

            # (5) Raise to the extended session and retry: still rejected until unlocked.
            #     Hint: DiagnosticSessionControl.Session.extendedDiagnosticSession
            #           the security level is 1
            client.change_session(____)
            try_write(client, 0x0111, new_limit_raw, "extended session, locked")
            client.unlock_security_access(____)
            try_write(client, 0x0111, new_limit_raw, "extended session, unlocked")
            # Narrow the negative side too: limit_min (DID 0x0110) with the opposite sign.
            try_write(client, 0x0110, -new_limit_raw, "limit_min too")
            print(f"      limits = {read_deg(client, 0x0110):+.1f} ~ "
                  f"{read_deg(client, 0x0111):+.1f} deg")

            # [4] Verify -----------------------------------------------------
            # Poll the actual angle over UDS while AD sweeps ±400 deg.
            print(f"[4] observing over UDS while AD sweeps ±{SWEEP_DEG:.0f} deg")
            stop = threading.Event()
            sweeper = threading.Thread(target=ad_sweep, args=(stop,), daemon=True)
            sweeper.start()
            peak = 0.0
            try:
                for _ in range(10):
                    angle = read_deg(client, 0x0100)
                    peak = max(peak, abs(angle))
                    print(f"      {angle:+7.1f} deg")
                    time.sleep(0.3)
            finally:
                stop.set()
                sweeper.join(timeout=1.0)
            print(f"      peak angle {peak:.1f} deg "
                  f"— commanded ±{SWEEP_DEG:.0f} deg, clipped at the limit")

            # [5] Faults -----------------------------------------------------
            # (6) Run the fault-injection routine and read all recorded DTCs.
            #     Hint: the fault-injection RID in the diagnostic spec; mask 0xFF means "all"
            #     -> also watch ALM/RD in 0x105 and the state in 0x106 on candump
            print("[5] fault injection")
            client.start_routine(____)
            for dtc in client.get_dtc_by_status_mask(____).service_data.dtcs:
                print(f"      DTC 0x{dtc.id:06X}  status 0x{dtc.status.get_byte_as_int():02X}")
            client.clear_dtc(0xFFFFFF)
            print(f"      DTCs after clear: "
                  f"{len(client.get_dtc_by_status_mask(0xFF).service_data.dtcs)}")

            # [6] Restore ----------------------------------------------------
            client.start_routine(0x0203)                            # clear the fault
            client.ecu_reset(ECUReset.ResetType.hardReset)          # limits back to default
            print(f"[6] ECU reset — limits = {read_deg(client, 0x0110):+.1f} ~ "
                  f"{read_deg(client, 0x0111):+.1f} deg (restored)")
    finally:
        # close the socket even after Ctrl-C
        bus.shutdown()


if __name__ == "__main__":
    main()
