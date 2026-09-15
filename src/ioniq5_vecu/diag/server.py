# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      server.py
# @brief     ISO-TP diagnostic server and per-ECU assembly
#
# @date      2026-09-12 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
ISO-TP diagnostic server: connects UdsHandler to the CAN bus.

  vcan0 ─ can.Bus (own socket) ─ isotp.CanStack (segmentation) ─ UdsHandler (UDS bytes)

  single frame   02 22 F1 95 00 00 00 00      up to 7 bytes of data
  multi frame    10 14 62 F1 90 ...           First Frame (length 0x014 = 20 bytes)
                 30 00 00 ...                 Flow Control
                 21 ... / 22 ...              Consecutive Frames

Each ECU gets its own socket on the same interface:

    vcan0 ─┬─ vECU dispatcher (0x1xx/0x2xx/0x3xx)
           ├─ ADA-S diagnostics (0x7A0 rx / 0x7A8 tx)
           ├─ ADA-B diagnostics (0x7B0 / 0x7B8)
           └─ ADE-A diagnostics (0x7C0 / 0x7C8)

The servers send nothing until a request arrives.
"""

from __future__ import annotations

import threading
import traceback
from typing import Optional

import can
import isotp

from ..bus import DEFAULT_CHANNEL, DEFAULT_INTERFACE
from ..ecus.base_servo import BaseServoEcu
from .constants import (
    DIAG_ADDRESSES,
    ECU_SERIALS,
    NEGATIVE_RESPONSE_SID,
    NRC_GENERAL_REJECT,
    SW_VERSION,
    VEHICLE_VIN,
)
from .did_registry import build_accel_registry, build_servo_registry
from .dtc_store import FAULT_DTCS, DtcStore
from .handler import UdsHandler
from .routines import (
    build_accel_reset,
    build_accel_routines,
    build_servo_reset,
    build_servo_routines,
)

# ISO-TP parameters
ISOTP_PARAMS = {
    "stmin": 0,                            # minimum gap between consecutive frames (ms)
    "blocksize": 0,                        # 0 = no intermediate Flow Control
    "tx_padding": 0x00,                    # pad frames to 8 bytes with 0x00
    "rx_flowcontrol_timeout": 1000,        # ms
    "rx_consecutive_frame_timeout": 1000,  # ms
}

_RECV_TIMEOUT_S = 0.2      # wake-up interval to check for stop
_ERROR_BACKOFF_S = 0.1     # delay after an unexpected error


class DiagServer:
    """ISO-TP diagnostic server for one ECU, on its own CAN socket.

    handler            the ECU's UdsHandler
    rxid / txid        ECU-side addresses: requests on rxid, responses on txid
    channel/interface  python-can bus settings (tests use interface="virtual")
    bus                optional open bus; an injected bus is not closed by stop()

    stop() closes the socket; create a new server to restart.
    """

    def __init__(self, handler: UdsHandler, rxid: int, txid: int,
                 channel: str = DEFAULT_CHANNEL, interface: str = DEFAULT_INTERFACE,
                 bus: Optional[can.BusABC] = None) -> None:
        self.handler = handler
        self.rxid = rxid
        self.txid = txid
        self._owns_bus = bus is None
        self.bus = bus if bus is not None else can.Bus(channel=channel,
                                                       interface=interface)
        address = isotp.Address(isotp.AddressingMode.Normal_11bits,
                                txid=txid, rxid=rxid)
        self.stack = isotp.CanStack(bus=self.bus, address=address,
                                    params=dict(ISOTP_PARAMS))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ── Request handling ──────────────────────────────────────────────────
    def _respond(self, req: bytes) -> Optional[bytes]:
        try:
            return self.handler.handle(req)
        except Exception:                  # noqa: BLE001 — handler bug
            # log it and answer generalReject
            traceback.print_exc()
            return bytes([NEGATIVE_RESPONSE_SID, req[0] if req else 0,
                          NRC_GENERAL_REJECT])

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                req = self.stack.recv(block=True, timeout=_RECV_TIMEOUT_S)
                if req is None:
                    continue
                rsp = self._respond(bytes(req))
                if rsp is not None:
                    self.stack.send(rsp)
            except Exception:              # noqa: BLE001 — bus / ISO-TP error
                traceback.print_exc()
                self._stop.wait(_ERROR_BACKOFF_S)

    # ── Lifecycle ─────────────────────────────────────────────────────────
    def start(self) -> "DiagServer":
        if self._thread is not None:
            return self
        self._stop.clear()
        self.stack.start()                 # ISO-TP worker thread
        self._thread = threading.Thread(
            target=self._loop, name=f"diag-{self.handler.node}", daemon=True)
        self._thread.start()
        return self

    def stop(self, join: bool = True) -> None:
        self._stop.set()
        if join and self._thread is not None:
            self._thread.join(timeout=_RECV_TIMEOUT_S * 5 + 0.5)
        self._thread = None
        self.stack.stop()
        if self._owns_bus:
            self.bus.shutdown()

    def __enter__(self) -> "DiagServer":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


def build_handler(ecu, vin: bytes = VEHICLE_VIN,
                  sw_version: bytes = SW_VERSION) -> UdsHandler:
    """Assemble the UdsHandler for one ECU (servo or accelerator).

    The serial number and DTC are selected by ecu.spec.node (KeyError for an unknown node).
    """
    node = ecu.spec.node
    dtc_store = DtcStore([FAULT_DTCS[node]])
    fault_code = FAULT_DTCS[node].code
    serial = ECU_SERIALS[node]
    if isinstance(ecu, BaseServoEcu):
        registry = build_servo_registry(ecu, vin, serial, sw_version)
        routines = build_servo_routines(ecu, dtc_store, fault_code)
        reset = build_servo_reset(ecu)
    else:
        registry = build_accel_registry(ecu, vin, serial, sw_version)
        routines = build_accel_routines(ecu, dtc_store, fault_code)
        reset = build_accel_reset(ecu)
    return UdsHandler(node, registry, dtc_store, routines, reset)


class DiagManager:
    """Creates and starts a DiagServer for each ECU in VEcu.ecus.

    ecus              BaseServoEcu / AccelEcu objects; addresses come from spec.node
    channel/interface bus for the diagnostic sockets

    ECUs without an entry in DIAG_ADDRESSES are skipped with a warning.
    """

    def __init__(self, ecus, channel: str = DEFAULT_CHANNEL,
                 interface: str = DEFAULT_INTERFACE) -> None:
        self.servers: list[DiagServer] = []
        for ecu in ecus:
            node = ecu.spec.node
            address = DIAG_ADDRESSES.get(node)
            if address is None:
                print(f"[diag] {node}: no diagnostic address, skipped")
                continue
            rxid, txid = address
            self.servers.append(
                DiagServer(build_handler(ecu), rxid, txid,
                           channel=channel, interface=interface))

    def describe(self) -> str:
        """One-line summary for the startup log, e.g. "ADA-S 0x7A0/0x7A8, ADA-B 0x7B0/0x7B8"."""
        return ", ".join(
            f"{s.handler.node.replace('_', '-')} 0x{s.rxid:03X}/0x{s.txid:03X}"
            for s in self.servers)

    def start(self) -> "DiagManager":
        for s in self.servers:
            s.start()
        return self

    def stop(self, join: bool = True) -> None:
        for s in self.servers:
            s.stop(join=join)

    def __enter__(self) -> "DiagManager":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
