# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      vecu.py
# @brief     Integrated vECU runner (steering + brake + accel)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
Integrated vECU runner: steering + brake + accel on one CanBus and one ManualChannel.

A single dispatcher reads the bus and passes every decoded frame to all ECUs
(each ECU ignores frames that are not its own). The ECUs run only their TX loops.

  RX  dispatcher -> SteeringEcu / BrakeEcu / AccelEcu.handle_frame
  TX  ECU control loops -> 0x104 / 0x204 / 0x314·0x315

UDS diagnostics (0x7A0 / 0x7B0 / 0x7C0) run on their own sockets and send nothing
until a request arrives. Disable them with --no-diag.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from .bus import CanBus
from .diag.server import DiagManager
from .ecus.accel import AccelEcu
from .ecus.brake import BrakeEcu
from .ecus.steering import SteeringEcu
from .io.manual_channel import ManualChannel


class VEcu:
    def __init__(self, canbus: CanBus,
                 manual: Optional[ManualChannel] = None) -> None:
        self.bus = canbus
        self.manual = manual
        self.ecus = [
            SteeringEcu(canbus, manual=manual),
            BrakeEcu(canbus, manual=manual),
            AccelEcu(canbus, manual=manual),
        ]
        self._stop = threading.Event()
        self._dispatcher: Optional[threading.Thread] = None

    def _dispatch_loop(self) -> None:
        while not self._stop.is_set():
            out = self.bus.recv(timeout=0.1)
            if out is None:
                continue
            name, sig, _ = out
            for ecu in self.ecus:
                ecu.handle_frame(name, sig)

    def start(self) -> "VEcu":
        self._stop.clear()
        for ecu in self.ecus:
            ecu.start(rx=False)           # TX loop only; the dispatcher handles RX
        self._dispatcher = threading.Thread(
            target=self._dispatch_loop, name="vecu-dispatch", daemon=True)
        self._dispatcher.start()
        return self

    def stop(self, join: bool = True) -> None:
        self._stop.set()
        if join and self._dispatcher:
            self._dispatcher.join(timeout=0.5)
        for ecu in self.ecus:
            ecu.stop(join=join)
        self._dispatcher = None

    def __enter__(self) -> "VEcu":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


def run(channel: str = "vcan0", interface: str = "socketcan",
        manual: bool = True, diag: bool = True) -> None:
    manual_ch = ManualChannel().start() if manual else None
    with CanBus(channel=channel, interface=interface) as bus:
        vecu = VEcu(bus, manual=manual_ch).start()
        nodes = ", ".join(e.spec.node for e in vecu.ecus)
        print(f"[vecu] running — {nodes} (bus {channel}/{interface}, single dispatcher). "
              f"Ctrl-C to quit")
        if manual_ch is not None:
            print(f"[vecu] manual UDP on :{manual_ch.port} "
                  f"(steer/brake back-drive, accel->APS_IN)")
        diag_mgr = DiagManager(vecu.ecus, channel=channel,
                               interface=interface).start() if diag else None
        if diag_mgr is not None:
            print(f"[vecu] UDS diagnostics — {diag_mgr.describe()} "
                  f"(silent until a request arrives)")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[vecu] stopped")
        finally:
            vecu.stop()
            if diag_mgr is not None:
                diag_mgr.stop()
            if manual_ch is not None:
                manual_ch.stop()


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description="Integrated vECU (steering + brake + accel, one bus, one manual channel)")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--no-manual", dest="manual", action="store_false",
                    help="disable the manual UDP side channel")
    ap.add_argument("--no-diag", dest="diag", action="store_false",
                    help="disable the UDS diagnostic layer")
    args = ap.parse_args()
    run(channel=args.channel, interface=args.interface, manual=args.manual,
        diag=args.diag)


if __name__ == "__main__":
    main()
