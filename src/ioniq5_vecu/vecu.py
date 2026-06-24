# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      vecu.py
# @brief     Integrated vECU runner (steering + brake + accel)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""Integrated vECU runner - steering + brake + accel in one process on 1 CanBus + 1 ManualChannel.

Running each ECU separately would (1) collide on the manual UDP port (47100) and
(2) read the same vcan0 from multiple sockets. The integrated runner makes the ECUs
share one bus and one manual channel.

Note: if ECUs each call bus.recv on the same bus they steal frames from each other.
So ECUs run only their control (TX) loops with start(rx=False), and a single
dispatcher here reads the bus and fans decoded frames out to every ECU's
handle_frame (each ECU ignores frames that aren't its own).

  RX  (dispatcher)        -> SteeringEcu / BrakeEcu / AccelEcu.handle_frame
  TX  (each ECU control loop)  0x104 / 0x204 / 0x314 / 0x315  (sent on the shared bus)
"""
from __future__ import annotations

import threading
import time
from typing import Optional

from .bus import CanBus
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
            ecu.start(rx=False)           # control (TX) loop only - rx via shared dispatcher
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
        manual: bool = True) -> None:
    manual_ch = ManualChannel().start() if manual else None
    with CanBus(channel=channel, interface=interface) as bus:
        vecu = VEcu(bus, manual=manual_ch).start()
        nodes = ", ".join(e.spec.node for e in vecu.ecus)
        print(f"[vecu] running - {nodes} (1 bus {channel}/{interface}, single dispatcher). "
              f"Ctrl-C to quit")
        if manual_ch is not None:
            print(f"[vecu] manual UDP on :{manual_ch.port} "
                  f"(steer/brake back-drive, accel->APS_IN)")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[vecu] stopped")
        finally:
            vecu.stop()
            if manual_ch is not None:
                manual_ch.stop()


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description="Integrated vECU (steering + brake + accel, 1 bus / 1 manual channel)")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--no-manual", dest="manual", action="store_false",
                    help="disable the manual UDP side channel")
    args = ap.parse_args()
    run(channel=args.channel, interface=args.interface, manual=args.manual)


if __name__ == "__main__":
    main()
