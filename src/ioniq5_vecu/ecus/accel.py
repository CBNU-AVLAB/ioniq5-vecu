# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      accel.py
# @brief     ADE-A acceleration vECU (APS pedal voltage emulator)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
ADE-A accelerator vECU (APS pedal voltage emulator).

Passes the driver pedal (IN) through to OUT unless an AD override is active.

  RX  ADE_A_311 : CAL_EN, OVR_VOLTAGE, OVR__PERCENT, APS_OVR_PERCENT_VALUE (%),
                  APS1/2_OVR_VOLTAGE_VALUE (V)
  TX  ADE_A_314 : BRK_S (%), APS_OVR_flag, BRK_OVR_flag, APS_IN_PERCENT, APS_OUT_PERCENT
      ADE_A_315 : APS1/2_IN_VOLTAGE, APS1/2_OUT_VOLTAGE (V)

OUT:
  OVR__PERCENT=1 -> OUT% = APS_OVR_PERCENT_VALUE
  OVR_VOLTAGE=1  -> OUT voltage = APS1/2_OVR_VOLTAGE_VALUE (OUT% derived from APS1)
  otherwise      -> OUT = IN
IN is the manual accel pedal; BRK_S is the manual brake pedal.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from ..bus import CanBus
from ..config import ACCEL, AccelSpec
from ..io.manual_channel import ManualChannel
from ..io.manual_protocol import NEUTRAL


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class AccelEcu:
    def __init__(self, canbus: CanBus, spec: AccelSpec = ACCEL,
                 manual: Optional[ManualChannel] = None) -> None:
        self.bus = canbus
        self.spec = spec
        self.manual = manual
        self.period = canbus.message(spec.status_msg).cycle_time / 1000.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        # override command state (updated by RX)
        self.cal_en = False
        self.ovr_pct_on = False
        self.ovr_v_on = False
        self.ovr_pct = 0.0
        self.ovr_v1 = 0.0
        self.ovr_v2 = 0.0

    # ── RX: override command ──────────────────────────────────────────────
    def handle_frame(self, name: str, sig: dict) -> None:
        """Apply a decoded 0x311 frame. Other frames are ignored."""
        if name != self.spec.ctrl_msg:
            return
        with self._lock:
            self.cal_en = bool(sig.get("CAL_EN", 0))
            self.ovr_v_on = bool(sig.get("OVR_VOLTAGE", 0))
            self.ovr_pct_on = bool(sig.get("OVR__PERCENT", 0))
            self.ovr_pct = float(sig.get("APS_OVR_PERCENT_VALUE", 0.0))
            self.ovr_v1 = float(sig.get("APS1_OVR_VOLTAGE_VALUE", 0.0))
            self.ovr_v2 = float(sig.get("APS2_OVR_VOLTAGE_VALUE", 0.0))

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            out = self.bus.recv(timeout=0.1)
            if out is not None:
                self.handle_frame(out[0], out[1])

    # ── %<->voltage mapping (APS1) ────────────────────────────────────────
    def _v_from_pct(self, pct: float) -> float:
        s = self.spec
        return s.aps1_v0 + _clamp(pct, 0.0, 100.0) / 100.0 * (s.aps1_v100 - s.aps1_v0)

    def _pct_from_v(self, v: float) -> float:
        s = self.spec
        return _clamp((v - s.aps1_v0) / (s.aps1_v100 - s.aps1_v0) * 100.0, 0.0, 100.0)

    # ── One cycle: inputs, output selection, send ─────────────────────────
    def _tick(self) -> None:
        s = self.spec
        mi = self.manual.get() if self.manual is not None else NEUTRAL
        with self._lock:
            opct_on, ov_on = self.ovr_pct_on, self.ovr_v_on
            opct, ov1, ov2 = self.ovr_pct, self.ovr_v1, self.ovr_v2

        # inputs (driver pedals)
        aps_in = _clamp(mi.accel, 0.0, 1.0) * 100.0
        brk_s = _clamp(mi.brake, 0.0, 1.0) * 100.0
        aps1_in = self._v_from_pct(aps_in)
        aps2_in = aps1_in * s.aps2_ratio

        # outputs
        if opct_on:                                  # AD percent command
            aps_out = _clamp(opct, 0.0, 100.0)
            aps1_out = self._v_from_pct(aps_out)
            aps2_out = aps1_out * s.aps2_ratio
        elif ov_on:                                  # AD voltage command
            aps1_out, aps2_out = ov1, ov2
            aps_out = self._pct_from_v(ov1)
        else:                                        # pass-through (driver)
            aps_out, aps1_out, aps2_out = aps_in, aps1_in, aps2_in

        aps_ovr_flag = 1 if aps_in > s.aps_driver_pct else 0
        # set while the driver brakes (BRK_S above the threshold)
        brk_ovr_flag = 1 if brk_s > s.brk_driver_pct else 0

        self.bus.send(s.status_msg, {
            "BRK_S": brk_s,
            "APS_OVR_flag": aps_ovr_flag,
            "BRK_OVR_flag": brk_ovr_flag,
            "APS_IN_PERCENT": aps_in,
            "APS_OUT_PERCENT": aps_out,
        })
        self.bus.send(s.voltage_msg, {
            "APS1_IN_VOLTAGE": aps1_in,
            "APS2_IN_VOLTAGE": aps2_in,
            "APS1_OUT_VOLTAGE": aps1_out,
            "APS2_OUT_VOLTAGE": aps2_out,
        })

    def _control_loop(self) -> None:
        next_t = time.monotonic()
        while not self._stop.is_set():
            self._tick()
            next_t += self.period
            sleep = next_t - time.monotonic()
            if sleep < 0:
                next_t = time.monotonic()
                sleep = 0
            self._stop.wait(sleep)

    # ── Lifecycle ─────────────────────────────────────────────────────────
    def start(self, rx: bool = True) -> "AccelEcu":
        """rx=True: own rx loop (standalone). rx=False: frames arrive through handle_frame."""
        if self._threads:
            return self
        self._stop.clear()
        loops = [(self._control_loop, "ctrl")]
        if rx:
            loops.insert(0, (self._rx_loop, "rx"))
        for target, nm in loops:
            t = threading.Thread(target=target, name=f"{self.spec.node}-{nm}",
                                 daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self, join: bool = True) -> None:
        self._stop.set()
        if join:
            for t in self._threads:
                t.join(timeout=self.period * 5 + 0.5)
        self._threads.clear()

    def __enter__(self) -> "AccelEcu":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


# ── Standalone run / candump demo ──────────────────────────────────────────
def _demo_driver(channel: str, interface: str, spec: AccelSpec,
                 stop: threading.Event) -> None:
    """Send an AD percent override (OVR__PERCENT=1) with a 0..100% triangle from a separate bus."""
    drv = CanBus(channel=channel, interface=interface)
    try:
        period_s = 4.0
        t0 = time.monotonic()
        while not stop.is_set():
            frac = ((time.monotonic() - t0) % period_s) / period_s
            tri = 1 - abs(2 * frac - 1)            # 0..1..0
            drv.send(spec.ctrl_msg, {
                "OVR__PERCENT": 1,
                "APS_OVR_PERCENT_VALUE": 100.0 * tri,
            }, fill_defaults=True)
            stop.wait(0.05)
    finally:
        drv.close()


def run(channel: str = "vcan0", interface: str = "socketcan",
        demo: bool = False, manual: bool = True, spec: AccelSpec = ACCEL) -> None:
    manual_ch = ManualChannel().start() if manual else None
    with CanBus(channel=channel, interface=interface) as bus:
        ecu = AccelEcu(bus, spec, manual=manual_ch).start()
        print(f"[{spec.node}] running — {spec.status_msg}/{spec.voltage_msg} @ "
              f"{ecu.period * 1000:.0f}ms. (Ctrl-C to quit)")
        if manual_ch is not None:
            print(f"[{spec.node}] manual UDP on :{manual_ch.port} "
                  f"(accel->APS_IN, brake->BRK_S)")
        stop = threading.Event()
        driver: Optional[threading.Thread] = None
        if demo:
            print(f"[{spec.node}] --demo: OVR__PERCENT=1 + APS_OVR_PERCENT_VALUE "
                  f"0~100% triangle")
            driver = threading.Thread(
                target=_demo_driver, args=(channel, interface, spec, stop),
                daemon=True,
            )
            driver.start()
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print(f"\n[{spec.node}] stopped")
        finally:
            stop.set()
            if driver:
                driver.join(timeout=0.5)
            ecu.stop()
            if manual_ch is not None:
                manual_ch.stop()


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="ADE-A accelerator (APS emulator) vECU")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--demo", action="store_true",
                    help="inject an AD %% override triangle (for candump)")
    ap.add_argument("--no-manual", dest="manual", action="store_false",
                    help="disable the manual UDP side channel")
    args = ap.parse_args()
    run(channel=args.channel, interface=args.interface,
        demo=args.demo, manual=args.manual)


if __name__ == "__main__":
    main()
