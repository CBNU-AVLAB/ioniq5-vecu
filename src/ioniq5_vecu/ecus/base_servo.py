# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      base_servo.py
# @brief     Single-axis position servo vECU common engine
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""Common engine for the single-axis position-servo vECU (shared steering/brake logic).

Steering (ADA-S) and braking (ADA-B) are the same servo controller (IDs 0x10x <->
0x20x, 1:1). The common RX/control/TX logic lives here; units, limits, message names
and manual mapping are injected via ServoSpec. steering.py / brake.py are thin
wrappers differing only in spec.

  RX  <ctrl_msg> SON       -> ServoModel.set_enabled
      <target_msg> target  -> ServoModel.set_target
  TX  <feedback_msg>       -> encoder_pos / servo_abs_pos (at cycle_time)

Manual back-drive (SON=0): branches on spec.manual_mode
  * rate     : integrate rate(-1..1) at manual_speed (hand-turning feel, like steering)
  * absolute : map 0..1 directly to [limit_min, limit_max] stroke (like the brake pedal)
"""
from __future__ import annotations

import threading
import time
from typing import Optional

from ..bus import CanBus
from ..config import ServoSpec
from ..io.manual_channel import ManualChannel
from ..models.servo import ServoModel


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class ServoFsm:
    """Minimal emulation of the controller state machine (dbc ADA_*_106 fsm_state_id).

    Real controller definition (per dbc comments):
      0 start / 1 wait driver ready / 2 read absolute position / 3 free wheeling
      / 4 wait RD on / 5 position control / 6 warning

    Here we walk the boot chain (START->...->FREE_WHEELING) one step per tick, then
    switch between FREE_WHEELING (manual) and POSITION_CONTROL (AD tracking) based on
    SON. On fault it drops to WARNING (fault source not modeled yet - indicator hook).
    """

    (START, WAIT_DRIVER_READY, READ_ABS_POS, FREE_WHEELING,
     WAIT_RD_ON, POSITION_CONTROL, WARNING) = range(7)
    _BOOT = (START, WAIT_DRIVER_READY, READ_ABS_POS, FREE_WHEELING)

    def __init__(self) -> None:
        self.state = self.START
        self._boot_i = 0

    def step(self, enabled: bool, fault: bool = False) -> int:
        """Pick the next state from SON/fault and return fsm_state_id."""
        if fault:
            self.state = self.WARNING
        elif self._boot_i < len(self._BOOT) - 1:
            self._boot_i += 1                        # advance one boot step
            self.state = self._BOOT[self._boot_i]
        else:
            self.state = self.POSITION_CONTROL if enabled else self.FREE_WHEELING
        return self.state


class BaseServoEcu:
    def __init__(
        self,
        canbus: CanBus,
        spec: ServoSpec,
        model: Optional[ServoModel] = None,
        manual: Optional[ManualChannel] = None,
    ) -> None:
        self.bus = canbus
        self.spec = spec
        self.model = model or ServoModel(
            spec.limit_min, spec.limit_max, spec.max_speed
        )
        self.manual = manual
        self.fsm = ServoFsm()
        self.fault = False   # WARNING/ALM trigger hook - fault source not implemented yet
        self.period = canbus.message(spec.feedback_msg).cycle_time / 1000.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # -- RX: apply command frames ----------------------------------------------
    def handle_frame(self, name: str, sig: dict) -> None:
        """Apply a decoded (name, signals). Public so the integrated runner's single
        dispatcher can fan the same frame to every ECU. Ignores frames that aren't its own."""
        s = self.spec
        if name == s.target_msg:
            with self._lock:
                self.model.set_target(sig[s.target_sig])
        elif name == s.servo_ctrl_msg:
            with self._lock:
                self.model.set_enabled(bool(sig.get(s.enable_sig, 0)))

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            out = self.bus.recv(timeout=0.1)
            if out is not None:
                self.handle_frame(out[0], out[1])

    # -- manual back-drive (only when control is OFF) --------------------------
    def _apply_manual(self, dt: float) -> None:
        if self.manual is None:
            return
        s = self.spec
        val = getattr(self.manual.get(), s.manual_axis)
        if s.manual_mode == "rate":
            if val:  # integrate by the speed the human turns at
                self.model.set_position(
                    self.model.position + val * s.manual_speed * dt
                )
        else:  # absolute: pedal depression (0..1) -> stroke
            self.model.set_position(
                s.limit_min + _clamp(val, 0.0, 1.0) * (s.limit_max - s.limit_min)
            )

    # -- control: integrate + send feedback/status/FSM -------------------------
    def _send_feedback(self, position: float) -> None:
        s = self.spec
        self.bus.send(s.feedback_msg, {s.encoder_sig: position, s.abs_sig: position})

    def _send_status(self, enabled: bool, fsm_state: int,
                     in_pos: bool, zsp: bool) -> None:
        """Send 0x105/0x205 servo status bits + 0x106/0x206 fsm_state_id.
        ALM/RD are 0=fault, 1=normal per dbc."""
        s = self.spec
        ready = 0 if self.fault else 1
        self.bus.send(s.status_msg, {
            s.enable_sig: 1 if enabled else 0,            # SON monitor
            "RD": ready,
            "ALM": ready,
            "INP": 1 if (enabled and in_pos) else 0,
            "ZSP": 1 if zsp else 0,
        }, fill_defaults=True)
        self.bus.send(s.fsm_msg, {"fsm_state_id": fsm_state}, fill_defaults=True)

    def _control_loop(self) -> None:
        last = time.monotonic()
        next_t = last
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - last
            last = now
            with self._lock:
                enabled = self.model.enabled
                if enabled:
                    position = self.model.step(dt)   # AD: track target
                else:
                    self._apply_manual(dt)           # manual back-drive
                    position = self.model.step(dt)   # disabled -> hold position (vel 0)
                in_pos = self.model.in_position()
                zsp = self.model.at_zero_speed()
            fsm_state = self.fsm.step(enabled, self.fault)
            self._send_feedback(position)
            self._send_status(enabled, fsm_state, in_pos, zsp)

            next_t += self.period
            sleep = next_t - time.monotonic()
            if sleep < 0:
                next_t = time.monotonic()
                sleep = 0
            self._stop.wait(sleep)

    # -- lifecycle -------------------------------------------------------------
    def start(self, rx: bool = True) -> "BaseServoEcu":
        """rx=True: read the bus in an own rx loop (standalone).
        rx=False: the integrated runner feeds frames via handle_frame, so control loop only."""
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

    def __enter__(self) -> "BaseServoEcu":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


# -- real vcan0 run / candump demo (shared steering & brake) -----------------
def _demo_driver(channel: str, interface: str, spec: ServoSpec,
                 lo: float, hi: float, period_s: float,
                 stop: threading.Event) -> None:
    """Sweep SON=1 + target as a lo~hi triangle on a separate bus to move the ECU.
    Lets candump show command (target_msg) and tracking (feedback_msg) together."""
    drv = CanBus(channel=channel, interface=interface)
    try:
        drv.send(spec.servo_ctrl_msg, {spec.enable_sig: 1}, fill_defaults=True)
        t0 = time.monotonic()
        while not stop.is_set():
            frac = ((time.monotonic() - t0) % period_s) / period_s
            tri = 1 - abs(2 * frac - 1)             # 0..1..0 triangle
            drv.send(spec.target_msg, {spec.target_sig: lo + (hi - lo) * tri})
            stop.wait(0.05)
    finally:
        drv.close()


def run_servo(spec: ServoSpec, channel: str = "vcan0",
              interface: str = "socketcan", demo: bool = False,
              manual: bool = True, demo_lo: float = 0.0, demo_hi: float = 0.0,
              demo_period: float = 4.0) -> None:
    manual_ch = ManualChannel().start() if manual else None
    with CanBus(channel=channel, interface=interface) as bus:
        ecu = BaseServoEcu(bus, spec, manual=manual_ch).start()
        print(f"[{spec.node}] running - sending {spec.feedback_msg} @ "
              f"{ecu.period * 1000:.0f}ms. (Ctrl-C to quit)")
        if manual_ch is not None:
            print(f"[{spec.node}] manual UDP on :{manual_ch.port} "
                  f"(back-drive when SON=0)")
        stop = threading.Event()
        driver: Optional[threading.Thread] = None
        if demo:
            print(f"[{spec.node}] --demo: injecting SON=1 + {spec.target_msg} "
                  f"{demo_lo:g}~{demo_hi:g}{spec.unit} triangle")
            driver = threading.Thread(
                target=_demo_driver,
                args=(channel, interface, spec, demo_lo, demo_hi, demo_period, stop),
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
