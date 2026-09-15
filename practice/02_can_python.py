#!/usr/bin/env python3
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      02_can_python.py
# @brief     Student stub: time-varying steering command through the dbc
#
# @date      2026-09-13 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
Exercise 1 — sweep the steering wheel left and right.

Sends a new target angle every 0.05 s and prints the actual angle reported by the
ECU next to it (tracking error).

  target = AMPLITUDE_DEG x sin(2 x pi x t / PERIOD_S)

Fill in 4 places (5 blanks): replace each `____` with the right value.
An unfilled `____` raises NameError on that line.

Setup
  1) Start the vECU in another terminal:
       PYTHONPATH=src .venv/bin/python -m ioniq5_vecu.vecu
  2) Optional — open the cluster to watch the needle:
       .venv/bin/python console/cluster.py      ->  http://127.0.0.1:8088
  3) Optional — watch the frames:
       candump vcan0,101:7FF,104:7FF

Run
  .venv/bin/python practice/02_can_python.py

API
  bus.send(message_name, {signal_name: physical_value}, fill_defaults=True)
  bus.recv(timeout=seconds)  ->  (message_name, signals, raw_frame) or None
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from ioniq5_vecu.bus import CanBus  # noqa: E402

AMPLITUDE_DEG = 300.0     # sweep amplitude (steering limit is ±480 deg)
PERIOD_S = 4.0            # time for one left-right cycle
RUN_S = 12.0              # total run time
SEND_INTERVAL_S = 0.05    # target send interval
PRINT_INTERVAL_S = 0.5    # print interval


def target_angle(elapsed: float) -> float:
    """Target steering angle (deg) at the elapsed time: a sine wave."""
    return AMPLITUDE_DEG * math.sin(2 * math.pi * elapsed / PERIOD_S)


def main() -> None:
    with CanBus(channel="vcan0") as bus:
        # (1) Turn the servo on. The ECU follows position commands only with SON=1.
        #     Hint: the signal dict is {"SON": 1}
        #     (fill_defaults=True fills the other signals with their dbc defaults)
        bus.send("ADA_S_100", ____, fill_defaults=True)
        print(f"Servo ON — sweeping ±{AMPLITUDE_DEG:.0f} deg for {RUN_S:.0f} s (Ctrl-C to stop)")

        t0 = time.monotonic()
        next_send = t0
        next_print = t0
        encoder = 0.0
        try:
            while True:
                now = time.monotonic()
                elapsed = now - t0
                if elapsed >= RUN_S:
                    break

                if now >= next_send:
                    # (2) Send the target angle. The dbc handles the 1/60 deg scaling,
                    #     so pass degrees.
                    #     Hint: the signal name is "target_pos"
                    bus.send("ADA_S_101", {____: target_angle(elapsed)})
                    next_send += SEND_INTERVAL_S

                out = bus.recv(timeout=0.01)
                if out is not None:
                    name, signals, _frame = out
                    # (3) Frames from all three ECUs arrive. Pick the steering feedback
                    #     and read the current angle.
                    #     Hint: the name of message 0x104 and its position signal
                    if name == ____:
                        encoder = signals[____]

                if now >= next_print:
                    target = target_angle(elapsed)
                    print(f"  t={elapsed:5.1f}s   target {target:+7.1f} deg"
                          f"   actual {encoder:+7.1f} deg   error {target - encoder:+6.1f} deg")
                    next_print += PRINT_INTERVAL_S
        except KeyboardInterrupt:
            print("\nstopped")
        finally:
            # (4) Turn the servo off. Keyboard control (console/input.py) works again.
            #     Hint: set SON to 0
            bus.send("ADA_S_100", {"SON": ____}, fill_defaults=True)
            print("Servo OFF")


if __name__ == "__main__":
    main()
