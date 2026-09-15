# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      steering.py
# @brief     ADA-S steering vECU
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
ADA-S steering vECU (BaseServoEcu with the STEERING spec).

  RX  ADA_S_100 SON / ADA_S_101 target_pos
  TX  ADA_S_104 encoder_pos / servo_abs_pos  (10ms)
Manual (SON=0): integrates the steer rate.
"""

from __future__ import annotations

from typing import Optional

from ..bus import CanBus
from ..config import STEERING, ServoSpec
from ..io.manual_channel import ManualChannel
from ..models.servo import ServoModel
from .base_servo import BaseServoEcu, run_servo


class SteeringEcu(BaseServoEcu):
    def __init__(self, canbus: CanBus, spec: ServoSpec = STEERING,
                 model: Optional[ServoModel] = None,
                 manual: Optional[ManualChannel] = None) -> None:
        super().__init__(canbus, spec, model, manual)


def run(channel: str = "vcan0", interface: str = "socketcan",
        demo: bool = False, manual: bool = True,
        spec: ServoSpec = STEERING) -> None:
    # demo: ±90 deg triangle
    run_servo(spec, channel=channel, interface=interface, demo=demo,
              manual=manual, demo_lo=-90.0, demo_hi=90.0)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="ADA-S steering vECU")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--demo", action="store_true",
                    help="inject SON=1 + a ±90 deg target triangle (for candump)")
    ap.add_argument("--no-manual", dest="manual", action="store_false",
                    help="disable the manual UDP side channel")
    args = ap.parse_args()
    run(channel=args.channel, interface=args.interface,
        demo=args.demo, manual=args.manual)


if __name__ == "__main__":
    main()
