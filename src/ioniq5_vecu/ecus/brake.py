# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      brake.py
# @brief     ADA-B braking vECU
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
ADA-B brake vECU (BaseServoEcu with the BRAKE spec).

  RX  ADA_B_200 SON / ADA_B_201 target_pos
  TX  ADA_B_204 encoder_pos / servo_abs_pos  (10ms)
Manual (SON=0): maps the brake pedal (0..1) to a 0~170 mm stroke.
"""

from __future__ import annotations

from typing import Optional

from ..bus import CanBus
from ..config import BRAKE, ServoSpec
from ..io.manual_channel import ManualChannel
from ..models.servo import ServoModel
from .base_servo import BaseServoEcu, run_servo


class BrakeEcu(BaseServoEcu):
    def __init__(self, canbus: CanBus, spec: ServoSpec = BRAKE,
                 model: Optional[ServoModel] = None,
                 manual: Optional[ManualChannel] = None) -> None:
        super().__init__(canbus, spec, model, manual)


def run(channel: str = "vcan0", interface: str = "socketcan",
        demo: bool = False, manual: bool = True,
        spec: ServoSpec = BRAKE) -> None:
    # demo: 0~170 mm stroke triangle
    run_servo(spec, channel=channel, interface=interface, demo=demo,
              manual=manual, demo_lo=0.0, demo_hi=170.0)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="ADA-B brake vECU")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--demo", action="store_true",
                    help="inject SON=1 + a 0~170 mm target triangle (for candump)")
    ap.add_argument("--no-manual", dest="manual", action="store_false",
                    help="disable the manual UDP side channel")
    args = ap.parse_args()
    run(channel=args.channel, interface=args.interface,
        demo=args.demo, manual=args.manual)


if __name__ == "__main__":
    main()
