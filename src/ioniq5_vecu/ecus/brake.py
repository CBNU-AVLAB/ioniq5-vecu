# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      brake.py
# @brief     ADA-B braking vECU
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""ADA-B braking vECU.

A thin wrapper over the common servo engine (base_servo.BaseServoEcu) with the
BRAKE spec injected. Same servo controller as steering - only units (mm), limits
(0~60) and messages (0x20x) differ.
  RX  ADA_B_201 target_pos / ADA_B_200 SON
  TX  ADA_B_204 encoder_pos / servo_abs_pos  (10ms)
Manual (SON=0): map the brake pedal (absolute 0..1) directly to 0~60mm stroke.
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
    # demo: 0~60mm stroke triangle
    run_servo(spec, channel=channel, interface=interface, demo=demo,
              manual=manual, demo_lo=0.0, demo_hi=60.0)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="ADA-B braking vECU")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--demo", action="store_true",
                    help="inject SON=1 + target 0~60mm triangle (for candump)")
    ap.add_argument("--no-manual", dest="manual", action="store_false",
                    help="disable the manual UDP side channel")
    args = ap.parse_args()
    run(channel=args.channel, interface=args.interface,
        demo=args.demo, manual=args.manual)


if __name__ == "__main__":
    main()
