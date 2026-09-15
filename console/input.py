#!/usr/bin/env python3
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      input.py
# @brief     Manual control keyboard input (host-native)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#            2026-07-17 updated by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#              : add PRND gear keys and mirror steering sign (left +1 / right -1)

"""
Manual-control keyboard input (host).

Sends the arrow keys as manual input to the vECU over the UDP side channel.

  Left/Right : steering rate (left +1 / right -1)
  Up : accel   Down : brake
  P/R/N/D : gear shown on the cluster   ESC / close window : quit

keys_to_manual() is a pure function and can be tested without pygame.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from ioniq5_vecu.io.manual_channel import ManualSender  # noqa: E402
from ioniq5_vecu.io.manual_protocol import (  # noqa: E402
    MANUAL_HOST,
    MANUAL_PORT,
    ManualInput,
)

from gear_link import DEFAULT_GEAR, GearSender  # console/gear_link.py  # noqa: E402

SEND_HZ = 50  # key state sends per second


def keys_to_manual(left: bool, right: bool, up: bool, down: bool) -> ManualInput:
    """Pressed arrow keys -> ManualInput. Steering: left = +1, right = -1."""
    steer = (1.0 if left else 0.0) - (1.0 if right else 0.0)
    accel = 1.0 if up else 0.0
    brake = 1.0 if down else 0.0
    return ManualInput(steer=steer, brake=brake, accel=accel)


def run(host: str = MANUAL_HOST, port: int = MANUAL_PORT) -> None:
    import pygame  # imported here so keys_to_manual() works without pygame

    pygame.init()
    screen = pygame.display.set_mode((360, 120))
    pygame.display.set_caption("IONIQ5 Manual Input")
    font = pygame.font.SysFont(None, 22)
    clock = pygame.time.Clock()

    # key -> gear (display only), applied on KEYDOWN
    gear_keys = {
        pygame.K_p: "P", pygame.K_r: "R", pygame.K_n: "N", pygame.K_d: "D",
    }

    sender = ManualSender(host, port)
    gear_sender = GearSender()
    gear = DEFAULT_GEAR
    gear_sender.send(gear)  # sync the initial gear if the cluster is already running
    print(f"[input] sending manual input -> {host}:{port}  (ESC / close window to quit)")
    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key in gear_keys:
                        gear = gear_keys[event.key]
                        gear_sender.send(gear)

            k = pygame.key.get_pressed()
            mi = keys_to_manual(
                left=k[pygame.K_LEFT],
                right=k[pygame.K_RIGHT],
                up=k[pygame.K_UP],
                down=k[pygame.K_DOWN],
            )
            sender.send(mi)

            screen.fill((20, 20, 28))
            txt = f"steer {mi.steer:+.0f}   accel {mi.accel:.0f}   brake {mi.brake:.0f}"
            screen.blit(font.render(txt, True, (220, 220, 230)), (16, 40))
            screen.blit(font.render(f"gear {gear}", True, (150, 210, 150)), (16, 70))
            pygame.display.flip()
            clock.tick(SEND_HZ)
    finally:
        # send neutral once before exiting
        sender.send(ManualInput())
        sender.close()
        gear_sender.close()
        pygame.quit()
        print("[input] stopped")


if __name__ == "__main__":
    try:
        run()
    except ImportError:
        print("pygame is required:  pip install pygame", file=sys.stderr)
        sys.exit(1)
