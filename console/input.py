#!/usr/bin/env python3
# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      input.py
# @brief     Manual control keyboard input (host-native)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""Manual keyboard input (host-native).

Maps arrow keys to manual control inputs and sends them to the container vECU over a
UDP side channel. The official CAN matrix has no "human moves it by hand" message, so
this goes over localhost UDP, never on vcan0.

  Left/Right : steer rate (-1/+1)   Up : accel   Down : brake   ESC/close : quit

Key->payload mapping is isolated in the pure function keys_to_manual() so it can be
tested without pygame/display. pygame is used only for key capture and rendering.
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

SEND_HZ = 50  # send key state 50x/sec (rate-based: stops the vECU if it stalls)


def keys_to_manual(left: bool, right: bool, up: bool, down: bool) -> ManualInput:
    """Pressed arrow keys (bool) -> ManualInput. Pure function (easy to test)."""
    steer = (1.0 if right else 0.0) - (1.0 if left else 0.0)
    accel = 1.0 if up else 0.0
    brake = 1.0 if down else 0.0
    return ManualInput(steer=steer, brake=brake, accel=accel)


def run(host: str = MANUAL_HOST, port: int = MANUAL_PORT) -> None:
    import pygame  # imported here only (headless tests use keys_to_manual only)

    pygame.init()
    screen = pygame.display.set_mode((360, 120))
    pygame.display.set_caption("IONIQ5 manual control - Left/Right steer, Up accel, Down brake")
    font = pygame.font.SysFont(None, 22)
    clock = pygame.time.Clock()

    sender = ManualSender(host, port)
    print(f"[input] sending manual input -> {host}:{port}  (ESC/close to quit)")
    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

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
            screen.blit(font.render(txt, True, (220, 220, 230)), (16, 48))
            pygame.display.flip()
            clock.tick(SEND_HZ)
    finally:
        # one neutral on exit (staleness also stops it, but stop immediately)
        sender.send(ManualInput())
        sender.close()
        pygame.quit()
        print("[input] stopped")


if __name__ == "__main__":
    try:
        run()
    except ImportError:
        print("pygame is required:  pip install pygame", file=sys.stderr)
        sys.exit(1)
