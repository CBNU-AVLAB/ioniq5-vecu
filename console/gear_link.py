#!/usr/bin/env python3
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      gear_link.py
# @brief     Console-internal UDP channel for gear display (display only, no vECU/CAN)
#
# @date      2026-07-17 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
Console-internal UDP channel for the gear display (not sent to the vECU or vcan0).

  GearSender   : input.py side, sends one ASCII gear letter on key press
  GearReceiver : cluster.py side, keeps the latest gear

The gear stays until the next change. Unknown values are ignored.
"""

from __future__ import annotations

import socket
import threading
from typing import Callable, Optional

GEAR_HOST = "127.0.0.1"
GEAR_PORT = 47101            # gear display UDP port
VALID_GEARS = ("P", "R", "N", "D")
DEFAULT_GEAR = "P"


class GearSender:
    """Sends the gear letter from input.py to cluster.py over UDP."""

    def __init__(self, host: str = GEAR_HOST, port: int = GEAR_PORT) -> None:
        self.addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, gear: str) -> None:
        """Send a gear letter. Invalid gears are not sent."""
        gear = gear.upper()
        if gear in VALID_GEARS:
            self._sock.sendto(gear.encode("ascii"), self.addr)

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "GearSender":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class GearReceiver:
    """Cluster-side gear receiver. Keeps the latest gear and calls on_gear for each one received."""

    def __init__(
        self,
        host: str = GEAR_HOST,
        port: int = GEAR_PORT,
        on_gear: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._on_gear = on_gear
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._gear = DEFAULT_GEAR
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def port(self) -> int:
        return self._sock.getsockname()[1]

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(16)
            except socket.timeout:
                continue
            except OSError:
                break  # socket closed
            gear = data.decode("ascii", "ignore").strip().upper()
            if gear not in VALID_GEARS:
                continue  # ignore unknown values
            with self._lock:
                self._gear = gear
            if self._on_gear is not None:
                self._on_gear(gear)

    def get(self) -> str:
        """Latest gear (kept until the next change)."""
        with self._lock:
            return self._gear

    def start(self) -> "GearReceiver":
        if self._thread and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="gear-rx", daemon=True)
        self._thread.start()
        return self

    def stop(self, join: bool = True) -> None:
        self._stop.set()
        self._sock.close()
        if join and self._thread:
            self._thread.join(timeout=0.5)
        self._thread = None

    def __enter__(self) -> "GearReceiver":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
