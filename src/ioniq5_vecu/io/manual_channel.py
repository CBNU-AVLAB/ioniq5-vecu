# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      manual_channel.py
# @brief     Manual control UDP side channel (sender/receiver)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""Manual control UDP side channel.

  ManualSender  : sender on the host console side (used by console/input.py).
  ManualChannel : receiver on the container vECU side. Holds the latest input for ECUs to read.

The receiver recvfrom's in a background thread and keeps the latest ManualInput plus
its receive time. get() applies staleness: if input stops (console quits / keys
released) it returns NEUTRAL automatically, so the actuator stops.
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Optional

from .manual_protocol import (
    MANUAL_HOST,
    MANUAL_PORT,
    NEUTRAL,
    ManualInput,
    decode,
    encode,
)

# Treat input as lost (NEUTRAL) if none arrives within this many seconds.
DEFAULT_STALE_AFTER = 0.2


class ManualSender:
    """UDP sender pushing manual input from host console -> container vECU."""

    def __init__(self, host: str = MANUAL_HOST, port: int = MANUAL_PORT) -> None:
        self.addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, mi: ManualInput) -> None:
        self._sock.sendto(encode(mi), self.addr)

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "ManualSender":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class ManualChannel:
    """vECU-side manual-input receiver. Provides the latest value with staleness applied."""

    def __init__(
        self,
        host: str = MANUAL_HOST,
        port: int = MANUAL_PORT,
        stale_after: float = DEFAULT_STALE_AFTER,
    ) -> None:
        self.stale_after = stale_after
        # SO_REUSEADDR is deliberately not used. With a single receiver, allowing a
        # duplicate bind would let UDP leak into a zombie socket on a second vECU and
        # fail silently. Without it, a duplicate start fails loudly with
        # "Address already in use".
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.1)
        self._lock = threading.Lock()
        self._latest: ManualInput = NEUTRAL
        self._ts: float = 0.0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def port(self) -> int:
        return self._sock.getsockname()[1]

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break  # socket closed
            mi = decode(data)
            if mi is None:
                continue  # ignore broken packet
            with self._lock:
                self._latest = mi
                self._ts = time.monotonic()

    def get(self) -> ManualInput:
        """Latest manual input. NEUTRAL if stale (no input for > stale_after seconds)."""
        with self._lock:
            if time.monotonic() - self._ts > self.stale_after:
                return NEUTRAL
            return self._latest

    def start(self) -> "ManualChannel":
        if self._thread and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="manual-rx", daemon=True
        )
        self._thread.start()
        return self

    def stop(self, join: bool = True) -> None:
        self._stop.set()
        self._sock.close()
        if join and self._thread:
            self._thread.join(timeout=0.5)
        self._thread = None

    def __enter__(self) -> "ManualChannel":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
