# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      bus.py
# @brief     vcan0 + cantools send/receive wrapper
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

"""
vcan0 + cantools send/receive wrapper.

Merges dbc/*.dbc into one cantools Database and sends/receives frames on SocketCAN
as a message name plus a signal dict.

* load_database() : merge dbc/*.dbc into one Database
* CanBus          : python-can Bus wrapper — send() / recv() / listen()
* PeriodicTx      : periodic sender driven by a monotonic clock
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Callable, Iterator, Optional, Tuple

import can
import cantools
from cantools.database import Database
from cantools.database.can import Message

ROOT = Path(__file__).resolve().parents[2]  # ioniq5-vecu/
DBC_DIR = ROOT / "dbc"

# Default channel: virtual CAN brought up on the host
DEFAULT_CHANNEL = "vcan0"
DEFAULT_INTERFACE = "socketcan"

# Decoded frame: (message name, signal dict, raw frame)
DecodedFrame = Tuple[str, dict, can.Message]


def load_database(dbc_dir: Path | str = DBC_DIR) -> Database:
    """Merge dbc/*.dbc into a single Database."""
    dbc_dir = Path(dbc_dir)
    files = sorted(dbc_dir.glob("*.dbc"))
    if not files:
        raise FileNotFoundError(
            f"{dbc_dir} does not contain any .dbc files. Run tools/xlsx_to_dbc.py first."
        )
    db = Database()
    for f in files:
        db.add_dbc_file(str(f))
    return db


class CanBus:
    """python-can Bus with the merged dbc.

    channel, interface   : SocketCAN by default (vcan0). Tests use "virtual".
    database             : prebuilt Database (default: load_database()).
    bus                  : already-open can.BusABC. An injected bus is not closed by close().
    receive_own_messages : also receive frames sent by this bus (loopback tests).
    """

    def __init__(
        self,
        channel: str = DEFAULT_CHANNEL,
        interface: str = DEFAULT_INTERFACE,
        *,
        database: Optional[Database] = None,
        bus: Optional[can.BusABC] = None,
        receive_own_messages: bool = False,
    ) -> None:
        self.db = database if database is not None else load_database()
        self._owns_bus = bus is None
        if bus is None:
            bus = can.Bus(
                channel=channel,
                interface=interface,
                receive_own_messages=receive_own_messages,
            )
        self.bus = bus
        # Several ECU control loops can share this bus; serialize sends.
        self._send_lock = threading.Lock()

    # ── Lifecycle ─────────────────────────────────────────────────────────
    def close(self) -> None:
        if self._owns_bus:
            self.bus.shutdown()

    def __enter__(self) -> "CanBus":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ── Lookup ────────────────────────────────────────────────────────────
    def message(self, name: str) -> Message:
        return self.db.get_message_by_name(name)

    def tx_messages(self, node: str) -> list[Message]:
        """Messages sent by `node` (that ECU's TX frames)."""
        return [m for m in self.db.messages if m.senders and node in m.senders]

    # ── Send ──────────────────────────────────────────────────────────────
    @staticmethod
    def _default_phys(sig) -> float:
        """Default physical value: raw_initial (GenSigStartValue) * scale + offset."""
        raw = sig.raw_initial or 0
        return raw * sig.scale + sig.offset

    @staticmethod
    def _clamp_to_field(sig, value):
        """Clamp a physical value to the range the signal's bit field can represent."""
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return value
        if sig.is_signed:
            rlo, rhi = -(1 << (sig.length - 1)), (1 << (sig.length - 1)) - 1
        else:
            rlo, rhi = 0, (1 << sig.length) - 1
        a = rlo * sig.scale + sig.offset
        b = rhi * sig.scale + sig.offset
        lo, hi = (a, b) if a <= b else (b, a)
        return lo if value < lo else hi if value > hi else value

    def send(
        self,
        name: str,
        signals: dict,
        *,
        padding: bool = True,
        fill_defaults: bool = False,
    ) -> can.Message:
        """Encode and send a message. Returns the sent can.Message.

        fill_defaults=True fills signals missing from `signals` with their dbc
        default (GenSigStartValue). Values are clamped to their bit fields.
        """
        m = self.db.get_message_by_name(name)
        if fill_defaults:
            full = {s.name: self._default_phys(s) for s in m.signals}
            full.update(signals)
            signals = full
        by_name = {s.name: s for s in m.signals}
        signals = {k: (self._clamp_to_field(by_name[k], v) if k in by_name else v)
                   for k, v in signals.items()}
        data = m.encode(signals, padding=padding, strict=True)
        frame = can.Message(
            arbitration_id=m.frame_id,
            data=data,
            is_extended_id=m.is_extended_frame,
        )
        with self._send_lock:
            self.bus.send(frame)
        return frame

    # ── Receive ───────────────────────────────────────────────────────────
    def decode(self, frame: can.Message) -> Optional[DecodedFrame]:
        """Raw frame -> (name, signals, frame). None if the ID is not in the dbc."""
        try:
            m = self.db.get_message_by_frame_id(frame.arbitration_id)
        except KeyError:
            return None
        signals = m.decode(frame.data, decode_choices=False, allow_truncated=True)
        return m.name, dict(signals), frame

    def recv(self, timeout: Optional[float] = None) -> Optional[DecodedFrame]:
        """Receive and decode one frame. None on timeout or for an unknown ID."""
        frame = self.bus.recv(timeout)
        if frame is None:
            return None
        return self.decode(frame)

    def listen(self, timeout: Optional[float] = 1.0) -> Iterator[DecodedFrame]:
        """Yield decoded frames forever (unknown IDs are skipped)."""
        while True:
            frame = self.bus.recv(timeout)
            if frame is None:
                continue
            decoded = self.decode(frame)
            if decoded is not None:
                yield decoded


class PeriodicTx:
    """Periodic sender with drift compensation.

    Calls producer() every period and sends the returned signals.
    If period is omitted, the message's dbc cycle_time is used.
    """

    def __init__(
        self,
        canbus: CanBus,
        name: str,
        producer: Callable[[], dict],
        period: Optional[float] = None,
    ) -> None:
        self.canbus = canbus
        self.name = name
        self.producer = producer
        if period is None:
            ct = canbus.message(name).cycle_time
            if not ct:
                raise ValueError(
                    f"{name} does not have cycle_time. Specify period directly."
                )
            period = ct / 1000.0
        self.period = period
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def _run(self) -> None:
        next_t = time.monotonic()
        while not self._stop.is_set():
            try:
                self.canbus.send(self.name, self.producer())
            except can.CanError:
                pass  # transient bus error: retry next period
            next_t += self.period
            # Schedule against absolute time to avoid drift.
            sleep = next_t - time.monotonic()
            if sleep < 0:
                next_t = time.monotonic()  # fell behind: rebase
                sleep = 0
            self._stop.wait(sleep)

    def start(self) -> "PeriodicTx":
        if self._thread and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name=f"PeriodicTx-{self.name}", daemon=True
        )
        self._thread.start()
        return self

    def stop(self, join: bool = True) -> None:
        self._stop.set()
        if join and self._thread:
            self._thread.join(timeout=self.period * 5 + 0.5)

    def __enter__(self) -> "PeriodicTx":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
