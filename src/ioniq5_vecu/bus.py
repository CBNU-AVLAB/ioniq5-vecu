# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      bus.py
# @brief     vcan0 + cantools send/receive wrapper
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""vcan0 + cantools send/receive wrapper.

Merges the three dbc/*.dbc files into one cantools Database and encodes (TX) /
decodes (RX) by message name + signal dict over SocketCAN (vcan0). All bit packing
goes through cantools (dbc) - no magic numbers in code.

Key parts:
* load_database()   : merge dbc/*.dbc into one (frame_id and names are globally unique).
* CanBus            : python-can Bus wrapper. send(name, signals) / recv() / listen().
* PeriodicTx        : monotonic-clock periodic transmitter. Each period re-encodes the
                      latest signal dict from producer() (for TX frames that change over
                      time, like encoder_pos).

For tests/offline, CanBus(interface="virtual", channel="...",
receive_own_messages=True) gives loopback verification without a real vcan0.
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

# Default channel: virtual CAN that the host brings up beforehand.
DEFAULT_CHANNEL = "vcan0"
DEFAULT_INTERFACE = "socketcan"

# Decode result (message name, signal dict, raw frame).
# typing.Tuple (not builtin tuple[...]) so this module-level alias evaluates on
# Python 3.8 too - `from __future__ import annotations` only defers annotations,
# not this assignment.
DecodedFrame = Tuple[str, dict, can.Message]


def load_database(dbc_dir: Path | str = DBC_DIR) -> Database:
    """Merge dbc/*.dbc into one Database and return it.

    The three ECUs' frame_ids (0x1xx/0x2xx/0x3xx) and message names don't overlap
    globally, so they can simply be merged.
    """
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
    """python-can Bus + merged dbc wrapper.

    Parameters
    ----------
    channel, interface : SocketCAN defaults (vcan0). Use "virtual" for tests.
    database : inject a prebuilt Database (otherwise load_database()).
    bus : inject an already-open can.BusABC (otherwise opened here). If injected,
          close() does not shut it down (ownership stays with the caller).
    receive_own_messages : also receive own TX frames (for loopback tests).
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
        # In the integrated runner several ECU control loops share one bus and send
        # concurrently. Serialize TX for safety.
        self._send_lock = threading.Lock()

    # -- lifecycle --------------------------------------------------------------
    def close(self) -> None:
        if self._owns_bus:
            self.bus.shutdown()

    def __enter__(self) -> "CanBus":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- lookup helpers ---------------------------------------------------------
    def message(self, name: str) -> Message:
        return self.db.get_message_by_name(name)

    def tx_messages(self, node: str) -> list[Message]:
        """Messages sent by `node` (that ECU's TX frames). For periodic-TX setup."""
        return [m for m in self.db.messages if m.senders and node in m.senders]

    # -- send -------------------------------------------------------------------
    @staticmethod
    def _default_phys(sig) -> float:
        """Signal default physical value = raw_initial(GenSigStartValue)*scale+offset."""
        raw = sig.raw_initial or 0
        return raw * sig.scale + sig.offset

    @staticmethod
    def _clamp_to_field(sig, value):
        """Clamp a physical value into the signal's representable bitfield range.

        Depending on scale/bit-width, the spec max may not fit the field by 1 LSB
        (e.g. a % signal with scale=100/65536 -> 100% = raw 65536 > 16-bit max).
        cantools encode raises OverflowError at such boundaries, so clamp to the field
        range just before sending to keep the simulator alive.
        """
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
        """Encode a message name + signal dict and send. Returns the sent can.Message.

        fill_defaults=True fills signals missing from the dict with each signal's
        default (dbc GenSigStartValue) - so control frames with many reserved bits
        (e.g. 0x100) only need their meaningful signals specified.

        Each signal is clamped to its bitfield range just before encoding (_clamp_to_field).
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

    # -- receive ----------------------------------------------------------------
    def decode(self, frame: can.Message) -> Optional[DecodedFrame]:
        """Raw can.Message -> (name, signal dict, frame). None if the ID is not in the dbc."""
        try:
            m = self.db.get_message_by_frame_id(frame.arbitration_id)
        except KeyError:
            return None
        signals = m.decode(frame.data, decode_choices=False, allow_truncated=True)
        return m.name, dict(signals), frame

    def recv(self, timeout: Optional[float] = None) -> Optional[DecodedFrame]:
        """Receive and decode one frame. None on timeout or off-matrix ID."""
        frame = self.bus.recv(timeout)
        if frame is None:
            return None
        return self.decode(frame)

    def listen(self, timeout: Optional[float] = 1.0) -> Iterator[DecodedFrame]:
        """Keep yielding decoded frames (off-matrix IDs are skipped)."""
        while True:
            frame = self.bus.recv(timeout)
            if frame is None:
                continue
            decoded = self.decode(frame)
            if decoded is not None:
                yield decoded


class PeriodicTx:
    """Monotonic-clock periodic transmitter (drift-corrected).

    Each period calls producer() for the latest signal dict, encodes it and sends.
    Use for TX frames whose values change over time (encoder_pos, servo status).

    If period is not given, the dbc cycle_time (ms) is used.
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
            # Drift correction: compute next wake time from an absolute schedule.
            sleep = next_t - time.monotonic()
            if sleep < 0:
                next_t = time.monotonic()  # too far behind: rebase
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
