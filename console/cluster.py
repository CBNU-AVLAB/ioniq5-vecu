#!/usr/bin/env python3
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      cluster.py
# @brief     IONIQ5 web instrument cluster (display only)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#            2026-07-17 updated by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#              : receive gear over gear_link and feed it to the display + speed model

"""
IONIQ5 web instrument cluster (display only).

  [vcan0] --cantools--> ClusterState --SSE--> [browser canvas]

* Backend (this file): stdlib http.server. A decode thread updates ClusterState and
  /stream sends snapshots to the browser at ~30 Hz.
* Frontend (console/web/): index.html + cluster.js.

Displayed values:
  speed         derived by vehicle_model from accel/brake
  steering deg  ADA_S_104 encoder_pos
  accel %       ADE_A_314 APS_OUT_PERCENT
  brake mm      ADA_B_204 encoder_pos (stroke)
  indicators    fsm_state_id (0x106/0x206), AD override bits (0x311)
  gear          from console/input.py over gear_link (UDP)
"""

from __future__ import annotations

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "src"))

from ioniq5_vecu.bus import CanBus  # noqa: E402
from ioniq5_vecu.config import ACCEL, BRAKE, STEERING  # noqa: E402

from gear_link import DEFAULT_GEAR, GearReceiver  # console/gear_link.py  # noqa: E402
from vehicle_model import VehicleModel  # console/vehicle_model.py  # noqa: E402

WEB_DIR = HERE / "web"
ASSETS_DIR = HERE / "assets"

# ── Decoded signals (from the specs in config.py) ───────────────────────────
STEER_MSG = STEERING.feedback_msg     # ADA_S_104
STEER_SIG = STEERING.encoder_sig      # encoder_pos
STEER_LIMIT = STEERING.limit_max      # ±480 deg

BRAKE_MSG = BRAKE.feedback_msg        # ADA_B_204
BRAKE_SIG = BRAKE.encoder_sig         # encoder_pos (mm stroke)
BRAKE_MAX_MM = BRAKE.limit_max        # 170 mm
BRAKE_ON_MM = 0.5                     # above this the brake image is shown

ACCEL_MSG = ACCEL.status_msg          # ADE_A_314
ACCEL_SIG = ACCEL.out_pct_sig         # APS_OUT_PERCENT (%)

# ── Indicator sources ───────────────────────────────────────────────────────
# Servo control/fault from fsm_state_id (0x106/0x206),
# accel control from the AD override bits (0x311).
STEER_FSM_MSG = STEERING.fsm_msg      # ADA_S_106
BRAKE_FSM_MSG = BRAKE.fsm_msg         # ADA_B_206
FSM_SIG = "fsm_state_id"
FSM_CONTROL = 5                       # position control -> green
FSM_WARNING = 6                       # warning -> red

ACCEL_CTRL_MSG = ACCEL.ctrl_msg       # ADE_A_311 (AD override command)

SPEED_MAX = 180.0                     # speedometer full scale (km/h)
BUS_TIMEOUT_S = 1.0                   # no displayed frame for this long -> disconnected


class ClusterState:
    """Decoded display state. Updated by the CAN thread, read by the SSE handler."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.speed = 0.0       # km/h
        self.steer_deg = 0.0
        self.accel_pct = 0.0
        self.brake_mm = 0.0
        self.steer_ctrl = False
        self.steer_fault = False
        self.brake_ctrl = False
        self.brake_fault = False
        self.accel_ctrl = False
        self.gear = DEFAULT_GEAR  # display-only gear from input.py (gear_link)
        self._last_rx = 0.0    # monotonic time of the last displayed frame; 0 = none yet

    def update(self, name: str, signals: dict) -> None:
        """Apply a decoded frame. Frames not shown on the cluster are ignored."""
        with self._lock:
            if name == STEER_MSG:
                self.steer_deg = float(signals[STEER_SIG])
            elif name == BRAKE_MSG:
                self.brake_mm = float(signals[BRAKE_SIG])
            elif name == ACCEL_MSG:
                self.accel_pct = float(signals[ACCEL_SIG])
            elif name == STEER_FSM_MSG:
                fsm = int(signals[FSM_SIG])
                self.steer_ctrl = fsm == FSM_CONTROL
                self.steer_fault = fsm == FSM_WARNING
            elif name == BRAKE_FSM_MSG:
                fsm = int(signals[FSM_SIG])
                self.brake_ctrl = fsm == FSM_CONTROL
                self.brake_fault = fsm == FSM_WARNING
            elif name == ACCEL_CTRL_MSG:
                # 0x311 is an AD command: update accel control only, not the connection state
                self.accel_ctrl = (bool(signals.get("OVR__PERCENT", 0))
                                   or bool(signals.get("OVR_VOLTAGE", 0)))
                return
            else:
                return  # not displayed
            self._last_rx = time.monotonic()  # a displayed frame arrived -> connected

    def set_speed(self, kmh: float) -> None:
        """Set the speed derived by vehicle_model."""
        with self._lock:
            self.speed = kmh

    def set_gear(self, gear: str) -> None:
        """Set the gear received over gear_link."""
        with self._lock:
            self.gear = gear

    def snapshot(self) -> dict:
        with self._lock:
            brake_mm = self.brake_mm
            connected = (self._last_rx > 0.0
                         and time.monotonic() - self._last_rx < BUS_TIMEOUT_S)
            return {
                "speed": self.speed,
                "speed_max": SPEED_MAX,
                "steer_deg": self.steer_deg,
                "steer_limit": STEER_LIMIT,
                "accel_pct": self.accel_pct,
                "brake_mm": brake_mm,
                "brake_max_mm": BRAKE_MAX_MM,
                "brake_on": brake_mm > BRAKE_ON_MM,
                "connected": connected,
                "steer_ctrl": self.steer_ctrl,
                "steer_fault": self.steer_fault,
                "brake_ctrl": self.brake_ctrl,
                "brake_fault": self.brake_fault,
                "accel_ctrl": self.accel_ctrl,
                "gear": self.gear,
            }


def _decode_loop(channel: str, interface: str, state: ClusterState,
                 stop: threading.Event) -> None:
    """Decode vcan0 into ClusterState (receive only)."""
    with CanBus(channel=channel, interface=interface) as bus:
        while not stop.is_set():
            out = bus.recv(timeout=0.2)
            if out is not None:
                state.update(out[0], out[1])


def _vehicle_loop(state: ClusterState, stop: threading.Event,
                  hz: float = 50.0) -> None:
    """Derive the speed from accel/brake into state.speed. Sends nothing on the bus."""
    vm = VehicleModel(max_kmh=SPEED_MAX)
    period = 1.0 / hz
    last = time.monotonic()
    while not stop.is_set():
        now = time.monotonic()
        dt = now - last
        last = now
        snap = state.snapshot()
        state.set_speed(
            vm.step(dt, snap["accel_pct"], snap["brake_mm"], snap["brake_max_mm"],
                    snap["gear"])
        )
        stop.wait(period)


# ── Static files + SSE HTTP handler ─────────────────────────────────────────
_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".svg": "image/svg+xml",
}


def _make_handler(state: ClusterState, stream_hz: float):
    period = 1.0 / stream_hz

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):  # silence request logs
            pass

        # Whitelisted static paths (no path traversal)
        def _resolve(self) -> Optional[Path]:
            path = self.path.split("?", 1)[0]
            if path == "/":
                return WEB_DIR / "index.html"
            if path.startswith("/assets/"):
                base, rel = ASSETS_DIR, path[len("/assets/"):]
            else:
                base, rel = WEB_DIR, path.lstrip("/")
            target = (base / rel).resolve()
            if base in target.parents and target.is_file():
                return target
            return None

        def _send_file(self, fp: Path) -> None:
            data = fp.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type",
                             _CONTENT_TYPES.get(fp.suffix, "application/octet-stream"))
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _send_stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(state.snapshot())
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return  # browser closed the stream

        def do_GET(self):
            if self.path.split("?", 1)[0] == "/stream":
                self._send_stream()
                return
            fp = self._resolve()
            if fp is None:
                self.send_error(404)
                return
            self._send_file(fp)

    return Handler


def run(channel: str = "vcan0", interface: str = "socketcan",
        host: str = "127.0.0.1", port: int = 8088,
        stream_hz: float = 30.0) -> None:
    state = ClusterState()
    stop = threading.Event()
    decoder = threading.Thread(
        target=_decode_loop, args=(channel, interface, state, stop),
        name="cluster-decode", daemon=True,
    )
    decoder.start()
    vehicle = threading.Thread(
        target=_vehicle_loop, args=(state, stop),
        name="cluster-vehicle", daemon=True,
    )
    vehicle.start()

    # gear display: input.py -> gear_link UDP -> state
    gear_rx = GearReceiver(on_gear=state.set_gear).start()

    httpd = ThreadingHTTPServer((host, port), _make_handler(state, stream_hz))
    httpd.daemon_threads = True
    print(f"[cluster] http://{host}:{port}  (vcan: {channel}/{interface})  Ctrl-C to quit")
    print(f"[cluster] gear input on :{gear_rx.port} (P/R/N/D keys in input.py)")
    if not (ASSETS_DIR / "ioniq5_basic.png").exists():
        print("[cluster] note: console/assets/ioniq5_basic.png (+_brake.png) not found; "
              "using fallback rendering.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[cluster] stopped")
    finally:
        stop.set()
        gear_rx.stop()
        httpd.shutdown()


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="IONIQ5 web cluster (display only)")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8088)
    args = ap.parse_args()
    run(channel=args.channel, interface=args.interface,
        host=args.host, port=args.port)


if __name__ == "__main__":
    main()
