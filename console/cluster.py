#!/usr/bin/env python3
# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      cluster.py
# @brief     IONIQ5 web instrument cluster (display only)
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#            2026-07-17 updated by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#              : receive gear over gear_link and feed it to the display + speed model
"""IONIQ5 instrument cluster (host-native, display only).

Decodes the official TX frames on vcan0 and shows them as a ccNC-style web cluster.
Nothing is transmitted on the bus (pure display).

Pipeline:
  [vcan0] --cantools--> ClusterState --SSE(text/event-stream)--> [browser Canvas]

* Backend (this file): stdlib http.server only. A CAN-decode thread updates
  ClusterState, and /stream streams snapshots to the browser at ~30Hz.
* Frontend (console/web/): index.html + cluster.js (Canvas render).

Display fields:
  (1) speed        : not on the bus -> derived by vehicle_model (display side)
  (2) steer deg    : ADA_S_104 encoder_pos
  (3) vehicle state: brake swaps the car image, lane bends with steering
  (4) accel %      : ADE_A_314 APS_OUT_PERCENT
  (5) brake mm     : ADA_B_204 encoder_pos (stroke)
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

# -- Decode sources (reuse config.py specs; dbc is the single source of truth for bits) --
# Steering/brake come from ServoSpec, accel from AccelSpec; message/signal names are not
# hardcoded here.
STEER_MSG = STEERING.feedback_msg     # ADA_S_104
STEER_SIG = STEERING.encoder_sig      # encoder_pos
STEER_LIMIT = STEERING.limit_max      # +/-480 deg

BRAKE_MSG = BRAKE.feedback_msg        # ADA_B_204
BRAKE_SIG = BRAKE.encoder_sig         # encoder_pos (mm stroke)
BRAKE_MAX_MM = BRAKE.limit_max        # 60mm (ADA-B limit)
BRAKE_ON_MM = 0.5                     # above this = "brake engaged" (image swap) - display threshold

ACCEL_MSG = ACCEL.status_msg          # ADE_A_314
ACCEL_SIG = ACCEL.out_pct_sig         # APS_OUT_PERCENT (%)

# -- Indicator (actuator state) sources --------------------------------------
# Servo control-mode/fault from fsm_state_id (0x106/0x206); accel control-mode from the
# AD override command (OVR_* bits of 0x311). FSM constants per dbc comments:
#   5 position control / 6 warning / 3 free wheeling, etc.
STEER_FSM_MSG = STEERING.fsm_msg      # ADA_S_106
BRAKE_FSM_MSG = BRAKE.fsm_msg         # ADA_B_206
FSM_SIG = "fsm_state_id"
FSM_CONTROL = 5                       # position control -> control mode (green)
FSM_WARNING = 6                       # warning -> fault (red)

ACCEL_CTRL_MSG = ACCEL.ctrl_msg       # ADE_A_311 (RX command: OVR_* = AD control)

SPEED_MAX = 180.0                     # speedometer gauge full scale (km/h)
BUS_TIMEOUT_S = 1.0                   # "disconnected" if no frame within this time


class ClusterState:
    """Decoded display state. The CAN thread updates it; the SSE handler reads snapshots."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.speed = 0.0       # km/h
        self.steer_deg = 0.0
        self.accel_pct = 0.0
        self.brake_mm = 0.0
        # indicator state - servos have control/fault, accel has control only
        self.steer_ctrl = False
        self.steer_fault = False
        self.brake_ctrl = False
        self.brake_fault = False
        self.accel_ctrl = False
        self.gear = DEFAULT_GEAR  # display-only gear (input.py keyboard -> gear_link UDP)
        self._last_rx = 0.0    # last frame rx time (monotonic); 0 = never received

    def update(self, name: str, signals: dict) -> None:
        """Apply a decoded (message name, signals). Display only, so unknown messages
        are ignored."""
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
                # 0x311 is an AD->ECU command frame - update accel control mode only;
                # do not use it for connection detection (_last_rx).
                self.accel_ctrl = (bool(signals.get("OVR__PERCENT", 0))
                                   or bool(signals.get("OVR_VOLTAGE", 0)))
                return
            else:
                return  # not a displayed frame -> ignore (also not used for connection detection)
            self._last_rx = time.monotonic()  # receiving a frame we display = bus connected

    def set_speed(self, kmh: float) -> None:
        """Apply the speed derived by vehicle_model (display side)."""
        with self._lock:
            self.speed = kmh

    def set_gear(self, gear: str) -> None:
        """Apply the gear letter received over gear_link (UDP) (display only)."""
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
    """Decode vcan0 and update ClusterState. Display only, no TX."""
    with CanBus(channel=channel, interface=interface) as bus:
        while not stop.is_set():
            out = bus.recv(timeout=0.2)
            if out is not None:
                state.update(out[0], out[1])


def _vehicle_loop(state: ClusterState, stop: threading.Event,
                  hz: float = 50.0) -> None:
    """Derive speed from accel/brake into state.speed (display side).
    Sends nothing on the bus."""
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


# -- Static-file + SSE HTTP handler ------------------------------------------
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

        def log_message(self, *args):  # silence
            pass

        # whitelist-based static mapping (prevents path traversal)
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
                return  # browser closed

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

    # gear display (input.py keyboard -> console-internal UDP). Just reflect the gear into state.
    gear_rx = GearReceiver(on_gear=state.set_gear).start()

    httpd = ThreadingHTTPServer((host, port), _make_handler(state, stream_hz))
    httpd.daemon_threads = True
    print(f"[cluster] http://{host}:{port}  (vcan: {channel}/{interface})  Ctrl-C to quit")
    print(f"[cluster] gear rx :{gear_rx.port} (input.py PRND keys)")
    if not (ASSETS_DIR / "ioniq5_basic.png").exists():
        print("[cluster] note: without console/assets/ioniq5_basic.png(+_brake.png) "
              "a fallback render is used.")
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

    ap = argparse.ArgumentParser(description="IONIQ5 web instrument cluster (display only)")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8088)
    args = ap.parse_args()
    run(channel=args.channel, interface=args.interface,
        host=args.host, port=args.port)


if __name__ == "__main__":
    main()
