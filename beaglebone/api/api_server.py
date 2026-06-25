#!/usr/bin/env python3
"""
api_server.py — BeagleBone HTTP observability API

Thin read-only adapter over SHM, SQLite, and the filesystem.
No state management, no control logic, no new IPC paths.

Endpoints:
    GET /state          Full sensor snapshot from SHM
    GET /devices        Per-device online status (heartbeat + reed + PIR slots)
    GET /health         systemctl is-active for each service
    GET /camera/latest  Newest JPEG from /data/doorbell/
    GET /camera/history inference.log tail + clips table from SQLite

Authoritative sources:
    SHM      /dev/shm/sensor_shm    (layout in shm_reader.py)
    SQLite   /home/debian/db/sensors.db
    Images   /data/doorbell/
    Log      /var/log/inference.log

SHM layout is owned by shm_reader.py. Edit offsets there, not here.
"""

import glob
import json
import logging
import os
import sqlite3
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer

from shm_reader import (
    read_shm,
    MAX_REEDS,
    MAX_PIRS,
    MAX_DOORBELL_CAMS,
    CAM_COUNT,
)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

API_HOST       = "127.0.0.1"
API_PORT       = 8080
DB_PATH        = "/home/debian/db/sensors.db"
DOORBELL_DIR   = "/data/doorbell"
INFERENCE_LOG  = "/var/log/inference.log"
LOG_TAIL_LINES = 200
CLIPS_LIMIT    = 50

SERVICES = [
    "data-controller.service",
    "doorbell.service",
    "doorbell_stream.service",
    "inference.service",
    "sensor-server.service",
]

# ---------------------------------------------------------------------------
# SQLite helpers
# ---------------------------------------------------------------------------

def _db_query(sql, params=()):
    try:
        con = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
        con.row_factory = sqlite3.Row
        rows = [dict(r) for r in con.execute(sql, params)]
        con.close()
        return rows
    except sqlite3.Error as e:
        logging.warning("DB query failed: %s", e)
        return []


# ---------------------------------------------------------------------------
# Health check
# ---------------------------------------------------------------------------

def _service_status(unit):
    try:
        r = subprocess.run(
            ["systemctl", "is-active", unit],
            capture_output=True, text=True, timeout=3
        )
        return r.stdout.strip()
    except Exception:
        return "unknown"


# ---------------------------------------------------------------------------
# Camera helpers
# ---------------------------------------------------------------------------

def _latest_jpeg():
    jpegs = glob.glob(os.path.join(DOORBELL_DIR, "*.jpg"))
    if not jpegs:
        return None
    return max(jpegs, key=os.path.getmtime)


def _log_tail(path, n):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            buf = bytearray()
            pos = f.tell()
            lines_found = 0
            while pos > 0 and lines_found < n + 1:
                read_size = min(4096, pos)
                pos -= read_size
                f.seek(pos)
                buf = f.read(read_size) + buf
                lines_found = buf.count(b"\n")
            lines = buf.decode("utf-8", errors="replace").splitlines()
            return lines[-n:] if len(lines) > n else lines
    except OSError:
        return []


# ---------------------------------------------------------------------------
# Request handler
# ---------------------------------------------------------------------------

class APIHandler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        logging.info("HTTP %s", fmt % args)

    def _send_json(self, code, obj):
        body = json.dumps(obj, default=str).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_error(self, code, msg):
        self._send_json(code, {"error": msg})

    def _handle_state(self):
        shm = read_shm()
        if shm is None:
            self._send_error(503, "SHM unavailable or no valid data")
            return
        self._send_json(200, {
            "temp":       shm["current_temp"],
            "motion":     shm["current_motion"],
            "light":      shm["current_light"],
            "lock":       shm["current_lock"],
            "batt_motor": shm["batt_motor"],
            "timestamp":  shm["timestamp"],
            "doorbell": {
                "pressed":   shm["doorbell_pressed"],
                "device_id": shm["doorbell_device_id"],
            },
            "doorbells": [
                {"id": i,
                 "online": bool(shm["doorbell_online"][i]),
                 "age_s":  shm["doorbell_age_s"][i]}
                for i in range(MAX_DOORBELL_CAMS)
            ],
            "cams": [
                {"id": i,
                 "online": bool(shm["cam_online"][i]),
                 "age_s":  shm["cam_age_s"][i]}
                for i in range(CAM_COUNT)
            ],
        })

    def _handle_devices(self):
        shm = read_shm()
        if shm is None:
            self._send_error(503, "SHM unavailable or no valid data")
            return
        hb_names = ["PIR", "LIGHT", "LOCK", "MOTOR", "LCD"]
        self._send_json(200, {
            "heartbeat_devices": [
                {"name": hb_names[i], "online": bool(shm["device_online"][i])}
                for i in range(len(hb_names))
            ],
            "reeds": [
                {"slot":   i + 1,
                 "online": bool(shm["reed_online"][i]),
                 "age_s":  shm["reed_age_s"][i]}
                for i in range(MAX_REEDS)
            ],
            "pirs": [
                {"slot":   i + 1,
                 "online": bool(shm["pir_online"][i]),
                 "age_s":  shm["pir_age_s"][i]}
                for i in range(MAX_PIRS)
            ],
        })

    def _handle_health(self):
        statuses = {svc: _service_status(svc) for svc in SERVICES}
        all_active = all(v == "active" for v in statuses.values())
        self._send_json(200 if all_active else 207, {
            "healthy":  all_active,
            "services": statuses,
        })

    def _handle_camera_latest(self):
        path = _latest_jpeg()
        if path is None:
            self._send_error(404, "no doorbell images available")
            return
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            self._send_error(404, "image not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _handle_camera_history(self):
        log_lines = _log_tail(INFERENCE_LOG, LOG_TAIL_LINES)
        clips = _db_query(
            "SELECT ts, camera, clip_path, frame_count, person, confidence, duration_ms "
            "FROM clips ORDER BY ts DESC LIMIT ?",
            (CLIPS_LIMIT,)
        )
        self._send_json(200, {"log": log_lines, "clips": clips})

    def do_GET(self):
        routes = {
            "/state":          self._handle_state,
            "/devices":        self._handle_devices,
            "/health":         self._handle_health,
            "/camera/latest":  self._handle_camera_latest,
            "/camera/history": self._handle_camera_history,
        }
        handler = routes.get(self.path)
        if handler is None:
            self._send_error(404, f"unknown endpoint: {self.path}")
            return
        try:
            handler()
        except Exception as e:
            logging.exception("handler error: %s", e)
            self._send_error(500, "internal error")

    def do_POST(self):
        self._send_error(501, "control endpoints not yet implemented — "
                              "no BBB→ESP32 command path exists")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    server = HTTPServer((API_HOST, API_PORT), APIHandler)
    logging.info("API server listening on %s:%d", API_HOST, API_PORT)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    server.server_close()


if __name__ == "__main__":
    main()
