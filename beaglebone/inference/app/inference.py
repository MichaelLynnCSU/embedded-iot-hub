#!/usr/bin/env python3
"""
inference.py — PIR-triggered person detection on BeagleBone.

Watches current_motion in shared memory. On increment, requests a JPEG
from the ESP32-CAM, runs TFLite person detection, saves result to /data/
and uploads to S3 via curl.
"""

import ctypes
import mmap
import os
import time
import socket
import struct
import logging
import subprocess
from datetime import datetime

# ── Paths ─────────────────────────────────────────────────────────────────────
MODEL_PATH   = "/opt/inference/models/detect.tflite"
LABEL_PATH   = "/opt/inference/models/labelmap.txt"
PENDING_DIR  = "/data/pending"
UPLOADED_DIR = "/data/uploaded"
FAILED_DIR   = "/data/failed"
LOG_PATH     = "/var/log/inference.log"

# ── ESP32-CAM TCP ──────────────────────────────────────────────────────────────
CAM_HOST     = "192.168.1.100"   # update to actual ESP32-CAM IP
CAM_PORT     = 8080
CAM_TIMEOUT  = 5.0

# ── S3 upload ─────────────────────────────────────────────────────────────────
S3_BUCKET    = "your-bucket-name"   # update
S3_REGION    = "us-east-1"          # update
AWS_PROFILE  = "default"

# ── Detection ─────────────────────────────────────────────────────────────────
PERSON_LABEL       = "person"
CONFIDENCE_THRESH  = 0.5
POLL_INTERVAL_S    = 0.1
DEBOUNCE_S         = 2.0

# ── Shared memory layout (must match SharedSensorData in shared_data.h) ───────
SHM_NAME           = "/sensor_shm"
CURRENT_MOTION_OFF = None   # resolved at runtime by scanning struct

logging.basicConfig(
    filename=LOG_PATH,
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s"
)
log = logging.getLogger(__name__)


# ── Shared memory reader ───────────────────────────────────────────────────────

def open_shm():
    """Map /sensor_shm read-only. Returns mmap object."""
    fd = os.open(f"/dev/shm{SHM_NAME}", os.O_RDONLY)
    mm = mmap.mmap(fd, 0, access=mmap.ACCESS_READ)
    os.close(fd)
    return mm


def read_current_motion(mm):
    """
    Read current_motion from shared memory.
    SharedSensorData layout (see shared_data.h):
      pthread_mutex_t shm_mutex        — 40 bytes on ARM Linux
      uint8_t device_online[6]         — 6 bytes
      padding to align double          — 2 bytes
      double current_temp              — 8 bytes
      int current_motion               — 4 bytes  ← offset 60
    """
    MOTION_OFFSET = 56
    mm.seek(MOTION_OFFSET)
    return struct.unpack_from('i', mm.read(4))[0]


# ── ESP32-CAM JPEG fetch ───────────────────────────────────────────────────────

def fetch_jpeg():
    """
    Request a JPEG frame from ESP32-CAM over TCP.
    Protocol: send 'CAPTURE\n', read 4-byte big-endian length, read JPEG bytes.
    Returns bytes or None on failure.
    """
    try:
        with socket.create_connection((CAM_HOST, CAM_PORT),
                                      timeout=CAM_TIMEOUT) as s:
            s.sendall(b'CAPTURE\n')
            raw_len = s.recv(4)
            if len(raw_len) < 4:
                log.error("CAM: short length header")
                return None
            jpeg_len = struct.unpack('>I', raw_len)[0]
            if jpeg_len == 0 or jpeg_len > 500_000:
                log.error("CAM: bad jpeg_len %d", jpeg_len)
                return None
            data = b''
            while len(data) < jpeg_len:
                chunk = s.recv(jpeg_len - len(data))
                if not chunk:
                    break
                data += chunk
            if len(data) != jpeg_len:
                log.error("CAM: incomplete JPEG %d/%d", len(data), jpeg_len)
                return None
            return data
    except Exception as e:
        log.error("CAM fetch failed: %s", e)
        return None


# ── TFLite inference ───────────────────────────────────────────────────────────

def load_labels():
    with open(LABEL_PATH) as f:
        return [l.strip() for l in f.readlines()]


def run_inference(jpeg_bytes, interpreter, labels, input_details, output_details):
    """
    Run person detection on jpeg_bytes.
    Returns (person_detected: bool, confidence: float).
    """
    import numpy as np
    from PIL import Image
    import io

    img = Image.open(io.BytesIO(jpeg_bytes)).convert('RGB')
    h   = input_details[0]['shape'][1]
    w   = input_details[0]['shape'][2]
    img = img.resize((w, h))
    inp = np.expand_dims(np.array(img, dtype=np.uint8), axis=0)

    interpreter.set_tensor(input_details[0]['index'], inp)
    interpreter.invoke()

    boxes   = interpreter.get_tensor(output_details[0]['index'])[0]
    classes = interpreter.get_tensor(output_details[1]['index'])[0]
    scores  = interpreter.get_tensor(output_details[2]['index'])[0]

    for i, score in enumerate(scores):
        if score < CONFIDENCE_THRESH:
            break
        label = labels[int(classes[i]) + 1] if int(classes[i]) + 1 < len(labels) else ""
        if label == PERSON_LABEL:
            return True, float(score)

    return False, 0.0


# ── Image storage ──────────────────────────────────────────────────────────────

def save_jpeg(jpeg_bytes, detected, confidence):
    """Save JPEG to /data/pending, return filepath."""
    ts    = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    label = "person" if detected else "noperson"
    fname = f"{ts}_{label}_{int(confidence*100):03d}.jpg"
    path  = os.path.join(PENDING_DIR, fname)
    with open(path, 'wb') as f:
        f.write(jpeg_bytes)
    log.info("Saved %s", path)
    return path


def upload_to_s3(path):
    """Upload file to S3 via AWS CLI. Move to uploaded/ or failed/."""
    key  = f"inference/{os.path.basename(path)}"
    cmd  = ["aws", "s3", "cp", path, f"s3://{S3_BUCKET}/{key}",
            "--region", S3_REGION]
    try:
        subprocess.run(cmd, check=True, timeout=30,
                       capture_output=True)
        dest = os.path.join(UPLOADED_DIR, os.path.basename(path))
        os.rename(path, dest)
        log.info("Uploaded %s", key)
    except Exception as e:
        dest = os.path.join(FAILED_DIR, os.path.basename(path))
        os.rename(path, dest)
        log.error("Upload failed for %s: %s", path, e)


# ── Disk watermark cleanup ────────────────────────────────────────────────────

def enforce_disk_limit(directory, max_files=200):
    """Delete oldest files if directory exceeds max_files."""
    files = sorted(
        [os.path.join(directory, f) for f in os.listdir(directory)],
        key=os.path.getmtime
    )
    while len(files) > max_files:
        os.remove(files.pop(0))
        log.warning("Disk limit: purged oldest file in %s", directory)


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    for d in (PENDING_DIR, UPLOADED_DIR, FAILED_DIR):
        os.makedirs(d, exist_ok=True)

    log.info("Loading TFLite model: %s", MODEL_PATH)
    from tflite_runtime.interpreter import Interpreter
    interpreter    = Interpreter(model_path=MODEL_PATH)
    interpreter.allocate_tensors()
    input_details  = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    labels         = load_labels()
    log.info("Model loaded. Input shape: %s", input_details[0]['shape'])

    log.info("Opening shared memory: %s", SHM_NAME)
    mm = open_shm()

    last_motion   = read_current_motion(mm)
    last_trigger  = 0.0
    log.info("Inference ready. Watching current_motion (last=%d)", last_motion)

    while True:
        time.sleep(POLL_INTERVAL_S)

        cur = read_current_motion(mm)
        if cur == last_motion:
            continue

        last_motion = cur
        now = time.monotonic()

        if (now - last_trigger) < DEBOUNCE_S:
            log.debug("Debounce skip")
            continue

        last_trigger = now
        log.info("Motion event — fetching JPEG")

        jpeg = fetch_jpeg()
        if jpeg is None:
            continue

        detected, confidence = run_inference(
            jpeg, interpreter, labels, input_details, output_details
        )
        log.info("Result: person=%s confidence=%.2f", detected, confidence)

        path = save_jpeg(jpeg, detected, confidence)
        enforce_disk_limit(PENDING_DIR)
        upload_to_s3(path)


if __name__ == "__main__":
    main()
