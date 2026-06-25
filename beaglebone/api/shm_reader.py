"""
shm_reader.py — SharedSensorData reader for BeagleBone API.

Owns all SHM offset constants and the _read_shm() function.
Offsets confirmed via ctypes.offsetof() on the target (ARM Linux,
glibc, 32-bit long). SHM size = 3448 bytes.

When shared_data.h changes:
  1. Add/remove fields in the offset block below.
  2. Re-run the offset script on the BeagleBone to confirm values:
       python3 -c "
       import ctypes
       # ... (see tools/shm_offsets.py)
       "
  3. Update _SHM_EXPECTED_SIZE.

Do not edit api_server.py for layout changes — edit here only.
"""

import logging
import mmap
import os
import struct as _struct

# ---------------------------------------------------------------------------
# SHM path
# ---------------------------------------------------------------------------

SHM_PATH = "/dev/shm/sensor_shm"

# Expected SHM size — checked at startup to catch layout drift.
_SHM_EXPECTED_SIZE = 3448

# ---------------------------------------------------------------------------
# Slot counts — must match config.h on the BeagleBone
# ---------------------------------------------------------------------------

MAX_REEDS         = 6
MAX_PIRS          = 5
MAX_DOORBELL_CAMS = 4
CAM_COUNT         = 3
DEVICE_COUNT      = 9   # heartbeat devices (PIR/LIGHT/LOCK/MOTOR/LCD + 4 unused)

# ---------------------------------------------------------------------------
# Head offsets — derived from struct layout, confirmed by offsetof()
#
#   offset  0  : pthread_mutex_t shm_mutex   (24 bytes, ARM glibc)
#   offset 24  : uint8_t device_online[9]    (9 bytes)
#   offset 33  : <7 bytes pad to align double>
#   offset 40  : double  current_temp
#   offset 48  : int     current_motion
#   offset 52  : int     current_light
#   offset 56  : int     current_lock
#   offset 60  : int     batt_motor
#   offset 64  : long    current_timestamp   (4 bytes, 32-bit long)
#   offset 68  : int     data_valid
# ---------------------------------------------------------------------------

_OFF_DEVICE_ONLINE  = 24
_OFF_CURRENT_TEMP   = 40
_OFF_CURRENT_MOTION = 48
_OFF_CURRENT_LIGHT  = 52
_OFF_CURRENT_LOCK   = 56
_OFF_BATT_MOTOR     = 60
_OFF_TIMESTAMP      = 64
_OFF_DATA_VALID     = 68

# ---------------------------------------------------------------------------
# Tail offsets — confirmed by ctypes.offsetof() on target, SHM size 3448
# ---------------------------------------------------------------------------

_OFF_DB_PRESSED     = 3372
_OFF_DB_DEVICE_ID   = 3373
_OFF_DB_TIMESTAMP   = 3376
_OFF_DB_AGE         = 3380   # uint16[4]
_OFF_DB_ONLINE      = 3388   # uint8[4]
_OFF_CAM_AGE        = 3392   # uint16[3]
_OFF_CAM_ONLINE     = 3398   # uint8[3]
_OFF_REED_AGE       = 3402   # uint16[6]
_OFF_REED_ONLINE    = 3414   # uint8[6]
_OFF_PIR_AGE        = 3420   # uint16[5]
_OFF_PIR_ONLINE     = 3430   # uint8[5]
# event_id           = 3440   # uint64 — always 0, not read


# ---------------------------------------------------------------------------
# Reader
# ---------------------------------------------------------------------------

def read_shm():
    """
    Open SHM read-only and return a dict of sensor fields.
    Returns None if SHM is unavailable or data_valid == 0.

    Lockless read — acceptable torn-read risk for observability use.
    """
    try:
        fd = os.open(SHM_PATH, os.O_RDONLY)
    except OSError as e:
        logging.warning("SHM open failed: %s", e)
        return None

    try:
        mm = mmap.mmap(fd, 0, access=mmap.ACCESS_READ)
    except OSError as e:
        os.close(fd)
        logging.warning("SHM mmap failed: %s", e)
        return None
    finally:
        os.close(fd)

    try:
        if mm.size() != _SHM_EXPECTED_SIZE:
            logging.error(
                "SHM size mismatch: got %d expected %d — "
                "recompile controller and update shm_reader.py",
                mm.size(), _SHM_EXPECTED_SIZE
            )
            return None

        def u8(off):  return _struct.unpack_from("B", mm, off)[0]
        def i32(off): return _struct.unpack_from("i", mm, off)[0]
        def u16(off): return _struct.unpack_from("H", mm, off)[0]
        def dbl(off): return _struct.unpack_from("d", mm, off)[0]
        def lng(off): return _struct.unpack_from("l", mm, off)[0]

        data_valid = i32(_OFF_DATA_VALID)
        if not data_valid:
            return None

        return {
            "data_valid":         data_valid,
            "timestamp":          lng(_OFF_TIMESTAMP),
            "current_temp":       dbl(_OFF_CURRENT_TEMP),
            "current_motion":     i32(_OFF_CURRENT_MOTION),
            "current_light":      i32(_OFF_CURRENT_LIGHT),
            "current_lock":       i32(_OFF_CURRENT_LOCK),
            "batt_motor":         i32(_OFF_BATT_MOTOR),
            "device_online":      [u8(_OFF_DEVICE_ONLINE + i)  for i in range(DEVICE_COUNT)],
            "doorbell_pressed":   u8(_OFF_DB_PRESSED),
            "doorbell_device_id": u8(_OFF_DB_DEVICE_ID),
            "doorbell_age_s":     [u16(_OFF_DB_AGE    + i*2)   for i in range(MAX_DOORBELL_CAMS)],
            "doorbell_online":    [u8(_OFF_DB_ONLINE   + i)    for i in range(MAX_DOORBELL_CAMS)],
            "cam_age_s":          [u16(_OFF_CAM_AGE    + i*2)  for i in range(CAM_COUNT)],
            "cam_online":         [u8(_OFF_CAM_ONLINE   + i)   for i in range(CAM_COUNT)],
            "reed_age_s":         [u16(_OFF_REED_AGE   + i*2)  for i in range(MAX_REEDS)],
            "reed_online":        [u8(_OFF_REED_ONLINE  + i)   for i in range(MAX_REEDS)],
            "pir_age_s":          [u16(_OFF_PIR_AGE    + i*2)  for i in range(MAX_PIRS)],
            "pir_online":         [u8(_OFF_PIR_ONLINE   + i)   for i in range(MAX_PIRS)],
        }

    except Exception as e:
        logging.warning("SHM read failed: %s", e)
        return None
    finally:
        mm.close()
