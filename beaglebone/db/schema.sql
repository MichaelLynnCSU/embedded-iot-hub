-- =============================================================================
-- schema.sql
-- Smart Home Sensor Database — BeagleBone Data Controller
-- Reference schema. Run migrate.sql on existing DB, not this file.
-- =============================================================================

-- -----------------------------------------------------------------------------
-- readings
-- One row per sensor frame from the ESP32/sensor server.
-- Saves every SENSOR_SAVE_INTERVAL seconds (not every frame).
-- -----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS readings (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    ts           INTEGER NOT NULL,
    source       TEXT    NOT NULL,
    avg_temp     REAL    DEFAULT 0,
    motion_count INTEGER DEFAULT 0,
    light_state  INTEGER DEFAULT 0,   -- 0=off  1=on
    lock_state   INTEGER DEFAULT 0,   -- 0=unlocked 1=locked
    motor_online INTEGER DEFAULT 0,   -- 0=offline 1=online
    batt_pir     INTEGER,             -- PIR battery SOC % (NULL if unknown)
    batt_lck     INTEGER,             -- lock battery SOC % (NULL if unknown)
    batt_motor   INTEGER              -- motor battery SOC % (NULL if unknown)
);

-- -----------------------------------------------------------------------------
-- uart_readings
-- One row per inbound UART frame from STM32 (PIR, LGT, LCK).
-- -----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS uart_readings (
    id     INTEGER PRIMARY KEY AUTOINCREMENT,
    ts     INTEGER NOT NULL,
    device TEXT    NOT NULL,          -- PIR | LGT | LCK
    value  INTEGER NOT NULL,          -- sensor value
    batt   INTEGER                    -- battery SOC % (NULL if not reported)
);

-- -----------------------------------------------------------------------------
-- reed_readings
-- One row per reed slot snapshot pushed to STM32.
-- Slot index matches DR1..DRn naming on STM32 UI.
-- -----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS reed_readings (
    id     INTEGER PRIMARY KEY AUTOINCREMENT,
    ts     INTEGER NOT NULL,
    slot   INTEGER NOT NULL,          -- 1-based slot index (DR1=1, DR2=2 ...)
    name   TEXT,                      -- device name from ESP32 JSON
    state  INTEGER NOT NULL,          -- 0=closed 1=open
    batt   INTEGER,                   -- battery SOC % (NULL if unknown)
    age    INTEGER                    -- seconds since last update
);

-- -----------------------------------------------------------------------------
-- motor_readings
-- One row per motor controller state snapshot.
-- -----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS motor_readings (
    id     INTEGER PRIMARY KEY AUTOINCREMENT,
    ts     INTEGER NOT NULL,
    online INTEGER NOT NULL,          -- 0=offline 1=online
    batt   INTEGER                    -- battery SOC % (NULL if unknown)
);

-- -----------------------------------------------------------------------------
-- device_events
-- Online/offline transitions and generation changes for all devices.
-- -----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS device_events (
    id     INTEGER PRIMARY KEY AUTOINCREMENT,
    ts     INTEGER NOT NULL,
    device TEXT    NOT NULL,          -- PIR | LGT | LCK | REED1..N | MOTOR
    event  TEXT    NOT NULL           -- online | offline | gen N->M (replaced)
);

-- -----------------------------------------------------------------------------
-- room_sensors
-- Latest state per room from ESP32 JSON room array.
-- -----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS room_sensors (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    ts        INTEGER NOT NULL,
    room_name TEXT    NOT NULL,
    sensor_id INTEGER NOT NULL,
    state     TEXT    NOT NULL CHECK(state IN ('open', 'closed')),
    location  TEXT,
    batt      INTEGER                 -- battery SOC % (NULL if not reported)
);

-- -----------------------------------------------------------------------------
-- Indexes for common query patterns
-- -----------------------------------------------------------------------------
CREATE INDEX IF NOT EXISTS idx_readings_ts        ON readings(ts);
CREATE INDEX IF NOT EXISTS idx_uart_readings_ts   ON uart_readings(ts);
CREATE INDEX IF NOT EXISTS idx_uart_readings_dev  ON uart_readings(device, ts);
CREATE INDEX IF NOT EXISTS idx_reed_readings_ts   ON reed_readings(ts);
CREATE INDEX IF NOT EXISTS idx_reed_readings_slot ON reed_readings(slot, ts);
CREATE INDEX IF NOT EXISTS idx_motor_readings_ts  ON motor_readings(ts);
CREATE INDEX IF NOT EXISTS idx_device_events_ts   ON device_events(ts);
CREATE INDEX IF NOT EXISTS idx_device_events_dev  ON device_events(device, ts);
CREATE INDEX IF NOT EXISTS idx_room_sensors_id    ON room_sensors(sensor_id, ts);
