#!/bin/bash
#==============================================================================
# test_db.sh
# Smart Home Sensor Database — insert and verify all logging paths.
# Run from beaglebone/db/:
#   chmod +x test_db.sh && ./test_db.sh
#==============================================================================

DB="./test.db"
PASS=0
FAIL=0
TS=$(date +%s)

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
NC='\033[0m'

#------------------------------------------------------------------------------
check()
{
   local desc="$1"
   local result="$2"
   local expected="$3"

   if [ "$result" = "$expected" ]; then
      echo -e "  ${GRN}PASS${NC} $desc"
      PASS=$((PASS + 1))
   else
      echo -e "  ${RED}FAIL${NC} $desc"
      echo -e "       expected: ${YLW}$expected${NC}"
      echo -e "       got:      ${YLW}$result${NC}"
      FAIL=$((FAIL + 1))
   fi
}

#------------------------------------------------------------------------------
echo ""
echo "=============================================="
echo " Smart Home DB Test"
echo " DB:  $DB"
echo " TS:  $TS"
echo "=============================================="

#------------------------------------------------------------------------------
echo ""
echo "[ readings ] — full sensor frame"

sqlite3 "$DB" "
INSERT INTO readings
  (ts, source, avg_temp, motion_count, light_state, lock_state,
   motor_online, batt_pir, batt_lck, batt_motor)
VALUES
  ($TS, 'vroom', 22.5, 3, 1, 0, 1, 87, 63, 79);
"

result=$(sqlite3 "$DB" "SELECT avg_temp FROM readings WHERE ts=$TS;")
check "avg_temp saved" "$result" "22.5"

result=$(sqlite3 "$DB" "SELECT motion_count FROM readings WHERE ts=$TS;")
check "motion_count saved" "$result" "3"

result=$(sqlite3 "$DB" "SELECT light_state FROM readings WHERE ts=$TS;")
check "light_state saved" "$result" "1"

result=$(sqlite3 "$DB" "SELECT lock_state FROM readings WHERE ts=$TS;")
check "lock_state saved" "$result" "0"

result=$(sqlite3 "$DB" "SELECT motor_online FROM readings WHERE ts=$TS;")
check "motor_online saved" "$result" "1"

result=$(sqlite3 "$DB" "SELECT batt_pir FROM readings WHERE ts=$TS;")
check "batt_pir saved" "$result" "87"

result=$(sqlite3 "$DB" "SELECT batt_lck FROM readings WHERE ts=$TS;")
check "batt_lck saved" "$result" "63"

result=$(sqlite3 "$DB" "SELECT batt_motor FROM readings WHERE ts=$TS;")
check "batt_motor saved" "$result" "79"

# NULL battery path
sqlite3 "$DB" "
INSERT INTO readings (ts, source, avg_temp, motion_count)
VALUES ($((TS+1)), 'vroom', 21.0, 0);
"
result=$(sqlite3 "$DB" "SELECT batt_pir FROM readings WHERE ts=$((TS+1));")
check "batt_pir NULL when unknown" "$result" ""

#------------------------------------------------------------------------------
echo ""
echo "[ uart_readings ] — PIR / LGT / LCK inbound frames"

sqlite3 "$DB" "
INSERT INTO uart_readings (ts, device, value, batt) VALUES ($TS, 'PIR', 5, 87);
INSERT INTO uart_readings (ts, device, value, batt) VALUES ($TS, 'LGT', 1, NULL);
INSERT INTO uart_readings (ts, device, value, batt) VALUES ($TS, 'LCK', 0, 63);
"

result=$(sqlite3 "$DB" "SELECT value FROM uart_readings WHERE ts=$TS AND device='PIR';")
check "PIR value saved" "$result" "5"

result=$(sqlite3 "$DB" "SELECT batt FROM uart_readings WHERE ts=$TS AND device='PIR';")
check "PIR battery saved" "$result" "87"

result=$(sqlite3 "$DB" "SELECT value FROM uart_readings WHERE ts=$TS AND device='LGT';")
check "LGT value saved" "$result" "1"

result=$(sqlite3 "$DB" "SELECT batt FROM uart_readings WHERE ts=$TS AND device='LGT';")
check "LGT battery NULL" "$result" ""

result=$(sqlite3 "$DB" "SELECT value FROM uart_readings WHERE ts=$TS AND device='LCK';")
check "LCK value saved" "$result" "0"

result=$(sqlite3 "$DB" "SELECT batt FROM uart_readings WHERE ts=$TS AND device='LCK';")
check "LCK battery saved" "$result" "63"

#------------------------------------------------------------------------------
echo ""
echo "[ reed_readings ] — door sensor slots"

sqlite3 "$DB" "
INSERT INTO reed_readings (ts, slot, name, state, batt, age)
VALUES ($TS, 1, 'ReedSensor1', 0, 91, 12);
INSERT INTO reed_readings (ts, slot, name, state, batt, age)
VALUES ($TS, 2, 'ReedSensor2', 1, 74, 305);
INSERT INTO reed_readings (ts, slot, name, state, batt, age)
VALUES ($TS, 3, NULL, 0, NULL, 999);
"

result=$(sqlite3 "$DB" "SELECT state FROM reed_readings WHERE ts=$TS AND slot=1;")
check "REED1 state=closed(0)" "$result" "0"

result=$(sqlite3 "$DB" "SELECT batt FROM reed_readings WHERE ts=$TS AND slot=1;")
check "REED1 battery saved" "$result" "91"

result=$(sqlite3 "$DB" "SELECT state FROM reed_readings WHERE ts=$TS AND slot=2;")
check "REED2 state=open(1)" "$result" "1"

result=$(sqlite3 "$DB" "SELECT age FROM reed_readings WHERE ts=$TS AND slot=2;")
check "REED2 age saved" "$result" "305"

result=$(sqlite3 "$DB" "SELECT name FROM reed_readings WHERE ts=$TS AND slot=3;")
check "REED3 name NULL when unknown" "$result" ""

result=$(sqlite3 "$DB" "SELECT batt FROM reed_readings WHERE ts=$TS AND slot=3;")
check "REED3 batt NULL when unknown" "$result" ""

#------------------------------------------------------------------------------
echo ""
echo "[ motor_readings ] — motor controller snapshots"

sqlite3 "$DB" "
INSERT INTO motor_readings (ts, online, batt) VALUES ($TS,     1, 79);
INSERT INTO motor_readings (ts, online, batt) VALUES ($((TS+2)), 0, NULL);
"

result=$(sqlite3 "$DB" "SELECT online FROM motor_readings WHERE ts=$TS;")
check "motor online=1 saved" "$result" "1"

result=$(sqlite3 "$DB" "SELECT batt FROM motor_readings WHERE ts=$TS;")
check "motor battery saved" "$result" "79"

result=$(sqlite3 "$DB" "SELECT online FROM motor_readings WHERE ts=$((TS+2));")
check "motor online=0 saved" "$result" "0"

result=$(sqlite3 "$DB" "SELECT batt FROM motor_readings WHERE ts=$((TS+2));")
check "motor battery NULL when unknown" "$result" ""

#------------------------------------------------------------------------------
echo ""
echo "[ device_events ] — online/offline transitions"

sqlite3 "$DB" "
INSERT INTO device_events (ts, device, event) VALUES ($TS,       'MOTOR', 'online');
INSERT INTO device_events (ts, device, event) VALUES ($((TS+10)), 'MOTOR', 'offline');
INSERT INTO device_events (ts, device, event) VALUES ($TS,       'REED1', 'online');
INSERT INTO device_events (ts, device, event) VALUES ($((TS+20)), 'REED2', 'gen 1->2 (device replaced)');
INSERT INTO device_events (ts, device, event) VALUES ($TS,       'PIR',   'offline');
"

result=$(sqlite3 "$DB" "SELECT event FROM device_events WHERE ts=$TS AND device='MOTOR';")
check "MOTOR online event saved" "$result" "online"

result=$(sqlite3 "$DB" "SELECT event FROM device_events WHERE ts=$((TS+10)) AND device='MOTOR';")
check "MOTOR offline event saved" "$result" "offline"

result=$(sqlite3 "$DB" "SELECT event FROM device_events WHERE ts=$((TS+20)) AND device='REED2';")
check "REED2 generation change saved" "$result" "gen 1->2 (device replaced)"

result=$(sqlite3 "$DB" "SELECT COUNT(*) FROM device_events WHERE ts=$TS;")
check "3 events at same timestamp" "$result" "3"

#------------------------------------------------------------------------------
echo ""
echo "[ room_sensors ] — room state from ESP32"

sqlite3 "$DB" "
INSERT INTO room_sensors (ts, room_name, sensor_id, state, location, batt)
VALUES ($TS, 'Living Room', 1, 'closed', 'front', 88);
INSERT INTO room_sensors (ts, room_name, sensor_id, state, location, batt)
VALUES ($TS, 'Garage',      2, 'open',   'side',  NULL);
"

result=$(sqlite3 "$DB" "SELECT state FROM room_sensors WHERE ts=$TS AND sensor_id=1;")
check "room 1 state=closed saved" "$result" "closed"

result=$(sqlite3 "$DB" "SELECT batt FROM room_sensors WHERE ts=$TS AND sensor_id=1;")
check "room 1 battery saved" "$result" "88"

result=$(sqlite3 "$DB" "SELECT state FROM room_sensors WHERE ts=$TS AND sensor_id=2;")
check "room 2 state=open saved" "$result" "open"

result=$(sqlite3 "$DB" "SELECT batt FROM room_sensors WHERE ts=$TS AND sensor_id=2;")
check "room 2 battery NULL" "$result" ""

# CHECK constraint test
sqlite3 "$DB" "
INSERT INTO room_sensors (ts, room_name, sensor_id, state, location)
VALUES ($TS, 'Test', 99, 'ajar', 'nowhere');
" 2>/dev/null
result=$(sqlite3 "$DB" "SELECT COUNT(*) FROM room_sensors WHERE sensor_id=99;")
check "invalid state rejected by CHECK constraint" "$result" "0"

#------------------------------------------------------------------------------
echo ""
echo "[ indexes ] — verify indexes exist"

for idx in idx_readings_ts idx_uart_readings_dev idx_reed_readings_slot \
           idx_motor_readings_ts idx_device_events_dev idx_room_sensors_id; do
   result=$(sqlite3 "$DB" "SELECT name FROM sqlite_master WHERE type='index' AND name='$idx';")
   check "index $idx exists" "$result" "$idx"
done

#------------------------------------------------------------------------------
echo ""
echo "[ latest room query ] — Repository pattern query"

sqlite3 "$DB" "
INSERT INTO room_sensors (ts, room_name, sensor_id, state, location)
VALUES ($((TS+100)), 'Living Room', 1, 'open', 'front');
"

result=$(sqlite3 "$DB" "
SELECT state FROM room_sensors
WHERE id IN (SELECT MAX(id) FROM room_sensors GROUP BY sensor_id)
AND sensor_id=1;
")
check "latest room state returns newest row" "$result" "open"

#------------------------------------------------------------------------------
echo ""
echo "=============================================="
echo -e " Results: ${GRN}$PASS passed${NC}  ${RED}$FAIL failed${NC}"
echo "=============================================="
echo ""

# Cleanup test data
sqlite3 "$DB" "
DELETE FROM readings      WHERE ts >= $TS;
DELETE FROM uart_readings WHERE ts >= $TS;
DELETE FROM reed_readings WHERE ts >= $TS;
DELETE FROM motor_readings WHERE ts >= $TS;
DELETE FROM device_events WHERE ts >= $TS;
DELETE FROM room_sensors  WHERE ts >= $TS OR sensor_id=99;
"

exit $FAIL
