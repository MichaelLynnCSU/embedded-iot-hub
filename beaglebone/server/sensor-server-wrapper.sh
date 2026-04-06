#!/bin/bash
BINARY="/home/debian/server/sensor_server"
PIPE="/tmp/sensor_pipe"

echo "[INFO] Waiting for controller pipe..."
while [ ! -p "$PIPE" ]; do
    sleep 1
done
echo "[INFO] Controller pipe found, starting sensor_server"

"$BINARY" &
PID=$!

sleep 2
systemd-notify --ready
systemd-notify "STATUS=Sensor server running"

wait $PID
exit $?

