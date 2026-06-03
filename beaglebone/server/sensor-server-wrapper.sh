#!/bin/bash
PIPE="/tmp/sensor_pipe"
echo "[INFO] Waiting for controller pipe..."
while [ ! -p "$PIPE" ]; do
    sleep 1
done
echo "[INFO] Controller pipe found, starting sensor_server"
exec /home/debian/server/sensor_server
