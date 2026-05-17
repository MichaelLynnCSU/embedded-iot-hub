#!/bin/bash
# Wrapper for lcd_display_db binary to signal systemd readiness

BINARY="/home/debian/lcd/lcd_display_db"

# Start the binary in the background
"$BINARY" &
PID=$!

# Optional: wait a bit for initialization (adjust if needed)
sleep 2

# Notify systemd that the service is ready
systemd-notify --ready
systemd-notify "STATUS=LCD display running"

# Wait for the binary to exit
wait $PID

# If binary exits, systemd sees the service has stopped
exit $?
