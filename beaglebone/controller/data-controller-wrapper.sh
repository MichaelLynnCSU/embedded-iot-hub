#!/bin/bash
/home/debian/controller/data_controller &
pid=$!
# Wait for data_controller to initialize
sleep 2
# Notify systemd we're ready
systemd-notify --ready 2>/dev/null || true
# Wait for process to exit
wait $pid
