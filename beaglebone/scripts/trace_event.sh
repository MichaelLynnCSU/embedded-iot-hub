#!/bin/bash
# trace_event.sh — run on the BeagleBone only.
# Traces a single event_id or frame_seq across every log domain confirmed
# during the event_id/eid/frame_seq audit.
#
# Usage:
#   ./trace_event.sh event <decimal_event_id>
#   ./trace_event.sh seq   <frame_seq>
#
# event mode converts the decimal id to the 16-hex-digit EVENT_ID_FMT form
# automatically and greps both forms across the right log files.
# seq mode greps frame_seq as-is across the server telemetry/event logs.

set -euo pipefail

MODE="${1:-}"
VAL="${2:-}"

if [[ -z "$MODE" || -z "$VAL" ]]; then
    echo "Usage: $0 event <decimal_event_id>"
    echo "       $0 seq   <frame_seq>"
    exit 1
fi

LOGS_CONTROLLER="/var/log/data_controller.log"
LOGS_CAMERA="/var/log/camera_manager.log"
LOGS_INFERENCE="/var/log/inference.log"
LOGS_DOORBELL="/var/log/doorbell_daemon.log"
LOGS_SERVER="/var/log/sensor_server.log"
LOGS_TELEMETRY="/var/log/telemetry.log"
LOGS_EVENTS="/var/log/events.log"

hr() { printf '%s\n' "------------------------------------------------------------"; }

if [[ "$MODE" == "event" ]]; then
    DEC="$VAL"
    HEX=$(printf '%016x\n' "$DEC")

    echo "Tracing event_id=$DEC  (hex form: $HEX)"
    hr

    echo "[1/4] camera_manager.log (decimal event_id, cam_tx_id, boot_epoch)"
    grep -n "event_id=${DEC}\b" "$LOGS_CAMERA" 2>/dev/null || echo "  (no match)"
    hr

    echo "[2/4] data_controller.log (decimal event_id — DISPATCH/SHM/UART, unified naming)"
    grep -n "event_id=${DEC}\b" "$LOGS_CONTROLLER" 2>/dev/null || echo "  (no match)"
    echo "  -- checking for stale 'eid=' usage (should be none post-rename) --"
    grep -n "eid=${DEC}\b" "$LOGS_CONTROLLER" 2>/dev/null && echo "  WARNING: found legacy eid= usage above" || echo "  (none — good)"
    hr

    echo "[3/4] inference.log (hex event_id via EVENT_ID_FMT)"
    grep -n "event_id=${HEX}\b" "$LOGS_INFERENCE" 2>/dev/null || echo "  (no match)"
    hr

    echo "[4/4] doorbell_daemon.log (hex event_id via EVENT_ID_FMT, if applicable)"
    grep -n "event_id=${HEX}\b" "$LOGS_DOORBELL" 2>/dev/null || echo "  (no match)"
    hr

    echo "Summary line counts:"
    for f in "$LOGS_CAMERA" "$LOGS_CONTROLLER" "$LOGS_INFERENCE" "$LOGS_DOORBELL"; do
        [[ -f "$f" ]] || continue
        c_dec=$(grep -c "event_id=${DEC}\b" "$f" 2>/dev/null || echo 0)
        c_hex=$(grep -c "event_id=${HEX}\b" "$f" 2>/dev/null || echo 0)
        printf "  %-28s dec=%-4s hex=%-4s\n" "$(basename "$f")" "$c_dec" "$c_hex"
    done
    hr

    echo "Bridged frame_seq check (indoor cam path only, as of 2026-07-04 — see README):"
    echo "  a line with both event_id and seq= confirms trigger-time bridging worked."
    grep -n "event_id=${DEC}\b.*seq=\|seq=.*event_id=${DEC}\b" "$LOGS_CAMERA" 2>/dev/null || echo "  (no bridged seq= found in camera_manager.log — expected on pre-bridge binaries or doorbell events)"
    grep -n "event_id=${HEX}\b.*seq=\|seq=.*event_id=${HEX}\b" "$LOGS_INFERENCE" 2>/dev/null || echo "  (no bridged seq= found in inference.log — expected on pre-bridge binaries or doorbell events)"

elif [[ "$MODE" == "seq" ]]; then
    SEQ="$VAL"
    echo "Tracing frame_seq=$SEQ  (server domain join key — separate from event_id)"
    hr

    echo "[1/2] events.log (low-volume, true state transitions only)"
    grep -n "seq=${SEQ}\b" "$LOGS_EVENTS" 2>/dev/null || echo "  (no match)"
    hr

    echo "[2/2] telemetry.log (high-volume, every tick)"
    grep -n "seq=${SEQ}\b" "$LOGS_TELEMETRY" 2>/dev/null || echo "  (no match)"
    hr

    echo "Cross-check: sensor_server.log / data_controller.log for the same seq"
    grep -n "seq=${SEQ}\b" "$LOGS_SERVER" "$LOGS_CONTROLLER" 2>/dev/null || echo "  (no match)"

else
    echo "Unknown mode '$MODE'. Use 'event' or 'seq'."
    exit 1
fi
