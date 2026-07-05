#!/bin/bash
# trace_event.sh — run on the BeagleBone only.
# Traces a single event_id or frame_seq across every log domain confirmed
# during the event_id/eid/frame_seq audit.
#
# Usage:
#   ./trace_event.sh event <decimal_event_id>
#   ./trace_event.sh seq   <frame_seq>
#   ./trace_event.sh frame <frame_seq> [field_filter]
#
# event mode converts the decimal id to the 16-hex-digit EVENT_ID_FMT form
# automatically and greps both forms across the right log files.
# seq mode greps frame_seq as-is across the server telemetry/event logs.
# frame mode pulls one telemetry.log tick and breaks it into one
# field-per-line (like `tr ' ' '\n'`), optionally filtered to fields
# matching field_filter (e.g. "age", "batt", "pir") — same idea as
# `tail -1 /var/log/telemetry.log | tr ' ' '\n' | grep age`, but keyed
# to a specific frame_seq instead of always the latest tick. Also lists
# every discrete events.log entry sharing that frame_seq, since more
# than one sensor can transition within the same tick.

set -euo pipefail

MODE="${1:-}"
VAL="${2:-}"
FIELD_FILTER="${3:-}"

if [[ -z "$MODE" || -z "$VAL" ]]; then
    echo "Usage: $0 event <decimal_event_id>"
    echo "       $0 seq   <frame_seq>"
    echo "       $0 frame <frame_seq> [field_filter]"
    exit 1
fi

LOGS_CONTROLLER="/var/log/data_controller.log"
LOGS_CAMERA="/var/log/camera_manager.log"
LOGS_INFERENCE="/var/log/inference.log"
LOGS_DOORBELL="/var/log/doorbell_daemon.log"
LOGS_SERVER="/var/log/sensor_server.log"
LOGS_TELEMETRY="/var/log/telemetry.log"
LOGS_EVENTS="/var/log/events.log"

# Log lines here commonly carry a dozen+ key=value fields (DISPATCH/telemetry
# especially) and blow past a normal terminal width as one unbroken line.
# Wrap at word boundaries instead of letting the terminal hard-wrap mid-field.
WRAP_WIDTH="${TRACE_WRAP_WIDTH:-100}"

hr() { printf '%s\n' "------------------------------------------------------------"; }

# grep_wrapped PATTERN FILE [FILE...] NOMATCH_MSG
# Greps for PATTERN in the given files, wraps any matched lines to
# WRAP_WIDTH at word boundaries, and prints NOMATCH_MSG if nothing matched.
# Safe under `set -e` — capture-then-check instead of relying on grep's
# own exit code, which is 1 on zero matches even though it prints nothing.
grep_wrapped() {
    local pattern="$1"; shift
    local nomatch_msg="${*: -1}"
    local files=("${@:1:$#-1}")
    local output
    output=$(grep -n -- "$pattern" "${files[@]}" 2>/dev/null || true)
    if [[ -n "$output" ]]; then
        echo "$output" | fold -s -w "$WRAP_WIDTH"
    else
        echo "  ($nomatch_msg)"
    fi
}

if [[ "$MODE" == "event" ]]; then
    DEC="$VAL"
    HEX=$(printf '%016x\n' "$DEC")

    echo "Tracing event_id=$DEC  (hex form: $HEX)"
    hr

    echo "[1/4] camera_manager.log (decimal event_id, cam_tx_id, boot_epoch)"
    grep_wrapped "event_id=${DEC}\b" "$LOGS_CAMERA" "no match"
    hr

    echo "[2/4] data_controller.log (decimal event_id — DISPATCH/SHM/UART, unified naming)"
    grep_wrapped "event_id=${DEC}\b" "$LOGS_CONTROLLER" "no match"
    echo "  -- checking for stale 'eid=' usage (should be none post-rename) --"
    eid_output=$(grep -n -- "eid=${DEC}\b" "$LOGS_CONTROLLER" 2>/dev/null || true)
    if [[ -n "$eid_output" ]]; then
        echo "$eid_output" | fold -s -w "$WRAP_WIDTH"
        echo "  WARNING: found legacy eid= usage above"
    else
        echo "  (none — good)"
    fi
    hr

    echo "[3/4] inference.log (hex event_id via EVENT_ID_FMT)"
    grep_wrapped "event_id=${HEX}\b" "$LOGS_INFERENCE" "no match"
    hr

    echo "[4/4] doorbell_daemon.log (hex event_id via EVENT_ID_FMT, if applicable)"
    grep_wrapped "event_id=${HEX}\b" "$LOGS_DOORBELL" "no match"
    hr

    echo "Summary line counts:"
    for f in "$LOGS_CAMERA" "$LOGS_CONTROLLER" "$LOGS_INFERENCE" "$LOGS_DOORBELL"; do
        [[ -f "$f" ]] || continue
        c_dec=$(grep -c "event_id=${DEC}\b" "$f" 2>/dev/null || true)
        c_hex=$(grep -c "event_id=${HEX}\b" "$f" 2>/dev/null || true)
        printf "  %-28s dec=%-4s hex=%-4s\n" "$(basename "$f")" "${c_dec:-0}" "${c_hex:-0}"
    done
    hr

    echo "Bridged frame_seq check (indoor cam path only, as of 2026-07-04 — see README):"
    echo "  a line with both event_id and seq= confirms trigger-time bridging worked."
    grep_wrapped "event_id=${DEC}\b.*seq=\|seq=.*event_id=${DEC}\b" "$LOGS_CONTROLLER" \
        "no bridged seq= found in data_controller.log — bridge originates here at sensor_dispatch.c"
    grep_wrapped "event_id=${DEC}\b.*seq=\|seq=.*event_id=${DEC}\b" "$LOGS_CAMERA" \
        "no bridged seq= found in camera_manager.log — expected if trigger was dropped before send, or on pre-bridge binaries/doorbell events"
    grep_wrapped "event_id=${HEX}\b.*seq=\|seq=.*event_id=${HEX}\b" "$LOGS_INFERENCE" \
        "no bridged seq= found in inference.log — expected on pre-bridge binaries or doorbell events"

elif [[ "$MODE" == "seq" ]]; then
    SEQ="$VAL"
    echo "Tracing frame_seq=$SEQ  (server domain join key — separate from event_id)"
    hr

    echo "[1/2] events.log (low-volume, true state transitions only)"
    grep_wrapped "seq=${SEQ}\b" "$LOGS_EVENTS" "no match"
    hr

    echo "[2/2] telemetry.log (high-volume, every tick)"
    grep_wrapped "seq=${SEQ}\b" "$LOGS_TELEMETRY" "no match"
    hr

    echo "Cross-check: sensor_server.log / data_controller.log for the same seq"
    grep_wrapped "seq=${SEQ}\b" "$LOGS_SERVER" "$LOGS_CONTROLLER" "no match"

elif [[ "$MODE" == "frame" ]]; then
    SEQ="$VAL"
    echo "Frame breakdown for frame_seq=$SEQ"
    [[ -n "$FIELD_FILTER" ]] && echo "Filtering fields matching: $FIELD_FILTER"
    hr

    echo "[1/2] telemetry.log — field-by-field breakdown"
    line=$(grep -- "seq=${SEQ}\b" "$LOGS_TELEMETRY" 2>/dev/null | tail -1 || true)
    if [[ -n "$line" ]]; then
        if [[ -n "$FIELD_FILTER" ]]; then
            echo "$line" | tr ' ' '\n' | grep -i -- "$FIELD_FILTER" | sed 's/^/  /' \
                || echo "  (no fields matched '$FIELD_FILTER' in this tick)"
        else
            echo "$line" | tr ' ' '\n' | sed 's/^/  /'
        fi
    else
        echo "  (no telemetry.log entry for frame_seq=$SEQ)"
    fi
    hr

    echo "[2/2] events.log — all discrete events sharing this frame_seq"
    echo "  (more than one line here means multiple sensors transitioned in the same tick)"
    events_output=$(grep -n -- "seq=${SEQ}\b" "$LOGS_EVENTS" 2>/dev/null || true)
    if [[ -n "$events_output" ]]; then
        echo "$events_output" | fold -s -w "$WRAP_WIDTH"
        event_count=$(echo "$events_output" | wc -l)
        echo "  --> $event_count event(s) found for this frame_seq"
    else
        echo "  (no discrete events for this frame_seq — telemetry-only tick)"
    fi

else
    echo "Unknown mode '$MODE'. Use 'event', 'seq', or 'frame'."
    exit 1
fi
