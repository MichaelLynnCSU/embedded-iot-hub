#!/usr/bin/env python3
"""
esp01_tcp_server_setup.py
=========================
Author : MichaelLynnCSU (https://github.com/MichaelLynnCSU)
Date   : 2025-01-01

Configure the ESP-01 as a TCP server on the BeagleBone.

Runs once at boot via systemd, sends AT commands to the ESP-01 over
UART, verifies WiFi connectivity, and starts a TCP server on TCP_PORT.
Exits cleanly so sensor_server can take over the UART.

Systemd integration:
    READY=1     sent after TCP server confirmed listening
    STOPPING=1  sent on failure or clean exit
"""

import sys
import time

import serial

try:
    from systemd import daemon as _daemon  # type: ignore
except ImportError:
    _daemon = None  # fallback when not running under systemd

# ========================= CONFIGURATION =====================================

UART_PORT        = "/dev/ttyS4"  # UART connected to ESP-01
BAUDRATE         = 115200        # ESP-01 default baud rate
TCP_PORT         = 5000          # TCP server port for sensor_server
AT_DEFAULT_WAIT  = 2             # default seconds to wait for AT response
AT_SHORT_WAIT    = 1             # shorter wait for fast commands
AT_TIMEOUT_SEC   = 180           # ESP-01 TCP connection timeout seconds
UART_OPEN_DELAY  = 1             # seconds to wait after opening UART
READ_POLL_DELAY  = 0.1           # seconds between in_waiting polls
SEPARATOR        = "=" * 60      # print separator string

# ========================= HELPERS ===========================================

def _notify(status: str) -> None:
    """Send a systemd notification if running under systemd."""
    if _daemon is not None:
        _daemon.notify(status)


def send_at_command(ser: serial.Serial, cmd: str, wait: int = AT_DEFAULT_WAIT) -> str:
    """
    Send an AT command to the ESP-01 and return the response string.

    Args:
        ser:  Open serial port connected to ESP-01.
        cmd:  AT command string without trailing CR/LF.
        wait: Seconds to wait before reading response.

    Returns:
        Decoded response string, empty string on read error.
    """
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    time.sleep(wait)

    response = b""
    while ser.in_waiting:
        response += ser.read(ser.in_waiting)
        time.sleep(READ_POLL_DELAY)

    return response.decode(errors="ignore")


def fail(message: str, response: str = "") -> None:
    """
    Print failure message, notify systemd, and exit with code 1.

    Args:
        message:  Human-readable failure description.
        response: Optional AT response string for diagnostics.
    """
    print(f"   \u2717 {message}")

    if response:
        print(f"   Response: {response}")

    _notify("STATUS=ESP-01 failed")
    _notify("STOPPING=1")
    sys.exit(1)


# ========================= MAIN ==============================================

def main() -> None:
    """
    Configure ESP-01 as TCP server and exit.

    Steps:
        1. Open UART
        2. Test AT communication
        3. Verify WiFi connection
        4. Get IP address
        5. Enable multiple connections (AT+CIPMUX=1)
        6. Start TCP server (AT+CIPSERVER)
        7. Set connection timeout (AT+CIPSTO)
    """
    print(SEPARATOR)
    print("ESP-01 TCP Server Configuration")
    print(SEPARATOR)

    # ── 1. Open UART ─────────────────────────────────────────────────────────
    try:
        ser = serial.Serial(UART_PORT, BAUDRATE, timeout=AT_DEFAULT_WAIT)
        time.sleep(UART_OPEN_DELAY)
    except Exception as exc:  # pylint: disable=broad-except
        fail("Cannot open UART port", str(exc))
        return  # unreachable — fail() exits, satisfies linters

    # ── 2. Test AT communication ──────────────────────────────────────────────
    print("\n1. Testing AT communication...")
    resp = ""
    for attempt in range(10):
        resp = send_at_command(ser, "AT")
        if "OK" in resp:
            print("   ✓ ESP-01 responding")
            break
        print(f"   Waiting for ESP-01... (attempt {attempt + 1}/10)")
        time.sleep(3)
    else:
        fail("No response from ESP-01 after 10 attempts", resp)

    # ── 3. Check WiFi ─────────────────────────────────────────────────────────
    print("\n2. Checking WiFi connection...")
    resp = send_at_command(ser, "AT+CWJAP?")
    if "No AP" in resp:
        fail("Not connected to WiFi")
    else:
        print("   \u2713 WiFi connected")
        if "+CWJAP:" in resp:
            ssid_line = next(
                (line for line in resp.split("\n") if "+CWJAP:" in line), ""
            )
            print(f"   {ssid_line.strip()}")

    # Reset ESP01 TCP server state before reconfiguring
    print("\n3b. Resetting TCP server state...")
    send_at_command(ser, "AT+CIPSERVER=0", 2)  # close server if running
    send_at_command(ser, "AT+CIPMUX=0", 1)     # reset to single connection
    time.sleep(1)
    print("   ✓ Reset complete")

    # ── 4. Get IP address ─────────────────────────────────────────────────────
    print("\n3. Getting IP address...")
    resp = send_at_command(ser, "AT+CIFSR")
    ip_found = False
    for line in resp.split("\n"):
        if "STAIP" in line:
            print(f"   {line.strip()}")
            ip_found = True

    if not ip_found:
        fail("Failed to get IP address", resp)

    # ── 5. Enable multiple connections ────────────────────────────────────────
    print("\n4. Configuring for multiple connections...")
    resp = send_at_command(ser, "AT+CIPMUX=1", AT_SHORT_WAIT)
    if "OK" in resp:
        print("   \u2713 Multiple connections enabled")
    else:
        fail("Failed to enable multiple connections", resp)

    # ── 6. Start TCP server ───────────────────────────────────────────────────
    print(f"\n5. Starting TCP server on port {TCP_PORT}...")
    resp = send_at_command(ser, f"AT+CIPSERVER=1,{TCP_PORT}", AT_DEFAULT_WAIT)
    if ("OK" in resp) or ("no change" in resp):
        print(f"   \u2713 TCP server listening on port {TCP_PORT}")
        _notify("READY=1")
        _notify("STATUS=TCP server configured")
    else:
        fail("Failed to start TCP server", resp)

    # ── 7. Set connection timeout ─────────────────────────────────────────────
    print("\n6. Setting timeout...")
    resp = send_at_command(ser, f"AT+CIPSTO={AT_TIMEOUT_SEC}", AT_SHORT_WAIT)
    if "OK" in resp:
        print(f"   \u2713 Timeout set to {AT_TIMEOUT_SEC} seconds")

    # ── Done ──────────────────────────────────────────────────────────────────
    print(f"\n{SEPARATOR}")
    print("ESP-01 TCP Server Ready!")
    print(SEPARATOR)
    print(f"\nVroom should send data to: <ESP_IP>:{TCP_PORT}")
    print("Configuration complete. Exiting...")
    print("sensor_server will now handle incoming data via UART")
    print(SEPARATOR)

    ser.close()
    _notify("STOPPING=1")
    sys.exit(0)


if __name__ == "__main__":
    main()

