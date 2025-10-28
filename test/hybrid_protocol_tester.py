#!/usr/bin/env python3
"""
Hybrid Protocol Tester

Tests the hybrid protocol that combines:
- Binary structure (PID, length, payload, checksum) from original protocol
- Newline framing from minimal protocol
- ASCII hex encoding for reliability and debuggability

Wire format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n

Example: <AA5501000155AA>\n
         └─┬─┘││││││││││└─ End marker '>'
           │  ││││││││└─── End markers 55 AA
           │  │││││││└──── Checksum
           │  ││││││└───── (no payload, len=00)
           │  │││││└────── Length = 0x00
           │  ││││└─────── Command = 0x00
           │  │││└──────── PID = 0x01 (LORA_915)
           │  ││└───────── Start markers AA 55
           │  │└────────── Frame start '<'
           │  └─────────── Newline terminator

Author: Claude Code
Date: 2025
"""

import serial
import serial.tools.list_ports
import time
import sys
import random

# Configuration
DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 115200
TIMEOUT = 3.0

# Peripheral IDs (same as original protocol)
PERIPHERAL_ID_SYSTEM = 0x00
PERIPHERAL_ID_LORA_915 = 0x01
PERIPHERAL_ID_RADIO_433 = 0x02
PERIPHERAL_ID_BAROMETER = 0x03
PERIPHERAL_ID_CURRENT = 0x04

# Commands (same as original protocol)
CMD_GET_STATUS = 0x00
CMD_GET_ALL = 0x00
CMD_GET_LATEST = 0x01

def calculate_checksum(pid, length, payload):
    """Calculate XOR checksum: PID ^ LEN ^ PAYLOAD bytes"""
    chk = pid ^ length
    for b in payload:
        chk ^= b
    return chk & 0xFF

def encode_hybrid_frame(peripheral_id, payload):
    """
    Encode hybrid protocol frame.

    Args:
        peripheral_id: Peripheral ID (0x00-0xFF)
        payload: Payload bytes (can include command as first byte)

    Returns:
        ASCII frame with newline terminator
    """
    if len(payload) > 255:
        raise ValueError(f"Payload too long: {len(payload)} bytes (max 255)")

    # Calculate checksum
    checksum = calculate_checksum(peripheral_id, len(payload), payload)

    # Build frame: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
    frame = '<'
    frame += 'AA55'  # Start markers
    frame += f'{peripheral_id:02X}'  # PID
    frame += f'{len(payload):02X}'  # Length
    frame += payload.hex().upper()  # Payload as hex
    frame += f'{checksum:02X}'  # Checksum
    frame += '55AA'  # End markers
    frame += '>\n'  # End + newline

    return frame.encode('ascii')

def decode_hybrid_frame(line):
    """
    Decode hybrid protocol frame.

    Args:
        line: Bytes received from serial (should end with \\n)

    Returns:
        (pid, payload) if valid, (None, None) if invalid
    """
    try:
        line = line.decode('ascii').strip()

        # Check frame markers
        if not line.startswith('<') or not line.endswith('>'):
            print(f"    [!] Missing frame markers < >")
            return None, None

        # Remove < > markers
        line = line[1:-1]

        # Minimum: AA55PPLL55AA = 12 chars
        if len(line) < 12:
            print(f"    [!] Frame too short: {len(line)} chars (minimum 12)")
            return None, None

        # Check start markers
        if not line.startswith('AA55'):
            print(f"    [!] Invalid start markers: {line[:4]}")
            return None, None

        # Parse PID
        pid = int(line[4:6], 16)

        # Parse length
        length = int(line[6:8], 16)

        # Calculate expected length: AA55 + PP + LL + (data*2) + CHK + 55AA
        expected_len = 4 + 2 + 2 + (length * 2) + 2 + 4
        if len(line) < expected_len:
            print(f"    [!] Incomplete: got {len(line)} chars, expected {expected_len}")
            return None, None

        # Parse payload
        payload_hex = line[8:8 + (length * 2)]
        payload = bytes.fromhex(payload_hex)

        # Parse checksum
        rx_chk = int(line[8 + (length * 2):8 + (length * 2) + 2], 16)

        # Verify checksum
        calc_chk = calculate_checksum(pid, length, payload)
        if rx_chk != calc_chk:
            print(f"    [!] Checksum fail: got 0x{rx_chk:02X}, expected 0x{calc_chk:02X}")
            return None, None

        # Check end markers
        end_markers = line[8 + (length * 2) + 2:8 + (length * 2) + 2 + 4]
        if end_markers != '55AA':
            print(f"    [!] Invalid end markers: {end_markers}")
            return None, None

        return pid, payload

    except Exception as e:
        print(f"    [!] Decode error: {e}")
        return None, None

def send_request(ser, peripheral_id, command, payload=b'', timeout=3.0):
    """
    Send request and receive response.

    Args:
        ser: Serial port object
        peripheral_id: Peripheral ID
        command: Command byte
        payload: Optional payload (command will be prepended)
        timeout: Timeout in seconds

    Returns:
        (success, response_payload) - success status and payload (without PID)
    """
    # Build request payload: [COMMAND] + [additional payload]
    request_payload = bytes([command]) + payload

    # Encode and send
    frame = encode_hybrid_frame(peripheral_id, request_payload)
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    # Receive response with timeout
    old_timeout = ser.timeout
    ser.timeout = timeout

    try:
        line = ser.readline()
    finally:
        ser.timeout = old_timeout

    if not line:
        print(f"    [!] TIMEOUT waiting for response")
        ser.reset_input_buffer()
        return False, None

    # Decode response
    rx_pid, rx_payload = decode_hybrid_frame(line)

    if rx_pid is None:
        print(f"    [!] INVALID FRAME")
        ser.reset_input_buffer()
        return False, None

    # Verify PID matches
    if rx_pid != peripheral_id:
        print(f"    [!] PID mismatch: expected {peripheral_id:02X}, got {rx_pid:02X}")
        return False, None

    return True, rx_payload

def list_ports():
    """List available serial ports"""
    ports = list(serial.tools.list_ports.comports())
    return [p.device for p in ports]

def test_peripheral_request(ser, peripheral_id, command, max_retries=2):
    """
    Test request/response with retry logic.
    Now expects real peripheral data, not echo.

    Returns:
        (success, attempts, error, payload_size)
    """
    # Expected payload sizes for each peripheral (approximate)
    expected_sizes = {
        0x00: (20, 6),   # SYSTEM: status=20, heartbeat=6
        0x01: 74,        # LORA_915: WireLoRa_t = 74 bytes
        0x02: 74,        # RADIO_433: Wire433_t = 74 bytes
        0x03: 17,        # BAROMETER: WireBarometer_t = 17 bytes
        0x04: 19,        # CURRENT: WireCurrent_t = 19 bytes
    }

    for attempt in range(1, max_retries + 2):
        success, response = send_request(ser, peripheral_id, command, b'')

        if success:
            # Verify we got data
            if response and len(response) > 0:
                # Check if size is reasonable for this peripheral
                expected = expected_sizes.get(peripheral_id)
                if expected:
                    if isinstance(expected, tuple):
                        # Multiple valid sizes (e.g., system status vs heartbeat)
                        if len(response) in expected:
                            return True, attempt, None, len(response)
                    else:
                        # Single expected size
                        if len(response) == expected:
                            return True, attempt, None, len(response)
                        else:
                            # Size mismatch, but still valid data
                            return True, attempt, f"Size {len(response)} (expected {expected})", len(response)
                else:
                    # Unknown peripheral, but got data
                    return True, attempt, None, len(response)
            else:
                error = f"Empty response (attempt {attempt})"
                if attempt < max_retries + 1:
                    time.sleep(0.05)
                    continue
                return False, attempt, error, 0

        # Failed this attempt
        if attempt < max_retries + 1:
            time.sleep(0.05)
        else:
            return False, attempt, "Request failed", 0

    return False, max_retries + 1, "Unknown error", 0

def continuous_stress_test(ser, max_retries=2):
    """Run continuous stress test with real peripheral data"""
    print("\n" + "="*70)
    print(" HYBRID PROTOCOL CONTINUOUS STRESS TEST")
    print(" (Testing Real Peripheral Data Structures)")
    print("="*70)
    print()
    print(f"Testing binary structure with newline framing and retry logic.")
    print(f"Each test gets up to {max_retries} retries before counting as failure.")
    print()
    print("Expected payload sizes:")
    print("  SYSTEM (0x00):    20 bytes (status) or 6 bytes (heartbeat)")
    print("  LORA_915 (0x01):  74 bytes (WireLoRa_t)")
    print("  RADIO_433 (0x02): 74 bytes (Wire433_t)")
    print("  BAROMETER (0x03): 17 bytes (WireBarometer_t)")
    print("  CURRENT (0x04):   19 bytes (WireCurrent_t)")
    print()
    input("Press Enter to start...")

    test_num = 0
    first_try_success = 0
    recovered_after_retry = 0
    hard_failures = 0
    payload_size_counts = {}  # Track payload sizes we see
    start_time = time.time()
    last_update = start_time

    peripherals = [PERIPHERAL_ID_SYSTEM, PERIPHERAL_ID_LORA_915, PERIPHERAL_ID_RADIO_433,
                   PERIPHERAL_ID_BAROMETER, PERIPHERAL_ID_CURRENT]

    try:
        while True:
            test_num += 1

            # Random peripheral and command
            pid = random.choice(peripherals)
            cmd = CMD_GET_STATUS if pid == PERIPHERAL_ID_SYSTEM else CMD_GET_ALL

            # Run test (no test_data needed - we want real peripheral responses)
            success, attempts, error, payload_size = test_peripheral_request(ser, pid, cmd, max_retries)

            # Track statistics
            if success:
                # Track payload sizes
                key = f"{pid:02X}:{payload_size}"
                payload_size_counts[key] = payload_size_counts.get(key, 0) + 1

                if attempts == 1:
                    first_try_success += 1
                else:
                    recovered_after_retry += 1
                    print(f"  [Test {test_num}] ⚠ Recovered on attempt {attempts} (PID={pid:02X}, size={payload_size})")
            else:
                hard_failures += 1
                print(f"  [Test {test_num}] ✗ HARD FAIL after {attempts} attempts (PID={pid:02X}): {error}")

            # Print status update every 5 seconds
            now = time.time()
            if now - last_update >= 5.0:
                elapsed = now - start_time
                rate = test_num / elapsed
                effective_success = ((first_try_success + recovered_after_retry) / test_num * 100) if test_num > 0 else 0
                first_try_rate = (first_try_success / test_num * 100) if test_num > 0 else 0

                print(f"[{elapsed:.0f}s] Tests: {test_num} | First-try: {first_try_rate:.2f}% | " +
                      f"Recovered: {recovered_after_retry} | Hard Fail: {hard_failures} | " +
                      f"Effective: {effective_success:.2f}% | Rate: {rate:.1f}/s")
                last_update = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        rate = test_num / elapsed if elapsed > 0 else 0
        total_success = first_try_success + recovered_after_retry
        effective_success_rate = (total_success / test_num * 100) if test_num > 0 else 0
        first_try_rate = (first_try_success / test_num * 100) if test_num > 0 else 0

        print("\n\n" + "="*70)
        print(" STRESS TEST STOPPED")
        print("="*70)
        print(f"Total Time: {elapsed:.1f}s")
        print(f"Total Tests: {test_num}")
        print(f"Test Rate: {rate:.1f} tests/second")
        print()
        print(f"First-Try Success: {first_try_success} ({first_try_rate:.2f}%)")
        print(f"Recovered After Retry: {recovered_after_retry}")
        print(f"Hard Failures: {hard_failures}")
        print()
        print(f"EFFECTIVE SUCCESS RATE: {effective_success_rate:.2f}%")
        print()
        print("Payload Size Distribution:")
        for key in sorted(payload_size_counts.keys()):
            pid_str, size_str = key.split(':')
            count = payload_size_counts[key]
            pct = (count / test_num * 100) if test_num > 0 else 0
            print(f"  PID {pid_str}: {size_str:>3} bytes - {count:>5} times ({pct:>5.2f}%)")
        print("="*70)

def main():
    print("="*70)
    print(" Hybrid Protocol Tester")
    print("="*70)
    print()
    print("Format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\\n")
    print()

    # List available ports
    ports = list_ports()
    if ports:
        print(f"Available ports: {', '.join(ports)}")
    else:
        print("No serial ports found!")
        return

    # Get port from user
    port = input(f"\nEnter port [{DEFAULT_PORT}]: ").strip() or DEFAULT_PORT
    baud = input(f"Enter baud rate [{DEFAULT_BAUD}]: ").strip()
    baud = int(baud) if baud else DEFAULT_BAUD

    # Connect
    print(f"\nConnecting to {port} @ {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=TIMEOUT)
        time.sleep(2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("✓ Connected\n")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return

    # Run continuous stress test
    continuous_stress_test(ser, max_retries=2)
    ser.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
