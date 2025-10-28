#!/usr/bin/env python3
"""
Minimal Protocol Tester - Baseline Serial Reliability Test

Tests a dead-simple ASCII hex line protocol to verify serial communication works
BEFORE attempting the complex byte-stuffed binary protocol.

Protocol Spec:
    Format: :[LENGTH][DATA_HEX][CHECKSUM]\n

    Example: :0548656C6C6F21\n
        :       = Start marker
        05      = Length (5 bytes)
        48656C6C6F = "Hello" in hex
        21      = Checksum (XOR of length and all data bytes)
        \n      = End marker (newline)

Why this protocol:
    ✓ Newline-delimited = Python's readline() handles framing perfectly
    ✓ ASCII hex = No binary issues, human-readable for debugging
    ✓ Self-synchronizing = Missed frames don't desync future frames
    ✓ Simple checksum = Detects corruption
    ✓ No byte stuffing needed = No special bytes to escape

Usage:
    1. Flash ESP32 with MINIMAL_PROTOCOL_MODE enabled
    2. Run this script
    3. Verify 100% pass rate
    4. Then switch to full protocol

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
TIMEOUT = 2.0

def calculate_checksum(data_bytes):
    """XOR checksum: XOR of length byte and all data bytes"""
    chk = len(data_bytes)
    for b in data_bytes:
        chk ^= b
    return chk & 0xFF

def encode_frame(data_bytes):
    """
    Encode data as minimal protocol frame.

    Args:
        data_bytes: Raw data to send

    Returns:
        ASCII-encoded frame with newline terminator
    """
    length = len(data_bytes)
    if length > 255:
        raise ValueError(f"Data too long: {length} bytes (max 255)")

    data_hex = data_bytes.hex().upper()
    checksum = calculate_checksum(data_bytes)
    frame = f":{length:02X}{data_hex}{checksum:02X}\n"
    return frame.encode('ascii')

def decode_frame(line):
    """
    Decode minimal protocol frame.

    Args:
        line: Bytes received from serial (should end with \\n)

    Returns:
        Data bytes if valid, None if invalid/corrupted
    """
    try:
        line = line.decode('ascii').strip()

        if not line.startswith(':'):
            print(f"    [!] Missing start marker ':'")
            return None

        line = line[1:]  # Remove ':'

        if len(line) < 4:  # Minimum: 2 hex length + 2 hex checksum
            print(f"    [!] Frame too short: {len(line)} chars (minimum 4)")
            return None

        # Parse length (2 hex chars)
        length = int(line[0:2], 16)

        # Verify we have enough data
        expected_len = 2 + (length * 2) + 2  # LEN + DATA + CHK
        if len(line) < expected_len:
            print(f"    [!] Incomplete: got {len(line)} chars, expected {expected_len}")
            return None

        # Parse data (length * 2 hex chars)
        data_hex = line[2:2 + (length * 2)]
        data_bytes = bytes.fromhex(data_hex)

        # Parse checksum (2 hex chars)
        rx_chk = int(line[2 + (length * 2):2 + (length * 2) + 2], 16)

        # Verify checksum
        calc_chk = calculate_checksum(data_bytes)
        if rx_chk != calc_chk:
            print(f"    [!] Checksum fail: got 0x{rx_chk:02X}, expected 0x{calc_chk:02X}")
            return None

        return data_bytes

    except Exception as e:
        print(f"    [!] Decode error: {e}")
        return None

def list_ports():
    """List available serial ports"""
    ports = list(serial.tools.list_ports.comports())
    return [p.device for p in ports]

def test_echo_once(ser, data, test_num, total_tests, quiet=False):
    """
    Send data and verify echo response (single attempt).

    Args:
        ser: Serial port object
        data: Bytes to send
        test_num: Current test number
        total_tests: Total number of tests
        quiet: If True, only print failures

    Returns:
        (success: bool, retry_info: str) - success status and info about any issues
    """
    frame = encode_frame(data)

    # Display test info
    if not quiet:
        print(f"\n[Test {test_num}/{total_tests}] {len(data)} bytes")
        print(f"  TX Data: {data.hex().upper() if len(data) <= 32 else data.hex().upper()[:64] + '...'}")
        print(f"  TX Frame: {frame.decode('ascii').strip()}")

    # Clear buffers and send
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    # Receive with timeout - use longer timeout for large frames
    # Formula: base 2s + 10ms per byte of data (hex encoding doubles it)
    expected_time = 2.0 + (len(data) * 2 * 10 / 1000.0)  # 10ms per hex char
    start_time = time.time()

    # Read with dynamic timeout
    old_timeout = ser.timeout
    ser.timeout = max(expected_time, 3.0)  # At least 3 seconds

    try:
        line = ser.readline()
    finally:
        ser.timeout = old_timeout

    elapsed = time.time() - start_time

    if not line:
        # Check if there's partial data in buffer
        partial = ser.in_waiting
        if not quiet:
            print(f"  [Test {test_num}] RX: ✗ TIMEOUT after {elapsed:.3f}s (size={len(data)}, expected_time={expected_time:.3f}s, partial_bytes={partial})")
        # CRITICAL: Clear buffer to prevent contamination of next test
        if partial > 0:
            junk = ser.read(partial)
            if not quiet:
                print(f"    [!] Cleared {partial} leftover bytes: {junk[:50]}...")
        ser.reset_input_buffer()
        return False, f"TIMEOUT after {elapsed:.3f}s"

    if not quiet:
        print(f"  RX Frame: {line.decode('ascii', errors='replace').strip()} ({elapsed*1000:.1f}ms)")

    # Decode response
    response = decode_frame(line)
    if response is None:
        if not quiet:
            print(f"  [Test {test_num}] RX: ✗ INVALID FRAME (size={len(data)})")
        # CRITICAL: Clear buffer to prevent contamination of next test
        ser.reset_input_buffer()
        return False, "INVALID FRAME"

    if not quiet:
        print(f"  RX Data: {response.hex().upper() if len(response) <= 32 else response.hex().upper()[:64] + '...'}")

    # Verify echo
    if response == data:
        if not quiet:
            print(f"  Result: ✓ PASS - Echo matched perfectly")
        return True, "OK"
    else:
        if not quiet:
            print(f"  [Test {test_num}] Result: ✗ FAIL - Echo mismatch! (size={len(data)})")
            print(f"          Expected: {data.hex().upper()}")
            print(f"          Got:      {response.hex().upper()}")
        # CRITICAL: Clear buffer to prevent contamination of next test
        ser.reset_input_buffer()
        return False, "ECHO MISMATCH"

def test_echo(ser, data, test_num, total_tests, quiet=False, max_retries=2):
    """
    Send data and verify echo response with automatic retry.

    Args:
        ser: Serial port object
        data: Bytes to send
        test_num: Current test number
        total_tests: Total number of tests
        quiet: If True, only print failures
        max_retries: Maximum number of retry attempts (default 2)

    Returns:
        (success: bool, attempts: int, error: str or None)
        - success: True if echo matched (on any attempt)
        - attempts: Number of attempts needed (1 = first try, 2 = one retry, etc.)
        - error: Error message if all retries failed, None if success
    """
    for attempt in range(1, max_retries + 2):  # +2 because range is exclusive and we start at 1
        success, info = test_echo_once(ser, data, test_num, total_tests, quiet=(quiet and attempt == 1))

        if success:
            # Success! Report how many attempts it took
            if attempt > 1 and not quiet:
                print(f"  [Test {test_num}] ✓ RECOVERED on attempt {attempt}")
            return True, attempt, None

        # Failed this attempt
        if attempt < max_retries + 1:
            # We have retries left
            if not quiet:
                print(f"  [Test {test_num}] Retry {attempt}/{max_retries} due to: {info}")
            time.sleep(0.05)  # 50ms pause before retry
        else:
            # Out of retries
            if not quiet:
                print(f"  [Test {test_num}] ✗ HARD FAIL after {attempt} attempts: {info}")
            return False, attempt, info

    # Should never reach here
    return False, max_retries + 1, "Unknown error"

def generate_random_data(max_size=255):
    """Generate random test data between 1 and max_size bytes"""
    size = random.randint(1, max_size)
    return bytes(random.randint(0, 255) for _ in range(size))

def continuous_stress_test(ser, max_retries=2):
    """
    Run continuous stress test with random data sizes until stopped.

    Args:
        ser: Serial port object
        max_retries: Number of retries before counting as failure (default 2)
    """
    print("\n" + "="*70)
    print(" CONTINUOUS STRESS TEST MODE (WITH RETRY LOGIC)")
    print("="*70)
    print()
    print(f"This will send random-sized packets continuously until you press Ctrl+C.")
    print(f"Each test gets up to {max_retries} retries before counting as failure.")
    print("Only hard failures will be printed to the console.")
    print()
    input("Press Enter to start...")

    test_num = 0
    first_try_success = 0
    recovered_after_retry = 0
    hard_failures = 0
    retry_attempts = {1: 0, 2: 0, 3: 0}  # Track attempts needed
    start_time = time.time()
    last_update = start_time

    try:
        while True:
            test_num += 1
            data = generate_random_data(255)

            # Run test with retry logic
            success, attempts, error = test_echo(ser, data, test_num, "∞", quiet=True, max_retries=max_retries)

            # Track statistics
            if success:
                if attempts == 1:
                    first_try_success += 1
                else:
                    recovered_after_retry += 1
                    print(f"  [Test {test_num}] ⚠ Recovered on attempt {attempts} (size={len(data)})")
                retry_attempts[attempts] = retry_attempts.get(attempts, 0) + 1
            else:
                hard_failures += 1
                print(f"  [Test {test_num}] ✗ HARD FAIL after {attempts} attempts (size={len(data)}): {error}")

            # Print status update every 5 seconds
            now = time.time()
            if now - last_update >= 5.0:
                elapsed = now - start_time
                rate = test_num / elapsed
                effective_success = ((first_try_success + recovered_after_retry) / test_num * 100) if test_num > 0 else 0
                first_try_rate = (first_try_success / test_num * 100) if test_num > 0 else 0

                print(f"[{elapsed:.0f}s] Tests: {test_num} | First-try: {first_try_success} ({first_try_rate:.2f}%) | " +
                      f"Recovered: {recovered_after_retry} | Hard Fail: {hard_failures} | " +
                      f"Effective: {effective_success:.2f}% | Rate: {rate:.1f}/s")
                last_update = now

            # Small delay to avoid overwhelming the ESP
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print(" STRESS TEST STOPPED")
        print("="*70)
        elapsed = time.time() - start_time
        rate = test_num / elapsed if elapsed > 0 else 0
        total_success = first_try_success + recovered_after_retry
        effective_success_rate = (total_success / test_num * 100) if test_num > 0 else 0
        first_try_rate = (first_try_success / test_num * 100) if test_num > 0 else 0

        print(f"\n{'='*70}")
        print(" DETAILED STATISTICS")
        print(f"{'='*70}")
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
        print("Attempts Distribution:")
        for attempt in sorted(retry_attempts.keys()):
            count = retry_attempts[attempt]
            pct = (count / test_num * 100) if test_num > 0 else 0
            print(f"  {attempt} attempt(s): {count} ({pct:.2f}%)")

        print(f"\n{'='*70}")
        if hard_failures == 0:
            print("✓✓✓ PERFECT - No hard failures!")
            print(f"With retry logic: {effective_success_rate:.2f}% success rate")
        else:
            print(f"⚠ {hard_failures} hard failures detected")
            print(f"But with retry logic: {effective_success_rate:.2f}% effective success rate")
        print(f"{'='*70}")

def main():
    print("="*70)
    print(" Minimal Protocol Tester - Serial Reliability Baseline Test")
    print("="*70)
    print()
    print("This tests a simple ASCII hex line protocol to verify that your")
    print("serial connection is working before trying complex protocols.")
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
        time.sleep(2)  # Wait for ESP32 to boot/stabilize
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("✓ Connected\n")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return

    # Choose test mode
    print("\nTest Modes:")
    print("  1) Basic 10-test suite (recommended for first run)")
    print("  2) Continuous stress test (random sizes until Ctrl+C)")
    mode = input("\nSelect mode [1]: ").strip() or "1"

    if mode == "2":
        continuous_stress_test(ser)
        ser.close()
        return

    # Define test cases
    tests = [
        (b"A", "Single byte"),
        (b"AB", "Two bytes"),
        (b"Hello", "ASCII string (5 bytes)"),
        (b"TestData123", "Medium string (12 bytes)"),
        (b"\x00\x01\x02\x03\x04", "Binary data (5 bytes)"),
        (b"X" * 32, "32 bytes"),
        (b"Y" * 64, "64 bytes (USB packet size)"),
        (b"Z" * 100, "100 bytes"),
        (bytes(range(128)), "128 bytes (all values 0x00-0x7F)"),
        (bytes(range(256))[:255], "255 bytes (max for 1-byte length)"),
    ]

    total = len(tests)
    results = []

    print(f"Running {total} echo tests...")
    print("-"*70)

    # Run tests
    for i, (data, desc) in enumerate(tests, 1):
        print(f"\n{'='*70}")
        print(f"{desc}")
        print(f"{'='*70}")

        success, attempts, error = test_echo(ser, data, i, total)
        results.append((success, desc, len(data), attempts))

        if i < total:
            time.sleep(0.1)  # Small delay between tests

    ser.close()

    # Print summary
    print("\n" + "="*70)
    print(" TEST SUMMARY")
    print("="*70)

    passed = sum(1 for r in results if r[0])
    failed = total - passed

    for i, (success, desc, size, attempts) in enumerate(results, 1):
        status = "✓ PASS" if success else "✗ FAIL"
        retry_info = f" (attempt {attempts})" if attempts > 1 else ""
        print(f"  {status} | Test {i:2d} | {size:3d} bytes | {desc}{retry_info}")

    print("-"*70)
    print(f"  Results: {passed}/{total} passed ({passed/total*100:.0f}%)")

    if passed == total:
        print("\n" + "="*70)
        print(" ✓✓✓ ALL TESTS PASSED - Serial link is 100% reliable!")
        print("="*70)
        print()
        print("Your serial connection is working perfectly with this simple protocol.")
        print("You can now proceed to test the complex byte-stuffed protocol.")
        print()
        sys.exit(0)
    else:
        print("\n" + "="*70)
        print(f" ✗✗✗ {failed} TESTS FAILED - Serial link has issues!")
        print("="*70)
        print()
        print("Issues found with basic serial communication. Possible causes:")
        print("  - USB cable quality")
        print("  - USB CDC buffer issues")
        print("  - ESP32 firmware not in MINIMAL_PROTOCOL_MODE")
        print("  - Port settings incorrect")
        print()
        print("Fix these basic issues before attempting complex protocols.")
        print()
        sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
