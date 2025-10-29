#!/usr/bin/env python3
"""
Simple serial monitor to debug what's actually being received
"""

import serial
import serial.tools.list_ports
import time

# Find COM port
ports = list(serial.tools.list_ports.comports())
print("Available ports:")
for i, port in enumerate(ports):
    print(f"  {i}: {port.device} - {port.description}")

if not ports:
    print("No ports found!")
    exit(1)

port_num = 0
if len(ports) > 1:
    port_num = int(input("Select port number: "))

ser = serial.Serial(ports[port_num].device, 115200, timeout=1)
print(f"\nConnected to {ports[port_num].device}")
print("=" * 80)
print("RAW OUTPUT:")
print("=" * 80)

time.sleep(2)  # Wait for ESP32 reset
ser.reset_input_buffer()

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline()
            try:
                # Try to decode as ASCII
                text = line.decode('ascii', errors='replace')
                print(f"ASCII: {text.strip()}")
            except:
                pass

            # Also show hex
            print(f"  HEX: {line.hex().upper()}")
            print()

        time.sleep(0.01)
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
