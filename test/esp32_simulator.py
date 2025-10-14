#!/usr/bin/env python3
"""
ESP32 Ground Station Simulator
Simulates the ESP32 embedded system for testing the binary communication protocol
without physical hardware.
"""

import serial
import struct
import time
import random
from enum import IntEnum
from dataclasses import dataclass


# Protocol Constants (must match config.h)
HELLO_BYTE = 0x7E
GOODBYE_BYTE = 0x7F

# Peripheral IDs
PERIPHERAL_ID_SYSTEM = 0x00

# Commands
CMD_GET_LORA_DATA = 0x01
CMD_GET_433_DATA = 0x02
CMD_GET_BAROMETER_DATA = 0x03
CMD_GET_CURRENT_DATA = 0x04
CMD_GET_ALL_DATA = 0x05
CMD_GET_STATUS = 0x06


class CommState(IntEnum):
    WAIT_FOR_HELLO = 0
    WAIT_FOR_PERIPHERAL_ID = 1
    WAIT_FOR_LENGTH = 2
    WAIT_FOR_MESSAGE_DATA = 3
    WAIT_FOR_GOODBYE = 4
    SENDING_RESPONSE = 5


@dataclass
class FakeSensorData:
    """Holds fake sensor readings"""
    # LoRa
    lora_online: bool = True
    lora_packet_count: int = 0
    lora_rssi: int = -85
    lora_snr: float = 3.5
    lora_latest_data: bytes = b'TestLoRaPacket123'

    # 433MHz
    radio433_online: bool = True
    radio433_packet_count: int = 0
    radio433_rssi: int = -92
    radio433_latest_data: bytes = b'Test433Packet456'

    # Barometer
    barometer_online: bool = True
    pressure_hpa: float = 1013.25
    temperature_c: float = 22.5
    altitude_m: float = 123.4

    # Current/Voltage
    current_online: bool = True
    current_a: float = 0.5
    voltage_v: float = 12.3
    power_w: float = 6.15
    raw_adc: int = 2048

    # System
    uptime_seconds: int = 0
    system_state: int = 1  # SYSTEM_OPERATIONAL
    wakeup_time: int = 1000
    free_heap: int = 100000
    chip_revision: int = 3


class ESP32Simulator:
    """Simulates ESP32 Ground Station behavior"""

    def __init__(self, serial_port, baud_rate=115200):
        """Initialize simulator with serial port"""
        self.serial = serial.Serial(serial_port, baud_rate, timeout=0.1)
        self.state = CommState.WAIT_FOR_HELLO
        self.peripheral_id = 0
        self.expected_length = 0
        self.message_buffer = bytearray()
        self.sensor_data = FakeSensorData()
        self.start_time = time.time()

        print(f"ESP32 Simulator started on {serial_port}")
        print("Waiting for commands from Pi...")

    def update_sensor_data(self):
        """Update fake sensor data to simulate real sensor readings"""
        # Simulate increasing packet counts
        if random.random() < 0.1:  # 10% chance
            self.sensor_data.lora_packet_count += 1
            # Generate new fake LoRa data
            self.sensor_data.lora_latest_data = f"LoRaPkt{self.sensor_data.lora_packet_count}".encode().ljust(20, b'\x00')

        if random.random() < 0.08:  # 8% chance
            self.sensor_data.radio433_packet_count += 1
            self.sensor_data.radio433_latest_data = f"433Pkt{self.sensor_data.radio433_packet_count}".encode().ljust(20, b'\x00')

        # Simulate slight pressure/temp variations
        self.sensor_data.pressure_hpa += random.uniform(-0.1, 0.1)
        self.sensor_data.temperature_c += random.uniform(-0.05, 0.05)
        self.sensor_data.altitude_m = 44330.0 * (1.0 - (self.sensor_data.pressure_hpa / 1013.25) ** 0.19029495)

        # Simulate current/voltage variations
        self.sensor_data.current_a = 0.5 + random.uniform(-0.02, 0.02)
        self.sensor_data.voltage_v = 12.3 + random.uniform(-0.1, 0.1)
        self.sensor_data.power_w = self.sensor_data.current_a * self.sensor_data.voltage_v

        # Update uptime
        self.sensor_data.uptime_seconds = int(time.time() - self.start_time)

    def pack_lora_data(self):
        """Pack LoRa data into binary format (WireLoRa_t)"""
        data = self.sensor_data.lora_latest_data[:64].ljust(64, b'\x00')
        return struct.pack(
            '<BHhf B 64s',  # version, packet_count, rssi_dbm, snr_db, latest_len, latest_data
            1,  # version
            self.sensor_data.lora_packet_count,
            self.sensor_data.lora_rssi,
            self.sensor_data.lora_snr,
            len(self.sensor_data.lora_latest_data),
            data
        )

    def pack_433_data(self):
        """Pack 433MHz data into binary format (Wire433_t)"""
        data = self.sensor_data.radio433_latest_data[:64].ljust(64, b'\x00')
        return struct.pack(
            '<BHh B 64s',  # version, packet_count, rssi_dbm, latest_len, latest_data
            1,  # version
            self.sensor_data.radio433_packet_count,
            self.sensor_data.radio433_rssi,
            len(self.sensor_data.radio433_latest_data),
            data
        )

    def pack_barometer_data(self):
        """Pack barometer data into binary format (WireBarometer_t)"""
        timestamp_ms = int(self.sensor_data.uptime_seconds * 1000)
        return struct.pack(
            '<B I fff',  # version, timestamp_ms, pressure_hpa, temperature_c, altitude_m
            1,  # version
            timestamp_ms,
            self.sensor_data.pressure_hpa,
            self.sensor_data.temperature_c,
            self.sensor_data.altitude_m
        )

    def pack_current_data(self):
        """Pack current sensor data into binary format (WireCurrent_t)"""
        timestamp_ms = int(self.sensor_data.uptime_seconds * 1000)
        return struct.pack(
            '<B I fff h',  # version, timestamp_ms, current_a, voltage_v, power_w, raw_adc
            1,  # version
            timestamp_ms,
            self.sensor_data.current_a,
            self.sensor_data.voltage_v,
            self.sensor_data.power_w,
            self.sensor_data.raw_adc
        )

    def pack_status(self):
        """Pack system status into binary format (WireStatus_t)"""
        # Build flags bitfield
        flags = 0
        if self.sensor_data.lora_online: flags |= (1 << 0)
        if self.sensor_data.radio433_online: flags |= (1 << 1)
        if self.sensor_data.barometer_online: flags |= (1 << 2)
        if self.sensor_data.current_online: flags |= (1 << 3)
        flags |= (1 << 4)  # pi_connected = true (we're connected!)

        return struct.pack(
            '<B I BB HH I I B',  # version, uptime, state, flags, pkt_lora, pkt_433, wakeup, heap, chip_rev
            1,  # version
            self.sensor_data.uptime_seconds,
            self.sensor_data.system_state,
            flags,
            self.sensor_data.lora_packet_count,
            self.sensor_data.radio433_packet_count,
            self.sensor_data.wakeup_time,
            self.sensor_data.free_heap,
            self.sensor_data.chip_revision
        )

    def send_response(self, payload):
        """Send binary response with protocol framing"""
        if len(payload) > 255:
            print(f"ERROR: Payload too large ({len(payload)} bytes)")
            return

        frame = bytes([HELLO_BYTE, PERIPHERAL_ID_SYSTEM, len(payload)]) + payload + bytes([GOODBYE_BYTE])
        self.serial.write(frame)
        self.serial.flush()
        print(f"  -> Sent response: {len(payload)} bytes")

    def send_error_response(self, error_msg):
        """Send error response"""
        msg_bytes = error_msg.encode('ascii')[:60]
        payload = struct.pack('<BB B', 1, 1, len(msg_bytes)) + msg_bytes  # version, error_code, len, msg
        self.send_response(payload)
        print(f"  -> Sent error: {error_msg}")

    def process_command(self, command):
        """Process received command and send appropriate response"""
        print(f"Processing command: 0x{command:02X}")

        self.update_sensor_data()  # Update fake sensor readings

        if command == CMD_GET_LORA_DATA:
            print("  Command: GET_LORA_DATA")
            payload = self.pack_lora_data()
            self.send_response(payload)

        elif command == CMD_GET_433_DATA:
            print("  Command: GET_433_DATA")
            payload = self.pack_433_data()
            self.send_response(payload)

        elif command == CMD_GET_BAROMETER_DATA:
            print("  Command: GET_BAROMETER_DATA")
            payload = self.pack_barometer_data()
            self.send_response(payload)

        elif command == CMD_GET_CURRENT_DATA:
            print("  Command: GET_CURRENT_DATA")
            payload = self.pack_current_data()
            self.send_response(payload)

        elif command == CMD_GET_ALL_DATA:
            print("  Command: GET_ALL_DATA")
            # Concatenate all data types
            payload = (self.pack_lora_data() +
                      self.pack_433_data() +
                      self.pack_barometer_data() +
                      self.pack_current_data())
            self.send_response(payload)

        elif command == CMD_GET_STATUS:
            print("  Command: GET_STATUS")
            payload = self.pack_status()
            self.send_response(payload)

        else:
            print(f"  ERROR: Unknown command 0x{command:02X}")
            self.send_error_response(f"Unknown command: 0x{command:02X}")

    def process(self):
        """Main processing loop - implements state machine"""
        if self.serial.in_waiting > 0:
            byte_val = ord(self.serial.read(1))

            if self.state == CommState.WAIT_FOR_HELLO:
                if byte_val == HELLO_BYTE:
                    self.state = CommState.WAIT_FOR_PERIPHERAL_ID
                    print(f"Received HELLO (0x{byte_val:02X})")
                else:
                    print(f"Expected HELLO, got 0x{byte_val:02X}")

            elif self.state == CommState.WAIT_FOR_PERIPHERAL_ID:
                self.peripheral_id = byte_val
                self.state = CommState.WAIT_FOR_LENGTH
                print(f"Peripheral ID: 0x{byte_val:02X}")

            elif self.state == CommState.WAIT_FOR_LENGTH:
                self.expected_length = byte_val
                self.message_buffer = bytearray()
                if self.expected_length > 0:
                    self.state = CommState.WAIT_FOR_MESSAGE_DATA
                else:
                    self.state = CommState.WAIT_FOR_GOODBYE
                print(f"Message length: {byte_val} bytes")

            elif self.state == CommState.WAIT_FOR_MESSAGE_DATA:
                self.message_buffer.append(byte_val)
                if len(self.message_buffer) >= self.expected_length:
                    self.state = CommState.WAIT_FOR_GOODBYE
                    print(f"Received message data ({len(self.message_buffer)} bytes)")

            elif self.state == CommState.WAIT_FOR_GOODBYE:
                if byte_val == GOODBYE_BYTE:
                    print(f"Received GOODBYE (0x{byte_val:02X})")
                    # Process the command
                    if self.peripheral_id == PERIPHERAL_ID_SYSTEM:
                        if len(self.message_buffer) >= 1:
                            command = self.message_buffer[0]
                            self.process_command(command)
                        else:
                            self.send_error_response("No command byte")
                    else:
                        self.send_error_response(f"Unknown peripheral ID: 0x{self.peripheral_id:02X}")
                else:
                    print(f"Expected GOODBYE, got 0x{byte_val:02X}")
                    self.send_error_response("Invalid packet format")

                # Reset state
                self.state = CommState.WAIT_FOR_HELLO
                self.message_buffer = bytearray()

    def run(self):
        """Main run loop"""
        print("\n=== ESP32 Simulator Ready ===")
        print("Listening for commands...")
        print("Press Ctrl+C to exit\n")

        try:
            while True:
                self.process()
                time.sleep(0.01)  # Small delay
        except KeyboardInterrupt:
            print("\n\nShutting down simulator...")
            self.serial.close()


def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage: python esp32_simulator.py <serial_port>")
        print("Example (Windows): python esp32_simulator.py COM10")
        print("Example (Linux): python esp32_simulator.py /dev/pts/2")
        sys.exit(1)

    serial_port = sys.argv[1]
    simulator = ESP32Simulator(serial_port)
    simulator.run()


if __name__ == "__main__":
    main()
