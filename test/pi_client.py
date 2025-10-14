#!/usr/bin/env python3
"""
Raspberry Pi Client for ESP32 Ground Station
Sends commands and receives binary data from the ESP32
This script will eventually run on the actual Raspberry Pi
"""

import serial
import struct
import time
from enum import IntEnum


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


class PiClient:
    """Raspberry Pi client for communicating with ESP32"""

    def __init__(self, serial_port, baud_rate=115200, timeout=2.0):
        """Initialize client with serial port"""
        self.serial = serial.Serial(serial_port, baud_rate, timeout=timeout)
        time.sleep(0.5)  # Wait for connection to stabilize
        print(f"Connected to ESP32 on {serial_port}")

    def send_command(self, command, payload=b''):
        """Send a command to ESP32 with protocol framing"""
        message = bytes([command]) + payload
        if len(message) > 255:
            raise ValueError(f"Message too large: {len(message)} bytes")

        frame = bytes([HELLO_BYTE, PERIPHERAL_ID_SYSTEM, len(message)]) + message + bytes([GOODBYE_BYTE])

        self.serial.write(frame)
        self.serial.flush()
        print(f"\n>>> Sent command: 0x{command:02X} ({len(message)} bytes)")

    def receive_response(self):
        """Receive and parse a response from ESP32"""
        # Read HELLO byte
        hello = self.serial.read(1)
        if not hello or hello[0] != HELLO_BYTE:
            raise ValueError(f"Expected HELLO (0x{HELLO_BYTE:02X}), got {hello.hex() if hello else 'nothing'}")

        # Read peripheral ID
        peripheral_id = ord(self.serial.read(1))

        # Read length
        length = ord(self.serial.read(1))

        # Read payload
        payload = self.serial.read(length)
        if len(payload) != length:
            raise ValueError(f"Expected {length} bytes, got {len(payload)}")

        # Read GOODBYE byte
        goodbye = self.serial.read(1)
        if not goodbye or goodbye[0] != GOODBYE_BYTE:
            raise ValueError(f"Expected GOODBYE (0x{GOODBYE_BYTE:02X}), got {goodbye.hex() if goodbye else 'nothing'}")

        print(f"<<< Received response: {length} bytes from peripheral 0x{peripheral_id:02X}")
        return payload

    def unpack_lora_data(self, payload):
        """Unpack LoRa data (WireLoRa_t)"""
        # struct: <BHhf B 64s
        if len(payload) < 74:
            raise ValueError(f"LoRa payload too small: {len(payload)} bytes")

        version, packet_count, rssi_dbm, snr_db, latest_len = struct.unpack('<BHhfB', payload[:11])
        latest_data = payload[11:11+64]

        actual_data = latest_data[:latest_len]

        print(f"  LoRa Data (v{version}):")
        print(f"    Packet Count: {packet_count}")
        print(f"    RSSI: {rssi_dbm} dBm")
        print(f"    SNR: {snr_db:.2f} dB")
        print(f"    Latest Data ({latest_len} bytes): {actual_data[:20].hex()}..." if latest_len > 0 else "    No data")

        return {
            'version': version,
            'packet_count': packet_count,
            'rssi_dbm': rssi_dbm,
            'snr_db': snr_db,
            'latest_data': actual_data
        }

    def unpack_433_data(self, payload):
        """Unpack 433MHz data (Wire433_t)"""
        # struct: <BHh B 64s
        if len(payload) < 70:
            raise ValueError(f"433MHz payload too small: {len(payload)} bytes")

        version, packet_count, rssi_dbm, latest_len = struct.unpack('<BHhB', payload[:7])
        latest_data = payload[7:7+64]

        actual_data = latest_data[:latest_len]

        print(f"  433MHz Data (v{version}):")
        print(f"    Packet Count: {packet_count}")
        print(f"    RSSI: {rssi_dbm} dBm")
        print(f"    Latest Data ({latest_len} bytes): {actual_data[:20].hex()}..." if latest_len > 0 else "    No data")

        return {
            'version': version,
            'packet_count': packet_count,
            'rssi_dbm': rssi_dbm,
            'latest_data': actual_data
        }

    def unpack_barometer_data(self, payload):
        """Unpack barometer data (WireBarometer_t)"""
        # struct: <B I fff
        if len(payload) < 17:
            raise ValueError(f"Barometer payload too small: {len(payload)} bytes")

        version, timestamp_ms, pressure_hpa, temperature_c, altitude_m = struct.unpack('<BIfff', payload[:17])

        print(f"  Barometer Data (v{version}):")
        print(f"    Timestamp: {timestamp_ms} ms")
        print(f"    Pressure: {pressure_hpa:.2f} hPa")
        print(f"    Temperature: {temperature_c:.2f} °C")
        print(f"    Altitude: {altitude_m:.2f} m")

        return {
            'version': version,
            'timestamp_ms': timestamp_ms,
            'pressure_hpa': pressure_hpa,
            'temperature_c': temperature_c,
            'altitude_m': altitude_m
        }

    def unpack_current_data(self, payload):
        """Unpack current sensor data (WireCurrent_t)"""
        # struct: <B I fff h
        if len(payload) < 19:
            raise ValueError(f"Current payload too small: {len(payload)} bytes")

        version, timestamp_ms, current_a, voltage_v, power_w, raw_adc = struct.unpack('<BIfffh', payload[:19])

        print(f"  Current Sensor Data (v{version}):")
        print(f"    Timestamp: {timestamp_ms} ms")
        print(f"    Current: {current_a:.3f} A")
        print(f"    Voltage: {voltage_v:.3f} V")
        print(f"    Power: {power_w:.3f} W")
        print(f"    Raw ADC: {raw_adc}")

        return {
            'version': version,
            'timestamp_ms': timestamp_ms,
            'current_a': current_a,
            'voltage_v': voltage_v,
            'power_w': power_w,
            'raw_adc': raw_adc
        }

    def unpack_status(self, payload):
        """Unpack system status (WireStatus_t)"""
        # struct: <B I BB HH I I B
        if len(payload) < 20:
            raise ValueError(f"Status payload too small: {len(payload)} bytes")

        data = struct.unpack('<BIBBHHIIB', payload[:20])
        version, uptime_sec, sys_state, flags, pkt_lora, pkt_433, wakeup_time, free_heap, chip_rev = data

        # Parse flags
        lora_online = bool(flags & (1 << 0))
        radio433_online = bool(flags & (1 << 1))
        barometer_online = bool(flags & (1 << 2))
        current_online = bool(flags & (1 << 3))
        pi_connected = bool(flags & (1 << 4))

        print(f"  System Status (v{version}):")
        print(f"    Uptime: {uptime_sec} seconds")
        print(f"    System State: {sys_state}")
        print(f"    LoRa: {'Online' if lora_online else 'Offline'} ({pkt_lora} packets)")
        print(f"    433MHz: {'Online' if radio433_online else 'Offline'} ({pkt_433} packets)")
        print(f"    Barometer: {'Online' if barometer_online else 'Offline'}")
        print(f"    Current Sensor: {'Online' if current_online else 'Offline'}")
        print(f"    Pi Connected: {pi_connected}")
        print(f"    Free Heap: {free_heap} bytes")
        print(f"    Chip Revision: {chip_rev}")

        return {
            'version': version,
            'uptime_seconds': uptime_sec,
            'system_state': sys_state,
            'lora_online': lora_online,
            'radio433_online': radio433_online,
            'barometer_online': barometer_online,
            'current_online': current_online,
            'pi_connected': pi_connected,
            'packet_count_lora': pkt_lora,
            'packet_count_433': pkt_433,
            'wakeup_time': wakeup_time,
            'free_heap': free_heap,
            'chip_revision': chip_rev
        }

    def get_lora_data(self):
        """Request and parse LoRa data"""
        self.send_command(CMD_GET_LORA_DATA)
        payload = self.receive_response()
        return self.unpack_lora_data(payload)

    def get_433_data(self):
        """Request and parse 433MHz data"""
        self.send_command(CMD_GET_433_DATA)
        payload = self.receive_response()
        return self.unpack_433_data(payload)

    def get_barometer_data(self):
        """Request and parse barometer data"""
        self.send_command(CMD_GET_BAROMETER_DATA)
        payload = self.receive_response()
        return self.unpack_barometer_data(payload)

    def get_current_data(self):
        """Request and parse current sensor data"""
        self.send_command(CMD_GET_CURRENT_DATA)
        payload = self.receive_response()
        return self.unpack_current_data(payload)

    def get_status(self):
        """Request and parse system status"""
        self.send_command(CMD_GET_STATUS)
        payload = self.receive_response()
        return self.unpack_status(payload)

    def get_all_data(self):
        """Request all data at once"""
        self.send_command(CMD_GET_ALL_DATA)
        payload = self.receive_response()

        print("\n=== ALL DATA ===")
        offset = 0

        # LoRa (74 bytes)
        lora = self.unpack_lora_data(payload[offset:offset+74])
        offset += 74

        # 433MHz (70 bytes)
        radio433 = self.unpack_433_data(payload[offset:offset+70])
        offset += 70

        # Barometer (17 bytes)
        barometer = self.unpack_barometer_data(payload[offset:offset+17])
        offset += 17

        # Current (19 bytes)
        current = self.unpack_current_data(payload[offset:offset+19])

        return {
            'lora': lora,
            '433mhz': radio433,
            'barometer': barometer,
            'current': current
        }

    def close(self):
        """Close serial connection"""
        self.serial.close()
        print("Connection closed")


def interactive_test(client):
    """Interactive test menu"""
    while True:
        print("\n" + "="*50)
        print("ESP32 Ground Station Test Client")
        print("="*50)
        print("1. Get LoRa Data")
        print("2. Get 433MHz Data")
        print("3. Get Barometer Data")
        print("4. Get Current Sensor Data")
        print("5. Get System Status")
        print("6. Get All Data")
        print("7. Run Continuous Test (all sensors, 1 sec interval)")
        print("0. Exit")
        print("="*50)

        choice = input("Enter choice: ").strip()

        try:
            if choice == '1':
                client.get_lora_data()
            elif choice == '2':
                client.get_433_data()
            elif choice == '3':
                client.get_barometer_data()
            elif choice == '4':
                client.get_current_data()
            elif choice == '5':
                client.get_status()
            elif choice == '6':
                client.get_all_data()
            elif choice == '7':
                print("\nRunning continuous test... (Press Ctrl+C to stop)")
                try:
                    while True:
                        print("\n--- Polling all sensors ---")
                        client.get_status()
                        client.get_lora_data()
                        client.get_433_data()
                        client.get_barometer_data()
                        client.get_current_data()
                        time.sleep(1)
                except KeyboardInterrupt:
                    print("\nStopped continuous test")
            elif choice == '0':
                break
            else:
                print("Invalid choice")

        except Exception as e:
            print(f"\nERROR: {e}")
            import traceback
            traceback.print_exc()


def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage: python pi_client.py <serial_port>")
        print("Example (Windows): python pi_client.py COM11")
        print("Example (Linux): python pi_client.py /dev/pts/3")
        sys.exit(1)

    serial_port = sys.argv[1]

    try:
        client = PiClient(serial_port)
        interactive_test(client)
    except KeyboardInterrupt:
        print("\n\nExiting...")
    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            client.close()
        except:
            pass


if __name__ == "__main__":
    main()
