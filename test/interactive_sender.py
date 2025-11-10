#!/usr/bin/env python3
"""
interactive_sender.py - Hybrid Protocol Interactive Test Tool

A Tkinter GUI to send commands to ESP32 using the hybrid protocol and
display responses in real-time.

HYBRID PROTOCOL:
================
Wire format (pure binary): <AA55[PID][LEN][PAYLOAD][CHK]55AA>

Example (hex): 3C AA 55 01 01 00 01 55 AA 3E
               └─ PID=01 (LoRa), LEN=01, CMD=00 (GET_ALL), CHK=00

Peripheral IDs:
  0x00 - SYSTEM (ESP32 control)
  0x01 - LORA_915 (915MHz LoRa)
  0x02 - RADIO_433 (433MHz Radio)
  0x03 - BAROMETER (MS5607)
  0x04 - CURRENT (Current/voltage sensor)
  0xFF - ALL (get data from all peripherals)

Generic Commands (0x00-0x0F):
  0x00 - CMD_GET_ALL (get all data from peripheral)
  0x01 - CMD_GET_STATUS (get status/health)
  0x02 - CMD_SET_POLL_RATE (start autonomous polling, payload: interval_ms)
  0x03 - CMD_STOP_POLL (stop autonomous polling)

System Commands (0x20-0x2F, PID=0x00 only):
  0x20 - CMD_SYSTEM_STATUS (get system status)
  0x21 - CMD_SYSTEM_WAKEUP (wake system)
  0x22 - CMD_SYSTEM_SLEEP (enter sleep)
  0x23 - CMD_SYSTEM_RESET (reset ESP32)

Features:
  - Serial port selection & connect/disconnect
  - Peripheral + Command dropdowns
  - Start/stop autonomous polling per peripheral
  - Real-time response display
  - Auto-clear log before TX for readability
  - Save/load presets

Requirements:
  pip install pyserial

Author: Claude Code
Date: 2025
"""

import sys
import time
import threading
import struct
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports

# ====================================================================
# PROTOCOL CONSTANTS
# ====================================================================

# Peripheral IDs
PERIPHERALS = {
    "SYSTEM":      0x00,
    "LORA_915":    0x01,
    "RADIO_433":   0x02,
    "BAROMETER":   0x03,
    "CURRENT":     0x04,
    "ALL":         0xFF,
}

# Generic commands (work for all peripherals)
GENERIC_COMMANDS = {
    "GET_ALL":        0x00,
    "GET_STATUS":     0x01,
    "SET_POLL_RATE":  0x02,
    "STOP_POLL":      0x03,
}

# System commands (only for PERIPHERAL_ID=0x00)
SYSTEM_COMMANDS = {
    "SYSTEM_STATUS":      0x20,
    "SYSTEM_WAKEUP":      0x21,
    "SYSTEM_SLEEP":       0x22,
    "SYSTEM_RESET":       0x23,
    "SYSTEM_PERF":        0x24,  # Toggle performance stats (payload: 0=off, 1=on)
    "SYSTEM_STATS":       0x25,  # Get transfer statistics
    "SYSTEM_STATS_RESET": 0x26,  # Reset transfer statistics
}

# Data structure sizes (for validation and parsing)
SIZE_HEARTBEAT = 6   # WireHeartbeat_t
SIZE_STATUS = 20     # WireStatus_t
SIZE_LORA = 74       # WireLoRa_t
SIZE_433 = 74        # Wire433_t (same as LoRa)
SIZE_BAROMETER = 17  # WireBarometer_t
SIZE_CURRENT = 19    # WireCurrent_t
SIZE_TRANSFER_STATS = 41  # WireTransferStats_t (1 + 5*8)

# ====================================================================
# DATA STRUCTURE UNPACKING FUNCTIONS
# ====================================================================

def unpack_heartbeat(data):
    """Unpack WireHeartbeat_t (6 bytes): version(1), uptime(4), state(1)"""
    result = {'partial': len(data) != SIZE_HEARTBEAT, 'actual_length': len(data), 'expected_length': SIZE_HEARTBEAT}
    offset = 0

    if len(data) >= offset + 1:
        result['version'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
    if len(data) >= offset + 4:
        result['uptime_seconds'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 1:
        result['system_state'] = struct.unpack('<B', data[offset:offset+1])[0]

    return result

def unpack_status(data):
    """Unpack WireStatus_t (20 bytes): version(1), uptime(4), state(1), flags(1), pkt_lora(2), pkt_433(2), wakeup_time(4), heap(4), chip_rev(1)"""
    result = {'partial': len(data) != SIZE_STATUS, 'actual_length': len(data), 'expected_length': SIZE_STATUS}
    offset = 0

    if len(data) >= offset + 1:
        result['version'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
    if len(data) >= offset + 4:
        result['uptime_seconds'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 1:
        result['system_state'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
    if len(data) >= offset + 1:
        result['sensor_flags'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
    if len(data) >= offset + 2:
        result['pkt_count_lora'] = struct.unpack('<H', data[offset:offset+2])[0]
        offset += 2
    if len(data) >= offset + 2:
        result['pkt_count_433'] = struct.unpack('<H', data[offset:offset+2])[0]
        offset += 2
    if len(data) >= offset + 4:
        result['wakeup_time'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 4:
        result['heap_free'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 1:
        result['chip_revision'] = struct.unpack('<B', data[offset:offset+1])[0]

    return result

def unpack_lora_data(data):
    """Unpack WireLoRa_t (74 bytes): version(1), pkt_count(2), rssi(2), snr(4), len(1), data(64)"""
    result = {'partial': len(data) != SIZE_LORA, 'actual_length': len(data), 'expected_length': SIZE_LORA}
    offset = 0

    if len(data) >= offset + 1:
        result['version'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
    if len(data) >= offset + 2:
        result['packet_count'] = struct.unpack('<H', data[offset:offset+2])[0]
        offset += 2
    if len(data) >= offset + 2:
        result['rssi'] = struct.unpack('<h', data[offset:offset+2])[0]
        offset += 2
    if len(data) >= offset + 4:
        result['snr'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 1:
        result['payload_length'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
    if len(data) >= offset + 64:
        payload_data = data[offset:offset+64]
        if 'payload_length' in result:
            result['payload'] = payload_data[:result['payload_length']].hex().upper()
        else:
            result['payload'] = payload_data.hex().upper()

    return result

def unpack_barometer_data(data):
    """Unpack WireBarometer_t (17 bytes): version(1), timestamp(4), pressure(4), temp(4), altitude(4)"""
    result = {'partial': len(data) != SIZE_BAROMETER, 'actual_length': len(data), 'expected_length': SIZE_BAROMETER}
    offset = 0

    if len(data) >= offset + 1:
        result['version'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
    if len(data) >= offset + 4:
        result['timestamp'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 4:
        result['pressure_pa'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 4:
        result['temperature_c'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 4:
        result['altitude_m'] = struct.unpack('<f', data[offset:offset+4])[0]

    return result

def unpack_current_data(data):
    """Unpack WireCurrent_t (19 bytes): version(1), timestamp(4), current(4), voltage(4), power(4), raw_adc(2)"""
    result = {'partial': len(data) != SIZE_CURRENT, 'actual_length': len(data), 'expected_length': SIZE_CURRENT}
    offset = 0

    if len(data) >= offset + 1:
        result['version'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
    if len(data) >= offset + 4:
        result['timestamp'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 4:
        result['current_a'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 4:
        result['voltage_v'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 4:
        result['power_w'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
    if len(data) >= offset + 2:
        result['raw_adc'] = struct.unpack('<h', data[offset:offset+2])[0]

    return result

def unpack_transfer_stats(data):
    """Unpack WireTransferStats_t (41 bytes): version(1), stats[5] (each: packets(4), bytes(4))"""
    result = {'partial': len(data) != SIZE_TRANSFER_STATS, 'actual_length': len(data), 'expected_length': SIZE_TRANSFER_STATS}
    offset = 0

    if len(data) >= offset + 1:
        result['version'] = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1

    # Unpack stats for each peripheral (SYSTEM, LORA_915, RADIO_433, BAROMETER, CURRENT)
    peripheral_names = ['SYSTEM', 'LORA_915', 'RADIO_433', 'BAROMETER', 'CURRENT']
    result['peripherals'] = {}

    for name in peripheral_names:
        if len(data) >= offset + 8:  # 4 bytes packets + 4 bytes bytes
            total_packets = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            total_bytes = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            result['peripherals'][name] = {
                'total_packets': total_packets,
                'total_bytes': total_bytes
            }

    return result

# ====================================================================
# PROTOCOL FUNCTIONS
# ====================================================================

def calculate_checksum(pid, length, payload):
    """Calculate XOR checksum: PID ^ LEN ^ PAYLOAD bytes"""
    chk = pid ^ length
    for b in payload:
        chk ^= b
    return chk & 0xFF

def encode_hybrid_frame(peripheral_id, payload):
    """
    Encode hybrid protocol frame (pure binary).

    Args:
        peripheral_id: Peripheral ID (0x00-0xFF)
        payload: Payload bytes (includes command as first byte)

    Returns:
        Binary frame: <AA55[PID][LEN][PAYLOAD][CHK]55AA>
    """
    if len(payload) > 255:
        raise ValueError(f"Payload too long: {len(payload)} bytes (max 255)")

    # Calculate checksum
    checksum = calculate_checksum(peripheral_id, len(payload), payload)

    # Build frame: <AA55[PID][LEN][PAYLOAD][CHK]55AA> (pure binary)
    frame = b'<'
    frame += b'\xAA\x55'  # Start markers
    frame += bytes([peripheral_id])  # PID
    frame += bytes([len(payload)])  # Length
    frame += payload  # Payload (raw bytes)
    frame += bytes([checksum])  # Checksum
    frame += b'\x55\xAA'  # End markers
    frame += b'>'  # End marker

    return frame

def format_frame_hex(frame):
    """Format binary frame as readable hex string"""
    return ' '.join(f'{b:02X}' for b in frame)

def decode_hybrid_frame(line):
    """
    Decode hybrid protocol frame (pure binary).

    Args:
        line: Bytes received from serial

    Returns:
        (pid, payload) if valid, (None, None) if invalid
    """
    try:
        # Minimum frame: < AA 55 PID LEN CHK 55 AA > = 9 bytes
        if len(line) < 9:
            return None, None

        # Check frame markers
        if line[0] != ord('<') or line[-1] != ord('>'):
            return None, None

        # Check start markers: AA55
        if line[1] != 0xAA or line[2] != 0x55:
            return None, None

        # Parse PID and length
        pid = line[3]
        length = line[4]

        # Calculate expected length: < AA 55 PID LEN [payload...] CHK 55 AA > = 9 + length
        expected_len = 9 + length
        if len(line) != expected_len:
            return None, None

        # Extract payload
        payload = line[5:5 + length]

        # Parse checksum
        rx_chk = line[5 + length]

        # Verify checksum
        calc_chk = calculate_checksum(pid, length, payload)
        if rx_chk != calc_chk:
            return None, None

        # Check end markers: 55 AA
        if line[6 + length] != 0x55 or line[7 + length] != 0xAA:
            return None, None

        return pid, bytes(payload)

    except Exception as e:
        return None, None

# ====================================================================
# GUI APPLICATION
# ====================================================================

class HybridProtocolGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Hybrid Protocol - Interactive Test Tool")
        self.root.geometry("1000x750")

        self.ser = None
        self.connected = False
        self.rx_thread = None
        self.rx_running = False

        # Auto-clear option
        self.auto_clear_var = tk.BooleanVar(value=True)

        # Autonomous polling state
        self.autonomous_enabled = {}  # {peripheral_id: bool}

        # Performance stats
        self.perf_stats = {
            'loops_per_sec': 0.0,
            'ms_per_loop': 0.0,
            'queue_size': 0
        }

        # Transfer statistics
        self.transfer_stats = None  # Will be populated when stats are requested
        self.stats_window = None    # Reference to stats popup window

        self.create_widgets()
        self.refresh_ports()

    def create_widgets(self):
        # ====================================================================
        # Connection Frame
        # ====================================================================
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.port_combo = ttk.Combobox(conn_frame, width=20, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, padx=5, sticky=tk.W)
        self.baud_var = tk.StringVar(value="921600")
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, width=10,
                                  values=["9600", "115200", "230400", "460800", "921600"], state='readonly')
        baud_combo.grid(row=0, column=4, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5)

        self.status_label = ttk.Label(conn_frame, text="● Disconnected", foreground="red")
        self.status_label.grid(row=0, column=6, padx=10)

        # ====================================================================
        # Message Builder Frame
        # ====================================================================
        cmd_frame = ttk.LabelFrame(self.root, text="Message Builder", padding=10)
        cmd_frame.pack(fill=tk.X, padx=10, pady=5)

        # Row 1: Peripheral and Command
        ttk.Label(cmd_frame, text="Peripheral:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.peripheral_var = tk.StringVar(value="SYSTEM")
        peripheral_combo = ttk.Combobox(cmd_frame, textvariable=self.peripheral_var, width=15,
                                       values=list(PERIPHERALS.keys()), state='readonly')
        peripheral_combo.grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)
        peripheral_combo.bind('<<ComboboxSelected>>', self.on_peripheral_changed)

        ttk.Label(cmd_frame, text="Command:").grid(row=0, column=2, padx=5, pady=2, sticky=tk.W)
        self.command_var = tk.StringVar(value="GET_ALL")
        self.command_combo = ttk.Combobox(cmd_frame, textvariable=self.command_var, width=20,
                                         state='readonly')
        self.command_combo.grid(row=0, column=3, padx=5, pady=2, sticky=tk.W)
        self.update_command_list()

        ttk.Button(cmd_frame, text="Send Command", command=self.send_command,
                  width=15).grid(row=0, column=4, padx=10, pady=2)

        # Row 2: Payload (for SET_POLL_RATE)
        ttk.Label(cmd_frame, text="Poll Interval (ms):").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.interval_var = tk.StringVar(value="1000")
        ttk.Entry(cmd_frame, textvariable=self.interval_var, width=10).grid(row=1, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cmd_frame, text="(used for SET_POLL_RATE command)").grid(row=1, column=2, columnspan=2, padx=5, pady=2, sticky=tk.W)

        # ====================================================================
        # Quick Actions Frame
        # ====================================================================
        quick_frame = ttk.LabelFrame(self.root, text="Quick Actions", padding=10)
        quick_frame.pack(fill=tk.X, padx=10, pady=5)

        row = 0
        col = 0
        buttons = [
            ("Get System Status", lambda: self.quick_send("SYSTEM", "SYSTEM_STATUS")),
            ("Get All Peripherals", lambda: self.quick_send("ALL", "GET_ALL")),
            ("Get LoRa", lambda: self.quick_send("LORA_915", "GET_ALL")),
            ("Get Barometer", lambda: self.quick_send("BAROMETER", "GET_ALL")),
            ("Get Current", lambda: self.quick_send("CURRENT", "GET_ALL")),
            ("Start LoRa Poll (1s)", lambda: self.start_polling("LORA_915", 1000)),
            ("Start Baro Poll (500ms)", lambda: self.start_polling("BAROMETER", 500)),
            ("Stop All Polling", self.stop_all_polling),
            ("Enable Perf Stats", lambda: self.toggle_perf_stats(True)),
            ("Disable Perf Stats", lambda: self.toggle_perf_stats(False)),
            ("Transfer Statistics", self.show_transfer_stats),
        ]

        for text, cmd in buttons:
            ttk.Button(quick_frame, text=text, command=cmd, width=20).grid(row=row, column=col, padx=5, pady=2)
            col += 1
            if col >= 4:
                col = 0
                row += 1

        # ====================================================================
        # Log Frame
        # ====================================================================
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Log text area
        self.response_text = scrolledtext.ScrolledText(log_frame, height=25, wrap=tk.WORD, font=('Consolas', 9))
        self.response_text.pack(fill=tk.BOTH, expand=True)

        # Add tags for colored text
        self.response_text.tag_config("tx", foreground="blue", font=('Consolas', 9, 'bold'))
        self.response_text.tag_config("rx", foreground="green", font=('Consolas', 9))
        self.response_text.tag_config("error", foreground="red", font=('Consolas', 9, 'bold'))
        self.response_text.tag_config("info", foreground="purple", font=('Consolas', 9))

        # ====================================================================
        # Log Control Frame
        # ====================================================================
        log_ctrl_frame = ttk.Frame(self.root)
        log_ctrl_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Checkbutton(log_ctrl_frame, text="Auto-clear log before TX",
                       variable=self.auto_clear_var).pack(side=tk.LEFT, padx=5)
        ttk.Button(log_ctrl_frame, text="Clear Log", command=self.clear_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(log_ctrl_frame, text="Copy Log", command=self.copy_log).pack(side=tk.LEFT, padx=5)

        # ====================================================================
        # Performance Stats Frame
        # ====================================================================
        perf_frame = ttk.Frame(self.root)
        perf_frame.pack(fill=tk.X, padx=10, pady=5)

        self.perf_label = ttk.Label(perf_frame,
                                    text="ESP32 Performance: --- loops/sec, --- ms/loop, Queue: ---",
                                    font=('Consolas', 9),
                                    foreground="blue")
        self.perf_label.pack(side=tk.LEFT, padx=5)

    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if not self.connected:
            port = self.port_combo.get()
            baud = int(self.baud_var.get())

            try:
                self.ser = serial.Serial(port, baud, timeout=1)
                time.sleep(2)  # Wait for ESP32 to reset
                self.ser.reset_input_buffer()
                self.connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="● Connected", foreground="green")
                self.log(f"[INFO] Connected to {port} @ {baud} baud", "info")

                # Start RX thread
                self.rx_running = True
                self.rx_thread = threading.Thread(target=self.rx_worker, daemon=True)
                self.rx_thread.start()

            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
        else:
            self.rx_running = False
            if self.ser:
                self.ser.close()
            self.connected = False
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="● Disconnected", foreground="red")
            self.log("[INFO] Disconnected", "info")

    def on_peripheral_changed(self, event=None):
        """Update command list when peripheral changes"""
        self.update_command_list()

    def update_command_list(self):
        """Update available commands based on selected peripheral"""
        peripheral = self.peripheral_var.get()

        if peripheral == "SYSTEM":
            # System peripheral: show both generic and system commands
            commands = list(GENERIC_COMMANDS.keys()) + list(SYSTEM_COMMANDS.keys())
        else:
            # Other peripherals: only generic commands
            commands = list(GENERIC_COMMANDS.keys())

        self.command_combo['values'] = commands
        if commands:
            self.command_combo.current(0)

    def send_command(self):
        """Send command based on GUI selections"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to serial port first")
            return

        # Auto-clear log if enabled
        if self.auto_clear_var.get():
            self.clear_log()

        peripheral_name = self.peripheral_var.get()
        command_name = self.command_var.get()

        peripheral_id = PERIPHERALS[peripheral_name]

        # Get command code
        if command_name in GENERIC_COMMANDS:
            command = GENERIC_COMMANDS[command_name]
        elif command_name in SYSTEM_COMMANDS:
            command = SYSTEM_COMMANDS[command_name]
        else:
            messagebox.showerror("Error", f"Unknown command: {command_name}")
            return

        # Build payload
        payload = bytearray([command])

        # Add interval for SET_POLL_RATE
        if command_name == "SET_POLL_RATE":
            try:
                interval = int(self.interval_var.get())
                payload.append(interval & 0xFF)  # Low byte
                payload.append((interval >> 8) & 0xFF)  # High byte
            except ValueError:
                messagebox.showerror("Error", "Invalid interval value")
                return

        # Encode and send
        frame = encode_hybrid_frame(peripheral_id, bytes(payload))
        self.ser.write(frame)

        # Log
        self.log(f"[TX] {peripheral_name} → {command_name}", "tx")
        self.log(f"     {format_frame_hex(frame)}", "tx")
        if command_name == "SET_POLL_RATE":
            interval_ms = int(self.interval_var.get())
            self.log(f"     Interval: {interval_ms}ms", "tx")

    def quick_send(self, peripheral_name, command_name):
        """Quick send helper for buttons"""
        self.peripheral_var.set(peripheral_name)
        self.command_var.set(command_name)
        self.update_command_list()
        self.send_command()

    def start_polling(self, peripheral_name, interval_ms):
        """Start autonomous polling for a peripheral"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to serial port first")
            return

        # Auto-clear log if enabled
        if self.auto_clear_var.get():
            self.clear_log()

        peripheral_id = PERIPHERALS[peripheral_name]
        command = GENERIC_COMMANDS["SET_POLL_RATE"]

        payload = bytearray([command])
        payload.append(interval_ms & 0xFF)
        payload.append((interval_ms >> 8) & 0xFF)

        frame = encode_hybrid_frame(peripheral_id, bytes(payload))
        self.ser.write(frame)

        self.autonomous_enabled[peripheral_id] = True
        self.log(f"[TX] {peripheral_name} → SET_POLL_RATE", "tx")
        self.log(f"     {format_frame_hex(frame)}", "tx")
        self.log(f"     Started polling every {interval_ms}ms", "info")

    def toggle_perf_stats(self, enable):
        """Enable or disable performance stats output"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to serial port first")
            return

        peripheral_id = PERIPHERALS["SYSTEM"]
        command = SYSTEM_COMMANDS["SYSTEM_PERF"]

        payload = bytearray([command])
        payload.append(1 if enable else 0)  # 1=enable, 0=disable

        frame = encode_hybrid_frame(peripheral_id, bytes(payload))
        self.ser.write(frame)

        self.log(f"[TX] SYSTEM → SYSTEM_PERF ({'ENABLE' if enable else 'DISABLE'})", "tx")
        self.log(f"     {format_frame_hex(frame)}", "tx")

    def stop_all_polling(self):
        """Stop autonomous polling for all peripherals"""
        if not self.connected:
            return

        # Auto-clear log if enabled
        if self.auto_clear_var.get():
            self.clear_log()

        for peripheral_name in PERIPHERALS.keys():
            peripheral_id = PERIPHERALS[peripheral_name]
            command = GENERIC_COMMANDS["STOP_POLL"]

            payload = bytes([command])
            frame = encode_hybrid_frame(peripheral_id, payload)
            self.ser.write(frame)

            self.autonomous_enabled[peripheral_id] = False

        self.log("[INFO] Stopped all autonomous polling", "info")

    def show_transfer_stats(self):
        """Show transfer statistics popup window"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to serial port first")
            return

        # Request stats from ESP32
        peripheral_id = PERIPHERALS["SYSTEM"]
        command = SYSTEM_COMMANDS["SYSTEM_STATS"]
        payload = bytes([command])
        frame = encode_hybrid_frame(peripheral_id, payload)
        self.ser.write(frame)

        self.log("[TX] SYSTEM → SYSTEM_STATS", "tx")
        self.log(f"     {format_frame_hex(frame)}", "tx")

        # Stats will be received via rx_worker and stored in self.transfer_stats
        # The popup will display when data arrives (handled in decode_payload)

    def open_stats_popup(self):
        """Open the statistics popup window with current stats"""
        if self.stats_window is not None and self.stats_window.winfo_exists():
            # Window already open, just bring to front and update
            self.stats_window.lift()
            self.update_stats_display()
            return

        # Create new popup window
        self.stats_window = tk.Toplevel(self.root)
        self.stats_window.title("Transfer Statistics")
        self.stats_window.geometry("400x300")

        # Title
        title_label = ttk.Label(self.stats_window, text="Data Transfer Statistics",
                                font=('Consolas', 12, 'bold'))
        title_label.pack(pady=10)

        # Stats display frame
        stats_frame = ttk.Frame(self.stats_window, padding=10)
        stats_frame.pack(fill=tk.BOTH, expand=True)

        # Create labels for each peripheral
        self.stats_labels = {}
        peripheral_names = ['SYSTEM', 'LORA_915', 'RADIO_433', 'BAROMETER', 'CURRENT']

        for i, name in enumerate(peripheral_names):
            # Peripheral name
            name_label = ttk.Label(stats_frame, text=f"{name}:", font=('Consolas', 10, 'bold'))
            name_label.grid(row=i, column=0, sticky=tk.W, padx=5, pady=5)

            # Stats text
            stats_label = ttk.Label(stats_frame, text="Loading...", font=('Consolas', 9))
            stats_label.grid(row=i, column=1, sticky=tk.W, padx=5, pady=5)
            self.stats_labels[name] = stats_label

        # Button frame
        button_frame = ttk.Frame(self.stats_window)
        button_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Button(button_frame, text="Refresh", command=self.show_transfer_stats).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Reset Stats", command=self.reset_transfer_stats).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Close", command=self.stats_window.destroy).pack(side=tk.RIGHT, padx=5)

        # Update display if we already have stats
        if self.transfer_stats:
            self.update_stats_display()

    def update_stats_display(self):
        """Update the statistics display in the popup window"""
        if not self.transfer_stats or not hasattr(self, 'stats_labels'):
            return

        peripherals = self.transfer_stats.get('peripherals', {})

        for name, label in self.stats_labels.items():
            if name in peripherals:
                stats = peripherals[name]
                packets = stats['total_packets']
                bytes_total = stats['total_bytes']
                # Format bytes nicely (KB, MB, etc.)
                if bytes_total < 1024:
                    bytes_str = f"{bytes_total} B"
                elif bytes_total < 1024 * 1024:
                    bytes_str = f"{bytes_total / 1024:.2f} KB"
                else:
                    bytes_str = f"{bytes_total / (1024 * 1024):.2f} MB"

                label.config(text=f"{packets:,} packets, {bytes_str}")
            else:
                label.config(text="No data")

    def reset_transfer_stats(self):
        """Reset transfer statistics on ESP32"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to serial port first")
            return

        # Confirm with user
        if not messagebox.askyesno("Reset Statistics",
                                   "Are you sure you want to reset all transfer statistics?\n\n"
                                   "This will reset the persistent counters stored on the ESP32."):
            return

        # Send reset command
        peripheral_id = PERIPHERALS["SYSTEM"]
        command = SYSTEM_COMMANDS["SYSTEM_STATS_RESET"]
        payload = bytes([command])
        frame = encode_hybrid_frame(peripheral_id, payload)
        self.ser.write(frame)

        self.log("[TX] SYSTEM → SYSTEM_STATS_RESET", "tx")
        self.log(f"     {format_frame_hex(frame)}", "tx")

        # Request updated stats after a short delay
        self.root.after(1000, self.show_transfer_stats)

    def rx_worker(self):
        """Background thread to receive and display responses"""
        frame_buffer = bytearray()

        while self.rx_running:
            try:
                # Only sleep if no data available
                if not self.ser or self.ser.in_waiting == 0:
                    time.sleep(0.001)  # 1ms sleep when idle
                    continue

                # Read one byte at a time, looking for complete frames ending with '>'
                byte = self.ser.read(1)
                if not byte:
                    continue

                frame_buffer.extend(byte)

                # Check if frame is complete (ends with '>')
                if byte[0] == ord('>'):
                    line = bytes(frame_buffer)
                    frame_buffer.clear()

                    # Check if it's a debug/info line (starts with '[')
                    line_str = line.decode('ascii', errors='ignore').strip()
                    if line_str.startswith('[PERF]'):
                        self.parse_perf_stats(line_str)
                        continue
                    elif line_str.startswith('['):
                        # All bracketed messages are debug/info - show in log
                        # Includes: [DEBUG], [AUTO], [RATE_LIMIT], [INFO], [SEND], etc.
                        self.log(line_str, "info")
                        continue

                    pid, payload = decode_hybrid_frame(line)

                    if pid is not None:
                        # Find peripheral name
                        peripheral_name = "UNKNOWN"
                        for name, pid_val in PERIPHERALS.items():
                            if pid_val == pid:
                                peripheral_name = name
                                break

                        self.log(f"[RX] {peripheral_name} (PID=0x{pid:02X}): {len(payload)} bytes", "rx")
                        self.log(f"     {format_frame_hex(line)}", "rx")

                        # Decode payload if possible
                        self.decode_payload(peripheral_name, payload)
                        self.log("")  # Blank line for readability
                    else:
                        # Not a valid frame, might be debug output
                        if line_str:
                            self.log(line_str, "info")

                # Handle case where frame starts with '[' (debug message)
                # These are text lines ending with '\n', not binary frames
                elif frame_buffer and frame_buffer[0] == ord('['):
                    # Check if we have a complete line (ends with \n)
                    if byte[0] in (ord('\n'), ord('\r')):
                        line_str = frame_buffer.decode('ascii', errors='ignore').strip()
                        frame_buffer.clear()

                        if line_str.startswith('[PERF]'):
                            self.parse_perf_stats(line_str)
                        elif line_str.startswith('['):
                            self.log(line_str, "info")

                # Prevent buffer overflow from malformed data
                if len(frame_buffer) > 512:
                    frame_buffer.clear()

            except Exception as e:
                if self.rx_running:  # Only log if not intentionally stopped
                    self.log(f"[ERROR] RX thread: {e}", "error")
                    time.sleep(0.1)

    def parse_perf_stats(self, line):
        """Parse performance stats from ESP32 and update GUI"""
        # Format: [PERF] 12345.6 loops/sec, 0.081 ms/loop, Queue: 3/8
        try:
            import re
            match = re.search(r'(\d+\.?\d*)\s+loops/sec,\s+(\d+\.?\d*)\s+ms/loop,\s+Queue:\s+(\d+)/8', line)
            if match:
                loops_per_sec = float(match.group(1))
                ms_per_loop = float(match.group(2))
                queue_size = int(match.group(3))

                self.perf_stats['loops_per_sec'] = loops_per_sec
                self.perf_stats['ms_per_loop'] = ms_per_loop
                self.perf_stats['queue_size'] = queue_size

                # Update GUI label
                def update_label():
                    self.perf_label.config(
                        text=f"ESP32 Performance: {loops_per_sec:.1f} loops/sec, {ms_per_loop:.3f} ms/loop, Queue: {queue_size}/8"
                    )
                self.root.after(0, update_label)
        except Exception:
            pass  # Silently ignore parsing errors

    def decode_payload(self, peripheral_name, payload):
        """Decode and display payload data"""
        if len(payload) == 0:
            return

        # Check for ACK/ERROR responses
        if len(payload) == 1 and payload[0] == 0x01:
            self.log("     → ACK", "rx")
            return
        elif len(payload) >= 2 and payload[0] == 0xFF:
            error_codes = {
                0x01: "INVALID_FRAME",
                0x02: "QUEUE_FULL",
                0x03: "INVALID_CMD",
                0x04: "INVALID_PID"
            }
            error_name = error_codes.get(payload[1], f"UNKNOWN (0x{payload[1]:02X})")
            self.log(f"     → ERROR: {error_name}", "error")
            return

        # Decode based on peripheral type
        try:
            if peripheral_name == "SYSTEM":
                if len(payload) == SIZE_HEARTBEAT:
                    data = unpack_heartbeat(payload)
                    self.log("     → Heartbeat (WireHeartbeat_t):", "rx")
                    self.log(f"         Version: {data.get('version', '?')}", "rx")
                    self.log(f"         Uptime: {data.get('uptime_seconds', '?')}s", "rx")
                    self.log(f"         State: {data.get('system_state', '?')}", "rx")
                elif len(payload) == SIZE_STATUS:
                    data = unpack_status(payload)
                    self.log("     → System Status (WireStatus_t):", "rx")
                    self.log(f"         Version: {data.get('version', '?')}", "rx")
                    self.log(f"         Uptime: {data.get('uptime_seconds', '?')}s", "rx")
                    self.log(f"         State: {data.get('system_state', '?')}", "rx")
                    self.log(f"         Sensor Flags: 0x{data.get('sensor_flags', 0):02X}", "rx")
                    self.log(f"         LoRa Packets: {data.get('pkt_count_lora', '?')}", "rx")
                    self.log(f"         433MHz Packets: {data.get('pkt_count_433', '?')}", "rx")
                    self.log(f"         Heap Free: {data.get('heap_free', '?')} bytes", "rx")
                    self.log(f"         Chip Rev: {data.get('chip_revision', '?')}", "rx")
                elif len(payload) == SIZE_TRANSFER_STATS:
                    data = unpack_transfer_stats(payload)
                    self.transfer_stats = data  # Store for popup window
                    self.log("     → Transfer Statistics (WireTransferStats_t):", "rx")
                    self.log(f"         Version: {data.get('version', '?')}", "rx")
                    peripherals = data.get('peripherals', {})
                    for name, stats in peripherals.items():
                        packets = stats['total_packets']
                        bytes_total = stats['total_bytes']
                        if bytes_total < 1024:
                            bytes_str = f"{bytes_total} B"
                        elif bytes_total < 1024 * 1024:
                            bytes_str = f"{bytes_total / 1024:.2f} KB"
                        else:
                            bytes_str = f"{bytes_total / (1024 * 1024):.2f} MB"
                        self.log(f"         {name}: {packets:,} packets, {bytes_str}", "rx")
                    # Open/update popup window
                    self.root.after(0, self.open_stats_popup)
                else:
                    self.log(f"     → Unknown SYSTEM payload: {len(payload)} bytes", "rx")
                    self.log(f"     → Hex: {payload.hex().upper()}", "rx")

            elif peripheral_name == "LORA_915":
                data = unpack_lora_data(payload)
                self.log("     → LoRa 915MHz Data (WireLoRa_t):", "rx")
                self.log(f"         Version: {data.get('version', '?')}", "rx")
                self.log(f"         Packet Count: {data.get('packet_count', '?')}", "rx")
                self.log(f"         RSSI: {data.get('rssi', '?')} dBm", "rx")
                self.log(f"         SNR: {data.get('snr', '?'):.2f} dB", "rx")
                self.log(f"         Payload Length: {data.get('payload_length', '?')} bytes", "rx")
                if 'payload' in data and data['payload']:
                    self.log(f"         Payload: {data['payload']}", "rx")
                if data.get('partial'):
                    self.log(f"         ⚠ PARTIAL: Expected {data.get('expected_length')} bytes, got {data.get('actual_length')}", "error")

            elif peripheral_name == "RADIO_433":
                data = unpack_lora_data(payload)  # Same structure as LoRa
                self.log("     → 433MHz Radio Data (Wire433_t):", "rx")
                self.log(f"         Version: {data.get('version', '?')}", "rx")
                self.log(f"         Packet Count: {data.get('packet_count', '?')}", "rx")
                self.log(f"         RSSI: {data.get('rssi', '?')} dBm", "rx")
                self.log(f"         SNR: {data.get('snr', '?'):.2f} dB", "rx")
                self.log(f"         Payload Length: {data.get('payload_length', '?')} bytes", "rx")
                if 'payload' in data and data['payload']:
                    self.log(f"         Payload: {data['payload']}", "rx")
                if data.get('partial'):
                    self.log(f"         ⚠ PARTIAL: Expected {data.get('expected_length')} bytes, got {data.get('actual_length')}", "error")

            elif peripheral_name == "BAROMETER":
                data = unpack_barometer_data(payload)
                self.log("     → Barometer Data (WireBarometer_t):", "rx")
                self.log(f"         Version: {data.get('version', '?')}", "rx")
                self.log(f"         Timestamp: {data.get('timestamp', '?')} ms", "rx")
                self.log(f"         Pressure: {data.get('pressure_pa', '?'):.2f} Pa", "rx")
                self.log(f"         Temperature: {data.get('temperature_c', '?'):.2f} °C", "rx")
                self.log(f"         Altitude: {data.get('altitude_m', '?'):.2f} m", "rx")
                if data.get('partial'):
                    self.log(f"         ⚠ PARTIAL: Expected {data.get('expected_length')} bytes, got {data.get('actual_length')}", "error")

            elif peripheral_name == "CURRENT":
                data = unpack_current_data(payload)
                self.log("     → Current Sensor Data (WireCurrent_t):", "rx")
                self.log(f"         Version: {data.get('version', '?')}", "rx")
                self.log(f"         Timestamp: {data.get('timestamp', '?')} ms", "rx")
                self.log(f"         Current: {data.get('current_a', '?'):.3f} A", "rx")
                self.log(f"         Voltage: {data.get('voltage_v', '?'):.3f} V", "rx")
                self.log(f"         Power: {data.get('power_w', '?'):.3f} W", "rx")
                self.log(f"         Raw ADC: {data.get('raw_adc', '?')}", "rx")
                if data.get('partial'):
                    self.log(f"         ⚠ PARTIAL: Expected {data.get('expected_length')} bytes, got {data.get('actual_length')}", "error")

            else:
                self.log(f"     → Unknown peripheral data: {len(payload)} bytes", "rx")
                self.log(f"     → Hex: {payload.hex().upper()}", "rx")

        except Exception as e:
            self.log(f"     → Decode error: {e}", "error")
            self.log(f"     → Raw hex: {payload.hex().upper()}", "rx")

    def log(self, message, tag=None):
        """Add message to response text"""
        def append():
            self.response_text.insert(tk.END, message + "\n", tag)
            self.response_text.see(tk.END)

        # Call from main thread
        self.root.after(0, append)

    def clear_log(self):
        """Clear response text"""
        self.response_text.delete(1.0, tk.END)

    def copy_log(self):
        """Copy log contents to clipboard"""
        log_contents = self.response_text.get(1.0, tk.END)
        self.root.clipboard_clear()
        self.root.clipboard_append(log_contents)
        self.log("[INFO] Log copied to clipboard", "info")

# ====================================================================
# MAIN
# ====================================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = HybridProtocolGUI(root)
    root.mainloop()
