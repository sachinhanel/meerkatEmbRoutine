# Test Tools

Python-based testing and interaction tools for the Meerkat Embedded Routine firmware.

## Contents

- **interactive_sender.py** - Full-featured GUI for testing and controlling ESP32 (primary testing tool)
- **esp32_simulator.py** - Simulates ESP32 responses for testing host scripts without hardware

## Interactive Sender GUI

### Features

- **Serial Connection Management**: Connect/disconnect with port auto-detection
- **Message Builder**: Construct and send custom commands
- **Quick Actions**: Pre-configured buttons for common operations
- **Real-time Logging**: Color-coded TX/RX/INFO messages
- **Autonomous Polling**: Start/stop periodic data transmission
- **Transfer Statistics**: View persistent data usage with popup window
- **Performance Monitoring**: Real-time loop performance metrics

### Installation

```bash
pip install pyserial
```

### Usage

```bash
python interactive_sender.py
```

### GUI Controls

**Connection:**
1. Select COM port from dropdown (auto-populated)
2. Baud rate: 921600 (default, don't change)
3. Click "Connect"

**Quick Actions:**
- Get System Status - Retrieve ESP32 system information
- Get All Peripherals - Query all sensor data at once
- Get LoRa / Barometer / Current - Query specific peripheral
- Start Polling - Begin autonomous periodic transmission
- Stop All Polling - Stop all autonomous transmission
- Enable/Disable Perf Stats - Toggle performance monitoring
- Transfer Statistics - Open persistent statistics popup

**Message Builder:**
1. Select Peripheral (SYSTEM, LORA_915, etc.)
2. Select Command (GET_ALL, SET_POLL_RATE, etc.)
3. Set parameters if needed (e.g., poll interval)
4. Click "Send Command"

**Log Controls:**
- Auto-clear log before TX - Clears log before each transmission
- Clear Log - Manually clear log
- Copy Log - Copy log to clipboard

### Transfer Statistics Popup

**Features:**
- View total packets and bytes sent per peripheral
- Data formatted as B, KB, or MB automatically
- Statistics persist across ESP32 reboots
- Refresh to update display
- Reset to clear all counters (with confirmation)

**Usage:**
1. Click "Transfer Statistics" button
2. Popup shows current stats
3. Click "Refresh" to request updated data
4. Click "Reset Stats" to clear (persists after reset)

### Performance Stats

When enabled (click "Enable Perf Stats"):
- Shows in status bar at bottom of GUI
- Format: `ESP32 Performance: X loops/sec, Y ms/loop, Queue: Z/8`
- Updates 2x per second
- Useful for debugging system load

### Log Color Coding

- **Blue** - Transmitted commands ([TX])
- **Green** - Received responses ([RX])
- **Purple** - Info/debug messages ([INFO], [PERF], etc.)
- **Red** - Errors ([ERROR])

## Advanced Usage

### Scripting

You can import functions from interactive_sender.py:

```python
from interactive_sender import encode_hybrid_frame, calculate_checksum

# Build a frame
peripheral_id = 0x01  # LoRa
command = 0x00  # GET_ALL
payload = bytes([command])
frame = encode_hybrid_frame(peripheral_id, payload)

# Send via pyserial
import serial
ser = serial.Serial('COM3', 921600)
ser.write(frame)
response = ser.read(100)
```

### Automated Testing

Create test scripts:

```python
import serial
import time
from interactive_sender import encode_hybrid_frame, decode_hybrid_frame

# Connect
ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=1)
time.sleep(2)

# Send GET_ALL to all peripherals
frame = encode_hybrid_frame(0xFF, bytes([0x00]))
ser.write(frame)

# Read responses
for i in range(5):  # Expect 5 responses
    data = ser.read(100)
    pid, payload = decode_hybrid_frame(data)
    print(f"Peripheral {pid}: {len(payload)} bytes")

ser.close()
```

## Troubleshooting

**GUI doesn't start:**
```bash
# Check Python version (3.7+)
python --version

# Reinstall pyserial
pip uninstall pyserial
pip install pyserial
```

**Port not listed:**
```bash
# Windows: Check Device Manager → Ports
# Linux: ls /dev/ttyUSB* /dev/ttyACM*
# Mac: ls /dev/cu.usbserial*

# Linux permissions:
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Connection fails:**
- Verify ESP32 is powered and running
- Check correct COM port selected
- Ensure no other program using the port
- Try unplugging and replugging USB

**No responses:**
- Check ESP32 is running firmware (see startup banner in log)
- Enable debug messages: look for [INFO] lines
- Verify baud rate is 921600
- Try "Get System Status" - should always respond

**Corrupted frames:**
- Check USB cable quality (use data cable, not just charging)
- Reduce electromagnetic interference
- Try different USB port
- Check ESP32 power supply is stable

## See Also

- [README.md](../README.md) - Project overview
- [docs/GETTING_STARTED.md](../docs/GETTING_STARTED.md) - Setup guide
- [docs/PROTOCOL.md](../docs/PROTOCOL.md) - Protocol specification
