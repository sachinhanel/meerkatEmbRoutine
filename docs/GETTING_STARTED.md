# Getting Started with Meerkat Embedded Routine

Complete step-by-step guide to setting up, flashing, and using the ESP32 firmware.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Hardware Setup](#hardware-setup)
3. [Software Installation](#software-installation)
4. [Flashing Firmware](#flashing-firmware)
5. [First Connection](#first-connection)
6. [Using the Interactive Sender](#using-the-interactive-sender)
7. [Raspberry Pi Setup](#raspberry-pi-setup)

## Prerequisites

### Required Hardware
- ESP32-S3 DevKit-C board (or compatible ESP32-WROOM/WROVER board)
- USB-C cable (data capable, not just charging)
- PC (Windows/Linux/Mac) for initial setup
- Raspberry Pi (for production deployment)

### Optional Hardware (for production mode)
- 915MHz LoRa module
- 433MHz radio module
- MS5607 barometer (I2C)
- Current/voltage sensor
- Jumper wires and breadboard

### Required Software
- Python 3.7 or newer
- PlatformIO (VS Code extension recommended)
- USB-to-Serial drivers (usually automatic, see [Drivers](#drivers))

## Hardware Setup

### ESP32 Connection

**For Flashing and Testing:**
```
PC USB port → USB-C cable → ESP32 USB-C port
```

**For Production (Raspberry Pi):**
```
Raspberry Pi USB port → USB-C cable → ESP32 USB-C port
```

**Important Notes:**
- The same USB-C port on the ESP32 is used for both programming and communication
- No need to switch cables between flashing and operation
- ESP32 will enumerate as a USB-to-Serial device (CP210x, CH340, or FTDI)

### Sensor Connections (Optional - Production Mode Only)

If using real sensors (`esp32-uart` environment):

**LoRa 915MHz (SPI):**
```
ESP32        LoRa Module
GPIO5   →   SCK
GPIO18  →   MISO
GPIO23  →   MOSI
GPIO15  →   CS
GPIO2   →   RST
GPIO4   →   DIO0
3.3V    →   VCC
GND     →   GND
```

**Barometer MS5607 (I2C):**
```
ESP32        MS5607
GPIO21  →   SDA
GPIO22  →   SCL
3.3V    →   VCC
GND     →   GND
```

*See config.h for complete pin definitions.*

## Software Installation

### 1. Install PlatformIO

**Option A: VS Code Extension (Recommended)**
1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Open VS Code Extensions (Ctrl+Shift+X)
3. Search for "PlatformIO IDE"
4. Click Install
5. Restart VS Code

**Option B: Command Line**
```bash
# Install PlatformIO Core
pip install platformio

# Verify installation
pio --version
```

### 2. Install Python Dependencies

```bash
# For interactive sender GUI
pip install pyserial

# Optional: for better error messages
pip install pyserial-asyncio
```

### 3. Clone Repository

```bash
git clone https://github.com/your-username/meerkatEmbRoutine.git
cd meerkatEmbRoutine
```

### 4. Verify Project Structure

```bash
# Should see:
# platformio.ini
# src/
# test/
# README.md
```

## Drivers

### Checking USB-to-Serial Chip

Your ESP32 board likely uses one of:
- **CP210x** (Silicon Labs) - Most common
- **CH340** (WCH) - Common on cheaper boards
- **FTDI** - Some older boards

### Installing Drivers

**Windows:**
- CP210x: [Download from Silicon Labs](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- CH340: [Download from WCH](http://www.wch.cn/downloads/CH341SER_EXE.html)

**Linux:**
```bash
# Usually built-in, verify with:
lsmod | grep usbserial

# If needed, load module:
sudo modprobe usbserial
```

**Mac:**
- CP210x: [Download from Silicon Labs](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- Usually works out of the box on recent macOS

### Verify Port Detection

**Windows:**
```
Device Manager → Ports (COM & LPT)
# Should see: Silicon Labs CP210x USB to UART Bridge (COM3)
```

**Linux:**
```bash
ls /dev/ttyUSB*
# Should see: /dev/ttyUSB0 (or similar)

# Check permissions:
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Mac:**
```bash
ls /dev/cu.*
# Should see: /dev/cu.usbserial-*
```

## Flashing Firmware

### Method 1: PlatformIO IDE (VS Code)

1. **Open Project:**
   - File → Open Folder → Select `meerkatEmbRoutine`

2. **Connect ESP32:**
   - Plug in USB-C cable
   - Wait for drivers to load

3. **Select Environment:**
   - Click on environment selector at bottom of VS Code
   - Choose:
     - `esp32-test` - Test mode (no sensors needed)
     - `esp32-uart` - Production mode (requires sensors)

4. **Upload:**
   - Click PlatformIO icon (alien head) in sidebar
   - Click "Upload" under your environment
   - Or click the → arrow icon in status bar

5. **Monitor Output:**
   - Serial monitor opens automatically
   - Should see: `[INFO] ESP32 BINARY PROTOCOL MODE`

### Method 2: PlatformIO CLI

```bash
# Navigate to project directory
cd meerkatEmbRoutine

# Flash test mode (no sensors)
pio run -e esp32-test --target upload

# Flash production mode (with sensors)
pio run -e esp32-uart --target upload

# Monitor serial output
pio device monitor -e esp32-test -b 115200
```

### Troubleshooting Uploads

**"Serial port not found":**
```bash
# List available ports
pio device list

# Manually set port in platformio.ini:
# upload_port = COM3  (Windows)
# upload_port = /dev/ttyUSB0  (Linux)
```

**"Upload failed" or timeout:**
```bash
# Try holding BOOT button while uploading
# Or reduce upload speed in platformio.ini:
# upload_speed = 460800  (or 115200)
```

**"Flash erase error":**
```bash
# Erase flash first
pio run -e esp32-test --target erase

# Then upload again
pio run -e esp32-test --target upload
```

## First Connection

### 1. Launch Interactive Sender

```bash
cd test
python interactive_sender.py
```

### 2. Connect to ESP32

1. **Select Port:**
   - Choose your COM port from dropdown
   - Baud rate: 921600 (default)

2. **Click "Connect":**
   - Status changes to "● Connected" (green)
   - Should see startup banner in log:
     ```
     [INFO] ====================================
     [INFO]   ESP32 BINARY PROTOCOL MODE
     [INFO]   Single-Threaded Event Loop
     [INFO] ====================================
     ```

### 3. Test Basic Communication

**Get System Status:**
- Click "Get System Status" button
- Should receive system information:
  - Uptime in seconds
  - Free heap memory
  - Sensor status flags
  - Packet counts

**Example Output:**
```
[TX] SYSTEM → SYSTEM_STATUS
     3C AA 55 00 01 20 21 55 AA 3E
[RX] SYSTEM (PID=0x00): 20 bytes
     → System Status (WireStatus_t):
         Version: 1
         Uptime: 42s
         State: 1
         Heap Free: 371608 bytes
```

### 4. Test Peripheral Data

**Get LoRa Data (Test Mode):**
- Click "Get LoRa" button
- Receives fake LoRa packet in test mode
- Or real data in production mode

## Using the Interactive Sender

### Overview - Your Primary Testing Tool

**The Interactive Sender is the main tool for testing and communicating with your ESP32.** It provides a graphical interface to send commands, receive responses, and monitor the protocol in real-time. This is essential for:

- **Development**: Testing new firmware features during coding
- **Debugging**: Verifying protocol communication and responses
- **Production Testing**: Validating ESP32 behavior before deployment
- **Monitoring**: Observing autonomous polling and data transfer

### Where to Run the Interactive Sender

**Important**: The Interactive Sender runs on the **host machine** that is physically connected to the ESP32 via USB.

**Scenario 1: Development on PC**
```
Your PC ←→ USB-C cable ←→ ESP32
         (Run interactive_sender.py on PC)
```
- Use this when developing firmware features
- Flash code from your IDE, then test immediately
- See responses in real-time as you modify code
- Debug protocol issues quickly

**Scenario 2: Testing on Raspberry Pi**
```
Raspberry Pi ←→ USB-C cable ←→ ESP32
            (Run interactive_sender.py on Pi)
```
- Use this when testing production deployment
- Verify Pi can communicate with ESP32
- Test autonomous polling in production environment
- Monitor long-term data transfer statistics

**Scenario 3: Hybrid Development**
```
1. Flash firmware: PC ←→ USB-C ←→ ESP32
2. Quick test: Run interactive_sender.py on PC
3. Deploy: Move ESP32 to Pi
4. Production test: Run interactive_sender.py on Pi
```

### Why the Interactive Sender is Critical

**Without the Interactive Sender**, you would need to:
- Manually craft binary frames (error-prone)
- Write custom scripts for every test
- Debug protocol issues blindly
- No visibility into responses

**With the Interactive Sender**, you get:
- ✅ Visual frame construction and validation
- ✅ Automatic checksum calculation
- ✅ Color-coded TX/RX logging
- ✅ Real-time performance monitoring
- ✅ Persistent statistics tracking
- ✅ One-click command execution
- ✅ Frame hex dump for debugging

### Typical Development Workflow

1. **Write firmware code** on your PC in VS Code
2. **Flash to ESP32** via PlatformIO (ESP32 stays connected to PC)
3. **Run interactive_sender.py** on the same PC
4. **Send test commands** and verify responses
5. **Iterate** - modify code, reflash, test again
6. **Deploy** - when stable, move ESP32 to Raspberry Pi
7. **Production test** - run interactive_sender.py on Pi to verify
8. **Production use** - write Python scripts using the same protocol

### GUI Layout

```
┌─────────────────────────────────────────┐
│ Serial Connection                       │
│ Port: [COM3 ▼]  Baud: 115200  [Connect]│
├─────────────────────────────────────────┤
│ Message Builder                         │
│ Peripheral: [SYSTEM ▼] Cmd: [GET_ALL ▼]│
│ [Send Command]                          │
├─────────────────────────────────────────┤
│ Quick Actions                           │
│ [System Status] [Get LoRa] [Start Poll]│
│ [Transfer Stats] [Enable Perf]   ...   │
├─────────────────────────────────────────┤
│ Log                                     │
│ [TX] Commands sent (blue)               │
│ [RX] Responses received (green)         │
│ [INFO] Debug messages (purple)          │
├─────────────────────────────────────────┤
│ [Auto-clear log] [Clear Log] [Copy Log]│
│ ESP32 Performance: 15234 loops/sec ...  │
└─────────────────────────────────────────┘
```

### Common Operations

**Enable Performance Monitoring:**
```
1. Click "Enable Perf Stats"
2. See real-time loop performance in status bar
3. Useful for debugging system load
```

**Start Autonomous Polling:**
```
1. Click "Start LoRa Poll (1s)"
2. ESP32 sends LoRa data every 1000ms automatically
3. Click "Stop All Polling" to stop
```

**View Transfer Statistics:**
```
1. Click "Transfer Statistics"
2. Popup shows data usage per peripheral
3. Click "Refresh" to update
4. Click "Reset Stats" to clear (persists across reboots)
```

**Send Custom Command:**
```
1. Select Peripheral from dropdown
2. Select Command from dropdown
3. Set parameters (e.g., poll interval)
4. Click "Send Command"
```

### Using Interactive Sender for Learning

**The Interactive Sender is also a learning tool** - you can see exactly what binary frames look like:

**Example: Sending GET_ALL to LoRa:**
```
[TX] LORA_915 → GET_ALL
     3C AA 55 01 01 00 00 55 AA 3E

Breaking down the frame:
3C        - Start marker '<'
AA 55     - Protocol header
01        - PID (LORA_915)
01        - Length (1 byte payload)
00        - Payload (CMD_GET_ALL)
00        - Checksum (0x01 ^ 0x01 ^ 0x00 = 0x00)
55 AA     - Protocol footer
3E        - End marker '>'
```

This helps you understand the protocol when writing your own scripts!

### From Testing Tool to Production Code

Once you've tested with the Interactive Sender, you can write production scripts using the same protocol:

**Development Workflow:**

1. **Use Interactive Sender to test:**
   - Verify commands work correctly
   - See exact frame format in hex
   - Confirm expected responses
   - Debug any protocol issues

2. **Then write your Python script:**
   ```python
   #!/usr/bin/env python3
   """Production script for autonomous data collection"""
   import serial
   import time
   from interactive_sender import encode_hybrid_frame, decode_hybrid_frame

   # Open connection (same settings as GUI)
   ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=1)
   time.sleep(2)  # Wait for ESP32 boot

   # Start LoRa polling at 1000ms (tested in GUI first!)
   frame = encode_hybrid_frame(0x01, bytes([0x02, 0xE8, 0x03]))
   ser.write(frame)

   # Collect data
   while True:
       if ser.in_waiting > 0:
           # Read frame (same logic as GUI rx_worker)
           data = ser.read(ser.in_waiting)
           pid, payload = decode_hybrid_frame(data)

           if pid == 0x01:  # LoRa data
               # Process and log...
               print(f"LoRa packet: {len(payload)} bytes")
   ```

3. **Test your script on Pi:**
   - Run your script on Raspberry Pi
   - Verify it works the same as the GUI
   - Monitor for any issues

**Key Benefit**: The Interactive Sender uses the exact same `encode_hybrid_frame()` and `decode_hybrid_frame()` functions that you'll use in production, so you can import them directly!

### Raspberry Pi Deployment

When moving from PC development to Raspberry Pi production:

1. **Copy Interactive Sender to Pi:**
   ```bash
   scp test/interactive_sender.py pi@raspberrypi:~/
   ```

2. **Test on Pi first:**
   ```bash
   # On Raspberry Pi
   python3 interactive_sender.py
   ```

3. **Once verified, write production script:**
   - Import functions from `interactive_sender.py`
   - Use same protocol logic
   - Add your application-specific code

## Next Steps

- Read [PROTOCOL.md](PROTOCOL.md) for complete protocol specification
- Explore example scripts in `test/` directory
- Check `src/config.h` for customization options
- Join discussions in GitHub Issues

## Getting Help

**Common Issues:**
- See [Troubleshooting](#troubleshooting-uploads) section above
- Check GitHub Issues for similar problems
- Enable debug output: Monitor `[INFO]` messages in GUI

**Support:**
- GitHub Issues: [Link to your repo issues]
- Documentation: `docs/` folder
- Examples: `test/` folder
