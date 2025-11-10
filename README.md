# Meerkat Embedded Routine

High-performance ESP32 firmware for multi-peripheral data collection and communication via a pure binary protocol. Designed for reliable, low-latency communication with a Raspberry Pi or PC host over UART.

## Overview

This project implements a single-threaded, priority-based event loop on ESP32-S3 that manages multiple sensors and radio peripherals, communicating with a host computer using a custom binary protocol over serial (UART). The system is optimized for minimal latency and maximum reliability.

### Key Features

- **Pure Binary Protocol**: Efficient framing with `<AA55[PID][LEN][PAYLOAD][CHK]55AA>` format
- **Multi-Peripheral Support**:
  - 915MHz LoRa module
  - 433MHz radio module
  - MS5607 barometer
  - Current/voltage sensor
  - System status monitoring
- **Autonomous Polling**: Peripherals can send data at configurable intervals
- **Non-blocking Architecture**: Priority-based loop ensures responsive command handling
- **Transfer Statistics**: Persistent tracking of data usage per peripheral
- **Performance Monitoring**: Real-time loop performance metrics
- **Test Mode**: Run without hardware for development and testing

## Project Structure

```
meerkatEmbRoutine/
├── src/
│   ├── main.cpp              # Main event loop and protocol handler
│   ├── config.h              # Protocol constants and peripheral IDs
│   ├── DataCollector.h/cpp   # Sensor data collection and packing
│   ├── TransferStats.h/cpp   # Persistent transfer statistics
│   └── sensors/              # Individual sensor drivers
├── test/
│   ├── interactive_sender.py # Python GUI for testing and control
│   └── README                # Test tools documentation
├── platformio.ini            # Build configuration
└── README.md                 # This file
```

## Hardware Requirements

### ESP32 Board
- **Board**: ESP32-S3 DevKit-C (or compatible ESP32-WROOM/WROVER)
- **Connection**: USB-C port on dev board
  - Used for both flashing firmware and serial communication
  - Connect to PC for initial programming
  - Connect to Raspberry Pi for production use

### Peripherals (Optional - Test Mode Available)
- 915MHz LoRa module (SX1276/SX1278)
- 433MHz radio module
- MS5607 barometer (I2C)
- Current/voltage sensor (ADC)

## Quick Start

### 1. Install Development Tools

**Required:**
- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- Python 3.7+ (for interactive sender GUI)

**Python Dependencies:**
```bash
pip install pyserial
```

### 2. Flash Firmware to ESP32

#### Option A: Using PlatformIO IDE (VS Code)
1. Open project folder in VS Code
2. Connect ESP32 via USB-C cable
3. Select environment:
   - `esp32-test` - Test mode (no hardware required)
   - `esp32-uart` - Production mode (requires sensors)
4. Click **PlatformIO: Upload** (→ arrow icon)

#### Option B: Using PlatformIO CLI
```bash
# Test mode (no sensors)
pio run -e esp32-test --target upload

# Production mode (with sensors)
pio run -e esp32-uart --target upload
```

**Upload Settings:**
- Port: Automatically detected (or set `upload_port = COMx` in platformio.ini)
- Baud: 921600 (falls back to 460800/115200 if connection is unstable)

### 3. Connect for Communication

**Development/Testing (PC):**
1. Keep ESP32 connected to PC via USB-C
2. Note the COM port (Windows) or `/dev/ttyUSBx` (Linux)
3. Open `test/interactive_sender.py` with Python

**Production (Raspberry Pi):**
1. Disconnect ESP32 from PC
2. Connect ESP32 USB-C port to Raspberry Pi USB port
3. ESP32 will enumerate as `/dev/ttyUSB0` or `/dev/ttyACM0`
4. Use the same serial interface (921600 baud)

### 4. Run Interactive Sender GUI

**The Interactive Sender is your primary testing tool** - it provides a graphical interface to send commands, receive responses, and monitor the protocol in real-time.

```bash
cd test
python interactive_sender.py
```

**Where to Run:**
- **On your PC** during development (while coding and testing)
- **On Raspberry Pi** when testing production deployment
- Same tool works on both platforms!

**Why It's Essential:**
- ✅ Visual frame construction and validation
- ✅ Automatic checksum calculation
- ✅ Color-coded TX/RX logging
- ✅ Real-time performance monitoring
- ✅ Persistent statistics tracking
- ✅ One-click command execution
- ✅ Frame hex dump for debugging

**GUI Features:**
- **Connection**: Select port and connect at 921600 baud
- **Quick Actions**: Pre-configured buttons for common operations
- **Message Builder**: Send custom commands to any peripheral
- **Autonomous Polling**: Start/stop periodic data transmission
- **Transfer Statistics**: View persistent data usage statistics
- **Performance Stats**: Monitor ESP32 loop performance

**Development Workflow:**
1. Write code on PC → Flash to ESP32
2. Keep ESP32 connected to PC
3. Run Interactive Sender on PC
4. Test commands and verify responses
5. Iterate until stable
6. Deploy ESP32 to Raspberry Pi
7. Run Interactive Sender on Pi to verify
8. Write production scripts using same protocol

## Usage Examples

### Get System Status
```python
# Via GUI: Click "Get System Status"
# Response shows uptime, heap, packet counts, sensor flags
```

### Start Autonomous Polling
```python
# Via GUI: Click "Start LoRa Poll (1s)"
# ESP32 will send LoRa data every 1000ms automatically
```

### View Transfer Statistics
```python
# Via GUI: Click "Transfer Statistics"
# Shows total packets and bytes sent per peripheral
# Statistics persist across reboots
```

### Monitor Performance
```python
# Via GUI: Click "Enable Perf Stats"
# See loops/sec, ms/loop, queue depth in status bar
```

## Protocol Overview

### Frame Format (Pure Binary)
```
<AA55[PID][LEN][PAYLOAD][CHK]55AA>

< (0x3C)     - Start marker
0xAA 0x55    - Protocol header
[PID]        - Peripheral ID (1 byte)
[LEN]        - Payload length (1 byte, 0-255)
[PAYLOAD]    - Raw binary data (LEN bytes)
[CHK]        - XOR checksum (1 byte)
0x55 0xAA    - Protocol footer
> (0x3E)     - End marker

Total: 9 + LEN bytes
```

### Peripheral IDs
- `0x00` - SYSTEM (ESP32 control)
- `0x01` - LORA_915 (915MHz LoRa)
- `0x02` - RADIO_433 (433MHz radio)
- `0x03` - BAROMETER (MS5607)
- `0x04` - CURRENT (Current/voltage sensor)
- `0xFF` - ALL (targets all peripherals)

### Common Commands
- `0x00` - GET_ALL (get data from peripheral)
- `0x01` - GET_STATUS (get health status)
- `0x02` - SET_POLL_RATE (start autonomous polling)
- `0x03` - STOP_POLL (stop autonomous polling)
- `0x20` - SYSTEM_STATUS (system info, PID=0x00 only)
- `0x24` - SYSTEM_PERF (toggle performance stats)
- `0x25` - SYSTEM_STATS (get transfer statistics)
- `0x26` - SYSTEM_STATS_RESET (reset transfer statistics)

See [PROTOCOL.md](docs/PROTOCOL.md) for complete protocol specification.

## Architecture

### Event Loop Priority
1. **Read incoming commands** (non-blocking, queues commands)
2. **Execute queued command** (one per loop, sends response)
3. **Update peripheral sensors** (scheduled, non-blocking)
4. **Send autonomous data** (if enabled, rate-limited)
5. **Report performance stats** (if enabled, 2x per second)
6. **Save transfer statistics** (every 30 seconds if dirty)

### Key Design Decisions
- **No FreeRTOS**: Single-threaded for predictable timing
- **Command Queue**: Prevents blocking on burst requests
- **Non-blocking I/O**: Sensors polled on schedule, not on request
- **Binary Protocol**: 50% smaller than ASCII hex encoding
- **Persistent Storage**: LittleFS for transfer statistics

## Development

### Test Mode
Run without hardware by using the `esp32-test` environment:

```bash
pio run -e esp32-test --target upload
```

Test mode features:
- Fake sensor data generation
- No hardware initialization
- Useful for protocol testing and GUI development

### Build Environments
- **esp32-test**: Test mode with fake sensors
- **esp32-uart**: Production mode with real hardware

### Serial Monitor
```bash
# PlatformIO serial monitor
pio device monitor -e esp32-test

# Or use interactive sender GUI
python test/interactive_sender.py
```

## Troubleshooting

### Upload Fails
- Ensure ESP32 is in bootloader mode (usually automatic)
- Try lower baud rate: `upload_speed = 460800` in platformio.ini
- Check USB cable supports data (not just charging)

### Serial Connection Issues
- Verify correct COM port selected
- Ensure identical baud rate setting between embedded system and sender
- Check ESP32 powered and running (LED should be on)
- Try unplugging/replugging USB cable

### No Sensor Data
- Verify running `esp32-uart` environment (not test mode)
- Check sensor wiring and power
- View debug output: enable `[INFO]` message logging in GUI

### Transfer Stats Not Persisting
- LittleFS partition may need formatting (automatic on first boot)
- Check ESP32 flash size is adequate (4MB+ recommended)
- Stats save every 30 seconds, wait before rebooting

## Performance

Typical performance on ESP32-S3 @ 240MHz:
- **Loop speed**: 10,000-20,000 loops/sec (idle)
- **Latency**: <5ms command response time
- **Max poll rate**: 30ms per peripheral (100ms for ALL)
- **Command queue**: 8 commands deep

## Future Work

This project has several planned enhancements. See [docs/FUTURE_WORK.md](docs/FUTURE_WORK.md) for complete details.

### Planned Features

**High Priority:**
- **SD Card Data Logging**: Local storage for autonomous operation and data backup
- **AIM Board Integration**: Full support for 4x Autonomous Instrument Module boards
- **Testing & Validation**: Comprehensive automated test suite and field testing

**Medium Priority:**
- **Peripheral-Specific Commands**: Fine-grained control (frequency, power, calibration, etc.)
- **Protocol Enhancements**: Bulk transfers, improved error reporting, heartbeat
- **Performance Optimizations**: DMA transfers, multi-core support, compile-time config

### How to Contribute

See feature details, implementation notes, and priority order in [docs/FUTURE_WORK.md](docs/FUTURE_WORK.md).

## Contributing

This is an academic/research project. If you find issues or have suggestions:
1. Check existing issues
2. Create detailed bug report with logs
3. Include ESP32 board variant and sensor configuration

## License

[Add your license here]

## Acknowledgments

- Built with [PlatformIO](https://platformio.org/)
- Uses [LoRa library](https://github.com/sandeepmistry/arduino-LoRa) by Sandeep Mistry
- Uses [RadioLib](https://github.com/jgromes/RadioLib) for radio communication
- Uses [MS5611 library](https://github.com/RobTillaart/MS5611) by Rob Tillaart

## Contact

[Add your contact information or university affiliation]
