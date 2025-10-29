# Interactive Test Sender Guide

## Overview

`interactive_sender.py` is a Tkinter GUI application for testing the ESP32 hybrid protocol interactively.

## Features

✅ **Serial Connection**
- Port selection with auto-detection
- Connect/disconnect
- Configurable baud rate (default 115200)

✅ **Command Sending**
- Dropdown selection for Peripheral + Command
- Automatic payload construction
- Support for SET_POLL_RATE with interval input

✅ **Quick Actions**
- Get System Status
- Get LoRa/Barometer data
- Start/Stop autonomous polling
- One-click common operations

✅ **Real-Time Display**
- Color-coded TX (blue), RX (green), errors (red)
- Automatic frame decoding
- Payload hex display
- ACK/ERROR detection

✅ **Autonomous Polling**
- Start polling per peripheral
- Stop all polling
- Monitor autonomous data in main log

## Installation

```bash
pip install pyserial
```

## Usage

### Start the GUI

```bash
cd test
python interactive_sender.py
```

### Connect to ESP32

1. Select COM port from dropdown
2. Click "Refresh" if port not visible
3. Click "Connect"
4. Wait for "Connected" status (green)

### Send Commands

#### Method 1: Dropdowns
1. Select **Peripheral** (e.g., "LORA_915")
2. Select **Command** (e.g., "GET_ALL")
3. Click **Send**

#### Method 2: Quick Actions
Click any quick action button:
- **Get System Status** - Retrieves uptime, queue size, etc.
- **Get All LoRa** - Gets LoRa data (74 bytes)
- **Get All Barometer** - Gets barometer data (17 bytes)
- **Start LoRa Polling (1s)** - Enables autonomous sending every 1s
- **Stop All Polling** - Disables all autonomous sending

### Start Autonomous Polling

1. Select peripheral (e.g., "LORA_915")
2. Select "SET_POLL_RATE" command
3. Enter **Interval (ms)** (e.g., 1000 for 1 second)
4. Click **Send**

Result: ESP32 will automatically send data every X milliseconds!

### Stop Autonomous Polling

1. Select peripheral
2. Select "STOP_POLL" command
3. Click **Send**

Or click **Stop All Polling** button to stop all peripherals.

## Commands

### Generic Commands (All Peripherals)

| Command | Code | Description |
|---------|------|-------------|
| GET_ALL | 0x00 | Get all data from peripheral (one-time) |
| GET_STATUS | 0x01 | Get status/health |
| SET_POLL_RATE | 0x02 | Start autonomous polling (requires interval) |
| STOP_POLL | 0x03 | Stop autonomous polling |

### System Commands (PID=0x00 only)

| Command | Code | Description |
|---------|------|-------------|
| SYSTEM_STATUS | 0x20 | Get full system status (20 bytes) |
| SYSTEM_WAKEUP | 0x21 | Wake from low-power state |
| SYSTEM_SLEEP | 0x22 | Enter low-power state |
| SYSTEM_RESET | 0x23 | Reset ESP32 |

## Peripherals

| Name | ID | Description |
|------|--------|-------------|
| SYSTEM | 0x00 | ESP32 system control |
| LORA_915 | 0x01 | 915MHz LoRa module |
| RADIO_433 | 0x02 | 433MHz radio |
| BAROMETER | 0x03 | MS5607 barometer |
| CURRENT | 0x04 | Current/voltage sensor |

## Example Workflow

### Get LoRa Data (One-Time)

1. Connect to ESP32
2. Click **"Get All LoRa"** button
3. See response in log:
   ```
   [TX] LORA_915: GET_ALL
        <AA5501010000AA55AA>
   [RX] LORA_915 (PID=01): 74 bytes
        <AA55014A[148 hex chars]CHKSUM55AA>
        Payload: 01000000000000...
        LoRa Data (WireLoRa_t): 74 bytes
   ```

### Start Autonomous Barometer Polling

1. Select "BAROMETER" peripheral
2. Select "SET_POLL_RATE" command
3. Enter "500" for interval (500ms)
4. Click **Send**
5. See ACK response
6. Barometer data arrives automatically every 500ms!

### Monitor System

1. Click **"Get System Status"** periodically
2. View:
   - Uptime
   - Commands received/executed
   - Queue size
   - Frame errors

## Log Display

### Color Codes
- **Blue (TX)**: Transmitted commands
- **Green (RX)**: Received responses
- **Red (ERROR)**: Errors, invalid frames
- **Purple (INFO)**: Status messages

### Response Types
- **ACK (0x01)**: Command acknowledged
- **ERROR (0xFF)**: Command failed (error code follows)
- **Data**: Peripheral data (74 bytes LoRa, 17 bytes barometer, etc.)

## Troubleshooting

### No COM Port Visible
- Check USB connection
- Click "Refresh" button
- Try different USB cable/port

### Connection Fails
- Check ESP32 is powered on
- Try different baud rate (115200 is default)
- Close other serial monitors (Arduino IDE, PlatformIO)

### No Response
- Check ESP32 is running hybrid protocol firmware
- Verify baud rate matches (115200)
- Check log for TX message (blue) - if visible, command was sent
- Try "Get System Status" - simplest command

### Invalid Frame Errors
- ESP32 might be sending debug prints - disable in firmware
- Check for correct firmware (hybrid protocol mode)
- Clear log and try again

## Protocol Details

### Frame Format

```
<AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
```

Example:
```
<AA5503030202F40130A555AA>\n
 │  │  │ │ └─ Payload: 02F401 (interval=1000ms)
 │  │  │ └─ LEN=03 (3 bytes)
 │  │  └─ PID=03 (BAROMETER)
 │  └─ Start markers (AA55)
 └─ Frame start '<'
```

### Checksum Calculation

```python
checksum = PID ^ LEN ^ PAYLOAD[0] ^ PAYLOAD[1] ^ ... ^ PAYLOAD[n-1]
```

## Tips

### Testing Without Hardware
- ESP32 in TEST_MODE generates fake sensor data
- All commands work, data is simulated
- Great for testing protocol/GUI

### Monitoring Autonomous Data
- Autonomous data appears in main log
- Use "Clear Log" to reset view
- Stop polling before testing other commands (reduces noise)

### Quick Testing
- Use quick action buttons for common operations
- No need to select dropdowns manually
- Faster workflow for repetitive tests

## Summary

This tool makes it easy to:
- ✅ Test all ESP32 commands interactively
- ✅ Monitor real-time responses
- ✅ Set up autonomous polling
- ✅ Debug protocol issues
- ✅ Experiment with different peripherals

Much easier than typing hex commands manually!
