# ESP32 Ground Station Testing Tools

This directory contains Python scripts for testing the ESP32 Ground Station communication protocol **without physical hardware**.

## Overview

The testing setup consists of two Python scripts that communicate over virtual serial ports:

1. **`esp32_simulator.py`** - Simulates the ESP32 embedded system
   - Implements the binary communication protocol
   - Generates realistic fake sensor data
   - Responds to commands just like the real hardware

2. **`pi_client.py`** - Raspberry Pi client (your actual Pi code)
   - Sends commands to request sensor data
   - Parses binary responses
   - Can be used with the simulator OR real hardware later

## Setup

### Requirements

```bash
pip install pyserial
```

### Creating Virtual Serial Ports

Since you don't have the hardware yet, you need to create **virtual serial port pairs** that connect the two scripts.

#### Windows

1. **Download and install [com0com](https://sourceforge.net/projects/com0com/)**
   - Free virtual serial port driver
   - Creates paired COM ports (e.g., COM10 ↔ COM11)

2. **Or use [Virtual Serial Port Driver](https://www.virtual-serial-port.org/)**
   - More user-friendly interface
   - Creates virtual port pairs

3. After installation, create a pair like `COM10` ↔ `COM11`

#### Linux/Mac

Use `socat` to create virtual serial port pairs:

```bash
# Install socat
sudo apt-get install socat  # Linux
brew install socat          # Mac

# Create a virtual serial port pair
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

This will output something like:
```
PTY is /dev/pts/2
PTY is /dev/pts/3
```

These are your paired ports!

## Running the Tests

### Step 1: Start the ESP32 Simulator

Open a terminal and run:

```bash
# Windows
python test/esp32_simulator.py COM10

# Linux/Mac
python test/esp32_simulator.py /dev/pts/2
```

You should see:
```
ESP32 Simulator started on COM10
Waiting for commands from Pi...

=== ESP32 Simulator Ready ===
Listening for commands...
Press Ctrl+C to exit
```

### Step 2: Start the Pi Client

Open a **second terminal** and run:

```bash
# Windows (use the OTHER port in the pair)
python test/pi_client.py COM11

# Linux/Mac
python test/pi_client.py /dev/pts/3
```

You should see the interactive menu:

```
==================================================
ESP32 Ground Station Test Client
==================================================
1. Get LoRa Data
2. Get 433MHz Data
3. Get Barometer Data
4. Get Current Sensor Data
5. Get System Status
6. Get All Data
7. Run Continuous Test (all sensors, 1 sec interval)
0. Exit
==================================================
Enter choice:
```

### Step 3: Test the Protocol!

Choose an option (e.g., `5` for System Status):

**Pi Client Terminal:**
```
>>> Sent command: 0x06 (1 bytes)
<<< Received response: 20 bytes from peripheral 0x00
  System Status (v1):
    Uptime: 45 seconds
    System State: 1
    LoRa: Online (3 packets)
    433MHz: Online (2 packets)
    Barometer: Online
    Current Sensor: Online
    Pi Connected: True
    Free Heap: 100000 bytes
    Chip Revision: 3
```

**ESP32 Simulator Terminal:**
```
Received HELLO (0x7E)
Peripheral ID: 0x00
Message length: 1 bytes
Received message data (1 bytes)
Received GOODBYE (0x7F)
Processing command: 0x06
  Command: GET_STATUS
  -> Sent response: 20 bytes
```

## Test Scenarios

### Quick Test - Get Status
```
Choice: 5
```
Tests basic communication and system status

### Full Test - Get All Data
```
Choice: 6
```
Tests receiving all sensor data in one request (LoRa + 433MHz + Barometer + Current)

### Continuous Monitoring
```
Choice: 7
```
Polls all sensors every second - simulates real-time monitoring

Watch the packet counts increase and sensor values change!

## How It Works

### Protocol Flow

```
Pi Client                           ESP32 Simulator
---------                           ---------------
1. Send command frame:
   [HELLO][ID][LEN][CMD][GOODBYE] ────────>

2. Receive & parse                 <- State machine processes
                                   <- Generates fake sensor data
                                   <- Packs into binary format

3.                                 <──────── Send response frame:
                                             [HELLO][ID][LEN][DATA][GOODBYE]

4. Receive & unpack data
5. Display results
```

### Binary Data Structures

All data is packed using **little-endian** byte order and matches the C structs in `config.h`:

- **WireLoRa_t**: 74 bytes - LoRa packet data
- **Wire433_t**: 70 bytes - 433MHz packet data
- **WireBarometer_t**: 17 bytes - Pressure, temp, altitude
- **WireCurrent_t**: 19 bytes - Current, voltage, power
- **WireStatus_t**: 20 bytes - System status

## Using With Real Hardware

Once you have the ESP32 hardware:

1. Upload your C++ code to the ESP32
2. Connect ESP32 to your PC/Pi via USB
3. Run **only** the Pi client:
   ```bash
   python test/pi_client.py /dev/ttyUSB0  # Linux
   python test/pi_client.py COM5          # Windows
   ```

The Pi client script works with **both** the simulator and real hardware!

## Troubleshooting

### "Permission denied" on Linux
```bash
sudo chmod 666 /dev/pts/X
# or add your user to dialout group
sudo usermod -a -G dialout $USER
```

### Timeout errors
- Make sure both scripts use the **paired** serial ports
- Check that the simulator is running before starting the client
- Increase timeout in `pi_client.py` if needed

### Data mismatch
- Verify HELLO_BYTE and GOODBYE_BYTE match between scripts and C++ code
- Check struct packing format strings match the C structs

## Next Steps

- Modify `esp32_simulator.py` to inject specific test scenarios (errors, timeouts, etc.)
- Add logging to file for analysis
- Create automated test scripts
- Port `pi_client.py` code to your actual Raspberry Pi application

## Files

- **esp32_simulator.py** - ESP32 hardware simulator (~330 lines)
- **pi_client.py** - Raspberry Pi client (~370 lines)
- **README.md** - This file

Both scripts are **pure Python** implementations of the binary protocol defined in your C++ code.
