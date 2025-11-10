# TEST MODE Guide

## What is TEST MODE?

TEST MODE allows you to run and test the ESP32 Ground Station code **without any sensor hardware connected**. Perfect for:
- Testing the communication protocol with your Pi
- Developing on a basic ESP32-WROOM board
- Debugging without worrying about sensor initialization failures

## How to Use

### 1. Upload to ESP32 in TEST MODE

```bash
# Build and upload with TEST MODE enabled
pio run -e esp32-test -t upload

# Monitor the serial output
pio device monitor -e esp32-test
```

### 2. What You'll See

```
======================================
    ESP32 Rocketry Ground Station
      **TEST MODE ENABLED**
      No Hardware Required
======================================
ESP32 Chip Rev: 3
Free Heap: 245876 bytes

TEST MODE: All sensors simulated
Ready for Pi protocol testing
Waiting for commands on USB Serial...
======================================

DataCollector: TEST MODE - Simulating all sensors online
DataCollector: All fake sensors ready for testing!

Setup complete - ESP32 Ground Station is running
[TEST MODE] System idle, waiting for Pi commands...
```

### 3. Test with Pi Client

Now you can use the Pi client script to test the protocol:

```bash
# From another terminal
python test/pi_client.py COM3  # Use your ESP32's COM port
```

### 4. What Gets Simulated

In TEST MODE, all sensor data is **fake but realistic**:

| Sensor | Fake Data |
|--------|-----------|
| **LoRa 915MHz** | Packet count increments, RSSI: -85 dBm, SNR: 3.5 dB, Data: "TestLoRaPacket" |
| **433MHz Radio** | Packet count increments, RSSI: -92 dBm, Data: "Test433Packet" |
| **Barometer (MS5607)** | Pressure: 1013.25 hPa, Temp: 22.5°C, Altitude: 123.4 m |
| **Current Sensor** | Current: 0.5 A, Voltage: 12.3 V, Power: 6.15 W |
| **System Status** | All sensors "Online", Pi connected when actively communicating |

### 5. Commands You Can Try

From the Pi client menu:
- **Option 1**: Get LoRa Data - See fake LoRa packet
- **Option 5**: Get System Status - See all sensors online
- **Option 6**: Get All Data - Receive all sensor data at once
- **Option 7**: Continuous test - Poll all sensors every second

## Switching Back to Normal Mode

To run with real hardware:

```bash
# Upload normal firmware (requires all sensor hardware)
pio run -e esp32-uart -t upload
pio device monitor -e esp32-uart
```

## How It Works

### Compile-Time Flag

The `TEST_MODE` flag is set in `platformio.ini`:

```ini
[env:esp32-test]
build_flags =
  -D TEST_MODE=1
```

### Code Changes

When `TEST_MODE` is defined:

1. **DataCollector** skips sensor initialization
   - No LoRa, RadioLib, MS5611, or CurrentSensor objects created
   - All sensors marked as "online" immediately

2. **Binary packers** return fake data
   - `packLoRaData()`, `pack433Data()`, etc. generate realistic test values
   - No hardware access attempted

3. **Main.cpp** shows test status messages
   - Welcome message indicates TEST MODE
   - Heartbeat every 10 seconds shows system is alive

### No Hardware Crashes

Without TEST_MODE, the code would crash trying to initialize sensors that don't exist. With TEST MODE:
- ✅ No SPI initialization
- ✅ No I2C initialization
- ✅ No GPIO pin configuration
- ✅ No sensor library calls
- ✅ Just USB Serial + Binary Protocol

## Benefits

1. **Test Protocol Without Hardware**
   - Validate binary packing/unpacking
   - Debug communication state machine
   - Develop Pi client code

2. **Use Any ESP32 Board**
   - Works on ESP32-WROOM (your current board)
   - Works on ESP32-S3 (your target board)
   - Just needs USB connection

3. **Fast Development Cycle**
   - No sensor wiring needed
   - No initialization delays
   - Instant testing

## Troubleshooting

### "Brownout detector was triggered"
- Your ESP32 isn't getting enough power
- Try a different USB cable or port
- Some computers' USB ports provide insufficient current

### Still getting reboot loops
- Make sure you're using `esp32-test` environment
- Check that TEST_MODE messages appear on serial
- Verify you built with: `pio run -e esp32-test -t upload`

### Can't connect with Pi client
- Make sure you're using the correct COM port
- ESP32 must be powered and running (check serial monitor first)
- Try resetting the ESP32 after upload

### No heartbeat messages
- Check that communicationTask is running
- Serial monitor should show heartbeat every 10 seconds
- If not, the ESP32 may have crashed - check for exceptions

## Next Steps

Once protocol testing works:
1. Wire up actual sensors to ESP32-S3
2. Upload normal firmware: `pio run -e esp32-uart -t upload`
3. Test with real sensor data!

TEST MODE bridges the gap between development and deployment.
