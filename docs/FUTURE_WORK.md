# Future Work & Planned Improvements

This document outlines planned features and improvements for the Meerkat Embedded Routine project.

## Table of Contents
1. [Peripheral-Specific Commands](#peripheral-specific-commands)
2. [SD Card Data Logging](#sd-card-data-logging)
3. [AIM Board Integration](#aim-board-integration)
4. [Protocol Enhancements](#protocol-enhancements)
5. [Performance Optimizations](#performance-optimizations)
6. [Testing & Validation](#testing--validation)

---

## Peripheral-Specific Commands

**Status:** Planned
**Priority:** Medium
**Estimated Effort:** 2-3 days

### Overview
Implement peripheral-specific commands (0x10-0x1F range) to allow fine-grained control of individual sensors and radios beyond the generic commands.

### Proposed Commands

#### LoRa-Specific (PID 0x01)
```c
#define CMD_LORA_GET_SNR        0x10  // Get only SNR value (faster than full data)
#define CMD_LORA_GET_RSSI       0x11  // Get only RSSI value
#define CMD_LORA_SET_FREQUENCY  0x12  // Change frequency (payload: freq_hz)
#define CMD_LORA_SET_POWER      0x13  // Set TX power (payload: dBm)
#define CMD_LORA_SET_SPREADING  0x14  // Set spreading factor (payload: SF)
#define CMD_LORA_SET_BANDWIDTH  0x15  // Set bandwidth (payload: BW)
#define CMD_LORA_SEND_PACKET    0x16  // Transmit packet (payload: data)
```

#### 433MHz Radio-Specific (PID 0x02)
```c
#define CMD_433_GET_SNR         0x10  // Get only SNR value
#define CMD_433_GET_RSSI        0x11  // Get only RSSI value
#define CMD_433_SET_FREQUENCY   0x12  // Change frequency
#define CMD_433_SET_POWER       0x13  // Set TX power
#define CMD_433_SEND_PACKET     0x16  // Transmit packet
```

#### Barometer-Specific (PID 0x03)
```c
#define CMD_BARO_GET_PRESSURE   0x10  // Get only pressure (4 bytes, faster)
#define CMD_BARO_GET_TEMP       0x11  // Get only temperature (4 bytes)
#define CMD_BARO_GET_ALTITUDE   0x12  // Get only altitude
#define CMD_BARO_CALIBRATE      0x13  // Calibrate sensor (payload: reference pressure)
#define CMD_BARO_SET_OVERSAMPLING 0x14  // Set oversampling rate
```

#### Current Sensor-Specific (PID 0x04)
```c
#define CMD_CURRENT_GET_VOLTAGE  0x10  // Get only voltage (4 bytes)
#define CMD_CURRENT_GET_CURRENT  0x11  // Get only current (4 bytes)
#define CMD_CURRENT_GET_POWER    0x12  // Get only power
#define CMD_CURRENT_CALIBRATE    0x13  // Zero-point calibration
#define CMD_CURRENT_SET_SHUNT    0x14  // Set shunt resistor value
```

### Benefits
- Faster responses (only requested data)
- More control over sensor configuration
- Ability to configure sensors on-the-fly
- Transmit capabilities for radio modules

### Implementation Notes
- Add command handlers in `handleCommand()` switch statement
- Each peripheral validates its own specific commands
- Return ERROR_INVALID_CMD for unsupported commands
- Add to Interactive Sender GUI as advanced options

---

## SD Card Data Logging

**Status:** Planned
**Priority:** High
**Estimated Effort:** 5-7 days

### Overview
Implement automatic background data logging to SD card as a **backup system**. The ESP32 continuously writes sensor data to the SD card in the background while simultaneously sending data over serial to the host. This provides data redundancy and ensures no data loss if the serial connection fails or the host goes down.

### Requirements

#### Hardware
- MicroSD card module (SPI interface)
- Connections:
  ```
  ESP32          SD Card Module
  GPIO18    →    SCK
  GPIO19    →    MISO
  GPIO23    →    MOSI
  GPIO5     →    CS
  3.3V      →    VCC
  GND       →    GND
  ```

#### Features
- **Automatic Background Logging**: Always-on logging that runs transparently alongside serial communication
- **Data Redundancy**: Every sensor reading sent over serial is also written to SD card
- **Timestamped Files**: Create new file per session or per day
- **Binary Format**: Efficient storage using same WireLoRa_t, Wire433_t structures
- **File Rotation**: Auto-delete old files when SD card fills
- **No Host Required**: Logging runs independently - ESP32 can operate completely standalone
- **Optional Retrieval**: Basic status/file commands for manual data recovery if needed

### Proposed File Structure

```
/logs/
  ├── session_20250210_120530.bin   # Binary log file
  ├── session_20250210_120530.meta  # Metadata (JSON/text)
  ├── session_20250211_083245.bin
  └── session_20250211_083245.meta

/config/
  └── logging_config.json            # Logging configuration

/stats/
  └── transfer_stats.bin             # Already implemented!
```

### Binary Log Format

```
File Header (32 bytes):
  - Magic number: 0xAA55AA55 (4 bytes)
  - Version: 1 (1 byte)
  - Session timestamp: Unix timestamp (4 bytes)
  - Sampling interval: ms (2 bytes)
  - Enabled peripherals: bitfield (1 byte)
  - Reserved: (20 bytes)

Data Records (variable):
  - Timestamp: Unix timestamp (4 bytes)
  - PID: Peripheral ID (1 byte)
  - Length: Data length (1 byte)
  - Data: Raw peripheral data (Length bytes)
  - CRC16: Record checksum (2 bytes)
```

### New System Commands

**Note:** These are minimal - logging runs automatically in the background. Commands are only for status monitoring and manual file management.

```c
#define CMD_SYSTEM_SD_STATUS      0x27  // Get SD card status (size, free, current file, records written)
#define CMD_SYSTEM_SD_START_LOG   0x28  // Enable logging (default: auto-start on boot)
#define CMD_SYSTEM_SD_STOP_LOG    0x29  // Disable logging (for maintenance)
#define CMD_SYSTEM_SD_LIST_FILES  0x2A  // List log files (optional - for manual recovery)
#define CMD_SYSTEM_SD_DELETE_FILE 0x2C  // Delete file (for freeing space)
#define CMD_SYSTEM_SD_FORMAT      0x2D  // Format SD card (careful!)
```

**Removed:** `CMD_SYSTEM_SD_GET_FILE` - Not needed for automatic backup. Files can be retrieved by removing SD card and reading directly on PC/Pi.

### Implementation Tasks

1. **SD Card Driver Integration**
   - Use Arduino SD library or SdFat
   - Initialize in `setup()` - create session file immediately
   - Handle card insertion/removal detection (optional)

2. **Automatic Logging Class** (`SDLogger.h/cpp`)
   ```cpp
   class SDLogger {
   public:
       bool begin();  // Initialize SD, create session file
       void logData(uint8_t pid, const uint8_t* data, size_t len);  // Write record
       void flush();  // Flush buffer to SD
       // Minimal status/management methods
   private:
       File current_file;
       uint8_t write_buffer[512];  // Circular buffer
       uint32_t records_written;
   };
   ```

3. **Integration into sendHybridFrame()**
   - **Key change:** After sending frame over serial, immediately log to SD
   ```cpp
   void sendHybridFrame(uint8_t pid, const uint8_t* payload, size_t len) {
       // Send over serial (existing code)
       Serial.write(START_FRAME, 2);
       // ... serial write ...

       // NEW: Automatically log to SD card
       sdLogger.logData(pid, payload, len);
   }
   ```

4. **Priority in Main Loop**
   - Add **Priority 7**: Flush SD write buffer (every 5 seconds or when buffer fills)
   - Non-blocking - uses buffered writes
   ```cpp
   // Priority 7: Flush SD buffer
   if (sdLogger.hasBufferedData() && (millis() - last_flush > 5000)) {
       sdLogger.flush();
       last_flush = millis();
   }
   ```

5. **File Rotation Logic**
   - Auto-create new file daily or when file exceeds 10MB
   - Auto-delete oldest files when SD >90% full
   - Keeps most recent data

6. **Interactive Sender Updates** (Minimal)
   - SD status display (card size, free space, records written)
   - Enable/disable logging toggle
   - No file browser needed - just remove SD card to access files directly

### Benefits
- **Data Redundancy**: Every sensor reading is automatically backed up - no data loss if serial fails
- **Standalone Operation**: ESP32 can operate completely independently without host
- **Simple Recovery**: Remove SD card and read files directly on PC/Pi (no special transfer protocol needed)
- **Transparent**: Logging happens in background - no changes needed to serial protocol or host scripts
- **Field Testing**: Capture complete dataset during autonomous field deployments

---

## AIM Board Integration

**Status:** Planned
**Priority:** High
**Estimated Effort:** 10-15 days

### Overview
Full integration and testing of AIM (Autonomous Instrument Module) boards for multi-sensor networks.

### Current Status
- PIDs reserved: 0x10-0x13 (4 AIM boards)
- No implementation yet

### Requirements

#### Hardware
- 4x AIM boards (custom PCBs with multiple sensors)
- I2C or SPI communication bus
- Each board has unique address

#### AIM Board Capabilities (Assumed)
- Multiple sensors per board:
  - Temperature sensors
  - Humidity sensors
  - Gas sensors
  - Light sensors
  - Accelerometers
- Onboard microcontroller (e.g., ATmega328P, STM32)
- I2C/SPI slave interface
- Interrupt-driven data ready signals

### Proposed Implementation

#### 1. Communication Protocol

**SPI Bus (Alternative):**
```
ESP32              AIM Boards
GPIO18 (SCK)  ──────┬── All boards
GPIO19 (MISO) ──────┤
GPIO23 (MOSI) ──────┤
GPIO5  (CS0)  ──────── AIM1
GPIO17 (CS1)  ──────── AIM2
GPIO16 (CS2)  ──────── AIM3
GPIO4  (CS3)  ──────── AIM4
```

#### 2. Data Structures

```cpp
// Wire format for AIM board data
struct WireAIM_t {
    uint8_t version;           // Protocol version
    uint8_t board_id;          // Board ID (0x10-0x13)
    uint8_t sensor_count;      // Number of sensors on this board
    uint32_t timestamp_ms;     // Measurement timestamp
    uint8_t status_flags;      // Sensor status bitfield

    // Variable sensor data (depends on board type)
    struct {
        uint8_t sensor_type;   // Sensor type identifier
        uint8_t data_len;      // Length of sensor data
        uint8_t data[16];      // Sensor data (max 16 bytes per sensor)
    } sensors[8];              // Up to 8 sensors per board
} __attribute__((packed));
```

#### 3. AIM-Specific Commands

```c
// Generic AIM commands (work for all AIM boards)
#define CMD_AIM_GET_INFO        0x10  // Get board info (sensors, firmware ver)
#define CMD_AIM_GET_SENSOR      0x11  // Get specific sensor data (payload: sensor ID)
#define CMD_AIM_CALIBRATE       0x12  // Calibrate sensors
#define CMD_AIM_SET_SAMPLE_RATE 0x13  // Set sampling rate per sensor
#define CMD_AIM_RESET           0x14  // Reset AIM board
#define CMD_AIM_FIRMWARE_UPDATE 0x15  // Update AIM firmware (advanced)
```

#### 4. Implementation Classes

**AIMBoard.h/cpp:**
```cpp
class AIMBoard {
private:
    uint8_t board_id;
    uint8_t i2c_address;
    bool online;
    uint32_t last_poll_time;

public:
    AIMBoard(uint8_t id, uint8_t addr);
    bool begin();
    bool isOnline();
    bool getData(uint8_t* buffer, size_t max_len);
    bool getSensorData(uint8_t sensor_id, uint8_t* data, size_t* len);
    bool calibrate();
    bool setSampleRate(uint8_t sensor_id, uint16_t rate_ms);
};
```

**AIMManager.h/cpp:**
```cpp
class AIMManager {
private:
    AIMBoard* boards[4];  // 4 AIM boards

public:
    AIMManager();
    bool begin();
    bool pollBoard(uint8_t board_id);
    size_t packBoardData(uint8_t board_id, uint8_t* buffer, size_t max_len);
    bool sendCommand(uint8_t board_id, uint8_t cmd, uint8_t* payload, size_t len);
};
```

#### 5. Integration Points

**In main.cpp:**
```cpp
// Add to global instances
AIMManager aimManager;

// In setup()
aimManager.begin();

// In updatePeripherals()
if (now - aim_poll_interval > AIM_POLL_RATE) {
    for (int i = 0; i < 4; i++) {
        aimManager.pollBoard(0x10 + i);
    }
}

// In handleCommand()
case CMD_GET_ALL:
    if (cmd.peripheralId >= 0x10 && cmd.peripheralId <= 0x13) {
        // Handle AIM board request
        size_t len = aimManager.packBoardData(cmd.peripheralId, buffer, size);
        sendHybridFrame(cmd.peripheralId, buffer, len);
    }
    break;
```

### Testing Plan

1. **Single Board Testing**
   - Test communication with one AIM board
   - Verify all sensors readable
   - Test polling rates

2. **Multi-Board Testing**
   - Test all 4 boards simultaneously
   - Verify no I2C/SPI conflicts
   - Test autonomous polling

3. **Stress Testing**
   - High-frequency polling (all boards at 10ms)
   - Long-duration testing (24+ hours)
   - Error recovery (disconnect/reconnect boards)

4. **Integration Testing**
   - Test with other peripherals (LoRa, Barometer, etc.)
   - Test with autonomous polling
   - Test with SD card logging

### Benefits
- **Scalability**: Support multiple sensor nodes
- **Modularity**: Easy to add/remove AIM boards
- **Flexibility**: Different sensor configurations per board
- **Network Architecture**: Foundation for sensor mesh network

---

## Protocol Enhancements

**Status:** Planned
**Priority:** Low-Medium
**Estimated Effort:** 3-5 days

### 1. Command Acknowledgment Options

Currently all commands either send data or ACK. Add option to disable ACK for performance:

```c
#define CMD_FLAG_NO_ACK  0x80  // OR with command byte to disable ACK

// Example usage:
uint8_t cmd = CMD_SET_POLL_RATE | CMD_FLAG_NO_ACK;  // No ACK needed
```

### 2. Bulk Data Transfer

For large data transfers (SD card files, firmware updates):

```c
#define CMD_SYSTEM_BULK_START   0x2E  // Start bulk transfer session
#define CMD_SYSTEM_BULK_DATA    0x2F  // Bulk data chunk
```

**Protocol:**
- Start session with metadata (total size, chunk size, checksum)
- Send chunks with sequence numbers
- Receiver ACKs each chunk or requests retransmission
- End with final checksum verification

### 3. Error Reporting Enhancement

Add detailed error information:

```cpp
struct WireError_t {
    uint8_t version;
    uint8_t error_code;      // ERROR_INVALID_CMD, etc.
    uint8_t peripheral_id;    // Which peripheral had the error
    uint32_t timestamp_ms;    // When error occurred
    char message[32];         // Human-readable error message
} __attribute__((packed));
```

### 4. Heartbeat/Keepalive

Automatic heartbeat to detect connection loss:

```c
#define CMD_SYSTEM_HEARTBEAT    0x30  // Auto-sent every N seconds
```

Configure via:
```c
#define CMD_SYSTEM_SET_HEARTBEAT 0x31  // Enable/disable, set interval
```

---

## Performance Optimizations

**Status:** Ongoing
**Priority:** Medium

### 1. DMA for Serial Communication

Use DMA transfers for UART to reduce CPU overhead:
- ESP32 UART DMA for TX/RX
- Frees CPU for sensor processing
- Lower latency, higher throughput

### 2. FreeRTOS Multi-Core

Optional FreeRTOS mode for ESP32 dual-core:
- Core 0: Serial communication, command processing
- Core 1: Sensor polling, data collection
- Requires careful synchronization

### 3. Compile-Time Configuration

Add build flags to disable unused peripherals:
```ini
build_flags =
  -D ENABLE_LORA=1
  -D ENABLE_433=0        # Disable 433MHz radio
  -D ENABLE_BAROMETER=1
  -D ENABLE_CURRENT=0    # Disable current sensor
  -D ENABLE_AIM=0        # Disable AIM boards
  -D ENABLE_SD_LOGGING=1
```

Benefits:
- Smaller firmware size
- Faster compile times
- Lower memory usage
- Easier testing of specific features

---

## Testing & Validation

**Status:** Ongoing
**Priority:** High

### 1. Automated Test Suite

Create comprehensive test suite:

**Unit Tests:**
- Protocol encoding/decoding
- Checksum calculation
- Frame validation
- Command parsing

**Integration Tests:**
- End-to-end communication
- Autonomous polling
- Error handling
- Statistics tracking

**Stress Tests:**
- High-frequency polling (1000 messages/sec)
- Long-duration runs (24+ hours)
- Memory leak detection
- Queue overflow handling

### 2. Hardware-in-Loop Testing

Automated testing with real hardware:
- Raspberry Pi controls test sequence
- Verifies all peripherals respond correctly
- Checks timing and latency
- Validates data integrity

### 3. Continuous Integration

Set up CI/CD pipeline:
- Auto-build on commit (PlatformIO)
- Run unit tests
- Generate documentation
- Create release binaries

### 4. Field Testing

Real-world validation:
- High-altitude balloon test
- Temperature extremes (-40°C to +85°C)
- Vibration testing
- Long-duration autonomous operation

---

## Documentation Improvements

**Status:** Ongoing
**Priority:** Medium

### Planned Additions

1. **Video Tutorials**
   - Getting started guide
   - Flashing firmware walkthrough
   - Interactive Sender demo
   - Troubleshooting common issues

2. **API Reference**
   - Complete command reference
   - All data structures documented
   - Code examples for each command

3. **Architecture Diagrams**
   - System block diagram
   - Data flow diagrams
   - State machine diagrams
   - Timing diagrams

4. **Application Notes**
   - Optimal polling rates
   - Power consumption analysis
   - Bandwidth calculations
   - Error recovery strategies

---

## Contributing

Interested in implementing any of these features? Here's how to contribute:

1. **Check existing issues** on GitHub for the feature
2. **Create a feature branch**: `git checkout -b feature/sd-card-logging`
3. **Follow code style** in existing files
4. **Add tests** for new functionality
5. **Update documentation** as you go
6. **Submit pull request** with detailed description

### Priority Order (Recommended)

For maximum impact, implement in this order:

1. **SD Card Logging** - High value for autonomous operation
2. **AIM Board Integration** - Core functionality for sensor network
3. **Peripheral-Specific Commands** - Enhanced control and flexibility
4. **Protocol Enhancements** - Better error handling and bulk transfers
5. **Performance Optimizations** - After core features stable
6. **Testing & Validation** - Throughout all development

---

## Questions or Suggestions?

Have ideas for other improvements? Open an issue on GitHub or submit a pull request with your proposed additions to this document!

**Last Updated:** 2025-01-10
**Maintained By:** [Your Name/Team]
