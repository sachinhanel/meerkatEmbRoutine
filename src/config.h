#ifndef CONFIG_H
#define CONFIG_H

// ====================================================================
// ESP32 Single-Threaded Firmware Configuration
// ====================================================================
// This firmware uses a priority-based event loop with command queue
// - No FreeRTOS tasks
// - Hybrid protocol only (binary + newline framing)
// - Non-blocking peripheral polling

// Pin Definitions for ESP32-S3
// SPI Pins (shared between LoRa and 433MHz modules)
#define SPI_SCK_PIN     11
#define SPI_MISO_PIN    13
#define SPI_MOSI_PIN    12
//_____

// LoRa 915MHz Module specific Pins
#define LORA_CS_PIN     41
#define LORA_RST_PIN    -1 //UNCONNECTED
#define LORA_DIO0_PIN   40

// 433MHz Radio Module specific Pins
#define RADIO433_CS_PIN 42
#define RADIO433_RST_PIN -1 //UNCONNECTED
#define RADIO433_DIO0_PIN 39

// Barometer Pins (I2C)
#define I2C_SDA_PIN     9
#define I2C_SCL_PIN     8
#define MS5607_I2C_ADDRESS 0x77  // MS5607 I2C address (can also be 0x76)

// Current Sensor Pin (Analog)
#define CURRENT_SENSOR_PIN 10 // might be wrong

// Raspberry Pi Communication
// Uses GPIO 19/20 (USB serial), configured automatically by Arduino framework
// Baud rate: 115200 (set in main.cpp)

// WiFi Configuration (optional, for remote debugging)
#define WIFI_SSID       "Telstra1842A7_EXT"
#define WIFI_PASSWORD   "hpr9a26puk"
#define TELNET_PORT     23

// Future Expansion:
// - Status LEDs (error, power indicators)
// - AIM board communication (backplane interface)
// - SD card logging (with automatic space management)


// ====================================================================
// HYBRID PROTOCOL - Binary + ASCII Hex Framing + Newline
// ====================================================================
//
// Format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
//
// Frame Structure:
//   '<'          - Start marker (1 byte ASCII)
//   'AA55'       - Protocol header (4 bytes ASCII hex)
//   [PID]        - Peripheral ID (2 bytes ASCII hex, e.g., "01" = 0x01)
//   [LEN]        - Payload length (2 bytes ASCII hex, e.g., "4A" = 74 bytes)
//   [PAYLOAD]    - Payload data (LEN*2 bytes ASCII hex, e.g., "0123" = 0x01,0x23)
//   [CHK]        - Checksum: PID ^ LEN ^ payload bytes (2 bytes ASCII hex)
//   '55AA'       - Protocol footer (4 bytes ASCII)
//   '>'          - End marker (1 byte ASCII)
//   '\n'         - Newline terminator
//
// Benefits:
//   - Human-readable in terminal/serial monitor
//   - Binary data encoded as ASCII hex
//   - Newline-delimited for easy parsing
//   - Checksum for error detection
//
// Examples:
//   Get LoRa data:  <AA55010100010155AA>
//   Response:       <AA550104A[148 hex chars]XX55AA>
//   Error response: <AA5501 02FF04XX55AA> (FF = error, 04 = invalid PID)
//
// Payload Format:
//   First byte: Command byte (0x00-0xFF)
//   Remaining bytes: Command-specific parameters or response data
//
// Command Ranges:
//   0x00-0x0F: Generic commands (work for ALL peripherals)
//   0x10-0x1F: Peripheral-specific commands
//   0x20-0x2F: System commands (only for PID=0x00)
//
// ====================================================================

// ====================================================================
// PERIPHERAL IDs - One ID per device/subsystem
// ====================================================================
#define PERIPHERAL_ID_SYSTEM        0x00  // ESP32 system control
#define PERIPHERAL_ID_LORA_915      0x01  // 915MHz LoRa module
#define PERIPHERAL_ID_LORA_433      0x02  // 433MHz LoRa module (same chip as 915, different freq)
#define PERIPHERAL_ID_BAROMETER     0x03  // MS5607 barometer
#define PERIPHERAL_ID_CURRENT       0x04  // Current/voltage sensor
#define PERIPHERAL_ID_AIM_1         0x10  // AIM board 1 (future)
#define PERIPHERAL_ID_AIM_2         0x11  // AIM board 2 (future)
#define PERIPHERAL_ID_AIM_3         0x12  // AIM board 3 (future)
#define PERIPHERAL_ID_AIM_4         0x13  // AIM board 4 (future)
#define PERIPHERAL_ID_ALL           0xFF  // Special ID: targets all sensor peripherals

// Backward compatibility alias
#define PERIPHERAL_ID_RADIO_433     PERIPHERAL_ID_LORA_433

// ====================================================================
// GENERIC COMMANDS - Work for SENSOR peripherals ONLY (0x00-0x0F)
// NOT for SYSTEM peripheral (use 0x20-0x2F for system commands)
// ====================================================================
#define CMD_GET_ALL         0x00  // Get all available data from this peripheral (one-time)
#define CMD_GET_STATUS      0x01  // Get status/health of this peripheral
#define CMD_SET_POLL_RATE   0x02  // Set autonomous polling rate (payload: 2 bytes interval_ms)
#define CMD_STOP_POLL       0x03  // Stop autonomous polling (no payload)
// 0x04-0x0F reserved for future generic commands

// ====================================================================
// SYSTEM COMMANDS - Only for PERIPHERAL_ID_SYSTEM (0x20-0x2F)
// ====================================================================
#define CMD_SYSTEM_STATUS   0x20  // Get full WireStatus_t (20 bytes)
#define CMD_SYSTEM_WAKEUP   0x21  // Wake up system from low-power state
#define CMD_SYSTEM_SLEEP    0x22  // Put system into low-power state
#define CMD_SYSTEM_RESET    0x23  // Reset entire ESP32
#define CMD_SYSTEM_PERF     0x24  // Toggle performance stats output (payload: 1 byte, 0=off, 1=on)
// 0x25-0x2F reserved for future system commands

// ====================================================================
// PERIPHERAL-SPECIFIC COMMANDS (0x10-0x1F) - Future expansion
// ====================================================================
// Each peripheral can define its own commands starting at 0x10
// Example for future LoRa-specific commands:
// #define CMD_LORA_GET_SNR        0x10  // Get only SNR value
// #define CMD_LORA_GET_RSSI       0x11  // Get only RSSI value
// #define CMD_LORA_SET_FREQUENCY  0x12  // Change frequency (payload: freq)
//
// Example for future barometer-specific commands:
// #define CMD_BARO_GET_PRESSURE   0x10  // Get only pressure
// #define CMD_BARO_GET_TEMP       0x11  // Get only temperature
// #define CMD_BARO_CALIBRATE      0x12  // Calibrate sensor

// Buffer Sizes
#define LORA_BUFFER_SIZE        256
#define RADIO433_BUFFER_SIZE    128
#define MAX_PACKET_SIZE         64

// Timing Constants (milliseconds)
#define SENSOR_READ_INTERVAL    100
#define RADIO_LISTEN_TIMEOUT    50
#define PI_COMM_TIMEOUT         1000

// LoRa 915MHz Configuration
#define LORA_FREQUENCY          915E6
#define LORA_BANDWIDTH          125E3
#define LORA_SPREADING_FACTOR   7
#define LORA_CODING_RATE        5
#define LORA_TX_POWER           17

// 433MHz LoRa Configuration (same chip as 915MHz, different frequency)
#define RADIO433_FREQUENCY      433E6  // 433MHz in Hz
#define RADIO433_BANDWIDTH      125E3  // Same as 915MHz
#define RADIO433_SPREADING_FACTOR   7  // Same as 915MHz
#define RADIO433_CODING_RATE    5      // Same as 915MHz
#define RADIO433_TX_POWER       17     // Same as 915MHz

// System State (added)
typedef enum {
    SYSTEM_WAITING_FOR_WAKEUP = 0,
    SYSTEM_OPERATIONAL = 1,
    SYSTEM_SLEEPING = 2
} SystemState_t;

// System Status (expanded to include state and wakeup time)
typedef struct {
    SystemState_t system_state;
    uint32_t wakeup_time;
    bool lora_online;
    bool radio433_online;
    bool barometer_online;
    bool current_sensor_online;
    bool pi_connected;
    uint32_t uptime_seconds;
    uint16_t packet_count_lora;
    uint16_t packet_count_433;
} SystemStatus_t;

// Data Structures

typedef struct {
    uint32_t timestamp;
    int16_t rssi;
    float snr;
    uint8_t data_length;
    uint8_t data[MAX_PACKET_SIZE];
} LoRaPacket_t;

// 433MHz module is also LoRa (same chip, different frequency)
typedef struct {
    uint32_t timestamp;
    int16_t rssi;
    float snr;  // LoRa has SNR
    uint8_t data_length;
    uint8_t data[MAX_PACKET_SIZE];
} Radio433Packet_t;

typedef struct {
    uint32_t timestamp;
    float pressure_hpa;
    float temperature_c;
    float altitude_m;
} BarometerData_t;

typedef struct {
    uint32_t timestamp;
    float current_a;
    float voltage_v;
    float power_w;
} CurrentData_t;

// ===============================
// Binary wire protocol structures
// Keep frames under 255 bytes (1-byte LENGTH)
// Little-endian, packed
#if defined(__GNUC__)
#define PACKED __attribute__((packed))
#else
#define PACKED
#pragma pack(push, 1)
#endif

typedef struct PACKED {
    uint8_t version;           // 1
    uint16_t packet_count;     // 2
    int16_t rssi_dbm;          // 2
    float snr_db;              // 4
    uint8_t latest_len;        // 1
    uint8_t latest_data[64];   // 64
} WireLoRa_t;                   // = 74 bytes

// 433MHz LoRa module uses same structure as 915MHz (same chip, different frequency)
typedef struct PACKED {
    uint8_t version;           // 1
    uint16_t packet_count;     // 2
    int16_t rssi_dbm;          // 2
    float snr_db;              // 4  (LoRa has SNR)
    uint8_t latest_len;        // 1
    uint8_t latest_data[64];   // 64
} Wire433_t;                    // = 74 bytes (same as WireLoRa_t)

typedef struct PACKED {
    uint8_t version;           // 1
    uint32_t timestamp_ms;     // 4
    float pressure_hpa;        // 4
    float temperature_c;       // 4
    float altitude_m;          // 4
} WireBarometer_t;              // = 17 bytes

typedef struct PACKED {
    uint8_t version;           // 1
    uint32_t timestamp_ms;     // 4
    float current_a;           // 4
    float voltage_v;           // 4
    float power_w;             // 4
    int16_t raw_adc;           // 2
} WireCurrent_t;                // = 19 bytes

typedef struct PACKED {
    uint8_t version;           // 1
    uint32_t uptime_seconds;   // 4
    uint8_t system_state;      // 1
} WireHeartbeat_t;              // = 6 bytes (minimal "I'm alive" message)

typedef struct PACKED {
    uint8_t version;           // 1
    uint32_t uptime_seconds;   // 4
    uint8_t system_state;      // 1
    uint8_t flags;             // 1 bitfield: b0 lora_online, b1 r433_online, b2 baro_online, b3 curr_online, b4 pi_connected
    uint16_t packet_count_lora;// 2
    uint16_t packet_count_433; // 2
    uint32_t wakeup_time;      // 4
    uint32_t free_heap;        // 4
    uint8_t chip_revision;     // 1
} WireStatus_t;                 // = 20 bytes

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

#undef PACKED

#endif // CONFIG_H