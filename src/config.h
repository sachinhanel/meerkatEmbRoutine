#ifndef CONFIG_H
#define CONFIG_H

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

// Raspberry Pi Communication  uses GPIO 19/20 instead now, which is usb and doenst need to be set
// #define PI_UART_TX      43
// #define PI_UART_RX      44
// #define PI_UART_BAUD    115200

// WiFi Configuration (for remote monitoring in TEST_MODE)
#define WIFI_SSID       "Telstra1842A7_EXT"
#define WIFI_PASSWORD   "hpr9a26puk"
#define TELNET_PORT     23

//OTHER DEVICES TO BE ADDED
//1. ERROR LED, POWER LED
//2. AIM BOARD COMMUNICATION PINS TO BACKPLANE
//3. SD CARD (doesnt need to be read from the pi but needs to make sure that there is adequate space to log data, throw erorr or delete old fils automatically if not enough space)


//THINGS TO DO
//NEED TO CHANGE THE 915 AND 433 TO SEND THROUGH BINARY RATHER THAN JSON, AS JSON IS TOO SLOW, PERHAPS CHANGE THE OTHER PERIPHELS OR KEEP THE SAME
//CURRENT NEEDS VOLTAGE ALSO

//




// Communication Protocol Constants
#define HELLO_BYTE      0x7E
#define RESPONSE_BYTE   0x7D  // ESP32 -> Pi responses (different from HELLO to avoid echo confusion)
#define GOODBYE_BYTE    0x7F

// Peripheral/Origin IDs
#define PERIPHERAL_ID_SYSTEM        0x00
#define PERIPHERAL_ID_LORA_915      0x01
#define PERIPHERAL_ID_RADIO_433     0x02
#define PERIPHERAL_ID_BAROMETER     0x03
#define PERIPHERAL_ID_CURRENT       0x04
#define PERIPHERAL_ID_AIM_1         0x10  //AIM BOARD IDS
#define PERIPHERAL_ID_AIM_2         0x11
#define PERIPHERAL_ID_AIM_3         0x12
#define PERIPHERAL_ID_AIM_4         0x13

// System Commands (PERIPHERAL_ID_SYSTEM)
#define CMD_SYSTEM_WAKEUP           0x20
#define CMD_SYSTEM_STATUS           0x21
#define CMD_SYSTEM_SLEEP            0x22
#define CMD_SYSTEM_RESET            0x23

// Data Request Commands
#define CMD_GET_LORA_DATA       0x31
#define CMD_GET_433_DATA        0x32
#define CMD_GET_BAROMETER_DATA  0x33
#define CMD_GET_CURRENT_DATA    0x34
#define CMD_GET_ALL_DATA        0x35
#define CMD_GET_STATUS          0x36

// Buffer Sizes
#define LORA_BUFFER_SIZE        256
#define RADIO433_BUFFER_SIZE    128
#define MAX_PACKET_SIZE         64

// Timing Constants (milliseconds)
#define SENSOR_READ_INTERVAL    100
#define RADIO_LISTEN_TIMEOUT    50
#define PI_COMM_TIMEOUT         1000

// LoRa Configuration
#define LORA_FREQUENCY          915E6
#define LORA_BANDWIDTH          125E3
#define LORA_SPREADING_FACTOR   7
#define LORA_CODING_RATE        5
#define LORA_TX_POWER           17

// 433MHz Radio Configuration
#define RADIO433_FREQUENCY      433.0
#define RADIO433_BITRATE        4.8
#define RADIO433_FREQ_DEV       5.0

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

typedef struct {
    uint32_t timestamp;
    int16_t rssi;
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

typedef struct PACKED {
    uint8_t version;           // 1
    uint16_t packet_count;     // 2
    int16_t rssi_dbm;          // 2
    uint8_t latest_len;        // 1
    uint8_t latest_data[64];   // 64
} Wire433_t;                    // = 70 bytes

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