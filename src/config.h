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

// Current Sensor Pin (Analog)
#define CURRENT_SENSOR_PIN 10 // might be wrong

// Raspberry Pi Communication  uses GPIO 19/20


// #define PI_UART_TX      43
// #define PI_UART_RX      44
// #define PI_UART_BAUD    115200

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
#define GOODBYE_BYTE    0x7F

// Peripheral/Origin IDs
#define PERIPHERAL_ID_SYSTEM        0x00
#define PERIPHERAL_ID_LORA_915      0x01
#define PERIPHERAL_ID_RADIO_433     0x02
#define PERIPHERAL_ID_BAROMETER     0x03
#define PERIPHERAL_ID_CURRENT       0x04
#define PERIPHERAL_ID_EXTERNAL_1    0x10  // For future plug-in modules
#define PERIPHERAL_ID_EXTERNAL_2    0x11
#define PERIPHERAL_ID_EXTERNAL_3    0x12

// System Commands (PERIPHERAL_ID_SYSTEM)
#define CMD_SYSTEM_WAKEUP           0x01
#define CMD_SYSTEM_STATUS           0x02
#define CMD_SYSTEM_SLEEP            0x03
#define CMD_SYSTEM_RESET            0x04

// Data Request Commands
#define CMD_GET_LORA_DATA       0x01
#define CMD_GET_433_DATA        0x02
#define CMD_GET_BAROMETER_DATA  0x03
#define CMD_GET_CURRENT_DATA    0x04
#define CMD_GET_ALL_DATA        0x05
#define CMD_GET_STATUS          0x06

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

#endif // CONFIG_H