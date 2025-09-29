/*
 * ESP32 Rocketry Ground Station - Single File Version
 * Complete system in one file for embedded deployment
 * 
 * Hardware Requirements:
 * - ESP32 Dev Board
 * - 915MHz LoRa Module (SPI)
 * - 433MHz Radio Module (SPI) 
 * - BME280 Barometer (I2C)
 * - Current Sensor (Analog)
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <LoRa.h>
#include <RadioLib.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

// ====================================================================
// CONFIGURATION AND CONSTANTS
// ====================================================================

// Pin Definitions
#define SPI_SCK_PIN     18
#define SPI_MISO_PIN    19
#define SPI_MOSI_PIN    23

// LoRa 915MHz Module Pins
#define LORA_CS_PIN     5
#define LORA_RST_PIN    14
#define LORA_DIO0_PIN   2

// 433MHz Radio Module Pins  
#define RADIO433_CS_PIN 15
#define RADIO433_RST_PIN 4
#define RADIO433_DIO0_PIN 16

// Barometer Pins (I2C)
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22

// Current Sensor Pin (Analog)
#define CURRENT_SENSOR_PIN 36

// Raspberry Pi Communication Pins (UART)
#define PI_UART_TX      17
#define PI_UART_RX      16
#define PI_UART_BAUD    115200

// Communication Protocol Constants
#define HELLO_BYTE      0xAA
#define GOODBYE_BYTE    0x55

// Data Request Commands
#define CMD_GET_LORA_DATA       0x01
#define CMD_GET_433_DATA        0x02
#define CMD_GET_BAROMETER_DATA  0x03
#define CMD_GET_CURRENT_DATA    0x04
#define CMD_GET_ALL_DATA        0x05
#define CMD_GET_STATUS          0x06

//i2c adresses for barometer
#define BME280_ADDRESS_PRIMARY 0x76
#define BME280_ADDRESS_ALTERNATE 0x77

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

// ====================================================================
// DATA STRUCTURES
// ====================================================================

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

typedef struct {
    bool lora_online;
    bool radio433_online;
    bool barometer_online;
    bool current_sensor_online;
    bool pi_connected;
    uint32_t uptime_seconds;
    uint16_t packet_count_lora;
    uint16_t packet_count_433;
} SystemStatus_t;

typedef enum {
    WAIT_FOR_HELLO,
    WAIT_FOR_COMMAND,
    WAIT_FOR_GOODBYE,
    SENDING_RESPONSE
} CommState_t;

// ====================================================================
// GLOBAL VARIABLES
// ====================================================================

// Sensor Hardware Objects
SX1278 radio433Module(new Module(RADIO433_CS_PIN, RADIO433_DIO0_PIN, RADIO433_RST_PIN));
Adafruit_BME280 bme280Sensor;
HardwareSerial piSerial(2);

// Data Buffers
LoRaPacket_t loraBuffer[LORA_BUFFER_SIZE];
Radio433Packet_t radio433Buffer[RADIO433_BUFFER_SIZE];
BarometerData_t latestBarometerData;
CurrentData_t latestCurrentData;

// Buffer Management
uint16_t loraBufferHead = 0, loraBufferTail = 0;
uint16_t radio433BufferHead = 0, radio433BufferTail = 0;
uint16_t loraPacketCount = 0, radio433PacketCount = 0;

// System Status
SystemStatus_t systemStatus;
uint32_t bootTime = 0;

// Communication State
CommState_t commState = WAIT_FOR_HELLO;
uint8_t receivedCommand = 0;
uint32_t lastActivityTime = 0;

// Sensor Status
bool loraInitialized = false;
bool radio433Initialized = false;
bool barometerInitialized = false;
bool currentSensorInitialized = false;

// Current Sensor Calibration
float voltageDividerRatio = 11.0;
float currentSensorSensitivity = 66.0; // mV/A for ACS712-30A
float voltageOffset = 0.0;
float currentOffset = 0.0;

// Current Sensor Filtering
const int CURRENT_SAMPLE_COUNT = 10;
float voltageSamples[CURRENT_SAMPLE_COUNT];
float currentSamples[CURRENT_SAMPLE_COUNT];
int sampleIndex = 0;
bool samplesFilled = false;

// Task Handles
TaskHandle_t dataCollectionTaskHandle = nullptr;
TaskHandle_t communicationTaskHandle = nullptr;

// Mutex for thread safety
SemaphoreHandle_t loraBufferMutex;
SemaphoreHandle_t radio433BufferMutex;
SemaphoreHandle_t barometerDataMutex;
SemaphoreHandle_t currentDataMutex;

// ====================================================================
// FORWARD DECLARATIONS
// ====================================================================

void setupSerial();
void setupSPI();
void setupI2C();
void initializeLoRa();
void initializeRadio433();
void initializeBarometer();
void initializeCurrentSensor();
void initializeCommunication();

void dataCollectionTask(void* parameters);
void communicationTask(void* parameters);
void mainLoopTask(void* parameters);

void checkLoRaPackets();
void checkRadio433Packets();
void updateBarometer();
void updateCurrentSensor();
void processCommProtocol();

void addLoRaPacket(const LoRaPacket_t& packet);
void addRadio433Packet(const Radio433Packet_t& packet);

bool getLoRaDataJSON(DynamicJsonDocument& doc);
bool getRadio433DataJSON(DynamicJsonDocument& doc);
bool getBarometerDataJSON(DynamicJsonDocument& doc);
bool getCurrentDataJSON(DynamicJsonDocument& doc);
bool getAllDataJSON(DynamicJsonDocument& doc);
bool getSystemStatusJSON(DynamicJsonDocument& doc);

void processCommand(uint8_t command);
void sendResponse(const String& response);
void sendErrorResponse(const String& error);
void resetCommState();

void handleSerialCommands();
void printWelcomeMessage();
void printSystemStatus();
void updateSystemStatus();

// ====================================================================
// SETUP FUNCTION
// ====================================================================

void setup() {
    // Initialize serial communication
    setupSerial();
    printWelcomeMessage();
    
    // Initialize communication buses
    setupSPI();
    setupI2C();
    
    // Create mutexes for thread safety
    loraBufferMutex = xSemaphoreCreateMutex();
    radio433BufferMutex = xSemaphoreCreateMutex();
    barometerDataMutex = xSemaphoreCreateMutex();
    currentDataMutex = xSemaphoreCreateMutex();
    
    // Initialize all sensors
    initializeLoRa();
    initializeRadio433();
    initializeBarometer();
    initializeCurrentSensor();
    initializeCommunication();
    
    // Initialize system status
    bootTime = millis();
    updateSystemStatus();
    
    // Create FreeRTOS tasks
    xTaskCreate(dataCollectionTask, "DataCollection", 4096, nullptr, 2, &dataCollectionTaskHandle);
    xTaskCreate(communicationTask, "Communication", 4096, nullptr, 3, &communicationTaskHandle);
    xTaskCreate(mainLoopTask, "MainLoop", 4096, nullptr, 1, nullptr);
    
    Serial.println("Setup complete - ESP32 Ground Station is running");
    Serial.println("Type 'help' for available commands");
}

// ====================================================================
// MAIN LOOP
// ====================================================================

void loop() {
    handleSerialCommands();
    delay(100);
}

// ====================================================================
// INITIALIZATION FUNCTIONS
// ====================================================================

void setupSerial() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);
    Serial.println();
}

void setupSPI() {
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    Serial.println("SPI initialized");
}

void setupI2C() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.println("I2C initialized");
}

void initializeLoRa() {
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa 915MHz: Failed to initialize");
        loraInitialized = false;
        return;
    }
    
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.enableCrc();
    
    loraInitialized = true;
    Serial.println("LoRa 915MHz: Initialized successfully");
}

void initializeRadio433() {
    int state = radio433Module.begin(RADIO433_FREQUENCY, RADIO433_BITRATE, 
                                    RADIO433_FREQ_DEV, 125.0, 10, 32);
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Radio433: Failed to initialize (error: %d)\n", state);
        radio433Initialized = false;
        return;
    }
    
    radio433Module.setEncoding(RADIOLIB_ENCODING_NRZ);
    radio433Module.setCRC(true);
    radio433Module.startReceive();
    
    radio433Initialized = true;
    Serial.println("Radio433: Initialized successfully");
}

void initializeBarometer() {
    if (!bme280Sensor.begin(BME280_ADDRESS_ALTERNATE, &Wire)) {
        if (!bme280Sensor.begin(BME280_ADDRESS_PRIMARY, &Wire)) {
            Serial.println("Barometer: Could not find BME280 sensor");
            barometerInitialized = false;
            return;
        }
    }
    
    bme280Sensor.setSampling(Adafruit_BME280::MODE_NORMAL,
                            Adafruit_BME280::SAMPLING_X2,
                            Adafruit_BME280::SAMPLING_X16,
                            Adafruit_BME280::SAMPLING_X1,
                            Adafruit_BME280::FILTER_X16,
                            Adafruit_BME280::STANDBY_MS_500);
    
    barometerInitialized = true;
    Serial.println("Barometer: BME280 initialized successfully");
}

void initializeCurrentSensor() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // Initialize sample arrays
    memset(voltageSamples, 0, sizeof(voltageSamples));
    memset(currentSamples, 0, sizeof(currentSamples));
    
    currentSensorInitialized = true;
    Serial.println("CurrentSensor: Initialized successfully");
}

void initializeCommunication() {
    piSerial.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
    piSerial.setTimeout(100);
    resetCommState();
    Serial.printf("CommProtocol: UART communication initialized at %d baud\n", PI_UART_BAUD);
}

// ====================================================================
// FREERTOS TASKS
// ====================================================================

void dataCollectionTask(void* parameters) {
    while (true) {
        if (loraInitialized) {
            checkLoRaPackets();
        }
        
        if (radio433Initialized) {
            checkRadio433Packets();
        }
        
        if (barometerInitialized) {
            updateBarometer();
        }
        
        if (currentSensorInitialized) {
            updateCurrentSensor();
        }
        
        static uint32_t lastStatusUpdate = 0;
        if (millis() - lastStatusUpdate > 5000) {
            updateSystemStatus();
            lastStatusUpdate = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
    }
}

void communicationTask(void* parameters) {
    while (true) {
        processCommProtocol();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mainLoopTask(void* parameters) {
    uint32_t lastStatusPrint = 0;
    
    while (true) {
        if (millis() - lastStatusPrint > 30000) {
            Serial.println("\n--- System Status Update ---");
            printSystemStatus();
            Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.println("---------------------------\n");
            lastStatusPrint = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ====================================================================
// SENSOR DATA COLLECTION
// ====================================================================

void checkLoRaPackets() {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        LoRaPacket_t newPacket;
        newPacket.timestamp = millis();
        newPacket.rssi = LoRa.packetRssi();
        newPacket.snr = LoRa.packetSnr();
        newPacket.data_length = min(packetSize, MAX_PACKET_SIZE);
        
        for (int i = 0; i < newPacket.data_length; i++) {
            newPacket.data[i] = LoRa.read();
        }
        
        addLoRaPacket(newPacket);
        loraPacketCount++;
        
        Serial.printf("LoRa RX: %d bytes, RSSI: %d dBm, SNR: %.2f dB\n", 
                      newPacket.data_length, newPacket.rssi, newPacket.snr);
    }
}

void checkRadio433Packets() {
    String receivedData;
    int state = radio433Module.readData(receivedData);
    
    if (state == RADIOLIB_ERR_NONE) {
        Radio433Packet_t newPacket;
        newPacket.timestamp = millis();
        newPacket.rssi = radio433Module.getRSSI();
        newPacket.data_length = min((int)receivedData.length(), MAX_PACKET_SIZE);
        
        memcpy(newPacket.data, receivedData.c_str(), newPacket.data_length);
        
        addRadio433Packet(newPacket);
        radio433PacketCount++;
        
        Serial.printf("433MHz RX: %d bytes, RSSI: %d dBm\n", 
                      newPacket.data_length, newPacket.rssi);
        
        radio433Module.startReceive();
    }
    else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        radio433Module.startReceive();
    }
}

void updateBarometer() {
    if (xSemaphoreTake(barometerDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        latestBarometerData.timestamp = millis();
        latestBarometerData.pressure_hpa = bme280Sensor.readPressure() / 100.0F;
        latestBarometerData.temperature_c = bme280Sensor.readTemperature();
        latestBarometerData.altitude_m = bme280Sensor.readAltitude(1013.25);
        
        xSemaphoreGive(barometerDataMutex);
    }
}

void updateCurrentSensor() {
    int rawADC = analogRead(CURRENT_SENSOR_PIN);
    float adcVoltage = (rawADC * 3.3) / 4095.0;
    float actualVoltage = (adcVoltage * voltageDividerRatio) + voltageOffset;
    float currentA = ((adcVoltage - 2.5) * 1000.0 / currentSensorSensitivity) + currentOffset;
    
    // Add to sample arrays
    voltageSamples[sampleIndex] = actualVoltage;
    currentSamples[sampleIndex] = currentA;
    sampleIndex = (sampleIndex + 1) % CURRENT_SAMPLE_COUNT;
    
    if (sampleIndex == 0) samplesFilled = true;
    
    // Calculate filtered values
    float voltageSum = 0, currentSum = 0;
    int count = samplesFilled ? CURRENT_SAMPLE_COUNT : sampleIndex;
    
    for (int i = 0; i < count; i++) {
        voltageSum += voltageSamples[i];
        currentSum += currentSamples[i];
    }
    
    if (xSemaphoreTake(currentDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        latestCurrentData.timestamp = millis();
        latestCurrentData.voltage_v = voltageSum / count;
        latestCurrentData.current_a = currentSum / count;
        latestCurrentData.power_w = latestCurrentData.voltage_v * latestCurrentData.current_a;
        
        xSemaphoreGive(currentDataMutex);
    }
}

// ====================================================================
// BUFFER MANAGEMENT
// ====================================================================

void addLoRaPacket(const LoRaPacket_t& packet) {
    if (xSemaphoreTake(loraBufferMutex, portMAX_DELAY) == pdTRUE) {
        loraBuffer[loraBufferHead] = packet;
        loraBufferHead = (loraBufferHead + 1) % LORA_BUFFER_SIZE;
        
        if (loraBufferHead == loraBufferTail) {
            loraBufferTail = (loraBufferTail + 1) % LORA_BUFFER_SIZE;
        }
        
        xSemaphoreGive(loraBufferMutex);
    }
}

void addRadio433Packet(const Radio433Packet_t& packet) {
    if (xSemaphoreTake(radio433BufferMutex, portMAX_DELAY) == pdTRUE) {
        radio433Buffer[radio433BufferHead] = packet;
        radio433BufferHead = (radio433BufferHead + 1) % RADIO433_BUFFER_SIZE;
        
        if (radio433BufferHead == radio433BufferTail) {
            radio433BufferTail = (radio433BufferTail + 1) % RADIO433_BUFFER_SIZE;
        }
        
        xSemaphoreGive(radio433BufferMutex);
    }
}

// ====================================================================
// COMMUNICATION PROTOCOL
// ====================================================================

void processCommProtocol() {
    // Timeout check
    if (commState != WAIT_FOR_HELLO && 
        (millis() - lastActivityTime > PI_COMM_TIMEOUT)) {
        resetCommState();
        return;
    }
    
    while (piSerial.available()) {
        uint8_t receivedByte = piSerial.read();
        lastActivityTime = millis();
        
        switch (commState) {
            case WAIT_FOR_HELLO:
                if (receivedByte == HELLO_BYTE) {
                    commState = WAIT_FOR_COMMAND;
                }
                break;
                
            case WAIT_FOR_COMMAND:
                receivedCommand = receivedByte;
                commState = WAIT_FOR_GOODBYE;
                break;
                
            case WAIT_FOR_GOODBYE:
                if (receivedByte == GOODBYE_BYTE) {
                    processCommand(receivedCommand);
                } else {
                    sendErrorResponse("Invalid packet format");
                }
                resetCommState();
                break;
                
            case SENDING_RESPONSE:
                resetCommState();
                break;
        }
    }
}

void processCommand(uint8_t command) {
    DynamicJsonDocument doc(2048); // Adjust size as needed
    String jsonResponse;
    bool success = false;
    
    switch (command) {
        case CMD_GET_LORA_DATA:
            success = getLoRaDataJSON(doc);
            break;
        case CMD_GET_433_DATA:
            success = getRadio433DataJSON(doc);
            break;
        case CMD_GET_BAROMETER_DATA:
            success = getBarometerDataJSON(doc);
            break;
        case CMD_GET_CURRENT_DATA:
            success = getCurrentDataJSON(doc);
            break;
        case CMD_GET_ALL_DATA:
            success = getAllDataJSON(doc);
            break;
        case CMD_GET_STATUS:
            success = getSystemStatusJSON(doc);
            break;
        default:
            sendErrorResponse("Unknown command");
            return;
    }
    
    if (success) {
        serializeJson(doc, jsonResponse);
        sendResponse(jsonResponse);
    } else {
        sendErrorResponse("Failed to process command");
    }
}

void sendResponse(const String& response) {
    commState = SENDING_RESPONSE;
    
    piSerial.write(HELLO_BYTE);
    
    uint16_t length = response.length();
    piSerial.write(length & 0xFF);
    piSerial.write((length >> 8) & 0xFF);
    
    piSerial.print(response);
    piSerial.write(GOODBYE_BYTE);
    piSerial.flush();
    
    Serial.printf("Response sent (%d bytes)\n", length);
}

void sendErrorResponse(const String& error) {
    DynamicJsonDocument errorDoc(256);
    errorDoc["error"] = error;
    errorDoc["timestamp"] = millis();
    
    String errorResponse;
    serializeJson(errorDoc, errorResponse);
    sendResponse(errorResponse);
}

void resetCommState() {
    commState = WAIT_FOR_HELLO;
    receivedCommand = 0;
}

// ====================================================================
// JSON DATA FUNCTIONS
// ====================================================================

bool getLoRaDataJSON(DynamicJsonDocument& doc) {
    if (!loraInitialized) {
        doc["error"] = "LoRa module offline";
        return false;
    }
    
    doc["module"] = "LoRa_915MHz";
    doc["online"] = true;
    doc["packet_count"] = loraPacketCount;
    doc["rssi"] = LoRa.packetRssi();
    doc["snr"] = LoRa.packetSnr();
    doc["current_channel_rssi"] = LoRa.rssi();    // Optional: live channel RSSI
    
    if (xSemaphoreTake(loraBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        JsonArray packetsArray = doc["packets"].to<JsonArray>();
        uint16_t current = loraBufferTail;
        int count = 0;
        
        while (current != loraBufferHead && count < 10) {
            JsonObject packet = packetsArray.createNestedObject();
            packet["timestamp"] = loraBuffer[current].timestamp;
            packet["rssi"] = loraBuffer[current].rssi;
            packet["snr"] = loraBuffer[current].snr;
            packet["length"] = loraBuffer[current].data_length;
            
            String hexData = "";
            for (int i = 0; i < loraBuffer[current].data_length; i++) {
                if (loraBuffer[current].data[i] < 16) hexData += "0";
                hexData += String(loraBuffer[current].data[i], HEX);
            }
            packet["data"] = hexData;
            
            current = (current + 1) % LORA_BUFFER_SIZE;
            count++;
        }
        
        loraBufferTail = loraBufferHead; // Clear buffer
        xSemaphoreGive(loraBufferMutex);
    }
    
    return true;
}

bool getRadio433DataJSON(DynamicJsonDocument& doc) {
    if (!radio433Initialized) {
        doc["error"] = "433MHz module offline";
        return false;
    }
    
    doc["module"] = "Radio_433MHz";
    doc["online"] = true;
    doc["packet_count"] = radio433PacketCount;
    doc["rssi"] = radio433Module.getRSSI();
    
    if (xSemaphoreTake(radio433BufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        JsonArray packetsArray = doc["packets"].to<JsonArray>();
        uint16_t current = radio433BufferTail;
        int count = 0;
        
        while (current != radio433BufferHead && count < 10) {
            JsonObject packet = packetsArray.createNestedObject();
            packet["timestamp"] = radio433Buffer[current].timestamp;
            packet["rssi"] = radio433Buffer[current].rssi;
            packet["length"] = radio433Buffer[current].data_length;
            
            String hexData = "";
            for (int i = 0; i < radio433Buffer[current].data_length; i++) {
                if (radio433Buffer[current].data[i] < 16) hexData += "0";
                hexData += String(radio433Buffer[current].data[i], HEX);
            }
            packet["data"] = hexData;
            
            current = (current + 1) % RADIO433_BUFFER_SIZE;
            count++;
        }
        
        radio433BufferTail = radio433BufferHead; // Clear buffer
        xSemaphoreGive(radio433BufferMutex);
    }
    
    return true;
}

bool getBarometerDataJSON(DynamicJsonDocument& doc) {
    if (!barometerInitialized) {
        doc["error"] = "Barometer offline";
        return false;
    }
    
    if (xSemaphoreTake(barometerDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        doc["module"] = "Barometer";
        doc["online"] = true;
        doc["timestamp"] = latestBarometerData.timestamp;
        doc["pressure_hpa"] = latestBarometerData.pressure_hpa;
        doc["temperature_c"] = latestBarometerData.temperature_c;
        doc["altitude_m"] = latestBarometerData.altitude_m;
        doc["humidity_percent"] = bme280Sensor.readHumidity();
        
        xSemaphoreGive(barometerDataMutex);
        return true;
    }
    
    doc["error"] = "Failed to read barometer data";
    return false;
}

bool getCurrentDataJSON(DynamicJsonDocument& doc) {
    if (!currentSensorInitialized) {
        doc["error"] = "Current sensor offline";
        return false;
    }
    
    if (xSemaphoreTake(currentDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        doc["module"] = "CurrentSensor";
        doc["online"] = true;
        doc["timestamp"] = latestCurrentData.timestamp;
        doc["current_a"] = latestCurrentData.current_a;
        doc["voltage_v"] = latestCurrentData.voltage_v;
        doc["power_w"] = latestCurrentData.power_w;
        doc["raw_adc"] = analogRead(CURRENT_SENSOR_PIN);
        
        xSemaphoreGive(currentDataMutex);
        return true;
    }
    
    doc["error"] = "Failed to read current sensor data";
    return false;
}

bool getAllDataJSON(DynamicJsonDocument& doc) {
    doc["timestamp"] = millis();
    
    DynamicJsonDocument loraDoc(2048);
    getLoRaDataJSON(loraDoc);
    doc["lora"] = loraDoc.as<JsonObject>();
    
    DynamicJsonDocument radio433Doc(2048);
    getRadio433DataJSON(radio433Doc);
    doc["radio433"] = radio433Doc.as<JsonObject>();
    
    DynamicJsonDocument baroDoc(2048);
    getBarometerDataJSON(baroDoc);
    doc["barometer"] = baroDoc.as<JsonObject>();
    
    DynamicJsonDocument currentDoc(2048);
    getCurrentDataJSON(currentDoc);
    doc["current"] = currentDoc.as<JsonObject>();
    
    DynamicJsonDocument statusDoc(2048);
    getSystemStatusJSON(statusDoc);
    doc["status"] = statusDoc.as<JsonObject>();
    
    return true;
}

bool getSystemStatusJSON(DynamicJsonDocument& doc) {
    updateSystemStatus();
    
    doc["uptime_seconds"] = systemStatus.uptime_seconds;
    doc["lora_online"] = systemStatus.lora_online;
    doc["radio433_online"] = systemStatus.radio433_online;
    doc["barometer_online"] = systemStatus.barometer_online;
    doc["current_sensor_online"] = systemStatus.current_sensor_online;
    doc["packet_count_lora"] = systemStatus.packet_count_lora;
    doc["packet_count_433"] = systemStatus.packet_count_433;
    doc["free_heap"] = ESP.getFreeHeap();
    doc["chip_revision"] = ESP.getChipRevision();
    
    return true;
}

// ====================================================================
// UTILITY FUNCTIONS
// ====================================================================

void updateSystemStatus() {
    systemStatus.uptime_seconds = (millis() - bootTime) / 1000;
    systemStatus.lora_online = loraInitialized;
    systemStatus.radio433_online = radio433Initialized;
    systemStatus.barometer_online = barometerInitialized;
    systemStatus.current_sensor_online = currentSensorInitialized;
    systemStatus.packet_count_lora = loraPacketCount;
    systemStatus.packet_count_433 = radio433PacketCount;
    systemStatus.pi_connected = (millis() - lastActivityTime < 30000);
}

void printSystemStatus() {
    Serial.println("=== System Status ===");
    Serial.printf("Uptime: %d seconds\n", systemStatus.uptime_seconds);
    Serial.printf("LoRa: %s (%d packets)\n", systemStatus.lora_online ? "Online" : "Offline", systemStatus.packet_count_lora);
    Serial.printf("433MHz: %s (%d packets)\n", systemStatus.radio433_online ? "Online" : "Offline", systemStatus.packet_count_433);
    Serial.printf("Barometer: %s\n", systemStatus.barometer_online ? "Online" : "Offline");
    Serial.printf("Current Sensor: %s\n", systemStatus.current_sensor_online ? "Online" : "Offline");
    Serial.printf("Pi Connected: %s\n", systemStatus.pi_connected ? "Yes" : "No");
    Serial.println("====================");
}

void printWelcomeMessage() {
    Serial.println("======================================");
    Serial.println("    ESP32 Rocketry Ground Station");
    Serial.println("       Single File Version 1.0");
    Serial.println("======================================");
    Serial.printf("ESP32 Chip Rev: %d\n", ESP.getChipRevision());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("======================================");
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();
        
        if (command == "help") {
            Serial.println("\n=== Available Commands ===");
            Serial.println("help     - Show this help");
            Serial.println("status   - Show system status");
            Serial.println("lora     - Show LoRa info");
            Serial.println("433      - Show 433MHz info");
            Serial.println("baro     - Show barometer");
            Serial.println("current  - Show current sensor");
            Serial.println("reboot   - Restart system");
            Serial.println("==========================\n");
        }
        else if (command == "status") {
            printSystemStatus();
        }
        else if (command == "lora") {
            Serial.printf("LoRa Status: %s\n", loraInitialized ? "Online" : "Offline");
            if (loraInitialized) {
                Serial.printf("Packets: %d, RSSI: %d dBm\n", loraPacketCount, LoRa.rssi());
            }
        }
        else if (command == "433") {
            Serial.printf("433MHz Status: %s\n", radio433Initialized ? "Online" : "Offline");
            if (radio433Initialized) {
                Serial.printf("Packets: %d, RSSI: %d dBm\n", radio433PacketCount, radio433Module.getRSSI());
            }
        }
        else if (command == "baro") {
            Serial.printf("Barometer Status: %s\n", barometerInitialized ? "Online" : "Offline");
            if (barometerInitialized) {
                Serial.printf("Pressure: %.2f hPa, Temp: %.2f°C\n", 
                             latestBarometerData.pressure_hpa, latestBarometerData.temperature_c);
            }
        }
        else if (command == "current") {
            Serial.printf("Current Sensor Status: %s\n", currentSensorInitialized ? "Online" : "Offline");
            if (currentSensorInitialized) {
                Serial.printf("V: %.3fV, I: %.3fA, P: %.3fW\n", 
                             latestCurrentData.voltage_v, latestCurrentData.current_a, latestCurrentData.power_w);
            }
        }
        else if (command == "reboot") {
            Serial.println("Rebooting...");
            ESP.restart();
        }
        else if (command.length() > 0) {
            Serial.println("Unknown command. Type 'help' for commands.");
        }
    }
}