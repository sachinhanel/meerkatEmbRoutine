/*
 * ESP32 Rocketry Ground Station - Single File Version
 * Complete system in one file for embedded deployment
 *
 * Hardware Requirements:
 * - ESP32 Dev Board
 * - 915MHz LoRa Module (SPI)
 * - 433MHz Radio Module (SPI)
 * - MS5607 Barometer (I2C)
 * - Current Sensor (Analog)
 */

#include <Arduino.h>
#include <WiFi.h>

#ifndef TEST_MODE
// Only include hardware libraries when not in test mode
#include <SPI.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <LoRa.h>
#include <RadioLib.h>
#include <MS5611.h>
#endif

//brings in config.h and its def also
#include "DataCollector.h"
#include "CommProtocol.h"

#ifdef TEST_MODE
#include "FlightDataReplay.h"
#endif

// ====================================================================
// All pin definitions, constants, and data structures are in config.h
// ====================================================================


// ====================================================================
// GLOBAL VARIABLES
// ====================================================================


DataCollector dataCollector;
CommProtocol commProtocol(&dataCollector, &Serial);

#ifdef TEST_MODE
// WiFi Telnet Server for remote serial monitoring
WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;

// Flight log replay state
bool replayEnabled = false;
int replayIndex = 0;
uint32_t lastReplayTime = 0;
const uint32_t REPLAY_INTERVAL_MS = 1000; // 1 line per second
#endif

#ifndef TEST_MODE
// Sensor Hardware Objects (only in normal mode)
SX1278 radio433Module(new Module(RADIO433_CS_PIN, RADIO433_DIO0_PIN, RADIO433_RST_PIN));
MS5611 ms5607Sensor;
HardwareSerial piSerial(2);
#endif

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




void dataCollectionTask(void* parameters);
void communicationTask(void* parameters);
void mainLoopTask(void* parameters);

void checkLoRaPackets();
void checkRadio433Packets();
void updateBarometer();
void updateCurrentSensor();


void addLoRaPacket(const LoRaPacket_t& packet);
void addRadio433Packet(const Radio433Packet_t& packet);

void handleSerialCommands();
void printWelcomeMessage();
void printSystemStatus();
void updateSystemStatus();

#ifdef TEST_MODE
// WiFi Telnet handling
void handleTelnetClients() {
    // Check for new clients
    if (telnetServer.hasClient()) {
        // Disconnect old client if exists
        if (telnetClient && telnetClient.connected()) {
            telnetClient.stop();
        }
        telnetClient = telnetServer.available();
        Serial.println("\n[WiFi] Telnet client connected!");
        telnetClient.println("=== ESP32 Remote Serial Monitor ===");
        telnetClient.printf("Connected to: %s\n", WiFi.localIP().toString().c_str());
        telnetClient.println("===================================\n");
    }
}

// Callback function when WAKEUP command is received
void onWakeupReceived() {
    Serial.println("[REPLAY] WAKEUP received - starting flight log replay!");
    replayEnabled = true;
    replayIndex = 0;
    lastReplayTime = millis();
}

// Function to send one flight log line as a LoRa packet
void sendFlightLogLine(const char* line) {
    // Create a fake LoRa packet from the flight log line
    WireLoRa_t packet{};
    packet.version = 1;
    packet.packet_count = replayIndex;

    // Parse RSSI and SNR from the line if present
    // Format: "RSSI:-82, SNR:12 | [...]"
    if (strncmp(line, "RSSI:", 5) == 0) {
        packet.rssi_dbm = atoi(line + 5);
        const char* snr_pos = strstr(line, "SNR:");
        if (snr_pos) {
            packet.snr_db = atof(snr_pos + 4);
        }
    } else {
        packet.rssi_dbm = -85;
        packet.snr_db = 3.5;
    }

    // Copy the entire line as packet data (truncate if needed)
    size_t line_len = strlen(line);
    packet.latest_len = (line_len > 64) ? 64 : line_len;
    memcpy(packet.latest_data, line, packet.latest_len);

    // Send as binary-framed LoRa packet
    Serial.write((uint8_t)HELLO_BYTE);
    Serial.write((uint8_t)PERIPHERAL_ID_LORA_915);
    Serial.write((uint8_t)sizeof(WireLoRa_t));
    Serial.write((uint8_t*)&packet, sizeof(WireLoRa_t));
    Serial.write((uint8_t)GOODBYE_BYTE);
    Serial.flush();

    // Also print to debug
    Serial.printf("[REPLAY] Sent line %d: %.50s%s\n",
                  replayIndex,
                  line,
                  (line_len > 50) ? "..." : "");
}

// Override Serial.print functions to also send to telnet
class TelnetSerial : public Print {
public:
    size_t write(uint8_t c) override {
        size_t n = Serial.write(c);
        if (telnetClient && telnetClient.connected()) {
            telnetClient.write(c);
        }
        return n;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        size_t n = Serial.write(buffer, size);
        if (telnetClient && telnetClient.connected()) {
            telnetClient.write(buffer, size);
        }
        return n;
    }
};
#endif

// ====================================================================
// SETUP FUNCTION
// ====================================================================

void setup() {
    // Initialize serial communication
    setupSerial();
    printWelcomeMessage();

#ifdef TEST_MODE
    // TEST MODE: Initialize WiFi for remote monitoring
    Serial.println("\nConnecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int wifi_timeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_timeout < 20) {
        delay(500);
        Serial.print(".");
        wifi_timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Telnet server on port %d\n", TELNET_PORT);
        Serial.println("Connect with: telnet <ip> 23");
        telnetServer.begin();
        telnetServer.setNoDelay(true);
    } else {
        Serial.println("\nWiFi connection failed - continuing without remote monitoring");
    }

    // TEST MODE: Initialize DataCollector with fake sensors
    dataCollector.begin();
#else
    // NORMAL MODE: Initialize communication buses
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

    // Initialize DataCollector with real sensors
    dataCollector.begin();
#endif

    // Initialize system status
    bootTime = millis();
    updateSystemStatus();

    printSystemStatus();

    // Create FreeRTOS tasks
    xTaskCreate(dataCollectionTask, "DataCollection", 4096, nullptr, 2, &dataCollectionTaskHandle);
    xTaskCreate(communicationTask, "Communication", 4096, nullptr, 3, &communicationTaskHandle);
    xTaskCreate(mainLoopTask, "MainLoop", 4096, nullptr, 1, nullptr);

    Serial.println("Setup complete - ESP32 Ground Station is running");
#ifdef TEST_MODE
    Serial.println("TEST MODE: Waiting for binary commands on Serial...");
    Serial.println("TEST MODE: Flight log replay ready - send WAKEUP command to start");

    // Register wakeup callback for flight log replay
    commProtocol.setWakeupCallback(onWakeupReceived);
#else
    Serial.println("Type 'help' for available commands");
#endif
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
    while (!Serial) delay(10); //wait for usb serial to be ready
    delay(1000);
    Serial.println();
}

#ifndef TEST_MODE
// All sensor initialization functions only in normal mode

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
    // Initialize MS5607 (compatible with MS5611 library)
    if (!ms5607Sensor.begin()) {
        Serial.println("Barometer: MS5607 begin failed");
        barometerInitialized = false;
        return;
    }

    // Reset and read PROM
    if (!ms5607Sensor.reset()) {
        Serial.println("Barometer: MS5607 reset/PROM read failed");
        barometerInitialized = false;
        return;
    }

    // Set high oversampling for better precision
    ms5607Sensor.setOversampling(OSR_ULTRA_HIGH);

    barometerInitialized = true;
    Serial.println("Barometer: MS5607 initialized successfully");
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
#endif // TEST_MODE



// ====================================================================
// FREERTOS TASKS
// ====================================================================

void dataCollectionTask(void* parameters) {
    while (true) {
#ifndef TEST_MODE
        // Only poll sensors in normal mode
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
#endif

        static uint32_t lastStatusUpdate = 0;
        if (millis() - lastStatusUpdate > 5000) {
            updateSystemStatus();
            lastStatusUpdate = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
    }
}

void communicationTask(void* parameters) {
    static uint32_t lastHeartbeat = 0;

    while (true) {
        commProtocol.process();

#ifdef TEST_MODE
        // Handle WiFi telnet clients
        handleTelnetClients();

        // Flight log replay logic
        if (replayEnabled) {
            uint32_t currentTime = millis();
            if (currentTime - lastReplayTime >= REPLAY_INTERVAL_MS) {
                // Send the current line
                sendFlightLogLine(flightLogLines[replayIndex]);

                // Move to next line (loop back to start if at end)
                replayIndex++;
                if (replayIndex >= flightLogLineCount) {
                    replayIndex = 0;
                    Serial.println("[REPLAY] Loop complete - restarting from beginning");
                }

                lastReplayTime = currentTime;
            }
        }

        // Send binary status update every second (only when NOT replaying, to avoid spam)
        if (!replayEnabled && (millis() - lastHeartbeat > 1000)) {
            commProtocol.sendStatusUpdate();
            lastHeartbeat = millis();
        }
#else
        // NORMAL MODE: Send periodic status update every 5 seconds
        // This acts as a heartbeat to show the ESP32 is still running
        if (millis() - lastHeartbeat > 5000) {
            commProtocol.sendStatusUpdate();
            lastHeartbeat = millis();
        }
#endif

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

#ifndef TEST_MODE
// Sensor reading functions only in normal mode

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
    if (!barometerInitialized) return;

    if (xSemaphoreTake(barometerDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        latestBarometerData.timestamp = millis();

        // Read temperature and pressure
        int result = ms5607Sensor.read();
        if (result == MS5611_READ_OK) {
            latestBarometerData.temperature_c = (float)ms5607Sensor.getTemperature();
            latestBarometerData.pressure_hpa = (float)ms5607Sensor.getPressure();

            // Calculate altitude (using standard sea level pressure)
            float ratio = latestBarometerData.pressure_hpa / 1013.25f;
            latestBarometerData.altitude_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));
        } else {
            Serial.printf("Barometer: Read failed with code %d\n", result);
        }

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
#endif // TEST_MODE

// ====================================================================
// COMMUNICATION PROTOCOL
// ====================================================================

// All communication protocol handling is now in CommProtocol.cpp
// Binary data transmission only - no JSON

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
    systemStatus.pi_connected = (millis() - commProtocol.getLastActivityTime() < 30000);
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
#ifdef TEST_MODE
    Serial.println("      **TEST MODE ENABLED**");
    Serial.println("      No Hardware Required");
#else
    Serial.println("       Single File Version 1.0");
#endif
    Serial.println("======================================");
    Serial.printf("ESP32 Chip Rev: %d\n", ESP.getChipRevision());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
#ifdef TEST_MODE
    Serial.println("\nTEST MODE: All sensors simulated");
    Serial.println("Ready for Pi protocol testing");
    Serial.println("Waiting for commands on USB Serial...");
#endif
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
#ifndef TEST_MODE
            if (loraInitialized) {
                Serial.printf("Packets: %d, RSSI: %d dBm\n", loraPacketCount, LoRa.rssi());
            }
#else
            Serial.printf("Packets: %d (simulated)\n", loraPacketCount);
#endif
        }
        else if (command == "433") {
            Serial.printf("433MHz Status: %s\n", radio433Initialized ? "Online" : "Offline");
#ifndef TEST_MODE
            if (radio433Initialized) {
                Serial.printf("Packets: %d, RSSI: %d dBm\n", radio433PacketCount, radio433Module.getRSSI());
            }
#else
            Serial.printf("Packets: %d (simulated)\n", radio433PacketCount);
#endif
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