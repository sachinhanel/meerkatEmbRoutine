// DataCollector.h
#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <Arduino.h>
#include "config.h"

#ifndef TEST_MODE
// Only include sensor headers when not in test mode
#include "sensors/LoRa915.h"
#include "sensors/Radio433.h"
#include "sensors/Barometer.h"
#include "sensors/CurrentVoltageSensor.h"
#endif

class DataCollector {
private:
#ifndef TEST_MODE
    LoRa915* lora_module;
    Radio433* radio433_module;
    Barometer* barometer;
    CurrentVoltageSensor* current_sensor;
#endif

    SystemStatus_t system_status;
    uint32_t boot_time;
    
    // Polling sequence variables
    uint32_t last_lora_check;
    uint32_t last_radio433_check;
    uint32_t last_barometer_update;
    uint32_t last_current_update;
    uint32_t last_status_update;
    
public:
    DataCollector();
    ~DataCollector();
    
    // Initialization
    bool begin();
    void end();
    
    // Polling functions (call these in main loop)
    void pollSensors();
    void pollRadios();
    
    // System state management
    void setSystemState(SystemState_t new_state);
    SystemState_t getSystemState() const { return system_status.system_state; }
    bool isOperational() const { return system_status.system_state == SYSTEM_OPERATIONAL; }
    
    // (JSON accessors removed; binary packers below)

    // Binary packers (return bytes written)
    size_t packLoRaData(uint8_t* out, size_t max_len);
    size_t pack433Data(uint8_t* out, size_t max_len);
    size_t packBarometerData(uint8_t* out, size_t max_len);
    size_t packCurrentData(uint8_t* out, size_t max_len);
    size_t packStatus(uint8_t* out, size_t max_len);
    
    // Individual sensor access
#ifndef TEST_MODE
    LoRa915* getLoRaModule() { return lora_module; }
    Radio433* getRadio433Module() { return radio433_module; }
    Barometer* getBarometerModule() { return barometer; }
    CurrentVoltageSensor* getCurrentSensorModule() { return current_sensor; }
#endif
    
    // Status
    const SystemStatus_t& getStatus() const { return system_status; }
    void updateSystemStatus();
    
    // Utility functions
    String formatTimestamp(uint32_t timestamp);
    void printSystemInfo();
};

#endif // DATA_COLLECTOR_H