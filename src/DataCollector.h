// DataCollector.h
#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "sensors/LoRa915.h"
#include "sensors/Radio433.h"
#include "sensors/Barometer.h"
#include "sensors/CurrentSensor.h"

class DataCollector {
private:
    LoRa915* lora_module;
    Radio433* radio433_module;
    Barometer* barometer;
    CurrentSensor* current_sensor;
    
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
    
    // Data access methods
    bool getLoRaData(DynamicJsonDocument& doc);
    bool get433Data(DynamicJsonDocument& doc);
    bool getBarometerData(DynamicJsonDocument& doc);
    bool getCurrentData(DynamicJsonDocument& doc);
    bool getAllData(DynamicJsonDocument& doc);
    bool getSystemStatus(DynamicJsonDocument& doc);
    
    // Individual sensor access
    LoRa915* getLoRaModule() { return lora_module; }
    Radio433* getRadio433Module() { return radio433_module; }
    Barometer* getBarometerModule() { return barometer; }
    CurrentSensor* getCurrentSensorModule() { return current_sensor; }
    
    // Status
    const SystemStatus_t& getStatus() const { return system_status; }
    void updateSystemStatus();
    
    // Utility functions
    String formatTimestamp(uint32_t timestamp);
    void printSystemInfo();
};

#endif // DATA_COLLECTOR_H