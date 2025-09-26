#include "DataCollector.h"

DataCollector::DataCollector() : 
    boot_time(0),
    last_lora_check(0),
    last_radio433_check(0),
    last_barometer_update(0),
    last_current_update(0),
    last_status_update(0) {
    
    // Initialize sensor objects
    lora_module = new LoRa915();
    radio433_module = new Radio433();
    barometer = new Barometer();
    current_sensor = new CurrentSensor();
    
    // Initialize system status
    memset(&system_status, 0, sizeof(system_status));
    system_status.system_state = SYSTEM_WAITING_FOR_WAKEUP;  // Start in sleep mode
    boot_time = millis();
}

DataCollector::~DataCollector() {
    end();
    delete lora_module;
    delete radio433_module;
    delete barometer;
    delete current_sensor;
}

bool DataCollector::begin() {
    Serial.println("DataCollector: Initializing all sensors...");
    
    // Initialize each sensor
    system_status.lora_online = lora_module->begin();
    system_status.radio433_online = radio433_module->begin();
    system_status.barometer_online = barometer->begin();
    system_status.current_sensor_online = current_sensor->begin();
    
    // Update system status
    updateSystemStatus();
    
    Serial.printf("DataCollector: Initialization complete. Status: LoRa:%s, 433MHz:%s, Barometer:%s, Current:%s\n",
                  system_status.lora_online ? "OK" : "FAIL",
                  system_status.radio433_online ? "OK" : "FAIL",
                  system_status.barometer_online ? "OK" : "FAIL",
                  system_status.current_sensor_online ? "OK" : "FAIL");
    
    Serial.println("DataCollector: System waiting for WAKEUP command from Raspberry Pi");
    
    return (system_status.lora_online || system_status.radio433_online || 
            system_status.barometer_online || system_status.current_sensor_online);
}

void DataCollector::end() {
    if (lora_module) lora_module->end();
    if (radio433_module) radio433_module->end();
    if (barometer) barometer->end();
    if (current_sensor) current_sensor->end();
}

void DataCollector::setSystemState(SystemState_t new_state) {
    SystemState_t old_state = system_status.system_state;
    system_status.system_state = new_state;
    
    if (new_state == SYSTEM_OPERATIONAL && old_state != SYSTEM_OPERATIONAL) {
        system_status.wakeup_time = millis();
        Serial.println("DataCollector: System is now OPERATIONAL");
    } else if (new_state == SYSTEM_WAITING_FOR_WAKEUP) {
        Serial.println("DataCollector: System entering SLEEP mode");
    }
}

void DataCollector::pollRadios() {
    uint32_t current_time = millis();
    
    // Always check LoRa for incoming packets (even when sleeping)
    if (lora_module->isOnline() && 
        (current_time - last_lora_check > RADIO_LISTEN_TIMEOUT)) {
        lora_module->checkForPackets();
        last_lora_check = current_time;
    }
    
    // Only check 433MHz when system is operational
    if (system_status.system_state == SYSTEM_OPERATIONAL &&
        radio433_module->isOnline() && 
        (current_time - last_radio433_check > RADIO_LISTEN_TIMEOUT)) {
        radio433_module->checkForPackets();
        last_radio433_check = current_time;
    }
}

void DataCollector::pollSensors() {
    // Only update sensors when system is operational
    if (system_status.system_state != SYSTEM_OPERATIONAL) {
        return;
    }
    
    uint32_t current_time = millis();
    
    // Update barometer
    if (barometer->isOnline() && 
        (current_time - last_barometer_update > SENSOR_READ_INTERVAL)) {
        barometer->update();
        last_barometer_update = current_time;
    }
    
    // Update current sensor
    if (current_sensor->isOnline() && 
        (current_time - last_current_update > SENSOR_READ_INTERVAL)) {
        current_sensor->update();
        last_current_update = current_time;
    }
    
    // Update system status periodically
    if (current_time - last_status_update > 5000) // Every 5 seconds
        updateSystemStatus();
        last_status_update = current_time;
    }
}

void DataCollector::updateSystemStatus() {
    system_status.uptime_seconds = (millis() - boot_time) / 1000;
    system_status.packet_count_lora = lora_module->getPacketCount();
    system_status.packet_count_433 = radio433_module->getPacketCount();
    
    // Re-check sensor status
    system_status.lora_online = lora_module->isOnline();
    system_status.radio433_online = radio433_module->isOnline();
    system_status.barometer_online = barometer->isOnline();
    system_status.current_sensor_online = current_sensor->isOnline();
}

bool DataCollector::getLoRaData(DynamicJsonDocument& doc) {
    if (!lora_module->isOnline()) {
        doc["error"] = "LoRa module offline";
        return false;
    }
    
    doc["module"] = "LoRa_915MHz";
    doc["online"] = true;
    doc["packet_count"] = lora_module->getPacketCount();
    doc["rssi"] = lora_module->getRSSI();
    doc["snr"] = lora_module->getSNR();
    
    // Get recent packets
    LoRaPacket_t packets[10];
    uint16_t packet_count;
    if (lora_module->getAllPackets(packets, 10, packet_count)) {
        // create nested array safely
        JsonArray packets_array = doc.createNestedArray("packets");
        for (int i = 0; i < packet_count; i++) {
            JsonObject packet = packets_array.createNestedObject();
            packet["timestamp"] = packets[i].timestamp;
            packet["rssi"] = packets[i].rssi;
            packet["snr"] = packets[i].snr;
            packet["length"] = packets[i].data_length;
            
            // Convert data to hex string
            String hex_data = "";
            for (int j = 0; j < packets[i].data_length; j++) {
                if (packets[i].data[j] < 16) hex_data += "0";
                hex_data += String(packets[i].data[j], HEX);
            }
            packet["data"] = hex_data;
        }
    }
    
    return true;
}

bool DataCollector::get433Data(DynamicJsonDocument& doc) {
    if (!radio433_module->isOnline()) {
        doc["error"] = "433MHz module offline";
        return false;
    }
    
    doc["module"] = "Radio_433MHz";
    doc["online"] = true;
    doc["packet_count"] = radio433_module->getPacketCount();
    doc["rssi"] = radio433_module->getRSSI();
    
    // Get recent packets
    Radio433Packet_t packets[10];
    uint16_t packet_count;
    if (radio433_module->getAllPackets(packets, 10, packet_count)) {
        // create nested array safely
        JsonArray packets_array = doc.createNestedArray("packets");
        for (int i = 0; i < packet_count; i++) {
            JsonObject packet = packets_array.createNestedObject();
            packet["timestamp"] = packets[i].timestamp;
            packet["rssi"] = packets[i].rssi;
            packet["length"] = packets[i].data_length;
            
            // Convert data to hex string
            String hex_data = "";
            for (int j = 0; j < packets[i].data_length; j++) {
                if (packets[i].data[j] < 16) hex_data += "0";
                hex_data += String(packets[i].data[j], HEX);
            }
            packet["data"] = hex_data;
        }
    }
    
    return true;
}

bool DataCollector::getBarometerData(DynamicJsonDocument& doc) {
    if (!barometer->isOnline()) {
        doc["error"] = "Barometer offline";
        return false;
    }
    
    BarometerData_t data;
    if (barometer->getLatestReading(data)) {
        doc["module"] = "Barometer";
        doc["online"] = true;
        doc["timestamp"] = data.timestamp;
        doc["pressure_hpa"] = data.pressure_hpa;
        doc["temperature_c"] = data.temperature_c;
        doc["altitude_m"] = data.altitude_m;
        doc["humidity_percent"] = barometer->getHumidity();
        return true;
    }
    
    doc["error"] = "Failed to read barometer data";
    return false;
}

bool DataCollector::getCurrentData(DynamicJsonDocument& doc) {
    if (!current_sensor->isOnline()) {
        doc["error"] = "Current sensor offline";
        return false;
    }
    
    CurrentData_t data;
    if (current_sensor->getLatestReading(data)) {
        doc["module"] = "CurrentSensor";
        doc["online"] = true;
        doc["timestamp"] = data.timestamp;
        doc["current_a"] = data.current_a;
        doc["voltage_v"] = data.voltage_v;
        doc["power_w"] = data.power_w;
        doc["raw_adc"] = current_sensor->getRawADC();
        return true;
    }
    
    doc["error"] = "Failed to read current sensor data";
    return false;
}

bool DataCollector::getAllData(DynamicJsonDocument& doc) {
    doc["timestamp"] = millis();
    
    // Use temporary documents and then assign their root objects into the main doc
    DynamicJsonDocument lora_doc(1024);
    getLoRaData(lora_doc);
    doc["lora"] = lora_doc.as<JsonObject>();
    
    DynamicJsonDocument radio433_doc(1024);
    get433Data(radio433_doc);
    doc["radio433"] = radio433_doc.as<JsonObject>();
    
    DynamicJsonDocument baro_doc(512);
    getBarometerData(baro_doc);
    doc["barometer"] = baro_doc.as<JsonObject>();
    
    DynamicJsonDocument current_doc(512);
    getCurrentData(current_doc);
    doc["current"] = current_doc.as<JsonObject>();
    
    DynamicJsonDocument status_doc(512);
    getSystemStatus(status_doc);
    doc["status"] = status_doc.as<JsonObject>();
    
    return true;
}

bool DataCollector::getSystemStatus(DynamicJsonDocument& doc) {
    updateSystemStatus();
    
    doc["uptime_seconds"] = system_status.uptime_seconds;
    doc["system_state"] = system_status.system_state;
    doc["lora_online"] = system_status.lora_online;
    doc["radio433_online"] = system_status.radio433_online;
    doc["barometer_online"] = system_status.barometer_online;
    doc["current_sensor_online"] = system_status.current_sensor_online;
    doc["pi_connected"] = system_status.pi_connected;
    doc["packet_count_lora"] = system_status.packet_count_lora;
    doc["packet_count_433"] = system_status.packet_count_433;
    doc["wakeup_time"] = system_status.wakeup_time;
    doc["free_heap"] = ESP.getFreeHeap();
    doc["chip_revision"] = ESP.getChipRevision();
    doc["sdk_version"] = ESP.getSdkVersion();
    
    return true;
}

String DataCollector::formatTimestamp(uint32_t timestamp) {
    uint32_t seconds = timestamp / 1000;
    uint32_t milliseconds = timestamp % 1000;
    uint32_t minutes = seconds / 60;
    seconds = seconds % 60;
    uint32_t hours = minutes / 60;
    minutes = minutes % 60;
    
    return String(hours) + ":" + String(minutes) + ":" + String(seconds) + "." + String(milliseconds);
}

void DataCollector::printSystemInfo() {
    Serial.println("=== System Information ===");
    Serial.printf("Uptime: %d seconds\n", system_status.uptime_seconds);
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("LoRa: %s (%d packets)\n", system_status.lora_online ? "Online" : "Offline", system_status.packet_count_lora);
    Serial.printf("433MHz: %s (%d packets)\n", system_status.radio433_online ? "Online" : "Offline", system_status.packet_count_433);
    Serial.printf("Barometer: %s\n", system_status.barometer_online ? "Online" : "Offline");
    Serial.printf("Current Sensor: %s\n", system_status.current_sensor_online ? "Online" : "Offline");
    Serial.println("===========================");
}