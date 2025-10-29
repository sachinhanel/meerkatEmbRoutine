#include "DataCollector.h"

DataCollector::DataCollector() :
    boot_time(0),
    last_lora_check(0),
    last_radio433_check(0),
    last_barometer_update(0),
    last_current_update(0),
    last_status_update(0) {

#ifdef TEST_MODE
    // Test mode - don't initialize sensor objects (no hardware)
    Serial.println("DataCollector: TEST_MODE enabled - skipping sensor initialization");
#else
    // Initialize sensor objects
    lora_module = new LoRa915();
    radio433_module = new Radio433();
    barometer = new Barometer();
    current_sensor = new CurrentVoltageSensor();
#endif

    // Initialize system status
    memset(&system_status, 0, sizeof(system_status));
#ifdef TEST_MODE
    system_status.system_state = SYSTEM_OPERATIONAL;  // Start operational in test mode
#else
    system_status.system_state = SYSTEM_WAITING_FOR_WAKEUP;  // Wait for wakeup in normal mode
#endif
    boot_time = millis();
}

DataCollector::~DataCollector() {
    end();
#ifndef TEST_MODE
    delete lora_module;
    delete radio433_module;
    delete barometer;
    delete current_sensor;
#endif
}

bool DataCollector::begin() {
#ifdef TEST_MODE
    Serial.println("DataCollector: TEST_MODE - Simulating all sensors online");
    system_status.lora_online = true;
    system_status.radio433_online = true;
    system_status.barometer_online = true;
    system_status.current_sensor_online = true;
    system_status.system_state = SYSTEM_OPERATIONAL;

    Serial.println("DataCollector: All fake sensors ready for testing!");
    return true;
#else
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
#endif
}

void DataCollector::end() {
#ifndef TEST_MODE
    if (lora_module) lora_module->end();
    if (radio433_module) radio433_module->end();
    if (barometer) barometer->end();
    if (current_sensor) current_sensor->end();
#endif
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
#ifndef TEST_MODE
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
#endif
}

void DataCollector::pollSensors() {
#ifndef TEST_MODE
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
    if (current_time - last_status_update > 5000) { // Every 5 seconds
        updateSystemStatus();
        last_status_update = current_time;
    }
#endif
}



void DataCollector::updateSystemStatus() {
    system_status.uptime_seconds = (millis() - boot_time) / 1000;

#ifndef TEST_MODE
    system_status.packet_count_lora = lora_module->getPacketCount();
    system_status.packet_count_433 = radio433_module->getPacketCount();

    // Re-check sensor status
    system_status.lora_online = lora_module->isOnline();
    system_status.radio433_online = radio433_module->isOnline();
    system_status.barometer_online = barometer->isOnline();
    system_status.current_sensor_online = current_sensor->isOnline();
#else
    // In test mode, counters are updated in packLoRaData() and pack433Data()
    // Don't increment here to avoid false counts on status requests
#endif
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

// =========================
// Binary packers
// =========================

size_t DataCollector::packLoRaData(uint8_t* out, size_t max_len) {
    if (max_len < sizeof(WireLoRa_t)) return 0;

#ifdef TEST_MODE
    // Generate fake LoRa data
    WireLoRa_t w{};
    w.version = 1;
    static uint16_t test_packet_count = 0;
    w.packet_count = test_packet_count++;
    // Also update system status counter to match
    system_status.packet_count_lora = w.packet_count;
    w.rssi_dbm = -85;
    w.snr_db = 3.5;
    const char* test_data = "TestLoRaPacket";
    w.latest_len = strlen(test_data);
    memcpy(w.latest_data, test_data, w.latest_len);
    memcpy(out, &w, sizeof(WireLoRa_t));
    return sizeof(WireLoRa_t);
#else
    if (!lora_module->isOnline()) return 0;
    WireLoRa_t w{};
    w.version = 1;
    w.packet_count = lora_module->getPacketCount();
    w.rssi_dbm = lora_module->getRSSI();
    w.snr_db = lora_module->getSNR();
    // include latest packet if available
    LoRaPacket_t pkt;
    if (lora_module->getLatestPacket(pkt)) {
        uint8_t len = pkt.data_length > 64 ? 64 : pkt.data_length;
        w.latest_len = len;
        memcpy(w.latest_data, pkt.data, len);
    } else {
        w.latest_len = 0;
    }
    memcpy(out, &w, sizeof(WireLoRa_t));
    return sizeof(WireLoRa_t);
#endif
}

size_t DataCollector::pack433Data(uint8_t* out, size_t max_len) {
    if (max_len < sizeof(Wire433_t)) return 0;

#ifdef TEST_MODE
    // Generate fake 433MHz data
    Wire433_t w{};
    w.version = 1;
    static uint16_t test_packet_count = 0;
    w.packet_count = test_packet_count++;
    // Also update system status counter to match
    system_status.packet_count_433 = w.packet_count;
    w.rssi_dbm = -92;
    const char* test_data = "Test433Packet";
    w.latest_len = strlen(test_data);
    memcpy(w.latest_data, test_data, w.latest_len);
    memcpy(out, &w, sizeof(Wire433_t));
    return sizeof(Wire433_t);
#else
    if (!radio433_module->isOnline()) return 0;
    Wire433_t w{};
    w.version = 1;
    w.packet_count = radio433_module->getPacketCount();
    w.rssi_dbm = radio433_module->getRSSI();
    Radio433Packet_t pkt;
    if (radio433_module->getLatestPacket(pkt)) {
        uint8_t len = pkt.data_length > 64 ? 64 : pkt.data_length;
        w.latest_len = len;
        memcpy(w.latest_data, pkt.data, len);
    } else {
        w.latest_len = 0;
    }
    memcpy(out, &w, sizeof(Wire433_t));
    return sizeof(Wire433_t);
#endif
}

size_t DataCollector::packBarometerData(uint8_t* out, size_t max_len) {
    if (max_len < sizeof(WireBarometer_t)) return 0;

#ifdef TEST_MODE
    // Generate fake barometer data
    WireBarometer_t w{};
    w.version = 1;
    w.timestamp_ms = millis();
    w.pressure_hpa = 1013.25;
    w.temperature_c = 22.5;
    w.altitude_m = 123.4;
    memcpy(out, &w, sizeof(WireBarometer_t));
    return sizeof(WireBarometer_t);
#else
    if (!barometer->isOnline()) return 0;
    WireBarometer_t w{};
    w.version = 1;
    BarometerData_t d;
    if (!barometer->getLatestReading(d)) return 0;
    w.timestamp_ms = d.timestamp;
    w.pressure_hpa = d.pressure_hpa;
    w.temperature_c = d.temperature_c;
    w.altitude_m = d.altitude_m;
    memcpy(out, &w, sizeof(WireBarometer_t));
    return sizeof(WireBarometer_t);
#endif
}

size_t DataCollector::packCurrentData(uint8_t* out, size_t max_len) {
    if (max_len < sizeof(WireCurrent_t)) return 0;

#ifdef TEST_MODE
    // Generate fake current sensor data
    WireCurrent_t w{};
    w.version = 1;
    w.timestamp_ms = millis();
    w.current_a = 0.5;
    w.voltage_v = 12.3;
    w.power_w = 6.15;
    w.raw_adc = 2048;
    memcpy(out, &w, sizeof(WireCurrent_t));
    return sizeof(WireCurrent_t);
#else
    if (!current_sensor->isOnline()) return 0;
    WireCurrent_t w{};
    w.version = 1;
    CurrentData_t d;
    if (!current_sensor->getLatestReading(d)) return 0;
    w.timestamp_ms = d.timestamp;
    w.current_a = d.current_a;
    w.voltage_v = d.voltage_v;
    w.power_w = d.power_w;
    w.raw_adc = current_sensor->getRawADC();
    memcpy(out, &w, sizeof(WireCurrent_t));
    return sizeof(WireCurrent_t);
#endif
}

size_t DataCollector::packHeartbeat(uint8_t* out, size_t max_len) {
    if (max_len < sizeof(WireHeartbeat_t)) return 0;
    updateSystemStatus();
    WireHeartbeat_t w{};
    w.version = 1;
    w.uptime_seconds = system_status.uptime_seconds;
    w.system_state = (uint8_t)system_status.system_state;
    memcpy(out, &w, sizeof(WireHeartbeat_t));
    return sizeof(WireHeartbeat_t);
}

size_t DataCollector::packStatus(uint8_t* out, size_t max_len) {
    if (max_len < sizeof(WireStatus_t)) return 0;
    updateSystemStatus();
    WireStatus_t w{};
    w.version = 1;
    w.uptime_seconds = system_status.uptime_seconds;
    w.system_state = (uint8_t)system_status.system_state;
    uint8_t flags = 0;
    if (system_status.lora_online) flags |= 1 << 0;
    if (system_status.radio433_online) flags |= 1 << 1;
    if (system_status.barometer_online) flags |= 1 << 2;
    if (system_status.current_sensor_online) flags |= 1 << 3;
    if (system_status.pi_connected) flags |= 1 << 4;
    w.flags = flags;
    w.packet_count_lora = system_status.packet_count_lora;
    w.packet_count_433 = system_status.packet_count_433;
    w.wakeup_time = system_status.wakeup_time;
    w.free_heap = ESP.getFreeHeap();
    w.chip_revision = ESP.getChipRevision();
    memcpy(out, &w, sizeof(WireStatus_t));
    return sizeof(WireStatus_t);
}