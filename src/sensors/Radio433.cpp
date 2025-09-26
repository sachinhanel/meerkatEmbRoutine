// Radio433.cpp
#include "Radio433.h"

Radio433::Radio433() : buffer_head(0), buffer_tail(0), packet_count(0), is_initialized(false) {
    buffer_mutex = xSemaphoreCreateMutex();
    radio_module = new SX1278(new Module(RADIO433_CS_PIN, RADIO433_DIO0_PIN, RADIO433_RST_PIN));
}

Radio433::~Radio433() {
    end();
    if (buffer_mutex != nullptr) {
        vSemaphoreDelete(buffer_mutex);
    }
    delete radio_module;
}

bool Radio433::begin() {
    // Initialize the 433MHz module
    int state = radio_module->begin(RADIO433_FREQUENCY, RADIO433_BITRATE, 
                                   RADIO433_FREQ_DEV, 125.0, 10, 32);
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Radio433: Failed to initialize (error: %d)\n", state);
        return false;
    }
    
    // Set additional parameters
    radio_module->setEncoding(RADIOLIB_ENCODING_NRZ);
    radio_module->setCRC(true);
    
    is_initialized = true;
    startReceive();
    Serial.println("Radio433: Initialized successfully");
    return true;
}

void Radio433::end() {
    if (is_initialized) {
        radio_module->standby();
        is_initialized = false;
    }
}

void Radio433::startReceive() {
    if (is_initialized) {
        radio_module->startReceive();
    }
}

void Radio433::checkForPackets() {
    if (!is_initialized) return;
    
    String received_data;
    int state = radio_module->readData(received_data);
    
    if (state == RADIOLIB_ERR_NONE) {
        Radio433Packet_t new_packet;
        new_packet.timestamp = millis();
        new_packet.rssi = radio_module->getRSSI();
        new_packet.data_length = min((int)received_data.length(), MAX_PACKET_SIZE);
        
        // Copy string data to packet buffer
        memcpy(new_packet.data, received_data.c_str(), new_packet.data_length);
        
        addPacketToBuffer(new_packet);
        packet_count++;
        
        Serial.printf("433MHz RX: %d bytes, RSSI: %d dBm\n", 
                      new_packet.data_length, new_packet.rssi);
        
        // Restart receiving
        startReceive();
    }
    else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        // Timeout is normal, restart receiving
        startReceive();
    }
    else if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("433MHz RX Error: %d\n", state);
        startReceive();
    }
}

void Radio433::addPacketToBuffer(const Radio433Packet_t& packet) {
    if (xSemaphoreTake(buffer_mutex, portMAX_DELAY) == pdTRUE) {
        packet_buffer[buffer_head] = packet;
        buffer_head = (buffer_head + 1) % RADIO433_BUFFER_SIZE;
        
        // Handle buffer overflow
        if (buffer_head == buffer_tail) {
            buffer_tail = (buffer_tail + 1) % RADIO433_BUFFER_SIZE;
        }
        
        xSemaphoreGive(buffer_mutex);
    }
}

bool Radio433::hasNewPackets() const {
    return buffer_head != buffer_tail;
}

uint16_t Radio433::getPacketCount() const {
    return packet_count;
}

bool Radio433::getLatestPacket(Radio433Packet_t& packet) {
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (buffer_head != buffer_tail) {
            // Get the most recent packet (one before head)
            uint16_t latest_index = (buffer_head - 1 + RADIO433_BUFFER_SIZE) % RADIO433_BUFFER_SIZE;
            packet = packet_buffer[latest_index];
            xSemaphoreGive(buffer_mutex);
            return true;
        }
        xSemaphoreGive(buffer_mutex);
    }
    return false;
}

bool Radio433::getAllPackets(Radio433Packet_t* packets, uint16_t max_packets, uint16_t& actual_count) {
    actual_count = 0;
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint16_t current = buffer_tail;
        while (current != buffer_head && actual_count < max_packets) {
            packets[actual_count] = packet_buffer[current];
            current = (current + 1) % RADIO433_BUFFER_SIZE;
            actual_count++;
        }
        // Clear the buffer after reading
        buffer_tail = buffer_head;
        xSemaphoreGive(buffer_mutex);
        return true;
    }
    return false;
}

int Radio433::getRSSI() {
    return is_initialized ? radio_module->getRSSI() : -999;
}

void Radio433::clearBuffer() {
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        buffer_head = buffer_tail = 0;
        xSemaphoreGive(buffer_mutex);
    }
}