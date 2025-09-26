// LoRa915.cpp
#include "LoRa915.h"

LoRa915::LoRa915() : buffer_head(0), buffer_tail(0), packet_count(0), is_initialized(false) {
    buffer_mutex = xSemaphoreCreateMutex();
}

LoRa915::~LoRa915() {
    end();
    if (buffer_mutex != nullptr) {
        vSemaphoreDelete(buffer_mutex);
    }
}

bool LoRa915::begin() {
    // Set CS pin for LoRa module
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa 915MHz: Failed to initialize");
        return false;
    }
    
    // Configure LoRa parameters
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setTxPower(LORA_TX_POWER);
    
    // Enable CRC
    LoRa.enableCrc();
    
    is_initialized = true;
    Serial.println("LoRa 915MHz: Initialized successfully");
    return true;
}

void LoRa915::end() {
    if (is_initialized) {
        LoRa.end();
        is_initialized = false;
    }
}

void LoRa915::checkForPackets() {
    if (!is_initialized) return;
    
    int packet_size = LoRa.parsePacket();
    if (packet_size > 0) {
        LoRaPacket_t new_packet;
        new_packet.timestamp = millis();
        new_packet.rssi = LoRa.packetRssi();
        new_packet.snr = LoRa.packetSnr();  // Fixed: use packetSnr() instead of snr()
        new_packet.data_length = min(packet_size, MAX_PACKET_SIZE);
        
        // Read packet data
        for (int i = 0; i < new_packet.data_length; i++) {
            new_packet.data[i] = LoRa.read();
        }
        
        addPacketToBuffer(new_packet);
        packet_count++;
        
        Serial.printf("LoRa RX: %d bytes, RSSI: %d dBm, SNR: %.2f dB\n", 
                      new_packet.data_length, new_packet.rssi, new_packet.snr);
    }
}

void LoRa915::addPacketToBuffer(const LoRaPacket_t& packet) {
    if (xSemaphoreTake(buffer_mutex, portMAX_DELAY) == pdTRUE) {
        packet_buffer[buffer_head] = packet;
        buffer_head = (buffer_head + 1) % LORA_BUFFER_SIZE;
        
        // Handle buffer overflow
        if (buffer_head == buffer_tail) {
            buffer_tail = (buffer_tail + 1) % LORA_BUFFER_SIZE;
        }
        
        xSemaphoreGive(buffer_mutex);
    }
}

bool LoRa915::hasNewPackets() const {
    return buffer_head != buffer_tail;
}

uint16_t LoRa915::getPacketCount() const {
    return packet_count;
}

bool LoRa915::getLatestPacket(LoRaPacket_t& packet) {
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (buffer_head != buffer_tail) {
            // Get the most recent packet (one before head)
            uint16_t latest_index = (buffer_head - 1 + LORA_BUFFER_SIZE) % LORA_BUFFER_SIZE;
            packet = packet_buffer[latest_index];
            xSemaphoreGive(buffer_mutex);
            return true;
        }
        xSemaphoreGive(buffer_mutex);
    }
    return false;
}

bool LoRa915::getAllPackets(LoRaPacket_t* packets, uint16_t max_packets, uint16_t& actual_count) {
    actual_count = 0;
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint16_t current = buffer_tail;
        while (current != buffer_head && actual_count < max_packets) {
            packets[actual_count] = packet_buffer[current];
            current = (current + 1) % LORA_BUFFER_SIZE;
            actual_count++;
        }
        // Clear the buffer after reading
        buffer_tail = buffer_head;
        xSemaphoreGive(buffer_mutex);
        return true;
    }
    return false;
}

bool LoRa915::sendPacket(const uint8_t* data, uint8_t length) {
    if (!is_initialized) return false;
    
    LoRa.beginPacket();
    LoRa.write(data, length);
    bool success = LoRa.endPacket();
    
    if (success) {
        Serial.printf("LoRa TX: %d bytes sent\n", length);
    } else {
        Serial.println("LoRa TX: Failed to send packet");
    }
    
    return success;
}

bool LoRa915::sendString(const String& message) {
    return sendPacket((const uint8_t*)message.c_str(), message.length());
}

int LoRa915::getRSSI() {
    return is_initialized ? LoRa.rssi() : -999;
}

float LoRa915::getSNR() {
    return is_initialized ? LoRa.packetSnr() : -999.0;  // Fixed: use packetSnr()
}

void LoRa915::clearBuffer() {
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        buffer_head = buffer_tail = 0;
        xSemaphoreGive(buffer_mutex);
    }
}
