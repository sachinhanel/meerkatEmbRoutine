// LoRa915.h
#ifndef LORA915_H
#define LORA915_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "config.h"

class LoRa915 {
private:
    LoRaPacket_t packet_buffer[LORA_BUFFER_SIZE];
    uint16_t buffer_head;
    uint16_t buffer_tail;
    uint16_t packet_count;
    bool is_initialized;
    SemaphoreHandle_t buffer_mutex;
    
    void addPacketToBuffer(const LoRaPacket_t& packet);
    
public:
    LoRa915();
    ~LoRa915();
    
    bool begin();
    void end();
    bool isOnline() const { return is_initialized; }
    
    // Receiving functions
    void checkForPackets();
    bool hasNewPackets() const;
    uint16_t getPacketCount() const;
    bool getLatestPacket(LoRaPacket_t& packet);
    bool getAllPackets(LoRaPacket_t* packets, uint16_t max_packets, uint16_t& actual_count);
    
    // Transmitting functions
    bool sendPacket(const uint8_t* data, uint8_t length);
    bool sendString(const String& message);
    
    // Status functions
    int getRSSI();
    float getSNR();
    void clearBuffer();
};

#endif // LORA915_H