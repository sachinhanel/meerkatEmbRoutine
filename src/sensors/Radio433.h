// Radio433.h
#ifndef RADIO433_H
#define RADIO433_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "config.h"

class Radio433 {
private:
    Radio433Packet_t packet_buffer[RADIO433_BUFFER_SIZE];
    uint16_t buffer_head;
    uint16_t buffer_tail;
    uint16_t packet_count;
    bool is_initialized;
    SemaphoreHandle_t buffer_mutex;
    
    SX1278* radio_module;
    
    void addPacketToBuffer(const Radio433Packet_t& packet);
    
public:
    Radio433();
    ~Radio433();
    
    bool begin();
    void end();
    bool isOnline() const { return is_initialized; }
    
    // Receiving functions
    void checkForPackets();
    bool hasNewPackets() const;
    uint16_t getPacketCount() const;
    bool getLatestPacket(Radio433Packet_t& packet);
    bool getAllPackets(Radio433Packet_t* packets, uint16_t max_packets, uint16_t& actual_count);
    
    // NOTE: 433MHz module is RECEIVE-ONLY
    // No transmit functions available
    
    // Status functions
    int getRSSI();
    void clearBuffer();
    void startReceive();
};

#endif // RADIO433_H