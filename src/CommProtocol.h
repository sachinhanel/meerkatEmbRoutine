// CommProtocol.h
#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "../include/config.h"
#include "DataCollector.h"

typedef enum {
    WAIT_FOR_HELLO,
    WAIT_FOR_PERIPHERAL_ID,
    WAIT_FOR_LENGTH,
    WAIT_FOR_MESSAGE_DATA,
    WAIT_FOR_GOODBYE,
    SENDING_RESPONSE
} CommState_t;

class CommProtocol {
private:
    Stream* serial_port;
    DataCollector* data_collector;

    CommState_t current_state;
    uint8_t received_peripheral_id;
    uint8_t expected_message_length;
    uint8_t received_message_length;
    uint8_t message_buffer[256];  // Buffer for incoming message
    uint32_t last_activity_time;
    uint32_t hello_received_time;

    // Buffer for building response
    String response_buffer;
    size_t response_index;

    // Callback for WAKEUP command (for TEST_MODE replay)
    void (*wakeup_callback)();
    
    bool validatePacket();
    void sendResponse(const String& response);
    void sendErrorResponse(const String& error_message);
    void processMessage(uint8_t peripheral_id, const uint8_t* message_data, uint8_t length);
    void processSystemCommand(uint8_t command);
    void resetState();
    
public:
    CommProtocol(DataCollector* collector, Stream* serial);
    ~CommProtocol();
    
    bool begin();
    void end();
    
    // Main processing function - call this in main loop
    void process();

    // Unsolicited messages (for testing/heartbeat)
    void sendStatusUpdate();

    // System state management
    bool isSystemAwake();
    void sendWakeupResponse();
    void setWakeupCallback(void (*callback)());

    // Status functions
    bool isConnected() const;
    uint32_t getLastActivityTime() const { return last_activity_time; }
    CommState_t getState() const { return current_state; }
    
    // Testing functions
    void sendTestMessage();
    void printStats();
};

#endif // COMM_PROTOCOL_H