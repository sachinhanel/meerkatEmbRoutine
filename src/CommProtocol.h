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
    void sendPeripheralErrorResponse(uint8_t peripheral_id, const String& error_message);
    void sendPeripheralResponse(uint8_t peripheral_id, const uint8_t* data, size_t length);

    void processMessage(uint8_t peripheral_id, const uint8_t* message_data, uint8_t length);

    // Peripheral command handlers
    void processSystemCommand(uint8_t command, const uint8_t* payload, uint8_t payload_length);
    void processLoRa915Command(uint8_t command, const uint8_t* payload, uint8_t payload_length);
    void processLoRa433Command(uint8_t command, const uint8_t* payload, uint8_t payload_length);
    void processBarometerCommand(uint8_t command, const uint8_t* payload, uint8_t payload_length);
    void processCurrentSensorCommand(uint8_t command, const uint8_t* payload, uint8_t payload_length);

    // Generic sensor command handlers (shared by all sensors)
    void handleSetPollRate(uint8_t peripheral_id, const uint8_t* payload, uint8_t length);
    void handleStopPoll(uint8_t peripheral_id);

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

    // Autonomous polling support
    void sendPeripheralData(uint8_t peripheral_id);  // Send data for one peripheral (used by polling & GET_ALL)

    // Testing functions
    void sendTestMessage();
    void printStats();
};

#endif // COMM_PROTOCOL_H