#include "CommProtocol.h"

// Debug printing control - disable when using binary protocol with Pi
// Set to false to prevent ASCII debug messages from mixing with binary data
#define COMM_DEBUG_ENABLED false

#if COMM_DEBUG_ENABLED
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif

CommProtocol::CommProtocol(DataCollector* collector, Stream* serial) :
    data_collector(collector),
    serial_port(serial),
    current_state(WAIT_FOR_HELLO),
    received_peripheral_id(0),
    expected_message_length(0),
    received_message_length(0),
    last_activity_time(0),
    hello_received_time(0),
    response_index(0),
    wakeup_callback(nullptr) {

    memset(message_buffer, 0, sizeof(message_buffer));
}

CommProtocol::~CommProtocol() {
    end(); 
}

bool CommProtocol::begin() {
    
    serial_port->setTimeout(100);
    resetState();
    
    DEBUG_PRINTLN("CommProtocol: USB Serial communication initialized");
    return true;
}

void CommProtocol::end() {

}

void CommProtocol::process() {
    // Check for timeout
    if (current_state != WAIT_FOR_HELLO && 
        (millis() - last_activity_time > PI_COMM_TIMEOUT)) {
        DEBUG_PRINTLN("CommProtocol: Timeout - resetting state");
        resetState();
        return;
    }
    
    // Process incoming data based on current state
    while (serial_port->available()) {
        uint8_t received_byte = serial_port->read();
        last_activity_time = millis();
        
        switch (current_state) {
            case WAIT_FOR_HELLO:
                if (received_byte == HELLO_BYTE) {
                    current_state = WAIT_FOR_PERIPHERAL_ID;
                    hello_received_time = millis();
                    DEBUG_PRINTLN("\n>>> CommProtocol: HELLO received (0x7E)");
                } else {
                    DEBUG_PRINTF(">>> CommProtocol: Expected HELLO (0x%02X), got 0x%02X\n",
                                  HELLO_BYTE, received_byte);
                }
                break;
                
            case WAIT_FOR_PERIPHERAL_ID:
                received_peripheral_id = received_byte;
                current_state = WAIT_FOR_LENGTH;
                DEBUG_PRINTF(">>> Peripheral ID: 0x%02X\n", received_peripheral_id);
                break;

            case WAIT_FOR_LENGTH:
                expected_message_length = received_byte;
                received_message_length = 0;
                current_state = (expected_message_length > 0) ? WAIT_FOR_MESSAGE_DATA : WAIT_FOR_GOODBYE;
                DEBUG_PRINTF(">>> Message Length: %d bytes\n", expected_message_length);
                break;

            case WAIT_FOR_MESSAGE_DATA:
                if (received_message_length < expected_message_length &&
                    received_message_length < sizeof(message_buffer)) {
                    message_buffer[received_message_length] = received_byte;
                    received_message_length++;

                    if (received_message_length >= expected_message_length) {
                        current_state = WAIT_FOR_GOODBYE;
                        DEBUG_PRINTF(">>> Message Data: ");
                        for (uint8_t i = 0; i < received_message_length; i++) {
                            DEBUG_PRINTF("%02X ", message_buffer[i]);
                        }
                        DEBUG_PRINTLN("");
                    }
                } else {
                    DEBUG_PRINTLN("CommProtocol: Message buffer overflow");
                    sendErrorResponse("Message too long");
                    resetState();
                }
                break;

            case WAIT_FOR_GOODBYE:
                if (received_byte == GOODBYE_BYTE) {
                    DEBUG_PRINTLN(">>> GOODBYE received (0x7F) - Processing command...");
                    processMessage(received_peripheral_id, message_buffer, received_message_length);
                } else {
                    DEBUG_PRINTF(">>> Expected GOODBYE (0x%02X), got 0x%02X\n",
                                  GOODBYE_BYTE, received_byte);
                    sendErrorResponse("Invalid packet format");
                }
                resetState();
                break;

            case SENDING_RESPONSE:
                // Should not receive data while sending response
                DEBUG_PRINTLN("CommProtocol: Unexpected data while sending response");
                resetState();
                break;
        }
    }
}

void CommProtocol::processMessage(uint8_t peripheral_id, const uint8_t* message_data, uint8_t length) {
    // Extract command (first byte of payload)
    if (length < 1) {
        DEBUG_PRINTLN("CommProtocol: Empty payload");
        sendPeripheralErrorResponse(peripheral_id, "Empty payload");
        return;
    }

    uint8_t command = message_data[0];
    DEBUG_PRINTF("CommProtocol: Routing to peripheral 0x%02X, command 0x%02X\n", peripheral_id, command);

    // Route to appropriate peripheral handler
    switch (peripheral_id) {
        case PERIPHERAL_ID_SYSTEM:
            processSystemCommand(command);
            break;

        case PERIPHERAL_ID_LORA_915:
            processLoRa915Command(command);
            break;

        case PERIPHERAL_ID_LORA_433:
            processLoRa433Command(command);
            break;

        case PERIPHERAL_ID_BAROMETER:
            processBarometerCommand(command);
            break;

        case PERIPHERAL_ID_CURRENT:
            processCurrentSensorCommand(command);
            break;

        default:
            DEBUG_PRINTF("CommProtocol: Unknown peripheral ID: 0x%02X\n", peripheral_id);
            sendPeripheralErrorResponse(peripheral_id, "Unknown peripheral");
            break;
    }
}

// ====================================================================
// SYSTEM COMMAND HANDLER (peripheral_id = 0x00)
// ====================================================================
void CommProtocol::processSystemCommand(uint8_t command) {
    DEBUG_PRINTF("CommProtocol: Processing SYSTEM command 0x%02X\n", command);
    uint8_t buf[64];
    size_t n;

    switch (command) {
        case CMD_GET_ALL:  // Same as CMD_GET_STATUS for system
            DEBUG_PRINTLN("  -> GET_ALL (System Status)");
            n = data_collector->packStatus(buf, sizeof(buf));
            if (n > 0) {
                sendPeripheralResponse(PERIPHERAL_ID_SYSTEM, buf, n);
            } else {
                sendPeripheralErrorResponse(PERIPHERAL_ID_SYSTEM, "No status");
            }
            break;

        case CMD_SYSTEM_WAKEUP:
            DEBUG_PRINTLN("  -> WAKEUP");
            data_collector->setSystemState(SYSTEM_OPERATIONAL);

            // Trigger callback if set (for TEST_MODE replay)
            if (wakeup_callback != nullptr) {
                wakeup_callback();
            }

            // Send acknowledgment - echo back the CMD_SYSTEM_WAKEUP byte
            buf[0] = CMD_SYSTEM_WAKEUP;
            sendPeripheralResponse(PERIPHERAL_ID_SYSTEM, buf, 1);
            DEBUG_PRINTLN("  -> System now OPERATIONAL");
            break;

        case CMD_SYSTEM_SLEEP:
            DEBUG_PRINTLN("  -> SLEEP");
            data_collector->setSystemState(SYSTEM_SLEEPING);
            buf[0] = CMD_SYSTEM_SLEEP;
            sendPeripheralResponse(PERIPHERAL_ID_SYSTEM, buf, 1);
            break;

        case CMD_SYSTEM_RESET:
            DEBUG_PRINTLN("  -> RESET");
            buf[0] = CMD_SYSTEM_RESET;
            sendPeripheralResponse(PERIPHERAL_ID_SYSTEM, buf, 1);
            delay(100);
            ESP.restart();
            break;

        default:
            DEBUG_PRINTF("  -> Unknown system command: 0x%02X\n", command);
            sendPeripheralErrorResponse(PERIPHERAL_ID_SYSTEM, "Unknown system command");
            break;
    }
}

// ====================================================================
// 915MHz LoRa HANDLER (peripheral_id = 0x01)
// ====================================================================
void CommProtocol::processLoRa915Command(uint8_t command) {
    DEBUG_PRINTF("CommProtocol: Processing LoRa 915MHz command 0x%02X\n", command);
    uint8_t buf[128];
    size_t n;

    switch (command) {
        case CMD_GET_ALL:
            DEBUG_PRINTLN("  -> GET_ALL (915MHz LoRa Data)");
            n = data_collector->packLoRaData(buf, sizeof(buf));
            if (n > 0) {
                sendPeripheralResponse(PERIPHERAL_ID_LORA_915, buf, n);
            } else {
                sendPeripheralErrorResponse(PERIPHERAL_ID_LORA_915, "No 915MHz LoRa data");
            }
            break;

        case CMD_GET_STATUS:
            DEBUG_PRINTLN("  -> GET_STATUS");
            // Future: Return LoRa module status (online, frequency, etc.)
            sendPeripheralErrorResponse(PERIPHERAL_ID_LORA_915, "Status not implemented");
            break;

        default:
            DEBUG_PRINTF("  -> Unknown 915MHz LoRa command: 0x%02X\n", command);
            sendPeripheralErrorResponse(PERIPHERAL_ID_LORA_915, "Unknown command");
            break;
    }
}

// ====================================================================
// 433MHz LoRa HANDLER (peripheral_id = 0x02)
// ====================================================================
// Note: 433MHz module is same LoRa chip as 915MHz, just different frequency
void CommProtocol::processLoRa433Command(uint8_t command) {
    DEBUG_PRINTF("CommProtocol: Processing LoRa 433MHz command 0x%02X\n", command);
    uint8_t buf[128];
    size_t n;

    switch (command) {
        case CMD_GET_ALL:
            DEBUG_PRINTLN("  -> GET_ALL (433MHz LoRa Data)");
            n = data_collector->pack433Data(buf, sizeof(buf));
            if (n > 0) {
                sendPeripheralResponse(PERIPHERAL_ID_LORA_433, buf, n);
            } else {
                sendPeripheralErrorResponse(PERIPHERAL_ID_LORA_433, "No 433MHz LoRa data");
            }
            break;

        case CMD_GET_STATUS:
            DEBUG_PRINTLN("  -> GET_STATUS");
            // Future: Return LoRa module status (online, frequency, etc.)
            sendPeripheralErrorResponse(PERIPHERAL_ID_LORA_433, "Status not implemented");
            break;

        default:
            DEBUG_PRINTF("  -> Unknown 433MHz LoRa command: 0x%02X\n", command);
            sendPeripheralErrorResponse(PERIPHERAL_ID_LORA_433, "Unknown command");
            break;
    }
}

// ====================================================================
// BAROMETER HANDLER (peripheral_id = 0x03)
// ====================================================================
void CommProtocol::processBarometerCommand(uint8_t command) {
    DEBUG_PRINTF("CommProtocol: Processing BAROMETER command 0x%02X\n", command);
    uint8_t buf[64];
    size_t n;

    switch (command) {
        case CMD_GET_ALL:
            DEBUG_PRINTLN("  -> GET_ALL (Barometer Data)");
            n = data_collector->packBarometerData(buf, sizeof(buf));
            if (n > 0) {
                sendPeripheralResponse(PERIPHERAL_ID_BAROMETER, buf, n);
            } else {
                sendPeripheralErrorResponse(PERIPHERAL_ID_BAROMETER, "No barometer data");
            }
            break;

        case CMD_GET_STATUS:
            DEBUG_PRINTLN("  -> GET_STATUS");
            sendPeripheralErrorResponse(PERIPHERAL_ID_BAROMETER, "Status not implemented");
            break;

        default:
            DEBUG_PRINTF("  -> Unknown barometer command: 0x%02X\n", command);
            sendPeripheralErrorResponse(PERIPHERAL_ID_BAROMETER, "Unknown command");
            break;
    }
}

// ====================================================================
// CURRENT SENSOR HANDLER (peripheral_id = 0x04)
// ====================================================================
void CommProtocol::processCurrentSensorCommand(uint8_t command) {
    DEBUG_PRINTF("CommProtocol: Processing CURRENT SENSOR command 0x%02X\n", command);
    uint8_t buf[64];
    size_t n;

    switch (command) {
        case CMD_GET_ALL:
            DEBUG_PRINTLN("  -> GET_ALL (Current/Voltage Data)");
            n = data_collector->packCurrentData(buf, sizeof(buf));
            if (n > 0) {
                sendPeripheralResponse(PERIPHERAL_ID_CURRENT, buf, n);
            } else {
                sendPeripheralErrorResponse(PERIPHERAL_ID_CURRENT, "No current sensor data");
            }
            break;

        case CMD_GET_STATUS:
            DEBUG_PRINTLN("  -> GET_STATUS");
            sendPeripheralErrorResponse(PERIPHERAL_ID_CURRENT, "Status not implemented");
            break;

        default:
            DEBUG_PRINTF("  -> Unknown current sensor command: 0x%02X\n", command);
            sendPeripheralErrorResponse(PERIPHERAL_ID_CURRENT, "Unknown command");
            break;
    }
}

void CommProtocol::sendResponse(const String& response) {
    current_state = SENDING_RESPONSE;

    // Send RESPONSE byte
    serial_port->write((uint8_t)RESPONSE_BYTE);

    // Send peripheral ID (system response)
    serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);

    // Send response length
    uint8_t length = min((int)response.length(), 255);
    serial_port->write(length);

    // Send response data
    if (length > 0) {
        serial_port->write((const uint8_t*)response.c_str(), length);
    }

    // Send GOODBYE byte
    serial_port->write((uint8_t)GOODBYE_BYTE);

    // Flush the output buffer
    serial_port->flush();

    DEBUG_PRINTF("CommProtocol: Response sent (%d bytes)\n", length);

    // Reset state machine to accept next command
    resetState();
}

void CommProtocol::sendErrorResponse(const String& error_message) {
    sendPeripheralErrorResponse(PERIPHERAL_ID_SYSTEM, error_message);
}

void CommProtocol::sendPeripheralErrorResponse(uint8_t peripheral_id, const String& error_message) {
    // Minimal binary error: version(1)=1, code(1)=1, len(1)=min(50), ascii message bytes (truncated)
    uint8_t buf[64];
    size_t msg_len = error_message.length();
    if (msg_len > 60) msg_len = 60;
    buf[0] = 1; // version
    buf[1] = 1; // code: generic error
    buf[2] = (uint8_t)msg_len;
    memcpy(&buf[3], error_message.c_str(), msg_len);
    current_state = SENDING_RESPONSE;
    serial_port->write((uint8_t)RESPONSE_BYTE);
    serial_port->write((uint8_t)peripheral_id);  // Use the correct peripheral ID
    serial_port->write((uint8_t)(3 + msg_len));
    serial_port->write(buf, 3 + msg_len);
    serial_port->write((uint8_t)GOODBYE_BYTE);
    serial_port->flush();

    // Reset state machine to accept next command
    resetState();
}

void CommProtocol::sendPeripheralResponse(uint8_t peripheral_id, const uint8_t* data, size_t length) {
    // Generic helper to send a response for any peripheral
    current_state = SENDING_RESPONSE;
    serial_port->write((uint8_t)RESPONSE_BYTE);
    serial_port->write((uint8_t)peripheral_id);
    serial_port->write((uint8_t)length);
    if (length > 0) {
        serial_port->write(data, length);
    }
    serial_port->write((uint8_t)GOODBYE_BYTE);
    serial_port->flush();
    DEBUG_PRINTF("CommProtocol: Response sent for peripheral 0x%02X (%d bytes)\n", peripheral_id, (int)length);

    // Reset state machine to accept next command
    resetState();
}

void CommProtocol::resetState() {
    current_state = WAIT_FOR_HELLO;
    received_peripheral_id = 0;
    expected_message_length = 0;
    received_message_length = 0;
    response_buffer = "";
    response_index = 0;
    memset(message_buffer, 0, sizeof(message_buffer));
}

bool CommProtocol::isConnected() const {
    // Consider connected if we've had activity within the last 30 seconds
    return (millis() - last_activity_time < 30000);
}

// void CommProtocol::sendTestMessage() {
//     DynamicJsonDocument test_doc(1024);
//     test_doc["message"] = "ESP32 Ground Station Test";
//     test_doc["timestamp"] = millis();
//     test_doc["uptime"] = millis() / 1000;
    
//     String test_response;
//     serializeJson(test_doc, test_response);
//     sendResponse(test_response);
    
//     DEBUG_PRINTLN("CommProtocol: Test message sent");
// }

void CommProtocol::sendStatusUpdate() {
    // Send heartbeat if waiting for wakeup, otherwise send full status
    uint8_t buf[64];
    size_t n;

    if (data_collector->getSystemState() == SYSTEM_WAITING_FOR_WAKEUP) {
        // Before wakeup: send simple heartbeat (6 bytes)
        n = data_collector->packHeartbeat(buf, sizeof(buf));
        if (n > 0) {
            serial_port->write((uint8_t)RESPONSE_BYTE);
            serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
            serial_port->write((uint8_t)n);
            serial_port->write(buf, n);
            serial_port->write((uint8_t)GOODBYE_BYTE);
            serial_port->flush();
            DEBUG_PRINTF("CommProtocol: Sent heartbeat (%d bytes)\n", (int)n);
        }
    } else {
        // After wakeup: send full status (20 bytes)
        n = data_collector->packStatus(buf, sizeof(buf));
        if (n > 0) {
            serial_port->write((uint8_t)RESPONSE_BYTE);
            serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
            serial_port->write((uint8_t)n);
            serial_port->write(buf, n);
            serial_port->write((uint8_t)GOODBYE_BYTE);
            serial_port->flush();
            DEBUG_PRINTF("CommProtocol: Sent status update (%d bytes)\n", (int)n);
        }
    }
}

void CommProtocol::setWakeupCallback(void (*callback)()) {
    wakeup_callback = callback;
}

void CommProtocol::printStats() {
    DEBUG_PRINTLN("=== Communication Protocol Stats ===");
    DEBUG_PRINTF("Current State: ");
    switch (current_state) {
        case WAIT_FOR_HELLO: Serial.println("WAIT_FOR_HELLO"); break;
        case WAIT_FOR_PERIPHERAL_ID: Serial.println("WAIT_FOR_PERIPHERAL_ID"); break;
        case WAIT_FOR_LENGTH: Serial.println("WAIT_FOR_LENGTH"); break;
        case WAIT_FOR_MESSAGE_DATA: Serial.println("WAIT_FOR_MESSAGE_DATA"); break;
        case WAIT_FOR_GOODBYE: Serial.println("WAIT_FOR_GOODBYE"); break;
        case SENDING_RESPONSE: Serial.println("SENDING_RESPONSE"); break;
        default: Serial.println("UNKNOWN"); break;
    }
    DEBUG_PRINTF("Last Activity: %d ms ago\n", millis() - last_activity_time);
    DEBUG_PRINTF("Connected: %s\n", isConnected() ? "Yes" : "No");
    DEBUG_PRINTF("Serial Available: %d bytes\n", serial_port->available());
    DEBUG_PRINTLN("====================================");
}