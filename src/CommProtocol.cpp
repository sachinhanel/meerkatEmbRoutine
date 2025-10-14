#include "CommProtocol.h"


CommProtocol::CommProtocol(DataCollector* collector, Stream* serial) :
    data_collector(collector),
    serial_port(serial),
    current_state(WAIT_FOR_HELLO),
    received_peripheral_id(0),
    expected_message_length(0),
    received_message_length(0),
    last_activity_time(0),
    hello_received_time(0),
    response_index(0) {

    memset(message_buffer, 0, sizeof(message_buffer));
}

CommProtocol::~CommProtocol() {
    end(); 
}

bool CommProtocol::begin() {
    
    serial_port->setTimeout(100);
    resetState();
    
    Serial.println("CommProtocol: USB Serial communication initialized");
    return true;
}

void CommProtocol::end() {

}

void CommProtocol::process() {
    // Check for timeout
    if (current_state != WAIT_FOR_HELLO && 
        (millis() - last_activity_time > PI_COMM_TIMEOUT)) {
        Serial.println("CommProtocol: Timeout - resetting state");
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
                    Serial.println("\n>>> CommProtocol: HELLO received (0x7E)");
                } else {
                    Serial.printf(">>> CommProtocol: Expected HELLO (0x%02X), got 0x%02X\n",
                                  HELLO_BYTE, received_byte);
                }
                break;
                
            case WAIT_FOR_PERIPHERAL_ID:
                received_peripheral_id = received_byte;
                current_state = WAIT_FOR_LENGTH;
                Serial.printf(">>> Peripheral ID: 0x%02X\n", received_peripheral_id);
                break;

            case WAIT_FOR_LENGTH:
                expected_message_length = received_byte;
                received_message_length = 0;
                current_state = (expected_message_length > 0) ? WAIT_FOR_MESSAGE_DATA : WAIT_FOR_GOODBYE;
                Serial.printf(">>> Message Length: %d bytes\n", expected_message_length);
                break;

            case WAIT_FOR_MESSAGE_DATA:
                if (received_message_length < expected_message_length &&
                    received_message_length < sizeof(message_buffer)) {
                    message_buffer[received_message_length] = received_byte;
                    received_message_length++;

                    if (received_message_length >= expected_message_length) {
                        current_state = WAIT_FOR_GOODBYE;
                        Serial.printf(">>> Message Data: ");
                        for (uint8_t i = 0; i < received_message_length; i++) {
                            Serial.printf("%02X ", message_buffer[i]);
                        }
                        Serial.println();
                    }
                } else {
                    Serial.println("CommProtocol: Message buffer overflow");
                    sendErrorResponse("Message too long");
                    resetState();
                }
                break;

            case WAIT_FOR_GOODBYE:
                if (received_byte == GOODBYE_BYTE) {
                    Serial.println(">>> GOODBYE received (0x7F) - Processing command...");
                    processMessage(received_peripheral_id, message_buffer, received_message_length);
                } else {
                    Serial.printf(">>> Expected GOODBYE (0x%02X), got 0x%02X\n",
                                  GOODBYE_BYTE, received_byte);
                    sendErrorResponse("Invalid packet format");
                }
                resetState();
                break;
                
            case SENDING_RESPONSE:
                // Should not receive data while sending response
                Serial.println("CommProtocol: Unexpected data while sending response");
                resetState();
                break;
        }
    }
}

void CommProtocol::processMessage(uint8_t peripheral_id, const uint8_t* message_data, uint8_t length) {
    // Route message based on peripheral ID
    if (peripheral_id == PERIPHERAL_ID_SYSTEM) {
        // System commands are single-byte commands
        if (length == 1) {
            processSystemCommand(message_data[0]);
        } else if (length == 0) {
            // Some commands might have no payload (like GET_STATUS)
            processSystemCommand(0);  // Or handle differently
        } else {
            Serial.printf("CommProtocol: Invalid system command length: %d\n", length);
            sendErrorResponse("Invalid command length");
        }
    } 
    else {
        // Handle other peripheral IDs here in the future
        Serial.printf("CommProtocol: Unknown peripheral ID: 0x%02X\n", peripheral_id);
        sendErrorResponse("Unknown peripheral");
    }
}

void CommProtocol::processSystemCommand(uint8_t command) {
    bool success = false;
    
    switch (command) {
        case CMD_GET_LORA_DATA: {
            Serial.println("CommProtocol: Processing GET_LORA_DATA (binary)");
            uint8_t buf[128];
            size_t n = data_collector->packLoRaData(buf, sizeof(buf));
            if (n > 0) {
                // Send framed binary payload
                current_state = SENDING_RESPONSE;
                serial_port->write((uint8_t)HELLO_BYTE);
                serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
                serial_port->write((uint8_t)n);
                serial_port->write(buf, n);
                serial_port->write((uint8_t)GOODBYE_BYTE);
                serial_port->flush();
                Serial.printf("CommProtocol: Response sent (%d bytes)\n", (int)n);
            } else {
                sendErrorResponse("No LoRa data");
            }
            return;
        }
        case CMD_GET_433_DATA: {
            Serial.println("CommProtocol: Processing GET_433_DATA (binary)");
            uint8_t buf[128];
            size_t n = data_collector->pack433Data(buf, sizeof(buf));
            if (n > 0) {
                current_state = SENDING_RESPONSE;
                serial_port->write((uint8_t)HELLO_BYTE);
                serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
                serial_port->write((uint8_t)n);
                serial_port->write(buf, n);
                serial_port->write((uint8_t)GOODBYE_BYTE);
                serial_port->flush();
                Serial.printf("CommProtocol: Response sent (%d bytes)\n", (int)n);
            } else {
                sendErrorResponse("No 433 data");
            }
            return;
        }
        case CMD_GET_BAROMETER_DATA: {
            Serial.println("CommProtocol: Processing GET_BAROMETER_DATA (binary)");
            uint8_t buf[64];
            size_t n = data_collector->packBarometerData(buf, sizeof(buf));
            if (n > 0) {
                current_state = SENDING_RESPONSE;
                serial_port->write((uint8_t)HELLO_BYTE);
                serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
                serial_port->write((uint8_t)n);
                serial_port->write(buf, n);
                serial_port->write((uint8_t)GOODBYE_BYTE);
                serial_port->flush();
                Serial.printf("CommProtocol: Response sent (%d bytes)\n", (int)n);
            } else {
                sendErrorResponse("No barometer data");
            }
            return;
        }
        case CMD_GET_CURRENT_DATA: {
            Serial.println("CommProtocol: Processing GET_CURRENT_DATA (binary)");
            uint8_t buf[64];
            size_t n = data_collector->packCurrentData(buf, sizeof(buf));
            if (n > 0) {
                current_state = SENDING_RESPONSE;
                serial_port->write((uint8_t)HELLO_BYTE);
                serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
                serial_port->write((uint8_t)n);
                serial_port->write(buf, n);
                serial_port->write((uint8_t)GOODBYE_BYTE);
                serial_port->flush();
                Serial.printf("CommProtocol: Response sent (%d bytes)\n", (int)n);
            } else {
                sendErrorResponse("No current data");
            }
            return;
        }
        case CMD_GET_ALL_DATA: {
            Serial.println("CommProtocol: Processing GET_ALL_DATA (binary)");
            // Compact bundle: LoRa + 433 + Baro + Current, if size allows. Otherwise send error.
            uint8_t buf[255];
            size_t off = 0;
            size_t n;
            if ((n = data_collector->packLoRaData(buf + off, sizeof(buf) - off)) == 0) { /* ok if offline */ }
            else off += n;
            if ((n = data_collector->pack433Data(buf + off, sizeof(buf) - off)) == 0) { }
            else off += n;
            if ((n = data_collector->packBarometerData(buf + off, sizeof(buf) - off)) == 0) { }
            else off += n;
            if ((n = data_collector->packCurrentData(buf + off, sizeof(buf) - off)) == 0) { }
            else off += n;
            if (off == 0) { sendErrorResponse("No data"); return; }
            current_state = SENDING_RESPONSE;
            serial_port->write((uint8_t)HELLO_BYTE);
            serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
            serial_port->write((uint8_t)off);
            serial_port->write(buf, off);
            serial_port->write((uint8_t)GOODBYE_BYTE);
            serial_port->flush();
            Serial.printf("CommProtocol: Response sent (%d bytes)\n", (int)off);
            return;
        }
        case CMD_GET_STATUS: {
            Serial.println("CommProtocol: Processing GET_STATUS (binary)");
            uint8_t buf[64];
            size_t n = data_collector->packStatus(buf, sizeof(buf));
            if (n > 0) {
                current_state = SENDING_RESPONSE;
                serial_port->write((uint8_t)HELLO_BYTE);
                serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
                serial_port->write((uint8_t)n);
                serial_port->write(buf, n);
                serial_port->write((uint8_t)GOODBYE_BYTE);
                serial_port->flush();
                Serial.printf("CommProtocol: Response sent (%d bytes)\n", (int)n);
            } else {
                sendErrorResponse("No status");
            }
            return;
        }
        default:
            Serial.printf("CommProtocol: Unknown command: 0x%02X\n", command);
            sendErrorResponse("Unknown command");
            return;
    }

    if (!success) {
        sendErrorResponse("Failed to process command");
    }
}

void CommProtocol::sendResponse(const String& response) {
    current_state = SENDING_RESPONSE;
    
    // Send HELLO byte
    serial_port->write((uint8_t)HELLO_BYTE);
    
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
    
    Serial.printf("CommProtocol: Response sent (%d bytes)\n", length);
}

void CommProtocol::sendErrorResponse(const String& error_message) {
    // Minimal binary error: version(1)=1, code(1)=1, len(1)=min(50), ascii message bytes (truncated)
    uint8_t buf[64];
    size_t msg_len = error_message.length();
    if (msg_len > 60) msg_len = 60;
    buf[0] = 1; // version
    buf[1] = 1; // code: generic error
    buf[2] = (uint8_t)msg_len;
    memcpy(&buf[3], error_message.c_str(), msg_len);
    current_state = SENDING_RESPONSE;
    serial_port->write((uint8_t)HELLO_BYTE);
    serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
    serial_port->write((uint8_t)(3 + msg_len));
    serial_port->write(buf, 3 + msg_len);
    serial_port->write((uint8_t)GOODBYE_BYTE);
    serial_port->flush();
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
    
//     Serial.println("CommProtocol: Test message sent");
// }

void CommProtocol::sendStatusUpdate() {
    // Send an unsolicited status update (useful for heartbeat/testing)
    uint8_t buf[64];
    size_t n = data_collector->packStatus(buf, sizeof(buf));
    if (n > 0) {
        serial_port->write((uint8_t)HELLO_BYTE);
        serial_port->write((uint8_t)PERIPHERAL_ID_SYSTEM);
        serial_port->write((uint8_t)n);
        serial_port->write(buf, n);
        serial_port->write((uint8_t)GOODBYE_BYTE);
        serial_port->flush();
        Serial.printf("CommProtocol: Sent unsolicited status update (%d bytes)\n", (int)n);
    }
}

void CommProtocol::printStats() {
    Serial.println("=== Communication Protocol Stats ===");
    Serial.printf("Current State: ");
    switch (current_state) {
        case WAIT_FOR_HELLO: Serial.println("WAIT_FOR_HELLO"); break;
        case WAIT_FOR_PERIPHERAL_ID: Serial.println("WAIT_FOR_PERIPHERAL_ID"); break;
        case WAIT_FOR_LENGTH: Serial.println("WAIT_FOR_LENGTH"); break;
        case WAIT_FOR_MESSAGE_DATA: Serial.println("WAIT_FOR_MESSAGE_DATA"); break;
        case WAIT_FOR_GOODBYE: Serial.println("WAIT_FOR_GOODBYE"); break;
        case SENDING_RESPONSE: Serial.println("SENDING_RESPONSE"); break;
        default: Serial.println("UNKNOWN"); break;
    }
    Serial.printf("Last Activity: %d ms ago\n", millis() - last_activity_time);
    Serial.printf("Connected: %s\n", isConnected() ? "Yes" : "No");
    Serial.printf("Serial Available: %d bytes\n", serial_port->available());
    Serial.println("====================================");
}