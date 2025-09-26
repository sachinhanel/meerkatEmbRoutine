#include "CommProtocol.h"

CommProtocol::CommProtocol(DataCollector* collector) : 
    data_collector(collector),
    current_state(WAIT_FOR_HELLO),
    received_peripheral_id(0),
    expected_message_length(0),
    received_message_length(0),
    last_activity_time(0),
    hello_received_time(0),
    response_index(0) {
    
    serial_port = &Serial2; // Use Serial2 for Pi communication
    memset(message_buffer, 0, sizeof(message_buffer));
}

CommProtocol::~CommProtocol() {
    end();
}

bool CommProtocol::begin() {
    // Initialize UART communication with Raspberry Pi
    serial_port->begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
    serial_port->setTimeout(100); // 100ms timeout for reads
    
    resetState();
    
    Serial.println("CommProtocol: UART communication initialized");
    Serial.printf("CommProtocol: Listening on Serial2 at %d baud\n", PI_UART_BAUD);
    
    return true;
}

void CommProtocol::end() {
    if (serial_port) {
        serial_port->end();
    }
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
                    Serial.println("CommProtocol: HELLO received");
                } else {
                    Serial.printf("CommProtocol: Expected HELLO (0x%02X), got 0x%02X\n", 
                                  HELLO_BYTE, received_byte);
                }
                break;
                
            case WAIT_FOR_PERIPHERAL_ID:
                received_peripheral_id = received_byte;
                current_state = WAIT_FOR_LENGTH;
                Serial.printf("CommProtocol: Peripheral ID received: 0x%02X\n", received_peripheral_id);
                break;
                
            case WAIT_FOR_LENGTH:
                expected_message_length = received_byte;
                received_message_length = 0;
                current_state = (expected_message_length > 0) ? WAIT_FOR_MESSAGE_DATA : WAIT_FOR_GOODBYE;
                Serial.printf("CommProtocol: Message length: %d bytes\n", expected_message_length);
                break;
                
            case WAIT_FOR_MESSAGE_DATA:
                if (received_message_length < expected_message_length && 
                    received_message_length < sizeof(message_buffer)) {
                    message_buffer[received_message_length] = received_byte;
                    received_message_length++;
                    
                    if (received_message_length >= expected_message_length) {
                        current_state = WAIT_FOR_GOODBYE;
                        Serial.printf("CommProtocol: Message data received (%d bytes)\n", received_message_length);
                    }
                } else {
                    Serial.println("CommProtocol: Message buffer overflow");
                    sendErrorResponse("Message too long");
                    resetState();
                }
                break;
                
            case WAIT_FOR_GOODBYE:
                if (received_byte == GOODBYE_BYTE) {
                    Serial.println("CommProtocol: GOODBYE received - processing message");
                    processMessage(received_peripheral_id, message_buffer, received_message_length);
                } else {
                    Serial.printf("CommProtocol: Expected GOODBYE (0x%02X), got 0x%02X\n", 
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

void CommProtocol::processCommand(uint8_t command) {
    JsonDocument doc;
    String json_response;
    bool success = false;
    
    switch (command) {
        case CMD_GET_LORA_DATA:
            Serial.println("CommProtocol: Processing GET_LORA_DATA");
            success = data_collector->getLoRaData(doc);
            break;
            
        case CMD_GET_433_DATA:
            Serial.println("CommProtocol: Processing GET_433_DATA");
            success = data_collector->get433Data(doc);
            break;
            
        case CMD_GET_BAROMETER_DATA:
            Serial.println("CommProtocol: Processing GET_BAROMETER_DATA");
            success = data_collector->getBarometerData(doc);
            break;
            
        case CMD_GET_CURRENT_DATA:
            Serial.println("CommProtocol: Processing GET_CURRENT_DATA");
            success = data_collector->getCurrentData(doc);
            break;
            
        case CMD_GET_ALL_DATA:
            Serial.println("CommProtocol: Processing GET_ALL_DATA");
            success = data_collector->getAllData(doc);
            break;
            
        case CMD_GET_STATUS:
            Serial.println("CommProtocol: Processing GET_STATUS");
            success = data_collector->getSystemStatus(doc);
            break;
            
        default:
            Serial.printf("CommProtocol: Unknown command: 0x%02X\n", command);
            sendErrorResponse("Unknown command");
            return;
    }
    
    if (success) {
        // Serialize JSON to string
        serializeJson(doc, json_response);
        sendResponse(json_response);
    } else {
        sendErrorResponse("Failed to process command");
    }
}

void CommProtocol::sendResponse(const String& response) {
    current_state = SENDING_RESPONSE;
    
    // Send HELLO byte
    serial_port->write(HELLO_BYTE);
    
    // Send peripheral ID (system response)
    serial_port->write(PERIPHERAL_ID_SYSTEM);
    
    // Send response length
    uint8_t length = min((int)response.length(), 255);
    serial_port->write(length);
    
    // Send response data
    if (length > 0) {
        serial_port->write((const uint8_t*)response.c_str(), length);
    }
    
    // Send GOODBYE byte
    serial_port->write(GOODBYE_BYTE);
    
    // Flush the output buffer
    serial_port->flush();
    
    Serial.printf("CommProtocol: Response sent (%d bytes)\n", length);
}

void CommProtocol::sendErrorResponse(const String& error_message) {
    JsonDocument error_doc;
    error_doc["error"] = error_message;
    error_doc["timestamp"] = millis();
    
    String error_response;
    serializeJson(error_doc, error_response);
    sendResponse(error_response);
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

void CommProtocol::sendTestMessage() {
    JsonDocument test_doc;
    test_doc["message"] = "ESP32 Ground Station Test";
    test_doc["timestamp"] = millis();
    test_doc["uptime"] = millis() / 1000;
    
    String test_response;
    serializeJson(test_doc, test_response);
    sendResponse(test_response);
    
    Serial.println("CommProtocol: Test message sent");
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