/**
 * @file hybrid_protocol.cpp
 * @brief Hybrid Protocol Implementation
 *
 * Combines the reliability of newline-delimited framing with
 * the binary structure of the original protocol.
 *
 * Wire format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
 */

#include <Arduino.h>
#include "config.h"
#include "hybrid_protocol.h"

#ifdef HYBRID_PROTOCOL_MODE

// Include DataCollector for generating responses
#include "DataCollector.h"

// Global instances for both standalone and integrated modes
DataCollector* g_dataCollector = nullptr;
Stream* g_serial_port = nullptr;
SemaphoreHandle_t* g_serial_mutex = nullptr;
uint32_t g_last_activity_time = 0;

void setupHybridProtocol() {
    Serial.begin(115200);
    Serial.setRxBufferSize(1024);
    while (!Serial && millis() < 3000) delay(10);
    delay(500);

    Serial.println("\n====================================");
    Serial.println("  HYBRID PROTOCOL MODE");
    Serial.println("  Binary + Newline Framing");
    Serial.println("====================================");
    Serial.println();
    Serial.println("Format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\\n");
    Serial.println("Ready for requests...\n");
}

uint8_t calculateHybridChecksum(uint8_t pid, uint8_t len, const uint8_t* payload) {
    uint8_t chk = pid ^ len;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= payload[i];
    }
    return chk;
}

void sendHybridFrame(uint8_t peripheral_id, const uint8_t* payload, size_t payload_len) {
    if (payload_len > HYBRID_MAX_PAYLOAD_SIZE) {
        if (g_serial_port) g_serial_port->println(":ERROR:PAYLOAD_TOO_LARGE");
        return;
    }

    Stream* port = g_serial_port ? g_serial_port : &Serial;

    // Lock mutex if available
    if (g_serial_mutex != nullptr && *g_serial_mutex != nullptr) {
        xSemaphoreTake(*g_serial_mutex, portMAX_DELAY);
    }

    // Calculate checksum
    uint8_t checksum = calculateHybridChecksum(peripheral_id, payload_len, payload);

    // Build frame as hex string
    // Format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n

    // Start marker
    port->print('<');

    // Start markers (hex)
    port->print("AA55");

    // Peripheral ID (hex)
    if (peripheral_id < 0x10) port->print('0');
    port->print(peripheral_id, HEX);

    // Length (hex)
    if (payload_len < 0x10) port->print('0');
    port->print(payload_len, HEX);

    // Payload (hex) - write in chunks for large payloads
    const size_t CHUNK_SIZE = 64;
    for (size_t i = 0; i < payload_len; i++) {
        if (payload[i] < 0x10) port->print('0');
        port->print(payload[i], HEX);

        // Every CHUNK_SIZE bytes, flush to prevent buffer overflow
        if (i % CHUNK_SIZE == 0 && i > 0) {
            port->flush();
        }
    }

    // Checksum (hex)
    if (checksum < 0x10) port->print('0');
    port->print(checksum, HEX);

    // End markers (hex)
    port->print("55AA");

    // End marker + newline
    port->print('>');
    port->println();  // Newline terminator
    port->flush();

    // Unlock mutex
    if (g_serial_mutex != nullptr && *g_serial_mutex != nullptr) {
        xSemaphoreGive(*g_serial_mutex);
    }
}

bool parseHybridFrame(const char* line, uint8_t* pid, uint8_t* cmd, uint8_t* payload, size_t* payload_len) {
    size_t len = strlen(line);

    // Minimum frame: "<AA55PPLL55AA>" = 14 chars + newline
    if (len < 14) {
        return false;
    }

    // Check start/end markers
    if (line[0] != '<' || line[len-1] != '>') {
        return false;
    }

    // Check start markers AA55
    if (line[1] != 'A' || line[2] != 'A' || line[3] != '5' || line[4] != '5') {
        return false;
    }

    // Parse PID (2 hex chars)
    char pid_hex[3] = {line[5], line[6], '\0'};
    *pid = strtol(pid_hex, NULL, 16);

    // Parse length (2 hex chars)
    char len_hex[3] = {line[7], line[8], '\0'};
    uint8_t data_len = strtol(len_hex, NULL, 16);
    *payload_len = data_len;

    // Calculate expected frame length
    // <AA55PPLL[DATA_HEX]CCKK55AA>
    // 1 + 4 + 2 + 2 + (data_len*2) + 2 + 4 + 1 = 16 + (data_len*2)
    size_t expected = 16 + (data_len * 2);
    if (len < expected) {
        return false;
    }

    // Parse payload (data_len * 2 hex chars)
    for (uint8_t i = 0; i < data_len; i++) {
        char byte_hex[3] = {line[9 + i*2], line[9 + i*2 + 1], '\0'};
        payload[i] = strtol(byte_hex, NULL, 16);
    }

    // Parse checksum (2 hex chars)
    char chk_hex[3] = {line[9 + data_len*2], line[9 + data_len*2 + 1], '\0'};
    uint8_t rx_checksum = strtol(chk_hex, NULL, 16);

    // Verify checksum
    uint8_t calc_checksum = calculateHybridChecksum(*pid, data_len, payload);
    if (rx_checksum != calc_checksum) {
        return false;
    }

    // Check end markers 55AA
    size_t end_pos = 9 + data_len*2 + 2;
    if (line[end_pos] != '5' || line[end_pos+1] != '5' ||
        line[end_pos+2] != 'A' || line[end_pos+3] != 'A') {
        return false;
    }

    // Extract command (first byte of payload for requests)
    if (data_len > 0) {
        *cmd = payload[0];
    } else {
        *cmd = 0;
    }

    return true;
}

void handleHybridRequest(uint8_t peripheral_id, uint8_t command, const uint8_t* payload, size_t payload_len) {
    // Handle requests based on peripheral ID and command

    uint8_t response[256];
    size_t response_len = 0;

    if (g_dataCollector == nullptr) {
        // No data collector available - send error
        uint8_t error_msg[] = "NO_DATA_COLLECTOR";
        sendHybridFrame(peripheral_id, error_msg, sizeof(error_msg) - 1);
        return;
    }

    // Handle different peripherals
    switch (peripheral_id) {
        case 0x00: {  // SYSTEM
            if (command == 0x00) {  // GET_STATUS
                response_len = g_dataCollector->packStatus(response, sizeof(response));
            } else if (command == 0x01) {  // HEARTBEAT
                response_len = g_dataCollector->packHeartbeat(response, sizeof(response));
            }
            break;
        }

        case 0x01: {  // LORA_915
            if (command == 0x00) {  // GET_ALL
                response_len = g_dataCollector->packLoRaData(response, sizeof(response));
            }
            break;
        }

        case 0x02: {  // RADIO_433
            if (command == 0x00) {  // GET_ALL
                response_len = g_dataCollector->pack433Data(response, sizeof(response));
            }
            break;
        }

        case 0x03: {  // BAROMETER
            if (command == 0x00) {  // GET_ALL
                response_len = g_dataCollector->packBarometerData(response, sizeof(response));
            }
            break;
        }

        case 0x04: {  // CURRENT
            if (command == 0x00) {  // GET_ALL
                response_len = g_dataCollector->packCurrentData(response, sizeof(response));
            }
            break;
        }

        default:
            // Unknown peripheral
            uint8_t error[] = "UNKNOWN_PERIPHERAL";
            sendHybridFrame(peripheral_id, error, sizeof(error) - 1);
            return;
    }

    // Send response if we have data
    if (response_len > 0) {
        sendHybridFrame(peripheral_id, response, response_len);
    } else {
        // No data or unknown command
        uint8_t error[] = "NO_DATA_OR_UNKNOWN_CMD";
        sendHybridFrame(peripheral_id, error, sizeof(error) - 1);
    }
}

void processHybridProtocol() {
    Stream* port = g_serial_port ? g_serial_port : &Serial;

    if (!port->available()) {
        return;
    }

    // Read line NON-BLOCKING - only process if complete frame available
    static char line[HYBRID_MAX_FRAME_SIZE];
    static size_t idx = 0;
    static unsigned long read_start = 0;

    // Start new frame
    if (idx == 0) {
        read_start = millis();
    }

    // Read available characters (non-blocking)
    while (port->available() && idx < HYBRID_MAX_FRAME_SIZE - 1) {
        char c = port->read();

        if (c == '\n' || c == '\r') {
            // Complete frame received!
            line[idx] = '\0';

            if (idx > 0) {
                // Update activity time
                g_last_activity_time = millis();

                // Parse frame
                uint8_t pid, cmd;
                uint8_t payload[HYBRID_MAX_PAYLOAD_SIZE];
                size_t payload_len;

                if (parseHybridFrame(line, &pid, &cmd, payload, &payload_len)) {
                    // Handle request
                    handleHybridRequest(pid, cmd, payload, payload_len);
                }
            }

            // Reset for next frame
            idx = 0;
            return;
        }

        line[idx++] = c;
    }

    // Timeout check - abandon partial frame after 1 second
    if (idx > 0 && millis() - read_start > 1000) {
        idx = 0;  // Abandon partial frame
    }
}

void runHybridProtocolMode() {
    setupHybridProtocol();

    // Initialize DataCollector
    g_dataCollector = new DataCollector();
    g_dataCollector->begin();
    g_serial_port = &Serial;

    // Main loop
    while (true) {
        processHybridProtocol();
        delay(1);  // Small delay to prevent tight loop
    }
}

// ====================================================================
// INTEGRATED MODE FUNCTIONS (for full firmware with FreeRTOS tasks)
// ====================================================================

void initHybridProtocol(DataCollector* dc, Stream* port, SemaphoreHandle_t* mutex) {
    g_dataCollector = dc;
    g_serial_port = port;
    g_serial_mutex = mutex;
    g_last_activity_time = millis();
}

// processHybridProtocol() already exists above - it works for both modes!

void sendHybridPeripheralData(uint8_t peripheral_id) {
    // Called by autonomous polling system
    uint8_t response[256];
    size_t response_len = 0;

    switch (peripheral_id) {
        case 0x00:
            response_len = g_dataCollector->packStatus(response, sizeof(response));
            break;
        case 0x01:
            response_len = g_dataCollector->packLoRaData(response, sizeof(response));
            break;
        case 0x02:
            response_len = g_dataCollector->pack433Data(response, sizeof(response));
            break;
        case 0x03:
            response_len = g_dataCollector->packBarometerData(response, sizeof(response));
            break;
        case 0x04:
            response_len = g_dataCollector->packCurrentData(response, sizeof(response));
            break;
    }

    if (response_len > 0) {
        sendHybridFrame(peripheral_id, response, response_len);
    }
}

#endif // HYBRID_PROTOCOL_MODE
