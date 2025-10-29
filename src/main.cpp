/**
 * @file main.cpp
 * @brief ESP32 Single-Threaded Firmware with Hybrid Protocol
 *
 * Architecture:
 * - Priority-based event loop (no FreeRTOS tasks)
 * - Command queue for Pi requests
 * - Non-blocking peripheral polling
 * - Hybrid protocol (binary structure + newline framing)
 *
 * Loop Priority:
 * 1. Read and queue incoming Pi commands (non-blocking)
 * 2. Execute ONE queued command
 * 3. Update peripheral sensors on schedule
 * 4. Send autonomous data if enabled
 */

#include <Arduino.h>
#include "config.h"
#include "DataCollector.h"

// Optionally keep FlightDataReplay for testing
#ifdef TEST_MODE
// #include "FlightDataReplay.h"  // Commented out for now, can re-enable later
#endif

// ====================================================================
// PROTOCOL CONSTANTS
// ====================================================================
// Note: Peripheral IDs and commands are defined in config.h

// Frame format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
#define HYBRID_MAX_FRAME_SIZE 512
#define HYBRID_MAX_PAYLOAD_SIZE 128

// Frame markers
#define HYBRID_FRAME_START '<'
#define HYBRID_FRAME_END '>'

// Response codes
#define RESP_ACK            0x01
#define RESP_ERROR          0xFF

// Error codes
#define ERROR_INVALID_FRAME 0x01
#define ERROR_QUEUE_FULL    0x02
#define ERROR_INVALID_CMD   0x03
#define ERROR_INVALID_PID   0x04

// ====================================================================
// COMMAND QUEUE
// ====================================================================

#define MAX_QUEUE_SIZE 8

struct Command {
    uint8_t peripheralId;
    uint8_t command;
    uint8_t payload[16];
    uint8_t payloadLen;
};

class CommandQueue {
private:
    Command queue[MAX_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;

public:
    CommandQueue() : head(0), tail(0), count(0) {}

    bool enqueue(const Command& cmd) {
        if (count >= MAX_QUEUE_SIZE) {
            return false;
        }
        queue[head] = cmd;
        head = (head + 1) % MAX_QUEUE_SIZE;
        count++;
        return true;
    }

    bool dequeue(Command* cmd) {
        if (count == 0) {
            return false;
        }
        *cmd = queue[tail];
        tail = (tail + 1) % MAX_QUEUE_SIZE;
        count--;
        return true;
    }

    bool isEmpty() const { return count == 0; }
    bool isFull() const { return count >= MAX_QUEUE_SIZE; }
    uint8_t size() const { return count; }
};

// ====================================================================
// PERIPHERAL POLLING SCHEDULE
// ====================================================================

struct PeripheralPoller {
    uint8_t peripheralId;
    uint32_t lastPollTime;        // Last time sensor was polled
    uint32_t lastSendTime;        // Last time data was sent (for autonomous)
    uint32_t pollInterval;        // milliseconds
    bool enabled;
    bool autonomousSendEnabled;   // Send data automatically
};

#define MAX_POLLERS 5

PeripheralPoller pollers[MAX_POLLERS] = {
    {PERIPHERAL_ID_LORA_915,  0, 0, 100,  true, false},  // LoRa 915MHz: poll every 100ms
    {PERIPHERAL_ID_RADIO_433, 0, 0, 100,  true, false},  // 433MHz: poll every 100ms
    {PERIPHERAL_ID_BAROMETER, 0, 0, 500,  true, false},  // Barometer: poll every 500ms
    {PERIPHERAL_ID_CURRENT,   0, 0, 1000, true, false},  // Current: poll every 1000ms
    {PERIPHERAL_ID_SYSTEM,    0, 0, 5000, true, false}   // System: poll every 5000ms
};

// Special poller for PID_ALL (sends all peripheral data)
PeripheralPoller allPeripheralsPoller = {PERIPHERAL_ID_ALL, 0, 0, 1000, false, false};

// ====================================================================
// GLOBAL INSTANCES
// ====================================================================

DataCollector dataCollector;
CommandQueue commandQueue;

// ====================================================================
// LOOP PERFORMANCE TRACKING
// ====================================================================

struct LoopStats {
    uint32_t loopCount;
    uint32_t lastReportTime;
    uint32_t lastLoopTime;
    float loopsPerSecond;
    float avgLoopTimeMs;
};

LoopStats loopStats = {0, 0, 0, 0.0f, 0.0f};

// Serial receive buffer (static for state machine)
static char rxFrameBuffer[HYBRID_MAX_FRAME_SIZE];
static size_t rxFrameIndex = 0;
static uint32_t rxFrameStartTime = 0;

// Activity tracking
uint32_t lastActivityTime = 0;
uint32_t bootTime = 0;

// Statistics
uint32_t totalCommandsReceived = 0;
uint32_t totalCommandsExecuted = 0;
uint32_t totalFrameErrors = 0;

// ====================================================================
// FUNCTION PROTOTYPES
// ====================================================================

// Main loop functions
void readIncomingCommands();
void executeQueuedCommand();
void updatePeripherals();
void sendAutonomousData();

// Protocol functions
uint8_t calculateChecksum(uint8_t pid, uint8_t len, const uint8_t* payload);
void sendHybridFrame(uint8_t pid, const uint8_t* payload, size_t len);
bool parseHybridFrame(const char* frame, uint8_t* pid, uint8_t* cmd, uint8_t* payload, uint8_t* payloadLen);

// Command handlers
void handleCommand(const Command& cmd);
void handleGetAllPeripherals();
void sendPeripheralData(uint8_t pid);
void sendAckResponse(uint8_t pid);
void sendErrorResponse(uint8_t pid, uint8_t errorCode);
void sendSystemStatus();

// Polling control
void enablePolling(uint8_t pid, uint16_t interval);
void disablePolling(uint8_t pid);
void enableAutonomousSend(uint8_t pid, uint16_t interval);
void disableAutonomousSend(uint8_t pid);
PeripheralPoller* findPoller(uint8_t pid);

// ====================================================================
// SETUP
// ====================================================================

void setup() {
    // Initialize serial
    Serial.begin(115200);
    Serial.setRxBufferSize(1024);
    while (!Serial && millis() < 3000) delay(10);
    delay(500);

    // Print banner
    Serial.println("\n====================================");
    Serial.println("  ESP32 HYBRID PROTOCOL MODE");
    Serial.println("  Single-Threaded Event Loop");
    Serial.println("====================================");
    Serial.println();
    Serial.println("Format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\\n");
    Serial.println();

    // Initialize data collector (sensors)
    dataCollector.begin();

    // Record boot time
    bootTime = millis();
    lastActivityTime = millis();

    Serial.println("Ready for commands!\n");
}

// ====================================================================
// MAIN EVENT LOOP
// ====================================================================

void loop() {
    uint32_t loopStartTime = micros();  // Track loop start for performance metrics

    // PRIORITY 1: Read and queue incoming Pi commands (non-blocking)
    readIncomingCommands();

    // PRIORITY 2: Execute ONE queued command (send response)
    executeQueuedCommand();

    // PRIORITY 3: Update peripheral sensors on schedule
    updatePeripherals();

    // PRIORITY 4: Send autonomous data if enabled
    sendAutonomousData();

    // PRIORITY 5: Report loop performance stats (twice per second) - DISABLED FOR NOW
    // loopStats.loopCount++;
    // uint32_t now = millis();
    // if (now - loopStats.lastReportTime >= 500) {  // Report every 500ms (2x per second)
    //     uint32_t elapsed = now - loopStats.lastReportTime;
    //     loopStats.loopsPerSecond = (loopStats.loopCount * 1000.0f) / elapsed;
    //     loopStats.avgLoopTimeMs = elapsed / (float)loopStats.loopCount;

    //     // Send performance stats as a special message
    //     Serial.printf("[PERF] %.1f loops/sec, %.3f ms/loop, Queue: %d/8\n",
    //         loopStats.loopsPerSecond, loopStats.avgLoopTimeMs, commandQueue.size());

    //     loopStats.loopCount = 0;
    //     loopStats.lastReportTime = now;
    // }

    // Very small delay to prevent watchdog issues but maximize responsiveness
    // Note: No delay for maximum speed - ESP32 yield() is called automatically
    yield();  // Let ESP32 handle background tasks (WiFi, etc.)
}

// ====================================================================
// PRIORITY 1: READ INCOMING COMMANDS (NON-BLOCKING)
// ====================================================================

void readIncomingCommands() {
    // Exit immediately if no data available
    if (!Serial.available()) {
        return;
    }

    // Start timeout tracking for new frame
    if (rxFrameIndex == 0) {
        rxFrameStartTime = millis();
    }

    // Read available characters (non-blocking)
    while (Serial.available() && rxFrameIndex < HYBRID_MAX_FRAME_SIZE - 1) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            // Complete frame received!
            if (rxFrameIndex > 0) {
                rxFrameBuffer[rxFrameIndex] = '\0';

                // Parse and queue command
                uint8_t pid, cmd;
                uint8_t payload[HYBRID_MAX_PAYLOAD_SIZE];
                uint8_t payloadLen;

                if (parseHybridFrame(rxFrameBuffer, &pid, &cmd, payload, &payloadLen)) {
                    // Valid frame - create command
                    Command newCmd;
                    newCmd.peripheralId = pid;
                    newCmd.command = cmd;
                    newCmd.payloadLen = min(payloadLen, (uint8_t)sizeof(newCmd.payload));
                    memcpy(newCmd.payload, payload, newCmd.payloadLen);

                    // Queue it (don't execute yet!)
                    if (commandQueue.enqueue(newCmd)) {
                        totalCommandsReceived++;
                        lastActivityTime = millis();
                    } else {
                        // Queue full - send error immediately
                        sendErrorResponse(pid, ERROR_QUEUE_FULL);
                    }
                } else {
                    // Invalid frame
                    totalFrameErrors++;
                    // Could send error response here if we could parse PID
                }

                rxFrameIndex = 0;
            }
            return;
        }

        rxFrameBuffer[rxFrameIndex++] = c;
    }

    // Timeout check - abandon partial frame after 1 second
    if (rxFrameIndex > 0 && millis() - rxFrameStartTime > 1000) {
        rxFrameIndex = 0;
    }
}

// ====================================================================
// PRIORITY 2: EXECUTE ONE QUEUED COMMAND
// ====================================================================

void executeQueuedCommand() {
    // Execute up to 3 commands per loop iteration if queue is getting full
    // This prevents queue overflow during burst requests
    uint8_t maxExecutions = commandQueue.size() > 5 ? 3 : 1;

    for (uint8_t i = 0; i < maxExecutions; i++) {
        Command cmd;

        // Dequeue one command
        if (!commandQueue.dequeue(&cmd)) {
            return;  // No more commands pending
        }

        // Execute it
        handleCommand(cmd);
        totalCommandsExecuted++;
    }
}

void handleCommand(const Command& cmd) {
    switch (cmd.command) {
        case CMD_GET_ALL:
            // Check if this is a request for ALL peripherals
            if (cmd.peripheralId == PERIPHERAL_ID_ALL) {
                handleGetAllPeripherals();
            } else {
                // Send single peripheral data
                sendPeripheralData(cmd.peripheralId);
            }
            break;

        case CMD_GET_STATUS:
            // Get status/health of this peripheral
            if (cmd.peripheralId == PERIPHERAL_ID_SYSTEM) {
                sendSystemStatus();
            } else {
                sendPeripheralData(cmd.peripheralId);
            }
            break;

        case CMD_SET_POLL_RATE:
            // Enable autonomous polling and sending
            // Note: payload[0] is the command byte, actual data starts at payload[1]
            if (cmd.payloadLen >= 3) {  // Need command + 2 bytes for interval
                uint16_t interval = cmd.payload[1] | (cmd.payload[2] << 8);

                // Rate limiting: minimum intervals to prevent overload
                const uint16_t MIN_INTERVAL_SINGLE = 30;   // 30ms for single peripheral
                const uint16_t MIN_INTERVAL_ALL = 100;      // 100ms for PID_ALL

                uint16_t minInterval = (cmd.peripheralId == PERIPHERAL_ID_ALL) ? MIN_INTERVAL_ALL : MIN_INTERVAL_SINGLE;

                if (interval < minInterval) {
                    Serial.printf("[RATE_LIMIT] PID=0x%02X interval=%ums < min=%ums - REJECTED\n",
                        cmd.peripheralId, interval, minInterval);
                    sendErrorResponse(cmd.peripheralId, ERROR_INVALID_CMD);
                } else {
                    enableAutonomousSend(cmd.peripheralId, interval);
                    sendAckResponse(cmd.peripheralId);
                }
            } else {
                sendErrorResponse(cmd.peripheralId, ERROR_INVALID_CMD);
            }
            break;

        case CMD_STOP_POLL:
            // Disable autonomous sending
            disableAutonomousSend(cmd.peripheralId);
            sendAckResponse(cmd.peripheralId);
            break;

        case CMD_SYSTEM_STATUS:
        case CMD_SYSTEM_WAKEUP:
            // Send system status (system commands)
            sendSystemStatus();
            break;

        default:
            // Unknown command
            sendErrorResponse(cmd.peripheralId, ERROR_INVALID_CMD);
            break;
    }
}

void handleGetAllPeripherals() {
    // Handle PID_ALL request - queue responses for each peripheral
    // This sends data for all peripherals, one per loop iteration

    // Rate limit: max one PID_ALL request per 100ms
    static uint32_t lastPidAllTime = 0;
    uint32_t now = millis();

    if (now - lastPidAllTime < 100) {
        sendErrorResponse(PERIPHERAL_ID_ALL, ERROR_INVALID_CMD);
        return;
    }
    lastPidAllTime = now;

    // Check if queue has room for 4 responses (one for each peripheral)
    if (commandQueue.size() + 4 > MAX_QUEUE_SIZE) {
        sendErrorResponse(PERIPHERAL_ID_ALL, ERROR_QUEUE_FULL);
        return;
    }

    // Queue internal response commands for each peripheral
    Command responseCmd;
    responseCmd.command = CMD_GET_ALL;
    responseCmd.payloadLen = 0;

    // Queue LoRa 915MHz response
    responseCmd.peripheralId = PERIPHERAL_ID_LORA_915;
    commandQueue.enqueue(responseCmd);

    // Queue 433MHz response
    responseCmd.peripheralId = PERIPHERAL_ID_RADIO_433;
    commandQueue.enqueue(responseCmd);

    // Queue Barometer response
    responseCmd.peripheralId = PERIPHERAL_ID_BAROMETER;
    commandQueue.enqueue(responseCmd);

    // Queue Current sensor response
    responseCmd.peripheralId = PERIPHERAL_ID_CURRENT;
    commandQueue.enqueue(responseCmd);

    // Note: No ACK sent - Pi will receive 4 data responses instead
}

// ====================================================================
// PRIORITY 3: UPDATE PERIPHERALS (NON-BLOCKING)
// ====================================================================

void updatePeripherals() {
    uint32_t now = millis();

    // Check each peripheral's polling schedule
    for (int i = 0; i < MAX_POLLERS; i++) {
        if (!pollers[i].enabled) {
            continue;
        }

        if (now - pollers[i].lastPollTime >= pollers[i].pollInterval) {
            // Time to update this peripheral
            switch (pollers[i].peripheralId) {
                case PERIPHERAL_ID_LORA_915:
                case PERIPHERAL_ID_RADIO_433:
                    // Non-blocking: check if radio data available
                    dataCollector.pollRadios();
                    break;

                case PERIPHERAL_ID_BAROMETER:
                case PERIPHERAL_ID_CURRENT:
                    // I2C read (~5ms blocking, acceptable)
                    dataCollector.pollSensors();
                    break;

                case PERIPHERAL_ID_SYSTEM:
                    // Update system stats (fast)
                    dataCollector.updateSystemStatus();
                    break;
            }

            pollers[i].lastPollTime = now;
        }
    }
}

// ====================================================================
// PRIORITY 4: SEND AUTONOMOUS DATA
// ====================================================================

void sendAutonomousData() {
    uint32_t now = millis();

    // Check PID_ALL first (special handling)
    if (allPeripheralsPoller.autonomousSendEnabled) {
        if (now - allPeripheralsPoller.lastSendTime >= allPeripheralsPoller.pollInterval) {
            // Send all peripherals by calling handleGetAllPeripherals
            Serial.printf("[AUTO] Sending PID_ALL batch\n");
            handleGetAllPeripherals();
            allPeripheralsPoller.lastSendTime = now;
            return;  // Send only one batch per loop iteration
        }
    }

    // Check each peripheral for autonomous sending
    // Send at most ONE peripheral per loop iteration to avoid blocking
    for (int i = 0; i < MAX_POLLERS; i++) {
        if (!pollers[i].autonomousSendEnabled) {
            continue;
        }

        // Check if it's time to send (use separate lastSendTime, not lastPollTime)
        if (now - pollers[i].lastSendTime >= pollers[i].pollInterval) {
            sendPeripheralData(pollers[i].peripheralId);
            pollers[i].lastSendTime = now;  // Update send timestamp
            return;  // Send only one per loop iteration
        }
    }
}

// ====================================================================
// PROTOCOL FUNCTIONS
// ====================================================================

uint8_t calculateChecksum(uint8_t pid, uint8_t len, const uint8_t* payload) {
    uint8_t chk = pid ^ len;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= payload[i];
    }
    return chk;
}

void sendHybridFrame(uint8_t pid, const uint8_t* payload, size_t len) {
    if (len > HYBRID_MAX_PAYLOAD_SIZE) {
        return;  // Payload too large
    }

    uint8_t chk = calculateChecksum(pid, len, payload);

    // Build frame: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
    Serial.print('<');
    Serial.print("AA55");
    Serial.printf("%02X", pid);
    Serial.printf("%02X", (uint8_t)len);

    // Payload as hex
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X", payload[i]);
    }

    Serial.printf("%02X", chk);
    Serial.print("55AA>\n");
}

bool parseHybridFrame(const char* frame, uint8_t* pid, uint8_t* cmd,
                      uint8_t* payload, uint8_t* payloadLen) {
    // Expected format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>

    // Check frame markers
    if (frame[0] != '<') {
        return false;
    }

    size_t frameLen = strlen(frame);
    if (frameLen < 12 || frame[frameLen - 1] != '>') {
        return false;  // Too short or missing '>'
    }

    // Check start markers: AA55
    if (strncmp(frame + 1, "AA55", 4) != 0) {
        return false;
    }

    // Parse PID (2 hex chars)
    char pidStr[3] = {frame[5], frame[6], '\0'};
    *pid = (uint8_t)strtol(pidStr, nullptr, 16);

    // Parse length (2 hex chars)
    char lenStr[3] = {frame[7], frame[8], '\0'};
    uint8_t len = (uint8_t)strtol(lenStr, nullptr, 16);

    // Check frame length matches
    size_t expectedLen = 1 + 4 + 2 + 2 + (len * 2) + 2 + 4 + 1;  // <AA55PPLL[data]CHKSUM55AA>
    if (frameLen < expectedLen) {
        return false;
    }

    // Parse payload (each byte is 2 hex chars)
    for (uint8_t i = 0; i < len; i++) {
        char byteStr[3] = {frame[9 + i * 2], frame[10 + i * 2], '\0'};
        payload[i] = (uint8_t)strtol(byteStr, nullptr, 16);
    }
    *payloadLen = len;

    // Extract command (first byte of payload)
    if (len > 0) {
        *cmd = payload[0];
    } else {
        *cmd = 0;
    }

    // Parse checksum (2 hex chars)
    size_t chkPos = 9 + len * 2;
    char chkStr[3] = {frame[chkPos], frame[chkPos + 1], '\0'};
    uint8_t rxChk = (uint8_t)strtol(chkStr, nullptr, 16);

    // Verify checksum
    uint8_t calcChk = calculateChecksum(*pid, len, payload);
    if (rxChk != calcChk) {
        return false;
    }

    // Check end markers: 55AA
    if (strncmp(frame + chkPos + 2, "55AA", 4) != 0) {
        return false;
    }

    return true;
}

// ====================================================================
// RESPONSE FUNCTIONS
// ====================================================================

void sendPeripheralData(uint8_t pid) {
    uint8_t buffer[HYBRID_MAX_PAYLOAD_SIZE];
    size_t len = 0;

    switch (pid) {
        case PERIPHERAL_ID_LORA_915:
            len = dataCollector.packLoRaData(buffer, HYBRID_MAX_PAYLOAD_SIZE);
            break;

        case PERIPHERAL_ID_RADIO_433:
            len = dataCollector.pack433Data(buffer, HYBRID_MAX_PAYLOAD_SIZE);
            break;

        case PERIPHERAL_ID_BAROMETER:
            len = dataCollector.packBarometerData(buffer, HYBRID_MAX_PAYLOAD_SIZE);
            break;

        case PERIPHERAL_ID_CURRENT:
            len = dataCollector.packCurrentData(buffer, HYBRID_MAX_PAYLOAD_SIZE);
            break;

        case PERIPHERAL_ID_SYSTEM:
            sendSystemStatus();
            return;

        default:
            sendErrorResponse(pid, ERROR_INVALID_PID);
            return;
    }

    if (len > 0) {
        sendHybridFrame(pid, buffer, len);
    } else {
        sendErrorResponse(pid, ERROR_INVALID_PID);
    }
}

void sendAckResponse(uint8_t pid) {
    uint8_t ackPayload[1] = {RESP_ACK};
    sendHybridFrame(pid, ackPayload, 1);
}

void sendErrorResponse(uint8_t pid, uint8_t errorCode) {
    uint8_t errorPayload[2] = {RESP_ERROR, errorCode};
    sendHybridFrame(pid, errorPayload, 2);
}

void sendSystemStatus() {
    // Use DataCollector's packStatus method to get proper WireStatus_t format (20 bytes)
    uint8_t buffer[HYBRID_MAX_PAYLOAD_SIZE];
    size_t len = dataCollector.packStatus(buffer, HYBRID_MAX_PAYLOAD_SIZE);

    if (len > 0) {
        sendHybridFrame(PERIPHERAL_ID_SYSTEM, buffer, len);
    } else {
        // Fallback: send error
        sendErrorResponse(PERIPHERAL_ID_SYSTEM, ERROR_INVALID_PID);
    }
}

// ====================================================================
// POLLING CONTROL FUNCTIONS
// ====================================================================

PeripheralPoller* findPoller(uint8_t pid) {
    for (int i = 0; i < MAX_POLLERS; i++) {
        if (pollers[i].peripheralId == pid) {
            return &pollers[i];
        }
    }
    return nullptr;
}

void enablePolling(uint8_t pid, uint16_t interval) {
    PeripheralPoller* poller = findPoller(pid);
    if (poller) {
        poller->enabled = true;
        if (interval > 0) {
            poller->pollInterval = interval;
        }
    }
}

void disablePolling(uint8_t pid) {
    PeripheralPoller* poller = findPoller(pid);
    if (poller) {
        poller->enabled = false;
    }
}

void enableAutonomousSend(uint8_t pid, uint16_t interval) {
    // Special handling for PID_ALL
    if (pid == PERIPHERAL_ID_ALL) {
        allPeripheralsPoller.enabled = true;
        allPeripheralsPoller.autonomousSendEnabled = true;
        allPeripheralsPoller.lastSendTime = millis();
        if (interval > 0) {
            allPeripheralsPoller.pollInterval = interval;
        }
        Serial.printf("[AUTO] PID=0x%02X (ALL) interval=%ums\n", pid, allPeripheralsPoller.pollInterval);
        return;
    }

    // Normal peripheral handling
    PeripheralPoller* poller = findPoller(pid);
    if (poller) {
        poller->enabled = true;
        poller->autonomousSendEnabled = true;
        poller->lastSendTime = millis();  // Initialize send timer
        if (interval > 0) {
            poller->pollInterval = interval;
        }
        // Debug: Show what was configured
        Serial.printf("[AUTO] PID=0x%02X interval=%ums\n", pid, poller->pollInterval);
    }
}

void disableAutonomousSend(uint8_t pid) {
    // Special handling for PID_ALL
    if (pid == PERIPHERAL_ID_ALL) {
        allPeripheralsPoller.autonomousSendEnabled = false;
        return;
    }

    // Normal peripheral handling
    PeripheralPoller* poller = findPoller(pid);
    if (poller) {
        poller->autonomousSendEnabled = false;
    }
}
