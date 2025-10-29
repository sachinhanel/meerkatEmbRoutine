# Codebase Cleanup and Simplification Plan

## Goal
Remove FreeRTOS complexity, remove old byte-stuffed protocol, simplify to single-threaded hybrid protocol with command queue.

---

## Files to KEEP (and simplify)

### Core Files
- ✅ **src/main.cpp** - Rewrite with single-threaded event loop
- ✅ **src/config.h** - Keep pin definitions, remove mode switches
- ✅ **src/DataCollector.cpp/h** - Keep as-is (works great for caching sensor data)
- ✅ **src/hybrid_protocol.cpp/h** - Simplify, integrate into main

### Sensor Files (if they exist)
- ✅ **src/sensors/Barometer.cpp/h** - Keep
- ✅ **src/sensors/Current.cpp/h** - Keep
- ✅ Keep any sensor abstraction code

### Test Files
- ✅ **test/hybrid_protocol_tester.py** - Keep (this is what we'll use!)
- ✅ **test/test_sender.py** - Keep for reference

---

## Files to DELETE (no longer needed)

### Old Protocol Files
- ❌ **src/CommProtocol.cpp/h** - DELETE (replaced by hybrid protocol)
- ❌ **src/minimal_protocol.cpp/h** - DELETE (was just for testing)
- ❌ **src/HybridCommProtocol.cpp/h** - DELETE (if exists, superseded by hybrid_protocol)

### FreeRTOS-Related
- ❌ **src/PollingManager.cpp/h** - DELETE (replaced by simpler polling in main loop)
- ❌ Any task-related code in main.cpp - DELETE

### Old Test Files
- ❌ **test/protocol_stuffed.py** - DELETE (testing old byte-stuffed protocol)
- ❌ **test/minimal_protocol_tester.py** - DELETE (testing old minimal protocol)

### Documentation for Removed Features
- ❌ **PROTOCOL_CHANGES_FOR_PI.md** - DELETE or UPDATE (old protocol docs)
- ❌ **REVERT_COMPLETE.md** - DELETE (no longer relevant)
- ❌ **THREADING_FIX_APPLIED.md** - DELETE (no longer using threading)
- ❌ **COMPILATION_FIXES.md** - DELETE (will be obsolete after cleanup)

---

## Code to REMOVE from main.cpp

### FreeRTOS Task System
```cpp
// DELETE all of this:
TaskHandle_t dataCollectionTaskHandle;
SemaphoreHandle_t loraBufferMutex;
SemaphoreHandle_t serialPortMutex;

void dataCollectionTask(void* parameters) { ... }  // DELETE
void communicationTask(void* parameters) { ... }   // DELETE
void mainLoopTask(void* parameters) { ... }        // DELETE

xTaskCreate(...)  // DELETE all task creation
xSemaphoreCreateMutex()  // DELETE all mutex creation
```

### Old Polling System
```cpp
// DELETE:
struct PollEntry { ... };
PollEntry pollList[MAX_POLL_ENTRIES];
void initPollList() { ... }
void addToPollList(...) { ... }
void removeFromPollList(...) { ... }
void checkAndSendPollingData() { ... }  // Replace with simpler version
```

### WiFi Telnet (TEST_MODE only, probably not needed)
```cpp
// DELETE if you don't use it:
WiFiServer telnetServer;
WiFiClient telnetClient;
void handleTelnetClients() { ... }
```

### Flight Data Replay (TEST_MODE, probably not needed)
```cpp
// DELETE if you don't use it:
#include "FlightDataReplay.h"
bool replayEnabled;
void handleFlightLogReplay() { ... }
```

---

## New Simplified Architecture

### src/main.cpp (New Structure)

```cpp
// ====================================================================
// ESP32 Single-Threaded Firmware with Hybrid Protocol
// ====================================================================

#include <Arduino.h>
#include "config.h"
#include "DataCollector.h"

// Protocol definitions
#define HYBRID_MAX_FRAME_SIZE 512
#define MAX_QUEUE_SIZE 8

// Command queue
struct Command {
    uint8_t peripheralId;
    uint8_t command;
    uint8_t payload[16];
    uint8_t payloadLen;
};

class CommandQueue {
    // ... (as shown above)
};

// Global instances
DataCollector dataCollector;
CommandQueue commandQueue;

// Peripheral polling schedule
struct PeripheralPoller {
    uint8_t peripheralId;
    uint32_t lastPollTime;
    uint32_t pollInterval;
    bool enabled;
    bool autonomousSendEnabled;  // Send data automatically
};

#define MAX_POLLERS 5
PeripheralPoller pollers[MAX_POLLERS] = {
    {0x01, 0, 100, true, false},   // LoRa
    {0x02, 0, 100, true, false},   // 433MHz
    {0x03, 0, 500, true, false},   // Barometer
    {0x04, 0, 1000, true, false},  // Current
    {0x00, 0, 5000, true, false}   // System status
};

// ====================================================================
// SETUP
// ====================================================================

void setup() {
    Serial.begin(115200);
    Serial.setRxBufferSize(1024);
    while (!Serial && millis() < 3000) delay(10);
    delay(500);

    Serial.println("\n====================================");
    Serial.println("  ESP32 HYBRID PROTOCOL MODE");
    Serial.println("  Single-Threaded Event Loop");
    Serial.println("====================================\n");

    dataCollector.begin();

    Serial.println("Ready!\n");
}

// ====================================================================
// MAIN EVENT LOOP
// ====================================================================

void loop() {
    // Priority 1: Read and queue incoming commands
    readIncomingCommands();

    // Priority 2: Execute one queued command
    executeQueuedCommand();

    // Priority 3: Update peripheral data
    updatePeripherals();

    // Priority 4: Send autonomous data
    sendAutonomousData();

    delay(1);
}

// ====================================================================
// IMPLEMENTATION FUNCTIONS
// ====================================================================

void readIncomingCommands() {
    // Non-blocking serial read
    // Parse frames
    // Queue commands (don't execute)
}

void executeQueuedCommand() {
    // Dequeue one command
    // Execute it
    // Send response
}

void updatePeripherals() {
    // Check polling schedule
    // Update sensors that are due
    // Non-blocking reads
}

void sendAutonomousData() {
    // Check if any peripheral has autonomous sending enabled
    // Send cached data if due
}

// Protocol helper functions
void parseHybridFrame(...) { ... }
void sendHybridFrame(...) { ... }
void sendPeripheralData(uint8_t pid) { ... }
void sendAckResponse(uint8_t pid) { ... }
void sendErrorResponse(uint8_t pid, uint8_t error) { ... }

// Polling control functions
void enablePolling(uint8_t pid, uint16_t interval) { ... }
void disablePolling(uint8_t pid) { ... }
```

### Key Simplifications

1. ✅ **No FreeRTOS tasks** - Just simple loop()
2. ✅ **No mutexes** - Single thread = no race conditions
3. ✅ **No complex polling system** - Simple array of pollers
4. ✅ **Command queue** - Handles concurrent requests gracefully
5. ✅ **One protocol** - Only hybrid protocol (remove byte-stuffed)
6. ✅ **DataCollector caching** - Sensors update on schedule, responses send cached data

---

## Hybrid Protocol Integration

Instead of separate `hybrid_protocol.cpp`, integrate it directly into main.cpp:

### Protocol Functions (move to main.cpp)

```cpp
uint8_t calculateChecksum(uint8_t pid, uint8_t len, const uint8_t* payload) {
    uint8_t chk = pid ^ len;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= payload[i];
    }
    return chk;
}

void sendHybridFrame(uint8_t pid, const uint8_t* payload, size_t len) {
    if (len > 128) return;  // Too large

    uint8_t chk = calculateChecksum(pid, len, payload);

    // Send: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
    Serial.print('<');
    Serial.print("AA55");
    Serial.printf("%02X", pid);
    Serial.printf("%02X", len);
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X", payload[i]);
    }
    Serial.printf("%02X", chk);
    Serial.print("55AA>\n");
}

bool parseHybridFrame(const char* frame, uint8_t* pid, uint8_t* cmd,
                      uint8_t* payload, uint8_t* payloadLen) {
    // Parse frame format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>
    // ... (implementation as in hybrid_protocol.cpp)
}
```

---

## Benefits of This Cleanup

### Before (Current State)
- 🔴 3 FreeRTOS tasks with priorities
- 🔴 5+ mutexes/semaphores
- 🔴 2-3 different protocol implementations
- 🔴 Complex polling manager
- 🔴 ~2000+ lines of code in main.cpp
- 🔴 Threading bugs and race conditions

### After (Cleaned Up)
- ✅ Single event loop (simple!)
- ✅ No mutexes needed
- ✅ One protocol (hybrid)
- ✅ Simple polling array
- ✅ ~500-800 lines of code in main.cpp
- ✅ No threading bugs possible!

---

## Migration Steps

### Phase 1: Cleanup (DELETE)
1. Remove FreeRTOS task code from main.cpp
2. Remove CommProtocol.cpp/h
3. Remove PollingManager.cpp/h
4. Remove minimal_protocol files
5. Remove old test files and docs

### Phase 2: Rewrite (CREATE NEW)
1. Create new main.cpp with single-threaded loop
2. Integrate hybrid protocol directly
3. Add command queue system
4. Add simple polling system
5. Keep DataCollector unchanged

### Phase 3: Test
1. Compile and upload
2. Run hybrid_protocol_tester.py
3. Verify ~60 tests/second performance
4. Test autonomous polling commands
5. Verify no hangs or crashes

---

## Estimated Effort

- ⏱️ **Cleanup**: 30 minutes (delete files, remove code blocks)
- ⏱️ **Rewrite**: 2-3 hours (new main.cpp structure)
- ⏱️ **Testing**: 1-2 hours (verify everything works)
- 🎯 **Total**: ~4-6 hours for complete overhaul

---

## Risk Mitigation

### Safety: Keep Backup
```bash
# Create backup branch before cleanup
git checkout -b backup-before-cleanup
git add .
git commit -m "Backup before single-threaded overhaul"

# Create new branch for cleanup
git checkout -b single-threaded-overhaul
```

### Incremental Testing
1. ✅ Remove files one at a time
2. ✅ Compile after each removal
3. ✅ Fix errors immediately
4. ✅ Test after major changes

---

## Summary

### What We're Building
- 🎯 Single-threaded event loop
- 🎯 Priority-based execution with command queue
- 🎯 Hybrid protocol only (remove old protocols)
- 🎯 Simple peripheral polling system
- 🎯 Clean, maintainable codebase (~500 lines)

### What We're Removing
- ❌ FreeRTOS tasks and mutexes
- ❌ Old byte-stuffed CommProtocol
- ❌ Complex PollingManager
- ❌ WiFi/Telnet (if not needed)
- ❌ Flight replay (if not needed)
- ❌ ~1500+ lines of unnecessary code

### Expected Results
- ✅ Simpler code (easier to understand)
- ✅ More reliable (no threading bugs)
- ✅ Better performance (no context switching)
- ✅ Easier to debug (linear execution)
- ✅ Maintainable (clear structure)

---

## Next Step

Ready to proceed? I can help you:

1. **Option A**: Start with cleanup (delete old files)
2. **Option B**: Create new main.cpp first (keep old as backup)
3. **Option C**: Walk through design more before coding

Which would you prefer?
