# Hybrid Protocol STANDALONE Mode Restored ✅

## What Was Done

I've restored the hybrid protocol to its **STANDALONE** version - the one that was getting excellent throughput (~61 tests/second) **BEFORE** FreeRTOS integration.

### Files Restored/Created

1. **src/hybrid_protocol.cpp** (YOU had this cached!)
   - Contains `runHybridProtocolMode()` function
   - Simple loop: `while(true) { processHybridProtocol(); delay(1); }`
   - NO FreeRTOS tasks
   - NO threading
   - NO mutexes (unless you integrate it later)

2. **src/hybrid_protocol.h** (CREATED)
   - Header file defining the API
   - Just declares `runHybridProtocolMode()`

3. **src/config.h** (MODIFIED)
   - Added `#define HYBRID_PROTOCOL_MODE` at line 7
   - When enabled, runs standalone hybrid protocol
   - When disabled, runs full firmware with FreeRTOS

4. **src/main.cpp** (MODIFIED)
   - setup() now calls `runHybridProtocolMode()` when HYBRID_PROTOCOL_MODE is enabled
   - This never returns - hybrid protocol runs in its own infinite loop
   - All FreeRTOS code is bypassed

---

## Current Configuration

### Enabled: HYBRID_PROTOCOL_MODE

Your firmware is now configured to run **STANDALONE HYBRID PROTOCOL** - the simple, fast, reliable version that was working great!

**What this means:**
- ✅ NO FreeRTOS tasks
- ✅ NO threading complications
- ✅ Simple request/response loop
- ✅ Non-blocking `processHybridProtocol()` with static state
- ✅ Fast throughput (~61 tests/second expected)
- ✅ TEST_MODE compatible (fake sensors via DataCollector)

**How it works:**
```cpp
void setup() {
    runHybridProtocolMode();  // Calls this and never returns
}

void runHybridProtocolMode() {
    setupHybridProtocol();  // Init Serial, print banner

    g_dataCollector = new DataCollector();
    g_dataCollector->begin();  // Initialize fake sensors (TEST_MODE)
    g_serial_port = &Serial;

    while (true) {
        processHybridProtocol();  // Process one frame if available
        delay(1);  // Small delay
    }
}
```

---

## Protocol Format

Wire format: `<AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n`

**Example Request (Get LoRa data):**
```
<AA5501000155AA>\n
 └─ '<' frame start
    AA55 - start markers (binary 0xAA 0x55 as hex)
    01 - PID (LORA_915)
    00 - Length (0 bytes payload)
    01 - Checksum (0x01 ^ 0x00 = 0x01)
    55AA - end markers (binary 0x55 0xAA as hex)
    '>' - frame end
    '\n' - newline terminator
```

**Example Response (LoRa data, 74 bytes):**
```
<AA55014A[148 hex chars for 74 bytes]CHKSUM55AA>\n
 └─ 0x01 = LORA_915
    0x4A = 74 bytes
    [74 bytes WireLoRa_t data structure as hex]
    CHKSUM = XOR checksum
```

---

## Testing Instructions

### Step 1: Build and Upload

Since PlatformIO isn't available in this terminal, you'll need to:

1. **Open Arduino IDE or PlatformIO**
2. **Verify compilation**:
   - Should compile cleanly with HYBRID_PROTOCOL_MODE defined
   - No FreeRTOS code will be compiled
3. **Upload to ESP32**

### Step 2: Run Python Tester

```bash
cd test
python hybrid_protocol_tester.py
```

**Expected Output:**
- Connection to ESP32 on COM port
- Continuous stress test with random peripheral requests
- ~50-60 tests/second throughput
- High success rate (98%+ base, 99.9%+ with retries)
- Consistent performance (no hangs or stalls)

### Step 3: Monitor Results

The tester will show:
```
[5s] Tests: 305 | First-try: 98.36% | Recovered: 5 | Hard Fail: 0 | Effective: 100.00% | Rate: 61.0/s
```

This is what you had before - **fast, reliable, simple**.

---

## Why This Version Works

### No Threading Complexity
- Single loop in `runHybridProtocolMode()`
- No task priorities to manage
- No mutex contention
- No task starvation

### Non-Blocking Protocol Processing
```cpp
void processHybridProtocol() {
    // Static variables maintain state across calls
    static char line[HYBRID_MAX_FRAME_SIZE];
    static size_t idx = 0;
    static unsigned long read_start = 0;

    // Only read available characters (non-blocking)
    while (port->available() && idx < HYBRID_MAX_FRAME_SIZE - 1) {
        char c = port->read();
        // Accumulate until newline...
    }

    // Process complete frames immediately
    // Abandon partial frames after 1 second
}
```

### Newline-Based Framing
- Each frame ends with `\n`
- Serial uses buffered `readline()` semantics
- Atomic frame boundaries
- No byte stuffing needed
- No desynchronization

### ASCII Hex Encoding
- Debuggable (can see frames in serial monitor)
- No special characters in payload
- Slightly larger (2x size) but more reliable
- Checksum catches transmission errors

---

## Switching Between Modes

### To Run Standalone Hybrid Protocol (CURRENT):
```cpp
// In src/config.h:
#define HYBRID_PROTOCOL_MODE  // ← Enabled
```

### To Run Full Firmware with FreeRTOS:
```cpp
// In src/config.h:
// #define HYBRID_PROTOCOL_MODE  // ← Commented out
```

When HYBRID_PROTOCOL_MODE is disabled, the firmware will:
- Include all FreeRTOS task code
- Use CommProtocol (byte-stuffed protocol)
- Initialize all hardware peripherals
- Run communicationTask, dataCollectionTask, mainLoopTask

---

## What's Different From Before?

### BEFORE (Failed Integration):
- Hybrid protocol running INSIDE communicationTask (FreeRTOS)
- High priority task (priority 3)
- Mutex protection for Serial writes
- Autonomous polling from other tasks
- Threading complications causing inconsistencies

### NOW (Working Standalone):
- Hybrid protocol running in simple loop (NO FreeRTOS)
- Just one execution context
- No threading
- No mutexes (unless needed later)
- No autonomous polling complications
- **SAME CODE** that was getting 61 tests/second!

---

## Next Steps

### Phase 1: Validate Standalone Works (NOW)
1. ✅ **DONE**: Restored standalone hybrid protocol
2. **TODO**: Build and upload firmware
3. **TODO**: Run `hybrid_protocol_tester.py`
4. **TODO**: Verify ~60 tests/second with high reliability

### Phase 2: Understand Why It Works
Once you confirm it works standalone:
- Document baseline performance metrics
- Understand what makes it reliable
- Identify why FreeRTOS integration failed

### Phase 3: Decide on Architecture
You have three options:

**Option A: Keep It Simple (Recommended for now)**
- Stay with standalone hybrid protocol
- Add non-blocking peripheral handlers as needed
- Only use threading when absolutely necessary (SPI conflicts)
- Advantage: Simple, reliable, easy to debug

**Option B: Integrate With FreeRTOS Carefully**
- Add hybrid protocol BACK into FreeRTOS
- Fix whatever caused the inconsistencies
- Use proper non-blocking I/O everywhere
- Test incrementally after each change

**Option C: Redesign Threading Architecture**
- Start from scratch with better task design
- Use queues for inter-task communication
- Separate I/O tasks from processing tasks
- More complex but potentially more robust

---

## Files Modified Summary

| File | Status | Changes |
|------|--------|---------|
| `src/hybrid_protocol.cpp` | ✅ Exists (you had cached) | Standalone implementation |
| `src/hybrid_protocol.h` | ✅ Created | Header file |
| `src/config.h` | ✅ Modified | Added HYBRID_PROTOCOL_MODE define |
| `src/main.cpp` | ✅ Modified | Calls runHybridProtocolMode() in setup() |
| `test/hybrid_protocol_tester.py` | ✅ Exists | Ready to test |

---

## Key Code Sections

### config.h (line 7):
```cpp
#define HYBRID_PROTOCOL_MODE  // Standalone hybrid protocol (NO FreeRTOS)
```

### main.cpp setup() (line 280-283):
```cpp
void setup() {
#ifdef HYBRID_PROTOCOL_MODE
    runHybridProtocolMode();  // Never returns
#else
    // Full firmware setup...
```

### hybrid_protocol.cpp runHybridProtocolMode() (line 303-316):
```cpp
void runHybridProtocolMode() {
    setupHybridProtocol();
    g_dataCollector = new DataCollector();
    g_dataCollector->begin();
    g_serial_port = &Serial;

    while (true) {
        processHybridProtocol();
        delay(1);
    }
}
```

---

## Troubleshooting

### If It Doesn't Compile:
- Check that `hybrid_protocol.h` exists
- Check that `config.h` has HYBRID_PROTOCOL_MODE defined
- Look for missing #endif or mismatched #ifdef blocks

### If Tests Are Slow:
- Check Serial baud rate (should be 115200)
- Check USB cable quality
- Check test script delay settings

### If Tests Fail:
- Check that ESP32 is in TEST_MODE (fake sensors)
- Check that DataCollector initializes properly
- Check Serial buffer size (should be 1024 bytes from line 28)
- Look at Serial Monitor to see if frames are being received

### If You Want to Go Back to Original Firmware:
Just comment out the define:
```cpp
// #define HYBRID_PROTOCOL_MODE
```

---

## Summary

✅ **Hybrid protocol STANDALONE mode restored**
✅ **NO FreeRTOS - just simple loop**
✅ **Same code that was getting 61 tests/second**
✅ **Ready to test!**

**Next Action**: Build, upload, and run `hybrid_protocol_tester.py` to verify it works as well as it did before!
