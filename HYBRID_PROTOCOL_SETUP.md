# Hybrid Protocol Setup Guide

## What is the Hybrid Protocol?

The **hybrid protocol** combines the best of both worlds:
- ✅ **Reliability** from minimal protocol (newline framing + `readline()`)
- ✅ **Structure** from original protocol (PID, commands, payload)
- ✅ **Debuggability** (ASCII hex - human readable)
- ✅ **~99.9% effective reliability** with retry logic

**Wire format:** `<AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n`

**Example:**
```
<AA5501000155AA>
 ││││││││││││││└─ Newline terminator
 │││││││││││││└── End marker '>'
 ││││││││││││└─── End markers 55 AA (hex)
 │││││││││││└──── Checksum (hex)
 ││││││││││└───── Payload (hex, can be empty)
 │││││││││└────── Length (hex)
 ││││││││└─────── Peripheral ID (hex)
 │││││││└──────── Start markers AA 55 (hex)
 ││││││└───────── Frame start '<'
 │││││└────────── Entire frame is ASCII
```

---

## Step 1: Configure ESP32 for Hybrid Protocol

1. Edit [src/config.h](src/config.h):

```cpp
// ====================================================================
// OPERATING MODES - Uncomment ONE mode
// ====================================================================
// #define MINIMAL_PROTOCOL_MODE  // Simple ASCII echo
#define HYBRID_PROTOCOL_MODE      // Binary + newline (RECOMMENDED) ← Enable this
// #define FULL_PROTOCOL_MODE     // Byte-stuffed binary
```

2. Upload firmware to ESP32

3. Open Serial Monitor (115200 baud) - you should see:
```
====================================
  HYBRID PROTOCOL MODE
  Binary + Newline Framing
====================================

Format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
Ready for requests...
```

---

## Step 2: Test with Python

### Basic Test:
```bash
cd test
python hybrid_protocol_tester.py
```

### What it does:
- Connects to ESP32
- Runs continuous stress test
- Sends requests with random peripherals and payload sizes
- Uses retry logic (2 retries = 3 total attempts)
- Shows statistics every 5 seconds

### Expected Output:
```
[5s] Tests: 85 | First-try: 98.82% | Recovered: 1 | Hard Fail: 0 |
     Effective: 100.00% | Rate: 17.0/s

[10s] Tests: 170 | First-try: 98.82% | Recovered: 2 | Hard Fail: 0 |
      Effective: 100.00% | Rate: 17.0/s
```

**Key metrics:**
- **First-try:** Base reliability (~98%)
- **Recovered:** Failures fixed by retry
- **Hard Fail:** Failures after all retries (should be ~0)
- **Effective:** User-facing reliability (should be ~99.9%+)

---

## Step 3: Understand the Protocol Flow

### Request Example (Get LORA Status):
```python
# Python sends:
frame = b'<AA5501000155AA>\n'
       # └─┬──┘││││││││││
       #   │   ││││││││││
       #   │   ││││││││└┴── End markers + newline
       #   │   │││││││└──── Checksum (0x01 ^ 0x00 ^ 0x01)
       #   │   ││││││└───── Command 0x00 (GET_ALL)
       #   │   │││││└────── Length 0x01 (1 byte payload)
       #   │   ││││└─────── PID 0x01 (LORA_915)
       #   │   │││└──────── Start markers
       #   │   ││└───────── Frame start
       #   └───┴┴────────── All ASCII hex

# ESP32 responds:
response = b'<AA5502010055AA>\n'
         # └─┬──┘││││││││││
         #   │   ││││││││└┴── End + newline
         #   │   │││││││└──── Checksum
         #   │   ││││││└───── Command 0x00 (echo)
         #   │   │││││└────── PID 0x01 (echo)
         #   │   ││││└─────── Length 0x02 (2 bytes)
         #   │   │││└──────── PID 0x01
         #   │   ││└───────── Start markers
```

### Why it Works:
1. **Newline delimiter:** Python's `readline()` waits until `\n` arrives
2. **ASCII hex:** No special bytes to escape, easy to debug
3. **Retry logic:** Handles periodic USB glitches automatically
4. **Atomic framing:** Complete frame or timeout, no partial reads

---

## Step 4: Compare to Old Protocol

| Feature | Old (Byte-Stuffed) | New (Hybrid) |
|---------|-------------------|--------------|
| Framing | Binary markers (0xAA, 0x55) | Newline (`\n`) |
| Encoding | Raw binary | ASCII hex |
| Parsing | 8-state machine | `readline()` |
| Special bytes | 3 (need escaping) | 0 |
| Reliability | ~95% (cascading failures) | ~98% base, ~99.9% with retry |
| Debugging | Need hex viewer | Can `cat /dev/ttyUSB0` |
| Size overhead | ~10% (stuffing) | ~2x (hex encoding) |
| Speed | ~87 bytes / 11ms | ~160 bytes / 11ms |

**Trade-off:** 2x size for 99.9% reliability + easy debugging = **WORTH IT**

---

## Step 5: Integration with Real Peripherals

Currently, the ESP32 echo server just echoes requests back. To integrate with real peripherals:

### Edit [src/hybrid_protocol.cpp](src/hybrid_protocol.cpp:134-158):

```cpp
void handleHybridRequest(uint8_t peripheral_id, uint8_t command,
                         const uint8_t* payload, size_t payload_len) {

    if (g_dataCollector == nullptr) {
        uint8_t error_msg[] = "NO_DATA_COLLECTOR";
        sendHybridFrame(peripheral_id, error_msg, sizeof(error_msg) - 1);
        return;
    }

    // Handle different peripherals
    switch (peripheral_id) {
        case 0x00: {  // SYSTEM
            if (command == 0x00) {  // GET_STATUS
                SystemStatus_t status = g_dataCollector->getSystemStatus();
                sendHybridFrame(peripheral_id, (uint8_t*)&status, sizeof(status));
            }
            break;
        }

        case 0x01: {  // LORA_915
            if (command == 0x00) {  // GET_ALL
                LoRaPacket_t packet = g_dataCollector->getLatestLoRaPacket();
                sendHybridFrame(peripheral_id, (uint8_t*)&packet, sizeof(packet));
            }
            break;
        }

        case 0x03: {  // BAROMETER
            BarometerData_t data = g_dataCollector->getBarometerData();
            sendHybridFrame(peripheral_id, (uint8_t*)&data, sizeof(data));
            break;
        }

        case 0x04: {  // CURRENT
            CurrentData_t data = g_dataCollector->getCurrentData();
            sendHybridFrame(peripheral_id, (uint8_t*)&data, sizeof(data));
            break;
        }

        default:
            uint8_t error[] = "UNKNOWN_PERIPHERAL";
            sendHybridFrame(peripheral_id, error, sizeof(error) - 1);
            break;
    }
}
```

---

## Step 6: Update Python Client

For production, integrate hybrid protocol into your main Python client:

```python
from hybrid_protocol_tester import encode_hybrid_frame, decode_hybrid_frame

class GroundStationClient:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=3.0)

    def request_with_retry(self, pid, command, payload=b'', max_retries=2):
        for attempt in range(max_retries + 1):
            # Send request
            frame = encode_hybrid_frame(pid, bytes([command]) + payload)
            self.ser.write(frame)
            self.ser.flush()

            # Read response
            line = self.ser.readline()
            if not line:
                if attempt < max_retries:
                    time.sleep(0.05)
                    continue
                return None, "TIMEOUT"

            # Decode
            rx_pid, rx_payload = decode_hybrid_frame(line)
            if rx_pid == pid:
                return rx_payload, None

        return None, "FAILED_ALL_RETRIES"

    def get_lora_data(self):
        payload, error = self.request_with_retry(0x01, 0x00)
        if error:
            return None
        return payload  # Parse as LoRaPacket_t

    def get_barometer_data(self):
        payload, error = self.request_with_retry(0x03, 0x00)
        if error:
            return None
        return payload  # Parse as BarometerData_t
```

---

## Troubleshooting

### Issue: "INVALID FRAME" errors
**Cause:** ESP32 not in HYBRID_PROTOCOL_MODE
**Fix:** Check [src/config.h](src/config.h) line 8, re-upload firmware

### Issue: First-try success < 95%
**Cause:** USB cable quality or interference
**Fix:** Try different USB cable, avoid USB hubs

### Issue: Hard failures > 1%
**Cause:** Something wrong with retry logic or severe USB issues
**Fix:** Check logs, increase timeout, try different computer

### Issue: "Payload too long"
**Cause:** Payload > 255 bytes
**Fix:** Split into multiple requests or increase protocol limit

---

## Next Steps

1. ✅ Test hybrid protocol with stress test → Verify ~99.9% reliability
2. ☐ Integrate with real peripherals (LoRa, barometer, etc.)
3. ☐ Update main Python ground station client
4. ☐ Deploy and test in actual rocketry scenario
5. ☐ Monitor production reliability metrics

---

## Success Metrics

- **First-try success:** ~98% (baseline, same as minimal protocol)
- **Effective success:** ~99.9% (with 2 retries)
- **Hard failures:** <0.1% (should be near zero)
- **Test rate:** 15-20 tests/second (stress test)
- **Production rate:** 5-10 requests/second (normal polling)

**If you see these numbers, your protocol is working correctly!**
