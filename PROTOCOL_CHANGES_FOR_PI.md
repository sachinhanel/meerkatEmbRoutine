# ESP32-Pi Communication Protocol Changes
## Summary for Pi-Side Implementation

---

## Overview

The communication protocol has been refactored to be **modular and scalable**. Each peripheral (sensor/device) now has its own ID and handles its own commands independently.

---

## Key Changes

### 1. **Framing Bytes** (UNCHANGED)
- **HELLO_BYTE (0x7E)**: Start of Pi → ESP32 message
- **RESPONSE_BYTE (0x7D)**: Start of ESP32 → Pi message (different from HELLO to avoid echo)
- **GOODBYE_BYTE (0x7F)**: End of message marker

### 2. **Message Structure**

#### Pi → ESP32 (Command):
```
[HELLO] [PERIPHERAL_ID] [LENGTH] [COMMAND] [optional data...] [GOODBYE]
```

#### ESP32 → Pi (Response):
```
[RESPONSE] [PERIPHERAL_ID] [LENGTH] [data...] [GOODBYE]
```

**Key Point**: The PERIPHERAL_ID in the response **matches** the PERIPHERAL_ID in the command. This allows the Pi to know which device responded.

---

## Peripheral IDs (One ID per device)

| Peripheral ID | Device | Description |
|--------------|--------|-------------|
| `0x00` | SYSTEM | ESP32 system control |
| `0x01` | LORA_915 | 915MHz LoRa module |
| `0x02` | LORA_433 | 433MHz LoRa module (same chip as 915MHz, different freq) |
| `0x03` | BAROMETER | MS5607 barometer |
| `0x04` | CURRENT | Current/voltage sensor |
| `0x10-0x13` | AIM_1 to AIM_4 | Future AIM boards |

---

## Generic Commands (Work for ALL peripherals: 0x00-0x0F)

| Command | Value | Description | Applies To |
|---------|-------|-------------|------------|
| `CMD_GET_ALL` | `0x00` | Get all available data from peripheral | All peripherals |
| `CMD_GET_STATUS` | `0x01` | Get status/health of peripheral | All peripherals |
| `CMD_RESET` | `0x02` | Reset peripheral | All peripherals (future) |
| `CMD_CONFIGURE` | `0x03` | Configure peripheral | All peripherals (future) |

---

## System-Only Commands (Only for PERIPHERAL_ID = 0x00)

| Command | Value | Description |
|---------|-------|-------------|
| `CMD_SYSTEM_WAKEUP` | `0x20` | Wake up system from low-power state |
| `CMD_SYSTEM_SLEEP` | `0x21` | Put system into low-power state |
| `CMD_SYSTEM_RESET` | `0x22` | Reset entire ESP32 |

---

## Message Examples

### Example 1: Get LoRa Data
**Pi → ESP32:**
```
[0x7E] [0x01] [0x01] [0x00] [0x7F]
 HELLO  LORA  len=1  GET_ALL GOODBYE
```

**ESP32 → Pi:**
```
[0x7D] [0x01] [0x4A] [<74 bytes WireLoRa_t>] [0x7F]
 RESP   LORA  len=74  data                    GOODBYE
```

### Example 2: Wake Up System
**Pi → ESP32:**
```
[0x7E] [0x00] [0x01] [0x20] [0x7F]
 HELLO  SYS   len=1  WAKEUP GOODBYE
```

**ESP32 → Pi:**
```
[0x7D] [0x00] [0x01] [0x20] [0x7F]
 RESP   SYS   len=1  ACK    GOODBYE
```

### Example 3: Get Barometer Data
**Pi → ESP32:**
```
[0x7E] [0x03] [0x01] [0x00] [0x7F]
 HELLO  BARO  len=1  GET_ALL GOODBYE
```

**ESP32 → Pi:**
```
[0x7D] [0x03] [0x11] [<17 bytes WireBarometer_t>] [0x7F]
 RESP   BARO  len=17  data                        GOODBYE
```

### Example 4: Get System Status
**Pi → ESP32:**
```
[0x7E] [0x00] [0x01] [0x00] [0x7F]
 HELLO  SYS   len=1  GET_ALL GOODBYE
```

**ESP32 → Pi:**
```
[0x7D] [0x00] [0x14] [<20 bytes WireStatus_t>] [0x7F]
 RESP   SYS   len=20  data                      GOODBYE
```

---

## Unsolicited Messages (ESP32 → Pi)

The ESP32 also sends **heartbeat/status messages** without being asked:

### Before Wakeup: Heartbeat (every 5 seconds)
```
[0x7D] [0x00] [0x06] [<6 bytes WireHeartbeat_t>] [0x7F]
 RESP   SYS   len=6   data                        GOODBYE
```
**Contains**: version, uptime, system_state (WAITING_FOR_WAKEUP)

### After Wakeup: Status (every 5 seconds)
```
[0x7D] [0x00] [0x14] [<20 bytes WireStatus_t>] [0x7F]
 RESP   SYS   len=20  data                      GOODBYE
```
**Contains**: Full system status (uptime, sensor flags, packet counts, heap, etc.)

---

## Data Structures (Binary Format)

All data structures are **little-endian, packed**.

### WireHeartbeat_t (6 bytes)
```python
struct.unpack('<BIB', data)
# version(1), uptime_seconds(4), system_state(1)
```

### WireStatus_t (20 bytes)
```python
struct.unpack('<BIBBHHIIB', data)
# version(1), uptime(4), state(1), flags(1),
# pkt_lora(2), pkt_433(2), wakeup_time(4), heap(4), chip_rev(1)
```

### WireLoRa_t (74 bytes)
```python
struct.unpack('<BHhfB64s', data)
# version(1), pkt_count(2), rssi(2), snr(4), len(1), data(64)
```

### Wire433_t (74 bytes) - Same as WireLoRa_t
**Note**: 433MHz module is also LoRa (same chip, different frequency)
```python
struct.unpack('<BHhfB64s', data)
# version(1), pkt_count(2), rssi(2), snr(4), len(1), data(64)
```

### WireBarometer_t (17 bytes)
```python
struct.unpack('<BIfff', data)
# version(1), timestamp(4), pressure(4), temp(4), altitude(4)
```

### WireCurrent_t (19 bytes)
```python
struct.unpack('<BIfffh', data)
# version(1), timestamp(4), current(4), voltage(4), power(4), raw_adc(2)
```

---

## Implementation Checklist for Pi Code

### 1. Update Constants
```python
# Framing bytes
HELLO_BYTE = 0x7E
RESPONSE_BYTE = 0x7D  # NEW! Different from HELLO
GOODBYE_BYTE = 0x7F

# Peripheral IDs
PERIPHERAL_ID_SYSTEM = 0x00
PERIPHERAL_ID_LORA_915 = 0x01
PERIPHERAL_ID_LORA_433 = 0x02  # 433MHz is also LoRa (same chip, different freq)
PERIPHERAL_ID_BAROMETER = 0x03
PERIPHERAL_ID_CURRENT = 0x04

# Backward compatibility
PERIPHERAL_ID_RADIO_433 = PERIPHERAL_ID_LORA_433

# Generic commands
CMD_GET_ALL = 0x00
CMD_GET_STATUS = 0x01
CMD_RESET = 0x02
CMD_CONFIGURE = 0x03

# System commands
CMD_SYSTEM_WAKEUP = 0x20
CMD_SYSTEM_SLEEP = 0x21
CMD_SYSTEM_RESET = 0x22
```

### 2. Update Send Function
```python
def send_command(peripheral_id, command, data=b''):
    """Send a command to a specific peripheral"""
    payload = bytes([command]) + data
    message = bytes([
        HELLO_BYTE,
        peripheral_id,
        len(payload)
    ]) + payload + bytes([GOODBYE_BYTE])

    serial_port.write(message)
```

### 3. Update Receive Function
```python
def read_response():
    """Read a response from ESP32"""
    # Wait for RESPONSE_BYTE (not HELLO_BYTE!)
    if serial_port.read(1)[0] != RESPONSE_BYTE:
        return None

    peripheral_id = serial_port.read(1)[0]
    length = serial_port.read(1)[0]
    payload = serial_port.read(length)
    goodbye = serial_port.read(1)[0]

    if goodbye != GOODBYE_BYTE:
        return None

    return {
        'peripheral_id': peripheral_id,
        'payload': payload
    }
```

### 4. Update Command Functions
```python
def get_lora_data():
    """Get LoRa data using new protocol"""
    send_command(PERIPHERAL_ID_LORA_915, CMD_GET_ALL)
    response = read_response()
    if response and response['peripheral_id'] == PERIPHERAL_ID_LORA_915:
        return unpack_lora_data(response['payload'])
    return None

def get_barometer_data():
    """Get barometer data using new protocol"""
    send_command(PERIPHERAL_ID_BAROMETER, CMD_GET_ALL)
    response = read_response()
    if response and response['peripheral_id'] == PERIPHERAL_ID_BAROMETER:
        return unpack_barometer_data(response['payload'])
    return None

def wakeup_system():
    """Wake up the ESP32"""
    send_command(PERIPHERAL_ID_SYSTEM, CMD_SYSTEM_WAKEUP)
    response = read_response()
    if response and response['peripheral_id'] == PERIPHERAL_ID_SYSTEM:
        # Response payload is [0x20] (CMD_SYSTEM_WAKEUP ACK)
        return response['payload'][0] == CMD_SYSTEM_WAKEUP
    return False

def get_system_status():
    """Get system status"""
    send_command(PERIPHERAL_ID_SYSTEM, CMD_GET_ALL)
    response = read_response()
    if response and response['peripheral_id'] == PERIPHERAL_ID_SYSTEM:
        return unpack_status(response['payload'])
    return None
```

### 5. Handle Unsolicited Messages
```python
def handle_unsolicited_message(response):
    """Handle unsolicited heartbeat/status messages"""
    if response['peripheral_id'] == PERIPHERAL_ID_SYSTEM:
        payload_len = len(response['payload'])

        if payload_len == 6:
            # Heartbeat message (before wakeup)
            data = struct.unpack('<BIB', response['payload'])
            print(f"Heartbeat: uptime={data[1]}s, state={data[2]}")

        elif payload_len == 20:
            # Status message (after wakeup)
            status = unpack_status(response['payload'])
            print(f"Status: uptime={status['uptime']}s, state={status['state']}")
```

---

## Migration Guide

### Old Code (System-centric commands):
```python
# OLD: All commands sent to PERIPHERAL_ID_SYSTEM
send_command(PERIPHERAL_ID_SYSTEM, CMD_GET_LORA_DATA)  # 0x31
```

### New Code (Peripheral-centric commands):
```python
# NEW: Commands sent to specific peripheral
send_command(PERIPHERAL_ID_LORA_915, CMD_GET_ALL)  # 0x00
```

---

## Benefits of New Architecture

✅ **Modular**: Each peripheral is self-contained
✅ **Scalable**: Easy to add new peripherals or commands
✅ **Clear Routing**: Peripheral ID directly identifies device
✅ **Consistent**: All peripherals use same command structure
✅ **Future-Proof**: Can add peripheral-specific commands (0x10-0x1F) later

---

## Future Expansion Example

Later, you can add peripheral-specific commands:

```python
# LoRa-specific commands (future)
CMD_LORA_GET_SNR = 0x10
CMD_LORA_GET_RSSI = 0x11
CMD_LORA_SET_FREQUENCY = 0x12

# Get only SNR from LoRa
send_command(PERIPHERAL_ID_LORA_915, CMD_LORA_GET_SNR)
# Response would be just 4 bytes (float SNR)
```

---

## Testing Checklist

- [ ] Test WAKEUP command (0x00, 0x20)
- [ ] Test GET_ALL for each peripheral (0x01-0x04, 0x00)
- [ ] Verify responses have correct PERIPHERAL_ID
- [ ] Test heartbeat message reception (6 bytes)
- [ ] Test status message reception (20 bytes)
- [ ] Verify RESPONSE_BYTE (0x7D) instead of HELLO_BYTE
- [ ] Test error handling for unknown peripheral
- [ ] Test error handling for unknown command

---

## Summary

**Main Changes:**
1. **Responses use RESPONSE_BYTE (0x7D)** instead of HELLO_BYTE (0x7E)
2. **One PERIPHERAL_ID per device** (not per command)
3. **Commands are peripheral-specific** (first byte of payload)
4. **Responses echo PERIPHERAL_ID** from the command
5. **Generic CMD_GET_ALL (0x00)** replaces old CMD_GET_*_DATA commands
6. **System commands** use 0x20-0x2F range

**Old system commands removed:**
- ~~CMD_GET_LORA_DATA (0x31)~~
- ~~CMD_GET_433_DATA (0x32)~~
- ~~CMD_GET_BAROMETER_DATA (0x33)~~
- ~~CMD_GET_CURRENT_DATA (0x34)~~
- ~~CMD_GET_ALL_DATA (0x35)~~
- ~~CMD_GET_STATUS (0x36)~~

**Replaced by:**
- Send CMD_GET_ALL (0x00) to specific PERIPHERAL_ID

This makes the protocol much cleaner and more maintainable!
