# Binary Protocol Specification

Complete specification of the Meerkat Embedded Routine binary communication protocol.

## Overview

The protocol uses pure binary framing for efficient, reliable communication between ESP32 and host (Raspberry Pi or PC) over UART at 921600 baud.

### Design Goals
- **Efficiency**: Binary encoding (50% smaller than ASCII hex)
- **Reliability**: Frame markers, length field, and checksum
- **Low Latency**: Minimal overhead, non-blocking design
- **Simplicity**: Fixed format, easy to implement

## Frame Format

### Structure (Pure Binary)

(All data communicated utilising the embedded system, to and from peripherals and the host pi are done in this format)

```
<AA55[PID][LEN][PAYLOAD][CHK]55AA>

Byte Offset  Field        Size    Description
───────────────────────────────────────────────────
0            START        1       Start marker '<' (0x3C)
1-2          HEADER       2       Protocol header 0xAA 0x55
3            PID          1       Peripheral ID (0x00-0xFF)
4            LEN          1       Payload length (0-255 bytes)
5 to 4+LEN   PAYLOAD      LEN     Command/data (raw binary)
5+LEN        CHECKSUM     1       XOR checksum
6+LEN to 7+LEN FOOTER    2       Protocol footer 0x55 0xAA
8+LEN        END          1       End marker '>' (0x3E)

Total frame size: 9 + LEN bytes
```

### Field Descriptions

**START MARKER** (`<` = 0x3C)
- Marks beginning of frame
- Helps receiver synchronize to frame boundary

**PROTOCOL HEADER** (0xAA 0x55)
- Fixed 2-byte signature
- Validates frame format
- Distinguishes from debug text

**PERIPHERAL ID (PID)**
- Identifies target peripheral or data source
- See [Peripheral IDs](#peripheral-ids) section

**LENGTH (LEN)**
- Number of bytes in PAYLOAD field
- Range: 0-255 bytes
- Allows variable-length payloads

**PAYLOAD**
- Raw binary data
- This 'payload' is the main data field of the packet and is used by default to request what type of data you are requesting of the specific device. I.e, to request all data points from the LORA 915 radio module, you would first send the peripheral ID 0x01, then 0x00 wihtin the payload to request all data. If you simply wanted to check the status of the LORA 915, you would send the peripheral ID as 0x01 and the payload as 0x01. 
- Remaining bytes are command-specific parameters or response data
- May contain null bytes (0x00) - protocol handles binary data correctly

**CHECKSUM (CHK)**
- XOR of PID, LEN, and all PAYLOAD bytes
- Simple but effective error detection
- Formula: `CHK = PID ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[LEN-1]`

**PROTOCOL FOOTER** (0x55 0xAA)
- Fixed 2-byte signature (inverse of header)
- Additional frame validation

**END MARKER** (`>` = 0x3E)
- Marks end of frame
- Receiver knows frame is complete

### Example Frames

**Get System Status:**
```
TX: 3C AA 55 00 01 20 21 55 AA 3E
    <  AA 55 00 01 20 21 55 AA >

    PID  = 0x00 (SYSTEM)
    LEN  = 0x01 (1 byte payload)
    PAYLOAD = 0x20 (CMD_SYSTEM_STATUS)
    CHK  = 0x00 ^ 0x01 ^ 0x20 = 0x21
```

**Set Poll Rate (LoRa, 1000ms):**
```
TX: 3C AA 55 01 03 02 E8 03 E8 55 AA 3E
    <  AA 55 01 03 02 E8 03 E8 55 AA >

    PID  = 0x01 (LORA_915)
    LEN  = 0x03 (3 bytes payload)
    PAYLOAD = 02 E8 03
              - 0x02 = CMD_SET_POLL_RATE
              - 0xE8 0x03 = 1000ms (little-endian)
    CHK  = 0x01 ^ 0x03 ^ 0x02 ^ 0xE8 ^ 0x03 = 0xE8
```

**ACK Response:**
```
RX: 3C AA 55 01 01 01 01 55 AA 3E
    <  AA 55 01 01 01 01 55 AA >

    PID  = 0x01 (LORA_915)
    LEN  = 0x01 (1 byte payload)
    PAYLOAD = 0x01 (RESP_ACK)
    CHK  = 0x01 ^ 0x01 ^ 0x01 = 0x01
```

## Peripheral IDs

| PID  | Name           | Description                        |
|------|----------------|------------------------------------|
| 0x00 | SYSTEM         | ESP32 system control and status    |
| 0x01 | LORA_915       | 915MHz LoRa module                 |
| 0x02 | RADIO_433      | 433MHz radio module                |
| 0x03 | BAROMETER      | MS5607 barometer sensor            |
| 0x04 | CURRENT        | Current/voltage sensor             |
| 0xFF | ALL            | Target all peripherals (GET_ALL)   |

**Notes:**
- PIDs 0x10-0x1F reserved for future AIM boards
- PID 0xFF only valid for `CMD_GET_ALL` - triggers multi-peripheral response

## Commands

### Command Ranges

| Range     | Type                    | Valid PIDs     |
|-----------|-------------------------|----------------|
| 0x00-0x0F | Generic commands        | 0x01-0x04, 0xFF|
| 0x10-0x1F | Peripheral-specific     | Varies         |
| 0x20-0x2F | System commands         | 0x00 only      |

### Generic Commands (0x00-0x0F)

Work for all sensor peripherals (PID 0x01-0x04, 0xFF):

| Code | Name            | Payload                          | Response              |
|------|-----------------|----------------------------------|-----------------------|
| 0x00 | GET_ALL         | None                             | Peripheral data       |
| 0x01 | GET_STATUS      | None                             | Status/health         |
| 0x02 | SET_POLL_RATE   | interval_ms (2 bytes, LE)        | ACK or ERROR          |
| 0x03 | STOP_POLL       | None                             | ACK                   |

**SET_POLL_RATE Payload:**
```
Byte 0: 0x02 (command)
Byte 1: Interval low byte (little-endian)
Byte 2: Interval high byte

Example: 1000ms = 0x02 0xE8 0x03
         500ms  = 0x02 0xF4 0x01
         100ms  = 0x02 0x64 0x00
```

**Rate Limits:**
- Single peripheral: minimum 30ms
- PID_ALL: minimum 100ms
- Violations return ERROR_INVALID_CMD

### System Commands (0x20-0x2F)

Only valid for PID = 0x00:

| Code | Name                 | Payload                  | Response              |
|------|----------------------|--------------------------|-----------------------|
| 0x20 | SYSTEM_STATUS        | None                     | WireStatus_t (20B)    |
| 0x21 | SYSTEM_WAKEUP        | None                     | WireStatus_t (20B)    |
| 0x22 | SYSTEM_SLEEP         | None                     | ACK                   |
| 0x23 | SYSTEM_RESET         | None                     | (ESP32 reboots)       |
| 0x24 | SYSTEM_PERF          | enable (1 byte)          | ACK                   |
| 0x25 | SYSTEM_STATS         | None                     | WireTransferStats_t   |
| 0x26 | SYSTEM_STATS_RESET   | None                     | ACK                   |

**SYSTEM_PERF Payload:**
```
Byte 0: 0x24 (command)
Byte 1: 0x00 = disable, 0x01 = enable
```

## Response Codes

### Standard Responses

**ACK (0x01):**
```
PID  = [original peripheral]
LEN  = 1
PAYLOAD = 0x01
```

**ERROR (0xFF):**
```
PID  = [original peripheral]
LEN  = 2
PAYLOAD = 0xFF [error_code]
```

### Error Codes

| Code | Name                | Description                          |
|------|---------------------|--------------------------------------|
| 0x01 | INVALID_FRAME       | Frame parsing failed                 |
| 0x02 | QUEUE_FULL          | Command queue overflow               |
| 0x03 | INVALID_CMD         | Unknown command or invalid params    |
| 0x04 | INVALID_PID         | Unknown peripheral ID                |

## Data Structures

### WireStatus_t (20 bytes)

System status response from `CMD_SYSTEM_STATUS`:

```c
Offset  Type     Field              Description
─────────────────────────────────────────────────────
0       uint8    version            Protocol version (1)
1-4     uint32   uptime_seconds     Uptime since boot (LE)
5       uint8    system_state       0=sleep, 1=operational
6       uint8    flags              Sensor online flags (bitfield)
7-8     uint16   packet_count_lora  LoRa packets received (LE)
9-10    uint16   packet_count_433   433MHz packets received (LE)
11-14   uint32   wakeup_time        Last wakeup timestamp (LE)
15-18   uint32   free_heap          Free heap bytes (LE)
19      uint8    chip_revision      ESP32 chip revision
```

**Sensor Flags (byte 6):**
```
Bit 0: LoRa online
Bit 1: 433MHz online
Bit 2: Barometer online
Bit 3: Current sensor online
Bit 4: Pi connected
Bits 5-7: Reserved
```

### WireLoRa_t (74 bytes)

LoRa data response from `CMD_GET_ALL` on PID 0x01:

```c
Offset  Type     Field              Description
─────────────────────────────────────────────────────
0       uint8    version            Data structure version
1-2     uint16   packet_count       Total packets received (LE)
3-4     int16    rssi_dbm           RSSI in dBm (LE, signed)
5-8     float    snr_db             SNR in dB (LE, IEEE 754)
9       uint8    latest_len         Length of latest packet
10-73   uint8[]  latest_data        Latest packet data (64 bytes max)
```

### Wire433_t (74 bytes)

Same structure as WireLoRa_t, for PID 0x02.

### WireBarometer_t (17 bytes)

Barometer data from PID 0x03:

```c
Offset  Type     Field              Description
─────────────────────────────────────────────────────
0       uint8    version            Data structure version
1-4     uint32   timestamp_ms       Measurement timestamp (LE)
5-8     float    pressure_hpa       Pressure in hPa (LE)
9-12    float    temperature_c      Temperature in °C (LE)
13-16   float    altitude_m         Calculated altitude in meters (LE)
```

### WireCurrent_t (19 bytes)

Current sensor data from PID 0x04:

```c
Offset  Type     Field              Description
─────────────────────────────────────────────────────
0       uint8    version            Data structure version
1-4     uint32   timestamp_ms       Measurement timestamp (LE)
5-8     float    current_a          Current in amperes (LE)
9-12    float    voltage_v          Voltage in volts (LE)
13-16   float    power_w            Calculated power in watts (LE)
17-18   int16    raw_adc            Raw ADC value (LE, signed)
```

### WireTransferStats_t (41 bytes)

Transfer statistics from `CMD_SYSTEM_STATS`:

```c
Offset  Type     Field              Description
─────────────────────────────────────────────────────
0       uint8    version            Data structure version
1-8     struct   stats[0]           SYSTEM stats (8 bytes)
9-16    struct   stats[1]           LORA_915 stats (8 bytes)
17-24   struct   stats[2]           RADIO_433 stats (8 bytes)
25-32   struct   stats[3]           BAROMETER stats (8 bytes)
33-40   struct   stats[4]           CURRENT stats (8 bytes)

Each stats struct (8 bytes):
  Offset  Type     Field
  0-3     uint32   total_packets   Total packets sent (LE)
  4-7     uint32   total_bytes     Total bytes sent (LE)
```

## Implementation Notes

### Checksum Calculation

```python
def calculate_checksum(pid, length, payload):
    """Calculate XOR checksum"""
    chk = pid ^ length
    for byte in payload:
        chk ^= byte
    return chk & 0xFF
```

```cpp
uint8_t calculateChecksum(uint8_t pid, uint8_t len, const uint8_t* payload) {
    uint8_t chk = pid ^ len;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= payload[i];
    }
    return chk;
}
```

### Frame Encoding (Python)

```python
def encode_hybrid_frame(peripheral_id, payload):
    """Encode binary frame"""
    if len(payload) > 255:
        raise ValueError("Payload too long")

    checksum = calculate_checksum(peripheral_id, len(payload), payload)

    frame = b'<'
    frame += b'\xAA\x55'
    frame += bytes([peripheral_id])
    frame += bytes([len(payload)])
    frame += payload
    frame += bytes([checksum])
    frame += b'\x55\xAA'
    frame += b'>'

    return frame
```

### Frame Decoding (Python)

```python
def decode_hybrid_frame(data):
    """Decode binary frame"""
    # Check minimum length
    if len(data) < 9:
        return None, None

    # Verify markers
    if data[0] != ord('<') or data[-1] != ord('>'):
        return None, None
    if data[1] != 0xAA or data[2] != 0x55:
        return None, None

    # Extract fields
    pid = data[3]
    length = data[4]

    # Verify frame length
    if len(data) != 9 + length:
        return None, None

    # Extract payload
    payload = data[5:5 + length]

    # Verify checksum
    rx_checksum = data[5 + length]
    calc_checksum = calculate_checksum(pid, length, payload)
    if rx_checksum != calc_checksum:
        return None, None

    # Verify footer
    if data[6 + length] != 0x55 or data[7 + length] != 0xAA:
        return None, None

    return pid, payload
```

### Receiver State Machine

The receiver reads bytes one at a time, looking for complete frames:

```python
frame_buffer = bytearray()

while True:
    byte = serial.read(1)
    frame_buffer.extend(byte)

    # Frame complete when '>' received
    if byte[0] == ord('>'):
        # Process frame
        pid, payload = decode_hybrid_frame(frame_buffer)
        if pid is not None:
            handle_frame(pid, payload)
        frame_buffer.clear()

    # Handle debug messages (start with '[')
    elif frame_buffer[0] == ord('['):
        if byte[0] in (ord('\n'), ord('\r')):
            # Debug message complete
            print(frame_buffer.decode('ascii'))
            frame_buffer.clear()
```

## Debug Messages

Debug messages are interleaved with binary frames:

**Format:** `[PREFIX] message text\n`

**Prefixes:**
- `[INFO]` - System information
- `[PERF]` - Performance statistics
- `[STATS]` - Transfer statistics events
- `[RATE_LIMIT]` - Rate limiting warnings

**Example:**
```
[INFO] ESP32 BINARY PROTOCOL MODE
[PERF] 12345.6 loops/sec, 0.081 ms/loop, Queue: 3/8
[STATS] Transfer statistics saved to flash
```

**Handling:**
- Receiver should filter lines starting with `[`
- Display to user but don't parse as binary frames
- Useful for debugging and monitoring

## Protocol Versioning

Current version: **1**

All data structures include a `version` byte (first byte):
- Allows future protocol extensions
- Receivers should check version and reject unknown versions
- Backward compatibility not guaranteed across major versions

## Performance Characteristics

**Frame Overhead:**
- Fixed: 9 bytes per frame
- Percentage overhead: decreases with payload size
  - 10 byte payload: 47% overhead (19 bytes total)
  - 50 byte payload: 15% overhead (59 bytes total)
  - 255 byte payload: 3.4% overhead (264 bytes total)

**Throughput (921600 baud):**
- Theoretical max: ~92 KB/sec
- Practical: ~70-85 KB/sec (with overhead and gaps)
- Latency: <2ms for command response

**Error Detection:**
- Checksum catches ~99.6% of single-bit errors
- Frame markers catch framing errors
- Length field catches truncation

## Security Considerations

**This protocol has NO security features:**
- No encryption
- No authentication
- No replay protection
- No integrity beyond simple checksum

**Use only on trusted, physically secured connections.**

For production systems requiring security:
- Add TLS/DTLS layer above this protocol
- Or implement authenticated encryption in application layer
- Never expose raw serial port to untrusted networks

## See Also

- [README.md](../README.md) - Project overview
- [GETTING_STARTED.md](GETTING_STARTED.md) - Setup instructions
- [src/config.h](../src/config.h) - Protocol constants
- [test/interactive_sender.py](../test/interactive_sender.py) - Reference implementation
