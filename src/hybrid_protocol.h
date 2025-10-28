#ifndef HYBRID_PROTOCOL_H
#define HYBRID_PROTOCOL_H

#include <Arduino.h>
#include "DataCollector.h"

// ====================================================================
// HYBRID PROTOCOL - STANDALONE MODE
// ====================================================================
//
// Wire format: <AA55[PID][LEN][PAYLOAD_HEX][CHK]55AA>\n
//
// Combines:
// - Binary structure (PID, length, payload, checksum) from original protocol
// - Newline framing for atomic readline() operations
// - ASCII hex encoding for reliability and debuggability
//
// Example: <AA5501000155AA>\n
//          └─ '<' frame start
//             AA55 - start markers (binary AA 55 encoded as hex)
//             01 - PID (LORA_915)
//             00 - Length (0 bytes)
//             01 - Checksum (01 ^ 00 = 01)
//             55AA - end markers (binary 55 AA encoded as hex)
//             '>' - frame end
//             '\n' - newline terminator
//
// ====================================================================

#define HYBRID_MAX_PAYLOAD_SIZE  128  // Maximum payload bytes
#define HYBRID_MAX_FRAME_SIZE    512  // Maximum ASCII frame size (includes hex encoding overhead)

// Frame markers
#define HYBRID_FRAME_START  '<'
#define HYBRID_FRAME_END    '>'

// Binary markers (encoded as hex in frame)
#define HYBRID_START_MARKER_1  0xAA
#define HYBRID_START_MARKER_2  0x55
#define HYBRID_END_MARKER_1    0x55
#define HYBRID_END_MARKER_2    0xAA

// ====================================================================
// STANDALONE MODE API (for testing - runs in simple loop, no FreeRTOS)
// ====================================================================

/**
 * Run hybrid protocol in standalone mode.
 * This blocks in setup() and never returns - perfect for testing!
 * NO FreeRTOS tasks, NO threading, just simple loop.
 */
void runHybridProtocolMode();

#endif // HYBRID_PROTOCOL_H
