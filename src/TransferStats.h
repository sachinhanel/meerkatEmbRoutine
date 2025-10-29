/**
 * @file TransferStats.h
 * @brief Track data transfer statistics per peripheral with persistent storage
 *
 * Features:
 * - Track bytes sent and packets sent for each peripheral
 * - Persist to flash storage (survives reboots)
 * - Low-priority periodic saves to avoid blocking main loop
 * - Retrieve stats via system command
 */

#ifndef TRANSFER_STATS_H
#define TRANSFER_STATS_H

#include <Arduino.h>
#include <LittleFS.h>
#include "config.h"

// Per-peripheral statistics
struct PeripheralStats {
    uint32_t total_packets;     // Total packets sent
    uint32_t total_bytes;       // Total bytes sent (payload only)
    uint32_t session_packets;   // Packets sent this session (since boot)
    uint32_t session_bytes;     // Bytes sent this session
};

// Wire format for statistics response (sent to Pi)
// Total size: 1 + 5*(4+4) = 41 bytes
struct WireTransferStats_t {
    uint8_t version;            // Protocol version

    // Stats for each peripheral (8 bytes each)
    struct {
        uint32_t total_packets;
        uint32_t total_bytes;
    } stats[5];  // SYSTEM, LORA_915, RADIO_433, BAROMETER, CURRENT
} __attribute__((packed));

class TransferStats {
public:
    TransferStats();

    // Initialize (mount filesystem and load stats)
    bool begin();

    // Record a transmission
    void recordTransmission(uint8_t peripheral_id, size_t bytes);

    // Save to flash (call periodically, not on every transmission)
    bool save();

    // Load from flash
    bool load();

    // Get stats for a specific peripheral
    PeripheralStats getStats(uint8_t peripheral_id);

    // Reset all stats (both persistent and session)
    void reset();

    // Pack statistics into wire format for transmission
    size_t packStats(uint8_t* out, size_t max_len);

    // Get time since last save (for low-priority save scheduling)
    uint32_t timeSinceLastSave() const { return millis() - last_save_time; }

    // Check if stats have changed since last save
    bool hasUnsavedChanges() const { return dirty; }

private:
    PeripheralStats stats[5];  // One for each peripheral (indices match peripheral IDs)
    uint32_t last_save_time;   // Last time stats were saved to flash
    bool dirty;                // Whether stats have changed since last save
    bool initialized;          // Whether filesystem is initialized

    // Helper to get array index from peripheral ID
    int getIndex(uint8_t peripheral_id);
};

#endif // TRANSFER_STATS_H
