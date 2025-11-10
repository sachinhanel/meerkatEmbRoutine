#include "TransferStats.h"

#define STATS_FILE "/transfer_stats.bin"
#define SAVE_INTERVAL_MS 30000  // Save every 30 seconds if dirty

TransferStats::TransferStats() :
    last_save_time(0),
    dirty(false),
    initialized(false) {
    // Initialize all stats to zero
    for (int i = 0; i < 5; i++) {
        stats[i].total_packets = 0;
        stats[i].total_bytes = 0;
        stats[i].session_packets = 0;
        stats[i].session_bytes = 0;
    }
}

bool TransferStats::begin() {
    // Mount LittleFS
    if (!LittleFS.begin(true)) {  // true = format on failure
        Serial.println("[STATS] Failed to mount LittleFS");
        return false;
    }

    initialized = true;
    Serial.println("[STATS] LittleFS mounted successfully");

    // Load existing stats from flash
    if (load()) {
        Serial.println("[STATS] Loaded transfer statistics from flash");
    } else {
        Serial.println("[STATS] No existing stats found, starting fresh");
    }

    last_save_time = millis();
    return true;
}

void TransferStats::recordTransmission(uint8_t peripheral_id, size_t bytes) {
    int idx = getIndex(peripheral_id);
    if (idx < 0) return;  // Invalid peripheral ID

    stats[idx].total_packets++;
    stats[idx].total_bytes += bytes;
    stats[idx].session_packets++;
    stats[idx].session_bytes += bytes;

    dirty = true;
}

bool TransferStats::save() {
    if (!initialized) {
        Serial.println("[STATS] Cannot save - filesystem not initialized");
        return false;
    }

    // Open file for writing
    File file = LittleFS.open(STATS_FILE, "w");
    if (!file) {
        Serial.println("[STATS] Failed to open file for writing");
        return false;
    }

    // Write magic number and version
    uint32_t magic = 0xDEADBEEF;
    uint8_t version = 1;
    file.write((uint8_t*)&magic, sizeof(magic));
    file.write(&version, 1);

    // Write stats for each peripheral (only persistent data)
    for (int i = 0; i < 5; i++) {
        file.write((uint8_t*)&stats[i].total_packets, sizeof(uint32_t));
        file.write((uint8_t*)&stats[i].total_bytes, sizeof(uint32_t));
    }

    file.close();

    last_save_time = millis();
    dirty = false;

    Serial.println("[STATS] Transfer statistics saved to flash");
    return true;
}

bool TransferStats::load() {
    if (!initialized) {
        return false;
    }

    // Check if file exists
    if (!LittleFS.exists(STATS_FILE)) {
        return false;
    }

    // Open file for reading
    File file = LittleFS.open(STATS_FILE, "r");
    if (!file) {
        Serial.println("[STATS] Failed to open file for reading");
        return false;
    }

    // Read and verify magic number
    uint32_t magic;
    if (file.read((uint8_t*)&magic, sizeof(magic)) != sizeof(magic) || magic != 0xDEADBEEF) {
        Serial.println("[STATS] Invalid magic number in stats file");
        file.close();
        return false;
    }

    // Read version
    uint8_t version;
    if (file.read(&version, 1) != 1 || version != 1) {
        Serial.println("[STATS] Unsupported stats file version");
        file.close();
        return false;
    }

    // Read stats for each peripheral
    for (int i = 0; i < 5; i++) {
        file.read((uint8_t*)&stats[i].total_packets, sizeof(uint32_t));
        file.read((uint8_t*)&stats[i].total_bytes, sizeof(uint32_t));
        // Session stats remain zero (will accumulate from this boot)
        stats[i].session_packets = 0;
        stats[i].session_bytes = 0;
    }

    file.close();
    return true;
}

PeripheralStats TransferStats::getStats(uint8_t peripheral_id) {
    int idx = getIndex(peripheral_id);
    if (idx < 0) {
        // Return empty stats for invalid ID
        PeripheralStats empty = {0, 0, 0, 0};
        return empty;
    }
    return stats[idx];
}

void TransferStats::reset() {
    for (int i = 0; i < 5; i++) {
        stats[i].total_packets = 0;
        stats[i].total_bytes = 0;
        stats[i].session_packets = 0;
        stats[i].session_bytes = 0;
    }
    dirty = true;
    save();  // Immediately save the reset
    Serial.println("[STATS] All transfer statistics reset");
}

size_t TransferStats::packStats(uint8_t* out, size_t max_len) {
    if (max_len < sizeof(WireTransferStats_t)) {
        return 0;
    }

    WireTransferStats_t w{};
    w.version = 1;

    // Pack stats for each peripheral
    for (int i = 0; i < 5; i++) {
        w.stats[i].total_packets = stats[i].total_packets;
        w.stats[i].total_bytes = stats[i].total_bytes;
    }

    memcpy(out, &w, sizeof(WireTransferStats_t));
    return sizeof(WireTransferStats_t);
}

int TransferStats::getIndex(uint8_t peripheral_id) {
    // Map peripheral ID to array index
    switch (peripheral_id) {
        case PERIPHERAL_ID_SYSTEM:      return 0;
        case PERIPHERAL_ID_LORA_915:    return 1;
        case PERIPHERAL_ID_RADIO_433:   return 2;
        case PERIPHERAL_ID_BAROMETER:   return 3;
        case PERIPHERAL_ID_CURRENT:     return 4;
        default: return -1;  // Invalid
    }
}
