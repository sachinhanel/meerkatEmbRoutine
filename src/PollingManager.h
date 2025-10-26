// PollingManager.h
#ifndef POLLING_MANAGER_H
#define POLLING_MANAGER_H

#include <Arduino.h>

// Forward declarations for polling functions defined in main.cpp
void addToPollList(uint8_t peripheral_id, uint16_t interval_ms);
void removeFromPollList(uint8_t peripheral_id);

#endif // POLLING_MANAGER_H
