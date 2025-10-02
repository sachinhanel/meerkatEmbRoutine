// Barometer.h
#ifndef BAROMETER_H
#define BAROMETER_H

#include <Arduino.h>
#include <Wire.h>
#include <MS5611.h>
#include "config.h"

class Barometer {
private:
    MS5611 ms;
    BarometerData_t latest_reading;
    bool is_initialized;
    float sea_level_pressure_hpa;
    uint32_t last_read_time;
    SemaphoreHandle_t data_mutex;
    
public:
    Barometer();
    ~Barometer();
    
    bool begin();
    void end();
    bool isOnline() const { return is_initialized; }
    
    // Reading functions
    void update();
    bool getLatestReading(BarometerData_t& data);
    
    // Individual parameter access
    float getPressure();
    float getTemperature();
    float getAltitude();
    float getHumidity(); // If BME280 is used
    
    // Calibration
    void setSeaLevelPressure(float pressure_hpa);
    float getSeaLevelPressure() const { return sea_level_pressure_hpa; }
    
    // Status
    uint32_t getLastReadTime() const { return last_read_time; }
};

#endif // BAROMETER_H