// CurrentSensor.h
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <Arduino.h>
#include "config.h"

class CurrentSensor {
private:
    CurrentData_t latest_reading;
    bool is_initialized;
    uint32_t last_read_time;
    SemaphoreHandle_t data_mutex;
    
    // Calibration parameters
    float voltage_divider_ratio;
    float current_sensor_sensitivity; // mV/A
    float voltage_offset;
    float current_offset;
    
    // Filtering
    static const int SAMPLE_COUNT = 10;
    float voltage_samples[SAMPLE_COUNT];
    float current_samples[SAMPLE_COUNT];
    int sample_index;
    bool samples_filled;
    
    float getFilteredVoltage();
    float getFilteredCurrent();
    
public:
    CurrentSensor();
    ~CurrentSensor();
    
    bool begin();
    void end();
    bool isOnline() const { return is_initialized; }
    
    // Reading functions
    void update();
    bool getLatestReading(CurrentData_t& data);
    
    // Individual parameter access
    float getCurrent();
    float getVoltage();
    float getPower();
    
    // Calibration functions
    void setVoltageDividerRatio(float ratio);
    void setCurrentSensorSensitivity(float sensitivity_mv_per_a);
    void setVoltageOffset(float offset_v);
    void setCurrentOffset(float offset_a);
    void calibrateZero(); // Call with no current flowing
    
    // Status
    uint32_t getLastReadTime() const { return last_read_time; }
    int getRawADC();
};

#endif // CURRENT_SENSOR_H