// CurrentVoltageSensor.h
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <Arduino.h>
#include "config.h"

class CurrentVoltageSensor {
private:
    CurrentData_t latest_reading;
    bool is_initialized;
    uint32_t last_read_time;
    SemaphoreHandle_t data_mutex;
    
    // Measurement and calibration parameters
    float voltage_divider_ratio;   // Bus voltage scaling (set 1.0 if unused)
    float voltage_offset;          // Bus voltage offset (V)
    // Shunt measurement parameters
    float shunt_resistance_ohms;   // Rshunt in ohms
    float amplifier_gain;          // Gain of the shunt amplifier
    float bias_voltage_v;          // Output bias voltage at 0 A (V)
    float current_offset;          // Additional software offset (A)
    
    // Filtering
    static const int SAMPLE_COUNT = 10;
    float voltage_samples[SAMPLE_COUNT];
    float current_samples[SAMPLE_COUNT];
    int sample_index;
    bool samples_filled;
    
    float getFilteredVoltage();
    float getFilteredCurrent();
    
public:
    CurrentVoltageSensor();
    ~CurrentVoltageSensor();
    
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
    
    // Calibration and configuration functions
    void setVoltageDividerRatio(float ratio);
    void setVoltageOffset(float offset_v);
    void setShuntResistance(float r_ohms);
    void setAmplifierGain(float gain);
    void setBiasVoltage(float bias_v);
    void setCurrentOffset(float offset_a);
    void calibrateZero(); // Call with no current flowing
    
    // Status
    uint32_t getLastReadTime() const { return last_read_time; }
    int getRawADC();
};

#endif // CURRENT_SENSOR_H