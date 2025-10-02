#include "CurrentVoltageSensor.h"
#include <Arduino.h>

CurrentVoltageSensor::CurrentVoltageSensor() : 
    is_initialized(false), 
    last_read_time(0),
    voltage_divider_ratio(1.0),
    voltage_offset(0.0),
    shunt_resistance_ohms(0.01f),
    amplifier_gain(50.0f),
    bias_voltage_v(0.0f),
    current_offset(0.0),
    sample_index(0),
    samples_filled(false) {
    
    data_mutex = xSemaphoreCreateMutex();
    memset(&latest_reading, 0, sizeof(latest_reading));
    memset(voltage_samples, 0, sizeof(voltage_samples));
    memset(current_samples, 0, sizeof(current_samples));
}

CurrentVoltageSensor::~CurrentVoltageSensor() {
    end();
    if (data_mutex != nullptr) {
        vSemaphoreDelete(data_mutex);
    }
}

bool CurrentVoltageSensor::begin() {
    // Initialize ADC
    //analogSetWidth(12); // 12-bit resolution (0-4095)    REENABLE LATER
    analogSetAttenuation(ADC_11db); // Full scale ~3.3V
    
    is_initialized = true;
    Serial.println("CurrentVoltageSensor: Initialized (shunt mode)");
    
    // Take initial readings to fill sample buffer
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        update();
        delay(10);
    }
    
    return true;
}

void CurrentVoltageSensor::end() {
    is_initialized = false;
}

void CurrentVoltageSensor::update() {
    if (!is_initialized) return;
    
    // Read raw ADC value
    int raw_adc = analogRead(CURRENT_SENSOR_PIN);
    
    // Convert to voltage (ESP32 ADC with 12-bit resolution and ~3.3V reference)
    float adc_voltage = (raw_adc * 3.3) / 4095.0;
    
    // Calculate bus voltage if this ADC is wired through a divider for Vbus measurement
    float actual_voltage = (adc_voltage * voltage_divider_ratio) + voltage_offset;
    
    // Shunt-based current: I = (Vadc - Vbias) / (Gain * Rshunt)
    float sensed_v = adc_voltage - bias_voltage_v;
    float denom = (amplifier_gain * shunt_resistance_ohms);
    float current_a = (denom != 0.0f) ? (sensed_v / denom) + current_offset : 0.0f;
    
    // Add to sample arrays for filtering
    voltage_samples[sample_index] = actual_voltage;
    current_samples[sample_index] = current_a;
    sample_index = (sample_index + 1) % SAMPLE_COUNT;
    
    if (sample_index == 0) {
        samples_filled = true;
    }
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        latest_reading.timestamp = millis();
        latest_reading.voltage_v = getFilteredVoltage();
        latest_reading.current_a = getFilteredCurrent();
        latest_reading.power_w = latest_reading.voltage_v * latest_reading.current_a;
        
        last_read_time = millis();
        xSemaphoreGive(data_mutex);
        
        Serial.printf("CurrentVoltageSensor: V=%.3f V, I=%.3f A, P=%.3f W\n",
                      latest_reading.voltage_v,
                      latest_reading.current_a,
                      latest_reading.power_w);
    }
}

float CurrentVoltageSensor::getFilteredVoltage() {
    if (!samples_filled && sample_index < 3) return voltage_samples[0]; // Not enough samples yet
    
    float sum = 0.0;
    int count = samples_filled ? SAMPLE_COUNT : sample_index;
    
    for (int i = 0; i < count; i++) {
        sum += voltage_samples[i];
    }
    
    return sum / count;
}

float CurrentVoltageSensor::getFilteredCurrent() {
    if (!samples_filled && sample_index < 3) return current_samples[0]; // Not enough samples yet
    
    float sum = 0.0;
    int count = samples_filled ? SAMPLE_COUNT : sample_index;
    
    for (int i = 0; i < count; i++) {
        sum += current_samples[i];
    }
    
    return sum / count;
}

bool CurrentVoltageSensor::getLatestReading(CurrentData_t& data) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        data = latest_reading;
        xSemaphoreGive(data_mutex);
        return true;
    }
    return false;
}

float CurrentVoltageSensor::getCurrent() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float current = latest_reading.current_a;
        xSemaphoreGive(data_mutex);
        return current;
    }
    return -999.0;
}

float CurrentVoltageSensor::getVoltage() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float voltage = latest_reading.voltage_v;
        xSemaphoreGive(data_mutex);
        return voltage;
    }
    return -999.0;
}

float CurrentVoltageSensor::getPower() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float power = latest_reading.power_w;
        xSemaphoreGive(data_mutex);
        return power;
    }
    return -999.0;
}

void CurrentVoltageSensor::setVoltageDividerRatio(float ratio) {
    voltage_divider_ratio = ratio;
    Serial.printf("CurrentVoltageSensor: Voltage divider ratio set to %.2f\n", ratio);
}

void CurrentVoltageSensor::setVoltageOffset(float offset_v) {
    voltage_offset = offset_v;
    Serial.printf("CurrentVoltageSensor: Voltage offset set to %.3f V\n", offset_v);
}

void CurrentVoltageSensor::setShuntResistance(float r_ohms) {
    shunt_resistance_ohms = r_ohms;
    Serial.printf("CurrentVoltageSensor: Shunt resistance set to %.6f ohms\n", r_ohms);
}

void CurrentVoltageSensor::setAmplifierGain(float gain) {
    amplifier_gain = gain;
    Serial.printf("CurrentVoltageSensor: Amplifier gain set to %.3f\n", gain);
}

void CurrentVoltageSensor::setBiasVoltage(float bias_v) {
    bias_voltage_v = bias_v;
    Serial.printf("CurrentVoltageSensor: Bias voltage set to %.3f V\n", bias_v);
}

void CurrentVoltageSensor::setCurrentOffset(float offset_a) {
    current_offset = offset_a;
    Serial.printf("CurrentVoltageSensor: Current offset set to %.3f A\n", offset_a);
}

void CurrentVoltageSensor::calibrateZero() {
    if (!is_initialized) return;
    
    Serial.println("CurrentVoltageSensor: Calibrating zero point...");
    
    // Take several readings to get average zero point
    float bias_sum = 0.0f;
    float current_sum = 0.0f;
    const int cal_samples = 50;
    
    for (int i = 0; i < cal_samples; i++) {
        int raw_adc = analogRead(CURRENT_SENSOR_PIN);
        float adc_voltage = (raw_adc * 3.3) / 4095.0;
        bias_sum += adc_voltage;
        float denom = (amplifier_gain * shunt_resistance_ohms);
        float zero_current = (denom != 0.0f) ? ((adc_voltage - bias_voltage_v) / denom) : 0.0f;
        current_sum += zero_current;
        
        delay(20);
    }
    
    float avg_bias = bias_sum / cal_samples;
    bias_voltage_v = avg_bias; // learn actual bias
    float avg_current_raw = current_sum / cal_samples;
    current_offset = -avg_current_raw; // additional trim to make 0 A
    
    Serial.printf("CurrentVoltageSensor: Zero calibration complete. Bias: %.3f V, Current offset: %.4f A\n", bias_voltage_v, current_offset);
}

int CurrentVoltageSensor::getRawADC() {
    return is_initialized ? analogRead(CURRENT_SENSOR_PIN) : -1;
}