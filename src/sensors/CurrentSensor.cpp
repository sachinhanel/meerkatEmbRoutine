#include "CurrentSensor.h"

CurrentSensor::CurrentSensor() : 
    is_initialized(false), 
    last_read_time(0),
    voltage_divider_ratio(11.0), // Default for common voltage divider
    current_sensor_sensitivity(66.0), // Default for ACS712-30A (66mV/A)
    voltage_offset(0.0),
    current_offset(0.0),
    sample_index(0),
    samples_filled(false) {
    
    data_mutex = xSemaphoreCreateMutex();
    memset(&latest_reading, 0, sizeof(latest_reading));
    memset(voltage_samples, 0, sizeof(voltage_samples));
    memset(current_samples, 0, sizeof(current_samples));
}

CurrentSensor::~CurrentSensor() {
    end();
    if (data_mutex != nullptr) {
        vSemaphoreDelete(data_mutex);
    }
}

bool CurrentSensor::begin() {
    // Initialize ADC
    analogSetWidth(12); // 12-bit resolution (0-4095)
    analogSetAttenuation(ADC_11db); // Full scale ~3.3V
    
    is_initialized = true;
    Serial.println("CurrentSensor: Initialized successfully");
    
    // Take initial readings to fill sample buffer
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        update();
        delay(10);
    }
    
    return true;
}

void CurrentSensor::end() {
    is_initialized = false;
}

void CurrentSensor::update() {
    if (!is_initialized) return;
    
    // Read raw ADC value
    int raw_adc = analogRead(CURRENT_SENSOR_PIN);
    
    // Convert to voltage (ESP32 ADC with 12-bit resolution and ~3.3V reference)
    float adc_voltage = (raw_adc * 3.3) / 4095.0;
    
    // Calculate actual voltage (accounting for voltage divider)
    float actual_voltage = (adc_voltage * voltage_divider_ratio) + voltage_offset;
    
    // Calculate current (assuming ACS712 or similar hall effect sensor)
    // ACS712 outputs ~2.5V at 0A, with sensitivity in mV/A
    float sensor_voltage = adc_voltage;
    float current_a = ((sensor_voltage - 2.5) * 1000.0 / current_sensor_sensitivity) + current_offset;
    
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
        
        Serial.printf("Current Sensor: V=%.3f V, I=%.3f A, P=%.3f W\n",
                      latest_reading.voltage_v,
                      latest_reading.current_a,
                      latest_reading.power_w);
    }
}

float CurrentSensor::getFilteredVoltage() {
    if (!samples_filled && sample_index < 3) return voltage_samples[0]; // Not enough samples yet
    
    float sum = 0.0;
    int count = samples_filled ? SAMPLE_COUNT : sample_index;
    
    for (int i = 0; i < count; i++) {
        sum += voltage_samples[i];
    }
    
    return sum / count;
}

float CurrentSensor::getFilteredCurrent() {
    if (!samples_filled && sample_index < 3) return current_samples[0]; // Not enough samples yet
    
    float sum = 0.0;
    int count = samples_filled ? SAMPLE_COUNT : sample_index;
    
    for (int i = 0; i < count; i++) {
        sum += current_samples[i];
    }
    
    return sum / count;
}

bool CurrentSensor::getLatestReading(CurrentData_t& data) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        data = latest_reading;
        xSemaphoreGive(data_mutex);
        return true;
    }
    return false;
}

float CurrentSensor::getCurrent() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float current = latest_reading.current_a;
        xSemaphoreGive(data_mutex);
        return current;
    }
    return -999.0;
}

float CurrentSensor::getVoltage() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float voltage = latest_reading.voltage_v;
        xSemaphoreGive(data_mutex);
        return voltage;
    }
    return -999.0;
}

float CurrentSensor::getPower() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float power = latest_reading.power_w;
        xSemaphoreGive(data_mutex);
        return power;
    }
    return -999.0;
}

void CurrentSensor::setVoltageDividerRatio(float ratio) {
    voltage_divider_ratio = ratio;
    Serial.printf("CurrentSensor: Voltage divider ratio set to %.2f\n", ratio);
}

void CurrentSensor::setCurrentSensorSensitivity(float sensitivity_mv_per_a) {
    current_sensor_sensitivity = sensitivity_mv_per_a;
    Serial.printf("CurrentSensor: Sensitivity set to %.2f mV/A\n", sensitivity_mv_per_a);
}

void CurrentSensor::setVoltageOffset(float offset_v) {
    voltage_offset = offset_v;
    Serial.printf("CurrentSensor: Voltage offset set to %.3f V\n", offset_v);
}

void CurrentSensor::setCurrentOffset(float offset_a) {
    current_offset = offset_a;
    Serial.printf("CurrentSensor: Current offset set to %.3f A\n", offset_a);
}

void CurrentSensor::calibrateZero() {
    if (!is_initialized) return;
    
    Serial.println("CurrentSensor: Calibrating zero point...");
    
    // Take several readings to get average zero point
    float voltage_sum = 0.0;
    float current_sum = 0.0;
    const int cal_samples = 50;
    
    for (int i = 0; i < cal_samples; i++) {
        int raw_adc = analogRead(CURRENT_SENSOR_PIN);
        float adc_voltage = (raw_adc * 3.3) / 4095.0;
        
        voltage_sum += adc_voltage;
        current_sum += ((adc_voltage - 2.5) * 1000.0 / current_sensor_sensitivity);
        
        delay(20);
    }
    
    float avg_current_raw = current_sum / cal_samples;
    current_offset = -avg_current_raw; // Offset to make current read 0A
    
    Serial.printf("CurrentSensor: Zero calibration complete. Current offset: %.4f A\n", current_offset);
}

int CurrentSensor::getRawADC() {
    return is_initialized ? analogRead(CURRENT_SENSOR_PIN) : -1;
}