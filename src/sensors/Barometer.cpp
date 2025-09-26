#include "Barometer.h"

Barometer::Barometer() : is_initialized(false), sea_level_pressure_hpa(1013.25), last_read_time(0) {
    data_mutex = xSemaphoreCreateMutex();
    memset(&latest_reading, 0, sizeof(latest_reading));
}

Barometer::~Barometer() {
    end();
    if (data_mutex != nullptr) {
        vSemaphoreDelete(data_mutex);
    }
}

bool Barometer::begin() {
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Try to initialize BME280 - try default address first, then alternate
    if (!bme.begin(0x77, &Wire)) {  // Default BME280 address
        if (!bme.begin(0x76, &Wire)) {  // Alternate BME280 address
            Serial.println("Barometer: Could not find BME280 sensor");
            return false;
        }
    }
    
    // Set up oversampling and filtering
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,     // Operating mode
                    Adafruit_BME280::SAMPLING_X2,     // Temperature oversampling
                    Adafruit_BME280::SAMPLING_X16,    // Pressure oversampling
                    Adafruit_BME280::SAMPLING_X1,     // Humidity oversampling
                    Adafruit_BME280::FILTER_X16,      // Filtering
                    Adafruit_BME280::STANDBY_MS_500); // Standby time
    
    is_initialized = true;
    Serial.println("Barometer: BME280 initialized successfully");
    
    // Take initial reading
    update();
    return true;
}

void Barometer::end() {
    is_initialized = false;
}

void Barometer::update() {
    if (!is_initialized) return;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        latest_reading.timestamp = millis();
        latest_reading.pressure_hpa = bme.readPressure() / 100.0F; // Convert Pa to hPa
        latest_reading.temperature_c = bme.readTemperature();
        latest_reading.altitude_m = bme.readAltitude(sea_level_pressure_hpa);
        
        last_read_time = millis();
        
        xSemaphoreGive(data_mutex);
        
        Serial.printf("Barometer: P=%.2f hPa, T=%.2f°C, Alt=%.2f m\n",
                      latest_reading.pressure_hpa, 
                      latest_reading.temperature_c,
                      latest_reading.altitude_m);
    }
}

bool Barometer::getLatestReading(BarometerData_t& data) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        data = latest_reading;
        xSemaphoreGive(data_mutex);
        return true;
    }
    return false;
}

float Barometer::getPressure() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float pressure = latest_reading.pressure_hpa;
        xSemaphoreGive(data_mutex);
        return pressure;
    }
    return -1.0;
}

float Barometer::getTemperature() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float temp = latest_reading.temperature_c;
        xSemaphoreGive(data_mutex);
        return temp;
    }
    return -999.0;
}

float Barometer::getAltitude() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        float alt = latest_reading.altitude_m;
        xSemaphoreGive(data_mutex);
        return alt;
    }
    return -999.0;
}

float Barometer::getHumidity() {
    return is_initialized ? bme.readHumidity() : -1.0;
}

void Barometer::setSeaLevelPressure(float pressure_hpa) {
    sea_level_pressure_hpa = pressure_hpa;
    Serial.printf("Barometer: Sea level pressure set to %.2f hPa\n", pressure_hpa);
}