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

    // Initialize MS56xx (MS5611 lib supports MS5607)
    ms.begin();
    if (ms.reset() != MS5611_OK) {
        Serial.println("Barometer: MS56xx reset failed");
        return false;
    }
    // Read factory calibration PROM
    if (ms.readPROM() != MS5611_OK) {
        Serial.println("Barometer: MS56xx PROM read failed");
        return false;
    }

    is_initialized = true;
    Serial.println("Barometer: MS56xx initialized successfully");

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

        // Perform D1 and D2 conversions with high OSR for better precision
        ms.setOversampling(MS5611_OSR_4096);
        if (ms.read() == MS5611_OK) {
            double temperature = ms.getTemperature(); // °C
            double pressure = ms.getPressure();       // mbar/hPa
            latest_reading.temperature_c = (float)temperature;
            latest_reading.pressure_hpa = (float)pressure;
        } else {
            // Keep previous values on read error
        }

        // Compute altitude from pressure
        // Barometric formula approximation
        // h = 44330 * (1 - (P/Pref)^(1/5.255))
        float ratio = latest_reading.pressure_hpa / sea_level_pressure_hpa;
        latest_reading.altitude_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));
        
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
    // MS5607 does not measure humidity
    return -1.0f;
}

void Barometer::setSeaLevelPressure(float pressure_hpa) {
    sea_level_pressure_hpa = pressure_hpa;
    Serial.printf("Barometer: Sea level pressure set to %.2f hPa\n", pressure_hpa);
}