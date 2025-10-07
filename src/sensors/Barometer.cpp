#include "Barometer.h"
// #include "MS5611.h"

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
    if (!ms.begin()) {
        Serial.println("Barometer: MS56xx begin failed");
        return false;
    }
    
    // reset() returns bool and internally reads PROM
    if (!ms.reset()) {
        Serial.println("Barometer: MS56xx reset/PROM read failed");
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
        ms.setOversampling(OSR_ULTRA_HIGH);  // or OSR_HIGH
        int result = ms.read();
        if (result == MS5611_READ_OK) {  // Note: no MS5611:: prefix
            double temperature = ms.getTemperature(); // °C
            double pressure = ms.getPressure();       // mbar/hPa
            latest_reading.temperature_c = (float)temperature;
            latest_reading.pressure_hpa = (float)pressure;
        } else {
            // Keep previous values on read error
            Serial.printf("Barometer: Read failed with code %d\n", result);
        }

        // Compute altitude from pressure
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