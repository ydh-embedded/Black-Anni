/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  BME280 MODULE - Implementation                                 ║
 * ║  🌡️ Atmospheric Sensor Measurements                             ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "bme280_module.h"

// Global BME280 Instance
BME280Sensor bme280;

// ═══════════════════════════════════════════════════════════════════
// CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════════

BME280Sensor::BME280Sensor() {
    _i2c_addr = I2C_ADDR_BME280;
    _temperature = 0;
    _humidity = 0;
    _pressure = 0;
    _altitude_baro = 0;
    _altitude_gps = 0;
    _altitude_fused = 0;
    _prev_altitude_fused = 0;
    _sea_level_pressure = 1013.25;  // Standard sea level pressure
    _last_update = 0;
    _error_count = 0;
    _initialized = false;
}

// ═══════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

bool BME280Sensor::begin(uint8_t i2c_addr) {
    _i2c_addr = i2c_addr;
    
    // Initialize BME280
    if (!_bme.begin(_i2c_addr)) {
        Serial.println("❌ BME280 nicht gefunden!");
        return false;
    }
    
    Serial.println("✓ BME280 gefunden!");
    
    // Configure Sensor
    _bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,     // Normal mode (continuous)
        Adafruit_BME280::SAMPLING_X2,     // Temperature oversampling x2
        Adafruit_BME280::SAMPLING_X16,    // Pressure oversampling x16
        Adafruit_BME280::SAMPLING_X1,     // Humidity oversampling x1
        Adafruit_BME280::FILTER_X16,      // IIR filter x16
        Adafruit_BME280::STANDBY_MS_1000  // 1s standby
    );
    
    delay(100);
    
    _initialized = true;
    
    return true;
}

bool BME280Sensor::isConnected() {
    return _initialized;
}

// ═══════════════════════════════════════════════════════════════════
// MEASUREMENT UPDATE
// ═══════════════════════════════════════════════════════════════════

bool BME280Sensor::update() {
    if (!_initialized) return false;
    
    // Read Sensor
    _temperature = _bme.readTemperature();
    _humidity = _bme.readHumidity();
    _pressure = _bme.readPressure() / 100.0;  // Pa to hPa
    
    // Check for invalid readings
    if (isnan(_temperature) || isnan(_humidity) || isnan(_pressure)) {
        _error_count++;
        return false;
    }
    
    // Calculate Barometric Altitude
    _altitude_baro = calculateAltitudeFromPressure(_pressure, _sea_level_pressure);
    
    // Fuse with GPS Altitude (if available)
    if (_altitude_gps > 0) {
        _altitude_fused = fuseAltitude(_altitude_baro, _altitude_gps, 
                                       _prev_altitude_fused, 0.95);
        _prev_altitude_fused = _altitude_fused;
    } else {
        _altitude_fused = _altitude_baro;
    }
    
    _last_update = millis();
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// SENSOR READINGS
// ═══════════════════════════════════════════════════════════════════

float BME280Sensor::getTemperature() {
    return _temperature;
}

float BME280Sensor::getHumidity() {
    return _humidity;
}

float BME280Sensor::getPressure() {
    return _pressure;
}

float BME280Sensor::getAltitude() {
    return _altitude_baro;
}

// ═══════════════════════════════════════════════════════════════════
// DERIVED VALUES
// ═══════════════════════════════════════════════════════════════════

float BME280Sensor::getDewPoint() {
    return calculateDewPoint(_temperature, _humidity);
}

float BME280Sensor::getSeaLevelPressure(float altitude_m) {
    return calculateSeaLevelPressure(_pressure, altitude_m);
}

// ═══════════════════════════════════════════════════════════════════
// ALTITUDE FUSION
// ═══════════════════════════════════════════════════════════════════

void BME280Sensor::setGPSAltitude(float gps_alt) {
    _altitude_gps = gps_alt;
}

float BME280Sensor::getFusedAltitude() {
    return _altitude_fused;
}

// ═══════════════════════════════════════════════════════════════════
// GET DATA STRUCTURE
// ═══════════════════════════════════════════════════════════════════

EnvironmentalData BME280Sensor::getData() {
    EnvironmentalData data;
    
    data.temperature = _temperature;
    data.humidity = _humidity;
    data.pressure = _pressure;
    data.altitude_baro = _altitude_baro;
    data.altitude_gps = _altitude_gps;
    data.altitude_fused = _altitude_fused;
    data.dew_point = getDewPoint();
    data.sea_level_pressure = _sea_level_pressure;
    data.millis_at_reading = millis();
    
    return data;
}

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

void BME280Sensor::setSeaLevelPressureReference(float pressure_hpa) {
    _sea_level_pressure = pressure_hpa;
}

bool BME280Sensor::isMeasuring() {
    return _initialized;
}

// ═══════════════════════════════════════════════════════════════════
// ERROR HANDLING
// ═══════════════════════════════════════════════════════════════════

uint16_t BME280Sensor::getErrorCount() {
    return _error_count;
}

void BME280Sensor::resetErrorCount() {
    _error_count = 0;
}
