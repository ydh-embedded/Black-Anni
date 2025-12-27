/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  BME280 MODULE - Atmospheric Sensor                             ║
 * ║  ⭐⭐ Temperature, Humidity, Pressure & Altitude                  ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef BME280_MODULE_H
#define BME280_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "../config.h"
#include "../data_structures.h"
#include "../calculations.h"

// ═══════════════════════════════════════════════════════════════════
// BME280 SENSOR CLASS
// ═══════════════════════════════════════════════════════════════════

class BME280Sensor {
public:
    BME280Sensor();
    
    // Initialisierung
    bool begin(uint8_t i2c_addr = I2C_ADDR_BME280);
    bool isConnected();
    
    // Measurement Update
    bool update();
    
    // Sensor Readings
    float getTemperature();      // °C
    float getHumidity();         // %
    float getPressure();         // hPa
    float getAltitude();         // m (barometric)
    
    // Derived Values
    float getDewPoint();         // °C
    float getSeaLevelPressure(float altitude_m = 0);  // hPa
    
    // Altitude Fusion (mit GPS)
    void setGPSAltitude(float gps_alt);
    float getFusedAltitude();
    
    // Get Complete Data Structure
    EnvironmentalData getData();
    
    // Configuration
    void setSeaLevelPressureReference(float pressure_hpa);
    
    // Calibration Status
    bool isMeasuring();
    
    // Error Handling
    uint16_t getErrorCount();
    void resetErrorCount();

private:
    Adafruit_BME280 _bme;
    uint8_t _i2c_addr;
    
    // Sensor Data
    float _temperature;
    float _humidity;
    float _pressure;
    float _altitude_baro;
    
    // GPS Fusion
    float _altitude_gps;
    float _altitude_fused;
    float _prev_altitude_fused;
    
    // Reference
    float _sea_level_pressure;
    
    // Timing
    uint32_t _last_update;
    
    // Error Tracking
    uint16_t _error_count;
    bool _initialized;
};

// ═══════════════════════════════════════════════════════════════════
// GLOBAL BME280 INSTANCE
// ═══════════════════════════════════════════════════════════════════

extern BME280Sensor bme280;

#endif // BME280_MODULE_H
