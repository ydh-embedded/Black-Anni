/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  INA260 MODULE - Power Monitor                                  ║
 * ║  ⚡ Voltage, Current & Power Measurement                        ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef INA260_MODULE_H
#define INA260_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include "../config.h"
#include "../data_structures.h"
#include "../calculations.h"

// ═══════════════════════════════════════════════════════════════════
// INA260 POWER MONITOR CLASS
// ═══════════════════════════════════════════════════════════════════

class INA260PowerMonitor {
public:
    INA260PowerMonitor();
    
    // Initialisierung
    bool begin(uint8_t i2c_addr = I2C_ADDR_INA260);
    bool isConnected();
    
    // Measurement Update
    bool update();
    
    // Measurements
    float getVoltage();          // V
    float getCurrent();          // A
    float getPower();            // W
    
    // Battery Status
    float getBatteryPercentage();
    bool isBatteryLow();
    bool isBatteryCritical();
    
    // Statistics
    float getEnergyConsumed();   // Wh
    float getAveragePower();     // W
    void resetStatistics();
    
    // Get Complete Data Structure
    PowerData getData();
    
    // Configuration
    void setAveragingMode(uint8_t avg);  // 1, 4, 16, 64, 128, 256, 512, 1024
    void setConversionTime(uint8_t time_us);
    
    // Error Handling
    uint16_t getErrorCount();
    void resetErrorCount();

private:
    Adafruit_INA260 _ina;
    uint8_t _i2c_addr;
    
    // Measurements
    float _voltage;
    float _current;
    float _power;
    
    // Battery Status
    float _battery_percentage;
    bool _battery_low;
    bool _battery_critical;
    
    // Statistics
    float _energy_consumed;      // Wh
    float _power_sum;
    uint32_t _power_count;
    uint32_t _session_start;
    
    // Timing
    uint32_t _last_update;
    float _dt;
    
    // Error Tracking
    uint16_t _error_count;
    bool _initialized;
};

// ═══════════════════════════════════════════════════════════════════
// GLOBAL INA260 INSTANCE
// ═══════════════════════════════════════════════════════════════════

extern INA260PowerMonitor ina260;

#endif // INA260_MODULE_H
