/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  INA260 MODULE - Implementation                                 ║
 * ║  ⚡ Power Monitoring & Battery Status                           ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "ina260_module.h"

// Global INA260 Instance
INA260PowerMonitor ina260;

// ═══════════════════════════════════════════════════════════════════
// CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════════

INA260PowerMonitor::INA260PowerMonitor() {
    _i2c_addr = I2C_ADDR_INA260;
    _voltage = 0;
    _current = 0;
    _power = 0;
    _battery_percentage = 100;
    _battery_low = false;
    _battery_critical = false;
    _energy_consumed = 0;
    _power_sum = 0;
    _power_count = 0;
    _session_start = 0;
    _last_update = 0;
    _dt = 0;
    _error_count = 0;
    _initialized = false;
}

// ═══════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

bool INA260PowerMonitor::begin(uint8_t i2c_addr) {
    _i2c_addr = i2c_addr;
    
    // Initialize INA260
    if (!_ina.begin(_i2c_addr)) {
        Serial.println("❌ INA260 nicht gefunden!");
        return false;
    }
    
    Serial.println("✓ INA260 gefunden!");
    
    // Configure Averaging (16 samples)
    _ina.setAveragingCount(INA260_COUNT_16);
    
    // Configure Conversion Time (1.1ms)
    _ina.setVoltageConversionTime(INA260_TIME_1_1_ms);
    _ina.setCurrentConversionTime(INA260_TIME_1_1_ms);
    
    delay(100);
    
    _initialized = true;
    _session_start = millis();
    
    return true;
}

bool INA260PowerMonitor::isConnected() {
    return _initialized;
}

// ═══════════════════════════════════════════════════════════════════
// MEASUREMENT UPDATE
// ═══════════════════════════════════════════════════════════════════

bool INA260PowerMonitor::update() {
    if (!_initialized) return false;
    
    // Calculate delta time
    uint32_t now = millis();
    if (_last_update > 0) {
        _dt = (now - _last_update) / 1000.0;  // seconds
    }
    _last_update = now;
    
    // Read Sensor
    _voltage = _ina.readBusVoltage() / 1000.0;  // mV to V
    _current = _ina.readCurrent() / 1000.0;     // mA to A
    _power = _ina.readPower() / 1000.0;         // mW to W
    
    // Check for invalid readings
    if (isnan(_voltage) || isnan(_current) || isnan(_power)) {
        _error_count++;
        return false;
    }
    
    // Calculate Battery Percentage (simple voltage-based)
    _battery_percentage = calculateBatteryPercentage(_voltage);
    
    // Battery Status Flags
    _battery_low = (_voltage < BATTERY_VOLTAGE_LOW);
    _battery_critical = (_voltage < BATTERY_VOLTAGE_CRITICAL);
    
    // Energy Consumption (Wh)
    if (_dt > 0) {
        _energy_consumed += (_power * _dt) / 3600.0;  // Wh
    }
    
    // Average Power
    _power_sum += _power;
    _power_count++;
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// MEASUREMENTS
// ═══════════════════════════════════════════════════════════════════

float INA260PowerMonitor::getVoltage() {
    return _voltage;
}

float INA260PowerMonitor::getCurrent() {
    return _current;
}

float INA260PowerMonitor::getPower() {
    return _power;
}

// ═══════════════════════════════════════════════════════════════════
// BATTERY STATUS
// ═══════════════════════════════════════════════════════════════════

float INA260PowerMonitor::getBatteryPercentage() {
    return _battery_percentage;
}

bool INA260PowerMonitor::isBatteryLow() {
    return _battery_low;
}

bool INA260PowerMonitor::isBatteryCritical() {
    return _battery_critical;
}

// ═══════════════════════════════════════════════════════════════════
// STATISTICS
// ═══════════════════════════════════════════════════════════════════

float INA260PowerMonitor::getEnergyConsumed() {
    return _energy_consumed;
}

float INA260PowerMonitor::getAveragePower() {
    if (_power_count == 0) return 0;
    return _power_sum / _power_count;
}

void INA260PowerMonitor::resetStatistics() {
    _energy_consumed = 0;
    _power_sum = 0;
    _power_count = 0;
    _session_start = millis();
}

// ═══════════════════════════════════════════════════════════════════
// GET DATA STRUCTURE
// ═══════════════════════════════════════════════════════════════════

PowerData INA260PowerMonitor::getData() {
    PowerData data;
    
    data.voltage = _voltage;
    data.current = _current;
    data.power = _power;
    data.battery_percentage = _battery_percentage;
    data.battery_low = _battery_low;
    data.battery_critical = _battery_critical;
    data.energy_consumed = _energy_consumed;
    data.avg_power = getAveragePower();
    data.millis_at_reading = millis();
    
    return data;
}

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

void INA260PowerMonitor::setAveragingMode(uint8_t avg) {
    // Map avg count to INA260 enum
    INA260_AveragingCount count;
    switch(avg) {
        case 1:    count = INA260_COUNT_1;    break;
        case 4:    count = INA260_COUNT_4;    break;
        case 16:   count = INA260_COUNT_16;   break;
        case 64:   count = INA260_COUNT_64;   break;
        case 128:  count = INA260_COUNT_128;  break;
        case 256:  count = INA260_COUNT_256;  break;
        case 512:  count = INA260_COUNT_512;  break;
        case 1024: count = INA260_COUNT_1024; break;
        default:   count = INA260_COUNT_16;   break;
    }
    
    _ina.setAveragingCount(count);
}

void INA260PowerMonitor::setConversionTime(uint8_t time_us) {
    // Map time to INA260 enum
    INA260_ConversionTime time;
    switch(time_us) {
        case 140:  time = INA260_TIME_140_us; break;
        case 204:  time = INA260_TIME_204_us; break;
        case 332:  time = INA260_TIME_332_us; break;
        case 588:  time = INA260_TIME_588_us; break;
        case 1100: time = INA260_TIME_1_1_ms; break;
        case 2116: time = INA260_TIME_2_116_ms; break;
        case 4156: time = INA260_TIME_4_156_ms; break;
        case 8244: time = INA260_TIME_8_244_ms; break;
        default:   time = INA260_TIME_1_1_ms; break;
    }
    
    _ina.setVoltageConversionTime(time);
    _ina.setCurrentConversionTime(time);
}

// ═══════════════════════════════════════════════════════════════════
// ERROR HANDLING
// ═══════════════════════════════════════════════════════════════════

uint16_t INA260PowerMonitor::getErrorCount() {
    return _error_count;
}

void INA260PowerMonitor::resetErrorCount() {
    _error_count = 0;
}
