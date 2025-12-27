/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  ADS1115 MODULE - Implementation                                ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "ads1115_module.h"

ADS1115Module ads1115;

ADS1115Module::ADS1115Module() {
    _i2c_addr = I2C_ADDR_ADS1115;
    _error_count = 0;
    _initialized = false;
}

bool ADS1115Module::begin(uint8_t i2c_addr) {
    _i2c_addr = i2c_addr;
    
    if (!_ads.begin(_i2c_addr)) {
        Serial.println("❌ ADS1115 nicht gefunden!");
        return false;
    }
    
    Serial.println("✓ ADS1115 gefunden!");
    
    // Set Gain (±6.144V range)
    _ads.setGain(GAIN_TWOTHIRDS);
    
    // Set Data Rate (860 SPS)
    _ads.setDataRate(RATE_ADS1115_860SPS);
    
    _initialized = true;
    return true;
}

bool ADS1115Module::isConnected() {
    return _initialized;
}

bool ADS1115Module::update() {
    if (!_initialized) return false;
    
    // Read all 4 channels
    _data.adc0_raw = _ads.readADC_SingleEnded(0);
    _data.adc1_raw = _ads.readADC_SingleEnded(1);
    _data.adc2_raw = _ads.readADC_SingleEnded(2);
    _data.adc3_raw = _ads.readADC_SingleEnded(3);
    
    // Convert to voltage (±6.144V range, 15-bit)
    float voltsPerBit = 0.1875 / 1000.0;  // mV per bit
    _data.adc0_voltage = _data.adc0_raw * voltsPerBit;
    _data.adc1_voltage = _data.adc1_raw * voltsPerBit;
    _data.adc2_voltage = _data.adc2_raw * voltsPerBit;
    _data.adc3_voltage = _data.adc3_raw * voltsPerBit;
    
    // Map to 0-100%
    float vmax = 5.0;  // Assuming 5V inputs
    _data.throttle_percent = constrain((_data.adc0_voltage / vmax) * 100.0, 0, 100);
    _data.brake_percent = constrain((_data.adc1_voltage / vmax) * 100.0, 0, 100);
    _data.clutch_percent = constrain((_data.adc2_voltage / vmax) * 100.0, 0, 100);
    _data.reserve_percent = constrain((_data.adc3_voltage / vmax) * 100.0, 0, 100);
    
    _data.millis_at_reading = millis();
    
    return true;
}

AnalogInputsData ADS1115Module::getData() {
    return _data;
}

uint16_t ADS1115Module::getErrorCount() {
    return _error_count;
}

void ADS1115Module::resetErrorCount() {
    _error_count = 0;
}
