/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  ADS1115 MODULE - Implementation (FIXED)                        ║
 * ║  FIXME: Critical Bug #5 - Voltage conversion corrected          ║
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
    
    // FIXME: Critical Bug #5 - Use correct voltage conversion
    // ADS1115 with GAIN_TWOTHIRDS (±6.144V range):
    // - 15-bit resolution (32768 steps)
    // - Voltage per bit: 6.144V / 32768 = 0.0001875 V/bit
    const float VOLTS_PER_BIT = 6.144 / 32768.0;
    
    _data.adc0_voltage = _data.adc0_raw * VOLTS_PER_BIT;
    _data.adc1_voltage = _data.adc1_raw * VOLTS_PER_BIT;
    _data.adc2_voltage = _data.adc2_raw * VOLTS_PER_BIT;
    _data.adc3_voltage = _data.adc3_raw * VOLTS_PER_BIT;
    
    // FIXME: Critical Bug #5 - Use actual ADC range (6.144V) not assumed 5V
    // This prevents throttle/brake/clutch readings from exceeding 100%
    const float VMAX = 6.144;  // Use actual ADC range, not assumed 5V
    
    _data.throttle_percent = constrain((_data.adc0_voltage / VMAX) * 100.0, 0, 100);
    _data.brake_percent = constrain((_data.adc1_voltage / VMAX) * 100.0, 0, 100);
    _data.clutch_percent = constrain((_data.adc2_voltage / VMAX) * 100.0, 0, 100);
    _data.reserve_percent = constrain((_data.adc3_voltage / VMAX) * 100.0, 0, 100);
    
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
