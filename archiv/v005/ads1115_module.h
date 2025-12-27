/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  ADS1115 MODULE - 16-bit ADC                                    ║
 * ║  📊 4-Channel Analog Inputs (Throttle, Brake, Clutch, Reserve) ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef ADS1115_MODULE_H
#define ADS1115_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "../config.h"
#include "../data_structures.h"

class ADS1115Module {
public:
    ADS1115Module();
    bool begin(uint8_t i2c_addr = I2C_ADDR_ADS1115);
    bool isConnected();
    bool update();
    
    // Get Analog Inputs Data
    AnalogInputsData getData();
    
    uint16_t getErrorCount();
    void resetErrorCount();

private:
    Adafruit_ADS1115 _ads;
    uint8_t _i2c_addr;
    AnalogInputsData _data;
    uint16_t _error_count;
    bool _initialized;
};

extern ADS1115Module ads1115;

#endif
