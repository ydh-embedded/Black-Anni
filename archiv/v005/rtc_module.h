/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  RTC MODULE - DS3231 Real Time Clock                            ║
 * ║  🕐 Accurate Timestamps for Logging                             ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef RTC_MODULE_H
#define RTC_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include "../config.h"
#include "../data_structures.h"

class RTCModule {
public:
    RTCModule();
    bool begin();
    bool isConnected();
    
    // Time/Date
    Timestamp getTimestamp();
    DateTime getDateTime();
    void setDateTime(DateTime dt);
    void syncWithGPS(const GPSData& gps);
    
    // Formatting
    String getISOString();
    String getFilenameString();  // YYYYMMDD_HHMMSS
    
    uint16_t getErrorCount();
    void resetErrorCount();

private:
    RTC_DS3231 _rtc;
    uint16_t _error_count;
    bool _initialized;
};

extern RTCModule rtc;

#endif
