/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  RTC MODULE - Implementation                                    ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "rtc_module.h"

RTCModule rtc;

RTCModule::RTCModule() {
    _error_count = 0;
    _initialized = false;
}

bool RTCModule::begin() {
    if (!_rtc.begin()) {
        Serial.println("❌ DS3231 RTC nicht gefunden!");
        return false;
    }
    
    Serial.println("✓ DS3231 RTC gefunden!");
    
    if (_rtc.lostPower()) {
        Serial.println("⚠️  RTC lost power - Zeit muss gesetzt werden!");
    }
    
    _initialized = true;
    return true;
}

bool RTCModule::isConnected() {
    return _initialized;
}

Timestamp RTCModule::getTimestamp() {
    Timestamp ts;
    
    if (!_initialized) {
        ts.valid = false;
        return ts;
    }
    
    DateTime now = _rtc.now();
    
    ts.year = now.year();
    ts.month = now.month();
    ts.day = now.day();
    ts.hour = now.hour();
    ts.minute = now.minute();
    ts.second = now.second();
    ts.millisecond = 0;  // DS3231 hat keine ms
    ts.unix = now.unixtime();
    ts.valid = true;
    
    return ts;
}

DateTime RTCModule::getDateTime() {
    return _rtc.now();
}

void RTCModule::setDateTime(DateTime dt) {
    _rtc.adjust(dt);
}

void RTCModule::syncWithGPS(const GPSData& gps) {
    if (!gps.gps_time.valid) return;
    
    DateTime gps_time(gps.gps_time.year, gps.gps_time.month, gps.gps_time.day,
                      gps.gps_time.hour, gps.gps_time.minute, gps.gps_time.second);
    
    setDateTime(gps_time);
    Serial.println("✓ RTC mit GPS synchronisiert!");
}

String RTCModule::getISOString() {
    DateTime now = _rtc.now();
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    return String(buffer);
}

String RTCModule::getFilenameString() {
    DateTime now = _rtc.now();
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%04d%02d%02d_%02d%02d%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    return String(buffer);
}

uint16_t RTCModule::getErrorCount() {
    return _error_count;
}

void RTCModule::resetErrorCount() {
    _error_count = 0;
}
