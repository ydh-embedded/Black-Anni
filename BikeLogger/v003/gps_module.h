/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  GPS MODULE - NEO-M9N (Multi-Band L1/L5)                        ║
 * ║  ⭐⭐⭐ Präzise Position, Geschwindigkeit & Heading                ║
 * ║                                                                  ║
 * ║  Features:                                                       ║
 * ║  - Multi-Band GPS L1 + L5 für höhere Genauigkeit               ║
 * ║  - 10Hz Update Rate (100ms zwischen Fixes)                      ║
 * ║  - Position (Lat/Lon/Alt)                                       ║
 * ║  - Speed & Heading                                              ║
 * ║  - Satellite Count & HDOP                                       ║
 * ║  - Date/Time (UTC)                                              ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <TinyGPS++.h>  // TinyGPS++ Library für NMEA Parsing
#include "../config.h"
#include "../data_structures.h"
#include "../calculations.h"

// ═══════════════════════════════════════════════════════════════════
// NEO-M9N CLASS
// ═══════════════════════════════════════════════════════════════════

class NEOM9N {
public:
    NEOM9N();
    
    // Initialisierung
    bool begin(HardwareSerial& serial = GPS_SERIAL, uint32_t baud = GPS_BAUD);
    bool isConnected();
    
    // Konfiguration
    void setUpdateRate(uint8_t rate_hz);  // 1-10 Hz
    void enableMultiBand(bool enable);     // L1 + L5
    void setDynamicModel(uint8_t model);   // Automotive, Airborne, etc.
    
    // Daten Update
    bool update();  // Liest alle verfügbaren Bytes
    bool hasNewData();
    uint32_t getLastUpdateMillis();
    
    // Position
    double getLatitude();
    double getLongitude();
    float getAltitude();          // GPS Altitude (m)
    bool isLocationValid();
    
    // Speed & Heading
    float getSpeedKmh();          // Geschwindigkeit (km/h)
    float getSpeedMs();           // Geschwindigkeit (m/s)
    float getHeading();           // Richtung (0-359°, 0=Nord)
    bool isHeadingValid();
    
    // Quality Indicators
    uint8_t getSatellites();      // Anzahl Satelliten
    float getHDOP();               // Horizontal Dilution of Precision
    uint8_t getFixType();         // 0=No Fix, 1=2D, 2=3D
    bool hasFix();
    
    // Time & Date
    uint8_t getHour();
    uint8_t getMinute();
    uint8_t getSecond();
    uint16_t getMillisecond();
    uint8_t getDay();
    uint8_t getMonth();
    uint16_t getYear();
    bool isTimeValid();
    bool isDateValid();
    
    // Distance & Bearing Calculations
    float distanceTo(double lat, double lon);  // Distanz zu Punkt (m)
    float bearingTo(double lat, double lon);   // Richtung zu Punkt (°)
    
    // Statistics
    uint32_t getCharsProcessed();
    uint32_t getSentencesWithFix();
    uint32_t getFailedChecksum();
    uint32_t getPassedChecksum();
    
    // Get Complete Data Structure
    GPSData getData();
    
    // Diagnostic
    void printDebugInfo();
    
    // Error Handling
    uint16_t getErrorCount();
    void resetErrorCount();

private:
    // TinyGPS++ Parser
    TinyGPSPlus _gps;
    
    // Serial Interface
    HardwareSerial* _serial;
    uint32_t _baud;
    
    // Cached Data
    GPSData _data;
    
    // Previous Position (für Distanz-Berechnung)
    double _prev_lat;
    double _prev_lon;
    uint32_t _prev_fix_millis;
    float _trip_distance;
    
    // Timing
    uint32_t _last_update;
    uint32_t _last_fix_time;
    bool _new_data;
    
    // Configuration
    uint8_t _update_rate_hz;
    bool _multiband_enabled;
    
    // Error Tracking
    uint16_t _error_count;
    uint32_t _timeout_count;
    
    // UBX Configuration
    bool sendUBX(uint8_t* packet, uint16_t len);
    void calculateUBXChecksum(uint8_t* packet, uint16_t len);
    bool configureUpdateRate(uint8_t rate_hz);
    bool configureMultiBand(bool enable);
};

// ═══════════════════════════════════════════════════════════════════
// GLOBAL GPS INSTANCE
// ═══════════════════════════════════════════════════════════════════

extern NEOM9N gps;

// ═══════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

// Quick Access Functions
inline bool hasGPSFix() { return gps.hasFix(); }
inline float getGPSSpeed() { return gps.getSpeedKmh(); }
inline float getGPSHeading() { return gps.getHeading(); }
inline uint8_t getGPSSatellites() { return gps.getSatellites(); }

// Position Formatting
String formatLatitude(double lat, uint8_t decimals = 6);
String formatLongitude(double lon, uint8_t decimals = 6);
String formatCoordinates(double lat, double lon, uint8_t decimals = 6);

#endif // GPS_MODULE_H
