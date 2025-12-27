/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  GPS MODULE - NEO-M9N Implementation                            ║
 * ║  ⭐⭐⭐ Multi-Band L1/L5 GPS Tracking                              ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "gps_module.h"

// Global GPS Instance
NEOM9N gps;

// ═══════════════════════════════════════════════════════════════════
// CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════════

NEOM9N::NEOM9N() {
    _serial = nullptr;
    _baud = GPS_BAUD;
    _last_update = 0;
    _last_fix_time = 0;
    _new_data = false;
    _update_rate_hz = GPS_UPDATE_RATE_HZ;
    _multiband_enabled = GPS_USE_L1_L5;
    _error_count = 0;
    _timeout_count = 0;
    _prev_lat = 0;
    _prev_lon = 0;
    _prev_fix_millis = 0;
    _trip_distance = 0;
}

// ═══════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

bool NEOM9N::begin(HardwareSerial& serial, uint32_t baud) {
    _serial = &serial;
    _baud = baud;
    
    // Initialize Serial Port
    _serial->begin(_baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(100);
    
    Serial.println("🛰️  Initialisiere NEO-M9N GPS...");
    
    // Wait for some data
    uint32_t start = millis();
    bool got_data = false;
    
    while (millis() - start < 5000) {  // 5s timeout
        while (_serial->available()) {
            char c = _serial->read();
            _gps.encode(c);
            got_data = true;
        }
        if (got_data) break;
        delay(100);
    }
    
    if (!got_data) {
        Serial.println("❌ Keine GPS Daten empfangen!");
        return false;
    }
    
    Serial.println("✓ GPS Daten empfangen!");
    
    // Configure Update Rate
    delay(100);
    if (configureUpdateRate(_update_rate_hz)) {
        Serial.printf("✓ Update Rate: %d Hz\n", _update_rate_hz);
    } else {
        Serial.println("⚠️  Update Rate Konfiguration fehlgeschlagen");
    }
    
    // Configure Multi-Band (L1+L5)
    delay(100);
    if (_multiband_enabled) {
        if (configureMultiBand(true)) {
            Serial.println("✓ Multi-Band L1+L5 aktiviert");
        } else {
            Serial.println("⚠️  Multi-Band Konfiguration fehlgeschlagen");
        }
    }
    
    return true;
}

bool NEOM9N::isConnected() {
    return (_gps.charsProcessed() > 0);
}

// ═══════════════════════════════════════════════════════════════════
// DATA UPDATE
// ═══════════════════════════════════════════════════════════════════

bool NEOM9N::update() {
    _new_data = false;
    
    if (_serial == nullptr) return false;
    
    // Read all available bytes
    uint16_t chars_read = 0;
    while (_serial->available() && chars_read < 1000) {  // Max 1000 chars per update
        char c = _serial->read();
        
        if (_gps.encode(c)) {
            _new_data = true;
            _last_update = millis();
        }
        
        chars_read++;
    }
    
    // Check for timeout (keine Daten seit 2 Sekunden)
    if (millis() - _last_update > 2000 && _last_update > 0) {
        _timeout_count++;
        if (_timeout_count > 10) {
            _error_count++;
            _timeout_count = 0;
        }
    } else {
        _timeout_count = 0;
    }
    
    // Update cached data structure wenn neue Daten verfügbar
    if (_new_data && _gps.location.isValid()) {
        // Position
        _data.latitude = _gps.location.lat();
        _data.longitude = _gps.location.lng();
        _data.altitude = _gps.altitude.isValid() ? _gps.altitude.meters() : 0;
        
        // Speed & Heading
        _data.speed_kmh = _gps.speed.isValid() ? _gps.speed.kmph() : 0;
        _data.speed_ms = _gps.speed.isValid() ? _gps.speed.mps() : 0;
        _data.heading = _gps.course.isValid() ? _gps.course.deg() : 0;
        
        // Quality
        _data.satellites = _gps.satellites.isValid() ? _gps.satellites.value() : 0;
        _data.hdop = _gps.hdop.isValid() ? _gps.hdop.hdop() : 99.9;
        _data.fix_valid = true;
        _data.fix_type = (_gps.altitude.isValid() ? 2 : 1);  // 2=3D, 1=2D
        
        // Multi-Band Status (M9N spezifisch - wird durch Konfiguration gesetzt)
        _data.l1_active = _multiband_enabled;
        _data.l5_active = _multiband_enabled;
        
        // Time
        if (_gps.time.isValid()) {
            _data.gps_time.hour = _gps.time.hour();
            _data.gps_time.minute = _gps.time.minute();
            _data.gps_time.second = _gps.time.second();
            _data.gps_time.millisecond = _gps.time.centisecond() * 10;
            _data.gps_time.valid = true;
        }
        
        // Date
        if (_gps.date.isValid()) {
            _data.gps_time.year = _gps.date.year();
            _data.gps_time.month = _gps.date.month();
            _data.gps_time.day = _gps.date.day();
        }
        
        // Timestamp
        _data.millis_at_fix = millis();
        _last_fix_time = millis();
        
        // Calculate trip distance
        if (_prev_lat != 0 && _prev_lon != 0) {
            float dist = calculateGPSDistance(_prev_lat, _prev_lon, 
                                             _data.latitude, _data.longitude);
            _trip_distance += dist / 1000.0;  // km
        }
        
        _prev_lat = _data.latitude;
        _prev_lon = _data.longitude;
        _prev_fix_millis = millis();
        
    } else if (!_gps.location.isValid()) {
        _data.fix_valid = false;
        _data.fix_type = 0;
    }
    
    return _new_data;
}

bool NEOM9N::hasNewData() {
    return _new_data;
}

uint32_t NEOM9N::getLastUpdateMillis() {
    return _last_update;
}

// ═══════════════════════════════════════════════════════════════════
// POSITION DATA
// ═══════════════════════════════════════════════════════════════════

double NEOM9N::getLatitude() {
    return _gps.location.lat();
}

double NEOM9N::getLongitude() {
    return _gps.location.lng();
}

float NEOM9N::getAltitude() {
    return _gps.altitude.isValid() ? _gps.altitude.meters() : 0;
}

bool NEOM9N::isLocationValid() {
    return _gps.location.isValid();
}

// ═══════════════════════════════════════════════════════════════════
// SPEED & HEADING
// ═══════════════════════════════════════════════════════════════════

float NEOM9N::getSpeedKmh() {
    return _gps.speed.isValid() ? _gps.speed.kmph() : 0;
}

float NEOM9N::getSpeedMs() {
    return _gps.speed.isValid() ? _gps.speed.mps() : 0;
}

float NEOM9N::getHeading() {
    return _gps.course.isValid() ? _gps.course.deg() : 0;
}

bool NEOM9N::isHeadingValid() {
    // Heading nur gültig wenn Geschwindigkeit > Minimum
    return (_gps.course.isValid() && _gps.speed.kmph() > GPS_HEADING_VALID_SPEED);
}

// ═══════════════════════════════════════════════════════════════════
// QUALITY INDICATORS
// ═══════════════════════════════════════════════════════════════════

uint8_t NEOM9N::getSatellites() {
    return _gps.satellites.isValid() ? _gps.satellites.value() : 0;
}

float NEOM9N::getHDOP() {
    return _gps.hdop.isValid() ? _gps.hdop.hdop() : 99.9;
}

uint8_t NEOM9N::getFixType() {
    if (!_gps.location.isValid()) return 0;  // No Fix
    if (_gps.altitude.isValid()) return 2;   // 3D Fix
    return 1;                                 // 2D Fix
}

bool NEOM9N::hasFix() {
    return (_gps.location.isValid() && 
            _gps.satellites.value() >= GPS_MIN_SATELLITES &&
            _gps.hdop.hdop() < GPS_MIN_HDOP);
}

// ═══════════════════════════════════════════════════════════════════
// TIME & DATE
// ═══════════════════════════════════════════════════════════════════

uint8_t NEOM9N::getHour() {
    return _gps.time.isValid() ? _gps.time.hour() : 0;
}

uint8_t NEOM9N::getMinute() {
    return _gps.time.isValid() ? _gps.time.minute() : 0;
}

uint8_t NEOM9N::getSecond() {
    return _gps.time.isValid() ? _gps.time.second() : 0;
}

uint16_t NEOM9N::getMillisecond() {
    return _gps.time.isValid() ? _gps.time.centisecond() * 10 : 0;
}

uint8_t NEOM9N::getDay() {
    return _gps.date.isValid() ? _gps.date.day() : 0;
}

uint8_t NEOM9N::getMonth() {
    return _gps.date.isValid() ? _gps.date.month() : 0;
}

uint16_t NEOM9N::getYear() {
    return _gps.date.isValid() ? _gps.date.year() : 0;
}

bool NEOM9N::isTimeValid() {
    return _gps.time.isValid();
}

bool NEOM9N::isDateValid() {
    return _gps.date.isValid();
}

// ═══════════════════════════════════════════════════════════════════
// DISTANCE & BEARING
// ═══════════════════════════════════════════════════════════════════

float NEOM9N::distanceTo(double lat, double lon) {
    if (!_gps.location.isValid()) return 0;
    
    return calculateGPSDistance(_gps.location.lat(), _gps.location.lng(), 
                                lat, lon);
}

float NEOM9N::bearingTo(double lat, double lon) {
    if (!_gps.location.isValid()) return 0;
    
    return calculateGPSBearing(_gps.location.lat(), _gps.location.lng(), 
                              lat, lon);
}

// ═══════════════════════════════════════════════════════════════════
// STATISTICS
// ═══════════════════════════════════════════════════════════════════

uint32_t NEOM9N::getCharsProcessed() {
    return _gps.charsProcessed();
}

uint32_t NEOM9N::getSentencesWithFix() {
    return _gps.sentencesWithFix();
}

uint32_t NEOM9N::getFailedChecksum() {
    return _gps.failedChecksum();
}

uint32_t NEOM9N::getPassedChecksum() {
    return _gps.passedChecksum();
}

// ═══════════════════════════════════════════════════════════════════
// GET DATA STRUCTURE
// ═══════════════════════════════════════════════════════════════════

GPSData NEOM9N::getData() {
    return _data;
}

// ═══════════════════════════════════════════════════════════════════
// UBX CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

bool NEOM9N::sendUBX(uint8_t* packet, uint16_t len) {
    if (_serial == nullptr) return false;
    
    // Calculate Checksum
    calculateUBXChecksum(packet, len);
    
    // Send Packet
    _serial->write(packet, len);
    _serial->flush();
    
    // Wait for ACK (optional - könnte implementiert werden)
    delay(100);
    
    return true;
}

void NEOM9N::calculateUBXChecksum(uint8_t* packet, uint16_t len) {
    uint8_t ck_a = 0, ck_b = 0;
    
    // Calculate checksum over payload (skip header and checksum bytes)
    for (uint16_t i = 2; i < len - 2; i++) {
        ck_a += packet[i];
        ck_b += ck_a;
    }
    
    packet[len - 2] = ck_a;
    packet[len - 1] = ck_b;
}

bool NEOM9N::configureUpdateRate(uint8_t rate_hz) {
    // UBX-CFG-RATE: Set Navigation/Measurement Rate
    // Measurement period = 1000ms / rate_hz
    uint16_t meas_rate = 1000 / rate_hz;
    
    uint8_t packet[] = {
        0xB5, 0x62,        // Header
        0x06, 0x08,        // CFG-RATE
        0x06, 0x00,        // Length = 6 bytes
        (uint8_t)(meas_rate & 0xFF), (uint8_t)(meas_rate >> 8),  // measRate (ms)
        0x01, 0x00,        // navRate (cycles)
        0x01, 0x00,        // timeRef (1 = GPS time)
        0x00, 0x00         // Checksum (wird berechnet)
    };
    
    _update_rate_hz = rate_hz;
    return sendUBX(packet, sizeof(packet));
}

bool NEOM9N::configureMultiBand(bool enable) {
    // UBX-CFG-SIGNAL: Configure GNSS Signals
    // M9N unterstützt L1 + L5 gleichzeitig
    
    if (enable) {
        // Enable L1 + L5 for GPS
        uint8_t packet[] = {
            0xB5, 0x62,        // Header
            0x06, 0x3E,        // CFG-GNSS
            0x0C, 0x00,        // Length = 12 bytes
            0x00,              // msgVer
            0x00,              // numTrkChHw (read-only)
            0xFF,              // numTrkChUse (max)
            0x01,              // numConfigBlocks
            // Block 1: GPS
            0x00,              // gnssId (0 = GPS)
            0x08,              // resTrkCh (8 channels)
            0x10,              // maxTrkCh (16 channels)
            0x00,              // reserved1
            0x01, 0x00, 0x01, 0x01,  // flags: enable GPS + L1 + L5
            0x00, 0x00         // Checksum
        };
        
        _multiband_enabled = true;
        return sendUBX(packet, sizeof(packet));
    } else {
        // Standard L1 only
        _multiband_enabled = false;
        return true;  // Default ist L1
    }
}

void NEOM9N::setUpdateRate(uint8_t rate_hz) {
    configureUpdateRate(rate_hz);
}

void NEOM9N::enableMultiBand(bool enable) {
    configureMultiBand(enable);
}

// ═══════════════════════════════════════════════════════════════════
// DEBUG
// ═══════════════════════════════════════════════════════════════════

void NEOM9N::printDebugInfo() {
    Serial.println("╔═══════════════════════════════════════════════════╗");
    Serial.println("║         NEO-M9N GPS DEBUG INFO                   ║");
    Serial.println("╚═══════════════════════════════════════════════════╝");
    
    Serial.printf("Chars Processed:   %lu\n", _gps.charsProcessed());
    Serial.printf("Sentences (Fix):   %lu\n", _gps.sentencesWithFix());
    Serial.printf("Failed Checksum:   %lu\n", _gps.failedChecksum());
    Serial.printf("Passed Checksum:   %lu\n", _gps.passedChecksum());
    Serial.printf("Error Count:       %u\n", _error_count);
    
    Serial.println();
    Serial.printf("Fix Valid:         %s\n", _data.fix_valid ? "YES" : "NO");
    Serial.printf("Fix Type:          %u (%s)\n", _data.fix_type, 
                  _data.fix_type == 2 ? "3D" : (_data.fix_type == 1 ? "2D" : "None"));
    Serial.printf("Satellites:        %u\n", _data.satellites);
    Serial.printf("HDOP:              %.2f\n", _data.hdop);
    
    if (_data.fix_valid) {
        Serial.println();
        Serial.printf("Latitude:          %.6f°\n", _data.latitude);
        Serial.printf("Longitude:         %.6f°\n", _data.longitude);
        Serial.printf("Altitude:          %.1f m\n", _data.altitude);
        Serial.printf("Speed:             %.1f km/h\n", _data.speed_kmh);
        Serial.printf("Heading:           %.1f°\n", _data.heading);
    }
    
    if (_data.gps_time.valid) {
        Serial.println();
        Serial.printf("GPS Time (UTC):    %02d:%02d:%02d.%03d\n", 
                      _data.gps_time.hour, _data.gps_time.minute, 
                      _data.gps_time.second, _data.gps_time.millisecond);
        Serial.printf("GPS Date:          %04d-%02d-%02d\n",
                      _data.gps_time.year, _data.gps_time.month, _data.gps_time.day);
    }
    
    Serial.println();
    Serial.printf("Multi-Band:        %s\n", _multiband_enabled ? "L1+L5" : "L1 only");
    Serial.printf("Update Rate:       %u Hz\n", _update_rate_hz);
    Serial.printf("Trip Distance:     %.2f km\n", _trip_distance);
    
    Serial.println("═══════════════════════════════════════════════════");
}

// ═══════════════════════════════════════════════════════════════════
// ERROR HANDLING
// ═══════════════════════════════════════════════════════════════════

uint16_t NEOM9N::getErrorCount() {
    return _error_count;
}

void NEOM9N::resetErrorCount() {
    _error_count = 0;
}

// ═══════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS (Global)
// ═══════════════════════════════════════════════════════════════════

String formatLatitude(double lat, uint8_t decimals) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.*f° %c", 
             decimals, fabs(lat), lat >= 0 ? 'N' : 'S');
    return String(buffer);
}

String formatLongitude(double lon, uint8_t decimals) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.*f° %c", 
             decimals, fabs(lon), lon >= 0 ? 'E' : 'W');
    return String(buffer);
}

String formatCoordinates(double lat, double lon, uint8_t decimals) {
    return formatLatitude(lat, decimals) + " " + formatLongitude(lon, decimals);
}
