/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  BIKELOGGER - Data Structures                                   ║
 * ║  Zentrale Datenstrukturen für alle Sensoren & Telemetrie        ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <Arduino.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════════
// TIMESTAMP STRUCTURE
// ═══════════════════════════════════════════════════════════════════

struct Timestamp {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    
    // Unix Timestamp (seconds since 1970-01-01)
    uint32_t unix;
    
    bool valid;
    
    // Constructor
    Timestamp() : year(0), month(0), day(0), hour(0), minute(0), 
                  second(0), millisecond(0), unix(0), valid(false) {}
};

// ═══════════════════════════════════════════════════════════════════
// GPS DATA STRUCTURE (NEO-M9N)
// ═══════════════════════════════════════════════════════════════════

struct GPSData {
    // Position (WGS84)
    double latitude;           // Breitengrad (°)
    double longitude;          // Längengrad (°)
    float altitude;            // Höhe über Meeresspiegel (m)
    
    // Speed & Heading
    float speed_kmh;           // Geschwindigkeit (km/h)
    float speed_ms;            // Geschwindigkeit (m/s)
    float heading;             // Richtung (0-359°, 0=Nord)
    
    // Quality Indicators
    uint8_t satellites;        // Anzahl Satelliten
    float hdop;                // Horizontal Dilution of Precision
    bool fix_valid;            // GPS Fix gültig
    uint8_t fix_type;          // 0=No Fix, 1=2D, 2=3D
    
    // Multi-Band Status
    bool l1_active;            // L1 Band aktiv
    bool l5_active;            // L5 Band aktiv
    
    // Timestamps
    Timestamp gps_time;        // GPS Zeit (UTC)
    uint32_t millis_at_fix;    // Millis() bei letztem Fix
    
    // Constructor
    GPSData() : latitude(0), longitude(0), altitude(0), 
                speed_kmh(0), speed_ms(0), heading(0),
                satellites(0), hdop(99.9), fix_valid(false), fix_type(0),
                l1_active(false), l5_active(false), millis_at_fix(0) {}
};

// ═══════════════════════════════════════════════════════════════════
// IMU DATA STRUCTURE (ICM-20948) ⭐⭐⭐⭐⭐ KILLER!
// ═══════════════════════════════════════════════════════════════════

struct IMUData {
    // Raw Sensor Data
    float accel_x, accel_y, accel_z;  // Beschleunigung (g)
    float gyro_x, gyro_y, gyro_z;     // Winkelgeschwindigkeit (°/s)
    float mag_x, mag_y, mag_z;        // Magnetfeld (µT)
    
    // Processed Data
    float pitch;               // Nick-Winkel (°)
    float roll;                // Roll-Winkel (°) = LEAN ANGLE! 🏍️
    float yaw;                 // Gier-Winkel (°)
    
    // G-Force (Fahrzeug-Koordinaten)
    float g_forward;           // Beschleunigung vorwärts (+) / rückwärts (-)
    float g_lateral;           // Beschleunigung seitlich (links/rechts)
    float g_vertical;          // Beschleunigung vertikal (hoch/runter)
    float g_total;             // Totale G-Force
    
    // Lean Angle (⭐ STAR FEATURE!)
    float lean_angle;          // Schräglage (0-64°)
    float max_lean_left;       // Max Lean links (Session)
    float max_lean_right;      // Max Lean rechts (Session)
    
    // Compass
    float compass_heading;     // Kompass-Richtung (0-359°)
    
    // Motion Detection
    bool wheelie_detected;     // Wheelie erkannt
    bool stoppie_detected;     // Stoppie erkannt
    bool is_cornering;         // In Kurve
    float cornering_speed;     // Kurvengeschwindigkeit (km/h)
    
    // Vibration
    float vibration_level;     // Vibrations-Level (RMS)
    float vibration_freq;      // Dominante Frequenz (Hz)
    
    // Calibration Status
    bool calibrated;
    float accel_offset_x, accel_offset_y, accel_offset_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    
    // Temperature
    float temperature;         // IMU Temperatur (°C)
    
    // Timestamp
    uint32_t millis_at_reading;
    
    // Constructor
    IMUData() : accel_x(0), accel_y(0), accel_z(0),
                gyro_x(0), gyro_y(0), gyro_z(0),
                mag_x(0), mag_y(0), mag_z(0),
                pitch(0), roll(0), yaw(0),
                g_forward(0), g_lateral(0), g_vertical(0), g_total(0),
                lean_angle(0), max_lean_left(0), max_lean_right(0),
                compass_heading(0),
                wheelie_detected(false), stoppie_detected(false),
                is_cornering(false), cornering_speed(0),
                vibration_level(0), vibration_freq(0),
                calibrated(false),
                accel_offset_x(0), accel_offset_y(0), accel_offset_z(0),
                gyro_offset_x(0), gyro_offset_y(0), gyro_offset_z(0),
                temperature(0), millis_at_reading(0) {}
};

// ═══════════════════════════════════════════════════════════════════
// ENVIRONMENTAL DATA STRUCTURE (BME280)
// ═══════════════════════════════════════════════════════════════════

struct EnvironmentalData {
    // Atmospheric Sensors
    float temperature;         // Temperatur (°C)
    float humidity;            // Luftfeuchtigkeit (%)
    float pressure;            // Luftdruck (hPa)
    
    // Calculated Altitude
    float altitude_baro;       // Barometrische Höhe (m)
    float altitude_gps;        // GPS Höhe (m)
    float altitude_fused;      // Fusionierte Höhe (Baro + GPS) 🏔️
    
    // Derived Values
    float dew_point;           // Taupunkt (°C)
    float sea_level_pressure;  // Normierter Luftdruck (hPa)
    
    // Timestamp
    uint32_t millis_at_reading;
    
    // Constructor
    EnvironmentalData() : temperature(0), humidity(0), pressure(0),
                          altitude_baro(0), altitude_gps(0), altitude_fused(0),
                          dew_point(0), sea_level_pressure(0),
                          millis_at_reading(0) {}
};

// ═══════════════════════════════════════════════════════════════════
// POWER DATA STRUCTURE (INA260)
// ═══════════════════════════════════════════════════════════════════

struct PowerData {
    // Measurements
    float voltage;             // Batteriespannung (V)
    float current;             // Strom (A)
    float power;               // Leistung (W)
    
    // Battery Status
    float battery_percentage;  // Ladezustand (0-100%)
    bool battery_low;          // Warnung: Batterie niedrig
    bool battery_critical;     // Warnung: Batterie kritisch
    
    // Statistics
    float energy_consumed;     // Verbrauchte Energie (Wh)
    float avg_power;           // Durchschnittsleistung (W)
    
    // Timestamp
    uint32_t millis_at_reading;
    
    // Constructor
    PowerData() : voltage(0), current(0), power(0),
                  battery_percentage(100), battery_low(false), battery_critical(false),
                  energy_consumed(0), avg_power(0),
                  millis_at_reading(0) {}
};

// ═══════════════════════════════════════════════════════════════════
// ANALOG INPUTS DATA STRUCTURE (ADS1115)
// ═══════════════════════════════════════════════════════════════════

struct AnalogInputsData {
    // Raw ADC Values (0-32767)
    int16_t adc0_raw, adc1_raw, adc2_raw, adc3_raw;
    
    // Converted Voltages (0-6.144V)
    float adc0_voltage, adc1_voltage, adc2_voltage, adc3_voltage;
    
    // Mapped Values (0-100%)
    float throttle_percent;    // Gasgriff
    float brake_percent;       // Bremse
    float clutch_percent;      // Kupplung
    float reserve_percent;     // Reserve
    
    // Timestamp
    uint32_t millis_at_reading;
    
    // Constructor
    AnalogInputsData() : adc0_raw(0), adc1_raw(0), adc2_raw(0), adc3_raw(0),
                         adc0_voltage(0), adc1_voltage(0), adc2_voltage(0), adc3_voltage(0),
                         throttle_percent(0), brake_percent(0), clutch_percent(0), reserve_percent(0),
                         millis_at_reading(0) {}
};

// ═══════════════════════════════════════════════════════════════════
// VEHICLE DATA STRUCTURE (Tacho, etc.)
// ═══════════════════════════════════════════════════════════════════

struct VehicleData {
    // Speed from Hall Sensor
    float speed_kmh;           // Geschwindigkeit (km/h)
    float wheel_rpm;           // Raddrehzahl (U/min)
    
    // Distance
    float trip_distance;       // Trip-Distanz (km)
    float total_distance;      // Gesamt-Distanz (km)
    
    // Engine (wenn verfügbar)
    float engine_rpm;          // Motordrehzahl (U/min)
    
    // Timestamp
    uint32_t millis_at_reading;
    
    // Constructor
    VehicleData() : speed_kmh(0), wheel_rpm(0),
                    trip_distance(0), total_distance(0),
                    engine_rpm(0),
                    millis_at_reading(0) {}
};

// ═══════════════════════════════════════════════════════════════════
// TELEMETRY DATA STRUCTURE (Combined)
// ═══════════════════════════════════════════════════════════════════

struct TelemetryData {
    Timestamp timestamp;
    GPSData gps;
    IMUData imu;
    EnvironmentalData env;
    PowerData power;
    AnalogInputsData analog;
    VehicleData vehicle;
    
    // Session Info
    uint32_t session_start_millis;
    uint32_t ride_time_seconds;
    bool logging_active;
    
    // Constructor
    TelemetryData() : session_start_millis(0), ride_time_seconds(0), 
                      logging_active(false) {}
};

// ═══════════════════════════════════════════════════════════════════
// SYSTEM STATUS STRUCTURE
// ═══════════════════════════════════════════════════════════════════

struct SystemStatus {
    // Hardware Status
    bool imu_online;
    bool gps_online;
    bool bme280_online;
    bool ina260_online;
    bool ads1115_online;
    bool rtc_online;
    bool oled_online;
    bool sd_online;
    
    // Error Counters
    uint16_t imu_errors;
    uint16_t gps_errors;
    uint16_t sensor_errors;
    uint16_t sd_errors;
    
    // Performance
    float loop_time_ms;
    float cpu_usage_percent;
    uint32_t free_heap;
    
    // Uptime
    uint32_t uptime_seconds;
    
    // Constructor
    SystemStatus() : imu_online(false), gps_online(false), bme280_online(false),
                     ina260_online(false), ads1115_online(false), rtc_online(false),
                     oled_online(false), sd_online(false),
                     imu_errors(0), gps_errors(0), sensor_errors(0), sd_errors(0),
                     loop_time_ms(0), cpu_usage_percent(0), free_heap(0),
                     uptime_seconds(0) {}
};

// ═══════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS (Inline)
// ═══════════════════════════════════════════════════════════════════

// Timestamp zu String (ISO 8601)
inline String timestampToString(const Timestamp& ts) {
    if (!ts.valid) return "INVALID";
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             ts.year, ts.month, ts.day, ts.hour, ts.minute, ts.second, ts.millisecond);
    return String(buffer);
}

// GPS Position zu String (Lat/Lon)
inline String gpsToString(const GPSData& gps) {
    if (!gps.fix_valid) return "NO FIX";
    
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.6f,%.6f", gps.latitude, gps.longitude);
    return String(buffer);
}

// Lean Angle zu String mit Icon
inline String leanAngleToString(float angle) {
    char buffer[32];
    if (angle < 0) {
        snprintf(buffer, sizeof(buffer), "%.1f° L", -angle);  // Links
    } else if (angle > 0) {
        snprintf(buffer, sizeof(buffer), "%.1f° R", angle);   // Rechts
    } else {
        snprintf(buffer, sizeof(buffer), "0.0°");             // Neutral
    }
    return String(buffer);
}

// Battery Status zu Icon
inline String batteryIcon(float voltage) {
    if (voltage >= BATTERY_VOLTAGE_FULL) return "█████";      // Voll
    if (voltage >= BATTERY_VOLTAGE_NOMINAL) return "████░";   // Gut
    if (voltage >= BATTERY_VOLTAGE_LOW) return "███░░";       // Niedrig
    if (voltage >= BATTERY_VOLTAGE_CRITICAL) return "██░░░";  // Kritisch
    return "█░░░░";                                            // Leer
}

#endif // DATA_STRUCTURES_H
