/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  BIKELOGGER - Calculation Functions                             ║
 * ║  Mathematische Funktionen für Telemetrie-Berechnungen           ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <Arduino.h>
#include <math.h>
#include "data_structures.h"
#include "config.h"

// ═══════════════════════════════════════════════════════════════════
// MATHEMATISCHE KONSTANTEN
// ═══════════════════════════════════════════════════════════════════

#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)
#define GRAVITY 9.80665  // m/s² (Standard-Erdbeschleunigung)

// ═══════════════════════════════════════════════════════════════════
// LEAN ANGLE CALCULATION (⭐⭐⭐⭐⭐ KILLER FEATURE!)
// ═══════════════════════════════════════════════════════════════════

/**
 * Berechnet Lean Angle aus Accelerometer-Daten
 * 
 * @param accel_x Beschleunigung X-Achse (g)
 * @param accel_y Beschleunigung Y-Achse (g)
 * @param accel_z Beschleunigung Z-Achse (g)
 * @return Lean Angle in Grad (-64° bis +64°, negativ=links, positiv=rechts)
 * 
 * Methode: Arctan2 basierend auf Schwerkraft-Vektor
 * Funktioniert im Stillstand und bei konstanter Geschwindigkeit
 * NICHT akkurat bei starker Beschleunigung/Bremsung!
 */
inline float calculateLeanAngle(float accel_x, float accel_y, float accel_z) {
    // Roll-Winkel aus Accelerometer (Bank-Winkel)
    // atan2(ay, az) gibt den Winkel zwischen Y und Z-Achse
    float roll = atan2(accel_y, accel_z) * RAD_TO_DEG;
    
    // Limitieren auf physikalisch mögliche Werte
    if (roll > MAX_LEAN_ANGLE_DEG) roll = MAX_LEAN_ANGLE_DEG;
    if (roll < -MAX_LEAN_ANGLE_DEG) roll = -MAX_LEAN_ANGLE_DEG;
    
    return roll;
}

/**
 * Verbesserte Lean Angle Berechnung mit Gyro-Integration
 * Kombiniert Accelerometer (langsam, stabil) mit Gyroscope (schnell, driftet)
 * 
 * @param accel_roll Roll aus Accelerometer (°)
 * @param gyro_x Gyro X-Achse (°/s)
 * @param dt Delta-Zeit seit letzter Messung (s)
 * @param prev_roll Vorheriger Roll-Winkel (°)
 * @param alpha Complementary Filter Konstante (0.0-1.0, typisch 0.96)
 * @return Gefilteter Lean Angle (°)
 */
inline float calculateLeanAngleFiltered(float accel_roll, float gyro_x, 
                                        float dt, float prev_roll, 
                                        float alpha = 0.96) {
    // Gyro-Integration: Schnelle Änderungen erfassen
    float gyro_roll = prev_roll + (gyro_x * dt);
    
    // Complementary Filter: 96% Gyro, 4% Accelerometer
    // Gyro für schnelle Bewegungen, Accel korrigiert Drift
    float roll = alpha * gyro_roll + (1.0 - alpha) * accel_roll;
    
    // Limitieren
    if (roll > MAX_LEAN_ANGLE_DEG) roll = MAX_LEAN_ANGLE_DEG;
    if (roll < -MAX_LEAN_ANGLE_DEG) roll = -MAX_LEAN_ANGLE_DEG;
    
    return roll;
}

// ═══════════════════════════════════════════════════════════════════
// G-FORCE CALCULATION
// ═══════════════════════════════════════════════════════════════════

/**
 * Berechnet G-Force in Fahrzeug-Koordinaten
 * Transformiert Sensor-Daten in fahrerbezogenes Koordinatensystem
 * 
 * @param accel_x, accel_y, accel_z Rohe Accelerometer-Werte (g)
 * @param pitch, roll, yaw Lage des Fahrzeugs (°)
 * @param g_forward, g_lateral, g_vertical Output: G-Force Komponenten
 */
inline void calculateGForce(float accel_x, float accel_y, float accel_z,
                           float pitch, float roll, float yaw,
                           float& g_forward, float& g_lateral, float& g_vertical) {
    // Rotation Matrix (vereinfacht für kleine Winkel)
    float pitch_rad = pitch * DEG_TO_RAD;
    float roll_rad = roll * DEG_TO_RAD;
    
    // Forward/Backward (X-Achse Fahrzeug)
    g_forward = accel_x * cos(pitch_rad) + accel_z * sin(pitch_rad);
    
    // Lateral/Sideways (Y-Achse Fahrzeug)
    g_lateral = accel_y * cos(roll_rad) - accel_z * sin(roll_rad);
    
    // Vertical/Up-Down (Z-Achse Fahrzeug) - Schwerkraft subtrahiert
    g_vertical = accel_z * cos(pitch_rad) * cos(roll_rad) - 1.0;
}

/**
 * Berechnet totale G-Force (Betrag)
 */
inline float calculateTotalGForce(float g_forward, float g_lateral, float g_vertical) {
    return sqrt(g_forward*g_forward + g_lateral*g_lateral + g_vertical*g_vertical);
}

// ═══════════════════════════════════════════════════════════════════
// WHEELIE / STOPPIE DETECTION
// ═══════════════════════════════════════════════════════════════════

/**
 * Erkennt Wheelie anhand Pitch-Winkel und Beschleunigung
 * 
 * @param pitch Nick-Winkel (°)
 * @param g_vertical Vertikale G-Force
 * @return true wenn Wheelie erkannt
 */
inline bool detectWheelie(float pitch, float g_vertical) {
    // Wheelie: Pitch > 15° UND vertikale Beschleunigung > 0.3g
    return (pitch > 15.0 && g_vertical > 0.3);
}

/**
 * Erkennt Stoppie anhand Pitch-Winkel und negativer Beschleunigung
 */
inline bool detectStoppie(float pitch, float g_forward) {
    // Stoppie: Pitch < -10° UND starke Bremsbeschleunigung
    return (pitch < -10.0 && g_forward < -0.8);
}

// ═══════════════════════════════════════════════════════════════════
// CORNERING DETECTION
// ═══════════════════════════════════════════════════════════════════

/**
 * Erkennt Kurvenfahrt anhand Lean Angle und lateraler G-Force
 * 
 * @param lean_angle Schräglage (°)
 * @param g_lateral Seitliche G-Force
 * @param speed_kmh Geschwindigkeit (km/h)
 * @return true wenn in Kurve
 */
inline bool detectCornering(float lean_angle, float g_lateral, float speed_kmh) {
    // Kurve: Lean > 10° ODER laterale G > 0.3 UND Geschwindigkeit > 20 km/h
    return ((fabs(lean_angle) > 10.0 || fabs(g_lateral) > 0.3) && speed_kmh > 20.0);
}

// ═══════════════════════════════════════════════════════════════════
// VIBRATION ANALYSIS
// ═══════════════════════════════════════════════════════════════════

/**
 * Berechnet RMS (Root Mean Square) Vibration Level
 * 
 * @param samples Array von Accelerometer-Samples
 * @param count Anzahl Samples
 * @return RMS Vibrations-Level
 */
inline float calculateVibrationRMS(float* samples, int count) {
    if (count == 0) return 0.0;
    
    float sum = 0.0;
    for (int i = 0; i < count; i++) {
        sum += samples[i] * samples[i];
    }
    
    return sqrt(sum / count);
}

// ═══════════════════════════════════════════════════════════════════
// ALTITUDE CALCULATIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * Berechnet Höhe aus Luftdruck (Barometrische Formel)
 * 
 * @param pressure Aktueller Luftdruck (hPa)
 * @param sea_level_pressure Luftdruck auf Meereshöhe (hPa, Standard: 1013.25)
 * @return Höhe (m)
 */
inline float calculateAltitudeFromPressure(float pressure, float sea_level_pressure = 1013.25) {
    // Barometrische Höhenformel (vereinfacht)
    // h = 44330 * (1 - (p/p0)^(1/5.255))
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}

/**
 * Berechnet Luftdruck auf Meereshöhe aus aktueller Höhe
 * 
 * @param pressure Aktueller Luftdruck (hPa)
 * @param altitude Bekannte Höhe (m)
 * @return Normalisierter Luftdruck (hPa)
 */
inline float calculateSeaLevelPressure(float pressure, float altitude) {
    // Umkehrung der barometrischen Formel
    return pressure / pow(1.0 - (altitude / 44330.0), 5.255);
}

/**
 * Fusioniert GPS und Barometer-Höhe mit Complementary Filter
 * 
 * @param altitude_baro Barometrische Höhe (m)
 * @param altitude_gps GPS Höhe (m)
 * @param prev_altitude Vorherige fusionierte Höhe (m)
 * @param alpha Filter-Konstante (0.0-1.0, typisch 0.95)
 * @return Fusionierte Höhe (m)
 */
inline float fuseAltitude(float altitude_baro, float altitude_gps, 
                         float prev_altitude, float alpha = 0.95) {
    // Baro: Hohe Auflösung, kurzzeitige Genauigkeit
    // GPS: Niedrige Auflösung, langfristige Genauigkeit
    // 95% Baro (schnelle Änderungen) + 5% GPS (Drift-Korrektur)
    return alpha * altitude_baro + (1.0 - alpha) * altitude_gps;
}

// ═══════════════════════════════════════════════════════════════════
// TEMPERATURE & HUMIDITY CALCULATIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * Berechnet Taupunkt aus Temperatur und Luftfeuchtigkeit
 * Magnus-Formel
 * 
 * @param temperature Temperatur (°C)
 * @param humidity Relative Luftfeuchtigkeit (%)
 * @return Taupunkt (°C)
 */
inline float calculateDewPoint(float temperature, float humidity) {
    const float a = 17.27;
    const float b = 237.7;
    
    float alpha = ((a * temperature) / (b + temperature)) + log(humidity / 100.0);
    return (b * alpha) / (a - alpha);
}

// ═══════════════════════════════════════════════════════════════════
// SPEED & DISTANCE CALCULATIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * Berechnet Geschwindigkeit aus Hall-Sensor Frequenz
 * 
 * @param pulse_interval_us Zeit zwischen zwei Pulsen (µs)
 * @return Geschwindigkeit (km/h)
 */
inline float calculateSpeedFromHallSensor(uint32_t pulse_interval_us) {
    if (pulse_interval_us == 0) return 0.0;
    
    // Frequenz (Hz) = 1.000.000 / interval_us
    float freq_hz = 1000000.0 / pulse_interval_us;
    
    // U/min = freq_hz * 60
    float rpm = freq_hz * 60.0;
    
    // Geschwindigkeit = (Radumfang [m] * U/min) / 60 * 3.6 [km/h]
    float wheel_circumference_m = WHEEL_CIRCUMFERENCE_MM / 1000.0;
    float speed_kmh = (wheel_circumference_m * rpm / 60.0) * 3.6;
    
    return speed_kmh;
}

/**
 * Berechnet zurückgelegte Distanz aus Pulsen
 * 
 * @param pulse_count Anzahl Hall-Sensor Pulse
 * @return Distanz (km)
 */
inline float calculateDistanceFromPulses(uint32_t pulse_count) {
    float wheel_circumference_m = WHEEL_CIRCUMFERENCE_MM / 1000.0;
    float distance_m = pulse_count * wheel_circumference_m / WHEEL_PULSES_PER_REV;
    return distance_m / 1000.0;  // km
}

// ═══════════════════════════════════════════════════════════════════
// GPS CALCULATIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * Berechnet Distanz zwischen zwei GPS-Koordinaten (Haversine)
 * 
 * @param lat1, lon1 Erste Position (°)
 * @param lat2, lon2 Zweite Position (°)
 * @return Distanz (m)
 */
inline float calculateGPSDistance(double lat1, double lon1, double lat2, double lon2) {
    const double earth_radius = 6371000.0;  // Erdradius in Metern
    
    double d_lat = (lat2 - lat1) * DEG_TO_RAD;
    double d_lon = (lon2 - lon1) * DEG_TO_RAD;
    
    double a = sin(d_lat/2) * sin(d_lat/2) +
               cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
               sin(d_lon/2) * sin(d_lon/2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return earth_radius * c;
}

/**
 * Berechnet Bearing (Richtung) zwischen zwei GPS-Koordinaten
 * 
 * @param lat1, lon1 Start-Position (°)
 * @param lat2, lon2 Ziel-Position (°)
 * @return Bearing (0-359°, 0=Nord)
 */
inline float calculateGPSBearing(double lat1, double lon1, double lat2, double lon2) {
    double d_lon = (lon2 - lon1) * DEG_TO_RAD;
    
    double y = sin(d_lon) * cos(lat2 * DEG_TO_RAD);
    double x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) -
               sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(d_lon);
    
    float bearing = atan2(y, x) * RAD_TO_DEG;
    
    // Normalisieren auf 0-359°
    return fmod((bearing + 360.0), 360.0);
}

// ═══════════════════════════════════════════════════════════════════
// BATTERY CALCULATIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * Berechnet Batterie-Ladezustand aus Spannung (12V Lead-Acid)
 * Sehr vereinfacht! Echte SOC braucht Coulomb-Counting
 * 
 * @param voltage Batteriespannung (V)
 * @return Ladezustand (0-100%)
 */
inline float calculateBatteryPercentage(float voltage) {
    // Grobe Abschätzung für 12V Blei-Säure
    // 12.6V = 100%, 12.0V = 50%, 11.5V = 0%
    
    if (voltage >= BATTERY_VOLTAGE_NOMINAL) return 100.0;
    if (voltage <= BATTERY_VOLTAGE_CRITICAL) return 0.0;
    
    // Linear zwischen CRITICAL und NOMINAL
    float percent = ((voltage - BATTERY_VOLTAGE_CRITICAL) / 
                     (BATTERY_VOLTAGE_NOMINAL - BATTERY_VOLTAGE_CRITICAL)) * 100.0;
    
    return constrain(percent, 0.0, 100.0);
}

// ═══════════════════════════════════════════════════════════════════
// FILTER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * Low-Pass Filter (Exponential Moving Average)
 * Glättet Sensor-Rauschen
 * 
 * @param new_value Neuer Messwert
 * @param prev_filtered Vorheriger gefilterter Wert
 * @param alpha Filter-Konstante (0.0-1.0, klein = stärker gefiltert)
 * @return Gefilterter Wert
 */
inline float lowPassFilter(float new_value, float prev_filtered, float alpha = 0.2) {
    return alpha * new_value + (1.0 - alpha) * prev_filtered;
}

/**
 * Deadband Filter (Totzone)
 * Ignoriert kleine Änderungen
 * 
 * @param value Eingangswert
 * @param threshold Schwellwert
 * @return Wert oder 0 wenn innerhalb Totzone
 */
inline float deadbandFilter(float value, float threshold) {
    if (fabs(value) < threshold) return 0.0;
    return value;
}

#endif // CALCULATIONS_H
