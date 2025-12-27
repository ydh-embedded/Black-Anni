/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  SD LOGGER MODULE - Implementation                              ║
 * ║  💾 CSV Data Logging                                            ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "sd_logger.h"
#include "rtc_module.h"

SDLogger sdLogger;

SDLogger::SDLogger() {
    _logging = false;
    _initialized = false;
    _bytes_written = 0;
    _records_written = 0;
    _last_flush = 0;
    _error_count = 0;
}

bool SDLogger::begin() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("❌ SD-Card Init fehlgeschlagen!");
        return false;
    }
    
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("✓ SD-Card: %llu MB\n", cardSize);
    
    _initialized = true;
    return true;
}

bool SDLogger::isConnected() {
    return _initialized;
}

bool SDLogger::startLogging() {
    if (_logging) return true;
    
    _filename = generateFilename();
    
    _logfile = SD.open(_filename.c_str(), FILE_WRITE);
    if (!_logfile) {
        Serial.println("❌ Konnte Logfile nicht erstellen!");
        _error_count++;
        return false;
    }
    
    writeCSVHeader();
    _logging = true;
    _bytes_written = 0;
    _records_written = 0;
    
    Serial.printf("✓ Logging gestartet: %s\n", _filename.c_str());
    
    return true;
}

bool SDLogger::stopLogging() {
    if (!_logging) return true;
    
    flush();
    _logfile.close();
    _logging = false;
    
    Serial.printf("✓ Logging gestoppt. %lu Records geschrieben.\n", _records_written);
    
    return true;
}

bool SDLogger::isLogging() {
    return _logging;
}

bool SDLogger::logTelemetry(const TelemetryData& data) {
    if (!_logging) return false;
    
    String csv = telemetryToCSV(data);
    size_t written = _logfile.println(csv);
    
    if (written == 0) {
        _error_count++;
        return false;
    }
    
    _bytes_written += written;
    _records_written++;
    
    // Auto-Flush alle 5 Sekunden
    if (millis() - _last_flush >= LOG_FLUSH_INTERVAL_MS) {
        flush();
    }
    
    return true;
}

bool SDLogger::flush() {
    if (!_logging) return false;
    _logfile.flush();
    _last_flush = millis();
    return true;
}

String SDLogger::getCurrentFilename() {
    return _filename;
}

uint32_t SDLogger::getBytesWritten() {
    return _bytes_written;
}

uint32_t SDLogger::getRecordsWritten() {
    return _records_written;
}

String SDLogger::generateFilename() {
    String name = LOG_FILENAME_PREFIX;
    name += rtc.getFilenameString();
    name += ".csv";
    return name;
}

bool SDLogger::writeCSVHeader() {
    _logfile.println("timestamp,millis,"
                     "gps_lat,gps_lon,gps_alt,gps_speed,gps_heading,gps_sats,gps_hdop,gps_fix,"
                     "imu_lean,imu_pitch,imu_roll,imu_yaw,"
                     "imu_g_fwd,imu_g_lat,imu_g_vert,imu_g_total,"
                     "imu_wheelie,imu_stoppie,imu_cornering,"
                     "imu_temp,imu_compass,"
                     "env_temp,env_hum,env_press,env_alt,"
                     "pwr_volt,pwr_curr,pwr_power,pwr_batt,"
                     "analog0,analog1,analog2,analog3");
    return true;
}

String SDLogger::telemetryToCSV(const TelemetryData& data) {
    char buffer[512];
    
    snprintf(buffer, sizeof(buffer),
             "%s,%lu,"  // timestamp, millis
             "%.6f,%.6f,%.1f,%.1f,%.1f,%u,%.2f,%u,"  // GPS
             "%.2f,%.2f,%.2f,%.2f,"  // IMU angles
             "%.2f,%.2f,%.2f,%.2f,"  // IMU g-force
             "%d,%d,%d,"  // IMU motion
             "%.1f,%.1f,"  // IMU temp, compass
             "%.1f,%.1f,%.1f,%.1f,"  // ENV
             "%.2f,%.2f,%.2f,%.1f,"  // PWR
             "%.2f,%.2f,%.2f,%.2f",  // ANALOG
             // Timestamp & Millis
             rtc.getISOString().c_str(),
             millis(),
             // GPS
             data.gps.latitude, data.gps.longitude, data.gps.altitude,
             data.gps.speed_kmh, data.gps.heading,
             data.gps.satellites, data.gps.hdop, data.gps.fix_type,
             // IMU Angles
             data.imu.lean_angle, data.imu.pitch, data.imu.roll, data.imu.yaw,
             // IMU G-Force
             data.imu.g_forward, data.imu.g_lateral, data.imu.g_vertical, data.imu.g_total,
             // IMU Motion
             data.imu.wheelie_detected, data.imu.stoppie_detected, data.imu.is_cornering,
             // IMU Others
             data.imu.temperature, data.imu.compass_heading,
             // ENV
             data.env.temperature, data.env.humidity, data.env.pressure, data.env.altitude_fused,
             // PWR
             data.power.voltage, data.power.current, data.power.power, data.power.battery_percentage,
             // ANALOG
             data.analog.throttle_percent, data.analog.brake_percent,
             data.analog.clutch_percent, data.analog.reserve_percent
    );
    
    return String(buffer);
}

uint16_t SDLogger::getErrorCount() {
    return _error_count;
}

void SDLogger::resetErrorCount() {
    _error_count = 0;
}
