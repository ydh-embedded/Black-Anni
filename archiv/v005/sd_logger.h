/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  SD LOGGER MODULE - Data Logging                                ║
 * ║  💾 CSV Logging auf 64GB SD-Card                               ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "../config.h"
#include "../data_structures.h"

class SDLogger {
public:
    SDLogger();
    bool begin();
    bool isConnected();
    
    // Logging Control
    bool startLogging();
    bool stopLogging();
    bool isLogging();
    
    // Write Data
    bool logTelemetry(const TelemetryData& data);
    bool flush();
    
    // File Management
    String getCurrentFilename();
    uint32_t getBytesWritten();
    uint32_t getRecordsWritten();
    
    // Error Handling
    uint16_t getErrorCount();
    void resetErrorCount();

private:
    File _logfile;
    String _filename;
    bool _logging;
    bool _initialized;
    uint32_t _bytes_written;
    uint32_t _records_written;
    uint32_t _last_flush;
    uint16_t _error_count;
    
    String generateFilename();
    bool writeCSVHeader();
    String telemetryToCSV(const TelemetryData& data);
};

extern SDLogger sdLogger;

#endif
