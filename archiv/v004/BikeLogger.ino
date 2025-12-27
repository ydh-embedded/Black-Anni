/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  BIKELOGGER - Main Sketch                                       ║
 * ║  Kawasaki ZX6R (1998) Telemetrie System                         ║
 * ║  Hardware: SparkFun Thing Plus ESP32-S3                         ║
 * ║                                                                  ║
 * ║  Author: Danny (yDh-embedded)                                   ║
 * ║  Version: 1.0.0                                                 ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <Wire.h>
#include <SPI.h>
#include "config.h"
#include "data_structures.h"
#include "calculations.h"

// Hardware Modules
#include "imu_module.h"      // ICM-20948 IMU (⭐⭐⭐⭐⭐ LEAN ANGLE!)
#include "gps_module.h"      // NEO-M9N GPS (⭐⭐⭐ MULTI-BAND L1/L5)
#include "oled_display.h"    // SSD1306 OLED Display (📺 LIVE DATA!)
#include "led_bargraph.h"    // 74HC595 LED Bargraph (💡 12 LEDs)

// ═══════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS & DATA
// ═══════════════════════════════════════════════════════════════════

// Telemetrie-Daten (Shared zwischen Tasks)
TelemetryData telemetry;
SystemStatus system_status;

// FreeRTOS Task Handles
TaskHandle_t taskIMU = NULL;
TaskHandle_t taskGPS = NULL;
TaskHandle_t taskSensors = NULL;
TaskHandle_t taskLogging = NULL;
TaskHandle_t taskDisplay = NULL;

// Mutexes für Thread-Safe Zugriff
SemaphoreHandle_t telemetryMutex;

// Status LED Blinker
bool statusLedState = false;
uint32_t lastStatusBlink = 0;

// ═══════════════════════════════════════════════════════════════════
// SETUP - Initialisierung
// ═══════════════════════════════════════════════════════════════════

void setup() {
    // Serial für Debugging
    Serial.begin(SERIAL_BAUD);
    delay(1000);
    
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║        BIKELOGGER - KAWASAKI ZX6R (1998)        ║");
    Serial.println("║              yDh-embedded GmbH                   ║");
    Serial.println("╚══════════════════════════════════════════════════╝");
    Serial.println();
    Serial.printf("Firmware Version: %s\n", FIRMWARE_VERSION);
    Serial.printf("Chip Model: %s\n", ESP.getChipModel());
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM Size: %d bytes\n", ESP.getPsramSize());
    Serial.println();

    // Status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);

    // Mutex erstellen
    telemetryMutex = xSemaphoreCreateMutex();
    if (telemetryMutex == NULL) {
        Serial.println("❌ FATAL: Mutex konnte nicht erstellt werden!");
        while(1) { delay(1000); }
    }

    // I2C Bus initialisieren
    Serial.println("📡 Initialisiere I2C Bus...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_SPEED);
    delay(100);

    // I2C Bus scannen
    scanI2CBus();

    // SPI Bus initialisieren (SD-Card)
    Serial.println("💾 Initialisiere SPI Bus (SD-Card)...");
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(100);

    // Hardware Module initialisieren
    Serial.println();
    Serial.println("🔧 Initialisiere Hardware-Module...");
    initializeHardware();

    // Session Start Timestamp
    telemetry.session_start_millis = millis();

    // FreeRTOS Tasks erstellen
    Serial.println();
    Serial.println("🚀 Starte FreeRTOS Tasks...");
    createTasks();

    Serial.println();
    Serial.println("✅ SETUP ABGESCHLOSSEN!");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println();

    // Status LED aus (wird durch Tasks gesteuert)
    digitalWrite(STATUS_LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
    // Hauptloop macht fast nichts - alles läuft in Tasks!
    
    // Status LED Heartbeat (1Hz)
    if (millis() - lastStatusBlink >= 1000) {
        lastStatusBlink = millis();
        statusLedState = !statusLedState;
        digitalWrite(STATUS_LED_PIN, statusLedState);
    }

    // System Status Update
    updateSystemStatus();

    // Debug Output (alle 5 Sekunden)
    static uint32_t lastDebugOutput = 0;
    if (DEBUG_MODE && millis() - lastDebugOutput >= 5000) {
        lastDebugOutput = millis();
        printDebugInfo();
    }

    delay(100);  // Main Loop mit 10Hz
}

// ═══════════════════════════════════════════════════════════════════
// I2C BUS SCANNER
// ═══════════════════════════════════════════════════════════════════

void scanI2CBus() {
    Serial.println("🔍 Scanne I2C Bus...");
    
    uint8_t devicesFound = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("   ✓ Device gefunden: 0x%02X", addr);
            
            // Device Namen zuordnen
            if (addr == I2C_ADDR_ICM20948) Serial.print(" (ICM-20948 IMU)");
            else if (addr == I2C_ADDR_BME280) Serial.print(" (BME280)");
            else if (addr == I2C_ADDR_INA260) Serial.print(" (INA260)");
            else if (addr == I2C_ADDR_ADS1115) Serial.print(" (ADS1115)");
            else if (addr == I2C_ADDR_RTC) Serial.print(" (DS3231 RTC)");
            else if (addr == I2C_ADDR_OLED) Serial.print(" (OLED Display)");
            else if (addr == I2C_ADDR_GPS) Serial.print(" (NEO-M9N GPS)");
            else Serial.print(" (Unknown)");
            
            Serial.println();
            devicesFound++;
        }
    }
    
    Serial.printf("   Gefundene Devices: %d\n", devicesFound);
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
// HARDWARE INITIALISIERUNG
// ═══════════════════════════════════════════════════════════════════

void initializeHardware() {
    // IMU initialisieren
    Serial.print("   🎯 ICM-20948 IMU... ");
    system_status.imu_online = initIMU();
    Serial.println(system_status.imu_online ? "✓" : "✗");

    // GPS initialisieren
    Serial.print("   🛰️  NEO-M9N GPS... ");
    system_status.gps_online = initGPS();
    Serial.println(system_status.gps_online ? "✓" : "✗");

    // BME280 initialisieren
    Serial.print("   🌡️  BME280... ");
    system_status.bme280_online = initBME280();
    Serial.println(system_status.bme280_online ? "✓" : "✗");

    // INA260 initialisieren
    Serial.print("   ⚡ INA260... ");
    system_status.ina260_online = initINA260();
    Serial.println(system_status.ina260_online ? "✓" : "✗");

    // ADS1115 initialisieren
    Serial.print("   📊 ADS1115... ");
    system_status.ads1115_online = initADS1115();
    Serial.println(system_status.ads1115_online ? "✓" : "✗");

    // RTC initialisieren
    Serial.print("   🕐 DS3231 RTC... ");
    system_status.rtc_online = initRTC();
    Serial.println(system_status.rtc_online ? "✓" : "✗");

    // OLED initialisieren
    Serial.print("   📺 OLED Display... ");
    system_status.oled_online = initOLED();
    Serial.println(system_status.oled_online ? "✓" : "✗");

    // SD-Card initialisieren
    Serial.print("   💾 SD-Card... ");
    system_status.sd_online = initSDCard();
    Serial.println(system_status.sd_online ? "✓" : "✗");

    // LED Bargraph initialisieren
    Serial.print("   💡 LED Bargraph... ");
    initLEDBarGraph();
    Serial.println("✓");
}

// ═══════════════════════════════════════════════════════════════════
// HARDWARE INIT STUB FUNCTIONS (werden in Modulen implementiert)
// ═══════════════════════════════════════════════════════════════════

bool initIMU() {
    if (!imu.begin()) {
        return false;
    }
    
    // IMU Self-Test
    if (!imu.selfTest()) {
        Serial.println("⚠️  IMU Self-Test fehlgeschlagen!");
    }
    
    // Kalibrierung (optional - kann auch später manuell gemacht werden)
    // Serial.println("⏳ Warte 3 Sekunden vor Kalibrierung...");
    // delay(3000);
    // imu.calibrate();
    
    return true;
}

bool initGPS() {
    if (!gps.begin()) {
        return false;
    }
    
    // Warte auf ersten Fix (max 30 Sekunden)
    Serial.println("⏳ Warte auf GPS Fix (max 30s)...");
    uint32_t start = millis();
    
    while (millis() - start < 30000) {
        gps.update();
        
        if (gps.hasFix()) {
            Serial.println("✓ GPS Fix erhalten!");
            Serial.printf("   Position: %.6f, %.6f\n", gps.getLatitude(), gps.getLongitude());
            Serial.printf("   Satelliten: %u\n", gps.getSatellites());
            Serial.printf("   HDOP: %.2f\n", gps.getHDOP());
            break;
        }
        
        // Status Update alle 2 Sekunden
        static uint32_t lastStatus = 0;
        if (millis() - lastStatus >= 2000) {
            lastStatus = millis();
            Serial.printf("   Satelliten: %u, HDOP: %.2f\n", 
                         gps.getSatellites(), gps.getHDOP());
        }
        
        delay(100);
    }
    
    if (!gps.hasFix()) {
        Serial.println("⚠️  Kein GPS Fix innerhalb 30s (GPS läuft weiter im Hintergrund)");
    }
    
    return true;
}

bool initBME280() {
    // TODO: Implementierung in bme280_module.cpp
    return false;  // Placeholder
}

bool initINA260() {
    // TODO: Implementierung in ina260_module.cpp
    return false;  // Placeholder
}

bool initADS1115() {
    // TODO: Implementierung in ads1115_module.cpp
    return false;  // Placeholder
}

bool initRTC() {
    // TODO: Implementierung in rtc_module.cpp
    return false;  // Placeholder
}

bool initOLED() {
    if (!oled.begin()) {
        return false;
    }
    
    // Auto Page Switch aktivieren (3s pro Seite)
    oled.setAutoPageSwitch(true);
    oled.setPageDuration(3000);
    
    return true;
}

bool initSDCard() {
    // TODO: Implementierung in sd_logger.cpp
    return false;  // Placeholder
}

void initLEDBarGraph() {
    ledBar.begin();
    
    // Set Speed Mode als Default
    ledBar.setMode(LED_MODE_SPEED);
    ledBar.setSpeedRange(0, 200);  // 0-200 km/h
}

// ═══════════════════════════════════════════════════════════════════
// FREERTOS TASKS ERSTELLEN
// ═══════════════════════════════════════════════════════════════════

void createTasks() {
    // IMU Task (100Hz) - HÖCHSTE PRIORITÄT!
    xTaskCreatePinnedToCore(
        taskIMULoop,              // Task Funktion
        "IMU_Task",               // Name
        TASK_STACK_SIZE_MEDIUM,   // Stack Size
        NULL,                     // Parameter
        TASK_PRIORITY_IMU,        // Priorität
        &taskIMU,                 // Task Handle
        1                         // Core 1 (Core 0 für WiFi)
    );
    Serial.println("   ✓ IMU Task erstellt (Core 1, 100Hz)");

    // GPS Task (10Hz)
    xTaskCreatePinnedToCore(
        taskGPSLoop,
        "GPS_Task",
        TASK_STACK_SIZE_MEDIUM,
        NULL,
        TASK_PRIORITY_GPS,
        &taskGPS,
        1
    );
    Serial.println("   ✓ GPS Task erstellt (Core 1, 10Hz)");

    // Sensors Task (Slower sensors)
    xTaskCreatePinnedToCore(
        taskSensorsLoop,
        "Sensors_Task",
        TASK_STACK_SIZE_MEDIUM,
        NULL,
        TASK_PRIORITY_SENSORS,
        &taskSensors,
        1
    );
    Serial.println("   ✓ Sensors Task erstellt (Core 1)");

    // Logging Task
    xTaskCreatePinnedToCore(
        taskLoggingLoop,
        "Logging_Task",
        TASK_STACK_SIZE_LARGE,
        NULL,
        TASK_PRIORITY_LOGGING,
        &taskLogging,
        0  // Core 0
    );
    Serial.println("   ✓ Logging Task erstellt (Core 0)");

    // Display Task
    xTaskCreatePinnedToCore(
        taskDisplayLoop,
        "Display_Task",
        TASK_STACK_SIZE_MEDIUM,
        NULL,
        TASK_PRIORITY_DISPLAY,
        &taskDisplay,
        0  // Core 0
    );
    Serial.println("   ✓ Display Task erstellt (Core 0)");
}

// ═══════════════════════════════════════════════════════════════════
// FREERTOS TASK LOOPS
// ═══════════════════════════════════════════════════════════════════

void taskIMULoop(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / IMU_SAMPLE_RATE_HZ);
    
    Serial.println("🎯 IMU Task gestartet!");
    
    while (true) {
        if (system_status.imu_online) {
            // IMU Daten aktualisieren
            if (imu.update()) {
                // Lock Mutex
                if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
                    // IMU Daten in Telemetrie kopieren
                    telemetry.imu = imu.getData();
                    
                    // Unlock Mutex
                    xSemaphoreGive(telemetryMutex);
                }
            } else {
                system_status.imu_errors++;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void taskGPSLoop(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / GPS_SAMPLE_RATE_HZ);
    
    Serial.println("🛰️  GPS Task gestartet!");
    
    while (true) {
        if (system_status.gps_online) {
            // GPS Daten aktualisieren
            if (gps.update()) {
                // Lock Mutex
                if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
                    // GPS Daten in Telemetrie kopieren
                    telemetry.gps = gps.getData();
                    
                    // GPS Altitude für Sensor Fusion
                    telemetry.env.altitude_gps = telemetry.gps.altitude;
                    
                    // Unlock Mutex
                    xSemaphoreGive(telemetryMutex);
                }
            } else {
                // Keine neuen Daten, aber kein Fehler
            }
            
            // Error Tracking
            if (!gps.hasFix() && gps.getCharsProcessed() > 1000) {
                // Nur Fehler zählen wenn GPS bereits Daten gesendet hat
                static uint32_t lastFixCheck = 0;
                if (millis() - lastFixCheck > 10000) {  // Alle 10s prüfen
                    lastFixCheck = millis();
                    system_status.gps_errors++;
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void taskSensorsLoop(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    Serial.println("🌡️  Sensors Task gestartet!");
    
    while (true) {
        // BME280 (1Hz)
        // updateBME280();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // INA260 (10Hz)
        // updateINA260();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // ADS1115 (10Hz)
        // updateADS1115();
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

void taskLoggingLoop(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    Serial.println("💾 Logging Task gestartet!");
    
    while (true) {
        // TODO: Daten auf SD-Card loggen
        // logDataToSD();
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz
    }
}

void taskDisplayLoop(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / OLED_UPDATE_RATE_HZ);
    
    // LED Bargraph Update schneller (20Hz)
    uint8_t led_update_counter = 0;
    const uint8_t led_update_divider = OLED_UPDATE_RATE_HZ / LED_BAR_UPDATE_RATE_HZ;  // 5/20 = 4
    
    Serial.println("📺 Display Task gestartet!");
    
    while (true) {
        // OLED Display Update (5Hz)
        if (system_status.oled_online) {
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(100))) {
                oled.update(telemetry);
                xSemaphoreGive(telemetryMutex);
            }
        }
        
        // LED Bargraph Update (20Hz - jeder 4. Zyklus wenn OLED 5Hz ist)
        led_update_counter++;
        if (led_update_counter >= led_update_divider) {
            led_update_counter = 0;
            
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(100))) {
                ledBar.update(telemetry);
                xSemaphoreGive(telemetryMutex);
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ═══════════════════════════════════════════════════════════════════
// SYSTEM STATUS UPDATE
// ═══════════════════════════════════════════════════════════════════

void updateSystemStatus() {
    system_status.uptime_seconds = millis() / 1000;
    system_status.free_heap = ESP.getFreeHeap();
}

// ═══════════════════════════════════════════════════════════════════
// DEBUG OUTPUT
// ═══════════════════════════════════════════════════════════════════

void printDebugInfo() {
    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("⏱️  Uptime: %lu s\n", system_status.uptime_seconds);
    Serial.printf("💾 Free Heap: %lu bytes\n", system_status.free_heap);
    
    Serial.println("\n📊 Hardware Status:");
    Serial.printf("   IMU:     %s", system_status.imu_online ? "✓ Online" : "✗ Offline");
    if (system_status.imu_online) Serial.printf(" (Errors: %d)", system_status.imu_errors);
    Serial.println();
    Serial.printf("   GPS:     %s\n", system_status.gps_online ? "✓ Online" : "✗ Offline");
    Serial.printf("   BME280:  %s\n", system_status.bme280_online ? "✓ Online" : "✗ Offline");
    Serial.printf("   INA260:  %s\n", system_status.ina260_online ? "✓ Online" : "✗ Offline");
    Serial.printf("   ADS1115: %s\n", system_status.ads1115_online ? "✓ Online" : "✗ Offline");
    Serial.printf("   RTC:     %s\n", system_status.rtc_online ? "✓ Online" : "✗ Offline");
    Serial.printf("   OLED:    %s\n", system_status.oled_online ? "✓ Online" : "✗ Offline");
    Serial.printf("   SD-Card: %s\n", system_status.sd_online ? "✓ Online" : "✗ Offline");
    
    // IMU Data
    if (system_status.imu_online) {
        Serial.println("\n🏍️  IMU Data:");
        if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(100))) {
            Serial.printf("   🎯 Lean Angle:  %.1f° %s\n", 
                         fabs(telemetry.imu.lean_angle),
                         telemetry.imu.lean_angle < 0 ? "L" : (telemetry.imu.lean_angle > 0 ? "R" : ""));
            Serial.printf("   📈 Max Lean:    L:%.1f° R:%.1f°\n", 
                         telemetry.imu.max_lean_left, telemetry.imu.max_lean_right);
            Serial.printf("   ⚡ G-Force:     Fwd:%.2f Lat:%.2f Tot:%.2f\n",
                         telemetry.imu.g_forward, telemetry.imu.g_lateral, telemetry.imu.g_total);
            Serial.printf("   🎪 Motion:      %s%s%s\n",
                         telemetry.imu.wheelie_detected ? "WHEELIE " : "",
                         telemetry.imu.stoppie_detected ? "STOPPIE " : "",
                         telemetry.imu.is_cornering ? "CORNERING" : "");
            Serial.printf("   🧭 Heading:     %.0f°\n", telemetry.imu.compass_heading);
            Serial.printf("   🌡️  Temp:        %.1f°C\n", telemetry.imu.temperature);
            Serial.printf("   ✓  Calibrated:  %s\n", telemetry.imu.calibrated ? "Yes" : "No");
            xSemaphoreGive(telemetryMutex);
        }
    }
    
    // GPS Data
    if (system_status.gps_online) {
        Serial.println("\n🛰️  GPS Data:");
        if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(100))) {
            Serial.printf("   📍 Fix:         %s (%uD)\n", 
                         telemetry.gps.fix_valid ? "VALID" : "INVALID",
                         telemetry.gps.fix_type);
            if (telemetry.gps.fix_valid) {
                Serial.printf("   🌍 Position:    %.6f, %.6f\n", 
                             telemetry.gps.latitude, telemetry.gps.longitude);
                Serial.printf("   ⛰️  Altitude:    %.1f m\n", telemetry.gps.altitude);
                Serial.printf("   🏃 Speed:       %.1f km/h\n", telemetry.gps.speed_kmh);
                Serial.printf("   🧭 Heading:     %.0f°\n", telemetry.gps.heading);
            }
            Serial.printf("   🛰️  Satellites:  %u\n", telemetry.gps.satellites);
            Serial.printf("   📊 HDOP:        %.2f\n", telemetry.gps.hdop);
            Serial.printf("   📡 Multi-Band:  %s\n", 
                         (telemetry.gps.l1_active && telemetry.gps.l5_active) ? "L1+L5" : "L1");
            if (telemetry.gps.gps_time.valid) {
                Serial.printf("   🕐 GPS Time:    %02d:%02d:%02d UTC\n",
                             telemetry.gps.gps_time.hour, 
                             telemetry.gps.gps_time.minute,
                             telemetry.gps.gps_time.second);
            }
            xSemaphoreGive(telemetryMutex);
        }
    }
    
    Serial.println("═══════════════════════════════════════════════════\n");
}
