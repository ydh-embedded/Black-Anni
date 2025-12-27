/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  BIKELOGGER - Configuration Header                              ║
 * ║  Kawasaki ZX6R (1998) Telemetrie System                         ║
 * ║  Hardware: SparkFun Thing Plus ESP32-S3                         ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
// SYSTEM CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

#define FIRMWARE_VERSION "1.0.0"
#define BIKE_MODEL "Kawasaki ZX6R (1998)"
#define LOGGER_NAME "BikeLogger"

// Debugging
#define DEBUG_MODE true
#define SERIAL_BAUD 115200

// ═══════════════════════════════════════════════════════════════════
// I2C BUS CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

// ESP32-S3 Qwiic/STEMMA QT Connector
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_SPEED 400000  // 400kHz Fast Mode

// I2C Device Addresses
#define I2C_ADDR_ICM20948   0x69  // 9-DoF IMU (default)
#define I2C_ADDR_BME280     0x77  // Atmospheric Sensor (SDO HIGH)
#define I2C_ADDR_INA260     0x40  // Current/Voltage Sensor (A0+A1 GND)
#define I2C_ADDR_ADS1115    0x48  // 16-bit ADC (ADDR → GND)
#define I2C_ADDR_RTC        0x68  // DS3231 Real Time Clock
#define I2C_ADDR_OLED       0x3C  // SSD1306 128x64 OLED Display
#define I2C_ADDR_GPS        0x42  // NEO-M9N (I2C mode - optional)

// ═══════════════════════════════════════════════════════════════════
// UART CONFIGURATION (GPS)
// ═══════════════════════════════════════════════════════════════════

// GPS NEO-M9N (UART preferred for high update rate)
#define GPS_SERIAL Serial1
#define GPS_RX_PIN 18
#define GPS_TX_PIN 17
#define GPS_BAUD 115200
#define GPS_UPDATE_RATE_HZ 10  // 10Hz für flüssiges Tracking

// ═══════════════════════════════════════════════════════════════════
// SPI CONFIGURATION (SD-CARD)
// ═══════════════════════════════════════════════════════════════════

#define SD_CS_PIN 5
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19
#define SD_SCK_PIN 18
#define SD_SPEED_MHZ 25  // 25MHz für SDHC/SDXC

// ═══════════════════════════════════════════════════════════════════
// GPIO PIN CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

// LED Bargraph (74HC595 Shift Register)
#define LED_DATA_PIN 13    // SER (Serial Data)
#define LED_CLOCK_PIN 14   // SRCLK (Shift Clock)
#define LED_LATCH_PIN 15   // RCLK (Latch Clock)
#define LED_COUNT 12       // 8 green + 4 red

// Hall Sensor (Tacho Signal)
#define HALL_SENSOR_PIN 34  // Analog Input (ADC1_CH6)
#define HALL_TRIGGER_LEVEL 2048  // 12-bit ADC midpoint

// Status LED (onboard)
#define STATUS_LED_PIN 2

// ═══════════════════════════════════════════════════════════════════
// SENSOR SAMPLING RATES
// ═══════════════════════════════════════════════════════════════════

#define IMU_SAMPLE_RATE_HZ 100     // 100Hz für Lean Angle (⭐⭐⭐⭐⭐)
#define GPS_SAMPLE_RATE_HZ 10      // 10Hz
#define BME280_SAMPLE_RATE_HZ 1    // 1Hz (langsame Änderungen)
#define INA260_SAMPLE_RATE_HZ 10   // 10Hz (Power Monitoring)
#define ADS1115_SAMPLE_RATE_HZ 10  // 10Hz (Analog Inputs)

// Display Update Rates
#define OLED_UPDATE_RATE_HZ 5      // 5Hz (200ms)
#define LED_BAR_UPDATE_RATE_HZ 20  // 20Hz (50ms) - smooth animation

// ═══════════════════════════════════════════════════════════════════
// IMU CONFIGURATION (ICM-20948)
// ═══════════════════════════════════════════════════════════════════

// Accelerometer Range
#define ACCEL_RANGE_G 16           // ±16g (für wheelies/stoppies)
#define ACCEL_SENSITIVITY 2048.0   // LSB/g @ ±16g

// Gyroscope Range
#define GYRO_RANGE_DPS 2000        // ±2000°/s
#define GYRO_SENSITIVITY 16.4      // LSB/(°/s) @ ±2000°/s

// Magnetometer Range
#define MAG_SENSITIVITY 0.15       // µT/LSB

// Lean Angle Limits (Kawasaki ZX6R)
#define MAX_LEAN_ANGLE_DEG 64.0    // Maximaler Schräglage (⭐ KILLER!)
#define WARNING_LEAN_ANGLE_DEG 55.0 // Warnung ab 55°

// Calibration
#define IMU_CALIBRATION_SAMPLES 500  // Samples für Offset-Kalibrierung

// ═══════════════════════════════════════════════════════════════════
// GPS CONFIGURATION (NEO-M9N)
// ═══════════════════════════════════════════════════════════════════

// Multi-Band Configuration
#define GPS_USE_L1_L5 true         // L1 + L5 für höhere Genauigkeit
#define GPS_MIN_SATELLITES 4       // Mindestanzahl Satelliten
#define GPS_MIN_HDOP 2.5           // Max HDOP für valide Position

// Speed & Heading
#define GPS_MIN_SPEED_KMH 5.0      // Unter 5km/h = Stillstand
#define GPS_HEADING_VALID_SPEED 10.0 // Heading nur über 10km/h gültig

// ═══════════════════════════════════════════════════════════════════
// BME280 CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

#define BME280_OVERSAMPLING_TEMP 2    // x2
#define BME280_OVERSAMPLING_PRESS 16  // x16 (höchste Genauigkeit)
#define BME280_OVERSAMPLING_HUM 1     // x1
#define BME280_FILTER_COEFF 16        // IIR Filter
#define BME280_STANDBY_TIME 1000      // 1s

// ═══════════════════════════════════════════════════════════════════
// INA260 CONFIGURATION (Power Monitoring)
// ═══════════════════════════════════════════════════════════════════

#define INA260_AVG_SAMPLES 16         // 16 samples averaging
#define INA260_CONVERSION_TIME 1100   // 1.1ms per conversion
#define INA260_SHUNT_RESISTANCE 0.002 // 2mΩ internal shunt

// Battery Thresholds (12V Lead-Acid)
#define BATTERY_VOLTAGE_FULL 14.4     // Vollgeladen (Lichtmaschine)
#define BATTERY_VOLTAGE_NOMINAL 12.6  // Ruhespannung voll
#define BATTERY_VOLTAGE_LOW 12.0      // Niedrig
#define BATTERY_VOLTAGE_CRITICAL 11.5 // Kritisch!

// ═══════════════════════════════════════════════════════════════════
// ADS1115 CONFIGURATION (Analog Inputs)
// ═══════════════════════════════════════════════════════════════════

#define ADS1115_GAIN_TWOTHIRDS 0  // ±6.144V (für 12V Divider)
#define ADS1115_SAMPLE_RATE 860   // 860 SPS
#define ADS1115_CHANNELS 4        // 4 Single-Ended Inputs

// Analog Input Assignment
#define ADS_CH0_NAME "Throttle"
#define ADS_CH1_NAME "Brake"
#define ADS_CH2_NAME "Clutch"
#define ADS_CH3_NAME "Reserve"

// ═══════════════════════════════════════════════════════════════════
// SD CARD LOGGING
// ═══════════════════════════════════════════════════════════════════

#define LOG_FILENAME_PREFIX "/ride_"  // ride_YYYYMMDD_HHMMSS.csv
#define LOG_BUFFER_SIZE 512           // Bytes
#define LOG_FLUSH_INTERVAL_MS 5000    // Alle 5s flushen

// File Format
#define LOG_FORMAT_CSV true           // CSV für Excel/Analysis
#define LOG_FORMAT_GPX false          // GPX für GPS-Track (optional)

// ═══════════════════════════════════════════════════════════════════
// DISPLAY CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

// OLED SSD1306 (128x64)
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET_PIN -1  // Shared reset

// Display Pages (zyklisch)
#define DISPLAY_PAGE_COUNT 4
#define DISPLAY_PAGE_SPEED 0      // Speed, Lean, G-Force
#define DISPLAY_PAGE_GPS 1        // GPS, Satellites, HDOP
#define DISPLAY_PAGE_POWER 2      // Voltage, Current, Power
#define DISPLAY_PAGE_ENV 3        // Temp, Press, Altitude

// Auto-Page-Switching
#define DISPLAY_PAGE_DURATION_MS 3000  // 3s pro Seite

// ═══════════════════════════════════════════════════════════════════
// LED BARGRAPH CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

// Display Modes
#define LED_MODE_SPEED 0    // Geschwindigkeit
#define LED_MODE_RPM 1      // Drehzahl (wenn verfügbar)
#define LED_MODE_LEAN 2     // Lean Angle

// Speed Bargraph (km/h)
#define LED_SPEED_MIN 0
#define LED_SPEED_MAX 200   // 200 km/h = 12 LEDs

// LED Thresholds (green → red transition)
#define LED_GREEN_COUNT 8   // LED 1-8 grün
#define LED_RED_COUNT 4     // LED 9-12 rot

// ═══════════════════════════════════════════════════════════════════
// VEHICLE SPECIFIC (Kawasaki ZX6R 1998)
// ═══════════════════════════════════════════════════════════════════

// Wheel Circumference (mm)
#define WHEEL_CIRCUMFERENCE_MM 1910  // 120/60-17 Vorderrad
#define WHEEL_PULSES_PER_REV 1       // Hall Sensor Pulse

// RPM Calculation (wenn Tacho-Signal verfügbar)
#define TACHO_PULSES_PER_REV 2       // Abhängig vom Geber

// Weight & Dimensions
#define BIKE_WEIGHT_KG 185           // Leergewicht (ca.)
#define RIDER_WEIGHT_KG 80           // Standard-Fahrergewicht

// ═══════════════════════════════════════════════════════════════════
// FREETRTOS TASK PRIORITIES
// ═══════════════════════════════════════════════════════════════════

#define TASK_PRIORITY_IMU 5         // Höchste (100Hz!)
#define TASK_PRIORITY_GPS 4
#define TASK_PRIORITY_SENSORS 3
#define TASK_PRIORITY_LOGGING 2
#define TASK_PRIORITY_DISPLAY 1     // Niedrigste

// Task Stack Sizes
#define TASK_STACK_SIZE_LARGE 4096
#define TASK_STACK_SIZE_MEDIUM 2048
#define TASK_STACK_SIZE_SMALL 1024

// ═══════════════════════════════════════════════════════════════════
// SAFETY & LIMITS
// ═══════════════════════════════════════════════════════════════════

#define MAX_G_FORCE 5.0              // Maximale G-Force (Warning)
#define MAX_VIBRATION_THRESHOLD 10.0 // Vibrations-Warning
#define WATCHDOG_TIMEOUT_MS 10000    // 10s Watchdog

#endif // CONFIG_H
