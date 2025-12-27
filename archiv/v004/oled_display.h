/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  OLED DISPLAY MODULE - SSD1306 (128x64)                         ║
 * ║  📺 Live Telemetrie Anzeige                                     ║
 * ║                                                                  ║
 * ║  Features:                                                       ║
 * ║  - Multi-Page Display (4 Seiten)                               ║
 * ║  - Auto Page Switching (3s pro Seite)                          ║
 * ║  - Live Speed, Lean Angle, GPS, Power                          ║
 * ║  - Status Icons & Warnings                                      ║
 * ║  - 5Hz Update Rate                                              ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "../config.h"
#include "../data_structures.h"

// ═══════════════════════════════════════════════════════════════════
// DISPLAY PAGES
// ═══════════════════════════════════════════════════════════════════

enum DisplayPage {
    PAGE_SPEED = 0,      // Speed, Lean Angle, G-Force
    PAGE_GPS = 1,        // GPS Status, Satellites, Position
    PAGE_POWER = 2,      // Voltage, Current, Power
    PAGE_ENV = 3,        // Temperature, Pressure, Altitude
    PAGE_COUNT = 4
};

// ═══════════════════════════════════════════════════════════════════
// OLED DISPLAY CLASS
// ═══════════════════════════════════════════════════════════════════

class OLEDDisplay {
public:
    OLEDDisplay();
    
    // Initialisierung
    bool begin(uint8_t i2c_addr = I2C_ADDR_OLED);
    bool isConnected();
    
    // Display Update
    void update(const TelemetryData& data);
    void clear();
    void refresh();
    
    // Page Management
    void setPage(DisplayPage page);
    void nextPage();
    void prevPage();
    DisplayPage getCurrentPage();
    void setAutoPageSwitch(bool enable);
    void setPageDuration(uint32_t duration_ms);
    
    // Manual Draw Functions
    void drawSpeedPage(const TelemetryData& data);
    void drawGPSPage(const TelemetryData& data);
    void drawPowerPage(const TelemetryData& data);
    void drawEnvPage(const TelemetryData& data);
    
    // Display Control
    void setBrightness(uint8_t brightness);  // 0-255
    void displayOn();
    void displayOff();
    void invertDisplay(bool invert);
    
    // Status Icons
    void drawGPSIcon(int16_t x, int16_t y, bool has_fix);
    void drawBatteryIcon(int16_t x, int16_t y, float voltage);
    void drawSatelliteIcon(int16_t x, int16_t y, uint8_t count);
    void drawWarningIcon(int16_t x, int16_t y);
    
    // Text Helpers
    void drawCenteredText(const char* text, int16_t y, uint8_t size = 1);
    void drawRightAlignedText(const char* text, int16_t x, int16_t y, uint8_t size = 1);
    
    // Progress Bars
    void drawProgressBar(int16_t x, int16_t y, int16_t w, int16_t h, float percent);
    void drawGaugeBar(int16_t x, int16_t y, int16_t w, int16_t h, float value, float min_val, float max_val);
    
    // Get Display Object (für custom drawing)
    Adafruit_SSD1306* getDisplay();

private:
    Adafruit_SSD1306 _display;
    uint8_t _i2c_addr;
    
    // Page Management
    DisplayPage _current_page;
    bool _auto_page_switch;
    uint32_t _page_duration;
    uint32_t _last_page_switch;
    
    // Update Tracking
    uint32_t _last_update;
    bool _is_on;
    
    // Helper Functions
    void drawHeader(const char* title);
    void drawFooter(const TelemetryData& data);
    void checkAutoPageSwitch();
    
    // Formatting Helpers
    String formatSpeed(float speed_kmh);
    String formatLeanAngle(float angle);
    String formatGForce(float g);
    String formatVoltage(float voltage);
    String formatCoordinates(double lat, double lon);
};

// ═══════════════════════════════════════════════════════════════════
// GLOBAL OLED INSTANCE
// ═══════════════════════════════════════════════════════════════════

extern OLEDDisplay oled;

#endif // OLED_DISPLAY_H
