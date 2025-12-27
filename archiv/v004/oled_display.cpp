/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  OLED DISPLAY MODULE - SSD1306 Implementation                   ║
 * ║  📺 Live Telemetrie auf 128x64 OLED                             ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "oled_display.h"

// Global OLED Instance
OLEDDisplay oled;

// ═══════════════════════════════════════════════════════════════════
// CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════════

OLEDDisplay::OLEDDisplay() 
    : _display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN) {
    _i2c_addr = I2C_ADDR_OLED;
    _current_page = PAGE_SPEED;
    _auto_page_switch = true;
    _page_duration = DISPLAY_PAGE_DURATION_MS;
    _last_page_switch = 0;
    _last_update = 0;
    _is_on = false;
}

// ═══════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

bool OLEDDisplay::begin(uint8_t i2c_addr) {
    _i2c_addr = i2c_addr;
    
    // Initialize Display
    if (!_display.begin(SSD1306_SWITCHCAPVCC, _i2c_addr)) {
        Serial.println("❌ SSD1306 OLED init fehlgeschlagen!");
        return false;
    }
    
    Serial.println("✓ SSD1306 OLED initialisiert!");
    
    // Clear Display
    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);
    _display.setTextSize(1);
    
    // Startup Screen
    _display.clearDisplay();
    _display.setTextSize(2);
    drawCenteredText("BIKELOGGER", 10, 2);
    _display.setTextSize(1);
    drawCenteredText("Kawasaki ZX6R", 32, 1);
    drawCenteredText("yDh-embedded", 44, 1);
    _display.display();
    
    delay(2000);
    
    _is_on = true;
    _last_page_switch = millis();
    
    return true;
}

bool OLEDDisplay::isConnected() {
    // SSD1306 hat keine WHO_AM_I - prüfe ob Display initialisiert wurde
    return _is_on;
}

// ═══════════════════════════════════════════════════════════════════
// DISPLAY UPDATE
// ═══════════════════════════════════════════════════════════════════

void OLEDDisplay::update(const TelemetryData& data) {
    if (!_is_on) return;
    
    // Auto Page Switch
    if (_auto_page_switch) {
        checkAutoPageSwitch();
    }
    
    // Clear Display
    _display.clearDisplay();
    
    // Draw current page
    switch (_current_page) {
        case PAGE_SPEED:
            drawSpeedPage(data);
            break;
        case PAGE_GPS:
            drawGPSPage(data);
            break;
        case PAGE_POWER:
            drawPowerPage(data);
            break;
        case PAGE_ENV:
            drawEnvPage(data);
            break;
    }
    
    // Draw Footer (Status Icons)
    drawFooter(data);
    
    // Update Display
    _display.display();
    _last_update = millis();
}

void OLEDDisplay::clear() {
    _display.clearDisplay();
    _display.display();
}

void OLEDDisplay::refresh() {
    _display.display();
}

// ═══════════════════════════════════════════════════════════════════
// PAGE 1: SPEED, LEAN ANGLE, G-FORCE
// ═══════════════════════════════════════════════════════════════════

void OLEDDisplay::drawSpeedPage(const TelemetryData& data) {
    // Header
    drawHeader("RIDE DATA");
    
    // Speed (groß, zentriert)
    _display.setTextSize(3);
    String speed_str = String((int)data.gps.speed_kmh);
    int16_t x = (OLED_WIDTH - (speed_str.length() * 18)) / 2;
    _display.setCursor(x, 16);
    _display.print(speed_str);
    
    _display.setTextSize(1);
    _display.setCursor(x + (speed_str.length() * 18) + 2, 30);
    _display.print("km/h");
    
    // Lean Angle Indicator
    int16_t lean_bar_y = 40;
    int16_t lean_bar_center = OLED_WIDTH / 2;
    int16_t lean_bar_width = 60;
    
    // Lean Angle Bar
    _display.drawRect(lean_bar_center - lean_bar_width/2, lean_bar_y, lean_bar_width, 6, SSD1306_WHITE);
    
    // Lean Position
    float lean_normalized = constrain(data.imu.lean_angle / MAX_LEAN_ANGLE_DEG, -1.0, 1.0);
    int16_t lean_pos = lean_bar_center + (int16_t)(lean_normalized * (lean_bar_width/2 - 2));
    _display.fillRect(lean_pos - 1, lean_bar_y + 1, 3, 4, SSD1306_WHITE);
    
    // Lean Angle Text
    _display.setTextSize(1);
    String lean_str = formatLeanAngle(data.imu.lean_angle);
    drawCenteredText(lean_str.c_str(), 48, 1);
    
    // G-Force (klein)
    _display.setCursor(0, 56);
    _display.printf("G: %.1f", data.imu.g_total);
    
    // Max Lean (klein)
    drawRightAlignedText(
        String("L:" + String((int)data.imu.max_lean_left) + " R:" + String((int)data.imu.max_lean_right)).c_str(),
        OLED_WIDTH, 56, 1
    );
}

// ═══════════════════════════════════════════════════════════════════
// PAGE 2: GPS STATUS
// ═══════════════════════════════════════════════════════════════════

void OLEDDisplay::drawGPSPage(const TelemetryData& data) {
    drawHeader("GPS STATUS");
    
    _display.setTextSize(1);
    
    // Fix Status
    _display.setCursor(0, 12);
    _display.print("Fix: ");
    if (data.gps.fix_valid) {
        _display.print(data.gps.fix_type == 2 ? "3D" : "2D");
    } else {
        _display.print("NONE");
    }
    
    // Satellites
    _display.setCursor(64, 12);
    _display.printf("Sat: %u", data.gps.satellites);
    
    // HDOP
    _display.setCursor(0, 22);
    _display.printf("HDOP: %.1f", data.gps.hdop);
    
    // Multi-Band
    _display.setCursor(64, 22);
    _display.print("L1");
    if (data.gps.l5_active) _display.print("+L5");
    
    if (data.gps.fix_valid) {
        // Position
        _display.setCursor(0, 34);
        _display.printf("Lat: %.5f", data.gps.latitude);
        
        _display.setCursor(0, 44);
        _display.printf("Lon: %.5f", data.gps.longitude);
        
        // Altitude & Heading
        _display.setCursor(0, 54);
        _display.printf("Alt: %.0fm", data.gps.altitude);
        
        _display.setCursor(64, 54);
        _display.printf("Hdg: %.0f", data.gps.heading);
        _display.print((char)247);  // Grad-Symbol
    } else {
        _display.setTextSize(2);
        drawCenteredText("NO FIX", 36, 2);
    }
}

// ═══════════════════════════════════════════════════════════════════
// PAGE 3: POWER MONITORING
// ═══════════════════════════════════════════════════════════════════

void OLEDDisplay::drawPowerPage(const TelemetryData& data) {
    drawHeader("POWER");
    
    _display.setTextSize(1);
    
    // Voltage (groß)
    _display.setTextSize(2);
    _display.setCursor(0, 14);
    _display.printf("%.1fV", data.power.voltage);
    
    // Battery Bar
    drawProgressBar(70, 16, 56, 10, data.power.battery_percentage);
    
    // Current
    _display.setTextSize(1);
    _display.setCursor(0, 32);
    _display.printf("Current: %.2fA", data.power.current);
    
    // Power
    _display.setCursor(0, 42);
    _display.printf("Power:   %.1fW", data.power.power);
    
    // Battery Status
    _display.setCursor(0, 54);
    if (data.power.battery_critical) {
        _display.print("CRITICAL!");
    } else if (data.power.battery_low) {
        _display.print("Battery Low");
    } else {
        _display.printf("Battery: %.0f%%", data.power.battery_percentage);
    }
}

// ═══════════════════════════════════════════════════════════════════
// PAGE 4: ENVIRONMENT
// ═══════════════════════════════════════════════════════════════════

void OLEDDisplay::drawEnvPage(const TelemetryData& data) {
    drawHeader("ENVIRONMENT");
    
    _display.setTextSize(1);
    
    // Temperature
    _display.setCursor(0, 14);
    _display.printf("Temp:     %.1f", data.env.temperature);
    _display.print((char)247);  // Grad-Symbol
    _display.print("C");
    
    // Humidity
    _display.setCursor(0, 26);
    _display.printf("Humidity: %.0f%%", data.env.humidity);
    
    // Pressure
    _display.setCursor(0, 38);
    _display.printf("Pressure: %.0f hPa", data.env.pressure);
    
    // Altitude
    _display.setCursor(0, 50);
    _display.printf("Altitude: %.0f m", data.env.altitude_fused);
}

// ═══════════════════════════════════════════════════════════════════
// PAGE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════

void OLEDDisplay::setPage(DisplayPage page) {
    if (page >= PAGE_COUNT) return;
    _current_page = page;
    _last_page_switch = millis();
}

void OLEDDisplay::nextPage() {
    _current_page = (DisplayPage)((_current_page + 1) % PAGE_COUNT);
    _last_page_switch = millis();
}

void OLEDDisplay::prevPage() {
    _current_page = (DisplayPage)((_current_page - 1 + PAGE_COUNT) % PAGE_COUNT);
    _last_page_switch = millis();
}

DisplayPage OLEDDisplay::getCurrentPage() {
    return _current_page;
}

void OLEDDisplay::setAutoPageSwitch(bool enable) {
    _auto_page_switch = enable;
    if (enable) _last_page_switch = millis();
}

void OLEDDisplay::setPageDuration(uint32_t duration_ms) {
    _page_duration = duration_ms;
}

void OLEDDisplay::checkAutoPageSwitch() {
    if (millis() - _last_page_switch >= _page_duration) {
        nextPage();
    }
}

// ═══════════════════════════════════════════════════════════════════
// DISPLAY CONTROL
// ═══════════════════════════════════════════════════════════════════

void OLEDDisplay::setBrightness(uint8_t brightness) {
    _display.ssd1306_command(SSD1306_SETCONTRAST);
    _display.ssd1306_command(brightness);
}

void OLEDDisplay::displayOn() {
    _display.ssd1306_command(SSD1306_DISPLAYON);
    _is_on = true;
}

void OLEDDisplay::displayOff() {
    _display.ssd1306_command(SSD1306_DISPLAYOFF);
    _is_on = false;
}

void OLEDDisplay::invertDisplay(bool invert) {
    _display.invertDisplay(invert);
}

// ═══════════════════════════════════════════════════════════════════
// DRAWING HELPERS
// ═══════════════════════════════════════════════════════════════════

void OLEDDisplay::drawHeader(const char* title) {
    _display.setTextSize(1);
    _display.setCursor(0, 0);
    _display.print(title);
    _display.drawFastHLine(0, 9, OLED_WIDTH, SSD1306_WHITE);
}

void OLEDDisplay::drawFooter(const TelemetryData& data) {
    // Status Bar am unteren Rand
    int16_t footer_y = OLED_HEIGHT - 1;
    
    // GPS Icon
    if (data.gps.fix_valid) {
        _display.fillCircle(4, footer_y - 3, 2, SSD1306_WHITE);
    } else {
        _display.drawCircle(4, footer_y - 3, 2, SSD1306_WHITE);
    }
    
    // IMU Icon (wenn kalibriert)
    if (data.imu.calibrated) {
        _display.fillRect(10, footer_y - 5, 3, 5, SSD1306_WHITE);
    }
    
    // Battery Icon
    int16_t bat_x = OLED_WIDTH - 12;
    _display.drawRect(bat_x, footer_y - 5, 10, 5, SSD1306_WHITE);
    _display.fillRect(bat_x + 10, footer_y - 3, 2, 1, SSD1306_WHITE);
    
    // Battery Fill
    int16_t fill_width = (data.power.battery_percentage / 100.0) * 8;
    if (fill_width > 0) {
        _display.fillRect(bat_x + 1, footer_y - 4, fill_width, 3, SSD1306_WHITE);
    }
}

void OLEDDisplay::drawCenteredText(const char* text, int16_t y, uint8_t size) {
    _display.setTextSize(size);
    int16_t x = (OLED_WIDTH - (strlen(text) * 6 * size)) / 2;
    _display.setCursor(x, y);
    _display.print(text);
}

void OLEDDisplay::drawRightAlignedText(const char* text, int16_t x, int16_t y, uint8_t size) {
    _display.setTextSize(size);
    int16_t text_x = x - (strlen(text) * 6 * size);
    _display.setCursor(text_x, y);
    _display.print(text);
}

void OLEDDisplay::drawProgressBar(int16_t x, int16_t y, int16_t w, int16_t h, float percent) {
    // Outline
    _display.drawRect(x, y, w, h, SSD1306_WHITE);
    
    // Fill
    int16_t fill_w = (int16_t)((percent / 100.0) * (w - 2));
    if (fill_w > 0) {
        _display.fillRect(x + 1, y + 1, fill_w, h - 2, SSD1306_WHITE);
    }
}

void OLEDDisplay::drawGaugeBar(int16_t x, int16_t y, int16_t w, int16_t h, float value, float min_val, float max_val) {
    float percent = ((value - min_val) / (max_val - min_val)) * 100.0;
    percent = constrain(percent, 0, 100);
    drawProgressBar(x, y, w, h, percent);
}

Adafruit_SSD1306* OLEDDisplay::getDisplay() {
    return &_display;
}

// ═══════════════════════════════════════════════════════════════════
// FORMATTING HELPERS
// ═══════════════════════════════════════════════════════════════════

String OLEDDisplay::formatSpeed(float speed_kmh) {
    return String((int)speed_kmh) + " km/h";
}

String OLEDDisplay::formatLeanAngle(float angle) {
    if (fabs(angle) < 1.0) return "0.0";
    
    char buffer[16];
    if (angle < 0) {
        snprintf(buffer, sizeof(buffer), "%.1fL", -angle);
    } else {
        snprintf(buffer, sizeof(buffer), "%.1fR", angle);
    }
    return String(buffer);
}

String OLEDDisplay::formatGForce(float g) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.1fg", g);
    return String(buffer);
}

String OLEDDisplay::formatVoltage(float voltage) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.1fV", voltage);
    return String(buffer);
}

String OLEDDisplay::formatCoordinates(double lat, double lon) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.5f,%.5f", lat, lon);
    return String(buffer);
}
