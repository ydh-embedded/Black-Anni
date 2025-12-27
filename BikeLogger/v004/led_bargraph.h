/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  LED BARGRAPH MODULE - 74HC595 Shift Register                  ║
 * ║  💡 12-LED Anzeige (8 grün + 4 rot)                            ║
 * ║                                                                  ║
 * ║  Features:                                                       ║
 * ║  - Speed Mode (0-200 km/h)                                      ║
 * ║  - Lean Angle Mode (0-64°)                                      ║
 * ║  - RPM Mode (optional)                                          ║
 * ║  - Custom Pattern Mode                                          ║
 * ║  - Smooth Animations                                            ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef LED_BARGRAPH_H
#define LED_BARGRAPH_H

#include <Arduino.h>
#include "../config.h"
#include "../data_structures.h"

// ═══════════════════════════════════════════════════════════════════
// LED MODES
// ═══════════════════════════════════════════════════════════════════

enum LEDMode {
    LED_MODE_SPEED = 0,     // Geschwindigkeit (0-200 km/h)
    LED_MODE_RPM = 1,       // Drehzahl (falls verfügbar)
    LED_MODE_LEAN = 2,      // Lean Angle (0-64°)
    LED_MODE_GFORCE = 3,    // G-Force (0-3g)
    LED_MODE_CUSTOM = 4,    // Custom Pattern
    LED_MODE_OFF = 5        // Alle LEDs aus
};

// ═══════════════════════════════════════════════════════════════════
// LED BARGRAPH CLASS
// ═══════════════════════════════════════════════════════════════════

class LEDBargraph {
public:
    LEDBargraph();
    
    // Initialisierung
    void begin();
    
    // Display Update
    void update(const TelemetryData& data);
    void clear();
    
    // Mode Management
    void setMode(LEDMode mode);
    LEDMode getMode();
    
    // Manual Control
    void setLevel(uint8_t level);  // 0-12 LEDs
    void setPattern(uint16_t pattern);  // 12-bit Pattern
    void setLED(uint8_t led, bool state);  // Einzelne LED setzen
    
    // Speed Mode Configuration
    void setSpeedRange(float min_kmh, float max_kmh);
    
    // Lean Angle Mode Configuration
    void setLeanRange(float min_deg, float max_deg);
    
    // Animations
    void animationStartup();
    void animationShutdown();
    void animationKnightRider();
    void animationWarning();
    
    // Brightness (via PWM auf RCLK - nicht implementiert auf 74HC595)
    // void setBrightness(uint8_t brightness);  // 0-255
    
    // Get Current Pattern
    uint16_t getCurrentPattern();

private:
    // Mode
    LEDMode _mode;
    
    // Current Pattern (12-bit, LSB = LED1, MSB = LED12)
    uint16_t _current_pattern;
    uint16_t _target_pattern;
    
    // Configuration
    float _speed_min;
    float _speed_max;
    float _lean_min;
    float _lean_max;
    
    // Timing
    uint32_t _last_update;
    uint8_t _animation_step;
    
    // Shift Register Control
    void shiftOut(uint16_t data);
    void updateShiftRegister();
    
    // Pattern Calculation
    uint16_t calculateSpeedPattern(float speed_kmh);
    uint16_t calculateRPMPattern(float rpm);
    uint16_t calculateLeanPattern(float lean_deg);
    uint16_t calculateGForcePattern(float g_force);
    uint16_t levelToPattern(uint8_t level);
    
    // Smooth Transition
    void smoothTransition(uint16_t target);
};

// ═══════════════════════════════════════════════════════════════════
// GLOBAL LED BARGRAPH INSTANCE
// ═══════════════════════════════════════════════════════════════════

extern LEDBargraph ledBar;

#endif // LED_BARGRAPH_H
