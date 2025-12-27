/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  LED BARGRAPH MODULE - 74HC595 Implementation                   ║
 * ║  💡 12-LED Live Visualisierung                                  ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "led_bargraph.h"

// Global LED Bargraph Instance
LEDBargraph ledBar;

// ═══════════════════════════════════════════════════════════════════
// CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════════

LEDBargraph::LEDBargraph() {
    _mode = LED_MODE_SPEED;
    _current_pattern = 0x0000;
    _target_pattern = 0x0000;
    
    // Default Ranges
    _speed_min = LED_SPEED_MIN;
    _speed_max = LED_SPEED_MAX;
    _lean_min = 0;
    _lean_max = MAX_LEAN_ANGLE_DEG;
    
    _last_update = 0;
    _animation_step = 0;
}

// ═══════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::begin() {
    // Configure Pins
    pinMode(LED_DATA_PIN, OUTPUT);
    pinMode(LED_CLOCK_PIN, OUTPUT);
    pinMode(LED_LATCH_PIN, OUTPUT);
    
    // Clear all LEDs
    clear();
    
    Serial.println("✓ LED Bargraph initialisiert");
    
    // Startup Animation
    animationStartup();
}

// ═══════════════════════════════════════════════════════════════════
// DISPLAY UPDATE
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::update(const TelemetryData& data) {
    uint16_t target = 0;
    
    switch (_mode) {
        case LED_MODE_SPEED:
            target = calculateSpeedPattern(data.gps.speed_kmh);
            break;
            
        case LED_MODE_RPM:
            target = calculateRPMPattern(data.vehicle.engine_rpm);
            break;
            
        case LED_MODE_LEAN:
            target = calculateLeanPattern(fabs(data.imu.lean_angle));
            break;
            
        case LED_MODE_GFORCE:
            target = calculateGForcePattern(data.imu.g_total);
            break;
            
        case LED_MODE_CUSTOM:
            target = _target_pattern;
            break;
            
        case LED_MODE_OFF:
            target = 0x0000;
            break;
    }
    
    // Smooth Transition
    smoothTransition(target);
    
    _last_update = millis();
}

void LEDBargraph::clear() {
    _current_pattern = 0x0000;
    _target_pattern = 0x0000;
    updateShiftRegister();
}

// ═══════════════════════════════════════════════════════════════════
// MODE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::setMode(LEDMode mode) {
    _mode = mode;
}

LEDMode LEDBargraph::getMode() {
    return _mode;
}

// ═══════════════════════════════════════════════════════════════════
// MANUAL CONTROL
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::setLevel(uint8_t level) {
    level = constrain(level, 0, LED_COUNT);
    _target_pattern = levelToPattern(level);
    _current_pattern = _target_pattern;
    updateShiftRegister();
}

void LEDBargraph::setPattern(uint16_t pattern) {
    _target_pattern = pattern & 0x0FFF;  // 12 bits
    _current_pattern = _target_pattern;
    updateShiftRegister();
}

void LEDBargraph::setLED(uint8_t led, bool state) {
    if (led >= LED_COUNT) return;
    
    if (state) {
        _current_pattern |= (1 << led);
    } else {
        _current_pattern &= ~(1 << led);
    }
    
    updateShiftRegister();
}

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::setSpeedRange(float min_kmh, float max_kmh) {
    _speed_min = min_kmh;
    _speed_max = max_kmh;
}

void LEDBargraph::setLeanRange(float min_deg, float max_deg) {
    _lean_min = min_deg;
    _lean_max = max_deg;
}

// ═══════════════════════════════════════════════════════════════════
// PATTERN CALCULATION
// ═══════════════════════════════════════════════════════════════════

uint16_t LEDBargraph::calculateSpeedPattern(float speed_kmh) {
    // Map Speed zu LED Level (0-12)
    speed_kmh = constrain(speed_kmh, _speed_min, _speed_max);
    
    float percent = (speed_kmh - _speed_min) / (_speed_max - _speed_min);
    uint8_t level = (uint8_t)(percent * LED_COUNT);
    
    return levelToPattern(level);
}

uint16_t LEDBargraph::calculateRPMPattern(float rpm) {
    // Beispiel: 0-12000 RPM → 0-12 LEDs
    float rpm_max = 12000.0;
    float percent = constrain(rpm / rpm_max, 0.0, 1.0);
    uint8_t level = (uint8_t)(percent * LED_COUNT);
    
    return levelToPattern(level);
}

uint16_t LEDBargraph::calculateLeanPattern(float lean_deg) {
    // Map Lean Angle zu LED Level
    lean_deg = constrain(lean_deg, _lean_min, _lean_max);
    
    float percent = (lean_deg - _lean_min) / (_lean_max - _lean_min);
    uint8_t level = (uint8_t)(percent * LED_COUNT);
    
    return levelToPattern(level);
}

uint16_t LEDBargraph::calculateGForcePattern(float g_force) {
    // Map G-Force zu LED Level (0-3g → 0-12 LEDs)
    float g_max = 3.0;
    float percent = constrain(g_force / g_max, 0.0, 1.0);
    uint8_t level = (uint8_t)(percent * LED_COUNT);
    
    return levelToPattern(level);
}

uint16_t LEDBargraph::levelToPattern(uint8_t level) {
    if (level == 0) return 0x0000;
    if (level >= LED_COUNT) return 0x0FFF;  // Alle 12 LEDs an
    
    // Level LEDs einschalten (LSB first)
    uint16_t pattern = 0;
    for (uint8_t i = 0; i < level; i++) {
        pattern |= (1 << i);
    }
    
    return pattern;
}

// ═══════════════════════════════════════════════════════════════════
// SMOOTH TRANSITION
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::smoothTransition(uint16_t target) {
    _target_pattern = target;
    
    // Sofort umschalten (für jetzt - könnte mit Animation erweitert werden)
    _current_pattern = _target_pattern;
    
    updateShiftRegister();
}

// ═══════════════════════════════════════════════════════════════════
// SHIFT REGISTER CONTROL
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::shiftOut(uint16_t data) {
    // 12 bits schieben (2 Bytes, nur 12 bits verwendet)
    // Bit 0-11 = LED 1-12
    
    digitalWrite(LED_LATCH_PIN, LOW);
    
    // Schiebe 12 bits (MSB first für 74HC595)
    for (int8_t i = 11; i >= 0; i--) {
        digitalWrite(LED_DATA_PIN, (data >> i) & 0x01);
        
        // Clock Pulse
        digitalWrite(LED_CLOCK_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(LED_CLOCK_PIN, LOW);
        delayMicroseconds(1);
    }
    
    // Latch (Output aktivieren)
    digitalWrite(LED_LATCH_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LED_LATCH_PIN, LOW);
}

void LEDBargraph::updateShiftRegister() {
    shiftOut(_current_pattern);
}

// ═══════════════════════════════════════════════════════════════════
// ANIMATIONS
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::animationStartup() {
    // LEDs nacheinander einschalten
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        setLED(i, true);
        delay(50);
    }
    
    delay(200);
    
    // Alle ausschalten
    clear();
    delay(100);
    
    // Kurz alle blinken
    setPattern(0x0FFF);
    delay(100);
    clear();
    delay(100);
    setPattern(0x0FFF);
    delay(100);
    clear();
}

void LEDBargraph::animationShutdown() {
    // LEDs nacheinander ausschalten
    for (int8_t i = LED_COUNT - 1; i >= 0; i--) {
        setLED(i, false);
        delay(50);
    }
}

void LEDBargraph::animationKnightRider() {
    // Knight Rider Style Animation
    for (uint8_t repeat = 0; repeat < 3; repeat++) {
        // Vorwärts
        for (uint8_t i = 0; i < LED_COUNT; i++) {
            setPattern(1 << i);
            delay(30);
        }
        
        // Rückwärts
        for (int8_t i = LED_COUNT - 1; i >= 0; i--) {
            setPattern(1 << i);
            delay(30);
        }
    }
    
    clear();
}

void LEDBargraph::animationWarning() {
    // Blink-Animation für Warnung
    for (uint8_t i = 0; i < 6; i++) {
        // Nur rote LEDs (LED 9-12)
        setPattern(0x0F00);  // Bits 8-11
        delay(150);
        clear();
        delay(150);
    }
}

// ═══════════════════════════════════════════════════════════════════
// GETTERS
// ═══════════════════════════════════════════════════════════════════

uint16_t LEDBargraph::getCurrentPattern() {
    return _current_pattern;
}
