/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  LED BARGRAPH MODULE - 74HC595 Shift Register (FIXED)          ║
 * ║  FIXME: Critical Bug #4 - Enum mismatch corrected              ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "led_bargraph.h"

// Global LED Bargraph Instance
LEDBargraph ledBar;

// ═══════════════════════════════════════════════════════════════════
// CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════════

LEDBargraph::LEDBargraph() {
    _mode = LED_MODE_SPEED;  // Default Mode
    _current_pattern = 0;
    _target_pattern = 0;
    _last_update = 0;
    _animation_step = 0;
    
    // Default Ranges
    _speed_min = 0;
    _speed_max = 200;
    _lean_min = 0;
    _lean_max = 64;
}

// ═══════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::begin() {
    // Configure GPIO Pins
    pinMode(LED_DATA_PIN, OUTPUT);
    pinMode(LED_CLOCK_PIN, OUTPUT);
    pinMode(LED_LATCH_PIN, OUTPUT);
    
    // Initialize pins to LOW
    digitalWrite(LED_DATA_PIN, LOW);
    digitalWrite(LED_CLOCK_PIN, LOW);
    digitalWrite(LED_LATCH_PIN, LOW);
    
    // Clear all LEDs
    clear();
    
    // Startup Animation
    animationStartup();
    
    Serial.println("✓ LED Bargraph initialized!");
}

// ═══════════════════════════════════════════════════════════════════
// UPDATE FUNCTION
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::update(const TelemetryData& data) {
    // FIXME: Critical Bug #4 - Use enum type directly, not integer comparison
    switch (_mode) {
        case LED_MODE_SPEED:
            _target_pattern = calculateSpeedPattern(data.gps.speed_kmh);
            break;
            
        case LED_MODE_RPM:
            // RPM Mode (wenn verfügbar)
            // _target_pattern = calculateRPMPattern(data.rpm);
            break;
            
        case LED_MODE_LEAN:
            _target_pattern = calculateLeanPattern(fabs(data.imu.lean_angle));
            break;
            
        case LED_MODE_GFORCE:
            _target_pattern = calculateGForcePattern(data.imu.g_total);
            break;
            
        case LED_MODE_CUSTOM:
            // Custom pattern bleibt unverändert
            break;
            
        case LED_MODE_OFF:
            _target_pattern = 0;
            break;
            
        default:
            // Fallback: Speed Mode
            _target_pattern = calculateSpeedPattern(data.gps.speed_kmh);
            break;
    }
    
    // Smooth Transition
    smoothTransition(_target_pattern);
    
    // Update Shift Register
    updateShiftRegister();
}

// ═══════════════════════════════════════════════════════════════════
// MODE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::setMode(LEDMode mode) {
    // FIXME: Critical Bug #4 - Validate enum value is in valid range
    if (mode >= LED_MODE_SPEED && mode <= LED_MODE_OFF) {
        _mode = mode;
        Serial.printf("🎨 LED Mode changed to: %d\n", _mode);
    } else {
        Serial.printf("⚠️  Invalid LED Mode: %d (using default)\n", mode);
        _mode = LED_MODE_SPEED;  // Fallback
    }
}

LEDMode LEDBargraph::getMode() {
    return _mode;
}

// ═══════════════════════════════════════════════════════════════════
// MANUAL CONTROL
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::setLevel(uint8_t level) {
    // Convert level (0-12) to pattern
    if (level > 12) level = 12;
    _target_pattern = levelToPattern(level);
    updateShiftRegister();
}

void LEDBargraph::setPattern(uint16_t pattern) {
    _target_pattern = pattern & 0x0FFF;  // Mask to 12 bits
    updateShiftRegister();
}

void LEDBargraph::setLED(uint8_t led, bool state) {
    if (led > 12) return;
    
    uint16_t mask = 1 << (led - 1);
    if (state) {
        _current_pattern |= mask;
    } else {
        _current_pattern &= ~mask;
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
    // Map speed to LED level (0-12)
    float range = _speed_max - _speed_min;
    if (range <= 0) return 0;
    
    float normalized = (speed_kmh - _speed_min) / range;
    normalized = constrain(normalized, 0.0, 1.0);
    
    uint8_t level = (uint8_t)(normalized * 12.0);
    return levelToPattern(level);
}

uint16_t LEDBargraph::calculateRPMPattern(float rpm) {
    // Map RPM to LED level (0-12)
    // Assuming 0-10000 RPM range
    float normalized = rpm / 10000.0;
    normalized = constrain(normalized, 0.0, 1.0);
    
    uint8_t level = (uint8_t)(normalized * 12.0);
    return levelToPattern(level);
}

uint16_t LEDBargraph::calculateLeanPattern(float lean_deg) {
    // Map lean angle to LED level (0-12)
    float range = _lean_max - _lean_min;
    if (range <= 0) return 0;
    
    float normalized = (lean_deg - _lean_min) / range;
    normalized = constrain(normalized, 0.0, 1.0);
    
    uint8_t level = (uint8_t)(normalized * 12.0);
    return levelToPattern(level);
}

uint16_t LEDBargraph::calculateGForcePattern(float g_force) {
    // Map G-Force to LED level (0-12)
    // Assuming 0-5g range
    float normalized = g_force / 5.0;
    normalized = constrain(normalized, 0.0, 1.0);
    
    uint8_t level = (uint8_t)(normalized * 12.0);
    return levelToPattern(level);
}

uint16_t LEDBargraph::levelToPattern(uint8_t level) {
    // Convert level (0-12) to 12-bit pattern
    // LED 1-8 = Green, LED 9-12 = Red
    
    if (level == 0) return 0x0000;
    if (level >= 12) return 0x0FFF;  // All 12 LEDs on
    
    // Create pattern with 'level' LEDs lit
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
    // Smooth transition from current to target pattern
    // Prevents flickering/jitter
    
    uint32_t now = millis();
    if (now - _last_update < 50) return;  // Update max 20Hz
    
    _last_update = now;
    
    // Simple hysteresis: only update if difference is significant
    if (_current_pattern != target) {
        _current_pattern = target;
    }
}

// ═══════════════════════════════════════════════════════════════════
// SHIFT REGISTER CONTROL
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::shiftOut(uint16_t data) {
    // Shift out 16 bits (actually only 12 used)
    // 74HC595 is 8-bit, so we need 2 cascaded chips
    
    // Shift out MSB first
    for (int i = 15; i >= 0; i--) {
        // Set data line
        digitalWrite(LED_DATA_PIN, (data >> i) & 1);
        
        // Clock pulse
        digitalWrite(LED_CLOCK_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(LED_CLOCK_PIN, LOW);
        delayMicroseconds(1);
    }
}

void LEDBargraph::updateShiftRegister() {
    // Shift out the pattern
    shiftOut(_current_pattern);
    
    // Latch pulse (transfer to output)
    digitalWrite(LED_LATCH_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LED_LATCH_PIN, LOW);
    delayMicroseconds(1);
}

// ═══════════════════════════════════════════════════════════════════
// ANIMATIONS
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::animationStartup() {
    // Knight Rider Effect beim Start
    for (uint8_t i = 0; i < 3; i++) {
        animationKnightRider();
    }
    
    // All LEDs on briefly
    setPattern(0x0FFF);
    delay(200);
    
    // Clear
    clear();
}

void LEDBargraph::animationShutdown() {
    // Fade out effect
    for (uint8_t level = 12; level > 0; level--) {
        setLevel(level);
        delay(50);
    }
    
    clear();
}

void LEDBargraph::animationKnightRider() {
    // Classic Knight Rider animation
    for (uint8_t i = 0; i < 12; i++) {
        setPattern(1 << i);
        delay(50);
    }
    
    for (int i = 10; i > 0; i--) {
        setPattern(1 << i);
        delay(50);
    }
}

void LEDBargraph::animationWarning() {
    // Flashing red LEDs (9-12)
    for (uint8_t i = 0; i < 5; i++) {
        setPattern(0x0F00);  // LEDs 9-12
        delay(100);
        clear();
        delay(100);
    }
}

// ═══════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void LEDBargraph::clear() {
    _current_pattern = 0;
    _target_pattern = 0;
    updateShiftRegister();
}

uint16_t LEDBargraph::getCurrentPattern() {
    return _current_pattern;
}
