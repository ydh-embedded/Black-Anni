/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  IMU MODULE - ICM-20948 Implementation (FIXED)                  ║
 * ║  FIXME: Critical Bug #3 - Global instance properly initialized  ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "imu_module.h"

// FIXME: Critical Bug #3 - Global IMU instance properly initialized
// Constructor is called automatically, initializing all members
ICM20948 imu;

// ═══════════════════════════════════════════════════════════════════
// CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════════

ICM20948::ICM20948() {
    _i2c_addr = I2C_ADDR_ICM20948;
    _calibrated = false;
    _error_count = 0;
    _last_update = 0;
    _dt = 0.01;  // 10ms default
    
    // Reset all data
    for (int i = 0; i < 3; i++) {
        _accel_raw[i] = 0;
        _gyro_raw[i] = 0;
        _mag_raw[i] = 0;
        _accel[i] = 0;
        _gyro[i] = 0;
        _mag[i] = 0;
        _accel_offset[i] = 0;
        _gyro_offset[i] = 0;
        _mag_offset[i] = 0;
        _mag_scale[i] = 1.0;
    }
    
    _pitch = _roll = _yaw = 0;
    _prev_roll = 0;
    _lean_angle = 0;
    _max_lean_left = 0;
    _max_lean_right = 0;
    _g_forward = _g_lateral = _g_vertical = _g_total = 0;
    _wheelie_detected = false;
    _stoppie_detected = false;
    _is_cornering = false;
    _cornering_speed = 0;
    _vibration_level = 0;
    _vibration_freq = 0;
    _vibration_index = 0;
    _compass_heading = 0;
    _temperature = 0;
    
    // Default ranges
    _accel_range = ACCEL_RANGE_G;
    _gyro_range = GYRO_RANGE_DPS;
    _accel_scale = ACCEL_SENSITIVITY;
    _gyro_scale = GYRO_SENSITIVITY;
    _mag_scale_factor = MAG_SENSITIVITY;
}

// ═══════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

bool ICM20948::begin(uint8_t i2c_addr) {
    _i2c_addr = i2c_addr;
    
    // Check WHO_AM_I
    selectBank(0);
    uint8_t id = whoAmI();
    
    if (id != ICM20948_DEVICE_ID) {
        Serial.printf("❌ ICM-20948 WHO_AM_I failed: 0x%02X (expected 0x%02X)\n", 
                      id, ICM20948_DEVICE_ID);
        return false;
    }
    
    Serial.println("✓ ICM-20948 gefunden!");
    
    // Reset Device
    selectBank(0);
    writeRegister(ICM20948_PWR_MGMT_1, 0x80);  // Device Reset
    delay(100);
    
    // Wake up device
    writeRegister(ICM20948_PWR_MGMT_1, 0x01);  // Auto select clock
    delay(10);
    
    // Enable Accelerometer & Gyroscope
    writeRegister(ICM20948_PWR_MGMT_2, 0x00);  // All sensors on
    delay(10);
    
    // Configure Gyroscope (Bank 2)
    selectBank(2);
    setGyroRange(_gyro_range);
    writeRegister(ICM20948_GYRO_CONFIG_1, 0x01);  // DLPF enabled, 196.6 Hz
    
    // Configure Accelerometer (Bank 2)
    setAccelRange(_accel_range);
    writeRegister(ICM20948_ACCEL_CONFIG_2, 0x01);  // DLPF enabled, 246 Hz
    
    // Configure Magnetometer (AK09916)
    initMagnetometer();
    
    // Self-Test
    if (!selfTest()) {
        Serial.println("⚠️  IMU Self-Test fehlgeschlagen!");
        return false;
    }
    
    return true;
}

bool ICM20948::isConnected() {
    selectBank(0);
    uint8_t id = whoAmI();
    return (id == ICM20948_DEVICE_ID);
}

uint8_t ICM20948::whoAmI() {
    return readRegister(ICM20948_WHO_AM_I);
}

// ═══════════════════════════════════════════════════════════════════
// SENSOR CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

void ICM20948::setAccelRange(uint8_t range) {
    selectBank(2);
    
    uint8_t config = 0;
    switch (range) {
        case 2:
            config = 0x00;
            _accel_scale = 16384.0;  // LSB/g
            break;
        case 4:
            config = 0x08;
            _accel_scale = 8192.0;
            break;
        case 8:
            config = 0x10;
            _accel_scale = 4096.0;
            break;
        case 16:
            config = 0x18;
            _accel_scale = 2048.0;  // LSB/g @ ±16g
            break;
        default:
            config = 0x18;
            _accel_scale = 2048.0;
    }
    
    writeRegister(ICM20948_ACCEL_CONFIG, config);
    _accel_range = range;
}

void ICM20948::setGyroRange(uint8_t range) {
    selectBank(2);
    
    uint8_t config = 0;
    switch (range) {
        case 250:
            config = 0x00;
            _gyro_scale = 131.0;  // LSB/(°/s)
            break;
        case 500:
            config = 0x08;
            _gyro_scale = 65.5;
            break;
        case 1000:
            config = 0x10;
            _gyro_scale = 32.8;
            break;
        case 2000:
            config = 0x18;
            _gyro_scale = 16.4;  // LSB/(°/s) @ ±2000°/s
            break;
        default:
            config = 0x18;
            _gyro_scale = 16.4;
    }
    
    writeRegister(ICM20948_GYRO_CONFIG_1, config | 0x01);  // DLPF enabled
    _gyro_range = range;
}

// ═══════════════════════════════════════════════════════════════════
// MAGNETOMETER INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

void ICM20948::initMagnetometer() {
    // Enable I2C Master Mode
    selectBank(0);
    writeRegister(ICM20948_USER_CTRL, 0x20);  // I2C Master Enable
    delay(10);
    
    // Configure I2C Master
    selectBank(3);
    writeRegister(0x01, 0x4D);  // I2C Master Clock 400kHz
    
    // Read AK09916 WHO_AM_I
    selectBank(0);
    uint8_t mag_id = readMagRegister(AK09916_WIA2);
    
    if (mag_id != AK09916_DEVICE_ID) {
        Serial.printf("⚠️  AK09916 Magnetometer not found (ID: 0x%02X)\n", mag_id);
        return;
    }
    
    // Configure Magnetometer (Continuous Mode 2, 100Hz)
    writeMagRegister(AK09916_CNTL2, 0x08);  // Mode 2
    
    Serial.println("✓ AK09916 Magnetometer initialized!");
}

// ═══════════════════════════════════════════════════════════════════
// SELF-TEST
// ═══════════════════════════════════════════════════════════════════

bool ICM20948::selfTest() {
    // Simplified self-test: just check if we can read data
    selectBank(0);
    
    // Read a few samples
    for (int i = 0; i < 10; i++) {
        readRawData();
        delay(10);
    }
    
    // Check if accelerometer is reading reasonable values (should be ~1g on Z)
    // This is a basic sanity check
    if (fabs(_accel[2]) < 0.5 || fabs(_accel[2]) > 1.5) {
        return false;  // Z-axis should be ~1g (gravity)
    }
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// DATA READING
// ═══════════════════════════════════════════════════════════════════

bool ICM20948::update() {
    uint32_t now = millis();
    _dt = (now - _last_update) / 1000.0;
    _last_update = now;
    
    if (!readRawData()) {
        _error_count++;
        return false;
    }
    
    // Convert raw to physical units
    convertRawToPhysical();
    
    // Calibration (if available)
    if (_calibrated) {
        applyCalibration();
    }
    
    // Calculate Lean Angle (main feature!)
    calculateLeanAngle();
    
    // Calculate G-Forces
    calculateGForces();
    
    // Detect Motion Events
    detectMotionEvents();
    
    // Update Compass Heading
    updateCompassHeading();
    
    return true;
}

bool ICM20948::readRawData() {
    selectBank(0);
    
    // Read Accelerometer (6 bytes)
    Wire.beginTransmission(_i2c_addr);
    Wire.write(ICM20948_ACCEL_XOUT_H);
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(_i2c_addr, (uint8_t)6);
    if (Wire.available() != 6) return false;
    
    _accel_raw[0] = (Wire.read() << 8) | Wire.read();
    _accel_raw[1] = (Wire.read() << 8) | Wire.read();
    _accel_raw[2] = (Wire.read() << 8) | Wire.read();
    
    // Read Gyroscope (6 bytes)
    Wire.beginTransmission(_i2c_addr);
    Wire.write(ICM20948_GYRO_XOUT_H);
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(_i2c_addr, (uint8_t)6);
    if (Wire.available() != 6) return false;
    
    _gyro_raw[0] = (Wire.read() << 8) | Wire.read();
    _gyro_raw[1] = (Wire.read() << 8) | Wire.read();
    _gyro_raw[2] = (Wire.read() << 8) | Wire.read();
    
    // Read Temperature (2 bytes)
    Wire.beginTransmission(_i2c_addr);
    Wire.write(ICM20948_TEMP_OUT_H);
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(_i2c_addr, (uint8_t)2);
    if (Wire.available() != 2) return false;
    
    int16_t temp_raw = (Wire.read() << 8) | Wire.read();
    _temperature = (temp_raw / 333.87) + 21.0;  // Temperature in °C
    
    return true;
}

void ICM20948::convertRawToPhysical() {
    // Convert raw accelerometer values to g
    _accel[0] = _accel_raw[0] / _accel_scale;
    _accel[1] = _accel_raw[1] / _accel_scale;
    _accel[2] = _accel_raw[2] / _accel_scale;
    
    // Convert raw gyroscope values to °/s
    _gyro[0] = _gyro_raw[0] / _gyro_scale;
    _gyro[1] = _gyro_raw[1] / _gyro_scale;
    _gyro[2] = _gyro_raw[2] / _gyro_scale;
}

void ICM20948::applyCalibration() {
    // Apply offset calibration
    for (int i = 0; i < 3; i++) {
        _accel[i] -= _accel_offset[i];
        _gyro[i] -= _gyro_offset[i];
    }
}

// ═══════════════════════════════════════════════════════════════════
// LEAN ANGLE CALCULATION (⭐ KILLER FEATURE!)
// ═══════════════════════════════════════════════════════════════════

void ICM20948::calculateLeanAngle() {
    // Roll = atan2(Y, Z) * 180/PI
    // Pitch = atan2(-X, sqrt(Y^2 + Z^2)) * 180/PI
    
    _roll = atan2(_accel[1], _accel[2]) * 180.0 / PI;
    _pitch = atan2(-_accel[0], sqrt(_accel[1]*_accel[1] + _accel[2]*_accel[2])) * 180.0 / PI;
    
    // Lean angle = Roll (positive = right, negative = left)
    _lean_angle = _roll;
    
    // Track maximum lean angles
    if (_lean_angle > _max_lean_right) {
        _max_lean_right = _lean_angle;
    }
    if (_lean_angle < -_max_lean_left) {
        _max_lean_left = -_lean_angle;
    }
}

// ═══════════════════════════════════════════════════════════════════
// G-FORCE CALCULATION
// ═══════════════════════════════════════════════════════════════════

void ICM20948::calculateGForces() {
    // G-Force components (in motorcycle frame)
    _g_forward = -_accel[0];   // Forward acceleration
    _g_lateral = _accel[1];    // Lateral (cornering)
    _g_vertical = _accel[2];   // Vertical (includes gravity)
    
    // Total G-Force (excluding gravity)
    float g_horiz = sqrt(_g_forward*_g_forward + _g_lateral*_g_lateral);
    _g_total = sqrt(g_horiz*g_horiz + (_g_vertical-1.0)*(_g_vertical-1.0));
}

// ═══════════════════════════════════════════════════════════════════
// MOTION EVENT DETECTION
// ═══════════════════════════════════════════════════════════════════

void ICM20948::detectMotionEvents() {
    // Wheelie Detection: High positive pitch + high acceleration
    if (_pitch > 20.0 && _g_forward > 0.5) {
        _wheelie_detected = true;
    } else {
        _wheelie_detected = false;
    }
    
    // Stoppie Detection: High negative pitch + high deceleration
    if (_pitch < -20.0 && _g_forward < -0.5) {
        _stoppie_detected = true;
    } else {
        _stoppie_detected = false;
    }
    
    // Cornering Detection: High lateral G-Force + lean angle
    if (fabs(_g_lateral) > 0.5 && fabs(_lean_angle) > 10.0) {
        _is_cornering = true;
    } else {
        _is_cornering = false;
    }
}

// ═══════════════════════════════════════════════════════════════════
// COMPASS HEADING
// ═══════════════════════════════════════════════════════════════════

void ICM20948::updateCompassHeading() {
    // Read magnetometer data
    readMagnetometerData();
    
    // Calculate heading from magnetometer
    float heading = atan2(_mag[1], _mag[0]) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
    
    _compass_heading = heading;
}

// ═══════════════════════════════════════════════════════════════════
// DATA RETRIEVAL
// ═══════════════════════════════════════════════════════════════════

IMUData ICM20948::getData() {
    IMUData data;
    
    data.accel_x = _accel[0];
    data.accel_y = _accel[1];
    data.accel_z = _accel[2];
    
    data.gyro_x = _gyro[0];
    data.gyro_y = _gyro[1];
    data.gyro_z = _gyro[2];
    
    data.mag_x = _mag[0];
    data.mag_y = _mag[1];
    data.mag_z = _mag[2];
    
    data.pitch = _pitch;
    data.roll = _roll;
    data.yaw = _yaw;
    
    data.lean_angle = _lean_angle;
    data.max_lean_left = _max_lean_left;
    data.max_lean_right = _max_lean_right;
    
    data.g_forward = _g_forward;
    data.g_lateral = _g_lateral;
    data.g_vertical = _g_vertical;
    data.g_total = _g_total;
    
    data.wheelie_detected = _wheelie_detected;
    data.stoppie_detected = _stoppie_detected;
    data.is_cornering = _is_cornering;
    
    data.compass_heading = _compass_heading;
    data.temperature = _temperature;
    data.calibrated = _calibrated;
    
    return data;
}

// ═══════════════════════════════════════════════════════════════════
// REGISTER ACCESS
// ═══════════════════════════════════════════════════════════════════

void ICM20948::selectBank(uint8_t bank) {
    writeRegister(ICM20948_REG_BANK_SEL, bank << 4);
    delayMicroseconds(100);
}

uint8_t ICM20948::readRegister(uint8_t reg) {
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(_i2c_addr, (uint8_t)1);
    return Wire.read();
}

void ICM20948::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t ICM20948::readMagRegister(uint8_t reg) {
    // Read from magnetometer via I2C Master
    selectBank(3);
    writeRegister(0x00, AK09916_I2C_ADDR | 0x80);  // Read mode
    writeRegister(0x01, reg);
    delay(1);
    
    selectBank(0);
    return readRegister(0x3D);  // EXT_SLV_SENS_DATA_00
}

void ICM20948::writeMagRegister(uint8_t reg, uint8_t value) {
    // Write to magnetometer via I2C Master
    selectBank(3);
    writeRegister(0x00, AK09916_I2C_ADDR);  // Write mode
    writeRegister(0x01, reg);
    writeRegister(0x02, value);
}

void ICM20948::readMagnetometerData() {
    // Read 3-axis magnetometer data
    // Implementation depends on I2C Master configuration
    // Placeholder for now
}

// ═══════════════════════════════════════════════════════════════════
// CALIBRATION
// ═══════════════════════════════════════════════════════════════════

void ICM20948::calibrate() {
    Serial.println("⏳ Kalibriere IMU (bitte still halten)...");
    
    // Reset offsets
    for (int i = 0; i < 3; i++) {
        _accel_offset[i] = 0;
        _gyro_offset[i] = 0;
    }
    
    // Collect samples
    float accel_sum[3] = {0, 0, 0};
    float gyro_sum[3] = {0, 0, 0};
    
    for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
        readRawData();
        convertRawToPhysical();
        
        for (int j = 0; j < 3; j++) {
            accel_sum[j] += _accel[j];
            gyro_sum[j] += _gyro[j];
        }
        
        delay(10);
    }
    
    // Calculate offsets
    for (int i = 0; i < 3; i++) {
        _accel_offset[i] = accel_sum[i] / IMU_CALIBRATION_SAMPLES;
        _gyro_offset[i] = gyro_sum[i] / IMU_CALIBRATION_SAMPLES;
    }
    
    // Z-axis should have 1g offset
    _accel_offset[2] -= 1.0;
    
    _calibrated = true;
    Serial.println("✓ IMU Kalibrierung abgeschlossen!");
}

uint16_t ICM20948::getErrorCount() {
    return _error_count;
}

void ICM20948::resetErrorCount() {
    _error_count = 0;
}
