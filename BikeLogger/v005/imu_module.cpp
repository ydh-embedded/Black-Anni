/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  IMU MODULE - ICM-20948 Implementation                          ║
 * ║  ⭐⭐⭐⭐⭐ LEAN ANGLE DETECTION BIS 64°!                            ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "imu_module.h"

// Global IMU Instance
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
    writeRegister(ICM20948_ACCEL_CONFIG, 0x01);  // DLPF enabled, 246 Hz
    
    // Set Sample Rate Divider (Bank 2)
    setSampleRateDivider(10);  // 100Hz (1125 / (1 + 10) = 102Hz)
    
    // Back to Bank 0
    selectBank(0);
    
    // Initialize Magnetometer
    if (!initMagnetometer()) {
        Serial.println("⚠️  Magnetometer init fehlgeschlagen (nicht kritisch)");
    } else {
        Serial.println("✓ Magnetometer (AK09916) initialisiert");
    }
    
    delay(100);
    
    return true;
}

bool ICM20948::isConnected() {
    return (whoAmI() == ICM20948_DEVICE_ID);
}

uint8_t ICM20948::whoAmI() {
    selectBank(0);
    return readRegister(ICM20948_WHO_AM_I);
}

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

void ICM20948::setAccelRange(uint8_t range) {
    selectBank(2);
    
    uint8_t config = 0;
    switch(range) {
        case 2:  config = 0x00; _accel_scale = 16384.0; break;  // ±2g
        case 4:  config = 0x02; _accel_scale = 8192.0;  break;  // ±4g
        case 8:  config = 0x04; _accel_scale = 4096.0;  break;  // ±8g
        case 16: config = 0x06; _accel_scale = 2048.0;  break;  // ±16g
        default: config = 0x06; _accel_scale = 2048.0;  break;  // ±16g default
    }
    
    _accel_range = range;
    writeRegister(ICM20948_ACCEL_CONFIG, config | 0x01);  // + DLPF
    
    selectBank(0);
}

void ICM20948::setGyroRange(uint16_t range) {
    selectBank(2);
    
    uint8_t config = 0;
    switch(range) {
        case 250:  config = 0x00; _gyro_scale = 131.0;  break;  // ±250°/s
        case 500:  config = 0x02; _gyro_scale = 65.5;   break;  // ±500°/s
        case 1000: config = 0x04; _gyro_scale = 32.8;   break;  // ±1000°/s
        case 2000: config = 0x06; _gyro_scale = 16.4;   break;  // ±2000°/s
        default:   config = 0x06; _gyro_scale = 16.4;   break;  // ±2000°/s default
    }
    
    _gyro_range = range;
    writeRegister(ICM20948_GYRO_CONFIG_1, config | 0x01);  // + DLPF
    
    selectBank(0);
}

void ICM20948::setSampleRateDivider(uint16_t divider) {
    selectBank(2);
    
    // Gyro Sample Rate Divider
    writeRegister(ICM20948_GYRO_SMPLRT_DIV, divider & 0xFF);
    
    // Accel Sample Rate Divider (12-bit)
    writeRegister(ICM20948_ACCEL_SMPLRT_DIV_1, (divider >> 8) & 0xFF);
    writeRegister(ICM20948_ACCEL_SMPLRT_DIV_2, divider & 0xFF);
    
    selectBank(0);
}

void ICM20948::enableLowPowerMode(bool enable) {
    selectBank(0);
    if (enable) {
        writeRegister(ICM20948_LP_CONFIG, 0x40);  // Duty Cycled Mode
    } else {
        writeRegister(ICM20948_LP_CONFIG, 0x00);  // Continuous Mode
    }
}

// ═══════════════════════════════════════════════════════════════════
// CALIBRATION
// ═══════════════════════════════════════════════════════════════════

void ICM20948::calibrate(uint16_t samples) {
    Serial.println("🔧 Starte IMU Kalibrierung...");
    Serial.println("   → Fahrzeug MUSS STILL STEHEN und GERADE sein!");
    
    // Reset offsets
    for (int i = 0; i < 3; i++) {
        _accel_offset[i] = 0;
        _gyro_offset[i] = 0;
    }
    
    // Accumulate samples
    float accel_sum[3] = {0, 0, 0};
    float gyro_sum[3] = {0, 0, 0};
    
    for (uint16_t i = 0; i < samples; i++) {
        readRawData();
        
        for (int j = 0; j < 3; j++) {
            accel_sum[j] += _accel[j];
            gyro_sum[j] += _gyro[j];
        }
        
        if (i % 100 == 0) {
            Serial.printf("   Sample %d/%d\r", i, samples);
        }
        
        delay(10);  // 10ms between samples
    }
    
    // Calculate averages
    for (int i = 0; i < 3; i++) {
        _gyro_offset[i] = gyro_sum[i] / samples;
        _accel_offset[i] = accel_sum[i] / samples;
    }
    
    // Accel Z sollte bei ~1g sein (Schwerkraft)
    _accel_offset[2] -= 1.0;
    
    _calibrated = true;
    
    Serial.println("\n✅ Kalibrierung abgeschlossen!");
    Serial.printf("   Accel Offset: [%.3f, %.3f, %.3f] g\n", 
                  _accel_offset[0], _accel_offset[1], _accel_offset[2]);
    Serial.printf("   Gyro Offset:  [%.3f, %.3f, %.3f] °/s\n", 
                  _gyro_offset[0], _gyro_offset[1], _gyro_offset[2]);
}

void ICM20948::resetCalibration() {
    _calibrated = false;
    for (int i = 0; i < 3; i++) {
        _accel_offset[i] = 0;
        _gyro_offset[i] = 0;
    }
}

bool ICM20948::isCalibrated() {
    return _calibrated;
}

// ═══════════════════════════════════════════════════════════════════
// DATA UPDATE
// ═══════════════════════════════════════════════════════════════════

bool ICM20948::update() {
    // Calculate delta time
    uint32_t now = millis();
    if (_last_update > 0) {
        _dt = (now - _last_update) / 1000.0;  // Convert to seconds
    }
    _last_update = now;
    
    // Read all sensor data
    if (!readRawData()) {
        _error_count++;
        return false;
    }
    
    // Process data
    processAccelData();
    processGyroData();
    
    // Calculate derived values
    calculateAttitude();
    calculateLeanAngle();
    calculateGForce();
    detectMotion();
    analyzeVibration();
    
    // Read magnetometer (less frequent)
    static uint8_t mag_counter = 0;
    if (++mag_counter >= 10) {  // Every 10th update (~10Hz if IMU is 100Hz)
        mag_counter = 0;
        processMagData();
        calculateCompass();
    }
    
    return true;
}

bool ICM20948::readRawData() {
    selectBank(0);
    
    uint8_t buffer[14];
    readRegisters(ICM20948_ACCEL_XOUT_H, buffer, 14);
    
    // Accelerometer (bytes 0-5)
    _accel_raw[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    _accel_raw[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    _accel_raw[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Gyroscope (bytes 6-11)
    _gyro_raw[0] = (int16_t)((buffer[6] << 8) | buffer[7]);
    _gyro_raw[1] = (int16_t)((buffer[8] << 8) | buffer[9]);
    _gyro_raw[2] = (int16_t)((buffer[10] << 8) | buffer[11]);
    
    // Temperature (bytes 12-13)
    _temp_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // Convert to physical units
    for (int i = 0; i < 3; i++) {
        _accel[i] = _accel_raw[i] / _accel_scale;
        _gyro[i] = _gyro_raw[i] / _gyro_scale;
    }
    
    // Temperature: ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21°C
    _temperature = ((_temp_raw - 0) / 333.87) + 21.0;
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// SENSOR DATA READING
// ═══════════════════════════════════════════════════════════════════

void ICM20948::readAccel(float& x, float& y, float& z) {
    x = _accel[0] - _accel_offset[0];
    y = _accel[1] - _accel_offset[1];
    z = _accel[2] - _accel_offset[2];
}

void ICM20948::readAccelRaw(int16_t& x, int16_t& y, int16_t& z) {
    x = _accel_raw[0];
    y = _accel_raw[1];
    z = _accel_raw[2];
}

void ICM20948::readGyro(float& x, float& y, float& z) {
    x = _gyro[0] - _gyro_offset[0];
    y = _gyro[1] - _gyro_offset[1];
    z = _gyro[2] - _gyro_offset[2];
}

void ICM20948::readGyroRaw(int16_t& x, int16_t& y, int16_t& z) {
    x = _gyro_raw[0];
    y = _gyro_raw[1];
    z = _gyro_raw[2];
}

void ICM20948::readMag(float& x, float& y, float& z) {
    x = (_mag[0] - _mag_offset[0]) * _mag_scale[0];
    y = (_mag[1] - _mag_offset[1]) * _mag_scale[1];
    z = (_mag[2] - _mag_offset[2]) * _mag_scale[2];
}

void ICM20948::readMagRaw(int16_t& x, int16_t& y, int16_t& z) {
    x = _mag_raw[0];
    y = _mag_raw[1];
    z = _mag_raw[2];
}

float ICM20948::readTemperature() {
    return _temperature;
}

// ═══════════════════════════════════════════════════════════════════
// PROCESSING FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void ICM20948::processAccelData() {
    // Apply calibration offsets
    float ax = _accel[0] - _accel_offset[0];
    float ay = _accel[1] - _accel_offset[1];
    float az = _accel[2] - _accel_offset[2];
    
    // Low-pass filter to reduce noise
    static float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
    ax_filtered = lowPassFilter(ax, ax_filtered, 0.3);
    ay_filtered = lowPassFilter(ay, ay_filtered, 0.3);
    az_filtered = lowPassFilter(az, az_filtered, 0.3);
    
    _accel[0] = ax_filtered;
    _accel[1] = ay_filtered;
    _accel[2] = az_filtered;
}

void ICM20948::processGyroData() {
    // Apply calibration offsets
    _gyro[0] -= _gyro_offset[0];
    _gyro[1] -= _gyro_offset[1];
    _gyro[2] -= _gyro_offset[2];
    
    // Deadband filter für Gyro (Drift-Reduktion)
    _gyro[0] = deadbandFilter(_gyro[0], 0.5);
    _gyro[1] = deadbandFilter(_gyro[1], 0.5);
    _gyro[2] = deadbandFilter(_gyro[2], 0.5);
}

void ICM20948::processMagData() {
    if (!isMagReady()) return;
    
    uint8_t buffer[8];
    readMagRegisters(AK09916_HXL, buffer, 8);
    
    // Check DRDY bit (bit 0 of ST2)
    if (!(buffer[7] & 0x01)) return;
    
    // Read magnetometer data
    _mag_raw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    _mag_raw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    _mag_raw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convert to µT
    for (int i = 0; i < 3; i++) {
        _mag[i] = _mag_raw[i] * _mag_scale_factor;
    }
}

void ICM20948::calculateAttitude() {
    // Pitch & Roll from Accelerometer
    float accel_pitch = atan2(_accel[0], sqrt(_accel[1]*_accel[1] + _accel[2]*_accel[2])) * RAD_TO_DEG;
    float accel_roll = atan2(_accel[1], _accel[2]) * RAD_TO_DEG;
    
    // Integrate Gyroscope
    _pitch += _gyro[0] * _dt;
    _roll += _gyro[1] * _dt;
    _yaw += _gyro[2] * _dt;
    
    // Complementary Filter (96% Gyro, 4% Accel)
    _pitch = 0.96 * _pitch + 0.04 * accel_pitch;
    _roll = 0.96 * _roll + 0.04 * accel_roll;
    
    // Keep yaw in range 0-360°
    _yaw = fmod(_yaw + 360.0, 360.0);
}

void ICM20948::calculateLeanAngle() {
    // Lean Angle = Roll (Schräglage links/rechts)
    _lean_angle = _roll;
    
    // Limit to physical maximum
    _lean_angle = constrain(_lean_angle, -MAX_LEAN_ANGLE_DEG, MAX_LEAN_ANGLE_DEG);
    
    // Track maximum lean angles
    if (_lean_angle < 0 && -_lean_angle > _max_lean_left) {
        _max_lean_left = -_lean_angle;
    }
    if (_lean_angle > 0 && _lean_angle > _max_lean_right) {
        _max_lean_right = _lean_angle;
    }
    
    _prev_roll = _roll;
}

void ICM20948::calculateGForce() {
    // G-Force in Fahrzeug-Koordinaten
    calculateGForce(_accel[0], _accel[1], _accel[2],
                   _pitch, _roll, _yaw,
                   _g_forward, _g_lateral, _g_vertical);
    
    // Totale G-Force
    _g_total = calculateTotalGForce(_g_forward, _g_lateral, _g_vertical);
}

void ICM20948::detectMotion() {
    // Wheelie Detection
    _wheelie_detected = detectWheelie(_pitch, _g_vertical);
    
    // Stoppie Detection
    _stoppie_detected = detectStoppie(_pitch, _g_forward);
    
    // Cornering Detection (needs speed from GPS)
    // _is_cornering = detectCornering(_lean_angle, _g_lateral, speed_kmh);
    _is_cornering = (fabs(_lean_angle) > 10.0 || fabs(_g_lateral) > 0.3);
}

void ICM20948::analyzeVibration() {
    // Calculate total acceleration magnitude (ohne Schwerkraft)
    float accel_mag = sqrt(_accel[0]*_accel[0] + _accel[1]*_accel[1] + (_accel[2]-1.0)*(_accel[2]-1.0));
    
    // Add to rolling buffer
    _vibration_buffer[_vibration_index] = accel_mag;
    _vibration_index = (_vibration_index + 1) % 32;
    
    // Calculate RMS vibration level
    _vibration_level = calculateVibrationRMS(_vibration_buffer, 32);
}

void ICM20948::calculateCompass() {
    // Tilt-compensated compass heading
    float cos_pitch = cos(_pitch * DEG_TO_RAD);
    float sin_pitch = sin(_pitch * DEG_TO_RAD);
    float cos_roll = cos(_roll * DEG_TO_RAD);
    float sin_roll = sin(_roll * DEG_TO_RAD);
    
    // Tilt compensation
    float mag_x = _mag[0] * cos_pitch + _mag[2] * sin_pitch;
    float mag_y = _mag[0] * sin_roll * sin_pitch + _mag[1] * cos_roll - _mag[2] * sin_roll * cos_pitch;
    
    // Calculate heading
    _compass_heading = atan2(mag_y, mag_x) * RAD_TO_DEG;
    
    // Normalize to 0-360°
    if (_compass_heading < 0) _compass_heading += 360.0;
}

// ═══════════════════════════════════════════════════════════════════
// GETTERS
// ═══════════════════════════════════════════════════════════════════

float ICM20948::getLeanAngle() { return _lean_angle; }
float ICM20948::getMaxLeanLeft() { return _max_lean_left; }
float ICM20948::getMaxLeanRight() { return _max_lean_right; }
void ICM20948::resetMaxLean() { _max_lean_left = 0; _max_lean_right = 0; }

float ICM20948::getPitch() { return _pitch; }
float ICM20948::getRoll() { return _roll; }
float ICM20948::getYaw() { return _yaw; }

float ICM20948::getGForward() { return _g_forward; }
float ICM20948::getGLateral() { return _g_lateral; }
float ICM20948::getGVertical() { return _g_vertical; }
float ICM20948::getGTotal() { return _g_total; }

bool ICM20948::isWheelie() { return _wheelie_detected; }
bool ICM20948::isStoppie() { return _stoppie_detected; }
bool ICM20948::isCornering() { return _is_cornering; }
float ICM20948::getCorneringSpeed() { return _cornering_speed; }

float ICM20948::getVibrationLevel() { return _vibration_level; }
float ICM20948::getVibrationFrequency() { return _vibration_freq; }

float ICM20948::getCompassHeading() { return _compass_heading; }

IMUData ICM20948::getData() {
    IMUData data;
    
    // Raw Data
    data.accel_x = _accel[0];
    data.accel_y = _accel[1];
    data.accel_z = _accel[2];
    data.gyro_x = _gyro[0];
    data.gyro_y = _gyro[1];
    data.gyro_z = _gyro[2];
    data.mag_x = _mag[0];
    data.mag_y = _mag[1];
    data.mag_z = _mag[2];
    
    // Attitude
    data.pitch = _pitch;
    data.roll = _roll;
    data.yaw = _yaw;
    
    // G-Force
    data.g_forward = _g_forward;
    data.g_lateral = _g_lateral;
    data.g_vertical = _g_vertical;
    data.g_total = _g_total;
    
    // Lean Angle
    data.lean_angle = _lean_angle;
    data.max_lean_left = _max_lean_left;
    data.max_lean_right = _max_lean_right;
    
    // Compass
    data.compass_heading = _compass_heading;
    
    // Motion
    data.wheelie_detected = _wheelie_detected;
    data.stoppie_detected = _stoppie_detected;
    data.is_cornering = _is_cornering;
    data.cornering_speed = _cornering_speed;
    
    // Vibration
    data.vibration_level = _vibration_level;
    data.vibration_freq = _vibration_freq;
    
    // Calibration
    data.calibrated = _calibrated;
    data.accel_offset_x = _accel_offset[0];
    data.accel_offset_y = _accel_offset[1];
    data.accel_offset_z = _accel_offset[2];
    data.gyro_offset_x = _gyro_offset[0];
    data.gyro_offset_y = _gyro_offset[1];
    data.gyro_offset_z = _gyro_offset[2];
    
    // Temperature
    data.temperature = _temperature;
    
    // Timestamp
    data.millis_at_reading = millis();
    
    return data;
}

// ═══════════════════════════════════════════════════════════════════
// MAGNETOMETER
// ═══════════════════════════════════════════════════════════════════

bool ICM20948::initMagnetometer() {
    // Enable I2C Master Mode
    selectBank(0);
    writeRegister(ICM20948_USER_CTRL, 0x20);  // I2C_MST_EN
    delay(10);
    
    // Configure I2C Master
    selectBank(3);
    writeRegister(0x01, 0x07);  // I2C_MST_CTRL: 400kHz
    
    // Reset Magnetometer
    writeMagRegister(AK09916_CNTL3, 0x01);  // SRST
    delay(100);
    
    // Check WHO_AM_I
    uint8_t mag_id = readMagRegister(AK09916_WIA2);
    if (mag_id != AK09916_DEVICE_ID) {
        return false;
    }
    
    // Set to Continuous Mode 4 (100Hz)
    writeMagRegister(AK09916_CNTL2, 0x08);
    delay(10);
    
    selectBank(0);
    return true;
}

bool ICM20948::isMagReady() {
    uint8_t st1 = readMagRegister(AK09916_ST1);
    return (st1 & 0x01);  // DRDY bit
}

// ═══════════════════════════════════════════════════════════════════
// I2C COMMUNICATION
// ═══════════════════════════════════════════════════════════════════

void ICM20948::selectBank(uint8_t bank) {
    writeRegister(ICM20948_REG_BANK_SEL, bank << 4);
}

bool ICM20948::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t ICM20948::readRegister(uint8_t reg) {
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_i2c_addr, (uint8_t)1);
    return Wire.read();
}

void ICM20948::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_i2c_addr, length);
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
}

bool ICM20948::writeMagRegister(uint8_t reg, uint8_t value) {
    selectBank(3);
    writeRegister(0x03, AK09916_I2C_ADDR);  // I2C_SLV0_ADDR (write)
    writeRegister(0x04, reg);               // I2C_SLV0_REG
    writeRegister(0x06, value);             // I2C_SLV0_DO
    writeRegister(0x05, 0x81);              // I2C_SLV0_CTRL (enable, 1 byte)
    delay(10);
    selectBank(0);
    return true;
}

uint8_t ICM20948::readMagRegister(uint8_t reg) {
    selectBank(3);
    writeRegister(0x03, AK09916_I2C_ADDR | 0x80);  // I2C_SLV0_ADDR (read)
    writeRegister(0x04, reg);                       // I2C_SLV0_REG
    writeRegister(0x05, 0x81);                      // I2C_SLV0_CTRL (enable, 1 byte)
    delay(10);
    selectBank(0);
    return readRegister(0x3B);  // EXT_SLV_SENS_DATA_00
}

void ICM20948::readMagRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    selectBank(3);
    writeRegister(0x03, AK09916_I2C_ADDR | 0x80);
    writeRegister(0x04, reg);
    writeRegister(0x05, 0x80 | length);
    delay(10);
    selectBank(0);
    readRegisters(0x3B, buffer, length);
}

// ═══════════════════════════════════════════════════════════════════
// ERROR HANDLING
// ═══════════════════════════════════════════════════════════════════

uint16_t ICM20948::getErrorCount() {
    return _error_count;
}

void ICM20948::resetErrorCount() {
    _error_count = 0;
}

// ═══════════════════════════════════════════════════════════════════
// SELF TEST
// ═══════════════════════════════════════════════════════════════════

bool ICM20948::selfTest() {
    Serial.println("🧪 Starte IMU Self-Test...");
    
    // Check connection
    if (!isConnected()) {
        Serial.println("❌ IMU nicht verbunden!");
        return false;
    }
    
    // Read some samples
    for (int i = 0; i < 10; i++) {
        readRawData();
        delay(10);
    }
    
    // Check if data is reasonable
    bool accel_ok = (fabs(_accel[2]) > 0.8 && fabs(_accel[2]) < 1.2);  // Z ~1g
    bool gyro_ok = (fabs(_gyro[0]) < 10.0 && fabs(_gyro[1]) < 10.0 && fabs(_gyro[2]) < 10.0);
    
    if (accel_ok && gyro_ok) {
        Serial.println("✅ Self-Test erfolgreich!");
        return true;
    } else {
        Serial.println("❌ Self-Test fehlgeschlagen!");
        if (!accel_ok) Serial.println("   Accelerometer außerhalb Toleranz");
        if (!gyro_ok) Serial.println("   Gyroscope außerhalb Toleranz");
        return false;
    }
}
