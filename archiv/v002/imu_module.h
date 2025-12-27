/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  IMU MODULE - ICM-20948 (9-DoF)                                 ║
 * ║  ⭐⭐⭐⭐⭐ KILLER FEATURE: LEAN ANGLE DETECTION!                    ║
 * ║                                                                  ║
 * ║  Features:                                                       ║
 * ║  - Lean Angle bis 64° (Links/Rechts)                           ║
 * ║  - G-Force Messung (Forward/Lateral/Vertical)                   ║
 * ║  - Wheelie/Stoppie Detection                                    ║
 * ║  - Cornering Detection                                          ║
 * ║  - Vibration Analysis                                           ║
 * ║  - Compass Heading                                              ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include "../config.h"
#include "../data_structures.h"
#include "../calculations.h"

// ═══════════════════════════════════════════════════════════════════
// ICM-20948 REGISTER DEFINITIONS
// ═══════════════════════════════════════════════════════════════════

// User Bank 0 Registers
#define ICM20948_WHO_AM_I           0x00
#define ICM20948_USER_CTRL           0x03
#define ICM20948_LP_CONFIG           0x05
#define ICM20948_PWR_MGMT_1          0x06
#define ICM20948_PWR_MGMT_2          0x07
#define ICM20948_INT_PIN_CFG         0x0F
#define ICM20948_INT_ENABLE          0x10
#define ICM20948_INT_ENABLE_1        0x11
#define ICM20948_INT_ENABLE_2        0x12
#define ICM20948_INT_ENABLE_3        0x13
#define ICM20948_ACCEL_XOUT_H        0x2D
#define ICM20948_GYRO_XOUT_H         0x33
#define ICM20948_TEMP_OUT_H          0x39
#define ICM20948_REG_BANK_SEL        0x7F

// User Bank 2 Registers
#define ICM20948_GYRO_SMPLRT_DIV     0x00
#define ICM20948_GYRO_CONFIG_1       0x01
#define ICM20948_GYRO_CONFIG_2       0x02
#define ICM20948_ACCEL_SMPLRT_DIV_1  0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2  0x11
#define ICM20948_ACCEL_CONFIG        0x14
#define ICM20948_ACCEL_CONFIG_2      0x15

// Magnetometer (AK09916) Registers
#define AK09916_I2C_ADDR            0x0C
#define AK09916_WIA2                0x01
#define AK09916_ST1                 0x10
#define AK09916_HXL                 0x11
#define AK09916_CNTL2               0x31
#define AK09916_CNTL3               0x32

// Device IDs
#define ICM20948_DEVICE_ID          0xEA
#define AK09916_DEVICE_ID           0x09

// ═══════════════════════════════════════════════════════════════════
// ICM-20948 CLASS
// ═══════════════════════════════════════════════════════════════════

class ICM20948 {
public:
    ICM20948();
    
    // Initialisierung
    bool begin(uint8_t i2c_addr = I2C_ADDR_ICM20948);
    bool isConnected();
    uint8_t whoAmI();
    
    // Konfiguration
    void setAccelRange(uint8_t range);  // 2, 4, 8, 16 g
    void setGyroRange(uint16_t range);  // 250, 500, 1000, 2000 dps
    void setSampleRateDivider(uint16_t divider);
    void enableLowPowerMode(bool enable);
    
    // Kalibrierung
    void calibrate(uint16_t samples = IMU_CALIBRATION_SAMPLES);
    void resetCalibration();
    bool isCalibrated();
    
    // Daten lesen
    bool update();
    bool readRawData();
    bool readProcessedData();
    
    // Accelerometer
    void readAccel(float& x, float& y, float& z);
    void readAccelRaw(int16_t& x, int16_t& y, int16_t& z);
    
    // Gyroscope
    void readGyro(float& x, float& y, float& z);
    void readGyroRaw(int16_t& x, int16_t& y, int16_t& z);
    
    // Magnetometer
    void readMag(float& x, float& y, float& z);
    void readMagRaw(int16_t& x, int16_t& y, int16_t& z);
    bool isMagReady();
    
    // Temperature
    float readTemperature();
    
    // ⭐ LEAN ANGLE (Hauptfeature!)
    float getLeanAngle();
    float getMaxLeanLeft();
    float getMaxLeanRight();
    void resetMaxLean();
    
    // Attitude (Lage)
    float getPitch();
    float getRoll();
    float getYaw();
    
    // G-Force
    float getGForward();
    float getGLateral();
    float getGVertical();
    float getGTotal();
    
    // Motion Detection
    bool isWheelie();
    bool isStoppie();
    bool isCornering();
    float getCorneringSpeed();
    
    // Vibration
    float getVibrationLevel();
    float getVibrationFrequency();
    
    // Compass
    float getCompassHeading();
    
    // Get Complete Data Structure
    IMUData getData();
    
    // Self-Test
    bool selfTest();
    
    // Error Handling
    uint16_t getErrorCount();
    void resetErrorCount();

private:
    // I2C Communication
    uint8_t _i2c_addr;
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
    void selectBank(uint8_t bank);
    
    // Magnetometer I2C
    bool writeMagRegister(uint8_t reg, uint8_t value);
    uint8_t readMagRegister(uint8_t reg);
    void readMagRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // Raw Sensor Data
    int16_t _accel_raw[3];
    int16_t _gyro_raw[3];
    int16_t _mag_raw[3];
    int16_t _temp_raw;
    
    // Processed Data
    float _accel[3];        // g
    float _gyro[3];         // °/s
    float _mag[3];          // µT
    float _temperature;     // °C
    
    // Attitude
    float _pitch, _roll, _yaw;
    float _prev_roll;  // Für Complementary Filter
    
    // Lean Angle
    float _lean_angle;
    float _max_lean_left;
    float _max_lean_right;
    
    // G-Force
    float _g_forward, _g_lateral, _g_vertical, _g_total;
    
    // Motion Detection
    bool _wheelie_detected;
    bool _stoppie_detected;
    bool _is_cornering;
    float _cornering_speed;
    
    // Vibration
    float _vibration_level;
    float _vibration_freq;
    float _vibration_buffer[32];  // Rolling buffer
    uint8_t _vibration_index;
    
    // Compass
    float _compass_heading;
    
    // Calibration
    bool _calibrated;
    float _accel_offset[3];
    float _gyro_offset[3];
    float _mag_offset[3];
    float _mag_scale[3];
    
    // Timing
    uint32_t _last_update;
    float _dt;  // Delta time in seconds
    
    // Configuration
    uint8_t _accel_range;   // g
    uint16_t _gyro_range;   // dps
    float _accel_scale;
    float _gyro_scale;
    float _mag_scale_factor;
    
    // Error Tracking
    uint16_t _error_count;
    
    // Processing Functions
    void processAccelData();
    void processGyroData();
    void processMagData();
    void calculateAttitude();
    void calculateLeanAngle();
    void calculateGForce();
    void detectMotion();
    void analyzeVibration();
    void calculateCompass();
    
    // Magnetometer Initialization
    bool initMagnetometer();
};

// ═══════════════════════════════════════════════════════════════════
// GLOBAL IMU INSTANCE (wird in .cpp definiert)
// ═══════════════════════════════════════════════════════════════════

extern ICM20948 imu;

// ═══════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

// Quick Access Functions (benutzen globales imu Objekt)
inline float getLeanAngle() { return imu.getLeanAngle(); }
inline float getGTotal() { return imu.getGTotal(); }
inline bool isWheelie() { return imu.isWheelie(); }
inline bool isStoppie() { return imu.isStoppie(); }
inline bool isCornering() { return imu.isCornering(); }

#endif // IMU_MODULE_H
