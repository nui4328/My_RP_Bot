#ifndef MY_GYRO160_H
#define MY_GYRO160_H

#include <Wire.h>

class my_GYRO160 {
public:
  static const uint8_t my_GYRO160_CHIP_ID = 0x00; // Register for chip ID
  static const uint8_t my_GYRO160_CMD = 0x7E;     // Command register
  static const uint8_t my_GYRO160_ACCEL_CONF = 0x40; // Accelerometer config
  static const uint8_t my_GYRO160_ACCEL_RANGE = 0x41; // Accelerometer range
  static const uint8_t my_GYRO160_GYRO_CONF = 0x42;  // Gyroscope config
  static const uint8_t my_GYRO160_GYRO_RANGE = 0x43; // Gyroscope range
  static const uint8_t my_GYRO160_GYRO_DATA = 0x0C;  // Data register start

  // Constants
  static const float GYRO_THRESHOLD; // Gyroscope threshold for deadzone
  static const float ALPHA;         // Complementary filter constant
  static const float ACCEL_FILTER_ALPHA; // Accelerometer low-pass filter constant

  // Constructor
  my_GYRO160(uint8_t address = 0x69);

  // Methods
  bool begin();
  void readAngles(float &roll, float &pitch, float &yaw);
  float gyro(char axis);
  void resetYaw();
  void resetAngles();

private:
  uint8_t _address;
  static unsigned long _lastTime;
  static float _angleX;
  static float _angleY;
  static float _angleZ;
  static float _gyroOffsetX;
  static float _gyroOffsetY;
  static float _gyroOffsetZ;
  static float _accelX_prev;
  static float _accelY_prev;
  static float _accelZ_prev;

  void writeRegister(uint8_t reg, uint8_t value);
  void readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
  bool calibrateGyro();
};

// Function to reset gyro angles
void reset_gyro160(my_GYRO160& gyro);

#endif