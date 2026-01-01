// ----------------------- my_GYRO160.h -----------------------
#ifndef MY_GYRO160_H
#define MY_GYRO160_H

#include <Arduino.h>
#include <Wire.h>

class my_GYRO160 {
public:
  // BMI160 Register Map (ค่าที่ถูกต้องตามโค้ดของคุณ)
  static const uint8_t BMI160_CHIP_ID       = 0x00;
  static const uint8_t BMI160_DATA          = 0x0C;  // GYRO data starts here
  static const uint8_t BMI160_ACCEL_CONF    = 0x40;
  static const uint8_t BMI160_ACCEL_RANGE   = 0x41;
  static const uint8_t BMI160_GYRO_CONF     = 0x42;
  static const uint8_t BMI160_GYRO_RANGE    = 0x43;
  static const uint8_t BMI160_CMD           = 0x7E;

  // ค่าคงที่ที่ใช้ใน .cpp
  static const float GYRO_DEADZONE;
  static const float ALPHA;
  static const float ACCEL_FILTER_ALPHA;
  static const float BIAS_ALPHA;

  my_GYRO160(uint8_t address = 0x69);

  bool begin();
  void readAngles(float &roll, float &pitch, float &yaw);
  float gyro(char axis);
  void resetYaw();
  void resetAngles();

  // ฟังก์ชันที่เพิ่มมาเพื่อแก้ drift
  bool isStationary();
  void reCalibrateGyro();
  void resetRunningBias();  // สำคัญมาก

private:
  uint8_t _address;

  // Static variables
  static unsigned long _lastTime;
  static float _angleX;
  static float _angleY;
  static float _angleZ;
  static float _gyroOffsetX;
  static float _gyroOffsetY;
  static float _gyroOffsetZ;
  static float _runningBiasZ;  // ต้องมีใน header

  static float _accelX_prev;
  static float _accelY_prev;
  static float _accelZ_prev;

  void writeRegister(uint8_t reg, uint8_t value);
  void readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
  bool calibrateGyro();
};

void reset_gyro160(my_GYRO160& gyro);

#endif