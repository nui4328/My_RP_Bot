// ----------------------- my_GYRO160.h -----------------------
#ifndef MY_GYRO160_H
#define MY_GYRO160_H

#include <Arduino.h>
#include <Wire.h>

class my_GYRO160 {
public:
  // BMI160 Register Map (สำคัญมาก)
  static const uint8_t BMI160_CHIP_ID       = 0x00;
  static const uint8_t BMI160_DATA          = 0x0C;  // เริ่มต้นที่ GYRO_X
  static const uint8_t BMI160_ACCEL_CONF    = 0x40;
  static const uint8_t BMI160_ACCEL_RANGE   = 0x41;
  static const uint8_t BMI160_GYRO_CONF     = 0x42;
  static const uint8_t BMI160_GYRO_RANGE    = 0x43;
  static const uint8_t BMI160_CMD           = 0x7E;

  // ค่าคงที่ที่ปรับแล้วให้ดีที่สุด
  static const float GYRO_DEADZONE;          // ตัด noise (หน่วย: °/s)
  static const float ALPHA;                  // Complementary filter (0.98 = gyro 98%)
  static const float ACCEL_FILTER_ALPHA;    // Low-pass filter สำหรับ accel

  my_GYRO160(uint8_t address = 0x69);  // 0x68 หรือ 0x69 ขึ้นกับ SDO pin

  bool begin();                        // เริ่มต้น + calibrate
  void readAngles(float &roll, float &pitch, float &yaw);
  float gyro(char axis);               // 'x', 'y', 'z'
  void resetYaw();
  void resetAngles();

private:
  uint8_t _address;

  static unsigned long _lastTime;
  static float _angleX, _angleY, _angleZ;
  static float _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ;
  static float _accelX_prev, _accelY_prev, _accelZ_prev;

  void writeRegister(uint8_t reg, uint8_t value);
  void readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
  bool calibrateGyro();
};

void reset_gyro160(my_GYRO160& gyro);

#endif