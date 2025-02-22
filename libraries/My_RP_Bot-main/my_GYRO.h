#ifndef my_GYRO_H
#define my_GYRO_H

#include <Wire.h>

#define my_ADDRESS 0x69  // ที่อยู่ I2C ของ my

class my_GYRO {
public:
  my_GYRO();
  bool begin();
  float gyro(char xyz);
  void resetAngles();
  void calibrateGyro();

private:
  float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
  float angleX, angleY, angleZ;
  unsigned long lastTime;

  bool initGYRO();
  void readGyro(int16_t &gx, int16_t &gy, int16_t &gz);
  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
  int16_t readRegister16(uint8_t reg);
};

#endif