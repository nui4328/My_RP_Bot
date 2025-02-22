#include "my_GYRO.h"

my_GYRO::my_GYRO() {
  gyroOffsetX = 0;
  gyroOffsetY = 0;
  gyroOffsetZ = 0;
  angleX = 0;
  angleY = 0;
  angleZ = 0;
  lastTime = 0;
}

bool my_GYRO::begin() {
  Wire.begin();
  if (!initGYRO()) {
    return false;
  }
  calibrateGyro();
  return true;
}

float my_GYRO::gyro(char xyz) {
  int16_t gx, gy, gz;
  readGyro(gx, gy, gz);

  // ปรับค่าด้วยออฟเซ็ตจากการคาริเบรท
  gx -= gyroOffsetX;
  gy -= gyroOffsetY;
  gz -= gyroOffsetZ;

  // คำนวณมุมจากไจโรสโคป
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;  // แปลงเวลาเป็นวินาที
  lastTime = currentTime;

  // คำนวณมุม (ใช้ sensitivity scale factor 65.5 สำหรับ ±500 dps)
  angleX += (gx / 65) * dt;
  angleY += (gy / 65) * dt;
  angleZ += (gz / 17) * dt;  // ใช้ 65.5 สำหรับ ±500 dps

  // คืนค่ามุมตามแกนที่ต้องการ
  switch (xyz) {
    case 'x': return angleX;
    case 'y': return angleY;
    case 'z': return angleZ;
    default: return 0.0;
  }
}

void my_GYRO::resetAngles() {
  angleX = 0;
  angleY = 0;
  angleZ = 0;
}

void my_GYRO::calibrateGyro() {
  int16_t gxSum = 0, gySum = 0, gzSum = 0;
  const int numSamples = 500;

  for (int i = 0; i < numSamples; i++) {
    int16_t gx, gy, gz;
    readGyro(gx, gy, gz);

    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(5);
  }

  // คำนวณออฟเซ็ต
  gyroOffsetX = gxSum / numSamples;
  gyroOffsetY = gySum / numSamples;
  gyroOffsetZ = gzSum / numSamples;
}

bool my_GYRO::initGYRO() {
  uint8_t id = readRegister(0x00);
  if (id != 0xD1) {  // GYRO มี ID 0xD1
    return false;
  }

  // ตั้งค่าโหมดปกติ (เปิดเฉพาะไจโรสโคป)
  writeRegister(0x7E, 0x15);  // เปิดไจโรสโคป
  delay(50);  // รอให้เซ็นเซอร์เริ่มต้น

  return true;
}

void my_GYRO::readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  gx = readRegister16(0x0C);  // อ่านค่าไจโรสโคป X
  gy = readRegister16(0x0E);  // อ่านค่าไจโรสโคป Y
  gz = readRegister16(0x10);  // อ่านค่าไจโรสโคป Z
}

void my_GYRO::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(my_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t my_GYRO::readRegister(uint8_t reg) {
  Wire.beginTransmission(my_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(my_ADDRESS, 1);
  return Wire.read();
}

int16_t my_GYRO::readRegister16(uint8_t reg) {
  Wire.beginTransmission(my_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(my_ADDRESS, 2);
  return (int16_t)(Wire.read() | (Wire.read() << 8));
}