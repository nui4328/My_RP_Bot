// ----------------------- my_GYRO160.cpp (เวอร์ชันแก้ error + warning สนิท) -----------------------
#include "my_GYRO160s.h"
#include <math.h>  // สำหรับ sqrt(), abs()

// ลบ #define RAD_TO_DEG ทิ้งไปเลย (Arduino มีอยู่แล้ว และค่าตรงกัน ไม่ต้องกำหนดซ้ำ)

// ค่าคงที่ที่ปรับแล้วให้ดีที่สุด
const float my_GYRO160::GYRO_DEADZONE       = 0.12f;   // ตัด noise ที่ทำให้ลอย
const float my_GYRO160::ALPHA              = 0.98f;   // เชื่อ gyro มาก
const float my_GYRO160::ACCEL_FILTER_ALPHA = 0.10f;   // กรอง accel เบา ๆ
const float my_GYRO160::BIAS_ALPHA         = 0.02f;   // เร่ง dynamic bias

// Low-pass filter สำหรับ gyro Z
static float gyroZ_filtered = 0.0f;
const float GYRO_LPF_ALPHA = 0.15f;  // 15% ค่าใหม่

// Static variables
unsigned long my_GYRO160::_lastTime = 0;
float my_GYRO160::_angleX = 0.0f;
float my_GYRO160::_angleY = 0.0f;
float my_GYRO160::_angleZ = 0.0f;
float my_GYRO160::_gyroOffsetX = 0.0f;
float my_GYRO160::_gyroOffsetY = 0.0f;
float my_GYRO160::_gyroOffsetZ = 0.0f;
float my_GYRO160::_runningBiasZ = 0.0f;
float my_GYRO160::_accelX_prev = 0.0f;
float my_GYRO160::_accelY_prev = 0.0f;
float my_GYRO160::_accelZ_prev = 0.0f;

// Constructor
my_GYRO160::my_GYRO160(uint8_t address) : _address(address) {}

// เริ่มต้นเซ็นเซอร์ (แก้การอ่าน Chip ID ให้ถูกต้อง - ไม่ต้อง dummy write)
bool my_GYRO160::begin() {
  Wire.begin();
  Wire.setClock(400000);

  // อ่าน Chip ID แบบถูกต้อง
  Wire.beginTransmission(_address);
  Wire.write(BMI160_CHIP_ID);
  Wire.endTransmission();
  Wire.requestFrom(_address, (uint8_t)1);
  uint8_t chipID = 0;
  if (Wire.available()) chipID = Wire.read();
  if (chipID != 0xD1) return false;

  // Soft reset
  writeRegister(BMI160_CMD, 0xB6);
  delay(100);

  // Normal mode
  writeRegister(BMI160_CMD, 0x11);  // ACCEL normal
  delay(10);
  writeRegister(BMI160_CMD, 0x15);  // GYRO normal
  delay(10);

  // Config
  writeRegister(BMI160_ACCEL_CONF,  0x2A);  // 400Hz
  writeRegister(BMI160_ACCEL_RANGE, 0x03);  // ±2g
  writeRegister(BMI160_GYRO_CONF,   0x2A);  // 400Hz
  writeRegister(BMI160_GYRO_RANGE,  0x00);  // ±2000 °/s

  delay(50);

  // Calibrate ครั้งแรก
  if (!calibrateGyro()) return false;

  _runningBiasZ = 0.0f;
  gyroZ_filtered = 0.0f;
  _lastTime = micros();

  return true;
}

// ตรวจว่าหุ่นยนต์นิ่งหรือไม่
bool my_GYRO160::isStationary() {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  float accMag = sqrt(ax*ax + ay*ay + az*az) / 16384.0f;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz) / 16.4f;

  return (abs(accMag - 1.0f) < 0.04f && gyroMag < 0.4f);
}

// อ่านมุม (เวอร์ชันแก้ drift สนิท)
void my_GYRO160::readAngles(float &roll, float &pitch, float &yaw) {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;

  float gyroX_raw = gx / 16.4f;
  float gyroY_raw = gy / 16.4f;
  float gyroZ_raw = gz / 16.4f;

  // Low-pass accel
  accelX = ACCEL_FILTER_ALPHA * accelX + (1.0f - ACCEL_FILTER_ALPHA) * _accelX_prev;
  accelY = ACCEL_FILTER_ALPHA * accelY + (1.0f - ACCEL_FILTER_ALPHA) * _accelY_prev;
  accelZ = ACCEL_FILTER_ALPHA * accelZ + (1.0f - ACCEL_FILTER_ALPHA) * _accelZ_prev;
  _accelX_prev = accelX;
  _accelY_prev = accelY;
  _accelZ_prev = accelZ;

  // ชดเชย offset + running bias
  float gyroX = gyroX_raw - _gyroOffsetX;
  float gyroY = gyroY_raw - _gyroOffsetY;
  float gyroZ = gyroZ_raw - _gyroOffsetZ - _runningBiasZ;

  // Deadzone
  if (abs(gyroX) < GYRO_DEADZONE) gyroX = 0.0f;
  if (abs(gyroY) < GYRO_DEADZONE) gyroY = 0.0f;
  if (abs(gyroZ) < GYRO_DEADZONE) gyroZ = 0.0f;

  // Low-pass filter สำหรับ gyroZ
  gyroZ = GYRO_LPF_ALPHA * gyroZ + (1.0f - GYRO_LPF_ALPHA) * gyroZ_filtered;
  gyroZ_filtered = gyroZ;

  // มุมจาก accelerometer
  float accelRoll  = atan2(accelY, accelZ) * RAD_TO_DEG;
  float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG;

  // Delta time
  unsigned long now = micros();
  float dt = (now - _lastTime) / 1000000.0f;
  _lastTime = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  // Integrate gyro
  float gyroAngleX = _angleX + gyroX * dt;
  float gyroAngleY = _angleY + gyroY * dt;
  float gyroAngleZ = _angleZ + gyroZ * dt;

  // Complementary filter
  _angleX = ALPHA * gyroAngleX + (1.0f - ALPHA) * accelRoll;
  _angleY = ALPHA * gyroAngleY + (1.0f - ALPHA) * accelPitch;

  // Dynamic bias correction
  if (isStationary()) {
    _runningBiasZ += BIAS_ALPHA * (0.0f - gyroZ);
  }

  // Yaw
  _angleZ = gyroAngleZ;

  // Wrap yaw
  while (_angleZ >  180.0f) _angleZ -= 360.0f;
  while (_angleZ <= -180.0f) _angleZ += 360.0f;

  roll  = _angleX;
  pitch = _angleY;
  yaw   = _angleZ;
}

// รีเซ็ต running bias
void my_GYRO160::resetRunningBias() {
  _runningBiasZ = 0.0f;
  gyroZ_filtered = 0.0f;
}

// Quick re-calibrate
void my_GYRO160::reCalibrateGyro() {
  const int SAMPLES = 120;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    sumX += gx / 16.4f;
    sumY += gy / 16.4f;
    sumZ += gz / 16.4f;
    delay(1);
  }

  _gyroOffsetX = sumX / SAMPLES;
  _gyroOffsetY = sumY / SAMPLES;
  _gyroOffsetZ = sumZ / SAMPLES;

  _runningBiasZ = 0.0f;
  gyroZ_filtered = 0.0f;
}

// ส่วนที่เหลือเหมือนเดิมของคุณ
float my_GYRO160::gyro(char axis) {
  float r, p, y;
  readAngles(r, p, y);
  switch (tolower(axis)) {
    case 'x': return r;
    case 'y': return p;
    case 'z': return y;
    default:  return 0.0f;
  }
}

void my_GYRO160::resetYaw()    { _angleZ = 0.0f; }
void my_GYRO160::resetAngles() { _angleX = _angleY = _angleZ = 0.0f; resetRunningBias(); }

void my_GYRO160::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void my_GYRO160::readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az,
                               int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(_address);
  Wire.write(BMI160_DATA);
  Wire.endTransmission();

  if (Wire.requestFrom(_address, (uint8_t)12) == 12) {
    *gx = Wire.read() | (Wire.read() << 8);
    *gy = Wire.read() | (Wire.read() << 8);
    *gz = Wire.read() | (Wire.read() << 8);
    *ax = Wire.read() | (Wire.read() << 8);
    *ay = Wire.read() | (Wire.read() << 8);
    *az = Wire.read() | (Wire.read() << 8);
  }
}

bool my_GYRO160::calibrateGyro() {
  const int SAMPLES = 300;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    sumX += gx / 16.4f;
    sumY += gy / 16.4f;
    sumZ += gz / 16.4f;
    delay(2);
  }

  _gyroOffsetX = sumX / SAMPLES;
  _gyroOffsetY = sumY / SAMPLES;
  _gyroOffsetZ = sumZ / SAMPLES;

  return (abs(_gyroOffsetX) < 5.0f && abs(_gyroOffsetY) < 5.0f && abs(_gyroOffsetZ) < 5.0f);
}

void reset_gyro160(my_GYRO160& gyro) {
  gyro.resetAngles();
}