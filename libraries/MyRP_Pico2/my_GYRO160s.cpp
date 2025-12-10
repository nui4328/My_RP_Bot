// ----------------------- my_GYRO160.cpp -----------------------
#include "my_GYRO160s.h"

// ค่าคงที่ที่ปรับแล้ว (ดีที่สุดสำหรับ BMI160)
//const float my_GYRO160::GYRO_DEADZONE       = 0.07f;   // ตัด noise (ต่ำสุดที่ยังไม่ drift)
//const float my_GYRO160::ALPHA              = 0.98f;   // 98% gyro, 2% accel
//const float my_GYRO160::ACCEL_FILTER_ALPHA = 0.1f;    // กรอง accel เบา ๆ

const float my_GYRO160::GYRO_DEADZONE = 0.30f;   // สำคัญสุด
const float my_GYRO160::ALPHA         = 0.96f;   // 96% gyro, 4% accel
const float my_GYRO160::ACCEL_FILTER_ALPHA = 0.14f;
// Static variables
unsigned long my_GYRO160::_lastTime = 0;
float my_GYRO160::_angleX = 0.0f;
float my_GYRO160::_angleY = 0.0f;
float my_GYRO160::_angleZ = 0.0f;
float my_GYRO160::_gyroOffsetX = 0.0f;
float my_GYRO160::_gyroOffsetY = 0.0f;
float my_GYRO160::_gyroOffsetZ = 0.0f;
float my_GYRO160::_accelX_prev = 0.0f;
float my_GYRO160::_accelY_prev = 0.0f;
float my_GYRO160::_accelZ_prev = 0.0f;

// Constructor
my_GYRO160::my_GYRO160(uint8_t address) : _address(address) {}

// เริ่มต้นเซ็นเซอร์ (สำคัญมาก: ลำดับถูกต้องตาม Datasheet)
bool my_GYRO160::begin() {
  Wire.begin();
  Wire.setClock(400000);  // Fast mode

  // ตรวจสอบ Chip ID
  Wire.beginTransmission(_address);
  Wire.write(BMI160_CHIP_ID);
  Wire.endTransmission();
  Wire.requestFrom(_address, (uint8_t)1);
  if (Wire.available()) {
    if (Wire.read() != 0xD1) return false;
  } else {
    return false;
  }

  // Soft reset
  writeRegister(BMI160_CMD, 0xB6);
  delay(100);

  // ตั้ง Normal Mode ก่อน แล้วค่อย config (สำคัญมาก!)
  writeRegister(BMI160_CMD, 0x11);  // ACCEL normal mode
  delay(10);
  writeRegister(BMI160_CMD, 0x15);  // GYRO normal mode
  delay(10);

  // ตั้งค่าเซ็นเซอร์
  writeRegister(BMI160_ACCEL_CONF,  0x2A);  // 400Hz, normal filter
  writeRegister(BMI160_ACCEL_RANGE, 0x03);  // ±2g
  writeRegister(BMI160_GYRO_CONF,   0x2A);  // 400Hz, normal filter
  writeRegister(BMI160_GYRO_RANGE,  0x00);  // ±2000 °/s

  delay(50);

  // Calibrate gyro offset (ต้องวางนิ่งตอนนี้!)
  if (!calibrateGyro()) {
    return false;
  }

  _lastTime = millis();
  return true;
}

// อ่านมุม (เวอร์ชันสุดท้ายที่แก้ปัญหาติด 0 แล้ว)
void my_GYRO160::readAngles(float &roll, float &pitch, float &yaw) {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  // แปลงหน่วย
  float accelX = ax / 16384.0f;   // ±2g → 16384 LSB/g
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;

  float gyroX_raw = gx / 16.4f;   // ±2000 dps → 16.4 LSB/dps
  float gyroY_raw = gy / 16.4f;
  float gyroZ_raw = gz / 16.4f;

  // Low-pass filter accelerometer
  accelX = ACCEL_FILTER_ALPHA * accelX + (1.0f - ACCEL_FILTER_ALPHA) * _accelX_prev;
  accelY = ACCEL_FILTER_ALPHA * accelY + (1.0f - ACCEL_FILTER_ALPHA) * _accelY_prev;
  accelZ = ACCEL_FILTER_ALPHA * accelZ + (1.0f - ACCEL_FILTER_ALPHA) * _accelZ_prev;
  _accelX_prev = accelX;
  _accelY_prev = accelY;
  _accelZ_prev = accelZ;

  // ชดเชย offset (เฉพาะตอนเริ่มต้น)
  float gyroX = gyroX_raw - _gyroOffsetX;
  float gyroY = gyroY_raw - _gyroOffsetY;
  float gyroZ = gyroZ_raw - _gyroOffsetZ;

  // Deadzone ตัด noise
  if (abs(gyroX) < GYRO_DEADZONE) gyroX = 0.0f;
  if (abs(gyroY) < GYRO_DEADZONE) gyroY = 0.0f;
  if (abs(gyroZ) < GYRO_DEADZONE) gyroZ = 0.0f;

  // มุมจาก accelerometer
  float accelRoll  = atan2(accelY, accelZ) * RAD_TO_DEG;
  float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG;

  // Delta time
  unsigned long now = millis();
  float dt = (now - _lastTime) / 1000.0f;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;  // ป้องกันค่าเพี้ยน
  _lastTime = now;

  // Integrate gyro
  float gyroAngleX = _angleX + gyroX * dt;
  float gyroAngleY = _angleY + gyroY * dt;
  float gyroAngleZ = _angleZ + gyroZ * dt;

  // Complementary filter
  _angleX = ALPHA * gyroAngleX + (1.0f - ALPHA) * accelRoll;
  _angleY = ALPHA * gyroAngleY + (1.0f - ALPHA) * accelPitch;
  _angleZ = gyroAngleZ;  // Yaw จาก gyro ล้วน

  // Wrap yaw ±180°
  while (_angleZ >  180.0f) _angleZ -= 360.0f;
  while (_angleZ <= -180.0f) _angleZ += 360.0f;

  roll  = _angleX;
  pitch = _angleY;
  yaw   = _angleZ;
}

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
void my_GYRO160::resetAngles() { _angleX = _angleY = _angleZ = 0.0f; }

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

  // ตรวจสอบว่า offset ไม่เกิน ±5 dps (ถ้าเกินแสดงว่าวางไม่นิ่ง)
  return (abs(_gyroOffsetX) < 5.0f && abs(_gyroOffsetY) < 5.0f && abs(_gyroOffsetZ) < 5.0f);
}

void reset_gyro160(my_GYRO160& gyro) {
  gyro.resetAngles();
}