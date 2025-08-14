#include "my_GYRO.h"

// กำหนดออบเจกต์
//my_GYRO my;

// ค่าคงที่
const float my_GYRO::GYRO_THRESHOLD = 0.3; // ลด threshold เพื่อความเข้มงวด
const float my_GYRO::ALPHA = 0.95; // ลด ALPHA เพื่อให้ accelerometer มีผลมากขึ้น
const float my_GYRO::ACCEL_FILTER_ALPHA = 0.1; // สำหรับกรอง accelerometer

// ตัวแปร static
unsigned long my_GYRO::_lastTime = 0;
float my_GYRO::_angleX = 0.0;
float my_GYRO::_angleY = 0.0;
float my_GYRO::_angleZ = 0.0;
float my_GYRO::_gyroOffsetX = 0.0;
float my_GYRO::_gyroOffsetY = 0.0;
float my_GYRO::_gyroOffsetZ = 0.0;
float my_GYRO::_accelX_prev = 0.0;
float my_GYRO::_accelY_prev = 0.0;
float my_GYRO::_accelZ_prev = 0.0;

// เริ่มต้นเซ็นเซอร์
bool my_GYRO::begin() {
  Wire.begin(); // เริ่ม I2C

  // ตรวจสอบ chip ID
  Wire.beginTransmission(_address);
  Wire.write(my_GYRO_CHIP_ID);
  Wire.endTransmission();
  Wire.requestFrom(_address, (uint8_t)1);
  if (Wire.available()) {
    uint8_t chipID = Wire.read();
    if (chipID != 0xD1) { // Chip ID = 0xD1
      return false;
    }
  } else {
    return false;
  }

  // รีเซ็ตเซ็นเซอร์
  writeRegister(my_GYRO_CMD, 0xB6); // Soft reset
  delay(100);

  // ตั้งค่า accelerometer (±2g, ODR 400Hz)
  writeRegister(my_GYRO_ACCEL_CONF, 0x2A); // ODR = 400Hz
  writeRegister(my_GYRO_ACCEL_RANGE, 0x03); // ±2g

  // ตั้งค่า gyroscope (±2000 dps, ODR 400Hz)
  writeRegister(my_GYRO_GYRO_CONF, 0x2A); // ODR = 400Hz
  writeRegister(my_GYRO_GYRO_RANGE, 0x00); // ±2000 dps

  // เริ่ม accelerometer และ gyroscope
  writeRegister(my_GYRO_CMD, 0x11); // เริ่ม accelerometer
  delay(50);
  writeRegister(my_GYRO_CMD, 0x15); // เริ่ม gyroscope
  delay(50);

  // คำนวณ offset ของ gyroscope
  if (!calibrateGyro()) {
    return false;
  }
  _lastTime = micros();
  return true;
}

void my_GYRO::readAngles(float &roll, float &pitch, float &yaw) {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  accelX = ACCEL_FILTER_ALPHA * accelX + (1.0 - ACCEL_FILTER_ALPHA) * _accelX_prev;
  accelY = ACCEL_FILTER_ALPHA * accelY + (1.0 - ACCEL_FILTER_ALPHA) * _accelY_prev;
  accelZ = ACCEL_FILTER_ALPHA * accelZ + (1.0 - ACCEL_FILTER_ALPHA) * _accelZ_prev;
  _accelX_prev = accelX;
  _accelY_prev = accelY;
  _accelZ_prev = accelZ;

  float gyroX = (gx / 16.4) - _gyroOffsetX;
  float gyroY = (gy / 16.4) - _gyroOffsetY;
  float gyroZ = (gz / 16.4) - _gyroOffsetZ;

  if (abs(gyroX) < GYRO_THRESHOLD) gyroX = 0.0;
  if (abs(gyroY) < GYRO_THRESHOLD) gyroY = 0.0;
  if (abs(gyroZ) < GYRO_THRESHOLD) gyroZ = 0.0;

  float accelRoll = atan2(accelY, accelZ) * 180.0 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  unsigned long currentTime = micros();
  float deltaTime = (currentTime - _lastTime) / 1000000.0;
  if (deltaTime > 0.1) deltaTime = 0.1; // จำกัดค่า deltaTime
  _lastTime = currentTime;

  float gyroAngleX = _angleX + gyroX * deltaTime;
  float gyroAngleY = _angleY + gyroY * deltaTime;
  float gyroAngleZ = _angleZ + gyroZ * deltaTime;

  _angleX = ALPHA * gyroAngleX + (1.0 - ALPHA) * accelRoll;
  _angleY = ALPHA * gyroAngleY + (1.0 - ALPHA) * accelPitch;
  _angleZ = gyroAngleZ; // yaw ใช้ gyroscope ล้วน ๆ

  roll = _angleX;
  pitch = _angleY;
  yaw = _angleZ;
}


// อ่านค่ามุมตามแกน ('x' = roll, 'y' = pitch, 'z' = yaw)
float my_GYRO::gyro(char axis) {
  float roll, pitch, yaw;
  readAngles(roll, pitch, yaw); // อัปเดตค่ามุมล่าสุด
  switch (axis) {
    case 'x':
    case 'X':
      return roll;
    case 'y':
    case 'Y':
      return pitch;
    case 'z':
    case 'Z':
      return yaw;
    default:
      return 0.0; // คืนค่า 0.0 หากแกนไม่ถูกต้อง
  }
}

// รีเซ็ตค่า Yaw เป็นศูนย์
void my_GYRO::resetYaw() {
  _angleZ = 0.0;
}

// รีเซ็ตมุมทั้งหมด (roll, pitch, yaw) เป็นศูนย์
void my_GYRO::resetAngles() {
  _angleX = 0.0;
  _angleY = 0.0;
  _angleZ = 0.0;
}

// ฟังก์ชันเขียนรีจิสเตอร์
void my_GYRO::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ฟังก์ชันอ่าน accelerometer และ gyroscope
void my_GYRO::readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(_address);
  Wire.write(my_GYRO_GYRO_DATA);
  Wire.endTransmission();
  Wire.requestFrom(_address, (uint8_t)12);

  if (Wire.available() == 12) {
    *gx = (Wire.read() | (Wire.read() << 8));
    *gy = (Wire.read() | (Wire.read() << 8));
    *gz = (Wire.read() | (Wire.read() << 8));
    *ax = (Wire.read() | (Wire.read() << 8));
    *ay = (Wire.read() | (Wire.read() << 8));
    *az = (Wire.read() | (Wire.read() << 8));
  }
}

// ฟังก์ชันคำนวณ offset ของ gyroscope
bool my_GYRO::calibrateGyro() {
  const int CALIBRATION_SAMPLES = 200; // เพิ่มจำนวนตัวอย่าง
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  float sumAX = 0.0, sumAY = 0.0, sumAZ = 0.0;
  float varX = 0.0, varY = 0.0, varZ = 0.0;
  int16_t gx, gy, gz;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t ax, ay, az;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    float gX = gx / 16.4;
    float gY = gy / 16.4;
    float gZ = gz / 16.4;
    float aX = ax / 16384.0;
    float aY = ay / 16384.0;
    float aZ = az / 16384.0;
    sumX += gX;
    sumY += gY;
    sumZ += gZ;
    sumAX += aX;
    sumAY += aY;
    sumAZ += aZ;
    varX += gX * gX;
    varY += gY * gY;
    varZ += gZ * gZ;
    delay(5); // ปรับให้ตรงกับ ODR 400Hz
  }

  // ตรวจสอบความนิ่งของเซ็นเซอร์
  float meanX = sumX / CALIBRATION_SAMPLES;
  float meanY = sumY / CALIBRATION_SAMPLES;
  float meanZ = sumZ / CALIBRATION_SAMPLES;
  varX = varX / CALIBRATION_SAMPLES - meanX * meanX;
  varY = varY / CALIBRATION_SAMPLES - meanY * meanY;
  varZ = varZ / CALIBRATION_SAMPLES - meanZ * meanZ;
  if (varX > 0.1 || varY > 0.1 || varZ > 0.1) {
    return false; // เซ็นเซอร์ไม่นิ่ง
  }

  _gyroOffsetX = meanX;
  _gyroOffsetY = meanY;
  _gyroOffsetZ = meanZ;
  _accelX_prev = sumAX / CALIBRATION_SAMPLES;
  _accelY_prev = sumAY / CALIBRATION_SAMPLES;
  _accelZ_prev = sumAZ / CALIBRATION_SAMPLES;
  return true;
}

void reset_gyro() // รีเซ็ตมุมก่อนเริ่ม
  {
    my_GYRO::resetAngles();
  }
