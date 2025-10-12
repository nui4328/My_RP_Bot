#include "my_GYRO160.h"

// Static variable definitions
const float my_GYRO160::GYRO_THRESHOLD = 0.3; // Reduced threshold for stricter deadzone
const float my_GYRO160::ALPHA = 0.95;        // Reduced ALPHA to give more weight to accelerometer
const float my_GYRO160::ACCEL_FILTER_ALPHA = 0.1; // For accelerometer filtering

unsigned long my_GYRO160::_lastTime = 0;
float my_GYRO160::_angleX = 0.0;
float my_GYRO160::_angleY = 0.0;
float my_GYRO160::_angleZ = 0.0;
float my_GYRO160::_gyroOffsetX = 0.0;
float my_GYRO160::_gyroOffsetY = 0.0;
float my_GYRO160::_gyroOffsetZ = 0.0;
float my_GYRO160::_accelX_prev = 0.0;
float my_GYRO160::_accelY_prev = 0.0;
float my_GYRO160::_accelZ_prev = 0.0;

// Constructor
my_GYRO160::my_GYRO160(uint8_t address) : _address(address) {}

// Initialize sensor
bool my_GYRO160::begin() {
  Wire1.begin(); // Start I2C1
  Wire1.setClock(400000); // Set I2C frequency to 400kHz

  // Check chip ID
  Wire1.beginTransmission(_address);
  Wire1.write(my_GYRO160_CHIP_ID);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, (uint8_t)1);
  if (Wire1.available()) {
    uint8_t chipID = Wire1.read();
    if (chipID != 0xD1) { // Chip ID = 0xD1
      return false;
    }
  } else {
    return false;
  }

  // Reset sensor
  writeRegister(my_GYRO160_CMD, 0xB6); // Soft reset
  delay(100);

  // Configure accelerometer (±2g, ODR 400Hz)
  writeRegister(my_GYRO160_ACCEL_CONF, 0x2A); // ODR = 400Hz
  writeRegister(my_GYRO160_ACCEL_RANGE, 0x03); // ±2g

  // Configure gyroscope (±2000 dps, ODR 400Hz)
  writeRegister(my_GYRO160_GYRO_CONF, 0x2A); // ODR = 400Hz
  writeRegister(my_GYRO160_GYRO_RANGE, 0x00); // ±2000 dps

  // Start accelerometer and gyroscope
  writeRegister(my_GYRO160_CMD, 0x11); // Start accelerometer
  delay(50);
  writeRegister(my_GYRO160_CMD, 0x15); // Start gyroscope
  delay(50);

  // Calculate gyroscope offsets
  if (!calibrateGyro()) {
    return false;
  }
  _lastTime = micros();
  return true;
}

void my_GYRO160::readAngles(float &roll, float &pitch, float &yaw) {
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

  // คำนวณค่า gyro และชดเชย offset
  float gyroX = (gx / 16.4) - _gyroOffsetX;
  float gyroY = (gy / 16.4) - _gyroOffsetY;
  float gyroZ = (gz / 16.4) - _gyroOffsetZ;

  // อัปเดต _gyroOffsetZ เมื่อเซ็นเซอร์อยู่นิ่ง
  const float GYRO_CALIBRATION_ALPHA = 0.01; // ค่าคงที่สำหรับ moving average
  const float GYRO_THRESHOLD = 0.1; // ลดจาก 0.5 เป็น 0.1 เพื่อเพิ่มความไว
  if (abs(gx / 16.4) < GYRO_THRESHOLD) {
    _gyroOffsetX = GYRO_CALIBRATION_ALPHA * (gx / 16.4) + (1.0 - GYRO_CALIBRATION_ALPHA) * _gyroOffsetX;
  }
  if (abs(gy / 16.4) < GYRO_THRESHOLD) {
    _gyroOffsetY = GYRO_CALIBRATION_ALPHA * (gy / 16.4) + (1.0 - GYRO_CALIBRATION_ALPHA) * _gyroOffsetY;
  }
  if (abs(gz / 16.4) < GYRO_THRESHOLD) {
    _gyroOffsetZ = GYRO_CALIBRATION_ALPHA * (gz / 16.4) + (1.0 - GYRO_CALIBRATION_ALPHA) * _gyroOffsetZ;
  }

  // ตัด noise เมื่อการเคลื่อนไหวน้อย
  if (abs(gyroX) < GYRO_THRESHOLD) gyroX = 0.0;
  if (abs(gyroY) < GYRO_THRESHOLD) gyroY = 0.0;
  if (abs(gyroZ) < GYRO_THRESHOLD) gyroZ = 0.0;

  float accelRoll = atan2(accelY, accelZ) * 180.0 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  unsigned long currentTime = micros();
  float deltaTime = (currentTime - _lastTime) / 1000000.0;
  if (deltaTime > 0.1) deltaTime = 0.1; // จำกัด deltaTime
  _lastTime = currentTime;

  float gyroAngleX = _angleX + gyroX * deltaTime;
  float gyroAngleY = _angleY + gyroY * deltaTime;
  float gyroAngleZ = _angleZ + gyroZ * deltaTime;

  _angleX = ALPHA * gyroAngleX + (1.0 - ALPHA) * accelRoll;
  _angleY = ALPHA * gyroAngleY + (1.0 - ALPHA) * accelPitch;
  _angleZ = gyroAngleZ; // Yaw ใช้ gyroscope เท่านั้น

  // Wrap yaw to [-180, 180]
  if (_angleZ > 180) _angleZ -= 360;
  else if (_angleZ < -180) _angleZ += 360;

  roll = _angleX;
  pitch = _angleY;
  yaw = _angleZ;
}

// Read angle for specified axis ('x' = roll, 'y' = pitch, 'z' = yaw)
float my_GYRO160::gyro(char axis) {
  float roll, pitch, yaw;
  readAngles(roll, pitch, yaw); // Update latest angles
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
      return 0.0; // Return 0.0 if axis is invalid
  }
}

// Reset yaw to zero
void my_GYRO160::resetYaw() {
  _angleZ = 0.0;
}

// Reset all angles (roll, pitch, yaw) to zero
void my_GYRO160::resetAngles() {
  _angleX = 0.0;
  _angleY = 0.0;
  _angleZ = 0.0;
}

// Write to register
void my_GYRO160::writeRegister(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.write(value);
  Wire1.endTransmission();
}

// Read accelerometer and gyroscope data
void my_GYRO160::readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire1.beginTransmission(_address);
  Wire1.write(my_GYRO160_GYRO_DATA);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, (uint8_t)12);

  if (Wire1.available() == 12) {
    *gx = (Wire1.read() | (Wire1.read() << 8));
    *gy = (Wire1.read() | (Wire1.read() << 8));
    *gz = (Wire1.read() | (Wire1.read() << 8));
    *ax = (Wire1.read() | (Wire1.read() << 8));
    *ay = (Wire1.read() | (Wire1.read() << 8));
    *az = (Wire1.read() | (Wire1.read() << 8));
  }
}

// Calibrate gyroscope offsets
bool my_GYRO160::calibrateGyro() {
  const int CALIBRATION_SAMPLES = 200; // Increased sample count
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
    delay(5); // Match ODR 400Hz
  }

  // Check sensor stability
  float meanX = sumX / CALIBRATION_SAMPLES;
  float meanY = sumY / CALIBRATION_SAMPLES;
  float meanZ = sumZ / CALIBRATION_SAMPLES;
  varX = varX / CALIBRATION_SAMPLES - meanX * meanX;
  varY = varY / CALIBRATION_SAMPLES - meanY * meanY;
  varZ = varZ / CALIBRATION_SAMPLES - meanZ * meanZ;
  if (varX > 0.1 || varY > 0.1 || varZ > 0.1) {
    return false; // Sensor not stable
  }

  _gyroOffsetX = meanX;
  _gyroOffsetY = meanY;
  _gyroOffsetZ = meanZ;
  _accelX_prev = sumAX / CALIBRATION_SAMPLES;
  _accelY_prev = sumAY / CALIBRATION_SAMPLES;
  _accelZ_prev = sumAZ / CALIBRATION_SAMPLES;
  return true;
}

void reset_gyro160(my_GYRO160& gyro) {
  gyro.resetAngles();
}
