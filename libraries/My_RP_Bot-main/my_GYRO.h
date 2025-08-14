#ifndef my_GYRO_h
#define my_GYRO_h

#include <Wire.h>

class my_GYRO {
public:
  // เริ่มต้นเซ็นเซอร์
  static bool begin();

  // อ่านค่ามุม (roll, pitch, yaw) ในหน่วยองศา
  static void readAngles(float &roll, float &pitch, float &yaw);

  // อ่านค่ามุมตามแกน ('x' = roll, 'y' = pitch, 'z' = yaw)
  static float gyro(char axis);

  // รีเซ็ตค่า Yaw เป็นศูนย์
  static void resetYaw();

  // รีเซ็ตมุมทั้งหมด (roll, pitch, yaw) เป็นศูนย์
  static void resetAngles();

private:
  // ที่อยู่ I2C
  static const uint8_t _address = 0x69;

  // รีจิสเตอร์ของเซ็นเซอร์
  static const uint8_t my_GYRO_CHIP_ID = 0x00;
  static const uint8_t my_GYRO_ACCEL_CONF = 0x40;
  static const uint8_t my_GYRO_GYRO_CONF = 0x42;
  static const uint8_t my_GYRO_ACCEL_RANGE = 0x41;
  static const uint8_t my_GYRO_GYRO_RANGE = 0x43;
  static const uint8_t my_GYRO_ACCEL_DATA = 0x12;
  static const uint8_t my_GYRO_GYRO_DATA = 0x0C;
  static const uint8_t my_GYRO_CMD = 0x7E;

  // ตัวแปรสำหรับคำนวณมุม
  static unsigned long _lastTime;
  static float _angleX, _angleY, _angleZ;
  static float _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ;
  static float _accelX_prev, _accelY_prev, _accelZ_prev; // สำหรับกรอง accelerometer
  static const float GYRO_THRESHOLD; // threshold ในหน่วย dps
  static const float ALPHA; // น้ำหนักสำหรับ complementary filter
  static const float ACCEL_FILTER_ALPHA; // น้ำหนักสำหรับกรอง accelerometer

  // ฟังก์ชันภายใน
  static void writeRegister(uint8_t reg, uint8_t value);
  static void readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
  static bool calibrateGyro();
  void reset_gyro(void);
};

// สร้างออบเจกต์ my_GYRO ด้วยที่อยู่ 0x69
extern my_GYRO my;

#endif
