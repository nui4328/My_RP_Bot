#ifndef _my_mpu_
#define _my_mpu_


#include <Arduino.h>
#include <Wire.h>
#define MPU6050_ADDR 0x68

float gyroZ=0; 
long gyroZ0 = 0;
float yaw;
float yawOld = 0;
float gyroAngleZ = 0; //Angle variable
int16_t gx = 0, gy = 0, gz = 0;
float dt_yaw = 0.0; 
unsigned long lastTime_yaw = millis();  // ตัวแปรเก็บเวลาครั้งล่าสุด

float rollOffset = 0;   // Offset ของ Roll
float pitchOffset = 0;  // Offset ของ Pitch

void setReferenceAxes(void);
void calibration_yaw(void);
void setReferenceYaw(void); 
void calibratePitch(void);  
void reset_PitchRoll(void);

void setup_mpu() 
  {
      Wire.begin();   
      //while (!Serial);    
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0x00);     // Set to 0 (wakes up the MPU6050)
      Wire.endTransmission(true);

      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x1b);  // เริ่มต้นที่ที่เก็บข้อมูลของ Gyroscope
      Wire.write(0x00);  // เริ่มต้นที่ที่เก็บข้อมูลของ Gyroscope
      Wire.endTransmission(true);
      lastTime_yaw = millis();

      setReferenceAxes() ;
      calibration_yaw();   
      setReferenceYaw();  
      reset_PitchRoll();
      calibratePitch();
  }
void reset_PitchRoll()
  {
    float rollOffset = 0;   // Offset ของ Roll
    float pitchOffset = 0;  // Offset ของ Pitch
  }
void reset_Yaw()
  {    
      int16_t gx, gy, gz;

    // อ่านค่าจาก Gyroscope (แกน X, Y, Z)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // เริ่มต้นที่ที่เก็บข้อมูลของ Gyroscope
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);  
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    // ตั้งค่า gyroZ0 ให้เป็นค่าเริ่มต้น
    gyroZ0 = gz;  // เก็บค่าของ gyroZ สำหรับการปรับเทียบ
    gyroAngleZ = 0;  // เริ่มต้นมุม Yaw เป็น 0
    
        // ปรับเทียบค่าของ gyroZ กับค่าเบื้องต้น (gyroZ0)
    float angularZ = (gz - gyroZ0) / 130.0 * dt_yaw; // แปลงค่า raw จาก Gyroscope เป็นมุม (degree)
    
    // ตรวจสอบและปรับความผิดพลาดที่สะสม (drift)
    if (fabs(angularZ) < 0.25 || fabs(angularZ) > 0.25) {
        gyroZ0 = 0;  // ป้องกันค่าผิดปกติที่เกิดจาก noise หรือ drift
    }
    // แสดงค่าปรับเทียบ
    //Serial.print("Yaw Offset Set to: ");
   // Serial.println(gyroZ0);     
  }
void calibration_yaw()
  {    
      int16_t gx, gy, gz;

    // อ่านค่าจาก Gyroscope (แกน X, Y, Z)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // เริ่มต้นที่ที่เก็บข้อมูลของ Gyroscope
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);  
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    // ตั้งค่า gyroZ0 ให้เป็นค่าเริ่มต้น
    gyroZ0 = gz;  // เก็บค่าของ gyroZ สำหรับการปรับเทียบ
    gyroAngleZ = 0;  // เริ่มต้นมุม Yaw เป็น 0
        // ปรับเทียบค่าของ gyroZ กับค่าเบื้องต้น (gyroZ0)
    float angularZ = (gz - gyroZ0) / 131.0 * dt_yaw; // แปลงค่า raw จาก Gyroscope เป็นมุม (degree)

    // ตรวจสอบและปรับความผิดพลาดที่สะสม (drift)
    if (fabs(angularZ) < 0.05 || fabs(angularZ) > 0.05) {
        gyroZ0 = 0;  // ป้องกันค่าผิดปกติที่เกิดจาก noise หรือ drift
    }
    // แสดงค่าปรับเทียบ
    Serial.print("Yaw Offset Set to: ");
    Serial.println(gyroZ0);     
  }
void setReferenceAxes() {
    int16_t ax, ay, az;
    // อ่านค่าปัจจุบันของ Accelerometer
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // Starting register of accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();

    // คำนวณ Roll และ Pitch สำหรับตำแหน่งปัจจุบัน
    rollOffset = atan2(ay, az) * 180.0 / PI;
    pitchOffset = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    Serial.print("Roll Offset Set to: ");
    Serial.println(rollOffset);
    Serial.print("Pitch Offset Set to: ");
    Serial.println(pitchOffset);
}
void setReferenceYaw() {
    int16_t gx, gy, gz;

    // อ่านค่าจาก Gyroscope (แกน X, Y, Z)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // เริ่มต้นที่ที่เก็บข้อมูลของ Gyroscope
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);  
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    // ตั้งค่า gyroZ0 ให้เป็นค่าเริ่มต้น
    gyroZ0 = gz;  // เก็บค่าของ gyroZ สำหรับการปรับเทียบ
    gyroAngleZ = 0;  // เริ่มต้นมุม Yaw เป็น 0
        // ปรับเทียบค่าของ gyroZ กับค่าเบื้องต้น (gyroZ0)
    float angularZ = (gz - gyroZ0) / 131.0 * dt_yaw; // แปลงค่า raw จาก Gyroscope เป็นมุม (degree)

    // ตรวจสอบและปรับความผิดพลาดที่สะสม (drift)
    if (fabs(angularZ) < 0.05 || fabs(angularZ) > 0.05) {
        gyroZ0 = 0;  // ป้องกันค่าผิดปกติที่เกิดจาก noise หรือ drift
    }
    // แสดงค่าปรับเทียบ
    Serial.print("Yaw Offset Set to: ");
    Serial.println(gyroZ0);
}

float readRoll()  {
    int16_t ax, ay, az;
    static float rollFiltered = 0;
    float alpha = 0.85;

    delayMicroseconds(100);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();

    float roll = atan2(ay, az) * 180.0 / PI;

    // หักลบ Offset
    roll -= rollOffset;

    // กรองค่า Roll ด้วย Low-Pass Filter
    rollFiltered = alpha * rollFiltered + (1 - alpha) * roll;

    return -rollFiltered;
}
float _readPitch() 
  {
    int16_t ax, ay, az;
     // แปลงค่าการเร่งเป็น g
      delayMicroseconds(350);  // เพิ่ม delay

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    float x = ax / 16384.0;
    float y = ay / 16384.0;
    float z = az / 16384.0;

    // คำนวณ Pitch และ Roll
    float pitch = atan2(-x, sqrt(y * y + z * z)) * 180 / PI;
    float roll = atan2(y, sqrt(x * x + z * z)) * 180 / PI;

    Serial.print("Pitch: ");
    Serial.print(int(pitch));
    Serial.print("°, Roll: ");
    Serial.println(int(roll));
    delay(10);
    return pitch;
 
  }
float readPitch() {
    int16_t ax, ay, az;
    static float pitchFiltered = 0;
    static float previousPitch = 0;
    float alpha = 0.85;  // ค่าสำหรับกรอง Low-Pass Filter
    float maxChangeRate = 2.0;  // จำกัดการเปลี่ยนแปลง
    static float pitchOffset = 0.15; // Offset สำหรับปรับค่ามุม Pitch

    // เพิ่ม Delay เพื่อให้การอ่านข้อมูลเสถียร
    delayMicroseconds(50);

    // อ่านข้อมูลจาก MPU6050 ผ่าน I2C
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // เริ่มต้นที่ Address 0x3B สำหรับ Accelerometer
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();

    // แปลงค่าดิบเป็น g
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    // คำนวณมุม Pitch จาก Accelerometer
    float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

    // หักลบ Offset
    pitch -= pitchOffset;

    // กรองค่ามุม Pitch ด้วย Low-Pass Filter
    pitchFiltered = alpha * pitchFiltered + (1 - alpha) * pitch;

    // จำกัดอัตราการเปลี่ยนแปลงของค่า Pitch
    if (abs(pitchFiltered - previousPitch) > maxChangeRate) {
        pitchFiltered = previousPitch + (pitchFiltered > previousPitch ? maxChangeRate : -maxChangeRate);
    }
    previousPitch = pitchFiltered;

    // คืนค่ามุม Pitch
    return pitchFiltered;
}

void calibratePitch() {
    int samples = 100; // จำนวนตัวอย่างสำหรับการคำนวณ Offset
    float offsetSum = 0;

    for (int i = 0; i < samples; i++) {
        // อ่านข้อมูลจาก MPU6050
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 6, true);

        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();

        float ax_g = ax / 16384.0;
        float ay_g = ay / 16384.0;
        float az_g = az / 16384.0;

        float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
        offsetSum += pitch;

        delay(5); // เพิ่ม Delay เพื่อให้การอ่านข้อมูลเสถียร
    }

    // คำนวณค่า Offset เฉลี่ย
    pitchOffset = offsetSum / samples;
}


float readYaw() {
    delayMicroseconds(50);
    int16_t gx, gy, gz;
    // อ่านข้อมูลจาก Gyroscope (แกน X, Y, Z)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // เริ่มต้นที่ที่เก็บข้อมูลของ Gyroscope
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);  
    
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
    
    // คำนวณเวลาที่ผ่านไปจากการอ่านค่า
    unsigned long currentTime = millis();
    dt_yaw = (currentTime - lastTime_yaw) / 1000.0;  // คำนวณเวลาเป็นวินาที
    lastTime_yaw = currentTime;

    // ปรับเทียบค่าของ gyroZ กับค่าเบื้องต้น (gyroZ0)
    float angularZ = (gz - gyroZ0) / 129.0 * dt_yaw; // แปลงค่า raw จาก Gyroscope เป็นมุม (degree)

    // ตรวจสอบและปรับความผิดพลาดที่สะสม (drift)
    if (fabs(angularZ) < 0.015 || fabs(angularZ) > 0.015) {
        gyroZ0 = 0;  // ป้องกันค่าผิดปกติที่เกิดจาก noise หรือ drift
    }

    // ปรับมุม Yaw โดยการหักลบค่าที่หมุนไปแล้ว
    gyroAngleZ -= angularZ;

    return (int)gyroAngleZ;  // คืนค่ามุม Yaw เป็นจำนวนเต็ม
}

int error_Yaw()
  {
      delayMicroseconds(100);  
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x43);  // Starting register of gyroscope data
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 6, true);      
      gx = Wire.read() << 8 | Wire.read();
      gy = Wire.read() << 8 | Wire.read();
      gz = Wire.read() << 8 | Wire.read();    
             
      unsigned long currentTime = millis();   
      dt_yaw = (currentTime - lastTime_yaw) / 1000.0; 
      lastTime_yaw = currentTime;        
      gyroZ = gz;        
      float angularZ = (gyroZ - gyroZ0) / 131.0 * dt_yaw; //angular z: =t
      if (fabs(angularZ) < 0.05) //
         {
            angularZ = 0.00;
         }
      gyroAngleZ -= angularZ; //returns the absolute value of the z-axis rotazion integral 
      yaw = - gyroAngleZ;
        
      return gyroAngleZ;  
  }

float error_Yaw_float()
  {
      delayMicroseconds(50);  
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x43);  // Starting register of gyroscope data
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 6, true);      
      gx = Wire.read() << 8 | Wire.read();
      gy = Wire.read() << 8 | Wire.read();
      gz = Wire.read() << 8 | Wire.read();    
             
      unsigned long currentTime = millis();   
      dt_yaw = (currentTime - lastTime_yaw) / 1000.0; 
      lastTime_yaw = currentTime;        
      gyroZ = gz;        
      float angularZ = (gyroZ - gyroZ0) / 131.0 * dt_yaw; //angular z: =t
      if (fabs(angularZ) < 0.05) //
         {
            angularZ = 0.00;
         }
      gyroAngleZ -= angularZ; //returns the absolute value of the z-axis rotazion integral 
      yaw = - gyroAngleZ;
        
      return gyroAngleZ;  
  }
void mpu()
  {    
      delayMicroseconds(50);  
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x43);  // Starting register of gyroscope data
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 6, true);      
      gx = Wire.read() << 8 | Wire.read();
      gy = Wire.read() << 8 | Wire.read();
      gz = Wire.read() << 8 | Wire.read();    
             
  }



#endif
