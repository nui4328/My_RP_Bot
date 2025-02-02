#ifndef _my_l3gd20_
#define _my_l3gd20_


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_L3GD20.h>
#define L3GD20H_ADDR 0x6B
#define CTRL_REG1 0x20
#define OUT_X_L   0x28
#define OUT_X_H   0x29
#define OUT_Y_L   0x2A
#define OUT_Y_H   0x2B
#define OUT_Z_L   0x2C
#define OUT_Z_H   0x2D

//#define USE_I2C
#define USE_SPI

#ifdef USE_I2C
  // The default constructor uses I2C
  Adafruit_L3GD20 gyro;
#else
  // To use SPI, you have to define the pins
  #define GYRO_CS 20 // labeled CS
  #define GYRO_DO 12 // labeled SA0
  #define GYRO_DI 4  // labeled SDA
  #define GYRO_CLK 5 // labeled SCL
  Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
#endif

float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
float offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
unsigned long L3GD20lastTime;
void set_gyro(void);
void calibrateGyro(void);
void resetAngle(void);

void set_gyro()
  {
    Serial.begin(9600);
    Wire.begin();
    if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1)
      {
        Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
      }
  }
    // ตั้งค่า L3GD20H
    Wire.beginTransmission(L3GD20H_ADDR);
    Wire.write(CTRL_REG1);
    Wire.write(0x0F); // เปิดใช้งานเซ็นเซอร์ทั้ง 3 แกน
    Wire.endTransmission();
  
    Serial.println("Calibrating gyroscope...");
    resetAngle();
    calibrateGyro(); // คาลิเบรตเซ็นเซอร์
     resetAngle();
    Serial.println("Calibration complete!");
  
    L3GD20lastTime = millis();
  }
int16_t readAxis(uint8_t regL, uint8_t regH) 
  {
    int16_t value;
    Wire.beginTransmission(L3GD20H_ADDR);
    Wire.write(regL | 0x80); // เปิด auto-increment
    Wire.endTransmission();
    Wire.requestFrom(L3GD20H_ADDR, 2);
  
    if (Wire.available() >= 2) 
      {
        uint8_t low = Wire.read();
        uint8_t high = Wire.read();
        value = (high << 8) | low;
      }
    return value;
  }
void resetAngle() {
  angleX = 0.0; // รีเซตค่า angleZ เป็น 0
  angleY = 0.0; // รีเซตค่า angleZ เป็น 0
  angleZ = 0.0; // รีเซตค่า angleZ เป็น 0
  Serial.println("Angle Z reset to 0°");
}

// ฟังก์ชันคาลิเบรตเซ็นเซอร์
void calibrateGyro() 
  {
    int numSamples = 1000; // จำนวนตัวอย่างสำหรับการคาลิเบรต
    long sumX = 0, sumY = 0, sumZ = 0;
  
    for (int i = 0; i < numSamples; i++) 
      {
        sumX += readAxis(OUT_X_L, OUT_X_H);
        sumY += readAxis(OUT_Y_L, OUT_Y_H);
        sumZ += readAxis(OUT_Z_L, OUT_Z_H);
        //delay(1); // หน่วงเวลาเล็กน้อยระหว่างการวัด
      }
  
    // คำนวณ Offset
    offsetX = (float)sumX / numSamples * 0.00875; // แปลงเป็น dps
    offsetY = (float)sumY / numSamples * 0.00875;
    offsetZ = (float)sumZ / numSamples * 0.00875;
  
    Serial.print("Offset X: "); Serial.println(offsetX);
    Serial.print("Offset Y: "); Serial.println(offsetY);
    Serial.print("Offset Z: "); Serial.println(offsetZ);
  }
float gyro_angle(char xyz)
  {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - L3GD20lastTime) / 1000.0; // แปลงเป็นวินาที
    L3GD20lastTime = currentTime;
  
    // อ่านค่าอัตราการหมุน และหักลบ Offset
    float rateX = (readAxis(OUT_X_L, OUT_X_H) * 0.00875) - offsetX;
    float rateY = (readAxis(OUT_Y_L, OUT_Y_H) * 0.00875) - offsetY;
    float rateZ = (readAxis(OUT_Z_L, OUT_Z_H) * 0.00875) - offsetZ;
  
    // คำนวณมุมสะสม
    angleX += rateX * deltaTime;
    angleY += rateY * deltaTime;
    angleZ += rateZ * deltaTime;

    
    if(xyz == 'x')
      {
        return angleY;
      }
    else if(xyz == 'y')
      {        
        return angleX;
      }
    else if(xyz == 'z')
      {
        return angleZ;
      }
    else
      {
        return -1;
      }
  }
/*
float spi_gyro(char xyz)
  {
      gyro.read();
      float angularRateX = gyro.data.x; // อัตราเชิงมุม (dps)
      float angularRateY = gyro.data.y; // อัตราเชิงมุม (dps)
      float angularRateZ = gyro.data.z; // อัตราเชิงมุม (dps)
      unsigned long currentTime = millis();
      float deltaTime = (currentTime - L3GD20lastTime) / 1000.0; // แปลงเป็นวินาที
      L3GD20lastTime = currentTime;
      angleX += angularRateX *  deltaTime; // รวมค่าอัตราเชิงมุมกับเวลาที่ผ่านไป
      angleY += angularRateY *  deltaTime; // รวมค่าอัตราเชิงมุมกับเวลาที่ผ่านไป
      angleZ += angularRateZ *  deltaTime; // รวมค่าอัตราเชิงมุมกับเวลาที่ผ่านไป
      
      //Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print(" ");
      //Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print(" ");
      //Serial.print("Z: "); Serial.println((int)angleZ); Serial.print(" ");
       if(xyz == 'x')
      {
        return angleY;
      }
    else if(xyz == 'y')
      {        
        return angleX;
      }
    else if(xyz == 'z')
      {
        return angleZ;
      }
    else
      {
        return -1;
      }
  }

*/
float spi_gyro(char xyz)
  {
      gyro.read();
float angularRateX = gyro.data.x; // อัตราเชิงมุม (dps)
float angularRateY = gyro.data.y; // อัตราเชิงมุม (dps)
float angularRateZ = gyro.data.z; // อัตราเชิงมุม (dps)

// กรองค่า angularRateZ หากมี noise ต่ำกว่าค่าที่กำหนด
if (abs(angularRateX) < 0.85) { // ค่าขอบเขต noise สามารถปรับได้
    angularRateX = 0;
}
if (abs(angularRateY) < 1.0) { // ค่าขอบเขต noise สามารถปรับได้
    angularRateY = 0;
}
if (abs(angularRateZ) < 0.85) { // ค่าขอบเขต noise สามารถปรับได้
    angularRateZ = 0;
}
unsigned long currentTime = millis();
float XdeltaTime = (currentTime - L3GD20lastTime) / 900.0; // แปลงเป็นวินาที
float YdeltaTime = (currentTime - L3GD20lastTime) / 1000.0; // แปลงเป็นวินาที
float deltaTime = (currentTime - L3GD20lastTime) / 900.0; // แปลงเป็นวินาที
L3GD20lastTime = currentTime;

angleX += angularRateX * XdeltaTime; // รวมค่าอัตราเชิงมุมกับเวลาที่ผ่านไป
angleY += angularRateY * deltaTime; // รวมค่าอัตราเชิงมุมกับเวลาที่ผ่านไป
angleZ += angularRateZ * deltaTime; // รวมค่าอัตราเชิงมุมกับเวลาที่ผ่านไป

// จำกัดค่าของ angleZ ให้อยู่ในช่วง -180 ถึง 180 องศา
if (angleZ > 180) {
    angleZ -= 360;
} else if (angleZ < -180) {
    angleZ += 360;
}

// แสดงผลค่า
//Serial.print("X: "); Serial.print(angleX); Serial.print(" ");
//Serial.print("Y: "); Serial.print(angleY); Serial.print(" ");
//Serial.print("Z: "); Serial.println(angleZ);

       if(xyz == 'x')
      {
        return angleY;
      }
    else if(xyz == 'y')
      {        
        return angleX;
      }
    else if(xyz == 'z')
      {
        return angleZ;
      }
    else
      {
        return -1;
      }
      
  }
#endif
