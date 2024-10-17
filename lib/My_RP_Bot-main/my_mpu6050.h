#ifndef _my_mpu_
#define _my_mpu_


#include <Arduino.h>
#include <Wire.h>
#define MPU6050_ADDR 0x68

float gyroZ; 
unsigned long lastTime_yak = 0;
float dt_yak;      //Differential time
long gyroZ0 = 0;
float yaw;
float yawOld = 0;
float gyroAngleZ = 0; //Angle variable
int16_t gx = 0, gy = 0, gz = 0;

void setup_mpu() 
  {
      Wire.begin();   
      //while (!Serial);    
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // Set to 0 (wakes up the MPU6050)
      Wire.endTransmission(true);
    
  }

void calibration_Yak()
  {
    
      delayMicroseconds(100); 
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x43);  // Starting register of gyroscope data
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 6, true);      
      gx = Wire.read() << 8 | Wire.read();
      gy = Wire.read() << 8 | Wire.read();
      gz = Wire.read() << 8 | Wire.read();    
             
      gyroAngleZ = 0;
      unsigned short times = 100; //Sampling times
      for (int i = 0; i < times; i++)
          {
            gyroZ = gz;     // gyroZ - Raw register values gyroscope Z axis
            gyroZ0 += gyroZ; //sum all measured values gyroZ
          } 
      gyroZ0 /= times; 
      delayMicroseconds(50); 
      

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
      dt_yak = (currentTime - lastTime_yak) / 1000.0; 
      lastTime_yak = currentTime;        
      gyroZ = gz;        
      float angularZ = (gyroZ - gyroZ0) / 131.0 * dt_yak; //angular z: =t
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
      dt_yak = (currentTime - lastTime_yak) / 1000.0; 
      lastTime_yak = currentTime;        
      gyroZ = gz;        
      float angularZ = (gyroZ - gyroZ0) / 131.0 * dt_yak; //angular z: =t
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
