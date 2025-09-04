#ifndef _my_rp2350_pro_
#define _my_rp2350_pro_

#include <Wire.h>
#include <my_GYRO.h>
#include <my_MCP3008.h>
my_MCP3008 adc;
#include <EncoderLibrary.h>
EncoderLibrary encoder(6, 7, 15, 20);

// กำหนดพินควบคุมมอเตอร์

#define PWMA 6     // PWM ซ้าย
#define AIN1 22
#define AIN2 23

#define PWMB 3     // PWM ขวา
#define BIN1 21
#define BIN2 20
//___--------------------------------------------->>
#define EEPROM_ADDRESS 0x50 // ที่อยู่ I2C ของ CAT24C256
const int numSensors = 8;
const int numSamples = 1000;
int sensorValuesA[numSensors][numSamples]; // เก็บข้อมูลเซนเซอร์จาก read_sensorA
int sensorMaxA[numSensors];
int sensorMinA[numSensors];
int sensorValuesB[numSensors][numSamples]; // เก็บข้อมูลเซนเซอร์จาก read_sensorB
int sensorMaxB[numSensors];
int sensorMinB[numSensors];
int sensorValuesC[2][numSamples]; // เก็บข้อมูลจาก analogRead(46) และ analogRead(47)
int sensorMaxC[2]; // Max สำหรับพิน 46 และ 47
int sensorMinC[2]; // Min สำหรับพิน 46 และ 47
int sensorMax_A[numSensors];
int sensorMin_A[numSensors];
int sensorMax_B[numSensors];
int sensorMin_B[numSensors];
int sensorMax_C[2]; // Max สำหรับพิน 26 และ 27
int sensorMin_C[2]; // Min สำหรับพิน 26 และ 27
float P, I, D, previous_I, previous_error,errors, PID_output, present_position, previous_integral; 
int numSensor = 6; 
int state_on_Line = 0;
int setpoint;
int _lastPosition;
int sensor_pin_A[] = {1,2,3,4,5,6}; 
int sensor_pin_B[] = {1,2,3,4,5,6}; 
const float I_MAX = 1000.0; // ขีดจำกัดบนของ integral
const float I_MIN = -1000.0; // ขีดจำกัดล่างของ integral

//-------------------------------------------------------->>fline
int rgb[] = {24, 25, 28};
bool pid_error = true;

const int ramp_delay = 6; // ms
int slmotor = 20, srmotor = 20; // PWM
int clml = -90, clmr = 90; // เลี้ยวซ้าย center
int crml = 90, crmr = -90; // เลี้ยวขวา center
int flml = -10, flmr = 100; // เลี้ยวซ้าย front
int frml = 100, frmr = -10; // เลี้ยวขวา front
int llmotor = 100, lrmotor = 50, ldelaymotor = 50; // เลี้ยวซ้าย speed
int rlmotor = 50, rrmotor = 100, rdelaymotor = 50; // เลี้ยวขวา speed
int break_ff = 5, break_fc = 30, break_bf = 10, break_bc = 20; // การหน่วง
int delay_f = 15; // การหน่วงก่อนเลี้ยว
float kd_f = 0.75, kd_b = 0.025; // Kd PID
float kp_slow = 0.2, ki_slow = 0.0001; // PID ช้า
float redius_wheel = 3.0; // รัศมีล้อ (cm)
int ch_p = 0;
bool _fw = true;
float new_encoder = 0;


//___--------------------------------------------->>
void get_EEP_Program(void);
void read_sensorA_program(void);

void setup_rp2350_pro() 
  {
    Wire.begin();
    Serial.begin(115200);
    my_GYRO::begin();
    my_GYRO::resetAngles();
    pinMode(24, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(28, OUTPUT);
    pinMode(32, OUTPUT);

    pinMode(33, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    for(int i = 0; i<2; i++)
      {
        for(int i = 0; i<3; i++)
          {
            digitalWrite(rgb[i],1);
            delay(50);
              digitalWrite(rgb[i],0);
              delay(50);
            }
        }
      digitalWrite(rgb[2],1);     
      get_EEP_Program();
      read_sensorA_program();
   
  }


/*
   get_maxmin_A();
   get_maxmin_B();
   get_maxmin_C(); 
   read_eepA();
   read_sensorA_program();
   read_eepB();
   read_sensorB_program();
   read_eepC();
   read_sensorC_program();
*/
uint16_t read_sensorA(int sensor) 
  {       
     adc.begin(14, 15, 16, 13 );
     adc.begin(14, 15, 16, 17 );  //adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
     return adc.readADC(sensor);  
  }

uint16_t read_sensorB(int sensor) 
  {       
     adc.begin(14, 15, 16, 17 ); 
     adc.begin(14, 15, 16, 13 );  //adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
     return adc.readADC(sensor);  
  }
int md_sensorA(int sensor) 
  {      
     return (sensorMax_A[sensor]+sensorMin_A[sensor])/2;
  }

int md_sensorB(int sensor) 
  {      
     return (sensorMax_B[sensor]+sensorMin_B[sensor])/2;
  }
int md_sensorC(int sensor) 
  {      
     return (sensorMax_C[sensor]+sensorMin_C[sensor])/2;
  }

// ฟังก์ชันเขียนข้อมูลลง EEPROM
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8)); // ส่ง MSB ของที่อยู่
  Wire.write((int)(eeAddress & 0xFF)); // ส่ง LSB ของที่อยู่
  for (int i = 0; i < dataLength; i++) {
    Wire.write(data[i]); // ส่งข้อมูลทีละไบต์
  }
  Wire.endTransmission();
  delay(5); // รอให้ EEPROM เขียนข้อมูลเสร็จ
}

// ฟังก์ชันอ่านข้อมูลจาก EEPROM
void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8)); // ส่ง MSB ของที่อยู่
  Wire.write((int)(eeAddress & 0xFF)); // ส่ง LSB ของที่อยู่
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, dataLength); // ขอข้อมูล
  for (int i = 0; i < dataLength; i++) {
    if (Wire.available()) {
      buffer[i] = Wire.read(); // อ่านข้อมูลทีละไบต์
    }
  }
}

void get_maxmin_A() 
  {
      // อ่านค่าและเก็บไว้สำหรับ read_sensorA
      for (int sample = 0; sample < numSamples; sample++) 
        {
          for (int sensor = 0; sensor < numSensors; sensor++) 
            {
              sensorValuesA[sensor][sample] =  read_sensorA(sensor); // สมมติว่ามีฟังก์ชันนี้
              delay(1);
            }
        }

      // คำนวณ Max และ Min ของแต่ละเซนเซอร์
      for (int sensor = 0; sensor < numSensors; sensor++) 
        {
          sensorMaxA[sensor] = sensorValuesA[sensor][0];
          sensorMinA[sensor] = sensorValuesA[sensor][0];
          
          for (int sample = 1; sample < numSamples; sample++) {
            int value = sensorValuesA[sensor][sample];
            if (value > sensorMaxA[sensor]) {
              sensorMaxA[sensor] = value;
            }
            if (value < sensorMinA[sensor]) {
              sensorMinA[sensor] = value;
            }
          }
        }

      // บันทึก sensorMaxA และ sensorMinA ลง EEPROM
      byte buffer[16];
      for (int i = 0; i < numSensors; i++) 
        {
          buffer[i * 2] = highByte(sensorMaxA[i]);
          buffer[i * 2 + 1] = lowByte(sensorMaxA[i]);
        }
      writeEEPROM(EEPROM_ADDRESS, 0, buffer, 16); // sensorMaxA ที่ที่อยู่ 0

      for (int i = 0; i < numSensors; i++) {
        buffer[i * 2] = highByte(sensorMinA[i]);
        buffer[i * 2 + 1] = lowByte(sensorMinA[i]);
      }
      writeEEPROM(EEPROM_ADDRESS, 16, buffer, 16); // sensorMinA ที่ที่อยู่ 16

      tone(32, 950, 100);
      delay(200);
      tone(32, 950, 200);
      delay(200);

      // แสดงผล
      Serial.println("Sensor A Results:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(sensorMaxA[sensor]);
        Serial.print(", Min: ");
        Serial.println(sensorMinA[sensor]);
      }

      // อ่านและตรวจสอบจาก EEPROM
      byte readBuffer[16];
      int readMaxA[numSensors], readMinA[numSensors];
      readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }
      readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }

      Serial.println("Sensor A Values read from EEPROM:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(readMaxA[sensor]);
        Serial.print(", Min: ");
        Serial.println(readMinA[sensor]);
      }
  }

void get_maxmin_B() {
  // อ่านค่าและเก็บไว้สำหรับ read_sensorB
  for (int sample = 0; sample < numSamples; sample++) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      sensorValuesB[sensor][sample] = read_sensorB(sensor); // สมมติว่ามีฟังก์ชันนี้
      delay(1);
    }
  }

  // คำนวณ Max และ Min ของแต่ละเซนเซอร์
  for (int sensor = 0; sensor < numSensors; sensor++) {
    sensorMaxB[sensor] = sensorValuesB[sensor][0];
    sensorMinB[sensor] = sensorValuesB[sensor][0];
    
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesB[sensor][sample];
      if (value > sensorMaxB[sensor]) {
        sensorMaxB[sensor] = value;
      }
      if (value < sensorMinB[sensor]) {
        sensorMinB[sensor] = value;
      }
    }
  }

  // บันทึก sensorMaxB และ sensorMinB ลง EEPROM
  byte buffer[16];
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMaxB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 32, buffer, 16); // sensorMaxB ที่ที่อยู่ 32

  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMinB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 48, buffer, 16); // sensorMinB ที่ที่อยู่ 48

  tone(32, 950, 100);
  delay(200);
  tone(32, 950, 200);
  delay(200);

  // แสดงผล
  Serial.println("Sensor B Results:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(sensorMaxB[sensor]);
    Serial.print(", Min: ");
    Serial.println(sensorMinB[sensor]);
  }

  // อ่านและตรวจสอบจาก EEPROM
  byte readBuffer[16];
  int readMaxB[numSensors], readMinB[numSensors];
  readEEPROM(EEPROM_ADDRESS, 32, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMaxB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 48, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMinB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor B Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(readMaxB[sensor]);
    Serial.print(", Min: ");
    Serial.println(readMinB[sensor]);
  }
}

void get_maxmin_C() {
  // อ่านค่าและเก็บไว้สำหรับ analogRead(46) และ analogRead(47)
  for (int sample = 0; sample < numSamples; sample++) {
    sensorValuesC[0][sample] = analogRead(46); // เซนเซอร์ C0 (GPIO 46)
    sensorValuesC[1][sample] = analogRead(47); // เซนเซอร์ C1 (GPIO 47)
    delay(5);
  }

  // คำนวณ Max และ Min สำหรับเซนเซอร์ C0 และ C1
  for (int sensor = 0; sensor < 2; sensor++) {
    sensorMaxC[sensor] = sensorValuesC[sensor][0];
    sensorMinC[sensor] = sensorValuesC[sensor][0];
    
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesC[sensor][sample];
      if (value > sensorMaxC[sensor]) {
        sensorMaxC[sensor] = value;
      }
      if (value < sensorMinC[sensor]) {
        sensorMinC[sensor] = value;
      }
    }
  }

  // บันทึก sensorMaxC และ sensorMinC ลง EEPROM
  byte buffer[4];
  for (int i = 0; i < 2; i++) {
    buffer[i * 2] = highByte(sensorMaxC[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 64, buffer, 4); // sensorMaxC ที่ที่อยู่ 64

  for (int i = 0; i < 2; i++) {
    buffer[i * 2] = highByte(sensorMinC[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 68, buffer, 4); // sensorMinC ที่ที่อยู่ 68

  tone(32, 950, 100);
  delay(200);
  tone(32, 950, 200);
  delay(200);

  // แสดงผล
  Serial.println("Sensor C Results:");
  Serial.print("Sensor C0 (Pin 46) => Max: ");
  Serial.print(sensorMaxC[0]);
  Serial.print(", Min: ");
  Serial.println(sensorMinC[0]);
  Serial.print("Sensor C1 (Pin 47) => Max: ");
  Serial.print(sensorMaxC[1]);
  Serial.print(", Min: ");
  Serial.println(sensorMinC[1]);
}

void read_eepA()
  {      
      // อ่านและตรวจสอบจาก EEPROM
      byte readBuffer[16];
      int readMaxA[numSensors], readMinA[numSensors];
      readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }
      readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }

      Serial.println("Sensor A Values read from EEPROM:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(readMaxA[sensor]);
        Serial.print(", Min: ");
        Serial.println(readMinA[sensor]);
        sensorMax_A[sensor] = readMaxA[sensor];
        sensorMin_A[sensor] = readMinA[sensor];
      }   
  }


void read_sensorA_program()
  { 
      Serial.println("Sensor MAX A Values read from program:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(sensorMax_A[sensor]);
        Serial.print(", Min: ");
        Serial.println(sensorMin_A[sensor]);
      }   
  }
void read_eepB()
  {    
      // อ่านและตรวจสอบจาก EEPROM
      byte readBuffer[16];
      int readMaxB[numSensors], readMinB[numSensors];
      readEEPROM(EEPROM_ADDRESS, 32, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMaxB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }
      readEEPROM(EEPROM_ADDRESS, 48, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMinB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }

      Serial.println("Sensor B Values read from EEPROM:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(readMaxB[sensor]);
        Serial.print(", Min: ");
        Serial.println(readMinB[sensor]);
        sensorMax_B[sensor] = readMaxB[sensor];
        sensorMin_B[sensor] = readMinB[sensor];
      }
  }
void read_sensorB_program()
  { 
      Serial.println("Sensor MAX B Values read from program:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(sensorMax_B[sensor]);
        Serial.print(", Min: ");
        Serial.println(sensorMin_B[sensor]);
      }   
  }

void read_eepC() {
  // อ่านและตรวจสอบจาก EEPROM
  byte readBuffer[4];
  int readMaxC[2], readMinC[2];
  readEEPROM(EEPROM_ADDRESS, 64, readBuffer, 4);
  for (int i = 0; i < 2; i++) {
    readMaxC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 68, readBuffer, 4);
  for (int i = 0; i < 2; i++) {
    readMinC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor C Values read from EEPROM:");
  Serial.print("Sensor C0 (Pin 46) => Max: ");
  Serial.print(readMaxC[0]);
  Serial.print(", Min: ");
  Serial.println(readMinC[0]);
  Serial.print("Sensor C1 (Pin 47) => Max: ");
  Serial.print(readMaxC[1]);
  Serial.print(", Min: ");
  Serial.println(readMinC[1]);
  sensorMax_C[0] = readMaxC[0];
  sensorMin_C[0] = readMinC[0];
  sensorMax_C[1] = readMaxC[1];
  sensorMin_C[1] = readMinC[1];
}

void read_sensorC_program() {
  Serial.println("Sensor C Values read from program:");
  Serial.print("Sensor C0 (Pin 46) => Max: ");
  Serial.print(sensorMax_C[0]);
  Serial.print(", Min: ");
  Serial.println(sensorMin_C[0]);
  Serial.print("Sensor C1 (Pin 47) => Max: ");
  Serial.print(sensorMax_C[1]);
  Serial.print(", Min: ");
  Serial.println(sensorMin_C[1]);
}

void get_EEP_Program()
  {
    read_eepA();
    read_eepB();
    read_eepC();
    read_sensorA_program();
    read_sensorB_program();
    read_sensorC_program();
  }

//-------------------------------------------------------------------------------------->>ควบคุมมอเตอร์

void sw()
  {
    tone(32, 3000, 100);
    delay(200); // รอ 1 วินาที
    tone(32, 3000, 200);
    delay(200); // รอ 1 วินาที

    int buttonState;
    unsigned long pressStartTime = 0;
    bool isPressed = false;   

    while(1)
      {       
        if(digitalRead(19) == 0)
            {
              tone(32, 950, 100);              
               digitalWrite(rgb[2],0);
               digitalWrite(rgb[1],1);
               delay(200); // รอ 1 วินาที
              get_maxmin_A();
              for(int i = 0; i<2; i++)
                {
                  digitalWrite(rgb[1],1);
                  delay(100);
                  digitalWrite(rgb[1],0);
                  delay(100);
                }
              digitalWrite(rgb[1],0);
              digitalWrite(rgb[2],1);
              delay(50);

            }
        if(digitalRead(12) == 0)
            {
              tone(32, 950, 100);
              digitalWrite(rgb[2],0);
               digitalWrite(rgb[1],1);
               delay(200); // รอ 1 วินาที
              get_maxmin_B();
              for(int i = 0; i<2; i++)
                {
                  digitalWrite(rgb[1],1);
                  delay(100);
                  digitalWrite(rgb[1],0);
                  delay(100);
                }
              digitalWrite(rgb[1],0);
              digitalWrite(rgb[2],1);
              delay(50);
            }
        Serial.print("From A ");
          for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorA(i));  // ใช้ read_sensorA
            Serial.print(" ");
          }
          Serial.print("   ");
          
          // แสดงค่าจาก Nano 0x09
          Serial.print("From B ");
          for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorB(i));  // ใช้ read_sensorB
            Serial.print(" ");
          }
          Serial.println("  ");
        // Serial.println(my_tcs('r'));
          
          delay(10);  // อัปเดตทุก 100ms
        Serial.println(" "); 
        buttonState = digitalRead(33);
        if (buttonState == LOW) 
          {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
            digitalWrite(rgb[2],0);
            if (!isPressed) \
              {
                pressStartTime = millis();  // บันทึกเวลาที่กดปุ่มครั้งแรก
                isPressed = true;
              } 
            else 
              {
                unsigned long pressDuration = millis() - pressStartTime;    
                if (pressDuration >= 3000) 
                  {  // กดค้าง 3 วินาที
                    tone(32, 950, 100);
                    delay(200); // รอ 1 วินาที
                    tone(32, 950, 200);
                    delay(200); // รอ 1 วินาที
                    Serial.println("Entering Mode A");
                    digitalWrite(rgb[2],0);
               digitalWrite(rgb[1],1);
               delay(200); // รอ 1 วินาที
              get_maxmin_C();
              for(int i = 0; i<2; i++)
                {
                  digitalWrite(rgb[1],1);
                  delay(100);
                  digitalWrite(rgb[1],0);
                  delay(100);
                }
              digitalWrite(rgb[1],0);
              digitalWrite(rgb[2],1);
              delay(50);
                    while (digitalRead(33) == LOW);  // รอให้ปล่อยปุ่ม
                    delay(200);  // ป้องกันการเด้งของปุ่ม
                  }
              }
          } 
        else 
          {
            if (isPressed) 
              {
                unsigned long pressDuration = millis() - pressStartTime;
                
                if (pressDuration >= 50 && pressDuration < 3000) 
                  {  
                    Serial.println("Entering Mode B");
                    break;
                  }
                isPressed = false;
              }
          }
      }
    tone(32, 3000, 400);
    delay(500);
  }

///----------------------------------------------------------------------------------->>>>
///---------------------------------------------------------------------------------->>>>
int sl, sr; // ตัวแปรความเร็วสำหรับมอเตอร์ซ้ายและขวา

// ฟังก์ชันควบคุมมอเตอร์ซ้าย/ขวา
void Motor(int pwmL, int pwmR) {
  delayMicroseconds(50); 
  // ตั้งความละเอียด PWM เป็น 12 บิต (0–4095)
  analogWriteResolution(12);

  // ตั้งความถี่ PWM เป็น 20kHz (ลดเสียงรบกวนมอเตอร์)
  analogWriteFreq(20000);
  // แปลงค่าจาก -100..100 ให้เป็น 0..4095
  int pwmValueL = map(abs(pwmL), 0, 100, 0, 4095);
  int pwmValueR = map(abs(pwmR), 0, 100, 0, 4095);

  // มอเตอร์ซ้าย
  if (pwmL > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwmL < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    pwmValueL = 0;
  }

  // มอเตอร์ขวา
  if (pwmR > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (pwmR < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    pwmValueR = 0;
  }

  // ส่งค่า PWM (0–4095)
  analogWrite(PWMA, pwmValueL);
  analogWrite(PWMB, pwmValueR);
}

//-------------------------------------------------------------------------------------->>ควบคุมมอเตอร์

//-------------------------------------------------------------------------------------->>ควบคุมservo

#include <Servo.h>
#define servo39 39
#define servo38 38
#define servo37 37
#define servo36 36
#define servo35 35
#define servo34 34
Servo servo_39;
Servo servo_38;
Servo servo_37;
Servo servo_36;
Servo servo_35;
Servo servo_34;


void servo(int servo,int angle)
{  
  if (servo==39)
    {
        servo_39.attach(servo39, 500, 2500);
        servo_39.write(angle);        
    }
  else if (servo==38)
    {
        servo_38.attach(servo38, 500, 2500);
        servo_38.write(angle);        
    }  
  else if (servo==37)
    {
        servo_37.attach(servo37, 500, 2500);
        servo_37.write(angle);        
    }
  else if (servo==36)
    {
        servo_36.attach(servo36, 500, 2500);
        servo_36.write(angle);        
    }
   else if (servo==35)
    {
        servo_35.attach(servo35, 500, 2500);
        servo_35.write(angle);        
    }
  else if (servo==34)
    {
        servo_34.attach(servo34, 500, 2500);
        servo_34.write(angle);        
    }
}

//-------------------------------------------------------------------------------------->>ควบคุมservo


//-------------------------------------------------------------------------------------->>ควบคุม PID


int position_A()  
   {        
      int min_sensor_values_A[] = { sensorMin_A[1], sensorMin_A[2], sensorMin_A[3], sensorMin_A[4], sensorMin_A[5], sensorMin_A[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int max_sensor_values_A[] = { sensorMax_A[1], sensorMax_A[2], sensorMax_A[3], sensorMax_A[4], sensorMax_A[5], sensorMax_A[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorA(sensor_pin_A[i]), min_sensor_values_A[i], max_sensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 0;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 5000;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }
int position_A_none()  
   {        
      int min_sensor_values_A[] = { sensorMin_A[1], sensorMin_A[2], sensorMin_A[3], sensorMin_A[4], sensorMin_A[5], sensorMin_A[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int max_sensor_values_A[] = { sensorMax_A[1], sensorMax_A[2], sensorMax_A[3], sensorMax_A[4], sensorMax_A[5], sensorMax_A[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorA(sensor_pin_A[i]), min_sensor_values_A[i], max_sensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 2500;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 2500;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }

float error_A()
    {
      if(pid_error == true)
        {
          present_position = position_A_none()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }
      else
        {
          present_position = position_A()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }                            
    }
float error_AA()
    {
          present_position = position_A()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;                          
    }
float error_AN()
    {
          present_position = position_A_none() / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;                          
    }
int position_B()  
   {        
      int min_sensor_values_A[] = { sensorMin_B[1], sensorMin_B[2], sensorMin_B[3], sensorMin_B[4], sensorMin_B[5], sensorMin_B[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int max_sensor_values_A[] = { sensorMax_B[1], sensorMax_B[2], sensorMax_B[3], sensorMax_B[4], sensorMax_B[5], sensorMax_B[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorB(sensor_pin_A[i]), min_sensor_values_A[i], max_sensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 0;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 5000;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }
int position_B_none()  
   {        
      int min_sensor_values_A[] = { sensorMin_B[1], sensorMin_B[2], sensorMin_B[3], sensorMin_B[4], sensorMin_B[5], sensorMin_B[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int max_sensor_values_A[] = { sensorMax_B[1], sensorMax_B[2], sensorMax_B[3], sensorMax_B[4], sensorMax_B[5], sensorMax_B[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorB(sensor_pin_A[i]), min_sensor_values_A[i], max_sensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 2500;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 2500;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }

float error_B()
    {
      if(pid_error == true)
        {
          present_position = position_B_none()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }
      else
        {
          present_position = position_B()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }                            
    }

float error_BB()
    {
      
          present_position = position_B()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
                                
    }

float error_BN()
    {
      
          present_position = position_B_none()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
                                 
    }



//------------------------------------------------------------------------------------------------------------------------>>>คำสั่งเดินตามเส้น
void fw(int sl, int sr, float kp)
  {
    while(1) 
    {
      delayMicroseconds(50);
      errors = error_A();
      P = errors;
      D = errors - previous_error;                  
      previous_error = errors;
      PID_output = (kp * P) + (0.0001 * I) + (0.025 * D); 
      Motor(sl - PID_output, sr + PID_output);
      
    }
      
  }
void fws(int sl, int sr, float kp) 
{
    int current_speed = 0;    // เริ่มจากความเร็ว 0
    int target_speed = (sl < sr) ? sl : sr;  // เลือกความเร็วต่ำสุดเป็นเป้าหมายเพื่อสมดุล
    const int ramp_step = 2;  // ความเร็วเพิ่มครั้งละ 2 (ปรับได้ตามความนุ่มนวลที่ต้องการ)
    const int ramp_delay = 10; // หน่วงระหว่างเพิ่มความเร็ว (ms)

    // Soft start เพิ่มความเร็วจนถึงเป้าหมาย
    while (current_speed < target_speed) 
    {
        errors = error_A();
        P = errors;
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(current_speed - PID_output, current_speed + PID_output);

        current_speed += ramp_step;
        if (current_speed > target_speed) current_speed = target_speed;

        delay(ramp_delay);
    }

    // เดินหน้าปกติเมื่อถึงความเร็วที่ต้องการ
    while (1) 
    {
        delayMicroseconds(50);
        errors = error_A();
        P = errors;
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(sl - PID_output, sr + PID_output);
    }
}



void fws(int sl, int sr, float kp, float distance) 
{
    int current_speed = 0;    // เริ่มจากความเร็ว 0 (หน่วย: cm/s)
    int target_speed = (sl < sr) ? sl : sr;  // เลือกความต่ำสุดเพื่อสมดุล
    const int ramp_step = 2;  // ความเร็วเพิ่มครั้งละ 2
    const int ramp_delay = 10; // หน่วงระหว่างเพิ่มความเร็ว (ms)
    float errors, P, D, previous_error = 0, PID_output;
    float I = 0;  // ตัวแปร Integral
    float traveled_distance = 0;  // ระยะทาง (หน่วย: เซนติเมตร)
    unsigned long last_time = millis();  // เก็บเวลาเริ่มต้น

    // Soft start เพิ่มความเร็วจนถึงเป้าหมาย
    while (current_speed < target_speed) 
    {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);  // อัพเดท Integral
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(current_speed - PID_output, current_speed + PID_output);

        // คำนวณระยะทางถ้ากำหนด distance > 0
        if (distance > 0) {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;  // หน่วยวินาที
            traveled_distance += current_speed * delta_time;  // ความเร็ว cm/s * เวลา วินาที = ระยะทาง cm
            last_time = current_time;

            // ตรวจสอบระยะทาง
            if (traveled_distance >= distance) 
            {
                Motor(0, 0);  // หยุดมอเตอร์
                return;  // ออกจากฟังก์ชัน
            }
        }

        current_speed += ramp_step;
        if (current_speed > target_speed) current_speed = target_speed;

        delay(ramp_delay);
    }

    // เดินหน้าปกติ
    while (1) 
    {
        errors = error_A();
        P = errors;
        I += errors * 0.00005;  // อัพเดท Integral (สำหรับ 50us)
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(sl - PID_output, sr + PID_output);

        // คำนวณระยะทางถ้ากำหนด distance > 0
        if (distance > 0) {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;  // หน่วยวินาที
            traveled_distance += target_speed * delta_time;  // ความเร็ว cm/s * เวลา วินาที = ระยะทาง cm
            last_time = current_time;

            // ตรวจสอบระยะทาง
            if (traveled_distance >= distance) 
            {
                Motor(0, 0);  // หยุดมอเตอร์
                return;  // ออกจากฟังก์ชัน
            }
        }

        delayMicroseconds(50);
    }

    ///////////////////////////////-------------------------------------------------------->>>>>>>>  PID

}



void wheel_redius(float rediuss) {
    redius_wheel = rediuss;
}

float wheel_distance() {
    return 2 * 3.14159 * redius_wheel / 10; // เซนติเมตร
}

void test_distance(int distance1) {
    new_encoder = 440 * distance1 / wheel_distance();
    Serial.println(new_encoder);
    encoder.resetEncoders();
    do {
        Motor(30, 30);
    } while (encoder.Poss_R() < new_encoder);
    Motor(-30, -30);
    delay(10);
    Motor(1, 1);
}

void to_slow_motor(int sl, int sr) {
    slmotor = sl;
    srmotor = sr;
}

void to_turn_center_l(int ml, int mr) {
    clml = ml;
    clmr = mr;
}

void to_turn_center_r(int ml, int mr) {
    crml = ml;
    crmr = mr;
}

void to_turn_front_l(int ml, int mr) {
    flml = ml;
    flmr = mr;
}

void to_turn_front_r(int ml, int mr) {
    frml = ml;
    frmr = mr;
}

void to_brake_fc(int ff, int fc) {
    break_ff = ff;
    break_fc = fc;
}

void to_brake_bc(int bf, int bc) {
    break_bf = bf;
    break_bc = bc;
}

void to_delay_f(int ff) {
    delay_f = ff;
}

void to_speed_turn_fl(int inM, int outM, int delayM) {
    llmotor = inM;
    lrmotor = outM;
    ldelaymotor = delayM;
}

void to_speed_turn_fr(int inM, int outM, int delayM) {
    rlmotor = inM;
    rrmotor = outM;
    rdelaymotor = delayM;
}

void kd_fw(float kd) {
    kd_f = kd;
}

void kd_bw(float kd) {
    kd_b = kd;
}

void kp_sl(float kp_sl, float ki_sl) {
    kp_slow = kp_sl;
    ki_slow = ki_sl;
}

void turn_speed_fl() {
    for (int t = 0; t < ldelaymotor; t++) {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_f * D);
        Motor(llmotor + PID_output, lrmotor - PID_output);
        delay(ramp_delay);
    }
}

void turn_speed_fr() {
    for (int t = 0; t < rdelaymotor; t++) {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_f * D);
        Motor(rlmotor + PID_output, rrmotor - PID_output);
        delay(ramp_delay);
    }
}

void bturn_speed_fl() {
    for (int t = 0; t < ldelaymotor; t++) {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_b * D);
        Motor(-(llmotor + PID_output), -((lrmotor - 10) - PID_output));
        delay(ramp_delay);
    }
    delay(2);
}

void bturn_speed_fr() {
    for (int t = 0; t < rdelaymotor; t++) {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_b * D);
        Motor(-(rlmotor + PID_output), -((rrmotor - 10) - PID_output));
        delay(ramp_delay);
    }
    delay(2);
}


void fline(int spl, int spr, float kp, float distance, char nfc, char splr, int power, String sensor, int endt) {
    _fw = true;
    char sensors[4];
    sensor.toCharArray(sensors, sizeof(sensors));
    int sensor_f = atoi(&sensors[1]);
    int target_speed = min(spl, spr); // PWM
    const int ramp_step = 2;
    float traveled_distance = 0;
    unsigned long last_time = millis();
    float speed_scale = 1.0; // สมมติ 1 PWM = 0.1 cm/s (ต้องปรับตามจริง)

    if (kp == 0)
       {
        I = kp_slow = ki_slow = 0;
       }
    if(kp < 6.5)
      {
          pid_error = true;
      }
    else
      {
        pid_error = false;
      }

    int current_speed = 0; // PWM
    if(spl == 0)
      {
        goto _line;
      }
    // Soft start
    if (!ch_p) 
      {
        while (current_speed < target_speed) 
          {
            if(kp <= 0.65)
              {
                          if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) 
                            {
                              errors = 0;
                            } 
                          else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) 
                            {
                                errors = 10;
                            } 
                          else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) 
                            {
                                errors = -10;
                            } 
                          else 
                            {
                              errors = error_A();
                            }
              }
           else  
              {
                  errors = error_A();
              }
            P = errors;
            I += errors * (ramp_delay / 1000.0);
            D = errors - previous_error;
            previous_error = errors;
            PID_output = (kp * P) + (0.000001 * I) + (kd_f * D);
            Motor(current_speed - PID_output, current_speed + PID_output);

            if (distance > 0) {
                unsigned long current_time = millis();
                float delta_time = (current_time - last_time) / 1000.0;
                traveled_distance += (current_speed * speed_scale) * delta_time;
                last_time = current_time;
                if (traveled_distance >= distance) {
                    Motor(0, 0);
                    return;
                }
            }
            current_speed += ramp_step;
            if (current_speed > target_speed) current_speed = target_speed;
            delay(ramp_delay);
            Serial.println(errors);
        }
      }

    // วิ่งปกติ
    
    while (1) 
      {
        if(kp <= 0.65)
                        {
                          if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) 
                            {
                              errors = 0;
                            } 
                          else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) 
                            {
                                errors = 10;
                            } 
                          else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) 
                            {
                                errors = -10;
                            } 
                          else 
                            {
                              errors = error_A();
                            }
                        }
                      else  
                        {
                          errors = error_AA();
                        } 
        P = errors;
        I += errors * 0.00005; // สำหรับ 50us
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.000001 * I) + (kd_f * D);
        Motor(spl - PID_output, spr + PID_output);       
        if (distance > 0) 
          {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 500.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;
            if (traveled_distance >= distance) {
                for(int i=spl; i>slmotor ; i--)
                  {
                    errors = error_A();
                    P = errors;
                    I += errors * 0.00005; // สำหรับ 50us
                    D = errors - previous_error;
                    previous_error = errors;
                    PID_output = (kp/2 * P) + (0.000001 * I) + (kd_f * D);
                    Motor(i - PID_output, i + PID_output);
                  }
                Motor(0, 0);
                
                break;
            }
          }
        else
          {
            if(kp >= 6.5 && kp < 1.5)
              {
                 if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)&& read_sensorA(2) < md_sensorA(2)) ||
                    (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) && read_sensorA(5) < md_sensorA(5))) 
                      {
                        break;
                      }
              }
            else if(kp < 6.5)
              {
                if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
                    (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) 
                      {
                        break;
                      }
              }
            else {}
            
          }
        delayMicroseconds(50);
     }
    
    _line:
    ch_p = 0;
    
    if (nfc == 'n')
      {          
          if (splr == 'p')
            {
              ch_p = 1;
              if(spl >= 1)
                {
                  while (1) 
                    {
                      if(kp <= 0.65)
                        {
                          if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) 
                            {
                              errors = 0;
                            } 
                          else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) 
                            {
                                errors = 10;
                            } 
                          else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) 
                            {
                                errors = -10;
                            } 
                          else 
                            {
                              errors = error_A();
                            }
                        }
                      else  
                        {
                          errors = error_A();
                        }
                        
                            P = errors;
                            I += errors * 0.00005;
                            D = errors - previous_error;
                            previous_error = errors;
                            PID_output = (kp_slow * P) + (ki_slow * D);
                            Motor(slmotor - PID_output, srmotor + PID_output);
                            delayMicroseconds(50);
                            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) ) ||
                                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) {
                                break;
                            }
                    }
                }
              else
                {
                  while (1) {
                        if(kp <= 0.65)
                        {
                          if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) 
                            {
                              errors = 0;
                            } 
                          else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) 
                            {
                                errors = 10;
                            } 
                          else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) 
                            {
                                errors = -10;
                            } 
                          else 
                            {
                              errors = error_A();
                            }
                        }
                      else  
                        {
                          errors = error_A();
                        }
                            P = errors;
                            I += errors * 0.00005;
                            D = errors - previous_error;
                            previous_error = errors;
                            PID_output = (kp_slow * P) + (ki_slow * D);
                            Motor(slmotor - PID_output, srmotor + PID_output);
                            delayMicroseconds(50);
                            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) ) ||
                                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) {
                                break;
                            }
                        }
                }
            }
        else if (splr == 's')
            {
              if(endt > 0)
                {
                  Motor(-spl, -spr);
                  delay(endt);
                  Motor(-1, -1);
                  delay(10); 
                }
              else
                {
                  for(int i=spl; i>15; i--)
                    {
                      errors = error_A();
                
                      P = errors;
                      I += errors * 0.00005;
                      D = errors - previous_error;
                      previous_error = errors;
                      PID_output = (kp_slow * P) + (ki_slow * D);
                      Motor(slmotor - PID_output, srmotor + PID_output);
                      delayMicroseconds(50);
                    }
                  
                }
               
              goto  _entN;
            }
      }
    else if (nfc == 'f') 
      {
        if (distance > 0 || spl == 0) 
          {
            if(spl==0)
              {
                while (1) 
                  {              
                    errors = error_A();
                    P = errors;
                    I += errors * 0.00005;
                    D = errors - previous_error;
                    previous_error = errors;
                    PID_output = (kp_slow * P) + (ki_slow * D);
                    Motor(slmotor - PID_output, srmotor + PID_output);
                    delayMicroseconds(50);
                    if ((read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(1) < md_sensorA(1)-50 )  ||
                        (read_sensorA(7) < md_sensorA(7)-50  && read_sensorA(6) < md_sensorA(6)-50  )) {
                        break;
                    }
                  }            
              }
            else
              {
                for(int i=spl;i>slmotor;i--)
                      {
                        errors = error_A();
                        P = errors;
                        I += errors * 0.00005;
                        D = errors - previous_error;
                        previous_error = errors;
                        PID_output = (kp_slow * P) + (ki_slow * D);
                        Motor(i - PID_output, i + PID_output);
                        delayMicroseconds(50);
                        if ((read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(1) < md_sensorA(1)-50 )  ||
                            (read_sensorA(7) < md_sensorA(7)-50  && read_sensorA(6) < md_sensorA(6)-50  )) 
                            {
                              goto en_for;
                            }
                        delay(3);
                      }
                while (1) 
                  {  
                        errors = error_A();
                        P = errors;
                        I += errors * 0.00005;
                        D = errors - previous_error;
                        previous_error = errors;
                        PID_output = (kp_slow * P) + (ki_slow * D);
                        Motor(slmotor - PID_output, slmotor + PID_output);
                        delayMicroseconds(50);
                        if ((read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(1) < md_sensorA(1)-50 )  ||
                            (read_sensorA(7) < md_sensorA(7)-50  && read_sensorA(6) < md_sensorA(6)-50  )) 
                            {
                              break;
                            }                          
                  }
                en_for: delay(10);  
              }
          }
        if (splr == 'p') 
          {
            ch_p = 1;
            while (1) 
              {
                Motor(spl, spr);
                delayMicroseconds(50);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) {
                    break;
                }
              }
            delay(10);
            if (endt > 0) 
              {
                Motor(-spl, -spr);
                delay(endt);
                Motor(-1, -1);
                delay(10);
              } 
            else 
              {
                Motor(0, 0);
              }

          } 
        else if (splr == 's') 
          {
            Motor(-spl, -spr);
            delay(endt);
            Motor(0, 0);
            delay(2);
          }
      } 
    else if (nfc == 'c') 
      {
        if(splr == 'p') 
          {
            ch_p = 1;
            while (1) 
              {
                if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                    errors = 0;
                } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                    errors = 10;
                } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                    errors = -10;
                } else {
                  errors = error_A();
                }
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
                Motor(spl - PID_output, spr + PID_output);
                delayMicroseconds(50);
                if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                    break;
                }
              }
            if (endt > 0) 
              {
                Motor(-spl, -spr);
                delay(endt);
                Motor(-1, -1);
                delay(10);
              } 
            else 
              {
                Motor(0, 0);
              }
          }
        else
          {
            if (distance > 0)
              {
                for(int i=spl;i>slmotor;i--)
                      {
                        if (read_sensorA(1) > md_sensorA(1) &&read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6)) {
                            errors = 0;
                        } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                            errors = 10;
                        } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                            errors = -10;
                        } else {
                          errors = error_A();
                        }
                        P = errors;
                        I += errors * 0.00005;
                        D = errors - previous_error;
                        previous_error = errors;
                        PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
                        Motor(i - PID_output, i + PID_output);
                        delayMicroseconds(50);
                        if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                            break;
                        }
                        delay(3);
                      }
              }
            while (1) 
              {
                if (read_sensorA(1) > md_sensorA(1) &&read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6)) {
                    errors = 0;
                } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                    errors = 10;
                } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                    errors = -10;
                } else {
                  errors = error_A();
                }
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
                Motor(slmotor - PID_output, slmotor + PID_output);
                delayMicroseconds(50);
                if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                    break;
                }
            }
          }
        if (splr == 's') {
            Motor(-spl, -spr);
            delay(endt);
            Motor(0, 0);
            delay(2);
        }
    }

    if (splr == 'l') 
      {
        if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0)) {
            while (1) {
                Motor(slmotor, srmotor);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) {
                    delay(delay_f);
                    break;
                }
            }
            Motor(-(slmotor), -(srmotor));
            delay(break_ff);
            for (int i = 0; i <= sensor_f; i++) {
                if(sensor_f >= 2)
                  {
                    do {Motor((flml * power) / 100, (flmr * power) / 100);
                        delayMicroseconds(50);} while (read_sensorA(i+2) < md_sensorA(i+2) - 50);
                  }
                do {
                    Motor((flml * power) / 100, (flmr * power) / 100);
                    delayMicroseconds(50);
                } while (read_sensorA(i) > md_sensorA(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
            delay(nfc == 'n' ? 2 : break_fc);
            int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
            int end_i = start_i ? sensor_f : sensor_f;
            for (int i = start_i; i <= end_i; i++) {
                do {
                    Motor((clml * power) / 100, (clmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fl();
        } else {
            Motor(-((clml * power) / 100), -((clmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      } 
    else if (splr == 'r') 
      {
        if (nfc == 'f') {
            while (1) {
                Motor(slmotor, srmotor);
                delayMicroseconds(50);
                if (read_sensorA(0) >= md_sensorA(0) && read_sensorA(7) >= md_sensorA(7)) {
                    break;
                }
            }
            delay(delay_f);
            Motor(-slmotor, -srmotor);
            delay(break_ff);
            for (int i = 7; i >= sensor_f; i--) {
               if(sensor_f <= 5)
                  {
                    do {Motor((frml * power) / 100, (frmr * power) / 100);
                    delayMicroseconds(50);
                    } while (read_sensorA(i-2) < md_sensorA(i-2) - 50); 
                  }
                do {
                    Motor((frml * power) / 100, (frmr * power) / 100);
                    delayMicroseconds(50);
                } while (read_sensorA(i) > md_sensorA(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
            delay(nfc == 'n' ? 2 : break_fc);
            int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
            for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--) {
                do {
                    Motor((crml * power) / 100, (crmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fr();
        } else {
            Motor(-((crml * power) / 100), -((crmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      }
    

    
    _entN: delay(5);
}


void fline(int spl, int spr, float kp, String distance, char nfc, char splr, int power, String sensor, int endt) {
    _fw = true;
    char sensors[4];
    sensor.toCharArray(sensors, sizeof(sensors));
    int sensor_f = atoi(&sensors[1]);
    int target_speed = min(spl, spr); // PWM
    const int ramp_step = 2;
    float traveled_distance = 0;
    unsigned long last_time = millis();
    float speed_scale = 1.0; // สมมติ 1 PWM = 0.1 cm/s (ต้องปรับตามจริง)

    if (kp == 0)
       {
        I = kp_slow = ki_slow = 0;
       }
    if(kp < 6.5)
      {
          pid_error = true;
      }
    else
      {
        pid_error = false;
      }

    int current_speed = 0; // PWM
    if(spl == 0)
      {
        goto _line;
      }
 
    // วิ่งปกติ
    
    while (1) 
      {
        if(kp <= 0.65)
                        {
                          if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) 
                            {
                              errors = 0;
                            } 
                          else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) 
                            {
                                errors = 10;
                            } 
                          else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) 
                            {
                                errors = -10;
                            } 
                          else 
                            {
                              errors = error_A();
                            }
                        }
                      else  
                        {
                          errors = error_AA();
                        } 
        P = errors;
        I += errors * 0.00005; // สำหรับ 50us
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.000001 * I) + (kd_f * D);
        Motor(spl - PID_output, spr + PID_output);       
        if (distance == "a0") 
          {
            if(read_sensorA(0) < md_sensorA(0))
              {
                break;
              }
          }
        else if (distance == "a1") 
          {
            if(read_sensorA(1) < md_sensorA(1))
              {
                break;
              }
          }
        else if (distance == "a6") 
          {
            if(read_sensorA(6) < md_sensorA(6))
              {
                break;
              }
          }
        else if (distance == "a7") 
          {
            if(read_sensorA(7) < md_sensorA(7))
              {
                break;
              }
          }
        else if (distance == "a07" || distance == "a70") 
          {
            if(read_sensorA(7) < md_sensorA(7) && read_sensorA(0) < md_sensorA(0))
              {
                break;
              }
          }
        delayMicroseconds(50);
     }
    
    _line:
    ch_p = 0;
    
    if (nfc == 'n')
      {          
          if (splr == 'p')
            {
              ch_p = 1;
              if(spl >= 1)
                {
                  while (1) 
                    {
                      if(kp <= 0.65)
                        {
                          if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) 
                            {
                              errors = 0;
                            } 
                          else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) 
                            {
                                errors = 10;
                            } 
                          else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) 
                            {
                                errors = -10;
                            } 
                          else 
                            {
                              errors = error_A();
                            }
                        }
                      else  
                        {
                          errors = error_A();
                        }
                        
                            P = errors;
                            I += errors * 0.00005;
                            D = errors - previous_error;
                            previous_error = errors;
                            PID_output = (kp_slow * P) + (ki_slow * D);
                            Motor(slmotor - PID_output, srmotor + PID_output);
                            delayMicroseconds(50);
                            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) ) ||
                                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) {
                                break;
                            }
                    }
                }
              else
                {
                  while (1) {
                        if(kp <= 0.65)
                        {
                          if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) 
                            {
                              errors = 0;
                            } 
                          else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) 
                            {
                                errors = 10;
                            } 
                          else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) 
                            {
                                errors = -10;
                            } 
                          else 
                            {
                              errors = error_A();
                            }
                        }
                      else  
                        {
                          errors = error_A();
                        }
                            P = errors;
                            I += errors * 0.00005;
                            D = errors - previous_error;
                            previous_error = errors;
                            PID_output = (kp_slow * P) + (ki_slow * D);
                            Motor(slmotor - PID_output, srmotor + PID_output);
                            delayMicroseconds(50);
                            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) ) ||
                                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) {
                                break;
                            }
                        }
                }
            }
        else if (splr == 's')
            {
              if(endt > 0)
                {
                  Motor(-spl, -spr);
                  delay(endt);
                  Motor(-1, -1);
                  delay(10); 
                }
              else
                {
                  for(int i=spl; i>15; i--)
                    {
                      errors = error_A();
                
                      P = errors;
                      I += errors * 0.00005;
                      D = errors - previous_error;
                      previous_error = errors;
                      PID_output = (kp_slow * P) + (ki_slow * D);
                      Motor(slmotor - PID_output, srmotor + PID_output);
                      delayMicroseconds(50);
                    }
                  
                }
               
              goto  _entN;
            }
      }
    else if (nfc == 'f') 
      {
        if (distance > 0 || spl == 0) 
          {
            if(spl==0)
              {
                while (1) 
                  {              
                    errors = error_A();
                    P = errors;
                    I += errors * 0.00005;
                    D = errors - previous_error;
                    previous_error = errors;
                    PID_output = (kp_slow * P) + (ki_slow * D);
                    Motor(slmotor - PID_output, srmotor + PID_output);
                    delayMicroseconds(50);
                    if ((read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(1) < md_sensorA(1)-50 )  ||
                        (read_sensorA(7) < md_sensorA(7)-50  && read_sensorA(6) < md_sensorA(6)-50  )) {
                        break;
                    }
                  }            
              }
            else
              {
                for(int i=spl;i>slmotor;i--)
                      {
                        errors = error_A();
                        P = errors;
                        I += errors * 0.00005;
                        D = errors - previous_error;
                        previous_error = errors;
                        PID_output = (kp_slow * P) + (ki_slow * D);
                        Motor(i - PID_output, i + PID_output);
                        delayMicroseconds(50);
                        if ((read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(1) < md_sensorA(1)-50 )  ||
                            (read_sensorA(7) < md_sensorA(7)-50  && read_sensorA(6) < md_sensorA(6)-50  )) 
                            {
                              goto en_for;
                            }
                        delay(3);
                      }
                while (1) 
                  {  
                        errors = error_A();
                        P = errors;
                        I += errors * 0.00005;
                        D = errors - previous_error;
                        previous_error = errors;
                        PID_output = (kp_slow * P) + (ki_slow * D);
                        Motor(slmotor - PID_output, slmotor + PID_output);
                        delayMicroseconds(50);
                        if ((read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(1) < md_sensorA(1)-50 )  ||
                            (read_sensorA(7) < md_sensorA(7)-50  && read_sensorA(6) < md_sensorA(6)-50  )) 
                            {
                              break;
                            }                          
                  }
                en_for: delay(10);  
              }
          }
        if (splr == 'p') 
          {
            ch_p = 1;
            while (1) 
              {
                Motor(spl, spr);
                delayMicroseconds(50);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) {
                    break;
                }
              }
            delay(10);
            if (endt > 0) 
              {
                Motor(-spl, -spr);
                delay(endt);
                Motor(-1, -1);
                delay(10);
              } 
            else 
              {
                Motor(0, 0);
              }

          } 
        else if (splr == 's') 
          {
            Motor(-spl, -spr);
            delay(endt);
            Motor(0, 0);
            delay(2);
          }
      } 
    else if (nfc == 'c') 
      {
        if(splr == 'p') 
          {
            ch_p = 1;
            while (1) 
              {
                if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                    errors = 0;
                } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                    errors = 10;
                } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                    errors = -10;
                } else {
                  errors = error_A();
                }
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
                Motor(spl - PID_output, spr + PID_output);
                delayMicroseconds(50);
                if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                    break;
                }
              }
            if (endt > 0) 
              {
                Motor(-spl, -spr);
                delay(endt);
                Motor(-1, -1);
                delay(10);
              } 
            else 
              {
                Motor(0, 0);
              }
          }
        else
          {
            if (distance > 0)
              {
                for(int i=spl;i>slmotor;i--)
                      {
                        if (read_sensorA(1) > md_sensorA(1) &&read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6)) {
                            errors = 0;
                        } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                            errors = 10;
                        } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                            errors = -10;
                        } else {
                          errors = error_A();
                        }
                        P = errors;
                        I += errors * 0.00005;
                        D = errors - previous_error;
                        previous_error = errors;
                        PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
                        Motor(i - PID_output, i + PID_output);
                        delayMicroseconds(50);
                        if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                            break;
                        }
                        delay(3);
                      }
              }
            while (1) 
              {
                if (read_sensorA(1) > md_sensorA(1) &&read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6)) {
                    errors = 0;
                } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                    errors = 10;
                } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                    errors = -10;
                } else {
                  errors = error_A();
                }
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
                Motor(slmotor - PID_output, slmotor + PID_output);
                delayMicroseconds(50);
                if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                    break;
                }
            }
          }
        if (splr == 's') {
            Motor(-spl, -spr);
            delay(endt);
            Motor(0, 0);
            delay(2);
        }
    }

    if (splr == 'l') 
      {
        if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0)) {
            while (1) {
                Motor(slmotor, srmotor);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) {
                    delay(delay_f);
                    break;
                }
            }
            Motor(-(slmotor), -(srmotor));
            delay(break_ff);
            for (int i = 0; i <= sensor_f; i++) {
                if(sensor_f >= 2)
                  {
                    do {Motor((flml * power) / 100, (flmr * power) / 100);
                        delayMicroseconds(50);} while (read_sensorA(i+2) < md_sensorA(i+2) - 50);
                  }
                do {
                    Motor((flml * power) / 100, (flmr * power) / 100);
                    delayMicroseconds(50);
                } while (read_sensorA(i) > md_sensorA(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
            delay(nfc == 'n' ? 2 : break_fc);
            int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
            int end_i = start_i ? sensor_f : sensor_f;
            for (int i = start_i; i <= end_i; i++) {
                do {
                    Motor((clml * power) / 100, (clmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fl();
        } else {
            Motor(-((clml * power) / 100), -((clmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      } 
    else if (splr == 'r') 
      {
        if (nfc == 'f') {
            while (1) {
                Motor(slmotor, srmotor);
                delayMicroseconds(50);
                if (read_sensorA(0) >= md_sensorA(0) && read_sensorA(7) >= md_sensorA(7)) {
                    break;
                }
            }
            delay(delay_f);
            Motor(-slmotor, -srmotor);
            delay(break_ff);
            for (int i = 7; i >= sensor_f; i--) {
               if(sensor_f <= 5)
                  {
                    do {Motor((frml * power) / 100, (frmr * power) / 100);
                    delayMicroseconds(50);
                    } while (read_sensorA(i-2) < md_sensorA(i-2) - 50); 
                  }
                do {
                    Motor((frml * power) / 100, (frmr * power) / 100);
                    delayMicroseconds(50);
                } while (read_sensorA(i) > md_sensorA(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
            delay(nfc == 'n' ? 2 : break_fc);
            int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
            for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--) {
                do {
                    Motor((crml * power) / 100, (crmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fr();
        } else {
            Motor(-((crml * power) / 100), -((crmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      }
    

    
    _entN: delay(5);
}



void bline(int spl, int spr, float kp, float distance, char nfc, char splr, int power, String sensor, int endt) {
    _fw = false; // โหมดถอยหลัง
    char sensors[4];
    sensor.toCharArray(sensors, sizeof(sensors));
    int sensor_f = atoi(&sensors[1]); // เช่น "b5" -> sensor_f = 5
    int target_speed = min(spl, spr); // PWM
    const int ramp_step = 2;
    float traveled_distance = 0;
    unsigned long last_time = millis();
    float speed_scale = 1.0; // จากการสอบเทียบก่อนหน้า

    if (kp == 0) {
        I = kp_slow = ki_slow = 0;
    }
    if (kp < 6.5) {
        pid_error = true;
    } else {
        pid_error = false;
    }

    int current_speed = 0; // PWM
    // Soft start
    if(spl == 0)
      {
        goto _line;
      }
 
    if (!ch_p) {
        while (current_speed < target_speed) {
            if(kp <= 0.65)
          {
            if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
            errors = 0;
            } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
             
                errors = 10;
            } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                errors = -10;
            } else {
                errors = error_B();
            }
            
          }
        else 
          {
            errors = error_BB();
          }
            P = errors;
            I += errors * (ramp_delay / 1000.0);
            if (I > 1000) I = 1000;
            if (I < -1000) I = -1000;
            D = errors - previous_error;
            previous_error = errors;
            PID_output = (kp * P) + (0.00005 * I) + (kd_f * D);
            Motor(-(current_speed + PID_output), -(current_speed - PID_output)); // ถอยหลัง
            if (distance > 0) {
                unsigned long current_time = millis();
                float delta_time = (current_time - last_time) / 1000.0;
                traveled_distance += (current_speed * speed_scale) * delta_time;
                last_time = current_time;
                if (traveled_distance >= distance) {
                    Motor(0, 0);
                    return;
                }
            }
            current_speed += ramp_step;
            if (current_speed > target_speed) current_speed = target_speed;
            delay(ramp_delay);
            Serial.println(errors);
        }
    }

    // วิ่งปกติ
    while (1) {
      
        if(kp <= 0.65)
          {
            if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
            errors = 0;
            } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
             
                errors = 10;
            } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                errors = -10;
            } else {
                errors = error_B();
            }
            
          }
        else 
          {
            errors = error_BB();
          }
        
        
        P = errors;
        I += errors * 0.00005;
        if (I > 1000) I = 1000;
        if (I < -1000) I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.00005 * I) + (kd_f * D);
        Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง
        
        if (distance > 0) 
          {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;
            if (traveled_distance >= distance) {
                Motor(0, 0);
                break;
            }
          }
        else
          {
            if(kp >= 6.5 && kp < 1.0)
              {
                 if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)&& read_sensorB(2) < md_sensorB(2)) ||
                    (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) && read_sensorB(5) < md_sensorB(5))) 
                      {
                        break;
                      }
              }
            else if(kp < 6.5)
              {
                if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
                    (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) )) 
                      {
                        break;
                      }
              }
            else {}

          }
        delayMicroseconds(100);
        Serial.println(errors);

        
    }
    
    _line:
    ch_p = 0;

    if (nfc == 'n') {
        if (splr == 'p') {
            ch_p = 1;
            while (1) {
                if(kp <= 0.65)
                  {
                    if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                    errors = 0;
                    } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                    
                        errors = 10;
                    } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                        errors = -10;
                    } else {
                        errors = error_B();
                    }
                    
                  }
                else 
                  {
                    errors = error_BB();
                  }
                P = errors;
                I += errors * 0.00005;
                if (I > 1000) I = 1000;
                if (I < -1000) I = -1000;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
                delayMicroseconds(50);
                if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1) ) ||
                    (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) )) {
                    break;
                }
            }
        } else if (splr == 's') {
            if (endt > 0) {
                Motor(spl, spr); // เบรก
                delay(endt);
                Motor(-1, -1);
                delay(10);
            }
            goto _entN;
        }
    } else if (nfc == 'f') {
        if (distance > 0 || spl == 0) {
            while (1) {
                  if(kp <= 0.65)
                    {
                      if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                      errors = 0;
                      } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                          errors = 10;
                      } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                          errors = -10;
                      } else {
                          errors = error_B();
                      }
                    }
                  else
                    {
                      errors = error_B();
                    }
                P = errors;
                I += errors * 0.00005;
                if (I > 1000) I = 1000;
                if (I < -1000) I = -1000;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
                delayMicroseconds(50);
                if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1) ) ||
                    (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) )) {
                    break;
                }
            }
        }
        if (splr == 'p') {
            ch_p = 1;
            while (1) {
                Motor(-spl, -spr); // ถอยหลัง
                delayMicroseconds(50);
                if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7)) {
                    break;
                }
            }
            delay(10);
        } else if (splr == 's') {
            Motor(spl, spr); // เบรก
            delay(endt);
            Motor(0, 0);
            delay(2);
        }
    } else if (nfc == 'c') 
        {
            
          if (splr == 'p') 
            {
                ch_p = 1;
                while (1) 
                  {
                    float error_L = map(read_sensorB(3), sensorMin_B[3], sensorMax_B[3], 0, 20);
                    float error_R = map(read_sensorB(4), sensorMin_B[4], sensorMax_B[4], 0, 20);
                    errors = error_L - error_R;
                    P = errors;
                    I += errors * 0.00005;
                    if (I > 1000) I = 1000;
                    if (I < -1000) I = -1000;
                    D = errors - previous_error;
                    previous_error = errors;
                    PID_output = (kp_slow * P) + (ki_slow * D);
                    Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง
                    delayMicroseconds(50);
                    if (analogRead(46) < md_sensorC(0) || analogRead(47) < md_sensorC(1)) {
                        break;
                    }
                  }
                if (endt > 0) 
                  {
                    Motor(spl, spr);
                    delay(endt);
                    Motor(1, 1);
                    delay(10);
                  } 
                else 
                  {
                    Motor(0, 0);
                  }
            }
          else
            {
              while (1) 
                {
                  if(kp <= 0.65)
                  {
                    if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                    errors = 0;
                    } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                        errors = 10;
                    } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                        errors = -10;
                    } else {
                        errors = error_B();
                    }
                  }
                else
                  {
                    errors = error_B();
                  }
                  P = errors;
                  D = errors - previous_error;
                  previous_error = errors;
                  PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
                  Motor(-(slmotor + PID_output), -(slmotor - PID_output)); // ถอยหลัง
                  delayMicroseconds(50);
                  if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                      break;
                  }
                }

            }
          if (splr == 's') {
              Motor(spl, spr); // เบรก
              delay(endt);
              Motor(0, 0);
              delay(2);
          }
    }

  if (splr == 'l') 
    {
        if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0)) 
          {
            while (1) {
                Motor(-slmotor, -srmotor);
                if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7)) {
                    delay(delay_f);
                    break;
                }
            }
            Motor(slmotor + 10, srmotor + 10);
            delay(break_bf);                             //---------------------------------------------------------------------------------->>>
            for (int i = 0; i <= sensor_f; i++) {
                do {
                    Motor(-((flmr*power)/100), -((flml*power)/100)); 
                    delayMicroseconds(50);
                } while (read_sensorB(i) > md_sensorB(i) - 50);
                delayMicroseconds(50);
            }
          }
        else 
          {
            Motor(nfc == 'n' ? 0 : slmotor, nfc == 'n' ? 0 : srmotor);
            delay(nfc == 'n' ? 2 : break_bc);
            int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
            int end_i = start_i ? sensor_f : sensor_f;
            for (int i = start_i; i <= end_i; i++) {
                do {
                    Motor((clml * power) / 100, (clmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
          }
        if (endt == 0) {
            turn_speed_fl();
        } else {
            Motor(-((clml * power) / 100), -((clmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      } 
    else if (splr == 'r') 
      {
        if (nfc == 'f') {
            while (1) {
                Motor(-slmotor, -srmotor);
                delayMicroseconds(50);
                if (read_sensorB(0) >= md_sensorB(0) && read_sensorB(7) >= md_sensorB(7)) {
                    break;
                }
            }
            delay(delay_f);
            Motor(slmotor, srmotor);
            delay(break_bf);
            for (int i = 7; i >= sensor_f; i--) {
                do {
                    Motor(-((frmr*power)/100), -((frml*power)/100));
                    delayMicroseconds(50);
                } while (read_sensorB(i) > md_sensorB(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : slmotor, nfc == 'n' ? 0 : srmotor);
            delay(nfc == 'n' ? 2 : break_bc);
            int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
            for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--) {
                do {
                    Motor((crml * power) / 100, (crmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fr();
        } else {
            Motor(-((crml * power) / 100), -((crmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
    }

    if (!(nfc == 'n' && splr == 's' && endt == 0)) {
        Motor(-1, -1); // หยุด
        delay(endt / 2);
    }

    _entN: delay(5);
}


void bline(int spl, int spr, float kp, String distance, char nfc, char splr, int power, String sensor, int endt) {
    _fw = false; // โหมดถอยหลัง
    char sensors[4];
    sensor.toCharArray(sensors, sizeof(sensors));
    int sensor_f = atoi(&sensors[1]); // เช่น "b5" -> sensor_f = 5
    int target_speed = min(spl, spr); // PWM
    const int ramp_step = 2;
    float traveled_distance = 0;
    unsigned long last_time = millis();
    float speed_scale = 1.0; // จากการสอบเทียบก่อนหน้า

    if (kp == 0) {
        I = kp_slow = ki_slow = 0;
    }
    if (kp < 6.5) {
        pid_error = true;
    } else {
        pid_error = false;
    }

    int current_speed = 0; // PWM
    // Soft start
    if(spl == 0)
      {
        goto _line;
      }
 
    // วิ่งปกติ
    while (1) {
      
        if(kp <= 0.65)
          {
            if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
            errors = 0;
            } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
             
                errors = 10;
            } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                errors = -10;
            } else {
                errors = error_B();
            }
            
          }
        else 
          {
            errors = error_BB();
          }
        
        
        P = errors;
        I += errors * 0.00005;
        if (I > 1000) I = 1000;
        if (I < -1000) I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.00005 * I) + (kd_f * D);
        Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง
        
        if (distance == "b0") 
          {
            if(read_sensorB(0) < md_sensorB(0))
              {
                break;
              }
          }
        else if (distance == "b1") 
          {
            if(read_sensorB(1) < md_sensorB(1))
              {
                break;
              }
          }
        else if (distance == "b6") 
          {
            if(read_sensorB(6) < md_sensorB(6))
              {
                break;
              }
          }
        else if (distance == "b7") 
          {
            if(read_sensorB(7) < md_sensorB(7))
              {
                break;
              }
          }
        else if (distance == "b07" || distance == "b70") 
          {
            if(read_sensorB(7) < md_sensorB(7) && read_sensorB(0) < md_sensorB(0))
              {
                break;
              }
          }
        delayMicroseconds(100);
        Serial.println(errors);

        
    }
    
    _line:
    ch_p = 0;

    if (nfc == 'n') {
        if (splr == 'p') {
            ch_p = 1;
            while (1) {
                if(kp <= 0.65)
                  {
                    if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                    errors = 0;
                    } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                    
                        errors = 10;
                    } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                        errors = -10;
                    } else {
                        errors = error_B();
                    }
                    
                  }
                else 
                  {
                    errors = error_BB();
                  }
                P = errors;
                I += errors * 0.00005;
                if (I > 1000) I = 1000;
                if (I < -1000) I = -1000;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
                delayMicroseconds(50);
                if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1) ) ||
                    (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) )) {
                    break;
                }
            }
        } else if (splr == 's') {
            if (endt > 0) {
                Motor(spl, spr); // เบรก
                delay(endt);
                Motor(-1, -1);
                delay(10);
            }
            goto _entN;
        }
    } else if (nfc == 'f') {
        if (distance > 0 || spl == 0) {
            while (1) {
                  if(kp <= 0.65)
                    {
                      if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                      errors = 0;
                      } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                          errors = 10;
                      } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                          errors = -10;
                      } else {
                          errors = error_B();
                      }
                    }
                  else
                    {
                      errors = error_B();
                    }
                P = errors;
                I += errors * 0.00005;
                if (I > 1000) I = 1000;
                if (I < -1000) I = -1000;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
                delayMicroseconds(50);
                if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1) ) ||
                    (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) )) {
                    break;
                }
            }
        }
        if (splr == 'p') {
            ch_p = 1;
            while (1) {
                Motor(-spl, -spr); // ถอยหลัง
                delayMicroseconds(50);
                if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7)) {
                    break;
                }
            }
            delay(10);
        } else if (splr == 's') {
            Motor(spl, spr); // เบรก
            delay(endt);
            Motor(0, 0);
            delay(2);
        }
    } else if (nfc == 'c') 
        {
            
          if (splr == 'p') 
            {
                ch_p = 1;
                while (1) 
                  {
                    float error_L = map(read_sensorB(3), sensorMin_B[3], sensorMax_B[3], 0, 20);
                    float error_R = map(read_sensorB(4), sensorMin_B[4], sensorMax_B[4], 0, 20);
                    errors = error_L - error_R;
                    P = errors;
                    I += errors * 0.00005;
                    if (I > 1000) I = 1000;
                    if (I < -1000) I = -1000;
                    D = errors - previous_error;
                    previous_error = errors;
                    PID_output = (kp_slow * P) + (ki_slow * D);
                    Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง
                    delayMicroseconds(50);
                    if (analogRead(46) < md_sensorC(0) || analogRead(47) < md_sensorC(1)) {
                        break;
                    }
                  }
                if (endt > 0) 
                  {
                    Motor(spl, spr);
                    delay(endt);
                    Motor(1, 1);
                    delay(10);
                  } 
                else 
                  {
                    Motor(0, 0);
                  }
            }
          else
            {
              while (1) 
                {
                  if(kp <= 0.65)
                  {
                    if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                    errors = 0;
                    } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                        errors = 10;
                    } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                        errors = -10;
                    } else {
                        errors = error_B();
                    }
                  }
                else
                  {
                    errors = error_B();
                  }
                  P = errors;
                  D = errors - previous_error;
                  previous_error = errors;
                  PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
                  Motor(-(slmotor + PID_output), -(slmotor - PID_output)); // ถอยหลัง
                  delayMicroseconds(50);
                  if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                      break;
                  }
                }

            }
          if (splr == 's') {
              Motor(spl, spr); // เบรก
              delay(endt);
              Motor(0, 0);
              delay(2);
          }
    }

  if (splr == 'l') 
    {
        if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0)) 
          {
            while (1) {
                Motor(-slmotor, -srmotor);
                if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7)) {
                    delay(delay_f);
                    break;
                }
            }
            Motor(slmotor + 10, srmotor + 10);
            delay(break_bf);                             //---------------------------------------------------------------------------------->>>
            for (int i = 0; i <= sensor_f; i++) {
                do {
                    Motor(-((flmr*power)/100), -((flml*power)/100)); 
                    delayMicroseconds(50);
                } while (read_sensorB(i) > md_sensorB(i) - 50);
                delayMicroseconds(50);
            }
          }
        else 
          {
            Motor(nfc == 'n' ? 0 : slmotor, nfc == 'n' ? 0 : srmotor);
            delay(nfc == 'n' ? 2 : break_bc);
            int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
            int end_i = start_i ? sensor_f : sensor_f;
            for (int i = start_i; i <= end_i; i++) {
                do {
                    Motor((clml * power) / 100, (clmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
          }
        if (endt == 0) {
            turn_speed_fl();
        } else {
            Motor(-((clml * power) / 100), -((clmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      } 
    else if (splr == 'r') 
      {
        if (nfc == 'f') {
            while (1) {
                Motor(-slmotor, -srmotor);
                delayMicroseconds(50);
                if (read_sensorB(0) >= md_sensorB(0) && read_sensorB(7) >= md_sensorB(7)) {
                    break;
                }
            }
            delay(delay_f);
            Motor(slmotor, srmotor);
            delay(break_bf);
            for (int i = 7; i >= sensor_f; i--) {
                do {
                    Motor(-((frmr*power)/100), -((frml*power)/100));
                    delayMicroseconds(50);
                } while (read_sensorB(i) > md_sensorB(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : slmotor, nfc == 'n' ? 0 : srmotor);
            delay(nfc == 'n' ? 2 : break_bc);
            int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
            for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--) {
                do {
                    Motor((crml * power) / 100, (crmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fr();
        } else {
            Motor(-((crml * power) / 100), -((crmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
    }

    if (!(nfc == 'n' && splr == 's' && endt == 0)) {
        Motor(-1, -1); // หยุด
        delay(endt / 2);
    }

    _entN: delay(5);
}
///--------------------------------------------------------------------------------------------->>>จบ bline

///-------------------------------------------------------------------->>> หมุนตัวเพื่อวาง
void place_left_out(int speed, int degree, int offset) 
  {
      
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(10);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + degree;
    
        // กำหนดค่าของ PID
        
        float lr_kp  = 2.05;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        float lr_ki  = 0.00001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        float lr_kd = 0.015; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 500;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1 && abs(output) < 5) break;
    
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            lastTime = now;
    
            if (dt > 0) {
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                previous_error = error;
    
                output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
            }
    
            // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
            output = constrain(output, -speed, speed);
    
            // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
            if(degree > 0)
              {
                Motor(output/4, -output);  
                delay(5);
              }
              else
              {
                
                 Motor(output, -(output/4));  
                delay(5);
              }
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            
            Motor(output/4, -output);
            delay(offset);
          }
        else
          {
            Motor(output, -(output/4)); 
            delay(offset);
          }
        Motor(-1, -1);
        delay(10);
  }

void place_left_in(int speed, int degree, int offset) 
  {
      
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(10);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + (-degree);
    
        // กำหนดค่าของ PID
        
        float lr_kp  = 2.05;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        float lr_ki  = 0.00001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        float lr_kd = 0.015; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 500;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1 && abs(output) < 5) break;
    
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            lastTime = now;
    
            if (dt > 0) {
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                previous_error = error;
    
                output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
            }
    
            // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
            output = constrain(output, -speed, speed);
    
            // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
            if(degree > 0)
              {
                Motor(output/4, -output);  
                delay(5);
              }
              else
              {
                
                 Motor(output, -(output/4));  
                delay(5);
              }
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            
            Motor(output/4, -output);
            delay(offset);
          }
        else
          {
            Motor(output, -(output/4)); 
            delay(offset);
          }
        Motor(-1, -1);
        delay(10);
  }

void place_right_in(int speed, int degree, int offset) 
  {
      
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(10);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + degree;
    
        // กำหนดค่าของ PID
        
        float lr_kp  = 2.05;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        float lr_ki  = 0.00001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        float lr_kd = 0.015; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 500;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1 && abs(output) < 5) break;
    
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            lastTime = now;
    
            if (dt > 0) {
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                previous_error = error;
    
                output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
            }
    
            // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
            output = constrain(output, -speed, speed);
    
            // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
            if(degree > 0)
              {
                Motor(output, -(output/4));  
                delay(5);
              }
              else
              {
                Motor(output/4, -output);  
                delay(5);
              }
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            Motor(output, -(output/4)); 
            delay(offset);
          }
        else
          {
            Motor(output/4, -output);
            delay(offset);
          }
        Motor(-1, -1);
        delay(10);
  }

void place_right_out(int speed, int degree, int offset) 
  {
      
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(10);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + (-degree);
    
        // กำหนดค่าของ PID
        
        float lr_kp  = 2.05;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        float lr_ki  = 0.00001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        float lr_kd = 0.015; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 500;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1 && abs(output) < 5) break;
    
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            lastTime = now;
    
            if (dt > 0) {
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                previous_error = error;
    
                output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
            }
    
            // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
            output = constrain(output, -speed, speed);
    
            // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
            if(degree > 0)
              {
                Motor(output, -(output/4));  
                delay(5);
              }
              else
              {
                Motor(output/4, -output);  
                delay(5);
              }
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            Motor(output, -(output/4)); 
            delay(offset);
          }
        else
          {
            Motor(output/4, -output);
            delay(offset);
          }
        Motor(-1, -1);
        delay(10);
  }

void fw_gyro(int spl, int spr, float kp,  float distance, int offset) 
{     
    int target_speed = min(spl, spr); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.5;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

    my_GYRO::resetAngles();
    float yaw_offset = my.gyro('z'); 
    float _integral = 0;
    float _prevErr = 0;
    unsigned long prevT = millis();   

    int maxLeftSpeed = spl;
    int maxRightSpeed = spr; 

    while (1) 
    {
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

        int leftSpeed  = constrain(spl - corr, -100, 100);
        int rightSpeed = constrain(spr + corr, -100, 100);
        Motor(leftSpeed, rightSpeed);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        delayMicroseconds(50);
    }

    // soft stop
    if(offset >0)
      {
        Motor(-15, -15); delay(offset);
        Motor(-1, -1);   delay(10);
        Motor(0, 0);     delay(10);
      }
    else{Motor(0, 0);delay(5);}
    
}

// ===========================
// ฟังก์ชันวิ่งถอยหลัง ใช้ Gyro + PID
// ===========================
void bw_gyro(int spl, int spr, float kp,  float distance, int offset) 
 {     
    int target_speed = min(spl, spr); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.6;  // ใช้ค่าที่คาลิเบรตจาก fw()

    my_GYRO::resetAngles();
    float yaw_offset = my.gyro('z'); 
    float _integral = 0;
    float _prevErr = 0;
    unsigned long prevT = millis();   

    int maxLeftSpeed = spl;
    int maxRightSpeed = spr; 

    while (1) 
    {
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

        float yaw = my.gyro('z') - yaw_offset;
        float err = -yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

        // สปีดถอยหลัง
        int leftSpeed  = constrain(-(spl - corr), -100, 100);
        int rightSpeed = constrain(-(spr + corr), -100, 100);
        Motor(leftSpeed, rightSpeed);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        delayMicroseconds(50);
    }

    if(offset >0)
      {
        Motor(-15, -15); delay(offset);
        Motor(-1, -1);   delay(10);
        Motor(0, 0);     delay(10);
      }
    else{Motor(0, 0);delay(5);}
  }
#endif

