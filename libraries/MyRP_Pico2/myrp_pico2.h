#ifndef _myrp_pico2_
#define _myrp_pico2_

#include <Wire.h>
#include <my_GYRO1600.h>

#include <my_MCP3008s.h>
my_MCP3008s adc;
#include <EncoderLibrarys.h>
EncoderLibrarys encoder(18, 22, 12, 11);
// กำหนดพินควบคุมมอเตอร์
my_GYRO1600 my; // สร้างอ็อบเจ็กต์ด้วยที่อยู่เริ่มต้น (0x69)

#define PWMA 19 // PWM ซ้าย
#define AIN1 20
#define AIN2 21

#define PWMB 6 // PWM ขวา
#define BIN1 8
#define BIN2 7
//___--------------------------------------------->>
#define EEPROM_ADDRESS 0x50
const int numSensors = 8;
const int numSamples = 1000;

int sensorValuesA[numSensors][numSamples];
int sensorMaxA[numSensors];
int sensorMinA[numSensors];
int sensorValuesB[numSensors][numSamples];
int sensorMaxB[numSensors];
int sensorMinB[numSensors];
int sensorValuesC[2][numSamples];
int sensorMaxC[2];
int sensorMinC[2];

float P, I, D, previous_I, previous_error, errors, PID_output, present_position, previous_integral;
int numSensor = 6;
int state_on_Line = 0;
int setpoint;
int _lastPosition;
int sensor_pin_A[] = {1, 2, 3, 4, 5, 6};
int sensor_pin_B[] = {1, 2, 3, 4, 5, 6};
const float I_MAX = 1000.0;  // ขีดจำกัดบนของ integral
const float I_MIN = -1000.0; // ขีดจำกัดล่างของ integral

int numSensor4 = 4;
int sensor_pin_4[] = {2, 3, 4, 5};

//-------------------------------------------------------->>fline

bool pid_error = true;

const int ramp_delay = 6;                                      // ms
int slmotor = 20, srmotor = 20;                                // PWM
int clml = -90, clmr = 90;                                     // เลี้ยวซ้าย center
int crml = 90, crmr = -90;                                     // เลี้ยวขวา center
int flml = -15, flmr = 100;                                    // เลี้ยวซ้าย front
int frml = 100, frmr = -15;                                    // เลี้ยวขวา front
int llmotor = 100, lrmotor = 50, ldelaymotor = 50;             // เลี้ยวซ้าย speed
int rlmotor = 50, rrmotor = 100, rdelaymotor = 50;             // เลี้ยวขวา speed
int break_ff = 5, break_fc = 30, break_bf = 10, break_bc = 20; // การหน่วง
int delay_f = 15;                                              // การหน่วงก่อนเลี้ยว
float kd_f = 0.55, kd_b = 0.025;                               // Kd PID
float kp_slow = 0.1, ki_slow = 0.0001;                         // PID ช้า
float redius_wheel = 3.0;                                      // รัศมีล้อ (cm)
int ch_p = 0;
bool _fw = true;
float new_encoder = 0;
float speed_scale_fw = 1.0; // สมมติ 1 PWM = 0.1 cm/s (ต้องปรับตามจริง)

float speed_scale_bw = 1.05; // สมมติ 1 PWM = 0.1 cm/s (ต้องปรับตามจริง)

String Freq_motor;

//___--------------------------------------------->>
void get_EEP_Program(void);
void read_sensorA_program(void);
int md_sensorC(int sensor);

void resetAngles()
{
  my.resetAngles();
}
void set_Freq(String fr_motor)
{
  Freq_motor = fr_motor;
}
void distance_scale_fw(float scale)
{
  speed_scale_fw = scale;
}
void distance_scale_bw(float scale)
{
  speed_scale_bw = scale;
}
void set_slow_motor(int sl, int sr)
{
  slmotor = sl;
  srmotor = sr;
}
void set_turn_center_l(int ml, int mr)
{
  clml = ml;
  clmr = mr;
}
void set_turn_center_r(int ml, int mr)
{
  crml = ml;
  crmr = mr;
}
void set_turn_front_l(int ml, int mr)
{
  flml = ml;
  flmr = mr;
}
void set_turn_front_r(int ml, int mr)
{
  frml = ml;
  frmr = mr;
}
void set_brake_fc(int ff, int fc)
{
  break_ff = ff;
  break_fc = fc;
}
void set_brake_bc(int ff, int fc)
{
  break_bf = ff;
  break_bc = fc;
}
void set_delay_f(int ff)
{
  delay_f = ff;
}
void set_speed_turn_fl(int inM, int outM, int delayM)
{
  llmotor = inM;
  lrmotor = outM;
  ldelaymotor = delayM;
}
void set_speed_turn_fr(int inM, int outM, int delayM)
{
  rlmotor = inM;
  rrmotor = outM;
  rdelaymotor = delayM;
}

void setup_robot()
{
  Serial.begin(9600);
  Wire.begin();
  analogReadResolution(12);

  // resetAngles();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  // Serial.print(analogRead(26));
  // Serial.print("  ");
  // Serial.print(analogRead(27));
  // Serial.print("     ");
  // Serial.print(md_sensorC(0));
  // Serial.print("  ");
  // Serial.println(md_sensorC(1));
  for (int i = 0; i < 2; i++)
  {
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_BUILTIN, 1);
      delay(50);
      digitalWrite(LED_BUILTIN, 0);
      delay(50);
    }
  }
  get_EEP_Program();
  // read_sensorA_program();
  // read_sensorC_program();

  if (!my.begin())
  {
    // Serial.println("Failed to initialize GYRO160!");
    // while (1);
  }
  resetAngles();
}

/*
   get_maxMinA();
   get_maxMinB();
   get_maxMinC();
   read_eepA();
   read_sensorA_program();
   read_eepB();
   read_sensorB_program();
   read_eepC();
   read_sensorC_program();
*/

uint16_t read_sensorA(int sensor)
{
  adc.begin(14, 15, 16, 13);
  adc.begin(14, 15, 16, 17); // adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
  return adc.readADC(sensor);
}

uint16_t read_sensorB(int sensor)
{
  adc.begin(14, 15, 16, 17);
  adc.begin(14, 15, 16, 13); // adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
  return adc.readADC(sensor);
}
int md_sensorA(int sensor)
{
  return (sensorMaxA[sensor] + sensorMinA[sensor]) / 2;
}

int md_sensorB(int sensor)
{
  return (sensorMaxB[sensor] + sensorMinB[sensor]) / 2;
}
int md_sensorC(int sensor)
{
  return (sensorMaxC[sensor] + sensorMinC[sensor]) / 2;
}

// EEPROM
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  for (int i = 0; i < dataLength; i++)
    Wire.write(data[i]);
  Wire.endTransmission();
  delay(5);
}

void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, dataLength);
  for (int i = 0; i < dataLength; i++)
  {
    if (Wire.available())
      buffer[i] = Wire.read();
  }
}

// ==================== get_maxMinA (เดิมของคุณ ใช้ได้อยู่แล้ว) ====================
void get_maxMinA()
{
  for (int sample = 0; sample < numSamples; sample++)
  {
    for (int sensor = 0; sensor < numSensors; sensor++)
    {
      sensorValuesA[sensor][sample] = read_sensorA(sensor);
      delay(1);
    }
  }
  for (int sensor = 0; sensor < numSensors; sensor++)
  {
    sensorMaxA[sensor] = sensorValuesA[sensor][0];
    sensorMinA[sensor] = sensorValuesA[sensor][0];
    for (int sample = 1; sample < numSamples; sample++)
    {
      int value = sensorValuesA[sensor][sample];
      if (value > sensorMaxA[sensor])
        sensorMaxA[sensor] = value;
      if (value < sensorMinA[sensor])
        sensorMinA[sensor] = value;
    }
  }
  byte buffer[16];
  for (int i = 0; i < numSensors; i++)
  {
    buffer[i * 2] = highByte(sensorMaxA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 0, buffer, 16);
  for (int i = 0; i < numSensors; i++)
  {
    buffer[i * 2] = highByte(sensorMinA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 16, buffer, 16);

  tone(9, 3000, 100);
  delay(200);
  tone(9, 3000, 200);
  delay(200);
}

// ==================== get_maxMinB (เหมือน A) ====================
void get_maxMinB()
{
  // เหมือน get_maxMinA แต่ใช้ sensorMaxB/sensorMinB และ address 32, 48
  for (int sample = 0; sample < numSamples; sample++)
  {
    for (int sensor = 0; sensor < numSensors; sensor++)
    {
      sensorValuesB[sensor][sample] = read_sensorB(sensor);
      delay(1);
    }
  }
  for (int sensor = 0; sensor < numSensors; sensor++)
  {
    sensorMaxB[sensor] = sensorValuesB[sensor][0];
    sensorMinB[sensor] = sensorValuesB[sensor][0];
    for (int sample = 1; sample < numSamples; sample++)
    {
      int value = sensorValuesB[sensor][sample];
      if (value > sensorMaxB[sensor])
        sensorMaxB[sensor] = value;
      if (value < sensorMinB[sensor])
        sensorMinB[sensor] = value;
    }
  }
  byte buffer[16];
  for (int i = 0; i < numSensors; i++)
  {
    buffer[i * 2] = highByte(sensorMaxB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 32, buffer, 16);
  for (int i = 0; i < numSensors; i++)
  {
    buffer[i * 2] = highByte(sensorMinB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 48, buffer, 16);

  tone(9, 3000, 100);
  delay(200);
  tone(9, 3000, 200);
  delay(200);
}

// ==================== get_maxMinC (ของคุณเดิม + constrain) ====================
void get_maxMinC()
{
  const int PIN_C0 = A0;
  const int PIN_C1 = A1;

  for (int sample = 0; sample < numSamples; sample++)
  {
    sensorValuesC[0][sample] = analogRead(PIN_C0);
    sensorValuesC[1][sample] = analogRead(PIN_C1);
    delay(5);
  }

  for (int sensor = 0; sensor < 2; sensor++)
  {
    sensorMaxC[sensor] = 0;
    sensorMinC[sensor] = 4095;
    for (int sample = 0; sample < numSamples; sample++)
    {
      uint16_t value = sensorValuesC[sensor][sample];
      if (value > sensorMaxC[sensor])
        sensorMaxC[sensor] = value;
      if (value < sensorMinC[sensor])
        sensorMinC[sensor] = value;
    }
    sensorMaxC[sensor] = constrain(sensorMaxC[sensor], 0, 4000);
    sensorMinC[sensor] = constrain(sensorMinC[sensor], 0, 4000);
  }

  uint8_t buffer[4];
  for (int i = 0; i < 2; i++)
  {
    buffer[i * 2] = highByte(sensorMaxC[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 64, buffer, 4);

  for (int i = 0; i < 2; i++)
  {
    buffer[i * 2] = highByte(sensorMinC[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 68, buffer, 4);

  tone(9, 3000, 150);
  delay(200);
  tone(9, 3000, 200);
  delay(250);
}

// ==================== read_eepA (แก้ให้โหลดเข้า sensorMaxA/sensorMinA จริง) ====================
void read_eepA()
{
  byte readBuffer[16];
  int readMaxA[numSensors], readMinA[numSensors];
  readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
  for (int i = 0; i < numSensors; i++)
  {
    readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
  for (int i = 0; i < numSensors; i++)
  {
    readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor A Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++)
  {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(readMaxA[sensor]);
    Serial.print(", Min: ");
    Serial.println(readMinA[sensor]);
    sensorMaxA[sensor] = readMaxA[sensor]; // โหลดเข้า sensorMaxA จริง
    sensorMinA[sensor] = readMinA[sensor]; // โหลดเข้า sensorMinA จริง
  }
}

// ==================== read_eepB (เหมือนกัน) ====================
void read_eepB()
{
  byte readBuffer[16];
  int readMaxB[numSensors], readMinB[numSensors];
  readEEPROM(EEPROM_ADDRESS, 32, readBuffer, 16);
  for (int i = 0; i < numSensors; i++)
  {
    readMaxB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 48, readBuffer, 16);
  for (int i = 0; i < numSensors; i++)
  {
    readMinB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor B Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++)
  {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(readMaxB[sensor]);
    Serial.print(", Min: ");
    Serial.println(readMinB[sensor]);
    sensorMaxB[sensor] = readMaxB[sensor]; // โหลดเข้า sensorMaxB จริง
    sensorMinB[sensor] = readMinB[sensor]; // โหลดเข้า sensorMinB จริง
  }
}

// ==================== read_eepC (แก้แล้ว) ====================
void read_eepC()
{
  byte readBuffer[4];
  int readMaxC[2], readMinC[2];
  readEEPROM(EEPROM_ADDRESS, 64, readBuffer, 4);
  for (int i = 0; i < 2; i++)
  {
    readMaxC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 68, readBuffer, 4);
  for (int i = 0; i < 2; i++)
  {
    readMinC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor C Values read from EEPROM:");
  Serial.print("Sensor C0 (Pin 26) => Max: ");
  Serial.print(readMaxC[0]);
  Serial.print(", Min: ");
  Serial.println(readMinC[0]);
  Serial.print("Sensor C1 (Pin 27) => Max: ");
  Serial.print(readMaxC[1]);
  Serial.print(", Min: ");
  Serial.println(readMinC[1]);

  sensorMaxC[0] = readMaxC[0];
  sensorMinC[0] = readMinC[0]; // โหลดเข้า sensorMaxC/sensorMinC จริง
  sensorMaxC[1] = readMaxC[1];
  sensorMinC[1] = readMinC[1];
}

// ==================== แสดงผล (ใช้ตัวแปรเดียวกัน) ====================
void read_sensorA_program()
{
  Serial.println("Sensor MAX A Values read from program:");
  for (int sensor = 0; sensor < numSensors; sensor++)
  {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(sensorMaxA[sensor]);
    Serial.print(", Min: ");
    Serial.println(sensorMinA[sensor]);
  }
}

void read_sensorB_program()
{
  Serial.println("Sensor MAX B Values read from program:");
  for (int sensor = 0; sensor < numSensors; sensor++)
  {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(sensorMaxB[sensor]);
    Serial.print(", Min: ");
    Serial.println(sensorMinB[sensor]);
  }
}

void read_sensorC_program()
{
  Serial.println("Sensor C Values read from program:");
  Serial.print("Sensor C0 (Pin 26) => Max: ");
  Serial.print(sensorMaxC[0]);
  Serial.print(", Min: ");
  Serial.println(sensorMinC[0]);
  Serial.print("Sensor C1 (Pin 27) => Max: ");
  Serial.print(sensorMaxC[1]);
  Serial.print(", Min: ");
  Serial.println(sensorMinC[1]);
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

//-------------------------------------------------------------------------------------->>ควบคุมมอเตอร์
void tone(int t, int d)
{
  tone(9, t, d);
  delay(d + 50);
}

void blinkLED(int count)
{
  for (int i = 0; i < count; i++)
  {
    digitalWrite(LED_BUILTIN, 1);
    delay(100);
    digitalWrite(LED_BUILTIN, 0);
    delay(100);
  }
}

void waitButton()
{
  while (digitalRead(3) == 1)
    ;         // รอจนกดปุ่ม
  delay(200); // debounce
  while (digitalRead(3) == 0)
    ; // รอจนปล่อยปุ่ม
  delay(200);
}
#define MCP3421_ADDR 0x68 // I2C Address when A0 = GND
int ADC_i2c()
{
  long ADC01 = 0;
  int adc_01;

  // อ่านจาก Wire (I2C0)
  Wire.requestFrom(MCP3421_ADDR, 4);
  if (Wire.available() == 4)
  {
    byte b1 = Wire.read();
    byte b2 = Wire.read();
    byte b3 = Wire.read();
    byte cfg = Wire.read();

    ADC01 = ((long)b1 << 16) | ((long)b2 << 8) | b3;
    if (b1 & 0x80)
      ADC01 |= 0xFF000000; // sign-extend
  }
  adc_01 = map(ADC01, 524048, 282, 4000, 0);
  // Serial.print("ADC01: "); Serial.print(adc_01);
  // delay(10);
  return adc_01;
}

void sw()
{
  tone(9, 3000, 100);
  delay(200);
  tone(9, 3000, 200);
  delay(200);

  int buttonState;
  unsigned long pressStartTime = 0;
  bool isPressed = false;

  if (ADC_i2c() < 4500 && ADC_i2c() > 3500)
  {
    while (1)
    {
      if (digitalRead(3) == 0)
      {
        digitalWrite(LED_BUILTIN, 1);
        tone(3000, 200);
        get_maxMinA();
        blinkLED(5);
      }

      // ปิด sensor B

      if (ADC_i2c() < 2500)
      {
        tone(3000, 100);
        tone(3000, 200);
        tone(3000, 200);
        get_maxMinB();
        blinkLED(5);
        tone(9, 3000, 100);
        delay(200);
        tone(9, 3000, 200);
        delay(200);
      }

      Serial.print("From A ");
      for (int i = 0; i < 8; i++)
      {
        Serial.print(read_sensorA(i));
        Serial.print(" ");
      }
      Serial.print("   ");

      // Comment การแสดงค่า sensor B

      Serial.print("From B ");
      for (int i = 0; i < 8; i++)
      {
        Serial.print(read_sensorB(i));
        Serial.print(" ");
      }

      // แสดงค่า sensor C แทน
      Serial.print("From C ");
      Serial.print("C0: ");
      Serial.print(analogRead(A0));
      Serial.print(" C1: ");
      Serial.print(analogRead(A1));

      Serial.println("  ");
      delay(10);
      Serial.println(" ");
      delay(200);

      if (digitalRead(2) == LOW)
      {
        if (!isPressed)
        {
          pressStartTime = millis();
          isPressed = true;
        }
        else
        {
          unsigned long pressDuration = millis() - pressStartTime;
          if (pressDuration >= 3000)
          {
            blinkLED(5);
            tone(3000, 100);
            tone(3000, 200);
            get_maxMinC();
            blinkLED(5);
            tone(9, 3000, 100);
            delay(200);
            tone(9, 3000, 200);
            delay(200);
            delay(50);
            while (digitalRead(2) == LOW)
              ;
            delay(200);
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
  }
  else
  {
    while (1)
    {
      if (digitalRead(3) == 0)
      {
        digitalWrite(LED_BUILTIN, 1);
        tone(3000, 200);
        get_maxMinA();
        blinkLED(5);

        while (1)
        {
          waitButton();
          tone(3000, 100);
          tone(3000, 200);
          get_maxMinC();
          blinkLED(5);
          tone(9, 3000, 100);
          delay(200);
          tone(9, 3000, 200);
          delay(200);

          // Comment sensor B ในโหมดนี้ด้วย

          waitButton();
          tone(3000, 100);
          tone(3000, 200);
          tone(3000, 200);
          get_maxMinB();
          blinkLED(5);
          tone(9, 3000, 100);
          delay(200);
          tone(9, 3000, 200);
          delay(200);

          break;
        }
      }

      if (digitalRead(2) == 0)
      {
        break;
      }

      Serial.print("From A ");
      for (int i = 0; i < 8; i++)
      {
        Serial.print(read_sensorA(i));
        Serial.print(" ");
      }
      Serial.print("   ");

      // Comment sensor B

      Serial.print("From B ");
      for (int i = 0; i < 8; i++)
      {
        Serial.print(read_sensorB(i));
        Serial.print(" ");
      }

      // แสดง sensor C แทน
      Serial.print("From C ");
      Serial.print("C0: ");
      Serial.print(analogRead(A0));
      Serial.print(" C1: ");
      Serial.print(analogRead(A1));

      Serial.println("  ");
    }
  }

  get_EEP_Program();
  tone(9, 3000, 400);
  delay(500);
}
///----------------------------------------------------------------------------------->>>>
///---------------------------------------------------------------------------------->>>>
int sl, sr; // ตัวแปรความเร็วสำหรับมอเตอร์ซ้ายและขวา

// ฟังก์ชันควบคุมมอเตอร์ซ้าย/ขวา
void Motor(int pwmL, int pwmR)
{
  // ตั้งความละเอียด PWM เป็น 12 บิต (0–4095)
  analogWriteResolution(12);
  // ตั้งความถี่ PWM เป็น 20kHz (ลดเสียงรบกวนมอเตอร์)
  if (Freq_motor == "DC_Motors")
  {
    analogWriteFreq(1000);
  }
  else
  {
    analogWriteFreq(20000);
  }
  delayMicroseconds(50);

  // แปลงค่าจาก -100..100 ให้เป็น 0..4095
  int pwmValueL = map(abs(pwmL), 0, 100, 0, 4095);
  int pwmValueR = map(abs(pwmR), 0, 100, 0, 4095);

  // มอเตอร์ซ้าย
  if (pwmL > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (pwmL < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    pwmValueL = 0;
  }

  // มอเตอร์ขวา
  if (pwmR > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if (pwmR < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    pwmValueR = 0;
  }

  // ส่งค่า PWM (0–4095)
  analogWrite(PWMA, pwmValueL);
  analogWrite(PWMB, pwmValueR);
}

//-------------------------------------------------------------------------------------->>ควบคุมมอเตอร์

//-------------------------------------------------------------------------------------->>ควบคุมserv

#include <Servo.h>

// กำหนดขาเซอร์โว
#define servo0 0
#define servo1 1
#define servo10 10
#define servo28 28

// สร้างออบเจ็กต์เซอร์โว
Servo servo_0;
Servo servo_1;
Servo servo_10;
Servo servo_28;

// ตัวแปรสำหรับเก็บค่า trim และมุมก่อนหน้า
int servo_tim10 = 0;
int servo_tim0 = 0;
int servo_tim1 = 0;
int servo_tim28 = 0;
int num_steps = 20;
float s10_before_deg = 120;
float s0_before_deg = 120;
float s1_before_deg = 50;
float s28_before_deg = 0;

// ฟังก์ชันตั้งค่า trim
void s0_trim(int _s0)
{
  servo_tim0 = _s0;
}

void s1_trim(int _s1)
{
  servo_tim1 = _s1; // แก้ไขจาก servo_tim10 เป็น servo_tim1
}

void s10_trim(int _s10)
{
  servo_tim10 = _s10; // แก้ไขจาก servo_tim10 เป็น servo_tim28
}

void s28_trim(int _s28)
{
  servo_tim28 = _s28; // แก้ไขจาก servo_tim10 เป็น servo_tim28
}

// ฟังก์ชันควบคุมเซอร์โว
void servo(int servo, int angle)
{
  if (servo == 10)
  {
    servo_10.attach(servo10, 500, 2500);
    servo_10.write(angle);
  }
  else if (servo == 0)
  {
    servo_0.attach(servo0, 500, 2500);
    servo_0.write(180 - angle);
    // servo_0.write(angle);
  }
  else if (servo == 1)
  {
    servo_1.attach(servo1, 500, 2500);
    servo_1.write(angle);
  }
  else if (servo == 28)
  {
    servo_28.attach(servo28, 500, 2500);
    servo_28.write(angle + servo_tim28);
  }
}

// ฟังก์ชันควบคุมเซอร์โว 10 และ 1 พร้อมกัน
void arm_left_right(float sl, float sr, int sp)
{
  float servo10_step = (sl - s10_before_deg) / num_steps;
  float servo1_step = (sr - s1_before_deg) / num_steps;

  for (int i = 0; i < num_steps; i++)
  {
    float servo10_pos = s10_before_deg + (i * servo10_step); // แก้ไขจาก servo1_step เป็น servo10_step
    float servo1_pos = s1_before_deg + (i * servo1_step);
    servo(10, servo10_pos);
    servo(1, servo1_pos);
    delay(sp);
    Serial.print("S10: ");
    Serial.println(servo10_pos);
    Serial.print("S1: ");
    Serial.println(servo1_pos);
  }
  s10_before_deg = sl;
  s1_before_deg = sr;
}

void arm_up_down(float sl, int sp)
{
  float servo0_step = (sl - s0_before_deg) / num_steps;

  for (int i = 0; i < num_steps; i++)
  {
    float servo0_pos = s0_before_deg + (i * servo0_step); // แก้ไขจาก servo1_step เป็น servo10_step6
    servo(0, servo0_pos);
    delay(sp);
    Serial.print("S0: ");
    Serial.println(servo0_pos);
  }
  s0_before_deg = sl;
}
//-------------------------------------------------------------------------------------->>ควบคุมservo

//-------------------------------------------------------------------------------------->>ควบคุม PID

/*
int position_A(int *sensor_pins, int *sensor_Minvalues, int *sensor_Maxvalues, int num_sensors, int *last_position) {
    bool onLine = false;
    long avg = 0;
    long sum = 0;

    for (uint8_t i = 0; i < num_sensors; i++) {
        // อ่านค่าเซนเซอร์และแปลงด้วย map
        long value = map(read_sensorA(sensor_pins[i]), sensor_Minvalues[i], sensor_Maxvalues[i], 1000, 0);

        if (value > 200) {
            onLine = true;
        }
        if (value > 50) {
            avg += (long)value * (i * 1000);
            sum += value;
        }
        delayMicroseconds(50);
    }

    if (!onLine) {
        // ถ้าไม่เจอเส้น ใช้ last_position เพื่อตัดสินใจ
        if (*last_position < (num_sensors - 1) * 1000 / 2) {
            return 0;
        } else {
            return (num_sensors - 1) * 1000; // คืนค่าสูงสุดตามจำนวนเซนเซอร์
        }
    }

    // อัปเดต last_position ผ่านพอยน์เตอร์
    *last_position = (sum == 0) ? *last_position : avg / sum;

    return *last_position;
}
*/
int position_A()
{
  int Minsensor_values_A[] = {sensorMinA[1], sensorMinA[2], sensorMinA[3], sensorMinA[4], sensorMinA[5], sensorMinA[6]}; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
  int Maxsensor_values_A[] = {sensorMaxA[1], sensorMaxA[2], sensorMaxA[3], sensorMaxA[4], sensorMaxA[5], sensorMaxA[6]}; // ค่าที่อ่านได้มากสุด สีขาว
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < 6; i++)
  {
    long value = map(read_sensorA(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0); // จากนั้นก็เก็บเข้าไปยังตัวแป value

    if (value > 200)
    {
      onLine = true;
    }
    if (value > 50)
    {
      avg += (long)value * (i * 1000);
      sum += value;
    }
    // delayMicroseconds(50);
  }
  if (!onLine) // เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
  {
    if (_lastPosition < (numSensor - 1) * 1000 / 2) // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
    {
      return 0;
    }
    else // แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
    {
      return 5000;
    }
  }
  _lastPosition = avg / sum; // นำมาหาค่าเฉลี่ย

  return _lastPosition; // ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
}
int position_A_none()
{
  int Minsensor_values_A[] = {sensorMinA[1], sensorMinA[2], sensorMinA[3], sensorMinA[4], sensorMinA[5], sensorMinA[6]}; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
  int Maxsensor_values_A[] = {sensorMaxA[1], sensorMaxA[2], sensorMaxA[3], sensorMaxA[4], sensorMaxA[5], sensorMaxA[6]}; // ค่าที่อ่านได้มากสุด สีขาว
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < 6; i++)
  {
    long value = map(read_sensorA(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0); // จากนั้นก็เก็บเข้าไปยังตัวแป value

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
  if (!onLine) // เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
  {
    if (_lastPosition < (numSensor - 1) * 1000 / 2) // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
    {
      return 2500;
    }
    else // แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
    {
      return 2500;
    }
  }
  _lastPosition = avg / sum; // นำมาหาค่าเฉลี่ย

  return _lastPosition; // ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
}

float error_A()
{
  if (pid_error == true)
  {
    present_position = position_A_none() / ((numSensor - 1) * 10);
    setpoint = 50.0;
    errors = setpoint - present_position;
    return errors;
  }
  else
  {
    present_position = position_A() / ((numSensor - 1) * 10);
    setpoint = 50.0;
    errors = setpoint - present_position;
    return errors;
  }
}
float error_AA()
{
  present_position = position_A() / ((numSensor - 1) * 10);
  setpoint = 50.0;
  errors = setpoint - present_position;
  return errors;
}
float error_AN()
{
  present_position = position_A_none() / ((numSensor - 1) * 10);
  setpoint = 50.0;
  errors = setpoint - present_position;
  return errors;
}
int position_B()
{
  int Minsensor_values_A[] = {sensorMinB[1], sensorMinB[2], sensorMinB[3], sensorMinB[4], sensorMinB[5], sensorMinB[6]}; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
  int Maxsensor_values_A[] = {sensorMaxB[1], sensorMaxB[2], sensorMaxB[3], sensorMaxB[4], sensorMaxB[5], sensorMaxB[6]}; // ค่าที่อ่านได้มากสุด สีขาว
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < 6; i++)
  {
    long value = map(read_sensorB(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0); // จากนั้นก็เก็บเข้าไปยังตัวแป value

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
  if (!onLine) // เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
  {
    if (_lastPosition < (numSensor - 1) * 1000 / 2) // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
    {
      return 0;
    }
    else // แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
    {
      return 5000;
    }
  }
  _lastPosition = avg / sum; // นำมาหาค่าเฉลี่ย

  return _lastPosition; // ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
}
int position_B_none()
{
  int Minsensor_values_A[] = {sensorMinB[1], sensorMinB[2], sensorMinB[3], sensorMinB[4], sensorMinB[5], sensorMinB[6]}; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
  int Maxsensor_values_A[] = {sensorMaxB[1], sensorMaxB[2], sensorMaxB[3], sensorMaxB[4], sensorMaxB[5], sensorMaxB[6]}; // ค่าที่อ่านได้มากสุด สีขาว
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < 6; i++)
  {
    long value = map(read_sensorB(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0); // จากนั้นก็เก็บเข้าไปยังตัวแป value

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
  if (!onLine) // เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
  {
    if (_lastPosition < (numSensor - 1) * 1000 / 2) // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
    {
      return 2500;
    }
    else // แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
    {
      return 2500;
    }
  }
  _lastPosition = avg / sum; // นำมาหาค่าเฉลี่ย

  return _lastPosition; // ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
}

float error_B()
{
  if (pid_error == true)
  {
    present_position = position_B_none() / ((numSensor - 1) * 10);
    setpoint = 50.0;
    errors = setpoint - present_position;
    return errors;
  }
  else
  {
    present_position = position_B() / ((numSensor - 1) * 10);
    setpoint = 50.0;
    errors = setpoint - present_position;
    return errors;
  }
}

float error_BB()
{

  present_position = position_B() / ((numSensor - 1) * 10);
  setpoint = 50.0;
  errors = setpoint - present_position;
  return errors;
}

float error_BN()
{

  present_position = position_B_none() / ((numSensor - 1) * 10);
  setpoint = 50.0;
  errors = setpoint - present_position;
  return errors;
}

//------------------------------------------------------------------------------------------------------------------------>>>คำสั่งเดินตามเส้น
void fw(int sl, int sr, float kp)
{
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
void fws(int sl, int sr, float kp)
{
  int current_speed = 0;                  // เริ่มจากความเร็ว 0
  int target_speed = (sl < sr) ? sl : sr; // เลือกความเร็วต่ำสุดเป็นเป้าหมายเพื่อสมดุล
  const int ramp_step = 2;                // ความเร็วเพิ่มครั้งละ 2 (ปรับได้ตามความนุ่มนวลที่ต้องการ)
  const int ramp_delay = 10;              // หน่วงระหว่างเพิ่มความเร็ว (ms)

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
    if (current_speed > target_speed)
      current_speed = target_speed;

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
  int current_speed = 0;                  // เริ่มจากความเร็ว 0 (หน่วย: cm/s)
  int target_speed = (sl < sr) ? sl : sr; // เลือกความต่ำสุดเพื่อสมดุล
  const int ramp_step = 2;                // ความเร็วเพิ่มครั้งละ 2
  const int ramp_delay = 10;              // หน่วงระหว่างเพิ่มความเร็ว (ms)
  float errors, P, D, previous_error = 0, PID_output;
  float I = 0;                        // ตัวแปร Integral
  float traveled_distance = 0;        // ระยะทาง (หน่วย: เซนติเมตร)
  unsigned long last_time = millis(); // เก็บเวลาเริ่มต้น

  // Soft start เพิ่มความเร็วจนถึงเป้าหมาย
  while (current_speed < target_speed)
  {
    errors = error_A();
    P = errors;
    I += errors * (ramp_delay / 1000.0); // อัพเดท Integral
    D = errors - previous_error;
    previous_error = errors;

    PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
    Motor(current_speed - PID_output, current_speed + PID_output);

    // คำนวณระยะทางถ้ากำหนด distance > 0
    if (distance > 0)
    {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0; // หน่วยวินาที
      traveled_distance += current_speed * delta_time;        // ความเร็ว cm/s * เวลา วินาที = ระยะทาง cm
      last_time = current_time;

      // ตรวจสอบระยะทาง
      if (traveled_distance >= distance)
      {
        Motor(0, 0); // หยุดมอเตอร์
        return;      // ออกจากฟังก์ชัน
      }
    }

    current_speed += ramp_step;
    if (current_speed > target_speed)
      current_speed = target_speed;

    delay(ramp_delay);
  }

  // เดินหน้าปกติ
  while (1)
  {
    errors = error_A();
    P = errors;
    I += errors * 0.00005; // อัพเดท Integral (สำหรับ 50us)
    D = errors - previous_error;
    previous_error = errors;

    PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
    Motor(sl - PID_output, sr + PID_output);

    // คำนวณระยะทางถ้ากำหนด distance > 0
    if (distance > 0)
    {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0; // หน่วยวินาที
      traveled_distance += target_speed * delta_time;         // ความเร็ว cm/s * เวลา วินาที = ระยะทาง cm
      last_time = current_time;

      // ตรวจสอบระยะทาง
      if (traveled_distance >= distance)
      {
        Motor(0, 0); // หยุดมอเตอร์
        return;      // ออกจากฟังก์ชัน
      }
    }

    delayMicroseconds(50);
  }

  ///////////////////////////////-------------------------------------------------------->>>>>>>>  PID
}

void wheel_redius(float rediuss)
{
  redius_wheel = rediuss;
}

float wheel_distance()
{
  return 2 * 3.14159 * redius_wheel / 10; // เซนติเมตร
}

void test_distance(int distance1)
{
  new_encoder = 440 * distance1 / wheel_distance();
  Serial.println(new_encoder);
  encoder.resetEncoders();
  do
  {
    Motor(30, 30);
  } while (encoder.Poss_R() < new_encoder);
  Motor(-30, -30);
  delay(10);
  Motor(1, 1);
}

void to_slow_motor(int sl, int sr)
{
  slmotor = sl;
  srmotor = sr;
}

void to_turn_center_l(int ml, int mr)
{
  clml = ml;
  clmr = mr;
}

void to_turn_center_r(int ml, int mr)
{
  crml = ml;
  crmr = mr;
}

void to_turn_front_l(int ml, int mr)
{
  flml = ml;
  flmr = mr;
}

void to_turn_front_r(int ml, int mr)
{
  frml = ml;
  frmr = mr;
}

void to_brake_fc(int ff, int fc)
{
  break_ff = ff;
  break_fc = fc;
}

void to_brake_bc(int bf, int bc)
{
  break_bf = bf;
  break_bc = bc;
}

void to_delay_f(int ff)
{
  delay_f = ff;
}

void to_speed_turn_fl(int inM, int outM, int delayM)
{
  llmotor = inM;
  lrmotor = outM;
  ldelaymotor = delayM;
}

void to_speed_turn_fr(int inM, int outM, int delayM)
{
  rlmotor = inM;
  rrmotor = outM;
  rdelaymotor = delayM;
}

void kd_fw(float kd)
{
  kd_f = kd;
}

void kd_bw(float kd)
{
  kd_b = kd;
}

void kp_sl(float kp_sl, float ki_sl)
{
  kp_slow = kp_sl;
  ki_slow = ki_sl;
}

void turn_speed_fl()
{
  for (int t = 0; t < ldelaymotor; t++)
  {
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

void turn_speed_fr()
{
  for (int t = 0; t < rdelaymotor; t++)
  {
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

void bturn_speed_fl()
{
  for (int t = 0; t < ldelaymotor; t++)
  {
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

void bturn_speed_fr()
{
  for (int t = 0; t < rdelaymotor; t++)
  {
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

void fline(int spl, int spr, float kp, float distance, char nfc, char splr, int power, String sensor, int endt)
{
  _fw = true;
  char sensors[4];
  sensor.toCharArray(sensors, sizeof(sensors));
  int sensor_f = atoi(&sensors[1]);
  int target_speed = min(spl, spr); // PWM
  const int ramp_step = 3;
  float traveled_distance = 0;
  unsigned long last_time = millis();

  float I_max = 1000.0, I_min = -1000.0;         // ขีดจำกัด Integral
  float D_max = 50.0, D_min = -50.0;             // ขีดจำกัด Derivative
  float alpha = 0.1;                             // ค่ากรอง
  static float filtered_D = 0.0;                 // คงสถานะ
  float output_max = 100.0, output_min = -100.0; // ขีดจำกัด Output

  if (kp == 0)
  {
    I = kp_slow = ki_slow = 0;
  }
  if (kp < 6.5)
  {
    pid_error = true;
  }
  else
  {
    pid_error = false;
  }

  int current_speed = 0; // PWM
  if (spl == 0)
  {
    goto _line;
  }
  // Soft start
  if (!ch_p)
  {
    while (current_speed < target_speed)
    {
      if (kp <= 0.45)
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
          errors = error_AA();
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
      delayMicroseconds(50);

      if (distance > 0)
      {
        unsigned long current_time = millis();
        float delta_time = (current_time - last_time) / 1000.0;
        traveled_distance += (current_speed * speed_scale_fw) * delta_time;
        last_time = current_time;
        if (traveled_distance >= distance)
        {
          Motor(0, 0);
          return;
        }
      }
      current_speed += ramp_step;
      if (current_speed > target_speed)
      {
        current_speed = target_speed;
        delay(ramp_delay);
      }
      // Serial.println(errors);
    }
  }

  // วิ่งปกติ

  while (1)
  {
    if (kp <= 0.45)
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
        errors = error_AA();
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
    delayMicroseconds(50);
    if (distance > 0)
    {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 500.0;
      traveled_distance += (target_speed * speed_scale_fw) * delta_time;
      last_time = current_time;
      if (traveled_distance >= distance)
      {
        for (int i = spl; i > slmotor; i--)
        {
          if (kp <= 0.45)
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
              errors = error_AA();
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
          PID_output = ((kp / 2) * P) + (0.000001 * I) + (kd_f * D);
          Motor(i - PID_output, i + PID_output);
          delayMicroseconds(50);
        }
        Motor(0, 0);

        break;
      }
    }
    else
    {
      if (kp >= 6.5 && kp < 1.0)
      {
        if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) && read_sensorA(2) < md_sensorA(2)) ||
            (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) && read_sensorA(5) < md_sensorA(5)))
        {
          break;
        }
      }
      else if (kp < 6.5)
      {
        if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
            (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6)))
        {
          break;
        }
      }
      else
      {
      }
    }
  }
_line:
  ch_p = 0;

  if (nfc == 'n')
  {
    if (splr == 'p')
    {
      ch_p = 1;
      if (spl >= 1)
      {
        while (1)
        {
          if (kp <= 0.65)
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
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
              (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6)))
          {
            break;
          }
        }
      }
      else
      {
        while (1)
        {
          if (kp <= 0.65)
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
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
              (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6)))
          {
            break;
          }
        }
      }
    }
    else if (splr == 's')
    {
      if (endt > 0)
      {
        Motor(-spl, -spr);
        delay(endt);
        Motor(-1, -1);
        delay(10);
      }
      else
      {
        for (int i = spl; i > 15; i--)
        {
          errors = error_A();

          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
        }
      }

      goto _entN;
    }
  }
  else if (nfc == 'f')
  {
    if (distance > 0 || spl == 0)
    {
      if (spl == 0)
      {
        while (1)
        {
          errors = error_A();
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) - 50 && read_sensorA(1) < md_sensorA(1) - 50) ||
              (read_sensorA(7) < md_sensorA(7) - 50 && read_sensorA(6) < md_sensorA(6) - 50))
          {
            break;
          }
        }
      }
      else
      {
        if (kp > 0.65)
        {
          kp_slow = kp;
        }
        for (int i = spl; i > slmotor; i--)
        {
          errors = error_A();
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(i - PID_output, i + PID_output);
          delayMicroseconds(50);
          if (kp >= 5.5)
          {
            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) && read_sensorA(2) < md_sensorA(2)) ||
                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) && read_sensorA(5) < md_sensorA(5)))
            {
              goto en_for;
            }
          }
          else if (kp < 5.5)
          {
            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6)))
            {
              goto en_for;
            }
          }
          else
          {
          }
        }
        while (1)
        {
          errors = error_A();
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, slmotor + PID_output);
          delayMicroseconds(50);
          if (kp >= 5.5)
          {
            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) && read_sensorA(2) < md_sensorA(2)) ||
                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) && read_sensorA(5) < md_sensorA(5)))
            {
              break;
            }
          }
          else if (kp < 5.5)
          {
            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6)))
            {
              break;
            }
          }
          else
          {
          }
        }
      en_for:
        delay(10);
      }
    }
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        Motor(spl, spr);
        delayMicroseconds(50);
        if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7))
        {
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
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
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
        P = errors;
        I += errors * 0.00005;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
        Motor(spl - PID_output, spr + PID_output);
        delayMicroseconds(50);
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
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
        for (int i = spl; i > slmotor; i--)
        {
          if (read_sensorA(1) > md_sensorA(1) && read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6))
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
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
          Motor(i - PID_output, i + PID_output);
          delayMicroseconds(50);
          if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
          {
            break;
          }
          delay(3);
        }
      }
      while (1)
      {
        if (read_sensorA(1) > md_sensorA(1) && read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6))
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
        P = errors;
        I += errors * 0.00005;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
        Motor(slmotor - PID_output, slmotor + PID_output);
        delayMicroseconds(50);
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
          break;
        }
      }
    }
    if (splr == 's')
    {
      Motor(-spl, -spr);
      delay(endt);
      Motor(0, 0);
      delay(2);
    }
  }

  if (splr == 'l')
  {
    if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0))
    {
      while (1)
      {
        delayMicroseconds(50);
        Motor(slmotor, srmotor);
        if (read_sensorA(7) > md_sensorA(7) && read_sensorA(0) > md_sensorA(0))
        {
          delay(delay_f);
          break;
        }
      }
      Motor(-(slmotor), -(srmotor));
      delay(break_ff);
      for (int i = 0; i <= sensor_f; i++)
      {
        do
        {
          Motor((flml * power) / 100, (flmr * power) / 100);
          delayMicroseconds(50);
        } while (read_sensorA(i) > md_sensorA(i) - 50);
        delayMicroseconds(50);
      }
    }
    else
    {
      Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
      delay(nfc == 'n' ? 2 : break_fc);
      int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
      int end_i = start_i ? sensor_f : sensor_f;
      for (int i = start_i; i <= end_i; i++)
      {
        do
        {
          Motor((clml * power) / 100, (clmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fl();
    }
    else
    {
      Motor(-((clml * power) / 100), -((clmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }
  else if (splr == 'r')
  {
    if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0))
    {
      while (1)
      {
        delayMicroseconds(50);
        Motor(slmotor, srmotor);
        if (read_sensorA(7) > md_sensorA(7) && read_sensorA(0) > md_sensorA(0))
        {
          delay(delay_f);
          break;
        }
      }
      Motor(-(slmotor), -(srmotor));
      delay(break_ff);
      for (int i = 7; i >= sensor_f; i--)
      {
        do
        {
          Motor((frml * power) / 100, (frmr * power) / 100);
          delayMicroseconds(50);
        } while (read_sensorA(i) > md_sensorA(i) - 50);
        delayMicroseconds(50);
      }
    }
    else
    {
      Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
      delay(nfc == 'n' ? 2 : break_fc);
      int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
      for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--)
      {
        do
        {
          Motor((crml * power) / 100, (crmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fr();
    }
    else
    {
      Motor(-((crml * power) / 100), -((crmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }

_entN:
  delay(5);
}

void fline(int spl, int spr, float kp, String distance, char nfc, char splr, int power, String sensor, int endt)
{
  _fw = true;
  char sensors[4];
  sensor.toCharArray(sensors, sizeof(sensors));
  int sensor_f = atoi(&sensors[1]);
  int target_speed = min(spl, spr); // PWM
  const int ramp_step = 2;
  float traveled_distance = 0;
  unsigned long last_time = millis();

  if (kp == 0)
  {
    I = kp_slow = ki_slow = 0;
  }
  if (kp < 6.5)
  {
    pid_error = true;
  }
  else
  {
    pid_error = false;
  }

  int current_speed = 0; // PWM
  if (spl == 0)
  {
    goto _line;
  }

  // วิ่งปกติ

  while (1)
  {
    if (kp <= 0.65)
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
      if (read_sensorA(0) < md_sensorA(0) || read_sensorA(1) > md_sensorA(1) && read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6))
      {
        break;
      }
    }
    else if (distance == "a1")
    {
      if (read_sensorA(1) < md_sensorA(1))
      {
        break;
      }
    }
    else if (distance == "a6")
    {
      if (read_sensorA(6) < md_sensorA(6))
      {
        break;
      }
    }
    else if (distance == "a7")
    {
      if (read_sensorA(7) < md_sensorA(7) || read_sensorA(1) > md_sensorA(1) && read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6))
      {
        break;
      }
    }
    else if (distance == "a07" || distance == "a70")
    {
      if (read_sensorA(7) < md_sensorA(7) && read_sensorA(0) < md_sensorA(0))
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
      if (spl >= 1)
      {
        while (1)
        {
          if (kp <= 0.65)
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
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
              (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6)))
          {
            break;
          }
        }
      }
      else
      {
        while (1)
        {
          if (kp <= 0.65)
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
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
              (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6)))
          {
            break;
          }
        }
      }
    }
    else if (splr == 's')
    {
      if (endt > 0)
      {
        Motor(-spl, -spr);
        delay(endt);
        Motor(-1, -1);
        delay(10);
      }
      else
      {
        for (int i = spl; i > 15; i--)
        {
          errors = error_A();

          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
        }
      }

      goto _entN;
    }
  }
  else if (nfc == 'f')
  {
    if (distance > 0 || spl == 0)
    {
      if (spl == 0)
      {
        while (1)
        {
          errors = error_A();
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) - 50 && read_sensorA(1) < md_sensorA(1) - 50) ||
              (read_sensorA(7) < md_sensorA(7) - 50 && read_sensorA(6) < md_sensorA(6) - 50))
          {
            break;
          }
        }
      }
      else
      {
        for (int i = spl; i > slmotor; i--)
        {
          errors = error_A();
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(i - PID_output, i + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) - 50 && read_sensorA(1) < md_sensorA(1) - 50) ||
              (read_sensorA(7) < md_sensorA(7) - 50 && read_sensorA(6) < md_sensorA(6) - 50))
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
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, slmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) - 50 && read_sensorA(1) < md_sensorA(1) - 50) ||
              (read_sensorA(7) < md_sensorA(7) - 50 && read_sensorA(6) < md_sensorA(6) - 50))
          {
            break;
          }
        }
      en_for:
        delay(10);
      }
    }
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        Motor(spl, spr);
        delayMicroseconds(50);
        if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7))
        {
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
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
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
        P = errors;
        I += errors * 0.00005;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
        Motor(spl - PID_output, spr + PID_output);
        delayMicroseconds(50);
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
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
        for (int i = spl; i > slmotor; i--)
        {
          if (read_sensorA(1) > md_sensorA(1) && read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6))
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
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
          Motor(i - PID_output, i + PID_output);
          delayMicroseconds(50);
          if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
          {
            break;
          }
          delay(3);
        }
      }
      while (1)
      {

        if (read_sensorA(1) > md_sensorA(1) && read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6))
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
        P = errors;
        I += errors * 0.00005;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
        Motor(slmotor - PID_output, slmotor + PID_output);
        delayMicroseconds(50);
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
          break;
        }
      }
    }
    if (splr == 's')
    {
      Motor(-spl, -spr);
      delay(endt);
      Motor(0, 0);
      delay(2);
    }
  }

  if (splr == 'l')
  {
    if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0))
    {
      while (1)
      {
        Motor(slmotor, srmotor);
        if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7))
        {
          delay(delay_f);
          break;
        }
      }
      Motor(-(slmotor), -(srmotor));
      delay(break_ff);
      for (int i = 0; i <= sensor_f; i++)
      {
        if (sensor_f >= 2)
        {
          do
          {
            Motor((flml * power) / 100, (flmr * power) / 100);
            delayMicroseconds(50);
          } while (read_sensorA(i + 2) < md_sensorA(i + 2) - 50);
        }
        do
        {
          Motor((flml * power) / 100, (flmr * power) / 100);
          delayMicroseconds(50);
        } while (read_sensorA(i) > md_sensorA(i) - 50);
        delayMicroseconds(50);
      }
    }
    else
    {
      Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
      delay(nfc == 'n' ? 2 : break_fc);
      int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
      int end_i = start_i ? sensor_f : sensor_f;
      for (int i = start_i; i <= end_i; i++)
      {
        do
        {
          Motor((clml * power) / 100, (clmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fl();
    }
    else
    {
      Motor(-((clml * power) / 100), -((clmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }
  else if (splr == 'r')
  {
    if (nfc == 'f')
    {
      while (1)
      {
        Motor(slmotor, srmotor);
        delayMicroseconds(50);
        if (read_sensorA(0) >= md_sensorA(0) && read_sensorA(7) >= md_sensorA(7))
        {
          break;
        }
      }
      delay(delay_f);
      Motor(-slmotor, -srmotor);
      delay(break_ff);
      for (int i = 7; i >= sensor_f; i--)
      {
        if (sensor_f <= 5)
        {
          do
          {
            Motor((frml * power) / 100, (frmr * power) / 100);
            delayMicroseconds(50);
          } while (read_sensorA(i - 2) < md_sensorA(i - 2) - 50);
        }
        do
        {
          Motor((frml * power) / 100, (frmr * power) / 100);
          delayMicroseconds(50);
        } while (read_sensorA(i) > md_sensorA(i) - 50);
        delayMicroseconds(50);
      }
    }
    else
    {
      Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
      delay(nfc == 'n' ? 2 : break_fc);
      int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
      for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--)
      {
        do
        {
          Motor((crml * power) / 100, (crmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fr();
    }
    else
    {
      Motor(-((crml * power) / 100), -((crmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }

_entN:
  delay(5);
}

void bline(int spl, int spr, float kp, float distance, char nfc, char splr, int power, String sensor, int endt)
{
  _fw = false; // โหมดถอยหลัง
  char sensors[4];
  sensor.toCharArray(sensors, sizeof(sensors));
  int sensor_f = atoi(&sensors[1]); // เช่น "b5" -> sensor_f = 5
  int target_speed = min(spl, spr); // PWM
  const int ramp_step = 3;
  float traveled_distance = 0;
  unsigned long last_time = millis();

  if (kp == 0)
  {
    I = kp_slow = ki_slow = 0;
  }
  if (kp < 6.5)
  {
    pid_error = true;
  }
  else
  {
    pid_error = false;
  }

  int current_speed = 0; // PWM
  // Soft start
  if (spl == 0)
  {
    goto _line;
  }

  if (!ch_p)
  {
    while (current_speed < target_speed)
    {
      if (kp <= 0.65)
      {
        if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
        {
          errors = 0;
        }
        else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
        {

          errors = 10;
        }
        else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
        {
          errors = -10;
        }
        else
        {
          errors = error_B();
        }
      }
      else
      {
        errors = error_BB();
      }
      P = errors;
      I += errors * (ramp_delay / 1000.0);
      if (I > 1000)
        I = 1000;
      if (I < -1000)
        I = -1000;
      D = errors - previous_error;
      previous_error = errors;
      PID_output = (kp * P) + (0.00005 * I) + (kd_f * D);
      Motor(-(current_speed + PID_output), -(current_speed - PID_output)); // ถอยหลัง
      if (distance > 0)
      {
        unsigned long current_time = millis();
        float delta_time = (current_time - last_time) / 1000.0;
        traveled_distance += (current_speed * (speed_scale_bw + 0.55)) * delta_time;
        last_time = current_time;
        if (traveled_distance >= distance)
        {
          Motor(0, 0);
          return;
        }
      }
      current_speed += ramp_step;
      if (current_speed > target_speed)
        current_speed = target_speed;
      delay(ramp_delay);
      Serial.println(errors);
    }
  }

  // วิ่งปกติ
  while (1)
  {

    if (kp <= 0.65)
    {
      if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
      {
        errors = 0;
      }
      else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
      {

        errors = 10;
      }
      else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
      {
        errors = -10;
      }
      else
      {
        errors = error_B();
      }
    }
    else
    {
      errors = error_BB();
    }

    P = errors;
    I += errors * 0.00005;
    if (I > 1000)
      I = 1000;
    if (I < -1000)
      I = -1000;
    D = errors - previous_error;
    previous_error = errors;
    PID_output = (kp * P) + (0.00005 * I) + (kd_f * D);
    Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง

    if (distance > 0)
    {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * (speed_scale_bw + 0.95)) * delta_time;
      last_time = current_time;
      if (traveled_distance >= distance)
      {
        Motor(0, 0);
        break;
      }
    }
    else
    {
      if (kp >= 6.5 && kp < 1.0)
      {
        if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1) && read_sensorB(2) < md_sensorB(2)) ||
            (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) && read_sensorB(5) < md_sensorB(5)))
        {
          break;
        }
      }
      else if (kp < 6.5)
      {
        if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
            (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6)))
        {
          break;
        }
      }
      else
      {
      }
    }
    delayMicroseconds(100);
    Serial.println(errors);
  }

_line:
  ch_p = 0;

  if (nfc == 'n')
  {
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        if (kp <= 0.65)
        {
          if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
          {
            errors = 0;
          }
          else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
          {

            errors = 10;
          }
          else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
          {
            errors = -10;
          }
          else
          {
            errors = error_B();
          }
        }
        else
        {
          errors = error_BB();
        }
        P = errors;
        I += errors * 0.00005;
        if (I > 1000)
          I = 1000;
        if (I < -1000)
          I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
        Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
        delayMicroseconds(50);
        if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
            (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6)))
        {
          break;
        }
      }
    }
    else if (splr == 's')
    {
      if (endt > 0)
      {
        Motor(spl, spr); // เบรก
        delay(endt);
        Motor(-1, -1);
        delay(10);
      }
      goto _entN;
    }
  }
  else if (nfc == 'f')
  {
    if (distance > 0 || spl == 0)
    {
      while (1)
      {
        if (kp <= 0.65)
        {
          if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
          {
            errors = 0;
          }
          else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
          {
            errors = 10;
          }
          else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
          {
            errors = -10;
          }
          else
          {
            errors = error_B();
          }
        }
        else
        {
          errors = error_B();
        }
        P = errors;
        I += errors * 0.00005;
        if (I > 1000)
          I = 1000;
        if (I < -1000)
          I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
        Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
        delayMicroseconds(50);
        if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
            (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6)))
        {
          break;
        }
      }
    }
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        Motor(-spl, -spr); // ถอยหลัง
        delayMicroseconds(50);
        if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7))
        {
          break;
        }
      }
      delay(10);
    }
    else if (splr == 's')
    {
      Motor(spl, spr); // เบรก
      delay(endt);
      Motor(0, 0);
      delay(2);
    }
  }
  else if (nfc == 'c')
  {

    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        float error_L = map(read_sensorB(3), sensorMinB[3], sensorMaxB[3], 0, 20);
        float error_R = map(read_sensorB(4), sensorMinB[4], sensorMaxB[4], 0, 20);
        errors = error_L - error_R;
        P = errors;
        I += errors * 0.00005;
        if (I > 1000)
          I = 1000;
        if (I < -1000)
          I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
        Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง
        delayMicroseconds(50);
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
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
        if (kp <= 0.65)
        {
          if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
          {
            errors = 0;
          }
          else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
          {
            errors = 10;
          }
          else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
          {
            errors = -10;
          }
          else
          {
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
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
          break;
        }
      }
    }
    if (splr == 's')
    {
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
      while (1)
      {
        Motor(-slmotor, -srmotor);
        if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7))
        {
          delay(delay_f);
          break;
        }
      }
      Motor(slmotor + 10, srmotor + 10);
      delay(break_bf); //---------------------------------------------------------------------------------->>>
      for (int i = 0; i <= sensor_f; i++)
      {
        do
        {
          Motor(-((flmr * power) / 100), -((flml * power) / 100));
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
      for (int i = start_i; i <= end_i; i++)
      {
        do
        {
          Motor((clml * power) / 100, (clmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fl();
    }
    else
    {
      Motor(-((clml * power) / 100), -((clmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }
  else if (splr == 'r')
  {
    if (nfc == 'f')
    {
      while (1)
      {
        Motor(-slmotor, -srmotor);
        delayMicroseconds(50);
        if (read_sensorB(0) >= md_sensorB(0) && read_sensorB(7) >= md_sensorB(7))
        {
          break;
        }
      }
      delay(delay_f);
      Motor(slmotor, srmotor);
      delay(break_bf);
      for (int i = 7; i >= sensor_f; i--)
      {
        do
        {
          Motor(-((frmr * power) / 100), -((frml * power) / 100));
          delayMicroseconds(50);
        } while (read_sensorB(i) > md_sensorB(i) - 50);
        delayMicroseconds(50);
      }
    }
    else
    {
      Motor(nfc == 'n' ? 0 : slmotor, nfc == 'n' ? 0 : srmotor);
      delay(nfc == 'n' ? 2 : break_bc);
      int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
      for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--)
      {
        do
        {
          Motor((crml * power) / 100, (crmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fr();
    }
    else
    {
      Motor(-((crml * power) / 100), -((crmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }

  if (!(nfc == 'n' && splr == 's' && endt == 0))
  {
    Motor(-1, -1); // หยุด
    delay(endt / 2);
  }

_entN:
  delay(5);
}

void bline(int spl, int spr, float kp, String distance, char nfc, char splr, int power, String sensor, int endt)
{
  _fw = false; // โหมดถอยหลัง
  char sensors[4];
  sensor.toCharArray(sensors, sizeof(sensors));
  int sensor_f = atoi(&sensors[1]); // เช่น "b5" -> sensor_f = 5
  int target_speed = min(spl, spr); // PWM
  const int ramp_step = 3;
  float traveled_distance = 0;
  unsigned long last_time = millis();

  if (kp == 0)
  {
    I = kp_slow = ki_slow = 0;
  }
  if (kp < 6.5)
  {
    pid_error = true;
  }
  else
  {
    pid_error = false;
  }

  int current_speed = 0; // PWM
  // Soft start
  if (spl == 0)
  {
    goto _line;
  }

  // วิ่งปกติ
  while (1)
  {

    if (kp <= 0.65)
    {
      if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
      {
        errors = 0;
      }
      else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
      {

        errors = 10;
      }
      else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
      {
        errors = -10;
      }
      else
      {
        errors = error_B();
      }
    }
    else
    {
      errors = error_BB();
    }

    P = errors;
    I += errors * 0.00005;
    if (I > 1000)
      I = 1000;
    if (I < -1000)
      I = -1000;
    D = errors - previous_error;
    previous_error = errors;
    PID_output = (kp * P) + (0.00005 * I) + (kd_f * D);
    Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง

    if (distance == "b0")
    {
      if (read_sensorB(0) < md_sensorB(0))
      {
        break;
      }
    }
    else if (distance == "b1")
    {
      if (read_sensorB(1) < md_sensorB(1))
      {
        break;
      }
    }
    else if (distance == "b6")
    {
      if (read_sensorB(6) < md_sensorB(6))
      {
        break;
      }
    }
    else if (distance == "b7")
    {
      if (read_sensorB(7) < md_sensorB(7))
      {
        break;
      }
    }
    else if (distance == "b07" || distance == "b70")
    {
      if (read_sensorB(7) < md_sensorB(7) && read_sensorB(0) < md_sensorB(0))
      {
        break;
      }
    }
    delayMicroseconds(100);
    Serial.println(errors);
  }

_line:
  delay(10);
  ch_p = 0;

  if (nfc == 'n')
  {
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        if (kp <= 0.65)
        {
          if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
          {
            errors = 0;
          }
          else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
          {

            errors = 10;
          }
          else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
          {
            errors = -10;
          }
          else
          {
            errors = error_B();
          }
        }
        else
        {
          errors = error_BB();
        }
        P = errors;
        I += errors * 0.00005;
        if (I > 1000)
          I = 1000;
        if (I < -1000)
          I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
        Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
        delayMicroseconds(50);
        if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
            (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6)))
        {
          break;
        }
      }
    }
    else if (splr == 's')
    {
      if (endt > 0)
      {
        Motor(spl, spr); // เบรก
        delay(endt);
        Motor(-1, -1);
        delay(10);
      }
      goto _entN;
    }
  }
  else if (nfc == 'f')
  {
    if (distance > 0 || spl == 0)
    {
      while (1)
      {
        if (kp <= 0.65)
        {
          if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
          {
            errors = 0;
          }
          else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
          {
            errors = 10;
          }
          else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
          {
            errors = -10;
          }
          else
          {
            errors = error_B();
          }
        }
        else
        {
          errors = error_B();
        }
        P = errors;
        I += errors * 0.00005;
        if (I > 1000)
          I = 1000;
        if (I < -1000)
          I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
        Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
        delayMicroseconds(50);
        if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
            (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6)))
        {
          break;
        }
      }
    }
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        Motor(-spl, -spr); // ถอยหลัง
        delayMicroseconds(50);
        if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7))
        {
          break;
        }
      }
      delay(10);
    }
    else if (splr == 's')
    {
      Motor(spl, spr); // เบรก
      delay(endt);
      Motor(0, 0);
      delay(2);
    }
  }
  else if (nfc == 'c')
  {

    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        float error_L = map(read_sensorB(3), sensorMinB[3], sensorMaxB[3], 0, 20);
        float error_R = map(read_sensorB(4), sensorMinB[4], sensorMaxB[4], 0, 20);
        errors = error_L - error_R;
        P = errors;
        I += errors * 0.00005;
        if (I > 1000)
          I = 1000;
        if (I < -1000)
          I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
        Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง
        delayMicroseconds(50);
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
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
        if (kp <= 0.65)
        {
          if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5))
          {
            errors = 0;
          }
          else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7))
          {
            errors = 10;
          }
          else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0))
          {
            errors = -10;
          }
          else
          {
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
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
          break;
        }
      }
    }
    if (splr == 's')
    {
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
      while (1)
      {
        Motor(-slmotor, -srmotor);
        if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7))
        {
          delay(delay_f);
          break;
        }
      }
      Motor(slmotor + 10, srmotor + 10);
      delay(break_bf); //---------------------------------------------------------------------------------->>>
      for (int i = 0; i <= sensor_f; i++)
      {
        do
        {
          Motor(-((flmr * power) / 100), -((flml * power) / 100));
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
      for (int i = start_i; i <= end_i; i++)
      {
        do
        {
          Motor((clml * power) / 100, (clmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fl();
    }
    else
    {
      Motor(-((clml * power) / 100), -((clmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }
  else if (splr == 'r')
  {
    if (nfc == 'f')
    {
      while (1)
      {
        Motor(-slmotor, -srmotor);
        delayMicroseconds(50);
        if (read_sensorB(0) >= md_sensorB(0) && read_sensorB(7) >= md_sensorB(7))
        {
          break;
        }
      }
      delay(delay_f);
      Motor(slmotor, srmotor);
      delay(break_bf);
      for (int i = 7; i >= sensor_f; i--)
      {
        do
        {
          Motor(-((frmr * power) / 100), -((frml * power) / 100));
          delayMicroseconds(50);
        } while (read_sensorB(i) > md_sensorB(i) - 50);
        delayMicroseconds(50);
      }
    }
    else
    {
      Motor(nfc == 'n' ? 0 : slmotor, nfc == 'n' ? 0 : srmotor);
      delay(nfc == 'n' ? 2 : break_bc);
      int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
      for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--)
      {
        do
        {
          Motor((crml * power) / 100, (crmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fr();
    }
    else
    {
      Motor(-((crml * power) / 100), -((crmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }

  if (!(nfc == 'n' && splr == 's' && endt == 0))
  {
    Motor(-1, -1); // หยุด
    delay(endt / 2);
  }

_entN:
  delay(5);
}
///--------------------------------------------------------------------------------------------->>>จบ bline

///-------------------------------------------------------------------->>> หมุนตัวเพื่อวาง

// ----------------------- ตรวจสอบ Gyro ใช้ได้หรือไม่ -----------------------
bool gyroOK(float stable_threshold = 2.0, int samples = 15)
{
  float sum = 0;
  float prev = my.gyro('z');

  // ถ้าอ่านค่าไม่ได้ → เซนเซอร์เสียทันที
  if (isnan(prev))
    return false;

  for (int i = 0; i < samples; i++)
  {
    float g = my.gyro('z');
    if (isnan(g))
      return false; // Gyro เสีย
    sum += abs(g - prev);
    prev = g;
    delay(4);
  }

  float avg = sum / samples;

  // ถ้าการเปลี่ยนของค่า Gyro น้อย = นิ่ง = ใช้ได้
  return avg < stable_threshold;
}

// ----------------------- place_left_in เวอร์ชันปลอดภัย -----------------------
void place_left_in(int speed, int degree, int offset)
{
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  my.reCalibrateGyro(); // calibrate offset ใหม่ (เร็ว ~150 ms)
                        // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  delay(50);

  bool useGyro = gyroOK(); // << เช็คครั้งเดียวจบ!

  // =====================================================
  // โหมดใช้งาน Gyro (หากเซนเซอร์ทำงานปกติ)
  // =====================================================
  if (useGyro && my.gyro('z') < 200 && my.gyro('z') > -200)
  {
    // อ่านค่านิ่งเริ่มต้น
    float initialDegree = 0;
    for (int i = 0; i < 5; i++)
    {
      initialDegree += my.gyro('z');
      delay(10);
    }
    initialDegree /= 5.0;

    float target = initialDegree - degree;

    // PID
    float kp = 2.05;
    float ki = 0.00001;
    float kd = 0.015;

    float error, lastErr = 0, integral = 0;
    float output = 0;
    unsigned long lastT = millis();
    unsigned long timeout = 500;
    unsigned long start = millis();

    while (true)
    {
      float cur = my.gyro('z');
      error = target - cur;

      if (abs(error) < 1 && abs(output) < 5)
        break;

      unsigned long now = millis();
      float dt = (now - lastT) / 1000.0;
      lastT = now;

      if (dt > 0)
      {
        integral += error * dt;
        float derivative = (error - lastErr) / dt;
        lastErr = error;

        output = kp * error + ki * integral + kd * derivative;
      }

      output = constrain(output, -speed, speed);

      // หมุนทิศที่ถูกต้อง

      Motor(-1, -output);
      delay(5);

      if (millis() - start > timeout)
        break; // กันค้าง
    }

    // หยุดหลังหมุน

    Motor(1, output), delay(offset);

    Motor(-1, -1);
    delay(10);
    return;
  }

  // =====================================================
  // Fallback Mode: ถ้า Gyro ไม่มี / เสีย → ใช้ Encoder แทน
  // =====================================================
  tone(9, 3000, 80); // แจ้งว่าเข้าโหมด fallback

  int encoderTarget = map(abs(degree), 0, 90, 0, 600);

  encoder.resetEncoders();

  // หมุนซ้ายแบบไม่มี Gyro
  while (encoder.Poss_R() < encoderTarget)
    Motor(-1, speed);

  Motor(-1, speed), delay(offset);

  Motor(-1, -1);
  delay(40);
}

// ----------------------- place_left_out เวอร์ชันปลอดภัย -----------------------
void place_left_out(int speed, int degree, int offset)
{
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  my.reCalibrateGyro(); // calibrate offset ใหม่ (เร็ว ~150 ms)
                        // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  delay(50);

  bool useGyro = gyroOK(); // << ตรวจสอบครั้งเดียว

  // =====================================================
  // MODE 1 : ใช้ Gyro หมุนแบบ PID
  // =====================================================
  if (useGyro && my.gyro('z') < 200 && my.gyro('z') > -200)
  {
    float initialDegree = 0;
    for (int i = 0; i < 5; i++)
    {
      initialDegree += my.gyro('z');
      delay(10);
    }
    initialDegree /= 5.0;

    float targetDegree = initialDegree + degree;

    float kp = 2.05;
    float ki = 0.00001;
    float kd = 0.015;

    float error = 0, lastErr = 0;
    float integral = 0, output = 0;

    unsigned long lastT = millis();
    unsigned long timeout = 500;
    unsigned long start = millis();

    while (true)
    {
      float current = my.gyro('z');
      error = targetDegree - current;

      if (abs(error) < 1 && abs(output) < 5)
        break;

      unsigned long now = millis();
      float dt = (now - lastT) / 1000.0;
      lastT = now;

      if (dt > 0)
      {
        integral += error * dt;
        float derivative = (error - lastErr) / dt;
        lastErr = error;
        output = kp * error + ki * integral + kd * derivative;
      }

      output = constrain(output, -speed, speed);

      Motor(1, -output);
      delay(5);

      if (millis() - start > timeout)
        break;
    }

    // --------- After Stop ---------

    Motor(-1, output);
    delay(offset);

    Motor(-1, -1);
    delay(10);

    return; // สำคัญ!
  }

  // =====================================================
  // MODE 2 : Fallback — ใช้ Encoder หมุน
  // =====================================================
  tone(9, 3000, 80); // แจ้งเตือนว่าต้องใช้ Encoder

  int enTarget = map(abs(degree), 0, 90, 0, 600);
  encoder.resetEncoders();

  // หมุนออกด้านซ้ายแบบไม่มี Gyro

  while (encoder.Poss_R() > -enTarget)
    Motor(1, -speed);

  // Offset move

  Motor(-1, speed), delay(offset);

  Motor(-1, -1);
  delay(40);
}

void place_right_in(int speed, int degree, int offset)
{
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  my.reCalibrateGyro(); // calibrate offset ใหม่ (เร็ว ~150 ms)
                        // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  delay(50);

  bool useGyro = gyroOK(); // << เช็คครั้งเดียวจบ!

  // =====================================================
  // โหมดใช้งาน Gyro (หากเซนเซอร์ทำงานปกติ)
  // =====================================================
  if (useGyro && my.gyro('z') < 200 && my.gyro('z') > -200)
  {
    // อ่านค่านิ่งเริ่มต้น
    float initialDegree = 0;
    for (int i = 0; i < 5; i++)
    {
      initialDegree += my.gyro('z');
      delay(10);
    }
    initialDegree /= 5.0;

    float target = initialDegree + degree;

    // PID
    float kp = 2.05;
    float ki = 0.00001;
    float kd = 0.015;

    float error, lastErr = 0, integral = 0;
    float output = 0;
    unsigned long lastT = millis();
    unsigned long timeout = 500;
    unsigned long start = millis();

    while (true)
    {
      float cur = my.gyro('z');
      error = target - cur;

      if (abs(error) < 1 && abs(output) < 5)
        break;

      unsigned long now = millis();
      float dt = (now - lastT) / 1000.0;
      lastT = now;

      if (dt > 0)
      {
        integral += error * dt;
        float derivative = (error - lastErr) / dt;
        lastErr = error;

        output = kp * error + ki * integral + kd * derivative;
      }

      output = constrain(output, -speed, speed);

      // หมุนทิศที่ถูกต้อง

      Motor(output, -1);

      delay(5);

      if (millis() - start > timeout)
        break; // กันค้าง
    }

    // หยุดหลังหมุน

    Motor(-output, 1), delay(offset);

    Motor(-1, -1);
    delay(10);
    return;
  }

  // =====================================================
  // Fallback Mode: ถ้า Gyro ไม่มี / เสีย → ใช้ Encoder แทน
  // =====================================================
  tone(9, 3000, 80); // แจ้งว่าเข้าโหมด fallback

  int encoderTarget = map(abs(degree), 0, 90, 0, 600);

  encoder.resetEncoders();

  // หมุนขวาแบบไม่มี Gyro
  while (encoder.Poss_L() < encoderTarget)
    Motor(speed, -1);

  // offset movement

  Motor(speed, -1), delay(offset);

  Motor(-1, -1);
  delay(40);
}

void place_right_out(int speed, int degree, int offset)
{
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  my.reCalibrateGyro(); // calibrate offset ใหม่ (เร็ว ~150 ms)
                        // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  delay(50);

  bool useGyro = gyroOK(); // << ตรวจสอบครั้งเดียว

  // =====================================================
  // MODE 1 : ใช้ Gyro หมุนแบบ PID
  // =====================================================
  if (useGyro && my.gyro('z') < 200 && my.gyro('z') > -200)
  {
    float initialDegree = 0;
    for (int i = 0; i < 5; i++)
    {
      initialDegree += my.gyro('z');
      delay(10);
    }
    initialDegree /= 5.0;

    float targetDegree = initialDegree - degree;

    float kp = 2.05;
    float ki = 0.00001;
    float kd = 0.015;

    float error = 0, lastErr = 0;
    float integral = 0, output = 0;

    unsigned long lastT = millis();
    unsigned long timeout = 500;
    unsigned long start = millis();

    while (true)
    {
      float current = my.gyro('z');
      error = targetDegree - current;

      if (abs(error) < 1 && abs(output) < 5)
        break;

      unsigned long now = millis();
      float dt = (now - lastT) / 1000.0;
      lastT = now;

      if (dt > 0)
      {
        integral += error * dt;
        float derivative = (error - lastErr) / dt;
        lastErr = error;
        output = kp * error + ki * integral + kd * derivative;
      }

      output = constrain(output, -speed, speed);

      Motor(output, 1);

      delay(5);

      if (millis() - start > timeout)
        break;
    }

    // --------- After Stop ---------

    Motor(output, -1);
    delay(offset);

    Motor(-1, -1);
    delay(10);

    return; // สำคัญ!
  }

  // =====================================================
  // MODE 2 : Fallback — ใช้ Encoder หมุน
  // =====================================================
  tone(9, 3000, 80); // แจ้งเตือนว่าต้องใช้ Encoder

  int enTarget = map(abs(degree), 0, 90, 0, 600);
  encoder.resetEncoders();

  // หมุนออกด้านซ้ายแบบไม่มี Gyro

  while (encoder.Poss_L() > -enTarget)
    Motor(-speed, -1);

  // Offset move

  Motor(speed, -1), delay(offset);

  Motor(-1, -1);
  delay(40);
}

void rotate_right(int speed, int degree, int offset)
{
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  my.reCalibrateGyro(); // calibrate offset ใหม่ (เร็ว ~150 ms)
                        // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  // คำนวณค่ามุมเริ่มต้น
  float initialDegree = 0;
  for (int i = 0; i < 5; i++)
  {
    initialDegree += my.gyro('z'); // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
    delay(10);
  }
  initialDegree /= 5.0;

  // คำนวณมุมเป้าหมาย
  float targetDegree = initialDegree + degree;

  // กำหนดค่าของ PID

  float lr_kp = 2.05;    // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
  float lr_ki = 0.00001; // ค่า Ki ปรับตามความแม่นยำในการหยุด
  float lr_kd = 0.015;   // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น

  float error = 0, previous_error = 0;
  float integral = 0, output = 0;
  float currentDegree = 0;

  unsigned long lastTime = millis();
  unsigned long timeout = 500; // กำหนดเวลา timeout
  unsigned long startTime = millis();

  while (true)
  {
    currentDegree = my.gyro('z');         // อ่านค่ามุมปัจจุบัน
    error = targetDegree - currentDegree; // คำนวณความผิดพลาด

    // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
    if (abs(error) < 1 && abs(output) < 5)
      break;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    if (dt > 0)
    {
      integral += error * dt;
      float derivative = (error - previous_error) / dt;
      previous_error = error;

      output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
    }

    // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
    output = constrain(output, -speed, speed);

    // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
    if (degree > 0)
    {
      Motor(output, -output);
      delay(5);
    }
    else
    {
      Motor(-output, output);
      delay(5);
    }

    // ตรวจจับ timeout หากใช้เวลานานเกินไป
    if (millis() - startTime > timeout)
    {
      break; // ออกจาก loop หากเวลาผ่านไปนานเกินไป
    }
  }

  // หยุดมอเตอร์หลังจากหมุนเสร็จ
  if (degree > 0)
  {
    Motor(output, -output);
    delay(offset);
  }
  else
  {
    Motor(-output, output);
    delay(offset);
  }
  Motor(-1, -1);
  delay(10);
}

void rotate_left(int speed, int degree, int offset)
{
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  my.reCalibrateGyro(); // calibrate offset ใหม่ (เร็ว ~150 ms)
                        // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  // คำนวณค่ามุมเริ่มต้น
  float initialDegree = 0;
  for (int i = 0; i < 5; i++)
  {
    initialDegree += my.gyro('z'); // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
    delay(10);
  }
  initialDegree /= 5.0;

  // คำนวณมุมเป้าหมาย
  float targetDegree = initialDegree + (-degree);

  // กำหนดค่าของ PID

  float lr_kp = 2.05;    // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
  float lr_ki = 0.00001; // ค่า Ki ปรับตามความแม่นยำในการหยุด
  float lr_kd = 0.015;   // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น

  float error = 0, previous_error = 0;
  float integral = 0, output = 0;
  float currentDegree = 0;

  unsigned long lastTime = millis();
  unsigned long timeout = 500; // กำหนดเวลา timeout
  unsigned long startTime = millis();

  while (true)
  {
    currentDegree = my.gyro('z');         // อ่านค่ามุมปัจจุบัน
    error = targetDegree - currentDegree; // คำนวณความผิดพลาด

    // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
    if (abs(error) < 1 && abs(output) < 5)
      break;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    if (dt > 0)
    {
      integral += error * dt;
      float derivative = (error - previous_error) / dt;
      previous_error = error;

      output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
    }

    // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
    output = constrain(output, -speed, speed);

    // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
    if (degree > 0)
    {
      Motor(output, -output);
      delay(5);
    }
    else
    {

      Motor(-output, output);
      delay(5);
    }

    // ตรวจจับ timeout หากใช้เวลานานเกินไป
    if (millis() - startTime > timeout)
    {
      break; // ออกจาก loop หากเวลาผ่านไปนานเกินไป
    }
  }

  // หยุดมอเตอร์หลังจากหมุนเสร็จ
  if (degree > 0)
  {

    Motor(output, -output);
    delay(offset);
  }
  else
  {
    Motor(-output, output);
    delay(offset);
  }
  Motor(-1, -1);
  delay(10);
}

void fw_gyro(int spl, int spr, float kp, float distance, int offset)
{
  int target_speed = min(spl, spr);
  float traveled_distance = 0;
  unsigned long last_time = millis();

  float speed_scale = 1.5; // <-- ใช้ค่าที่คำนวณจากการวัดจริง

  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
  my.reCalibrateGyro(); // calibrate offset ใหม่ (เร็ว ~150 ms)
                        // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!
  my.resetAngles();     // รีเซ็ตมุมทั้งหมด
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
    if (dt <= 0)
      dt = 0.001;
    prevT = now;

    float yaw = my.gyro('z') - yaw_offset;
    float err = yaw;

    _integral += err * dt;
    float deriv = (err - _prevErr) / dt;
    _prevErr = err;
    float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

    int leftSpeed = constrain(spl - corr, -100, 100);
    int rightSpeed = constrain(spr + corr, -100, 100);
    Motor(leftSpeed, rightSpeed);

    if (distance > 0)
    {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance)
        break;
    }

    delayMicroseconds(50);
  }

  // soft stop
  if (offset > 0)
  {
    Motor(-15, -15);
    delay(offset);
    Motor(-1, -1);
    delay(10);
  }
  else
  {
    Motor(0, 0);
    delay(5);
  }
}

void fw_gyro(int spl, int spr, float kp, int distance, String sensorss, char sp, int offset)
{
  int target_speed = min(spl, spr);
  float traveled_distance = 0;
  unsigned long last_time = millis();

  float speed_scale = 1.5; // <-- ใช้ค่าที่คำนวณจากการวัดจริง

  resetAngles();
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
    if (dt <= 0)
      dt = 0.001;
    prevT = now;

    float yaw = my.gyro('z') - yaw_offset;
    float err = yaw;

    _integral += err * dt;
    float deriv = (err - _prevErr) / dt;
    _prevErr = err;
    float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

    int leftSpeed = constrain(spl - corr, -100, 100);
    int rightSpeed = constrain(spr + corr, -100, 100);
    Motor(leftSpeed, rightSpeed);
    if (distance > 0)
    {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance)
        break;
    }

    delayMicroseconds(50);
  }
  while (1)
  {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0)
      dt = 0.001;
    prevT = now;

    float yaw = my.gyro('z') - yaw_offset;
    float err = yaw;

    _integral += err * dt;
    float deriv = (err - _prevErr) / dt;
    _prevErr = err;
    float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

    int leftSpeed = constrain(spl - corr, -100, 100);
    int rightSpeed = constrain(spr + corr, -100, 100);
    Motor(leftSpeed, rightSpeed);
    if (sensorss == "a0")
    {
      if (read_sensorA(0) < md_sensorA(0))
      {
        break;
      }
    }
    else if (sensorss == "a7")
    {
      if (read_sensorA(7) < md_sensorA(7))
      {
        break;
      }
    }
    else if (sensorss == "b7")
    {
      if (read_sensorB(7) < md_sensorB(7))
      {
        break;
      }
    }
    else if (sensorss == "b0")
    {
      if (read_sensorB(0) < md_sensorB(0))
      {
        break;
      }
    }
    else if (sensorss == "c0")
    {
      if (analogRead(A0) < (sensorMinC[0] + sensorMaxC[0]) / 2)
      {
        break;
      }
    }
    else if (sensorss == "c1")
    {
      if (analogRead(A1) < (sensorMinC[1] + sensorMaxC[1]) / 2)
      {
        break;
      }
    }
    delayMicroseconds(50);
  }
  if (sp == 'p')
  {
    if (sensorss == "a0")
    {
      do
      {
        Motor(spl, spr);
        delayMicroseconds(50);
      } while (read_sensorA(0) < md_sensorA(0));
    }
    else if (sensorss == "a7")
    {
      do
      {
        Motor(spl, spr);
        delayMicroseconds(50);
      } while (read_sensorA(7) < md_sensorA(7));
    }
    else if (sensorss == "b7")
    {
      do
      {
        Motor(spl, spr);
        delayMicroseconds(50);
      } while (read_sensorB(0) < md_sensorB(0));
    }
    else if (sensorss == "b0")
    {
      do
      {
        Motor(spl, spr);
        delayMicroseconds(50);
      } while (read_sensorB(7) < md_sensorB(7));
    }
    else if (sensorss == "c0")
    {
      do
      {
        Motor(spl, spr);
        delayMicroseconds(50);
      } while (analogRead(A0) < (sensorMinC[0] + sensorMaxC[0]) / 2);
    }
    else if (sensorss == "c1")
    {
      do
      {
        Motor(spl, spr);
        delayMicroseconds(50);
      } while (analogRead(A1) < (sensorMinC[0] + sensorMaxC[1]) / 2);
    }
  }
  else
  {
  }
  // soft stop
  if (offset > 0)
  {
    Motor(-15, -15);
    delay(offset);
    Motor(-1, -1);
    delay(10);
  }
  else
  {
    Motor(0, 0);
    delay(5);
  }
}
// ===========================
// ฟังก์ชันวิ่งถอยหลัง ใช้ Gyro + PID
// ===========================
void bw_gyro(int spl, int spr, float kp, float distance, int offset)
{
  int target_speed = min(spl, spr);
  float traveled_distance = 0;
  unsigned long last_time = millis();

  float speed_scale = 1.6; // ใช้ค่าที่คาลิเบรตจาก fw()

  resetAngles();
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
    if (dt <= 0)
      dt = 0.001;
    prevT = now;

    float yaw = my.gyro('z') - yaw_offset;
    float err = -yaw;

    _integral += err * dt;
    float deriv = (err - _prevErr) / dt;
    _prevErr = err;
    float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

    // สปีดถอยหลัง
    int leftSpeed = constrain(-(spl - corr), -100, 100);
    int rightSpeed = constrain(-(spr + corr), -100, 100);
    Motor(leftSpeed, rightSpeed);

    if (distance > 0)
    {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance)
        break;
    }

    delayMicroseconds(50);
  }

  if (offset > 0)
  {
    Motor(-15, -15);
    delay(offset);
    Motor(1, 1);
    delay(10);
  }
  else
  {
    Motor(0, 0);
    delay(5);
  }
}

void bw_gyro(int spl, int spr, float kp, int distance, String sensorss, char sp, int offset)
{
  int target_speed = min(spl, spr);
  float traveled_distance = 0;
  unsigned long last_time = millis();

  float speed_scale = 1.6; // ใช้ค่าที่คาลิเบรตจาก fw()

  resetAngles();
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
    if (dt <= 0)
      dt = 0.001;
    prevT = now;

    float yaw = my.gyro('z') - yaw_offset;
    float err = -yaw;

    _integral += err * dt;
    float deriv = (err - _prevErr) / dt;
    _prevErr = err;
    float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

    // สปีดถอยหลัง
    int leftSpeed = constrain(-(spl - corr), -100, 100);
    int rightSpeed = constrain(-(spr + corr), -100, 100);
    Motor(leftSpeed, rightSpeed);

    if (distance > 0)
    {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance)
        break;
    }

    delayMicroseconds(50);
  }

  while (1)
  {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0)
      dt = 0.001;
    prevT = now;

    float yaw = my.gyro('z') - yaw_offset;
    float err = -yaw;

    _integral += err * dt;
    float deriv = (err - _prevErr) / dt;
    _prevErr = err;
    float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

    // สปีดถอยหลัง
    int leftSpeed = constrain(-(spl - corr), -100, 100);
    int rightSpeed = constrain(-(spr + corr), -100, 100);
    Motor(leftSpeed, rightSpeed);

    if (sensorss == "a0")
    {
      if (read_sensorA(0) < md_sensorA(0))
      {
        break;
      }
    }
    else if (sensorss == "a7")
    {
      if (read_sensorA(7) < md_sensorA(7))
      {
        break;
      }
    }
    else if (sensorss == "b7")
    {
      if (read_sensorB(7) < md_sensorB(7))
      {
        break;
      }
    }
    else if (sensorss == "b0")
    {
      if (read_sensorB(0) < md_sensorB(0))
      {
        break;
      }
    }
    else if (sensorss == "c0")
    {
      if (analogRead(A0) < (sensorMinC[0] + sensorMaxC[0]) / 2)
      {
        break;
      }
    }
    else if (sensorss == "c1")
    {
      if (analogRead(A1) < (sensorMinC[1] + sensorMaxC[1]) / 2)
      {
        break;
      }
    }
    delayMicroseconds(50);
  }
  if (sp == 'p')
  {
    if (sensorss == "a0")
    {
      do
      {
        Motor(-spl, -spr);
        delayMicroseconds(50);
      } while (read_sensorA(0) < md_sensorA(0));
      delay(5);
    }
    else if (sensorss == "a7")
    {
      do
      {
        Motor(-spl, -spr);
        delayMicroseconds(50);
      } while (read_sensorA(7) < md_sensorA(7));
      delay(5);
    }
    else if (sensorss == "b0")
    {
      do
      {
        Motor(-spl, -spr);
        delayMicroseconds(50);
      } while (read_sensorB(0) < md_sensorB(0));
      delay(5);
    }
    else if (sensorss == "b7")
    {
      do
      {
        Motor(-spl, -spr);
        delayMicroseconds(50);
      } while (read_sensorB(7) < md_sensorB(7));
      delay(5);
    }
    else if (sensorss == "c0")
    {
      do
      {
        Motor(-spl, -spr);
        delayMicroseconds(50);
      } while (analogRead(A0) < (sensorMinC[0] + sensorMaxC[0]) / 2);
    }
    else if (sensorss == "c1")
    {
      do
      {
        Motor(-spl, -spr);
        delayMicroseconds(50);
      } while (analogRead(A1) < (sensorMinC[1] + sensorMaxC[1]) / 2);
    }
  }
  else
  {
  }

  if (offset > 0)
  {
    Motor(-15, -15);
    delay(offset);
    Motor(1, 1);
    delay(10);
  }
  else
  {
    Motor(0, 0);
    delay(5);
  }
}

//-------------------------------------------------------------------->>>
int position_4sensor()
{
  int Minsensor_values_A[] = {sensorMinA[2], sensorMinA[3], sensorMinA[4], sensorMinA[5]}; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
  int Maxsensor_values_A[] = {sensorMaxA[2], sensorMaxA[3], sensorMaxA[4], sensorMaxA[5]}; // ค่าที่อ่านได้มากสุด สีขาว
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
    long value = map(read_sensorA(sensor_pin_4[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0); // จากนั้นก็เก็บเข้าไปยังตัวแป value

    if (value > 200)
    {
      onLine = true;
    }
    if (value > 50)
    {
      avg += (long)value * (i * 1000);
      sum += value;
    }
    // delayMicroseconds(50);
  }
  if (!onLine) // เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
  {
    if (_lastPosition < (numSensor4 - 1) * 1000 / 2) // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
    {
      return 0;
    }
    else // แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
    {
      return 3000;
    }
  }
  _lastPosition = avg / sum; // นำมาหาค่าเฉลี่ย

  return _lastPosition; // ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
}

float error_4sensor()
{
  present_position = position_4sensor() / ((numSensor4 - 1) * 10);
  setpoint = 50.0;
  errors = setpoint - present_position;
  return errors;
}

void fline4sensor(int spl, int spr, float kp, String distance, char nfc, char splr, int power, String sensor, int endt)
{
  _fw = true;
  char sensors[4];
  sensor.toCharArray(sensors, sizeof(sensors));
  int sensor_f = atoi(&sensors[1]);
  int target_speed = min(spl, spr); // PWM
  const int ramp_step = 2;
  float traveled_distance = 0;
  unsigned long last_time = millis();

  if (kp == 0)
  {
    I = kp_slow = ki_slow = 0;
  }
  if (kp < 6.5)
  {
    pid_error = true;
  }
  else
  {
    pid_error = false;
  }

  int current_speed = 0; // PWM
  if (spl == 0)
  {
    goto _line;
  }

  // วิ่งปกติ

  while (1)
  {
    if (kp <= 0.65)
    {
      if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5))
      {
        errors = 0;
      }
      else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(7) < md_sensorA(7))
      {
        errors = 10;
      }
      else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(0) < md_sensorA(0))
      {
        errors = -10;
      }
      else
      {
        errors = error_4sensor();
      }
    }
    else
    {
      errors = error_4sensor();
    }
    P = errors;
    I += errors * 0.00005; // สำหรับ 50us
    D = errors - previous_error;
    previous_error = errors;
    PID_output = (kp * P) + (0.000001 * I) + (kd_f * D);
    Motor(spl - PID_output, spr + PID_output);
    if (distance == "a0")
    {
      if (read_sensorA(0) < md_sensorA(0))
      {
        break;
      }
    }
    else if (distance == "a7")
    {
      if (read_sensorA(7) < md_sensorA(7))
      {
        break;
      }
    }
    else if (distance == "a07" || distance == "a70")
    {
      if (read_sensorA(7) < md_sensorA(7) && read_sensorA(0) < md_sensorA(0))
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
      if (spl >= 1)
      {
        while (1)
        {
          if (kp <= 0.65)
          {
            if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5))
            {
              errors = 0;
            }
            else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(7) < md_sensorA(7))
            {
              errors = 10;
            }
            else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(0) < md_sensorA(0))
            {
              errors = -10;
            }
            else
            {
              errors = error_4sensor();
            }
          }
          else
          {
            errors = error_4sensor();
          }

          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0)) ||
              (read_sensorA(7) < md_sensorA(7)))
          {
            break;
          }
        }
      }
      else
      {
        while (1)
        {
          if (kp <= 0.65)
          {
            if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5))
            {
              errors = 0;
            }
            else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(7) < md_sensorA(7))
            {
              errors = 10;
            }
            else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(0) < md_sensorA(0))
            {
              errors = -10;
            }
            else
            {
              errors = error_4sensor();
            }
          }
          else
          {
            errors = error_4sensor();
          }
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0)) ||
              (read_sensorA(7) < md_sensorA(7)))
          {
            break;
          }
        }
      }
    }
    else if (splr == 's')
    {
      if (endt > 0)
      {
        Motor(-spl, -spr);
        delay(endt);
        Motor(-1, -1);
        delay(10);
      }
      else
      {
        for (int i = spl; i > 15; i--)
        {
          errors = error_4sensor();

          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
        }
      }

      goto _entN;
    }
  }
  else if (nfc == 'f')
  {
    if (distance > 0 || spl == 0)
    {
      if (spl == 0)
      {
        while (1)
        {
          errors = error_4sensor();
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, srmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) - 50) ||
              (read_sensorA(7) < md_sensorA(7) - 50))
          {
            break;
          }
        }
      }
      else
      {
        for (int i = spl; i > slmotor; i--)
        {
          errors = error_4sensor();
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(i - PID_output, i + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) - 50) ||
              (read_sensorA(7) < md_sensorA(7) - 50))
          {
            goto en_for;
          }
          delay(3);
        }
        while (1)
        {
          errors = error_4sensor();
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp_slow * P) + (0.000001 * I) + (0.025 * D);
          Motor(slmotor - PID_output, slmotor + PID_output);
          delayMicroseconds(50);
          if ((read_sensorA(0) < md_sensorA(0) - 50) ||
              (read_sensorA(7) < md_sensorA(7) - 50))
          {
            break;
          }
        }
      en_for:
        delay(10);
      }
    }
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        Motor(spl, spr);
        delayMicroseconds(50);
        if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7))
        {
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
    if (splr == 'p')
    {
      ch_p = 1;
      while (1)
      {
        if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5))
        {
          errors = 0;
        }
        else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(7) < md_sensorA(7))
        {
          errors = 10;
        }
        else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(0) < md_sensorA(0))
        {
          errors = -10;
        }
        else
        {
          errors = error_4sensor();
        }
        P = errors;
        I += errors * 0.00005;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
        Motor(spl - PID_output, spr + PID_output);
        delayMicroseconds(50);
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
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
        for (int i = spl; i > slmotor; i--)
        {
          if (read_sensorA(1) > md_sensorA(1) && read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6))
          {
            errors = 0;
          }
          else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(7) < md_sensorA(7))
          {
            errors = 10;
          }
          else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(0) < md_sensorA(0))
          {
            errors = -10;
          }
          else
          {
            errors = error_4sensor();
          }
          P = errors;
          I += errors * 0.00005;
          D = errors - previous_error;
          previous_error = errors;
          PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
          Motor(i - PID_output, i + PID_output);
          delayMicroseconds(50);
          if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
          {
            break;
          }
          delay(3);
        }
      }
      while (1)
      {

        if (read_sensorA(1) > md_sensorA(1) && read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6))
        {
          errors = 0;
        }
        else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(7) < md_sensorA(7))
        {
          errors = 10;
        }
        else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(0) < md_sensorA(0))
        {
          errors = -10;
        }
        else
        {
          errors = error_4sensor();
        }
        P = errors;
        I += errors * 0.00005;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.0000001 * I) + (0.0125 * D);
        Motor(slmotor - PID_output, slmotor + PID_output);
        delayMicroseconds(50);
        if (analogRead(A0) < (sensorMinC[0] + md_sensorC(0)) / 2 || analogRead(A1) < (sensorMinC[1] + md_sensorC(1)) / 2)
        {
          break;
        }
      }
    }
    if (splr == 's')
    {
      Motor(-spl, -spr);
      delay(endt);
      Motor(0, 0);
      delay(2);
    }
  }

  if (splr == 'l')
  {
    if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0))
    {
      while (1)
      {
        Motor(slmotor, srmotor);
        if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7))
        {
          delay(delay_f);
          break;
        }
      }
      Motor(-(slmotor), -(srmotor));
      delay(break_ff);
      for (int i = 0; i <= sensor_f; i++)
      {
        if (sensor_f >= 2)
        {
          do
          {
            Motor((flml * power) / 100, (flmr * power) / 100);
            delayMicroseconds(50);
          } while (read_sensorA(i + 2) < md_sensorA(i + 2) - 50);
        }
        do
        {
          Motor((flml * power) / 100, (flmr * power) / 100);
          delayMicroseconds(50);
        } while (read_sensorA(i) > md_sensorA(i) - 50);
        delayMicroseconds(50);
      }
    }
    else
    {
      Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
      delay(nfc == 'n' ? 2 : break_fc);
      int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
      int end_i = start_i ? sensor_f : sensor_f;
      for (int i = start_i; i <= end_i; i++)
      {
        do
        {
          Motor((clml * power) / 100, (clmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fl();
    }
    else
    {
      Motor(-((clml * power) / 100), -((clmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }
  else if (splr == 'r')
  {
    if (nfc == 'f')
    {
      while (1)
      {
        Motor(slmotor, srmotor);
        delayMicroseconds(50);
        if (read_sensorA(0) >= md_sensorA(0) && read_sensorA(7) >= md_sensorA(7))
        {
          break;
        }
      }
      delay(delay_f);
      Motor(-slmotor, -srmotor);
      delay(break_ff);
      for (int i = 7; i >= sensor_f; i--)
      {
        if (sensor_f <= 5)
        {
          do
          {
            Motor((frml * power) / 100, (frmr * power) / 100);
            delayMicroseconds(50);
          } while (read_sensorA(i - 2) < md_sensorA(i - 2) - 50);
        }
        do
        {
          Motor((frml * power) / 100, (frmr * power) / 100);
          delayMicroseconds(50);
        } while (read_sensorA(i) > md_sensorA(i) - 50);
        delayMicroseconds(50);
      }
    }
    else
    {
      Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
      delay(nfc == 'n' ? 2 : break_fc);
      int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
      for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--)
      {
        do
        {
          Motor((crml * power) / 100, (crmr * power) / 100);
          delayMicroseconds(50);
        } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
        delayMicroseconds(50);
      }
    }
    if (endt == 0)
    {
      turn_speed_fr();
    }
    else
    {
      Motor(-((crml * power) / 100), -((crmr * power) / 100));
      delay(endt);
      Motor(-1, -1);
      delay(10);
    }
  }

_entN:
  delay(5);
}

#endif
