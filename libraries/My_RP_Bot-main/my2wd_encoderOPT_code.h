#ifndef _my_code_
#define _my_code_

#include "myencoder_2motor.h"
#include "my2WD_encoder_OPTsensor.h"
#include "my_servo_optM.h"
#include <my_GYRO.h>
my_GYRO my;

#define ENCODER_PIN 20  // ขารับสัญญาณ encoder
volatile long encoderCount = 0;
int servo_dir;
int encoder_before = 0;



// ค่าคงที่สำหรับการคำนวณ
const float wheelDiameter = 5.0;                      // เส้นผ่านศูนย์กลางล้อ (เซนติเมตร)
const float wheelCircumference = PI * wheelDiameter;  // เส้นรอบวงล้อ
const int pulsesPerRevolution = 450;                  // จำนวนพัลส์ต่อรอบ
const float pulsesPerCm = pulsesPerRevolution / wheelCircumference; // พัลส์ต่อเซนติเมตร

bool lines;
bool lines_fw;
bool lines_bw;
bool sett_f = false;
bool ch_lr = false;
bool ch_set_fb = false;
bool ch_bw = false;
bool line_l = true;
bool line_r = true;
char ch_lrs;

bool set_bb = false;
int motor_slow = 18;
int fw_to_rotate = 180;

// กำหนดค่า PID
float Kp = 0.05;  // Proportional gain
float Ki = 0.000015; // Integral gain
float Kd = 0.015;  // Derivative gain

// ตัวแปรสำหรับ PID Control
float previousError = 0;
float integral = 0;

int servo_28_close = 135;   // แขนซ้าย
int servo_27_close = 130;   // แขนขวา
unsigned long lastTime = millis();
bool error_servo  = false;
void encoderISR(void) ;
void set_f(int _time);

float lr_kp, lr_ki, lr_kd;
bool chopsticks  = false;
unsigned long lastTimes = millis();
float Kpp = 0.8, Kii = 0.0, Kdd = 0.3;
float _integral = 0, _prevErr = 0;
unsigned long prevT;

void set_pid_moveLR(float _lr_kp, float _lr_ki, float _lr_kd)
  {
    lr_kp = _lr_kp;
    lr_ki = _lr_ki;
    lr_kd = _lr_kd;
  }

void set_pid_chopsticks(float _lr_kp, float _lr_ki, float _lr_kd)
  {
    Kpp= _lr_kp;
    Kii = _lr_ki;
    Kdd = _lr_kd;
  }

void setup_OPT()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
     my.begin();
     pinMode(25, OUTPUT);
     pinMode(20, INPUT_PULLUP);
     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);
     my.resetAngles();
      _integral = _prevErr = 0;
      prevT = millis();
  }

  void robot_start()
  {
    bz(100);
    bz(100); 
    eep_to_code();
    digitalWrite(25, 1);
    int buttonState;
    unsigned long pressStartTime = 0;
    bool isPressed = false;   

    while(1)
      {       
        for(int i = 0; i<8; i++)
          {
            Serial.print(mcp_f(i));
            Serial.print("  ");
            delay(10);
          }
        Serial.println(" "); 
        buttonState = digitalRead(9);
        if (buttonState == LOW) 
          {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
            digitalWrite(25, 0);
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
                    Serial.println("Entering Mode A");
                    add_sensor_F();
                    while (digitalRead(9) == LOW);  // รอให้ปล่อยปุ่ม
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
    bz(500);
    mydisplay_background(black);
    mydisplay("End", 5, 10 ,2, white);    
  }

void test_bord()
  {
    for(int i=0; i<3; i++)
      {
        servo(27, 20);servo(28, 20); servo(20, 20); delay(1000); 
        servo(27, 120);servo(28, 120); servo(20, 120); delay(1000); 
      }
    while(digitalRead(9)==1)
      {
        Serial.print(encoder.Poss_L()); 
        Serial.print("  "); 
        Serial.println(encoder.Poss_R()); 
      }
    bz(200);
    encoder.resetEncoders();
    for(int i=0; i<2; i++)
      {
        Motor(30, 30); delay(1000);
        Motor(-30, -30); encoder.Poss_L(); encoder.Poss_R();delay(1000);
      }
    Motor(0, 0);
  }
void encoderISR() 
  {   
        encoderCount++;  // หมุนตามเข็มนาฬิกา เพิ่มค่า
  }

//----------------------------------------------------->>  

void arm_up_down(int _position)  //------------------------>> ฟังก์ชัน เลื่อนแขนขึ้น
{
    int position_current = abs(_position - encoder_before);
    encoderCount = 0;
    unsigned long startTime = millis(); // ใช้จับเวลาเผื่อหลุดลูป
    if (_position > encoder_before)
    {
        do {
            servo(29, 180);
            delay(1);  // ป้องกันอ่าน encoder ไม่ทัน
            if (millis() - startTime > 3000) break;  // ถ้าเกิน 3 วิ ให้หลุดลูป
        } while (encoderCount <= position_current);
    }
    else
    {
        do {
            servo(29, 0);
            delay(1);
            if (millis() - startTime > 3000) break;
        } while (encoderCount <= position_current);
    }

    servo(29, 90);  // หยุดมอเตอร์
    encoder_before = _position;
    Serial.println(encoder_before);
}

void arm_Slide(int position)
  {
    if( position > 0)
      {
        servo(29, 180);
        delay(abs(position));
        servo(29, 90);
      }
    else  if( position < 0)
      {
        servo(29, 0);
        delay(abs(position));
        servo(29, 90);
      }
    else  
      {
        do{servo(29, 0);}while(digitalRead(20)==1);
        servo(29, 90); delay(50);
        do{servo(29, 180);}while(digitalRead(20)==0);
        servo(29, 90); delay(50);
      }
  }
void arm_up(int _position)  //------------------------>> ฟังก์ชัน เลื่อนแขนขึ้น
  {
    servo(29, 180);
    delay(_position);
    servo(29, 90);
  }

void arm_down(int _position)  //------------------------>> ฟังก์ชัน เลื่อนแขนขึ้น
  {
    servo(29, 0);
    delay(_position);
    servo(29, 90);
  }




void moveLR(int speed, int degree) 
  {
    if(lines == true || sett_f == true ||set_bb == true)
       {
          if(lines_fw == true || sett_f == true)
              {
                encoder.resetEncoders();
                do{Motor(-20, -20);}while(encoder.Poss_L() > -fw_to_rotate);
                Motor(20, 20); delay(20);
                Motor(-1, -1);
                delay(10); 
              }
          else if(lines_bw == true ||set_bb == true)
            {
                encoder.resetEncoders();
                do{Motor(20, 20);}while(encoder.Poss_L() < fw_to_rotate);
                Motor(-20, -20); delay(20);
                Motor(1, 1);
                delay(10); 
            }
        }
     else
        {
            Motor(-2, -2);
            delay(10); 
        } 
        Motor(-2, -2);
        delay(20);
        my.resetAngles();
        delay(10);
    
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(5);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + degree;
    
        // กำหนดค่าของ PID
        /*
        lr_kp  = 1.20;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        lr_kp  = 0.0001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        flr_kp = 0.03; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        */
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 300;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1.0 && abs(output) < 5) break;
    
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
            Motor(output, -output);  
            delay(5);
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            Motor(-10, 10);
            delay(20);
          }
        else
          {
            Motor(10, -10);
            delay(20);
          }
        Motor(-1, -1);
        delay(10);
  }


void moveLR(int _line, int speed, int degree) 
  {
      // เริ่มต้นการหมุน
      encoder.resetEncoders();
      if(sett_f == true)
         {
              encoder.resetEncoders();
              do{Motor(-12, -12);}while(encoder.Poss_L() > -_line);
              Motor(15, 15); delay(30);
              Motor(-1, -1);
              delay(10); 
          }
       else if(set_bb  == true)
          {
              encoder.resetEncoders();
              do{Motor(15, 15);}while(encoder.Poss_L() < _line);
              Motor(-20, -20); delay(30);
              Motor(1, 1);
              delay(10); 
          }
        sett_f = false;
        set_bb = false;
        Motor(-1, -1);
        delay(50);
        my.resetAngles();
        delay(10);
    
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
        /*
        lr_kp  = 1.20;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        lr_kp  = 0.0001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        flr_kp = 0.03; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        */
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
            if (abs(error) < 1.0 && abs(output) < 5) break;
    
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
            Motor(output, -output);  
            delay(5);
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            Motor(-10, 10);
            delay(20);
          }
        else
          {
            Motor(10, -10);
            delay(20);
          }
        Motor(-1, -1);
        delay(10);
  }
  
 

// ฟังก์ชัน PID Control
int computePID(float setpoint, float currentValue) 
{
    // คำนวณ error
    float error = setpoint - currentValue;
    
    // คำนวณ integral
    integral += error;
    
    // คำนวณ derivative
    float derivative = error - previousError;
    
    // คำนวณ output
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    if(chopsticks  == true)
      {
        output = (Kp*1.5 * error) + (Ki * integral) + (Kd * derivative);
      }
   
    
    
    // อัพเดต previousError
    previousError = error;
    
    return (int)output;
}

void fw(int spl, int spr, float kps, int targetDistanceCm, String _line) 
{  
    char lr;
    encoder.resetEncoders();
    lines_fw = true;  
    lines_bw = false;  

    if (set_bb == true) {
        targetDistanceCm = targetDistanceCm + 10;
        set_bb = false;
    }

    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซต Motor และ Gyro
    Motor(-1, -1); delay(10);
    my.resetAngles();

    // *** เริ่มต้นตั้งค่าตัวแปรสำหรับ PID ***
    float yaw_offset = my.gyro('z'); // << เก็บค่าตอนเริ่มต้น
    _integral = 0;
    _prevErr = 0;
    prevT = millis();

    // เตรียมตัวสำหรับการเร่งช้าๆ
    float rampUpDistance = targetPulses * 0.2;   // ช่วงเร่ง 20% แรก
    float rampDownDistance = targetPulses * 0.8; // ช่วงเริ่มผ่อน 20% ท้าย
    int minSpeed = 20; // กำหนดสปีดขั้นต่ำ
    int maxLeftSpeed = spl;
    int maxRightSpeed = spr;

    while (true) {
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว
        float currentPulses = (leftPulses + rightPulses) / 2;
        float remainingPulses = targetPulses - currentPulses;

        // อ่านเวลา
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; // ป้องกันหาร 0
        prevT = now;

        // อ่าน Gyro และคำนวณ Error
        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        // PID
        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kps * err + Kii * _integral + Kdd * deriv;

        // คำนวณสปีดพื้นฐาน
        int baseLeftSpeed = maxLeftSpeed;
        int baseRightSpeed = maxRightSpeed;

        // ทำ Ramp-up และ Ramp-down
        if (currentPulses < rampUpDistance) {
            float rampFactor = currentPulses / rampUpDistance;
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }
        else if (currentPulses > rampDownDistance) {
            float rampFactor = (targetPulses - currentPulses) / (targetPulses - rampDownDistance);
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }

        // สั่งมอเตอร์ โดยชดเชย PID correction
        int leftSpeed = constrain(baseLeftSpeed - corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed + corr, -100, 100);

        Motor(leftSpeed, rightSpeed);
        
       // Serial.println(yaw); // Debug ดูค่า yaw

        if (currentPulses >= targetPulses) {
            break;
          }
        if (mcp_f(0) < md_mcp_f(0)-30 && mcp_f(3) > md_mcp_f(3)) 
          {
            Motor(leftSpeed, rightSpeed/2);
            my.resetAngles();
          } 
        else if (mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-30) 
          {
            Motor(leftSpeed/2, rightSpeed);
            my.resetAngles();
          }
        else if (mcp_f(1) < md_mcp_f(1) - 50 || mcp_f(2) < md_mcp_f(2) - 50) 
          {
            Motor(-30, -30); delay(30);  
            Motor(-1, -1); delay(10);  

            while (1) {
                if (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) > md_mcp_f(2)) {
                    lr = 'l';
                    Motor(-10, 30);        
                }
                else if (mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2)- 50) {
                    lr = 'r';
                    Motor(30, -10);           
                }
                else if ((mcp_f(0) < md_mcp_f(0) - 50&& mcp_f(3) < md_mcp_f(3)- 50)
                         || (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) < md_mcp_f(2)- 50)) {
                    if (lr == 'l') {
                        Motor(15, -15); delay(20);
                        Motor(1, -1); delay(10);
                        Motor(0, 0); delay(10);
                        break; 
                    }
                    if (lr == 'r') {
                        Motor(-15, 15); delay(20);
                        Motor(-1, 1); delay(10);
                        Motor(0, 0); delay(10);
                        break; 
                    }
                    else {
                        Motor(-15, -15); delay(20);
                        Motor(1, 1); delay(10);
                        Motor(0, 0); delay(10);
                        break;
                    }
                }
                else {
                    Motor(motor_slow, motor_slow);
                }
            }   
            encoder.resetEncoders();
            do{Motor(-20, -20);}while(encoder.Poss_L() > -fw_to_rotate);
            Motor(20, 20); delay(20);
            Motor(-1, -1);
            delay(10);             
            break;                  
        }
    }

    // พอถึงระยะ ต้องตรวจเส้นหรือไม่
    if (_line == "line") {
        while (1) {
            Motor(motor_slow, motor_slow);

            if (mcp_f(0) < md_mcp_f(0) - 50 && mcp_f(3) > md_mcp_f(3)) {                    
                Motor(40, -10);
            }
            else if (mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3) - 50 ){
                Motor(-10, 40);
            }
            else if (mcp_f(1) < md_mcp_f(1) - 50 || mcp_f(2) < md_mcp_f(2) - 50) {
                Motor(-30, -30); delay(30);  
                Motor(-1, -1); delay(10);  

                while (1) {
                    if (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) > md_mcp_f(2)) {
                        lr = 'l';
                        Motor(-10, 30);        
                    }
                    else if (mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2)- 50) {
                        lr = 'r';
                        Motor(30, -10);           
                    }
                    else if ((mcp_f(0) < md_mcp_f(0) - 50&& mcp_f(3) < md_mcp_f(3)- 50)
                             || (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) < md_mcp_f(2)- 50)) {
                        if (lr == 'l') {
                            Motor(15, -15); delay(20);
                            Motor(1, -1); delay(10);
                            Motor(0, 0); delay(10);
                            break; 
                        }
                        if (lr == 'r') {
                            Motor(-15, 15); delay(20);
                            Motor(-1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break; 
                        }
                        else {
                            Motor(-15, -15); delay(20);
                            Motor(1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break;
                        }
                    }
                    else {
                        Motor(motor_slow, motor_slow);
                    }
                }                  
                break;                  
            }                  
        }
        lines = true;
    } 
    else {
        Motor(-1, -1);
        delay(50);
        lines = false;
    }

    sett_f = false;
    set_bb = false;
}


void fw(int spl, int spr, float kps, int targetDistanceCm, String _line, int positions) 
{  
    char lr;
    encoder.resetEncoders();
    lines_fw = true;  
    lines_bw = false;  

    if (set_bb == true) {
        targetDistanceCm = targetDistanceCm + 10;
        set_bb = false;
    }

    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซต Motor และ Gyro
    Motor(-1, -1); delay(10);
    my.resetAngles();

    // *** เริ่มต้นตั้งค่าตัวแปรสำหรับ PID ***
    float yaw_offset = my.gyro('z'); // << เก็บค่าตอนเริ่มต้น
    _integral = 0;
    _prevErr = 0;
    prevT = millis();

    // เตรียมตัวสำหรับการเร่งช้าๆ
    float rampUpDistance = targetPulses * 0.2;   // ช่วงเร่ง 20% แรก
    float rampDownDistance = targetPulses * 0.8; // ช่วงเริ่มผ่อน 20% ท้าย
    int minSpeed = 20; // กำหนดสปีดขั้นต่ำ
    int maxLeftSpeed = spl;
    int maxRightSpeed = spr;

    lastTime = millis();  
    while (true) {
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว
        float currentPulses = (leftPulses + rightPulses) / 2;
        float remainingPulses = targetPulses - currentPulses;

        // อ่านเวลา
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; // ป้องกันหาร 0
        prevT = now;

        // อ่าน Gyro และคำนวณ Error
        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        // PID
        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kps * err + Kii * _integral + Kdd * deriv;

        // คำนวณสปีดพื้นฐาน
        int baseLeftSpeed = maxLeftSpeed;
        int baseRightSpeed = maxRightSpeed;

        // ทำ Ramp-up และ Ramp-down
        if (currentPulses < rampUpDistance) {
            float rampFactor = currentPulses / rampUpDistance;
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }
        else if (currentPulses > rampDownDistance) {
            float rampFactor = (targetPulses - currentPulses) / (targetPulses - rampDownDistance);
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }

        // สั่งมอเตอร์ โดยชดเชย PID correction
        int leftSpeed = constrain(baseLeftSpeed - corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed + corr, -100, 100);

        Motor(leftSpeed, rightSpeed);
        if (currentPulses >= targetPulses) 
          {
            break;
          }

        if(positions > 0)
          {
            servo(29, 180);
          }
        else
          {
            servo(29, 0);
          }        
        if (millis() - lastTime >= abs(positions) || digitalRead(20)==1) 
          {
              servo(29, 90);
              
          }

        if (mcp_f(0) < md_mcp_f(0)-30 && mcp_f(3) > md_mcp_f(3)) 
          {
            Motor(leftSpeed, rightSpeed/2);
            my.resetAngles();
          } 
        else if (mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-30) 
          {
            Motor(leftSpeed/2, rightSpeed);
            my.resetAngles();
          }
        else if (mcp_f(1) < md_mcp_f(1) - 50 || mcp_f(2) < md_mcp_f(2) - 50) 
          {
            Motor(-30, -30); delay(30);  
            Motor(-1, -1); delay(10);  

            while (1) {
                if (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) > md_mcp_f(2)) {
                    lr = 'l';
                    Motor(-10, 30);        
                }
                else if (mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2)- 50) {
                    lr = 'r';
                    Motor(30, -10);           
                }
                else if ((mcp_f(0) < md_mcp_f(0) - 50&& mcp_f(3) < md_mcp_f(3)- 50)
                         || (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) < md_mcp_f(2)- 50)) {
                    if (lr == 'l') {
                        Motor(15, -15); delay(20);
                        Motor(1, -1); delay(10);
                        Motor(0, 0); delay(10);
                        break; 
                    }
                    if (lr == 'r') {
                        Motor(-15, 15); delay(20);
                        Motor(-1, 1); delay(10);
                        Motor(0, 0); delay(10);
                        break; 
                    }
                    else {
                        Motor(-15, -15); delay(20);
                        Motor(1, 1); delay(10);
                        Motor(0, 0); delay(10);
                        break;
                    }
                }
                else {
                    Motor(motor_slow, motor_slow);
                }
            }   
            encoder.resetEncoders();
            do{Motor(-20, -20);}while(encoder.Poss_L() > -fw_to_rotate);
            Motor(20, 20); delay(20);
            Motor(-1, -1);
            delay(10);             
            break;                  
        }
    }

    // พอถึงระยะ ต้องตรวจเส้นหรือไม่
    if (_line == "line") {
        while (1) {
            Motor(motor_slow, motor_slow);

            if (mcp_f(0) < md_mcp_f(0) - 50 && mcp_f(3) > md_mcp_f(3)) {                    
                Motor(40, -10);
            }
            else if (mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3) - 50 ){
                Motor(-10, 40);
            }
            else if (mcp_f(1) < md_mcp_f(1) - 50 || mcp_f(2) < md_mcp_f(2) - 50) {
                Motor(-30, -30); delay(30);  
                Motor(-1, -1); delay(10);  

                while (1) {
                    if (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) > md_mcp_f(2)) {
                        lr = 'l';
                        Motor(-10, 30);        
                    }
                    else if (mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2)- 50) {
                        lr = 'r';
                        Motor(30, -10);           
                    }
                    else if ((mcp_f(0) < md_mcp_f(0) - 50&& mcp_f(3) < md_mcp_f(3)- 50)
                             || (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) < md_mcp_f(2)- 50)) {
                        if (lr == 'l') {
                            Motor(15, -15); delay(20);
                            Motor(1, -1); delay(10);
                            Motor(0, 0); delay(10);
                            break; 
                        }
                        if (lr == 'r') {
                            Motor(-15, 15); delay(20);
                            Motor(-1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break; 
                        }
                        else {
                            Motor(-15, -15); delay(20);
                            Motor(1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break;
                        }
                    }
                    else {
                        Motor(motor_slow, motor_slow);
                    }
                }                  
                break;                  
            }                  
        }
        lines = true;
    } 
    else {
        Motor(-1, -1);
        delay(50);
        lines = false;
    }

    sett_f = false;
    set_bb = false;
}


void bw(int spl, int spr, float kps, int targetDistanceCm, String _line) 
  {
    // เริ่มต้นตั้งค่า
    char lr;
    float Kp = kps;
    float Ki = 0.0;
    float Kd = 0.0;

    lines_fw = false;   
    lines_bw = true;

    if (set_bb == true) {
        targetDistanceCm += 15;
        set_bb = false;
    }

    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซตมอเตอร์และ Gyro
    Motor(1, 1); delay(10);
    my.resetAngles();

    // ตั้งค่าตัวแปร PID
    float yaw_offset = my.gyro('z');
    _integral = 0;
    _prevErr = 0;
    prevT = millis();

    // กำหนดช่วงเร่งและผ่อน
    float rampUpDistance = targetPulses * 0.2;
    float rampDownDistance = targetPulses * 0.8;
    int minSpeed = 20;
    int maxLeftSpeed = spl;
    int maxRightSpeed = spr;

    while (true) {
        // อ่าน encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        float currentPulses = (leftPulses + rightPulses) / 2;
        float remainingPulses = targetPulses - abs(currentPulses);

        // เวลา
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001;
        prevT = now;

        // PID จาก Gyro
        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;

        float corr = Kp * err + Ki * _integral + Kd * deriv;

        // Ramp-up / Ramp-down
        int baseLeftSpeed = maxLeftSpeed;
        int baseRightSpeed = maxRightSpeed;

        if (abs(currentPulses) < rampUpDistance) {
            float rampFactor = abs(currentPulses) / rampUpDistance;
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }
        else if (abs(currentPulses) > rampDownDistance) {
            float rampFactor = (targetPulses - abs(currentPulses)) / (targetPulses - rampDownDistance);
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }

        // ส่งค่ามอเตอร์ ชดเชย PID
        int leftSpeed = constrain(baseLeftSpeed + corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed - corr, -100, 100);
        Motor(-leftSpeed, -rightSpeed);

        // ตรวจว่าเดินถึงระยะหรือยัง
        if (abs(currentPulses) >= targetPulses) {
            break;
        }

        // ตรวจ MCP (ปรับทิศ)
        if (mcp_f(4) < md_mcp_f(4) - 30 && mcp_f(7) > md_mcp_f(7)) {
            Motor(-leftSpeed / 2, -rightSpeed);
            my.resetAngles();
        } 
        else if (mcp_f(4) > md_mcp_f(4) && mcp_f(7) < md_mcp_f(7) - 30) {
            Motor(-leftSpeed, -rightSpeed / 2);
            my.resetAngles();
        }
    }

    // หยุดมอเตอร์หลังหลุดจาก while
    Motor(0, 0); delay(10);

    // ถ้าต้องจับเส้นต่อ
    if (_line == "line") {
        while (1) {
            if (mcp_f(4) < md_mcp_f(4) - 30 && mcp_f(7) > md_mcp_f(7)) {
                Motor(10, -40);
            }
            else if (mcp_f(4) > md_mcp_f(4) && mcp_f(7) < md_mcp_f(7) - 30) {
                Motor(-40, 10);
            }
            else if (mcp_f(5) < md_mcp_f(5) || mcp_f(6) < md_mcp_f(6)) {
                while (1) {
                    if (mcp_f(5) < md_mcp_f(5) && mcp_f(6) > md_mcp_f(6)) {
                        Motor(-45, 10);
                    }
                    else if (mcp_f(5) > md_mcp_f(5) && mcp_f(6) < md_mcp_f(6)) {
                        Motor(10, -45);
                    }
                    else {
                        Motor(20, 20); delay(10);
                        Motor(-1, -1); delay(100);
                        Motor(0, 0); delay(10);
                        break;
                    }
                }
                break;
            }
            else {
                Motor(-motor_slow, -motor_slow);
            }
        }

        lines = true;
        delay(10);
        Motor(1, 1); delay(10);
    } 
    else {
        Motor(1, 1); delay(20);
        lines = false;
    }

    sett_f = false;
    set_bb = false;
}

void bw(int spl, int spr, float kps, int targetDistanceCm, String _line, int positions) 
  {
    // เริ่มต้นตั้งค่า
    char lr;
    float Kp = kps;
    float Ki = 0.0;
    float Kd = 0.0;

    lines_fw = false;   
    lines_bw = true;

    if (set_bb == true) {
        targetDistanceCm += 15;
        set_bb = false;
    }

    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซตมอเตอร์และ Gyro
    Motor(1, 1); delay(10);
    my.resetAngles();

    // ตั้งค่าตัวแปร PID
    float yaw_offset = my.gyro('z');
    _integral = 0;
    _prevErr = 0;
    prevT = millis();

    // กำหนดช่วงเร่งและผ่อน
    float rampUpDistance = targetPulses * 0.2;
    float rampDownDistance = targetPulses * 0.8;
    int minSpeed = 20;
    int maxLeftSpeed = spl;
    int maxRightSpeed = spr;

    lastTime = millis();  
    while (true) {
        // อ่าน encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        float currentPulses = (leftPulses + rightPulses) / 2;
        float remainingPulses = targetPulses - abs(currentPulses);

        // เวลา
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001;
        prevT = now;

        // PID จาก Gyro
        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;

        float corr = Kp * err + Ki * _integral + Kd * deriv;

        // Ramp-up / Ramp-down
        int baseLeftSpeed = maxLeftSpeed;
        int baseRightSpeed = maxRightSpeed;

        if (abs(currentPulses) < rampUpDistance) {
            float rampFactor = abs(currentPulses) / rampUpDistance;
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }
        else if (abs(currentPulses) > rampDownDistance) {
            float rampFactor = (targetPulses - abs(currentPulses)) / (targetPulses - rampDownDistance);
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }

        // ส่งค่ามอเตอร์ ชดเชย PID
        int leftSpeed = constrain(baseLeftSpeed + corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed - corr, -100, 100);
        Motor(-leftSpeed, -rightSpeed);

        // ตรวจว่าเดินถึงระยะหรือยัง
        if (abs(currentPulses) >= targetPulses) {
            break;
        }

        if(positions > 0)
          {
            servo(29, 180);
          }
        else
          {
            servo(29, 0);
          }        
        if (millis() - lastTime >= abs(positions) || digitalRead(20)==1) 
          {
              servo(29, 90);
              
          }

        // ตรวจ MCP (ปรับทิศ)
        if (mcp_f(4) < md_mcp_f(4) - 30 && mcp_f(7) > md_mcp_f(7)) {
            Motor(-leftSpeed / 2, -rightSpeed);
            my.resetAngles();
        } 
        else if (mcp_f(4) > md_mcp_f(4) && mcp_f(7) < md_mcp_f(7) - 30) {
            Motor(-leftSpeed, -rightSpeed / 2);
            my.resetAngles();
        }
    }

    // หยุดมอเตอร์หลังหลุดจาก while
    Motor(0, 0); delay(10);

    // ถ้าต้องจับเส้นต่อ
    if (_line == "line") {
        while (1) {
            if (mcp_f(4) < md_mcp_f(4) - 30 && mcp_f(7) > md_mcp_f(7)) {
                Motor(10, -40);
            }
            else if (mcp_f(4) > md_mcp_f(4) && mcp_f(7) < md_mcp_f(7) - 30) {
                Motor(-40, 10);
            }
            else if (mcp_f(5) < md_mcp_f(5) || mcp_f(6) < md_mcp_f(6)) {
                while (1) {
                    if (mcp_f(5) < md_mcp_f(5) && mcp_f(6) > md_mcp_f(6)) {
                        Motor(-45, 10);
                    }
                    else if (mcp_f(5) > md_mcp_f(5) && mcp_f(6) < md_mcp_f(6)) {
                        Motor(10, -45);
                    }
                    else {
                        Motor(20, 20); delay(10);
                        Motor(-1, -1); delay(100);
                        Motor(0, 0); delay(10);
                        break;
                    }
                }
                break;
            }
            else {
                Motor(-motor_slow, -motor_slow);
            }
        }

        lines = true;
        delay(10);
        Motor(1, 1); delay(10);
    } 
    else {
        Motor(1, 1); delay(20);
        lines = false;
    }

    sett_f = false;
    set_bb = false;
}


void fw_chopsticks(int spl, int spr, float kps, int targetDistanceCm, String _line) 
  {  
    chopsticks  = true;
    char lr ;
   
    encoder.resetEncoders();
    lines_fw = true;  
    lines_bw = false;  
    if(set_bb == true)
      {
        targetDistanceCm = targetDistanceCm + 10;
        set_bb = false;
      }   
      // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;
      // รีเซต Encoder และ Gyro
    Motor(0,0);delay(50);
    my.resetAngles();
       // *** เริ่มต้นตั้งค่าตัวแปรสำหรับ PID ***
    float yaw_offset = my.gyro('z'); // << เก็บค่าตอนเริ่มต้น
    _integral = 0;
    _prevErr = 0;
    prevT = millis();
   
   
      while (true) {
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();
        
        // คำนวณระยะทางที่เคลื่อนที่แล้ว (เฉลี่ยจากล้อซ้ายและขวา)
        float currentPulses = (leftPulses + rightPulses) / 2;
        float remainingPulses = targetPulses - currentPulses;
    
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) return;
        prevT = now;

        float yaw = my.gyro('z');
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;

        float corr = kps*err + Kii*_integral + Kdd*deriv;

        int leftSpeed  = constrain(spl - corr, -100, 100);
        int rightSpeed = constrain(spr+ corr, -100, 100);

        Motor(leftSpeed , rightSpeed);
        Serial.println(yaw);
        if (currentPulses >= targetPulses) {
                    break;
                }   
           
     }

   if(_line == "line")
      {  
        while(1)      
           {    
              Motor(motor_slow, motor_slow);        
              if(mcp_f(0) < md_mcp_f(0)-30 && mcp_f(3) > md_mcp_f(3))
                  {                    
                    Motor(40, -10);
                  }
              else if(mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-30)
                  {
                    Motor(-10, 40);
                  }
              else if(mcp_f(1) < md_mcp_f(1) || mcp_f(2) < md_mcp_f(2))
                  {
                    Motor(-30, -30); delay(30);  
                    Motor(-1, -1);delay(10);  
                    while(1)
                      {
                        if(mcp_f(1) < md_mcp_f(1) && mcp_f(2) > md_mcp_f(2)) 
                          {
                            lr = 'l';
                             Motor(-10, 30);        
                          }
                        else if(mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2))
                          {
                            lr = 'r';
                            Motor(30, -10);           
                          }
                        else if(mcp_f(0) < md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)
                              || mcp_f(1) < md_mcp_f(1) && mcp_f(2) < md_mcp_f(2))
                          {   
                            if(lr == 'l')
                              {
                                Motor(15, -15);delay(20);
                                Motor(1, -1);delay(10);
                                Motor(0, 0); delay(10);
                                break; 
                              }
                            if(lr == 'r')
                              {
                                Motor(-15, 15);delay(20);
                                Motor(-1, 1);delay(10);
                                Motor(0, 0); delay(10);
                                break; 
                              }
                             else
                              {
                                Motor(-15, -15);delay(20);
                                Motor(1, 1);delay(10);
                                Motor(0, 0); delay(10);
                                break;
                              }            
                          }
                         else
                          {
                            Motor(motor_slow, motor_slow);
                          }
                        }                  
                      break;                  
                      
                  }                  
          }
  
        lines = true;         
                           
      } 
    else
      {
        Motor(-1, -1);
        delay(50);
        lines = false;
      }   
    sett_f = false; 
    set_bb = false;
    chopsticks  = false; 
}



void fw_bridge(int spl, int spr, float kps, int targetDistanceCm, String _line) 
{  
  char lr;
  encoder.resetEncoders();
  lines_fw = true;  
  lines_bw = false;  

  if (set_bb == true) {
      targetDistanceCm = targetDistanceCm + 10;
      set_bb = false;
  }

  // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
  float targetPulses = targetDistanceCm * pulsesPerCm;

  // รีเซต Motor และ Gyro
  Motor(-1, -1); delay(10);
  my.resetAngles();

  // *** เริ่มต้นตั้งค่าตัวแปรสำหรับ PID ***
  float yaw_offset = my.gyro('z'); // << เก็บค่าตอนเริ่มต้น
  _integral = 0;
  _prevErr = 0;
  prevT = millis();

  // เตรียมตัวสำหรับการเร่งช้าๆ
  float rampUpDistance = targetPulses * 0.2;   // ช่วงเร่ง 20% แรก
  float rampDownDistance = targetPulses * 0.8; // ช่วงเริ่มผ่อน 20% ท้าย
  int minSpeed = 20; // กำหนดสปีดขั้นต่ำ
  int maxLeftSpeed = spl;
  int maxRightSpeed = spr;

  while (true) {
      // อ่านค่าจาก Encoder
      float leftPulses = encoder.Poss_L();
      float rightPulses = encoder.Poss_R();

      // คำนวณระยะทางที่เคลื่อนที่แล้ว
      float currentPulses = (leftPulses + rightPulses) / 2;
      float remainingPulses = targetPulses - currentPulses;

      // อ่านเวลา
      unsigned long now = millis();
      float dt = (now - prevT) / 1000.0;
      if (dt <= 0) dt = 0.001; // ป้องกันหาร 0
      prevT = now;

      // อ่าน Gyro และคำนวณ Error
      float yaw = my.gyro('z') - yaw_offset;
      float err = yaw;

      // PID
      _integral += err * dt;
      float deriv = (err - _prevErr) / dt;
      _prevErr = err;
      float corr = kps * err + Kii * _integral + Kdd * deriv;

      // คำนวณสปีดพื้นฐาน
      int baseLeftSpeed = maxLeftSpeed;
      int baseRightSpeed = maxRightSpeed;

      // ทำ Ramp-up และ Ramp-down
      if (currentPulses < rampUpDistance) {
          float rampFactor = currentPulses / rampUpDistance;
          baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
          baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
      }
      else if (currentPulses > rampDownDistance) {
          float rampFactor = (targetPulses - currentPulses) / (targetPulses - rampDownDistance);
          baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
          baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
      }

      // สั่งมอเตอร์ โดยชดเชย PID correction
      int leftSpeed = constrain(baseLeftSpeed - corr, -100, 100);
      int rightSpeed = constrain(baseRightSpeed + corr, -100, 100);

      Motor(leftSpeed, rightSpeed);

     // Serial.println(yaw); // Debug ดูค่า yaw

      if (currentPulses >= targetPulses) {
          break;
        }
      if (mcp_f(0) < md_mcp_f(0)-100 && mcp_f(3) > md_mcp_f(3)) 
        {
          Motor(leftSpeed, 2);
          my.resetAngles();
        } 
      else if (mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-100 ) 
        {
          Motor(2, rightSpeed);
          my.resetAngles();
        }
      else if (mcp_f(1) < md_mcp_f(1) -100  || mcp_f(2) < md_mcp_f(2) -100 ) 
        {
          Motor(-30, -30); delay(30);  
          Motor(-1, -1); delay(10);  

          while (1) {
              if (mcp_f(1) < md_mcp_f(1)-100  && mcp_f(2) > md_mcp_f(2)) {
                  lr = 'l';
                  Motor(-10, 30);        
              }
              else if (mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2)-100 ) {
                  lr = 'r';
                  Motor(30, -10);           
              }
              else if ((mcp_f(0) < md_mcp_f(0) -100 && mcp_f(3) < md_mcp_f(3)-100 )
                       || (mcp_f(1) < md_mcp_f(1)-100  && mcp_f(2) < md_mcp_f(2)-100 )) {
                  if (lr == 'l') {
                      Motor(15, -15); delay(20);
                      Motor(1, -1); delay(10);
                      Motor(0, 0); delay(10);
                      break; 
                  }
                  if (lr == 'r') {
                      Motor(-15, 15); delay(20);
                      Motor(-1, 1); delay(10);
                      Motor(0, 0); delay(10);
                      break; 
                  }
                  else {
                      Motor(-15, -15); delay(20);
                      Motor(1, 1); delay(10);
                      Motor(0, 0); delay(10);
                      break;
                  }
              }
              else {
                  Motor(motor_slow, motor_slow);
              }
          }   
          encoder.resetEncoders();
          do{Motor(-20, -20);}while(encoder.Poss_L() > -fw_to_rotate);
          Motor(20, 20); delay(20);
          Motor(-1, -1);
          delay(10);             
          break;                  
      }
  }

  // พอถึงระยะ ต้องตรวจเส้นหรือไม่
  if (_line == "line") {
      while (1) {
          Motor(motor_slow, motor_slow);

          if (mcp_f(0) < md_mcp_f(0) - 50 && mcp_f(3) > md_mcp_f(3)) {                    
              Motor(40, -10);
          }
          else if (mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3) - 50 ){
              Motor(-10, 40);
          }
          else if (mcp_f(1) < md_mcp_f(1) - 50 || mcp_f(2) < md_mcp_f(2) - 50) {
              Motor(-30, -30); delay(30);  
              Motor(-1, -1); delay(10);  

              while (1) {
                  if (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) > md_mcp_f(2)) {
                      lr = 'l';
                      Motor(-10, 30);        
                  }
                  else if (mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2)- 50) {
                      lr = 'r';
                      Motor(30, -10);           
                  }
                  else if ((mcp_f(0) < md_mcp_f(0) - 50&& mcp_f(3) < md_mcp_f(3)- 50)
                           || (mcp_f(1) < md_mcp_f(1)- 50 && mcp_f(2) < md_mcp_f(2)- 50)) {
                      if (lr == 'l') {
                          Motor(15, -15); delay(20);
                          Motor(1, -1); delay(10);
                          Motor(0, 0); delay(10);
                          break; 
                      }
                      if (lr == 'r') {
                          Motor(-15, 15); delay(20);
                          Motor(-1, 1); delay(10);
                          Motor(0, 0); delay(10);
                          break; 
                      }
                      else {
                          Motor(-15, -15); delay(20);
                          Motor(1, 1); delay(10);
                          Motor(0, 0); delay(10);
                          break;
                      }
                  }
                  else {
                      Motor(motor_slow, motor_slow);
                  }
              }                  
              break;                  
          }                  
      }
      lines = true;
  } 
  else {
      Motor(-1, -1);
      delay(50);
      lines = false;
  }

  sett_f = false;
  set_bb = false;
}

void set_f(int _time)
  {
    sett_f = true;
    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(mcp_f(1) < md_mcp_f(1) && mcp_f(2) > md_mcp_f(2)) 
              {
                 Motor(-2, 20);        
              }
            else if(mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2))
              {
                Motor(20, -2);           
              }
            else if(mcp_f(0) < md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)
                  || mcp_f(1) < md_mcp_f(1) && mcp_f(2) < md_mcp_f(2))
              {   
                Motor(-20, -20);delay(20);
                Motor(1, 1);delay(10);
                Motor(0, 0); delay(10);
                break;            
              }
            else
              {
                Motor(10, 10); 
              }
          }
        if(i < _time-1)
          {
            Motor(-20, -20);delay(50);
          }
      }
      Motor(0, 0); delay(50);
  }
void set_b(int _time)
  {    
    set_bb = true;
    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(mcp_f(4) < md_mcp_f(7) && mcp_f(7) > md_mcp_f(7)) 
              {
                ch_lrs = 'l';
                 Motor(-20, 5);        
              }
            else if(mcp_f(4) > md_mcp_f(4) && mcp_f(7) < md_mcp_f(7))
              {
                ch_lrs = 'r';
                Motor(5, -20);           
              }
           else if(mcp_f(7) < md_mcp_f(7) && mcp_f(4) < md_mcp_f(4))
              {   
                if(ch_lrs == 'l')
                  {
                     Motor(15, 5);delay(20);
                  }
                else
                  {
                     Motor(5, 15);delay(20);
                  }
                Motor(-1, -1); delay(50);
                break;            
              }
            else
              {
                Motor(-15, -15);
              }
          }
        if(i < _time-1)
          {
            while(1)
              {
                Motor(15, 15);
                if(mcp_f(7) > md_mcp_f(7) && mcp_f(4) > md_mcp_f(4))
                  {
                    break;
                  }
              }
            while(1)
              {
                Motor(15, 15);
                if(mcp_f(5) > md_mcp_f(5) && mcp_f(6) > md_mcp_f(6))
                  {
                    Motor(-5, -5); delay(10);
                    break;
                  }
              }
          }
      }
    my.resetAngles();
      
  }


#endif
