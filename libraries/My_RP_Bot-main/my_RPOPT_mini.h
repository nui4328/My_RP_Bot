#ifndef _my_OPT_
#define _my_OPT_

#include <Wire.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0x50
#include "my_MCP3008.h"
my_MCP3008 adc;

#include "EncoderLibrary.h"
EncoderLibrary encoder(6, 7, 15, 20);
#include <Servo.h>
#define sevopico27 27
#define sevopico28 28
#define sevopico8 8
Servo servo_pico27;
Servo servo_pico28;
Servo servo_pico8;
float redius_wheel;
float new_encoder;

uint16_t sensor_F0[502];
uint16_t sensor_F1[502];
uint16_t sensor_F2[502];
uint16_t sensor_F3[502];
uint16_t sensor_F4[502];
uint16_t sensor_F5[502];
uint16_t sensor_F6[502];
uint16_t sensor_F7[502];

uint16_t readData_eep[16];
uint16_t sensor_max[8]; 
uint16_t sensor_min[8];

uint16_t max_sensor(int sensor);
uint16_t min_sensor(int sensor);
uint16_t md_sensor(int sensor);

float P, I, D, previous_I, previous_error,errors, PID_output, present_position, previous_integral; 
uint16_t numSensor = 4; 
uint16_t sensor_pin[] = {1,2,3,4};
uint16_t state_on_Line = 0;
uint16_t setpoint;
uint16_t _lastPosition;
int mt_l, mt_r;  // ความเร็วสูงสุดของมอเตอร์ซ้ายและขวา
int sl = 0;       // ความเร็วของมอเตอร์ซ้าย
int sr = 0;       // ความเร็วของมอเตอร์ขวา
bool _p = false;
int fq;

void setup_robot(void);
void add_sensor(void);
void read_eep(void);
uint16_t read_sensor(int sensor) ;

void servo(int servo,int angle)
{  
  if (servo==27)
    {
        servo_pico27.attach(sevopico27, 500, 2500);
        servo_pico27.write(angle);        
    }
  else if (servo==28)
    {
        servo_pico28.attach(sevopico28,500, 2500);
        servo_pico28.write(angle);      
    }  
  else if (servo==8)
    {
        servo_pico8.attach(sevopico8,500, 2500);
        servo_pico8.write(angle);               
    }
}

void Freq_motor(int fq_m)
   {
      fq = fq_m;
      Serial.println(fq);
   }
void setup_robot() 
   {
       analogWriteResolution(12);
       analogWriteFreq( 20000);
       pinMode(10,OUTPUT);
       pinMode(14,OUTPUT);
       pinMode(12,OUTPUT);            
       pinMode(22,OUTPUT); 
       pinMode(18,OUTPUT);
       pinMode(19,OUTPUT);
       pinMode(20,OUTPUT);
       //EEPROM.begin(2034); // initialize EEPROM with 512 bytes
       analogReadResolution(12);
 
       encoder.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
       encoder.resetEncoders();  //--------------------->> ฟังก์ชันรอก 
       EEPROM.begin(2048); // initialize EEPROM with 512 bytes
       analogReadResolution(12);     
       adc.begin(2, 4, 3, 13 );   ///adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out      
       read_eep();  //--->ดึงค่าเซนเซอร์ที่เก็บไวใน eep
       delay(50);
   }

   void bz(int st)
   {
     pinMode(21, OUTPUT);
     digitalWrite(21, 1);
     delay(st);
     digitalWrite(21, 0);
     delay(st);
   }
 void sw()
   {
     bz(100);
     bz(100);
     while(1)
       {
         if(digitalRead(9) == 0)
           {
             break;
           }
         if(digitalRead(11) == 0)
           {
             add_sensor();
           }
         for (int i = 0; i < 8; i ++) 
             {
                Serial.print(read_sensor(i));
                Serial.print("  ");
             }
             Serial.println("  ");
       }
     read_eep();
     bz(300);
   }

uint16_t read_sensor(int sensor) 
   {        
      return adc.readADC(sensor);  
   }
 
void add_sensor()
   {
       bz(100);
       for (int i = 0; i < 50; i ++)         
          {  
             
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);               
          }
       for (int i = 50; i < 100; i ++)         
          {            
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
        for (int i = 100; i < 150; i ++)         
          {  
              
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
       for (int i = 150; i < 200; i ++)         
          {  
              
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
       for (int i = 200; i < 250; i ++)         
          {  
              
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
        for (int i = 250; i < 300; i ++)         
          {  
             
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
        for (int i = 300; i < 350; i ++)         
          {  
             
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
        for (int i = 350; i < 400; i ++)         
          {  
             
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
       for (int i = 400; i < 450; i ++)         
          {  
               
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
        for (int i = 450; i < 502; i ++)         
          {  
                
             sensor_F0[i] = read_sensor(0);
             sensor_F1[i] = read_sensor(1);
             sensor_F2[i] = read_sensor(2);
             sensor_F3[i] = read_sensor(3);
             sensor_F4[i] = read_sensor(4);
             sensor_F5[i] = read_sensor(5);
             sensor_F6[i] = read_sensor(6);
             sensor_F7[i] = read_sensor(7);   
             delay(10);                
          }
        
       Serial.println(" ");
       uint16_t maxVal0 = sensor_F0[0];
       uint16_t minVal0 = sensor_F0[0];
       for (int i = 0; i < (sizeof(sensor_F0) / sizeof(sensor_F0[0])); i++) 
          {
             maxVal0 = max(sensor_F0[i],maxVal0);
             minVal0 = min(sensor_F0[i],minVal0);
             sensor_max[0] = maxVal0;
             sensor_min[0] = minVal0;        
          } 
       Serial.print("Val0 --> ");Serial.print(maxVal0);    Serial.print(" ");   Serial.println(minVal0);
 
      
       uint16_t maxVal1 = sensor_F1[0];
       uint16_t minVal1 = sensor_F1[0];
       for (int i = 0; i < (sizeof(sensor_F1) / sizeof(sensor_F1[0])); i++) 
          {
             maxVal1 = max(sensor_F1[i],maxVal1);
             minVal1 = min(sensor_F1[i],minVal1);
             sensor_max[1] = maxVal1;
             sensor_min[1] = minVal1;         
          }
       Serial.print("Val1 --> ");Serial.print(maxVal1);    Serial.print(" ");   Serial.println(minVal1);
       uint16_t maxVal2 = sensor_F2[0];
       uint16_t minVal2 = sensor_F2[0];
       for (int i = 0; i < (sizeof(sensor_F2) / sizeof(sensor_F2[0])); i++) 
          {
             maxVal2 = max(sensor_F2[i],maxVal2);
             minVal2 = min(sensor_F2[i],minVal2);
             sensor_max[2] = maxVal2;
             sensor_min[2] = minVal2;         
          }
       Serial.print("Val2 --> ");Serial.print(maxVal2);    Serial.print(" ");   Serial.println(minVal2);
 
       uint16_t maxVal3 = sensor_F3[0];
       uint16_t minVal3 = sensor_F3[0];
       for (int i = 0; i < (sizeof(sensor_F3) / sizeof(sensor_F3[0])); i++) 
          {
             maxVal3 = max(sensor_F3[i],maxVal3);
             minVal3 = min(sensor_F3[i],minVal3);
             sensor_max[3] = maxVal3;
             sensor_min[3] = minVal3;         
          }
       Serial.print("Val3 --> ");Serial.print(maxVal3);    Serial.print(" ");   Serial.println(minVal3);
       uint16_t maxVal4 = sensor_F4[0];
       uint16_t minVal4 = sensor_F4[0];
       for (int i = 0; i < (sizeof(sensor_F4) / sizeof(sensor_F4[0])); i++) 
          {
             maxVal4 = max(sensor_F4[i],maxVal4);
             minVal4 = min(sensor_F4[i],minVal4);
             sensor_max[4] = maxVal4;
             sensor_min[4] = minVal4;         
          }
       Serial.print("Val4 --> ");Serial.print(maxVal4);    Serial.print(" ");   Serial.println(minVal4);
       uint16_t maxVal5 = sensor_F5[0];
       uint16_t minVal5 = sensor_F5[0];
       for (int i = 0; i < (sizeof(sensor_F5) / sizeof(sensor_F5[0])); i++) 
          {
             maxVal5 = max(sensor_F5[i],maxVal5);
             minVal5 = min(sensor_F5[i],minVal5);
             sensor_max[5] = maxVal5;
             sensor_min[5] = minVal5;         
          }
       Serial.print("Val5 --> ");Serial.print(maxVal5);    Serial.print(" ");   Serial.println(minVal5);
       uint16_t maxVal6 = sensor_F6[0];
       uint16_t minVal6 = sensor_F6[0];
       for (int i = 0; i < (sizeof(sensor_F6) / sizeof(sensor_F6[0])); i++) 
          {
             maxVal6 = max(sensor_F6[i],maxVal6);
             minVal6 = min(sensor_F6[i],minVal6);
             sensor_max[6] = maxVal6;
             sensor_min[6] = minVal6;         
          }
       Serial.print("Val6 --> ");Serial.print(maxVal6);    Serial.print(" ");   Serial.println(minVal6);
       uint16_t maxVal7 = sensor_F7[0];
       uint16_t minVal7 = sensor_F7[0];
       for (int i = 0; i < (sizeof(sensor_F7) / sizeof(sensor_F7[0])); i++) 
          {
             maxVal7 = max(sensor_F7[i],maxVal7);
             minVal7 = min(sensor_F7[i],minVal7);
             sensor_max[7] = maxVal7;
             sensor_min[7] = minVal7;         
          }
       Serial.print("Val7 --> ");Serial.print(maxVal7);    Serial.print(" ");   Serial.println(minVal7);
 
 
       //////////////////////////////////////------------------------------>> เก็บลง EEP
 
       uint16_t Data_sensor[16] = {maxVal0, maxVal1, maxVal2, maxVal3, maxVal4, maxVal5, maxVal6, maxVal7
                              ,minVal0, minVal1, minVal2, minVal3, minVal4, minVal5, minVal6, minVal7};
       for (int i = 0; i < 16; i ++) 
         {
          Serial.print(Data_sensor[i]);Serial.print("  ");
         }
       Serial.println("  ");
       
       // การเขียนข้อมูลลง EEPROM
       for (int i = 0; i < 16; i++) {
           EEPROM.put((2 * i) + 3, Data_sensor[i]);
       }
 
       // บันทึกข้อมูลลงใน EEPROM เพียงครั้งเดียว
       EEPROM.commit(); 
 
       // การอ่านข้อมูลจาก EEPROM
       uint16_t readData_sensor[16];
       for (int i = 0; i < 16; i++) {
           EEPROM.get((2 * i) + 3, readData_sensor[i]);
       }
 
       // แสดงผลข้อมูลที่เขียนและอ่านจาก EEPROM
       Serial.print("Data written to EEPROM: ");
       for (int i = 0; i < 16; i++) {
           Serial.print(Data_sensor[i]);
           Serial.print("  ");
       }
       Serial.println();
 
       Serial.print("Data read from EEPROM: ");
       for (int i = 0; i < 16; i++) {
           Serial.print(readData_sensor[i]);
           Serial.print("  ");
       }
       Serial.println();
     bz(300); bz(100); bz(100); 
   }
 
 void read_eep()
   {    
     for (int i = 0; i < 16; i++) 
       {
         EEPROM.get((2 * i) + 3, readData_eep[i]);
       }
     Serial.print("Data read from EEPROM: ");
       for (int i = 0; i < 16; i++) {
           Serial.print(readData_eep[i]);
           Serial.print("  ");
       }
     Serial.println();
     Serial.print("sensor min: ");
       for (int i = 0; i < 8; i++) {
           Serial.print(min_sensor(i));
           Serial.print("  ");
       }
     Serial.println();
     Serial.print("sensor max: ");
       for (int i = 0; i < 8; i++) {
           Serial.print(max_sensor(i));
           Serial.print("  ");
       }
       Serial.println();
     Serial.print("sensor md: ");
       for (int i = 0; i < 8; i++) {
           Serial.print(md_sensor(i));
           Serial.print("  ");
       }
       Serial.println();
   }
 
 uint16_t max_sensor(int sensor)
     {     
        if(sensor == 0)
           {
              return readData_eep[0];
           }
        else if(sensor == 1)
           {
              return readData_eep[1];
           }
        else if(sensor == 2)
           {
              return readData_eep[2];
           }
        else if(sensor == 3)
           {
             return readData_eep[3];
           }
        else if(sensor == 4)
           {
             return readData_eep[4];
           }
       else if(sensor == 5)
           {
              return readData_eep[5];
           }
        else if(sensor == 6)
           {
              return readData_eep[6];
           }
        else if(sensor == 7)
           {
              return readData_eep[7];
           }
         return -1;    
     }
     
 uint16_t min_sensor(int sensor)
     {     
        if(sensor == 0)
           {
              return readData_eep[8];
           }
        else if(sensor == 1)
           {
              return readData_eep[9];
           }
        else if(sensor == 2)
           {
              return readData_eep[10];
           }
        else if(sensor == 3)
           {
             return readData_eep[11];
           }
        else if(sensor == 4)
           {
             return readData_eep[12];
           }
       else if(sensor == 5)
           {
              return readData_eep[13];
           }
        else if(sensor == 6)
           {
              return readData_eep[14];
           }
        else if(sensor == 7)
           {
              return readData_eep[15];
           }
         return -1;     
     }
 
 uint16_t md_sensor(int sensor)
     {      
       uint16_t md = 0;
       md = (max_sensor(sensor) + min_sensor(sensor))/2;
       return md;      
     }
 
 uint16_t Position_4()  
    {        
       uint16_t min_sensor_values_F[] = { min_sensor(1),min_sensor(2),min_sensor(3),min_sensor(4)  }; //ค่าที่อ่านได้น้อยสุดหรือ สีดำ
       uint16_t max_sensor_values_F[] = { max_sensor(1),max_sensor(2),max_sensor(3),max_sensor(4)  } ; //ค่าที่อ่านได้มากสุด สีขาว                
       bool onLine = false;
       long avg = 0;
       long sum = 0;
       for (uint8_t i = 0; i < 4 ; i++) 
           {              
               long value = map(read_sensor(sensor_pin[i]), min_sensor_values_F[i], max_sensor_values_F[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value
 
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
                  return 3000;                  
 
                }
 
           }
         _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย
 
         return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
     }
 
 float error_sensor()
     {
         present_position = Position_4() / ((numSensor - 1) * 10) ;
         setpoint = 50.0;
         errors = setpoint - present_position;           
         return errors;                             
     }
void to_set_motor_LR(int ML, int MR)
   {
     mt_l = ML;
     mt_r = MR;
   }
void Motor(int spl,int spr)    
  {  
     delayMicroseconds(50);        
     int mtl = (4095*mt_l)/100;
     int mtr = (4095*mt_r)/100;
     sl = map(spl, -100, 100, -mtl, mtl);
     sr = map(spr, -100, 100, -mtr, mtr);      

     sl = constrain(sl, -4095, 4095);
     sr = constrain(sr, -4095, 4095);
              
     if(sr>0)
         {
           digitalWrite(14,HIGH);
           digitalWrite(12,LOW);     
           analogWrite(10,sr);
        }
     else if(sr<0)
        {    
           digitalWrite(14,LOW);
           digitalWrite(12,HIGH);
           analogWrite(10,-sr);
        }
     else
        {        
           digitalWrite(12,LOW);
           digitalWrite(14,LOW);
           analogWrite(10,0);
        }  
           
     if(sl>0)
        {
           digitalWrite(19,HIGH);
           digitalWrite(18,LOW);
           analogWrite(22,sl);
        }
     else if(sl<0)
        {    
           digitalWrite(19,LOW);
           digitalWrite(18,HIGH);
           analogWrite(22,-sl);
        }
     else
        {        
           digitalWrite(18,LOW);
           digitalWrite(19,LOW);
           analogWrite(22,0);
        }              
   }

//-------------------------------------->> Moverobot
void fw_line(int sl, int sr, float kp, char sp, String sensor, int offset) 
  {
    char sensors[4];  // Declare a char array to store the converted string
    sensor.toCharArray(sensors, sizeof(sensors));
    int sensor_f = atoi(&sensors[1]); 
    int time_to_accelerate_map = map(sl, 0, 100, 50, 500);
    unsigned long start_time = millis();  // บันทึกเวลาตอนเริ่มต้น
    unsigned long time_to_accelerate = 0; 
    if(_p == true)
      {
        time_to_accelerate = 0;
      }
    else
      {
        time_to_accelerate = time_to_accelerate_map; // เวลาที่ใช้ในการเร่งความเร็ว (เช่น 1 วินาที) 
      }
    
    
    while(1) 
      {
        unsigned long elapsed_time = millis() - start_time;  // เวลาที่ผ่านไป
       float speed_factor = float(elapsed_time) / time_to_accelerate; // ค่า 0 ถึง 1 ที่แสดงถึงการเพิ่มความเร็ว
        if (speed_factor > 1.0) speed_factor = 1.0; // จำกัดไม่ให้เกิน 1.0
        
        int current_sl = int(sl * speed_factor); // คำนวณความเร็วปัจจุบัน
        int current_sr = int(sr * speed_factor); // คำนวณความเร็วปัจจุบัน
        
        int I_limit = 1000;
        if (I > I_limit) 
          {
            I = I_limit;
          } 
        else if (I < -I_limit) 
          {
            I = -I_limit;
          }
        
        delayMicroseconds(50);
        float errors = error_sensor();
        float P = errors;
        I = I + errors;
        float D = errors - previous_error;
        previous_error = errors;
        
        float PID_output = (kp * P) + (0.00001 * I) + (0.05 * D);
        
        // จำกัดการควบคุมมอเตอร์ไม่ให้เกินค่าที่กำหนด
        if (PID_output > 100) 
          {
            PID_output = 100;
          } 
        else if (PID_output < -100) 
          {
            PID_output = -100;
          }
        
        // ควบคุมมอเตอร์ให้หมุนตามค่าที่คำนวณได้
        Motor(current_sl - PID_output, current_sr + PID_output);
        
        if (sensor == "a0") 
          {
            if (read_sensor(0) < md_sensor(0)) 
              {
                break;
              }
          } 
        else if (sensor == "a5") 
          {
            if (read_sensor(5) < md_sensor(5)) 
              {
                break;
              }
          }
      }

    // ถ้าต้องการให้ทำการเปลี่ยนทิศทางหรือตรวจสอบเงื่อนไขเพิ่มเติม
    if (sp == 'p') 
      {
        _p = true;
        while (1) 
          {            
            int I_limit = 1000;
            if (I > I_limit) 
              {
                I = I_limit;
              } 
            else if (I < -I_limit) 
              {
                I = -I_limit;
              }
            
            delayMicroseconds(50);
            float errors = error_sensor();
            float P = errors;
            I = I + errors;
            float D = errors - previous_error;
            previous_error = errors;
            
            float PID_output = (kp * P) + (0.00001 * I) + (0.05 * D);
            
            // จำกัดการควบคุมมอเตอร์ไม่ให้เกินค่าที่กำหนด
            if (PID_output > 100) 
              {
                PID_output = 100;
              } 
            else if (PID_output < -100) 
              {
                PID_output = -100;
              }
            
            // ควบคุมมอเตอร์ให้หมุนตามค่าที่คำนวณได้
            Motor(sl - PID_output, sr + PID_output);
            
            if (sensor == "a0") 
              {
                if (read_sensor(0) > md_sensor(0)) 
                  {
                    break;
                  }
              } 
            else if (sensor == "a5") 
              {
                if (read_sensor(5) > md_sensor(5)) 
                  {
                    break;
                  }
              }
          }
      }
    else
      {
        _p = false;
      }
    // ถ้ามีการขยับมอเตอร์หรือเปลี่ยนทิศทางในกรณีที่ offset > 0
    if (offset > 0) {
        Motor(-sl, -sr); 
        delay(offset);
        Motor(-1, -1); 
        delay(offset);
    } else {
        // อื่นๆ
    }
}

///------------------------------------>>> Move encoder
void fw_distance(int sl, int sr, int distance1, int offset)
  {
    float circumference = 2 * 3.14 * 24;
    new_encoder = 440 * distance1 / (circumference /10) ;
    Serial.println(new_encoder);
    encoder.resetEncoders();
    do{Motor(sl, sr);}while(encoder.Poss_R() < new_encoder || encoder.Poss_L() < new_encoder);
    Motor(-sl, -sr);
    delay(10);
    Motor(1, 1);
  }
void bw_distance(int sl, int sr, int distance1, int offset)
  {
    float circumference = 2 * 3.14 * 24;
    new_encoder = 440 * distance1 / (circumference /10) ;
    Serial.println(new_encoder);
    encoder.resetEncoders();
    do{Motor(sl, sr);}while(encoder.Poss_R() < new_encoder || encoder.Poss_L() < new_encoder);
    Motor(-sl, -sr);
    delay(10);
    Motor(1, 1);
  }
void fw_distance(int sl, int sr, float kp, int distance1, int offset) {
    float circumference = 2 * 3.14 * 24;
    new_encoder = 440 * distance1 / (circumference / 10);
    Serial.println(new_encoder);
    
    float previous_error = 0;
    float I = 0;
    encoder.resetEncoders();
    
    unsigned long start_time = millis();  // บันทึกเวลาตอนเริ่มต้น
    int time_to_accelerate_map = map(sl, 0, 100, 50, 500);
    unsigned long time_to_accelerate = 0; 
    
    if(_p == true) {
        time_to_accelerate = 0;
    } else {
        time_to_accelerate = time_to_accelerate_map; // เวลาที่ใช้ในการเร่งความเร็ว (เช่น 1 วินาที) 
    }
    
    while (encoder.Poss_R() < new_encoder || encoder.Poss_L() < new_encoder) {
        unsigned long elapsed_time = millis() - start_time;  // เวลาที่ผ่านไป
        
        // คำนวณความเร็วที่ต้องการในช่วงเวลานั้น
        float speed_factor = float(elapsed_time) / time_to_accelerate; // ค่า 0 ถึง 1 ที่แสดงถึงการเพิ่มความเร็ว
        if (speed_factor > 1.0) speed_factor = 1.0; // จำกัดไม่ให้เกิน 1.0
        
        // ตรวจสอบระยะทางที่เหลือก่อนจะลดความเร็ว
        int remaining_distance = new_encoder - encoder.Poss_R(); // คำนวณระยะทางที่เหลือ
        if (remaining_distance < 3) {
            // คำนวณอัตราการลดความเร็ว (จากความเร็วเต็มที่ไปจนถึง 10)
            float speed_ratio = float(remaining_distance) / 5.0; // คำนวณอัตราการลดความเร็ว (จาก 1 ถึง 0)
            int reduced_speed = int(10 + (sl - 10) * speed_ratio);  // ความเร็วที่ลดลงตามระยะทางที่เหลือ
            sl = reduced_speed;
            sr = reduced_speed;
        }
        
        int current_sl = int(sl * speed_factor); // คำนวณความเร็วปัจจุบัน
        int current_sr = int(sr * speed_factor); // คำนวณความเร็วปัจจุบัน
        
        int I_limit = 1000;
        if (I > I_limit) {
            I = I_limit;
        } else if (I < -I_limit) {
            I = -I_limit;
        }
        
        delayMicroseconds(50);
        float errors = error_sensor();
        float P = errors;
        I = I + errors;
        float D = errors - previous_error;
        previous_error = errors;
        
        float PID_output = (kp * P) + (0.00001 * I) + (0.05 * D);
        
        // จำกัดการควบคุมมอเตอร์ไม่ให้เกินค่าที่กำหนด
        if (PID_output > 100) {
            PID_output = 100;
        } else if (PID_output < -100) {
            PID_output = -100;
        }
        
        // ควบคุมมอเตอร์ให้หมุนตามค่าที่คำนวณได้
        Motor(current_sl - PID_output, current_sr + PID_output);
    }
    
    // หลังจากเดินทางเสร็จแล้ว
    if (offset > 0) {
        Motor(-sl, -sr); delay(offset);
        Motor(-1, -1); delay(offset);
    } else {
        // อื่นๆ
    }
}


void bw_distance(int sl, int sr, float kp, int distance1, int offset) {
    float circumference = 2 * 3.14 * 24;
    new_encoder = 440 * distance1 / (circumference / 10);
    Serial.println(new_encoder);
    
    float previous_error = 0;
    float I = 0;
    encoder.resetEncoders();
    
    unsigned long start_time = millis();  // บันทึกเวลาตอนเริ่มต้น
    int time_to_accelerate_map = map(sl, 0, 100, 50, 500);
    unsigned long time_to_accelerate = 0; 
    if(_p == true)
      {
        time_to_accelerate = 0;
      }
    else
      {
        time_to_accelerate = time_to_accelerate_map; // เวลาที่ใช้ในการเร่งความเร็ว (เช่น 1 วินาที) 
      }
    
    while (encoder.Poss_R() > -new_encoder || encoder.Poss_L() > -new_encoder) {
        unsigned long elapsed_time = millis() - start_time;  // เวลาที่ผ่านไป
        
        // คำนวณความเร็วที่ต้องการในช่วงเวลานั้น
        float speed_factor = float(elapsed_time) / time_to_accelerate; // ค่า 0 ถึง 1 ที่แสดงถึงการเพิ่มความเร็ว
        if (speed_factor > 1.0) speed_factor = 1.0; // จำกัดไม่ให้เกิน 1.0
        
        int current_sl = int(sl * speed_factor); // คำนวณความเร็วปัจจุบัน
        int current_sr = int(sr * speed_factor); // คำนวณความเร็วปัจจุบัน
        
        int I_limit = 1000;
        if (I > I_limit) {
            I = I_limit;
        } else if (I < -I_limit) {
            I = -I_limit;
        }
        float error_L = map(read_sensor(6), min_sensor(6), max_sensor(6), 0, 30 );
        float error_R = map(read_sensor(7), min_sensor(7), max_sensor(7), 0, 30 );
        delayMicroseconds(50);
        float errors = error_L - error_R; 
        float P = errors;
        I = I + errors;
        float D = errors - previous_error;
        previous_error = errors;
        
        float PID_output = (kp * P) + (0.00001 * I) + (0.05 * D);
        
        // จำกัดการควบคุมมอเตอร์ไม่ให้เกินค่าที่กำหนด
        if (PID_output > 100) {
            PID_output = 100;
        } else if (PID_output < -100) {
            PID_output = -100;
        }
        
        // ควบคุมมอเตอร์ให้หมุนตามค่าที่คำนวณได้
        Motor(-(current_sl - PID_output), -(current_sr + PID_output));
    }
    
    // หลังจากเดินทางเสร็จแล้ว
    if (offset > 0) {
        Motor(sl, sr); delay(offset);
        Motor(1, 1); delay(offset);
    } else {
        // อื่นๆ
    }
}

void fw_2sensor(int sl, int sr, float kp, int distance1, int offset) {
    float circumference = 2 * 3.14 * 24;
    new_encoder = 440 * distance1 / (circumference / 10);
    Serial.println(new_encoder);
    
    float previous_error = 0;
    float I = 0;
    encoder.resetEncoders();
    
    unsigned long start_time = millis();  // บันทึกเวลาตอนเริ่มต้น
    int time_to_accelerate_map = map(sl, 0, 100, 50, 500);
    unsigned long time_to_accelerate = 0; 
    if(_p == true)
      {
        time_to_accelerate = 0;
      }
    else
      {
        time_to_accelerate = time_to_accelerate_map; // เวลาที่ใช้ในการเร่งความเร็ว (เช่น 1 วินาที) 
      }
    
    while (encoder.Poss_R() > -new_encoder || encoder.Poss_L() > -new_encoder) {
        unsigned long elapsed_time = millis() - start_time;  // เวลาที่ผ่านไป
        
        // คำนวณความเร็วที่ต้องการในช่วงเวลานั้น
        float speed_factor = float(elapsed_time) / time_to_accelerate; // ค่า 0 ถึง 1 ที่แสดงถึงการเพิ่มความเร็ว
        if (speed_factor > 1.0) speed_factor = 1.0; // จำกัดไม่ให้เกิน 1.0
        
        int current_sl = int(sl * speed_factor); // คำนวณความเร็วปัจจุบัน
        int current_sr = int(sr * speed_factor); // คำนวณความเร็วปัจจุบัน
        
        int I_limit = 1000;
        if (I > I_limit) {
            I = I_limit;
        } else if (I < -I_limit) {
            I = -I_limit;
        }
        float error_L = map(read_sensor(2), min_sensor(2), max_sensor(2), 0, 50 );
        float error_R = map(read_sensor(4), min_sensor(4), max_sensor(4), 0, 50 );
        delayMicroseconds(50);
        float errors = error_L - error_R; 
        float P = errors;
        I = I + errors;
        float D = errors - previous_error;
        previous_error = errors;
        Serial.println(errors);
        float PID_output = (kp * P) + (0.00001 * I) + (0.05 * D);
        
        // จำกัดการควบคุมมอเตอร์ไม่ให้เกินค่าที่กำหนด
        if (PID_output > 100) {
            PID_output = 100;
        } else if (PID_output < -100) {
            PID_output = -100;
        }
        
        // ควบคุมมอเตอร์ให้หมุนตามค่าที่คำนวณได้
        Motor(current_sl + PID_output, current_sr - PID_output);
        /*
        if(read_sensor(0) < md_sensor(0))
          {
            do{ Motor(-10, 40); delayMicroseconds(50);} while( read_sensor(5) > md_sensor(5) ); 
            delayMicroseconds(50);
            Motor(10, -40);delay(30);
            Motor(-1, -1);delay(30000000);
          }
        */
    }
    
    // หลังจากเดินทางเสร็จแล้ว
    if (offset > 0) {
        Motor(sl, sr); delay(offset);
        Motor(1, 1); delay(offset);
    } else {
        // อื่นๆ
    }
}

///-------------------------------->>> Move left 
void turn_left_wheel(int distance1, int speed, float angle, int offset) {
   // คำนวณจำนวน encoder ticks ที่ต้องการสำหรับการหมุนที่ระยะ angle
   float circumference = 2 * 3.14 * 24; // ความยาวรอบวงล้อ (24 มม. เป็นขนาดล้อสมมุติ)
   int encoder_ticks_per_rev = 1000; // จำนวน encoder ticks ต่อการหมุนหนึ่งรอบ
   float distance_per_degree = circumference / 360; // ระยะทางที่เคลื่อนที่ในแต่ละองศา
   int ticks_to_turn = angle * distance_per_degree / (circumference / encoder_ticks_per_rev); // คำนวณจำนวน encoder ticks สำหรับการหมุน angle
   
   new_encoder = 360 * distance1 / (circumference / 10);
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < new_encoder || encoder.Poss_R() < new_encoder) {
       Motor(20, 20); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
   }
   Motor(-1, -1); delay(20); // หยุดมอเตอร์เมื่อหมุนเสร็จ
   // หมุนไปทางซ้าย (ความเร็ว -speed สำหรับล้อซ้าย, +speed สำหรับล้อขวา)
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < ticks_to_turn && encoder.Poss_R() < ticks_to_turn) {
       Motor(-speed, speed); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
   }
   Motor(speed/2, -speed/2); delay(offset); 
   Motor(-1, -1); delay(offset); // หยุดมอเตอร์เมื่อหมุนเสร็จ
}

void turn_left_wheel(int distance1, int speed, String sensor, int offset) 
 {
   char sensors[4];  // Declare a char array to store the converted string
   sensor.toCharArray(sensors, sizeof(sensors));
   int sensor_f = atoi(&sensors[1]); 
   // คำนวณจำนวน encoder ticks ที่ต้องการสำหรับการหมุนที่ระยะ angle
   float circumference = 2 * 3.14 * 24; // ความยาวรอบวงล้อ (24 มม. เป็นขนาดล้อสมมุติ)    int ticks_to_turn = angle * distance_per_degree / (circumference / encoder_ticks_per_rev); // คำนวณจำนวน encoder ticks สำหรับการหมุน angle
   
   new_encoder = 360 * distance1 / (circumference / 10);
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < new_encoder || encoder.Poss_R() < new_encoder) 
     {
       Motor(20, 20); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
     }
   Motor(-1, -1); delay(20); // หยุดมอเตอร์เมื่อหมุนเสร็จ
   Motor(-speed, speed);delay(25);
   for ( int i = 0; i <= sensor_f; i++ )
     {
       do{ Motor(-speed, speed); delayMicroseconds(50);} while( read_sensor(i) > md_sensor(i) ); 
       delayMicroseconds(50);
     } 

   Motor(speed/2, -speed/2); delay(offset); 
   Motor(-1, -1); delay(offset); // หยุดมอเตอร์เมื่อหมุนเสร็จ
}

void turn_left_sensor(int distance1, int speed, float angle, int offset) {
   // ขนาดรอบวงของล้อ
   float circumference = 2 * 3.14 * 24;  // ขนาดล้อ 24 มม. (ตัวอย่าง)    
   // จำนวน encoder ticks ต่อการหมุน 1 รอบ
   int encoder_ticks_per_rev = 800;  // ตัวอย่างค่า encoder ticks ต่อการหมุน 1 รอบ (ขึ้นอยู่กับ encoder ของคุณ)
   // คำนวณจำนวน ticks ที่ต้องการสำหรับการหมุนที่ระยะ angle
   int ticks_for_angle = encoder_ticks_per_rev * angle / 180;  // คำนวณ ticks สำหรับการหมุน angle องศา

   new_encoder = 360 * distance1 / (circumference / 10);
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < new_encoder || encoder.Poss_R() < new_encoder) {
       Motor(20, 20); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
   }
   Motor(-1, -1); delay(20); // หยุดมอเตอร์เมื่อหมุนเสร็จ
   encoder.resetEncoders();
   Motor(-15, speed); delay(25);
       while (encoder.Poss_R() < ticks_for_angle) {
           Motor(-15, speed);  // หมุนมอเตอร์ซ้ายไปข้างหน้า, ขวาหยุด
           delayMicroseconds(50);
       }
  
   Motor(15, -(speed/2)); delay(offset); 
   Motor(-1, -1); delay(10); // หยุดมอเตอร์เมื่อหมุนเสร็จ
}

void turn_left_sensor(int distance1, int speed, String sensor, int offset) {
   char sensors[4];  // Declare a char array to store the converted string
   sensor.toCharArray(sensors, sizeof(sensors));
   int sensor_f = atoi(&sensors[1]); 
   // คำนวณจำนวน encoder ticks ที่ต้องการสำหรับการหมุนที่ระยะ angle
   float circumference = 2 * 3.14 * 24; // ความยาวรอบวงล้อ (24 มม. เป็นขนาดล้อสมมุติ)    int ticks_to_turn = angle * distance_per_degree / (circumference / encoder_ticks_per_rev); // คำนวณจำนวน encoder ticks สำหรับการหมุน angle
   
   new_encoder = 360 * distance1 / (circumference / 10);
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < new_encoder || encoder.Poss_R() < new_encoder) 
     {
       Motor(20, 20); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
     }
   Motor(-1, -1); delay(20); // หยุดมอเตอร์เมื่อหมุนเสร็จ
   Motor(-15, speed); delay(25);
       do{ Motor(-15, speed);  delayMicroseconds(50);} while( read_sensor(sensor_f) > md_sensor(sensor_f) ); 
       delayMicroseconds(50);
       
   Motor(15, -(speed/2)); delay(offset); 
   Motor(-1, -1); delay(10); // หยุดมอเตอร์เมื่อหมุนเสร็จ
}

//---------------------------------------------------------->>  Move right

void turn_right_wheel(int distance1, int speed, float angle, int offset) {
   // คำนวณจำนวน encoder ticks ที่ต้องการสำหรับการหมุนที่ระยะ angle
   float circumference = 2 * 3.14 * 24; // ความยาวรอบวงล้อ (24 มม. เป็นขนาดล้อสมมุติ)
   int encoder_ticks_per_rev = 1000; // จำนวน encoder ticks ต่อการหมุนหนึ่งรอบ
   float distance_per_degree = circumference / 360; // ระยะทางที่เคลื่อนที่ในแต่ละองศา
   int ticks_to_turn = angle * distance_per_degree / (circumference / encoder_ticks_per_rev); // คำนวณจำนวน encoder ticks สำหรับการหมุน angle
   
   new_encoder = 360 * distance1 / (circumference / 10);
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < new_encoder || encoder.Poss_R() < new_encoder) {
       Motor(20, 20); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
   }
   Motor(-1, -1); delay(20); // หยุดมอเตอร์เมื่อหมุนเสร็จ
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน

   // หมุนไปทางขวา (ความเร็ว +speed สำหรับล้อซ้าย, -speed สำหรับล้อขวา)
   while (encoder.Poss_L() < ticks_to_turn && encoder.Poss_R() < ticks_to_turn) {
       Motor(speed, -speed); // หมุนมอเตอร์ (หมุนขวา)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
   }
   Motor(-speed/2, speed/2); delay(offset); 
   Motor(-1, -1); delay(offset); // หยุดมอเตอร์เมื่อหมุนเสร็จ
}

void turn_right_wheel(int distance1, int speed, String sensor, int offset) {
   char sensors[4];  // Declare a char array to store the converted string
   sensor.toCharArray(sensors, sizeof(sensors));
   int sensor_f = atoi(&sensors[1]); 
   // คำนวณจำนวน encoder ticks ที่ต้องการสำหรับการหมุนที่ระยะ angle
   float circumference = 2 * 3.14 * 24; // ความยาวรอบวงล้อ (24 มม. เป็นขนาดล้อสมมุติ)    int ticks_to_turn = angle * distance_per_degree / (circumference / encoder_ticks_per_rev); // คำนวณจำนวน encoder ticks สำหรับการหมุน angle
   
   new_encoder = 360 * distance1 / (circumference / 10);
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < new_encoder || encoder.Poss_R() < new_encoder) 
     {
       Motor(20, 20); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
     }
   Motor(-1, -1); delay(20); // หยุดมอเตอร์เมื่อหมุนเสร็จ
   Motor(speed, -speed); delay(50);
   for ( int i = 5; i >= sensor_f; i -- )
     {
       do{ Motor(speed, -speed); delayMicroseconds(50);} while( read_sensor(i) > md_sensor(i) ); 
       delayMicroseconds(50);
     } 

   Motor(-speed/2, speed/2); delay(offset); 
   Motor(-1, -1); delay(offset); // หยุดมอเตอร์เมื่อหมุนเสร็จ
}


void turn_right_sensor(int distance1, int speed, float angle, int offset) {
   // ขนาดรอบวงของล้อ
   float circumference = 2 * 3.14 * 24;  // ขนาดล้อ 24 มม. (ตัวอย่าง)    
   // จำนวน encoder ticks ต่อการหมุน 1 รอบ
   int encoder_ticks_per_rev = 800;  // ตัวอย่างค่า encoder ticks ต่อการหมุน 1 รอบ (ขึ้นอยู่กับ encoder ของคุณ)
   // คำนวณจำนวน ticks ที่ต้องการสำหรับการหมุนที่ระยะ angle
   int ticks_for_angle = encoder_ticks_per_rev * angle / 180;  // คำนวณ ticks สำหรับการหมุน angle องศา

   new_encoder = 360 * distance1 / (circumference / 10);
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < new_encoder || encoder.Poss_R() < new_encoder) {
       Motor(20, 20); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
   }
   Motor(-1, -1); delay(20); // หยุดมอเตอร์เมื่อหมุนเสร็จ
   encoder.resetEncoders();
   
       while (encoder.Poss_L() < ticks_for_angle) {
           Motor(speed,-15 );  // หมุนมอเตอร์ซ้ายไปข้างหน้า, ขวาหยุด
           delayMicroseconds(50);
       }
  
   Motor(-(speed/2),15 ); delay(offset); 
   Motor(-1, -1); delay(10); // หยุดมอเตอร์เมื่อหมุนเสร็จ
}

void turn_right_sensor(int distance1, int speed, String sensor, int offset) {
   char sensors[4];  // Declare a char array to store the converted string
   sensor.toCharArray(sensors, sizeof(sensors));
   int sensor_f = atoi(&sensors[1]); 
   // คำนวณจำนวน encoder ticks ที่ต้องการสำหรับการหมุนที่ระยะ angle
   float circumference = 2 * 3.14 * 24; // ความยาวรอบวงล้อ (24 มม. เป็นขนาดล้อสมมุติ)    int ticks_to_turn = angle * distance_per_degree / (circumference / encoder_ticks_per_rev); // คำนวณจำนวน encoder ticks สำหรับการหมุน angle
   
   new_encoder = 360 * distance1 / (circumference / 10);
   encoder.resetEncoders(); // รีเซ็ต encoder ก่อนเริ่มหมุน
   while (encoder.Poss_L() < new_encoder || encoder.Poss_R() < new_encoder) 
     {
       Motor(20, 20); // หมุนมอเตอร์ (หมุนซ้าย)
       delayMicroseconds(50); // เวลาหน่วงเพื่อให้มอเตอร์มีเวลาหมุน
     }
   Motor(-1, -1); delay(20); // หยุดมอเตอร์เมื่อหมุนเสร็จ
  
       do{ Motor(speed, -15 );  delayMicroseconds(50);} while( read_sensor(sensor_f) > md_sensor(sensor_f) ); 
       delayMicroseconds(50);
      
   Motor(-(speed/2), 15); delay(offset); 
   Motor(-1, -1); delay(10); // หยุดมอเตอร์เมื่อหมุนเสร็จ
}

//---------------------------------------------------------------->>

#endif
