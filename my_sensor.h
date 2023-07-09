#ifndef _my_sensor_
#define _my_sensor_

#include <Wire.h>
#include <EEPROM.h>
#include "my_motor.h"
#include "my_MCP3008.h"

#define EEPROM_ADDR 0x50
#define DATA_sensor_SIZE 20
#define DATA_sensor_SIZE_B 16

my_MCP3008 adc;
int cal_motor_l, cal_motor_r, delay_cal;

int sensor_F0[502];
int sensor_F1[502];
int sensor_F2[502];
int sensor_F3[502];
int sensor_F4[502];
int sensor_F5[502];
int sensor_F6[502];
int sensor_F7[502];
int sensor_add_F[8];
int sensor_max[8]; 
int sensor_min[8];

int sensor_B0[502];
int sensor_B1[502];
int sensor_B2[502];
int sensor_B3[502];
int sensor_B4[502];
int sensor_B5[502];
int sensor_B6[502];
int sensor_B7[502];
int sensor_add_B[8];
int sensor_maxB[8]; 
int sensor_minB[8];

int sensor_26[502];
int sensor_27[502];
int sensor_PA[2];
int sensor_max_PA[2];
int sensor_min_PA[2];

float P, I, D, previous_I, previous_error,errors, PID_output, present_position, previous_integral; 
int numSensor = 8; 
uint16_t sensor_pin_F[] = {0,1,2,3,4,5,6,7}; 
uint16_t sensor_pin_B[] = {0,1,2,3,4,5,6,7}; 
uint16_t state_on_Line = 0;
uint16_t setpoint;
uint32_t _lastPosition;
 
///////////////////////////////------------------------------->>>

void sensor_set()
   {
      EEPROM.begin(512); // initialize EEPROM with 512 bytes
      analogReadResolution(12);
   }
void bz(int dl)
  {
    pinMode(21,OUTPUT);
    digitalWrite(21,HIGH);
    delay(dl);
    digitalWrite(21,LOW);
    delay(dl);    
  }

void sw_GP11()
  {
    pinMode(11, INPUT);
    while(digitalRead(11) == 1)
      {
        Serial.println(digitalRead(11));
      }
  }
  
void sw_GP6()
  {
    pinMode(6, INPUT);
    while(digitalRead(6) == 1)
      {
        Serial.println(digitalRead(6));
      }
  }
void rgb(int r, int g, int b)
  {
    pinMode(24, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(15, OUTPUT);
    digitalWrite(24, r);
    digitalWrite(25, g);
    digitalWrite(15, b);
  }
/////////////////////////////////////////////-------------------------->
void pos_motor_cal(int spl, int spr, int delay_calA)
  {
    cal_motor_l = spl;
    cal_motor_r = spr;
    delay_cal = delay_calA;
  }
/////////////////////////////////----------------------->>>>
int mcp_f(int sensor) 
  {     
     adc.begin(2, 4, 3, 5 );    
     adc.begin(2, 4, 3, 13 );  
     return adc.readADC(sensor);  
  }
/////////////////////////////////----------------------->>>>

int mcp_b(int sensor) 
  {   
     adc.begin(2, 4, 3, 13 );      
     adc.begin(2, 4, 3, 5 );    
     return adc.readADC(sensor);   
  } 
/////////////////////////////////----------------------->>>>  

void add_sensor_F()
  {
      bz(100);
      for (int i = 0; i < 50; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r); 
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);               
         }
      for (int i = 50; i < 100; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
       for (int i = 100; i < 150; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r)  ;  
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
      for (int i = 150; i < 200; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);  
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
      for (int i = 200; i < 250; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r);  
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
       for (int i = 250; i < 300; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r); 
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
       for (int i = 300; i < 350; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r); 
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
       for (int i = 350; i < 400; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
      for (int i = 400; i < 450; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r);   
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
       for (int i = 450; i < 502; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);    
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(delay_cal);                
         }
      Motor(1, 1); delay(100);  
      Motor(0, 0); delay(10); 
      Serial.println(" ");
      for(int i = 0; i< 502; i++)
        {
          Serial.print(sensor_F0[i]);
          Serial.println(" ");
          Serial.println(i);
        }
      bz(300);
      sw_GP11();
      bz(300);
      
      for (int i = 0; i < 50; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r);                   
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27); 
            delay(delay_cal);               
         }
      for (int i = 50; i < 100; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);                   
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27);  
            delay(delay_cal);                
         }
       for (int i = 100; i < 150; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r)  ;                
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27); 
            delay(delay_cal);                
         }
      for (int i = 150; i < 200; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);                   
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27); 
            delay(delay_cal);                
         }
      for (int i = 200; i < 250; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r);                  
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27);
            delay(delay_cal);                
         }
       for (int i = 250; i < 300; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);                   
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27);  
            delay(delay_cal);                
         }
       for (int i = 300; i < 350; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r);                  
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27);
            delay(delay_cal);                
         }
       for (int i = 350; i < 400; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);                   
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27);  
            delay(delay_cal);                
         }
      for (int i = 400; i < 450; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r);                  
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27);  
            delay(delay_cal);                
         }
       for (int i = 450; i < 502; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);                   
            sensor_26[i] = analogRead(26);
            sensor_27[i] = analogRead(27);   
            delay(delay_cal);                
         }
      Motor(1, 1); delay(100);  
      Motor(0, 0); delay(10); 
      bz(300);
      for(int i = 0; i< 502; i++)
        {
          Serial.print(sensor_26[i]);
          Serial.println(" ");
          Serial.println(i);
        }
      
      int maxVal0 = sensor_F0[0];
      int minVal0 = sensor_F0[0];
      for (int i = 0; i < (sizeof(sensor_F0) / sizeof(sensor_F0[0])); i++) 
         {
            maxVal0 = max(sensor_F0[i],maxVal0);
            minVal0 = min(sensor_F0[i],minVal0);
            sensor_max[0] = maxVal0;
            sensor_min[0] = minVal0;        
         } 
      Serial.print("Val0 --> ");Serial.print(maxVal0);    Serial.print(" ");   Serial.println(minVal0);

     
      int maxVal1 = sensor_F1[0];
      int minVal1 = sensor_F1[0];
      for (int i = 0; i < (sizeof(sensor_F1) / sizeof(sensor_F1[0])); i++) 
         {
            maxVal1 = max(sensor_F1[i],maxVal1);
            minVal1 = min(sensor_F1[i],minVal1);
            sensor_max[1] = maxVal1;
            sensor_min[1] = minVal1;         
         }
      Serial.print("Val1 --> ");Serial.print(maxVal1);    Serial.print(" ");   Serial.println(minVal1);
      int maxVal2 = sensor_F2[0];
      int minVal2 = sensor_F2[0];
      for (int i = 0; i < (sizeof(sensor_F2) / sizeof(sensor_F2[0])); i++) 
         {
            maxVal2 = max(sensor_F2[i],maxVal2);
            minVal2 = min(sensor_F2[i],minVal2);
            sensor_max[2] = maxVal2;
            sensor_min[2] = minVal2;         
         }
      Serial.print("Val2 --> ");Serial.print(maxVal2);    Serial.print(" ");   Serial.println(minVal2);

      int maxVal3 = sensor_F3[0];
      int minVal3 = sensor_F3[0];
      for (int i = 0; i < (sizeof(sensor_F3) / sizeof(sensor_F3[0])); i++) 
         {
            maxVal3 = max(sensor_F3[i],maxVal3);
            minVal3 = min(sensor_F3[i],minVal3);
            sensor_max[3] = maxVal3;
            sensor_min[3] = minVal3;         
         }
      Serial.print("Val3 --> ");Serial.print(maxVal3);    Serial.print(" ");   Serial.println(minVal3);
      int maxVal4 = sensor_F4[0];
      int minVal4 = sensor_F4[0];
      for (int i = 0; i < (sizeof(sensor_F4) / sizeof(sensor_F4[0])); i++) 
         {
            maxVal4 = max(sensor_F4[i],maxVal4);
            minVal4 = min(sensor_F4[i],minVal4);
            sensor_max[4] = maxVal4;
            sensor_min[4] = minVal4;         
         }
      Serial.print("Val4 --> ");Serial.print(maxVal4);    Serial.print(" ");   Serial.println(minVal4);
      int maxVal5 = sensor_F5[0];
      int minVal5 = sensor_F5[0];
      for (int i = 0; i < (sizeof(sensor_F5) / sizeof(sensor_F5[0])); i++) 
         {
            maxVal5 = max(sensor_F5[i],maxVal5);
            minVal5 = min(sensor_F5[i],minVal5);
            sensor_max[5] = maxVal5;
            sensor_min[5] = minVal5;         
         }
      Serial.print("Val5 --> ");Serial.print(maxVal5);    Serial.print(" ");   Serial.println(minVal5);
      int maxVal6 = sensor_F6[0];
      int minVal6 = sensor_F6[0];
      for (int i = 0; i < (sizeof(sensor_F6) / sizeof(sensor_F6[0])); i++) 
         {
            maxVal6 = max(sensor_F6[i],maxVal6);
            minVal6 = min(sensor_F6[i],minVal6);
            sensor_max[6] = maxVal6;
            sensor_min[6] = minVal6;         
         }
      Serial.print("Val6 --> ");Serial.print(maxVal6);    Serial.print(" ");   Serial.println(minVal6);
      int maxVal7 = sensor_F7[0];
      int minVal7 = sensor_F7[0];
      for (int i = 0; i < (sizeof(sensor_F7) / sizeof(sensor_F7[0])); i++) 
         {
            maxVal7 = max(sensor_F7[i],maxVal7);
            minVal7 = min(sensor_F7[i],minVal7);
            sensor_max[7] = maxVal7;
            sensor_min[7] = minVal7;         
         }
      Serial.print("Val7 --> ");Serial.print(maxVal7);    Serial.print(" ");   Serial.println(minVal7);


////////////////////////---------------------------------->>>>>>>>

      int maxVal26 = sensor_26[0];
      int minVal26 = sensor_26[0];
      for (int i = 0; i < (sizeof(sensor_26) / sizeof(sensor_26[0])); i++) 
         {
            maxVal26 = max(sensor_26[i],maxVal26);
            minVal26 = min(sensor_26[i],minVal26);
            sensor_max_PA[0] = maxVal26;
            sensor_min_PA[0] = minVal26;         
         }
      Serial.print("Val26 --> ");Serial.print(maxVal26);    Serial.print(" ");   Serial.println(minVal26);

      int maxVal27 = sensor_27[0];
      int minVal27 = sensor_27[0];
      for (int i = 0; i < (sizeof(sensor_27) / sizeof(sensor_27[0])); i++) 
         {
            maxVal27 = max(sensor_27[i],maxVal27);
            minVal27 = min(sensor_27[i],minVal27);
            sensor_max_PA[1] = maxVal27;
            sensor_min_PA[1] = minVal27;         
         }
      Serial.print("Val27 --> ");Serial.print(maxVal27);    Serial.print(" ");   Serial.println(minVal27);

      
//////////////////////////.........................>>>>>>>>>>>>

      int Data_sensor[DATA_sensor_SIZE] = {maxVal26, minVal26, maxVal27, minVal27,
                             maxVal0, maxVal1, maxVal2, maxVal3, maxVal4, maxVal5, maxVal6, maxVal7
                             ,minVal0, minVal1, minVal2, minVal3, minVal4, minVal5, minVal6, minVal7};
      EEPROM.put(0, Data_sensor); // write data to EEPROM address 0
      EEPROM.commit(); // save changes to EEPROM
    
      int readData_sensor[DATA_sensor_SIZE];
      EEPROM.get(0, readData_sensor); // read data from EEPROM address 0
      Serial.print("data26 -> EEPROM : ");
      for (int i = 0; i < 20; i ++) 
        {
         Serial.print(readData_sensor[i]);Serial.print("  ");
        }
      Serial.println("  ");
      
      bz(100);
      bz(100);
   
  }

void add_sensor_B()
  {
      bz(100);
      for (int i = 0; i < 50; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r); 
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);               
         }
      for (int i = 50; i < 100; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
       for (int i = 100; i < 150; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r)  ;  
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
      for (int i = 150; i < 200; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);  
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
      for (int i = 200; i < 250; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r);  
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
       for (int i = 250; i < 300; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r); 
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
       for (int i = 300; i < 350; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r); 
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
       for (int i = 350; i < 400; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
      for (int i = 400; i < 450; i ++)         
         {  
            Motor(cal_motor_l, cal_motor_r);   
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
       for (int i = 450; i < 502; i ++)         
         {  
            Motor(-cal_motor_l, -cal_motor_r);    
            sensor_B0[i] = mcp_b(0);
            sensor_B1[i] = mcp_b(1);
            sensor_B2[i] = mcp_b(2);
            sensor_B3[i] = mcp_b(3);
            sensor_B4[i] = mcp_b(4);
            sensor_B5[i] = mcp_b(5);
            sensor_B6[i] = mcp_b(6);
            sensor_B7[i] = mcp_b(7);   
            delay(delay_cal);                
         }
      Motor(1, 1); delay(100);  
      Motor(0, 0); delay(10); 
      Serial.println(" ");
      for(int i = 0; i< 502; i++)
        {
          Serial.print(sensor_B0[i]);
          Serial.println(" ");
          Serial.println(i);
        }
      bz(300);
     
      int maxVal0 = sensor_B0[0];
      int minVal0 = sensor_B0[0];
      for (int i = 0; i < (sizeof(sensor_B0) / sizeof(sensor_B0[0])); i++) 
         {
            maxVal0 = max(sensor_B0[i],maxVal0);
            minVal0 = min(sensor_B0[i],minVal0);
            sensor_maxB[0] = maxVal0;
            sensor_minB[0] = minVal0;        
         } 
      Serial.print("Val0 --> ");Serial.print(maxVal0);    Serial.print(" ");   Serial.println(minVal0);
     
      int maxVal1 = sensor_B1[0];
      int minVal1 = sensor_B1[0];
      for (int i = 0; i < (sizeof(sensor_B1) / sizeof(sensor_B1[0])); i++) 
         {
            maxVal1 = max(sensor_B1[i],maxVal1);
            minVal1 = min(sensor_B1[i],minVal1);
            sensor_maxB[1] = maxVal1;
            sensor_minB[1] = minVal1;         
         }
      Serial.print("Val1 --> ");Serial.print(maxVal1);    Serial.print(" ");   Serial.println(minVal1);
      int maxVal2 = sensor_B2[0];
      int minVal2 = sensor_B2[0];
      for (int i = 0; i < (sizeof(sensor_B2) / sizeof(sensor_B2[0])); i++) 
         {
            maxVal2 = max(sensor_B2[i],maxVal2);
            minVal2 = min(sensor_B2[i],minVal2);
            sensor_maxB[2] = maxVal2;
            sensor_minB[2] = minVal2;         
         }
      Serial.print("Val2 --> ");Serial.print(maxVal2);    Serial.print(" ");   Serial.println(minVal2);

      int maxVal3 = sensor_B3[0];
      int minVal3 = sensor_B3[0];
      for (int i = 0; i < (sizeof(sensor_B3) / sizeof(sensor_B3[0])); i++) 
         {
            maxVal3 = max(sensor_B3[i],maxVal3);
            minVal3 = min(sensor_B3[i],minVal3);
            sensor_maxB[3] = maxVal3;
            sensor_minB[3] = minVal3;         
         }
      Serial.print("Val3 --> ");Serial.print(maxVal3);    Serial.print(" ");   Serial.println(minVal3);
      int maxVal4 = sensor_B4[0];
      int minVal4 = sensor_B4[0];
      for (int i = 0; i < (sizeof(sensor_B4) / sizeof(sensor_B4[0])); i++) 
         {
            maxVal4 = max(sensor_B4[i],maxVal4);
            minVal4 = min(sensor_B4[i],minVal4);
            sensor_maxB[4] = maxVal4;
            sensor_minB[4] = minVal4;         
         }
      Serial.print("Val4 --> ");Serial.print(maxVal4);    Serial.print(" ");   Serial.println(minVal4);
      int maxVal5 = sensor_B5[0];
      int minVal5 = sensor_B5[0];
      for (int i = 0; i < (sizeof(sensor_B5) / sizeof(sensor_B5[0])); i++) 
         {
            maxVal5 = max(sensor_B5[i],maxVal5);
            minVal5 = min(sensor_B5[i],minVal5);
            sensor_maxB[5] = maxVal5;
            sensor_minB[5] = minVal5;         
         }
      Serial.print("Val5 --> ");Serial.print(maxVal5);    Serial.print(" ");   Serial.println(minVal5);
      int maxVal6 = sensor_B6[0];
      int minVal6 = sensor_B6[0];
      for (int i = 0; i < (sizeof(sensor_B6) / sizeof(sensor_B6[0])); i++) 
         {
            maxVal6 = max(sensor_B6[i],maxVal6);
            minVal6 = min(sensor_B6[i],minVal6);
            sensor_maxB[6] = maxVal6;
            sensor_minB[6] = minVal6;         
         }
      Serial.print("Val6 --> ");Serial.print(maxVal6);    Serial.print(" ");   Serial.println(minVal6);
      int maxVal7 = sensor_B7[0];
      int minVal7 = sensor_B7[0];
      for (int i = 0; i < (sizeof(sensor_B7) / sizeof(sensor_B7[0])); i++) 
         {
            maxVal7 = max(sensor_B7[i],maxVal7);
            minVal7 = min(sensor_B7[i],minVal7);
            sensor_maxB[7] = maxVal7;
            sensor_minB[7] = minVal7;         
         }
      Serial.print("Val7 --> ");Serial.print(maxVal7);    Serial.print(" ");   Serial.println(minVal7);


//////////////////////////.........................>>>>>>>>>>>>

      int Data_sensor_B[DATA_sensor_SIZE_B] = {sensor_maxB[0], sensor_maxB[1], sensor_maxB[2], sensor_maxB[3], sensor_maxB[4], sensor_maxB[5], sensor_maxB[6], sensor_maxB[7]
                             ,sensor_minB[0], sensor_minB[1], sensor_minB[2], sensor_minB[3], sensor_minB[4], sensor_minB[5], sensor_minB[6], sensor_minB[7]};
      EEPROM.put(80, Data_sensor_B); // write data to EEPROM address 0
      EEPROM.commit(); // save changes to EEPROM
    
      int readData_sensor_B[DATA_sensor_SIZE_B];
      EEPROM.get(1, readData_sensor_B); // read data from EEPROM address 0
      Serial.print("data -> EEPROM : ");
      for (int i = 0; i < 16; i ++) 
        {
         Serial.print(readData_sensor_B[i]);Serial.print("  ");
        }
      Serial.println("  ");
      
      bz(100);
      bz(100);
  }

uint16_t max_analogRead(int sensor)
    {     
       int readData_sensor[DATA_sensor_SIZE];
       EEPROM.get(0, readData_sensor); // read data from EEPROM address 0
       if(sensor == 26)
          {
             return readData_sensor[0];
          }
       else if(sensor == 27)
          {
             return readData_sensor[2];
          }
        return -1;       
    }
uint16_t min_analogRead(int sensor)
    {     
       int readData_sensor[DATA_sensor_SIZE];
       EEPROM.get(0, readData_sensor); // read data from EEPROM address 0
       if(sensor == 26)
          {
             return readData_sensor[1];
          }
       else if(sensor == 27)
          {
             return readData_sensor[3];
          }
        return -1;       
    }
uint16_t md_adc(int sensor)
    {
      int md = 0;
      md = (max_analogRead(sensor) + min_analogRead(sensor))/2;
      return md;
    }
    
uint16_t max_mcp_f(int sensor)
    {     
      int readData_sensor[DATA_sensor_SIZE];
      EEPROM.get(0, readData_sensor); // read data from EEPROM address 0
       if(sensor == 0)
          {
             return readData_sensor[4];
          }
       else if(sensor == 1)
          {
             return readData_sensor[5];
          }
       else if(sensor == 2)
          {
             return readData_sensor[6];
          }
       else if(sensor == 3)
          {
             return readData_sensor[7];
          }
       else if(sensor == 4)
          {
             return readData_sensor[8];
          }
      else if(sensor == 5)
          {
             return readData_sensor[9];
          }
       else if(sensor == 6)
          {
             return readData_sensor[10];
          }
       else if(sensor == 7)
          {
             return readData_sensor[11];
          }
        return -1;    
    }
    
uint16_t min_mcp_f(int sensor)
    {     
      int readData_sensor[DATA_sensor_SIZE];
      EEPROM.get(0, readData_sensor); // read data from EEPROM address 0
       if(sensor == 0)
          {
             return readData_sensor[12];
          }
       else if(sensor == 1)
          {
             return readData_sensor[13];
          }
       else if(sensor == 2)
          {
             return readData_sensor[14];
          }
       else if(sensor == 3)
          {
             return readData_sensor[15];
          }
       else if(sensor == 4)
          {
             return readData_sensor[16];
          }
      else if(sensor == 5)
          {
             return readData_sensor[17];
          }
       else if(sensor == 6)
          {
             return readData_sensor[18];
          }
       else if(sensor == 7)
          {
             return readData_sensor[19];
          }
        return -1;    
    }

uint16_t md_mcp_f(int sensor)
    {
      int md = 0;
      md = (max_mcp_f(sensor) + min_mcp_f(sensor))/2;
      return md;
    }

/////////////////////////////////////------------------------------------------>>>
uint16_t max_mcp_b(int sensor)
    {     
      int readData_sensor_B[DATA_sensor_SIZE_B];
      EEPROM.get(80, readData_sensor_B); // read data from EEPROM address 0
       if(sensor == 0)
          {
             return readData_sensor_B[0];
          }
       else if(sensor == 1)
          {
             return readData_sensor_B[1];
          }
       else if(sensor == 2)
          {
             return readData_sensor_B[2];
          }
       else if(sensor == 3)
          {
             return readData_sensor_B[3];
          }
       else if(sensor == 4)
          {
             return readData_sensor_B[4];
          }
      else if(sensor == 5)
          {
             return readData_sensor_B[5];
          }
       else if(sensor == 6)
          {
             return readData_sensor_B[6];
          }
       else if(sensor == 7)
          {
             return readData_sensor_B[7];
          }
        return -1;    
    }
    
uint16_t min_mcp_b(int sensor)
    {     
      int readData_sensor_B[DATA_sensor_SIZE_B];
      EEPROM.get(1, readData_sensor_B); // read data from EEPROM address 0
       if(sensor == 0)
          {
             return readData_sensor_B[8];
          }
       else if(sensor == 1)
          {
             return readData_sensor_B[9];
          }
       else if(sensor == 2)
          {
             return readData_sensor_B[10];
          }
       else if(sensor == 3)
          {
             return readData_sensor_B[11];
          }
       else if(sensor == 4)
          {
             return readData_sensor_B[12];
          }
      else if(sensor == 5)
          {
             return readData_sensor_B[13];
          }
       else if(sensor == 6)
          {
             return readData_sensor_B[14];
          }
       else if(sensor == 7)
          {
             return readData_sensor_B[15];
          }
        return -1;    
    }

uint16_t md_mcp_b(int sensor)
    {
      int md = 0;
      md = (max_mcp_b(sensor) + min_mcp_b(sensor))/2;
      return md;
    }
///////////////////////----------------------------->>>>

uint16_t Position()  
   {        
      uint16_t min_sensor_values_F[] = { min_mcp_f(0),min_mcp_f(1),min_mcp_f(2),min_mcp_f(3),min_mcp_f(4),min_mcp_f(5),min_mcp_f(6),min_mcp_f(7)  }; //ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      uint16_t max_sensor_values_F[] = { max_mcp_f(0),max_mcp_f(1),max_mcp_f(2),max_mcp_f(3),max_mcp_f(4),max_mcp_f(5),max_mcp_f(6),max_mcp_f(7)  } ; //ค่าที่อ่านได้มากสุด สีขาว                
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < numSensor ; i++) 
          {              
              long value = map(mcp_f(sensor_pin_F[i]), min_sensor_values_F[i], max_sensor_values_F[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

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
                  return 3500;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 3500;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }

int error_F()
    {
        present_position = Position() / ((numSensor - 1) * 10) ;
        setpoint = 50.0;
        errors = setpoint - present_position;   
        
        I = 0;
        previous_I = 0;
        previous_error = 0;
        //P = errors;
        I = I + previous_I;
        D = errors - previous_error ;            
        previous_I=I;
        previous_error=errors  ;  
        //Serial.println(errors);
        return errors;
                             
    }


uint16_t PositionB()  
   {        
      uint16_t min_sensor_values_B[] = { min_mcp_b(0),min_mcp_b(1),min_mcp_b(2),min_mcp_b(3),min_mcp_b(4),min_mcp_b(5),min_mcp_b(6),min_mcp_b(7)  }; //ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      uint16_t max_sensor_values_B[] = { max_mcp_b(0),max_mcp_b(1),max_mcp_b(2),max_mcp_b(3),max_mcp_b(4),max_mcp_b(5),max_mcp_b(6),max_mcp_b(7)  } ; //ค่าที่อ่านได้มากสุด สีขาว                
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < numSensor ; i++) 
          {              
              long value = map(mcp_b(sensor_pin_F[i]), min_sensor_values_B[i], max_sensor_values_B[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

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
                  return 3500;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 3500;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }

int error_B()
    {
        present_position = PositionB() / ((numSensor - 1) * 10) ;
        setpoint = 50.0;
        errors = setpoint - present_position;   
        
        I = 0;
        previous_I = 0;
        previous_error = 0;
        //P = errors;
        I = I + previous_I;
        D = errors - previous_error ;            
        previous_I=I;
        previous_error=errors  ;  
        //Serial.println(errors);
        return errors;
                             
    }

 void sw()
  {
     
      pinMode(11, INPUT);
      
      bz(100); 
      bz(100);
      while(digitalRead(9) == 1)
         {
         for(int i=0; i<600; i++)
         {
            rgb(1, 0, 0);
            if(digitalRead(11) == 0)
               {
                  add_sensor_F();
               }
            if(digitalRead(6) == 0)
               {
                  add_sensor_B();
               }
            for(int i=26; i<28; i++)
               {
                  Serial.print(analogRead(i));
                  Serial.print("  ");
                  if(digitalRead(9) == 0)
                     {
                     goto end_sw;
                     }
               }
            Serial.print("    ");
            
            for(int i=0; i<8; i++)
               {
                  Serial.print(mcp_f(i));
                  Serial.print("  ");
                  if(digitalRead(9) == 0)
                     {
                     goto end_sw;
                     }
               }
            Serial.print("    ");
            
            for(int i=0; i<8; i++)
               {
                  Serial.print(mcp_b(i));
                  Serial.print("  ");
                  if(digitalRead(9) == 0)
                     {
                     goto end_sw;
                     }                
               }
            Serial.println("");
         }
   
         for(int i=0; i<600; i++)
         {
            rgb(0, 1, 0);
            if(digitalRead(11) == 0)
               {
                  add_sensor_F();
               }
            if(digitalRead(6) == 0)
               {
                  add_sensor_B();
               }
            for(int i=26; i<28; i++)
               {
                  Serial.print(analogRead(i));
                  Serial.print("  ");
                  if(digitalRead(9) == 0)
                     {
                     goto end_sw;
                     }               
               }
            Serial.print("    ");
            
            for(int i=0; i<8; i++)
               {
                  Serial.print(mcp_f(i));
                  Serial.print("  ");
                  if(digitalRead(9) == 0)
                     {
                     goto end_sw;
                     }                
               }
            Serial.print("    ");
            
            for(int i=0; i<8; i++)
               {
                  Serial.print(mcp_b(i));
                  Serial.print("  ");
               }
            Serial.println("");
         }
         for(int i=0; i<600; i++)
         {
            rgb(0, 0, 1);
            if(digitalRead(11) == 0)
               {
                  add_sensor_F();
               }
            if(digitalRead(6) == 0)
               {
                  add_sensor_B();
               }
            for(int i=26; i<28; i++)
               {
                  Serial.print(analogRead(i));
                  Serial.print("  ");
                  if(digitalRead(9) == 0)
                     {
                     goto end_sw;
                     }                
               }
            Serial.print("    ");
            
            for(int i=0; i<8; i++)
               {
                  Serial.print(mcp_f(i));
                  Serial.print("  ");
               }
            Serial.print("    ");
            
            for(int i=0; i<8; i++)
               {
                  Serial.print(mcp_b(i));
                  Serial.print("  ");
                  if(digitalRead(9) == 0)
                     {
                     goto end_sw;
                     }                
               }
            Serial.println("");
         }
         }
         end_sw:
         bz(300);

      
  }
  
 void sw_eep()
  {
      pinMode(11, INPUT);
      bz(100); 
      bz(100);
      while(digitalRead(9) == 1)
         {
            if(digitalRead(11) == 0)
               {
                  add_sensor_F();
               }
            if(digitalRead(6) == 0)
               {
                  add_sensor_B();
               }

            int readData_sensor_F[DATA_sensor_SIZE];
            EEPROM.get(0, readData_sensor_F); // read data from EEPROM address 0
            Serial.print("EEP_F: ");
            for (int i = 0; i < 16; i ++) 
               {
                  Serial.print(readData_sensor_F[i]);Serial.print("  ");
               }
               Serial.println("  ");
               Serial.println("  ");
               Serial.println("  ");
            
            int readData_sensor_B[DATA_sensor_SIZE_B];
            EEPROM.get(80, readData_sensor_B); // read data from EEPROM address 0
            Serial.print("  EEP_B: ");
            for (int i = 0; i < 16; i ++) 
               {
                  Serial.print(readData_sensor_B[i]);Serial.print("  ");
               }
               Serial.println("  ");
            
            delay(300);
         
         }

        
  }
#endif
