#ifndef _my_sensor_
#define _my_sensor_

#include <Wire.h>
#include <EEPROM.h>
#include "my_2motor.h"
#include "my_MCP3008.h"
#include "Adafruit_ST7735.h"

#define EEPROM_ADDR 0x50
#define DATA_sensor_SIZE 16
#define DATA_sensor_SIZE_B 16

my_MCP3008 adc;

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

int sensor_26[502];
int sensor_27[502];
int sensor_PA[2];
int sensor_max_PA[2];
int sensor_min_PA[2];

float P, I, D, previous_I, previous_error,errors, PID_output, present_position, previous_integral; 
int numSensor = 8; 
uint16_t sensor_pin_F[] = {0,1,2,3,4,5,6,7}; 
uint16_t state_on_Line = 0;
uint16_t setpoint;
uint32_t _lastPosition;

Adafruit_ST7735 tft = Adafruit_ST7735(15, 14, -1);
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

float p = 3.1415926;

uint16_t red = ST77XX_YELLOW;
uint16_t yellow =  ST77XX_RED ;   
uint16_t green = ST77XX_GREEN;
uint16_t black = ST77XX_WHITE;
uint16_t white = ST77XX_BLACK;
uint16_t blue = ST77XX_BLUE;
///////////////////////////////------------------------------->>>

void sensor_set()
   {
      EEPROM.begin(512); // initialize EEPROM with 512 bytes
      analogReadResolution(12);
      tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
      tft.setRotation(3);
      tft.fillScreen(ST77XX_WHITE);
      delay(50);
   }
void testdrawtext(String text, int x, int y, int size_text, uint16_t color, uint16_t led_color) {
  tft.fillScreen(led_color);
  tft.setCursor(x, y);
  tft.setTextSize(size_text);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
  //delay(50);
}

void testtext(char *text, uint16_t color) {
   tft.setTextSize(2);
   tft.setCursor(30, 30);
   tft.setTextColor(color);
   tft.setTextWrap(true);
   tft.print(text);
}
void bz(int dl)
  {
    pinMode(21,OUTPUT);
    digitalWrite(21,HIGH);
    delay(dl);
    digitalWrite(21,LOW);
    delay(dl);    
  }
int mcp_f(int sensor) 
  {         
     adc.begin(5, 4, 12, 13 );  
     //adc.begin(27, 20, 28, 26 ); 
     return adc.readADC(sensor);  
  }
void sw()
  {         
      bz(100); 
      bz(100);
      while(digitalRead(9) == 1)
         {
            testdrawtext("MY-MAKERS", 30, 20, 2,white,black);
         }
        
         end_sw:
         bz(300);     
  }
/////////////////////////////////----------------------->>>>


void add_sensor_F()
  {
      bz(100);
      for (int i = 0; i < 50; i ++)         
         {  
 
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);               
         }
      for (int i = 50; i < 100; i ++)         
         {  
            
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }
       for (int i = 100; i < 150; i ++)         
         {  

            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }
      for (int i = 150; i < 200; i ++)         
         {  
              
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }
      for (int i = 200; i < 250; i ++)         
         {  
  
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }
       for (int i = 250; i < 300; i ++)         
         {  
             
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }
       for (int i = 300; i < 350; i ++)         
         {  

            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }
       for (int i = 350; i < 400; i ++)         
         {  
            
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }
      for (int i = 400; i < 450; i ++)         
         {  
  
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }
       for (int i = 450; i < 502; i ++)         
         {  
                
            sensor_F0[i] = mcp_f(0);
            sensor_F1[i] = mcp_f(1);
            sensor_F2[i] = mcp_f(2);
            sensor_F3[i] = mcp_f(3);
            sensor_F4[i] = mcp_f(4);
            sensor_F5[i] = mcp_f(5);
            sensor_F6[i] = mcp_f(6);
            sensor_F7[i] = mcp_f(7);   
            delay(10);                
         }

      for(int i = 0; i< 502; i++)
        {
          Serial.print(sensor_F0[i]);
          Serial.println(" ");
          Serial.println(i);
        }
      bz(300);
           
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


      int Data_sensor[DATA_sensor_SIZE] = {maxVal0, maxVal1, maxVal2, maxVal3, maxVal4, maxVal5, maxVal6, maxVal7
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

    
uint16_t max_mcp_f(int sensor)
    {     
      int readData_sensor[DATA_sensor_SIZE];
      EEPROM.get(0, readData_sensor); // read data from EEPROM address 0
       if(sensor == 0)
          {
             return readData_sensor[0];
          }
       else if(sensor == 1)
          {
             return readData_sensor[1];
          }
       else if(sensor == 2)
          {
             return readData_sensor[2];
          }
       else if(sensor == 3)
          {
             return readData_sensor[3];
          }
       else if(sensor == 4)
          {
             return readData_sensor[4];
          }
      else if(sensor == 5)
          {
             return readData_sensor[5];
          }
       else if(sensor == 6)
          {
             return readData_sensor[6];
          }
       else if(sensor == 7)
          {
             return readData_sensor[7];
          }
        return -1;    
    }
    
uint16_t min_mcp_f(int sensor)
    {     
      int readData_sensor[DATA_sensor_SIZE];
      EEPROM.get(0, readData_sensor); // read data from EEPROM address 0
       if(sensor == 0)
          {
             return readData_sensor[8];
          }
       else if(sensor == 1)
          {
             return readData_sensor[9];
          }
       else if(sensor == 2)
          {
             return readData_sensor[10];
          }
       else if(sensor == 3)
          {
             return readData_sensor[11];
          }
       else if(sensor == 4)
          {
             return readData_sensor[12];
          }
      else if(sensor == 5)
          {
             return readData_sensor[13];
          }
       else if(sensor == 6)
          {
             return readData_sensor[14];
          }
       else if(sensor == 7)
          {
             return readData_sensor[15];
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



 
  



#endif
