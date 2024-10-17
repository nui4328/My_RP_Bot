#ifndef _my_sensor_
#define _my_sensor_

#include <Wire.h>
#include <EEPROM.h>
#include "myencoder_2motor.h"
#include "my_MCP3008.h"
#include "Adafruit_ST7735.h"
#include "my_TCS34725.h"

#include "EncoderLibrary.h"
EncoderLibrary encoder(11, 10, 24, 23);

#define EEPROM_ADDR 0x50
#define DATA_sensor_SIZE 16
#define DATA_sensor_SIZE_B 16

my_MCP3008 adc;

my_TCS34725 tcss = my_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
uint16_t readRED_eep[3];
uint16_t readGREEN_eep[3];
uint16_t readBLUE_eep[3];
uint16_t readYELLO_eep[3];
float red_eep[3];
float green_eep[3];
float blue_eep[3];
float yello_eep[3];
String check_color="none";



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

int cl_sp, cr_sp;
void add_sensor_F(void);
void get_red_to_eep(void);
void get_green_to_eep(void);
void get_blue_to_eep(void);
void get_yello_to_eep(void);
void read_eppcolor(void);
void get_color(void);
String check_collor(void);
void begin_robot(void);

///////////////////////////////------------------------------->>>
void cal_censor(int spcl, int spcr)
   {
      cl_sp = spcl;
      cr_sp = spcr;
      add_sensor_F();
   }
void sensor_set()
   {
      EEPROM.begin(512); // initialize EEPROM with 512 bytes
      analogReadResolution(12);
      tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
      tft.setRotation(3);
      tft.fillScreen(ST77XX_WHITE);
        
      encoder.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
      encoder.resetEncoders();  //--------------------->> ฟังก์ชันรอก  
      read_eppcolor();
      delay(50);
   }
void mydisplay(String text, int x, int y, int size_text, uint16_t color) 
   {
  
      tft.setCursor(x, y);
      tft.setTextSize(size_text);
      tft.setTextColor(color);
      tft.setTextWrap(true);
      tft.print(text);
      //delay(50);
   }

void mydisplay_background(uint16_t color_b)
   {
      tft.fillScreen(color_b);
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
    delay(dl);
    pinMode(21,OUTPUT);
    digitalWrite(21,1);
    delay(dl);
    digitalWrite(21,0);
    delay(dl/2);    
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
      mydisplay_background(black);
      while(digitalRead(9) == 1)
         {
            mydisplay("MY-MAKERS", 20, 30, 2, white);
            delay(10);
         }
        

         bz(500);     
  }
/////////////////////////////////----------------------->>>>


void add_sensor_F()
  {
      bz(100);
      for (int i = 0; i < 50; i ++)         
         {  
            Motor(cl_sp, cr_sp);
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
            Motor(cl_sp, cr_sp);
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
            Motor(cl_sp, cr_sp);
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
            Motor(cl_sp, cr_sp);  
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
            Motor(cl_sp, cr_sp);
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
            Motor(cl_sp, cr_sp); 
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
            Motor(cl_sp, cr_sp);
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
            Motor(cl_sp, cr_sp);
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
            Motor(cl_sp, cr_sp);
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
            Motor(cl_sp, cr_sp);   
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
      Motor(0, 0); delay(100);
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
      for (int i = 0; i < DATA_sensor_SIZE; i++) 
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

void red_color()
  {    
    uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    for (int i = 0; i < 3; i++) 
      {
        Serial.print(collor[i]); 
        Serial.print(", ");
      }
    Serial.println("");

  }


void get_red_to_eep()
  {    
    delay(2000);
    uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    
    for (int i = 0; i < 3; i++) {
        Serial.print(collor[i]); 
        Serial.print(", ");
    }
    Serial.println("");
    
    // บันทึกค่า r, g, b ลง EEPROM
    uint16_t red_eeps[3] = {r, g, b};
    EEPROM.put(101, red_eeps); // บันทึกทั้งอาเรย์ red_eeps เริ่มที่อยู่ 101
    EEPROM.commit(); // ยืนยันการบันทึก
    delay(10);
    
    // อ่านค่าที่บันทึกไว้ใน EEPROM
    uint16_t readRED_eep[3]; // ประกาศอาเรย์ขนาด 3 สำหรับเก็บค่าที่อ่านจาก EEPROM
    EEPROM.get(101, readRED_eep); // อ่านข้อมูลจาก EEPROM ที่อยู่ 101
    delay(10);
    
    // แสดงค่าที่อ่านออกมาจาก EEPROM
    for (int i = 0; i < 3; i++) {
        Serial.print(readRED_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("");
    
    // คำนวณค่าเฉลี่ยและปกติของค่า RGB
    float red_averag = (readRED_eep[0] + readRED_eep[1] + readRED_eep[2]) / 3.0;
    red_eep[0] = readRED_eep[0] / red_averag;
    red_eep[1] = readRED_eep[1] / red_averag;
    red_eep[2] = readRED_eep[2] / red_averag;
    
    Serial.println(red_averag); // แสดงค่าเฉลี่ย
    
    // แสดงค่าที่ปกติแล้ว
    for (int i = 0; i < 3; i++) {
        Serial.print(red_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("  ");

      bz(100);
      bz(100);
  }

void get_green_to_eep()
  {   
    delay(2000);
    uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    
    for (int i = 0; i < 3; i++) {
        Serial.print(collor[i]); 
        Serial.print(", ");
    }
    Serial.println("");
    
    // บันทึกค่า r, g, b ลง EEPROM
    uint16_t green_eeps[3] = {r, g, b};
    EEPROM.put(201, green_eeps); // บันทึกทั้งอาเรย์ red_eeps เริ่มที่อยู่ 101
    EEPROM.commit(); // ยืนยันการบันทึก
    delay(10);
    
    // อ่านค่าที่บันทึกไว้ใน EEPROM
    uint16_t readGREEN_eep[3]; // ประกาศอาเรย์ขนาด 3 สำหรับเก็บค่าที่อ่านจาก EEPROM
    EEPROM.get(201, readGREEN_eep); // อ่านข้อมูลจาก EEPROM ที่อยู่ 101
    delay(10);
    
    // แสดงค่าที่อ่านออกมาจาก EEPROM
    for (int i = 0; i < 3; i++) {
        Serial.print(readGREEN_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("");
    
    // คำนวณค่าเฉลี่ยและปกติของค่า RGB
    float green_averag = (readGREEN_eep[0] + readGREEN_eep[1] + readGREEN_eep[2]) / 3.0;
    green_eep[0] = readGREEN_eep[0] / green_averag;
    green_eep[1] = readGREEN_eep[1] / green_averag;
    green_eep[2] = readGREEN_eep[2] / green_averag;
    
    Serial.println(green_averag); // แสดงค่าเฉลี่ย
    
    // แสดงค่าที่ปกติแล้ว
    for (int i = 0; i < 3; i++) {
        Serial.print(green_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("  ");


      bz(100);
      bz(100);
  }

void get_blue_to_eep()
  {     
    delay(2000);
      uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    
    for (int i = 0; i < 3; i++) {
        Serial.print(collor[i]); 
        Serial.print(", ");
    }
    Serial.println("");
    
    // บันทึกค่า r, g, b ลง EEPROM
    uint16_t blue_eeps[3] = {r, g, b};
    EEPROM.put(301, blue_eeps); // บันทึกทั้งอาเรย์ red_eeps เริ่มที่อยู่ 101
    EEPROM.commit(); // ยืนยันการบันทึก
    delay(10);
    
    // อ่านค่าที่บันทึกไว้ใน EEPROM
    uint16_t readGREEN_eep[3]; // ประกาศอาเรย์ขนาด 3 สำหรับเก็บค่าที่อ่านจาก EEPROM
    EEPROM.get(301, readBLUE_eep); // อ่านข้อมูลจาก EEPROM ที่อยู่ 101
    delay(10);
    
    // แสดงค่าที่อ่านออกมาจาก EEPROM
    for (int i = 0; i < 3; i++) {
        Serial.print(readBLUE_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("");
    
    // คำนวณค่าเฉลี่ยและปกติของค่า RGB
    float blue_averag = (readBLUE_eep[0] + readBLUE_eep[1] + readBLUE_eep[2]) / 3.0;
    blue_eep[0] = readBLUE_eep[0] / blue_averag;
    blue_eep[1] = readBLUE_eep[1] / blue_averag;
    blue_eep[2] = readBLUE_eep[2] / blue_averag;
    
    Serial.println(blue_averag); // แสดงค่าเฉลี่ย
    
    // แสดงค่าที่ปกติแล้ว
    for (int i = 0; i < 3; i++) {
        Serial.print(blue_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("  ");


      bz(100);
      bz(100);
  }

void get_yello_to_eep()
  {   
    delay(2000);
         uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    
    for (int i = 0; i < 3; i++) {
        Serial.print(collor[i]); 
        Serial.print(", ");
    }
    Serial.println("");
    
    // บันทึกค่า r, g, b ลง EEPROM
    uint16_t yello_eeps[3] = {r, g, b};
    EEPROM.put(401, yello_eeps); // บันทึกทั้งอาเรย์ red_eeps เริ่มที่อยู่ 101
    EEPROM.commit(); // ยืนยันการบันทึก
    delay(10);
    
    // อ่านค่าที่บันทึกไว้ใน EEPROM
    uint16_t readYELLO_eep[3]; // ประกาศอาเรย์ขนาด 3 สำหรับเก็บค่าที่อ่านจาก EEPROM
    EEPROM.get(401, readYELLO_eep); // อ่านข้อมูลจาก EEPROM ที่อยู่ 101
    delay(10);
    
    // แสดงค่าที่อ่านออกมาจาก EEPROM
    for (int i = 0; i < 3; i++) {
        Serial.print(readYELLO_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("");
    
    // คำนวณค่าเฉลี่ยและปกติของค่า RGB
    float yello_averag = (readYELLO_eep[0] + readYELLO_eep[1] + readYELLO_eep[2]) / 3.0;
    yello_eep[0] = readYELLO_eep[0] / yello_averag;
    yello_eep[1] = readYELLO_eep[1] / yello_averag;
    yello_eep[2] = readYELLO_eep[2] / yello_averag;
    
    Serial.println(yello_averag); // แสดงค่าเฉลี่ย
    
    // แสดงค่าที่ปกติแล้ว
    for (int i = 0; i < 3; i++) {
        Serial.print(yello_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("  ");


      bz(100);
      bz(100);
  }

void read_eppcolor()
  {

      EEPROM.get(101, readRED_eep); 
      EEPROM.get(201, readGREEN_eep); 
      EEPROM.get(301, readBLUE_eep); 
      EEPROM.get(401, readYELLO_eep);         
      for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readRED_eep[i]);
         Serial.print("  ");
         delay(10);
        }
      Serial.println("  ");

       for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readGREEN_eep[i]);
         Serial.print("  ");
         delay(10);
        }
      Serial.println("  ");

       for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readBLUE_eep[i]);
         Serial.print("  ");
         delay(10);
        }
      Serial.println("  ");

       for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readYELLO_eep[i]);
         Serial.print("  ");
         delay(10);
        }
      Serial.println("  ");

      float red_averag = (readRED_eep[0] + readRED_eep[1] + readRED_eep[2])/3;
         red_eep[0] = readRED_eep[0]/red_averag;
         red_eep[1] = readRED_eep[1]/red_averag;
         red_eep[2] = readRED_eep[2]/red_averag;
     
         float green_averag = (readGREEN_eep[0] + readGREEN_eep[1] + readGREEN_eep[2])/3;
         green_eep[0] = readGREEN_eep[0]/green_averag;
         green_eep[1] = readGREEN_eep[1]/green_averag;
         green_eep[2] = readGREEN_eep[2]/green_averag;

         
         float blue_averag = (readBLUE_eep[0] + readBLUE_eep[1] + readBLUE_eep[2])/3;
         blue_eep[0] = readBLUE_eep[0]/blue_averag;
         blue_eep[1] = readBLUE_eep[1]/blue_averag;
         blue_eep[2] = readBLUE_eep[2]/blue_averag;

         
         float yello_averag = (readYELLO_eep[0] + readYELLO_eep[1] + readYELLO_eep[2])/3;
         yello_eep[0] = readYELLO_eep[0]/yello_averag;
         yello_eep[1] = readYELLO_eep[1]/yello_averag;
         yello_eep[2] = readYELLO_eep[2]/yello_averag;

         
      
  }
void get_color()
  {
     get_red_to_eep();        
     sw();
     get_green_to_eep();      
     sw();
     get_blue_to_eep();
     sw();
     get_yello_to_eep();
  }


void check_collors()
  {       
         float red, green, blue;
         uint16_t r, g, b, c;
         tcss.getRawData(&r, &g, &b, &c);
             
         float averag = (r+g+b)/3;
         red = r/averag; ; 
         green = g/averag; 
         blue = b/averag;
                
         if(red < red_eep[0]+0.20 && red > red_eep[0]- 0.20  
            && green < red_eep[1]+0.20 && green > red_eep[1]- 0.20 
            && blue < red_eep[2]+0.20 && blue > red_eep[2]- 0.20  )
            {
              check_color = "red";    
            }
         else if(red < green_eep[0]+0.20 && red > green_eep[0]- 0.20  
            && green < green_eep[1]+0.20 && green > green_eep[1]- 0.20 
            && blue < green_eep[2]+0.20 && blue > green_eep[2]- 0.20  )
            {
              check_color = "green";                        
            } 
   
         else if(red < blue_eep[0]+0.20 && red > blue_eep[0]- 0.20  
            && green < blue_eep[1]+0.20 && green > blue_eep[1]- 0.20 
            && blue < blue_eep[2]+0.20 && blue > blue_eep[2]- 0.20  )
            {
              check_color = "blue";
            }
         else if(red < yello_eep[0]+0.20 && red > yello_eep[0]- 0.20  
            && green < yello_eep[1]+0.20 && green > yello_eep[1]- 0.20 
            && blue < yello_eep[2]+0.20 && blue > yello_eep[2]- 0.20  )
            {
              check_color = "yello";
            }
          else
            {
              check_color = "none";
            }
  }
String check_collor()
  {       
         float red, green, blue;
         uint16_t r, g, b, c;
         tcss.getRawData(&r, &g, &b, &c);
             
         float averag = (r+g+b)/3;
         red = r/averag; ; 
         green = g/averag; 
         blue = b/averag;
                
         if(red < red_eep[0]+0.20 && red > red_eep[0]- 0.20  
            && green < red_eep[1]+0.20 && green > red_eep[1]- 0.20 
            && blue < red_eep[2]+0.20 && blue > red_eep[2]- 0.20  )
            {
              check_color = "red";    
            }
         else if(red < green_eep[0]+0.20 && red > green_eep[0]- 0.20  
            && green < green_eep[1]+0.20 && green > green_eep[1]- 0.20 
            && blue < green_eep[2]+0.20 && blue > green_eep[2]- 0.20  )
            {
              check_color = "green";                        
            } 
   
         else if(red < blue_eep[0]+0.20 && red > blue_eep[0]- 0.20  
            && green < blue_eep[1]+0.20 && green > blue_eep[1]- 0.20 
            && blue < blue_eep[2]+0.20 && blue > blue_eep[2]- 0.20  )
            {
              check_color = "blue";
            }
         else if(red < yello_eep[0]+0.20 && red > yello_eep[0]- 0.20  
            && green < yello_eep[1]+0.20 && green > yello_eep[1]- 0.20 
            && blue < yello_eep[2]+0.20 && blue > yello_eep[2]- 0.20  )
            {
              check_color = "yello";
            }
          else
            {
              check_color = "none";
            }
         return check_color;
  }


void begin_robot()
  {
    bz(100);
    bz(100); 
    while(1)
      {        
        mydisplay_background(black);
        mydisplay("Select Mode", 10, 30, 2, white);
        if(analogRead(29) > 500 && analogRead(29) < 1500)
          {
            bz(100);
            while(1)
              {
                mydisplay_background(black);
                mydisplay(" robot-start", 10, 10, 2, white);
                mydisplay("Press button", 10, 40, 2, white);
                mydisplay("    GP9   ", 10, 60, 2, white);
                if(digitalRead(9)==0) 
                  {
                    goto end_begin;
                  }
                if(analogRead(29) > 1500 || analogRead(29) < 500)
                  {
                    break;
                  }
                
              }
          }
      
        else if(analogRead(29) > 1500 && analogRead(29) < 2500)
          {
            bz(100);
            while(1)
              {
                mydisplay_background(black);
                mydisplay(" Read-sensor", 10, 10, 2, white);
                mydisplay("Press button", 10, 40, 2, white);
                mydisplay("    GP9   ", 10, 60, 2, white);
                if(digitalRead(9)==0) 
                  {
                    bz(100);
                    while(1)
                      {
                         float red, green, blue;
                         uint16_t r, g, b, c;
                         tcss.getRawData(&r, &g, &b, &c);
                             
                         float averag = (r+g+b)/3;
                         red = r/averag; ; 
                         green = g/averag; 
                         blue = b/averag;
                        check_collor();
                        String mcp_f0 = String(mcp_f(0));
                        String mcp_f1 = String(mcp_f(1));
                        String mcp_f2 = String(mcp_f(2));
                        String mcp_f3 = String(mcp_f(3));
                        String mcp_f4 = String(mcp_f(4));
                        String mcp_f5 = String(mcp_f(5));
                        String mcp_f6 = String(mcp_f(6));
                        String mcp_f7 = String(mcp_f(7));
                        String en_posL = String(encoder.Poss_L());
                        String en_posR = String(encoder.Poss_R());
                        String knob = String(analogRead(29));
                        String color = String(check_color);
                        String ccr = String(red);
                        String ccg = String(green);
                        String ccb = String(blue);
                        mydisplay_background(black);
                        mydisplay(mcp_f0 +"  "+ mcp_f1 +"  "+ mcp_f2+"  "+ mcp_f3, 5, 10 ,1, white);
                        mydisplay(knob, 120, 20 ,1, white);
                        mydisplay(mcp_f7 +"  "+ mcp_f6 +"  "+ mcp_f5+"  "+ mcp_f4, 5, 25 ,1, white);
                        mydisplay("R G B  "+ccr +"  "+ ccg +"  "+ ccb , 5, 40 ,1, white);
                        mydisplay(en_posL+"  "+en_posR+"  "+color, 5, 55 ,2, white);
                        
                        if(analogRead(29) > 2500 || analogRead(29) < 1500)
                          {
                            break;
                          }
                      }                    
                  }
               if(analogRead(29) > 2500 || analogRead(29) < 1500)
                  {
                    break;
                  }
                
              }
          }
      
        else if(analogRead(29) > 2500 && analogRead(29) < 3500)
          {
            bz(100);
            while(1)
              {
                mydisplay_background(black);
                mydisplay(" Cal-Sensor", 10, 10, 2, white);
                mydisplay("Press button", 10, 40, 2, white);
                mydisplay("    GP9   ", 10, 60, 2, white);
                if(digitalRead(9)==0) 
                  {                    
                    mydisplay_background(black);
                    mydisplay("Moving ROBOT", 10, 10, 2, white);
                    mydisplay("     ON     ", 10, 30, 2, white);
                    mydisplay(" black&white ", 10, 50, 2, white);
                    cal_censor(0, 0);
                    break;
                  }
                if(analogRead(29) > 3500 || analogRead(29) < 2500)
                  {
                    break;
                  }
                
              }
          }
        else if(analogRead(29) > 3500 && analogRead(29) < 4000)
          {
            bz(100);
            while(1)
              {
                mydisplay_background(black);
                mydisplay(" Get_Color", 10, 10, 2, white);
                mydisplay("Press button", 10, 40, 2, white);
                mydisplay("    GP9   ", 10, 60, 2, white);
                if(digitalRead(9)==0) 
                  {
                    bz(300); delay(1000);
                    while(digitalRead(9)==1)
                      {
                        mydisplay_background(black);
                        mydisplay(" Get_RED", 10, 10, 2, white);
                        mydisplay("Press button", 10, 40, 2, white);
                        mydisplay("    GP9   ", 10, 60, 2, white);
                      }
                    bz(100);
                    get_red_to_eep();    delay(200);    

                    while(digitalRead(9)==1)
                      {
                        mydisplay_background(black);
                        mydisplay(" Get_GREEN", 10, 10, 2, white);
                        mydisplay("Press button", 10, 40, 2, white);
                        mydisplay("    GP9   ", 10, 60, 2, white);
                      }
                    bz(100);
                    get_green_to_eep();     delay(200);   
                    
                    while(digitalRead(9)==1)
                      {
                        mydisplay_background(black);
                        mydisplay(" Get_BLUE", 10, 10, 2, white);
                        mydisplay("Press button", 10, 40, 2, white);
                        mydisplay("    GP9   ", 10, 60, 2, white);
                      }
                    bz(100);
                    get_blue_to_eep();  delay(200);  
                    
                    while(digitalRead(9)==1)
                      {
                        mydisplay_background(black);
                        mydisplay(" Get_YELLO", 10, 10, 2, white);
                        mydisplay("Press button", 10, 40, 2, white);
                        mydisplay("    GP9   ", 10, 60, 2, white);
                      }
                    bz(100);
                    get_yello_to_eep(); delay(200);  
                    bz(100);
                    bz(100);
                    
                    read_eppcolor();
                    while(1)
                      {
                        float red, green, blue;
                         uint16_t r, g, b, c;
                         tcss.getRawData(&r, &g, &b, &c);
                             
                         float averag = (r+g+b)/3;
                         red = r/averag; ; 
                         green = g/averag; 
                         blue = b/averag;
                        check_collor();
                        String mcp_f0 = String(mcp_f(0));
                        String mcp_f1 = String(mcp_f(1));
                        String mcp_f2 = String(mcp_f(2));
                        String mcp_f3 = String(mcp_f(3));
                        String mcp_f4 = String(mcp_f(4));
                        String mcp_f5 = String(mcp_f(5));
                        String mcp_f6 = String(mcp_f(6));
                        String mcp_f7 = String(mcp_f(7));
                        String en_posL = String(encoder.Poss_L());
                        String en_posR = String(encoder.Poss_R());
                        String knob = String(analogRead(29));
                        String color = String(check_color);
                        String ccr = String(red);
                        String ccg = String(green);
                        String ccb = String(blue);
                        mydisplay_background(black);
                        mydisplay(mcp_f0 +"  "+ mcp_f1 +"  "+ mcp_f2+"  "+ mcp_f3, 5, 10 ,1, white);
                        mydisplay(knob, 120, 20 ,1, white);
                        mydisplay(mcp_f7 +"  "+ mcp_f6 +"  "+ mcp_f5+"  "+ mcp_f4, 5, 25 ,1, white);
                        mydisplay("R G B  "+ccr +"  "+ ccg +"  "+ ccb , 5, 40 ,1, white);
                        mydisplay(en_posL+"  "+en_posR+"  "+color, 5, 55 ,2, white);
                      
                        if(analogRead(29) > 4000 || analogRead(29) < 3500)
                          {
                            break;
                          }
                      }                  
                    
                    
                    break;
                  }
                if(analogRead(29) > 4000 || analogRead(29) < 3500)
                  {
                    break;
                  }
                
              }
          }
         
         
      }

     end_begin: delay(10);
     bz(400);
  }

 
  



#endif
