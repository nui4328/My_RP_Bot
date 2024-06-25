#include <my_rp2040.h>
#include <my_TCS34725.h>
#define EEPROM_ADDR 0x50

#include <VL6180X.h>

VL6180X dis;

unsigned long last_time = millis();
int servo_down = 70;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servo_up = servo_down + 85;       //-------------------->> ตั้งค่า มื่อจับยกขึ้นตั้งฉากกับพื้น
int servoL_open = 168;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 19;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา

int servo_close = 130; 


void setup() 
  {    
      _seting();         //-------------------->>   ฟังก์ชัน ตั้งค่าต่าง ๆ ใน tab seting

      servo_down_open();
       //servo_down_close();
       //servo_up_close(); 
       //servo_big();
       
       sw();             //-------------------->> ฟังก์ชันรอกดปุ่ม
       // calibrate_M();sw(); 
       // read_sensor_M();

      //////////////////------------------------>>>>>  Run funtion

    
       
              
      //////////////////------------------------>>>>>  

  }

void loop() 
  {
    /*
    Serial.println(my_tcs('r'));
   
   
    for(int i=0; i<8; i++)
      {
        Serial.print(mcp_m(i)); 
        Serial.print("  ");
      }
    Serial.println("");
    */
    Serial.println(dis.readRangeSingleMillimeters());
    //Serial.println(analogRead(28));

   
  }
