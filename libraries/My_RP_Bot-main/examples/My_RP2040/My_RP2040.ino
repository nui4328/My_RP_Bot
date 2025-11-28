#include <my_rp2040.h>
#include <my_TCS34725.h>
#define EEPROM_ADDR 0x50

unsigned long last_time = millis();
int servo_down = 50;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
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
       

      //////////////////------------------------>>>>>  Run funtion


  

   
           
      //////////////////------------------------>>>>>  

  }

void loop() 
  {
    for(int i=0; i<8; i++)
      {
        Serial.print(mcp_m(i)); 
        Serial.print("  ");
      }
    Serial.println("");

    /*
    Motor(50, 50); // คำสั่งให้มอเตอร์ทำงานที่ค่า spl = 50 และ spr = 50
    delay(1000);
    Motor(-50, -50); // คำสั่งให้มอเตอร์ทำงานที่ค่า spl = -50 และ spr = -50
    delay(1000);
    */
  }
