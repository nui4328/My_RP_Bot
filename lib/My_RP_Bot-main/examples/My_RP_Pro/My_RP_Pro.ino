#include <my_pro_rp.h>
//#include <my_TCS34725.h>
#define EEPROM_ADDR 0x50
#include <EncoderLibrary.h>
//#include <VL6180X.h>
//VL6180X dis;

EncoderLibrary encoder(0, 15, 24, 20);
//------------------------->>

int servo_down = 60;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servo_up = servo_down + 85;       //-------------------->> ตั้งค่า มื่อจับยกขึ้นตั้งฉากกับพื้น
int servoL_open = 168;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 19;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา

int servo_close = 130; 


void setup() 
  {    
      _seting();         //-------------------->>   ฟังก์ชัน ตั้งค่าต่าง ๆ ใน tab seting
      encoder.setupEncoder(); // เรียกฟังก์ชัน setupEncoder
      encoder.resetEncoders(); 

      
      sw();             //-------------------->> ฟังก์ชันรอกดปุ่ม       
      
  
      //-------------------------------------------------------------------------->>>>>  Run funtion






      //--------------------------------------------------------------------------
 

  }

void loop() 
  {
    //Motor(40, 40); delay(1000);
    //Motor(-40, -40); delay(1000);

   
    Serial.print("Encoder_L: ");
    Serial.print(encoder.Poss_L());
    Serial.print("     ");
    Serial.print("Encoder_R: ");
    Serial.println(encoder.Poss_R());

    delay(100);  // Delay for better readability in serial out
   
  }
