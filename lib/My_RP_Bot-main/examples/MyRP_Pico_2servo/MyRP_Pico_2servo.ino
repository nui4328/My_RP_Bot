#include <my_MINI_RP.h>
#define EEPROM_ADDR 0x50
#include <EncoderLibrary.h>
EncoderLibrary encoder(6, 7, 15, 20);
#include <my_TCS34725.h> 


void setup()
  {
      _seting();                 //-------------------->> ฟังก์ชัน ตั้งค่าต่าง ๆ ใน tab seting
      encoder.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
      encoder.resetEncoders();  //--------------------->> ฟังก์ชันรอก
     

      //arm_down_open();
      //arm_down_close();
      
      
      sw();             //-------------------->> ฟังก์ชันรอกดปุ่ม       
      //test_bord();
///----------------------------------------------------------->>>>  เขียนโค๊ดที่นี่      
     down_open();
///----------------------------------------------------------->>>>  เขียนโค๊ดที่นี่ 

  }  ///-->> บรรทัดห้ามลบ


///-------------------------->> คงไว้ห้ามลบ
void loop() {

     Serial.println(encoder.Poss_R());
     delay(10);
  }
///-------------------------->> คงไว้ห้ามลบ
