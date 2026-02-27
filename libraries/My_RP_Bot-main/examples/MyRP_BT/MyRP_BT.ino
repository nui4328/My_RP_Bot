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
      arm_down_close();
      
      
      sw();             //-------------------->> ฟังก์ชันรอกดปุ่ม       
      
///------------------------------------------------------------------>>>>  เขียนโค๊ดที่นี่      
  arm_down_open();
  fline(50,50,0.25,10,'f','s',50, "a2", 30);
  arm_close_up();
  delay(300);
  delay(500);
   bline(50,50,0.25,0,'c','r',50, "a5", 50);
   arm_down_open();
  sw();

///------------------------------------------------------------------>>>>  เขียนโค๊ดที่นี่ 

  }  ///-->> บรรทัดห้ามลบ


///-------------------------->> คงไว้ห้ามลบ
void loop() {

     Serial.println(encoder.Poss_R());
     Serial.println(encoder.Poss_L());
     delay(10);
  }
///-------------------------->> คงไว้ห้ามลบ
