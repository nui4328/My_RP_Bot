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
   
  test_bord();sw();
  
     arm_ready();
  fline(80,80,0.28,18,'n','s',80, "a1", 2);
  arm_down_close();
  fline(20,20,0.28,0,'f','l',80, "a2", 0);
  fline(20,20,0.28,0,'f','r',80, "a5", 0);
  fline(70,70,0.28,20,'f','r',80, "a5", 0);     
  fline(20,20,0.28,0,'f','r',80, "a5", 0);
  fline(90,90,0.38,55,'f','s',80, "a1", 2);
  place_letf_in(30, 50);delay(100);
  arm_down_open();
  place_letf_out(20, 50);

  bline(60,60,0.38,0,'c','l',50, "a1", 50);
  arm_ready();
  fline(70,70,0.28,56,'n','s',80, "a1", 2);
  fline(15,15,0.28,10,'n','s',80, "a1", 20);
  arm_down_close(); delay(200);

  bline(70,70,0.28,60,'c','r',50, "a6", 50);
  fline(70,70,0.28,15,'n','s',80, "a1", 20);
  place_letf_in(30, 50);delay(100);
  arm_down_open();
  place_letf_out(20, 50);
  fline(20,20,0.28,0,'f','p',80, "a6", 30);delay(200);
  
  bline(60,60,0.38,0,'c','r',50, "a6", 1);
  bline(0,0,0.38,0,'n','r',50, "a6", 60);
  arm_ready();
  fline(70,70,0.38,0,'f','s',80, "a1", 2);
  fline(15,15,0.28,10,'n','s',80, "a1", 20);
  arm_down_close(); delay(100);

  bline(0,0,0.38,0,'c','r',50, "a6", 1);
  bline(0,0,0.38,0,'n','r',50, "a6", 70);

  fline(70,70,0.38,40,'n','s',80, "a1", 1);
  fline(20,20,0.38,20,'n','s',80, "a1", 20);delay(200);
  place_letf_in(20, 30); delay(100);
  arm_down_open();
  place_letf_out(20, 30);
  
  fline(10,10,0.28,0,'f','p',80, "a6", 30);

  bline(80,80,0.38,0,'c','r',50, "a6", 50);
  fline(60,60,0.38,20,'n','s',50, "a6", 30);





///------------------------------------------------------------------>>>>  เขียนโค๊ดที่นี่ 

  }  ///-->> บรรทัดห้ามลบ


///-------------------------->> คงไว้ห้ามลบ
void loop() {

     Serial.println(encoder.Poss_R());
     delay(10);
  }
///-------------------------->> คงไว้ห้ามลบ
