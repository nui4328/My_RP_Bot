#include <my_rp_2wd.h>
#include <Wire.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0x50
  


//------------------------->>

int sensor_maxs[] = {936, 942, 825, 755, 941, 822}; 
int sensor_mins[] = {219, 219, 175, 156, 239, 144}; 
 

unsigned long lasts_time = millis();

int tl = 500;   //------> เซตการหมุนตัว
int tr = 500;

void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
     
     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);

     servo(23,130);
     
     sw_start();   

//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้ 

    Motor(80,77); delay(500); 
    fw(100, 97,0, 'r', 1);   // เดินหน้าถึงเส้น เซตบลัลล้าน 1 ครั้ง หมุนขวา  (หากต้องการเดินกำหนดบล๊อกให้เปล๊่ยนจาก 0 เป็นเลขอื่น)
    set_bl_b(2);          // เซตบลัลล้าน  ด้านหลัง 2 ครั้ง
    fw(100, 97,0, 'r', 2);    
    fw(63, 60,0, 'r', 2);
    fw(100, 97,0, 'l', 1);
    set_bl_b(2);
    
    fw(100, 97,0, 's', 1);
    open_servo();
    open_servo();
    set_bl(1);
    
    bw(93, 90,350, 'l', 1);  // เดินถอยหลัง เป็น เวลา 350 (1 บล๊อก) เซตบลัลล้าน 1 ครั้ง หมุนขวา
    set_bl_b(3);
    Motor(80,77); delay(2100);
    Motor(0,0); delay(50);
    Motor(70,-70);delay(350);
    Motor(0,0); delay(50);
    fw(100, 97,0, 'r', 2);
    set_bl_b(3);
    fw(100, 97,0, 's', 1);
    open_servo();               // ปล่อยกล่อง
    open_servo();
    
    bw(100, 97,0, 'r', 1);  // เดินถอยหลังถึงเส้น เซตบลัลล้าน 1 ครั้ง หมุนซ้าย
    set_bl_b(3);         // เซตบลัลล้าน  ด้านหน้า2 ครั้ง
    fw(93, 90,1000, 'r', 1);
    set_bl_b(2);
    fw(100, 97,0, 'l', 2);
    fw(100, 97,0, 'l', 1);
    set_bl_b(2);
    fw(100, 97,0, 's', 1);
    open_servo();
    open_servo();
    set_bl(1);

    bw(100, 97,0, 'l', 1);  // เดินถอยหลังถึงเส้น เซตบลัลล้าน 1 ครั้ง หมุนซ้าย
    set_bl_b(3);         // เซตบลัลล้าน  ด้านหน้า2 ครั้ง
    fw(93, 90,1000, 'r', 1);
    set_bl_b(2);
    fw(100, 97,0, 's', 2);
    open_servo();
    open_servo();
    set_bl(1);

    bw(100, 97,0, 'r', 1);
    Motor(0,0); delay(50);
    Motor(-40,0); delay(150);
    Motor(0,0); delay(50);
    fw(95, 90,430, 'l', 1);
    set_bl_b(3); 
    fw(100, 97,0, 'l', 1);
    fw(95, 90,400, 'r', 1);
    set_bl_b(3);
    Motor(80,77); delay(2100);
    Motor(0,0); delay(50);
    Motor(70,-70);delay(350);
    Motor(0,0); delay(50);
    fw(100, 97,0, 'r', 1);
    fw(100, 97,0, 'l', 1);
    fw(100, 97,0, 'l', 1);
    fw(100, 97,0, 'l', 1);
    set_bl_b(3);
    Motor(80,77); delay(1630);
    Motor(0,0); delay(50);
    
    
    


    
    
    
    

    
    
    

//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  


    for(int i = 0; i<7; i++)
      {
        Serial.print(md_sensors(i));Serial.print("  ");
      }
     Serial.println(" ");


  }
