#include <my_rp_2wd.h>
#include<Wire.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0x50

unsigned long last_time = millis();
int sensor_maxs[] = {800, 800, 800, 800, 800, 800};
int sensor_mins[] = {150, 150, 150, 150, 150, 150};

bool line;
bool ch_lr = false;
bool ch_set_fb = false;
bool ch_bw = false;

int line_set_fb = 150;          //------> ตั้งค่า เดินหน้า ถอบหลัง ก่อนเลี้ยว  (เซตบัลล้านเส้น)
int line_none_set_fb = 170;     //------> ตั้งค่า เดินหน้า ถอบหลัง ก่อนเลี้ยว  (ไม่เซตบัลล้านเส้น)

int motor_slow = 25;
void setup()
  {
    Serial.begin(9500);
    sensor_set();
    mydisplay_background(black);
    mydisplay("MY-PIPER", 20, 50 ,2, white);
    
    servo(23, 90);
/*
    while(1)
      {
        mission();
      }
  */  
    _sw();

    //--------------------------------------------->>>>
 
for(int i = 0; i < 3; i++)
  {
   fw( 60,60, 300, "line"); 
   //set_f(1);
   tl(85, 200);
   set_b(2);

   fw( 60,60, 800, "line"); 
   set_f(1);
   tl(85, 200);
   set_b(2);

   fw( 60,60, 500, "none_line"); 
   tl(85, 200);
   set_b(5);

   fw( 60,60, 800, "line"); 
   set_f(1);
   tl(85, 200);
   set_b(5);
  }
/*
     _sw();
   fw( 55,58, 300, "line");
   set_f(2);
   tr(80, 160);
   set_b(2);
   
   fw( 56, 56, 150, "line");
   tr(80, 160);
   set_b(2);
   
   fw( 55,58, 150, "line");   
   tl(80, 170);
   set_b(2);
   
   fw( 55,58, 250, "line");
   mission();   //--------------------------->> วางกล่อง
   set_f(4);
   
   bw( 55,58, 200, "no_line");
   tl(80, 170);
   set_b(4);

   fw( 55, 60, 500, "line");
   set_f(3);
   tr(80, 160);
   fw( 55,58, 150, "line");
   tr(80, 160);
   set_b(2);
   fw( 55,58, 250, "line");
   mission();   //--------------------------->> วางกล่อง
   set_f(5);
   
   bw( 55,53, 300, "line");
   tr(80, 160);
   set_b(4);
   
   fw( 60, 60, 400, "no_line");
   tl(80, 170);
   set_b(4);
   fw( 55,58, 400, "line");
   set_f(5);
   tr(80, 160);
   
   fw( 55,58, 150, "line");   
   tr(80, 160);
   set_b(2);
   
   fw( 55,58, 150, "line");
   tl(80, 170);
   
   fw( 55,58, 150, "line");
   tl(80, 170);
   set_b(4);
   
   fw( 55,58, 200, "line");
   tl(80, 170);
   set_b(4);
   
   fw( 55,58, 500, "line"); 
   set_f(10);
*/

 ///---------------------------------------------------->>
  }
void loop()
  {
    _en();  // จบภาระกิจ
    Serial.println(md_sensors(3));
  }
