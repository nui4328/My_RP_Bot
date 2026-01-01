#include <myrp_pico2.h>

//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
int servo_down = 55;        //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 120;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 120;      //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
#define MCP3421_ADDR 0x68  // I2C Address when A0 = GND  


void setup() 
  {
    setup_robot() ;
    set_motor();
    encoder.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
    encoder.resetEncoders();  //--------------------->> ฟังก์ชันรอก
    //----------------------------------------------------------------------------->> ตั้งค่ามือจับ
   // s34_trim(0);
   // s35_trim(20);
   // s36_trim(0);
  //----------------------------------------------------------------------------->> ตั้งค่ามือจับ
    arm_down_open();
   //------------------------->>  ส่วนของคำสั่งในการเตรียมแขนกล 
 
    sw();  //--->> คำสั่งรอกดปุ่ม
    
   
    ////------------------------------------------------------------------------------>> รันคำสั่งต่าง ๆ ที่นี่
   

    fline4sensor(50, 50, 0.55, "a0", 'f', 'l', 90, "a5", 1);
    fline4sensor(50, 50, 0.85, "a0", 'f', 'l', 90, "a2", 1);
    fline4sensor(50, 50, 0.85, "a7", 'f', 'r', 50, "a5", 10);

    ////------------------------------------------------------------------------------>> จบการรันคำสั่งต่าง ๆ 
  
  }

void loop() 
  {
    
     //Serial.print(); Serial.println( "   " ); 
    //Motor(30, 30);servo(10, 40);servo(1, 40);servo(0, 40);delay(1000);
    //Motor(-30, -30);servo(10, 140);servo(1, 140);servo(0, 140);delay(1000);
    //Motor(30, 30); delay(1000);
    //Motor(-30, -30); delay(1000);
     // Serial.print(encoder.Poss_L());Serial.print("  "); Serial.println(encoder.Poss_R());

    //Serial.print("adc :");Serial.print(position_4sensor()); Serial.println( "   " ); Serial.print(error_4sensor());  
   
   // Serial.print("my.gyro('z') :");Serial.print(my.gyro('z')); Serial.println( "   " ); 
    Serial.print( analogRead(26) );   Serial.print( "  " );   Serial.print( analogRead(27) ); Serial.print( "     " );
    Serial.print(  md_sensorC(0));   Serial.print( "  " );   Serial.println(  md_sensorC(1) ); 

    Serial.print( sensorMinC[0] );   Serial.print( "   " );   Serial.print( sensorMinC[1] ); 
    Serial.print( "   " ); 
    // Serial.print( sensorMaxC[0] );   Serial.print( "   " );   Serial.println( sensorMaxC[1] );  
    delay(10);
      
    delay(10);
     //Serial.print(sensorMin_A[0] );

  }
