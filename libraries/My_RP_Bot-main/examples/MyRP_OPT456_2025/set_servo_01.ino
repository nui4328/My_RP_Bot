
#define sevopico0 0
#define sevopico1 1
Servo servo_pico0;
Servo servo_pico1;
void _servo(int servo,int angle)
  {  
    if (servo==0)
      {
          servo_pico0.attach(sevopico0, 300, 2600);
          servo_pico0.write(180-(angle + trim_servo_0));        
      }                                             //------>> ฟังก์ชัน สำหรับใช้งาน SERVO pin 0 และ 1
    else if (servo==1)
      {
          servo_pico1.attach(sevopico1,300, 2600);
          servo_pico1.write(angle + trim_servo_1);      
      }
  }

 void reset_arm()
  {
    do{servo(29, 0);}while(digitalRead(20)==1);
    servo(29, 90); delay(50);
    do{servo(29, 180);}while(digitalRead(20)==0);
    servo(29, 90); delay(50);
    pinMode(20, INPUT_PULLUP);
  }
void test_slide()
  {
    servo(29, 0); delay(1000);
    servo(29, 90); delay(1000);
    servo(29, 180); delay(1000);
    servo(29, 90); delay(1000);
  }
