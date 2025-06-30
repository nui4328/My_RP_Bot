
#define sevopico0 0
#define sevopico1 1
#define sevopico4 4
#define sevopico15 15
Servo servo_pico0;
Servo servo_pico1;
Servo servo_pico4;
Servo servo_pico15;
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
    else if (servo==4)
      {
          servo_pico4.attach(sevopico4,300, 2600);
          servo_pico4.write(angle);   
      } 
    else if (servo==15)
      {
          servo_pico15.attach(sevopico15,300, 2600);
          servo_pico15.write(angle);   
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


void shake_servo_L(int angle, int _num)
  {
    _servo(0, angle);delay(300);
    servo(27, servo_27_close-40); delay(100);
    for(int i=0; i<_num; i++)
      {
        for(int i = angle-15; i<angle+15; i++)
          {
            _servo(0, i);
            delay(10);
          }
        
        for(int i = angle + 15; i>angle - 15; i--)
          {
            _servo(0, i);
            delay(10);
          }
        
      }
    _servo(0, angle);delay(200);
    servo(27, servo_27_close-100); delay(200);
  }
void shake_servo_R(int angle, int _num)
  {
    _servo(1, angle);delay(300);
    servo(28, servo_28_close-40); delay(100);
    for(int i=0; i<_num; i++)
      {
        for(int i = angle-15; i<angle+15; i++)
          {
            _servo(1, i);
            delay(10);
          }
        
        for(int i = angle + 15; i>angle - 15; i--)
          {
            _servo(1, i);
            delay(10);
          }
        
      }
    _servo(1, angle);delay(200);
     servo(28, servo_28_close-100);  delay(200);
  }