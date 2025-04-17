
#define sevopico0 0
#define sevopico1 1
Servo servo_pico0;
Servo servo_pico1;
void _servo(int servo,int angle)
  {  
    if (servo==0)
      {
          servo_pico0.attach(sevopico0, 500, 2500);
          servo_pico0.write(angle);        
      }                                             //------>> ฟังก์ชัน สำหรับใช้งาน SERVO pin 0 และ 1
    else if (servo==1)
      {
          servo_pico1.attach(sevopico1,500, 2500);
          servo_pico1.write(angle+5);      
      }
  }
