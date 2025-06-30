#include <Servo.h>
#define servo39 39
#define servo38 38
#define servo37 37
#define servo36 36
#define servo35 35
#define servo34 34
Servo servo_39;
Servo servo_38;
Servo servo_37;
Servo servo_36;
Servo servo_35;
Servo servo_34;


void servo(int servo,int angle)
{  
  if (servo==39)
    {
        servo_39.attach(servo39, 500, 2500);
        servo_39.write(angle);        
    }
  else if (servo==38)
    {
        servo_38.attach(servo38, 500, 2500);
        servo_38.write(angle);        
    }  
  else if (servo==37)
    {
        servo_37.attach(servo37, 500, 2500);
        servo_37.write(angle);        
    }
  else if (servo==36)
    {
        servo_36.attach(servo36, 500, 2500);
        servo_36.write(angle);        
    }
   else if (servo==35)
    {
        servo_35.attach(servo35, 500, 2500);
        servo_35.write(angle);        
    }
  else if (servo==34)
    {
        servo_34.attach(servo34, 500, 2500);
        servo_34.write(angle);        
    }
}
