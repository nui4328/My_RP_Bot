#ifndef _my_servo_
#define _my_servo_


//#include <Arduino.h>
#include <Servo.h>

#define sevopico0 0
#define sevopico1 1
#define sevopico8 8
#define sevopico28 28

Servo servo_pico0;
Servo servo_pico1;
Servo servo_pico8;
Servo servo_pico28;

void servo(int servo,int angle)
{  
  if (servo==0)
    {
        servo_pico0.attach(sevopico0, 500, 2500);
        servo_pico0.write(angle);        
    }
  else if (servo==1)
    {
        servo_pico1.attach(sevopico1,500, 2500);
        servo_pico1.write(angle);      
    }

  
  else if (servo==8)
    {
        servo_pico8.attach(sevopico8,500, 2500);
        servo_pico8.write(angle);               
    }
  else if (servo==28)
    {
        servo_pico28.attach(sevopico28,500, 2500);
        servo_pico28.write(angle);               
    }

}


#endif
