#ifndef _my_servo_
#define _my_servo_


#include <Arduino.h>
#include <Servo.h>


#define sevopico20 20
#define sevopico27 27
#define sevopico28 28


Servo servo_pico20;
Servo servo_pico27;
Servo servo_pico28;

void servo(int servo,int angle)
{  
  if (servo==20)
    {
        servo_pico20.attach(sevopico20, 500, 2500);
        servo_pico20.write(angle);        
    }
  else if (servo==27)
    {
        servo_pico27.attach(sevopico27,500, 2500);
        servo_pico27.write(angle);      
    }

  
  else if (servo==28)
    {
        servo_pico28.attach(sevopico28,500, 2500);
        servo_pico28.write(angle);               
    }


}


#endif
