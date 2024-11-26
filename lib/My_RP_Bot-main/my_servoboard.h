#ifndef _my_servo_
#define _my_servo_


#include <Arduino.h>
#include <Servo.h>

#define sevopico23 23
#define sevopico10 10
#define sevopico11 11
#define sevopico12 12
//#define sevopico28 28

Servo servo_pico23;
Servo servo_pico10;
Servo servo_pico11;
Servo servo_pico12;
//Servo servo_pico28;

void servo(int servo,int angle)
{  
  if (servo==23)
    {
        servo_pico23.attach(sevopico23, 500, 2500);
        servo_pico23.write(angle);        
    }
  
  else if (servo==10)
    {
        servo_pico10.attach(sevopico10,500, 2500);
        servo_pico10.write(angle);               
    }
  else if (servo==11)
    {
        servo_pico11.attach(sevopico11,500, 2500);
        servo_pico11.write(angle);      
    }
  else if (servo==12)
    {
        servo_pico12.attach(sevopico12,500, 2500);
        servo_pico12.write(angle);       
    }
}


#endif
