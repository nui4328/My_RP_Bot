#ifndef _my_servo_
#define _my_servo_


#include <Arduino.h>
#include <Servo.h>

#define sevopico23 23
#define sevopico8 8
#define sevopico7 7
#define sevopico29 29
//#define sevopico28 28

Servo servo_pico23;
Servo servo_pico8;
Servo servo_pico7;
Servo servo_pico29;
//Servo servo_pico28;

void servo(int servo,int angle)
{  
  if (servo==23)
    {
        servo_pico23.attach(sevopico23, 500, 2500);
        servo_pico23.write(angle);        
    }
  
  else if (servo==8)
    {
        servo_pico8.attach(sevopico8,500, 2500);
        servo_pico8.write(angle);               
    }
  else if (servo==7)
    {
        servo_pico7.attach(sevopico7,500, 2500);
        servo_pico7.write(angle);      
    }
/*
  else if (servo==28)
    {
        servo_pico28.attach(sevopico28,500, 2500);
        servo_pico28.write(angle);       
    }
*/
  else if (servo==29)
    {
        servo_pico29.attach(sevopico29,500, 2500);
        servo_pico29.write(angle);       
    }
  
}


#endif
