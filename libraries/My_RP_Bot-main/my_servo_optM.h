#ifndef _my_servo_
#define _my_servo_


#include <Arduino.h>
#include <Servo.h>

#define sevopico26 26
#define sevopico27 27
#define sevopico28 28
#define sevopico29 29

Servo servo_pico26;
Servo servo_pico27;
Servo servo_pico28;
Servo servo_pico29;

void servo(int servo,int angle)
{  
  if (servo==26)
    {
        servo_pico26.attach(sevopico26);
        servo_pico26.write(angle);        
    }
  
  else if (servo==27)
    {
        servo_pico27.attach(sevopico27);
        servo_pico27.write(angle);               
    }
  else if (servo==28)
    {
        servo_pico28.attach(sevopico28);
        servo_pico28.write(angle);      
    }
  else if (servo==29)
    {
        servo_pico29.attach(sevopico29);
        servo_pico29.write(angle);       
    }
  
}


#endif
