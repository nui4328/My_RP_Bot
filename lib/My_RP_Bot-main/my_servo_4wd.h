#ifndef _my_servo_
#define _my_servo_


#include <Arduino.h>
#include <Servo.h>

#define sevopico23 23
#define sevopico28 28
#define sevopico22 22
//#define sevopico26 26
//#define sevopico228 228

Servo servo_pico23;
Servo servo_pico28;
Servo servo_pico22;
//Servo servo_pico26;
//Servo servo_pico228;

void servo(int servo,int angle)
{  
  if (servo==23)
    {
        servo_pico23.attach(sevopico23, 500, 2500);
        servo_pico23.write(angle);        
    }
  else if (servo==22)
    {
        servo_pico22.attach(sevopico22,500, 2500);
        servo_pico22.write(angle);      
    }

  
  else if (servo==28)
    {
        servo_pico28.attach(sevopico28,500, 2500);
        servo_pico28.write(angle);               
    }


}


#endif
