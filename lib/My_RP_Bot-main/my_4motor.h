#ifndef _my_4motor_
#define _my_4motor_
#include <arduino.h>
void Motor(int sp_fl, int sp_fr, int sp_bl, int sp_br)
  { 
    //delay(1);
    //analogWriteResolution(10);
    //analogWriteFreq( 800);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
      
    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);
    pinMode(20,OUTPUT);
    pinMode(8,OUTPUT);

    int fl = map(sp_fl, -100, 100, -255,255); 
    int fr = map(sp_fr, -100, 100, -255,255); 
    int bl = map(sp_bl, -100, 100, -255,255); 
    int br = map(sp_br, -100, 100, -255,255); 
    if(fl>255)
      {
        fl = 255;
      }
    else if(fl<-255)
      {
        fl = -255;
      }
      
    if(fr>255)
      {
        fr = 255;
      }
    else if(fr<-255)
      {
        fr = -255;
      } 

    if(bl>255)
      {
        bl = 255;
      }
    else if(bl<-255)
      {
        bl = -255;
      }

    if(br>255)
      {
        br = 255;
      }
    else if(br<-255)
      {
        br = -255;
      }
      
    if(fl>0)
      {
        analogWrite(7,fl);
        analogWrite(6,0);
      }
    else if(fl<0)
      {
        analogWrite(6,-fl);
        analogWrite(7,0);
      }
    else
      {
        analogWrite(6,0);
        analogWrite(7,0);
      }

    if(bl>0)
      {
        analogWrite(11,bl);
        analogWrite(10,0);
      }
    else if(bl<0)
      {
        analogWrite(10,-bl);
        analogWrite(11,0);
      }
    else
      {
        analogWrite(10,0);
        analogWrite(11,0);
      }

    if(fr>0)
      {
        analogWrite(8,fr);
        analogWrite(20,0);
      }
    else if(fr<0)
      {
        analogWrite(20,-fr);
        analogWrite(8,0);
      }
    else
      {
        analogWrite(8,0);
        analogWrite(20,0);
      }

   if(br>0)
      {
        analogWrite(18,br);
        analogWrite(19,0);
      }
    else if(br<0)
      {
        analogWrite(19,-br);
        analogWrite(18,0);
      }
    else
      {
        analogWrite(19,0);
        analogWrite(18,0);
      }
  }
void Motors(int sp_fl, int sp_fr, int sp_bl, int sp_br)
  { 
    delay(1);
    analogWriteResolution(8);
    analogWriteFreq( 1000);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
      
    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);
    pinMode(20,OUTPUT);
    pinMode(8,OUTPUT);

    int fl = map(sp_fl, -100, 100, -255,255); 
    int fr = map(sp_fr, -100, 100, -255,255); 
    int bl = map(sp_bl, -100, 100, -255,255); 
    int br = map(sp_br, -100, 100, -255,255); 
       if(fl>255)
      {
        fl = 255;
      }
    else if(fl<-255)
      {
        fl = -255;
      }
      
    if(fr>255)
      {
        fr = 255;
      }
    else if(fr<-255)
      {
        fr = -255;
      } 

    if(bl>255)
      {
        bl = 255;
      }
    else if(bl<-255)
      {
        bl = -255;
      }

    if(br>255)
      {
        br = 255;
      }
    else if(br<-255)
      {
        br = -255;
      }
      
    if(fl>0)
      {
        analogWrite(7,fl);
        digitalWrite(6,0);
      }
    else if(fl<0)
      {
        analogWrite(6,-fl);
        digitalWrite(7,0);
      }
    else
      {
        digitalWrite(6,1);
        digitalWrite(7,1);
      }

    if(bl>0)
      {
        analogWrite(11,bl);
        digitalWrite(10,0);
      }
    else if(bl<0)
      {
        analogWrite(10,-bl);
        digitalWrite(11,0);
      }
    else
      {
        digitalWrite(10,1);
        digitalWrite(11,1);
      }

    if(fr>0)
      {
        analogWrite(8,fr);
        digitalWrite(20,0);
      }
    else if(fr<0)
      {
        analogWrite(20,-fr);
        digitalWrite(8,0);
      }
    else
      {
        digitalWrite(8,1);
        digitalWrite(20,1);
      }

   if(br>0)
      {
        analogWrite(18,br);
        digitalWrite(19,0);
      }
    else if(br<0)
      {
        analogWrite(19,-br);
        digitalWrite(18,0);
      }
    else
      {
        digitalWrite(19,1);
        digitalWrite(18,1);
      }
  }
#endif
