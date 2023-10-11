#ifndef _my_4motor_
#define _my_4motor_

void Motor_A(int sp_motor)
  { 
    delay(1);
    analogWriteResolution(10);
    analogWriteFreq(800);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    int spfl = map(sp_motor, -100, 100, -1024,1024); 
    if(spfl>1024)
      {
        spfl = 1024;
      }
    else if(spfl<-1024)
      {
        spfl = -1024;
      }                   
    if(spfl>0)
      {
        analogWrite(7,spfl);
        digitalWrite(6,0);
      }
    else if(spfl<0)
      {
        analogWrite(6,-spfl);
        digitalWrite(7,0);
      }
    else
      {
        digitalWrite(6,0);
        digitalWrite(7,0);
      }
  }
void Motor_B(int sp_motor)
  { 
    delay(1);
    analogWriteResolution(10);
    analogWriteFreq(800);
    pinMode(20,OUTPUT);
    pinMode(8,OUTPUT);
    int spfr = map(sp_motor, -100, 100, -1024,1024); 
    if(spfr>1024)
      {
        spfr = 1024;
      }
    else if(spfr<-1024)
      {
        spfr = -1024;
      }                   
    if(spfr>0)
      {
        analogWrite(8,spfr);
        digitalWrite(20,0);
      }
    else if(spfr<0)
      {
        analogWrite(20,-spfr);
        digitalWrite(8,0);
      }
    else
      {
        analogWrite(8,0);
        digitalWrite(20,0);
      }
  }
void Motor_C(int sp_motor)
  { 
    delay(1);
    analogWriteResolution(8);
    analogWriteFreq( 800);     
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    int spbl = map(sp_motor, -100, 100, -1024,1024); 
    if(spbl>1024)
      {
        spbl = 1024;
      }
    else if(spbl<-1024)
      {
        spbl = -1024;
      }                   
    if(spbl>0)
      {
        analogWrite(11,spbl);
        digitalWrite(10,0);
      }
    else if(spbl<0)
      {
        analogWrite(10,-spbl);
        digitalWrite(11,0);
      }
    else
      {
        analogWrite(10,0);
        digitalWrite(11,0);
      }
  }
void Motor_D(int sp_motor)
  { 
    delay(1);
    analogWriteResolution(8);
    analogWriteFreq( 800);     
    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);
    int spbr = map(sp_motor, -100, 100, -1024,1024); 
    if(spbr>1024)
      {
        spbr = 1024;
      }
    else if(spbr<-1024)
      {
        spbr = -1024;
      }                   
    if(spbr>0)
      {
        analogWrite(18,spbr);
        digitalWrite(19,0);
      }
    else if(spbr<0)
      {
        analogWrite(19,-spbr);
        digitalWrite(18,0);
      }
    else
      {
        analogWrite(18,0);
        digitalWrite(19,0);
      }
  }
void Motor(int sp_fl, int sp_fr, int sp_bl, int sp_br)
  { 
    delay(1);
    analogWriteResolution(10);
    analogWriteFreq( 800);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
      
    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);
    pinMode(20,OUTPUT);
    pinMode(8,OUTPUT);

    int fl = map(sp_fl, -100, 100, -1024,1024); 
    int fr = map(sp_fr, -100, 100, -1024,1024); 
    int bl = map(sp_bl, -100, 100, -1024,1024); 
    int br = map(sp_br, -100, 100, -1024,1024); 
    if(fl>1024)
      {
        fl = 1024;
      }
    else if(fl<-1024)
      {
        fl = -1024;
      }
      
    if(fr>1024)
      {
        fr = 1024;
      }
    else if(fr<-1024)
      {
        fr = -1024;
      } 

    if(bl>1024)
      {
        bl = 1024;
      }
    else if(bl<-1024)
      {
        bl = -1024;
      }

    if(br>1024)
      {
        br = 1024;
      }
    else if(br<-1024)
      {
        br = -1024;
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
void Motors(int sp_fl, int sp_fr, int sp_bl, int sp_br)
  { 
    delay(1);
    analogWriteResolution(10);
    analogWriteFreq( 800);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
      
    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);
    pinMode(20,OUTPUT);
    pinMode(8,OUTPUT);

    int fl = map(sp_fl, -100, 100, -1024,1024); 
    int fr = map(sp_fr, -100, 100, -1024,1024); 
    int bl = map(sp_bl, -100, 100, -1024,1024); 
    int br = map(sp_br, -100, 100, -1024,1024); 
    if(fl>1024)
      {
        fl = 1024;
      }
    else if(fl<-1024)
      {
        fl = -1024;
      }
      
    if(fr>1024)
      {
        fr = 1024;
      }
    else if(fr<-1024)
      {
        fr = -1024;
      } 

    if(bl>1024)
      {
        bl = 1024;
      }
    else if(bl<-1024)
      {
        bl = -1024;
      }

    if(br>1024)
      {
        br = 1024;
      }
    else if(br<-1024)
      {
        br = -1024;
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
