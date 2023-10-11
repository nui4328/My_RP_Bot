
void Motor_AA(int sp_motor)
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
void Motor_BB(int sp_motor)
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
void Motor_CC(int sp_motor)
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
void Motor_DD(int sp_motor)
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
