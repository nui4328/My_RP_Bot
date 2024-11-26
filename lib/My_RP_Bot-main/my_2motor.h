#ifndef _my_motor_
#define _my_motor_
int fq;
int sl = 0;
int sr = 0;
void Freq_motor(int fq_m)
   {
      fq = fq_m;
      Serial.println(fq);
   }
void Motor(int spl,int spr)    
   {   
      delay(1);
      analogWriteResolution(16);
      analogWriteFreq(400);
      pinMode(7,OUTPUT);
      pinMode(6,OUTPUT);
      pinMode(8,OUTPUT);            
      pinMode(22,OUTPUT); 
      pinMode(18,OUTPUT);
      pinMode(19,OUTPUT);

      sl = map(spl, -100, 100, -95536,95536);
      sr = map(spr, -100, 100, -95536,95536);      

if(sr>95536)
      sr = 95536;
      else if(sr<-95536)
      sr = -95536;
               
                  if(sl>95536)
                  sl = 95536;
                  else if(sl<-95536)
                  sl = -95536;
               
                  if(sl>0)
                     {
                        digitalWrite(8,HIGH);
                        digitalWrite(6,LOW);     
                        analogWrite(7,sl);
                     }
                  else if(sl<0)
                     {    
                        digitalWrite(8,LOW);
                        digitalWrite(6,HIGH);
                        analogWrite(7,-sl);
                     }
                  else
                     {        
                        digitalWrite(8,LOW);
                        digitalWrite(6,LOW);
                        analogWrite(7,0);
                     }  
            
                  if(sr>0)
                     {
                        digitalWrite(18,HIGH);
                        digitalWrite(19,LOW);
                        analogWrite(22,sr);
                     }
                  else if(sr<0)
                     {    
                        digitalWrite(18,LOW);
                        digitalWrite(19,HIGH);
                        analogWrite(22,-sr);
                     }
                  else
                     {        
                        digitalWrite(18,LOW);
                        digitalWrite(19,LOW);
                        analogWrite(22,0);
                     }              
    }
#endif
