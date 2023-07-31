#ifndef _my_motor_
#define _my_motor_


void Motor(int spl,int spr)    
   {   
      analogWriteResolution(12);
      analogWriteFreq( 1000);
      pinMode(7,OUTPUT);
      pinMode(6,OUTPUT);
      pinMode(8,OUTPUT);            
      pinMode(22,OUTPUT); 
      pinMode(18,OUTPUT);
      pinMode(19,OUTPUT);
      pinMode(19,OUTPUT);
      int sl = 0;
      int sr = 0;
      sl = map(spl, -100, 100, -4095,4095);
      sr = map(spr, -100, 100, -4095,4095);      

if(sr>4095)
      sr = 4095;
      else if(sr<-4095)
      sr = -4095;
               
                  if(sl>4095)
                  sl = 4095;
                  else if(sl<-4095)
                  sl = -4095;
               
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
                        analogWrite(7,4095);
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
                        analogWrite(22,4095);
                     }              
    }
#endif
