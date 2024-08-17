#ifndef _my_motor_
#define _my_motor_



int mt_l, mt_r;
int sl = 0;
int sr = 0;
int sm = 0;

void to_set_motor_LR(int ML, int MR)
    {
      mt_l = ML;
      mt_r = MR;
    }



void Motor(int spl,int spr)    
   {  
      delayMicroseconds(50);        
      int mtl = (4095*mt_l)/100;
      int mtr = (4095*mt_r)/100;
      sl = map(spl, -100, 100, -mtl, mtl);
      sr = map(spr, -100, 100, -mtr, mtr);      

      sl = constrain(sl, -4095, 4095);
      sr = constrain(sr, -4095, 4095);
               
      if(sl>0)
          {
            digitalWrite(14,HIGH);
            digitalWrite(12,LOW);     
            analogWrite(10,sl);
         }
      else if(sl<0)
         {    
            digitalWrite(14,LOW);
            digitalWrite(12,HIGH);
            analogWrite(10,-sl);
         }
      else
         {        
            digitalWrite(12,LOW);
            digitalWrite(14,LOW);
            analogWrite(10,0);
         }  
            
      if(sr>0)
         {
            digitalWrite(19,HIGH);
            digitalWrite(18,LOW);
            analogWrite(22,sr);
         }
      else if(sr<0)
         {    
            digitalWrite(19,LOW);
            digitalWrite(18,HIGH);
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
