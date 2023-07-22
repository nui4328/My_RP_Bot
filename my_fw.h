#ifndef _my_fw_
#define _my_fw_
#include "my_sensor.h"
unsigned long lasts_time=millis();
unsigned long T=0 ;
int slmotor, srmotor, clml, clmr, crml, crmr, flml, flmr, frml, frmr, break_ff, break_fc, delay_f, llmotor, lrmotor, ldelaymotor, rlmotor, rrmotor, rdelaymotor  ;
float kd_f, kd_b, kp_slow, ki_slow;  
void to_slow_motor(int sl, int sr)
     {       
        slmotor = sl;  
        srmotor = sr;       
     }
void to_turn_center_l(int ml,int mr)
     {       
        clml = ml;
        clmr = mr;
     }
void to_turn_center_r(int ml,int mr)
     {       
        crml = ml;
        crmr = mr;
     }
void to_turn_front_l(int ml,int mr)
     {       
        flml = ml;
        flmr = mr;
     }
void to_turn_front_r(int ml,int mr)
     {       
        frml = ml;
        frmr = mr;
     }
void to_brake_fc(int ff, int fc)
     {       
        break_ff = ff;  
        break_fc = fc;       
     }
void to_delay_f(int ff)
     {       
        delay_f = ff;       
     }
void to_speed_turn_fl(int inM, int outM, int delayM )
     {
        llmotor = inM;
        lrmotor = outM;
        ldelaymotor = delayM;
     }
void to_speed_turn_fr(int inM, int outM, int delayM )
     {
        rlmotor = inM;
        rrmotor = outM;
        rdelaymotor = delayM;
     }
void kd_fw(float kd_fw)
  {
	  kd_f = kd_fw;
  }
void kd_bw(float kd_bw)
  {
	  kd_b = kd_bw;
  }
void kp_sl (float kp_sl, float ki_sl) 
  {
    kp_slow = kp_sl;
    ki_slow = ki_sl;
  }
void turn_speed_fl()
  {
    T=0;
    do
      {
        T++;
          PID_output = (0.2 * error_F()) + (0.00015 * I) + (0.2 * D);
          Motor(llmotor + PID_output,lrmotor - PID_output); 
      }
      while (T<ldelaymotor);
      T=0;   
 }

void turn_speed_fr()
  {
    T=0;
    do
      {
        T++;
          PID_output = (0.2 * error_F()) + (0.00015 * I) + (0.2 * D);
          Motor(rlmotor + PID_output,rrmotor - PID_output); 
      }
      while (T<rdelaymotor);
      T=0;   
  }

void bturn_speed_fl()
  {
    T=0;
    do
      {
        T++;
          PID_output = (0.2 * error_F()) + (0.00015 * I) + (0.2 * D);
          Motor(-(llmotor + PID_output),-((lrmotor-10) - PID_output)); 
      }
      while (T<ldelaymotor);
      T=0; delay(5);  
  }

void bturn_speed_fr()
  {
    T=0;
    do
      {
        T++;
          PID_output = (0.2 * error_F()) + (0.00015 * I) + (2* D);
          Motor(-(rlmotor + PID_output),-((rrmotor-10) - PID_output)); 
      }
      while (T<rdelaymotor);
      T=0; delay(5);
  
}
////////////////////////-------------------------------------->>>>>

void fline (int spl ,int spr, float kp, int tim, char nfc, char splr, int power, String sensor, int endt)     //////เดินหน้าแบบตั่งค่า  ความเร็ว  KP  KI  KD และเวลาเอง
  {    
      char sensors[4];  // Declare a char array to store the converted string
      sensor.toCharArray(sensors, sizeof(sensors));
      int sensor_f = atoi(&sensors[1]); 
      //while(1){Serial.print(sensor); Serial.print(" aray ");  Serial.print(sensors[1]); Serial.print(" sensor_f");  Serial.println(sensor_f); delay(500); }
      if (tim >= 1)
         {
            lasts_time=millis();
            while(millis()-lasts_time <tim)
               {
                  PID_output = (kp * error_F()) + (0.00015 * I) + (kd_f * D);
                  Motor(spl - PID_output, spr + PID_output);                       
               }
         }
      else
         {
            while(1)
               {                   
                  PID_output = (kp * error_F()) + (0.00015 * I) + (kd_f * D);
                  Motor(spl - PID_output, spr + PID_output);   
                  if( (mcp_f(2)<md_mcp_f(2) && mcp_f(3)<md_mcp_f(3) && mcp_f(4)<md_mcp_f(4)&& mcp_f(5)<md_mcp_f(5))
                     ||(mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(3)<md_mcp_f(3))
                     ||(mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(2)<md_mcp_f(2))
                     || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                     || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(5)<md_mcp_f(5))
                     )
                        {
                           break;
                        }                       
               }
         }
      if (nfc == 'n')
        {
          if (splr == 'p')
            {
              if(spl >= 1)
                {
                  while(1)
                    {                   
                      PID_output = (kp_slow * error_F()) + (0.00015 * I) + (ki_slow * D);
                      Motor(spl - PID_output, spr + PID_output); 
                      if( (mcp_f(2)<md_mcp_f(2) && mcp_f(3)<md_mcp_f(3) && mcp_f(4)<md_mcp_f(4)&& mcp_f(5)<md_mcp_f(5))
                      ||(mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(3)<md_mcp_f(3))
                      ||(mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(2)<md_mcp_f(2))
                      || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                      || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(5)<md_mcp_f(5))
                      )
                          {
                            break;
                          }                          
                    }
                }
              else
                {
                  while(1)
                    {                   
                      PID_output = (kp_slow * error_F()) + (0.00015 * I) + (ki_slow * D);
                      Motor(slmotor - PID_output, srmotor + PID_output); 
                      if( (mcp_f(2)<md_mcp_f(2) && mcp_f(3)<md_mcp_f(3) && mcp_f(4)<md_mcp_f(4)&& mcp_f(5)<md_mcp_f(5))
                          ||(mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(3)<md_mcp_f(3))
                          || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                          )
                            {
                                break;
                            }                        
                    }
                }
            }
          else{}
        }
      else if (nfc == 'f')
        {
          if (tim >= 1)
            {
              while(1)
                {                   
                    PID_output = (kp_slow * error_F()) + (0.00015 * I) + (ki_slow * D);
                    Motor(slmotor - PID_output, srmotor + PID_output);    
                    if( (mcp_f(2)<md_mcp_f(2) && mcp_f(3)<md_mcp_f(3) && mcp_f(4)<md_mcp_f(4)&& mcp_f(5)<md_mcp_f(5))
                          ||(mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(3)<md_mcp_f(3))
                          || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                          )
                            {
                                break;
                            }                         
                }
            }
          else{}
        }
      else if (nfc == 'c')
        {
          if (tim >= 1 and spl == 0)
            {
              while(1)
                {                     
                    PID_output = (kp_slow * error_F()) + (0.00015 * I) + (ki_slow * D);
                    Motor(slmotor - PID_output, srmotor + PID_output);    
                    if( analogRead(26) <= md_adc(26)-50 
                      || analogRead(27) <= md_adc(27)-50 
                      )
                          {
                          break;
                          }                          
                }                
            }
          else{}
        
          if (splr == 'p' ){}
          else
            {
              while(1)
                {                   
                    PID_output = (kp_slow * error_F()) + (0.00015 * I) + (ki_slow * D);
                    Motor(slmotor - PID_output, srmotor + PID_output);  
                    if( analogRead(26) <= md_adc(26)-50 
                          || analogRead(27) <= md_adc(27)-50 
                          )
                            {
                            break;
                            }                           
                }
            }
            
        }
      else{}

      if (splr == 's')
        {
          Motor(-(slmotor+10) ,-(srmotor+10)) ;
          delay(endt);
          Motor(0,0);
          delay(5);
        }
      else if (splr == 'p')
        {
          delay(10); 
          if(nfc == 'c')
            {
                while(1)
                  {  
                      Motor(spl,spr); delay(5); 
                      if( mcp_f(0)>md_mcp_f(0) && mcp_f(7)>md_mcp_f(7))   
                        {
                          break;
                        }                                                               
                  } 
                delay(10);
                while(1)
                  {                    
                      PID_output = (kp_slow * error_F()) + (0.00015 * I) + (ki_slow * D);
                      Motor(spl - PID_output, spr + PID_output);  
                      if( analogRead(26) <= md_adc(26)-50 
                            || analogRead(27) <= md_adc(27)-50 
                              )
                                  {
                                  break;
                                  }                           
                  }
            }
          else
            {
              while(1)
                  {  
                      Motor(spl,spr);  
                      if( mcp_f(0)>md_mcp_f(0) && mcp_f(7)>md_mcp_f(7))   
                        {
                            break;
                        }                                                               
                  }                  
              delay(40); 
            }
        
        }
    else if (splr == 'l')
        {
          if ((nfc == 'f') || (nfc == 'n' && spl > 0 && tim == 0))
            {
              while(1)
                {  
                  Motor(slmotor,srmotor); delay(5); 
                  if( mcp_f(0)>md_mcp_f(0) && mcp_f(7)>md_mcp_f(7))    
                    {
                        break;
                    }                                                               
                }
              delay(delay_f);
              Motor(-slmotor,-srmotor); delay(break_ff); 

              for ( int i = 0; i <= sensor_f; i++ )
                {
                  do{ Motor((flml*power)/100,(flmr*power)/100); } while( mcp_f(i) > md_mcp_f(i)-50 ); delay(5);
                  do{ Motor((flml*power)/100,(flmr*power)/100); } while( mcp_f(0) < md_mcp_f(0)-50 ); 
                } 
                                        
            }
          else
            {
              if(nfc=='n')
                {
                  Motor(0,0); delay(5);
                }
              else
                {
                  if (spl > 0 )
                    {
                      delay(10);
                      Motor(-(slmotor) ,-(srmotor)) ; delay(break_fc);
                      //Motor(0,0); delay(5);
                    }
                  else
                    {
                      delay(10);
                      Motor(-slmotor ,-srmotor) ; delay(break_fc);
                      //Motor(0,0); delay(5);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f >= 5)
                    {
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_f(i+5) > md_mcp_f(i+5)); delay(5);
                        } 
                    }
                  else
                    {
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_f(i) > md_mcp_f(i)); delay(5);
                        } 
                    }
                }
              else
                {
                  for ( int i = 0; i <= sensor_f; i++ )
                    {
                      do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_b(i) > md_mcp_f(i)); delay(5);
                    } 
                }           
            }
          if(endt==0)
            {
              turn_speed_fl();
            }
          else
            {
              Motor(-((clml*power)/100),-((clmr*power)/100));delay(endt); 
              Motor(0,0);delay(10);                
            }
        }
      else if (splr == 'r')
        {
          if (nfc == 'f')
            {
              while(1)
                {  
                  Motor(slmotor,srmotor); delay(5); 
                  if( mcp_f(0) >= md_mcp_f(0)+100 && mcp_f(7) >= md_mcp_f(7)+100 )  
                    {
                        break;
                    }                                                               
                }
              delay(delay_f);
              Motor(-slmotor,-srmotor); delay(break_ff); 
              for ( int i = 7; i >= sensor_f; i -- )
                {
                  do{ Motor((frml*power)/100,(frmr*power)/100);  } while (mcp_f(i) > md_mcp_f(i)-50);delay(5);  
                  do{ Motor((frml*power)/100,(frmr*power)/100);  } while (mcp_f(7) < md_mcp_f(7)-50);
                }
              
            }
          else
            {
              if(nfc=='n')
                {
                  Motor(0,0); delay(5);
                }
              else
                {
                  if (spl > 0 )
                    {
                      delay(10);
                      Motor(-(slmotor) ,-(srmotor)) ; delay(break_fc);
                      //Motor(0,0); delay(5);
                    }
                  else
                    {
                      delay(10);
                      Motor(-slmotor ,-srmotor) ; delay(break_fc);
                      //Motor(0,0); delay(5);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f <= 2)
                    {
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_f(i-5) > md_mcp_f(i-5)); delay(5);
                        }
                    }
                  else
                    {
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_f(i) > md_mcp_f(i)); delay(5);
                        }
                    }
                }
              else
                {
                  for ( int i = 7; i >= sensor_f; i -- )
                    {
                      do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_b(i) > md_mcp_f(i)); delay(5);
                    }
                }
            }
          if(endt==0)
            {
              turn_speed_fr();
            }
          else
            {
              Motor(-((crml*power)/100),-((crmr*power)/100)); delay(endt);  
              //Motor(1,1);delay(5);
              Motor(0,0);delay(10);
            } 
        }
      else{}
  

  }


void bline (int spl ,int spr, float kp, int tim, char nfc, char splr, int power, String sensor, int endt)     //////เดินหน้าแบบตั่งค่า  ความเร็ว  KP  KI  KD และเวลาเอง
  {    
      char sensors[4];  // Declare a char array to store the converted string
      sensor.toCharArray(sensors, sizeof(sensors));
      int sensor_f = atoi(&sensors[1]); 
      //while(1){Serial.print(sensor); Serial.print(" aray1 ");  Serial.print(sensors[0]); Serial.print(" aray1 ");  Serial.print(sensors[1]); Serial.print(" sensor_f");  Serial.println(sensor_f); delay(500); }
    if (tim > 1)
      {
        lasts_time=millis();
        while(millis()-lasts_time <tim)
          {
            PID_output = (kp * error_B()) + (0.00015 * I) + (kd_b * D);
            Motor(-(spl + PID_output), -(spr - PID_output));                       
          }
      }
    else
      {
        while(1)
          {                   
            PID_output = (kp * error_B()) + (0.00015 * I) + (kd_b * D);
            Motor(-(spl + PID_output), -(spr - PID_output)); 
            if( (mcp_b(2)<md_mcp_b(2) && mcp_b(3)<md_mcp_b(3) && mcp_b(4)<md_mcp_b(4)&& mcp_b(5)<md_mcp_b(5))
                     ||(mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)<md_mcp_b(3))
                     ||(mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)>md_mcp_b(3))
                     ||(mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(2)<md_mcp_b(2))
                     || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)<md_mcp_b(4))
                     || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)>md_mcp_b(4))
                     || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(5)<md_mcp_b(5))
                     )
                        {
                           break;
                        }                         
          }
      }
    if (nfc == 'n')
      {
        if (splr == 'p')
          {
            if(spl > 1)
              {
                while(1)
                  {                   
                    PID_output = (0.3 * error_B()) + (0.00015 * I) + (kd_b* D);
                    Motor(-(spl + PID_output), -(spr - PID_output));  
                    if( (mcp_b(0) < md_mcp_b(0)) && (mcp_b(1) < md_mcp_b(1)) && (mcp_b(6) < md_mcp_b(6)) && (mcp_b(7) < md_mcp_b(7))
                        || (mcp_b(0) < md_mcp_b(0)) && (mcp_b(1) < md_mcp_b(1)) && (mcp_b(2) < md_mcp_b(2))
                        || (mcp_b(7) < md_mcp_b(7)) && (mcp_b(6) < md_mcp_b(6)) && (mcp_b(5) < md_mcp_b(5)) ) 
                      {
                        break;
                      }                         
                  }
              }
            else
              {
                while(1)
                  {                   
                    PID_output = (kp_slow * error_B()) + (0.00015 * I) + (ki_slow* D);
                    Motor(-(slmotor + PID_output), -(srmotor - PID_output)); 
                    if( (mcp_b(0) < md_mcp_b(0)) && (mcp_b(1) < md_mcp_b(1)) && (mcp_b(6) < md_mcp_b(6)) && (mcp_b(7) < md_mcp_b(7))
                        || (mcp_b(0) < md_mcp_b(0)) && (mcp_b(1) < md_mcp_b(1)) && (mcp_b(2) < md_mcp_b(2))
                        || (mcp_b(7) < md_mcp_b(7)) && (mcp_b(6) < md_mcp_b(6)) && (mcp_b(5) < md_mcp_b(5)) ) 
                      {
                        break;
                      }                         
                  }
              }
          }
      }
    else if (nfc == 'f')
      {
        if (tim > 1 and spl == 0)
          {
            while(1)
              {                   
                PID_output = (kp_slow * error_B()) + (0.00015 * I) + (ki_slow* D);
                Motor(-(slmotor + PID_output), -(srmotor - PID_output));    
                if( analogRead(26) <= md_adc(26)
                      || analogRead(27) <= md_adc(27)
                      )
                          {
                          break;
                          }                          
              }
          }
        else{}
      }
    else if (nfc == 'c')
      {
        if( splr == 'p')
          {
            if (spl == 0)
              {
                Motor(-slmotor, -srmotor);
                delay(10);
                while(1)
                  {                     
                    PID_output = (kp_slow * error_B()) + (0.00015 * I) + (ki_slow* D);
                    Motor(-(slmotor + PID_output), -(srmotor - PID_output)); 
                    if( analogRead(26) <= md_adc(26)
                      || analogRead(27) <= md_adc(27)
                      )
                          {
                          break;
                          }                         
                  }
              }
            else
              {        
                while(1)
                  {  
                    PID_output = (kp_slow * error_B()) + (0.00015 * I) + (ki_slow * D);
                    Motor(-(spl + PID_output), -(spr - PID_output)); 
                    if( mcp_b(1) > md_mcp_b(1) && mcp_b(6) > md_mcp_b(6) )  
                      {
                        break;
                      }                                                               
                  }
              }
          }
        else
          {
            if(kp == 0)
              {
                  while(1)
                    { 
                      Motor(-(slmotor ), -(srmotor));
                      if( analogRead(26) <= md_adc(26)
                          || analogRead(27) <= md_adc(27)
                        )
                          {
                            break;
                          }                                              
                    }
              }
            else  
              {
                 while(1)
                    {                     
                      PID_output = (kp_slow * error_B()) + (0.00015 * I) + (ki_slow * D);
                      Motor(-(slmotor + PID_output), -(srmotor - PID_output));
                      if( analogRead(26) <= md_adc(26)
                            || analogRead(27) <= md_adc(27)
                            )
                                {
                                break;
                                }     
                                          
                    }
              }
           
          }          
      }
    //else{}
  
    if (splr == 's')
      {
        Motor(slmotor+10 ,srmotor+10) ;
        delay(endt);
        Motor(0,0);
        delay(5);
      }

    else if (splr == 'p')
      {        
        while(1)
          {  
            PID_output = (kp_slow * error_B()) + (0.00015 * I) + (ki_slow* D);
            Motor(-(spl + PID_output), -(spr - PID_output)); 
            if( mcp_b(1) > md_mcp_b(1) && mcp_b(6) > md_mcp_b(6) )  
              {
                break;
              }                                                                
          }                  
        delay(20); 
      }
    else if (splr == 'l')
      {
        
        if ((nfc == 'f') || (nfc == 'n' && spl > 0 && tim == 0))
          {
            while(1)
              {  
                 Motor(-slmotor,-srmotor); delay(5); 
                if( mcp_b(1) > md_mcp_b(1) && mcp_b(6) > md_mcp_b(6) )  
                  {
                      break;
                  }                                                               
              }
            delay(10);

            for ( int i = 0; i < sensor_f; i++ )
              {
                 do{ Motor(-((flmr*power)/100), -((flml*power)/100)); } while( mcp_f(i) > md_mcp_f(i) ); delay(5);
             
              }                            
          }
        else
          { 
            if (spl > 0 )
              {
                Motor(slmotor+20 ,srmotor+20) ; delay(break_fc);
                Motor(0,0); delay(5);
              }
            else{}
           
            if (sensor[0] == 'a')
                {
                  if (sensor_f >= 5)
                    {
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_f(i+5) > md_mcp_f(i+5)); delay(5);
                        } 
                    }
                  else
                    {
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_f(i) > md_mcp_f(i)); delay(5);
                        } 
                    }
                }
              else
                {
                  for ( int i = 0; i <= sensor_f; i++ )
                    {
                      do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_b(i) > md_mcp_f(i)); delay(5);
                    } 
                }           
            }
        if(endt==0)
          {
            turn_speed_fl();
          }
        else
          {
            Motor(-((clml*power)/100),-((clmr*power)/100));delay(endt); 
            Motor(0,0);delay(5);                
          }
      
      }
     else if (splr == 'r')
      {
        if (nfc == 'f')
          {
             while(1)
              {  
                Motor(-slmotor,-srmotor); delay(5); 
                if( mcp_b(1) > md_mcp_b(1) && mcp_b(6) > md_mcp_b(6) )  
                  {
                      break;
                  }                                                               
              }
            delay(10);
            for ( int i = 7; i > sensor_f; i -- )
              {
                do{ Motor(-((frmr*power)/100), -((frml*power)/100));  } while (mcp_b(i) > md_mcp_b(i));delay(5);  
              }
          }
        else
            {
              if(nfc=='n')
                {
                  Motor(0,0); delay(5);
                }
              else
                {
                  if (spl > 0 )
                    {
                      delay(10);
                      Motor(slmotor ,srmotor) ; delay(break_fc);
                      //Motor(0,0); delay(5);
                    }
                  else
                    {
                      delay(10);
                      Motor(slmotor+20 ,srmotor+20) ; delay(break_fc);
                      //Motor(0,0); delay(5);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f <= 2)
                    {
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_f(i-5) > md_mcp_f(i-5)); delay(5);
                        }
                    }
                  else
                    {
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_f(i) > md_mcp_f(i)); delay(5);
                        }
                    }
                }
              else
                {
                  for ( int i = 7; i >= sensor_f; i -- )
                    {
                      do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_b(i) > md_mcp_f(i)); delay(5);
                    }
                }
            }
        if(endt==0)
          {
            turn_speed_fr();
          }
        else
          {
            Motor(-((crml*power)/100),-((crmr*power)/100)); delay(endt);  
            Motor(0,0);delay(5);
          } 
      }
    else{}

  }

void fline_2sensor(int sl, int sr, float kp, int tm, String line,int sensor, char sp, int ofset)
  {
    char sensors[4];  // Declare a char array to store the converted string
    line.toCharArray(sensors, sizeof(sensors));
    int sensor_l = atoi(&sensors[0]); 
    int sensor_r = atoi(&sensors[2]); 
    if(tm > 0)
      {
        unsigned long lasts_time = millis();
        while(millis() - lasts_time < tm)
          {
            float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
            float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + previous_I;
            D = errors - previous_error ;            
            previous_I=I;
            previous_error=errors  ;  
            PID_output = (kp * P) + (0.00015 * I) + (kp* D); 

            Motor(sl + PID_output, sr - PID_output);
            //Serial.println(errors);
  
          }
          
      }
    else
      {
        while(1)
          {
            float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
            float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + previous_I;
            D = errors - previous_error ;            
            previous_I=I;
            previous_error=errors  ;  
            PID_output = (kp * P) + (0.00015 * I) + (kp* D); 

            Motor(sl + PID_output, sr - PID_output);
            //Serial.println(errors);
            if(sensor == 0)
                  {
                      if(mcp_f(0)<md_mcp_f(0))
                          {
                              break;
                          }
                  }
            else if(sensor == 7)
                  {
                      if(mcp_f(7)<md_mcp_f(7))
                          {
                              break;
                          }
                  }
             if(sensor == 26)
                  {
                      if(analogRead(26)<md_adc(26))
                          {
                              break;
                          }
                  }
              else if(sensor == 27)
                  {
                      if(analogRead(27)<md_adc(27))
                          {
                              break;
                          }
                  }

          }
      }

 ///////////////////////////////////////////////////////////////     
    if(sp == 's')
      {    
      }
    else
      {
        
        while(1)
          {
            float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 40 );
            float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 40 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + previous_I;
            D = errors - previous_error ;            
            previous_I=I;
            previous_error=errors  ;  
            PID_output = (kp * P) + (0.00015 * I) + (kp* D); 

            Motor(sl + PID_output, sr - PID_output);
            //Serial.println(errors);
            if(sensor==0 || sensor==7 )
              {
                if(mcp_f(sensor) > md_mcp_f(sensor))
                  {
                    break;
                  }
              }
            if(sensor==26 || sensor==27 )
              {
                if(analogRead(sensor)>md_adc(sensor))
                  {
                    break;
                  }
              }
              
          }
          
        unsigned long lasts_timea = millis();
        while(millis() - lasts_timea < 30)
          {
            float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 40 );
            float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 40 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + previous_I;
            D = errors - previous_error ;            
            previous_I=I;
            previous_error=errors  ;  
            PID_output = (kp * P) + (0.00015 * I) + (kp* D); 

            Motor(sl + PID_output, sr - PID_output);
            //Serial.println(errors);
  
          }
      }
    if(ofset > 0)
      {
        Motor(-sl, -sr);
        delay(ofset);
        Motor(0, 0);
        delay(50);
      }
    else  {}
 //////////////////////////////////////////////////////////////     
  }
 
#endif
