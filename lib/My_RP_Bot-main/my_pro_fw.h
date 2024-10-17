#ifndef _my_fw_
#define _my_fw_
#include "my_sensor.h"
#include "EncoderLibrary.h"
extern EncoderLibrary encoder; // ใช้ extern สำหรับการประกาศ
unsigned long lasts_time=millis();
unsigned long T=0 ;
int slmotor, srmotor, clml, clmr, crml, crmr, flml, flmr, frml, frmr, break_ff, break_fc, break_bf, break_bc, delay_f, llmotor, lrmotor, ldelaymotor, rlmotor, rrmotor, rdelaymotor  ;
float kd_f, kd_b, kp_slow, ki_slow;  
float redius_wheel;
float new_encoder;
int ch_p = 0;
bool _fw ;
void wheel_redius(float rediuss)
     {       
        redius_wheel = rediuss;  
     }

float wheel_distance()
  {
    float circumference = 2 * 3.14 *redius_wheel ;
    return circumference /10 ;
  }
void test_distance(int distance1)
  {
    new_encoder = 440 * distance1 / wheel_distance() ;
    Serial.println(new_encoder);
    encoder.resetEncoders();
    do{Motor(30, 30);}while(encoder.Poss_R() < new_encoder);
    Motor(-30, -30);
    delay(10);
    Motor(1, 1);
  }

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
void to_brake_bc(int ff, int fc)
     {       
        break_bf = ff;  
        break_bc = fc;       
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
      T=0; delay(2);  
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
      T=0; delay(2);
  
}
////////////////////////-------------------------------------->>>>>

void fline (int spl ,int spr, float kp, int tim, char nfc, char splr, int power, String sensor, int endt)     //////เดินหน้าแบบตั่งค่า  ความเร็ว  KP  KI  KD และเวลาเอง
  {  
      _fw = true;  
      char sensors[4];  // Declare a char array to store the converted string
      sensor.toCharArray(sensors, sizeof(sensors));
      int sensor_f = atoi(&sensors[1]); 
      //while(1){Serial.print(sensor); Serial.print(" aray ");  Serial.print(sensors[1]); Serial.print(" sensor_f");  Serial.println(sensor_f); delay(500); }
      if(kp == 0)
          {
              kp = 0;
              I = 0;
              kd_b = 0;
              kd_f = 0;
              kp_slow = 0;
              ki_slow = 0;
          }
      if (tim >= 1)
         {
            new_encoder = 380 * (tim-5) / wheel_distance() ;
            encoder.resetEncoders();
           // lasts_time=millis();
            for(int i=10; i<spl; i++)
              {                
                PID_output = (kp * kf('p')) + (0.00001 * kf('i')) + (kd_f * kf('d'));
                Motor(i - PID_output, i + PID_output); 
                delay(2);  
              }
            while(encoder.Poss_R() < new_encoder)
           // while(millis()-lasts_time <tim)
               {  
                  PID_output = (kp * kf('p')) + (0.00001 * kf('i')) + (kd_f * kf('d'));
                  Motor(spl - PID_output, spr + PID_output);   
                  delayMicroseconds(50);
                      
               }
          
            if(spl >= slmotor)
              {
                for(int i=spl; i > slmotor; i-=2)
                  {                
                    PID_output = (kp * kf('p')) + (0.00001 * kf('i')) + (kd_f * kf('d'));
                    Motor(i - PID_output, i + PID_output); 
                    if((mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(3)<md_mcp_f(3))                    
                            || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                            )
                            {
                              break;
                            }
                    delay(1);  
                  }
              }
            
         }
      else
         {
          if(spl == 0 && tim == 0)
            {delay(10);}
          else
            {
              if(ch_p == 0)
                {
                   for(int i=5; i<spl; i++)
                    {                
                      PID_output = (kp * kf('p')) + (0.00001 * kf('i')) + (kd_f * kf('d'));
                      Motor(i - PID_output, i + PID_output); 
                      if((mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(3)<md_mcp_f(3))                    
                        || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                        )
                        {
                          break;
                        }
                      delay(3);  
                    }
                }
                while(1)
                  {    
                    bg_while:  

                      PID_output = (kp * kf('p')) + (0.00001 * kf('i')) + (kd_f * kf('d'));
                      if(mcp_f(3)<md_mcp_f(3) && mcp_f(0)<md_mcp_f(0) || mcp_f(4)<md_mcp_f(4) && mcp_f(7)<md_mcp_f(7)
                        || mcp_f(4)<md_mcp_f(4) && mcp_f(0)<md_mcp_f(0) || mcp_f(3)<md_mcp_f(3) && mcp_f(7)<md_mcp_f(7))
                        {
                          PID_output = 0;
                        }
                      Motor(spl - PID_output, spr + PID_output);   
                      delayMicroseconds(50);
                      if((mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(3)<md_mcp_f(3))                    
                        || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                        )
                            {
                              break;
                            }    
                    /*
                      if(mcp_f(3)<md_mcp_f(3)+100 && mcp_f(4)<md_mcp_f(4)+70)
                        {
                          while(1)
                            {
                              float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
                              float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
                              errors = error_L - error_R;  

                              P = errors;
                              I = I + errors;
                              D = errors - previous_error;                    
                              previous_error=errors  ;

                              PID_output = (kp * P) + (0 * I) + (kd_f* D); 

                              Motor(spl + PID_output,spr - PID_output);
                              if((mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(3)<md_mcp_f(3))                    
                                || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                                )
                                    {
                                      break;
                                    }  
                              if(mcp_f(3)>md_mcp_f(3) && mcp_f(4)>md_mcp_f(4))
                                {
                                  goto bg_while;
                                }
                            }
                          break;
                        } 
                    */ 
                    ch_p = 0;                        
                  }
            }
         }
      if (nfc == 'n')
        {
          if (splr == 'p')
            {
              ch_p = 1;
              if(spl >= 1)
                {
                  while(1)
                    {  
                      float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
                          float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
                          errors = error_L - error_R;  

                          P = errors;
                          I = I + errors;
                          D = errors - previous_error;                    
                          previous_error=errors  ;

                          PID_output = (kp_slow * P) + (0 * I) + (ki_slow* D); 
      
                      Motor(spl + PID_output, spr - PID_output); delayMicroseconds(50);
                      if( (mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(2)<md_mcp_f(2))
                      || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
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
                      float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
                          float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
                          errors = error_L - error_R;  

                          P = errors;
                          I = I + errors;
                          D = errors - previous_error;                    
                          previous_error=errors  ;

                          PID_output = (kp_slow * P) + (0 * I) + (ki_slow* D);        

                      Motor(slmotor + PID_output, srmotor - PID_output); delayMicroseconds(50);
                      if( (mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(2)<md_mcp_f(2))
                          || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                          )
                            {
                                break;
                            }                        
                    }
                }
            }
          if (splr == 's')
            {
              goto  _entN;
            }
        }
      else if (nfc == 'f')
        {
          if (tim >= 1 || spl == 0 && tim == 0)
            {
              while(1)
                {   
                    PID_output = (kp_slow* kf('p')) + (0.0 * kf('i')) + (ki_slow * kf('d'));                                
                          
                    Motor(slmotor - PID_output, srmotor + PID_output);    delayMicroseconds(50);
                    if((mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1) && mcp_f(2)<md_mcp_f(2))
                          || (mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6) && mcp_f(4)<md_mcp_f(4))
                          )
                            {
                                break;
                            }                         
                }
            }
          else{}
          if (splr == 'p' )
            {
              ch_p = 1;
              while(1)
                  {  
                      Motor(spl,spr); delayMicroseconds(50);
                      if( mcp_f(0)>md_mcp_f(0) && mcp_f(7)>md_mcp_f(7))   
                        {
                          break;
                        }                                                               
                  } 
                delay(10);
                goto _entN; 
            }
          if (splr == 's')
            {
              Motor(-spl, -spr) ;
              delay(endt);
              Motor(0,0);
              delay(2);
              goto _entN; 
            }
        }
      else if (nfc == 'c')
        {

          if (tim >= 1 and spl == 0)
            {
              while(1)
                {                     
                        float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
                          float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
                          errors = error_L - error_R;  

                          P = errors;
                          I = I + errors;
                          D = errors - previous_error;                    
                          previous_error=errors  ;

                          PID_output = (kp_slow * P) + (0 * I) + (ki_slow* D); 
      
  
                    Motor(slmotor + PID_output, srmotor - PID_output);    
                    delayMicroseconds(50);
                    if( analogRead(26) <= md_adc(26)-50 
                      || analogRead(27) <= md_adc(27)-50 
                      )
                          {
                          break;
                          }                          
                }                
            }
          else{}
        
          if (splr == 'p' )
            {
              ch_p = 1;
              while(1)
                  {      
                    float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
                          float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
                          errors = error_L - error_R;  

                          P = errors;
                          I = I + errors;
                          D = errors - previous_error;                    
                          previous_error=errors  ;

                          PID_output = (kp_slow * P) + (0 * I) + (ki_slow* D);    
                      Motor(spl + PID_output, spr - PID_output);  
                      delayMicroseconds(50);
                      if( analogRead(26) <= md_adc(26)-50 
                            || analogRead(27) <= md_adc(27)-50 
                              )
                                  {
                                  break;
                                  }                           
                  }
                goto _entN; 
            }
          else
            {
              while(1)
                {                   
                    float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
                          float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
                          errors = error_L - error_R;  

                          P = errors;
                          I = I + errors;
                          D = errors - previous_error;                    
                          previous_error=errors  ;

                          PID_output = (kp_slow * P) + (0 * I) + (ki_slow* D); 
      
                        
                    Motor(slmotor + PID_output, srmotor - PID_output);  
                    delayMicroseconds(50);
                    if( analogRead(26) <= md_adc(26)-50 
                          || analogRead(27) <= md_adc(27)-50 
                          )
                            {
                            break;
                            }                           
                }
            }
          if (splr == 's')
            {
              Motor(-spl, -spr) ;
              delay(endt);
              Motor(0,0);
              delay(2);
              goto _entN; 
            }
            
        }
      else{}      
        
     if (splr == 'l')
        {
          if ((nfc == 'f') || (nfc == 'n' && spl > 0 && tim == 0))
            {
              while(1)
                {  
                  Motor(slmotor,srmotor);  
                  if( mcp_f(0)>md_mcp_f(0) && mcp_f(7)>md_mcp_f(7))    
                    {
                      delay(delay_f);
                      break;
                    }                                                               
                }
              
              Motor(-(slmotor+20),-(srmotor+20)); delay(break_ff); 

              for ( int i = 0; i <= sensor_f; i++ )
                {
                  do{ Motor((flml*power)/100,(flmr*power)/100); delayMicroseconds(50);} while( mcp_f(i) > md_mcp_f(i)-50 ); delayMicroseconds(50);
                  //do{ Motor((flml*power)/100,(flmr*power)/100); } while( mcp_f(0) < md_mcp_f(0)-50 ); 
                } 
                                        
            }
          else
            {
              if(nfc=='n')
                {
                  Motor(0,0); delay(2);
                }
              else
                {
                  if (spl > 0 )
                    {
                      
                      Motor(-(slmotor) ,-(srmotor)) ; delay(break_fc);
                      //Motor(1,1); delay(2);
                    }
                  else
                    {
                      
                      Motor(-slmotor ,-srmotor) ; delay(break_fc);
                      Motor(1,1); delay(2);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f >= 5)
                    {
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100);delayMicroseconds(50); } while (mcp_f(i+5) > md_mcp_f(i+5)); delayMicroseconds(50);
                        } 
                    }
                  else
                    {
                      Motor((clml*power)/100,(clmr*power)/100); delayMicroseconds(50);
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); delayMicroseconds(50);} while (mcp_f(i) > md_mcp_f(i)); delayMicroseconds(50);
                        } 
                    }
                }
              else
                {
                  for ( int i = 0; i <= sensor_f; i++ )
                    {
                      do{ Motor((clml*power)/100,(clmr*power)/100);delayMicroseconds(50); } while (mcp_b(i) > md_mcp_b(i)); delayMicroseconds(50);
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
              Motor(-1, -1);delay(10);                
            }
        }
      else if (splr == 'r')
        {
          if (nfc == 'f')
            {
              while(1)
                {  
                  Motor(slmotor,srmotor); delayMicroseconds(50); 
                  if( mcp_f(0) >= md_mcp_f(0) && mcp_f(7) >= md_mcp_f(7) )  
                    {
                        break;
                    }                                                               
                }
              delay(delay_f);
              Motor(-slmotor,-srmotor); delay(break_ff); 
              for ( int i = 7; i >= sensor_f; i -- )
                {
                  do{ Motor((frml*power)/100,(frmr*power)/100); delayMicroseconds(50); } while (mcp_f(i) > md_mcp_f(i)-50);delayMicroseconds(50);  
                  //do{ Motor((frml*power)/100,(frmr*power)/100);  } while (mcp_f(7) < md_mcp_f(7)-50);
                }
              
            }
          else
            {
              if(nfc=='n')
                {
                  Motor(0,0); delay(2);
                }
              else
                {
                  if (spl > 0 )
                    {
                      
                      Motor(-(slmotor) ,-(srmotor)) ; delay(break_fc);
                      //Motor(0,0); delay(2);
                    }
                  else
                    {
                     
                      Motor(-slmotor ,-srmotor) ; delay(break_fc);
                      //Motor(0,0); delay(2);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f <= 2)
                    {
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100); delayMicroseconds(50);  } while (mcp_f(i-5) > md_mcp_f(i-5)); delayMicroseconds(50);
                        }
                    }
                  else
                    {
                       Motor((crml*power)/100,(crmr*power)/100);  delay(30);
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);delayMicroseconds(50);   } while (mcp_f(i) > md_mcp_f(i)); delayMicroseconds(50);
                        }
                    }
                }
              else
                {
                  for ( int i = 7; i >= sensor_f; i -- )
                    {
                      do{ Motor((crml*power)/100,(crmr*power)/100); delayMicroseconds(50);  } while (mcp_b(i) > md_mcp_b(i)); delayMicroseconds(50);
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
              //Motor(1,1);delay(2);
              Motor(-1,-1);delay(10);
            } 
        }
      else{}
      _entN:
      if(nfc=='n' && splr == 's' && endt==0)
        {
        }
      else
        {
          Motor(-1,-1);delay(endt); 
        }
           

  }

void bline (int spl ,int spr, float kp, int tim, char nfc, char splr, int power, String sensor, int endt)     //////เดินหน้าแบบตั่งค่า  ความเร็ว  KP  KI  KD และเวลาเอง
  {    
      char sensors[4];  // Declare a char array to store the converted string
      sensor.toCharArray(sensors, sizeof(sensors));
      int sensor_f = atoi(&sensors[1]); 
      //while(1){Serial.print(sensor); Serial.print(" aray1 ");  Serial.print(sensors[0]); Serial.print(" aray1 ");  Serial.print(sensors[1]); Serial.print(" sensor_f");  Serial.println(sensor_f); delay(500); }
     
     
        if(kp == 0)
          {
              kp = 0;
              I = 0;
              kd_b = 0;
              kd_f = 0;
              kp_slow = 0;
              ki_slow = 0;
          }
      if (tim >= 1)
         {
            new_encoder = 350 * (tim - 10)/ wheel_distance() ;
            encoder.resetEncoders();
            for(int i=5; i<spl; i++)
                    {                
                      PID_output = (kp * error_B()) + (0.0 * I) + (kd_b * D);
                      Motor(-(i + PID_output), -(i - PID_output));  
                      if((mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)<md_mcp_b(3))                    
                        || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)<md_mcp_b(4))
                        )
                        {
                          break;
                        }
                      delay(2);  
                    }
            while(encoder.Poss_R() > -new_encoder)
               {  
                  PID_output = (kp * error_B()) + (0.0 * I) + (kd_b * D);
                  Motor(-(spl + PID_output), -(spr - PID_output));                       
               }
            for(int i=spl; i > slmotor; i-=2)
                    {                
                      PID_output = (kp * error_B()) + (0.0 * I) + (kd_b * D);
                      Motor(-(i + PID_output), -(i - PID_output));  
                      if((mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)<md_mcp_b(3))                    
                        || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)<md_mcp_b(4))
                        )
                        {
                          break;
                        }
                      delay(1);  
                    }
         }
    else
      {
        if(spl == 0 && tim == 0)
          {delay(10);}
        else
          {
            if(ch_p == 0 || _fw == true && ch_p == 1 )
                {
                   for(int i=0; i<spl; i++)
                    {                
                      PID_output = (kp * error_B()) + (0.00001 * I) + (kd_b * D);
                      Motor(-(i + PID_output), -(i - PID_output));  
                      if((mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)<md_mcp_b(3))                    
                        || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)<md_mcp_b(4))
                        )
                        {
                          break;
                        }
                      delay(5);  
                    }
                }
            //Motor(-1, -1); delay(30); sw();
            while(1)
              {     
                bg_while:  
                delayMicroseconds(50);            
                PID_output = (kp * error_B()) + (0.00001 * I) + (kd_b * D);
                Motor(-(spl + PID_output), -(spr - PID_output)); 
                if((mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)<md_mcp_b(3))                    
                  || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)<md_mcp_b(4))
                  )
                  {
                     break;
                  } 
                /*
                if(mcp_b(3)<md_mcp_b(3) && mcp_b(4)<md_mcp_b(4))
                        {
                          I = 0;
                          previous_error = 0;
                          while(1)
                            {
                              float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 20 );
                              float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 20 );
                              errors = error_L - error_R;  

                              P = errors;
                              I = I + errors;
                              D = errors - previous_error;                    
                              previous_error=errors  ;
                              
                              PID_output = (kp * P) + (0.00001 * I) + (kd_b* D); 

                              Motor(-(spl - PID_output),-(spr + PID_output));

                              if((mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)<md_mcp_b(3))                    
                                || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)<md_mcp_b(4))
                                )
                                {
                                  break;
                                }
                                    {
                                      break;
                                    }  
                              if(mcp_b(3)>md_mcp_b(3) && mcp_b(4)>md_mcp_b(4))
                                {
                                  goto bg_while;
                                }
                            }
                          //break;
                        }  
                */
                ch_p = 0;                               
              }
          }
      }
     if (nfc == 'n')
        {
          if (splr == 'p')
            {
              ch_p = 1; 
              if(spl >= 1)
                {
                  while(1)
                    {  
                        PID_output = (kp * kb('p')) + (0.0 * kb('i')) + (kd_b * kb('d'));  
                        Motor(-(spl + PID_output), -(spr - PID_output));  
                        if(  (mcp_b(0) < md_mcp_b(0)) && (mcp_b(1) < md_mcp_b(1)) && (mcp_b(2) < md_mcp_b(2))
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
                      PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d'));   
                      Motor(-(slmotor + PID_output), -(srmotor - PID_output)); 
                      if( (mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)<md_mcp_b(3))
                          || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)<md_mcp_b(4))
                          )
                            {
                                break;
                            }                        
                    }
                }
            }
          if (splr == 's')
            {
              goto  _entN;
            }
        }
      else if (nfc == 'f')
        {
          if (tim >= 1 || spl == 0 && tim == 0)
            {
              while(1)
                {                   
                    PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d'));   
                    Motor(-(slmotor + PID_output), -(srmotor - PID_output));    
                    if((mcp_b(0)<md_mcp_b(0) && mcp_b(1)<md_mcp_b(1) && mcp_b(3)<md_mcp_b(3))
                          || (mcp_b(7)<md_mcp_b(7) && mcp_b(6)<md_mcp_b(6) && mcp_b(4)<md_mcp_b(4))
                          )
                            {
                                break;
                            }                         
                }
            }
          else{}
          if (splr == 'p' )
            {
              ch_p = 1; 
              while(1)
                  {  
                      Motor(-spl,-spr); 
                      if( mcp_b(0)>md_mcp_b(0) && mcp_b(7)>md_mcp_b(7))   
                        {
                          break;
                        }                                                               
                  } 
                delay(10);
                goto _entN; 
            }
          if (splr == 's')
            {
              Motor(spl, spr) ;
              delay(endt);
              Motor(0,0);
              delay(2);
              goto _entN; 
            }
        }
      else if (nfc == 'c')
        {
          ch_p = 1; 
          if (tim >= 1 and spl == 0)
            {
              while(1)
                {                     
                    PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d'));   
                    Motor(-(slmotor + PID_output), -(srmotor - PID_output));    
                    if( analogRead(26) < md_adc(26)-100 
                      || analogRead(27) < md_adc(27)-100 
                      )
                          {
                          break;
                          }                          
                }                
            }
          else{}
        
          if (splr == 'p' )
            {
              ch_p = 1; 
              while(1)
                  {                    
                      PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d'));   
                      Motor(-(spl + PID_output), -(spr - PID_output));  
                      if( analogRead(26) < md_adc(26)-100 
                      || analogRead(27) < md_adc(27)-100 
                              )
                                  {
                                  break;
                                  }                           
                  }
                goto _entN; 
            }
          else
            {
              while(1)
                {                   
                    PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d'));   
                    Motor(-(slmotor + PID_output), -(srmotor - PID_output));  
                    if( analogRead(26) < md_adc(26)-100 
                      || analogRead(27) < md_adc(27)-100 
                          )
                            {
                            break;
                            }                           
                }
            }
          if (splr == 's')
            {
              Motor(spl, spr) ;
              delay(endt);
              Motor(0,0);
              delay(2);
              goto _entN; 
            }
            
        }
      else{}      
        
     if (splr == 'l')
        {
          if ((nfc == 'f') || (nfc == 'n' && spl > 0 && tim == 0))
            {
              while(1)
                {  
                  Motor(-slmotor,-srmotor);  
                  if( mcp_b(0)>md_mcp_b(0) && mcp_b(7)>md_mcp_b(7))    
                    {
                        break;
                    }                                                               
                }
              delay(delay_f);
              Motor(slmotor,srmotor); delay(break_bf); 

              for ( int i = 0; i < sensor_f; i++ )
              {
                 do{ Motor(-((flmr*power)/100), -((flml*power)/100)); } while( mcp_f(i) > md_mcp_f(i) ); delay(2);
             
              }  
                                        
            }
          else
            {
              if(nfc=='n')
                {
                  Motor(1,1); delay(2);
                }
              else
                {
                  if (spl > 0 )
                    {
                      if(tim > 0)
                        {
                          Motor(slmotor ,srmotor) ; delay(break_bc/2);
                          Motor(1,1); delay(2);
                        }
                      else 
                        {
                          Motor(slmotor ,srmotor) ; delay(break_bc);
                          Motor(1,1); delay(2);
                        }
                      
                    }
                  else
                    {
                      
                      Motor(slmotor ,srmotor) ; delay(break_bc/2);
                      Motor(1,1); delay(2);
                    }
                }
            if (sensor[0] == 'a')
                {
                  if (sensor_f >= 5)
                    {
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); delayMicroseconds(50);} while (mcp_f(i+5) > md_mcp_f(i+5)); delayMicroseconds(50);
                        } 
                    }
                  else
                    {
                      Motor((clml*power)/100,(clmr*power)/100); delay(30);
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); delayMicroseconds(50);} while (mcp_f(i) > md_mcp_f(i)); delayMicroseconds(50);
                        } 
                    }
                }
            else
                {
                  for ( int i = 0; i <= sensor_f; i++ )
                    {
                      do{ Motor((clml*power)/100,(clmr*power)/100); delayMicroseconds(50);} while (mcp_b(i) > md_mcp_b(i)); delayMicroseconds(50);
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
            Motor(1, 1);delay(10);                
          }
      
        }
      else if (splr == 'r')
        {
          if (nfc == 'f')
            {
              while(1)
              {  
                Motor(-slmotor,-srmotor); delayMicroseconds(50);
                if( mcp_b(1) > md_mcp_b(1) && mcp_b(6) > md_mcp_b(6) )  
                  {
                      break;
                  }                                                               
              }
              delay(delay_f);
              Motor(slmotor,srmotor); delay(break_bf); 
              for ( int i = 7; i > sensor_f; i -- )
                {
                    do{ Motor(-((frmr*power)/100), -((frml*power)/100));  delayMicroseconds(50);} while (mcp_b(i) > md_mcp_b(i));delayMicroseconds(50);
                }
              
            }
          else
            {
              if(nfc=='n')
                {
                  Motor(1,1); delay(2);
                }
              else
                {
                  if (spl > 0 )
                    {
                      if(tim > 0)
                        {
                          Motor(slmotor ,srmotor) ; delay(break_bc/2);
                          Motor(1,1); delay(2);
                        }
                      else 
                        {
                          Motor(slmotor ,srmotor) ; delay(break_bc);
                          Motor(1,1); delay(2);
                        }
                      
                    }
                  else
                    {
                      
                      Motor(slmotor ,srmotor) ; delay(break_bc/2);
                      Motor(1,1); delay(2);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f <= 2)
                    {
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100); delayMicroseconds(50);  } while (mcp_f(i-5) > md_mcp_f(i-5)); delayMicroseconds(50);
                        }
                    }
                  else
                    {
                      Motor((crml*power)/100,(crmr*power)/100); delay(30);
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100); delayMicroseconds(50);  } while (mcp_f(i) > md_mcp_f(i)); delayMicroseconds(50);
                        }
                    }
                }
              else
                {
                  for ( int i = 7; i >= sensor_f; i -- )
                    {
                      do{ Motor((crml*power)/100,(crmr*power)/100); delayMicroseconds(50);  } while (mcp_b(i) > md_mcp_b(i)); delayMicroseconds(50);
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
                Motor(1, 1);delay(10);
            } 
        }
      else{}
      _entN:
      if(nfc=='n' && splr == 's' && endt==0)
        {
        }
      else
        {
          Motor(-1,-1);delay(endt); 
        }

  }
void fline_2sensor(int sl, int sr, float kp, int tm, String line,String sensor, char sp, int ofset)
  {
    char sensors[4];  // Declare a char array to store the converted string
    line.toCharArray(sensors, sizeof(sensors));
    int sensor_l = atoi(&sensors[0]); 
    int sensor_r = atoi(&sensors[2]); 
    if(tm > 0)
      {
        new_encoder = 400 * tm / wheel_distance() ;
        encoder.resetEncoders();
        for(int i=10; i<sl; i++)
          {
            float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
            float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
            errors = error_L - error_R;       
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors  ;
            PID_output = (kp * P) + (0.00001 * I) + (0.2* D); 
            
            Motor(i + PID_output, i - PID_output); delayMicroseconds(50);
            if(mcp_f(sensor_l-1) < md_mcp_f(sensor_l-1) || mcp_f(sensor_l-2) < md_mcp_f(sensor_l-2) && mcp_f(sensor_l) > md_mcp_f(sensor_l))
              {
                do{Motor(sl/2, sl);} while(mcp_f(sensor_l) > md_mcp_f(sensor_l));
              }
            if(mcp_f(sensor_r+1) < md_mcp_f(sensor_r+1) || mcp_f(sensor_r+2) < md_mcp_f(sensor_r+2) && mcp_f(sensor_r) > md_mcp_f(sensor_r) )
              {
                do{Motor(sl, sl/2);} while(mcp_f(sensor_r) > md_mcp_f(sensor_r));
              }
            delay(3);
          }
        while(encoder.Poss_R() < new_encoder)
          {
            float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
            float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
            errors = error_L - error_R;     
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors  ;
            PID_output = (kp * P) + (0.00001 * I) + (0.15* D); 
            
            Motor(sl + PID_output, sr - PID_output);delayMicroseconds(50);
            if(mcp_f(sensor_l-1) < md_mcp_f(sensor_l-1) || mcp_f(sensor_l-2) < md_mcp_f(sensor_l-2) && mcp_f(sensor_l) > md_mcp_f(sensor_l))
              {
                do{Motor(sl/2, sl);} while(mcp_f(sensor_l) > md_mcp_f(sensor_l));
              }
            if(mcp_f(sensor_r+1) < md_mcp_f(sensor_r+1) || mcp_f(sensor_r+2) < md_mcp_f(sensor_r+2) && mcp_f(sensor_r) > md_mcp_f(sensor_r) )
              {
                do{Motor(sl, sl/2);} while(mcp_f(sensor_r) > md_mcp_f(sensor_r));
              }
          }
          
      }
    else
      {
        if(sl == 0 && tm == 0)
          {delay(10);}
        else
          {
            if(ch_p == 0)
                {
                   for(int i=5; i<sl; i++)
                      {
                        float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
                        float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
                        errors = error_L - error_R;       
                        P = errors;
                        I = I + errors;
                        D = errors - previous_error;                    
                        previous_error=errors  ;
                        PID_output = (kp * P) + (0.00001 * I) + (0.2* D); 
                                                
                        Motor(i + PID_output, i - PID_output);
                        if(mcp_f(sensor_l-1) < md_mcp_f(sensor_l-1) || mcp_f(sensor_l-2) < md_mcp_f(sensor_l-2) && mcp_f(sensor_l) > md_mcp_f(sensor_l))
                          {
                            do{Motor(sl/2, sl);} while(mcp_f(sensor_l) > md_mcp_f(sensor_l));
                          }
                        if(mcp_f(sensor_r+1) < md_mcp_f(sensor_r+1) || mcp_f(sensor_r+2) < md_mcp_f(sensor_r+2) && mcp_f(sensor_r) > md_mcp_f(sensor_r) )
                          {
                            do{Motor(sl, sl/2);} while(mcp_f(sensor_r) > md_mcp_f(sensor_r));
                          }
                        delay(3);
                      }
                }
              
            while(1)
              {
                float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
                float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
                errors = error_L - error_R;  
                P = errors;
                I = I + errors;
                D = errors - previous_error;                    
                previous_error=errors  ;
                PID_output = (kp * P) + (0.0 * I) + (0.2* D);  

                Motor(sl + PID_output, sr - PID_output);delayMicroseconds(50);
                if(mcp_f(sensor_l-1) < md_mcp_f(sensor_l-1) || mcp_f(sensor_l-2) < md_mcp_f(sensor_l-2) && mcp_f(sensor_l) > md_mcp_f(sensor_l))
                  {
                    do{Motor(sl/2, sl);} while(mcp_f(sensor_l) > md_mcp_f(sensor_l));
                  }
                if(mcp_f(sensor_r+1) < md_mcp_f(sensor_r+1) || mcp_f(sensor_r+2) < md_mcp_f(sensor_r+2) && mcp_f(sensor_r) > md_mcp_f(sensor_r) )
                  {
                    do{Motor(sl, sl/2);} while(mcp_f(sensor_r) > md_mcp_f(sensor_r));
                  }
                
                if(sensor == "a0")
                      {
                          if(mcp_f(0)<md_mcp_f(0) )
                              {
                                  break;
                              }
                      }
                  else if(sensor == "a7")
                      {
                          if(mcp_f(7)<md_mcp_f(7) )
                              {
                                  break;
                              }
                      }
                else if(sensor == "b0")
                      {
                          if(mcp_b(0) < md_mcp_b(0))
                              {
                                  break;
                              }
                      }
                  else if(sensor == "b7")
                      {
                          if(mcp_b(7) < md_mcp_b(7))
                              {
                                  break;
                              }
                      }
                else if(sensor == "26")
                      {
                          if(analogRead(26)<md_adc(26))
                              {
                                  break;
                              }
                      }
                  else if(sensor == "27")
                      {
                          if(analogRead(27)<md_adc(27))
                              {
                                  break;
                              }
                      }
              }
          }
      }

    ///////////////////////////////////////////////////////////////     
        if(sp == 'p')
          {
            ch_p = 1;
            while(1)
              {
                float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
                float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
                errors = error_L - error_R;  
                P = errors;
                I = I + errors;
                D = errors - previous_error;                    
                previous_error=errors  ;
                PID_output = (kp * P) + (0.0 * I) + (0.2* D); 

                Motor(sl + PID_output, sr - PID_output);delayMicroseconds(50);
                //Serial.println(errors);

                if(sensor == "a0")
                      {
                          if(mcp_f(0)>md_mcp_f(0))
                              {
                                  break;
                              }
                      }
                  else if(sensor == "a7")
                      {
                          if(mcp_f(7)>md_mcp_f(7))
                              {
                                  break;
                              }
                      }
                else if(sensor == "b0")
                      {
                          if(mcp_b(0) > md_mcp_b(0))
                              {
                                  break;
                              }
                      }
                  else if(sensor == "b7")
                      {
                          if(mcp_b(7) > md_mcp_b(7))
                              {
                                  break;
                              }
                      }
                else if(sensor == "26")
                      {
                          if(analogRead(26)>md_adc(26))
                              {
                                  break;
                              }
                      }
                  else if(sensor == "27")
                      {
                          if(analogRead(27)>md_adc(27))
                              {
                                  break;
                              }
                      }
              }
                  unsigned long lasts_timea = millis();
        while(millis() - lasts_timea < 60)
          {
            float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
            float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
            errors = error_L - error_R;  
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors  ;
            PID_output = (kp * P) + (0.0 * I) + (0.2* D); 

            Motor(sl + PID_output, sr - PID_output);
            //Serial.println(errors);
  
          }
      }
    else{}
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


void fline_white(int spl ,int spr, float kp, int tim, char nfc, char splr, int power, String sensor, int endt)     //////เดินหน้าแบบตั่งค่า  ความเร็ว  KP  KI  KD และเวลาเอง
  {    
      char sensors[4];  // Declare a char array to store the converted string
      sensor.toCharArray(sensors, sizeof(sensors));
      int sensor_f = atoi(&sensors[1]); 
      //while(1){Serial.print(sensor); Serial.print(" aray ");  Serial.print(sensors[1]); Serial.print(" sensor_f");  Serial.println(sensor_f); delay(500); }
      if (tim >= 1)
         {
            new_encoder = 400 * tim / wheel_distance() ;
            encoder.resetEncoders();
            while(encoder.Poss_R() > -new_encoder)
               {
                 if(mcp_f(7)>md_mcp_f(7) && mcp_f(4)>md_mcp_f(4) || mcp_f(0)>md_mcp_f(0) && mcp_f(3)>md_mcp_f(3) )
                    {
                      kp = 0;
                      kd_f = 0;
                    }  
                  PID_output = (kp * kf('p')) + (0.0 * kf('i')) + (kd_f * kf('d'));
                  Motor(spl + PID_output, spr - PID_output);                       
               }
         }
      else
         {
            while(1)
               {    
             
                  PID_output = (kp * kf('p')) + (0.0 * kf('i')) + (kd_f * kf('d'));
                  Motor(spl + PID_output, spr - PID_output);   
                  if((mcp_f(0)>md_mcp_f(0) && mcp_f(1)>md_mcp_f(1) && mcp_f(3)>md_mcp_f(3))
                     ||(mcp_f(0)>md_mcp_f(0) && mcp_f(1)>md_mcp_f(1) && mcp_f(2)>md_mcp_f(2))
                     //|| (mcp_f(7)>md_mcp_f(7) && mcp_f(6)>md_mcp_f(6) && mcp_f(4)>md_mcp_f(4))
                     || (mcp_f(7)>md_mcp_f(7) && mcp_f(6)>md_mcp_f(6) && mcp_f(5)>md_mcp_f(5))
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
                      PID_output = (kp_slow * kf('p')) + (0.0 * kf('i')) + (ki_slow * kf('d'));      
                      Motor(spl + PID_output, spr - PID_output); 
                      if( (mcp_f(0)>md_mcp_f(0) && mcp_f(1)>md_mcp_f(1) && mcp_f(3)>md_mcp_f(3))
                      //||(mcp_f(2)>md_mcp_f(2) && mcp_f(3)>md_mcp_f(3) && mcp_f(4)>md_mcp_f(4)&& mcp_f(5)>md_mcp_f(5))

                      ||(mcp_f(0)>md_mcp_f(0) && mcp_f(1)>md_mcp_f(1) && mcp_f(2)>md_mcp_f(2))
                      //|| (mcp_f(7)>md_mcp_f(7) && mcp_f(6)>md_mcp_f(6) && mcp_f(4)>md_mcp_f(4))
                      || (mcp_f(7)>md_mcp_f(7) && mcp_f(6)>md_mcp_f(6) && mcp_f(5)>md_mcp_f(5))
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
                      PID_output = (kp_slow * kf('p')) + (0.0 * kf('i')) + (ki_slow * kf('d'));   
                      Motor(slmotor + PID_output, srmotor - PID_output); 
                      if( (mcp_f(0)>md_mcp_f(0) && mcp_f(1)>md_mcp_f(1) && mcp_f(3)>md_mcp_f(3))
                          || (mcp_f(7)>md_mcp_f(7) && mcp_f(6)>md_mcp_f(6) && mcp_f(4)>md_mcp_f(4))
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
                    PID_output = (kp_slow * kf('p')) + (0.0 * kf('i')) + (ki_slow * kf('d'));   
                    Motor(slmotor + PID_output, srmotor - PID_output);    
                    if( (mcp_f(0)>md_mcp_f(0) && mcp_f(1)>md_mcp_f(1) && mcp_f(3)>md_mcp_f(3))
                          || (mcp_f(7)>md_mcp_f(7) && mcp_f(6)>md_mcp_f(6) && mcp_f(4)>md_mcp_f(4))
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
                    PID_output = (kp_slow * kf('p')) + (0.0 * kf('i')) + (ki_slow * kf('d'));   
                    Motor(slmotor + PID_output, srmotor - PID_output);    
                    if( analogRead(26) >= md_adc(26)+200 
                      || analogRead(27) >= md_adc(27)+200
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
                    PID_output = (kp_slow * kf('p')) + (0.0 * kf('i')) + (ki_slow * kf('d'));   
                    Motor(slmotor + PID_output, srmotor - PID_output);  
                    if( analogRead(26) >= md_adc(26)+200 
                          || analogRead(27) >= md_adc(27)+200
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
          delay(2);
        }
      else if (splr == 'p')
        {
          delay(10); 
          if(nfc == 'c')
            {
                while(1)
                  {  
                      Motor(spl,spr); 
                      if( mcp_f(0)>md_mcp_f(0) && mcp_f(7)>md_mcp_f(7))   
                        {
                          break;
                        }                                                               
                  } 
                delay(10);
                while(1)
                  {                    
                      PID_output = (kp_slow * kf('p')) + (0.0 * kf('i')) + (ki_slow * kf('d'));   
                      Motor(spl + PID_output, spr - PID_output);  
                      if( analogRead(26) >= md_adc(26)+200 
                            || analogRead(27) >= md_adc(27)+200 
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
                      if( mcp_f(0) > md_mcp_f(0) && mcp_f(7) > md_mcp_f(7))   
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
                  Motor(slmotor,srmotor);  
                  if( mcp_f(0)>md_mcp_f(0) && mcp_f(7)>md_mcp_f(7))    
                    {
                        break;
                    }                                                               
                }
              delay(delay_f);
              Motor(-slmotor,-srmotor); delay(break_ff); 

              for ( int i = 0; i <= sensor_f; i++ )
                {
                  do{ Motor((flml*power)/100,(flmr*power)/100); } while( mcp_f(i) < md_mcp_f(i)-50 ); delay(2);
                  do{ Motor((flml*power)/100,(flmr*power)/100); } while( mcp_f(0) > md_mcp_f(0)-50 ); 
                } 
                                        
            }
          else
            {
              if(nfc=='n')
                {
                  Motor(0,0); delay(2);
                }
              else
                {
                  if (spl > 0 )
                    {
                      
                      Motor(-(slmotor) ,-(srmotor)) ; delay(break_fc);
                      //Motor(0,0); delay(2);
                    }
                  else
                    {
                      
                      Motor(-slmotor ,-srmotor) ; delay(break_fc);
                      //Motor(0,0); delay(2);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f >= 5)
                    {
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_f(i+5) < md_mcp_f(i+5)); delay(2);
                        } 
                    }
                  else
                    {
                      Motor((clml*power)/100,(clmr*power)/100); delay(2);
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_f(i) < md_mcp_f(i)); delay(2);
                        } 
                    }
                }
              else
                {
                  for ( int i = 0; i <= sensor_f; i++ )
                    {
                      do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_b(i) < md_mcp_f(i)); delay(2);
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
                  Motor(slmotor,srmotor); delay(2); 
                  if( mcp_f(0) >= md_mcp_f(0)+100 && mcp_f(7) <= md_mcp_f(7)+100 )  
                    {
                        break;
                    }                                                               
                }
              delay(delay_f);
              Motor(-slmotor,-srmotor); delay(break_ff); 
              for ( int i = 7; i >= sensor_f; i -- )
                {
                  do{ Motor((frml*power)/100,(frmr*power)/100);  } while (mcp_f(i) < md_mcp_f(i)-50);delay(2);  
                  do{ Motor((frml*power)/100,(frmr*power)/100);  } while (mcp_f(7) > md_mcp_f(7)-50);
                }
              
            }
          else
            {
              if(nfc=='n')
                {
                  Motor(0,0); delay(2);
                }
              else
                {
                  if (spl > 0 )
                    {
                      
                      Motor(-(slmotor) ,-(srmotor)) ; delay(break_fc);
                      //Motor(0,0); delay(2);
                    }
                  else
                    {
                     
                      Motor(-slmotor ,-srmotor) ; delay(break_fc);
                      //Motor(0,0); delay(2);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f <= 2)
                    {
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_f(i-5) < md_mcp_f(i-5)); delay(2);
                        }
                    }
                  else
                    {
                       Motor((crml*power)/100,(crmr*power)/100);  delay(30);
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_f(i) < md_mcp_f(i)); delay(2);
                        }
                    }
                }
              else
                {
                  for ( int i = 7; i >= sensor_f; i -- )
                    {
                      do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_b(i) < md_mcp_f(i)); delay(2);
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
              //Motor(1,1);delay(2);
              Motor(0,0);delay(10);
            } 
        }
      else{}
  

  }

void bline_white (int spl ,int spr, float kp, int tim, char nfc, char splr, int power, String sensor, int endt)     //////เดินหน้าแบบตั่งค่า  ความเร็ว  KP  KI  KD และเวลาเอง
  {    
      char sensors[4];  // Declare a char array to store the converted string
      sensor.toCharArray(sensors, sizeof(sensors));
      int sensor_f = atoi(&sensors[1]); 
      //while(1){Serial.print(sensor); Serial.print(" aray1 ");  Serial.print(sensors[0]); Serial.print(" aray1 ");  Serial.print(sensors[1]); Serial.print(" sensor_f");  Serial.println(sensor_f); delay(500); }
    if (tim > 1)
      {
        new_encoder = 400 * tim / wheel_distance() ;
        encoder.resetEncoders();
        while(encoder.Poss_R() > -new_encoder)
          {
            if(mcp_b(7)>md_mcp_b(7) && mcp_b(4)>md_mcp_b(4) || mcp_b(0)>md_mcp_b(0) && mcp_b(3)>md_mcp_b(3) )
                    {
                      kp = 0;
                      kd_b = 0;
                    }
            PID_output = (kp * kb('p')) + (0.0 * kb('i')) + (kd_b * kb('d'));
            Motor(-(spl - PID_output), -(spr + PID_output));                       
          }
      }
    else
      {
        while(1)
          {               
            PID_output = (kp * kb('p')) + (0.0 * kb('i')) + (kd_b * kb('d'));
            Motor(-(spl - PID_output), -(spr + PID_output));    
            if( (mcp_b(2)>md_mcp_b(2) && mcp_b(3)>md_mcp_b(3) && mcp_b(4)>md_mcp_b(4)&& mcp_b(5)>md_mcp_b(5))
                     ||(mcp_b(0)>md_mcp_b(0) && mcp_b(1)>md_mcp_b(1) && mcp_b(3)>md_mcp_b(3))
                     ||(mcp_b(0)>md_mcp_b(0) && mcp_b(1)>md_mcp_b(1) && mcp_b(3)>md_mcp_b(3))
                     ||(mcp_b(0)>md_mcp_b(0) && mcp_b(1)>md_mcp_b(1) && mcp_b(2)>md_mcp_b(2))
                     || (mcp_b(7)>md_mcp_b(7) && mcp_b(6)>md_mcp_b(6) && mcp_b(4)>md_mcp_b(4))
                     || (mcp_b(7)>md_mcp_b(7) && mcp_b(6)>md_mcp_b(6) && mcp_b(4)>md_mcp_b(4))
                     || (mcp_b(7)>md_mcp_b(7) && mcp_b(6)>md_mcp_b(6) && mcp_b(5)>md_mcp_b(5))
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
                    PID_output = (kp * kb('p')) + (0.0 * kb('i')) + (kd_b * kb('d'));  
                    Motor(-(spl - PID_output), -(spr + PID_output));    
                    if( (mcp_b(0) > md_mcp_b(0)) && (mcp_b(1) > md_mcp_b(1)) && (mcp_b(6) > md_mcp_b(6)) && (mcp_b(7) > md_mcp_b(7))
                        || (mcp_b(0) > md_mcp_b(0)) && (mcp_b(1) > md_mcp_b(1)) && (mcp_b(2) > md_mcp_b(2))
                        || (mcp_b(7) > md_mcp_b(7)) && (mcp_b(6) > md_mcp_b(6)) && (mcp_b(5) > md_mcp_b(5)) ) 
                      {
                        break;
                      }                         
                  }
              }
            else
              {
                while(1)
                  {           
                    PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d'));    
                    Motor(-(slmotor - PID_output), -(srmotor + PID_output)); 
                    if( (mcp_b(0) > md_mcp_b(0)) && (mcp_b(1) > md_mcp_b(1)) && (mcp_b(6) > md_mcp_b(6)) && (mcp_b(7) > md_mcp_b(7))
                        || (mcp_b(0) > md_mcp_b(0)) && (mcp_b(1) > md_mcp_b(1)) && (mcp_b(2) > md_mcp_b(2))
                        || (mcp_b(7) > md_mcp_b(7)) && (mcp_b(6) > md_mcp_b(6)) && (mcp_b(5) > md_mcp_b(5)) )
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
                PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d')); 
                Motor(-(slmotor - PID_output), -(srmotor + PID_output));    
                if( analogRead(26) >= md_adc(26)
                      || analogRead(27) >= md_adc(27)+100
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
                    PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d')); 
                    Motor(-(slmotor - PID_output), -(srmotor + PID_output)); 
                    if( analogRead(26) >= md_adc(26)+100
                      || analogRead(27) >= md_adc(27)+100
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
                    PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d')); 
                    Motor(-(spl - PID_output), -(spr + PID_output)); 
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
                      if( analogRead(26) >= md_adc(26)+100
                          || analogRead(27) >= md_adc(27)+100
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
                      PID_output = (kp_slow * kb('p')) + (0.0 * kb('i')) + (ki_slow * kb('d')); 
                      Motor(-(slmotor - PID_output), -(srmotor + PID_output));
                      if( analogRead(26) >= md_adc(26)+100
                            || analogRead(27) >= md_adc(27)+100
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
        delay(2);
      }

    else if (splr == 'p')
      {        
        while(1)
          {  
            PID_output = (kp * kb('p')) + (0.0 * kb('i')) + (kd_b * kb('d'));  
            Motor(-(spl - PID_output), -(spr + PID_output)); 
            if( mcp_b(1) < md_mcp_b(1) && mcp_b(6) < md_mcp_b(6) )  
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
                 Motor(-slmotor,-srmotor); 
                if( mcp_b(1) < md_mcp_b(1) && mcp_b(6) < md_mcp_b(6) )  
                  {
                      break;
                  }                                                               
              }
            delay(10);

            for ( int i = 0; i < sensor_f; i++ )
              {
                 do{ Motor(-((flmr*power)/100), ((flml*power)/100)); } while( mcp_f(i) < md_mcp_f(i) ); delay(2);
             
              }                            
          }
        else
          { 
            if (spl > 0 )
              {
                Motor(slmotor+20 ,srmotor+20) ; delay(break_bc);
                Motor(0,0); delay(2);
              }
            else{}
           
            if (sensor[0] == 'a')
                {
                  if (sensor_f >= 5)
                    {
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_f(i+5) < md_mcp_f(i+5)); delay(2);
                        } 
                    }
                  else
                    {
                      Motor((clml*power)/100,(clmr*power)/100); delay(30);
                      for ( int i = 0; i <= sensor_f; i++ )
                        {
                          do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_f(i) < md_mcp_f(i)); delay(2);
                        } 
                    }
                }
            else
                {
                  for ( int i = 0; i <= sensor_f; i++ )
                    {
                      do{ Motor((clml*power)/100,(clmr*power)/100); } while (mcp_b(i) < md_mcp_f(i)); delay(2);
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
            Motor(1, 1);delay(10);                
          }
      
      }
     else if (splr == 'r')
      {
        if (nfc == 'f')
          {
             while(1)
              {  
                Motor(-slmotor,-srmotor); 
                if( mcp_b(1) > md_mcp_b(1) && mcp_b(6) > md_mcp_b(6) )  
                  {
                      break;
                  }                                                               
              }
            delay(10);
            for ( int i = 7; i > sensor_f; i -- )
              {
                do{ Motor(-((frmr*power)/100), -((frml*power)/100));  } while (mcp_b(i) < md_mcp_b(i));delay(2);  
              }
          }
        else
            {
              if(nfc=='n')
                {
                  Motor(0,0); delay(2);
                }
              else
                {
                  if (spl > 0 )
                    {
                      delay(10);
                      Motor(slmotor ,srmotor) ; delay(break_bc);
                      //Motor(0,0); delay(2);
                    }
                  else
                    {
                      delay(10);
                      Motor(slmotor+20 ,srmotor+20) ; delay(break_bc);
                      //Motor(0,0); delay(2);
                    }
                }
              if (sensor[0] == 'a')
                {
                  if (sensor_f <= 2)
                    {
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_f(i-5) < md_mcp_f(i-5)); delay(2);
                        }
                    }
                  else
                    {
                      Motor((crml*power)/100,(crmr*power)/100); delay(30);
                      for ( int i = 7; i >= sensor_f; i -- )
                        {
                          do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_f(i) < md_mcp_f(i)); delay(2);
                        }
                    }
                }
              else
                {
                  for ( int i = 7; i >= sensor_f; i -- )
                    {
                      do{ Motor((crml*power)/100,(crmr*power)/100);   } while (mcp_b(i) < md_mcp_f(i)); delay(2);
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
            Motor(1, 1);delay(10);
          } 
      }
    else{}

  }

  void bline_setpoint(int spl ,int spr, float kp, int setpoint, String sensor, int endt)
    {
      if(kp == 0)
        {
          I = 0;
          D = 0;
        }
      while(1)
         {
              errors = error_b(setpoint);   // setpoint > 50 จะแทร๊กขวา
              I = 0;
              previous_I = 0;
              previous_error = 0;
              P = errors;
              I = I + errors;
              D = errors - previous_error;                    
              previous_error=errors;  
              PID_output = (kp * P) + (0.0000015 * I) + (0.5* D); 
  
              Motor(-(spl + PID_output),-(spr - PID_output));
              if(sensor == "a0")
                    {
                        if(mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1))
                            {
                                break;
                            }
                    }
                else if(sensor == "a7")
                    {
                        if(mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6))
                            {
                                break;
                            }
                    }
               else if(sensor == "b0")
                    {
                        if(mcp_b(0) < md_mcp_b(0))
                            {
                                break;
                            }
                    }
                else if(sensor == "b7")
                    {
                        if(mcp_b(7) < md_mcp_b(7))
                            {
                                break;
                            }
                    }
               else if(sensor == "26")
                    {
                        if(analogRead(26)<md_adc(26))
                            {
                                break;
                            }
                    }
                else if(sensor == "27")
                    {
                        if(analogRead(27)<md_adc(27))
                            {
                                break;
                            }
                    }
         } 
      if(endt > 0)
         {
            Motor(spl,spr);delay(endt);
            Motor(1,1);delay(10); 
         }
       else
         {}
         
  
    }
  
  void fline_setpoint(int spl ,int spr, float kp, int setpoint, String sensor, int endt)
    {
      
      while(1)
         {
            errors = error_f(setpoint);   // setpoint > 50 จะแทร๊กขวา
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors  ;
            
            PID_output = (kp * P) + (0.0 * I) + (0.2* D); 
  
              Motor(spl - PID_output,spr + PID_output);
              if(sensor == "a0")
                    {
                        if(mcp_f(0)<md_mcp_f(0) && mcp_f(1)<md_mcp_f(1))
                            {
                                break;
                            }
                    }
                else if(sensor == "a7")
                    {
                        if(mcp_f(7)<md_mcp_f(7) && mcp_f(6)<md_mcp_f(6))
                            {
                                break;
                            }
                    }
               else if(sensor == "b0")
                    {
                        if(mcp_b(0) < md_mcp_b(0))
                            {
                                break;
                            }
                    }
                else if(sensor == "b7")
                    {
                        if(mcp_b(7) < md_mcp_b(7))
                            {
                                break;
                            }
                    }
               else if(sensor == "26")
                    {
                        if(analogRead(26)<md_adc(26))
                            {
                                break;
                            }
                    }
                else if(sensor == "27")
                    {
                        if(analogRead(27)<md_adc(27))
                            {
                                break;
                            }
                    }
         } 
      if(endt > 0)
         {
            Motor(-spl,-spr);delay(endt);
            Motor(1,1);delay(10); 
         }
       else
         {}
    }
void fline_2sensor_white(int sl, int sr, float kp, int tm, String line,String sensor, char sp, int ofset)
  {
    char sensors[4];  // Declare a char array to store the converted string
    line.toCharArray(sensors, sizeof(sensors));
    int sensor_l = atoi(&sensors[0]); 
    int sensor_r = atoi(&sensors[2]); 
    if(tm > 0)
      {
        new_encoder = 400 * tm / wheel_distance() ;
        encoder.resetEncoders();
        while(encoder.Poss_R() > -new_encoder)
          {
            float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 50 );
            float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 50 );
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

            Motor(sl - PID_output, sr + PID_output);
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
            PID_output = (kp * P) + (0.00015 * I) + (0.6* D); 

            Motor(sl - PID_output, sr + PID_output);
            //Serial.println(errors);

             if(sensor == "a0")
                  {
                      if(mcp_f(0)> max_mcp_f(0)-200)
                          {
                              break;
                          }
                  }
              else if(sensor == "a7")
                  {
                      if(mcp_f(7)>max_mcp_f(7)-200)
                          {
                              break;
                          }
                  }
             else if(sensor == "b0")
                  {
                      if(mcp_b(0) >max_mcp_b(0)-200)
                          {
                              break;
                          }
                  }
              else if(sensor == "b7")
                  {
                      if(mcp_b(7) > max_mcp_b(7)-200)
                          {
                              break;
                          }
                  }
             else if(sensor == "26")
                  {
                      if(analogRead(26)>max_analogRead(26)-60)
                          {
                              break;
                          }
                  }
              else if(sensor == "27")
                  {
                      if(analogRead(27)>max_analogRead(27)-60)
                          {
                              break;
                          }
                  }
          }
      }

 ///////////////////////////////////////////////////////////////     
    if(sp == 'p')
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
            PID_output = (kp * P) + (0.00015 * I) + (0.2* D); 

            Motor(sl - PID_output, sr + PID_output);
            //Serial.println(errors);

             if(sensor == "a0")
                  {
                      if(mcp_f(0)<md_mcp_f(0))
                          {
                              break;
                          }
                  }
              else if(sensor == "a7")
                  {
                      if(mcp_f(7)<md_mcp_f(7))
                          {
                              break;
                          }
                  }
             else if(sensor == "b0")
                  {
                      if(mcp_b(0) < md_mcp_b(0))
                          {
                              break;
                          }
                  }
              else if(sensor == "b7")
                  {
                      if(mcp_b(7) < md_mcp_b(7))
                          {
                              break;
                          }
                  }
             else if(sensor == "26")
                  {
                      if(analogRead(26)<md_adc(26))
                          {
                              break;
                          }
                  }
              else if(sensor == "27")
                  {
                      if(analogRead(27)<md_adc(27))
                          {
                              break;
                          }
                  }
          }
        unsigned long lasts_timea = millis();
        while(millis() - lasts_timea < 60)
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
            PID_output = (kp * P) + (0.00015 * I) + (0.4* D); 

            Motor(sl - PID_output, sr + PID_output);
            //Serial.println(errors);
  
          }
      }
    else{}
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
