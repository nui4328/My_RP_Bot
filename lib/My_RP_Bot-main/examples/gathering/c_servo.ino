
float PID_outputs=0,errorss=0,Is=0,Ds=0;


void  servo_up_open()  // ยก กางออก
  {
    servo(8,servo_up );  //กลาง
    
    servo(7,15); //ขวา
    servo(23,165); //ซ้าย  
    
  }

void servo_down_open()   // ลง กางออก
  {
    servo(7,15); //ขวา
    servo(23,165); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }

void  servo_up_close()  // ยก หุบเข้า
  {
     servo(7,135); //ขวา
     servo(23,60); //ซ้าย    
    servo(8,servo_up );  //กลาง
  }

void  servo_down_close()  // ลง หุบเข้า
  {    
     servo(7,135); //ขวา
      servo(23,60); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }

void servo_big()    //
  {
      servo(7,125); //ขวา
      servo(23,70); //ซ้าย     
      servo(8,servo_down );  //กลาง  
  }

void  mission_L()  //  a
     {
          Motor(-1,30);
          delay(180);
          Motor(1,-30);
          delay(20);
          Motor(0,0);
          delay(20);
          
          servo_down_open();
    
          delay(300);
          Motor(1,-30);
          delay(180);
          Motor(-1,30);
          delay(20);
          Motor(0,0);
          delay(20);   
     }
void  mission_R()
     {
          Motor(30,-1);
          delay(180);
          Motor(-30,1);
          delay(20);
          Motor(0,0);
          delay(40);
          
          servo_down_open();
    
          delay(300);
          Motor(-30,1);
          delay(180);
          Motor(30,-1);
          delay(20);
          Motor(0,0);
          delay(20);   
     }
void  mission_L_M()
     {
          Motor(-1,30);
          delay(380);
          Motor(1,-30);
          delay(30);
          Motor(0,0);
          delay(20);
          
          servo_down_open();
    
          delay(300);
          Motor(1,-30);
          delay(390);
          Motor(-1,30);
          delay(20);
          Motor(0,0);
          delay(20);   
     }
     
void  mission_R_M()
     {
          Motor(30,-1);
          delay(380);
          Motor(-30,1);
          delay(30);
          Motor(0,0);
          delay(40);
          
          servo_down_open();
    
          delay(300);
          Motor(-30,1);
          delay(390);
          Motor(30,-1);
          delay(20);
          Motor(0,0);
          delay(20);   
     }
    
void mission(int SPL ,int SPR, float KP,int TIM,String box) 
    {         
          servo(7,80); //ขวา
          servo(23,95); //ซ้าย    
          servo(8,servo_down );  //กลาง
          
          
          delay(100);

           lasts_time_servo=millis();
                 while(millis()-lasts_time_servo<TIM  )
                       {
                         PID_output = (KP * error_F()) + (0.00015 * I) + (0.4* D);
                         Motor(SPL - PID_output,SPR + PID_output);                      /*
                             */                                          
                        }
           Motor(-20,-20);delay(15);
           Motor(0,0);delay(100);

          if (box == "big")
            {
               servo_big();        
            }
          else
            {
                servo_down_close();
            }
          
                  
    }
    
  void slow_servo()
    {
      for(int i = 110; i > 20 ; i-- )
        {
          servo(8, i);
          delay(4);
        }
    }
    
void slow_down()
    {
      for(int i = servo_up; i > servo_down ; i-- )
        {
          servo(8, i);
          delay(4);
        }
    }
void box_throw()
  {
    servo_down_open();
    delay(500);
    fline(60,60,0.5,200,'n','s',100, "a5", 20);
    delay(500); 
    servo(8,servo_up );
    delay(500);

    bline(0,0,0.5,50,'c','r',100, "a5", 20);
    //bline(0,0,0.5,50,'c','l',100, "a2", 20);
  }
  void fw34 (int ml, int mr, float kp, float ki, int tim , int ofset)
    {
        unsigned long lasts_time_cha = millis();
        while(millis() - lasts_time_cha < tim)
          {
            int error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
            int error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + previous_I;
            D = errors - previous_error ;            
            previous_I=I;
            previous_error=errors  ;  
            PID_output = (kp * P) + (0.00015 * I) + (ki* D); 

            Motor(ml + PID_output, mr - PID_output);
            //Serial.println(errors);
  
          }  
        if(ofset > 0)
           {
              Motor(-ml,-mr);delay(ofset);
              Motor(0,0);delay(150);
           }
        else
           {
           } 
                 
    }
void fw34 (int ml, int mr, float kp,  int tim , int sensor, int ofset)
    {
      
        unsigned long lasts_time_cha = millis();
        while(millis() - lasts_time_cha < tim)
          {
              int error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
              int error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
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
  
              Motor(ml + PID_output, mr - PID_output);
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
              else
                  {
                    
                  }
              //Serial.println(errors);
    
          }  
        if(ofset > 0)
           {
              Motor(-ml,-mr);delay(ofset);
              Motor(0,0);delay(150);
           }
        else
           {
           } 
                 
    }

void fw(int sl, int sr, float kp, int tm, int sensor, char sp, int ofset)
  {

    if(tm > 0)
      {
        unsigned long lasts_time = millis();
        while(millis() - lasts_time < tm)
          {
            float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
            float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
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
            float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
            float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
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
  
          }
      }

 ///////////////////////////////////////////////////////////////     
    if(sp == 's')
      {
        Motor(-sl, -sr);
        delay(ofset);
        Motor(0, 0);
        delay(50);
      }
    else
      {
        
        while(1)
          {
            float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
            float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
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
            if(mcp_f(sensor)<md_mcp_f(sensor))
                  {
                    break;
                  }
              
          }
          
        unsigned long lasts_time = millis();
        while(millis() - lasts_time < 50)
          {
            float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
            float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
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
 //////////////////////////////////////////////////////////////     
  }

void fline_L(int sl, int sr, float kp, int tm, int ofset)
  {
    do{Motor(20,5);}while(mcp_f(2)>md_mcp_f(2));
        while(1)
          {
            float error_L = map(mcp_f(1), min_mcp_f(1), max_mcp_f(1), 0, 30 );
            float error_R = map(mcp_f(2), min_mcp_f(2), max_mcp_f(2), 0, 30 );
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
            if(mcp_f(0)<md_mcp_f(0)&&mcp_f(2)<md_mcp_f(2))
              {
                break;
              }
            
          }
        unsigned long lasts_time = millis();
        while(millis() - lasts_time < tm)
          {
            float error_L = map(mcp_f(1), min_mcp_f(1), max_mcp_f(1), 0, 30 );
            float error_R = map(mcp_f(2), min_mcp_f(2), max_mcp_f(2), 0, 30 );
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
      Motor(-sl, -sr);
      delay(ofset);
      Motor(0, 0);
  }
  void fline_R(int sl, int sr, float kp, int tm, int ofset)
  {
    do{Motor(5,20);}while(mcp_f(5)>md_mcp_f(5));
        while(1)
          {
            float error_L = map(mcp_f(5), min_mcp_f(5), max_mcp_f(5), 0, 30 );
            float error_R = map(mcp_f(6), min_mcp_f(6), max_mcp_f(6), 0, 30 );
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
            if(mcp_f(7)<md_mcp_f(7)&&mcp_f(5)<md_mcp_f(5))
              {
                break;
              }
            
          }
        unsigned long lasts_time = millis();
        while(millis() - lasts_time < tm)
          {
            float error_L = map(mcp_f(5), min_mcp_f(5), max_mcp_f(5), 0, 30 );
            float error_R = map(mcp_f(6), min_mcp_f(6), max_mcp_f(6), 0, 30 );
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
      Motor(-sl, -sr);
      delay(ofset);
      Motor(0, 0);
  }
