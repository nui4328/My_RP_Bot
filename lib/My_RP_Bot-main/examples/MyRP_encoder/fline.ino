
float errorss()
  {
    float error_L = map(analogRead(26), min_26, max_26, 0, 40 );
    float error_R = map(analogRead(27), min_27, max_27, 0, 40 );
    float _errorss = error_L - error_R; 
    return _errorss;
  }

void trak_L(int sl, int sr, float kp, int encode)
  {
    encoderPos = 0;
        while(encoderPos < encode)
          {  
            float error_L = map(analogRead(26), min_26, max_26, -10, 40 ); 
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = error_L;
            I = I + previous_I;
            D = error_L - previous_error ;            
            previous_I=I;
            previous_error=error_L  ;  
            PID_output = (kp * P) + (0.0 * I) + (0.5* D);             
            Motor(sl + PID_output, sr - PID_output);
            if(analogRead(27) > min_27+50)
              {
                break;
              }
          }
        Motor(0, 0);
  }
void trak_R(int sl, int sr, float kp)
  {
        while(1)
          {  
            float error_R = map(analogRead(27), min_27, max_27, -10, 30 );
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = error_R;
            I = I + previous_I;
            D = error_R - previous_error ;            
            previous_I=I;
            previous_error=error_R  ;  
            PID_output = (kp * P) + (0.0 * I) + (0.5* D);             
            Motor(sl - PID_output, sr + PID_output);
          }
  }
void fline(int sl, int sr, float kp)
  {
        while(1)
          {   
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errorss();
            I = I + previous_I;
            D = errorss() - previous_error ;            
            previous_I=I;
            previous_error=errorss()  ;  
            PID_output = (kp * P) + (0.0 * I) + (0.5* D);             
            Motor(sl + PID_output, sr - PID_output);
            if(analogRead(28) > 2800)   ///-------------------->>> ถ้าข้างหน้าเจอ
              {
                Motor(-20,-20); delay(20); 
                Motor(0,0); delay(200);
                Motor(0,0); delay(400);
                calibration_Yak();calibration_Yak();
                do{Motor(-30,30);}while(error_Yaw() > -170);  //----------->> (error_Yaw() > -170) กลับตัวไม่ตรงให้แก้ 170 
                Motor(35,-35);delay(20);
                Motor(0,0); delay(100);
                goto en_a; 
              }
            if(analogRead(27) < min_27-250)
              {
                Motor(-20,-20); delay(20);
                Motor(0,0); delay(100);
                bz(200);
                begin_27:
                encoderPos = 0;
                do{Motor(20,20);}while(encoderPos < 1200);
                Motor(-20,-20); delay(20);
                Motor(0,0); delay(400);
                calibration_Yak();calibration_Yak();
                do{Motor(30,-30);}while(error_Yaw() < 90);  //----------->> หมุนตัวไม่ตรงให้แก้ 90
                Motor(-35,35);delay(20);
                Motor(0,0); delay(100);
               
                    encoderPos = 0;
                    while(encoderPos < 1400)
                      { 
                        Motor(20,20);
                        if(analogRead(27) > min_27+50)
                          {
                            while(encoderPos < 1400)
                              {
                                if(analogRead(27) > (min_27+max_27)/2)
                                  {
                                    Motor(23, 20);
                                  }
                                else
                                  {
                                    Motor(20, 23);
                                  }
                              }
                            break;
                          }
                      } 
                    break;    
                  
                
              }
           if(analogRead(26) < min_26-250)
              {
                encoderPos = 0;
                while(encoderPos < 1300)
                  {
                    Motor(20,20);
                    if(analogRead(28) > 2200)
                      {
                        Motor(-20,-20); delay(20); 
                        Motor(0,0); delay(200); 
                        if(analogRead(27) < min_27-50)
                          {
                            goto begin_27;
                          }
                        else if(analogRead(26) < min_26-50)
                          {
                            calibration_Yak();calibration_Yak();
                            do{Motor(-30,30);}while(error_Yaw() > -90);
                            Motor(35,-35);delay(20);
                            Motor(0,0); delay(100);

                            encoderPos = 0;
                            while(encoderPos < 1400)
                              {
                                Motor(20,20);
                                if(analogRead(26) > min_26+50)
                                  {
                                    while(encoderPos < 1400)
                                      {
                                        if(analogRead(26) > (min_26+max_27)/2)
                                          {
                                            Motor(20, 23);
                                          }
                                        else
                                          {
                                            Motor(23, 20);
                                          }
                                      }
                                    break;
                                  }
                               }
                             goto en_a;                              
                          }
                        else
                          {
                            Motor(0,0); delay(400);
                            calibration_Yak();calibration_Yak();
                            do{Motor(-30,30);}while(error_Yaw() > -170);
                            Motor(35,-35);delay(20);
                            Motor(0,0); delay(100);
                            goto en_a;
                          }                        
                      }
                   
                  }
                
              }
            

          }
       en_a:
        Motor(0,0);
        
  }
