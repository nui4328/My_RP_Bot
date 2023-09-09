void fline_2sensor(int sl, int sr, float kp, int tm, String line, int sensor, char sp, int ofset)
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
            PID_output = (kp * P) + (0.00015 * I) + (0.6* D); 

            Motor(sl + PID_output, sr - PID_output);
            //Serial.println(errors);
            if(mcp_f(sensor)<md_mcp_f(sensor))
              {
                break;
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

            Motor(sl + PID_output, sr - PID_output);
            //Serial.println(errors);

            if(mcp_f(sensor)>md_mcp_f(sensor))
              {
                break;
              }            
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
