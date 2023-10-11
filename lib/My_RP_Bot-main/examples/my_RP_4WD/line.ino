
         //float P, I, D, previous_I, previous_error,errors, PID_output, present_position, previous_integral; 
         /*
          errors = -----------;
          I = 0;
          previous_I = 0;
          previous_error = 0;
          P = errors;
          I = I + previous_I;
          D = errors - previous_error ;            
          previous_I=I;
          previous_error=errors  ;  
          PID_output = (5 * P) + (0.00015 * I) +(15* D); 
         */
float position_FW()
  {
    float position_L = map(mcp_f(0), sensor_maxs[0], sensor_mins[0], 0, 30);
    float position_R = map(mcp_f(1), sensor_maxs[1], sensor_mins[1], 0, 30);
    return position_L - position_R;
  }
void FW_line()
  {
    
  }
