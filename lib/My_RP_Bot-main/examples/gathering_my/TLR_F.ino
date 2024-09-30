void TL_F(int spl, String sensor, int endt)
  {
    while(1)
      {
            float error_L = map(mcp_m(1), min_mcp_m(1), max_mcp_m(1), 0, 30 );
            float error_R = map(mcp_m(2), min_mcp_m(2), max_mcp_m(2), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + previous_I;
            D = errors - previous_error ;            
            previous_I=I;
            previous_error=errors  ;  
            PID_output = (0.3 * P) + (0.0000015 * I) + (0.5* D); 

            Motor(25 - PID_output,25 + PID_output);
             if(mcp_m(1) > md_mcp_m(1) && mcp_m(2) > md_mcp_m(2))
                {
                  delay(5); 
                  Motor(-1,-1);delay(5);
                  break;
                }
      }
    do{Motor(-spl/2, spl);}while(mcp_m(3) > md_mcp_m(3)-30);delay(10); 
    if(sensor == "m2")
      {
        do{Motor(-spl/2, spl);}while(mcp_m(2) > md_mcp_m(2)-30);
      }
    else if(sensor == "m1")
      {
        do{Motor(-spl/2, spl);}while(mcp_m(1) > md_mcp_m(1)-30);
      }
    else if(sensor == "m0")
      {
        do{Motor(-spl/2, spl);}while(mcp_m(0) > md_mcp_m(0)-30);
      }
    else if(sensor == "m3")
      {
        do{Motor(-spl/2, spl);}while(mcp_m(3) > md_mcp_m(3)-30);
      }
    else
      {
        do{Motor(-spl/2, spl);}while(mcp_m(3) < md_mcp_m(3));
        //Motor(1,1);delay(10);
        do{Motor(-spl/2, spl);}while(mcp_m(1) > md_mcp_m(1)-30);
      }
    
    Motor(spl, -spl);delay(endt);
    Motor(1,1);delay(2); 
    
  }
 void TR_F(int spl, String sensor, int endt)
  {
    while(1)
      {
            float error_L = map(mcp_m(1), min_mcp_m(1), max_mcp_m(1), 0, 30 );
            float error_R = map(mcp_m(2), min_mcp_m(2), max_mcp_m(2), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + previous_I;
            D = errors - previous_error ;            
            previous_I=I;
            previous_error=errors  ;  
            PID_output = (0.3 * P) + (0.0000015 * I) + (0.5* D); 

            Motor(25 - PID_output,25 + PID_output);
             if(mcp_m(1) > md_mcp_m(1) && mcp_m(2) > md_mcp_m(2))
                {
                  delay(5); 
                  Motor(-1,-1);delay(5);
                  break;
                }
      }
    do{Motor(spl, -spl/2);}while(mcp_m(0) > md_mcp_m(0)-30); delay(10);
    if(sensor == "m1")
      {
        do{Motor(spl, -spl/2);}while(mcp_m(1) > md_mcp_m(1)-30);
      }
    else if(sensor == "m2")
      {
        do{Motor(spl, -spl/2);}while(mcp_m(2) > md_mcp_m(2)-30);
      }
    else if(sensor == "m3")
      {
        do{Motor(spl, -spl/2);}while(mcp_m(3) > md_mcp_m(3)-30);
      }
    else if(sensor == "m0")
      {
        do{Motor(spl, -spl/2);}while(mcp_m(0) > md_mcp_m(0)-30);
      }
    else
      {
        
        do{Motor(spl, -spl/2);}while(mcp_m(0) < md_mcp_m(0));
        //Motor(1,1);delay(10);
        do{Motor(spl, -spl/2);}while(mcp_m(2) > md_mcp_m(2)-30);
      }
    Motor(-spl, spl);delay(endt);
    Motor(1,1);delay(2); 
    
  }
