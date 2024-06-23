
void fline_ch_can(int spl ,int spr, float kp, int tim, int endt)    ///////////////--------> เดินเข้าไปคีบกระป๋องแบบวัดระยะ
  {
        fline(spl,spr, 0.8, 100, 'n', 's',80, "a4", 0);
        last_time = millis();
        while(millis()-last_time < tim)
          {
            float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
            float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors; 
            PID_output = (kp * P) + (0.0000015 * I) + (0.5* D); 
            Motor(spl + PID_output,spr - PID_output);
                    
          }
        if(endt > 0)
          {
             Motor(-20,-20);delay(endt);
             Motor(1,1);delay(10); 
          }
        else
        {}
  }
void fline_xl(int spl ,int spr, float kp,  int endt) 
  {
    fline(spl,spr, 0.8, 200, 'n', 's',80, "a4", 0);
    while(1)
      {
            float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
            float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors; 
            PID_output = (kp * P) + (0.0000015 * I) + (0.5* D); 

            Motor(spl + PID_output,spr - PID_output);
            if(mcp_m(4) < 200 && mcp_m(5) < 200)
              {
                Motor(-30, -30);
                delay(10);
                Motor(-1, -1);
                delay(50);
                break;
              }  
       }
     Motor(-25, 70);delay(50);
     do{Motor(-25, 70);}while(mcp_b(2) > md_mcp_b(2));delay(50);
     do{Motor(-25, 70);}while(mcp_f(1) > md_mcp_f(1));delay(10);
     do{Motor(-25, 70);}while(mcp_f(1) < md_mcp_f(1));delay(10);
     do{Motor(-25, 70);}while(mcp_f(2) > md_mcp_f(2));
     Motor(25, -70);delay(endt);
     Motor(1, 1);delay(10);
  }

void fline_xr(int spl ,int spr, float kp,  int endt) 
  {
    fline(spl,spr, 0.8, 200, 'n', 's',80, "a4", 0);
    while(1)
      {
            float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
            float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors;  
            PID_output = (kp * P) + (0.0000015 * I) + (0.5* D); 

            Motor(spl + PID_output,spr - PID_output);
            if(mcp_m(4) < 200 && mcp_m(5) < 200)
              {
                Motor(-20, -25);
                delay(10);
                Motor(1, 1);
                delay(50);
                break;
              }  
       }
     Motor(70, -25);delay(50);
     do{Motor(70, -25);}while(mcp_b(6) > md_mcp_b(6));delay(50);
     do{Motor(70, -25);}while(mcp_f(6) > md_mcp_f(6));delay(10);
     do{Motor(70, -25);}while(mcp_f(6) < md_mcp_f(6));delay(10);
     do{Motor(70, -25);}while(mcp_f(5) > md_mcp_f(5));
     Motor(-80, 25);delay(endt);
     Motor(1, 1);delay(10);
  }


void bline_xr(int spl ,int spr, float kp,  int endt) 
  {
    bline(spl,spr, 0.8, 100, 'n', 's',80, "a4", 1);
    while(1)
      {
            float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 30 );
            float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors;  
            PID_output = (kp * P) + (0.0000015 * I) + (0.2* D); 

            Motor(-(spl - PID_output),-(spr + PID_output));
            if(analogRead(27)<md_adc(27) || analogRead(26)<md_adc(26))
              {
                Motor(1, 1);
                delay(10);
                break;
              }  
       }
     while(1)
      {
            float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 30 );
            float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors;  
            PID_output = (kp * P) + (0.0000015 * I) + (0.2* D); 

            Motor(-(spl/2 - PID_output),-(spr/2 + PID_output));
            if(mcp_m(4) < 200 && mcp_m(5) < 200)
              {                       
                Motor(20, 620);
                delay(10);
                Motor(1, 1);
                delay(20);
                break;
              }  
       }
       
       
     do{Motor(60, -60);}while(mcp_f(7) > md_mcp_f(7));delay(10);
     do{Motor(60, -60);}while(mcp_f(7) < md_mcp_f(7));delay(10);
     do{Motor(60, -60);}while(mcp_f(7) > md_mcp_f(7));delay(10);
     do{Motor(60, -60);}while(mcp_f(4) > md_mcp_f(4));
     Motor(60, 100);delay(endt);
     Motor(1, 1);delay(5);
  }
void bline_xl(int spl ,int spr, float kp,  int endt) 
  {
    bline(spl,spr, 0.8, 100, 'n', 's',80, "a4", 1);
    while(1)
      {
            float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 30 );
            float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors;  
            PID_output = (kp * P) + (0.0000015 * I) + (0.4* D); 

            Motor(-(spl - PID_output),-(spr + PID_output));
            if(analogRead(27)<md_adc(27) || analogRead(26)<md_adc(26))
              {
                Motor(1, 1);
                delay(10);
                break;
              }  
       }
     while(1)
      {
            float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 30 );
            float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors; 
            PID_output = (kp * P) + (0.0000015 * I) + (0.4* D); 

            Motor(-(spl/2 - PID_output),-(spr/2 + PID_output));
            if (mcp_m(4) < 200 && mcp_m(5) < 200)
              {   
                Motor(20, 20);
                delay(10);
                Motor(1, 1);
                delay(20);
                break;
              }  
       }  
     do{Motor(-60, 60);}while(mcp_f(0) > md_mcp_f(0));delay(10);
     do{Motor(-60, 60);}while(mcp_f(0) < md_mcp_f(0));delay(10);
     do{Motor(-60, 60);}while(mcp_f(0) > md_mcp_f(0));delay(10);
     do{Motor(-60, 60);}while(mcp_f(3) > md_mcp_f(3));
     Motor(100, 50);delay(endt);
     Motor(1, 1);delay(5);
  }

void bline11_xl(int spl ,int spr, float kp,  int endt) 
  {
    bline(spl,spr, 0.8, 200, 'n', 's',80, "a4", 1);
    while(1)
      {
            float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 30 );
            float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors;  
            PID_output = (kp * P) + (0.0000015 * I) + (0.2* D); 

            Motor(-(spl - PID_output),-(spr + PID_output));
            if(analogRead(27) < md_adc(27))
              {   
                Motor(40, 40);
                delay(10);
                break;
              }  
       }
    while(1)
      {
            float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 30 );
            float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors;  
            PID_output = (kp * P) + (0.0000015 * I) + (0.4* D); 

            Motor(-(spl - PID_output),-(spr + PID_output));
            if(mcp_m(5) < 200)
              {                           
                Motor(40, 40);
                delay(20);
                break;
              }  
       }
       
     do{Motor(-60, 60);}while(mcp_f(7) > md_mcp_f(7));delay(10);
     do{Motor(-60, 60);}while(mcp_f(7) < md_mcp_f(7));delay(10);
     do{Motor(-60, 60);}while(mcp_f(7) > md_mcp_f(7));delay(10);
     do{Motor(-60, 60);}while(mcp_f(7) < md_mcp_f(7));delay(10);
     do{Motor(-60, 60);}while(mcp_f(7) > md_mcp_f(7));delay(10);
     do{Motor(-60, 60);}while(mcp_f(7) < md_mcp_f(7));delay(10);
     do{Motor(-60, 60);}while(mcp_f(3) > md_mcp_f(3));
     Motor(100, 60);delay(endt);
     Motor(1, 1);delay(5);
  }

void bline12_xr(int spl ,int spr, float kp,  int endt) 
  {
    bline(spl,spr, 0.8, 200, 'n', 's',80, "a4", 1);
    while(1)
      {
            float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 30 );
            float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors;  
            PID_output = (kp * P) + (0.0000015 * I) + (0.2* D); 

            Motor(-(spl - PID_output),-(spr + PID_output));
            if(analogRead(26) < md_adc(26))
              {   
                
                Motor(1, 1);
                delay(10);
                break;
              }  
       }

     while(1)
      {
            float error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 30 );
            float error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + errors;
            D = errors - previous_error;                    
            previous_error=errors; 
            PID_output = (kp * P) + (0.0000015 * I) + (0.2* D); 

            Motor(-(spl - PID_output),-(spr + PID_output));
            if(mcp_m(4) < 200)
              {          
                Motor(40, 40);
                delay(20);
                Motor(1, 1);
                delay(50);
                break;
              }  
       }
       
     do{Motor(60, -60);}while(mcp_f(0) > md_mcp_f(0));delay(10);
     do{Motor(60, -60);}while(mcp_f(0) < md_mcp_f(0));delay(10);
     do{Motor(60, -60);}while(mcp_f(0) > md_mcp_f(0));delay(10);
     do{Motor(60, -60);}while(mcp_f(0) < md_mcp_f(0));delay(10);
     do{Motor(60, -60);}while(mcp_f(0) > md_mcp_f(0));delay(10);
     do{Motor(60, -60);}while(mcp_f(0) < md_mcp_f(0));delay(10);
     do{Motor(60, -60);}while(mcp_f(4) > md_mcp_f(4));
     Motor(60, 100);delay(endt);
     Motor(1, 1);delay(5);
  }
