void fw (int spl ,int spr, float kp, int offset)     
  {  
   
       while(1)
          {    
              Serial.println(error_0());
              P = error_0();
              I = I + error_0();
              D = error_0() - previous_error;                    
              previous_error=error_0() ;
            
              PID_output = (kp * P) + (0.0 * I) + (0.2* D); 
              
              Motor(spl - PID_output, spr + PID_output);   
              delayMicroseconds(50);
              if((mcp_0(0)<md_mcp_0(0) && mcp_0(1)<md_mcp_0(1) && mcp_0(3)<md_mcp_0(3))                    
                  || (mcp_0(7)<md_mcp_0(7) && mcp_0(6)<md_mcp_0(6) && mcp_0(4)<md_mcp_0(4))
                )
                  {
                              break;
                  } 
           }

        Motor(-spl, -spl); delay(offset);
        Motor(-1, -1); delay(50);
        Motor(0, 0); delay(50);
        
        
  }   

void bw (int spl ,int spr, float kp, int offset)     
  {  
   
       while(1)
          {    
              Serial.println(error_20());
              P = error_20();
              I = I + error_20();
              D = error_20() - previous_error;                    
              previous_error=error_20() ;
            
              PID_output = (kp * P) + (0.0 * I) + (0.2* D); 
              
              Motor(-(spl + PID_output), -(spr - PID_output));   
              delayMicroseconds(50);
              if((mcp_20(0)<md_mcp_20(0) && mcp_20(1)<md_mcp_20(1) && mcp_20(3)<md_mcp_20(3))                    
                  || (mcp_20(7)<md_mcp_20(7) && mcp_20(6)<md_mcp_20(6) && mcp_20(4)<md_mcp_20(4))
                )
                  {
                              break;
                  } 
           }

        Motor(spl, spl); delay(offset);
        Motor(-1, -1); delay(50);
        Motor(0, 0); delay(50);
        
        
  }   
                   
                   
