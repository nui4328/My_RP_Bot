void st()
{
      for(int i=0; i<3; i++)
      {   
        fw_a3_a4(50, 50, 0.3, 0.4, 0);
        stop_f26();
        tx_right(40, 25);
        fw_a3_a4(50, 50, 0.3, 0.4, 20);delay(200); 
  
        bw_a3_a4(50, 50, 0.3, 0.4);
        stop_ba0();
        tx_right(40, 25);

        fr();

        fw_a3_a4(60, 60, 0.3, 0.4, 0);
        stop_f26();
        tx_left(40, 35);
        fw_a3_a4(50, 50, 0.3, 0.4, 20);delay(200);  
  
        bw_a3_a4(50, 50, 0.3, 0.4);
        stop_ba0();
        tx_left(40, 35);

        fl();

        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fline(50,50,0.2,0,'c','r',80, "a5", 30);
        fline(50,50,0.2,400,'n','s',80, "a4", 15);delay(200); 
        bline(50,50,0.2,0,'c','r',80, "a5", 30);
        fline(50,50,0.2,0,'c','p',80, "a4", 30);

        fl();

        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fline(50,50,0.2,0,'c','l',80, "a2", 30);
        fline(50,50,0.2,400,'n','s',80, "a4", 15);delay(200); 
        bline(50,50,0.2,0,'c','l',80, "a2", 30);
        fline(50,50,0.2,0,'c','p',80, "a4", 30);

        fr();

      ////----------------------------->> 8_2
        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fw_a3_a4(50, 50, 0.3, 0.4, 0);
        stop_f26();
        tx_right(40, 25);
        fw_a3_a4(50, 50, 0.3, 0.4, 20);delay(200); 
  
        bw_a3_a4(50, 50, 0.3, 0.4);
        stop_ba0();
        tx_right(40, 25);
        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fline(50,50,0.2,0,'c','p',80, "a4", 30);

        fr();

        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fw_a3_a4(60, 60, 0.3, 0.4, 0);
        stop_f26();
        tx_left(40, 35);
        fw_a3_a4(50, 50, 0.3, 0.4, 20);delay(200);  
  
        bw_a3_a4(50, 50, 0.3, 0.4);
        stop_ba0();
        tx_left(40, 35);
        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fline(50,50,0.2,0,'c','p',80, "a4", 30);
        fl();
      }
   
 
}


/*
 void maxaaa()


    {
for(int i=0; i<6; i++)
      {
        fline(50,50,0.2,0,'c','p',80, "a4", 5);
        fw_a3_a4(50, 50, 0.5, 0.4);
        stop_f26();
        tx_right(40, 25);
        fline(50,50,0.2,400,'n','s',80, "a4", 15);delay(200); 
  
        bw_a3_a4(50, 50, 0.5, 0.4);
        stop_ba0();
        tx_right(40, 25);

        fline(50,50,0.2,0,'c','p',80, "a4", 5);
        fw_a3_a4(60, 60, 0.5, 0.4);
        stop_f26();
        tx_left(40, 35);
        fline(50,50,0.2,500,'n','s',80, "a4", 15);delay(200); 
  
        bw_a3_a4(50, 50, 0.5, 0.4);
        stop_ba0();
        tx_left(40, 35);

        fline(50,50,0.2,0,'c','p',80, "a4", 5);
        fw_a3_a4(60, 60, 0.5, 0.4, 0);
        stop_f26();
        tx_left(40, 35);
        fline(50,50,0.2,500,'n','s',80, "a4", 15);delay(200); 
  
        bw_a3_a4(50, 50, 0.5, 0.4);
        stop_ba0();
        tx_left(40, 35);
        
        fline(50,50,0.2,0,'c','p',80, "a4", 5);
        fw_a3_a4(60, 60, 0.5, 0.4,0);
        stop_f26();
        tx_right(40, 25);
        fline(50,50,0.2,500,'n','s',80, "a4", 15);delay(200); 
  
        bw_a3_a4(50, 50, 0.5, 0.4);
        stop_ba0();
        tx_right(40, 25);


        fline(50,50,0.2,0,'c','p',80, "a4", 5);
        fline(60,60,0.2,0,'c','p',80, "a4", 5);
        fline(60,60,0.2,0,'c','p',80, "a4", 5);
        fw_a3_a4(50, 50, 0.5, 0.4);
        stop_f26();
        tx_right(40, 25);
        fline(50,50,0.2,500,'n','s',80, "a4", 15);delay(200); 
          
        bw_a3_a4(50, 50, 0.5, 0.4);
        stop_ba0();
        tx_right(40, 25);

        fline(50,50,0.2,0,'c','p',80, "a4", 5);
        fline(60,60,0.2,0,'c','p',80, "a4", 5);
        fline(60,60,0.2,0,'c','p',80, "a4", 5);
        fw_a3_a4(50, 50, 0.5, 0.4);
        stop_f26();
        tx_right(40, 25);
        fline(50,50,0.2,500,'n','s',80, "a4", 15);delay(200); 
          
        bw_a3_a4(50, 50, 0.5, 0.4);
        stop_ba0();
        tx_right(40, 25);


        fline(50,50,0.2,0,'c','p',80, "a4", 5);
        fline(60,60,0.2,0,'c','p',80, "a4", 5);
        fline(60,60,0.2,0,'c','p',80, "a4", 5);
/        fw_a3_a4(50, 50, 0.5, 0.4,0);
        stop_f26();
        tx_left(40, 35);
        fline(50,50,0.2,500,'n','s',80, "a4", 15);delay(200); 
          
        bw_a3_a4(50, 50, 0.5, 0.4);
        stop_ba0();
        tx_left(40, 35);

        fline(50,50,0.2,0,'c','p',80, "a4", 5);
        fline(60,60,0.2,0,'c','p',80, "a4", 5);
        fline(60,60,0.2,0,'c','p',80, "a4", 5);
        fw_a3_a4(50, 50, 0.5, 0.4,0);
        stop_f26();
        tx_left(40, 35);
        fline(50,50,0.2,500,'n','s',80, "a4", 15);delay(200); 
          
        bw_a3_a4(50, 50, 0.5, 0.4);
        stop_ba0();
        tx_left(40, 35);
       }

 */     
      

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






      
 
