void fline_dis(int spl, int spr, float kp, int sensor,int dis, int ofset)
  {
    while(1)
      {
         PID_output = (kp * error_F()) + (0.00015 * I) + (kp * D);
         Motor(spl - PID_output, spr + PID_output); 
         if(analogRead(29) > dis)
            {
              break;
            }
         if(sensor == 0)
            {
              if(mcp_f(0)< md_mcp_f(0))
                {
                  break;
                }                
            }
         else if(sensor == 7)
            {
              if(mcp_f(7)< md_mcp_f(7))
                {
                  break;
                } 
            }
      }
    if(ofset > 0)
      {
        Motor(-spl, -spr);
        delay(ofset);
        Motor(0, 0);
        delay(50);
      }
     else{}
  }
