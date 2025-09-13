void test_bord()
  {
    
    for(int i=0; i<3; i++)
      {
        servo(0, 160); servo(8, 160); servo(1, 160);delay(1000);
        servo(0, 10);servo(8, 10); servo(1, 10); delay(1000);
      }
    bz(200);
  
    while(1)
      {
        Serial.print("Encoder_L: ");
        Serial.print(encoder.Poss_L());
        Serial.print("     ");
        Serial.print("Encoder_R: ");
        Serial.println(encoder.Poss_R());
    
        delay(10);  // Delay for better readability in serial out
        if(digitalRead(9) == 0)
          {
            break;
          }
      }
    bz(200);
    while(digitalRead(9)== 1)
      {
        Motor(30, 30);delay(1000);
        Motor(-30, -30);delay(1000);
        Motor(0, 0);delay(1000);
        
      }
    bz(200);
    while(1)
      {
        Serial.print(my_tcs('r'));
        Serial.print("  ");
        Serial.print(my_tcs('g'));
        Serial.print("  ");
        Serial.println(my_tcs('b'));
    
        delay(10);  // Delay for better readability in serial out
        if(digitalRead(9) == 0)
          {
            break;
          }
      }
    bz(200);
  }
  
void encoder_poss()
  {    
    Serial.print("Encoder_L: ");
    Serial.print(encoder.Poss_L());
    Serial.print("     ");
    Serial.print("Encoder_R: ");
    Serial.println(encoder.Poss_R());

    delay(100);  // Delay for better readability in serial out
  }

void fw_ch_line(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(mcp_f(0) < md_mcp_f(0)-50 && mcp_f(7) > md_mcp_f(7)-50)
              {
                Motor(-5 ,20);
              }
            else if(mcp_f(0) > md_mcp_f(0)-50 && mcp_f(7) < md_mcp_f(7)-50)
              {
                Motor(20 ,-5);
              }
            else if(mcp_f(0) > md_mcp_f(0)-50 && mcp_f(7) > md_mcp_f(7)-50)
              {          
                Motor(15 ,15);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
        if(num > 1)
          {
            Motor(-15 ,-15);
            delay(50);
            Motor(-1 ,-1);
          }
      }
    
  }

void bw_ch_line(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(mcp_b(0) < md_mcp_b(0)-50 && mcp_b(7) > md_mcp_b(7)-50)
              {
                Motor(-20 ,5);
              }
            else if(mcp_b(0) > md_mcp_b(0)-50 && mcp_b(7) < md_mcp_b(7)-50)
              {
                Motor(5 ,-20);
              }
            else if(mcp_b(0) > md_mcp_b(0)-50 && mcp_b(7) > md_mcp_b(7)-50)
              {          
                Motor(-15 ,-20);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
        if(num > 1)
          {
            Motor(10 ,10);
            delay(50);
            Motor(-1 ,-1);
          }
      }
    
  }

