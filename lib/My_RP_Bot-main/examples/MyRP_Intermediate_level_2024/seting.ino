
void encoder_cout()
  {
    String sen_en = String (encoderPos);  
    mydisplay_background(black);
    mydisplay(sen_en, 20, 30, 2, white);
  }
void display_error()
  {
        // String sen_mpu = String (error_Yaw());
         String sen_en = String (encoderPos);    
         mydisplay_background(black);
       //  mydisplay("mpu "+ sen_mpu, 20, 10, 2, white);
         mydisplay("en "+ sen_en, 20, 40, 2, white);
  }
void sw_start()
  {
    bz(100);
    bz(100);    

    while(digitalRead(9)==1 )
      {
        if(analogRead(29) > 4000)
           {
            cal_censor(0, 0);
           }
        String mcp_f0 = String(mcp_f(0));
        String mcp_f1 = String(mcp_f(1));
        String mcp_f2 = String(mcp_f(2));
        String mcp_f3 = String(mcp_f(3));
        String mcp_f4 = String(mcp_f(4));
        String mcp_f5 = String(mcp_f(5));
        String mcp_f6 = String(mcp_f(6));
        String mcp_f7 = String(mcp_f(7));
        String en_pos = String(encoderPos);
        String knob = String(analogRead(29));
        mydisplay_background(black);
        mydisplay(mcp_f0 +"  "+ mcp_f1 +"  "+ mcp_f2+"  "+ mcp_f3, 5, 10 ,1, white);
        mydisplay(knob, 120, 20 ,1, white);
        mydisplay(mcp_f7 +"  "+ mcp_f6 +"  "+ mcp_f5+"  "+ mcp_f4, 5, 30 ,1, white);
        mydisplay(en_pos, 5, 50 ,2, white);
        for(int i = 0; i<6; i++)
          {
            Serial.print(mcp_f(i));
            Serial.print("  ");
            delay(10);
          }
        Serial.println(" ");
      }
    bz(500);
    mydisplay_background(black);
    mydisplay("End", 5, 10 ,2, white);

    
  }
