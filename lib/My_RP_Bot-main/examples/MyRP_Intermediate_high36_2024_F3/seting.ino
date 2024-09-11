
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
         float red, green, blue;
         uint16_t r, g, b, c;
         tcss.getRawData(&r, &g, &b, &c);
             
         float averag = (r+g+b)/3;
         red = r/averag; ; 
         green = g/averag; 
         blue = b/averag;
        check_collor();
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
        String color = String(color_ch);
        String ccr = String(red);
        String ccg = String(green);
        String ccb = String(blue);
        mydisplay_background(black);
        mydisplay(mcp_f0 +"  "+ mcp_f1 +"  "+ mcp_f2+"  "+ mcp_f3, 5, 10 ,1, white);
        mydisplay(knob, 120, 20 ,1, white);
        mydisplay(mcp_f7 +"  "+ mcp_f6 +"  "+ mcp_f5+"  "+ mcp_f4, 5, 25 ,1, white);
        mydisplay(ccr +"  "+ ccg +"  "+ ccb , 30, 40 ,1, white);
        mydisplay(en_pos+"  "+color_ch, 5, 55 ,2, white);
        /*
        for(int i = 0; i<6; i++)
          {
            Serial.print(mcp_f(i));
            Serial.print("  ");
            delay(10);
          }
        Serial.println(" ");
        */
      }
    bz(500);
    mydisplay_background(black);
    mydisplay("End", 5, 10 ,2, white);    
  }

void begin_robot()
  {
    bz(100);
    bz(100); 
    while(1)
      {        
        mydisplay_background(black);
        mydisplay("Select Mode", 10, 30, 2, white);
        if(analogRead(29) > 500 && analogRead(29) < 1500)
          {
            bz(100);
            while(1)
              {
                mydisplay_background(black);
                mydisplay(" robot-start", 10, 10, 2, white);
                mydisplay("Press button", 10, 40, 2, white);
                mydisplay("    GP9   ", 10, 60, 2, white);
                if(digitalRead(9)==0) 
                  {
                    goto end_begin;
                  }
                if(analogRead(29) > 1500 || analogRead(29) < 500)
                  {
                    break;
                  }
                
              }
          }
        else if(analogRead(29) > 1500 && analogRead(29) < 2500)
          {
            bz(100);
            while(1)
              {
                mydisplay_background(black);
                mydisplay(" Read-sensor", 10, 10, 2, white);
                mydisplay("Press button", 10, 40, 2, white);
                mydisplay("    GP9   ", 10, 60, 2, white);
                if(digitalRead(9)==0) 
                  {
                    bz(100);
                    while(1)
                      {
                         float red, green, blue;
                         uint16_t r, g, b, c;
                         tcss.getRawData(&r, &g, &b, &c);
                             
                         float averag = (r+g+b)/3;
                         red = r/averag; ; 
                         green = g/averag; 
                         blue = b/averag;
                        check_collor();
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
                        String color = String(color_ch);
                        String ccr = String(red);
                        String ccg = String(green);
                        String ccb = String(blue);
                        mydisplay_background(black);
                        mydisplay(mcp_f0 +"  "+ mcp_f1 +"  "+ mcp_f2+"  "+ mcp_f3, 5, 10 ,1, white);
                        mydisplay(knob, 120, 20 ,1, white);
                        mydisplay(mcp_f7 +"  "+ mcp_f6 +"  "+ mcp_f5+"  "+ mcp_f4, 5, 25 ,1, white);
                        mydisplay(ccr +"  "+ ccg +"  "+ ccb , 30, 40 ,1, white);
                        mydisplay(en_pos+"  "+color_ch, 5, 55 ,2, white);
                        /*
                        for(int i = 0; i<6; i++)
                          {
                            Serial.print(mcp_f(i));
                            Serial.print("  ");
                            delay(10);
                          }
                        Serial.println(" ");
                        */
                        if(analogRead(29) > 2500 || analogRead(29) < 1500)
                          {
                            break;
                          }
                      }                    
                  }
               if(analogRead(29) > 2500 || analogRead(29) < 1500)
                  {
                    break;
                  }
                
              }
          }
        else if(analogRead(29) > 2500 && analogRead(29) < 3500)
          {
            bz(100);
            while(1)
              {
                mydisplay_background(black);
                mydisplay(" Cal-Sensor", 10, 10, 2, white);
                mydisplay("Press button", 10, 40, 2, white);
                mydisplay("    GP9   ", 10, 60, 2, white);
                if(digitalRead(9)==0) 
                  {                    
                    mydisplay_background(black);
                    mydisplay("Moving ROBOT", 10, 10, 2, white);
                    mydisplay("     ON     ", 10, 30, 2, white);
                    mydisplay(" black&white ", 10, 50, 2, white);
                    cal_censor(0, 0);
                    break;
                  }
                if(analogRead(29) > 3500 || analogRead(29) < 2500)
                  {
                    break;
                  }
                
              }
          }
        else if(analogRead(29) > 3500 && analogRead(29) < 4000)
          {
            bz(100);
            while(1)
              {
                mydisplay_background(black);
                mydisplay(" Get_Color", 10, 10, 2, white);
                mydisplay("Press button", 10, 40, 2, white);
                mydisplay("    GP9   ", 10, 60, 2, white);
                if(digitalRead(9)==0) 
                  {
                    bz(300); delay(1000);
                    while(digitalRead(9)==1)
                      {
                        mydisplay_background(black);
                        mydisplay(" Get_RED", 10, 10, 2, white);
                        mydisplay("Press button", 10, 40, 2, white);
                        mydisplay("    GP9   ", 10, 60, 2, white);
                      }
                    bz(100);
                    get_red_to_eep();    delay(200);    

                    while(digitalRead(9)==1)
                      {
                        mydisplay_background(black);
                        mydisplay(" Get_GREEN", 10, 10, 2, white);
                        mydisplay("Press button", 10, 40, 2, white);
                        mydisplay("    GP9   ", 10, 60, 2, white);
                      }
                    bz(100);
                    get_green_to_eep();     delay(200);   
                    
                    while(digitalRead(9)==1)
                      {
                        mydisplay_background(black);
                        mydisplay(" Get_BLUE", 10, 10, 2, white);
                        mydisplay("Press button", 10, 40, 2, white);
                        mydisplay("    GP9   ", 10, 60, 2, white);
                      }
                    bz(100);
                    get_blue_to_eep();  delay(200);  
                    
                    while(digitalRead(9)==1)
                      {
                        mydisplay_background(black);
                        mydisplay(" Get_YELLO", 10, 10, 2, white);
                        mydisplay("Press button", 10, 40, 2, white);
                        mydisplay("    GP9   ", 10, 60, 2, white);
                      }
                    bz(100);
                    get_yello_to_eep(); delay(200);  
                    bz(100);
                    bz(100);
                    
                    read_eppcolor();
                    while(1)
                      {
                         float red, green, blue;
                         uint16_t r, g, b, c;
                         tcss.getRawData(&r, &g, &b, &c);
                             
                         float averag = (r+g+b)/3;
                         red = r/averag; ; 
                         green = g/averag; 
                         blue = b/averag;
                        check_collor();
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
                        String color = String(color_ch);
                        String ccr = String(red);
                        String ccg = String(green);
                        String ccb = String(blue);
                        mydisplay_background(black);
                        mydisplay(mcp_f0 +"  "+ mcp_f1 +"  "+ mcp_f2+"  "+ mcp_f3, 5, 10 ,1, white);
                        mydisplay(knob, 120, 20 ,1, white);
                        mydisplay(mcp_f7 +"  "+ mcp_f6 +"  "+ mcp_f5+"  "+ mcp_f4, 5, 25 ,1, white);
                        mydisplay(ccr +"  "+ ccg +"  "+ ccb , 30, 40 ,1, white);
                        mydisplay(en_pos+"  "+color_ch, 5, 55 ,2, white);
                        /*
                        for(int i = 0; i<6; i++)
                          {
                            Serial.print(mcp_f(i));
                            Serial.print("  ");
                            delay(10);
                          }
                        Serial.println(" ");
                        */
                        if(analogRead(29) > 4000 || analogRead(29) < 3500)
                          {
                            break;
                          }
                      }                  
                    
                    
                    break;
                  }
                if(analogRead(29) > 4000 || analogRead(29) < 3500)
                  {
                    break;
                  }
                
              }
          }
      }

     end_begin: delay(10);
     bz(400);
  }
