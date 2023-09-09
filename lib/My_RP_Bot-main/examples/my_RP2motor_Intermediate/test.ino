
void encoder_cout()
  {
    String sen_en = String (encoderPos);  
    mydisplay_background(black);
    mydisplay(sen_en, 20, 30, 2, white);
  }
void display_error()
  {
         String sen_mpu = String (error_Yaw());
         String sen_en = String (encoderPos);    
         mydisplay_background(black);
         mydisplay("mpu "+ sen_mpu, 20, 10, 2, white);
         mydisplay("en "+ sen_en, 20, 40, 2, white);
  }
void sw_start()
  {
    bz(100);
    bz(100);
     while(digitalRead(9)==1)
        {

          //Serial.println(my_tcs('r'));
 
          String sen_0 = String (mcp_f(0));
          String sen_1 = String (mcp_f(1));
          String sen_2 = String (mcp_f(2));
          String sen_3 = String (mcp_f(3));
          String sen_4 = String (mcp_f(4));
          String sen_5 = String (mcp_f(5));
          String sen_6 = String (mcp_f(6));
          String sen_7 = String (mcp_f(7));  
          String mpu = String (error_Yaw());  
          mydisplay_background(black);
          mydisplay(sen_0 +" "+ sen_1+" "+"MPU", 10, 5, 2, white);
          mydisplay(sen_2 +" "+ sen_3+" "+ mpu, 10, 30, 2, white);
          mydisplay(sen_4 +" "+ sen_5, 10, 60, 2, white);
          Serial.print(mcp_f(6));
          Serial.print("  ");
          Serial.println(md_mcp_f(6));
         
            
        } 
     bz(300);
  }
