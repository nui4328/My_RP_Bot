
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
     while(digitalRead(9)==1)
        {
          //Serial.println(my_tcs('r'));

          String sen_mpu = String (error_Yaw());
          String sen_en = String (encoderPos);
         
          String rang_1 = String (analogRead(28));  
          /* String rang_2 = String (range2);  
          String rang_3 = String (range3);  
          */    
          mydisplay_background(black);
          mydisplay("mpu "+ sen_mpu, 30, 10, 2, white);
          mydisplay("en "+ sen_en, 30, 30, 2, white);
        
          mydisplay("en "+ rang_1, 30, 50, 2, white);
          /* mydisplay("en "+ rang_2, 30, 40, 1, white);
          mydisplay("en "+ rang_3, 30, 50, 1, white);
          */
        
        } 
     bz(300);
  }
