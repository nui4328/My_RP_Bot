

void display_error()
  {
         String sen_mpu = String (mcp_f(0));
         String sen_en = String (mcp_f(1));    
         mydisplay_background(black);
         mydisplay("mpu "+ sen_mpu, 20, 10, 2, white);
         mydisplay("en "+ sen_en, 20, 40, 2, white);
  }
int md_sensors(int sensor)
  {
    int sen_md = (sensor_maxs[sensor] + sensor_mins[sensor]) / 2;
    return sen_md;
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
          String mpu = String ( 0 );  
          mydisplay_background(black);
          mydisplay(sen_0 +" "+ sen_1+" "+sen_2, 10, 5, 2, white);
          mydisplay(sen_3 +" "+ sen_4+" "+ sen_5, 10, 30, 2, white);
          mydisplay(sen_6 +" "+ sen_7, 10, 60, 2, white);
          Serial.print(mcp_f(6));
          Serial.print("  ");
          Serial.println(md_mcp_f(6));
         
            
        } 
     bz(300);
  }
