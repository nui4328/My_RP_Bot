void check_box_to_floor()
  {
    if(color_box == color_floor_R)
      {
        mission_R();
      }
    else if(color_box == color_floor_MD)
      {
        mission_MD();
      }
    else 
      {
        mission_L();
      }
   
  }

void check_box()
  {
    for(int i=0; i<3; i++)
      {
        if(my_tcs('r') >= 1.4 )
        {
          Serial.println("red");
          //bz(100); delay(100);
          rgb(0, 0, 1);
          color_box = "red";
          break;
        }
      else if(my_tcs('g') > 1 && my_tcs('r')< my_tcs('b'))
        {
          Serial.println("green");
         // bz(100); delay(100);
         // bz(300); delay(100);
          rgb(0, 1, 0);
          color_box = "green";
          break;
        }
      
      else if(my_tcs('g') > 1 && my_tcs('r')> my_tcs('b'))
        {
          Serial.println("yello");
          //bz(100); 
          //bz(100);
          //bz(100);delay(100);
          //bz(300); delay(100);
          rgb(0, 1, 1);
          color_box = "yello";
          break;
        }
      else
        {
          color_box = "none";
          Serial.println("none");          
 
        }
      if(color_box == "none")
        {
          servo(23,90);  //หุบ       
          delay(50); 
          servo(23,120);  //หุบ       
          delay(100);
        }

      }
    
  }


/*
void put_colorbox_red()
  {      
     while(digitalRead(9)==1)
        {
           float color[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
           for(int i = 0; i<3; i++)
              {
                Serial.print(color[i]);Serial.print("  ");
              }
           Serial.println(" ");
        }
     bz(200);
     delay(300);         
     float color_box[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
     EEPROM.put(300, color_box); // write data to EEPROM address 0
     EEPROM.commit(); // save changes to EEPROM
    
     float readData_sensor[3];
     EEPROM.get(300, readData_sensor); // read data from EEPROM address 0
     Serial.print("data -> EEPROM : ");
     for (int i = 0; i < 3; i ++) 
        {
           Serial.print(readData_sensor[i]);Serial.print("  ");
        }
     Serial.println("  ");
      
     bz(100);
     bz(100); 

  }
void read_eep_box_red()
  {
     float readData_sensorR[4];
     EEPROM.get(300, readData_sensorR); // read data from EEPROM address 0
     Serial.println("datared -> EEPROM : ");
     for (int i = 0; i < 4; i ++) 
        {
          Serial.print(readData_sensorR[i]);Serial.print("  ");
        }
     Serial.println("  ");
     for(int i=0; i<4; i++)
        {
          boxcolor_red[i]=readData_sensorR[i];
        }
     //bz(100);
     //bz(100);
     Serial.print(" boxcolor_red_r ");Serial.print(boxcolor_red[0]);
     Serial.print(" boxcolor_red_g ");Serial.print(boxcolor_red[1]);
     Serial.print(" boxcolor_red_b ");Serial.println(boxcolor_red[2]);
  }
void put_colorbox_green()
  {      
     while(digitalRead(9)==1)
        {
            float color[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
            for(int i = 0; i<3; i++)
                {
                   Serial.print(color[i]);Serial.print("  ");
                }
             Serial.println(" ");
        }
     bz(200);
     delay(300);
         
     float color_box[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
     EEPROM.put(150, color_box); // write data to EEPROM address 0
     EEPROM.commit(); // save changes to EEPROM
    
     float readData_sensor[3];
     EEPROM.get(150, readData_sensor); // read data from EEPROM address 0
     Serial.print("data -> EEPROM : ");
     for (int i = 0; i < 3; i ++) 
        {
           Serial.print(readData_sensor[i]);Serial.print("  ");
        }
      Serial.println("  ");
      
      bz(100);
      bz(100); 

  }
void read_eep_box_green()
  {
      float readData_sensor[4];
      EEPROM.get(150, readData_sensor); // read data from EEPROM address 0
      Serial.println("datagreen -> EEPROM : ");
      for (int i = 0; i < 4; i ++) 
        {
          Serial.print(readData_sensor[i]);Serial.print("  ");
        }
      Serial.println("  ");
     for(int i=0; i<4; i++)
        {
          boxcolor_green[i]=readData_sensor[i];
        }
      //bz(100);
      //bz(100);
     Serial.print(" boxcolor_green_r ");Serial.print(boxcolor_green[0]);
     Serial.print(" boxcolor_green_g ");Serial.print(boxcolor_green[1]);
     Serial.print(" boxcolor_green_b ");Serial.println(boxcolor_green[2]);
  }
void put_colorbox_blue()
  {      
     while(digitalRead(9)==1)
        {
            float color[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
            for(int i = 0; i<3; i++)
                {
                  Serial.print(color[i]);Serial.print("  ");
                }
            Serial.println(" ");
        }
     bz(200);
     delay(300);
         
     float color_box[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
     EEPROM.put(200, color_box); // write data to EEPROM address 0
     EEPROM.commit(); // save changes to EEPROM
    
     float readData_sensor[3];
     EEPROM.get(200, readData_sensor); // read data from EEPROM address 0
     Serial.print("data -> EEPROM : ");
     for (int i = 0; i < 3; i ++) 
        {
           Serial.print(readData_sensor[i]);Serial.print("  ");
        }
     Serial.println("  ");
      
     //bz(100);
     //bz(100); 

  }
void read_eep_box_blue()
  {
     float readData_sensor[4];
     EEPROM.get(200, readData_sensor); // read data from EEPROM address 0
     Serial.println("data -> EEPROM : ");
     for (int i = 0; i < 3; i ++) 
        {
          Serial.print(readData_sensor[4]);Serial.print("  ");
        }
     Serial.println("  ");
     for(int i=0; i<3; i++)
        {
          boxcolor_blue[i]=readData_sensor[i];
        }
     //bz(100);
     //bz(100);
  }
void put_colorbox_yello()
  {      
     while(digitalRead(9)==1)
        {
            float color[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
            for(int i = 0; i<3; i++)
                {
                  Serial.print(color[i]);Serial.print("  ");
                }
            Serial.println(" ");
        }
     bz(200);
     delay(300);
         
     float color_box[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
     EEPROM.put(250, color_box); // write data to EEPROM address 0
     EEPROM.commit(); // save changes to EEPROM
    
     float readData_sensor[3];
     EEPROM.get(250, readData_sensor); // read data from EEPROM address 0
     Serial.print("data -> EEPROM : ");
     for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readData_sensor[i]);Serial.print("  ");
        }
     Serial.println("  ");
      
     //bz(100);
     //bz(100); 
  }
void read_eep_box_yello()
  {
     float readData_sensor[4];
     EEPROM.get(250, readData_sensor); // read data from EEPROM address 0
     Serial.println("datayello -> EEPROM : ");
     for (int i = 0; i < 4; i ++) 
        {
          Serial.print(readData_sensor[i]);Serial.print("  ");
        }
     Serial.println("  ");
    for(int i=0; i<4; i++)
        {
          boxcolor_yello[i]=readData_sensor[i];
        }
     //bz(100);
     //bz(100);
     Serial.print(" boxcolor_yello_r ");Serial.print(boxcolor_yello[0]);
     Serial.print(" boxcolor_yello_g ");Serial.print(boxcolor_yello[1]);
     Serial.print(" boxcolor_yello_b ");Serial.println(boxcolor_yello[2]);
  }

void read_tcs()
  {
    bz(200);
    while(1)
      {
        float color[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
        for(int i = 0; i<3; i++)
          {
            Serial.print(color[i]);Serial.print("  ");
          }
         Serial.println(" ");
      }
  }
*/
