void check_collor()
  {   
    
         float red, green, blue;
         uint16_t r, g, b, c;
         tcss.getRawData(&r, &g, &b, &c);
             
         float averag = (r+g+b)/3;
         red = r/averag; ; 
         green = g/averag; 
         blue = b/averag;
                
         if(red < red_eep[0]+0.20 && red > red_eep[0]- 0.20  
            && green < red_eep[1]+0.20 && green > red_eep[1]- 0.20 
            && blue < red_eep[2]+0.20 && blue > red_eep[2]- 0.20  )
            {
              color_ch = "red";    
            }
         else if(red < green_eep[0]+0.20 && red > green_eep[0]- 0.20  
            && green < green_eep[1]+0.20 && green > green_eep[1]- 0.20 
            && blue < green_eep[2]+0.20 && blue > green_eep[2]- 0.20  )
            {
              color_ch = "green";                        
            } 
   
         else if(red < blue_eep[0]+0.20 && red > blue_eep[0]- 0.20  
            && green < blue_eep[1]+0.20 && green > blue_eep[1]- 0.20 
            && blue < blue_eep[2]+0.20 && blue > blue_eep[2]- 0.20  )
            {
              color_ch = "blue";
            }
         else if(red < yello_eep[0]+0.20 && red > yello_eep[0]- 0.20  
            && green < yello_eep[1]+0.20 && green > yello_eep[1]- 0.20 
            && blue < yello_eep[2]+0.20 && blue > yello_eep[2]- 0.20  )
            {
              color_ch = "yello";
            }
          else
            {
              color_ch = "none";
            }
  }

void get_color()
  {
     get_red_to_eep();        
     sw_start();
     get_green_to_eep();      
     sw_start();
     get_blue_to_eep();
     sw_start();
     get_yello_to_eep();
  }

void get_red_to_eep()
  {    
    delay(2000);
    uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    
    for (int i = 0; i < 3; i++) {
        Serial.print(collor[i]); 
        Serial.print(", ");
    }
    Serial.println("");
    
    // บันทึกค่า r, g, b ลง EEPROM
    uint16_t red_eeps[3] = {r, g, b};
    EEPROM.put(101, red_eeps); // บันทึกทั้งอาเรย์ red_eeps เริ่มที่อยู่ 101
    EEPROM.commit(); // ยืนยันการบันทึก
    delay(10);
    
    // อ่านค่าที่บันทึกไว้ใน EEPROM
    uint16_t readRED_eep[3]; // ประกาศอาเรย์ขนาด 3 สำหรับเก็บค่าที่อ่านจาก EEPROM
    EEPROM.get(101, readRED_eep); // อ่านข้อมูลจาก EEPROM ที่อยู่ 101
    delay(10);
    
    // แสดงค่าที่อ่านออกมาจาก EEPROM
    for (int i = 0; i < 3; i++) {
        Serial.print(readRED_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("");
    
    // คำนวณค่าเฉลี่ยและปกติของค่า RGB
    float red_averag = (readRED_eep[0] + readRED_eep[1] + readRED_eep[2]) / 3.0;
    red_eep[0] = readRED_eep[0] / red_averag;
    red_eep[1] = readRED_eep[1] / red_averag;
    red_eep[2] = readRED_eep[2] / red_averag;
    
    Serial.println(red_averag); // แสดงค่าเฉลี่ย
    
    // แสดงค่าที่ปกติแล้ว
    for (int i = 0; i < 3; i++) {
        Serial.print(red_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("  ");

      bz(100);
      bz(100);
  }

void get_green_to_eep()
  {   
    delay(2000);
    uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    
    for (int i = 0; i < 3; i++) {
        Serial.print(collor[i]); 
        Serial.print(", ");
    }
    Serial.println("");
    
    // บันทึกค่า r, g, b ลง EEPROM
    uint16_t green_eeps[3] = {r, g, b};
    EEPROM.put(201, green_eeps); // บันทึกทั้งอาเรย์ red_eeps เริ่มที่อยู่ 101
    EEPROM.commit(); // ยืนยันการบันทึก
    delay(10);
    
    // อ่านค่าที่บันทึกไว้ใน EEPROM
    uint16_t readGREEN_eep[3]; // ประกาศอาเรย์ขนาด 3 สำหรับเก็บค่าที่อ่านจาก EEPROM
    EEPROM.get(201, readGREEN_eep); // อ่านข้อมูลจาก EEPROM ที่อยู่ 101
    delay(10);
    
    // แสดงค่าที่อ่านออกมาจาก EEPROM
    for (int i = 0; i < 3; i++) {
        Serial.print(readGREEN_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("");
    
    // คำนวณค่าเฉลี่ยและปกติของค่า RGB
    float green_averag = (readGREEN_eep[0] + readGREEN_eep[1] + readGREEN_eep[2]) / 3.0;
    green_eep[0] = readGREEN_eep[0] / green_averag;
    green_eep[1] = readGREEN_eep[1] / green_averag;
    green_eep[2] = readGREEN_eep[2] / green_averag;
    
    Serial.println(green_averag); // แสดงค่าเฉลี่ย
    
    // แสดงค่าที่ปกติแล้ว
    for (int i = 0; i < 3; i++) {
        Serial.print(green_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("  ");


      bz(100);
      bz(100);
  }

void get_blue_to_eep()
  {     
    delay(2000);
      uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    
    for (int i = 0; i < 3; i++) {
        Serial.print(collor[i]); 
        Serial.print(", ");
    }
    Serial.println("");
    
    // บันทึกค่า r, g, b ลง EEPROM
    uint16_t blue_eeps[3] = {r, g, b};
    EEPROM.put(301, blue_eeps); // บันทึกทั้งอาเรย์ red_eeps เริ่มที่อยู่ 101
    EEPROM.commit(); // ยืนยันการบันทึก
    delay(10);
    
    // อ่านค่าที่บันทึกไว้ใน EEPROM
    uint16_t readGREEN_eep[3]; // ประกาศอาเรย์ขนาด 3 สำหรับเก็บค่าที่อ่านจาก EEPROM
    EEPROM.get(301, readBLUE_eep); // อ่านข้อมูลจาก EEPROM ที่อยู่ 101
    delay(10);
    
    // แสดงค่าที่อ่านออกมาจาก EEPROM
    for (int i = 0; i < 3; i++) {
        Serial.print(readBLUE_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("");
    
    // คำนวณค่าเฉลี่ยและปกติของค่า RGB
    float blue_averag = (readBLUE_eep[0] + readBLUE_eep[1] + readBLUE_eep[2]) / 3.0;
    blue_eep[0] = readBLUE_eep[0] / blue_averag;
    blue_eep[1] = readBLUE_eep[1] / blue_averag;
    blue_eep[2] = readBLUE_eep[2] / blue_averag;
    
    Serial.println(blue_averag); // แสดงค่าเฉลี่ย
    
    // แสดงค่าที่ปกติแล้ว
    for (int i = 0; i < 3; i++) {
        Serial.print(blue_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("  ");


      bz(100);
      bz(100);
  }

void get_yello_to_eep()
  {   
    delay(2000);
         uint16_t r, g, b, c;
    tcss.getRawData(&r, &g, &b, &c); // ดึงค่าข้อมูลสี RGB
    uint16_t collor[3] = {r, g, b};
    
    for (int i = 0; i < 3; i++) {
        Serial.print(collor[i]); 
        Serial.print(", ");
    }
    Serial.println("");
    
    // บันทึกค่า r, g, b ลง EEPROM
    uint16_t yello_eeps[3] = {r, g, b};
    EEPROM.put(401, yello_eeps); // บันทึกทั้งอาเรย์ red_eeps เริ่มที่อยู่ 101
    EEPROM.commit(); // ยืนยันการบันทึก
    delay(10);
    
    // อ่านค่าที่บันทึกไว้ใน EEPROM
    uint16_t readYELLO_eep[3]; // ประกาศอาเรย์ขนาด 3 สำหรับเก็บค่าที่อ่านจาก EEPROM
    EEPROM.get(401, readYELLO_eep); // อ่านข้อมูลจาก EEPROM ที่อยู่ 101
    delay(10);
    
    // แสดงค่าที่อ่านออกมาจาก EEPROM
    for (int i = 0; i < 3; i++) {
        Serial.print(readYELLO_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("");
    
    // คำนวณค่าเฉลี่ยและปกติของค่า RGB
    float yello_averag = (readYELLO_eep[0] + readYELLO_eep[1] + readYELLO_eep[2]) / 3.0;
    yello_eep[0] = readYELLO_eep[0] / yello_averag;
    yello_eep[1] = readYELLO_eep[1] / yello_averag;
    yello_eep[2] = readYELLO_eep[2] / yello_averag;
    
    Serial.println(yello_averag); // แสดงค่าเฉลี่ย
    
    // แสดงค่าที่ปกติแล้ว
    for (int i = 0; i < 3; i++) {
        Serial.print(yello_eep[i]);
        Serial.print("  ");
        delay(10);
    }
    Serial.println("  ");


      bz(100);
      bz(100);
  }

void read_eppcolor()
  {

      EEPROM.get(101, readRED_eep); 
      EEPROM.get(201, readGREEN_eep); 
      EEPROM.get(301, readBLUE_eep); 
      EEPROM.get(401, readYELLO_eep);         
      for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readRED_eep[i]);
         Serial.print("  ");
         delay(10);
        }
      Serial.println("  ");

       for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readGREEN_eep[i]);
         Serial.print("  ");
         delay(10);
        }
      Serial.println("  ");

       for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readBLUE_eep[i]);
         Serial.print("  ");
         delay(10);
        }
      Serial.println("  ");

       for (int i = 0; i < 3; i ++) 
        {
         Serial.print(readYELLO_eep[i]);
         Serial.print("  ");
         delay(10);
        }
      Serial.println("  ");

      float red_averag = (readRED_eep[0] + readRED_eep[1] + readRED_eep[2])/3;
         red_eep[0] = readRED_eep[0]/red_averag;
         red_eep[1] = readRED_eep[1]/red_averag;
         red_eep[2] = readRED_eep[2]/red_averag;
     
         float green_averag = (readGREEN_eep[0] + readGREEN_eep[1] + readGREEN_eep[2])/3;
         green_eep[0] = readGREEN_eep[0]/green_averag;
         green_eep[1] = readGREEN_eep[1]/green_averag;
         green_eep[2] = readGREEN_eep[2]/green_averag;

         
         float blue_averag = (readBLUE_eep[0] + readBLUE_eep[1] + readBLUE_eep[2])/3;
         blue_eep[0] = readBLUE_eep[0]/blue_averag;
         blue_eep[1] = readBLUE_eep[1]/blue_averag;
         blue_eep[2] = readBLUE_eep[2]/blue_averag;

         
         float yello_averag = (readYELLO_eep[0] + readYELLO_eep[1] + readYELLO_eep[2])/3;
         yello_eep[0] = readYELLO_eep[0]/yello_averag;
         yello_eep[1] = readYELLO_eep[1]/yello_averag;
         yello_eep[2] = readYELLO_eep[2]/yello_averag;
      
  }

void mision_color()
  {
    //num_encoder = encoderPos;
    
    Motor(-20, -20); delay(20);
    Motor(1, 1);  delay(80);
    Motor(0, 0);    delay(200);
    check_collor();check_collor();
    Motor(-20, -20); delay(100);
    Motor(20, 20); delay(10);
    Motor(0, 0);    delay(80);
    num_encoder = encoderPos;
   
    
    if(color_ch == "red")
      {
        servo_red();         
      }
    else if(color_ch == "green")
      {
        servo_green();         
      }
    else if(color_ch == "blue")
      {
        servo_blue();         
      }
    do{Motor(-20, -20);}while(mcp_f(0) < md_mcp_f(0)&& mcp_f(3) < md_mcp_f(3));
    while(1)
              {
                if(mcp_f(0) < md_mcp_f(0) && mcp_f(3) > md_mcp_f(3)) 
                  {
                     Motor(-20, 45);        
                  }
                else if(mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3))
                  {
                    Motor(45, -20);           
                  }
                else if(mcp_f(0) < md_mcp_f(0) && mcp_f(3) < md_mcp_f(3))
                  {   
                    Motor(-20, -20);delay(10);
                    Motor(1, 1);delay(100);
                    break;
                  }
                else
                  {
                    Motor(20, 20);
                  }
              }
    encoderPos = 0;

    /*
    if(LR_free == true)
      {
        if(num_encoder < 800)
          {
            do{Motor(-40, -40);}while(encoderPos > -1550);
          }
        else
          {
            do{Motor(-40, -40);}while(encoderPos > -(num_encoder+1100));
          }        
      }
    else
      {
        if(num_encoder < 800)
          {
            do{Motor(-40, -40);}while(encoderPos > -1550);
          }
        else
          {
            do{Motor(-40, -40);}while(encoderPos > -(num_encoder+100));
          }
        
      }
    */

    do{Motor(-40, -40);}while(encoderPos > -1200);
    num_encoder = 0;
    Motor(30, 30); delay(20);
    Motor(-1, -1); delay(30);
    Motor(0, 0);   delay(100);  
    encoderPos = 0;
    do{Motor(-70, 70);}while(encoderPos > -(rotate_color));   //------------------>> หมุ่นซ้าย 
    Motor(30, -30); delay(30);  Motor(1, 1); delay(100); 
     
  }

void mision_color_yello()
  {
    Motor(20, 20); delay(80);
    num_encoder = encoderPos;
    Motor(-10, -10); delay(10);
    Motor(-1, -1);  delay(80);
    Motor(0, 0);   delay(80);
    
    color_ch = "none";
    check_collor();
    if(color_ch == "yello")
      {
         Motor(-20, -20); delay(130);
         Motor(20, 20); delay(10);
         Motor(0, 0);   delay(10);
         bz(100); bz(100); 
         servo_yello();delay(100);
      }
     else
      {
         ch_poit ++;
         bz(100);
         Motor(-1, -1); delay(30);
         ch_point = true; 
         LR_free = false;
         if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
          {
            bz(300);bz(300);bz(300);
            servo(27, 20);
            delay(300000);       
          }
         goto non_yello;
      }     
    encoderPos = 0;
    if(LR_free == true)
      {
        if(num_encoder < 800)
          {
            do{Motor(-40, -40);}while(encoderPos > -1450);
          }
        else
          {
            do{Motor(-40, -40);}while(encoderPos > -(num_encoder+300));
          }        
      }
    else
      {
        if(num_encoder < 800)
          {
            do{Motor(-40, -40);}while(encoderPos > -1450);
          }
        else
          {
            do{Motor(-40, -40);}while(encoderPos > -(num_encoder+100));
          }
      }
    num_encoder = 0;
    Motor(30, 30); delay(20);
    Motor(-1, -1); delay(30);
    Motor(0, 0);   delay(100);
    encoderPos = 0; 
    do{Motor(-70, 70);}while(encoderPos > -(rotate_color));   //------------------>> หมุ่นซ้าย 
    Motor(30, -30); delay(30);  Motor(1, 1); delay(100);  
    non_yello: delay(10);  
  }
void read_floor()
  {
    while(1)
      {
        check_collor();
      }
  }
