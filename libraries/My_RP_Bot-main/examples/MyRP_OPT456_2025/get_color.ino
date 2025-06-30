

void get_color_arm()
  {
    
    servo(27, servo_27_close);   
     servo(28, servo_28_close);
     delay(300);
     arm_Slide(400);
     _servo(1, 17);
     delay(500);
    get_color_right();
    delay(200);
    _servo(1, 60);delay(200);

    _servo(0, 15);
     delay(500);
    get_color_left();
    delay(200);
    _servo(0, 60);
  }


void get_color_right()
  {
      uint16_t r = my_tcs('r');
      uint16_t g = my_tcs('g');
      uint16_t b = my_tcs('b');
      
      // อ่านค่าสีจาก EEPROM
      uint16_t red_r = readEEPROM(RED_SET_ADDR);
      uint16_t red_g = readEEPROM(RED_SET_ADDR + 2);
      uint16_t red_b = readEEPROM(RED_SET_ADDR + 4);
      
      uint16_t green_r = readEEPROM(GREEN_SET_ADDR);
      uint16_t green_g = readEEPROM(GREEN_SET_ADDR + 2);
      uint16_t green_b = readEEPROM(GREEN_SET_ADDR + 4);
      
      uint16_t yellow_r = readEEPROM(YELLOW_SET_ADDR);
      uint16_t yellow_g = readEEPROM(YELLOW_SET_ADDR + 2);
      uint16_t yellow_b = readEEPROM(YELLOW_SET_ADDR + 4);
      
      // ตรวจสอบว่าค่าสีใน EEPROM มีความสมเหตุสมผล
      bool validRed = (red_r != 0 || red_g != 0 || red_b != 0);
      bool validGreen = (green_r != 0 || green_g != 0 || green_b != 0);
      bool validYellow = (yellow_r != 0 || yellow_g != 0 || yellow_b != 0);
      
      if (!validRed || !validGreen || !validYellow) {
        Serial.println("Error: Some colors not stored properly in EEPROM!");
        return;
      }
      
      // คำนวณระยะห่าง
      float distRed = compareColors(r, g, b, red_r, red_g, red_b);
      float distGreen = compareColors(r, g, b, green_r, green_g, green_b);
      float distYellow = compareColors(r, g, b, yellow_r, yellow_g, yellow_b);
      
      // แสดงเฉพาะชื่อสีที่ใกล้เคียงที่สุด
      if (distRed <= distGreen && distRed <= distYellow) {
        Serial.println("Red");
        color_right = "Red";
        bz(50);
      } else if (distGreen <= distRed && distGreen <= distYellow) {
        Serial.println("Green");
        color_right = "Green";
        bz(50);bz(50);
      } else {
        Serial.println("Yellow");
        color_right = "Yellow";
        bz(50);bz(150);bz(50);
      }
  }

void get_color_left()
  {
      uint16_t r = my_tcs('r');
      uint16_t g = my_tcs('g');
      uint16_t b = my_tcs('b');
      
      // อ่านค่าสีจาก EEPROM
      uint16_t red_r = readEEPROM(RED_SET_ADDR);
      uint16_t red_g = readEEPROM(RED_SET_ADDR + 2);
      uint16_t red_b = readEEPROM(RED_SET_ADDR + 4);
      
      uint16_t green_r = readEEPROM(GREEN_SET_ADDR);
      uint16_t green_g = readEEPROM(GREEN_SET_ADDR + 2);
      uint16_t green_b = readEEPROM(GREEN_SET_ADDR + 4);
      
      uint16_t yellow_r = readEEPROM(YELLOW_SET_ADDR);
      uint16_t yellow_g = readEEPROM(YELLOW_SET_ADDR + 2);
      uint16_t yellow_b = readEEPROM(YELLOW_SET_ADDR + 4);
      
      // ตรวจสอบว่าค่าสีใน EEPROM มีความสมเหตุสมผล
      bool validRed = (red_r != 0 || red_g != 0 || red_b != 0);
      bool validGreen = (green_r != 0 || green_g != 0 || green_b != 0);
      bool validYellow = (yellow_r != 0 || yellow_g != 0 || yellow_b != 0);
      
      if (!validRed || !validGreen || !validYellow) {
        Serial.println("Error: Some colors not stored properly in EEPROM!");
        return;
      }
      
      // คำนวณระยะห่าง
      float distRed = compareColors(r, g, b, red_r, red_g, red_b);
      float distGreen = compareColors(r, g, b, green_r, green_g, green_b);
      float distYellow = compareColors(r, g, b, yellow_r, yellow_g, yellow_b);
      
      // แสดงเฉพาะชื่อสีที่ใกล้เคียงที่สุด
      if (distRed <= distGreen && distRed <= distYellow) {
        Serial.println("Red");
        color_left = "Red";
        bz(50);
      } else if (distGreen <= distRed && distGreen <= distYellow) {
        Serial.println("Green");
        color_left = "Green";
        bz(50);
        bz(50);
      } else {
        Serial.println("Yellow");
        color_left = "Yellow";
        bz(50);
        bz(150);bz(50);
      }
  }