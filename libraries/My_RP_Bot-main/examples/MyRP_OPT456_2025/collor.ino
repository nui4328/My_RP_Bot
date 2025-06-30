
// ฟังก์ชันสำหรับเขียนข้อมูล 16 บิตลง CAT24C128
void writeEEPROM(uint16_t address, uint16_t data) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((uint8_t)(address >> 8));   // MSB
  Wire.write((uint8_t)(address & 0xFF)); // LSB
  Wire.write((uint8_t)(data >> 8));      // MSB of data
  Wire.write((uint8_t)(data & 0xFF));   // LSB of data
  Wire.endTransmission();
  delay(5); // รอการเขียนข้อมูล
}

// ฟังก์ชันสำหรับอ่านข้อมูล 16 บิตจาก CAT24C128
uint16_t readEEPROM(uint16_t address) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((uint8_t)(address >> 8));   // MSB
  Wire.write((uint8_t)(address & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 2);
  uint16_t data = (Wire.read() << 8) | Wire.read();
  return data;
}

// ฟังก์ชันสำหรับเก็บชุดค่าสีแดง (R, G, B)
void saveRed() {
  uint16_t r = my_tcs('r');
  uint16_t g = my_tcs('g');
  uint16_t b = my_tcs('b');
  writeEEPROM(RED_SET_ADDR, r);
  writeEEPROM(RED_SET_ADDR + 2, g);
  writeEEPROM(RED_SET_ADDR + 4, b);
  Serial.println("Stored Red Color Set (R, G, B): " + String(r) + ", " + String(g) + ", " + String(b));
}

// ฟังก์ชันสำหรับเก็บชุดค่าสีเขียว (R, G, B)
void saveGreen() {
  uint16_t r = my_tcs('r');
  uint16_t g = my_tcs('g');
  uint16_t b = my_tcs('b');
  writeEEPROM(GREEN_SET_ADDR, r);
  writeEEPROM(GREEN_SET_ADDR + 2, g);
  writeEEPROM(GREEN_SET_ADDR + 4, b);
  Serial.println("Stored Green Color Set (R, G, B): " + String(r) + ", " + String(g) + ", " + String(b));
}

// ฟังก์ชันสำหรับเก็บชุดค่าสีเหลือง (R, G, B)
void saveYellow() {
  uint16_t r = my_tcs('r');
  uint16_t g = my_tcs('g');
  uint16_t b = my_tcs('b');
  writeEEPROM(YELLOW_SET_ADDR, r);
  writeEEPROM(YELLOW_SET_ADDR + 2, g);
  writeEEPROM(YELLOW_SET_ADDR + 4, b);
  Serial.println("Stored Yellow Color Set (R, G, B): " + String(r) + ", " + String(g) + ", " + String(b));
}

// ฟังก์ชันสำหรับเก็บค่าสีทั้งสามสีโดยใช้ปุ่ม
void storeColors() {
  currentState = RED; // เริ่มที่ RED
  Serial.println("Mode: Store Red color. Press button to capture.");
  
  while (currentState != SEARCH) {
    static int lastButtonState = HIGH;
    int buttonState = digitalRead(BUTTON_PIN);
    
    if (lastButtonState == HIGH && buttonState == LOW) {
      delay(50); // Debounce
      if (digitalRead(BUTTON_PIN) == LOW) {
        switch (currentState) {
          case RED:
            saveRed();
            currentState = GREEN;
            Serial.println("Mode: Store Green color. Press button to capture.");
            break;
          case GREEN:
            saveGreen();
            currentState = YELLOW;
            Serial.println("Mode: Store Yellow color. Press button to capture.");
            break;
          case YELLOW:
            saveYellow();
            currentState = SEARCH;
            colorsStored = true;
            Serial.println("All colors stored. Starting color search.");
            break;
        }
      }
    }
    lastButtonState = buttonState;
    
    // แสดงค่าสีที่ตรวจจับได้แบบเรียลไทม์
    uint16_t r = my_tcs('r');
    uint16_t g = my_tcs('g');
    uint16_t b = my_tcs('b');
    Serial.println("Detected Color:");
    Serial.println("Red: " + String(r));
    Serial.println("Green: " + String(g));
    Serial.println("Blue: " + String(b));
    Serial.println("--------------------");
    delay(100);
  }
}

// ฟังก์ชันสำหรับคำนวณระยะห่าง Euclidean ใน RGB space
float compareColors(uint16_t r1, uint16_t g1, uint16_t b1, uint16_t r2, uint16_t g2, uint16_t b2) {
  return sqrt(pow((int32_t)r1 - r2, 2) + pow((int32_t)g1 - g2, 2) + pow((int32_t)b1 - b2, 2));
}

// ฟังก์ชันสำหรับค้นหาและแสดงผลสีที่ใกล้เคียงที่สุด (เฉพาะชื่อสี)
void searchColor() {
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
  } else if (distGreen <= distRed && distGreen <= distYellow) {
    Serial.println("Green");
  } else {
    Serial.println("Yellow");
  }
}



void _setup_color() {
  Wire.begin(); // เริ่มต้น I2C
  Serial.begin(115200); // เริ่มต้น Serial Monitor
  while (!Serial); // รอจนกว่า Serial จะพร้อม
  pinMode(BUTTON_PIN, INPUT_PULLUP); // ตั้งค่า GP9 เป็น input พร้อม pull-up resistor
  //storeColors(); // เรียกฟังก์ชันเก็บค่าสี

}

void _loop() {
  if (colorsStored) {
    searchColor();
    delay(10); // ค้นหาสีทุก 0.5 วินาที
  }
}