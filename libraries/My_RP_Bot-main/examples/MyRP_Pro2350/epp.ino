/*
   get_maxmin_A();
   get_maxmin_B();
   get_maxmin_C(); 
   read_eepA();
   read_sensorA_program();
   read_eepB();
   read_sensorB_program();
   read_eepC();
   read_sensorC_program();
*/


// ฟังก์ชันเขียนข้อมูลลง EEPROM
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8)); // ส่ง MSB ของที่อยู่
  Wire.write((int)(eeAddress & 0xFF)); // ส่ง LSB ของที่อยู่
  for (int i = 0; i < dataLength; i++) {
    Wire.write(data[i]); // ส่งข้อมูลทีละไบต์
  }
  Wire.endTransmission();
  delay(5); // รอให้ EEPROM เขียนข้อมูลเสร็จ
}

// ฟังก์ชันอ่านข้อมูลจาก EEPROM
void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8)); // ส่ง MSB ของที่อยู่
  Wire.write((int)(eeAddress & 0xFF)); // ส่ง LSB ของที่อยู่
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, dataLength); // ขอข้อมูล
  for (int i = 0; i < dataLength; i++) {
    if (Wire.available()) {
      buffer[i] = Wire.read(); // อ่านข้อมูลทีละไบต์
    }
  }
}



void get_maxmin_A() {
  // อ่านค่าและเก็บไว้สำหรับ read_sensorA
  for (int sample = 0; sample < numSamples; sample++) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      sensorValuesA[sensor][sample] =  read_sensorA(sensor); // สมมติว่ามีฟังก์ชันนี้
      delay(1);
    }
  }

  // คำนวณ Max และ Min ของแต่ละเซนเซอร์
  for (int sensor = 0; sensor < numSensors; sensor++) {
    sensorMaxA[sensor] = sensorValuesA[sensor][0];
    sensorMinA[sensor] = sensorValuesA[sensor][0];
    
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesA[sensor][sample];
      if (value > sensorMaxA[sensor]) {
        sensorMaxA[sensor] = value;
      }
      if (value < sensorMinA[sensor]) {
        sensorMinA[sensor] = value;
      }
    }
  }

  // บันทึก sensorMaxA และ sensorMinA ลง EEPROM
  byte buffer[16];
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMaxA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 0, buffer, 16); // sensorMaxA ที่ที่อยู่ 0

  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMinA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 16, buffer, 16); // sensorMinA ที่ที่อยู่ 16

  tone(32, 950, 100);
  delay(200);
  tone(32, 950, 200);
  delay(200);

  // แสดงผล
  Serial.println("Sensor A Results:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(sensorMaxA[sensor]);
    Serial.print(", Min: ");
    Serial.println(sensorMinA[sensor]);
  }

  // อ่านและตรวจสอบจาก EEPROM
  byte readBuffer[16];
  int readMaxA[numSensors], readMinA[numSensors];
  readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor A Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(readMaxA[sensor]);
    Serial.print(", Min: ");
    Serial.println(readMinA[sensor]);
  }
}

void get_maxmin_B() {
  // อ่านค่าและเก็บไว้สำหรับ read_sensorB
  for (int sample = 0; sample < numSamples; sample++) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      sensorValuesB[sensor][sample] = read_sensorB(sensor); // สมมติว่ามีฟังก์ชันนี้
      delay(1);
    }
  }

  // คำนวณ Max และ Min ของแต่ละเซนเซอร์
  for (int sensor = 0; sensor < numSensors; sensor++) {
    sensorMaxB[sensor] = sensorValuesB[sensor][0];
    sensorMinB[sensor] = sensorValuesB[sensor][0];
    
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesB[sensor][sample];
      if (value > sensorMaxB[sensor]) {
        sensorMaxB[sensor] = value;
      }
      if (value < sensorMinB[sensor]) {
        sensorMinB[sensor] = value;
      }
    }
  }

  // บันทึก sensorMaxB และ sensorMinB ลง EEPROM
  byte buffer[16];
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMaxB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 32, buffer, 16); // sensorMaxB ที่ที่อยู่ 32

  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMinB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 48, buffer, 16); // sensorMinB ที่ที่อยู่ 48

  tone(32, 950, 100);
  delay(200);
  tone(32, 950, 200);
  delay(200);

  // แสดงผล
  Serial.println("Sensor B Results:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(sensorMaxB[sensor]);
    Serial.print(", Min: ");
    Serial.println(sensorMinB[sensor]);
  }

  // อ่านและตรวจสอบจาก EEPROM
  byte readBuffer[16];
  int readMaxB[numSensors], readMinB[numSensors];
  readEEPROM(EEPROM_ADDRESS, 32, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMaxB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 48, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMinB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor B Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(readMaxB[sensor]);
    Serial.print(", Min: ");
    Serial.println(readMinB[sensor]);
  }
}

void get_maxmin_C() {
  // อ่านค่าและเก็บไว้สำหรับ analogRead(46) และ analogRead(47)
  for (int sample = 0; sample < numSamples; sample++) {
    sensorValuesC[0][sample] = analogRead(46); // เซนเซอร์ C0 (GPIO 46)
    sensorValuesC[1][sample] = analogRead(47); // เซนเซอร์ C1 (GPIO 47)
    delay(5);
  }

  // คำนวณ Max และ Min สำหรับเซนเซอร์ C0 และ C1
  for (int sensor = 0; sensor < 2; sensor++) {
    sensorMaxC[sensor] = sensorValuesC[sensor][0];
    sensorMinC[sensor] = sensorValuesC[sensor][0];
    
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesC[sensor][sample];
      if (value > sensorMaxC[sensor]) {
        sensorMaxC[sensor] = value;
      }
      if (value < sensorMinC[sensor]) {
        sensorMinC[sensor] = value;
      }
    }
  }

  // บันทึก sensorMaxC และ sensorMinC ลง EEPROM
  byte buffer[4];
  for (int i = 0; i < 2; i++) {
    buffer[i * 2] = highByte(sensorMaxC[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 64, buffer, 4); // sensorMaxC ที่ที่อยู่ 64

  for (int i = 0; i < 2; i++) {
    buffer[i * 2] = highByte(sensorMinC[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 68, buffer, 4); // sensorMinC ที่ที่อยู่ 68

  tone(32, 950, 100);
  delay(200);
  tone(32, 950, 200);
  delay(200);

  // แสดงผล
  Serial.println("Sensor C Results:");
  Serial.print("Sensor C0 (Pin 46) => Max: ");
  Serial.print(sensorMaxC[0]);
  Serial.print(", Min: ");
  Serial.println(sensorMinC[0]);
  Serial.print("Sensor C1 (Pin 47) => Max: ");
  Serial.print(sensorMaxC[1]);
  Serial.print(", Min: ");
  Serial.println(sensorMinC[1]);
}

void read_eepA()
  {      
      // อ่านและตรวจสอบจาก EEPROM
      byte readBuffer[16];
      int readMaxA[numSensors], readMinA[numSensors];
      readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }
      readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }

      Serial.println("Sensor A Values read from EEPROM:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(readMaxA[sensor]);
        Serial.print(", Min: ");
        Serial.println(readMinA[sensor]);
        sensorMax_A[sensor] = readMaxA[sensor];
        sensorMin_A[sensor] = readMinA[sensor];
      }   
  }


void read_sensorA_program()
  { 
      Serial.println("Sensor MAX A Values read from program:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(sensorMax_A[sensor]);
        Serial.print(", Min: ");
        Serial.println(sensorMin_A[sensor]);
      }   
  }
void read_eepB()
  {    
      // อ่านและตรวจสอบจาก EEPROM
      byte readBuffer[16];
      int readMaxB[numSensors], readMinB[numSensors];
      readEEPROM(EEPROM_ADDRESS, 32, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMaxB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }
      readEEPROM(EEPROM_ADDRESS, 48, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMinB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }

      Serial.println("Sensor B Values read from EEPROM:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(readMaxB[sensor]);
        Serial.print(", Min: ");
        Serial.println(readMinB[sensor]);
        sensorMax_B[sensor] = readMaxB[sensor];
        sensorMin_B[sensor] = readMinB[sensor];
      }
  }
void read_sensorB_program()
  { 
      Serial.println("Sensor MAX B Values read from program:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(sensorMax_B[sensor]);
        Serial.print(", Min: ");
        Serial.println(sensorMin_B[sensor]);
      }   
  }

void read_eepC() {
  // อ่านและตรวจสอบจาก EEPROM
  byte readBuffer[4];
  int readMaxC[2], readMinC[2];
  readEEPROM(EEPROM_ADDRESS, 64, readBuffer, 4);
  for (int i = 0; i < 2; i++) {
    readMaxC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 68, readBuffer, 4);
  for (int i = 0; i < 2; i++) {
    readMinC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor C Values read from EEPROM:");
  Serial.print("Sensor C0 (Pin 46) => Max: ");
  Serial.print(readMaxC[0]);
  Serial.print(", Min: ");
  Serial.println(readMinC[0]);
  Serial.print("Sensor C1 (Pin 47) => Max: ");
  Serial.print(readMaxC[1]);
  Serial.print(", Min: ");
  Serial.println(readMinC[1]);
  sensorMax_C[0] = readMaxC[0];
  sensorMin_C[0] = readMinC[0];
  sensorMax_C[1] = readMaxC[1];
  sensorMin_C[1] = readMinC[1];
}

void read_sensorC_program() {
  Serial.println("Sensor C Values read from program:");
  Serial.print("Sensor C0 (Pin 46) => Max: ");
  Serial.print(sensorMax_C[0]);
  Serial.print(", Min: ");
  Serial.println(sensorMin_C[0]);
  Serial.print("Sensor C1 (Pin 47) => Max: ");
  Serial.print(sensorMax_C[1]);
  Serial.print(", Min: ");
  Serial.println(sensorMin_C[1]);
}

void get_EEP_Program()
  {
    read_eepA();
    read_eepB();
    read_eepC();
    read_sensorA_program();
    read_sensorB_program();
    read_sensorC_program();
  }