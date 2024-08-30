

void rangs()
  {
    // อ่านข้อมูลจากเซ็นเซอร์ 1
      I2CMux.openChannel(1);
      range1 = sensor1.readRangeSingleMillimeters();
      I2CMux.closeChannel(1);
    
      // อ่านข้อมูลจากเซ็นเซอร์ 2
      I2CMux.openChannel(2);
      range2 = sensor2.readRangeSingleMillimeters();
      I2CMux.closeChannel(2);
    
      // อ่านข้อมูลจากเซ็นเซอร์ 3
      I2CMux.openChannel(3);
      range3 = sensor3.readRangeSingleMillimeters();
      I2CMux.closeChannel(3);
 
  }
void cal_dis_L()
  {
     int eep_26 = analogRead(26);
     EEPROM.put(100, eep_26); // write data to EEPROM address 0
     EEPROM.commit(); // save changes to EEPROM

     int eep_27 = analogRead(27);
     EEPROM.put(150, eep_27); // write data to EEPROM address 0
     EEPROM.commit(); // save changes to EEPROM
                                          
     int read_26;
     EEPROM.get(100, read_26);  
     Serial.print("26 -> EEPROM: ");
     Serial.println(read_26);

     int read_27;
     EEPROM.get(150, read_27);  
     Serial.print("27 -> EEPROM: ");
     Serial.println(read_27);
  }
void cal_dis_R()
  {
     int eep_26 = analogRead(26);
     EEPROM.put(200, eep_26); // write data to EEPROM address 0
     EEPROM.commit(); // save changes to EEPROM

     int eep_27 = analogRead(27);
     EEPROM.put(250, eep_27); // write data to EEPROM address 0
     EEPROM.commit(); // save changes to EEPROM
                                          
     int read_26;
     EEPROM.get(200, read_26);  
     Serial.print("26 -> EEPROM: ");
     Serial.println(read_26);

     int read_27;
     EEPROM.get(250, read_27);  
     Serial.print("27 -> EEPROM: ");
     Serial.println(read_27);
  }

void read_epp_sensor()
  {
     int read_26_min;
     EEPROM.get(200, read_26_min);  
     min_26 = read_26_min;
     Serial.print("min26 -> EEPROM: ");
     Serial.println(min_26);
     
     int read_26_max;
     EEPROM.get(100, read_26_max);  
     max_26 = read_26_max; 
     Serial.print("max26 -> EEPROM: ");
     Serial.println(max_26);

     int read_27_min;
     EEPROM.get(150, read_27_min);  
     min_27 = read_27_min;
     Serial.print("min27 -> EEPROM: ");
     Serial.println(min_27);    
    
     int read_27_max;
     EEPROM.get(250, read_27_max);  
     max_27 = read_27_max; 
     Serial.print("max27 -> EEPROM: ");
     Serial.println(max_27);
  }
