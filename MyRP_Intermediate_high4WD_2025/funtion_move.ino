

void move_tl(int sp, int targetPulses)
  {

    encoder.resetEncoders();
    while(1)
      {
        // อ่านค่าจาก Encoder
        float leftPulses = -encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว (เฉลี่ยจากล้อซ้ายและขวา)
        float currentPulses = (leftPulses + rightPulses) / 2;

        // ตรวจสอบว่าถึงเป้าหมายหรือยัง
        Serial.print(currentPulses);Serial.print(" ");
        Serial.print(targetPulses);Serial.println(" ");
        Motor(-sp, sp);
        if (currentPulses >= targetPulses) 
          {
            Motor(sp, -sp); delay(40);
          
            Motor(-1, -1); // หยุดมอเตอร์
            delay(50);
            break;       // ออกจากลูป
          }
      }
  }

void move_tr(int sp,int targetPulses)
  {
    
    encoder.resetEncoders();
    while(1)
      {
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = -encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว (เฉลี่ยจากล้อซ้ายและขวา)
        float currentPulses = (leftPulses + rightPulses) / 2;

        // ตรวจสอบว่าถึงเป้าหมายหรือยัง
        Serial.print(currentPulses);Serial.print(" ");
        Serial.print(targetPulses);Serial.println(" ");
        Motor(sp, -sp);
        if (currentPulses >= targetPulses) 
          {
            Motor(-sp, sp); delay(40);
          
            Motor(-1, -1); // หยุดมอเตอร์
            delay(50);
            break;       // ออกจากลูป
          }
      }
  }
void move_fw(int sp,int targetDistanceCm)
  {
    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซ็ต Encoder
    encoder.resetEncoders();  
    while (true) 
      {
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว (เฉลี่ยจากล้อซ้ายและขวา)
        float currentPulses = (leftPulses + rightPulses) / 2;
        if (currentPulses >= targetPulses) 
          {
            Motor(-10, -10); // หยุดมอเตอร์
            delay(30);
            Motor(1, 1); // หยุดมอเตอร์
            delay(30);
            break;       // ออกจากลูป
          }
        
        Motor(sp, sp);
        delay(5); // เพิ่มความเสถียรของลูป
      }
  }
void move_fw_free(int sp,int targetDistanceCm)
  {
    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซ็ต Encoder
    encoder.resetEncoders();  
    while (true) 
      {
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว (เฉลี่ยจากล้อซ้ายและขวา)
        float currentPulses = (leftPulses + rightPulses) / 2;
        if (currentPulses >= targetPulses) 
          {
            break;       // ออกจากลูป
          }
        
        Motor(sp, sp);
        delay(2); // เพิ่มความเสถียรของลูป
      }
  }

void move_bw(int sp,int targetDistanceCm)
  {
    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซ็ต Encoder
    encoder.resetEncoders();  
    while (true) 
      {
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว (เฉลี่ยจากล้อซ้ายและขวา)
        float currentPulses = (leftPulses + rightPulses) / 2;
        if (currentPulses <= -targetPulses) 
          {
            Motor(-10, -10); // หยุดมอเตอร์
            delay(30);
            Motor(1, 1); // หยุดมอเตอร์
            delay(30);
            break;       // ออกจากลูป
          }
        
        Motor(-sp, -sp);
        delay(5); // เพิ่มความเสถียรของลูป
      }
  }
/*
void move_tl(int sp, int deg)
  {
    angleZ = 0;
    angleZ = 0;
    delay(40);

    while(1)
      {
        Motor(-sp, sp);
        if(spi_gyro('z') > deg-40)
          {
            sp = 20;
          }
        if(spi_gyro('z') > deg)
          {
            Motor(sp, -sp);delay(40);
            Motor(-1, -1);delay(100);
            break;
          }        
      }
  }

void move_tr(int sp,int deg)
  {
    angleZ = 0;
    angleZ = 0;
    delay(50);
    
    while(1)
      {
        Motor(sp, -sp);
        if(spi_gyro('z') < -(deg-40))
          {
            sp = 20;
          }
        if(spi_gyro('z') < -deg)
          {
            Motor(-sp, sp);delay(40);
            Motor(-1, -1);delay(100);
            break;
          }

      }
  }
*/
