void place_left_out(int speed, int degree, int offset) 
  {
      
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(10);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + degree;
    
        // กำหนดค่าของ PID
        
        float lr_kp  = 2.05;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        float lr_ki  = 0.00001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        float lr_kd = 0.015; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 500;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1 && abs(output) < 5) break;
    
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            lastTime = now;
    
            if (dt > 0) {
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                previous_error = error;
    
                output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
            }
    
            // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
            output = constrain(output, -speed, speed);
    
            // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
            if(degree > 0)
              {
                Motor(output/4, -output);  
                delay(5);
              }
              else
              {
                
                 Motor(output, -(output/4));  
                delay(5);
              }
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            
            Motor(output/4, -output);
            delay(offset);
          }
        else
          {
            Motor(output, -(output/4)); 
            delay(offset);
          }
        Motor(-1, -1);
        delay(10);
  }

void place_left_in(int speed, int degree, int offset) 
  {
      
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(10);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + (-degree);
    
        // กำหนดค่าของ PID
        
        float lr_kp  = 2.05;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        float lr_ki  = 0.00001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        float lr_kd = 0.015; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 500;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1 && abs(output) < 5) break;
    
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            lastTime = now;
    
            if (dt > 0) {
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                previous_error = error;
    
                output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
            }
    
            // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
            output = constrain(output, -speed, speed);
    
            // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
            if(degree > 0)
              {
                Motor(output/4, -output);  
                delay(5);
              }
              else
              {
                
                 Motor(output, -(output/4));  
                delay(5);
              }
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            
            Motor(output/4, -output);
            delay(offset);
          }
        else
          {
            Motor(output, -(output/4)); 
            delay(offset);
          }
        Motor(-1, -1);
        delay(10);
  }

void place_right_in(int speed, int degree, int offset) 
  {
      
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(10);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + degree;
    
        // กำหนดค่าของ PID
        
        float lr_kp  = 2.05;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        float lr_ki  = 0.00001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        float lr_kd = 0.015; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 500;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1 && abs(output) < 5) break;
    
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            lastTime = now;
    
            if (dt > 0) {
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                previous_error = error;
    
                output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
            }
    
            // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
            output = constrain(output, -speed, speed);
    
            // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
            if(degree > 0)
              {
                Motor(output, -(output/4));  
                delay(5);
              }
              else
              {
                Motor(output/4, -output);  
                delay(5);
              }
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            Motor(output, -(output/4)); 
            delay(offset);
          }
        else
          {
            Motor(output/4, -output);
            delay(offset);
          }
        Motor(-1, -1);
        delay(10);
  }

void place_right_out(int speed, int degree, int offset) 
  {
      
        // คำนวณค่ามุมเริ่มต้น
        float initialDegree = 0;
        for (int i = 0; i < 5; i++) {
            initialDegree += my.gyro('z');  // ใช้ค่าที่อ่านได้จากเซ็นเซอร์
            delay(10);
        }
        initialDegree /= 5.0;
    
        // คำนวณมุมเป้าหมาย
        float targetDegree = initialDegree + (-degree);
    
        // กำหนดค่าของ PID
        
        float lr_kp  = 2.05;  // ปรับค่า Kp เพื่อให้การหมุนเร็วขึ้น
        float lr_ki  = 0.00001;  // ค่า Ki ปรับตามความแม่นยำในการหยุด
        float lr_kd = 0.015; // ปรับค่า Kd เพื่อให้การหยุดมีความแม่นยำขึ้น
        
        float error = 0, previous_error = 0;
        float integral = 0, output = 0;
        float currentDegree = 0;
    
        unsigned long lastTime = millis();
        unsigned long timeout = 500;  // กำหนดเวลา timeout
        unsigned long startTime = millis();
    
        while (true) {
            currentDegree = my.gyro('z');  // อ่านค่ามุมปัจจุบัน
            error = targetDegree - currentDegree;  // คำนวณความผิดพลาด
    
            // ตรวจสอบเงื่อนไขการหยุดเมื่อใกล้ถึงมุมที่ต้องการ
            if (abs(error) < 1 && abs(output) < 5) break;
    
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            lastTime = now;
    
            if (dt > 0) {
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                previous_error = error;
    
                output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
            }
    
            // จำกัดค่าของ output ให้อยู่ในช่วงความเร็วที่ต้องการ
            output = constrain(output, -speed, speed);
    
            // ควบคุมมอเตอร์ให้หมุนตาม PID ที่คำนวณ
            if(degree > 0)
              {
                Motor(output, -(output/4));  
                delay(5);
              }
              else
              {
                Motor(output/4, -output);  
                delay(5);
              }
    
            // ตรวจจับ timeout หากใช้เวลานานเกินไป
            if (millis() - startTime > timeout) {
                break;  // ออกจาก loop หากเวลาผ่านไปนานเกินไป
            }
        }
    
        // หยุดมอเตอร์หลังจากหมุนเสร็จ
        if(degree>0)
          {
            Motor(output, -(output/4)); 
            delay(offset);
          }
        else
          {
            Motor(output/4, -output);
            delay(offset);
          }
        Motor(-1, -1);
        delay(10);
  }