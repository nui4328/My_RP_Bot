



uint16_t read_sensorA(int sensor) 
  {       
     adc.begin(14, 15, 16, 13 );
     adc.begin(14, 15, 16, 17 );  //adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
     return adc.readADC(sensor);  
  }

uint16_t read_sensorB(int sensor) 
  {       
     adc.begin(14, 15, 16, 17 ); 
     adc.begin(14, 15, 16, 13 );  //adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
     return adc.readADC(sensor);  
  }

int position_A()  
   {        
      int min_sensor_values_A[] = { sensorMin_A[1], sensorMin_A[2], sensorMin_A[3], sensorMin_A[4], sensorMin_A[5], sensorMin_A[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int max_sensor_values_A[] = { sensorMax_A[1], sensorMax_A[2], sensorMax_A[3], sensorMax_A[4], sensorMax_A[5], sensorMax_A[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorA(sensor_pin_A[i]), min_sensor_values_A[i], max_sensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 0;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 5000;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }
int position_A_none()  
   {        
      int min_sensor_values_A[] = { sensorMin_A[1], sensorMin_A[2], sensorMin_A[3], sensorMin_A[4], sensorMin_A[5], sensorMin_A[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int max_sensor_values_A[] = { sensorMax_A[1], sensorMax_A[2], sensorMax_A[3], sensorMax_A[4], sensorMax_A[5], sensorMax_A[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorA(sensor_pin_A[i]), min_sensor_values_A[i], max_sensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 2500;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 2500;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }

float error_A()
    {
      if(pid_error == true)
        {
          present_position = position_A_none()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }
      else
        {
          present_position = position_A()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }                            
    }

int position_B()  
   {        
      int min_sensor_values_A[] = { sensorMin_B[1], sensorMin_B[2], sensorMin_B[3], sensorMin_B[4], sensorMin_B[5], sensorMin_B[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int max_sensor_values_A[] = { sensorMax_B[1], sensorMax_B[2], sensorMax_B[3], sensorMax_B[4], sensorMax_B[5], sensorMax_B[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorA(sensor_pin_A[i]), min_sensor_values_A[i], max_sensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 0;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 5000;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }
int position_B_none()  
   {        
      int min_sensor_values_A[] = { sensorMin_B[1], sensorMin_B[2], sensorMin_B[3], sensorMin_B[4], sensorMin_B[5], sensorMin_B[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int max_sensor_values_A[] = { sensorMax_B[1], sensorMax_B[2], sensorMax_B[3], sensorMax_B[4], sensorMax_B[5], sensorMax_B[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorB(sensor_pin_A[i]), min_sensor_values_A[i], max_sensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 2500;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 2500;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }

float error_B()
    {
      if(pid_error == true)
        {
          present_position = position_B_none()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }
      else
        {
          present_position = position_B()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }                            
    }

void fw(int sl, int sr, float kp)
  {
    while(1) 
    {
      delayMicroseconds(50);
      errors = error_A();
      P = errors;
      D = errors - previous_error;                  
      previous_error = errors;
      PID_output = (kp * P) + (0.0001 * I) + (0.025 * D); 
      Motor(sl - PID_output, sr + PID_output);
      
    }
      
  }
void fws(int sl, int sr, float kp) 
{
    int current_speed = 0;    // เริ่มจากความเร็ว 0
    int target_speed = (sl < sr) ? sl : sr;  // เลือกความเร็วต่ำสุดเป็นเป้าหมายเพื่อสมดุล
    const int ramp_step = 2;  // ความเร็วเพิ่มครั้งละ 2 (ปรับได้ตามความนุ่มนวลที่ต้องการ)
    const int ramp_delay = 10; // หน่วงระหว่างเพิ่มความเร็ว (ms)

    // Soft start เพิ่มความเร็วจนถึงเป้าหมาย
    while (current_speed < target_speed) 
    {
        errors = error_A();
        P = errors;
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(current_speed - PID_output, current_speed + PID_output);

        current_speed += ramp_step;
        if (current_speed > target_speed) current_speed = target_speed;

        delay(ramp_delay);
    }

    // เดินหน้าปกติเมื่อถึงความเร็วที่ต้องการ
    while (1) 
    {
        delayMicroseconds(50);
        errors = error_A();
        P = errors;
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(sl - PID_output, sr + PID_output);
    }
}



void fws(int sl, int sr, float kp, float distance) 
{
    int current_speed = 0;    // เริ่มจากความเร็ว 0 (หน่วย: cm/s)
    int target_speed = (sl < sr) ? sl : sr;  // เลือกความต่ำสุดเพื่อสมดุล
    const int ramp_step = 2;  // ความเร็วเพิ่มครั้งละ 2
    const int ramp_delay = 10; // หน่วงระหว่างเพิ่มความเร็ว (ms)
    float errors, P, D, previous_error = 0, PID_output;
    float I = 0;  // ตัวแปร Integral
    float traveled_distance = 0;  // ระยะทาง (หน่วย: เซนติเมตร)
    unsigned long last_time = millis();  // เก็บเวลาเริ่มต้น

    // Soft start เพิ่มความเร็วจนถึงเป้าหมาย
    while (current_speed < target_speed) 
    {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);  // อัพเดท Integral
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(current_speed - PID_output, current_speed + PID_output);

        // คำนวณระยะทางถ้ากำหนด distance > 0
        if (distance > 0) {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;  // หน่วยวินาที
            traveled_distance += current_speed * delta_time;  // ความเร็ว cm/s * เวลา วินาที = ระยะทาง cm
            last_time = current_time;

            // ตรวจสอบระยะทาง
            if (traveled_distance >= distance) 
            {
                Motor(0, 0);  // หยุดมอเตอร์
                return;  // ออกจากฟังก์ชัน
            }
        }

        current_speed += ramp_step;
        if (current_speed > target_speed) current_speed = target_speed;

        delay(ramp_delay);
    }

    // เดินหน้าปกติ
    while (1) 
    {
        errors = error_A();
        P = errors;
        I += errors * 0.00005;  // อัพเดท Integral (สำหรับ 50us)
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(sl - PID_output, sr + PID_output);

        // คำนวณระยะทางถ้ากำหนด distance > 0
        if (distance > 0) {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;  // หน่วยวินาที
            traveled_distance += target_speed * delta_time;  // ความเร็ว cm/s * เวลา วินาที = ระยะทาง cm
            last_time = current_time;

            // ตรวจสอบระยะทาง
            if (traveled_distance >= distance) 
            {
                Motor(0, 0);  // หยุดมอเตอร์
                return;  // ออกจากฟังก์ชัน
            }
        }

        delayMicroseconds(50);
    }

    ///////////////////////////////-------------------------------------------------------->>>>>>>>

}