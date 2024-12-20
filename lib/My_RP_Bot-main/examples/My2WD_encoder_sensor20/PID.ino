void fw (int spl ,int spr, float kp, float kd, int offset)     
  {  
       float max_integral = 1000;  // จำกัดค่าอินทิกรัลไม่ให้เกิน
       while(1)
          {    
              float error = error_20();

              // คำนวณส่วนสัมพัทธ์ (Proportional)
              P = error;
          
              // คำนวณส่วนอินทิกรัล (Integral) พร้อมการป้องกันการสะสมค่ามากเกินไป
              I = I + error;
              if (I > max_integral) 
                  {
                      I = max_integral;  // จำกัดค่าของ I
                  } 
              else if (I < -max_integral) 
                  {
                      I = -max_integral;  // จำกัดค่าของ I
                  }
          
              // คำนวณส่วนอนุพันธ์ (Derivative)
              D = error - previous_error;
              previous_error = error;
          
              // คำนวณผลลัพธ์จาก PID
              PID_output = (kp * P) + (0.000015 * I) + (kd * D);
          
              // ปรับความเร็วของมอเตอร์ตาม PID
              Motor(spl - PID_output, spr + PID_output);
          
              // หน่วงเวลาเพื่อความเสถียร
              delayMicroseconds(50);  // ปรับค่าหน่วงเวลาให้เหมาะสม
              
           }

        Motor(-spl, -spl); delay(offset);
        Motor(-1, -1); delay(50);
        Motor(0, 0); delay(50);        
        
  }   
