void read_me()
    {
      sw();                           //--------->>> คำสั่งรอปุ่มกด
      bz(100);                        //--------->>> คำสั่งเปิดเสียง        
      Motor(30, 30);                  //--------->>> คำสั่งควบคุมการทำงานของมอเตอร์
      Motor(-30, -30); 
      
      mcp_20(0);                       //--------->>> คำสั่งแสดงค่าของ sensor แผงหน้าที่อ่านได้ ขณะนั้น mcp_f(0 - 7);
      md_mcp_20(0);                    //--------->>> คำสั่งแสดงค่ากลางของ sensor  md_mcp_f(0 - 7);
      max_mcp_20(0);                   //--------->>> คำสั่งแสดงค่าสูงสุดของ sensor  max_mcp_f(0 - 7);
      min_mcp_20(0);                   //--------->>> คำสั่งแสดงค่าต่ำสุดของ sensor  min_mcp_f(0 - 7);
      
      analogRead(26);                 //--------->>> คำสั่งแสดงค่าของ sensor ใต้ท้อง  analogRead(26 - 27); 

      
    }

void _swt()
  {
    bz(100);
    bz(100);
    while(digitalRead(9) == 1)
      {
        Serial.println(digitalRead(9));
        delay(10);
        if(analogRead(29) > 4000   )
          {
            bz(100); delay(500);
            add_sensor_B();
            
          }
      }
    
    bz(300);
     
         Serial.println("  ");
         Serial.println("  ");
         for(int i=0; i<DATA_sensor_SIZE; i++)
               {
                  EEPROM.get((2*i)*2, readData_sensor_F[i]); // read data from EEPROM address 0    
                  delay(1);        
               }
         for (int i = 0; i < DATA_sensor_SIZE; i ++) 
            {
               Serial.print(readData_sensor_F[i]);Serial.print("  ");
               delay(1);
            }
            Serial.println("  ");

         for(int i=0; i<DATA_sensor_SIZE_B; i++)
               {
                  EEPROM.get((50+i)*2, readData_sensor_B[i]); // read data from EEPROM address 0    
                  delay(1);        
               }
         for (int i = 0; i < DATA_sensor_SIZE_B; i ++) 
            {
               Serial.print(readData_sensor_B[i]);Serial.print("  ");
            }
            Serial.println("  ");
         
         //----------------------------------------->>
         for(int i=0; i<DATA_sensor_SIZE_M; i++)
               {
                  EEPROM.get((70+i)*2, readData_sensor_M[i]); // read data from EEPROM address 0    
                  delay(1);        
               }
         for (int i = 0; i < DATA_sensor_SIZE_M; i ++) 
            {
               Serial.print(readData_sensor_M[i]);Serial.print("  ");
            }
            Serial.println("  ");
  }
