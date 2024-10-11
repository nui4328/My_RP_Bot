void read_me()
    {
      sw();                           //--------->>> คำสั่งรอปุ่มกด
      bz(100);                        //--------->>> คำสั่งเปิดเสียง        
      Motor(30, 30);                  //--------->>> คำสั่งควบคุมการทำงานของมอเตอร์
      Motor(-30, -30); 
      
      mcp_f(0);                       //--------->>> คำสั่งแสดงค่าของ sensor แผงหน้าที่อ่านได้ ขณะนั้น mcp_f(0 - 7);
      md_mcp_f(0);                    //--------->>> คำสั่งแสดงค่ากลางของ sensor  md_mcp_f(0 - 7);
      max_mcp_f(0);                   //--------->>> คำสั่งแสดงค่าสูงสุดของ sensor  max_mcp_f(0 - 7);
      min_mcp_f(0);                   //--------->>> คำสั่งแสดงค่าต่ำสุดของ sensor  min_mcp_f(0 - 7);

      
      analogRead(26);                 //--------->>> คำสั่งแสดงค่าของ sensor ใต้ท้อง  analogRead(26 - 27);
 

      //servo(23,90);                   //--------->>> คำสั่งควบคุมการทำงาน servo servo(23,90);  พารามิเตอร์ ตัวที่ 1 ประกอบด้วย 23, 10, 11, 12
                                      //--------->>>  พารามิเตอร์ ตัวที่ 2 ใส่ค่าตั้งแต่  0 -180

      //mydisplay("MY-MAKERS", 20, 30, 2, white);  /// red   yello   green    black   white 
      //mydisplay_background( uint16_t led_color)
           
    }
void read_eep()
  {
    for(int i = 0; i<8; i++)
      {
        Serial.print(md_mcp_f(i));Serial.print("  ");
      }
    Serial.println(" ");
     
    for(int i = 0; i<8; i++)
      {
        Serial.print(max_mcp_f(i));Serial.print("  ");
      }
     Serial.println(" ");
     
    for(int i = 0; i<8; i++)
      {
        Serial.print(min_mcp_f(i));Serial.print("  ");
      }
     Serial.println(" ");

  }
