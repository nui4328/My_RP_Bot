void read_me()
    {
      fw(40, 40, 5000, "line");     //เดินหน้า ความเร็ว 40 เป็นระยะ 5000  แล้วเดินช้า ๆ เข้าหาเส้น
      fw(40, 40, 5000, "non_line"); //เดินหน้า ความเร็ว 40 เป็นระยะ 5000  หยุดไม่เดินเข้าหาเส้น 
      fw_non_sensor(40, 40, 5000, "line");  // ใช้สำหรับขึ้นสะพานและตะเกียบ
      fw_non_sensor(40, 40, 5000, "non_line");
      
      bw(40, 40, 5000, "line"); 
      bw(40, 40, 5000, "non_line");  
      
      rotate_right(70, 1000);       // หมุนขวา ด้วยความเร็ว 90% ด้วยการหมุนของล้อที่ 1000 รอบ
      rotate_left(70, 1000);
      
      set_f(2);    // เมื่อต้องการเซตหุ่นยนต์ให้ตรง เลข 2 คือ จำนวนครั้งที่วิ่งเข้าหาเส้น
      set_b(2);

      Place_box();

      //---------------------------------------------->>

 
      
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
