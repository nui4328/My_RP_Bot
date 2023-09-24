void read_me()
    {
      sw();                           //--------->>> คำสั่งรอปุ่มกด
      bz(100);                        //--------->>> คำสั่งเปิดเสียง        
                                      //--------->>> คำสั่งควบคุมการทำงานของมอเตอร์
      //Motor(int sp_fl, int sp_fr, int sp_bl, int sp_br)
      Motor(50, 50, 50, 50);
      
      mcp_f(0);                       //--------->>> คำสั่งแสดงค่าของ sensor แผงหน้าที่อ่านได้ ขณะนั้น mcp_f(0 - 7);
      md_mcp_f(0);                    //--------->>> คำสั่งแสดงค่ากลางของ sensor  md_mcp_f(0 - 7);
      max_mcp_f(0);                   //--------->>> คำสั่งแสดงค่าสูงสุดของ sensor  max_mcp_f(0 - 7);
      min_mcp_f(0);                   //--------->>> คำสั่งแสดงค่าต่ำสุดของ sensor  min_mcp_f(0 - 7);

      
      analogRead(26);                 //--------->>> คำสั่งแสดงค่าของ sensor ใต้ท้อง  analogRead(26 - 27);
 

      //servo(23,90);                   //--------->>> คำสั่งควบคุมการทำงาน servo servo(23,90);  พารามิเตอร์ ตัวที่ 1 ประกอบด้วย 23, 22, 28
                                      //--------->>>  พารามิเตอร์ ตัวที่ 2 ใส่ค่าตั้งแต่  0 -180

      //mydisplay("MY-MAKERS", 20, 30, 2, white);  /// red   yello   green    black   white 
      //mydisplay_background( uint16_t led_color)
           // #include <my_TCS34725.h>     
       // my_tcs('r'),   อ่านค่า r
       // my_tcs('g'),   อ่านค่า g
       // my_tcs('b')    อ่านค่า b
      ////-------------------------------****
      
     // #include <my_mpu6050.h>
    //  setup_mpu();               //  ก๊อปไปวางใน void setup
    //  error_Yaw()                // ส่งค่า error ออกมา
    //error_Yaw_float()
    //  calibration_Yak();         // สำหรับคาริเบลท ให้ค่า error_Yaw() เป็น 0

    //  adc.begin(5, 4, 12, 0 );     
      //  adc.readADC(sensor);


 

    }
