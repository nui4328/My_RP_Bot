void ReadMe()
  {
    
    fw( 55, 55, 250, "line");        //เดินหน้าความเร็ว 55, 55 เวลาการวิ่ง 250(กำหนดให้อยู่กลางแผ่น)แล้วเดินหน้าต่อเข้าหาเส้นแล้วหยุดอยู่ที่เส้น    
    fw( 55, 55, 250, "non_line");    //เดินหน้าความเร็ว 55, 55 เวลาการวิ่ง 250(กำหนดให้อยู่กลางแผ่น) (หุ่นยนต์ไม่ถึ่งเส้น)
    bw( 55, 56, 250, "line");        //ถอยหลังความเร็ว 55, 55 เวลาการวิ่ง 250(กำหนดให้อยู่กลางแผ่น)แล้วเดินหน้าต่อเข้าหาเส้นแล้วหยุดอยู่ที่เส้น    
    bw( 55, 56, 250, "non_line");    //เดินหน้าความเร็ว 55, 55 เวลาการวิ่ง 250(กำหนดให้อยู่กลางแผ่น) (หุ่นยนต์ไม่ถึ่งเส้น)
    tr(90, 200); //เลี้ยวซ้ายความเร็วในการหมุน 90 เวลาในการหมุน 200 
    tl(90, 200); //เลี้ยวขวาความเร็วในการหมุน 90 เวลาในการหมุน 200
    set_f(2); //การ set ให้หุ่นยนต์ตรงโดยการเดินหน้าเข้าหาเส้น เลข 2 ในวงเล็บคือ จำนวนการเข้าไปเซต
    set_b(2); //การทำให้หุ่นยนต์ตรงโดยการเช็กเส้นข้างหลัง
    mission(); //ปล่อยกล่อง

    fw_no_sensor( 55, 56, 250, "line");  //เดินหาหน้าเข้าหาเส้นความเร็ว 50, 50 เวลาการวิ่ง 250   (ไม่เช็คเส้นข้างหน้า  เอาไว้วิ่งขึ้นสะพานกับตะเกรียบ) 
    fw_no_sensor( 55, 56, 250, "non_line");
   
  }