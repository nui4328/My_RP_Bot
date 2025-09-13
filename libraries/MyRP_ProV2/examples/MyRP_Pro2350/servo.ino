
int servo_down = 50;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 170;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 10;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา


void arm_ready()  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  {
    servo(36, servo_down);
    servo(34, servoL_open - 30);
    servo(35, servoR_open + 30);
  }

void arm_down_open()
  {
    servo(36, servo_down);
    delay(300);
    servo(34, servoL_open);
    servo(35, servoR_open);
  }
void arm_up_open()
  {
    servo(36, servo_down + 90);
    servo(34, servoL_open);
    servo(35, servoR_open);
    servo(36, servo_down + 90);
  }
void arm_down_close()
  {
    servo(36, servo_down);
    delay(50);
    servo(34, servoL_open-110);
    servo(35, servoR_open+110);
  }

void arm_up_close()
  {
    servo(36, servo_down+90);
    servo(34, servoL_open-110);
    servo(35, servoR_open+110);
  }
void arm_big_box()
  {
    servo(36, servo_down);
    servo(34, servoL_open - 70);
    servo(35, servoR_open + 70);
  }

void fw_ch_line(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
              {
                Motor(-5 ,20);
              }
            else if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) < md_sensorA(7)-50)
              {
                Motor(20 ,-5);
              }
            else if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
              {          
                Motor(15 ,15);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
        if(num > 1)
          {
            Motor(-15 ,-15);
            delay(50);
            Motor(-1 ,-1);
          }
      }
    
  }

void bw_ch_line(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(read_sensorB(0) < md_sensorB(0)-50 && read_sensorB(7) > md_sensorB(7)-50)
              {
                Motor(-20 ,5);
              }
            else if(read_sensorB(0) > md_sensorB(0)-50 && read_sensorB(7) < md_sensorB(7)-50)
              {
                Motor(5 ,-20);
              }
            else if(read_sensorB(0) > md_sensorB(0)-50 && read_sensorB(7) > md_sensorB(7)-50)
              {          
                Motor(-15 ,-20);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
        if(num > 1)
          {
            Motor(10 ,10);
            delay(50);
            Motor(-1 ,-1);
          }
      }
    
  }
