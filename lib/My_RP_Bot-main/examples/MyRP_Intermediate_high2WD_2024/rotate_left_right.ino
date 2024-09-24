
void rotate_lefts(int pw, int offset)
  {
    if(line_r == false)
      {
        offset -= 100;
      }
    if(line_l == false)
      {
        offset += 100;
      }
    if(lines == true)
      {
        if(lines_fw == true)
          {
            encoderPos = 0;
            do{Motor(-30, -30);}while(encoderPos > -fw_to_rotate);
            Motor(30, 30); delay(30);
            Motor(-1, -1);
            delay(100); 
          }
        else
          {
            encoderPos = 0;
            do{Motor(30, 30);}while(encoderPos < fw_to_rotate);
            Motor(-30, -30); delay(30);
            Motor(1, 1);
            delay(100); 
          }
      }
     else
      {
        Motor(1, 1);
        delay(100); 
      }
    encoderPos = 0;
    do{Motor(-pw, pw);}while(encoderPos > -offset);
    Motor(30, -30); delay(30);
    Motor(1, 1);
    delay(100);
    line_l = true;
    line_r = true;
  }

///------------------------------------------------------------->>>

void rotate_rights(int pw, int offset)
  {
    if(line_r == false)
      {
        offset += 100;
      }
    if(line_l == false)
      {
        offset -= 100;
      }
    if(lines == true)
      {
        if(lines_fw == true)
          {
            encoderPos = 0;
            do{Motor(-30, -30);}while(encoderPos > -fw_to_rotate);
            Motor(30, 30); delay(30);
            Motor(-1, -1);
            delay(100); 
          }
        else
          {
            encoderPos = 0;
            do{Motor(30, 30);}while(encoderPos < fw_to_rotate);
            Motor(-30, -30); delay(30);
            Motor(1, 1);
            delay(100); 
          }
      }
     else
      {
        Motor(1, 1);
        delay(100); 
      }
    encoderPos = 0;
    do{Motor(pw, -pw);}while(encoderPos < offset);
    Motor(-30, 30); delay(30);
    Motor(1, 1);
    delay(100);
    line_l = true;
    line_r = true;
  }
