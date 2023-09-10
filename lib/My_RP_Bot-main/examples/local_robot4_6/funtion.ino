void fw_ch_07(int sl, int sr, int offset)
{
  while (1)
  {
    float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
    float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
    errors = error_L - error_R;
    I = 0;
    previous_I = 0;
    previous_error = 0;
    P = errors;
    I = I + previous_I;
    D = errors - previous_error ;
    previous_I = I;
    previous_error = errors  ;
    PID_output = (0.2 * P) + (0.00015 * I) + (0.2 * D);
    Motor(sl + PID_output, sr - PID_output);
    if (mcp_f(0) < md_mcp_f(0) || mcp_f(7) < md_mcp_f(7))
    {
      break;
    }
  }
  if (offset > 0)
  {
    Motor(-sl, -sr);  delay(offset);
    Motor(0, 0);  delay(offset);
  }
  else {}


}
void fw_2sensor(int sl, int sr, float kp, int tm, String line, String sensor, int ofset)
{
  char sensors[4];  // Declare a char array to store the converted string
  line.toCharArray(sensors, sizeof(sensors));
  int sensor_l = atoi(&sensors[0]);
  int sensor_r = atoi(&sensors[2]);
  if (tm > 0)
  {
    unsigned long lasts_time = millis();
    while (millis() - lasts_time < tm)
    {
      float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
      float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
      errors = error_L - error_R;
      I = 0;
      previous_I = 0;
      previous_error = 0;
      P = errors;
      I = I + previous_I;
      D = errors - previous_error ;
      previous_I = I;
      previous_error = errors  ;
      PID_output = (kp * P) + (0.00015 * I) + (kp * D);

      Motor(sl + PID_output, sr - PID_output);
      //Serial.println(errors);

    }

  }
  else
  {
    while (1)
    {
      float error_L = map(mcp_f(sensor_l), min_mcp_f(sensor_l), max_mcp_f(sensor_l), 0, 30 );
      float error_R = map(mcp_f(sensor_r), min_mcp_f(sensor_r), max_mcp_f(sensor_r), 0, 30 );
      errors = error_L - error_R;
      I = 0;
      previous_I = 0;
      previous_error = 0;
      P = errors;
      I = I + previous_I;
      D = errors - previous_error ;
      previous_I = I;
      previous_error = errors  ;
      PID_output = (kp * P) + (0.00015 * I) + (kp * D);

      Motor(sl + PID_output, sr - PID_output);
      //Serial.println(errors);

      if (sensor == "a0")
      {
        if (mcp_f(0) < md_mcp_f(0))
        {
          break;
        }
      }
      else if (sensor == "a7")
      {
        if (mcp_f(7) < md_mcp_f(7))
        {
          break;
        }
      }
      else if (sensor == "b0")
      {
        if (mcp_b(0) < md_mcp_b(0))
        {
          break;
        }
      }
      else if (sensor == "b7")
      {
        if (mcp_b(7) < md_mcp_b(7))
        {
          break;
        }
      }
      else if (sensor == "26")
      {
        if (analogRead(26) < md_adc(26))
        {
          break;
        }
      }
      else if (sensor == "27")
      {
        if (analogRead(27) < md_adc(27))
        {
          break;
        }
      }

    }
  }

  ///////////////////////////////////////////////////////////////

  if (ofset > 0)
  {
    Motor(-sl, -sr);
    delay(ofset);
    Motor(0, 0);
    delay(50);
  }
  else  {}
  //////////////////////////////////////////////////////////////
}

void fw_a3_a4(int ml, int mr, float kp, int ofset)
{
  while (1)
  {
    int error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
    int error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
    errors = error_L - error_R;
    I = 0;
    previous_I = 0;
    previous_error = 0;
    P = errors;
    I = I + previous_I;
    D = errors - previous_error ;
    previous_I = I;
    previous_error = errors  ;
    PID_output = (kp * P) + (0.00015 * I) + (kp * D);

    Motor(ml + PID_output, mr - PID_output);

    if (mcp_f(2) < md_mcp_f(2) && mcp_f(5) < md_mcp_f(5)
        || mcp_f(3) > md_mcp_f(3) && mcp_f(4) > md_mcp_f(4))
    {
      break;
    }
  }
  if (ofset > 0)
  {
    Motor(-ml, -mr); delay(ofset);
    Motor(0, 0); delay(50);
  }
  else
  {
  }

}

void bw_a3_a4(int ml, int mr, float kp, int ofset)
{
  while (1)
  {
    int error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 20 );
    int error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 20 );
    errors = error_L - error_R;
    I = 0;
    previous_I = 0;
    previous_error = 0;
    P = errors;
    I = I + previous_I;
    D = errors - previous_error ;
    previous_I = I;
    previous_error = errors  ;
    PID_output = (kp * P) + (0.00015 * I) + (kp * D);

    Motor(-(ml - PID_output), -(mr + PID_output));
    Serial.println(errors);
    if (mcp_b(2) < md_mcp_b(2) && mcp_b(5) < md_mcp_b(5)
        || mcp_b(3) > md_mcp_b(3) && mcp_b(4) > md_mcp_b(4))
    {
      break;
    }
  }
  if (ofset > 0)
  {
    Motor(ml, mr); delay(ofset);
    Motor(0, 0); delay(50);
  }
  else
  {
  }
}

void fw_stop_center()
{
  while (1)
  {
    int error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
    int error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
    errors = error_L - error_R;
    I = 0;
    previous_I = 0;
    previous_error = 0;
    P = errors;
    I = I + previous_I;
    D = errors - previous_error ;
    previous_I = I;
    previous_error = errors  ;
    PID_output = (0.2 * P) + (0.00015 * I) + (0.5 * D);

    Motor(30 + PID_output, 30 - PID_output);
    Serial.println(errors);
    if (analogRead(26) < md_adc(26) && analogRead(27) < md_adc(27) )
    {
      break;
    }
  }
  Motor(-10, -10); delay(10);
  Motor(0, 0); delay(10);
  if (mcp_f(3) > md_mcp_f(3) && mcp_f(4) < md_mcp_f(4))
  {
    do {
      Motor(30, -30);
    } while (mcp_f(3) > md_mcp_f(3) - 150);
    Motor(-20, 20); delay(10);
    Motor(0, 0); delay(200);
  }
  else if (mcp_f(4) > md_mcp_f(4) && mcp_f(3) < md_mcp_f(3))
  {
    do {
      Motor(-30, 30);
    } while (mcp_f(4) > md_mcp_f(4) - 150);
    Motor(20, -20); delay(10);
    Motor(0, 0); delay(200);
  }
}


void bw_stop_ba7()
{
  while (1)
  {
    int error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 20 );
    int error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 20 );
    errors = error_L - error_R;
    I = 0;
    previous_I = 0;
    previous_error = 0;
    P = errors;
    I = I + previous_I;
    D = errors - previous_error ;
    previous_I = I;
    previous_error = errors  ;
    PID_output = (0.2 * P) + (0.00015 * I) + (0.5 * D);

    Motor(-(25 - PID_output), -(25 + PID_output));
    Serial.println(errors);
    if (mcp_f(7) < md_mcp_f(7))
    {
      break;
    }
  }
  Motor(40, 40); delay(50);
  Motor(0, 0); delay(50);
}
void bw_stop_ba0()
{
  while (1)
  {
    int error_L = map(mcp_b(3), min_mcp_b(3), max_mcp_b(3), 0, 20 );
    int error_R = map(mcp_b(4), min_mcp_b(4), max_mcp_b(4), 0, 20 );
    errors = error_L - error_R;
    I = 0;
    previous_I = 0;
    previous_error = 0;
    P = errors;
    I = I + previous_I;
    D = errors - previous_error ;
    previous_I = I;
    previous_error = errors  ;
    PID_output = (0.2 * P) + (0.00015 * I) + (0.5 * D);

    Motor(-(25 - PID_output), -(25 + PID_output));
    //Serial.println(errors);
    if (mcp_f(0) < md_mcp_f(0))
    {
      break;
    }
  }
  Motor(40, 40); delay(50);
  Motor(0, 0); delay(50);
}
void tx_left(int sp, int ofset)
{
  Motor(0, 0); delay(50);
  do {
    Motor(-sp, sp);
  } while (mcp_f(0) > md_mcp_f(0) - 100); delay(10);
  do {
    Motor(-sp, sp);
  } while (mcp_f(0) < md_mcp_f(0) - 100); delay(10);
  do {
    Motor(-sp, sp);
  } while (mcp_f(0) > md_mcp_f(0) - 100); delay(10);
  do {
    Motor(-sp, sp);
  } while (mcp_f(0) > md_mcp_f(0) - 100); delay(10);
  do {
    Motor(-sp, sp);
  } while (mcp_f(3) > md_mcp_f(3) - 100); delay(10);

  Motor(sp, -sp); delay(ofset);
  Motor(0, 0); delay(50);
}
void tx_right(int sp, int ofset)
{
  Motor(0, 0); delay(50);
  do {
    Motor(sp, -sp);
  } while (mcp_f(7) > md_mcp_f(7) - 100); delay(10);
  do {
    Motor(sp, -sp);
  } while (mcp_f(7) < md_mcp_f(7) - 100); delay(10);
  do {
    Motor(sp, -sp);
  } while (mcp_f(7) > md_mcp_f(7) - 100); delay(10);
  do {
    Motor(sp, -sp);
  } while (mcp_f(7) > md_mcp_f(7) - 100); delay(10);
  do {
    Motor(sp, -sp);
  } while (mcp_f(4) > md_mcp_f(4) - 100); delay(10);

  Motor(-sp, sp); delay(ofset);
  Motor(0, 0); delay(50);
}

void fw_to_box(int ml, int mr, float kp, int tim , int ofset)
{
  unsigned long lasts_timeb = millis();
  while (millis() - lasts_timeb < 500)
    //while(1)
  {
    int error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 20 );
    int error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 20 );
    errors = error_L - error_R;
    I = 0;
    previous_I = 0;
    previous_error = 0;
    P = errors;
    I = I + previous_I;
    D = errors - previous_error ;
    previous_I = I;
    previous_error = errors  ;
    PID_output = (kp * P) + (0.00015 * I) + (kp * D);

    Motor(ml + PID_output, mr - PID_output);
    //Serial.println(errors);

    if (analogRead(28) >= 2500)
    {
      break;
    }
    if (mcp_f(3) > md_mcp_f(3) && mcp_f(4) > md_mcp_f(4))
    {
      break;
    }
  }

  unsigned long lasts_timea = millis();
  while (millis() - lasts_timea < tim)
  {
    float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 40 );
    float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 40 );
    errors = error_L - error_R;
    I = 0;
    previous_I = 0;
    previous_error = 0;
    P = errors;
    I = I + previous_I;
    D = errors - previous_error ;
    previous_I = I;
    previous_error = errors  ;
    PID_output = (kp * P) + (0.00015 * I) + (kp * D);

    Motor(15 + PID_output, 15 - PID_output);
    if (mcp_f(3) < md_mcp_f(3) && mcp_f(4) < md_mcp_f(4) && mcp_f(2) < md_mcp_f(2) && mcp_f(5) < md_mcp_f(5))
    {
      break;
    }
  }
  Motor(15, 15); delay(200);
  Motor(-15, -15); delay(10);
  Motor(0, 0); delay(50);

}
void circle()
{
     fline_2sensor(30,30,0.7,0,"3:4","a0",'p',10);delay(30);
     do{Motor(-15,40);}while(mcp_f(3) > md_mcp_f(3));
     Motor(15,-40); delay(20);
     Motor(0, 0); delay(100);

     fline_2sensor(20,20,0.6,0,"3:4","a0",'p',0);
     do{Motor(40,-20);}while(mcp_f(3) > md_mcp_f(3));
     fline_2sensor(25,25,0.8,0,"3:4","a0",'p',0);
     fline(20,20, 0.2, 100, 'n', 'l',40, "a4", 20); 

     fline_2sensor(25,25,0.8,0,"3:4","a0",'p',0);delay(20);
     do{Motor(40,-20);}while(mcp_f(3) > md_mcp_f(3));
     fline_2sensor(30,30,0.7,0,"3:4","a0",'p',20);
     fline(20,20, 0.1, 50, 'n', 'l',40, "a3", 20);

}
