
// ค่าคงที่

const int ramp_delay = 6; // ms
// ตัวแปร global
int slmotor = 20, srmotor = 20; // PWM
int clml = -90, clmr = 90; // เลี้ยวซ้าย center
int crml = 90, crmr = -90; // เลี้ยวขวา center
int flml = -20, flmr = 100; // เลี้ยวซ้าย front
int frml = 100, frmr = -20; // เลี้ยวขวา front
int llmotor = 100, lrmotor = 50, ldelaymotor = 50; // เลี้ยวซ้าย speed
int rlmotor = 50, rrmotor = 100, rdelaymotor = 50; // เลี้ยวขวา speed
int break_ff = 5, break_fc = 30, break_bf = 10, break_bc = 20; // การหน่วง
int delay_f = 10; // การหน่วงก่อนเลี้ยว
float kd_f = 0.75, kd_b = 0.025; // Kd PID
float kp_slow = 0.2, ki_slow = 0.0001; // PID ช้า
float redius_wheel = 3.0; // รัศมีล้อ (cm)
int ch_p = 0;
bool _fw = true;
float new_encoder = 0;

void wheel_redius(float rediuss) {
    redius_wheel = rediuss;
}

float wheel_distance() {
    return 2 * 3.14159 * redius_wheel / 10; // เซนติเมตร
}

void test_distance(int distance1) {
    new_encoder = 440 * distance1 / wheel_distance();
    Serial.println(new_encoder);
    encoder.resetEncoders();
    do {
        Motor(30, 30);
    } while (encoder.Poss_R() < new_encoder);
    Motor(-30, -30);
    delay(10);
    Motor(1, 1);
}

void to_slow_motor(int sl, int sr) {
    slmotor = sl;
    srmotor = sr;
}

void to_turn_center_l(int ml, int mr) {
    clml = ml;
    clmr = mr;
}

void to_turn_center_r(int ml, int mr) {
    crml = ml;
    crmr = mr;
}

void to_turn_front_l(int ml, int mr) {
    flml = ml;
    flmr = mr;
}

void to_turn_front_r(int ml, int mr) {
    frml = ml;
    frmr = mr;
}

void to_brake_fc(int ff, int fc) {
    break_ff = ff;
    break_fc = fc;
}

void to_brake_bc(int bf, int bc) {
    break_bf = bf;
    break_bc = bc;
}

void to_delay_f(int ff) {
    delay_f = ff;
}

void to_speed_turn_fl(int inM, int outM, int delayM) {
    llmotor = inM;
    lrmotor = outM;
    ldelaymotor = delayM;
}

void to_speed_turn_fr(int inM, int outM, int delayM) {
    rlmotor = inM;
    rrmotor = outM;
    rdelaymotor = delayM;
}

void kd_fw(float kd) {
    kd_f = kd;
}

void kd_bw(float kd) {
    kd_b = kd;
}

void kp_sl(float kp_sl, float ki_sl) {
    kp_slow = kp_sl;
    ki_slow = ki_sl;
}

void turn_speed_fl() {
    for (int t = 0; t < ldelaymotor; t++) {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_f * D);
        Motor(llmotor + PID_output, lrmotor - PID_output);
        delay(ramp_delay);
    }
}

void turn_speed_fr() {
    for (int t = 0; t < rdelaymotor; t++) {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_f * D);
        Motor(rlmotor + PID_output, rrmotor - PID_output);
        delay(ramp_delay);
    }
}

void bturn_speed_fl() {
    for (int t = 0; t < ldelaymotor; t++) {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_b * D);
        Motor(-(llmotor + PID_output), -((lrmotor - 10) - PID_output));
        delay(ramp_delay);
    }
    delay(2);
}

void bturn_speed_fr() {
    for (int t = 0; t < rdelaymotor; t++) {
        errors = error_A();
        P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_b * D);
        Motor(-(rlmotor + PID_output), -((rrmotor - 10) - PID_output));
        delay(ramp_delay);
    }
    delay(2);
}


void fline(int spl, int spr, float kp, float distance, char nfc, char splr, int power, String sensor, int endt) {
    _fw = true;
    char sensors[4];
    sensor.toCharArray(sensors, sizeof(sensors));
    int sensor_f = atoi(&sensors[1]);
    int target_speed = min(spl, spr); // PWM
    const int ramp_step = 2;
    float traveled_distance = 0;
    unsigned long last_time = millis();
    float speed_scale = 1.0; // สมมติ 1 PWM = 0.1 cm/s (ต้องปรับตามจริง)

    if (kp == 0)
       {
        I = kp_slow = ki_slow = 0;
       }
    if(kp < 6.5)
      {
          pid_error = true;
      }
    else
      {
        pid_error = false;
      }

    int current_speed = 0; // PWM
    if(spl == 0)
      {
        goto _line;
      }
    // Soft start
    if (!ch_p) {
        while (current_speed < target_speed) {
            if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                errors = 0;
            } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                errors = 10;
            } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                errors = -10;
            } else {
              errors = error_A();
            }
            P = errors;
            I += errors * (ramp_delay / 1000.0);
            D = errors - previous_error;
            previous_error = errors;
            PID_output = (kp * P) + (0.000001 * I) + (kd_f * D);
            Motor(current_speed - PID_output, current_speed + PID_output);

            if (distance > 0) {
                unsigned long current_time = millis();
                float delta_time = (current_time - last_time) / 1000.0;
                traveled_distance += (current_speed * speed_scale) * delta_time;
                last_time = current_time;
                if (traveled_distance >= distance) {
                    Motor(0, 0);
                    return;
                }
            }
            current_speed += ramp_step;
            if (current_speed > target_speed) current_speed = target_speed;
            delay(ramp_delay);
            Serial.println(errors);
        }
    }

    // วิ่งปกติ
    while (1) 
      {
        if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                errors = 0;
            } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                errors = 10;
            } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                errors = -10;
            } else {
              errors = error_A();
            }
        P = errors;
        I += errors * 0.00005; // สำหรับ 50us
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.000001 * I) + (kd_f * D);
        Motor(spl - PID_output, spr + PID_output);       
        if (distance > 0) 
          {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 500.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;
            if (traveled_distance >= distance) {
                for(int i=spl; i>slmotor ; i--)
                  {
                    errors = error_A();
                    P = errors;
                    I += errors * 0.00005; // สำหรับ 50us
                    D = errors - previous_error;
                    previous_error = errors;
                    PID_output = (kp/2 * P) + (0.000001 * I) + (kd_f * D);
                    Motor(i - PID_output, i + PID_output);
                  }
                Motor(0, 0);
                break;
            }
          }
        else
          {
             if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
            (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) 
              {
                break;
              }
          }
        delayMicroseconds(50);
     }
    
    _line:
    ch_p = 0;
    
    if (nfc == 'n')
      {          
          if (splr == 'p')
            {
              ch_p = 1;
              if(spl >= 1)
                {
                  while (1) {
                    if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                        errors = 0;
                    } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                        errors = 10;
                    } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                        errors = -10;
                    } else {
                      errors = error_A();
                    }
                        P = errors;
                        I += errors * 0.00005;
                        D = errors - previous_error;
                        previous_error = errors;
                        PID_output = (kp_slow * P) + (ki_slow * D);
                        Motor(slmotor - PID_output, srmotor + PID_output);
                        delayMicroseconds(50);
                        if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) ) ||
                            (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) {
                            break;
                        }
                    }
                }
              else
                {
                  while (1) {
                        if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                            errors = 0;
                        } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                            errors = 10;
                        } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                            errors = -10;
                        } else {
                          errors = error_A();
                        }
                            P = errors;
                            I += errors * 0.00005;
                            D = errors - previous_error;
                            previous_error = errors;
                            PID_output = (kp_slow * P) + (ki_slow * D);
                            Motor(slmotor - PID_output, srmotor + PID_output);
                            delayMicroseconds(50);
                            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) ) ||
                                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) {
                                break;
                            }
                        }
                }
            }
        else if (splr == 's')
            {
              if(endt > 0)
                {
                  Motor(-spl, -spr);
                  delay(endt);
                  Motor(-1, -1);
                  delay(10); 
                }
              else
                {
                  for(int i=spl; i>15; i--)
                    {
                      errors = error_A();
                
                      P = errors;
                      I += errors * 0.00005;
                      D = errors - previous_error;
                      previous_error = errors;
                      PID_output = (kp_slow * P) + (ki_slow * D);
                      Motor(slmotor - PID_output, srmotor + PID_output);
                      delayMicroseconds(50);
                    }
                  
                }
               
              goto  _entN;
            }
      }
    else if (nfc == 'f') 
      {
        if (distance > 0 || spl == 0) {
            while (1) {
                errors = error_A();
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(slmotor - PID_output, srmotor + PID_output);
                delayMicroseconds(50);
                if ((read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(1) < md_sensorA(1)-50 )  ||
                    (read_sensorA(7) < md_sensorA(7)-50  && read_sensorA(6) < md_sensorA(6)-50  )) {
                    break;
                }
            }
        }
        if (splr == 'p') 
          {
            ch_p = 1;
            while (1) 
              {
                Motor(spl, spr);
                delayMicroseconds(50);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) {
                    break;
                }
              }
            delay(10);
            if (endt > 0) 
              {
                Motor(-spl, -spr);
                delay(endt);
                Motor(-1, -1);
                delay(10);
              } 
            else 
              {
                Motor(0, 0);
              }

          } 
        else if (splr == 's') 
          {
            Motor(-spl, -spr);
            delay(endt);
            Motor(0, 0);
            delay(2);
          }
      } 
    else if (nfc == 'c') 
      {
        if(splr == 'p') 
          {
            ch_p = 1;
            while (1) 
              {
                if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                    errors = 0;
                } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                    errors = 10;
                } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                    errors = -10;
                } else {
                  errors = error_A();
                }
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
                Motor(spl - PID_output, spr + PID_output);
                delayMicroseconds(50);
                if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                    break;
                }
              }
            if (endt > 0) 
              {
                Motor(-spl, -spr);
                delay(endt);
                Motor(-1, -1);
                delay(10);
              } 
            else 
              {
                Motor(0, 0);
              }
          }
        else
          {
            while (1) 
              {
                if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) && read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                    errors = 0;
                } else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                    errors = 10;
                } else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                    errors = -10;
                } else {
                  errors = error_A();
                }
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
                Motor(slmotor - PID_output, slmotor + PID_output);
                delayMicroseconds(50);
                if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                    break;
                }
            }
          }
        if (splr == 's') {
            Motor(-spl, -spr);
            delay(endt);
            Motor(0, 0);
            delay(2);
        }
    }

    if (splr == 'l') 
      {
        if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0)) {
            while (1) {
                Motor(slmotor, srmotor);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) {
                    delay(delay_f);
                    break;
                }
            }
            Motor(-(slmotor + 20), -(srmotor + 20));
            delay(break_ff);
            for (int i = 0; i <= sensor_f; i++) {
                do {
                    Motor((flml * power) / 100, (flmr * power) / 100);
                    delayMicroseconds(50);
                } while (read_sensorA(i) > md_sensorA(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
            delay(nfc == 'n' ? 2 : break_fc);
            int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
            int end_i = start_i ? sensor_f : sensor_f;
            for (int i = start_i; i <= end_i; i++) {
                do {
                    Motor((clml * power) / 100, (clmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fl();
        } else {
            Motor(-((clml * power) / 100), -((clmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      } 
    else if (splr == 'r') 
      {
        if (nfc == 'f') {
            while (1) {
                Motor(slmotor, srmotor);
                delayMicroseconds(50);
                if (read_sensorA(0) >= md_sensorA(0) && read_sensorA(7) >= md_sensorA(7)) {
                    break;
                }
            }
            delay(delay_f);
            Motor(-slmotor, -srmotor);
            delay(break_ff);
            for (int i = 7; i >= sensor_f; i--) {
                do {
                    Motor((frml * power) / 100, (frmr * power) / 100);
                    delayMicroseconds(50);
                } while (read_sensorA(i) > md_sensorA(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
            delay(nfc == 'n' ? 2 : break_fc);
            int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
            for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--) {
                do {
                    Motor((crml * power) / 100, (crmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fr();
        } else {
            Motor(-((crml * power) / 100), -((crmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      }
    

    
    _entN: delay(5);
}


void bline(int spl, int spr, float kp, float distance, char nfc, char splr, int power, String sensor, int endt) {
    _fw = false; // โหมดถอยหลัง
    char sensors[4];
    sensor.toCharArray(sensors, sizeof(sensors));
    int sensor_f = atoi(&sensors[1]); // เช่น "b5" -> sensor_f = 5
    int target_speed = min(spl, spr); // PWM
    const int ramp_step = 2;
    float traveled_distance = 0;
    unsigned long last_time = millis();
    float speed_scale = 1.25; // จากการสอบเทียบก่อนหน้า

    if (kp == 0) {
        I = kp_slow = ki_slow = 0;
    }
    if (kp < 6.5) {
        pid_error = true;
    } else {
        pid_error = false;
    }

    int current_speed = 0; // PWM
    // Soft start
    if(spl == 0)
      {
        goto _line;
      }
    if (!ch_p) {
        while (current_speed < target_speed) {
            if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                errors = 0;
            } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                errors = 10;
            } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                errors = -10;
            } else {
                errors = error_B();
            }
            P = errors;
            I += errors * (ramp_delay / 1000.0);
            if (I > 1000) I = 1000;
            if (I < -1000) I = -1000;
            D = errors - previous_error;
            previous_error = errors;
            PID_output = (kp * P) + (0.00005 * I) + (kd_f * D);
            Motor(-(current_speed + PID_output), -(current_speed - PID_output)); // ถอยหลัง
            if (distance > 0) {
                unsigned long current_time = millis();
                float delta_time = (current_time - last_time) / 1000.0;
                traveled_distance += (current_speed * speed_scale) * delta_time;
                last_time = current_time;
                if (traveled_distance >= distance) {
                    Motor(0, 0);
                    return;
                }
            }
            current_speed += ramp_step;
            if (current_speed > target_speed) current_speed = target_speed;
            delay(ramp_delay);
            Serial.println(errors);
        }
    }

    // วิ่งปกติ
    while (1) {
        if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
            errors = 0;
        } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
            errors = 10;
        } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
            errors = -10;
        } else {
            errors = error_B();
        }
        P = errors;
        I += errors * 0.00005;
        if (I > 1000) I = 1000;
        if (I < -1000) I = -1000;
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (kp * P) + (0.00005 * I) + (kd_f * D);
        Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง
        
        if (distance > 0) {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;
            if (traveled_distance >= distance) {
                Motor(0, 0);
                break;
            }
        }
        else
          {
              if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
            (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) )) {
            break;
        }
          }
        delayMicroseconds(100);
        Serial.println(errors);
    }
    _line:
    ch_p = 0;

    if (nfc == 'n') {
        if (splr == 'p') {
            ch_p = 1;
            while (1) {
                if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                    errors = 0;
                } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                    errors = 10;
                } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                    errors = -10;
                } else {
                    errors = error_B();
                }
                P = errors;
                I += errors * 0.00005;
                if (I > 1000) I = 1000;
                if (I < -1000) I = -1000;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
                delayMicroseconds(50);
                if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1) ) ||
                    (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) )) {
                    break;
                }
            }
        } else if (splr == 's') {
            if (endt > 0) {
                Motor(spl, spr); // เบรก
                delay(endt);
                Motor(-1, -1);
                delay(10);
            }
            goto _entN;
        }
    } else if (nfc == 'f') {
        if (distance > 0 || spl == 0) {
            while (1) {
                if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                    errors = 0;
                } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                    errors = 10;
                } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                    errors = -10;
                } else {
                    errors = error_B();
                }
                P = errors;
                I += errors * 0.00005;
                if (I > 1000) I = 1000;
                if (I < -1000) I = -1000;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(-(slmotor + PID_output), -(srmotor - PID_output)); // ถอยหลัง
                delayMicroseconds(50);
                if ((read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1) ) ||
                    (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6) )) {
                    break;
                }
            }
        }
        if (splr == 'p') {
            ch_p = 1;
            while (1) {
                Motor(-spl, -spr); // ถอยหลัง
                delayMicroseconds(50);
                if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7)) {
                    break;
                }
            }
            delay(10);
        } else if (splr == 's') {
            Motor(spl, spr); // เบรก
            delay(endt);
            Motor(0, 0);
            delay(2);
        }
    } else if (nfc == 'c') 
        {
            
          if (splr == 'p') 
            {
                ch_p = 1;
                while (1) 
                  {
                    float error_L = map(read_sensorB(3), sensorMin_B[3], sensorMax_B[3], 0, 20);
                    float error_R = map(read_sensorB(4), sensorMin_B[4], sensorMax_B[4], 0, 20);
                    errors = error_L - error_R;
                    P = errors;
                    I += errors * 0.00005;
                    if (I > 1000) I = 1000;
                    if (I < -1000) I = -1000;
                    D = errors - previous_error;
                    previous_error = errors;
                    PID_output = (kp_slow * P) + (ki_slow * D);
                    Motor(-(spl + PID_output), -(spr - PID_output)); // ถอยหลัง
                    delayMicroseconds(50);
                    if (analogRead(46) < md_sensorC(0) || analogRead(47) < md_sensorC(1)) {
                        break;
                    }
                  }
                if (endt > 0) 
                  {
                    Motor(spl, spr);
                    delay(endt);
                    Motor(1, 1);
                    delay(10);
                  } 
                else 
                  {
                    Motor(0, 0);
                  }
            }
          else
            {
              while (1) 
                {
                  if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) && read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                      errors = 0;
                  } else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                      errors = 10;
                  } else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                      errors = -10;
                  } else {
                      errors = error_B();
                  }
                  P = errors;
                  D = errors - previous_error;
                  previous_error = errors;
                  PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
                  Motor(-(slmotor + PID_output), -(slmotor - PID_output)); // ถอยหลัง
                  delayMicroseconds(50);
                  if (analogRead(46) < md_sensorC(0)-50 || analogRead(47) < md_sensorC(1)-50) {
                      break;
                  }
                }

            }
          if (splr == 's') {
              Motor(spl, spr); // เบรก
              delay(endt);
              Motor(0, 0);
              delay(2);
          }
    }

  if (splr == 'l') 
    {
        if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0)) 
          {
            while (1) {
                Motor(-slmotor, -srmotor);
                if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7)) {
                    delay(delay_f);
                    break;
                }
            }
            Motor(slmotor + 10, srmotor + 10);
            delay(break_bf);                             //---------------------------------------------------------------------------------->>>
            for (int i = 0; i <= sensor_f; i++) {
                do {
                    Motor(-((flmr*power)/100), -((flml*power)/100)); 
                    delayMicroseconds(50);
                } while (read_sensorB(i) > md_sensorB(i) - 50);
                delayMicroseconds(50);
            }
          }
        else 
          {
            Motor(nfc == 'n' ? 0 : slmotor, nfc == 'n' ? 0 : srmotor);
            delay(nfc == 'n' ? 2 : break_bc);
            int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
            int end_i = start_i ? sensor_f : sensor_f;
            for (int i = start_i; i <= end_i; i++) {
                do {
                    Motor((clml * power) / 100, (clmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
          }
        if (endt == 0) {
            turn_speed_fl();
        } else {
            Motor(-((clml * power) / 100), -((clmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
      } 
    else if (splr == 'r') 
      {
        if (nfc == 'f') {
            while (1) {
                Motor(-slmotor, -srmotor);
                delayMicroseconds(50);
                if (read_sensorB(0) >= md_sensorB(0) && read_sensorB(7) >= md_sensorB(7)) {
                    break;
                }
            }
            delay(delay_f);
            Motor(slmotor, srmotor);
            delay(break_bf);
            for (int i = 7; i >= sensor_f; i--) {
                do {
                    Motor(-((frmr*power)/100), -((frml*power)/100));
                    delayMicroseconds(50);
                } while (read_sensorB(i) > md_sensorB(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : slmotor, nfc == 'n' ? 0 : srmotor);
            delay(nfc == 'n' ? 2 : break_bc);
            int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
            for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--) {
                do {
                    Motor((crml * power) / 100, (crmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fr();
        } else {
            Motor(-((crml * power) / 100), -((crmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
    }

    if (!(nfc == 'n' && splr == 's' && endt == 0)) {
        Motor(-1, -1); // หยุด
        delay(endt / 2);
    }

    _entN: delay(5);
}



/*
void fline(int spl, int spr, float kp, float distance, char nfc, char splr, int power, String sensor, int endt) {
    _fw = true;
    char sensors[4];
    sensor.toCharArray(sensors, sizeof(sensors));
    int sensor_f = atoi(&sensors[1]);
    int target_speed = min(spl, spr);
    const int ramp_step = 2;

    if (kp == 0) {
        I = kp_slow = ki_slow = 0;
    }

    if (distance > 0) 
    {
        new_encoder = 440 * distance / wheel_distance();
        encoder.resetEncoders();
        int current_speed = 10;

        if (!ch_p) {
            while (current_speed < target_speed && encoder.Poss_R() < new_encoder) {
              if(read_sensorA(0) > md_sensorA(0) && read_sensorA(3) > md_sensorA(3), read_sensorA(4) > md_sensorA(4) && read_sensorA(7) > md_sensorA(7))
                {
                  PID_output = 0;

                }
              else if(read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6), read_sensorA(7) > md_sensorA(7))
                {
                  PID_output = 10;

                }
              else if(read_sensorA(2) > md_sensorA(2) && read_sensorA(1) > md_sensorA(1), read_sensorA(0) > md_sensorA(0))
                {
                  PID_output = -10;

                }
                errors = error_A();
                P = errors;
                I += errors * (ramp_delay / 1000.0);
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp * P) + (0.00001 * I) + (kd_f * D);
                Motor(current_speed - PID_output, current_speed + PID_output);

                if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) && read_sensorA(3) < md_sensorA(3)) ||
                    (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) && read_sensorA(4) < md_sensorA(4))) {
                    break;
                }
                current_speed += ramp_step;
                delay(ramp_delay);

            }
        }

        while (encoder.Poss_R() < new_encoder) {
              if(read_sensorA(0) > md_sensorA(0) && read_sensorA(3) > md_sensorA(3), read_sensorA(4) > md_sensorA(4) && read_sensorA(7) > md_sensorA(7))
                {
                  PID_output = 0;
                }
              else if(read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6), read_sensorA(7) > md_sensorA(7))
                {
                  PID_output = 10;
                }
              else if(read_sensorA(2) > md_sensorA(2) && read_sensorA(1) > md_sensorA(1), read_sensorA(0) > md_sensorA(0))
                {
                  PID_output = -10;
                }
            errors = error_A();
            P = errors;
            I += errors * 0.00005;
            D = errors - previous_error;
            previous_error = errors;
            PID_output = (kp * P) + (0.00001 * I) + (kd_f * D);
            Motor(spl - PID_output, spr + PID_output);

            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) && read_sensorA(3) < md_sensorA(3)) ||
                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) && read_sensorA(4) < md_sensorA(4))) {
                break;
            }
            delayMicroseconds(50);
            Serial.println(PID_output);
        }
        
    } 
    else 
      {
        if (spl == 0) 
          {
            delay(10);
          } 
        else if (!ch_p) 
          {
            for (int i = 5; i < spl; i += ramp_step) {
              if(read_sensorA(0) > md_sensorA(0) && read_sensorA(3) > md_sensorA(3), read_sensorA(4) > md_sensorA(4) && read_sensorA(7) > md_sensorA(7))
                {
                  PID_output = 0;
                }
              else if(read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6), read_sensorA(7) > md_sensorA(7))
                {
                  PID_output = 10;
                }
              else if(read_sensorA(2) > md_sensorA(2) && read_sensorA(1) > md_sensorA(1), read_sensorA(0) > md_sensorA(0))
                {
                  PID_output = -10;
                }
                errors = error_A();
                P = errors;
                I += errors * (ramp_delay / 1000.0);
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp / 1.5 * P) + (0.00001 * I) + (kd_f * D);
                Motor(i - PID_output, i + PID_output);

                if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) ) ||
                    (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) 
                  {
                    break;
                  }
                delay(ramp_delay);
                Serial.println(PID_output);
            }
          }
        while (1) 
          {
              if(read_sensorA(0) > md_sensorA(0) && read_sensorA(3) > md_sensorA(3), read_sensorA(4) > md_sensorA(4) && read_sensorA(7) > md_sensorA(7))
                {
                  PID_output = 0;
                }
              else if(read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6), read_sensorA(7) > md_sensorA(7))
                {
                  PID_output = 10;
                }
              else if(read_sensorA(2) > md_sensorA(2) && read_sensorA(1) > md_sensorA(1), read_sensorA(0) > md_sensorA(0))
                {
                  PID_output = -10;
                }
            errors = error_A();
            P = errors;
            D = errors - previous_error;                  
            previous_error = errors;
            PID_output = (kp * P) + (0.00001 * I) + (kd_f * D);
            Motor(spl - PID_output, spr + PID_output);
            delayMicroseconds(50);
            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) ) ||
                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) )) {
                break;
            }
            Serial.println(PID_output);
          }
    }

    ch_p = 0;

    if (nfc == 'n' && splr == 'p') {
        ch_p = 1;
        while (1) {
          if(read_sensorA(0) > md_sensorA(0) && read_sensorA(3) > md_sensorA(3), read_sensorA(4) > md_sensorA(4) && read_sensorA(7) > md_sensorA(7))
            {
               PID_output = 0;

            }
            float error_L = map(read_sensorA(3), sensorMin_A[3], sensorMax_A[3], 0, 30);
            float error_R = map(read_sensorA(4), sensorMin_A[4], sensorMax_A[4], 0, 30);
            errors = error_L - error_R;
            P = errors;
            I += errors * 0.00005;
            D = errors - previous_error;
            previous_error = errors;
            PID_output = (kp_slow * P) + (ki_slow * D);
            Motor((spl >= 1 ? spl : slmotor) - PID_output, (spl >= 1 ? spr : slmotor) + PID_output);
            delayMicroseconds(50);
            if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) && read_sensorA(2) < md_sensorA(2)) ||
                (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) && read_sensorA(4) < md_sensorA(4))) {
                break;
            }
        }
    } else if (nfc == 'f') {
        if (distance > 0 || spl == 0) {
            while (1) {
              if(read_sensorA(0) > md_sensorA(0) && read_sensorA(3) > md_sensorA(3), read_sensorA(4) > md_sensorA(4) && read_sensorA(7) > md_sensorA(7))
                {
                   PID_output = 0;

                }
                errors = error_A();
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(slmotor - PID_output, srmotor + PID_output);
                delayMicroseconds(50);
                if ((read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1) && read_sensorA(2) < md_sensorA(1)) ||
                    (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6) && read_sensorA(4) < md_sensorA(4))) {
                    break;
                }
            }
        }
        if (splr == 'p') {
            ch_p = 1;
            while (1) {
                Motor(spl, spr);
                delayMicroseconds(50);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) {
                    break;
                }
            }
            delay(10);
        } else if (splr == 's') {
            Motor(-spl, -spr);
            delay(endt);
            Motor(0, 0);
            delay(2);
        }
    } else if (nfc == 'c') {
        while (1) {
          if(read_sensorA(0) > md_sensorA(0) && read_sensorA(3) > md_sensorA(3), read_sensorA(4) > md_sensorA(4) && read_sensorA(7) > md_sensorA(7))
                {
                  errors = 0;
                }
              else if(read_sensorA(5) > md_sensorA(5) && read_sensorA(6) > md_sensorA(6), read_sensorA(7) > md_sensorA(7))
                {
                  errors = 10;
                }
              else if(read_sensorA(2) > md_sensorA(2) && read_sensorA(1) > md_sensorA(1), read_sensorA(0) > md_sensorA(0))
                {
                  errors = -10;
                }
            errors = 0; //error_A();
            P = errors;
            I += errors * 0.00005;
            D = errors - previous_error;
            previous_error = errors;
            PID_output = (kp / 2.5 * P) + (0.0000001 * I) + (0.0125 * D);
            Motor(slmotor - PID_output, slmotor + PID_output);
            delayMicroseconds(50);
            
            if (analogRead(46) < md_sensorC(0) || analogRead(47) <  md_sensorC(1) ) {
                break;
            }
            
        }
        if (splr == 'p') {
            ch_p = 1;
            while (1) {
                float error_L = map(read_sensorA(3), sensorMin_A[3], sensorMax_A[3], 0, 20);
                float error_R = map(read_sensorA(4), sensorMin_A[4], sensorMax_A[4], 0, 20);
                errors = error_L - error_R;
                P = errors;
                I += errors * 0.00005;
                D = errors - previous_error;
                previous_error = errors;
                PID_output = (kp_slow * P) + (ki_slow * D);
                Motor(spl - PID_output, spr + PID_output);
                delayMicroseconds(50);
                if (analogRead(46) < md_sensorC(0) || analogRead(47) <= md_sensorC(1) ) {
                    break;
                }
            }
        }
        if (splr == 's') {
            Motor(-spl, -spr);
            delay(endt);
            Motor(0, 0);
            delay(2);
        }
    }

    if (splr == 'l') {
        if (nfc == 'f' || (nfc == 'n' && spl > 0 && distance == 0)) {
            while (1) {
                Motor(slmotor, srmotor);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) {
                    delay(delay_f);
                    break;
                }
            }
            Motor(-(slmotor + 20), -(srmotor + 20));
            delay(break_ff);
            for (int i = 0; i <= sensor_f; i++) {
                do {
                    Motor((flml * power) / 100, (flmr * power) / 100);
                    delayMicroseconds(50);
                } while (read_sensorA(i) > md_sensorA(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
            delay(nfc == 'n' ? 2 : break_fc);
            int start_i = (sensor[0] == 'a' && sensor_f >= 5) ? 5 : 0;
            int end_i = start_i ? sensor_f : sensor_f;
            for (int i = start_i; i <= end_i; i++) {
                do {
                    Motor((clml * power) / 100, (clmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fl();
        } else {
            Motor(-((clml * power) / 100), -((clmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
    } else if (splr == 'r') {
        if (nfc == 'f') {
            while (1) {
                Motor(slmotor, srmotor);
                delayMicroseconds(50);
                if (read_sensorA(0) >= md_sensorA(0) && read_sensorA(7) >= md_sensorA(7)) {
                    break;
                }
            }
            delay(delay_f);
            Motor(-slmotor, -srmotor);
            delay(break_ff);
            for (int i = 7; i >= sensor_f; i--) {
                do {
                    Motor((frml * power) / 100, (frmr * power) / 100);
                    delayMicroseconds(50);
                } while (read_sensorA(i) > md_sensorA(i) - 50);
                delayMicroseconds(50);
            }
        } else {
            Motor(nfc == 'n' ? 0 : -slmotor, nfc == 'n' ? 0 : -srmotor);
            delay(nfc == 'n' ? 2 : break_fc);
            int start_i = (sensor[0] == 'a' && sensor_f <= 2) ? 5 : 0;
            for (int i = 7; i >= (sensor[0] == 'a' ? sensor_f - start_i : sensor_f); i--) {
                do {
                    Motor((crml * power) / 100, (crmr * power) / 100);
                    delayMicroseconds(50);
                } while ((sensor[0] == 'a' ? read_sensorA(i - start_i) : read_sensorB(i)) > (sensor[0] == 'a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                delayMicroseconds(50);
            }
        }
        if (endt == 0) {
            turn_speed_fr();
        } else {
            Motor(-((crml * power) / 100), -((crmr * power) / 100));
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
    }

    if (!(nfc == 'n' && splr == 's' && endt == 0)) {
        Motor(1, 1);
        delay(endt / 2);
    }
  
}
*/


