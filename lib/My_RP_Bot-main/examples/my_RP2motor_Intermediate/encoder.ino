

// ฟังก์ชั่นการโทรกลับเมื่อมีการเปลี่ยนแปลงค่าในพิน A
void encoderCallback() {
    int aVal = digitalRead(pinA);
    if (aVal != prevAVal) {
        if (digitalRead(pinB) != aVal) {
            encoderDir = -1;
        } else {
            encoderDir = 1;
        }
        encoderPos += encoderDir;
    }
    prevAVal = aVal;
}

void setup_encoder() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), encoderCallback, CHANGE);
    prevAVal = digitalRead(pinA);    
}

void loop_encoder() {
    Serial.print("Encoder position: ");
    Serial.println(encoderPos);
    delay(100);
}

void fw_encoder(int spl, int spr, volatile int pw, char slr)
  {
    calibration_Yak(); 
    encoderPos=0;
    while(encoderPos < pw*300)      
        {
          errors = error_Yaw_float();
          I = 0;
          previous_I = 0;
          previous_error = 0;
          P = errors;
          I = I + previous_I;
          D = errors - previous_error ;            
          previous_I=I;
          previous_error=errors  ;  
          PID_output = (3 * P) + (0.00015 * I) + (3* D); 
          
          Motor(spl - PID_output, spl + PID_output); 

          
       }
    Motor(spl, spr);delay(100);
    Motor(0,0);delay(100);

    if(slr='r')
      {
         calibration_Yak(); 
         do{Motor(65,-65);}while(error_Yaw_float()<=80);
         Motor(-65,65);delay(100);
         Motor(0,0);delay(200);
      }
  }
