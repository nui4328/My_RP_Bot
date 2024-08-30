

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
