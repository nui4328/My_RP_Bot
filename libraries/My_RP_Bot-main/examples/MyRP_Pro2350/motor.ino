
int sl, sr; // ตัวแปรความเร็วสำหรับมอเตอร์ซ้ายและขวา

// ฟังก์ชันควบคุมมอเตอร์ซ้าย/ขวา
void Motor(int pwmL, int pwmR) {
  // ตั้งความละเอียด PWM เป็น 12 บิต (0–4095)
  analogWriteResolution(12);

  // ตั้งความถี่ PWM เป็น 20kHz (ลดเสียงรบกวนมอเตอร์)
  analogWriteFreq(20000);
  // แปลงค่าจาก -100..100 ให้เป็น 0..4095
  int pwmValueL = map(abs(pwmL), 0, 100, 0, 4095);
  int pwmValueR = map(abs(pwmR), 0, 100, 0, 4095);

  // มอเตอร์ซ้าย
  if (pwmL > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwmL < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    pwmValueL = 0;
  }

  // มอเตอร์ขวา
  if (pwmR > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (pwmR < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    pwmValueR = 0;
  }

  // ส่งค่า PWM (0–4095)
  analogWrite(PWMA, pwmValueL);
  analogWrite(PWMB, pwmValueR);
}
