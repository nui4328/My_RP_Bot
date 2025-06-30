// ฟังก์ชัน normalize มุม
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// หมุนหุ่นยนต์ตามมุมที่กำหนดด้วย PID
// หมุนหุ่นยนต์
void rotateRobot(int speed, int degree) {
  speed = constrain(speed, 0, 100);

  // หยุดมอเตอร์และรีเซ็ตมุม
  Motor(0, 0);
  delay(100);
  my_GYRO::resetAngles();
  delay(10);

  // อ่านมุมเริ่มต้นแบบเฉลี่ย
  float initialDegree = 0;
  for (int i = 0; i < 10; i++) {
    initialDegree += my.gyro('z');
    delay(5);
  }
  initialDegree /= 10.0;

  // ตั้งมุมเป้าหมาย
  float targetDegree = initialDegree + degree;
  float error = 0, previous_error = 0;
  float integral = 0, output = 0;

  unsigned long lastTime = millis();
  unsigned long timeout = 2000;
  unsigned long startTime = millis();

  while (true) {
    float currentDegree = my.gyro('z');
    error = targetDegree - currentDegree;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    if (dt > 0) {
      integral += error * dt;
      float derivative = (error - previous_error) / dt;
      previous_error = error;
      output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
    }

    // 🔸 จำกัด output เมื่อใกล้มุมเป้าหมาย เพื่อเบรกไม่ให้เลย
    if (abs(error) < 15) {
      output = constrain(output, -20, 20);
    } else {
      output = constrain(output, -speed, speed);
    }

    Motor(output, -output); // ซ้ายบวก ขวาลบ
    delay(5);

    // 🔸 ปรับเงื่อนไขหยุดให้แม่นขึ้น
    if (abs(error) < 3 && abs(output) < 5) break;
    if (millis() - startTime > timeout) break;

    // Debug
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" deg, Output: ");
    Serial.print(output);
    Serial.println("%");
  }

  // หยุดมอเตอร์
  Motor(0, 0);
  delay(50);

  // แสดงมุมสุดท้าย
  float finalDegree = my.gyro('z');
  Serial.print("Final angle: ");
  Serial.print(finalDegree);
  Serial.print(" deg, Target: ");
  Serial.print(targetDegree);
  Serial.println(" deg");
}
