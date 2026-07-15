// Non-blocking three-sensor measurement.  Logical/serial order is right,
// front, left.  A value of -1 means no valid echo.

void printUltrasonic() {
  Serial.print(F("US,"));
  Serial.print(ultrasonicCm[ULTRASONIC_RIGHT], 1);
  Serial.print(',');
  Serial.print(ultrasonicCm[ULTRASONIC_FRONT], 1);
  Serial.print(',');
  Serial.println(ultrasonicCm[ULTRASONIC_LEFT], 1);
}

void updateUltrasonic() {
  const unsigned long nowUs = micros();
  switch (ultrasonicState) {
    case ULTRASONIC_IDLE:
      if (millis() - lastUltrasonicStartMs >= ULTRASONIC_BETWEEN_SENSORS_MS) {
        digitalWrite(ultrasonicTrigPins[ultrasonicSensorIndex], LOW);
        ultrasonicPhaseStartUs = nowUs;
        ultrasonicState = ULTRASONIC_TRIGGER_LOW;
      }
      break;
    case ULTRASONIC_TRIGGER_LOW:
      if (nowUs - ultrasonicPhaseStartUs >= 3UL) {
        digitalWrite(ultrasonicTrigPins[ultrasonicSensorIndex], HIGH);
        ultrasonicPhaseStartUs = nowUs;
        ultrasonicState = ULTRASONIC_TRIGGER_HIGH;
      }
      break;
    case ULTRASONIC_TRIGGER_HIGH:
      if (nowUs - ultrasonicPhaseStartUs >= 10UL) {
        digitalWrite(ultrasonicTrigPins[ultrasonicSensorIndex], LOW);
        ultrasonicPhaseStartUs = nowUs;
        ultrasonicState = ULTRASONIC_WAIT_RISE;
      }
      break;
    case ULTRASONIC_WAIT_RISE:
      if (digitalRead(ultrasonicEchoPins[ultrasonicSensorIndex]) == HIGH) {
        ultrasonicEchoRiseUs = nowUs;
        ultrasonicState = ULTRASONIC_WAIT_FALL;
      } else if (nowUs - ultrasonicPhaseStartUs >= ULTRASONIC_ECHO_TIMEOUT_US) {
        ultrasonicCm[ultrasonicSensorIndex] = -1.0F;
        lastUltrasonicStartMs = millis();
        ultrasonicSensorIndex = (ultrasonicSensorIndex + 1) % ULTRASONIC_COUNT;
        ultrasonicState = ULTRASONIC_IDLE;
      }
      break;
    case ULTRASONIC_WAIT_FALL:
      if (digitalRead(ultrasonicEchoPins[ultrasonicSensorIndex]) == LOW) {
        const float cm = ((nowUs - ultrasonicEchoRiseUs) * 0.0343F) / 2.0F;
        ultrasonicCm[ultrasonicSensorIndex] = (cm >= 5.0F && cm <= 400.0F) ? cm : -1.0F;
        lastUltrasonicStartMs = millis();
        ultrasonicSensorIndex = (ultrasonicSensorIndex + 1) % ULTRASONIC_COUNT;
        ultrasonicState = ULTRASONIC_IDLE;
      } else if (nowUs - ultrasonicEchoRiseUs >= ULTRASONIC_ECHO_TIMEOUT_US) {
        ultrasonicCm[ultrasonicSensorIndex] = -1.0F;
        lastUltrasonicStartMs = millis();
        ultrasonicSensorIndex = (ultrasonicSensorIndex + 1) % ULTRASONIC_COUNT;
        ultrasonicState = ULTRASONIC_IDLE;
      }
      break;
  }
}

// The policy switch keeps sensor collection independent from motion policy.
const char *forwardBlockReason(int m1, int m2, int m3, int m4) {
  if (!ULTRASONIC_BLOCKING_ENABLED) return nullptr;
  const int rightForward = (m1 > 0 ? m1 : 0) + (m4 > 0 ? m4 : 0);
  const int leftForward = (m2 > 0 ? m2 : 0) + (m3 > 0 ? m3 : 0);
  if (rightForward == 0 && leftForward == 0) return nullptr;
  if (ultrasonicCm[ULTRASONIC_FRONT] >= 0.0F && ultrasonicCm[ULTRASONIC_FRONT] <= FRONT_STOP_DISTANCE_CM) return "FRONT";
  return nullptr;
}

void stopForObstacle(const char *reason) {
  releaseAllMotors();
  timeoutStopped = true;
  Serial.print(F("BLOCK:"));
  Serial.println(reason);
}
