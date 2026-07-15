// Non-blocking centre/front ultrasonic sensor: TRIG 26, ECHO 27.
// The US wire format stays three columns for web compatibility: -1,front,-1.

void printUltrasonic() {
  Serial.print(F("US,-1,"));
  Serial.print(frontDistanceCm, 1);
  Serial.println(F(",-1"));
}

void printUltrasonicDebug() {
  Serial.print(F("US:FRONT="));
  Serial.println(frontDistanceCm, 1);
}

void updateUltrasonic() {
  const unsigned long nowUs = micros();
  switch (ultrasonicState) {
    case ULTRASONIC_IDLE:
      if (millis() - lastUltrasonicStartMs >= ULTRASONIC_BETWEEN_SENSORS_MS) {
        digitalWrite(FRONT_TRIG_PIN, LOW); ultrasonicPhaseStartUs = nowUs; ultrasonicState = ULTRASONIC_TRIGGER_LOW;
      }
      break;
    case ULTRASONIC_TRIGGER_LOW:
      if (nowUs - ultrasonicPhaseStartUs >= 3UL) { digitalWrite(FRONT_TRIG_PIN, HIGH); ultrasonicPhaseStartUs = nowUs; ultrasonicState = ULTRASONIC_TRIGGER_HIGH; }
      break;
    case ULTRASONIC_TRIGGER_HIGH:
      if (nowUs - ultrasonicPhaseStartUs >= 10UL) { digitalWrite(FRONT_TRIG_PIN, LOW); ultrasonicPhaseStartUs = nowUs; ultrasonicState = ULTRASONIC_WAIT_RISE; }
      break;
    case ULTRASONIC_WAIT_RISE:
      if (digitalRead(FRONT_ECHO_PIN) == HIGH) { ultrasonicEchoRiseUs = nowUs; ultrasonicState = ULTRASONIC_WAIT_FALL; }
      else if (nowUs - ultrasonicPhaseStartUs >= ULTRASONIC_ECHO_TIMEOUT_US) { frontDistanceCm = -1.0F; lastUltrasonicStartMs = millis(); ultrasonicState = ULTRASONIC_IDLE; }
      break;
    case ULTRASONIC_WAIT_FALL:
      if (digitalRead(FRONT_ECHO_PIN) == LOW) {
        const float cm = ((nowUs - ultrasonicEchoRiseUs) * 0.0343F) / 2.0F;
        frontDistanceCm = (cm >= 5.0F && cm <= 400.0F) ? cm : -1.0F;
        lastUltrasonicStartMs = millis(); ultrasonicState = ULTRASONIC_IDLE;
      } else if (nowUs - ultrasonicEchoRiseUs >= ULTRASONIC_ECHO_TIMEOUT_US) { frontDistanceCm = -1.0F; lastUltrasonicStartMs = millis(); ultrasonicState = ULTRASONIC_IDLE; }
      break;
  }
}

// Both sides must be commanded forward.  Any pivot has one side backward, so
// left/right in-place turns remain available even when an obstacle is close.
const char *forwardBlockReason(int m1, int m2, int m3, int m4) {
  if (!ULTRASONIC_BLOCKING_ENABLED) return nullptr;
  const bool drivingForward = m1 > 0 && m2 > 0 && m3 > 0 && m4 > 0;
  if (drivingForward && frontDistanceCm >= 0.0F && frontDistanceCm <= FRONT_STOP_DISTANCE_CM) return "FRONT";
  return nullptr;
}

void stopForObstacle(const char *reason) {
  releaseAllMotors(); timeoutStopped = true;
  Serial.print(F("BLOCK:")); Serial.println(reason);
}
