/*
  M3 continuous + serial-triggered M4 test

  Motor Shield V1 / L293D:
    M3 runs continuously at PWM 255.
    Send W or w at 9600 baud to run M4 for 1 second.
*/
#include <AFMotor.h>

// ================= Easy-to-adjust parameters =================
const uint8_t M3_PWM = 255;
const uint8_t M4_PWM = 255;
const unsigned long M4_RUN_TIME_MS = 1000UL;
const uint8_t M3_DIRECTION = BACKWARD;
const uint8_t M4_DIRECTION = BACKWARD;
// =============================================================

AF_DCMotor motorM3(3);
AF_DCMotor motorM4(4);

bool m4Running = false;
unsigned long m4StartedAt = 0;

void startM4() {
  motorM4.setSpeed(M4_PWM);
  motorM4.run(M4_DIRECTION);
  m4StartedAt = millis();
  m4Running = true;
  Serial.println(F("M4,RUN"));
}

void stopM4() {
  motorM4.setSpeed(0);
  motorM4.run(RELEASE);
  m4Running = false;
  Serial.println(F("M4,STOP"));
}

void setup() {
  Serial.begin(9600);

  motorM3.setSpeed(M3_PWM);
  motorM3.run(M3_DIRECTION);

  motorM4.setSpeed(0);
  motorM4.run(RELEASE);

  Serial.println(F("READY: send W to run M4 for 1000 ms"));
}

void loop() {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();
    if (command == 'W' || command == 'w') {
      startM4();
    }
  }

  if (m4Running && millis() - m4StartedAt >= M4_RUN_TIME_MS) {
    stopM4();
  }
}
