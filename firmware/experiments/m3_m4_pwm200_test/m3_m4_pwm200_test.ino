/*
  Temporary M3 + M4 PWM 200 test

  Hardware:
    Arduino Mega + Motor Shield V1 / L293D
    M3 motor on shield output M3
    M4 motor on shield output M4

  Serial monitor: 9600 baud, Newline
    START  run M3 and M4
    STOP   stop both immediately
    M3     run M3 only
    M4     run M4 only
*/
#include <AFMotor.h>

// ================= Easy-to-adjust parameters =================
const uint8_t MOTOR_PWM = 200;       // 0..255
const uint8_t M3_DIRECTION = BACKWARD;
const uint8_t M4_DIRECTION = BACKWARD;
const bool AUTO_START = true;        // true: both run immediately after boot
// =============================================================

AF_DCMotor motorM3(3);
AF_DCMotor motorM4(4);

char commandBuffer[16];
uint8_t commandLength = 0;

void stopMotor(AF_DCMotor &motor) {
  motor.setSpeed(0);
  motor.run(RELEASE);
}

void stopBoth() {
  stopMotor(motorM3);
  stopMotor(motorM4);
  Serial.println(F("OK:STOP,M3=0,M4=0"));
}

void runM3() {
  motorM3.setSpeed(MOTOR_PWM);
  motorM3.run(M3_DIRECTION);
}

void runM4() {
  motorM4.setSpeed(MOTOR_PWM);
  motorM4.run(M4_DIRECTION);
}

void runBoth() {
  runM3();
  runM4();
  Serial.print(F("OK:START,M3="));
  Serial.print(MOTOR_PWM);
  Serial.print(F(",M4="));
  Serial.println(MOTOR_PWM);
}

void executeCommand(char *command) {
  for (char *cursor = command; *cursor != '\0'; ++cursor) {
    if (*cursor >= 'a' && *cursor <= 'z') {
      *cursor = *cursor - 'a' + 'A';
    }
  }

  if (strcmp(command, "START") == 0) {
    runBoth();
  } else if (strcmp(command, "STOP") == 0) {
    stopBoth();
  } else if (strcmp(command, "M3") == 0) {
    stopBoth();
    runM3();
    Serial.print(F("OK:M3,"));
    Serial.println(MOTOR_PWM);
  } else if (strcmp(command, "M4") == 0) {
    stopBoth();
    runM4();
    Serial.print(F("OK:M4,"));
    Serial.println(MOTOR_PWM);
  } else if (*command != '\0') {
    Serial.println(F("ERR:USE START,STOP,M3,M4"));
  }
}

void setup() {
  Serial.begin(9600);
  stopBoth();
  Serial.println(F("READY:M3_M4_PWM200_TEST"));
  Serial.println(F("COMMANDS:START,STOP,M3,M4"));

  if (AUTO_START) {
    runBoth();
  }
}

void loop() {
  while (Serial.available() > 0) {
    const char received = (char)Serial.read();
    if (received == '\n' || received == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        executeCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = received;
    } else {
      commandLength = 0;
      Serial.println(F("ERR:COMMAND_TOO_LONG"));
    }
  }
}
