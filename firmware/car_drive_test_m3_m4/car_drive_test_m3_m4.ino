/*
  Whole-car drive test: Arduino Mega + Motor Shield V1/L293D.

  Active outputs:
    M1 = right-side motor
    M2 = left-side motor
    M3/M4 = unused and always released

  Serial commands at 9600 baud (each followed by a newline):
    F / B / L / R / STOP
    SPD,0..255              set test PWM
    M,leftPWM,rightPWM      direct side test (-255..255)

  A command must arrive at least once every 700 ms while moving. The Pi test
  page does this automatically; a disconnected browser therefore stops safely.
*/
#include <AFMotor.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// Start conservatively for the first wheels-off-ground test; adjust with SPD.
const uint8_t DEFAULT_PWM = 100;
const unsigned long COMMAND_TIMEOUT_MS = 700UL;
const unsigned long REVERSE_PAUSE_MS = 80UL;

// Change either sign to -1 only if that physical side moves backward on F.
const int8_t LEFT_FORWARD_SIGN = 1;
const int8_t RIGHT_FORWARD_SIGN = 1;

AF_DCMotor motor1(1);  // right side
AF_DCMotor motor2(2);  // left side
AF_DCMotor motor3(3);  // unused
AF_DCMotor motor4(4);  // unused

int currentLeft = 0;
int currentRight = 0;
uint8_t testPwm = DEFAULT_PWM;
unsigned long lastCommandMs = 0;
bool timeoutStopped = true;

int clampPwm(int value) { return constrain(value, -255, 255); }

void applyOne(AF_DCMotor &motor, int value) {
  value = clampPwm(value);
  motor.setSpeed((uint8_t)abs(value));
  if (value > 0) motor.run(FORWARD);
  else if (value < 0) motor.run(BACKWARD);
  else motor.run(RELEASE);
}

void stopDrive(bool announce = true) {
  motor1.setSpeed(0); motor2.setSpeed(0);
  motor3.setSpeed(0); motor4.setSpeed(0);
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
  currentLeft = currentRight = 0;
  if (announce) Serial.println(F("DRIVE,STOP"));
}

void applySides(int left, int right) {
  left = clampPwm(left);
  right = clampPwm(right);
  const bool reversesLeft = currentLeft != 0 && left != 0 && ((currentLeft > 0) != (left > 0));
  const bool reversesRight = currentRight != 0 && right != 0 && ((currentRight > 0) != (right > 0));
  if (reversesLeft || reversesRight) {
    stopDrive(false);
    delay(REVERSE_PAUSE_MS);
  }

  // M1 is physically right, M2 physically left.
  applyOne(motor1, right * RIGHT_FORWARD_SIGN);
  applyOne(motor2, left * LEFT_FORWARD_SIGN);
  currentLeft = left;
  currentRight = right;

  Serial.print(F("DRIVE,LEFT=")); Serial.print(left);
  Serial.print(F(",RIGHT=")); Serial.println(right);
}

bool parseInteger(const char *text, int &out) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  const long value = strtol(text, &end, 10);
  if (*end != '\0' || value < -255 || value > 255) return false;
  out = (int)value;
  return true;
}

void executeCommand(char *line) {
  for (char *p = line; *p; ++p) *p = (char)toupper(*p);
  if (strcmp(line, "STOP") == 0 || strcmp(line, "S") == 0) {
    stopDrive();
    timeoutStopped = true;
  } else if (strcmp(line, "F") == 0) {
    applySides(testPwm, testPwm);
  } else if (strcmp(line, "B") == 0) {
    applySides(-testPwm, -testPwm);
  } else if (strcmp(line, "L") == 0) {
    applySides(-testPwm, testPwm);
  } else if (strcmp(line, "R") == 0) {
    applySides(testPwm, -testPwm);
  } else if (strncmp(line, "SPD,", 4) == 0) {
    int value;
    if (parseInteger(line + 4, value) && value >= 0) {
      testPwm = (uint8_t)value;
      Serial.print(F("DRIVE,PWM=")); Serial.println(testPwm);
    } else Serial.println(F("ERR:SPD must be 0..255"));
  } else if (strncmp(line, "M,", 2) == 0) {
    char *comma = strchr(line + 2, ',');
    int left, right;
    if (comma != nullptr) {
      *comma = '\0';
      if (parseInteger(line + 2, left) && parseInteger(comma + 1, right)) applySides(left, right);
      else Serial.println(F("ERR:M,left,right"));
    } else Serial.println(F("ERR:M,left,right"));
  } else if (strcmp(line, "STATUS") == 0) {
    Serial.print(F("STATUS,LEFT=")); Serial.print(currentLeft);
    Serial.print(F(",RIGHT=")); Serial.println(currentRight);
  } else {
    Serial.println(F("ERR: use F,B,L,R,STOP,SPD,n,M,left,right,STATUS"));
    return;
  }
  lastCommandMs = millis();
  if (strcmp(line, "STOP") != 0 && strcmp(line, "S") != 0) timeoutStopped = false;
}

void setup() {
  Serial.begin(9600);
  stopDrive(false);
  lastCommandMs = millis();
  Serial.println(F("READY:CAR_DRIVE_TEST,M1=RIGHT,M2=LEFT,TIMEOUT=700"));
}

void loop() {
  static char buffer[48];
  static uint8_t length = 0;
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (length > 0) { buffer[length] = '\0'; executeCommand(buffer); length = 0; }
    } else if (length < sizeof(buffer) - 1) {
      buffer[length++] = c;
    } else {
      length = 0; stopDrive(); Serial.println(F("ERR:COMMAND_TOO_LONG"));
    }
  }

  if (!timeoutStopped && millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    stopDrive();
    timeoutStopped = true;
    Serial.println(F("TIMEOUT:STOP"));
  }
}
