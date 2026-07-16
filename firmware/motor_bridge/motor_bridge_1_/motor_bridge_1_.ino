#include <Arduino.h>
#include <AFMotor.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <Wire.h>
#include <Servo.h>

// ============================================================
// Raspberry Pi -> Arduino motor bridge  +  IMU  +  Speed PID
//
// Serial protocol (9600 baud, one command per line):
//
//   Motor control:
//     M,m1,m2,m3,m4\n         raw PWM (-255..255) for each motor
//
//   Speed control (rear wheels):
//     V,leftPPS,rightPPS\n    target speed in pulses/sec (signed)
//                              PID overrides M3 / M4 in M commands
//
//   PID tuning:
//     KP,value\n  KI,value\n  KD,value\n
//
//   Queries:
//     STOP | S                 stop + disable speed control
//     IMU                      query Euler angles
//     SPD                      query wheel speeds + PID state
//     US                       query the front ultrasonic distance
//     SV,angle                 set SG90 servo angle (0..180)
//
//   Motor mapping:
//     M1 = right front   M2 = left front
//     M3 = left rear     M4 = right rear
//
//   Hall encoders:
//     Pin 18 (INT3) = left rear wheel
//     Pin 19 (INT2) = right rear wheel
//     8-pole magnet → 4 pulses per revolution
//
//   Speed unit: pulses/second (pps).
//     Convert m/s → pps:  pps = 4 * (m/s) / (π * wheel_diameter_m)
// ============================================================

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// AFMotor accepts PWM values from 0 to 255.
const int MOTOR_COMMAND_LIMIT = 255;

// If no valid motor command is received within this interval,
// release all motors.
const unsigned long COMMAND_TIMEOUT_MS = 1000;

// While the robot is stationary, publish a clear serial heartbeat so that a
// connected Serial Monitor can confirm the Arduino is alive and stopped.
const unsigned long STOPPED_REPORT_INTERVAL_MS = 1000;

// Before changing a motor from forward to backward, release all
// motors briefly to reduce current and mechanical shock.
const unsigned long REVERSE_PAUSE_MS = 80;

// ============================================================
// One forward-facing ultrasonic sensor.  It stops forward travel only.
//
// Only the front TRIG/ECHO pin pair is used by this firmware.
// ============================================================

constexpr bool ULTRASONIC_BLOCKING_ENABLED = true;
constexpr uint8_t FRONT_TRIG_PIN = 26;
constexpr uint8_t FRONT_ECHO_PIN = 27;

constexpr unsigned long ULTRASONIC_ECHO_TIMEOUT_US = 30000UL;
constexpr unsigned long ULTRASONIC_BETWEEN_SENSORS_MS = 35UL;
// Only the single centre/front sensor is safety active.  A distance at or
// below this threshold blocks forward wheel commands; pivots and reverse are
// intentionally left available.
constexpr float FRONT_STOP_DISTANCE_CM = 10.0F;

float frontDistanceCm = -1.0F;

enum UltrasonicState {
  ULTRASONIC_IDLE,
  ULTRASONIC_TRIGGER_LOW,
  ULTRASONIC_TRIGGER_HIGH,
  ULTRASONIC_WAIT_RISE,
  ULTRASONIC_WAIT_FALL
};

UltrasonicState ultrasonicState = ULTRASONIC_IDLE;
unsigned long ultrasonicPhaseStartUs = 0;
unsigned long ultrasonicEchoRiseUs = 0;
unsigned long lastUltrasonicStartMs = 0;

// ============================================================
// MPU-6500 IMU constants and configuration
// ============================================================

// I2C address (AD0 pin low = 0x68, AD0 pin high = 0x69)
const uint8_t MPU_ADDR = 0x68;

// Register addresses
const uint8_t MPU_REG_WHO_AM_I     = 0x75;
const uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;
const uint8_t MPU_REG_GYRO_CONFIG  = 0x1B;
const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU_REG_CONFIG       = 0x1A;
const uint8_t MPU_REG_SMPLRT_DIV   = 0x19;
const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU_REG_GYRO_XOUT_H  = 0x43;

// Expected WHO_AM_I response for MPU-6500
const uint8_t MPU_WHO_AM_I_EXPECTED = 0x70;

// IMU timing
const unsigned long IMU_SAMPLE_INTERVAL_MS = 10;   // 100 Hz internal sample rate
const unsigned long IMU_REPORT_INTERVAL_MS = 100;   // 10 Hz report rate to serial

// Complementary filter: higher alpha = more trust in gyro, less in accel
const float COMPLEMENTARY_ALPHA = 0.96f;

// Sensor full-scale ranges (must match register configuration)
const float GYRO_FULL_SCALE_DPS  = 250.0f;   // FS_SEL = 0 => +/-250 deg/s
const float ACCEL_FULL_SCALE_G   = 2.0f;     // AFS_SEL = 0 => +/-2 g

// Conversion: 16-bit signed range / full-scale range
const float GYRO_LSB_PER_DPS  = 32768.0f / GYRO_FULL_SCALE_DPS;   // 131.072
const float ACCEL_LSB_PER_G   = 32768.0f / ACCEL_FULL_SCALE_G;    // 16384.0

// Degrees conversion
// Arduino.h already defines RAD_TO_DEG as a macro on AVR boards.
const float RAD_TO_DEG_FACTOR = 180.0f / PI;

// Gyro calibration: number of stationary samples at startup
const int GYRO_CALIBRATION_SAMPLES = 200;

// ============================================================
// Hall-effect encoder (wheel speed) constants
// ============================================================

// 8-pole magnet ring → 4 full pulses per wheel revolution.
const float PULSES_PER_REVOLUTION = 4.0f;

// Encoder input pins (Arduino Mega external interrupt pins).
const uint8_t ENCODER_LEFT_PIN  = 18;  // INT3 — left rear motor (M3)
const uint8_t ENCODER_RIGHT_PIN = 19;  // INT2 — right rear motor (M4)

// SG90 signal wire.  Power the servo from a suitable 5 V supply with a
// common ground to the Mega; do not power it from the Mega's 5 V pin.
constexpr uint8_t SERVO_PIN = 22;
constexpr int SERVO_CENTER_ANGLE = 90;
Servo panServo;

// Speed is computed from pulse counts at this interval (ms).
const unsigned long SPEED_CALC_INTERVAL_MS = 50;   // 20 Hz

// PID update rate for rear-wheel speed control (ms).
const unsigned long PID_UPDATE_INTERVAL_MS = 50;   // 20 Hz

// PID output limits (PWM).
const float PID_OUTPUT_MIN = 0.0f;
const float PID_OUTPUT_MAX = 255.0f;

// Maximum length of one serial command, including the final '\0'.
const size_t COMMAND_BUFFER_SIZE = 64;

char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;
bool discardUntilNewline = false;

unsigned long lastValidCommandTime = 0;
bool timeoutStopped = true;

// Last commands after clamping.
int currentMotorCommands[4] = {0, 0, 0, 0};
unsigned long lastStoppedReportTime = 0;

// ============================================================
// MPU-6500 IMU state variables
// ============================================================

// True if the MPU-6500 was detected and initialised successfully.
bool imuPresent = false;

// Timing book-keeping for non-blocking IMU reads.
unsigned long lastImuSampleTime = 0;
unsigned long lastImuReportTime = 0;
unsigned long lastGyroTimestamp = 0;

// Euler angles (degrees).
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

// Gyro bias offsets (degrees/second), computed during calibration.
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// ============================================================
// Hall encoder — pulse counting (updated by ISRs)
// ============================================================

// volatile: written in ISR context, read in loop().
volatile unsigned long encoderPulsesLeft  = 0;
volatile unsigned long encoderPulsesRight = 0;

// ============================================================
// Speed control state
// ============================================================

// True when speed control is active for the rear wheels.
bool speedControlEnabled = false;

// Target speeds in pulses per second (signed: + = forward, – = backward).
float targetSpeedLeft  = 0.0f;
float targetSpeedRight = 0.0f;

// Measured speeds (pulses/second).
float currentSpeedLeft  = 0.0f;
float currentSpeedRight = 0.0f;

// PID output: signed PWM values for the rear motors.
int pidOutputLeft  = 0;
int pidOutputRight = 0;

// PID internal state.
float pidIntegralLeft   = 0.0f;
float pidIntegralRight  = 0.0f;
float pidLastErrorLeft  = 0.0f;
float pidLastErrorRight = 0.0f;

// PID gains (tune these!).
float speedKp = 2.0f;
float speedKi = 0.8f;
float speedKd = 0.05f;

// Speed calculation timer.
unsigned long lastSpeedCalcTime = 0;

// Copies of the pulse counters at the previous speed calculation tick.
unsigned long prevPulsesLeft  = 0;
unsigned long prevPulsesRight = 0;

#if 0
int clampMotorCommand(int value) {
  return constrain(value, -MOTOR_COMMAND_LIMIT, MOTOR_COMMAND_LIMIT);
}

int commandDirection(int value) {
  if (value > 0) {
    return 1;
  }
  if (value < 0) {
    return -1;
  }
  return 0;
}

void applyOneMotor(AF_DCMotor &motor, int command) {
  command = clampMotorCommand(command);
  const uint8_t pwm = (uint8_t)abs(command);

  motor.setSpeed(pwm);

  if (command > 0) {
    motor.run(FORWARD);
  } else if (command < 0) {
    motor.run(BACKWARD);
  } else {
    motor.run(RELEASE);
  }
}

void releaseAllMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  for (int i = 0; i < 4; ++i) {
    currentMotorCommands[i] = 0;
  }

  // Reset speed-control state so a fresh V command is required.
  speedControlEnabled   = false;
  targetSpeedLeft       = 0.0f;
  targetSpeedRight      = 0.0f;
  pidIntegralLeft       = 0.0f;
  pidIntegralRight      = 0.0f;
  pidLastErrorLeft      = 0.0f;
  pidLastErrorRight     = 0.0f;
  pidOutputLeft         = 0;
  pidOutputRight        = 0;
}

#endif
#include "motor_control.h"

// ============================================================
// MPU-6500 I2C helper functions
// ============================================================

// Write a single byte to an MPU-6500 register.
// Returns true on success (I2C ACK received).
bool writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// Burst-read `count` bytes starting from `startReg` into `buffer`.
// Returns true if all requested bytes were received.
bool readMPURegisters(uint8_t startReg, uint8_t *buffer, size_t count) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(MPU_ADDR, count);
  if (received != count) {
    return false;
  }

  for (size_t i = 0; i < count; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

// ============================================================
// MPU-6500 sensor fusion
// ============================================================

// Calibrate the gyroscope by averaging stationary samples.
// The robot MUST remain still during this call (~400 ms).
void calibrateGyro() {
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; ++i) {
    uint8_t buffer[6];
    if (readMPURegisters(MPU_REG_GYRO_XOUT_H, buffer, 6)) {
      int16_t gx = (int16_t)((buffer[0] << 8) | buffer[1]);
      int16_t gy = (int16_t)((buffer[2] << 8) | buffer[3]);
      int16_t gz = (int16_t)((buffer[4] << 8) | buffer[5]);
      sumX += gx;
      sumY += gy;
      sumZ += gz;
    }
    delay(2);
  }

  // Convert average raw value to degrees/second.
  gyroBiasX = (sumX / GYRO_CALIBRATION_SAMPLES) / GYRO_LSB_PER_DPS;
  gyroBiasY = (sumY / GYRO_CALIBRATION_SAMPLES) / GYRO_LSB_PER_DPS;
  gyroBiasZ = (sumZ / GYRO_CALIBRATION_SAMPLES) / GYRO_LSB_PER_DPS;
}

// Initialise the MPU-6500: wake up, configure ranges, calibrate gyro.
// Returns true if the sensor is present and ready.
bool initMPU6500() {
  // 1. Verify chip identity.
  uint8_t whoAmI = 0;
  if (!readMPURegisters(MPU_REG_WHO_AM_I, &whoAmI, 1)) {
    return false;
  }
  if (whoAmI != MPU_WHO_AM_I_EXPECTED) {
    return false;
  }

  // 2. Wake up from sleep (write 0 to PWR_MGMT_1).
  if (!writeMPURegister(MPU_REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(10);

  // 3. Gyro config: FS_SEL = 0 (+/-250 dps), no self-test.
  if (!writeMPURegister(MPU_REG_GYRO_CONFIG, 0x00)) {
    return false;
  }

  // 4. Accel config: AFS_SEL = 0 (+/-2 g), no self-test.
  if (!writeMPURegister(MPU_REG_ACCEL_CONFIG, 0x00)) {
    return false;
  }

  // 5. DLPF_CFG = 3 (44.8 Hz gyro bandwidth, 44.8 Hz accel bandwidth
  //    at 1 kHz internal sample rate).
  if (!writeMPURegister(MPU_REG_CONFIG, 0x03)) {
    return false;
  }

  // 6. Sample rate divider = 0 => 1 kHz / (1+0) = 1 kHz internal rate.
  if (!writeMPURegister(MPU_REG_SMPLRT_DIV, 0x00)) {
    return false;
  }

  // 7. Gyro bias calibration (robot must be stationary).
  calibrateGyro();

  lastGyroTimestamp = millis();
  return true;
}

// Burst-read 14 bytes from ACCEL_XOUT_H:
//   accel X/Y/Z (6), temperature (2), gyro X/Y/Z (6)
// Returns true on successful read.
bool readRawIMU(int16_t *accel, int16_t *gyro) {
  uint8_t buffer[14];
  if (!readMPURegisters(MPU_REG_ACCEL_XOUT_H, buffer, 14)) {
    return false;
  }

  accel[0] = (int16_t)((buffer[0]  << 8) | buffer[1]);   // AX
  accel[1] = (int16_t)((buffer[2]  << 8) | buffer[3]);   // AY
  accel[2] = (int16_t)((buffer[4]  << 8) | buffer[5]);   // AZ

  // buffer[6..7] = temperature, skipped

  gyro[0]  = (int16_t)((buffer[8]  << 8) | buffer[9]);   // GX
  gyro[1]  = (int16_t)((buffer[10] << 8) | buffer[11]);  // GY
  gyro[2]  = (int16_t)((buffer[12] << 8) | buffer[13]);  // GZ

  return true;
}

// Complementary filter: fuse accelerometer gravity vector with
// gyroscope angular velocity integration.
// \param dt  Time delta in seconds since previous sample.
void updateAttitude(int16_t *accelRaw, int16_t *gyroRaw, float dt) {
  // Convert raw to physical units.
  float ax = accelRaw[0] / ACCEL_LSB_PER_G;
  float ay = accelRaw[1] / ACCEL_LSB_PER_G;
  float az = accelRaw[2] / ACCEL_LSB_PER_G;

  float gx = (gyroRaw[0] / GYRO_LSB_PER_DPS) - gyroBiasX;
  float gy = (gyroRaw[1] / GYRO_LSB_PER_DPS) - gyroBiasY;
  float gz = (gyroRaw[2] / GYRO_LSB_PER_DPS) - gyroBiasZ;

  // Roll / pitch from the accelerometer gravity vector.
  float accelRoll  = atan2(ay, az) * RAD_TO_DEG_FACTOR;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG_FACTOR;

  // Gyro-only integration step.
  float gyroRoll  = roll  + gx * dt;
  float gyroPitch = pitch + gy * dt;
  float gyroYaw   = yaw   + gz * dt;

  // Complementary filter — roll and pitch.
  roll  = COMPLEMENTARY_ALPHA * gyroRoll  +
          (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
  pitch = COMPLEMENTARY_ALPHA * gyroPitch +
          (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;

  // Yaw: gyro-only integration (drifts without magnetometer).
  yaw = gyroYaw;
}

// Print current Euler angles to serial.
void printIMU() {
  Serial.print(F("IMU,"));
  Serial.print(roll, 2);
  Serial.print(',');
  Serial.print(pitch, 2);
  Serial.print(',');
  Serial.println(yaw, 2);
}

// ============================================================
// Hall encoder interrupt service routines
// ============================================================

// Left rear encoder (pin 18, INT3).
void encoderLeftISR() {
  encoderPulsesLeft++;
}

// Right rear encoder (pin 19, INT2).
void encoderRightISR() {
  encoderPulsesRight++;
}

// ============================================================
// Speed calculation and PID speed control
// ============================================================

// Read accumulated pulse counts and compute instantaneous speed (pps).
// Must be called at regular intervals (SPEED_CALC_INTERVAL_MS).
void updateSpeedMeasurements() {
  unsigned long now = millis();
  float dt = (now - lastSpeedCalcTime) / 1000.0f;

  // Guard against bogus dt on the first call or after an overflow.
  if (dt <= 0.0f || dt > 1.0f) {
    prevPulsesLeft  = encoderPulsesLeft;
    prevPulsesRight = encoderPulsesRight;
    lastSpeedCalcTime = now;
    return;
  }

  // Atomically read and reset for the next window.
  noInterrupts();
  unsigned long pulsesL = encoderPulsesLeft;
  unsigned long pulsesR = encoderPulsesRight;
  interrupts();

  unsigned long deltaL = pulsesL - prevPulsesLeft;
  unsigned long deltaR = pulsesR - prevPulsesRight;

  // Convert to pulses per second, then apply a simple low-pass.
  float rawL = deltaL / dt;
  float rawR = deltaR / dt;
  const float speedAlpha = 0.7f;
  currentSpeedLeft  = speedAlpha * rawL + (1.0f - speedAlpha) * currentSpeedLeft;
  currentSpeedRight = speedAlpha * rawR + (1.0f - speedAlpha) * currentSpeedRight;

  prevPulsesLeft  = pulsesL;
  prevPulsesRight = pulsesR;
  lastSpeedCalcTime = now;
}

// Run one PID iteration for the rear wheels.
// \param dt  Time delta in seconds (≈ SPEED_CALC_INTERVAL_MS / 1000).
void updateSpeedPID(float dt) {
  // ----- Left rear -----
  float errorL = targetSpeedLeft - currentSpeedLeft;
  pidIntegralLeft  += errorL * dt;
  // Anti-windup: clamp integral.
  pidIntegralLeft   = constrain(pidIntegralLeft, -50.0f, 50.0f);
  float derivativeL = (errorL - pidLastErrorLeft) / dt;
  float outputL     = speedKp * errorL
                    + speedKi * pidIntegralLeft
                    + speedKd * derivativeL;
  pidLastErrorLeft  = errorL;

  // ----- Right rear -----
  float errorR = targetSpeedRight - currentSpeedRight;
  pidIntegralRight  += errorR * dt;
  pidIntegralRight  = constrain(pidIntegralRight, -50.0f, 50.0f);
  float derivativeR = (errorR - pidLastErrorRight) / dt;
  float outputR     = speedKp * errorR
                    + speedKi * pidIntegralRight
                    + speedKd * derivativeR;
  pidLastErrorRight = errorR;

  // Clamp PID output to valid PWM range.
  pidOutputLeft  = (int)constrain(outputL, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
  pidOutputRight = (int)constrain(outputR, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);

  // When target is zero, force output to zero and reset integrators.
  if (targetSpeedLeft == 0.0f) {
    pidOutputLeft    = 0;
    pidIntegralLeft  = 0.0f;
  }
  if (targetSpeedRight == 0.0f) {
    pidOutputRight    = 0;
    pidIntegralRight  = 0.0f;
  }
}

// Print current wheel speeds to serial:
// SPD,currentL,currentR,targetL,targetR,pidL,pidR
void printSpeed() {
  Serial.print(F("SPD,"));
  Serial.print(currentSpeedLeft, 1);
  Serial.print(',');
  Serial.print(currentSpeedRight, 1);
  Serial.print(',');
  Serial.print(targetSpeedLeft, 1);
  Serial.print(',');
  Serial.print(targetSpeedRight, 1);
  Serial.print(',');
  Serial.print(pidOutputLeft);
  Serial.print(',');
  Serial.println(pidOutputRight);
}

// Emit a periodic idle heartbeat.  This is intentionally independent of the
// remote-control watchdog: after power-up, or whenever all motors are stopped,
// the Serial Monitor can visibly confirm the safe state.
void reportStoppedState() {
  for (int i = 0; i < 4; ++i) {
    if (currentMotorCommands[i] != 0) {
      return;
    }
  }

  const unsigned long now = millis();
  if (now - lastStoppedReportTime >= STOPPED_REPORT_INTERVAL_MS) {
    Serial.println(F("STATUS:STOPPED"));
    lastStoppedReportTime = now;
  }
}

#include "ultrasonic_avoidance.h"

bool needsReversePause(const int nextCommands[4]) {
  for (int i = 0; i < 4; ++i) {
    const int oldDirection = commandDirection(currentMotorCommands[i]);
    const int newDirection = commandDirection(nextCommands[i]);

    if (oldDirection != 0 &&
        newDirection != 0 &&
        oldDirection != newDirection) {
      return true;
    }
  }
  return false;
}

void applyMotorCommands(int m1, int m2, int m3, int m4) {
  int nextCommands[4] = {
    clampMotorCommand(m1),
    clampMotorCommand(m2),
    clampMotorCommand(m3),
    clampMotorCommand(m4),
  };

  // In speed-control mode the rear-motor PWM values come from the PID
  // controller — not from the M command.
  if (speedControlEnabled) {
    nextCommands[2] = clampMotorCommand(pidOutputLeft);   // M3 – left rear
    nextCommands[3] = clampMotorCommand(pidOutputRight);  // M4 – right rear
  }

  const char *blockReason = forwardBlockReason(
    nextCommands[0], nextCommands[1], nextCommands[2], nextCommands[3]
  );
  if (blockReason != nullptr) {
    stopForObstacle(blockReason);
    return;
  }

  bool changed = false;
  for (int i = 0; i < 4; ++i) {
    if (nextCommands[i] != currentMotorCommands[i]) {
      changed = true;
      break;
    }
  }

  // A heartbeat with unchanged values only refreshes the timeout.
  if (!changed) {
    return;
  }

  if (needsReversePause(nextCommands)) {
    releaseAllMotors();
    delay(REVERSE_PAUSE_MS);
  }

  applyOneMotor(motor1, nextCommands[0]);
  applyOneMotor(motor2, nextCommands[1]);
  applyOneMotor(motor3, nextCommands[2]);
  applyOneMotor(motor4, nextCommands[3]);

  for (int i = 0; i < 4; ++i) {
    currentMotorCommands[i] = nextCommands[i];
  }

  Serial.print(F("OK:M,"));
  Serial.print(nextCommands[0]);
  Serial.print(',');
  Serial.print(nextCommands[1]);
  Serial.print(',');
  Serial.print(nextCommands[2]);
  Serial.print(',');
  Serial.println(nextCommands[3]);
}

bool parseIntegerStrict(const char *text, int &result) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *endPointer = nullptr;
  long value = strtol(text, &endPointer, 10);

  if (endPointer == text || *endPointer != '\0') {
    return false;
  }

  // Accept a wider input range, then clamp safely later.
  if (value < -32768L || value > 32767L) {
    return false;
  }

  result = (int)value;
  return true;
}

// Parse a floating-point string.  Returns true on success.
bool parseFloat(const char *text, float &result) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *endPointer = nullptr;
  // AVR libc used by the Mega exposes strtod(), but not strtof().
  double value = strtod(text, &endPointer);

  if (endPointer == text || *endPointer != '\0') {
    return false;
  }

  result = value;
  return true;
}

// Parse a speed-control command:  V,leftTarget,rightTarget
// Both values are in pulses per second (signed).
bool parseSpeedCommand(char *line, float &leftTarget, float &rightTarget) {
  char *savePointer = nullptr;
  char *token = strtok_r(line, ",", &savePointer);

  if (token == nullptr || toupper(token[0]) != 'V' || token[1] != '\0') {
    return false;
  }

  if (!parseFloat(strtok_r(nullptr, ",", &savePointer), leftTarget)) {
    return false;
  }
  if (!parseFloat(strtok_r(nullptr, ",", &savePointer), rightTarget)) {
    return false;
  }

  // Reject extra fields.
  token = strtok_r(nullptr, ",", &savePointer);
  return token == nullptr;
}

bool parseMotorCommand(char *line, int output[4]) {
  // Expected form: M,m1,m2,m3,m4
  char *savePointer = nullptr;
  char *token = strtok_r(line, ",", &savePointer);

  if (token == nullptr || toupper(token[0]) != 'M' || token[1] != '\0') {
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    token = strtok_r(nullptr, ",", &savePointer);
    if (!parseIntegerStrict(token, output[i])) {
      return false;
    }
  }

  // Reject extra fields.
  token = strtok_r(nullptr, ",", &savePointer);
  return token == nullptr;
}

void executeLine(char *line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }

  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "STOP") == 0 || strcmp(line, "S") == 0) {
    releaseAllMotors();
    lastValidCommandTime = millis();
    timeoutStopped = true;
    Serial.println(F("OK:STOP"));
    return;
  }

  // Query current IMU Euler angles (does not affect motor state).
  if (strcmp(line, "IMU") == 0) {
    if (imuPresent) {
      printIMU();
    } else {
      Serial.println(F("IMU,ABSENT"));
    }
    lastValidCommandTime = millis();
    return;
  }

  // Query current wheel speeds.
  if (strcmp(line, "SPD") == 0) {
    printSpeed();
    lastValidCommandTime = millis();
    return;
  }

  // Query the one front ultrasonic reading: US,frontCm.
  if (strcmp(line, "US") == 0) {
    printUltrasonic();
    lastValidCommandTime = millis();
    return;
  }

  // Servo commands deliberately do not refresh lastValidCommandTime: moving
  // the camera/accessory must never keep a driving motor command alive.
  if (strncmp(line, "SV,", 3) == 0) {
    int angle;
    if (parseIntegerStrict(line + 3, angle)) {
      angle = constrain(angle, 0, 180);
      panServo.write(angle);
      Serial.print(F("OK:SV,"));
      Serial.println(angle);
    } else {
      Serial.println(F("ERR:BAD_COMMAND"));
    }
    return;
  }

  // Set PID gains: KP,value  /  KI,value  /  KD,value
  if (strncmp(line, "KP,", 3) == 0) {
    float val;
    if (parseFloat(line + 3, val)) {
      speedKp = val;
      Serial.print(F("OK:KP,"));
      Serial.println(speedKp, 3);
    } else {
      Serial.println(F("ERR:BAD_COMMAND"));
    }
    lastValidCommandTime = millis();
    return;
  }
  if (strncmp(line, "KI,", 3) == 0) {
    float val;
    if (parseFloat(line + 3, val)) {
      speedKi = val;
      Serial.print(F("OK:KI,"));
      Serial.println(speedKi, 3);
    } else {
      Serial.println(F("ERR:BAD_COMMAND"));
    }
    lastValidCommandTime = millis();
    return;
  }
  if (strncmp(line, "KD,", 3) == 0) {
    float val;
    if (parseFloat(line + 3, val)) {
      speedKd = val;
      Serial.print(F("OK:KD,"));
      Serial.println(speedKd, 3);
    } else {
      Serial.println(F("ERR:BAD_COMMAND"));
    }
    lastValidCommandTime = millis();
    return;
  }

  // Speed-control command: V,leftTarget,rightTarget (pulses/second).
  float spdL, spdR;
  if (line[0] == 'V' || line[0] == 'v') {
    if (parseSpeedCommand(line, spdL, spdR)) {
      const int leftDirection = (spdL > 0.0F) ? 1 : ((spdL < 0.0F) ? -1 : 0);
      const int rightDirection = (spdR > 0.0F) ? 1 : ((spdR < 0.0F) ? -1 : 0);
      const char *blockReason = forwardBlockReason(
        0, 0, leftDirection, rightDirection
      );
      if (blockReason != nullptr) {
        stopForObstacle(blockReason);
        return;
      }
      speedControlEnabled = true;
      targetSpeedLeft  = spdL;
      targetSpeedRight = spdR;
      // Reset integrators on a new setpoint.
      pidIntegralLeft   = 0.0f;
      pidIntegralRight  = 0.0f;
      pidLastErrorLeft  = 0.0f;
      pidLastErrorRight = 0.0f;
      Serial.print(F("OK:V,"));
      Serial.print(spdL, 1);
      Serial.print(',');
      Serial.println(spdR, 1);
    } else {
      Serial.println(F("ERR:BAD_COMMAND"));
    }
    // V command does NOT refresh the motor watchdog by itself —
    // the RPi must still send M commands for the front motors.
    return;
  }

  int commands[4] = {0, 0, 0, 0};

  if (!parseMotorCommand(line, commands)) {
    releaseAllMotors();
    timeoutStopped = true;
    Serial.println(F("ERR:BAD_COMMAND"));
    return;
  }

  // A syntactically valid command refreshes the communication watchdog.
  lastValidCommandTime = millis();
  timeoutStopped = false;

  applyMotorCommands(
    commands[0],
    commands[1],
    commands[2],
    commands[3]
  );
}

void setup() {
  Serial.begin(9600);

  releaseAllMotors();
  lastValidCommandTime = millis();
  timeoutStopped = true;

  // ---- Centre/front ultrasonic sensor: TRIG 26, ECHO 27 ----
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  digitalWrite(FRONT_TRIG_PIN, LOW);
  // Start the first reading immediately after setup.
  lastUltrasonicStartMs = millis() - ULTRASONIC_BETWEEN_SENSORS_MS;

  // ---- Hall encoder pins ----
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN),
                  encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN),
                  encoderRightISR, RISING);

  panServo.attach(SERVO_PIN);
  panServo.write(SERVO_CENTER_ANGLE);

  // Initialise speed timestamps so the first dt is sane.
  lastSpeedCalcTime = millis();

  // Initialise I2C on pins 20 (SDA) and 21 (SCL).
  Wire.begin();

  // Attempt MPU-6500 initialisation.
  imuPresent = initMPU6500();

  if (imuPresent) {
    Serial.println(F("READY:MOTOR_BRIDGE,MAX=255,TIMEOUT=1000,SERVO=22,IMU=OK,ENC=18,19,US=FRONT(26,27)"));
  } else {
    Serial.println(F("READY:MOTOR_BRIDGE,MAX=255,TIMEOUT=1000,SERVO=22,IMU=ABSENT,ENC=18,19,US=FRONT(26,27)"));
  }
}

void loop() {
  while (Serial.available() > 0) {
    const char received = (char)Serial.read();

    if (discardUntilNewline) {
      if (received == '\n') {
        discardUntilNewline = false;
        commandLength = 0;
      }
      continue;
    }

    if (received == '\n') {
      commandBuffer[commandLength] = '\0';

      // Remove a possible Windows-style carriage return.
      if (commandLength > 0 &&
          commandBuffer[commandLength - 1] == '\r') {
        commandBuffer[commandLength - 1] = '\0';
      }

      executeLine(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = received;
    } else {
      commandLength = 0;
      discardUntilNewline = true;
      releaseAllMotors();
      timeoutStopped = true;
      Serial.println(F("ERR:COMMAND_TOO_LONG"));
    }
  }

  // Keep sensor sampling independent from remote-control command timing.
  updateUltrasonic();

  // An obstacle may appear after a previously safe forward command.  Stop it
  // here instead of waiting for the next remote heartbeat.
  const char *activeBlockReason = forwardBlockReason(
    currentMotorCommands[0], currentMotorCommands[1],
    currentMotorCommands[2], currentMotorCommands[3]
  );
  if (activeBlockReason != nullptr) {
    stopForObstacle(activeBlockReason);
  }

  if (!timeoutStopped &&
      millis() - lastValidCommandTime > COMMAND_TIMEOUT_MS) {
    releaseAllMotors();
    timeoutStopped = true;
    Serial.println(F("TIMEOUT:STOP"));
  }

  // ---- Non-blocking IMU sampling ----
  if (imuPresent) {
    unsigned long now = millis();

    // Read raw IMU data at the configured sample rate.
    if (now - lastImuSampleTime >= IMU_SAMPLE_INTERVAL_MS) {
      int16_t accelRaw[3], gyroRaw[3];
      if (readRawIMU(accelRaw, gyroRaw)) {
        float dt = (now - lastGyroTimestamp) / 1000.0f;
        // Reject unreasonably large gaps (e.g. motor-reversal pause).
        if (dt > 0.0f && dt < 0.5f) {
          updateAttitude(accelRaw, gyroRaw, dt);
        }
        lastGyroTimestamp = now;
      }
      lastImuSampleTime = now;
    }
  }

  // ---- Speed measurement + PID speed control ----
  {
    unsigned long now = millis();
    if (now - lastSpeedCalcTime >= SPEED_CALC_INTERVAL_MS) {
      float dt = (now - lastSpeedCalcTime) / 1000.0f;
      if (dt > 0.0f && dt < 1.0f) {
        updateSpeedMeasurements();

        if (speedControlEnabled) {
          updateSpeedPID(dt);
        }
      }
      lastSpeedCalcTime = now;
    }
  }

  reportStoppedState();
}
