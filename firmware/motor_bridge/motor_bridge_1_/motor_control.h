// Low-level motor output and safe-stop reset.

int clampMotorCommand(int value) {
  return constrain(value, -MOTOR_COMMAND_LIMIT, MOTOR_COMMAND_LIMIT);
}

int commandDirection(int value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

void applyOneMotor(AF_DCMotor &motor, int command) {
  command = clampMotorCommand(command);
  motor.setSpeed((uint8_t)abs(command));
  if (command > 0) motor.run(FORWARD);
  else if (command < 0) motor.run(BACKWARD);
  else motor.run(RELEASE);
}

void resetDriveControlState() {
  speedControlEnabled = false;
  targetSpeedLeft = 0.0f;
  targetSpeedRight = 0.0f;
  pidIntegralLeft = 0.0f;
  pidIntegralRight = 0.0f;
  pidLastErrorLeft = 0.0f;
  pidLastErrorRight = 0.0f;
  pidOutputLeft = 0;
  pidOutputRight = 0;
}

// M1/M2 are the vehicle drive motors. Timed card motors M3/M4 deliberately
// keep their independent state when the drive watchdog, STOP, or obstacle
// guard stops the vehicle.
void releaseDriveMotors() {
  motor1.setSpeed(0); motor2.setSpeed(0);
  motor1.run(RELEASE); motor2.run(RELEASE);
  currentMotorCommands[0] = 0;
  currentMotorCommands[1] = 0;
  resetDriveControlState();
}

// Full release is reserved for initialisation. Runtime drive stops use
// releaseDriveMotors() so active M3/M4 cycles can finish safely.
void releaseAllMotors() {
  releaseDriveMotors();
  motor3.setSpeed(0); motor4.setSpeed(0);
  motor3.run(RELEASE); motor4.run(RELEASE);
  currentMotorCommands[2] = 0;
  currentMotorCommands[3] = 0;
}
