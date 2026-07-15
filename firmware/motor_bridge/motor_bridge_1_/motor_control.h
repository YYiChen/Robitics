// Low-level four-motor output and complete safe-stop reset.

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

void releaseAllMotors() {
  motor1.setSpeed(0); motor2.setSpeed(0);
  motor3.setSpeed(0); motor4.setSpeed(0);
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
  for (int i = 0; i < 4; ++i) currentMotorCommands[i] = 0;

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
