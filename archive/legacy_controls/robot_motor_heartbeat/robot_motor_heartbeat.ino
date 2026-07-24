#include <AFMotor.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// =====================================================
// 电机定义
// M1 = 左前
// M2 = 右前
// M3 = 左后
// M4 = 右后
// =====================================================

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// =====================================================
// 可调参数
// =====================================================

// 正常 PWM 范围
const int MIN_SPEED = 0;
const int MAX_SPEED = 255;

// 启动瞬间使用的 PWM
const int START_BOOST_SPEED = 255;

// 启动冲击持续时间，单位 ms
// 建议从 200 开始，最多先不要超过 350
const unsigned long START_BOOST_TIME = 220;

// 电机已经转起来后，允许使用的最低巡航 PWM
const int MIN_HOLD_SPEED = 150;

// 转弯时内侧轮速度比例
const float TURN_RATIO = 0.70;

// 超过该时间没有收到新命令，自动停车
const unsigned long COMMAND_TIMEOUT = 1000;

// =====================================================
// 串口状态
// =====================================================

char commandBuffer[32];
uint8_t commandLength = 0;

unsigned long lastCommandTime = 0;
bool timeoutStopped = true;

// 当前运行状态
char currentAction = 'S';
int currentSpeed = 0;

// =====================================================
// 基础电机控制
// =====================================================

void setLeftSpeed(int speedValue) {
  speedValue = constrain(speedValue, 0, 255);

  motor1.setSpeed(speedValue);
  motor3.setSpeed(speedValue);
}

void setRightSpeed(int speedValue) {
  speedValue = constrain(speedValue, 0, 255);

  motor2.setSpeed(speedValue);
  motor4.setSpeed(speedValue);
}

void setBothSpeed(int leftSpeed, int rightSpeed) {
  setLeftSpeed(leftSpeed);
  setRightSpeed(rightSpeed);
}

void leftForward() {
  motor1.run(FORWARD);
  motor3.run(FORWARD);
}

void leftBackward() {
  motor1.run(BACKWARD);
  motor3.run(BACKWARD);
}

void rightForward() {
  motor2.run(FORWARD);
  motor4.run(FORWARD);
}

void rightBackward() {
  motor2.run(BACKWARD);
  motor4.run(BACKWARD);
}

void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// =====================================================
// 速度处理
// =====================================================

int getHoldSpeed(int requestedSpeed) {
  requestedSpeed = constrain(requestedSpeed, MIN_SPEED, MAX_SPEED);

  if (requestedSpeed == 0) {
    return 0;
  }

  if (requestedSpeed < MIN_HOLD_SPEED) {
    return MIN_HOLD_SPEED;
  }

  return requestedSpeed;
}

// =====================================================
// 启动增压
// =====================================================

void startWithBoost(
  int leftTargetSpeed,
  int rightTargetSpeed,
  bool leftDirectionForward,
  bool rightDirectionForward
) {
  leftTargetSpeed = getHoldSpeed(leftTargetSpeed);
  rightTargetSpeed = getHoldSpeed(rightTargetSpeed);

  // 先停车，避免直接反转产生巨大冲击
  stopAll();
  delay(50);

  // 设置方向
  if (leftDirectionForward) {
    leftForward();
  } else {
    leftBackward();
  }

  if (rightDirectionForward) {
    rightForward();
  } else {
    rightBackward();
  }

  // 启动瞬间使用高 PWM
  int leftBoost = leftTargetSpeed > 0 ? START_BOOST_SPEED : 0;
  int rightBoost = rightTargetSpeed > 0 ? START_BOOST_SPEED : 0;

  setBothSpeed(leftBoost, rightBoost);
  delay(START_BOOST_TIME);

  // 启动后降到目标巡航速度
  setBothSpeed(leftTargetSpeed, rightTargetSpeed);
}

// =====================================================
// 各种运动动作
// =====================================================

void moveForward(int speedValue) {
  speedValue = getHoldSpeed(speedValue);

  startWithBoost(
    speedValue,
    speedValue,
    true,
    true
  );
}

void moveBackward(int speedValue) {
  speedValue = getHoldSpeed(speedValue);

  startWithBoost(
    speedValue,
    speedValue,
    false,
    false
  );
}

void turnLeft(int speedValue) {
  speedValue = getHoldSpeed(speedValue);

  int leftSpeed = (int)(speedValue * TURN_RATIO);
  leftSpeed = getHoldSpeed(leftSpeed);

  startWithBoost(
    leftSpeed,
    speedValue,
    true,
    true
  );
}

void turnRight(int speedValue) {
  speedValue = getHoldSpeed(speedValue);

  int rightSpeed = (int)(speedValue * TURN_RATIO);
  rightSpeed = getHoldSpeed(rightSpeed);

  startWithBoost(
    speedValue,
    rightSpeed,
    true,
    true
  );
}

void pivotLeft(int speedValue) {
  speedValue = getHoldSpeed(speedValue);

  startWithBoost(
    speedValue,
    speedValue,
    false,
    true
  );
}

void pivotRight(int speedValue) {
  speedValue = getHoldSpeed(speedValue);

  startWithBoost(
    speedValue,
    speedValue,
    true,
    false
  );
}

void backLeft(int speedValue) {
  speedValue = getHoldSpeed(speedValue);

  int leftSpeed = (int)(speedValue * TURN_RATIO);
  leftSpeed = getHoldSpeed(leftSpeed);

  startWithBoost(
    leftSpeed,
    speedValue,
    false,
    false
  );
}

void backRight(int speedValue) {
  speedValue = getHoldSpeed(speedValue);

  int rightSpeed = (int)(speedValue * TURN_RATIO);
  rightSpeed = getHoldSpeed(rightSpeed);

  startWithBoost(
    speedValue,
    rightSpeed,
    false,
    false
  );
}

// =====================================================
// 执行串口命令
// =====================================================

void executeCommand(char action, int speedValue) {
  action = toupper(action);
  speedValue = constrain(speedValue, MIN_SPEED, MAX_SPEED);

  lastCommandTime = millis();
  timeoutStopped = false;

  // 重复收到相同命令时，只刷新超时时间
  // 避免每 200 ms 都进行一次启动冲击
  if (
    action == currentAction &&
    speedValue == currentSpeed &&
    action != 'S'
  ) {
    return;
  }

  switch (action) {
    case 'F':
      moveForward(speedValue);
      break;

    case 'B':
      moveBackward(speedValue);
      break;

    case 'L':
      turnLeft(speedValue);
      break;

    case 'R':
      turnRight(speedValue);
      break;

    case 'Q':
      pivotLeft(speedValue);
      break;

    case 'E':
      pivotRight(speedValue);
      break;

    case 'Z':
      backLeft(speedValue);
      break;

    case 'C':
      backRight(speedValue);
      break;

    case 'S':
      stopAll();
      speedValue = 0;
      break;

    default:
      stopAll();
      currentAction = 'S';
      currentSpeed = 0;

      Serial.println("ERR:UNKNOWN_COMMAND");
      return;
  }

  currentAction = action;
  currentSpeed = speedValue;

  Serial.print("OK:");
  Serial.print(action);
  Serial.print(",");
  Serial.println(speedValue);
}

// =====================================================
// 解析命令
//
// 命令格式：
// F,180
// B,180
// S,0
// =====================================================

void parseCommand(char *command) {
  if (command[0] == '\0') {
    return;
  }

  char action = command[0];
  int speedValue = 180;

  char *commaPosition = strchr(command, ',');

  if (commaPosition != NULL) {
    speedValue = atoi(commaPosition + 1);
  }

  executeCommand(action, speedValue);
}

// =====================================================
// 初始化
// =====================================================

void setup() {
  Serial.begin(9600);

  stopAll();

  currentAction = 'S';
  currentSpeed = 0;

  lastCommandTime = millis();

  Serial.println("READY");
}

// =====================================================
// 主循环
// =====================================================

void loop() {
  // 接收串口命令
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();

    if (receivedChar == '\n') {
      commandBuffer[commandLength] = '\0';

      if (commandLength > 0) {
        parseCommand(commandBuffer);
      }

      commandLength = 0;
    }
    else if (receivedChar != '\r') {
      if (commandLength < sizeof(commandBuffer) - 1) {
        commandBuffer[commandLength] = receivedChar;
        commandLength++;
      }
      else {
        commandLength = 0;

        stopAll();

        currentAction = 'S';
        currentSpeed = 0;

        Serial.println("ERR:COMMAND_TOO_LONG");
      }
    }
  }

  // 通信中断自动停车
  if (
    !timeoutStopped &&
    millis() - lastCommandTime > COMMAND_TIMEOUT
  ) {
    stopAll();

    currentAction = 'S';
    currentSpeed = 0;
    timeoutStopped = true;

    Serial.println("TIMEOUT:STOP");
  }
}