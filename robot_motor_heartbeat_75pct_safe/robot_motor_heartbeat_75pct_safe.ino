#include <AFMotor.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// =====================================================
// 电机映射
// M1 = 右前
// M2 = 左前
// M3 = 左后
// M4 = 右后
//
// 左侧：M2 + M3
// 右侧：M1 + M4
// =====================================================

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// =====================================================
// 安全参数
// =====================================================

// AFMotor 的 PWM 范围为 0~255。
// 191 / 255 = 74.9%，因此任何情况下都不会超过 75% 占空比。
const int PWM_MAX_SAFE = 191;

// 启动瞬间使用的 PWM，同样严格限制在 75% 以下。
const int START_BOOST_PWM = 191;

// 启动增压时间。
// 电池问题已经修复，因此先使用较短的 180 ms，减少驱动板发热。
const unsigned long START_BOOST_MS = 180;

// 正反转切换前先释放电机，降低瞬间冲击。
const unsigned long REVERSE_PAUSE_MS = 80;

// 转弯时内侧轮速度比例。
const float TURN_RATIO = 0.70;

// 超过 1 秒没有收到树莓派心跳，自动停车。
const unsigned long CMD_TIMEOUT_MS = 1000;

// =====================================================
// 串口与运行状态
// =====================================================

char cmdBuffer[32];
uint8_t cmdLength = 0;

unsigned long lastCmdTime = 0;
bool timeoutStopped = true;

char currentAction = 'S';
int currentPwm = 0;

// =====================================================
// 安全 PWM 限制
// =====================================================

int safePwm(int requestedPwm) {
  return constrain(requestedPwm, 0, PWM_MAX_SAFE);
}

void setLeftSpeed(int pwm) {
  pwm = safePwm(pwm);

  motor2.setSpeed(pwm);  // 左前
  motor3.setSpeed(pwm);  // 左后
}

void setRightSpeed(int pwm) {
  pwm = safePwm(pwm);

  motor1.setSpeed(pwm);  // 右前
  motor4.setSpeed(pwm);  // 右后
}

void setBothSpeed(int leftPwm, int rightPwm) {
  setLeftSpeed(leftPwm);
  setRightSpeed(rightPwm);
}

// =====================================================
// 电机方向
// =====================================================

void leftForward() {
  motor2.run(FORWARD);
  motor3.run(FORWARD);
}

void leftBackward() {
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
}

void rightForward() {
  motor1.run(FORWARD);
  motor4.run(FORWARD);
}

void rightBackward() {
  motor1.run(BACKWARD);
  motor4.run(BACKWARD);
}

void releaseAllMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void stopAll() {
  releaseAllMotors();
  currentAction = 'S';
  currentPwm = 0;
}

// =====================================================
// 根据动作获取左右方向
//
//  1 = FORWARD
// -1 = BACKWARD
//  0 = RELEASE
// =====================================================

void getDirections(
  char action,
  int &leftDirection,
  int &rightDirection
) {
  switch (action) {
    case 'F':
    case 'L':
    case 'R':
      leftDirection = 1;
      rightDirection = 1;
      break;

    case 'B':
    case 'Z':
    case 'C':
      leftDirection = -1;
      rightDirection = -1;
      break;

    case 'Q':
      leftDirection = -1;
      rightDirection = 1;
      break;

    case 'E':
      leftDirection = 1;
      rightDirection = -1;
      break;

    default:
      leftDirection = 0;
      rightDirection = 0;
      break;
  }
}

void applyDirections(int leftDirection, int rightDirection) {
  if (leftDirection > 0) {
    leftForward();
  } else if (leftDirection < 0) {
    leftBackward();
  }

  if (rightDirection > 0) {
    rightForward();
  } else if (rightDirection < 0) {
    rightBackward();
  }
}

// =====================================================
// 根据动作计算左右 PWM
// =====================================================

void getTargetSpeeds(
  char action,
  int requestedPwm,
  int &leftPwm,
  int &rightPwm
) {
  int outerPwm = safePwm(requestedPwm);
  int innerPwm = safePwm((int)(outerPwm * TURN_RATIO));

  switch (action) {
    case 'L':
    case 'Z':
      leftPwm = innerPwm;
      rightPwm = outerPwm;
      break;

    case 'R':
    case 'C':
      leftPwm = outerPwm;
      rightPwm = innerPwm;
      break;

    case 'F':
    case 'B':
    case 'Q':
    case 'E':
      leftPwm = outerPwm;
      rightPwm = outerPwm;
      break;

    default:
      leftPwm = 0;
      rightPwm = 0;
      break;
  }
}

// =====================================================
// 执行动作
// 启动或反转时：191 PWM 持续 180 ms，然后降到目标 PWM。
// 同方向切换动作时不重复启动增压。
// =====================================================

void applyMotion(char action, int requestedPwm) {
  int newLeftDirection = 0;
  int newRightDirection = 0;
  int oldLeftDirection = 0;
  int oldRightDirection = 0;

  int leftTargetPwm = 0;
  int rightTargetPwm = 0;

  getDirections(
    action,
    newLeftDirection,
    newRightDirection
  );

  getDirections(
    currentAction,
    oldLeftDirection,
    oldRightDirection
  );

  getTargetSpeeds(
    action,
    requestedPwm,
    leftTargetPwm,
    rightTargetPwm
  );

  bool needsBoost =
    currentAction == 'S' ||
    newLeftDirection != oldLeftDirection ||
    newRightDirection != oldRightDirection;

  if (needsBoost) {
    releaseAllMotors();
    delay(REVERSE_PAUSE_MS);

    applyDirections(
      newLeftDirection,
      newRightDirection
    );

    // 启动增压也不会超过 191，即 75%。
    setBothSpeed(
      newLeftDirection == 0 ? 0 : START_BOOST_PWM,
      newRightDirection == 0 ? 0 : START_BOOST_PWM
    );

    delay(START_BOOST_MS);
  } else {
    applyDirections(
      newLeftDirection,
      newRightDirection
    );
  }

  setBothSpeed(
    leftTargetPwm,
    rightTargetPwm
  );
}

// =====================================================
// 串口命令
//
// F,180  前进
// B,180  后退
// L,180  前进左转
// R,180  前进右转
// Q,180  原地左转
// E,180  原地右转
// Z,180  后退左转
// C,180  后退右转
// S,0    停车
//
// 即使收到 F,255，也会自动限制为 F,191。
// =====================================================

bool isValidAction(char action) {
  return
    action == 'F' ||
    action == 'B' ||
    action == 'L' ||
    action == 'R' ||
    action == 'Q' ||
    action == 'E' ||
    action == 'Z' ||
    action == 'C' ||
    action == 'S';
}

void executeCommand(char action, int requestedPwm) {
  action = toupper(action);

  if (!isValidAction(action)) {
    stopAll();
    timeoutStopped = true;
    Serial.println("ERR:UNKNOWN_CMD");
    return;
  }

  // 停车命令立即执行。
  if (action == 'S') {
    bool wasMoving = currentAction != 'S';

    stopAll();
    timeoutStopped = true;

    if (wasMoving) {
      Serial.println("OK:S,0");
    }
    return;
  }

  int actualPwm = safePwm(requestedPwm);

  lastCmdTime = millis();
  timeoutStopped = false;

  // 树莓派每 0.2 秒重发同一命令。
  // 同一命令只刷新看门狗，不重复启动增压。
  if (
    action == currentAction &&
    actualPwm == currentPwm
  ) {
    return;
  }

  applyMotion(action, actualPwm);

  currentAction = action;
  currentPwm = actualPwm;

  // 返回实际采用的安全 PWM。
  Serial.print("OK:");
  Serial.print(action);
  Serial.print(",");
  Serial.println(actualPwm);
}

void parseAndExecute(char *command) {
  if (command[0] == '\0') {
    return;
  }

  char action = command[0];
  int requestedPwm = 180;

  char *commaPosition = strchr(command, ',');

  if (commaPosition != NULL) {
    requestedPwm = atoi(commaPosition + 1);
  }

  executeCommand(action, requestedPwm);
}

// =====================================================
// 初始化
// =====================================================

void setup() {
  Serial.begin(9600);

  stopAll();
  timeoutStopped = true;
  lastCmdTime = millis();

  Serial.println("READY:MAX_PWM=191");
}

// =====================================================
// 主循环
// =====================================================

void loop() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();

    if (receivedChar == '\n') {
      cmdBuffer[cmdLength] = '\0';

      if (cmdLength > 0) {
        parseAndExecute(cmdBuffer);
      }

      cmdLength = 0;
    }
    else if (receivedChar != '\r') {
      if (cmdLength < sizeof(cmdBuffer) - 1) {
        cmdBuffer[cmdLength++] = receivedChar;
      }
      else {
        cmdLength = 0;
        stopAll();
        timeoutStopped = true;
        Serial.println("ERR:CMD_TOO_LONG");
      }
    }
  }

  if (
    !timeoutStopped &&
    millis() - lastCmdTime > CMD_TIMEOUT_MS
  ) {
    stopAll();
    timeoutStopped = true;
    Serial.println("TIMEOUT:STOP");
  }
}
