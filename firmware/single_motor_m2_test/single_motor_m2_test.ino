/*
  M2 single DC-motor test for an Arduino Motor Shield V1 / L293D clone.

  Hardware:
    - Motor shield installed on the Arduino
    - One DC motor connected to the shield's M2 terminal
    - Motor power connected to the shield's external motor-power input
      (do not power a motor from the Arduino USB 5 V pin)

  The APDS-9960 compatible module is driven directly over I2C.
  No additional Arduino library is required.
*/
#include <AFMotor.h>
#include <Wire.h>
#include <Servo.h>

// ==================== Easy-to-change parameters ====================
const uint8_t MOTOR_SPEED = 180;          // 0..255; reduced to limit multi-card feeding
const unsigned long RUN_TIME_MS = 100UL;  // motor ON time each cycle, in ms
const unsigned long PAUSE_TIME_MS = 1000UL; // motor OFF interval, in ms
const bool AUTO_REPEAT = true;            // true = repeat ON then OFF forever
const bool MOTOR_ENABLED_ON_BOOT = true; // false = motor stays OFF after upload
const unsigned long SENSOR_REPORT_INTERVAL_MS = 100UL; // serial output period
const bool CARD_DETECTION_MODE = false;    // true = stable IR card-presence sensing
// Previous testing established that BACKWARD is the motor's physical forward
// direction with the current M2 wiring. Change this one value if wiring changes.
const uint8_t MOTOR_FORWARD_DIRECTION = FORWARD;
const unsigned long REVERSE_PAUSE_MS = 80UL;
const uint8_t SERVO_SIGNAL_PIN = 23;      // Servo signal wire; D22 remains HW-487
const uint8_t SERVO_START_ANGLE = 0;
// ================== HW-487 card-feed closed-loop parameters =================
const uint8_t HW487_SIGNAL_PIN = 22;       // HW-487 S pin
const uint8_t HW487_BLOCKED_LEVEL = HIGH;  // HIGH when a card blocks the slot
const unsigned long HW487_DEBOUNCE_MS = 20UL;
const uint8_t PASSING_SPEED = 150;         // reduced, but above the motor's stall range
const unsigned long FEED_PULSE_GAP_MS = 80UL;
const unsigned long NEXT_CARD_DELAY_MS = 1000UL;
// ============================================================================
// ====================================================================

AF_DCMotor motorM2(2);
Servo cardServo;

// APDS-9960 I2C registers. Mega D20=SDA, D21=SCL; sensor address is 0x39.
const uint8_t APDS9960_ADDRESS = 0x39;
const uint8_t APDS_ENABLE = 0x80;
const uint8_t APDS_ATIME = 0x81;
const uint8_t APDS_WTIME = 0x83;
const uint8_t APDS_PPULSE = 0x8E;
const uint8_t APDS_CONTROL = 0x8F;
const uint8_t APDS_CONFIG2 = 0x90;
const uint8_t APDS_ID = 0x92;
const uint8_t APDS_CDATAL = 0x94;
const uint8_t APDS_RDATAL = 0x96;
const uint8_t APDS_GDATAL = 0x98;
const uint8_t APDS_BDATAL = 0x9A;
const uint8_t APDS_PDATA = 0x9C;
const uint8_t APDS_GCONF1 = 0xA2;
const uint8_t APDS_GCONF2 = 0xA3;
const uint8_t APDS_GOFFSET_U = 0xA4;
const uint8_t APDS_GOFFSET_D = 0xA5;
const uint8_t APDS_GOFFSET_L = 0xA7;
const uint8_t APDS_GOFFSET_R = 0xA9;
const uint8_t APDS_GPULSE = 0xA6;
const uint8_t APDS_GCONF3 = 0xAA;
const uint8_t APDS_GCONF4 = 0xAB;
const uint8_t APDS_GFLVL = 0xAE;
const uint8_t APDS_GSTATUS = 0xAF;
const uint8_t APDS_GFIFO_U = 0xFC;

bool motorRunning = false;
uint8_t activeMotorDirection = RELEASE;
unsigned long motorStartMs = 0;
unsigned long activeRunTimeMs = RUN_TIME_MS;
bool automaticCycleActive = false;
bool waitingForNextCycle = false;
unsigned long pauseStartMs = 0;
bool apdsPresent = false;
unsigned long lastSensorReportMs = 0;
unsigned long lastGesturePollMs = 0;
bool gestureCapturing = false;
int16_t gestureFirstU = 0, gestureFirstD = 0, gestureFirstL = 0, gestureFirstR = 0;
int16_t gestureLastU = 0, gestureLastD = 0, gestureLastL = 0, gestureLastR = 0;

enum CardFeedState {
  CARD_FEED_DISABLED,
  CARD_FEEDING,
  CARD_WAIT_SENSOR,
  CARD_WAIT_PASS,
  CARD_WAIT_NEXT
};

CardFeedState cardFeedState = CARD_FEED_DISABLED;
bool hw487LastRawBlocked = false;
bool hw487StableBlocked = false;
unsigned long hw487RawChangedAtMs = 0;
unsigned long nextCardStartAtMs = 0;
unsigned long nextFeedPulseAtMs = 0;
unsigned long cardsPassed = 0;
bool cardSeenInSensorSlot = false;
uint8_t servoAngle = SERVO_START_ANGLE;

void stopMotor() {
  motorM2.setSpeed(0);
  motorM2.run(RELEASE);  // coast/release; motor output is off
  motorRunning = false;
  activeMotorDirection = RELEASE;
  Serial.println(F("STOPPED"));
}

void startMotor(uint8_t direction, uint8_t speed, unsigned long durationMs) {
  // Release briefly before reversing to reduce electrical and mechanical shock.
  if (motorRunning && direction != activeMotorDirection) {
    motorM2.setSpeed(0);
    motorM2.run(RELEASE);
    delay(REVERSE_PAUSE_MS);
  }
  motorM2.setSpeed(speed);
  motorM2.run(direction);
  motorStartMs = millis();
  activeRunTimeMs = durationMs;
  activeMotorDirection = direction;
  motorRunning = true;

  Serial.print(F("RUNNING,M2,DIRECTION="));
  Serial.print(direction == MOTOR_FORWARD_DIRECTION ? F("FORWARD") : F("REVERSE"));
  Serial.print(F(",SPEED="));
  Serial.print(speed);
  Serial.print(F(",TIME_MS="));
  Serial.println(durationMs);
}

bool hw487BeamBlocked() {
  return digitalRead(HW487_SIGNAL_PIN) == HW487_BLOCKED_LEVEL;
}

void beginNextCardFeed() {
  if (hw487StableBlocked) {
    // A card was already in the slot: keep it moving slowly until it exits.
    startMotor(MOTOR_FORWARD_DIRECTION, PASSING_SPEED, 0);
    cardSeenInSensorSlot = true;
    cardFeedState = CARD_WAIT_PASS;
    Serial.println(F("CARD_FEED,CONTINUING_CARD_IN_SLOT"));
    return;
  }
  // One short controlled push, rather than continuously pulling the whole deck.
  startMotor(MOTOR_FORWARD_DIRECTION, MOTOR_SPEED, RUN_TIME_MS);
  cardSeenInSensorSlot = false;
  cardFeedState = CARD_FEEDING;
  Serial.println(F("CARD_FEED,SENDING"));
}

void disableCardFeed() {
  cardFeedState = CARD_FEED_DISABLED;
  stopMotor();
  Serial.println(F("CARD_FEED,DISABLED"));
}

void updateCardFeedLoop() {
  const bool rawBlocked = hw487BeamBlocked();
  const unsigned long now = millis();

  if (rawBlocked != hw487LastRawBlocked) {
    hw487LastRawBlocked = rawBlocked;
    hw487RawChangedAtMs = now;
  }

  // Debounce the HW-487 optical output so one card yields one event only.
  if (rawBlocked != hw487StableBlocked && now - hw487RawChangedAtMs >= HW487_DEBOUNCE_MS) {
    hw487StableBlocked = rawBlocked;
    Serial.println(hw487StableBlocked ? F("HW487,BEAM_BLOCKED") : F("HW487,BEAM_CLEAR"));

    // BLOCKED means a card has entered the slot.  It may be motor-driven or
    // still coasting after a stop; in both cases wait for its trailing edge.
    if (hw487StableBlocked && cardFeedState == CARD_FEEDING) {
      cardSeenInSensorSlot = true;
      Serial.println(F("CARD_FEED,CARD_IN_SLOT"));
      // Keep moving, but slowly, until the trailing edge clears the slot.
      // Stopping here can leave the card physically stuck in the sensor.
      motorM2.setSpeed(PASSING_SPEED);
      cardFeedState = CARD_WAIT_PASS;
      Serial.print(F("CARD_FEED,SLOW_PASS,PWM="));
      Serial.println(PASSING_SPEED);
      return;
    }

    if (hw487StableBlocked &&
        (cardFeedState == CARD_WAIT_SENSOR || cardFeedState == CARD_WAIT_PASS ||
         cardFeedState == CARD_WAIT_NEXT)) {
      cardSeenInSensorSlot = true;
      if (cardFeedState == CARD_WAIT_SENSOR || cardFeedState == CARD_WAIT_NEXT) {
        cardFeedState = CARD_WAIT_PASS;
      }
      if (!motorRunning) {
        startMotor(MOTOR_FORWARD_DIRECTION, PASSING_SPEED, 0);
        Serial.println(F("CARD_FEED,CONTINUING_CARD_IN_SLOT"));
      }
      Serial.println(F("CARD_FEED,CARD_IN_SLOT"));
    }

    // CLEAR after a confirmed block means the trailing edge has passed the
    // optical slot, so this card has completely gone through.
    if (!hw487StableBlocked && cardFeedState == CARD_WAIT_PASS && cardSeenInSensorSlot) {
      stopMotor();
      ++cardsPassed;
      Serial.print(F("CARD_PASSED,COUNT="));
      Serial.println(cardsPassed);
      nextCardStartAtMs = now + NEXT_CARD_DELAY_MS;
      cardFeedState = CARD_WAIT_NEXT;
      cardSeenInSensorSlot = false;
    }

  // A second card can coast into the sensor after the motor stops. Count
    // it when its trailing edge clears the slot and restart the full 1-second
    // wait from that actual pass time before driving another card.
    if (!hw487StableBlocked && cardFeedState == CARD_WAIT_NEXT && cardSeenInSensorSlot) {
      ++cardsPassed;
      Serial.print(F("CARD_COAST_PASSED,COUNT="));
      Serial.println(cardsPassed);
      nextCardStartAtMs = now + NEXT_CARD_DELAY_MS;
      cardSeenInSensorSlot = false;
    }
  }

  // Do not send another card until 1 s has elapsed AND the previous card has
  // fully cleared the sensor slot.
  if (cardFeedState == CARD_WAIT_NEXT && !hw487StableBlocked && now >= nextCardStartAtMs) {
    beginNextCardFeed();
  }

  // Keep advancing in small, low-speed steps until the next card reaches the
  // sensor. This prevents the one-pulse stall without reverting to continuous
  // high-speed pulling.
  if (cardFeedState == CARD_WAIT_SENSOR && !hw487StableBlocked &&
      now >= nextFeedPulseAtMs) {
    startMotor(MOTOR_FORWARD_DIRECTION, MOTOR_SPEED, RUN_TIME_MS);
    cardFeedState = CARD_FEEDING;
    Serial.println(F("CARD_FEED,NEXT_PULSE"));
  }
}

void printHelp() {
  Serial.println(F("Commands (send with Newline):"));
  Serial.println(F("  0                  stop card feeder immediately"));
  Serial.println(F("  1                  start closed-loop card feeding"));
  Serial.println(F("  F                  forward once (current physical forward)"));
  Serial.println(F("  B                  reverse once"));
  Serial.println(F("  F,160,3000         forward at PWM 160 for 3000 ms"));
  Serial.println(F("  B,160,3000         reverse at PWM 160 for 3000 ms"));
  Serial.println(F("  SV,0 / SV,90 / SV,180  set servo angle on D23"));
  Serial.println(F("  RUN                run once with MOTOR_SPEED/RUN_TIME_MS"));
  Serial.println(F("  RUN,160,3000       run at PWM 160 for 3000 ms"));
  Serial.println(F("  STOP               stop immediately"));
  Serial.println(F("  STATUS             show state"));
}

void printStatus() {
  Serial.print(F("STATUS,M2,"));
  Serial.println(motorRunning ? F("RUNNING") : F("STOPPED"));
}

void scanI2cBus() {
  Serial.println(F("I2C,SCAN_START"));
  bool foundAny = false;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("I2C,FOUND,0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      foundAny = true;
    }
  }
  if (!foundAny) Serial.println(F("I2C,NONE_FOUND"));
  Serial.println(F("I2C,SCAN_END"));
}

bool readI2cRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool writeI2cRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readI2cWord(uint8_t address, uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, (uint8_t)2) != 2) return false;
  const uint8_t low = Wire.read();
  const uint8_t high = Wire.read();
  value = ((uint16_t)high << 8) | low;
  return true;
}

bool initApds9960Compatible() {
  // Card mode uses reflected IR only, avoiding RGB/gesture simultaneous-mode
  // issues on some 0x9E compatible modules.
  if (!writeI2cRegister(APDS9960_ADDRESS, APDS_ENABLE, 0x00) ||
      // Conservative factory-style pulse settings are more stable on the
      // 0x9E compatible board than a continuous maximum-power IR drive.
      !writeI2cRegister(APDS9960_ADDRESS, APDS_PPULSE, 0x87) ||
      !writeI2cRegister(APDS9960_ADDRESS, APDS_CONTROL, 0x0C) ||
      !writeI2cRegister(APDS9960_ADDRESS, APDS_CONFIG2, 0x00)) {
    return false;
  }
  if (CARD_DETECTION_MODE) {
    if (!writeI2cRegister(APDS9960_ADDRESS, APDS_ENABLE, 0x05)) return false; // PON + PEN
    delay(20); // Allow the first proximity integration cycle to complete.
    return true;
  }
  return writeI2cRegister(APDS9960_ADDRESS, APDS_ATIME, 0xDB) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_WTIME, 0xFF) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GCONF1, 0x40) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GCONF2, 0x66) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GOFFSET_U, 0x00) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GOFFSET_D, 0x00) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GOFFSET_L, 0x00) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GOFFSET_R, 0x00) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GPULSE, 0xC9) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GCONF3, 0x00) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_GCONF4, 0x02) &&
         writeI2cRegister(APDS9960_ADDRESS, APDS_ENABLE, 0x4F);
}

void reportGestureFifo() {
  uint8_t gestureStatus = 0, fifoLevel = 0;
  if (!readI2cRegister(APDS9960_ADDRESS, APDS_GSTATUS, gestureStatus) ||
      !(gestureStatus & 0x01) ||
      !readI2cRegister(APDS9960_ADDRESS, APDS_GFLVL, fifoLevel) || fifoLevel == 0) {
    return;
  }

  // Read one U/D/L/R gesture sample.  Repeated samples while a hand moves
  // show the changing values in Serial Monitor and confirm gesture sensing.
  Wire.beginTransmission(APDS9960_ADDRESS);
  Wire.write(APDS_GFIFO_U);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(APDS9960_ADDRESS, (uint8_t)4) != 4) return;
  const uint8_t up = Wire.read();
  const uint8_t down = Wire.read();
  const uint8_t left = Wire.read();
  const uint8_t right = Wire.read();
  Serial.print(F("GESTURE_RAW,U=")); Serial.print(up);
  Serial.print(F(",D=")); Serial.print(down);
  Serial.print(F(",L=")); Serial.print(left);
  Serial.print(F(",R=")); Serial.println(right);
}

void reportApds9960() {
  if (!apdsPresent || millis() - lastSensorReportMs < SENSOR_REPORT_INTERVAL_MS) return;
  lastSensorReportMs = millis();

  uint8_t proximity = 0;
  if (CARD_DETECTION_MODE) {
    if (readI2cRegister(APDS9960_ADDRESS, APDS_PDATA, proximity)) {
      Serial.print(F("CARD,PROX="));
      Serial.println(proximity);
    } else {
      Serial.println(F("CARD,READ_ERROR"));
    }
    return;
  }

  uint16_t ambient = 0, red = 0, green = 0, blue = 0;
  const bool lightOk = readI2cWord(APDS9960_ADDRESS, APDS_CDATAL, ambient) &&
                       readI2cWord(APDS9960_ADDRESS, APDS_RDATAL, red) &&
                       readI2cWord(APDS9960_ADDRESS, APDS_GDATAL, green) &&
                       readI2cWord(APDS9960_ADDRESS, APDS_BDATAL, blue);
  const bool proximityOk = readI2cRegister(APDS9960_ADDRESS, APDS_PDATA, proximity);
  if (lightOk && proximityOk) {
    Serial.print(F("APDS,AMBIENT=")); Serial.print(ambient);
    Serial.print(F(",R=")); Serial.print(red);
    Serial.print(F(",G=")); Serial.print(green);
    Serial.print(F(",B=")); Serial.print(blue);
    Serial.print(F(",PROX=")); Serial.println(proximity);
  } else {
    Serial.println(F("APDS,READ_ERROR"));
  }

  reportGestureFifo();
}

void handleCommand(char *command) {
  if (strncmp(command, "SV,", 3) == 0) {
    const long angle = strtol(command + 3, nullptr, 10);
    if (angle >= 0 && angle <= 180) {
      servoAngle = (uint8_t)angle;
      cardServo.write(servoAngle);
      Serial.print(F("SERVO,ANGLE="));
      Serial.println(servoAngle);
    } else {
      Serial.println(F("ERR: servo angle must be 0..180"));
    }
    return;
  }
  if (strcmp(command, "0") == 0) {
    automaticCycleActive = false;
    waitingForNextCycle = false;
    disableCardFeed();
    Serial.println(F("MOTOR_SWITCH,OFF"));
    return;
  }
  if (strcmp(command, "1") == 0) {
    automaticCycleActive = false;
    waitingForNextCycle = false;
    beginNextCardFeed();
    Serial.println(F("MOTOR_SWITCH,ON"));
    return;
  }
  if (strcmp(command, "F") == 0 || strcmp(command, "f") == 0) {
    automaticCycleActive = false;
    waitingForNextCycle = false;
    cardFeedState = CARD_FEED_DISABLED;
    startMotor(MOTOR_FORWARD_DIRECTION, MOTOR_SPEED, RUN_TIME_MS);
    return;
  }
  if (strcmp(command, "B") == 0 || strcmp(command, "b") == 0) {
    automaticCycleActive = false;
    waitingForNextCycle = false;
    cardFeedState = CARD_FEED_DISABLED;
    startMotor(MOTOR_FORWARD_DIRECTION == FORWARD ? BACKWARD : FORWARD,
               MOTOR_SPEED, RUN_TIME_MS);
    return;
  }
  if (strcmp(command, "RUN") == 0) {
    automaticCycleActive = false;
    waitingForNextCycle = false;
    cardFeedState = CARD_FEED_DISABLED;
    startMotor(MOTOR_FORWARD_DIRECTION, MOTOR_SPEED, RUN_TIME_MS);
    return;
  }
  if (strcmp(command, "STOP") == 0) {
    automaticCycleActive = false;
    waitingForNextCycle = false;
    stopMotor();
    return;
  }
  if (strcmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }

  // Format: RUN,<speed 0..255>,<time in ms>
  if (strncmp(command, "RUN,", 4) == 0) {
    char *speedText = command + 4;
    char *comma = strchr(speedText, ',');
    if (comma != nullptr) {
      *comma = '\0';
      const long speed = strtol(speedText, nullptr, 10);
      const long duration = strtol(comma + 1, nullptr, 10);
      if (speed >= 0 && speed <= 255 && duration > 0) {
        automaticCycleActive = false;
        waitingForNextCycle = false;
        cardFeedState = CARD_FEED_DISABLED;
        startMotor(MOTOR_FORWARD_DIRECTION, (uint8_t)speed, (unsigned long)duration);
        return;
      }
    }
  }

  // Format: F,<speed 0..255>,<time in ms> or B,<speed 0..255>,<time in ms>
  if ((command[0] == 'F' || command[0] == 'f' || command[0] == 'B' || command[0] == 'b') &&
      command[1] == ',') {
    char *speedText = command + 2;
    char *comma = strchr(speedText, ',');
    if (comma != nullptr) {
      *comma = '\0';
      const long speed = strtol(speedText, nullptr, 10);
      const long duration = strtol(comma + 1, nullptr, 10);
      if (speed >= 0 && speed <= 255 && duration > 0) {
        automaticCycleActive = false;
        waitingForNextCycle = false;
        cardFeedState = CARD_FEED_DISABLED;
        const uint8_t direction = (command[0] == 'F' || command[0] == 'f')
          ? MOTOR_FORWARD_DIRECTION
          : (MOTOR_FORWARD_DIRECTION == FORWARD ? BACKWARD : FORWARD);
        startMotor(direction, (uint8_t)speed, (unsigned long)duration);
        return;
      }
    }
  }

  Serial.println(F("ERR: use F, B, F,<speed>,<ms>, B,<speed>,<ms>, 0, or STATUS"));
}

void setup() {
  Serial.begin(9600);
  stopMotor();  // Always begin in a safe stopped state.
  cardServo.attach(SERVO_SIGNAL_PIN);
  cardServo.write(servoAngle);
  pinMode(HW487_SIGNAL_PIN, INPUT);
  hw487LastRawBlocked = hw487BeamBlocked();
  hw487StableBlocked = hw487LastRawBlocked;
  hw487RawChangedAtMs = millis();
  Serial.println(F("READY: M2 single-motor test"));
  Serial.println(hw487StableBlocked ? F("HW487,BEAM_BLOCKED") : F("HW487,BEAM_CLEAR"));
  printHelp();

  // Arduino Mega: D20 = SDA and D21 = SCL. These are I2C pins, not UART.
  Wire.begin();
  // Do not let a disconnected/miswired I2C device freeze the whole sketch.
  Wire.setWireTimeout(25000UL, true);
  delay(50);  // Let the APDS-9960 finish powering up before reading its ID.
  uint8_t sensorId = 0;
  if (readI2cRegister(APDS9960_ADDRESS, APDS_ID, sensorId)) {
    Serial.print(F("APDS9960,ID=0x"));
    if (sensorId < 16) Serial.print('0');
    Serial.println(sensorId, HEX);
  } else {
    Serial.println(F("APDS9960,ID_READ_FAIL"));
  }
  apdsPresent = initApds9960Compatible();
  if (apdsPresent) {
    Serial.println(F("APDS9960,READY,ID_0x9E_COMPATIBLE"));
  } else {
    Serial.println(F("APDS9960,CONFIG_FAIL"));
    scanI2cBus();
  }

  if (MOTOR_ENABLED_ON_BOOT) {
    delay(1000);  // time to move hands/wheels clear after reset
    beginNextCardFeed();
  }
}

void loop() {
  // Stop after each ON period, then start the next cycle after PAUSE_TIME_MS.
  if (cardFeedState == CARD_FEEDING && motorRunning &&
      activeRunTimeMs > 0 && millis() - motorStartMs >= activeRunTimeMs) {
    stopMotor();
    cardFeedState = CARD_WAIT_SENSOR;
    nextFeedPulseAtMs = millis() + FEED_PULSE_GAP_MS;
    Serial.println(F("CARD_FEED,PULSE_COMPLETE"));
  } else if (cardFeedState == CARD_FEED_DISABLED && motorRunning &&
             activeRunTimeMs > 0 && millis() - motorStartMs >= activeRunTimeMs) {
    stopMotor();
    if (automaticCycleActive) {
      waitingForNextCycle = true;
      pauseStartMs = millis();
    }
  }
  if (waitingForNextCycle && millis() - pauseStartMs >= PAUSE_TIME_MS) {
    waitingForNextCycle = false;
    startMotor(MOTOR_FORWARD_DIRECTION, MOTOR_SPEED, RUN_TIME_MS);
  }

  updateCardFeedLoop();

  reportApds9960();

  static char buffer[48];
  static uint8_t length = 0;
  while (Serial.available() > 0) {
    const char received = (char)Serial.read();
    if (received == '\n' || received == '\r') {
      if (length > 0) {
        buffer[length] = '\0';
        handleCommand(buffer);
        length = 0;
      }
    } else if (length < sizeof(buffer) - 1) {
      buffer[length++] = received;
    } else {
      length = 0;
      Serial.println(F("ERR: command too long"));
    }
  }
}
