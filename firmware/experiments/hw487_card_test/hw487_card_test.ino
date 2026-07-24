/*
  HW-487 / KY-010 slotted photo-interrupter: card-pass test.

  Arduino Mega wiring:
    HW-487 '-' (or G)  -> GND
    HW-487 '+' (middle) -> 5V
    HW-487 'S'          -> D2

  Slide the edge of one card through the slot. Each beam block is counted once.
*/

const uint8_t HW487_SIGNAL_PIN = 22;
const uint8_t SENSOR_BLOCKED_LEVEL = HIGH; // Change to LOW only if your board is inverted.
const unsigned long DEBOUNCE_MS = 25UL;

bool lastRawBlocked = false;
bool stableBlocked = false;
unsigned long rawChangedAtMs = 0;
unsigned long cardCount = 0;

bool beamIsBlocked() {
  return digitalRead(HW487_SIGNAL_PIN) == SENSOR_BLOCKED_LEVEL;
}

void printState() {
  Serial.print(F("HW487,BEAM_"));
  Serial.println(stableBlocked ? F("BLOCKED") : F("CLEAR"));
}

void setup() {
  Serial.begin(9600);
  pinMode(HW487_SIGNAL_PIN, INPUT);

  lastRawBlocked = beamIsBlocked();
  stableBlocked = lastRawBlocked;
  rawChangedAtMs = millis();

  Serial.println(F("READY: HW-487 card-pass test"));
  Serial.println(F("Pass one card through the sensor slot."));
  printState();
}

void loop() {
  const bool rawBlocked = beamIsBlocked();
  const unsigned long now = millis();

  if (rawBlocked != lastRawBlocked) {
    lastRawBlocked = rawBlocked;
    rawChangedAtMs = now;
  }

  // Accept a new state only after it remains unchanged for DEBOUNCE_MS.
  if (rawBlocked != stableBlocked && now - rawChangedAtMs >= DEBOUNCE_MS) {
    stableBlocked = rawBlocked;
    printState();

    if (stableBlocked) {
      ++cardCount;
      Serial.print(F("CARD_DETECTED,COUNT="));
      Serial.println(cardCount);
    }
  }
}
