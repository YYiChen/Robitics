/*
  Arduino Mega 2560 - HC-SR04 left-front channel isolation test.

  Wiring (connect ONLY this sensor for the test):
    HC-SR04 VCC  -> Mega 5V
    HC-SR04 GND  -> Mega GND
    HC-SR04 TRIG -> Mega D23
    HC-SR04 ECHO -> Mega D24

  Serial Monitor: 9600 baud

  This sketch never accesses pins 25-28.  A repeated timeout here means the
  cause is not the three-sensor polling logic; inspect this sensor, its power,
  and the four wires connected to D23/D24.
*/

constexpr uint8_t TRIG_PIN = 25;
constexpr uint8_t ECHO_PIN = 24;
constexpr unsigned long ECHO_TIMEOUT_US = 30000UL;
constexpr unsigned long SAMPLE_INTERVAL_MS = 200UL;

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long echoUs = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (echoUs == 0) {
    return -1.0F;
  }

  const float distanceCm = echoUs * 0.0343F / 2.0F;
  return (distanceCm >= 2.0F && distanceCm <= 400.0F) ? distanceCm : -1.0F;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println(F("HC-SR04 D23/D24 isolation test ready"));
}

void loop() {
  const float distanceCm = readDistanceCm();
  if (distanceCm < 0.0F) {
    Serial.println(F("timeout (no ECHO pulse on D24)"));
  } else {
    Serial.print(distanceCm, 1);
    Serial.println(F(" cm"));
  }
  delay(SAMPLE_INTERVAL_MS);
}
