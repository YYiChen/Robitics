/*
  Arduino Mega 2560: three front ultrasonic-sensor wiring test.

  Left front : TRIG 24, ECHO 25
  Front      : TRIG 26, ECHO 27
  Right front: TRIG 28, ECHO 29

  Serial output (115200 baud):
    US,leftCm,frontCm,rightCm
  A value of -1 means the sensor timed out (no valid echo within 30 ms).

  This sketch is deliberately standalone for wiring and mounting validation.
  Do not run it together with the motor/PID sketch.  After validation, merge
  the pins and serial format into the robot firmware using a non-blocking
  polling state machine.
*/

constexpr uint8_t LEFT_TRIG_PIN = 25;
constexpr uint8_t LEFT_ECHO_PIN = 24;
constexpr uint8_t FRONT_TRIG_PIN = 26;
constexpr uint8_t FRONT_ECHO_PIN = 27;
constexpr uint8_t RIGHT_TRIG_PIN = 29;
constexpr uint8_t RIGHT_ECHO_PIN = 28;

constexpr unsigned long ECHO_TIMEOUT_US = 30000UL;
constexpr unsigned long BETWEEN_SENSORS_MS = 60UL;
constexpr unsigned long REPORT_INTERVAL_MS = 250UL;

float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long echoUs = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (echoUs == 0) {
    return -1.0F;
  }

  // Round-trip time in microseconds; sound speed is about 0.0343 cm/us.
  const float distanceCm = (echoUs * 0.0343F) / 2.0F;
  return (distanceCm >= 5.0F && distanceCm <= 400.0F) ? distanceCm : -1.0F;
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  digitalWrite(LEFT_TRIG_PIN, LOW);
  digitalWrite(FRONT_TRIG_PIN, LOW);
  digitalWrite(RIGHT_TRIG_PIN, LOW);
  Serial.println(F("US test ready: left,front,right (cm)"));
}

void loop() {
  const float leftCm = readDistanceCm(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  delay(BETWEEN_SENSORS_MS);

  const float frontCm = readDistanceCm(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  delay(BETWEEN_SENSORS_MS);

  const float rightCm = readDistanceCm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  Serial.print(F("US,"));
  Serial.print(leftCm, 1);
  Serial.print(',');
  Serial.print(frontCm, 1);
  Serial.print(',');
  Serial.println(rightCm, 1);

  delay(REPORT_INTERVAL_MS);
}
