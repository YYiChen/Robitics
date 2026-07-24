
int ledPin = 9;    // LED connected to digital pin 9
int brightness = 0; // Initial brightness
int fadeAmount = 15; // Amount to change the brightness

void setup() {
  pinMode(ledPin, OUTPUT); // Set ledPin as an output
}

void loop() {
  analogWrite(ledPin, brightness); // Set the brightness using PWM

  brightness = brightness + fadeAmount; // Increase brightness

  // Reverse the direction of the brightness change
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

  delay(30); // Delay for 30 milliseconds to control the speed of the change
}
