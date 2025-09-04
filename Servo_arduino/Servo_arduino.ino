// Use the built-in Servo library to generate control pulses.
#include <Servo.h>

// Create a Servo object to control one servo motor.
Servo sg90;

void setup() {
  // Attach the servo signal to digital pin 4 (change if you have different pin pwm).
  sg90.attach(4);

  // Open the USB serial connection at 9600 baud.
  // This must match the baud rate used on the Python side.
  Serial.begin(9600);

  // Move the servo to a neutral/mid position at startup (90 degrees).
  sg90.write(90);
}

void loop() {
  // Only proceed if there is at least one byte waiting on the serial port.
  if (Serial.available()) {

    // Read characters from serial until a newline ('\n') is found.
    // Python sends lines like "137\n".
    String line = Serial.readStringUntil('\n');

    // Remove any leading/trailing whitespace (e.g., '\r' from Windows line endings).
    line.trim();

    // Ignore empty lines to avoid sending bogus commands to the servo.
    if (line.length() > 0) {

      // Convert the ASCII text (e.g., "137") to an integer (137).
      int angle = line.toInt();

      // Ensure the value is safely within the servo’s valid degree range.
      angle = constrain(angle, 0, 180);

      // Command the servo to move to that angle (in degrees).
      sg90.write(angle);
    }
  }
}
