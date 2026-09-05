#include <AFMotor.h>
#include <SoftwareSerial.h>

// Motors
AF_DCMotor motor1(1, MOTOR12_1KHZ);  // Front Left
AF_DCMotor motor2(2, MOTOR12_1KHZ);  // Front Right
AF_DCMotor motor3(3, MOTOR34_1KHZ);  // Back Left
AF_DCMotor motor4(4, MOTOR34_1KHZ);  // Back Right

const int MAX_SPEED = 150; // Max motor speed

// SoftwareSerial for Bluetooth (RX = 10, TX = 9)
SoftwareSerial BTSerial(10, 9); // RX, TX

void setup() {
  Serial.begin(9600);   // Debug serial
  BTSerial.begin(9600); // Bluetooth module

  // Initialize motors stopped
  stopMotors();
  setAllMotorsSpeed(MAX_SPEED);

  Serial.println("Bluetooth 4-motor robot ready!");
}

void loop() {
  if (BTSerial.available()) {
    char command = BTSerial.read();
    Serial.print("Received command: ");
    Serial.println(command);

    switch (command) {
      case 'F': moveForward(); break;   // Forward
      case 'B': moveBackward(); break;  // Backward
      case 'L': turnLeft(); break;      // Turn left
      case 'R': turnRight(); break;     // Turn right
      case 'S': stopMotors(); break;    // Stop all motors
      default: stopMotors(); break;     // Default: stop
    }
  }
}

// ---- Motor control functions ----
void setAllMotorsSpeed(int speedVal) {
  speedVal = constrain(speedVal, 0, MAX_SPEED);
  motor1.setSpeed(speedVal);
  motor2.setSpeed(speedVal);
  motor3.setSpeed(speedVal);
  motor4.setSpeed(speedVal);
}

void moveForward() {
  motor1.run(FORWARD); motor2.run(FORWARD);
  motor3.run(FORWARD); motor4.run(FORWARD);
}

void moveBackward() {
  motor1.run(BACKWARD); motor2.run(BACKWARD);
  motor3.run(BACKWARD); motor4.run(BACKWARD);
}

void turnLeft() {
  motor1.run(BACKWARD); motor2.run(FORWARD);
  motor3.run(BACKWARD); motor4.run(FORWARD);
}

void turnRight() {
  motor1.run(FORWARD); motor2.run(BACKWARD);
  motor3.run(FORWARD); motor4.run(BACKWARD);
}

void stopMotors() {
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
}