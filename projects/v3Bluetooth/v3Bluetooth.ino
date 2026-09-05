#include <AFMotor.h>
#include <SoftwareSerial.h>

// Motors
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Bluetooth
SoftwareSerial BTSerial(2,3);

// Speed modes
int speedMode = 2;
int speedLevels[] = {60, 150, 200};

int currentSpeed = 0;
int targetSpeed = 60;

int currentBackwardSpeed = 0;
int targetBackwardSpeed = 60;

int accelStep = 5;

// Direction flags
bool forwardFlag = false;
bool backwardFlag = false;
bool leftFlag = false;
bool rightFlag = false;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  stopMotors();
}

void loop() {
  while (BTSerial.available()) {
    char cmd = BTSerial.read();

    switch (cmd) {
      case 'F': forwardFlag = true; backwardFlag = false; break;
      case 'B': backwardFlag = true; forwardFlag = false; break;
      case 'L': leftFlag = true; rightFlag = false; break;
      case 'R': rightFlag = true; leftFlag = false; break;
      case 'S':
        forwardFlag = backwardFlag = leftFlag = rightFlag = false;
        break;

      case '1': speedMode = 1; break;
      case '2': speedMode = 2; break;
      case '3': speedMode = 3; break;
    }

    targetSpeed = speedLevels[speedMode - 1];

    // backward speed modes
    if (speedMode == 1) targetBackwardSpeed = 60;
    if (speedMode == 2) targetBackwardSpeed = 100;
    if (speedMode == 3) targetBackwardSpeed = 150;
  }

  // Smooth acceleration
  if (currentSpeed < targetSpeed) currentSpeed += accelStep;
  if (currentSpeed > targetSpeed) currentSpeed -= accelStep;

  if (currentBackwardSpeed < targetBackwardSpeed) currentBackwardSpeed += accelStep;
  if (currentBackwardSpeed > targetBackwardSpeed) currentBackwardSpeed -= accelStep;

  applyMovement();
}

// ================= MOTOR LOGIC =================
void applyMovement() {
  float turnFactor = 0.7; // diagonal bias

  // STOP
  if (!forwardFlag && !backwardFlag && !leftFlag && !rightFlag) {
    stopMotors();
    return;
  }

  // -------- FORWARD --------
  if (forwardFlag) {
    motor1.run(FORWARD); motor2.run(FORWARD);
    motor3.run(FORWARD); motor4.run(FORWARD);

    int leftSpeed = currentSpeed;
    int rightSpeed = currentSpeed;

    if (leftFlag)  leftSpeed *= turnFactor;
    if (rightFlag) rightSpeed *= turnFactor;

    motor1.setSpeed(leftSpeed);
    motor3.setSpeed(leftSpeed);
    motor2.setSpeed(rightSpeed);
    motor4.setSpeed(rightSpeed);
    return;
  }

  // -------- BACKWARD --------
  if (backwardFlag) {
    motor1.run(BACKWARD); motor2.run(BACKWARD);
    motor3.run(BACKWARD); motor4.run(BACKWARD);

    int leftSpeed = currentBackwardSpeed;
    int rightSpeed = currentBackwardSpeed;

    if (leftFlag)  leftSpeed *= turnFactor;
    if (rightFlag) rightSpeed *= turnFactor;

    motor1.setSpeed(leftSpeed);
    motor3.setSpeed(leftSpeed);
    motor2.setSpeed(rightSpeed);
    motor4.setSpeed(rightSpeed);
    return;
  }

  // -------- ROTATION ONLY --------
  if (leftFlag) {
    motor1.run(BACKWARD); motor3.run(BACKWARD);
    motor2.run(FORWARD);  motor4.run(FORWARD);
    motor1.setSpeed(currentSpeed);
    motor2.setSpeed(currentSpeed);
    motor3.setSpeed(currentSpeed);
    motor4.setSpeed(currentSpeed);
    return;
  }

  if (rightFlag) {
    motor1.run(FORWARD);  motor3.run(FORWARD);
    motor2.run(BACKWARD); motor4.run(BACKWARD);
    motor1.setSpeed(currentSpeed);
    motor2.setSpeed(currentSpeed);
    motor3.setSpeed(currentSpeed);
    motor4.setSpeed(currentSpeed);
    return;
  }
}

void stopMotors() {
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
  currentSpeed = 0;
  currentBackwardSpeed = 0;
}