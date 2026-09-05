#include <AFMotor.h>
#include <SoftwareSerial.h>

// Motors
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Bluetooth setup (HC-05)
SoftwareSerial BTSerial(9, 10); // RX, TX

// Speed modes
int speedMode = 2;                   // Default medium
int speedLevels[] = {100, 150, 200}; // 1=low, 2=medium, 3=high
int currentSpeed = 0;                // Current motor speed
int targetSpeed = 150;               // Target speed for smooth acceleration
int accelStep = 5;                   // Speed increment per loop iteration

// Direction flags
bool forwardFlag = false;
bool backwardFlag = false;
bool leftFlag = false;
bool rightFlag = false;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600); // Must match HC-05 baud
  Serial.println("Bluetooth Serial Robot Ready (No Delay)");
  stopMotors();
}

void loop() {
  // 1️⃣ Handle incoming Bluetooth commands immediately
  while (BTSerial.available()) {
    char cmd = BTSerial.read();

    switch(cmd) {
      // Direction commands
      case 'F': forwardFlag = true; break;
      case 'B': backwardFlag = true; break;
      case 'L': leftFlag = true; break;
      case 'R': rightFlag = true; break;
      case 'S': forwardFlag = backwardFlag = leftFlag = rightFlag = false; break;

      // Speed modes
      case '1': speedMode = 1; break;
      case '2': speedMode = 2; break;
      case '3': speedMode = 3; break;
    }

    // Update target speed
    targetSpeed = speedLevels[speedMode - 1];

    Serial.print("Cmd: "); Serial.print(cmd);
    Serial.print(" | SpeedMode: "); Serial.println(speedMode);
  }

  // 2️⃣ Smooth acceleration without delay
  if (currentSpeed < targetSpeed) currentSpeed += accelStep;
  if (currentSpeed > targetSpeed) currentSpeed -= accelStep;

  // 3️⃣ Apply motor movement based on current flags
  applyMovement(currentSpeed);

  // No delay → loop runs as fast as possible
}

// ===== Motor control function =====
void applyMovement(int speedVal) {
  if (!forwardFlag && !backwardFlag && !leftFlag && !rightFlag) {
    stopMotors();
    return;
  }

  // Determine motor directions for all combinations
  if (forwardFlag && leftFlag) {       // Forward-left
    motor1.run(FORWARD); motor2.run(FORWARD);
    motor3.run(FORWARD); motor4.run(RELEASE);
  }
  else if (forwardFlag && rightFlag) { // Forward-right
    motor1.run(FORWARD); motor2.run(FORWARD);
    motor3.run(RELEASE); motor4.run(FORWARD);
  }
  else if (backwardFlag && leftFlag) { // Backward-left
    motor1.run(BACKWARD); motor2.run(BACKWARD);
    motor3.run(BACKWARD); motor4.run(RELEASE);
  }
  else if (backwardFlag && rightFlag) { // Backward-right
    motor1.run(BACKWARD); motor2.run(BACKWARD);
    motor3.run(RELEASE); motor4.run(BACKWARD);
  }
  else if (forwardFlag) {  // Forward
    motor1.run(FORWARD); motor2.run(FORWARD);
    motor3.run(FORWARD); motor4.run(FORWARD);
  }
  else if (backwardFlag) { // Backward
    motor1.run(BACKWARD); motor2.run(BACKWARD);
    motor3.run(BACKWARD); motor4.run(BACKWARD);
  }
  else if (leftFlag) {     // Turn Left
    motor1.run(BACKWARD); motor2.run(FORWARD);
    motor3.run(BACKWARD); motor4.run(FORWARD);
  }
  else if (rightFlag) {    // Turn Right
    motor1.run(FORWARD); motor2.run(BACKWARD);
    motor3.run(FORWARD); motor4.run(BACKWARD);
  }

  // Set motor speed
  motor1.setSpeed(speedVal);
  motor2.setSpeed(speedVal);
  motor3.setSpeed(speedVal);
  motor4.setSpeed(speedVal);
}

// Stop all motors
void stopMotors() {
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
  currentSpeed = 0;
  targetSpeed = speedLevels[speedMode - 1];
}