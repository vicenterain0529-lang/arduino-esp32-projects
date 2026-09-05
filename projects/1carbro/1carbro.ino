#include <IRremote.hpp>
#include <AFMotor.h>  // L293D motor shield

#define IR_RECEIVE_PIN 9  // IR module on SER1 Y pin

// LAFVIN remote raw values
#define RAW_UP       0xB946FF
#define RAW_DOWN     0x5A1D74
#define RAW_LEFT     0xBB44FF
#define RAW_RIGHT    0xBC43FF
#define RAW_OK       0xBF40FF
#define RAW_CALIB1   0xE916FF  // Button 1 - individual motor calibration
#define RAW_CALIB2   0xE619FF  // Button 2 - all motors calibration

// Motors
AF_DCMotor motor1(1); // Front Left
AF_DCMotor motor2(2); // Front Right
AF_DCMotor motor3(3); // Back Left
AF_DCMotor motor4(4); // Back Right

const int MAX_SPEED = 150; // max speed

bool calibrationDone = false;

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("IR Robot Ready - Max speed 150");

  // Set all motors speed to MAX_SPEED at startup
  setAllMotorsSpeed(MAX_SPEED);
  stopMotors();
}

void loop() {
  if (IrReceiver.decode()) {
    uint32_t rawValue = IrReceiver.decodedIRData.decodedRawData;
    Serial.print("IR Value: 0x");
    Serial.println(rawValue, HEX);

    // Universal STOP button
    if (rawValue == RAW_OK) {
      stopMotors();
      Serial.println("STOP pressed!");
    }
    // Individual motor calibration
    else if (rawValue == RAW_CALIB1) {
      Serial.println("Individual motor calibration started!");
      calibrateIndividual();
      Serial.println("Individual motor calibration finished!");
    }
    // All motors calibration
    else if (rawValue == RAW_CALIB2) {
      Serial.println("All motors calibration started!");
      calibrateAll();
      Serial.println("All motors calibration finished!");
    }
    // Directional movement
    else if (rawValue == RAW_UP) {
      moveForward();
      Serial.println("Move: FORWARD");
    }
    else if (rawValue == RAW_DOWN) {
      moveBackward();
      Serial.println("Move: BACKWARD");
    }
    else if (rawValue == RAW_LEFT) {
      turnLeft();
      Serial.println("Move: LEFT");
    }
    else if (rawValue == RAW_RIGHT) {
      turnRight();
      Serial.println("Move: RIGHT");
    }

    IrReceiver.resume(); // ready for next IR signal
  }
}

// === Calibration Functions ===
void calibrateIndividual() {
  motor1.run(FORWARD); delay(1500); motor1.run(RELEASE);
  motor2.run(FORWARD); delay(1500); motor2.run(RELEASE);
  motor3.run(FORWARD); delay(1500); motor3.run(RELEASE);
  motor4.run(FORWARD); delay(1500); motor4.run(RELEASE);
}

void calibrateAll() {
  setAllMotorsSpeed(MAX_SPEED); // ensure all at max
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(2000);
  stopMotors();
}

// === Movement Functions ===
void moveForward() {
  setAllMotorsSpeed(MAX_SPEED);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveBackward() {
  setAllMotorsSpeed(MAX_SPEED);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turnLeft() {
  setAllMotorsSpeed(MAX_SPEED);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void turnRight() {
  setAllMotorsSpeed(MAX_SPEED);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

// Universal stop
void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Set same speed to all motors
void setAllMotorsSpeed(int speedVal) {
  motor1.setSpeed(speedVal);
  motor2.setSpeed(speedVal);
  motor3.setSpeed(speedVal);
  motor4.setSpeed(speedVal);
}