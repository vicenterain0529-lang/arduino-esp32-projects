#include <IRremote.h>
#include <AFMotor.h>  // L293D motor shield

#define IR_RECEIVE_PIN 9  // IR module connected to SER1 Y pin

// LAFVIN remote raw values
#define RAW_UP     0xB946FF00
#define RAW_DOWN   0x5A1D7462
#define RAW_LEFT   0xBB44FF00
#define RAW_RIGHT  0xBC43FF00
#define RAW_OK     0xBF40FF00
#define RAW_SPEED1 0xE916FF00
#define RAW_SPEED2 0xE619FF00
#define RAW_SPEED3 0xF20DFF00

// Motors
AF_DCMotor motor1(1); // Front Left
AF_DCMotor motor2(2); // Front Right
AF_DCMotor motor3(3); // Back Left
AF_DCMotor motor4(4); // Back Right

int motorSpeed = 150; // Default speed (0-255)

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("IR Robot Ready");
  setAllMotorsSpeed(motorSpeed);
}

void loop() {
  if (IrReceiver.decode()) {
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      uint32_t rawValue = IrReceiver.decodedIRData.decodedRawData;
      Serial.print("Raw IR Value: 0x");
      Serial.println(rawValue, HEX);

      handleIR(rawValue);
    }
    IrReceiver.resume();
  }
}

// Handle IR commands
void handleIR(uint32_t rawValue) {
  // Speed buttons
  if (rawValue == RAW_SPEED1) { motorSpeed = 80;  setAllMotorsSpeed(motorSpeed); Serial.println("Speed: SLOW"); return; }
  if (rawValue == RAW_SPEED2) { motorSpeed = 150; setAllMotorsSpeed(motorSpeed); Serial.println("Speed: MEDIUM"); return; }
  if (rawValue == RAW_SPEED3) { motorSpeed = 230; setAllMotorsSpeed(motorSpeed); Serial.println("Speed: FAST"); return; }

  // Movement buttons
  if (rawValue == RAW_UP)    { moveForward();  Serial.println("Move: FORWARD"); return; }
  if (rawValue == RAW_DOWN)  { moveBackward(); Serial.println("Move: BACKWARD"); return; }
  if (rawValue == RAW_LEFT)  { turnLeft();     Serial.println("Move: LEFT"); return; }
  if (rawValue == RAW_RIGHT) { turnRight();    Serial.println("Move: RIGHT"); return; }

  // Stop button
  if (rawValue == RAW_OK)    { stopMotors();   Serial.println("STOP"); return; }
}

// Motor functions
void setAllMotorsSpeed(int speedVal) {
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
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}