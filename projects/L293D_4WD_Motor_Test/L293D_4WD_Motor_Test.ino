#include <AFMotor.h>

AF_DCMotor motor1(1);  // Left  Front
AF_DCMotor motor2(2);  // Right Front
AF_DCMotor motor3(3);  // Left  Rear
AF_DCMotor motor4(4);  // Right Rear

#define SPEED_SLOW   60
#define SPEED_MEDIUM 150
#define SPEED_FAST   200

int speedMode = 2;
int curSpeed  = SPEED_MEDIUM;

// Move helpers
void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void setForward(int spd) {
  motor1.run(FORWARD);  motor2.run(FORWARD);
  motor3.run(FORWARD);  motor4.run(FORWARD);
  motor1.setSpeed(spd); motor3.setSpeed(spd);
  motor2.setSpeed(spd); motor4.setSpeed(spd);
}

void setBackward(int spd) {
  motor1.run(BACKWARD); motor2.run(BACKWARD);
  motor3.run(BACKWARD); motor4.run(BACKWARD);
  motor1.setSpeed(spd); motor3.setSpeed(spd);
  motor2.setSpeed(spd); motor4.setSpeed(spd);
}

// Pivot turns (same pattern as your main sketch)
void setLeft(int spd) {
  motor1.run(BACKWARD); motor3.run(BACKWARD);
  motor2.run(FORWARD);  motor4.run(FORWARD);
  motor1.setSpeed(spd); motor3.setSpeed(spd);
  motor2.setSpeed(spd); motor4.setSpeed(spd);
}

void setRight(int spd) {
  motor1.run(FORWARD);  motor3.run(FORWARD);
  motor2.run(BACKWARD); motor4.run(BACKWARD);
  motor1.setSpeed(spd); motor3.setSpeed(spd);
  motor2.setSpeed(spd); motor4.setSpeed(spd);
}

void applySpeedMode() {
  switch (speedMode) {
    case 1: curSpeed = SPEED_SLOW; break;
    case 2: curSpeed = SPEED_MEDIUM; break;
    case 3: curSpeed = SPEED_FAST; break;
    default: speedMode = 2; curSpeed = SPEED_MEDIUM; break;
  }
}

void setup() {
  Serial.begin(9600);

  stopAll();
  applySpeedMode();

  Serial.println(F("=== Motor TEST (L293D shield) ==="));
  Serial.println(F("Commands: F B L R S | speed: 1/2/3"));
  Serial.println(F("Set line ending: None or NL"));
}

void loop() {
  if (!Serial.available()) return;

  char cmd = (char)Serial.read();
  if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;

  // normalize lowercase -> uppercase
  if (cmd >= 'a' && cmd <= 'z') cmd = (char)(cmd - ('a' - 'A'));

  switch (cmd) {
    case 'F': setForward(curSpeed); break;
    case 'B': setBackward(curSpeed); break;
    case 'L': setLeft(curSpeed); break;
    case 'R': setRight(curSpeed); break;
    case 'S': stopAll(); break;

    case '1': speedMode = 1; applySpeedMode(); Serial.println(F("Speed=1 (SLOW)")); break;
    case '2': speedMode = 2; applySpeedMode(); Serial.println(F("Speed=2 (MED)"));  break;
    case '3': speedMode = 3; applySpeedMode(); Serial.println(F("Speed=3 (FAST)")); break;

    default:
      Serial.print(F("Unknown: "));
      Serial.println(cmd);
      break;
  }
}