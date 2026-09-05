// ===============================
// Lafvin 4WD Smart Obstacle Avoidance
// Fast reaction & improved stuck handling
// ===============================

// Motor pins
const int LEFT_DIR = 2;
const int LEFT_PWM = 5;
const int RIGHT_DIR = 4;
const int RIGHT_PWM = 6;
const bool LEFT_FORWARD = HIGH;
const bool RIGHT_FORWARD = LOW;

// Ultrasonic pins
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// Speed settings
const int SPEED_BACK = 100;
const int SPEED_TURN = 150;
const int SPEED_FORWARD = 150;

// Timing (ms)
const int TIME_BACK = 200;    // faster back
const int TIME_TURN = 400;    // faster turn
const int TIME_PAUSE = 50;    // shorter pause
const int distance_min = 10;

// Stuck/corner escape variables
int stuckCount = 0;
const int STUCK_THRESHOLD = 3;
bool lastTurnLeft = true;

int consecutiveTurns = 0;
const int CONSECUTIVE_TURN_LIMIT = 2; // backup if too many same-direction turns

// ===============================
// Forward declarations (safe for Arduino)
// ===============================
void forward(int speed);
void backward(int speed);
void left(int speed);
void right(int speed);
void fullstop();
long getDistance();
void avoidObstacle();

// ===============================
// Setup
// ===============================
void setup() {
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  randomSeed(analogRead(A0));
  Serial.begin(9600);
  Serial.println("Robot Ready");
}

// ===============================
// Distance function
// ===============================
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long distance = duration * 0.0343 / 2;
  if (distance == 0 || distance > 400) distance = 999;
  return distance;
}

// ===============================
// Movement functions
// ===============================
void forward(int speed) {
  digitalWrite(LEFT_DIR, LEFT_FORWARD);
  analogWrite(LEFT_PWM, speed);
  digitalWrite(RIGHT_DIR, RIGHT_FORWARD);
  analogWrite(RIGHT_PWM, speed);
}

void backward(int speed) {
  digitalWrite(LEFT_DIR, !LEFT_FORWARD);
  analogWrite(LEFT_PWM, speed);
  digitalWrite(RIGHT_DIR, !RIGHT_FORWARD);
  analogWrite(RIGHT_PWM, speed);
}

void left(int speed) {
  digitalWrite(LEFT_DIR, !LEFT_FORWARD);
  analogWrite(LEFT_PWM, speed - 30); // inner wheel slower for smooth curve
  digitalWrite(RIGHT_DIR, RIGHT_FORWARD);
  analogWrite(RIGHT_PWM, speed);
}

void right(int speed) {
  digitalWrite(LEFT_DIR, LEFT_FORWARD);
  analogWrite(LEFT_PWM, speed);
  digitalWrite(RIGHT_DIR, !RIGHT_FORWARD);
  analogWrite(RIGHT_PWM, speed - 30); // inner wheel slower
}

void fullstop() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

// ===============================
// Obstacle avoidance
// ===============================
void avoidObstacle() {
  fullstop();
  delay(TIME_PAUSE);

  // Step 0: If stuck too long, backup first
  if(consecutiveTurns >= CONSECUTIVE_TURN_LIMIT) {
    backward(SPEED_BACK);
    delay(TIME_BACK + 100); // longer backup to escape
    fullstop();
    delay(50);
    consecutiveTurns = 0;
  }

  // Step 1: Back up slightly
  backward(SPEED_BACK);
  delay(TIME_BACK);
  fullstop();
  delay(50);

  // Step 2: Measure left
  left(SPEED_TURN);
  delay(150);
  long leftDistance = getDistance();
  fullstop();
  delay(50);

  // Step 3: Measure right
  right(SPEED_TURN);
  delay(300);
  long rightDistance = getDistance();
  fullstop();
  delay(50);

  // Step 4: Decide turn
  bool turnLeftFlag;
  if(stuckCount >= STUCK_THRESHOLD) {
    turnLeftFlag = !lastTurnLeft; // force opposite if stuck
    stuckCount = 0;
    consecutiveTurns = 0;
  } else {
    turnLeftFlag = leftDistance > rightDistance;
  }

  // Step 5: Execute turn
  if(turnLeftFlag) left(SPEED_TURN);
  else right(SPEED_TURN);
  delay(TIME_TURN);
  fullstop();
  delay(50);

  // Step 6: Update tracking
  if(turnLeftFlag == lastTurnLeft) consecutiveTurns++;
  else consecutiveTurns = 1;

  lastTurnLeft = turnLeftFlag;
  stuckCount++;
}

// ===============================
// Main loop
// ===============================
void loop() {
  long distance = getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  if(distance > distance_min) {
    forward(SPEED_FORWARD);
  } else {
    avoidObstacle();
  }

  delay(10); // fast loop for near-instant reaction
}