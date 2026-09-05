  // ===============================
  // Lafvin 4WD Robot - Ultrasonic Auto
  // Using your directional system
  // ===============================

  // Motor pins
  const int MOTOR_A_DIR = 2; // Left side direction
  const int MOTOR_A_PWM = 5; // Left side PWM
  const int MOTOR_B_DIR = 4; // Right side direction
  const int MOTOR_B_PWM = 6; // Right side PWM

  // Ultrasonic pins
  const int TRIG_PIN = 12;
  const int ECHO_PIN = 13;

  // Motor polarities (set these according to wiring)
  const bool A_MOTOR = HIGH; // Left forward
  const bool B_MOTOR = LOW;  // Right forward

  // Speed settings
  const int SPEED_NORMAL = 150; // Forward speed
  const int SPEED_BACKUP = 100; // Backward speed
  const int SPEED_TURN   = 120; // Turn speed

  // Timing (ms)
  const int TIME_BACKUP = 400; // Reverse duration
  const int TIME_TURN   = 800; // Turn duration
  const int TIME_PAUSE  = 200; // Pause before reversing

  // Safety distance (cm)
  const int DISTANCE_MIN = 20;

  // ===============================
  // Setup
  // ===============================
  void setup() {
    Serial.begin(9600);
    
    // Motor pins
    pinMode(MOTOR_A_DIR, OUTPUT);
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_B_DIR, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);

    // Ultrasonic pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    fullstop(); // Make sure robot is stopped at start
    Serial.println("Robot ready");

    // Seed for random turns
    randomSeed(analogRead(A0));
  }

  // ===============================
  // Loop
  // ===============================
  void loop() {
    long distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance > DISTANCE_MIN) {
      // Path clear - move forward
      forward(SPEED_NORMAL);
    } else {
      // Obstacle detected
      avoidObstacle();
    }

    delay(50); // stabilize readings
  }

  // ===============================
  // Movement Functions
  // ===============================
  void forward(int speed) {
    digitalWrite(MOTOR_A_DIR, A_MOTOR);
    analogWrite(MOTOR_A_PWM, speed);
    digitalWrite(MOTOR_B_DIR, B_MOTOR);
    analogWrite(MOTOR_B_PWM, speed);
  }

  void backward(int speed) {
    digitalWrite(MOTOR_A_DIR, !A_MOTOR);
    analogWrite(MOTOR_A_PWM, speed);
    digitalWrite(MOTOR_B_DIR, !B_MOTOR);
    analogWrite(MOTOR_B_PWM, speed);
  }

  void turnLeft(int speed) {
    // Both sides forward, but left slightly slower (prevents spinning)
    digitalWrite(MOTOR_A_DIR, !A_MOTOR);
    analogWrite(MOTOR_A_PWM, speed - 20); // left slower
    digitalWrite(MOTOR_B_DIR, B_MOTOR);
    analogWrite(MOTOR_B_PWM, speed);
  }

  void turnRight(int speed) {
    // Both sides forward, but right slightly slower (prevents spinning)
    digitalWrite(MOTOR_A_DIR, A_MOTOR);
    analogWrite(MOTOR_A_PWM, speed);
    digitalWrite(MOTOR_B_DIR, !B_MOTOR);
    analogWrite(MOTOR_B_PWM, speed - 20); // right slower
  }

  void fullstop() {
    analogWrite(MOTOR_A_PWM, 0);
    analogWrite(MOTOR_B_PWM, 0);
  }

  // ===============================
  // Obstacle Avoidance
  // ===============================
  void avoidObstacle() {
  fullstop();
  delay(TIME_PAUSE);

  // Step 1: Back up a little first if consecutive turns exceeded limit
  if(consecutiveTurns >= CONSECUTIVE_TURN_LIMIT) {
    backward(SPEED_BACK);
    delay(TIME_BACK + 100); // slightly longer backup
    fullstop();
    delay(50);
    consecutiveTurns = 0; // reset counter after forced backup
  }

  // Step 2: Back up normally
  backward(SPEED_BACK);
  delay(TIME_BACK);
  fullstop();
  delay(50);

  // Step 3: Measure left
  left(SPEED_TURN);
  delay(150);
  long leftDistance = getDistance();
  fullstop();
  delay(50);

  // Step 4: Measure right
  right(SPEED_TURN);
  delay(300);
  long rightDistance = getDistance();
  fullstop();
  delay(50);

  // Step 5: Decide turn
  bool turnLeftFlag;

  if (stuckCount >= STUCK_THRESHOLD) {
    // force opposite of last turn if stuck
    turnLeftFlag = !lastTurnLeft;
    stuckCount = 0;
    consecutiveTurns = 0;
  } else {
    turnLeftFlag = leftDistance > rightDistance;
  }

  // Step 6: Execute turn
  if (turnLeftFlag) left(SPEED_TURN);
  else right(SPEED_TURN);
  delay(TIME_TURN);
  fullstop();
  delay(50);

  // Step 7: Update tracking
  if(turnLeftFlag == lastTurnLeft) consecutiveTurns++;
  else consecutiveTurns = 1; // reset if direction changed

  lastTurnLeft = turnLeftFlag;
  stuckCount++;
}
  // ===============================
  // Ultrasonic Distance Function
  // ===============================
  long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    long distance = duration * 0.034 / 2;

    if (distance == 0 || distance > 400) {
      distance = 999; // consider clear if sensor fails
    }

    return distance;
  }