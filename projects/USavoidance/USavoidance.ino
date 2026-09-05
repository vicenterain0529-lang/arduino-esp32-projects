// Ultrasonic: GND-Echo-Trig-VCC
//   Echo  → Pin 12
//   Trig  → Pin 13
// Motors:
//   Left  Dir → Pin 2,  PWM → Pin 5
//   Right Dir → Pin 4,  PWM → Pin 6

// If robot moves BACKWARD when it should go FORWARD, set this to true
const bool FIX_REVERSE = true;  

// Speed settings (0-255). Lower = slower but more control
const int SPEED_NORMAL = 150;    // Normal driving speed
const int SPEED_BACKUP = 150;    // Reversing speed  
const int SPEED_TURN   = 150;    // Turning speed

// Timing (milliseconds)
const int TIME_BACKUP  = 500;    // How long to reverse when blocked
const int TIME_TURN    = 500;    // How long to turn when avoiding obstacle
const int TIME_PAUSE   = 200;    // Pause before reversing

// Safety distance (cm) - robot stops and turns if obstacle closer than this
const int DISTANCE_MIN = 20;

// Motor pins
const int PIN_LEFT_DIR  = 2;   // Left motor direction
const int PIN_LEFT_PWM  = 5;   // Left motor speed
const int PIN_RIGHT_DIR = 4;   // Right motor direction  
const int PIN_RIGHT_PWM = 6;   // Right motor speed

// Ultrasonic pins
const int PIN_TRIG = 13;   // Trigger (sends pulse)
const int PIN_ECHO = 12;   // Echo (receives pulse)

// Working variables
long echoTime;      // Raw time from sensor
int distanceCm;     // Calculated distance

void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  randomSeed(analogRead(A0));
}

// ============================================
// MAIN LOOP - FIXED!
// ============================================

void loop() {
  fullStop();                    // ✅ Fixed: was stop()
  delay(50);                     
  
  long distance = measureDistance();  // ✅ Fixed: was getDistance()
  
  if(distance > DISTANCE_MIN) {    // ✅ Fixed: was Distance_min
    driveForward(SPEED_NORMAL);   // ✅ Fixed: was forward()
    delay(100);              
  } 
  else {
    avoidObstacle();              // ✅ Fixed: was obstacleAvoidance()
    driveBackward(SPEED_BACKUP);  // ✅ Fixed: was backward(SPEED_BACK)
    delay(TIME_BACKUP);           // ✅ Fixed: was TIME_BACK
    fullStop();                   // ✅ Fixed: was stop()
  }
}

// ============================================
// MOVEMENT FUNCTIONS
// ============================================

void driveForward(int speed) {
  digitalWrite(PIN_LEFT_DIR, FIX_REVERSE ? LOW : HIGH);
  analogWrite(PIN_LEFT_PWM, speed);
  digitalWrite(PIN_RIGHT_DIR, FIX_REVERSE ? LOW : HIGH);
  analogWrite(PIN_RIGHT_PWM, speed);
}

void driveBackward(int speed) {
  digitalWrite(PIN_LEFT_DIR, FIX_REVERSE ? HIGH : LOW);
  analogWrite(PIN_LEFT_PWM, speed);
  digitalWrite(PIN_RIGHT_DIR, FIX_REVERSE ? HIGH : LOW);
  analogWrite(PIN_RIGHT_PWM, speed);
}

void turnLeft(int speed) {
  digitalWrite(PIN_LEFT_DIR, FIX_REVERSE ? HIGH : LOW);
  analogWrite(PIN_LEFT_PWM, speed);
  digitalWrite(PIN_RIGHT_DIR, FIX_REVERSE ? LOW : HIGH);
  analogWrite(PIN_RIGHT_PWM, speed);
}

void turnRight(int speed) {
  digitalWrite(PIN_LEFT_DIR, FIX_REVERSE ? LOW : HIGH);
  analogWrite(PIN_LEFT_PWM, speed);
  digitalWrite(PIN_RIGHT_DIR, FIX_REVERSE ? HIGH : LOW);
  analogWrite(PIN_RIGHT_PWM, speed);
}

void fullStop() {
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
}

// ============================================
// OBSTACLE AVOIDANCE
// ============================================

void avoidObstacle() {
  fullStop();
  delay(TIME_PAUSE);
  
  driveBackward(SPEED_BACKUP);
  delay(TIME_BACKUP);
  fullStop();
  
  if (random(2) == 0) {
    turnLeft(SPEED_TURN);
  } else {
    turnRight(SPEED_TURN);
  }
  delay(TIME_TURN);
  fullStop();
}

// ============================================
// SENSOR
// ============================================

int measureDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  echoTime = pulseIn(PIN_ECHO, HIGH, 30000);
  distanceCm = echoTime * 0.034 / 2;
  
  if (distanceCm == 0 || distanceCm > 400) {
    distanceCm = 999;
  }
  
  return distanceCm;
}