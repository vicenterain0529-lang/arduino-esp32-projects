const int MOTOR_A_DIR = 2;
const int MOTOR_A_PWM = 5;
const int MOTOR_B_DIR = 4;
const int MOTOR_B_PWM = 6;
const bool A_FORWARD = HIGH;   // Motor A spins forward when pin is HIGH
const bool B_FORWARD = LOW; 
int speed = 150;   // Motor B spins forward when pin is LOW

void setup() {
  // Set motor pins as outputs
  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  // Start with motors stopped
  Stop();
  
  // Optional: Start serial for debugging
  Serial.begin(9600);
  Serial.println("Robot Ready!");
}

void loop() {
  // EXAMPLE 1: Simple forward-backward-turn sequence
  // Uncomment the line below to run demo mode:
  // demoMode();
  
  // EXAMPLE 2: Remote control via Serial Monitor
  // Type: f=forward, b=backward, l=left, r=right, s=stop
  serialControlMode();
}

void Stop() {
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
}

void Move_Forward(int speed)     
{                                
  // Code inside runs when called
  digitalWrite(MOTOR_A_DIR, A_FORWARD);
  analogWrite(MOTOR_A_PWM, speed);
  
  digitalWrite(MOTOR_B_DIR, B_FORWARD);
  analogWrite(MOTOR_B_PWM, speed);
}                               

void Move_Backward(int speed)    
{                                
  // Code inside runs when called
  digitalWrite(MOTOR_A_DIR, !A_FORWARD);
  analogWrite(MOTOR_A_PWM, speed);
  
  digitalWrite(MOTOR_B_DIR, !B_FORWARD);
  analogWrite(MOTOR_B_PWM, speed);
}                                

void Move_Left(int speed)    
{                                
  // Code inside runs when called
  digitalWrite(MOTOR_A_DIR, !A_FORWARD);
  analogWrite(MOTOR_A_PWM, speed);
  
  digitalWrite(MOTOR_B_DIR, B_FORWARD);
  analogWrite(MOTOR_B_PWM, speed);
}                                

void Move_Right(int speed)    
{                                
  // Code inside runs when called
  digitalWrite(MOTOR_A_DIR, A_FORWARD);
  analogWrite(MOTOR_A_PWM, speed);
  
  digitalWrite(MOTOR_B_DIR, !B_FORWARD);
  analogWrite(MOTOR_B_PWM, speed);
}                                




























