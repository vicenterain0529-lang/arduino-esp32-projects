const int MOTOR_A1_DIR = 2;
const int MOTOR_A1_PWM = 5;
const int MOTOR_B1_DIR = 4;
const int MOTOR_B1_PWM = 6;
const bool A1_Forward = HIGH;
const bool B1_Forward = LOW;



void setup() {
pinMode(MOTOR_A1_DIR,OUTPUT);
pinMode(MOTOR_A1_PWM,OUTPUT);
pinMode(MOTOR_B1_DIR,OUTPUT);
pinMode(MOTOR_B1_PWM,OUTPUT);

Stop();
 Serial.begin(9600);
 Serial.println("Robot Ready");

}

void Stop()
{
  analogWrite(MOTOR_A1_PWM, 0);
  analogWrite(MOTOR_B1_PWM, 0);

}

void Forward(int speed = 150)
{
  digitalWrite(MOTOR_A1_DIR, A1_Forward);
  analogWrite(MOTOR_A1_PWM, speed);
   digitalWrite(MOTOR_B1_DIR, B1_Forward);
  analogWrite(MOTOR_B1_PWM, speed);

}

void Backward(int speed = 150)
{
  digitalWrite(MOTOR_A1_DIR, !A1_Forward);
  analogWrite(MOTOR_A1_PWM, speed);
   digitalWrite(MOTOR_B1_DIR, !B1_Forward);
  analogWrite(MOTOR_B1_PWM, speed);

}

void Left(int speed = 150)
{
  digitalWrite(MOTOR_A1_DIR, !A1_Forward);
  analogWrite(MOTOR_A1_PWM, speed);
   digitalWrite(MOTOR_B1_DIR, B1_Forward);
  analogWrite(MOTOR_B1_PWM, speed);

}

void Right(int speed = 150)
{
  digitalWrite(MOTOR_A1_DIR, A1_Forward);
  analogWrite(MOTOR_A1_PWM, speed);
   digitalWrite(MOTOR_B1_DIR, !B1_Forward);
  analogWrite(MOTOR_B1_PWM, speed);

}


void loop() {
  
  Forward(150);
  delay(2000);
  Backward(150);
   delay(2000);

  Left(150);
   delay(2000);
  Right(150);
  delay(2000);
  Stop();
  delay(1000);
  

}


