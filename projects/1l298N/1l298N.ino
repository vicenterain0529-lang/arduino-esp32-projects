#include <L298N.h>

// Pin definitions
#define ENA 6   // Right motor speed (PWM)
#define IN1 7   // Right motor direction
#define IN2 8   // Right motor direction

#define ENB 5   // Left motor speed (PWM)
#define IN3 9   // Left motor direction
#define IN4 10  // Left motor direction

// Create motor objects (Standard order)
L298N rightMotor(ENA, IN1, IN2);
L298N leftMotor(ENB, IN3, IN4); 

// Speed set to 200 to prevent stalling in reverse
int motorSpeed = 200; 
bool running = false;

void setup() {
  Serial.begin(9600);
  
  // Set speed once in setup
  rightMotor.setSpeed(motorSpeed);
  leftMotor.setSpeed(motorSpeed);
  
  rightMotor.stop();
  leftMotor.stop();
  
  Serial.println("=== CLEAN L298N MOTOR TEST ===");
  Serial.println("Type '1' to start, '0' to stop");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == '1') {
      running = true;
      Serial.println("FORWARD");
      rightMotor.forward();
      leftMotor.forward(); 
    }
    else if (command == '0') {
      running = false;
      Serial.println("STOP");
      rightMotor.stop();
      leftMotor.stop();
    }
  }
  
  if (running) {
    delay(8000);
    
    Serial.println("REVERSE - BOTH MOTORS");
    rightMotor.backward();
    leftMotor.backward(); 
    
    delay(8000);
    
    Serial.println("STOP - Press '1' to run again");
    rightMotor.stop();
    leftMotor.stop();
    running = false;
  }
}