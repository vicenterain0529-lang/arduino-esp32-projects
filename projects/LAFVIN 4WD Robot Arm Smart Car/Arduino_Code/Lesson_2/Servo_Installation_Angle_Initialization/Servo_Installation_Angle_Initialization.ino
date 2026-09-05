#include <Servo.h>
int claw_degrees;
int arm_degrees;
int base_degrees;
Servo myservo1;
Servo myservo2;
Servo myservo3;

void setup()
{
claw_degrees = 90;
arm_degrees = 135;
base_degrees = 90;
myservo1.attach(9);
myservo2.attach(10);
myservo3.attach(11);
}

void loop()
{
  myservo1.write(claw_degrees);
  delay(200);
  myservo2.write(arm_degrees);
    delay(200);
  myservo3.write(base_degrees);
    delay(200);
}
