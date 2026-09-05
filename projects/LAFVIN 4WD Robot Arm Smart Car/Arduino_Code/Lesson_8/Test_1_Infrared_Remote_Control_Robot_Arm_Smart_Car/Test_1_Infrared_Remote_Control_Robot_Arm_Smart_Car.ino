#include "IR_remote.h"
#include "keymap.h"

IRremote ir(3);

#include <Servo.h>

int base_degress;
int arm_degress;
int claw_degress;

Servo myservo1;
Servo myservo2;
Servo myservo3;



void setup()
{
  IRremote ir(3);

  base_degress = 90;
  arm_degress = 90;
  claw_degress = 90;
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
  myservo1.write(claw_degress);
  delay(500);
  myservo2.write(arm_degress);
  delay(500);
  myservo3.write(base_degress);
  delay(500);
  Stop();
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop(){
  if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_UP) {
    Move_Forward(100);
    delay(300);
    Stop();

  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_DOWN) {
    Move_Backward(100);
    delay(300);
    Stop();
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_LEFT) {
    Rotate_Left(70);
    delay(300);
    Stop();
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_RIGHT) {
    Rotate_Right(70);
    delay(300);
    Stop();
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_OK) {
    Stop();
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_9) {
    claw_degress = claw_degress - 5;
    if (claw_degress <= 50) {
      claw_degress = 50;

    }
    myservo1.write(claw_degress);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_7) {
    claw_degress = claw_degress + 5;
    if (claw_degress >= 180) {
      claw_degress = 180;

    }
    myservo1.write(claw_degress);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_2) {
    arm_degress = arm_degress + 5;
    if (arm_degress >= 180) {
      arm_degress = 180;

    }
    myservo2.write(arm_degress);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_8) {
    arm_degress = arm_degress - 5;
    if (arm_degress <= 0) {
      arm_degress = 0;

    }
    myservo2.write(arm_degress);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_4) {
    base_degress = base_degress + 5;
    if (base_degress >= 180) {
      base_degress = 180;

    }
    myservo3.write(base_degress);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_6) {
    base_degress = base_degress - 5;
    if (base_degress <= 0) {
      base_degress = 0;

    }
    myservo3.write(base_degress);
    delay(2);
  }

}




void Move_Backward(int speed) {
  digitalWrite(2,LOW);
  analogWrite(5,speed);
  digitalWrite(4,HIGH);
  analogWrite(6,speed);
}

void Rotate_Right(int speed) {
  digitalWrite(2,HIGH);
  analogWrite(5,speed);
  digitalWrite(4,HIGH);
  analogWrite(6,speed);
}

void Rotate_Left(int speed) {
  digitalWrite(2,LOW);
  analogWrite(5,speed);
  digitalWrite(4,LOW);
  analogWrite(6,speed);
}

void Stop() {
  digitalWrite(2,LOW);
  analogWrite(5,0);
  digitalWrite(4,HIGH);
  analogWrite(6,0);
}

void Move_Forward(int speed) {
  digitalWrite(2,HIGH);
  analogWrite(5,speed);
  digitalWrite(4,LOW);
  analogWrite(6,speed);
}
