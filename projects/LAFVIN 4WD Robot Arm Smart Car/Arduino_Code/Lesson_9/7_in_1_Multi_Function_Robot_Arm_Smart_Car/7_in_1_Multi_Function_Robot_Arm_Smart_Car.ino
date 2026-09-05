#include "IR_remote.h"
#include "keymap.h"

IRremote ir(3);

#include <Servo.h>

String BLE_val;
 int base_degrees;
 int arm_degrees;
 int claw_degrees;
 boolean forward_flag;
 boolean backward_flag;
 boolean left_flag;
 boolean right_flag;
 boolean claw_close_flag;
 boolean claw_open_flag;
 boolean arm_forward_flag;
 boolean claw_recracted_flag;
 boolean base_anticlockwise_flag;
 boolean base_clockwise_flag;
 boolean menory_action_flag;
 boolean Avoidance_Function_flag;
 boolean Following_Function_flag;
 boolean Anti_drop_Function_flag;
 boolean Line_tracking_Function_flag;
 boolean Gravity_sensor_Function_flag;
 int Left_Tra_Value;
 int Center_Tra_Value;
 int Right_Tra_Value;
 int Black_Line;
 int distance;
 int actions_count;
 int auto_count;
 int low_speed;
 int medium_speed;
 int high_speed;
 int speed_car;
int claw_read_degress[20]={0,0,0};

int arm_read_degress[20]={0,0,0};

int base_read_degress[20]={0,0,0};

Servo myservo1;
Servo myservo2;
Servo myservo3;
void read_degress() {
  if (actions_count <= 19) {
    claw_read_degress[(int)((actions_count + 1) - 1)] = myservo1.read();
    delay(50);
    arm_read_degress[(int)((actions_count + 1) - 1)] = myservo2.read();
    delay(50);
    base_read_degress[(int)((actions_count + 1) - 1)] = myservo3.read();
    delay(50);
    actions_count = actions_count + 1;
    auto_count = actions_count;
    Serial.println(auto_count);

  }
}

void IR_control_Function() {
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
  } else if (false) {
  } else if (false) {
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_7) {
    claw_degrees = claw_degrees + 5;
    if (claw_degrees >= 180) {
      claw_degrees = 180;

    }
    myservo1.write(claw_degrees);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_9) {
    claw_degrees = claw_degrees - 5;
    if (claw_degrees <= 50) {
      claw_degrees = 50;

    }
    myservo1.write(claw_degrees);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_2) {
    arm_degrees = arm_degrees + 5;
    if (arm_degrees >= 180) {
      arm_degrees = 180;

    }
    myservo2.write(arm_degrees);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_8) {
    arm_degrees = arm_degrees - 5;
    if (arm_degrees <= 0) {
      arm_degrees = 0;

    }
    myservo2.write(arm_degrees);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_4) {
    base_degrees = base_degrees + 5;
    if (base_degrees >= 180) {
      base_degrees = 180;

    }
    myservo3.write(base_degrees);
    delay(2);
  } else if (ir.getIrKey(ir.getCode(),1) == IR_KEYCODE_6) {
    base_degrees = base_degrees - 5;
    if (base_degrees <= 0) {
      base_degrees = 0;

    }
    myservo3.write(base_degrees);
    delay(2);
  }
}

void auto_do() {
  Serial.println(auto_count);
  if (0 != auto_count) {
    menory_action_flag = true;

  }
  actions_count = 0;
  claw_degrees = myservo1.read();
  arm_degrees = myservo2.read();
  base_degrees = myservo3.read();
  while (menory_action_flag) {
    for (int i = (1); i <= (auto_count); i = i + (1)) {
      if (Serial.read() == 's') {
        menory_action_flag = false;
        break;

      }
      if (claw_degrees < claw_read_degress[(int)(i - 1)]) {
        while (claw_degrees < claw_read_degress[(int)(i - 1)]) {
          claw_degrees = claw_degrees + 1;
          myservo1.write(claw_degrees);
          delay(15);
        }

      } else {
        while (claw_degrees > claw_read_degress[(int)(i - 1)]) {
          claw_degrees = claw_degrees - 1;
          myservo1.write(claw_degrees);
          delay(15);
        }

      }
      if (Serial.read() == 's') {
        menory_action_flag = false;
        break;

      }
      if (arm_degrees < arm_read_degress[(int)(i - 1)]) {
        while (arm_degrees < arm_read_degress[(int)(i - 1)]) {
          arm_degrees = arm_degrees + 1;
          myservo2.write(arm_degrees);
          delay(15);
        }

      } else {
        while (arm_degrees > arm_read_degress[(int)(i - 1)]) {
          arm_degrees = arm_degrees - 1;
          myservo2.write(arm_degrees);
          delay(15);
        }

      }
      if (Serial.read() == 's') {
        menory_action_flag = false;
        break;

      }
      if (base_degrees < base_read_degress[(int)(i - 1)]) {
        while (base_degrees < base_read_degress[(int)(i - 1)]) {
          base_degrees = base_degrees + 1;
          myservo3.write(base_degrees);
          delay(15);
        }

      } else {
        while (base_degrees > base_read_degress[(int)(i - 1)]) {
          base_degrees = base_degrees - 1;
          myservo3.write(base_degrees);
          delay(15);
        }

      }
      if (Serial.read() == 's') {
        menory_action_flag = false;
        break;

      }
    }
  }
}

void claw_close() {
  claw_close_flag = true;
  while (claw_close_flag) {
    claw_degrees = claw_degrees + 1;
    myservo1.write(claw_degrees);
    Serial.println(claw_degrees);
    delay(10);
    if (claw_degrees >= 180) {
      claw_degrees = 180;

    }
    if (Serial.read() == 's') {
      claw_close_flag = false;

    }
  }
}

void claw_open() {
  claw_close_flag = true;
  while (claw_close_flag) {
    claw_degrees = claw_degrees - 1;
    myservo1.write(claw_degrees);
    Serial.println(claw_degrees);
    delay(10);
    if (claw_degrees <= 50) {
      claw_degrees = 50;

    }
    if (Serial.read() == 's') {
      claw_close_flag = false;

    }
  }
}

void arm_up() {
  arm_forward_flag = true;
  while (arm_forward_flag) {
    arm_degrees = arm_degrees + 1;
    myservo2.write(arm_degrees);
    delay(10);
    Serial.println(arm_degrees);
    if (arm_degrees >= 180) {
      arm_degrees = 180;

    }
    if (Serial.read() == 's') {
      arm_forward_flag = false;

    }
  }
}

void arm_down() {
  claw_recracted_flag = true;
  while (claw_recracted_flag) {
    arm_degrees = arm_degrees - 1;
    myservo2.write(arm_degrees);
    Serial.println(arm_degrees);
    delay(10);
    if (arm_degrees <= 0) {
      arm_degrees = 0;

    }
    if (Serial.read() == 's') {
      claw_recracted_flag = false;

    }
  }
}

void Stop() {
  digitalWrite(2,LOW);
  analogWrite(5,0);
  digitalWrite(4,HIGH);
  analogWrite(6,0);
}

void arm_base_anticlockwise() {
  base_anticlockwise_flag = true;
  while (base_anticlockwise_flag) {
    base_degrees = base_degrees + 1;
    myservo3.write(base_degrees);
    Serial.println(base_degrees);
    delay(10);
    if (base_degrees >= 180) {
      base_degrees = 180;

    }
    if (Serial.read() == 's') {
      base_anticlockwise_flag = false;

    }
  }
}

void arm_base_clockwise() {
  base_clockwise_flag = true;
  while (base_clockwise_flag) {
    base_degrees = base_degrees - 1;
    myservo3.write(base_degrees);
    Serial.println(base_degrees);
    delay(10);
    if (base_degrees <= 0) {
      base_degrees = 0;

    }
    if (Serial.read() == 's') {
      base_clockwise_flag = false;

    }
  }
}

void Line_tracking_Function() {
  Line_tracking_Function_flag = true;
  while (Line_tracking_Function_flag) {
    Left_Tra_Value = digitalRead(7);
    Center_Tra_Value = digitalRead(8);
    Right_Tra_Value = digitalRead(A1);
    if (Left_Tra_Value != Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value != Black_Line)) {
      Move_Forward(120);

    } else if (Left_Tra_Value == Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value != Black_Line)) {
      Rotate_Left(80);
    } else if (Left_Tra_Value == Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value != Black_Line)) {
      Rotate_Left(120);
    } else if (Left_Tra_Value != Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value == Black_Line)) {
      Rotate_Right(120);
    } else if (Left_Tra_Value != Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value == Black_Line)) {
      Rotate_Right(80);
    } else if (Left_Tra_Value == Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value == Black_Line)) {
      Stop();
    }
    if (Serial.read() == 'S') {
      Line_tracking_Function_flag = false;
      Stop();

    }
  }
}

void Move_Backward(int speed) {
  digitalWrite(2,LOW);
  analogWrite(5,speed);
  digitalWrite(4,HIGH);
  analogWrite(6,speed);
}

float checkdistance() {
  digitalWrite(12, LOW);
  delayMicroseconds(2);
  digitalWrite(12, HIGH);
  delayMicroseconds(10);
  digitalWrite(12, LOW);
  float distance = pulseIn(13, HIGH) / 58.00;
  delay(10);
  return distance;
}

void Following_Function() {
  int Following_distance = 0;
  Following_Function_flag = true;
  while (Following_Function_flag) {
    Following_distance = checkdistance();
    if (Following_distance < 15) {
      Move_Backward(80);

    } else if (15 <= Following_distance && Following_distance <= 20) {
      Stop();
    } else if (20 <= Following_distance && Following_distance <= 25) {
      Move_Forward(80);
    } else if (25 <= Following_distance && Following_distance <= 30) {
      Move_Forward(100);
    } else {
      Stop();

    }
    if (Serial.read() == 'S') {
      Following_Function_flag = false;
      Stop();

    }
  }
}

void Rotate_Left(int speed) {
  digitalWrite(2,LOW);
  analogWrite(5,speed);
  digitalWrite(4,LOW);
  analogWrite(6,speed);
}

void Rotate_Right(int speed) {
  digitalWrite(2,HIGH);
  analogWrite(5,speed);
  digitalWrite(4,HIGH);
  analogWrite(6,speed);
}

void Move_Forward(int speed) {
  digitalWrite(2,HIGH);
  analogWrite(5,speed);
  digitalWrite(4,LOW);
  analogWrite(6,speed);
}

void Anti_drop_Function() {
  Anti_drop_Function_flag = true;
  while (Anti_drop_Function_flag) {
    Left_Tra_Value = digitalRead(7);
    Center_Tra_Value = digitalRead(8);
    Right_Tra_Value = digitalRead(A1);
    if (Left_Tra_Value != Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value != Black_Line)) {
      Move_Forward(60);

    } else {
      Move_Backward(60);
      delay(600);
      Rotate_Left(60);
      delay(500);

    }
    if (Serial.read() == 'S') {
      Anti_drop_Function_flag = false;
      Stop();

    }
  }
}

void Move_backward_Function() {
  backward_flag = true;
  while (backward_flag) {
    Move_Backward(speed_car);
    if (Serial.read() == 'S') {
      backward_flag = false;
      Stop();

    }
  }
}

void Move_forward_Function() {
  forward_flag = true;
  while (forward_flag) {
    Move_Forward(speed_car);
    if (Serial.read() == 'S') {
      forward_flag = false;
      Stop();

    }
  }
}

void Avoidance_Function() {
  int Avoidance_distance = 0;
  Avoidance_Function_flag = true;
  while (Avoidance_Function_flag) {
    Avoidance_distance = checkdistance();
    if (Avoidance_distance <= 25) {
      if (Avoidance_distance <= 15) {
        Stop();
        delay(100);
        Move_Backward(100);
        delay(600);

      } else {
        Stop();
        delay(100);
        Rotate_Left(100);
        delay(600);

      }

    } else {
      Move_Forward(70);

    }
    if (Serial.read() == 'S') {
      Avoidance_Function_flag = false;
      Stop();

    }
  }
}

void Turn_right_Function() {
  right_flag = true;
  while (right_flag) {
    Rotate_Right(speed_car);
    if (Serial.read() == 'S') {
      right_flag = false;
      Stop();

    }
  }
}

void Turn_left_Function() {
  left_flag = true;
  while (left_flag) {
    Rotate_Left(speed_car);
    if (Serial.read() == 'S') {
      left_flag = false;
      Stop();

    }
  }
}

char bluetooth_val;

void Gravity_sensor_Function() {
  Gravity_sensor_Function_flag = true;
  while (Gravity_sensor_Function_flag) {
    if (Serial.available())
    {
      bluetooth_val = Serial.read();
      Serial.println(bluetooth_val);
      if (bluetooth_val == 'F') {
        Move_Forward(speed_car);

      } else if (bluetooth_val == 'B') {
        Move_Backward(speed_car);
      } else if (bluetooth_val == 'L') {
        Rotate_Left(speed_car);
      } else if (bluetooth_val == 'R') {
        Rotate_Right(speed_car);
      } else if (bluetooth_val == 'p') {
        Stop();
      } else if (bluetooth_val == 'X') {
        speed_car = low_speed;
      } else if (bluetooth_val == 'Y') {
        speed_car = medium_speed;
      } else if (bluetooth_val == 'Z') {
        speed_car = high_speed;
      }
      if (bluetooth_val == 'S') {
        Gravity_sensor_Function_flag = false;
        Stop();

      }
    }
  }
}

void setup(){
  IRremote ir(3);

  Serial.begin(9600);
  BLE_val = "";
  base_degrees = 90;
  arm_degrees = 90;
  claw_degrees = 90;
  forward_flag = false;
  backward_flag = false;
  left_flag = false;
  right_flag = false;
  claw_close_flag = false;
  claw_open_flag = false;
  arm_forward_flag = false;
  claw_recracted_flag = false;
  base_anticlockwise_flag = false;
  base_clockwise_flag = false;
  menory_action_flag = false;
  Avoidance_Function_flag = false;
  Following_Function_flag = false;
  Anti_drop_Function_flag = false;
  Line_tracking_Function_flag = false;
  Gravity_sensor_Function_flag = false;
  Left_Tra_Value = 1;
  Center_Tra_Value = 1;
  Right_Tra_Value = 1;
  Black_Line = 1;
  distance = 0;
  actions_count = 0;
  auto_count = 0;
  low_speed = 60;
  medium_speed = 120;
  high_speed = 160;
  speed_car = 60;
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
  myservo1.write(claw_degrees);
  delay(500);
  myservo2.write(arm_degrees);
  delay(500);
  myservo3.write(base_degrees);
  delay(500);
  Stop();
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(A1, INPUT);
  pinMode(12, OUTPUT);
  pinMode(13, INPUT);
  Serial.begin(9600);
}

void loop(){
  while (Serial.available() > 0) {
    BLE_val = BLE_val + ((char)(Serial.read()));
    delay(2);
  }
  if (0 < String(BLE_val).length() && 2 >= String(BLE_val).length()) {
    Serial.println(String(BLE_val).length());
    Serial.println(BLE_val);
    switch (String(BLE_val).charAt(0)) {
     case 'o':
      claw_open();
      break;
     case 'c':
      claw_close();
      break;
     case 'u':
      arm_up();
      break;
     case 'd':
      arm_down();
      break;
     case 'l':
     
      arm_base_anticlockwise();
      break;
     case 'r':
      arm_base_clockwise();
      break;
     case 'F':
      Move_forward_Function();
      break;
     case 'B':
      Move_backward_Function();
      break;
     case 'L':
      Turn_left_Function();
      break;
     case 'R':
      Turn_right_Function();
      break;
     case 'S':
      Stop();
      break;
     case 'm':
      Serial.println(actions_count);
      read_degress();
      break;
     case 'a':
      auto_do();
      break;
     case 'X':
      speed_car = low_speed;
      break;
     case 'Y':
      speed_car = medium_speed;
      break;
     case 'Z':
      speed_car = high_speed;
      break;
     case 'A':
      Avoidance_Function();
      break;
     case 'D':
      Anti_drop_Function();
      break;
     case 'W':
      Following_Function();
      break;
     case 'T':
      Line_tracking_Function();
      break;
     case 'G':
      Gravity_sensor_Function();
      break;
    }
    BLE_val = "";

  } else {
    BLE_val = "";

  }
  IR_control_Function();

}
