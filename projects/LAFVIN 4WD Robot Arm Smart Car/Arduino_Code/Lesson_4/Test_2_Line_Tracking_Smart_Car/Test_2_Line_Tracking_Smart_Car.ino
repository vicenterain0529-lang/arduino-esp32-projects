 int Left_Tra_Value;
 int Center_Tra_Value;
 int Right_Tra_Value;
 int Black_Line;

void setup(){
  Left_Tra_Value = 1;
  Center_Tra_Value = 1;
  Right_Tra_Value = 1;
  Black_Line = 1;
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(A1, INPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop(){
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

}




void Move_Forward(int speed) {
  digitalWrite(2,HIGH);
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
