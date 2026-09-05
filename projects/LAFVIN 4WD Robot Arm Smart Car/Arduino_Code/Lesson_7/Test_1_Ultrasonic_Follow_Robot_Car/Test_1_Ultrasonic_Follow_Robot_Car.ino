float checkdistance() 
{
  digitalWrite(12, LOW);
  delayMicroseconds(2);
  digitalWrite(12, HIGH);
  delayMicroseconds(10);
  digitalWrite(12, LOW);
  float distance = pulseIn(13, HIGH) / 58.00;
  delay(10);
  return distance;
}



void Stop() {
  digitalWrite(2,LOW);
  analogWrite(5,0);
  digitalWrite(4,HIGH);
  analogWrite(6,0);
}

void setup(){
  pinMode(12, OUTPUT);
  pinMode(13, INPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop(){
  int Following_distance = 0;
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

void Move_Backward(int speed) {
  digitalWrite(2,LOW);
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
