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


void setup(){
  pinMode(12, OUTPUT);
  pinMode(13, INPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop()
{
  int Avoidance_distance = 0;

  Avoidance_distance = checkdistance();
  if (Avoidance_distance <= 25) 
  {
    if (Avoidance_distance <= 15) 
    {
      Stop();
      delay(100);
      Move_Backward(100);
      delay(600);
    } 
    else
    {
      Stop();
      delay(100);
      Rotate_Left(100);
      delay(600);
    }
  } 
  else 
  {
    Move_Forward(70);
  }

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
