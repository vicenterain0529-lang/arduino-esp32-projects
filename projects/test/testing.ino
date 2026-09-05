int redPin = 3;      // PWM pin
int butPin = 4;
int yellowPin = 5;   // PWM pin
int greenPin = 6;  
int potPin = A0;  // PWM pin - FIX 1: Added green pin!
int trigPin = 7;
int echoPin = 8;
bool ledOn = false;
int lastButtonState = HIGH;
long duration;
float distance;

int brightness = 0;
int fadeAmount = 5;

unsigned long previousTime = 0;


int state = 0;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);  
  pinMode(greenPin, OUTPUT);   // FIX 1: Set green pin as an OUTPUT
  pinMode(butPin, INPUT_PULLUP);
  pinMode(potPin, INPUT);
  pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

void loop() {


unsigned long currentTime = millis();
int potVal = analogRead(potPin);
int brightness = map(potVal, 0, 1023, 0, 255);
float potVolts = potVal * 5.00 / 1023.00;

digitalWrite(trigPin,LOW);
delayMicroseconds(2);


digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin,LOW);


duration = pulseIn(echoPin, HIGH);

distance = duration * 0.0343 / 2;



if (currentTime - previousTime >= 500){

previousTime = currentTime;

Serial.print("Distance: ");
Serial.print(distance);
Serial.println(" cm");


Serial.print("POTENTIOMETER VALUE: ");
Serial.print(potVolts);
Serial.println(" V");

}
if(distance >= 30){
analogWrite(greenPin, brightness);
analogWrite(redPin, 0);
analogWrite(yellowPin, 0);
}
else if(distance >= 11 && distance < 30){
analogWrite(yellowPin, brightness);
analogWrite(greenPin, 0);

}
else if(distance < 10){
analogWrite(redPin, brightness);
analogWrite(yellowPin, 0);
analogWrite(greenPin, 0);
}




}


  