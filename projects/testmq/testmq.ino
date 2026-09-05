// MQ-2 Gas Sensor with Alert (Fixed)
#include <math.h>

int mqPin = A0;       // MQ-2 analog pin
int ledPin = 13;      // Onboard LED
int buzzerPin = 12;   // Optional buzzer
float Ro = 10.0;      // Sensor resistance in clean air
float RL = 5.0;       // Load resistor in kΩ (usually 5kΩ)
int calibrationSamples = 100;
int sampleInterval = 50;

// Alert thresholds (ppm)
float LPG_threshold = 300;
float Smoke_threshold = 300;
float H2_threshold = 300;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  Serial.println("Calibrating MQ-2 in clean air...");
  calibrateSensor();
  Serial.print("Calibration done. Ro = ");
  Serial.println(Ro);
  Serial.println("Starting gas readings...");
  delay(2000);
}

void loop() {
  float sensorValue = readSensor();
  float Rs = (5.0 - sensorValue) / sensorValue * RL; // Sensor resistance
  float ratio = Rs / Ro;

  // MQ-2 log-log formula from datasheet
  float lpg = pow(10, ((log10(ratio) - -0.47) / -0.42));
  float smoke = pow(10, ((log10(ratio) - 0.21) / -0.59));
  float h2 = pow(10, ((log10(ratio) - 0.62) / -0.48));

  // Print readings (decimals allowed)
  Serial.print("Rs/Ro: "); Serial.print(ratio, 2);
  Serial.print("  LPG: "); Serial.print(lpg, 2);
  Serial.print(" ppm  Smoke: "); Serial.print(smoke, 2);
  Serial.print(" ppm  H2: "); Serial.println(h2, 2);

  // Alert if any gas exceeds threshold
  if (lpg > LPG_threshold || smoke > Smoke_threshold || h2 > H2_threshold) {
    digitalWrite(ledPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
    digitalWrite(buzzerPin, LOW);
  }

  delay(1000);
}

// --------------------------
// Functions
// --------------------------
float readSensor() {
  float val = 0;
  for(int i=0; i<10; i++){
    val += analogRead(mqPin);
    delay(10);
  }
  val /= 10.0;
  return val * (5.0 / 1023.0); // Convert ADC to voltage
}

void calibrateSensor() {
  float val = 0;
  Serial.println("Place sensor in clean air and wait...");
  for(int i=0; i<calibrationSamples; i++){
    val += readSensor();
    delay(sampleInterval);
  }
  val /= calibrationSamples;
  float Rs_air = (5.0 - val) / val * RL;
  Ro = Rs_air / 9.8; // Clean air factor
}