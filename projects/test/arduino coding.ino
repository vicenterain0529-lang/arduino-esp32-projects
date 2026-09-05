int redPin = 3;      // PWM pin
int butPin = 4;
int yellowPin = 5;   // PWM pin
int greenPin = 6;    // PWM pin - FIX 1: Added green pin!

bool ledOn = false;
int lastButtonState = HIGH;

int brightness = 0;
int fadeAmount = 5;

unsigned long previousTime = 0;
const unsigned long fadeInterval = 20;

int state = 0;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);  
  pinMode(greenPin, OUTPUT);   // FIX 1: Set green pin as an OUTPUT
  pinMode(butPin, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  // Read button
  int butState = digitalRead(butPin);

  // Detect new button press (only if we are currently in State 0 / Idle)
  if (state == 0 && butState == LOW && lastButtonState == HIGH) {
    ledOn = true;
    state = 1;

    brightness = 0;
    fadeAmount = 5;

    previousTime = millis();

    Serial.println("Fade Started");
    delay(50); // debounce
  }

  // -------------------------
  // STATE 1 : Fade Red LED
  // -------------------------
  if (state == 1) {
    // Update brightness every 20 ms
    if (millis() - previousTime >= fadeInterval) {
      previousTime = millis();

      brightness += fadeAmount;
      analogWrite(redPin, brightness);

      // Reverse at maximum brightness
      if (brightness >= 255) {
        brightness = 255;
        fadeAmount = -5;
      }

      // Finished fading back down
      if (brightness <= 0) {
        brightness = 0;
        analogWrite(redPin, 0);
        
        state = 2;                // Move to State 2 (The Pause)
        previousTime = millis();  // Record the exact time the pause started!
        Serial.println("Red finished. Pausing for 3 seconds...");
      }
    }
  }

  // -------------------------
  // STATE 2 : Wait 3 Seconds
  // -------------------------
  if (state == 2) {
    // Check if 3 seconds (3000 ms) have passed since the Red LED finished
    if (millis() - previousTime >= 3000) {
      state = 3;                // Move to State 3 (Yellow Fade)
      brightness = 0;           // Reset brightness to start at 0
      fadeAmount = 5;           // Reset fade direction to go up
      previousTime = millis();  // Reset timer to start the fade interval
      Serial.println("Yellow Fade Started");
    }
  }

  // -------------------------
  // STATE 3 : Fade Yellow LED
  // -------------------------
  if (state == 3) {
    // Update brightness every 20 ms
    if (millis() - previousTime >= fadeInterval) {
      previousTime = millis();

      brightness += fadeAmount;
      analogWrite(yellowPin, brightness);

      // Reverse at maximum brightness
      if (brightness >= 255) {
        brightness = 255;
        fadeAmount = -5;
      }

      // Finished fading back down
      if (brightness <= 0) {
        brightness = 0;
        analogWrite(yellowPin, 0);

        state = 4;                // Move to State 4 (The Second Pause)
        previousTime = millis();  // Record the exact time this pause started!
        Serial.println("Yellow finished. Next color in 3 seconds.");
      }
    }
  }

  // -------------------------
  // STATE 4 : Wait 3 Seconds
  // -------------------------
  if (state == 4) {
    // Check if 3 seconds (3000 ms) have passed since the yellow LED finished
    if (millis() - previousTime >= 3000) {
      state = 5;                // Move to State 5 (Green Fade)
      brightness = 0;           // Reset brightness to start at 0
      fadeAmount = 5;           // Reset fade direction to go up
      previousTime = millis();  // Reset timer to start the fade interval
      Serial.println("Green Fade Started");
    }
  }

  // -------------------------
  // STATE 5 : Fade Green LED
  // -------------------------
  if (state == 5) {
    if (millis() - previousTime >= fadeInterval) {
      previousTime = millis(); // FIX 2: Added this so the timer actually resets!

brightness += fadeAmount;
analogWrite(greenPin, brightness); // FIX 1: Changed yellowPin to greenPin

      // Reverse at maximum brightness
      if (brightness >= 255) {
        brightness = 255;
        fadeAmount = -5;
      }

      // Finished fading back down
      if (brightness <= 0) {
        brightness = 0;
        analogWrite(greenPin, 0); // FIX 1: Changed yellowPin to greenPin

        state = 0;                // FIX 4: Go back to State 0 so the sequence completes
        ledOn = false;            // Reset the LED flag
        Serial.println("Green finished. Sequence complete! Ready for next button press.");
      }
    }
  }

  // Save button state
  lastButtonState = butState; // FIX 3: Cleaned up the brackets so this line compiles inside loop()
}