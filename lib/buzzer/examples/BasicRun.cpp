#include <Arduino.h>
#include <buzzer.h>

const int buz = 33;
const int buttonPin = 2; // Pin for the button
buzzer myBuzzer(buz);
unsigned long duration = 100;

unsigned long lastPressTime = 0;
unsigned long doublePressDelay = 400; // Max gap for double press
bool buttonState = false;
bool lastButtonState = false;
bool waitForSecondPress = false;

void singlePressAction() {
  Serial.println("Single Press Detected!");
  // your single press logic here
  myBuzzer.errorBeep();
}

void doublePressAction() {
  Serial.println("Double Press Detected!");
  // your double press logic here
  myBuzzer.startupTone();
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); //button
}

void loop() {  
//   myBuzzer.stopSound();

  myBuzzer.update(); // call this in every loop to handle tone playing
  buttonState = !digitalRead(buttonPin); // Active LOW

  unsigned long currentTime = millis();

  // Detect rising edge (button just pressed)
  if (buttonState && !lastButtonState) {
    if (waitForSecondPress && (currentTime - lastPressTime <= doublePressDelay)) {
      doublePressAction();
      waitForSecondPress = false;
    } else {
      waitForSecondPress = true;
      lastPressTime = currentTime;
    }
  }

  // Timeout to consider it a single press
  if (waitForSecondPress && (currentTime - lastPressTime > doublePressDelay)) {
    singlePressAction();
    waitForSecondPress = false;
  }

  lastButtonState = buttonState;
}
