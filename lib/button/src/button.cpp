#include "button.h"

button::button(int pin, unsigned long doubleDelay) {
  this->pin = pin;
  pinMode(pin, INPUT_PULLDOWN);
  lastButtonState = false;
  waitForSecondPress = false;
  lastPressTime = 0;
  doublePressDelay = doubleDelay;
  singlePressCallback = nullptr;
  doublePressCallback = nullptr;
}

void button::setSinglePressCallback(void (*func)()) {
  singlePressCallback = func;
}

void button::setDoublePressCallback(void (*func)()) {
  doublePressCallback = func;
}

void button::update() {
  buttonState = digitalRead(pin); // active LOW

  unsigned long currentTime = millis();

  if (buttonState && !lastButtonState) {
    if (waitForSecondPress && (currentTime - lastPressTime <= doublePressDelay)) {
      if (doublePressCallback) doublePressCallback();
      waitForSecondPress = false;
    } else {
      waitForSecondPress = true;
      lastPressTime = currentTime;
    }
  }

  if (waitForSecondPress && (currentTime - lastPressTime > doublePressDelay)) {
    if (singlePressCallback) singlePressCallback();
    waitForSecondPress = false;
  }

  lastButtonState = buttonState;
}
