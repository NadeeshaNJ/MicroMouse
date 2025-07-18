// #include <Arduino.h>
// #include <buzzer.h>

// const int buz = 33;
// const int buttonPin = 2; // Pin for the button
// buzzer myBuzzer(buz);

// unsigned long lastPressTime = 0;
// unsigned long doublePressDelay = 1000; // Max gap for double press
// bool buttonState = false;
// bool lastButtonState = false;
// bool waitForSecondPress = false;

// void singlePressAction() {
//   Serial.println("Single Press Detected!");
//   // your single press logic here
//   myBuzzer.beepOnce();
// }

// void doublePressAction() {
//   Serial.println("Double Press Detected!");
//   // your double press logic here
//   myBuzzer.beepTwice();
// }

// void setup() {
//     Serial.begin(115200);
//     pinMode(buttonPin, INPUT_PULLDOWN); //make sure to use INPUT_PULLDOWN if you want the button to be active LOW(necessary)
// }

// void loop() {  
// //   myBuzzer.stopSound();

//   myBuzzer.update(); // call this in every loop to handle tone playing

//   buttonState = digitalRead(buttonPin);
//   if(buttonState == HIGH) {
//     Serial.println("Button"); // Play a tone when button is pressed
//   }

//   unsigned long currentTime = millis();

//   // Detect rising edge (button just pressed)
//   if (buttonState && !lastButtonState) {
//     if (waitForSecondPress && (currentTime - lastPressTime <= doublePressDelay)) {
//       doublePressAction();
//       waitForSecondPress = false;
//     } else {
//       waitForSecondPress = true;
//       lastPressTime = currentTime;
//     }
//   }

//   // Timeout to consider it a single press
//   if (waitForSecondPress && (currentTime - lastPressTime > doublePressDelay)) {
//     singlePressAction();
//     waitForSecondPress = false;
//   }

//   lastButtonState = buttonState;
// }

#include <Arduino.h>
#include <button.h>

button myButton(2);  // GPIO 2

void onSinglePress() {
  Serial.println("Single Press");
}

void onDoublePress() {
  Serial.println("Double Press");
}

void setup() {
  Serial.begin(115200);
  myButton.setSinglePressCallback(onSinglePress);
  myButton.setDoublePressCallback(onDoublePress);
}

void loop() {
  myButton.update();
}
