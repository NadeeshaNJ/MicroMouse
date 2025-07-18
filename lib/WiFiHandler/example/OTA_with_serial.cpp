#include <Arduino.h>
#include <WiFiHandler.h>

// WiFi credentials
const char* ssid = "Dialog NNJ";
const char* password = "Mixtures";
// Web server and WebSerial
WiFiHandler OTA;
int count =0;
int last = millis();
void setup() {
  Serial.begin(115200);
  OTA.begin(ssid, password);

}

void loop() {
  ArduinoOTA.handle();
  if (millis() - last > 1000) {
    count++;
    OTA.webSerial.println("Hello World!");
    last = millis();
  }

}
