#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <MycilaWebSerial.h>
#include <ArduinoOTA.h>

class WiFiHandler {
public:
  WiFiHandler();
  void begin(const char* ssid, const char* password);
    WebSerial webSerial;      
private:
  AsyncWebServer server;           // now owned by the class
};

#endif // WIFI_HANDLER_H
