#include <WiFiHandler.h>
#include <Arduino.h>


WiFiHandler::WiFiHandler() : server(80){
  // Constructor implementation
}

void WiFiHandler::begin(const char* ssid, const char* password) {
Serial.println("Booting...");

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi failed. Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Connected to WiFi.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // WebSerial setup
  webSerial.onMessage([](const std::string& msg) {
    Serial.println(msg.c_str());
  });
  webSerial.begin(&server);
  webSerial.setBuffer(100);
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->redirect("/webserial");
  });
  server.begin();

  // OTA setup
  ArduinoOTA
    .onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready for OTA & WebSerial");
  Serial.print("WebSerial at: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/webserial");
}