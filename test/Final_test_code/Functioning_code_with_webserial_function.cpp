#include <WiFiHandler.h>
#include <Arduino.h>
#include <VL6180XManagerV2.h>
//#include <Floodfill.h>
#include <MotorPIDbyNJ.h>
#include <RobotNavigatorV2.h>
#include <GyroPID.h>

//#define Serial OTA.webSerial

// WiFi credentials
const char* ssid = "Dialog NNJ";
const char* password = "Mixtures";
// Web server and WebSerial
WiFiHandler OTA;
int count = 0;
int last = millis();

int xshutPins[] = {32, 17, 16, 15, 4};
int sensorCorrections[] = { 6, 16, 0, 43, 26};  // mm to subtract from each sensor
VL6180XManagerV2 sensorGroup(xshutPins, 5, sensorCorrections);

//Floodfill solveMaze;
//Floodfill solveMaze;
int dist = 0;
MotorPIDbyNJ leftMotor(25, 26, 18, 5);
MotorPIDbyNJ rightMotor(14, 27, 19, 23);
GyroPID imuController;
RobotNavigatorV2 Motors(&leftMotor, &rightMotor, &imuController);
// Motors.setSensorGroup(&sensorGroup); // moved to setup()
void updateLeftEncoder() { leftMotor.updateEncoder(); }
void updateRightEncoder() { rightMotor.updateEncoder(); }

int row = 0;
int col = 0;
int facingDirection = 0;

int nextMove = 0; // 0 = North, 1 = East, 2 = South, 3 = West

int lastMove = -1;
bool justFinishedMove = false;
void setup() {
  Serial.begin(115200);
  // Use explicit I2C pins for ESP32 (SDA, SCL).
  // This prevents pin muxing issues that can occur when WiFi is enabled.
  Wire.begin(21, 22);
  Wire.setClock(400000);
  sensorGroup.begin();
  imuController.begin();
  
  OTA.begin(ssid, password);

  Motors.setSensorGroup(&sensorGroup);
  
  //solveMaze.setThreshhold(80);

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  leftMotor.setPID(3, 0.05, 0.4, 8);
  rightMotor.setPID(3, 0.05, 0.4, 8);

}

bool testMoveDone = false;
void loop() {
  // Must call frequently so OTA works
  ArduinoOTA.handle();

  // Read sensors periodically (non-blocking in the manager implementation)
  static unsigned long lastSensorPrint = 0;
  if (millis() - lastSensorPrint > 1000) {
    // readAll() returns a vector<int> of distances for V2 manager
    std::vector<int> distances = sensorGroup.readAll();

    // Print distances to both Serial and WebSerial (if available)
    Serial.print("Distances:");
    for (size_t i = 0; i < distances.size(); ++i) {
      Serial.print(' ');
      Serial.print(distances[i]);
    }
    Serial.println();

    // Use OTA.webSerial if it's connected; guard in case it's not available
    // (WiFiHandler::webSerial is expected to be set up in OTA.begin())
    #ifdef ARDUINO
    // try-catch not available; just print — WebSerial prints safely if initialized
    OTA.webSerial.print("Distances:");
    for (size_t i = 0; i < distances.size(); ++i) {
      OTA.webSerial.print(' ');
      OTA.webSerial.print(distances[i]);
    }
    OTA.webSerial.println();
    #endif

    lastSensorPrint = millis();
  }

  // Run a single short test movement once, non-blocking
  if (!testMoveDone) {
    Serial.println("Starting short test movement...");
    
    Motors.turnRight();
    Motors.moveForward();
    Motors.turnRight();
    Motors.moveForward();
    Motors.turnRight();
    testMoveDone = true;
    Serial.println("Test movement queued");
  }

  delay(10);
}
