// Keep main minimal: implement the extern primitives expected by Floodfill module
#include <Arduino.h>
#include <Wire.h>
#include <vector>

#include <VL6180XManagerV2.h>
#include <MotorPIDbyNJ.h>
#include <RobotNavigatorV2.h>
#include <GyroPID.h>
#include <button.h>

// C hooks and step from Floodfill (implemented in lib/Floodfill/src/Floodfill_SearchRun.cpp)
extern "C" void ffSetRunMode(int mode);
extern "C" bool ffAtCenter();
extern "C" bool ffAtStart();
void runFloodfillStep();

// GPIO2 button config (polled, no interrupts)
#define BTN_PIN 2
#define MULTICLICK_GAP_MS 500
#define LONG_PRESS_MS 700

// Hardware instances
int xshutPins[] = {32, 17, 16, 15, 4};
int sensorCorrections[] = {6, 16, 0, 43, 26}; // mm to subtract from each sensor
VL6180XManagerV2 sensorGroup(xshutPins, 5, sensorCorrections);

MotorPIDbyNJ leftMotor(25, 26, 18, 5);
MotorPIDbyNJ rightMotor(14, 27, 19, 23);
GyroPID imuController;
RobotNavigatorV2 Motors(&leftMotor, &rightMotor, &imuController);

// Encoder ISRs
void updateLeftEncoder() { leftMotor.updateEncoder(); }
void updateRightEncoder() { rightMotor.updateEncoder(); }

// --- Extern hooks required by Floodfill module ---
void moveForward() { Motors.moveForward(); }
void turnLeft() { Motors.turnLeft(); }
void turnRight() { Motors.turnRight(); }
std::vector<int> getDistances() { return sensorGroup.readAll(); }

// Blocking selection using the polling-based button library on GPIO2
volatile int g_selectedMode = 0; // 1=SearchOnly, 2=Double, 3=Quick

int selectRunModeBlocking() {
  button modeBtn(BTN_PIN, MULTICLICK_GAP_MS);

  // Wire callbacks to set the selection
  modeBtn.setSinglePressCallback([]() { g_selectedMode = 1; });
  modeBtn.setDoublePressCallback([]() { g_selectedMode = 2; });

  unsigned long start = millis();
  unsigned long holdStart = 0;

  Serial.println("Press button: 1=Search, 2=Double, Hold=Quick (2s timeout)...");

  while (millis() - start < 2000 && g_selectedMode == 0) {
    modeBtn.update();

    // Long-press detection for Quick
    int level = digitalRead(BTN_PIN); // library configured INPUT_PULLDOWN; HIGH means pressed
    if (level == HIGH) {
      if (holdStart == 0) holdStart = millis();
      if (millis() - holdStart >= LONG_PRESS_MS) { g_selectedMode = 3; break; }
    } else {
      holdStart = 0;
    }

    delay(5);
  }

  if (g_selectedMode == 0) g_selectedMode = 1; // default to SearchOnly
  return g_selectedMode;
}

void setup() {
  Serial.begin(115200);

  // Explicit I2C init helps avoid pin mux issues (WiFi, etc.)
  Wire.begin(21, 22);
  Wire.setClock(400000);
  sensorGroup.begin();
  imuController.begin();
  Motors.setSensorGroup(&sensorGroup);

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  // One-time mode selection using GPIO2 button (polled)
  int mode = selectRunModeBlocking();
  ffSetRunMode(mode);
  Serial.print("Selected mode: ");
  Serial.println(mode == 1 ? "SearchOnly" : mode == 2 ? "Double" : "Quick");
}

void loop() {
  // Non-blocking step from floodfill/doublesearch controller
  runFloodfillStep();

  // Optional: If you want to stop after reaching center in search-only
  // if (ffAtCenter()) { while (true) { delay(1000); } }

  delay(10);
}
