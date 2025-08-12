#include <VL6180XManagerV2.h>

int xshutPins[] = {32, 17, 16, 15, 4};
VL6180XManagerV2 sensorGroup(xshutPins, 3);

void setup() {
    Wire.begin();
    Serial.begin(115200);
    sensorGroup.begin();
}

void loop() {
    std::vector<int> distances = sensorGroup.readAll();

    for (int i = 0; i < distances.size(); i++) {
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": Distance = ");
        Serial.print(distances[i]); //distance = -1 for errors
        Serial.println(" mm");
    }

    delay(100); // Adjust read rate
}
