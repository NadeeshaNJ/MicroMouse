#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <MadgwickAHRS.h> //MadgwickAHRS by PaulStoffregen

MPU9250_asukiaaa imu;
Madgwick filter;

unsigned long lastUpdate = 0;
float gx, gy, gz, ax, ay, az;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.setWire(&Wire);
  imu.beginAccel();
  imu.beginGyro();

  Serial.println("Starting IMU...");
}

void loop() {
  if (imu.accelUpdate() == 0 && imu.gyroUpdate() == 0) {
    // Read raw data
    ax = imu.accelX();
    ay = imu.accelY();
    az = imu.accelZ();
    gx = imu.gyroX() * DEG_TO_RAD;  // Convert to rad/s
    gy = imu.gyroY() * DEG_TO_RAD;
    gz = imu.gyroZ() * DEG_TO_RAD;

    float dt = (millis() - lastUpdate) / 1000.0f;
    lastUpdate = millis();

    // Update filter (no mag input!)
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // Get orientation
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();  // Relative yaw (starts at 0)

    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.println(yaw);  // relative yaw
  }

  delay(10);
}
