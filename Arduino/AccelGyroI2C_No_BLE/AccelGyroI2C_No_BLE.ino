#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <math.h>

// I2C slave address
#define SLAVE_ADDRESS 8  

// IMU and timing definitions
#define WAIT_TIME 100     // How often to update (in milliseconds)
float x, y, z;
int angleX = 0;
int angleY = 0;
unsigned long previousMillis = 0;

// IMU data variables (dummy defaults; tiltX and tiltY will be updated from sensor)
float tiltX = 1.23, tiltY = 2.34, tiltZ = 3.45;
float angularVelX = 0.12, angularVelY = 0.23, angularVelZ = 0.34;
float cartX = 10.1, cartY = 20.2, cartZ = 30.3;

void setup() {
  Serial.begin(9600);

  // Initialize IMU sensor.
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  // Initialize I2C as a slave.
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
}

void loop() {
  // Update IMU reading at defined intervals.
  unsigned long currentMillis = millis();
  if (IMU.accelerationAvailable() && (currentMillis - previousMillis >= WAIT_TIME)) {
    previousMillis = currentMillis;
    IMU.readAcceleration(x, y, z);
    // Calculate tilt angles in degrees.
    angleX = atan2(x, sqrt(y * y + z * z)) * 180 / PI;
    angleY = atan2(y, sqrt(x * x + z * z)) * 180 / PI;
    tiltX = angleX;
    tiltY = angleY;
    
    // Print the IMU tilt angles along with the BLE distance value.
    Serial.print("TiltX: ");
    Serial.print(tiltX);
    Serial.print(" TiltY: ");
    Serial.println(tiltY);
  }
}

// I2C request event: sends nine floats (tilt, angular velocity, cartesian coordinates).
void requestEvent() {
  sendFloat(tiltX);
  sendFloat(tiltY);
  sendFloat(tiltZ);
  sendFloat(angularVelX);
  sendFloat(angularVelY);
  sendFloat(angularVelZ);
  sendFloat(cartX);
  sendFloat(cartY);
  sendFloat(cartZ);
}

// Helper function to send a float value as bytes over I2C.
void sendFloat(float value) {
  byte *data = (byte*)&value;
  for (int i = 0; i < sizeof(float); i++) {
    Wire.write(data[i]);
  }
}
