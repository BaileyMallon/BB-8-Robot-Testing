#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <math.h>

// BLE UUIDs (should match the peripheral’s service and characteristic UUIDs)
const char* serviceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* characteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

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

// Global variable to store BLE distance reading (as received from the peripheral)
String bleDistance = "0";

// BLE central variables
BLEDevice peripheralDevice;
BLECharacteristic distanceCharacteristic;

void setup() {
  Serial.begin(9600);

  // Initialize BLE in central mode.
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  Serial.println("BLE Central initialized");

  // Start scanning for peripherals advertising the desired service UUID.
  BLE.scanForUuid(serviceUuid);

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
  // Poll BLE events.
  BLE.poll();
  
  // If not connected to a BLE peripheral, try to find and connect.
  if (!peripheralDevice || !peripheralDevice.connected()) {
    BLEDevice foundDevice = BLE.available();
    if (foundDevice) {
      // Check that the device has the expected name.
      if (foundDevice.hasLocalName() && (String)foundDevice.localName() == "Dome (Peripheral)") {
        BLE.stopScan();
        Serial.print("Connecting to ");
        Serial.println(foundDevice.localName());
        if (foundDevice.connect()) {
          Serial.println("Connected!");
          // Discover the service and then obtain the distance characteristic.
          if (foundDevice.discoverService(serviceUuid)) {
            distanceCharacteristic = foundDevice.characteristic(characteristicUuid);
            if (!distanceCharacteristic) {
              Serial.println("Distance characteristic not found!");
              foundDevice.disconnect();
            }
          } else {
            Serial.println("Service not found!");
            foundDevice.disconnect();
          }
          peripheralDevice = foundDevice;
        } else {
          Serial.println("Failed to connect!");
        }
      }
    }
  } else {
    // If connected, try to read the distance characteristic.
    if (distanceCharacteristic.canRead()) {
      String newDistance = String((const char*) distanceCharacteristic.value());
      if (newDistance.length() > 0) {
        bleDistance = newDistance;
      }
    }
  }

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
    Serial.print(tiltY);
    Serial.print(" BLE Distance: ");
    Serial.println(bleDistance);
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
