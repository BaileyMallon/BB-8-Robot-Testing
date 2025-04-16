#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <ArduinoBLE.h>

// Define custom service and characteristic UUIDs (must match central's UUIDs)
#define SERVICE_UUID         "19B10000-E8F2-537E-4F6C-D104768A1214"
#define CHARACTERISTIC_UUID  "19B10001-E8F2-537E-4F6C-D104768A1214"

TFLI2C com_lidar;

int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR;
int tfDistInt;  // Variable to store the sensor distance as an integer

// Create a BLE service and a BLE string characteristic (max length 20)
BLEService tfDistService(SERVICE_UUID);
BLEStringCharacteristic tfDistCharacteristic(CHARACTERISTIC_UUID, BLERead | BLENotify, 20);

// Non-blocking update variables
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 50; // Update every 50 ms

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open
  Wire.begin();

  // Initialize BLE in peripheral mode.
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setDeviceName("NanoBLE_Peripheral");
  BLE.setLocalName("NanoBLE_Peripheral");

  // Set up BLE service and characteristic.
  BLE.setAdvertisedService(tfDistService);
  tfDistService.addCharacteristic(tfDistCharacteristic);
  BLE.addService(tfDistService);

  // Initialize the characteristic with an empty string.
  tfDistCharacteristic.writeValue("100");

  // Start advertising so centrals can find this device.
  BLE.advertise();
  Serial.println("BLE Peripheral started, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    while (central.connected()) {
      unsigned long currentMillis = millis();
      if (currentMillis - lastUpdateTime >= updateInterval) {
        lastUpdateTime = currentMillis;
        // Update distance if new sensor data is available.
        if (com_lidar.getData(tfDist, tfAddr)) {
          tfDistInt = (int)tfDist;  // Convert sensor reading to int
          String distanceStr = String(tfDistInt);  // Convert to string
          // Update the BLE characteristic with the new string.
          tfDistCharacteristic.writeValue(distanceStr);
          Serial.println(distanceStr + " cm / " + String(tfDistInt / 2.54) + " inches");
        }
      }
      BLE.poll();  // Process BLE events
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
  BLE.poll();  // Continue processing BLE events when no central is connected.
}
