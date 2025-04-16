#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h> 


TFLI2C com_lidar;

int16_t  tfDist;
int16_t  tfAddr = TFL_DEF_ADR;

void setup() {
  Serial.begin(9600);
  Wire.begin();

}

void loop() {
  if(com_lidar.getData(tfDist, tfAddr)){
        Serial.println(String(tfDist)+" cm / " + String(tfDist/2.54)+" inches");
    }
    delay(50);
}
