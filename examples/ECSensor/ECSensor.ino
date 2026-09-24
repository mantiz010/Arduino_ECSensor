/*
 * Legacy ECSensor example.
 *
 * This example uses the original ECSensor API and the hard-coded
 * ATmega328P pin mapping from ECSensor.h:
 *   CapPos: D2
 *   CapNeg: D4
 *   EC:     D7
 *
 * For ESP32 family boards use the ESP32_ECSensorV2 example instead.
 */

#include <ECSensor.h>

ECSensor sensor;

void setup() {
  Serial.begin(115200);
  sensor.begin();
}

void loop() {
  Serial.print("Discharge cycles: ");
  Serial.println(sensor.readSensor());
  delay(2000);
}
