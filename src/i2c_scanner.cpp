
// I2C Scanner for ESP32-S3 with custom pins
// Connect your I2C device and open the serial monitor at 115200 baud
#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 39
#define SCL_PIN 40
/*
 * I2C Scanner
 * Scans the I2C bus for devices and prints their addresses to the serial monitor.
 * 
 * This code is adapted for ESP32-S3 using custom SDA and SCL pins.
 * Make sure to connect your I2C device to the correct pins (SDA to GPIO39, SCL to GPIO40).
 * 
 * To use:
 * 1. Upload this code to your ESP32-S3.
 * 2. Open the Serial Monitor at 115200 baud.
 * 3. The scanner will print the addresses of any detected I2C devices every 3 seconds.
 */

 /*
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\nI2C Scanner starting...");
  Wire.begin(SDA_PIN, SCL_PIN);
}

void loop() {

  int nDevices = 0;
  Serial.println("Scanning...");
  uint8_t error;
  uint8_t address;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16) Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  delay(3000); // Wait 3 seconds before next scan
}
*/