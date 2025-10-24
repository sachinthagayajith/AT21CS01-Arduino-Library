/*
 * AT21CS01 Basic Read Write Example
 * 
 * This example demonstrates basic operations with AT21CS01 EEPROM:
 * - Device detection
 * - Reading device serial number
 * - Writing data to EEPROM
 * - Reading data back and verification
 * 
 * Created by Sachintha Gayajith
 * 
 * Hardware Connections:
 * - Connect AT21CS01 SI/O pin to ESP32 GPIO13 (or change pin in code)
 * - Connect 2.1kΩ pull-up resistor between SI/O and 3.3V
 * - Connect AT21CS01 GND to ESP32 GND
 */

#include "AT21CS01.h"

// Initialize AT21CS01 with SI/O on GPIO13
AT21CS01 eeprom(13);

// Test data
uint8_t writeData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
uint8_t readData[8];
uint8_t serialNumber[8];

void setup() {
  Serial.begin(115200);
  Serial.println("AT21CS01 Basic Read/Write Test");
  
  // Reset and check for device presence
  if(eeprom.reset() == 0x00) {
    Serial.println("Device found!");
    
    // Read serial number
    eeprom.ScanDeviceAddress();
    Serial.print("Device Address: 0x");
    Serial.println(serialNumber[0], HEX);
    
    // Write test data to address 0x20
    Serial.println("\nWriting test data...");
    eeprom.eepromWrite(0xA0, 0x20, writeData, 8);
    
    // Read back the data
    Serial.println("Reading back data...");
    eeprom.randomRead(0xA0, 0x20, readData, 8);
    
    // Verify the data
    bool dataMatch = true;
    Serial.println("\nVerification:");
    for(int i = 0; i < 8; i++) {
      Serial.print("Addr 0x");
      Serial.print(0x20 + i, HEX);
      Serial.print(": Written=0x");
      Serial.print(writeData[i], HEX);
      Serial.print(", Read=0x");
      Serial.println(readData[i], HEX);
      
      if(writeData[i] != readData[i]) {
        dataMatch = false;
      }
    }
    
    Serial.println(dataMatch ? "\nVerification PASSED!" : "\nVerification FAILED!");
  } else {
    Serial.println("Device not found! Check connections.");
  }
}

void loop() {
  // Empty loop
}