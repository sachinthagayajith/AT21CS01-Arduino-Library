/*
 * AT21CS01 Load Cell Calibration Example
 * 
 * This example shows how to store and retrieve load cell calibration data
 * using the AT21CS01 EEPROM. It demonstrates a practical application
 * developed during internship at #S Fabrication Company.
 * 
 * Created by Sachintha Gayajith
 * 
 * Hardware Connections:
 * - Connect AT21CS01 SI/O pin to ESP32 GPIO13 (or change pin in code)
 * - Connect 2.1kΩ pull-up resistor between SI/O and 3.3V
 * - Connect AT21CS01 GND to ESP32 GND
 * Optional Display:
 * - LCD display for status (see pin definitions below)
 */

#include "AT21CS01.h"
#include "LiquidCrystal.h"

// Pin Definitions
const int EEPROM_PIN = 13;    // AT21CS01 SI/O pin
const int LCD_RS = 14;
const int LCD_EN = 27;
const int LCD_D4 = 19;
const int LCD_D5 = 18;
const int LCD_D6 = 17;
const int LCD_D7 = 16;

// Initialize devices
AT21CS01 eeprom(EEPROM_PIN);
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Calibration data structure
struct CalibrationData {
  uint8_t version;           // Data structure version
  uint8_t numPoints;         // Number of calibration points
  uint32_t serialNumber;     // Load cell serial number
  float calibrationPoints[5];// Calibration values
};

CalibrationData calData;

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  
  Serial.println("AT21CS01 Load Cell Calibration Example");
  lcd.print("Load Cell Cal");
  
  // Check for EEPROM
  if(eeprom.reset() != 0x00) {
    Serial.println("EEPROM not found!");
    lcd.setCursor(0, 1);
    lcd.print("EEPROM Error");
    while(1);
  }
  
  // Example calibration data
  calData.version = 1;
  calData.numPoints = 3;
  calData.serialNumber = 12345678;
  calData.calibrationPoints[0] = 0.0;    // Zero point
  calData.calibrationPoints[1] = 1000.0;  // Mid point
  calData.calibrationPoints[2] = 2000.0;  // Full scale
  
  // Write calibration data
  Serial.println("\nWriting calibration data...");
  lcd.setCursor(0, 1);
  lcd.print("Writing Cal...");
  
  // Write data in chunks since our struct is larger than 8 bytes
  uint8_t *dataPtr = (uint8_t*)&calData;
  
  // Write first 8 bytes
  eeprom.eepromWrite(0xA0, 0x00, dataPtr, 8);
  delay(10);  // Wait for write to complete
  
  // Write next 8 bytes
  eeprom.eepromWrite(0xA0, 0x08, dataPtr + 8, 8);
  delay(10);
  
  // Write remaining bytes
  eeprom.eepromWrite(0xA0, 0x10, dataPtr + 16, 8);
  
  // Read and verify
  CalibrationData readData;
  uint8_t *readPtr = (uint8_t*)&readData;
  
  Serial.println("\nReading calibration data...");
  lcd.setCursor(0, 1);
  lcd.print("Reading Cal... ");
  
  // Read data in chunks
  eeprom.randomRead(0xA0, 0x00, readPtr, 8);
  eeprom.randomRead(0xA0, 0x08, readPtr + 8, 8);
  eeprom.randomRead(0xA0, 0x10, readPtr + 16, 8);
  
  // Display results
  Serial.println("\nCalibration Data:");
  Serial.print("Version: "); Serial.println(readData.version);
  Serial.print("Points: "); Serial.println(readData.numPoints);
  Serial.print("Serial: "); Serial.println(readData.serialNumber);
  Serial.println("Calibration Points:");
  for(int i = 0; i < readData.numPoints; i++) {
    Serial.print(i); Serial.print(": ");
    Serial.println(readData.calibrationPoints[i]);
  }
  
  lcd.clear();
  lcd.print("Cal Verified");
}

void loop() {
  // Empty loop
}