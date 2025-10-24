/*********************************************************************
 * AT21CS01 EEPROM Test and Calibration Example
 * 
 * Created by: Sachintha Gayajith
 * During internship at: 3S Fabrication Company, Dankotuwa, Sri Lanka
 * Purpose: Calibration data storage and verification for load cell systems
 * 
 * This example demonstrates the usage of the AT21CS01 Arduino library,
 * which was developed based on the Dallas Semiconductor One-Wire protocol
 * principles. The original One-Wire library was studied and adapted to
 * meet the specific timing requirements of the AT21CS01 EEPROM.
 * 
 * Hardware Setup:
 * - ESP32 GPIO13 -> AT21CS01 SI/O pin (with 2.1kΩ pull-up to 3.3V)
 * - LCD display for status and data visualization
 * - Push button for triggering write operations
 * 
 * Features:
 * - Device detection and serial number reading
 * - Memory page read/write operations
 * - Load cell calibration data storage
 * - Data verification through LCD display
 * 
 * All timing parameters have been verified using oscilloscope measurements
 * to ensure compliance with AT21CS01 datasheet specifications.
 *********************************************************************/

#include "AT21CS01.h"
#include "LiquidCrystal.h"

const int rs = 14, en = 27, d4 = 19, d5 = 18, d6 = 17, d7 = 16;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
AT21CS01 ds(13);  //EEPROM connect with GPIO13

uint8_t erasepage[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  //sets page = 0xFF
uint8_t main_array_page_1_input_lcd[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
uint8_t main_array_page_1_output_lcd[8];
uint8_t serialNumber[8];    //store the serial umber(64 bit serial number)
uint8_t Manufacture_ID[8];  //store the Manufacture ID

#define ARRAY_SIZE 8
#define PUSH_BUTTON 15

//these variable use for srote vluve of reading from main memory
uint8_t Read_page_1[8];
uint8_t Read_page_2[8];
uint8_t Read_page_3[8];
uint8_t Read_page_4[8];
uint8_t Read_page_5[8];
uint8_t Read_page_6[8];
uint8_t Read_page_7[8];
uint8_t Read_page_8[8];
uint8_t Read_page_9[8];
uint8_t Read_page_10[8];
uint8_t Read_page_11[8];
uint8_t Read_page_12[8];
uint8_t Read_page_13[8];
uint8_t Read_page_14[8];
uint8_t Read_page_15[8];
uint8_t Read_page_16[8];

bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 5;

uint8_t UINT8_HEX_VALUVE;  //these variable use to read the serial input  load cell calibration data
uint8_t Version_Of_Data_Structure;
uint8_t No_Of_Calibration_Points;
uint32_t Factry_Serial_Number;

bool display_switch;
bool verification_switch;
bool compaire, LONGSHOW = 0, LONGSHOW1 = 0;

//Globle varable for comman prssing

String command1, command_v;

bool press_logic;
bool press, lcdclear2 = 1, lcdclear = 1;
bool verify_display;
void setup() {
  pinMode(PUSH_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);


  lcd.begin(16, 2);
  ds.Reset();  //Reset the EEPROM
  press = false;

  Serial.print("HE: help command");
}

void loop() {

  //ds.Reset();
  SERIAL_NUMBER_READ();
  if (serialNumber[0] == 0xA0) {
    lcdclear2 = 1;
    if (lcdclear == 1) {
      lcd.clear();
      lcdclear = 0;
    }
    lcd.setCursor(0, 0);
    lcd.print("Device Found");
    display_OUT(serialNumber, 1);
    delay(5);

    if (digitalRead(PUSH_BUTTON) == LOW) {
      SERIAL_NUMBER_READ();
      lcd.clear();
      ds.eepromWrite(0xA0, 0x20, main_array_page_1_input_lcd, 8);
      lcd.setCursor(0, 0);
      lcd.print("Writing...");
      lcd.setCursor(0, 1);
      lcd.print("12345678");
      delay(1000);
      lcd.clear();
      ds.randomRead(0xA0, 0x20, main_array_page_1_output_lcd, 8);
      lcd.setCursor(0, 0);
      lcd.print("Reading...");
      lcd.setCursor(0, 1);
      for (int i = 0; i < 8; i++) {
        lcd.print(main_array_page_1_output_lcd[i]);
        //lcd.print(" ");
      }
      delay(1000);
      lcd.clear();
      //compaire  =compareArrays(main_array_page_1_input_lcd, main_array_page_1_output_lcd, 8);
      if (compareArrays(main_array_page_1_input_lcd, main_array_page_1_output_lcd, 8)) {
        lcd.setCursor(0, 0);
        lcd.print("VERIFICATION:");
        lcd.setCursor(0, 1);
        lcd.print("PASS");
        //
        // delay(2);

        delay(50);
        LONGSHOW1 = 1;
        //delay(2);
        ds.eepromWrite(0xA0, 0x20, erasepage, 8);
        while (LONGSHOW1 == 1) {
          delay(2);
          ds.randomRead(0xB0, 0x00, serialNumber, 8);
          delay(2);
          if (serialNumber[0] != 0xA0) {
            LONGSHOW1 = 0;
            delay(2);
          }
        }


      } else {
        lcd.setCursor(0, 0);
        lcd.print("VERIFICATION:");
        lcd.setCursor(0, 1);
        lcd.print("FAIL");
        delay(2);
        LONGSHOW = 1;
        while (LONGSHOW == 1) {
          delay(2);
          ds.randomRead(0xB0, 0x00, serialNumber, 8);
          delay(2);
          if (serialNumber[0] == 0xA0) {
            delay(2);
            LONGSHOW = 0;
          }
        }
      }
    }
  }

  else {
    lcdclear = 1;
    if (lcdclear2 == 1) {
      lcd.clear();
      lcdclear2 = 0;
    }
    lcd.setCursor(0, 0);
    lcd.print("NO DEVICE");
    delay(5);
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\r');
    command.trim();  // Remove leading/trailing spaces

    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      command1 = command.substring(0, spaceIndex);    // "VR"
      command_v = command.substring(spaceIndex + 1);  // "1"
    } else {
      command1 = command;
      command_v = "";
    }

    processCommand(command1, command_v);
  }
}

//end void loop____________________________________________________________________________________________


// Function to process incoming serial commands
void processCommand(String command1x, String command_vx) {

  if (command1x == "RS") {
    ds.Reset();
    SERIAL_NUMBER_READ_1();
    Serial.print('\r');
  } else if (command1x == "ID") {
    ds.Reset();
    MANUFACTURE_ID_READ();
    Serial.print('\r');
  } else if (command1x == "RP") {
    ds.Reset();
    RESET_EEPROM();
    Serial.print('\r');
  } else if (command1x == "SC") {
    SCAN_DEVICE_ADDRESS();
    Serial.print('\r');
  } else if ((command1x == "VR") && (command_vx != "")) {

    int inputValue = command_vx.toInt();  //  convert string to int

    if (inputValue >= 0 && inputValue <= 255) {
      Version_Of_Data_Structure = (uint8_t)inputValue;
      ds.singleByteWrite(0xA0, 0x00, Version_Of_Data_Structure);
      Serial.print("write vertin number :");
      Serial.print(Version_Of_Data_Structure);

    } else {
      Serial.print("ERROR");
    }

    Serial.print('\r');
  }
  //Nomber of calibration point
  else if ((command1x == "CB") && (command_vx != "")) {
    int inputValue = command_vx.toInt();  //  convert string to int

    if (inputValue >= 0 && inputValue <= 255) {
      uint8_t No_of_cal_ponts = (uint8_t)inputValue;
      ds.singleByteWrite(0xA0, 0x01, No_of_cal_ponts);
      Serial.print("write On Of Calibration Point :");
      Serial.print(No_of_cal_ponts);

    } else {
      Serial.print("ERROR");
    }

    Serial.print('\r');
  }
  //Reserved for future use 0x02,0x03
  else if ((command1x == "F1") && (command_vx != "")) {
    int inputValue = command_vx.toInt();  //  convert string to int

    if (inputValue >= 0 && inputValue <= 255) {
      uint8_t Future_Used_1 = (uint8_t)inputValue;
      ds.singleByteWrite(0xA0, 0x02, Future_Used_1);
      Serial.print("write Future used 0x02 :");
      Serial.print(Future_Used_1);

    } else {
      Serial.print("ERROR");
    }

    Serial.print('\r');
  }
  //Reserved for furute use 0x03
  else if ((command1x == "F2") && (command_vx != "")) {
    int inputValue = command_vx.toInt();  //  convert string to int

    if (inputValue >= 0 && inputValue <= 255) {
      uint8_t Future_Used_2 = (uint8_t)inputValue;
      ds.singleByteWrite(0xA0, 0x03, Future_Used_2);
      Serial.print("write Future used 0x03 :");
      Serial.print(Future_Used_2);

    } else {
      Serial.print("ERROR");
    }

    Serial.print('\r');
  }
  //Factry Serial Number
  else if ((command1x == "FS") && (command_vx != "")) {
    uint32_t inputValue_FSI = strtoul(command_vx.c_str(), NULL, 10);  // base 10 conversion

    uint32_t Factry_Serial_Num = (uint32_t)inputValue_FSI;
    delay(5);
    uint8_t byte0 = (uint8_t)(Factry_Serial_Num & 0xFF);          // LSB: bits 0-7
    uint8_t byte1 = (uint8_t)((Factry_Serial_Num >> 8) & 0xFF);   // bits 8-15
    uint8_t byte2 = (uint8_t)((Factry_Serial_Num >> 16) & 0xFF);  // bits 16-23
    uint8_t byte3 = (uint8_t)((Factry_Serial_Num >> 24) & 0xFF);  // MSB: bits 24-31
    delay(5);
    Serial.print("Factry Serial Number: ");
    Serial.print(Factry_Serial_Num);


    ds.Reset();
    ds.singleByteWrite(0xA0, 0x04, byte0);
    ds.singleByteWrite(0xA0, 0x05, byte1);
    ds.singleByteWrite(0xA0, 0x06, byte2);
    ds.singleByteWrite(0xA0, 0x07, byte3);

    Serial.print('\r');
  }

  //MAXIMUM CAPACITY.......
  else if ((command1x == "MX") && (command_vx != "")) {
    uint32_t inputValue_MAX = strtoul(command_vx.c_str(), NULL, 10);  // base 10 conversion

    uint32_t MAX_CAPACITY = (uint32_t)inputValue_MAX;
    delay(5);
    uint8_t byte0 = (uint8_t)(MAX_CAPACITY & 0xFF);          // LSB: bits 0-7
    uint8_t byte1 = (uint8_t)((MAX_CAPACITY >> 8) & 0xFF);   // bits 8-15
    uint8_t byte2 = (uint8_t)((MAX_CAPACITY >> 16) & 0xFF);  // bits 16-23
    uint8_t byte3 = (uint8_t)((MAX_CAPACITY >> 24) & 0xFF);  // MSB: bits 24-31
    delay(5);
    Serial.print("Maximum Capacity: ");
    Serial.print(MAX_CAPACITY);


    ds.Reset();
    ds.singleByteWrite(0xA0, 0x08, byte0);
    ds.singleByteWrite(0xA0, 0x09, byte1);
    ds.singleByteWrite(0xA0, 0x0A, byte2);
    ds.singleByteWrite(0xA0, 0x0B, byte3);
    Serial.print('\r');
  }
  //CALIBRATION POINT 1 APPLIED LOAD IN NEWTONS
  else if ((command1x == "L1") && (command_vx != "")) {
    uint32_t inputValue_L1 = strtoul(command_vx.c_str(), NULL, 10);  // base 10 conversion
    if (inputValue_L1 <= 4294967295) {
      uint32_t Calibration_Point_1_Load = (uint32_t)inputValue_L1;
      delay(5);
      uint8_t byte0 = (uint8_t)(Calibration_Point_1_Load & 0xFF);          // LSB: bits 0-7
      uint8_t byte1 = (uint8_t)((Calibration_Point_1_Load >> 8) & 0xFF);   // bits 8-15
      uint8_t byte2 = (uint8_t)((Calibration_Point_1_Load >> 16) & 0xFF);  // bits 16-23
      uint8_t byte3 = (uint8_t)((Calibration_Point_1_Load >> 24) & 0xFF);  // MSB: bits 24-31

      delay(5);
      ds.singleByteWrite(0xA0, 0x0C, byte0);
      ds.singleByteWrite(0xA0, 0x0D, byte1);
      ds.singleByteWrite(0xA0, 0x0E, byte2);
      ds.singleByteWrite(0xA0, 0x0F, byte3);

      Serial.print("Calibration Point 1.Applied Load in Newtons: ");
      Serial.print(Calibration_Point_1_Load);



    } else {
      Serial.print("ERROR");
    }
    Serial.print('\r');
  }


  //CALIBRATION POINT 1 mV/V OUTPUT  @ applied load
  else if ((command1x == "V1") && (command_vx != "")) {
    float inputValue_V1 = command_vx.toFloat();
    Serial.print("Calibration point1.mV/V : ");
    Serial.print(inputValue_V1, 6);  // Print 10 decimal places

    union {
      float f1;
      uint32_t u1;

    } converter;

    converter.f1 = inputValue_V1;
    uint32_t raw = converter.u1;

    // Extract fields
    delay(5);
    uint32_t sign = (raw >> 31) & 0x1;
    delay(5);
    uint32_t exponent = (raw >> 23) & 0xFF;
    delay(5);
    uint32_t mantissa = raw & 0x7FFFFF;
    delay(5);
    //Serial.println(sign);
    //Serial.println(exponent);
    //Serial.println(mantissa);
    delay(5);
    uint8_t byte0 = (uint8_t)(raw & 0xFF);
    delay(5);                                      // LSB: bits 0-7
    uint8_t byte1 = (uint8_t)((raw >> 8) & 0xFF);  // bits 8-15
    delay(5);
    uint8_t byte2 = (uint8_t)((raw >> 16) & 0xFF);  // bits 16-23
    delay(5);
    uint8_t byte3 = (uint8_t)((raw >> 24) & 0xFF);  // MSB: bits 24-31
    delay(5);
    //Serial.print(byte3,HEX); Serial.print(byte2,HEX); Serial.print(byte1,HEX); Serial.println(byte0,HEX);

    ds.singleByteWrite(0xA0, 0x10, byte0);
    ds.singleByteWrite(0xA0, 0x11, byte1);
    ds.singleByteWrite(0xA0, 0x12, byte2);
    ds.singleByteWrite(0xA0, 0x13, byte3);

  }

  //CALIBRATION POINT 2.APPLIED LOAD IN NEWTONS
  else if ((command1x == "L2") && (command_vx != "")) {
    uint32_t inputValue_L2 = strtoul(command_vx.c_str(), NULL, 10);  // base 10 conversion
    if (inputValue_L2 <= 4294967295) {
      uint32_t Calibration_Point_2_Load = (uint32_t)inputValue_L2;
      delay(5);
      uint8_t byte0 = (uint8_t)(Calibration_Point_2_Load & 0xFF);          // LSB: bits 0-7
      uint8_t byte1 = (uint8_t)((Calibration_Point_2_Load >> 8) & 0xFF);   // bits 8-15
      uint8_t byte2 = (uint8_t)((Calibration_Point_2_Load >> 16) & 0xFF);  // bits 16-23
      uint8_t byte3 = (uint8_t)((Calibration_Point_2_Load >> 24) & 0xFF);  // MSB: bits 24-31

      delay(5);
      ds.singleByteWrite(0xA0, 0x14, byte0);
      delay(5);
      ds.singleByteWrite(0xA0, 0x15, byte1);
      delay(5);
      ds.singleByteWrite(0xA0, 0x16, byte2);
      delay(5);
      ds.singleByteWrite(0xA0, 0x17, byte3);

      Serial.print("calibration point 2 (Newton): ");
      Serial.print(Calibration_Point_2_Load);



    } else {
      Serial.print("ERROR");
    }
    Serial.print('\r');
  }

  //CALIBRATION POINT 2.mV/V OUTPUT APPLIED
  else if ((command1x == "V2") && (command_vx != "")) {
    float inputValue_V2 = command_vx.toFloat();
    Serial.print("Calibration point2.mV/V : ");
    Serial.print(inputValue_V2, 6);  // Print 10 decimal places

    union {
      float f2;
      uint32_t u2;

    } converter;

    converter.f2 = inputValue_V2;
    uint32_t raw2 = converter.u2;

    // Extract fields
    delay(5);
    uint32_t sign2 = (raw2 >> 31) & 0x1;
    delay(5);
    uint32_t exponent2 = (raw2 >> 23) & 0xFF;
    delay(5);
    uint32_t mantissa2 = raw2 & 0x7FFFFF;
    delay(10);
    //Serial.println(sign2);
    //Serial.println(exponent2);
    //Serial.println(mantissa2);
    delay(5);
    uint8_t byte0 = (uint8_t)(raw2 & 0xFF);
    delay(5);                                       // LSB: bits 0-7
    uint8_t byte1 = (uint8_t)((raw2 >> 8) & 0xFF);  // bits 8-15
    delay(5);
    uint8_t byte2 = (uint8_t)((raw2 >> 16) & 0xFF);  // bits 16-23
    delay(5);
    uint8_t byte3 = (uint8_t)((raw2 >> 24) & 0xFF);  // MSB: bits 24-31
    delay(5);
    //Serial.print(byte3,HEX); Serial.print(byte2,HEX); Serial.print(byte1,HEX); Serial.println(byte0,HEX);

    ds.singleByteWrite(0xA0, 0x18, byte0);
    delay(5);
    ds.singleByteWrite(0xA0, 0x19, byte1);
    delay(5);
    ds.singleByteWrite(0xA0, 0x1A, byte2);
    delay(5);
    ds.singleByteWrite(0xA0, 0x1B, byte3);
    delay(5);

  }


  //PAGE READ

  else if (command1x == "RD") {
    //Read_Memory_User();
  } else if (command1x == "RH") {
    ALL_PAGES_READ();
  }
  //READ
  else if (command1x == "W1") {
    Read_Version();
  } else if (command1x == "W2") {
    Read_Calibration_Points();
  } else if (command1x == "W3") {
    Read_Future_Use_1();
  } else if (command1x == "W5") {
    Read_Factry_Serial();
  } else if (command1x == "W6") {
    Read_Max_Capacity();
  } else if (command1x == "W7") {
    Read_Calibration_Point1_Load();
  } else if (command1x == "W8") {
    Read_Calibration_Point1_Out();
  } else if (command1x == "W9") {
    Read_Calibration_Point2_Load();
  } else if (command1x == "WA") {
    Read_Calibration_Point2_Out();
  } else if (command1x == "HE") {
    Help();
  }

  else {
    Serial.print("err");
  }






}  //end command

void display_OUT(uint8_t* data, int row) {
  // Create a buffer for all 16 hex characters plus null terminator
  char hexBuffer[17];  // 16 characters + null terminator
  char* bufPtr = hexBuffer;

  // Convert all 8 bytes to a single hex string
  for (int i = 0; i < 8; i++) {
    sprintf(bufPtr, "%02X", data[i]);
    bufPtr += 2;  // Move pointer 2 positions ahead
  }

  // Ensure string is null-terminated
  hexBuffer[16] = '\0';
  lcd.setCursor(0, row);
  lcd.print(hexBuffer);
}
//___________________________________________________________
// Function to compare two arrays
bool compareArrays(uint8_t arr1[], uint8_t arr2[], int size) {
  for (int i = 0; i < size; i++) {
    if (arr1[i] != arr2[i]) {
      return false;  // Arrays are different
    }
  }
  return true;  // Arrays are identical
}

void LCD_DISPLAY_OUT() {

  //if(device_found ==true)
  //{
  ds.Reset();
  SERIAL_NUMBER_READ();
  if (serialNumber[0] == 0xA0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Device Found");
    display_OUT(serialNumber, 1);
    delay(300);
  }
  if (serialNumber[0] == 0xFF || serialNumber[0] == 0x00 || serialNumber[0] != 0xA0) {
    //device_found ==false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DEVICE FAILD");
    //delay(200);
  }

}
//bool verify_display;
void DISPLAY_PUSH_BOUTON_FUNCTION() {
}
//**********************************************************************

byte addesbyte[1];
byte databyte[1];
void onebutewrite() {
  ds.singleByteWrite(0xA0, 0x48, 0xA5);  //BY THIS FUNCTIN WE CAN WRITE THE 8BIT VALUVE IN TO THE MAIN MEMMORY
  //Serial.println("single byte write page 10, 1st");
}
//____________________________________

void RESET_EEPROM() {
  uint8_t RESET_BIT = ds.reset();
  Serial.print("Reset pulse  :");
  Serial.print(RESET_BIT);
}

//end reset____________________________________________________________________________________

void SCAN_DEVICE_ADDRESS() {
  uint8_t DeviceAddress = ds.ScanDeviceAddress();

  Serial.print("device Address : ");
  Serial.print(DeviceAddress);
}
//end _________________________________________________________________________________________
void SERIAL_NUMBER_READ_1() {
  ds.randomRead(0xB0, 0x00, serialNumber, 8);  //read the 64-bit serial number*/ dont edit
                                               // Serial.print("serial Number: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(serialNumber[i], HEX);
    Serial.print(" ");
  }
  //Serial.println();
  //Serial.print('\r');
}
void SERIAL_NUMBER_READ() {
  ds.randomRead(0xB0, 0x00, serialNumber, 8);  //read the 64-bit serial number*/ dont edit
}
//_____________________________________________________________________________________

void MANUFACTURE_ID_READ() {
  ds.ManufactureIDRead(0xC1, Manufacture_ID, 3);  //read the 64-bit serial number*/
  Serial.print("Manufactute_ID: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(Manufacture_ID[i], HEX);
    Serial.print(" ");
  }
}

void ALL_PAGES_READ() {
  ds.randomRead(0xA0, 0x00, Read_page_1, 8);


  Serial.print("page 1 : ");
  for (int i = 0; i < 8; i++) {
    Serial.print(Read_page_1[i], HEX);
    Serial.print(" ");
  }



  ds.randomRead(0xA0, 0x08, Read_page_2, 8);

  Serial.print("Page 2");
  for (int i = 0; i < 8; i++) {
    Serial.print(Read_page_2[i], HEX);
    Serial.print(" ");
  }

  ds.randomRead(0xA0, 0x10, Read_page_3, 8);


  Serial.print(" page 3 : ");
  for (int i = 0; i < 8; i++) {
    Serial.print(Read_page_3[i], HEX);
    Serial.print(" ");
  }

  ds.randomRead(0xA0, 0x18, Read_page_4, 8);
  Serial.print(" page 4 : ");
  for (int i = 0; i < 8; i++) {
    Serial.print(Read_page_4[i], HEX);
    Serial.print(" ");
  }


  Serial.print('\r');
}

//1
void Read_Version() {
  ds.Reset();
  ds.randomRead(0xA0, 0x00, Read_page_1, 8);
  Serial.print("01: Version of data stracture : ");
  delay(5);
  Serial.print(Read_page_1[0]);
}
//2
void Read_Calibration_Points() {
  ds.randomRead(0xA0, 0x00, Read_page_1, 8);
  Serial.print("02: Nomber of calibraion points: ");
  Serial.print(Read_page_1[1]);
}
//3
void Read_Future_Use_1() {
  delay(5);
  ds.Reset();
  uint8_t Read_F[8];
  ds.randomRead(0xA0, 0x00, Read_F, 8);
  ds.Reset();
  Serial.print("03: Address 0x02:");
  delay(10);
  Serial.print(Read_F[2], HEX);
  delay(5);

  Serial.print("04: ddress 0x02:");
  Serial.print(Read_F[3]);
}
//4
void Read_Factry_Serial() {
  uint8_t ADD_0x04;
  uint8_t ADD_0x05;
  uint8_t ADD_0x06;
  uint8_t ADD_0x07;
  ds.Reset();
  ds.randomRead(0xA0, 0x00, Read_page_1, 8);
  ADD_0x04 = Read_page_1[4];
  delay(5);
  ADD_0x05 = Read_page_1[5];
  delay(5);
  ADD_0x06 = Read_page_1[6];
  delay(5);
  ADD_0x07 = Read_page_1[7];
  delay(5);

  uint32_t F_Serial_Number = ((uint32_t)ADD_0x07 << 24) | ((uint32_t)ADD_0x06 << 16) | ((uint32_t)ADD_0x05 << 8) | ((uint32_t)ADD_0x04);
  Serial.print("05: Factory Serial Number : ");
  Serial.print(F_Serial_Number);
}
//5
void Read_Max_Capacity() {
  ds.Reset();
  ds.randomRead(0xA0, 0x08, Read_page_2, 8);
  uint8_t ADD_0x08;
  uint8_t ADD_0x09;
  uint8_t ADD_0x0A;
  uint8_t ADD_0x0B;

  ADD_0x08 = Read_page_2[0];
  delay(5);
  ADD_0x09 = Read_page_2[1];
  delay(5);
  ADD_0x0A = Read_page_2[2];
  delay(5);
  ADD_0x0B = Read_page_2[3];
  delay(5);
  uint32_t LOAD_MAX_CAPACITY = ((uint32_t)ADD_0x0B << 24) | ((uint32_t)ADD_0x0A << 16) | ((uint32_t)ADD_0x09 << 8) | ((uint32_t)ADD_0x08);


  Serial.print("06.  Maximum Capacity (decimal): ");
  Serial.print(LOAD_MAX_CAPACITY);
}
//6
void Read_Calibration_Point1_Load() {

  ds.Reset();
  ds.randomRead(0xA0, 0x08, Read_page_2, 8);
  uint8_t ADD_0x0C;
  uint8_t ADD_0x0D;
  uint8_t ADD_0x0E;
  uint8_t ADD_0x0F;

  ADD_0x0C = Read_page_2[4];
  delay(5);
  ADD_0x0D = Read_page_2[5];
  delay(5);
  ADD_0x0E = Read_page_2[6];
  delay(5);
  ADD_0x0F = Read_page_2[7];
  delay(5);

  uint32_t R_Calibration_Point_1 = ((uint32_t)ADD_0x0F << 24) | ((uint32_t)ADD_0x0E << 16) | ((uint32_t)ADD_0x0D << 8) | ((uint32_t)ADD_0x0C);

  Serial.print("07. Calibration_Point_1 Applied Load in Newtons: ");
  Serial.print(R_Calibration_Point_1);
}
//07
void Read_Calibration_Point1_Out() {

  uint8_t Read_Vm[8];
  ds.Reset();
  ds.randomRead(0xA0, 0x10, Read_Vm, 8);
  uint8_t ADD_0x10;
  uint8_t ADD_0x11;
  uint8_t ADD_0x12;
  uint8_t ADD_0x13;


  delay(5);
  ADD_0x10 = Read_Vm[0];
  delay(5);
  ADD_0x11 = Read_Vm[1];
  delay(5);
  ADD_0x12 = Read_Vm[2];
  delay(5);
  ADD_0x13 = Read_Vm[3];
  delay(5);

  // Reconstruct the 32-bit raw value
  uint32_t raw_3 = ((uint32_t)ADD_0x13 << 24) | ((uint32_t)ADD_0x12 << 16) | ((uint32_t)ADD_0x11 << 8) | ((uint32_t)ADD_0x10);


  uint32_t sign_3 = (raw_3 >> 31) & 0x1;
  uint32_t exponent_3 = (raw_3 >> 23) & 0xFF;
  uint32_t mantissa_3 = raw_3 & 0x7FFFFF;

  union {
    float f3;
    uint32_t u3;
  } converter_3;

  converter_3.u3 = raw_3;
  float decodedValue_3 = converter_3.f3;


  Serial.print("08. Calibration Point 1. mv/V output @ applied load: ");
  Serial.print(decodedValue_3, 6);
}
//8
void Read_Calibration_Point2_Load() {

  uint8_t Read_L2[8];
  ds.Reset();
  ds.randomRead(0xA0, 0x10, Read_L2, 8);

  uint8_t ADD_0x14;
  uint8_t ADD_0x15;
  uint8_t ADD_0x16;
  uint8_t ADD_0x17;
  ADD_0x14 = Read_L2[4];
  ADD_0x15 = Read_L2[5];
  ADD_0x16 = Read_L2[6];
  delay(5);
  ADD_0x17 = Read_L2[7];

  uint32_t R_Calibration_Point_2 = ((uint32_t)ADD_0x17 << 24) | ((uint32_t)ADD_0x16 << 16) | ((uint32_t)ADD_0x15 << 8) | ((uint32_t)ADD_0x14);

  Serial.print("09.Calibration_Point_2 Applied Load in Newtons: ");
  Serial.print(R_Calibration_Point_2);
}
//09
void Read_Calibration_Point2_Out() {

  uint8_t Read_V2[8];
  ds.Reset();
  ds.randomRead(0xA0, 0x18, Read_V2, 8);

  uint8_t ADD_0x18;
  uint8_t ADD_0x19;
  uint8_t ADD_0x1A;
  uint8_t ADD_0x1B;

  ADD_0x18 = Read_V2[0];
  ADD_0x19 = Read_V2[1];
  ADD_0x1A = Read_V2[2];
  ADD_0x1B = Read_V2[3];


  uint32_t raw_4 = ((uint32_t)ADD_0x1B << 24) | ((uint32_t)ADD_0x1A << 16) | ((uint32_t)ADD_0x19 << 8) | ((uint32_t)ADD_0x18);

  uint32_t sign_4 = (raw_4 >> 31) & 0x1;
  uint32_t exponent_4 = (raw_4 >> 23) & 0xFF;
  uint32_t mantissa_4 = raw_4 & 0x7FFFFF;


  union {
    float f4;
    uint32_t u4;

  } converter_4;

  converter_4.u4 = raw_4;
  float decodedValue_4 = converter_4.f4;


  Serial.print("10. Calibration Point 2 @ mv/V : ");
  Serial.print(decodedValue_4, 6);
}

void Help() {
  Serial.print("Commands:");
  Serial.print("WRITE:");
  Serial.print("VR: Write Version of this data structure  ");
  Serial.print("CB: Write Number of Calibration points  ");
  Serial.print("F1: Write Reserved for future use 0x02 set to 0");
  Serial.print("F2: Write Reserved for future use 0x03 set to 0");
  Serial.print("FS: Write Factory Serial Number");
  Serial.print("MX: Write Max capacity in Newtons");
  Serial.print("L1: Write Calibration point 1. in Newtons");
  Serial.print("V1: Write Calibration point 1. in mV/V");
  Serial.print("L2: Write Calibration point 1. in Newtons");
  Serial.print("V2: Write Calibration point 1. in mV/V");

  Serial.print("W1: Write Version of this data structure  ");
  Serial.print("W2: Write Number of Calibration points  ");
  Serial.print("W3: Write Reserved for future use 0x02 set to 0");
  Serial.print("W3: Write Reserved for future use 0x03 set to 0");
  Serial.print("W5: Write Factory Serial Number");
  Serial.print("W6: Write Max capacity in Newtons");
  Serial.print("W7: Write Calibration point 1. in Newtons");
  Serial.print("W8: Write Calibration point 1. in mV/V");
  Serial.print("W9: Write Calibration point 1. in Newtons");
  Serial.print("WA: Write Calibration point 1. in mV/V");

  Serial.print("RS: Print EEPROM's Serial Number");
  Serial.print("ID: Print EEPROM's Manufacture ID");
  Serial.print("RP: Print Detect Reset Pulse");
  Serial.print("SC: Scan slave device address");
}


// Calculate CRC-8 for the 56-bit serial number (7 bytes)
// Using polynomial X^8 + X^5 + X^4 + 1
uint8_t CalculateSerialNumberCRC(const uint8_t* serialNumber, uint8_t length) {
  uint8_t crc = 0;
  // Process each byte of the serial number
  for (uint8_t i = 0; i < length; i++) {
    uint8_t inbyte = serialNumber[i];
    // Process each bit of the current byte
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;

      // If mixed bit is 1, XOR with polynomial 0x8C (X^8 + X^5 + X^4 + 1)
      if (mix) {
        crc ^= 0x8C;
      }

      inbyte >>= 1;
    }
  }

  return crc;
}

// Function to verify a complete serial number including the CRC byte
bool VerifySerialNumberCRC(const uint8_t* serialNumber) {
  // Calculate CRC from the first 7 bytes
  uint8_t CalculatedCRC = CalculateSerialNumberCRC(serialNumber, 7);

  if ((serialNumber[7] == CalculatedCRC) && (serialNumber[0] == A0, HEX)) {
    return 1;

  } else {
    return 0;
  }
}
