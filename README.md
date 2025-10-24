# AT21CS01 Arduino Library

An Arduino library for interfacing with the AT21CS01 1-Wire EEPROM, providing reliable communication and memory operations.

## Features

- Custom one-wire protocol implementation specific to AT21CS01
- Support for both Standard and High-Speed communication modes
- Complete memory read/write operations
- Device address scanning
- Manufacturer ID access
- CRC error checking

## Hardware Setup

- Connect AT21CS01 SI/O pin to GPIO (configured as open-drain)
- Use 2.1kΩ pull-up resistor between SI/O and 3.3V
- Connect AT21CS01 GND to system ground

## Installation

1. Download this repository
2. In Arduino IDE: Sketch -> Include Library -> Add .ZIP Library
3. Select the downloaded ZIP file

## Usage

```cpp
#include "AT21CS01.h"

AT21CS01 eeprom(13);  // Connect SI/O to GPIO 13

void setup() {
  Serial.begin(115200);
  
  // Reset device
  if(eeprom.reset() == 0x00) {
    Serial.println("Device found!");
  }
}
```

## Examples

- **EEPROM_Basic_Test**: Basic read/write operations
- **Load_Cell_Calibration**: Example for storing calibration data

## Development History

This library was developed during an internship at #S Fabrication Company (Dankotuwa, Sri Lanka) for storing load cell calibration data. The implementation is based on studying the Dallas Semiconductor One-Wire protocol and adapting it to meet AT21CS01's specific requirements.

All timing parameters have been verified using oscilloscope measurements to ensure compliance with the AT21CS01 datasheet specifications.

## License

This project is released under the MIT License. See the LICENSE file for details.

## Author

Sachintha Gayajith

## Acknowledgments

- Based on principles from Dallas Semiconductor One-Wire protocol
- Developed during internship at #S Fabrication Company
- Tested with ESP32 microcontroller platform
<img width="796" height="376" alt="image" src="https://github.com/user-attachments/assets/fd1a2aa9-c512-4903-a12f-562e4cd0d1bd" />.
## Hardware Setup

- Connect AT21CS01 data pin to GPIO 4.  
- Use Rpull as 2.1k resistor.  
- Connect VCC to 3.3V and GND to GND.  
- Verify timing with an oscilloscope if needed.
