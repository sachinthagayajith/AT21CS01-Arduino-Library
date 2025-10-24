/**
 * @file AT21CS01.h
 * @brief Arduino library for AT21CS01 1-Wire EEPROM
 * @author Sachintha Gayajith
 * @copyright (c) 2023-2024
 * 
 * This library provides an interface for communicating with the AT21CS01 1-Wire EEPROM
 * using an ESP32 or other Arduino-compatible microcontrollers. The AT21CS01 is a 
 * 1Kbit (128 x 8-bit) EEPROM that operates over a single-wire interface with 
 * energy harvesting capabilities.
 * 
 * Features:
 * - Custom one-wire protocol implementation specific to AT21CS01
 * - Support for both Standard and High-Speed communication modes
 * - Complete memory read/write operations
 * - Device address scanning
 * - Manufacturer ID access
 * - CRC error checking
 * 
 * Hardware Configuration:
 * - Connect AT21CS01 SI/O pin to GPIO (configured as open-drain)
 * - Use 2.1kΩ pull-up resistor between SI/O and 3.3V
 * - Connect AT21CS01 GND to system ground
 * 
 * Tested on ESP32 platform with oscilloscope verification of timing parameters.
 */

#ifndef AT21CS01_h
#define AT21CS01_h

#ifdef __cplusplus

#include <stdint.h>

#if defined(__AVR__)
#include <util/crc16.h>
#endif

#if ARDUINO >= 100
#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif



#ifndef AT21CS01_SEARCH
#define AT21CS01_SEARCH 1
#endif

// You can exclude CRC checks altogether by defining this to 0
#ifndef AT21CS01_CRC
#define AT21CS01_CRC 1
#endif

// Select the table-lookup method of computing the 8-bit CRC
// by setting this to 1.  The lookup table enlarges code size by
// about 250 bytes.  It does NOT consume RAM 
//   If you disable this, a slower
// but very compact algorithm is used.
#ifndef aT21CS01_CRC8_TABLE
#define AT21CS01_CRC8_TABLE 1
#endif

// You can allow 16-bit CRC checks by defining this to 1
// (Note that AT21CS01_CRC must also be 1.)
#ifndef AT21CS01_CRC16
#define AT21CS01_CRC16 1
#endif

// Board-specific macros for direct GPIO
#include "AT21CS01_direct_regtype.h"
extern int communicationSpeed;

/**
 * @brief Main class for interfacing with AT21CS01 1-Wire EEPROM
 * 
 * This class implements the complete protocol for communicating with
 * AT21CS01 EEPROM devices. It handles all low-level timing requirements
 * and provides high-level functions for memory operations.
 * 
 * Timing specifications are based on AT21CS01 datasheet and have been
 * verified using oscilloscope measurements.
 */
class AT21CS01
{
  private:
    IO_REG_TYPE bitmask;          ///< GPIO pin bitmask for direct port manipulation
    volatile IO_REG_TYPE *baseReg; ///< Base register for GPIO operations

#if AT21CS01_SEARCH 
    // global search state
    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    bool LastDeviceFlag;
     
#endif

  public:
    /**
     * @brief Default constructor
     */
    AT21CS01() { }

    /**
     * @brief Constructor with pin initialization
     * @param pin GPIO pin number connected to AT21CS01 SI/O line
     */
    AT21CS01(uint8_t pin) { begin(pin); }

    /**
     * @brief Initialize the interface with specified GPIO pin
     * @param pin GPIO pin number connected to AT21CS01 SI/O line
     * 
     * The pin will be configured as input initially (high-Z state).
     * Requires external 2.1kΩ pull-up resistor to 3.3V.
     */
    void begin(uint8_t pin);

    /**
     * @brief Perform a 1-Wire reset cycle
     * @return 0x00 if device responds with presence pulse, 0xFF if no device detected
     * 
     * Generates reset timing sequence:
     * 1. tHTSS delay (200μs std/150μs high-speed) with line floating
     * 2. tRESET low pulse (480μs)
     * 3. tRRT recovery time (20μs)
     * 4. tDRR delay (1μs)
     * 5. Sample presence pulse
     * 
     * The bus must be free (floating high) before calling this function.
     */
    uint8_t reset(void);
        /**
     * @brief Read acknowledgment bit from device
     * @return 0xFF if ACK (line pulled low), 0x00 if NACK
     * 
     * Timing:
     * - High-Speed: tRD=1μs, tMRS=1μs, tBIT=9μs
     * - Standard:   tRD=4μs, tMRS=2μs, tBIT=34μs
     */
    uint8_t ack(void);

    /**
     * @brief Transmit logic '1' bit
     * 
     * Timing sequence:
     * 1. Drive line low (1μs)
     * 2. Release to float high (14μs)
     * Total bit time = 15μs
     */
    void tx1(void);

    /**
     * @brief Transmit logic '0' bit
     * 
     * Timing sequence:
     * 1. Drive line low (10μs)
     * 2. Release to float high (5μs)
     * Total bit time = 15μs
     */
    void tx0(void);

    /**
     * @brief Transmit a byte of data
     * @param dataByte Byte to transmit
     * @return ACK/NACK from device
     * 
     * Transmits 8 bits MSB first, followed by ACK/NACK read
     */
    uint8_t txByte(uint8_t dataByte);

    /**
     * @brief Read a byte of data from device
     * @return Byte read from device
     * 
     * Reads 8 bits MSB first, sends ACK after reception
     */
    uint8_t readByte(void);

    /**
     * @brief Scan for device address on the bus
     * @return Device address byte if found
     * 
     * Performs device discovery sequence to identify AT21CS01 devices
     * present on the one-wire bus.
     */
    uint8_t ScanDeviceAddress(void);

    /**
     * @brief Read data from random memory location
     * @param dByte Device address byte
     * @param aByte Memory address byte (0-127)
     * @param readData Pointer to buffer to store read data
     * @param count Number of bytes to read (1-128)
     * 
     * Performs random read operation starting from specified address.
     * Can read across page boundaries.
     */
    void randomRead(uint8_t dByte, uint8_t aByte, uint8_t *readData, uint8_t count);

    /**
     * @brief Read device manufacturer ID
     * @param dByte Device address byte
     * @param readData Pointer to buffer to store manufacturer ID
     * @param count Number of ID bytes to read
     * 
     * Reads the manufacturer ID from the device's permanent ROM.
     */
    void ManufactureIDRead(uint8_t dByte, uint8_t *readData, uint8_t count);

    /**
     * @brief Write data to EEPROM
     * @param dByte Device address byte
     * @param aByte Starting memory address (0-127)
     * @param writeData Array of data bytes to write
     * @param count Number of bytes to write
     * 
     * Performs write operation starting at specified address.
     * Handles page boundary crossing automatically.
     * WARNING: Waits for write completion before returning.
     */
    void eepromWrite(uint8_t dByte, uint8_t aByte, uint8_t writeData[], uint8_t count);

    /**
     * @brief Write 16-bit counter value
     * @param dByte Device address byte
     * @param aByte Counter memory address
     * @param writeData 16-bit counter value
     */
    void countWrite(uint8_t dByte, uint8_t aByte, uint16_t writeData);

    /**
     * @brief Read 16-bit counter value
     * @param dByte Device address byte
     * @param aByte Counter memory address
     * @return 16-bit counter value read from specified address
     */
    uint16_t countRead(uint8_t dByte, uint8_t aByte);

    /**
     * @brief Write a single byte to EEPROM
     * @param device_Byte Device address byte
     * @param mem_Byte Memory address (0-127)
     * @param writeData Data byte to write
     * 
     * Performs a single byte write operation.
     * Function waits for write completion before returning.
     * This is the most basic write operation supported by the device.
     */
    void singleByteWrite(uint8_t device_Byte, uint8_t mem_Byte, uint8_t writeData);

    /**
     * @brief Read a single byte from EEPROM
     * @param Device_Address_byte Device address byte
     * @param Memory_Address_Byte Memory address (0-127)
     * @return Byte read from specified address
     * 
     * Performs a single byte read operation.
     * This is the most basic read operation supported by the device.
     */
    uint8_t SingleByteRead(uint8_t Device_Address_byte, uint8_t Memory_Address_Byte);

    uint8_t Reset(void);
    uint8_t acknack(void);

    // Issue a 1-Wire rom select command, you do the reset first.
    void select(const uint8_t rom[8]);

    // Issue a 1-Wire rom skip command, to address all on bus.
    void skip(void);
    void start(void);

    // Write a byte. If 'power' is one then the wire is held high at
    // the end for parasitically powered devices. You are responsible
    // for eventually depowering it by calling depower() or doing
    // another read or write.

       uint8_t write(uint8_t v, uint8_t power = 1);

    void write_bytes(const uint8_t *buf, uint16_t count, bool power = 1);

    // Read a byte.
    uint8_t read(void);

    void read_bytes(uint8_t *buf, uint16_t count);
   

    // Write a bit. The bus is always left powered at the end, see
    // note in write() about that.
    void write_bit(uint8_t v);

    // Read a bit.
    uint8_t read_bit(void);

    // Stop forcing power onto the bus. You only need to do this if
    // you used the 'power' flag to write() or used a write_bit() call
    // and aren't about to do another read or write. You would rather
    // not leave this powered if you don't have to, just in case
    // someone shorts your bus.
    void depower(void);

#if AT21CS01_SEARCH
    // Clear the search state so that if will start from the beginning again.
    void reset_search();

    // Setup the search to find the device type 'family_code' on the next call
    // to search(*newAddr) if it is present.
    void target_search(uint8_t family_code);

    // Look for the next device. Returns 1 if a new address has been
    // returned. A zero might mean that the bus is shorted, there are
    // no devices, or you have already retrieved all of them.  It
    // might be a good idea to check the CRC to make sure you didn't
    // get garbage.  The order is deterministic. You will always get
    // the same devices in the same order.
    bool search(uint8_t *newAddr, bool search_mode = true);
#endif

#if AT21CS01_CRC
    // Compute a Dallas Semiconductor 8 bit CRC, these are used in the
    // ROM and scratchpad registers.
    static uint8_t crc8(const uint8_t *addr, uint8_t len);

#if AT21CS01_CRC16
    // Compute the 1-Wire CRC16 and compare it against the received CRC.
    // Example usage (reading a DS2408):
    //    // Put everything in a buffer so we can compute the CRC easily.
    //    uint8_t buf[13];
    //    buf[0] = 0xF0;    // Read PIO Registers
    //    buf[1] = 0x88;    // LSB address
    //    buf[2] = 0x00;    // MSB address
    //    WriteBytes(net, buf, 3);    // Write 3 cmd bytes
    //    ReadBytes(net, buf+3, 10);  // Read 6 data bytes, 2 0xFF, 2 CRC16
    //    if (!CheckCRC16(buf, 11, &buf[11])) {
    //        // Handle error.
    //    }     
    //          
    // @param input - Array of bytes to checksum.
    // @param len - How many bytes to use.
    // @param inverted_crc - The two CRC16 bytes in the received data.
    //                       This should just point into the received data,
    //                       *not* at a 16-bit integer.
    // @param crc - The crc starting value (optional)
    // @return True, iff the CRC matches.
    static bool check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc = 0);

    // Compute a Dallas Semiconductor 16 bit CRC.  This is required to check
    // the integrity of data received from many 1-Wire devices.  Note that the
    // CRC computed here is *not* what you'll get from the 1-Wire network,
    // for two reasons:
    //   1) The CRC is transmitted bitwise inverted.
    //   2) Depending on the endian-ness of your processor, the binary
    //      representation of the two-byte return value may have a different
    //      byte order than the two bytes you get from 1-Wire.
    // @param input - Array of bytes to checksum.
    // @param len - How many bytes to use.
    // @param crc - The crc starting value (optional)
    // @return The CRC16, as defined by Dallas Semiconductor.
    static uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc = 0);
#endif
#endif
};

// Prevent this name from leaking into Arduino sketches
#ifdef IO_REG_TYPE
#undef IO_REG_TYPE
#endif

#endif // __cplusplus
#endif // AT21CS01_h