        
#include <Arduino.h>
#include "AT21CS01.h"
#include "AT21CS01_direct_gpio.h"



int communicationSpeed =0 ;          // Initialize with default value

#ifdef ARDUINO_ARCH_ESP32
// due to the dual core esp32, a critical section works better than disabling interrupts
#  define noInterrupts() {portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux)
#  define interrupts() portEXIT_CRITICAL(&mux);}
// for info on this, search "IRAM_ATTR" at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/general-notes.html 
#  define CRIT_TIMING IRAM_ATTR
#else
#  define CRIT_TIMING 
#endif


void AT21CS01::begin(uint8_t pin)
{
	pinMode(pin, INPUT);
	bitmask = PIN_TO_BITMASK(pin);
	baseReg = PIN_TO_BASEREG(pin);

}

//___________________________________________________________________________
uint8_t CRIT_TIMING AT21CS01::reset(void)
{
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;
  uint8_t temp;
	
  DIRECT_MODE_INPUT(reg, mask);
  delayMicroseconds(200); //delay for tHTSS (STD Speed)  

	//noInterrupts();
  DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	//interrupts();
	delayMicroseconds(480); //delay for tRESET (STD Speed

	//noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(20);//delay for tRRT
  

  //noInterrupts();
  DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	//interrupts();
	delayMicroseconds(1); //delay for tDRR
  
  
  //noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(6);

  r = DIRECT_READ(reg, mask);
  if(r == 0){ temp = 0x00; }     //if device ACKs
  else { temp = 0xFF; }   
  
 
	
  
	delayMicroseconds(21);
  //DIRECT_MODE_INPUT(reg, mask);

  communicationSpeed = 0; 
	return temp;

}

//_______________________________________________________________________________________
void CRIT_TIMING AT21CS01::start(void)
{
  IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

  DIRECT_MODE_INPUT(reg, mask);	// allow it to float
  if(communicationSpeed = 0)    //DUT set for High-Speed
  {
    delayMicroseconds(150);       //delay tHTSS (High Speed)

  }

  else
  {
    delayMicroseconds(600);       //delay tHTSS (High Speed)

  }
  
}
//___________________________________________________________________________________

//Transmit Logic '1' Routine
void CRIT_TIMING AT21CS01::tx1(void)
{
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

    noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(1);
		DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
    delayMicroseconds(14);
		interrupts();

}
//___________________________________________________________________________________
void CRIT_TIMING AT21CS01::tx0(void)
{
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

    noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(10);
		DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
    delayMicroseconds(5);
		interrupts();

}

//Receive ACK/NACK Routine _______________________________________________________________
uint8_t CRIT_TIMING AT21CS01::ack(void)
{
  IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t temp; //temp variable
  uint8_t r;

	if (communicationSpeed == 0)
  {
   
	 DIRECT_MODE_OUTPUT(reg, mask);
	 DIRECT_WRITE_LOW(reg, mask);
	 delayMicroseconds(1);             //delay tRD (High Speed)
   DIRECT_MODE_INPUT(reg, mask);	
	 delayMicroseconds(1);             //delay tMRS  (High Speed)
   r = DIRECT_READ(reg, mask);
  if(r == 0){ temp = 0xFF; }     //if device ACKs
  else { temp = 0x00; }   
   delayMicroseconds(9);             //delay tBIT (High Speed)

  }
  else
  {
   
	 DIRECT_MODE_OUTPUT(reg, mask);
	 DIRECT_WRITE_LOW(reg, mask);
	 delayMicroseconds(4);             //delay tRD (STD Speed)
   DIRECT_MODE_INPUT(reg, mask);	
	 delayMicroseconds(2);             //delay tMRS  (STD Speed)
   r = DIRECT_READ(reg, mask);
  if(r == 0){ temp = 0xFF; }     //if device ACKs
  else { temp = 0x00; }   
   delayMicroseconds(34);             //delay tBIT (STD Speed)
  }

	return temp;
}

uint8_t CRIT_TIMING AT21CS01::acknack(void)
{
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;

	noInterrupts();
	DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	delayMicroseconds(1);
	DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(1);
	r = DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(9);
	return r;
}

/*
//______________________________________________________________________
uint8_t  CRIT_TIMING AT21CS01::txByte(uint8_t dataByte)
{

	uint8_t temp;

   for (uint8_t ii = 0; ii < 8 ; ii++){        //transmit data byte
        if (0x80  & dataByte)  {   tx1();  }
        else   {   tx1();  }
        dataByte <<= 1;
    } 
    temp = acknack();                           //ACK/NACK Bit
return temp;

}
*/
//____________________________________________________________________________
uint8_t CRIT_TIMING AT21CS01::readByte (void)
{
    IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

  uint8_t temp;                                   //temp variable
  uint8_t dataByte = 0x00;                        //variable for data read

    if (communicationSpeed == 0)    {           //DUT set for High-Speed
        for (int8_t ii = 0; ii < 8 ; ii++){
            DIRECT_MODE_OUTPUT(reg, mask);
	          DIRECT_WRITE_LOW(reg, mask);
	          delayMicroseconds(1);               //delay tRD (High Speed)
                                  
            DIRECT_MODE_INPUT(reg, mask);	
	          delayMicroseconds(1);               //delay tMRS  (High Speed)

            temp = DIRECT_READ(reg, mask);       //read state of the SI/O pin
            temp &= 0x01;                       //NDs the bit read w/ bitMask 
            dataByte = (uint8_t)((dataByte << 1) | temp);
            delayMicroseconds(9);                   //delay tBIT (High Speed)
        }   
    }
        
    else    {                                   //DUT set for Standard Speed    
        for (int8_t ii = 0; ii < 8 ; ii++){
            DIRECT_MODE_OUTPUT(reg, mask);
	          DIRECT_WRITE_LOW(reg, mask);
	          delayMicroseconds(4);               //delay tRD (High Speed)
                                  
            DIRECT_MODE_INPUT(reg, mask);	
	          delayMicroseconds(2);               //delay tMRS  (High Speed)
            temp = DIRECT_READ(reg, mask);       //read state of the SI/O pin
            temp &= 0x01;                       //NDs the bit read w/ bitMask 
            dataByte = (uint8_t)((dataByte << 1) | temp);
            delayMicroseconds(34);                   //delay tBIT (High Speed)
        }
    }
return dataByte;
}
//______________________________________________________________________________

uint8_t CRIT_TIMING AT21CS01::ScanDeviceAddress(void)
{

    uint8_t temp;
    uint8_t address;
    
    for(uint8_t ii =0; ii < 8; ii++)     //loop for all 8 slave addresses
	{
		  Reset();                           //perform device discovery
		
			start();                           //start condition 
			temp = ii; 
			temp <<= 1;                         //shift address to proper bit position
			temp |= 0xA0;                       //add EEPROM op code and set R/W = 0		 
			if(write(temp) == 0x00)             //If the device ACKs
			{
				address = ii;                     //set slave address equal to loop count
				address <<= 1;                    //shift address to proper bit position
				start();                          //start condition 
				break;                            //break from loop
			}
		
	}
    
return address;                         //return ACK'd slave address
}
//______________________________________________________________________________
void CRIT_TIMING AT21CS01::randomRead(uint8_t dByte, uint8_t aByte, uint8_t *readData, uint8_t count)
{

    dByte |= 0x00;   //set slave address                

    start();                                  //start condition    
    write(dByte);                             //Device Address Byte     
    write(aByte);                             //Word Address Byte
    start();                                 //start condition 
    write((uint8_t)(dByte | 0x01));            //Device Address Byte
    for (uint8_t ii = 0; ii < count; ii++)	{   //loop for bytes to be written         
        readData[ii] = readByte();              //read a byte 
        if (ii < (count-1)) {   tx0();  }       //send an ACK
    }    
    tx1();                                      //send a NACK
    start();                                    //stop condition
/*    
    printf("    Data Read =");                  //TX character string over UART
    for (uint8_t ii = 0; ii < count; ii++)  {
        printf(" 0x");                          //TX character string over UART
        printf("%02X", readData[ii]);           //"%02X" for two char ^case hex
    }
    printf("\n");                               //create a new line
*/
}

void CRIT_TIMING AT21CS01::ManufactureIDRead(uint8_t dByte, uint8_t *readData, uint8_t count)
{
  //dByte |= 0x00;   //set slave address                

    start();                                  //start condition    
    write(dByte);                             //Device Address Byte     
    //acknack();                             // sleve ack dont put it 
    

    readData[7] = readByte();  //read 7th  byte
    tx0();                     //send a ACK by master         
    readData[6] = readByte();
    tx0();
    readData[5] = readByte();
    tx1();                                       //send a NACK by master
    start();                                     //stop condition

}

//THIS FUNCTION IS CORRECT TEST BY OSILESCOPE______________________________________________________________________________
uint8_t CRIT_TIMING AT21CS01::Reset(void)
{
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;
  uint8_t temp;
	uint8_t retries = 125;

	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);
	interrupts();
	// wait until the wire is high... just in case
	do {
		if (--retries == 0) return 0;
		delayMicroseconds(2);
	} while ( !DIRECT_READ(reg, mask));

	DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(200);
  noInterrupts();
	DIRECT_WRITE_LOW(reg, mask);
	DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
	interrupts();
	delayMicroseconds(480);
	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(20);

  noInterrupts();
	DIRECT_WRITE_LOW(reg, mask);
	DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
	interrupts();
	delayMicroseconds(1);

  DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(2);

	r = !DIRECT_READ(reg, mask);
  //if(r == 0){ temp = 0x00; }     //if device ACKs
  //else { temp = 0xFF; }
	interrupts();
	delayMicroseconds(21);
	return r;
}

//______________________________________________________________________________
void CRIT_TIMING AT21CS01::write_bit(uint8_t v)
{
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

	if (v & 1) {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(1);
		DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
		interrupts();
		delayMicroseconds(14);
	} else {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(10);
		DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
		interrupts();
		delayMicroseconds(5);
	}
}
/*
uint8_t CRIT_TIMING AT21CS01::read_bit(void)
{
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;

	noInterrupts();
	DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	delayMicroseconds(1);
	DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(1);
	r = DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(9);
	return r;
}
*/ 


uint8_t AT21CS01::write(uint8_t v, uint8_t power /*= */) {
    uint8_t bitMask;
    uint8_t temp;

    for (bitMask = 0x80; bitMask; bitMask >>= 1) {
	AT21CS01::write_bit( (bitMask & v)?1:0);
    }
   /* if ( !power) {
	     noInterrupts();
	      DIRECT_MODE_INPUT(baseReg, bitmask);
	      DIRECT_WRITE_LOW(baseReg, bitmask);
	     interrupts();
    }*/
    temp = acknack(); 
    return temp;

}

/*
void AT21CS01::write_bytes(const uint8_t *buf, uint16_t count, bool power  = 0 ) {
  for (uint16_t i = 0 ; i < count ; i++)
    write(buf[i]);
  if (!power) {
    noInterrupts();
    DIRECT_MODE_INPUT(baseReg, bitmask);
    DIRECT_WRITE_LOW(baseReg, bitmask);
    interrupts();
  }
}
*/
// Read a byte
//

/*
uint8_t AT21CS01::read() {
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( AT21CS01::read_bit()) r |= bitMask;
    }
    return r;
}

void AT21CS01::read_bytes(uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read();
}

*/
//_________________________________________________________________________________
void CRIT_TIMING AT21CS01::eepromWrite(uint8_t dByte, uint8_t aByte, uint8_t writeData[], uint8_t count){
  IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
    
    dByte |= 0x00;   //set slave address 
    
    start();                                  //start condition    
    write(dByte);                              //Device Address Byte     
    write(aByte);                              //Word Address Byte
    for (uint8_t ii = 0; ii < count; ii++)	{   //loop for data to be written
        write(writeData[ii]);                  //Data Byte(s) to be written   
    }
    start();                                  //stop condition
    delayMicroseconds(5000);                     //tWC Delay    
}
//___________________________________________________________________________________
void CRIT_TIMING AT21CS01::countWrite(uint8_t dByte, uint8_t aByte, uint16_t writeData){
    IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

    uint8_t temp;                               //temp variable
    dByte |= 0x00;   //set slave address 
    
    start();                                  //start condition    
    write(dByte);                              //Device Address Byte     
    write(aByte);                              //Word Address Byte
    temp = ((writeData >> 8) & 0xFF);          //shift write data to the right by 8 anded with mask
    write(temp);                               //write temp byte to device
    temp = (writeData & 0xFF);                 //write data anded with mask
    write(temp);                               //write temp byte to device
    start();                                   //stop condition
    delayMicroseconds(5000);                    //tWC Delay    
}
//_________________________________________________________________________________
void CRIT_TIMING AT21CS01::singleByteWrite(uint8_t device_Byte, uint8_t add_Byte, uint8_t writeData){
    IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	__attribute__((unused)) volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

    uint8_t temp;                              //temp variable
    device_Byte |= 0x00;                       //set slave address 
    
    start();                                   //start condition    
    write(device_Byte);                        //Device Address Byte     
    write(add_Byte);                           //Word Address Byte
      
    write(writeData);                         //Data Byte(s) to be written   
    
    
    start();                                   //stop condition
    delayMicroseconds(5000);                   //tWC Delay    
}
//There are some errors in SinglrByteRead and coundRead function,
//write and read valuve are not same()
uint8_t AT21CS01::SingleByteRead(uint8_t Device_Address_Byte, uint8_t Memory_Address_Byte)
{
  
  uint8_t value;
  Device_Address_Byte |= 0x00;       //Set slave address A(read/write main memory)0(read)|00(slave addres)
  start();
  write(Device_Address_Byte);
  write(Memory_Address_Byte);
  start();
  write(uint8_t(Device_Address_Byte|0x01));
  tx0();
  value =readByte();
  tx1();
  start();
  delayMicroseconds(100);

  return value;


}
//___________________________________________________________________________________
uint16_t AT21CS01::countRead(uint8_t dByte, uint8_t aByte){
    
    uint16_t temp;
    uint8_t msb;                               //temp variable 
    uint8_t lsb;                               //temp variable  
    dByte |= 0x00;   //set slave address                

    start();                                  //start condition    
    write(dByte);                             //Device Address Byte     
    write(aByte);                             //Word Address Byte
    start();                                  //start condition 
    write((uint8_t)(dByte | 0x01));           //Device Address Byte        
    msb =  readByte();                        //read a byte
    tx0();                                    //send ACK
    //readData = ((temp << 8)& 0xFF);         //sets temp as MSB of readData
    //readData = (temp & 0xFF);
    //readData = (temp & 0xFF);
    //(readData << 8) & 0xFF00;
    lsb =  readByte();                          //read a byte       
    tx1();                                      //send NACK
    //readData = (temp & 0xFF);                 //sets temp as LSB of readData
    //readData = ((temp >> 8)& 0xFF); 
   
    temp = (msb <<8)|(lsb & 0xFF);
    start();                                   //stop condition
/*   
    printf("MSB = "); 
    printf("%02X\n", msb); 
    printf("LSB = "); 
    printf("%02X\n", lsb); 
    printf("count = "); 
    printf("%02X\n", temp); 
*/
    return temp;
}
//___________________________________________________________________________________


// undef defines for no particular reason
#ifdef ARDUINO_ARCH_ESP32
#  undef noInterrupts() {portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux)
#  undef interrupts() portEXIT_CRITICAL(&mux);}
#endif
// for info on this, search "IRAM_ATTR" at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/general-notes.html 
#undef CRIT_TIMING 