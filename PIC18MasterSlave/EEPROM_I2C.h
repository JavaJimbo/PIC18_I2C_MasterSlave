/* #include "EEPROM_I2C.h" 
 * 
 * Created 8-08-22 Jim Sedgwick
 */



// BUFFERSIZE is equal to the number of bytes in the internal program Write Buffer of the PIC 18F2550.
#define BUFFERSIZE	64				

unsigned char EepromWriteBlock(unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned short numBytes);
unsigned char EepromReadBlock(unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned char numBytes);
unsigned char EepromWriteByte(unsigned char device, unsigned int address, unsigned char dataByte);
unsigned char EepromReadByte(unsigned char device, unsigned int address, unsigned char *ptrDataByte);





