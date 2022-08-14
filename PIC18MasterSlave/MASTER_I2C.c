/* MASTER_I2C - written by Jim Sedgwick
 *
 * 7-27-22: Revised for PIC 18F44K22 using I2C2 bus
 * 8-08-22: Works with either bus I2C1 or I2C2 on PIC 18F44K22
 *          Desired bus must be defined in Project.h: USE_I2C_MASTER_BUS_2 or USE_I2C_MASTER_BUS_1
 * 
 */

#include    "Project.h"


#ifdef MASTER

//	This routine sets up I2C functions for master mode
void InitializeI2Cmaster (void)
{
	SSPxxCON1 = 0x00;
    SSPxxCON2 = 0x00;
    SSPxxCON3 = 0x00;
    
    SSPxxCON1bits.SSPEN = 1; // Enables I2C port pins SDAx and SCLx
    SSPxxCON1bits.SSPM3 = 1; // I2C Host mode, clock = FOSC / (4 * (SSPxxADD+1)))
    SSPxxCON1bits.SSPM2 = 0;
    SSPxxCON1bits.SSPM1 = 0;
    SSPxxCON1bits.SSPM0 = 0;
            
	SSPxxADD = 159;	// FBUS = OSC/4 /(N+1)  So for 100 kHz clock, set N = 159 when OSC = 64,000,000
	SSPxxSTAT = 0;	// Clear MSSP status register
    SSPxxIF = 0;			// CLEAR FLAG

	// Make sure I2C is STOPPED:
	i2c_Stop();
}


//  Send stop condition
// 	  - data low-high while clock high
void i2c_Stop(void)
{
	SSPxxCON2bits.PEN = 1;			// Send STOP condition
	while (SSPxxIF == 0); // WAIT TILL SSPxxIF SET IN PIR REGISTER
	SSPxxIF = 0;			// CLEAR FLAG
}

//	Send start condition
void i2c_Start(void)
{
	SSPxxCON2bits.SEN = 1;			// Send START condition
	while (SSPxxIF == 0); // WAIT TILL SSPxxIF SET IN PIR REGISTER
	SSPxxIF = 0;			// CLEAR FLAG
}


//	Send restart condition
void i2c_Restart(void)
{
	SSPxxCON2bits.RSEN = 1;			// Send REPEATED START condition
	while (SSPxxIF == 0); // WAIT TILL SSPxxIF SET IN PIR REGISTER
	SSPxxIF = 0;			// CLEAR FLAG
}

//	Send a byte to the slave
// 	  - returns true on error
void i2c_SendByte(unsigned char byte)
{

	SSPxxBUF = byte;		// Send byte
	while (SSPxxIF == 0); // WAIT TILL SSPxxIF SET IN PIR REGISTER
	SSPxxIF = 0;			// CLEAR FLAG	
}

//	send control byte and data direction to the slave
//  	- 7-bit control byte shifted up one bit
// 	  	- direction (0 = write, 1 = read)
void i2c_SendControlByte(unsigned char controlByte, unsigned char direction)
{
unsigned char outByte;

		outByte = ((controlByte << 1) & 0b11111110) | direction;
        // outByte = (controlByte & 0b11111110) | direction;
        i2c_SendByte (outByte);
}

//	Check for an acknowledge from slave EEPROM after sending it a command or data
// 	  - returns ack or ~ack
unsigned char i2c_GetAcknowledge(void)
{
unsigned char ack;

	ack = SSPxxCON2bits.ACKSTAT;
	return ack;
}



unsigned char i2c_ReadByte(void)
{
unsigned char byte;
	
	SSPxxCON2bits.RCEN = 1;	// Enable receive mode for I2C
	while (SSPxxIF == 0); // WAIT TILL SSPxxIF SET IN PIR REGISTER
	SSPxxIF = 0;			// CLEAR FLAG	
	byte = SSPxxBUF;
	return byte;
}

//	Send an (~)acknowledge to the slave
//		- status of I2C_LAST implies this is the last byte to be sent
//    	- if there are more bytes, then ACK must be brought low
//		- if there are no more bytes, then ACK is left high
//
// Returns nothing
void i2c_SendAcknowledge(unsigned char status)
{
    if (status & 0x01) 
        SSPxxCON2bits.ACKDT = 0; // drive line low -> more to come
    else SSPxxCON2bits.ACKDT = 1; // line left high -> last expected byte has been received
	SSPxxCON2bits.ACKEN = 1;	// Initiate acknowledge sequence
	while (SSPxxIF == 0); // WAIT TILL SSPxxIF SET IN PIR REGISTER
	SSPxxIF = 0;			// CLEAR FLAG
}


/*********************** ROUTINES FOR WRITING TO SLAVE PIC ******************************/

unsigned char i2c_SlaveWrite(unsigned char device, unsigned char numBytes, unsigned char *ptrDataOut) 
{
unsigned char i, dataByte; //, WCOLflag;
unsigned char errorFlag;

	errorFlag = 0x00;	// Clear error flag

	// Send START & WRITE command and get acknowledge:
	i2c_Start();		
    i2c_SendControlByte(device, I2C_WRITE);
	errorFlag |= i2c_GetAcknowledge();

    if (!errorFlag)
    {
    	// Now send DATA and get acknowledges following each byte:
    	for(i=0; i<numBytes; i++)
    	{
            dataByte = ptrDataOut[i];
    		i2c_SendByte(dataByte);		
        	errorFlag |= i2c_GetAcknowledge();
        }
    }
    else 
    {
        DelayMs(100);
        printf("\rSLAVE WRITE NO ACK");
    }

	// Send STOP condition
	i2c_Stop();
    
    return errorFlag;
}


unsigned char i2c_SlaveRead(unsigned char device, unsigned char numBytes, unsigned char *ptrDataIn)
{
unsigned char i;
unsigned char errorFlag;

	errorFlag = 0x00;	// Clear error flag
    
	// Send START & READ command and get acknowledge.
    // Note: No WRITE operation needed, 
    // since no address bytes are being sent after 7 bit device byte:
	i2c_Start();		
    i2c_SendControlByte(device, I2C_READ);
	errorFlag |= i2c_GetAcknowledge();    

    // Now receive bytes from slave:
	i = 0;	
	do {
		// Now read each DATA byte, followed by acknowledge:
		ptrDataIn[i] = (unsigned char) i2c_ReadByte();
		i++;				
		if (i < numBytes)
			i2c_SendAcknowledge(I2C_MORE);  // Send ACK after receiving each byte
		else
			i2c_SendAcknowledge(I2C_LAST);	// No ACK after last byte
	} while (i < numBytes);
    
	// Send STOP condition
	i2c_Stop();    
    return errorFlag;
}

#endif