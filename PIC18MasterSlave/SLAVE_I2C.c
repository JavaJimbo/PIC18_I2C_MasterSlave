// 	SLAVE_I2C - written by Jim Sedgwick
// 	Contains reading and writing slave routines.
// 	Revision history
//	2008-4-6 JBS:	MODIFIED FOR 16F877 - I2C switched over to RC3, RC4
#include    "Project.h"

#ifdef SLAVE
#define PACKET_LENGTH 5 // All incoming and outgoing I2C pcakets are 5 bytes - one command byte followed by 4 data bytes




extern unsigned char errorFlag;
extern unsigned char slaveBuffer[]; // The same buffer is used for both incoming and outgoing data.
									// As soon as a complete packet is received, it is sent back out to Master
									// This enables Master to check that the complete packet was correctly sent. 


//	This routine sets up I2C functions for slave mode
void Initialize_I2C_Slave (void)
{
    unsigned char I2CAddress = 0;
	SSPxCON1 = 0x00;     
    SSPxCON2 = 0x00;
    SSPxCON3 = 0x00;
    
    SSPxCON1bits.SSPEN = 1; // Enables I2C port pins SDAx and SCLx
    SSPxCON1bits.SSPM3 = 0; // I2C Client mode, seven bit address
    SSPxCON1bits.SSPM2 = 1;
    SSPxCON1bits.SSPM1 = 1;
    SSPxCON1bits.SSPM0 = 0;
    
    SSPxCON1bits.CKP = 1; // Release clock
    
    SSPxCON2bits.SEN = 0; // Make sure clock stretching is disabled'
     
    I2CAddress = SLAVE_ADDRESS;
    SSPxADD = (I2CAddress << 1) & 0b11111110;
	SSPxSTAT = 0;	// Clear MSSP status register
    SSPxIF = 0;			// CLEAR FLAG
}

void Reset_I2C_Slave (void)
{
    SSPxCON1bits.SSPEN = 1; // Enables I2C port pins SDAx and SCLx;
}

//	I2C_Slave_Handler - Triggered by incoming I2C request from MASTER
//						Determines which state of I2C transaction
//						and services request accordingly.
//						
// 	The I2C code checks SSPSTAT register for 5 states:
//
//	State 1: I2C WRITE operation, last byte was an address byte:
//	SSPSTAT bits:	S=1, STAT_DA=0, STAT_RW=0, STAT_BF=1
//
//	State 2: I2C WRITE operation, last byte was a data byte:
//	SSPSTAT bits:	S=1, STAT_DA=1, STAT_RW=0, STAT_BF=1
//
//	State 3: I2C READ operation, last byte was an address byte:
//	SSPSTAT bits:	S=1, STAT_DA=0, STAT_RW=1, STAT_BF=0
//
//	State 4: I2C READ operation, last byte was a data byte:
//	SSPSTAT bits:	S=1, STAT_DA=1, STAT_RW=1, STAT_BF=0
//
//	State 5: Slave I2C logic reset by NACK from master:
//	SSPSTAT bits:	S=1, STAT_DA=1, STAT_RW=0, STAT_BF=0
//
// 	Needed bit definitions from SSPSTAT register 
//	STAT_BF		*	1= Buffer full
//	UA		
//	STAT_RW		*	1= Read
//	STAT_S	*	1= Start
//	STOP	
//	STAT_DA		*	1= Data
//	CKE		
//	SMP		
//  Returns number of bytes if data is ready, 0 otherwise

int I2C_Slave_Handler(unsigned char *ptrWRITEbuffer, unsigned char *ptrREADbuffer)
{
unsigned char state=0;
unsigned char dummy;
static unsigned char i=0;
int numBytes;
	
	if ((1==SSPxSTATbits.S)&&(0==SSPxSTATbits.D_A)&&(0==SSPxSTATbits.R_W)&&(1==SSPxSTATbits.BF)) state=1;         // MASTER ADDRESS WRITE
	else if ((1==SSPxSTATbits.S)&&(1==SSPxSTATbits.D_A)&&(0==SSPxSTATbits.R_W)&&(1==SSPxSTATbits.BF)) state=2;    // MASTER DATA WRITE
	else if ((1==SSPxSTATbits.S)&&(0==SSPxSTATbits.D_A)&&(1==SSPxSTATbits.R_W)) state=3;                          // MASTER ADDRESS READ
	else if ((1==SSPxSTATbits.S)&&(1==SSPxSTATbits.D_A)&&(1==SSPxSTATbits.R_W)) state=4;                          // MASTER DATA READ
	else if ((1==SSPxSTATbits.S)&&(1==SSPxSTATbits.D_A)&&(0==SSPxSTATbits.R_W)&&(0==SSPxSTATbits.BF)) state=5;    // MASTER NACK RESET       
	else state=6;

	if(1 == state)			//	State 1: MASTER WRITE TO SLAVE operation, last byte was an address byte:
	{						//  Clear data buffer:
		i=0;				//  Clear the index	
		dummy = SSPxBUF;		//  Do dummy read.	
	}		
	else if(2 == state)					//	State 2: MASTER WRITE TO SLAVE operation, last byte was a data byte.
	{
		ptrWRITEbuffer[i] = SSPxBUF;		// Read incoming data
		i++;
		numBytes=i;
		if(i >= MAXBUFFER)i=0;		
		return(numBytes);	// Return number of bytes received when complete packet has been received
	}
	else if(3 == state)					//	State 3: MASTER READ FROM SLAVE operation, last byte was an address byte.
	{
		i = 0;							// Clear index to send first byte
		while(1 == SSPxSTATbits.BF) dummy = SSPxBUF;		// Do dummy read to clear address from buffer NOTE: THIS CONFLICTS WITH MICROCHIP APP NOTE!!!	
		do{
            SSPxCON1bits.WCOL = 0;						// Clear collision flag
			SSPxBUF = ptrWRITEbuffer[i];	// Write outgoing data
		}
        while (1 == SSPxCON1bits.WCOL);				// Check for collision
		SSPxCON1bits.CKP=1;							// Release clock
		i++;				
	}
	else if(4 == state)					//	State 4: MASTER READ FROM SLAVE operation, last byte was a data byte:	
	{		
		while (1 == SSPxSTATbits.BF);					// Is buffer full? If so, wait
		do {
			SSPxCON1bits.WCOL = 0;						// Clear collision flag
			SSPxBUF = ptrREADbuffer[i];	// Write outgoing data
		} while(1 == SSPxCON1bits.WCOL);				// Check for collision
        SSPxCON1bits.CKP = 1; // Release clock
		i++;		
		if (i >= MAXBUFFER) i = 0;				// Check index for overruns			
	}
	else if(5 == state)					//	State 5: Slave I2C logic reset by NACK from master:
	{
		;			 					// Slave logic is reset
	}
	else
	{
		;								// Error case
	}	
	return(0); // No data ready;
}

#endif