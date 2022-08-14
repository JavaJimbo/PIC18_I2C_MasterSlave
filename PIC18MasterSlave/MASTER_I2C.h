/* #include "MASTER_I2C.h" 
 *
 * Created 8-08-22 Jim Sedgwick
 * 
 */
#define USE_I2C_MASTER_BUS_1


#ifdef USE_I2C_MASTER_BUS_1
#define SSPxxCON1 SSP1CON1
#define SSPxxCON1bits SSP1CON1bits
#define SSPxxCON2bits SSP1CON2bits
#define SSPxxCON2 SSP1CON2
#define SSPxxCON3 SSP1CON3
#define SSPxxCON3bits SSP1CON3bits
#define SSPxxADD SSP1ADD
#define SSPxxSTAT SSP1STAT
#define SSPxxIF SSP1IF
#define SSPxxBUF SSP1BUF
#elif USE_I2C_MASTER_BUS_2
#define SSPxxCON1 SSP2CON1
#define SSPxxCON1bits SSP2CON1bits
#define SSPxxCON2bits SSP2CON2bits
#define SSPxxCON2 SSP2CON2
#define SSPxxCON3 SSP2CON3
#define SSPxxCON3bits SSP2CON3bits
#define SSPxxADD SSP2ADD
#define SSPxxSTAT SSP2STAT
#define SSPxxIF SSP2IF
#define SSPxxBUF SSP2BUF
#else
    #error NO I2C BUS SELECTED
#endif

#define EEPROM 0x50 			// This addresses the external EEPROM that holds the PIC program code
#define SENSIRION 0x44
#define MULTIPLEXER 0x70



#define I2C_READ	0x01		/* read bit used with address */
#define I2C_WRITE	0x00		/* write bit used with address */

#define I2C_ERROR	(-1)
#define I2C_LAST	FALSE		/* SendAck: no more bytes to send */
#define I2C_MORE	TRUE		/* SendAck: more bytes to send */

			

unsigned char 	i2c_GetAcknowledge(void);
unsigned char	i2c_ReadByte(void);
void            i2c_SendControlByte(unsigned char, unsigned char);
void            i2c_SendByte(unsigned char);
void			i2c_Start(void);
void			i2c_Restart(void);
void			i2c_Stop(void);
void			i2c_SendAcknowledge(unsigned char);
void 			InitializeI2Cmaster (void);
unsigned char   i2c_SlaveWrite (unsigned char device, unsigned char numBytes, unsigned char *ptrDataOut);
unsigned char   i2c_SlaveRead  (unsigned char device, unsigned char numBytes, unsigned char *ptrDataIn);





