/* #include "SLAVE_I2C.h" 
 *
 * Created 8-08-22 Jim Sedgwick
 * 
 */

#define USE_I2C_SLAVE_BUS_2

#ifdef USE_I2C_SLAVE_BUS_1
#define SSPxCON1 SSP1CON1
#define SSPxCON1bits SSP1CON1bits
#define SSPxCON2bits SSP1CON2bits
#define SSPxCON2 SSP1CON2
#define SSPxCON3 SSP1CON3
#define SSPxCON3bits SSP1CON3bits
#define SSPxADD SSP1ADD
#define SSPxSTAT SSP1STAT
#define SSPxSTATbits SSP1STATbits
#define SSPxIF SSP1IF
#define SSPxIE SSP1IE
#define SSPxBUF SSP1BUF
#endif

#ifdef USE_I2C_SLAVE_BUS_2
#define SSPxCON1 SSP2CON1
#define SSPxCON1bits SSP2CON1bits
#define SSPxCON2bits SSP2CON2bits
#define SSPxCON2 SSP2CON2
#define SSPxCON3 SSP2CON3
#define SSPxCON3bits SSP2CON3bits
#define SSPxADD SSP2ADD
#define SSPxSTAT SSP1STAT
#define SSPxSTATbits SSP2STATbits
#define SSPxIF SSP2IF
#define SSPxIE SSP1IE
#define SSPxBUF SSP2BUF
#endif


			
void Initialize_I2C_Slave (void);
void Reset_I2C_Slave (void);
int I2C_Slave_Handler(unsigned char *ptrWRITEbuffer, unsigned char *ptrREADbuffer);


