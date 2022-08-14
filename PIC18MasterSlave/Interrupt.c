#include "Project.h"

extern int I2CDataBytesReceived;
extern unsigned char I2C_WRITEdata[], I2C_READdata[];

extern unsigned char RxBuffer[];
extern unsigned char RxBufferFull;

static void interrupt isr(void) 
{
    unsigned char ch;
    static int RxIndex = 0;  
    static short Timer2Counter = 0;
    
    // Check for framing errors
    while (RCSTA1bits.FERR) 
    {
        ch = RC1REG;
        RxIndex = 0;
    }
    // Check for overrun errors
    if (RCSTA1bits.OERR) 
    {
        RCSTA1bits.CREN = 0; // Clear error
        RCSTA1bits.CREN = 1; // Re-enable reception
        RxIndex = 0;
    }
    
    if (PIR1bits.RC1IF)
    {
        PIR1bits.RC1IF = 0;
        ch = RCREG1;        
        if (ch != 0 && ch != '\n')
        {
            if (RxIndex < MAXBUFFER)
            {
                RxBuffer[RxIndex++] = ch;
                if (ch == '\r')
                {
                    RxBuffer[RxIndex] = '\0';
                    RxBufferFull = true;
                    RxIndex = 0;
                }
            }
            else RxIndex = 0;
        }        
    }
    
    
    if (TMR2IF)
    {
        TMR2IF = 0;
        Timer2Counter++;
        if (Timer2Counter >= 10000)
        {
            Timer2Counter = 0;
            if (TEST_OUT) TEST_OUT = 0;
            else TEST_OUT = 1;
        }
    }
#ifdef SLAVE    
    while (SSP2IF)
    {
        SSP2IF = 0;
        I2CDataBytesReceived = I2C_Slave_Handler (I2C_WRITEdata, I2C_READdata);
    }
#endif    
}

