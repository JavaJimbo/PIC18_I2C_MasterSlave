#include "Project.h"
/* 
 * File:   TCA9548_MPX.c
 * Author: Jim Sedgwick
 *
 * Routines for setting and reading back the selected channel on the Adafruit TCA9548A breakout board.
 * Created on July 31, 2022, 2:56 AM
 * 
 */

// This routine sets the channel on the Adafruit TCA9548A breakout board.
// Up to eight multiplexer boards can be used and each can connect up to eight I2C slave devices.
// So a total of 64 slaves can be individually switched to one master.
// Although the TC9548 allows multiple slaves to be connected simultaneoulsy,
// this routine only connects one slave to the master.
//
// Inputs:
//      multiplexerID: Selects which one of eight boards 0-7 to connect to master.
//      channel: selects which one of eight I2C slaves 0-7 are selectwed
// Returns:
//      0 if no errors, or 1 if multiplexer doesn't ACK back to master commands.
unsigned char SetMultiplexerChannel(unsigned char multiplexerID, unsigned char channel)
{
unsigned char errorFlag = 0x00;	// Clear error flag
unsigned char controlRegister = 0x01;

    if (channel > 7) return 0xFF;
    if (multiplexerID > 7) return 0xFF;

    i2c_Start();
    i2c_SendControlByte(MULTIPLEXER | multiplexerID, I2C_WRITE);    
    errorFlag = i2c_GetAcknowledge();  
    
    if (!errorFlag)
    {
        controlRegister = 0x01;
        controlRegister = controlRegister << channel;    
        // Select MPX channel 0-7
        i2c_SendByte(controlRegister);
        errorFlag = errorFlag | i2c_GetAcknowledge();  
    }
    i2c_Stop();

    return errorFlag;
}

// This routine reads back the selected channel. See above routine for explanation.
// Inputs:
//      multiplexerID: Selects which one of eight boards 0-7 to connect to master.
// Outputs:
//      channel: indicates which slave is enabled by multiplexer
// Returns:
//      0 if no errors, or 1 if multiplexer doesn't ACK back to master commands.
unsigned char GetMultiplexerChannel(unsigned char multiplexerID, unsigned char *ptrChannel)
{
unsigned char errorFlag = 0x00;	// Clear error flag
unsigned char controlRegister = 0x00;
unsigned char channel = 0x00;

    if (multiplexerID > 7) 
    {
        *ptrChannel = 0;
        return 0xFF;
    }

    i2c_Start();
    i2c_SendControlByte(MULTIPLEXER | multiplexerID, I2C_READ);
    errorFlag = i2c_GetAcknowledge();
    
    if (!errorFlag)
    {        
        controlRegister = i2c_ReadByte();
        i2c_SendAcknowledge(I2C_LAST); // No acknowledge because only one byte is being read
        channel = 0;
        while (controlRegister > 1) 
        {
            channel++;
            controlRegister = controlRegister >> 1;
        }
    }
    i2c_Stop();        
    *ptrChannel = channel;
    return errorFlag;
}