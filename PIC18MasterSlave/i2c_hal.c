//==============================================================================
// S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT85 Sample Code
// File      :  i2c_hal.c
// Author    :  RFU
// Date      :  17-Mai-2018
// Controller:  STM32F100RB
// IDE       :  µVision V5.25.2.0
// Compiler  :  Armcc
// Brief     :  I2C hardware abstraction layer
//==============================================================================

#include "Project.h"

//------------------------------------------------------------------------------
etError I2C_WriteByteGetAck(uint8_t txByte)
{
unsigned char ack = 0;
    i2c_SendByte(txByte);
    ack = i2c_GetAcknowledge();
    return ack;
}

//------------------------------------------------------------------------------
uint8_t I2C_ReadByteSendAck(etI2cAck ack)
{
uint8_t rxByte = 0;

    rxByte = i2c_ReadByte();  
    // send acknowledge if necessary
    if (ack == ACK) i2c_SendAcknowledge(I2C_MORE);
    else i2c_SendAcknowledge(I2C_LAST);

    // wait to see byte package on scope
    DelayUs (20);
    return rxByte;
}

//------------------------------------------------------------------------------
etError I2c_GeneralCallReset(void)
{
unsigned char error = 0;

    i2c_Start();  
    i2c_SendByte(0x00); 
    error = i2c_GetAcknowledge();
    if (!error) i2c_SendByte(0x06);
    error = i2c_GetAcknowledge();
    return error;
}
