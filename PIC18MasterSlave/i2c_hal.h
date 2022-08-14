//==============================================================================
// S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT3x Sample Code
// File      :  i2c_hal.h
// Author    :  RFU
// Date      :  17-Mai-2018
// Controller:  STM32F100RB
// IDE       :  µVision V5.25.2.0
// Compiler  :  Armcc
// Brief     :  I2C hardware abstraction layer 
//==============================================================================

#ifndef I2C_HAL_H
#define I2C_HAL_H

#include "system.h"
// #include "Project.h"
#include <stdint.h>

typedef enum{
  ACK    = 0,
  NO_ACK = 1,
}etI2cAck;


//==============================================================================
etError I2C_WriteByteGetAck(uint8_t txByte);
//==============================================================================
// Writes a byte to I2C-Bus and checks acknowledge.
//------------------------------------------------------------------------------
// input:  txByte       transmit byte
//
// return: error:       ACK_ERROR = no acknowledgment
//                      NO_ERROR  = no error

//==============================================================================
uint8_t I2C_ReadByteSendAck(etI2cAck ack);
//==============================================================================
// Reads a byte on I2C-Bus.
//------------------------------------------------------------------------------
// input:  ack          Acknowledge: ACK or NO_ACK
//
// return: rxByte

//==============================================================================
etError I2c_GeneralCallReset(void);
//==============================================================================
// Reset all i2c devices through general call.
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR = no acknowledgment
//                      NO_ERROR  = no error

#endif
