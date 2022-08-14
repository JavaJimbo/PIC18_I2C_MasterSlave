//==============================================================================
// S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT85 Sample Code
// File      :  sht85.c
// Author    :  RFU
// Date      :  17-May-2018
// Brief     :  Sensor Layer: Implementation of functions for sensor access.
//==============================================================================

#include "Project.h"
#define CRC_POLYNOMIAL  0x31 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001 ???? was 0x131



static etError StartWriteAccess(void);
static etError StartReadAccess(void);
static void StopAccess(void);
static etError WriteCommand(etCommands command);
static etError Read2BytesAndCrc(uint16_t* data, bool finAck, uint8_t timeout);
static uint8_t CalcCrc(uint8_t data[], uint8_t nbrOfBytes);
static etError CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
static float CalcTemperatureCelsius(uint16_t rawValue);
static float CalcTemperatureFahrenheit(uint16_t rawValue);
static float CalcHumidity(uint16_t rawValue);

//------------------------------------------------------------------------------
void SHT85_Init(void)
{
  InitializeI2Cmaster(); // init I2C
}

//------------------------------------------------------------------------------
etError SHT85_ReadSerialNumber(uint32_t *serialNumber)
{
  etError error; // error code
  uint16_t serialNumWords[2];
  uint32_t serialNum32;
  
    error = StartWriteAccess();
  
    // write "read serial number" command
    // if (!error) error = WriteCommand(CMD_READ_SERIALNBR);  
    if (!error) error = WriteCommand(CMD_READ_SHT4X_SERIAL_NUMBER);
    else printf("Error #1");
  
    // if no error, start read access
    if (!error) error = StartReadAccess();
    else printf("Error #2");
  
    // if no error, read first serial number word
    if (!error) error = Read2BytesAndCrc(&serialNumWords[0], true, 100);
    else printf("Error #3");
  
  // if no error, read second serial number word
    if (!error) error = Read2BytesAndCrc(&serialNumWords[1], false, 0);
    
    StopAccess();
  
    // if no error, calc serial number as 32-bit integer
    if (!error) {
        serialNum32 = ( uint32_t) serialNumWords[0];
        *serialNumber = (serialNum32 << 16);
        serialNum32 = ( uint32_t) serialNumWords[1];
        *serialNumber = *serialNumber | serialNum32;
    }
    else *serialNumber = 0x0000;
  
    return error;
}

//------------------------------------------------------------------------------
etError SHT85_ReadStatus(uint16_t* status)
{
  etError error; // error code

  error = StartWriteAccess();
  
  // if no error, write "read status" command
  if (!error) error = WriteCommand(CMD_READ_STATUS);
  
  // if no error, start read access
  if (!error) error = StartReadAccess();
  
  // if no error, read status
  if (!error) error = Read2BytesAndCrc(status, false, 0);
  
  StopAccess();
  
  return error;
}

//------------------------------------------------------------------------------
etError SHT85_ClearAllAlertFlags(void)
{
  etError error; // error code
  
  error = StartWriteAccess();
  
  // if no error, write clear status register command
  if (!error)  error = WriteCommand(CMD_CLEAR_STATUS);
  
  StopAccess();
  
  return error;
}

//------------------------------------------------------------------------------
etError SHT85_SingleMeasurment(float* temperature, float* humidity,
                               etSingleMeasureModes measureMode,
                               uint8_t timeout)
{
  etError  error;           // error code
  uint16_t rawValueTemp;    // temperature raw value from sensor
  uint16_t rawValueHumi;    // humidity raw value from sensor
  
  error  = StartWriteAccess();
  
  // if no error, start measurement
  if (!error) error = WriteCommand((etCommands)measureMode);
  // if no error, wait until measurement ready
  if (!error)
  {
    // poll every 1ms for measurement ready until timeout
    while (timeout--) 
    {
      // check if the measurement has finished
      error = StartReadAccess();
      
      // if measurement has finished -> exit loop
      if (!error) break;
      
      DelayMs(1);
    }
    
    // check for timeout error
    if (timeout == 0)  error = TIMEOUT_ERROR;
  }
  
  // if no error, read temperature and humidity raw values
  if (!error)
  {
    error |= Read2BytesAndCrc(&rawValueTemp, true, 0);
    error |= Read2BytesAndCrc(&rawValueHumi, false, 0);
  }
  
  StopAccess();
  
  // if no error, calculate temperature in °C and humidity in %RH
    if (!error) 
    {
        *temperature = CalcTemperatureFahrenheit(rawValueTemp);
        *humidity = CalcHumidity(rawValueHumi);
    }  
    return error;
}

//------------------------------------------------------------------------------
etError SHT85_StartPeriodicMeasurment(etPeriodicMeasureModes measureMode)
{
etError error; // error code
  
    error = StartWriteAccess();
  
    // if no error, start periodic measurement 
    if (!error) error = WriteCommand ((etCommands) measureMode);
  
    StopAccess();
  
  return error;
}


//------------------------------------------------------------------------------
etError SHT85_StopPeriodicMeasurment(void)
{
etError error; // error code
  
    error = StartWriteAccess();
  
    // if no error, write breake command
    if (!error) error = WriteCommand(CMD_BREAK);
  
    StopAccess();
  
    return error;
}

//------------------------------------------------------------------------------
etError SHT4X_ReadSensor(float* temperature, float* humidity)
{
etError  error = 0;        // error code
uint16_t rawValueTemp; // raw temperature from sensor
uint16_t rawValueHumi; // raw humidity from sensor
  
     error = StartWriteAccess();
  
    // if no error, read measurements
    if (!error) error = WriteCommand(CMD_SHT4X_MEASURE_TRH_HI_HIGH_PREC);    
    if (!error) error = StartReadAccess();      
    if (!error) error = Read2BytesAndCrc(&rawValueTemp, true, 0);  
    if (!error) error = Read2BytesAndCrc(&rawValueHumi, false, 0);  
  
    // if no error, calculate temperature in °C and humidity in %RH
    if (!error) 
    { 
        *temperature = CalcTemperatureFahrenheit(rawValueTemp);
        *humidity = CalcHumidity(rawValueHumi);
    }  
    StopAccess();  
    return error;
}

//------------------------------------------------------------------------------
etError SHT85_ReadMeasurementBuffer(float* temperature, float* humidity)
{
etError  error = 0;        // error code
uint16_t rawValueTemp; // raw temperature from sensor
uint16_t rawValueHumi; // raw humidity from sensor
  
     error = StartWriteAccess();
  
    // if no error, read measurements
    if (!error) error = WriteCommand(CMD_FETCH_DATA);    
    if (!error) error = StartReadAccess();      
    if (!error) error = Read2BytesAndCrc(&rawValueTemp, true, 0);  
    if (!error) error = Read2BytesAndCrc(&rawValueHumi, false, 0);  
  
    // if no error, calculate temperature in °C and humidity in %RH
    if (!error) 
    { 
        *temperature = CalcTemperatureFahrenheit(rawValueTemp);
        *humidity = CalcHumidity(rawValueHumi);
    }  
    StopAccess();  
    return error;
}

//------------------------------------------------------------------------------
etError SHT85_EnableHeater(void)
{
  etError error; // error code
  
    error = StartWriteAccess();  
    // if no error, write heater enable command
    if (!error) error = WriteCommand(CMD_HEATER_ENABLE);  
    StopAccess();  
    
    return error;
}

//------------------------------------------------------------------------------
etError SHT85_DisableHeater(void)
{
etError error; // error code
  
    error = StartWriteAccess();
  
    // if no error, write heater disable command
    if (!error)  error = WriteCommand(CMD_HEATER_DISABLE);
  
    StopAccess();
  
    return error;
}

//------------------------------------------------------------------------------
etError SHT85_SoftReset(void)
{
    etError error; 
    
    error = StartWriteAccess();      
    if (!error) error  = WriteCommand(CMD_SOFT_RESET);  
    StopAccess();  
    DelayMs(50);  
    
    return error;
}

//------------------------------------------------------------------------------
static etError StartWriteAccess(void)
{
unsigned char ACKresponse;      
    i2c_Start();
    i2c_SendControlByte(SENSIRION, I2C_WRITE);
    ACKresponse = i2c_GetAcknowledge();  
    return NO_ERROR;
}

//------------------------------------------------------------------------------
static etError StartReadAccess(void)
{
unsigned char ACKresponse;    
    i2c_Start();
    i2c_SendControlByte(SENSIRION, I2C_READ);
    ACKresponse = i2c_GetAcknowledge();
    return ACKresponse;
}

//------------------------------------------------------------------------------
static void StopAccess(void)
{
   i2c_Stop();
}

//------------------------------------------------------------------------------
static etError WriteCommand(etCommands command)
{
  etError error = 0x00; // error code
  
  // write the upper 8 bits of the command to the sensor
  error = I2C_WriteByteGetAck(command >> 8);
  
  // write the lower 8 bits of the command to the sensor
  // error |= I2C_WriteByteGetAck(command & 0xFF); $$$$
  
  return error;
}

//------------------------------------------------------------------------------
static etError Read2BytesAndCrc(uint16_t* data, bool finAck, uint8_t timeout)
{
  etError error;    // error code
  uint8_t bytes[2]; // read data array
  uint8_t checksum; // checksum byte
  
  // read two data bytes and one checksum byte
  bytes[0] = I2C_ReadByteSendAck(ACK);
  bytes[1] = I2C_ReadByteSendAck(ACK);
  checksum = I2C_ReadByteSendAck(finAck ? ACK : NO_ACK);

  // verify checksum
  error = CheckCrc(bytes, 2, checksum);
  
  // combine the two bytes to a 16-bit value
  *data = (bytes[0] << 8) | bytes[1];
  
  return error;
}

//------------------------------------------------------------------------------
static uint8_t CalcCrc(uint8_t data[], uint8_t nbrOfBytes)
{
  uint8_t ByteBit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter
  
  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
    crc ^= (data[byteCtr]);
    for(ByteBit = 8; ByteBit > 0; --ByteBit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      } else {
        crc = (crc << 1);
      }
    }
  }
  
  return crc;
}

//------------------------------------------------------------------------------
static etError CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
  // calculates 8-Bit checksum
  uint8_t crc = CalcCrc(data, nbrOfBytes);
  
  // verify checksum
  return (crc != checksum) ? CHECKSUM_ERROR : NO_ERROR;
  // return NO_ERROR; // $$$$
}

//------------------------------------------------------------------------------
/*
static float CalcTemperatureCelsius(uint16_t rawValue)
{
  // calculate temperature [°C]
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}
*/

static float CalcTemperatureFahrenheit(uint16_t rawValue)
{
  // calculate temperature [°F]
  // T = -49 + 315 * rawValue / (2^16-1)
  return 315.0f * (float)rawValue / 65535.0f - 49.0f;
}


//------------------------------------------------------------------------------
static float CalcHumidity(uint16_t rawValue)
{
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100.0f * (float)rawValue / 65535.0f;
}
