#include "Project.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "DELAY.h"
#include <xc.h>

/* Modbus.c
 * There are two ranges of user-defined function codes, i.e. 65 to 72 and from 100 to 110.
 * 
 * Four types of registers: 
 * 
 * Discrete Input - single bit, Read Only
 * Coils - single bit, Read/Write
 * Input - 16 bit integer, Read Only
 * Holding - 16 bit integer, Read/Write
 * 
 * 
 * 
 */
extern unsigned short RH_ADSampled;

extern TableType arrRHCalTable[MAX_RH_SETPOINTS]; 
extern short numberOfSetpoints;
unsigned char functionCode = 0x00;
extern unsigned char DeviceID;
extern unsigned char ModbusRxBuffer[];
extern unsigned char ModbusTxBuffer[];
extern unsigned short ModbusRxLength;
extern unsigned short ModbusTxLength;
unsigned short setpointIndex, address;

extern long RHmeasured;
extern short CorrectedTempCelsius_x10;
extern short TempCelsius_x10;
extern short CorrectedTempFahrenheit_x10;
extern long DewpointMeasured;

unsigned char dataByteHIGH, dataByteLOW;

extern unsigned char CheckCRC (unsigned char *ptrRxModbus, short RxModbusLength);
extern void loadCalibrationTable();

extern unsigned char EEprom_read(unsigned char  address);
extern unsigned char  EEprom_write(unsigned char  address, unsigned char  data);

unsigned char getLOWbyte(unsigned short input);
unsigned char getHIGHbyte(unsigned short input);
unsigned short  combineBytes(unsigned char high, unsigned char low);
unsigned short  generateCRC(unsigned char * data, unsigned char length);
unsigned short processModbus(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);
void prepareOutputArray(unsigned char * data);
char checkException(unsigned char * data);

unsigned short CalculateModbusCRC(unsigned char *input_str, short num_bytes);
unsigned short CalibrateRHsetpoint(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);

unsigned short SetSlaveID(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);
unsigned short CalibrateTemperatureOffset(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);
unsigned short CalibrateTemperatureGain(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);

unsigned short  getStartRegister(unsigned char *ptrRxModbus, unsigned char offset);
unsigned short  getNumberOfRegisters(unsigned char *ptrRxModbus, unsigned char offset);

unsigned short  ReportSlaveID(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);
unsigned short  writeModbusHoldingRegisters(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus, short *ptrLength, unsigned char *errorCode);
unsigned short  ReadInputRegisters(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);
unsigned short  ReadWriteMultipleRegisters(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);
unsigned short SetSlaveID(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);
unsigned short GetSlaveID(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus);

extern unsigned short CalculateRH_X10(short numSetpoints, short ADcounts, TableType *ptrTable);

unsigned char getFunctionCode(unsigned char *ptrRxArray)
{
    if (ptrRxArray == 0) return 0;
    unsigned char functionCode = ptrRxArray[1];
    return functionCode;
    
    /*
    switch (functionCode)
    {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 8:
        case 16:
        case 17:   
        case 23:
            break;
            
        default: 
            functionCode = 0;
            break;
    }
    return functionCode;
    */
}


unsigned short  getStartRegister(unsigned char *ptrRxModbus, unsigned char offset)
{    
    unsigned short ModbusStartRegister;
    ConvertType convert;
    convert.b[1] = ptrRxModbus[2+offset];
    convert.b[0] = ptrRxModbus[3+offset];
    ModbusStartRegister = convert.integer & 0x03FF; // Mask off upper address bits
                                                     // This board uses registers 0-1023 or 0-3FF
    return ModbusStartRegister;
}

unsigned short getNumberOfRegisters(unsigned char *ptrRxModbus, unsigned char offset)
{    
    unsigned short NumberOfRegisters;
    ConvertType convert;
    convert.b[1] = ptrRxModbus[4+offset];
    convert.b[0] = ptrRxModbus[5+offset];
    NumberOfRegisters = convert.integer & 0x03FF; // Mask off upper address bits
                                                  // This board uses registers 0-1023 or 0-3FF
    return NumberOfRegisters;   
}

unsigned short writeModbusHoldingRegisters(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus, short *ptrLength, unsigned char *errorCode)
{
    unsigned short address, numberOfRegisters;
    // Validate data address.  If invalid, send error code
    address = getStartRegister(ptrRxModbus, 0);
    if (address == 0) *errorCode = ERROR_ILLEGAL_DATA_ADDRESS;
    
    // Number of registers to read/write:
    numberOfRegisters = getNumberOfRegisters(ptrRxModbus, 0);
    return 0;
}

/*
    // Compute the CRC for the incoming packet.    
    if (!CheckCRC(ptrRxModbus, RxModbusLength)) return 0;    
    
    // Check slave ID. If there is a match, process request.
    // Otherwise exit routine and return 0 to indicate no response
    if (ptrRxModbus[0] != DeviceID && ptrRxModbus[0] != 0)
        return 0;
*/ 

unsigned short processModbus(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{    
    unsigned short CRCresult;
    unsigned char errorCode = 0;
    unsigned short TxPacketLength = 0;
    unsigned short SubFunctionCode = 0x0000;
    static unsigned short SlaveCommCounter = 0;
    static unsigned short BusCommCounter = 0;
    ConvertType convert;        
    
    BusCommCounter++;
    
    if (ptrRxModbus == NULL || ptrTxModbus == NULL) return 0;
            
    // Validate function code. If invalid, send error code
    functionCode = getFunctionCode(ptrRxModbus);
    
    SlaveCommCounter++;    
    
    switch (functionCode)
    {        
        case 0:
            errorCode = ERROR_ILLEGAL_FUNCTION;
            break;
        case 1: // Read coils
            break;
        case 2: // Read discrete inputs
            break;
        case 3: // Read holding registers
            break;
        case 4: // Read input registers 
            TxPacketLength = ReadInputRegisters(ptrRxModbus, ptrTxModbus);
            break;
        case 5: // Write Single Coil
            break;
        case 6: // Write Single Register
            break;
        case 7: // Read Exception Status
            break;
        case 8: // Diagnostics
            convert.b[0] = ptrRxModbus[3];
            convert.b[1] = ptrRxModbus[2];
            SubFunctionCode = convert.integer;
            if (SubFunctionCode == 0) 
            {
                ptrTxModbus[4] = ptrRxModbus[4];
                ptrTxModbus[5] = ptrRxModbus[5];
                TxPacketLength = 6;                
            }
            else if (SubFunctionCode == 11) 
            {
                convert.integer = BusCommCounter;
                ptrTxModbus[4] = convert.b[MSB_INDEX];
                ptrTxModbus[5] = convert.b[LSB_INDEX];
                TxPacketLength = 6;                
            }                        
            else if (SubFunctionCode == 14) 
            {
                convert.integer = SlaveCommCounter;
                ptrTxModbus[4] = convert.b[MSB_INDEX];
                ptrTxModbus[5] = convert.b[LSB_INDEX];
                TxPacketLength = 6;                
            }    
            else errorCode = ERROR_ILLEGAL_FUNCTION;
            break;
        case 11: // Get Comm Event Counter
            break;
        case 12: // Get Comm Event Log
            break;
        case 15: // Write Multiple Coils
            break;
        case 16: // Write Multiple Registers 
            break;
        case 17: // Report Server ID
            TxPacketLength = ReportSlaveID(ptrRxModbus, ptrTxModbus);
            break;
        case 20: // Read File Record
            break;
        case 21: // Write File Record
            break;
        case 22: // Mask Write Register
            break;
        case 23: // Read/Write Multiple Registers
            TxPacketLength = ReadWriteMultipleRegisters(ptrRxModbus, ptrTxModbus);            
            break;
        case 24: // Read FIFO Queue
            break;
        case 43: // Encapsulate Interface Transport
            break;
        case 123: // Calibrate setpoint
            TxPacketLength = CalibrateRHsetpoint(ptrRxModbus, ptrTxModbus);
            loadCalibrationTable();
            break;
        case GET_SLAVE_ID_FUNC_CODE: // Get Slave ID
            TxPacketLength = GetSlaveID (ptrRxModbus, ptrTxModbus);
            break;            
        case SET_SLAVE_ID_FUNC_CODE: // Set Slave ID
            TxPacketLength = SetSlaveID (ptrRxModbus, ptrTxModbus);
            break;
        case CALIBRATE_TEMP_OFFSET_FUNC_CODE: // Calibrate temperature offset
            TxPacketLength = CalibrateTemperatureOffset (ptrRxModbus, ptrTxModbus);
            loadCalibrationTable();
            break;
        case CALIBRATE_TEMP_GAIN_FUNC_CODE:  // Calibrate temperature gain
            TxPacketLength = CalibrateTemperatureGain (ptrRxModbus, ptrTxModbus);
            loadCalibrationTable();
            break;
            
        default:
            break;            
    }        
    // Special case: if Slave ID is a zero, then command is for all sensors, so don't respond:
    if (ptrRxModbus[0] == 0) return 0;
    
    if (errorCode) 
    {
        functionCode |= 0x80;    
        ptrTxModbus[2] = errorCode;
    }
    ptrTxModbus[0] = ptrRxModbus[0]; // Slave address stays the same for all commands
    ptrTxModbus[1] = functionCode; // Function code stays the same for all commands    
            
    CRCresult = CalculateModbusCRC(ptrTxModbus, TxPacketLength);
    
    convert.integer = CRCresult;
    ptrTxModbus[TxPacketLength] = convert.b[0];
    ptrTxModbus[TxPacketLength+1] = convert.b[1];
    TxPacketLength = TxPacketLength + 2;    

    // Otherwise return the number of byte to send back to master:
    return TxPacketLength;
}

unsigned short ReportSlaveID(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{
    unsigned short packetLength;
    
    ptrTxModbus[0] = ptrRxModbus[0];    // Slave address
    ptrTxModbus[1] = ptrRxModbus[1];    // Function code 
    ptrTxModbus[2] = 0x02;              // Two data bytes
    ptrTxModbus[3] = ptrTxModbus[0];    // Server ID is same as Slave ID, for now
    ptrTxModbus[4] = 0xFF;              // Run indicator status = ON
    packetLength = 5;                   
    return packetLength;    
}


unsigned short ReadInputRegisters(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{
    unsigned short address, packetLength = 0, numberOfRegisters = 0;
    unsigned short errorCode = 0;
    ConvertType convert;
    
    ptrTxModbus[0] = ptrRxModbus[0]; // Slave address stays the same for all commands
    ptrTxModbus[1] = ptrRxModbus[1]; // Function code stays the same for all commands        
    
    // Validate data address.  If invalid, send error code
    address = getStartRegister(ptrRxModbus, 0);
    numberOfRegisters = getNumberOfRegisters(ptrRxModbus, 0);
    
    if (address != VERSION_NUMBER_ADDRESS
        && address != RH_INPUT_ADDRESS 
        && address != DEWPOINT_INPUT_ADDRESS
        && address != TEMPERATURE_CELSIUS_INPUT_ADDRESS
        && address != TEMPERATURE_FAHRENHEIT_INPUT_ADDRESS       
        && address != VERSION_NUMBER_ADDRESS) errorCode = ERROR_ILLEGAL_DATA_ADDRESS;
    
    if (numberOfRegisters != 1) errorCode = ERROR_ILLEGAL_DATA_VALUE;
    
    if (errorCode)
    {
        ptrTxModbus[1] = ptrTxModbus[1] | 0x80; // Set highest bit on function code to indicate an error
        ptrTxModbus[2] = errorCode;
        packetLength = 3;
    }
    else
    {        
        if (address == VERSION_NUMBER_ADDRESS) convert.integer = VERSION_NUMBER;
        else if (address == RH_INPUT_ADDRESS) convert.integer = (unsigned short) RHmeasured;
        else if (address == DEWPOINT_INPUT_ADDRESS) convert.integer = (unsigned short) DewpointMeasured;
        else if (address == TEMPERATURE_CELSIUS_INPUT_ADDRESS) 
        {
            convert.integer = (unsigned short) CorrectedTempCelsius_x10;            
            LED_OUT = 1;
        }
        else if (address == TEMPERATURE_FAHRENHEIT_INPUT_ADDRESS) convert.integer = (unsigned short) CorrectedTempFahrenheit_x10;
        else return 0;
        
        ptrTxModbus[2] = 0x02; // Number of data bytes to be transmitted = 2
        ptrTxModbus[3] = convert.b[1]; // MSB data byte comes first
        ptrTxModbus[4] = convert.b[0]; // LSB data byte comes second        
        packetLength = 5;
    }    
    return packetLength;
}


// CALIBRATE COMMAND:
//      0               1                2             3             4               5           6-7
//   slave ID       func code     total setpoints  setpoint #     RH MSB           RH LSB        CRC
//
// RESPONSE:
//      0               1                2             3             4               5           6-7
//   slave ID       func code         RH MSB         RH LSB        AD MSB          AD LSB        CRC 
unsigned short CalibrateRHsetpoint(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{
    unsigned char errorCode = 0;  
    unsigned short packetLength;
    ConvertType convert;
    
    ptrTxModbus[0] = ptrRxModbus[0]; // Slave address stays the same for all commands
    ptrTxModbus[1] = ptrRxModbus[1]; // Function code stays the same for all commands       
    ptrTxModbus[2] = ptrRxModbus[2]; // Number of setpoints
    ptrTxModbus[3] = ptrRxModbus[3]; // Setpoint index
    ptrTxModbus[4] = ptrRxModbus[4]; // RH MSB
    ptrTxModbus[5] = ptrRxModbus[5]; // RH LSB
            
    if (ptrRxModbus[2] <= MAX_RH_SETPOINTS && ptrRxModbus[3] < MAX_RH_SETPOINTS)
    {
        numberOfSetpoints = (short) ptrRxModbus[2];
        setpointIndex = (short) ptrRxModbus[3];       
        address = (setpointIndex * 4) + CAL_START_EEPROM_ADDRESS;
         
        // STORE CURRENT TWO BYTE AD COUNT IN EEPROM AND PLACE IN CAL TABLE:        
        convert.integer  = RH_ADSampled;            
        EEprom_write (address, convert.b[0]);
        EEprom_write (address+1, convert.b[1]);
        arrRHCalTable[setpointIndex].ADcounts = RH_ADSampled;  
        
        // STORE TWO BYTE RH VALUE SENT FROM MODBUS MASTER IN EEPROM AND PLACE IN CAL TABLE:        
        convert.b[1] = ptrRxModbus[4];  // MSB first
        convert.b[0] = ptrRxModbus[5];  // LSB second
        arrRHCalTable[setpointIndex].RHpercent = convert.integer;
        EEprom_write (address+2, convert.b[0]);
        EEprom_write (address+3, convert.b[1]);            
        EEprom_write (NUMBER_OF_SETPOINTS_EEPROM_ADDRESS, numberOfSetpoints);
        
        // SEND AD COUNT BACK TO MASTER:
        ptrTxModbus[6] = convert.b[1]; // AD count MSB
        ptrTxModbus[7] = convert.b[0]; // AD count LSB             
    } else errorCode = ERROR_ILLEGAL_DATA_VALUE;
    
    if (errorCode) 
    {
        ptrTxModbus[1] = ptrTxModbus[1] | 0x80; // Set highest bit on function code to indicate an error
        ptrTxModbus[2] = errorCode;
        packetLength = 3;
    }
    else packetLength = 8;
    return packetLength;   
}
    
// READ MULTIPLE REGISTERS COMMAND:
//      0            1               2-3                  4-5                  6-7                    8-9                   10
//   slave ID    func code     Read Start Address    Quantity to Read     Write Start Address    Quantity to Write   Write Byte Count
//
// RESPONSE:
//      0             1                2             
//   slave ID    func code      Write Byte Count     
unsigned short ReadWriteMultipleRegisters(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{
    unsigned short i, RxIndex, TxIndex, EEPROMaddress, readStartAddress, writeStartAddress, packetLength, numberOfReadRegisters, numberOfWriteRegisters;
    unsigned char errorCode = 0;    
    
    ptrTxModbus[0] = ptrRxModbus[0]; // Slave address stays the same for all commands
    ptrTxModbus[1] = ptrRxModbus[1]; // Function code stays the same for all commands    
    
    readStartAddress = getStartRegister(ptrRxModbus, 0);
    writeStartAddress = getStartRegister(ptrRxModbus, 4);
    
    numberOfReadRegisters = getNumberOfRegisters(ptrRxModbus, 0);
    ptrTxModbus[2] = numberOfReadRegisters * 2;
    
    numberOfWriteRegisters = getNumberOfRegisters(ptrRxModbus, 4);
    EEPROMaddress = writeStartAddress;
    RxIndex = 11;    
    // Get write data and copy it to EEPROM     
    for (i = 0; i < numberOfWriteRegisters; i++)
    {     
        dataByteHIGH = ptrRxModbus[RxIndex++];
        dataByteLOW = ptrRxModbus[RxIndex++];
        
        EEprom_write (EEPROMaddress++, dataByteLOW);  
        DelayMs(1);                        
        EEprom_write (EEPROMaddress++, dataByteHIGH);
        DelayMs(1);
    }    
    while(WR) continue;
    EEPROMaddress = readStartAddress;    
    TxIndex = 3;    
    // Now read data from EEPROM
    for (i = 0; i < numberOfReadRegisters; i++)
    {    
        dataByteLOW = EEprom_read (EEPROMaddress++);
        dataByteHIGH = EEprom_read (EEPROMaddress++);
        
        ptrTxModbus[TxIndex++] = dataByteHIGH;
        ptrTxModbus[TxIndex++] = dataByteLOW;   
    }

    if (errorCode) 
    {
        ptrTxModbus[1] = ptrTxModbus[1] | 0x80; // Set highest bit on function code to indicate an error
        ptrTxModbus[2] = errorCode;
        packetLength = 3;
    }
    else packetLength = (numberOfReadRegisters * 2) + 3;        
    return packetLength;
}



// CALIBRATE TEMPERATURE OFFSET COMMAND
//      0               1                 2             3             4-5
//   Slave ID       func code      Ref Temp C MSB    Temp C LSB       CRC   
//
// RESPONSE:
//      0               1                 2             3              4             5          6-7
//   Slave ID       func code      Ref Temp C MSB    Temp C LSB    Offset MSB   Offset LSB    CRC   

unsigned short CalibrateTemperatureOffset(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{
    unsigned short packetLength; 
    unsigned char MSBbyte, LSBbyte;
    ConvertType convert;
    
    ptrTxModbus[0] = ptrRxModbus[0]; // Slave address stays the same for all commands
    ptrTxModbus[1] = ptrRxModbus[1]; // Function code stays the same for all commands       
    ptrTxModbus[2] = ptrRxModbus[2]; // Temperature Celsius MSB
    ptrTxModbus[3] = ptrRxModbus[3]; // Temperature Celsius LSB           
    
    // Get reference offset temperature from master:
    MSBbyte = ptrRxModbus[2];  // MSB first
    LSBbyte = ptrRxModbus[3];  // LSB second
    
    // Store reference offset temperature in EEPROM LSB first
    address = REFERENCE_OFFSET_TEMPERATURE_ADDRESS;
    EEprom_write (address, LSBbyte);
    EEprom_write (address+1, MSBbyte);    
    
    // Store measured offset temperature in EEPROM and also send back to master:
    address = MEASURED_OFFSET_TEMPERATURE_ADDRESS;
    convert.integer = (unsigned short) TempCelsius_x10;
    EEprom_write (address, convert.b[0]);
    EEprom_write (address+1, convert.b[1]);
    
    ptrTxModbus[4] = convert.b[1]; // Send measured temp back to Master, MSB first
    ptrTxModbus[5] = convert.b[0]; 
    
    packetLength = 6;
    return packetLength;   
}


// CALIBRATE TEMPERATURE GAIN COMMAND
//      0               1                 2                  3             4-5
//   Slave ID       func code        Ref Temp C MSB    Ref Temp C LSB      CRC   
//
// RESPONSE:
//      0               1                 2                   3             4             5          6-7
//   Slave ID       func code        Ref Temp C MSB    Ref Temp C LSB   Offset MSB   Offset LSB      CRC   
unsigned short CalibrateTemperatureGain(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{
    unsigned short packetLength;
    unsigned char MSBbyte, LSBbyte;
    ConvertType convert;
    
    ptrTxModbus[0] = ptrRxModbus[0]; // Slave address stays the same for all commands
    ptrTxModbus[1] = ptrRxModbus[1]; // Function code stays the same for all commands       
    ptrTxModbus[2] = ptrRxModbus[2]; // Temperature Celsius MSB
    ptrTxModbus[3] = ptrRxModbus[3]; // Temperature Celsius LSB
                
    // Get reference temperature from master:
    MSBbyte = ptrRxModbus[2];  // MSB first
    LSBbyte = ptrRxModbus[3];  // LSB second
    
    // Store reference temperature in EEPROM, LSB first
    address = REFERENCE_GAIN_TEMPERATURE_ADDRESS;
    EEprom_write (address, LSBbyte);
    EEprom_write (address+1, MSBbyte);
    
    // Store current measured temperature in EEPROM:
    address = MEASURED_GAIN_TEMPERATURE_ADDRESS; 
    convert.integer = (unsigned short) TempCelsius_x10;
    EEprom_write (address, convert.b[0]);
    EEprom_write (address+1, convert.b[1]);
    
    // Send measured temperature back to master:
    ptrTxModbus[4] = convert.b[1]; // Send MSB first
    ptrTxModbus[5] = convert.b[0]; // LSB        
    
    packetLength = 6;  // Total bytes to send back to master = 6 not including CRC
    return packetLength;   
}

// SET SLAVE ID COMMAND
//      0            1               2
//   slave ID    func code       New Slave ID
//
// RESPONSE:
//      0             1                2             
//   slave ID    func code       New Slave ID
unsigned short SetSlaveID(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{    
    ptrTxModbus[0] = ptrRxModbus[0]; // Slave address stays the same for all commands
    ptrTxModbus[1] = ptrRxModbus[1]; // Function code stays the same for all commands    
    ptrTxModbus[2] = ptrRxModbus[2]; // Return new ID    
    
    if (ptrRxModbus[2] == 0xFF && DeviceID != 0xFF) return 0;
       
    DeviceID = ptrRxModbus[2]; // Set new device ID
    EEprom_write(SLAVE_ID_EEPROM_ADDRESS, DeviceID);      
    while(WR) continue;
    
    return 3;
}

unsigned short GetSlaveID(unsigned char *ptrRxModbus, unsigned char *ptrTxModbus)
{    
    ptrTxModbus[0] = ptrRxModbus[0]; // Slave address stays the same for all commands
    ptrTxModbus[1] = ptrRxModbus[1]; // Function code stays the same for all commands        
    ptrTxModbus[2] = DeviceID;       // Return device ID    
    
    return 3;
}


