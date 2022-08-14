
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "DELAY.h"
#include <xc.h>
#include "Project.h"
#include "EEPROM_I2C.h"

union {
    unsigned char b[2];
    unsigned int integer;
} convert;

#define ACK 0
#define NACK 1
#define SENSIRION_DATA_LENGTH 6

unsigned char GetSingleShotSensirionMeasurement(float *TemperatureF, float *RH)
{
unsigned char errorFlag = 0x00;
unsigned char response;
unsigned char retries = 0;
unsigned char singleShotData[SENSIRION_DATA_LENGTH];
short i = 0;
float floValue;


    i2c_Start();
    i2c_SendControlByte(SENSIRION, I2C_WRITE); // Send Sensirion address
    errorFlag = errorFlag | i2c_GetAcknowledge();
    
	i2c_SendByte(0x20);  // Send high byte of single shot command WAS 0x24
	errorFlag = errorFlag | i2c_GetAcknowledge();    
    
	i2c_SendByte(0x32);  // Send low byte of single shot command (for high reliability))
	errorFlag = errorFlag | i2c_GetAcknowledge();    
    // i2c_Stop(); 
    
    
    if (errorFlag)
    {
        *TemperatureF = 0;
        *RH = 0;
        printf("\rERROR");
        return errorFlag;
    }
    
        i2c_Start();
        i2c_SendControlByte(SENSIRION, I2C_READ); // Check for ACK to indicating measurement complete    
        if (i2c_GetAcknowledge() == NACK)
        {
            i2c_Stop();
            *TemperatureF = 0;
            *RH = 0;
            printf("\rNO DATA");
            return errorFlag;            
        }
    /*
    retries = 0;
    do {        
        DelayMs(1);
        i2c_Start();
        i2c_SendControlByte(SENSIRION, I2C_READ); // Check for ACK to indicating measurement complete
        if (i2c_GetAcknowledge() == ACK) break;
        i2c_Stop();
        retries++;
    } while(retries < 10);    
    */
        
    //if (retries < 10)
    {
		i = 0;	
		do {
			// Now read each DATA byte, followed by acknowledge:
			singleShotData[i] = i2c_ReadByte();
			i++;		
			if (i < SENSIRION_DATA_LENGTH)
				i2c_SendAcknowledge(I2C_MORE);
			else
				i2c_SendAcknowledge(I2C_LAST);	// Leave acknowledge high for last byte	
		} while (i < SENSIRION_DATA_LENGTH);
		i2c_Stop();   
        printf("\r\rTMSB: %02X, TLSB: %02X, CRC: %02X, RH MSB: %02X, RH LSB: %02X, CRC: %02X", singleShotData[0], singleShotData[1], singleShotData[2], singleShotData[3], singleShotData[4], singleShotData[5]);
        
        // Get temp Fahrenheit:
        convert.b[0] = singleShotData[1];
        convert.b[1] = singleShotData[0];
        floValue = (float)convert.integer;                
        *TemperatureF = -49 + (315 * (floValue / 65535));
        
        // Get %RH:
        convert.b[0] = singleShotData[5];
        convert.b[1] = singleShotData[4];    
        floValue = (float)convert.integer; 
        *RH = 100 * (floValue / 65535);
    }
/*        
    else 
    {
        i2c_Stop();
        errorFlag = 0xff;
        printf("\rSENSIRION TIMEOUT");
        return errorFlag;
    }    
*/ 
    return errorFlag;
}
