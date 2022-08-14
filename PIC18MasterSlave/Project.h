// #include "Project.h"

#include <xc.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "TCA9548_MPX.h"
#include "sht85.h"
#include "i2c_hal.h"

// Uncomment ONE of these defines:
#define MASTER
 // #define SLAVE

#include "EEPROM_I2C.h"

#ifdef MASTER
    #include "MASTER_I2C.h"
#endif

#ifdef SLAVE
    #include "SLAVE_I2C.h" 
#endif
#define SLAVE_ADDRESS 0x27
typedef struct
{
    unsigned short ADcounts;
    unsigned short RHpercent;
} TableType;


#define CAL_START_EEPROM_ADDRESS 0x00

#include "DELAY.h"
#include "TCA9548_MPX.h"
#include "StdDefs.h"

#include "MiscFunctions.h"




#define TEST_OUT LATDbits.LATD2


#define MAXBUFFER 32


#define TRUE true
#define FALSE false
#define FOREVER 1

#define PWM_OUT LATCbits.LATC0 // $$$$ 

#define CR 13
#define LF 10
#define BACKSPACE 8
#define SPACE 32
#define ESCAPE 27
#define RIGHT_ARROW 67
#define LEFT_ARROW 68
#define UP_ARROW 65
#define DOWN_ARROW 66



typedef union 
{
	unsigned char b[2];
	unsigned short integer;
} ConvertIntType;


typedef union 
{
	unsigned char b[4];
	float floVal;
} ConvertFloatType;

