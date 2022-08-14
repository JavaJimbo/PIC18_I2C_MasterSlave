/*      PIC18_ProtoDemo for MC Pros PIC-P40-28 Prototype board using PIC18F44K22 
 *      Compiler: XC8 V2.36 MPLABX IDE V6.00
 *
 * 7-27-22:    Created new project. Board voltage set to 3.3V
 *      Got I2C writes and reads working for 24LC256 using I2C2
 *      7-29-22: Imported GitHub routines for reading Sensirion SHT85 humidity & temp sensor
 *      7-31-22: Added routines for the TCA9548A multiplexer - works great.
 *      8-8-22: 
 *      8-8-22:
 *      8-9-22:  I2C Master and Slave work for both WRITES and READS,
 *               but slave keeps dropping out and not recovering,
 *               although it doesn't appear to be latching up.
 *               Added PLL, increased system clock to 64 Mhz, set I2C clock to 100 kHz          
 */
#include "Project.h"
#include "Config.h"

// #define PUSHBUTTON LATEbits.LATE2
#define PUSHBUTTON PORTEbits.RE2

#define RCAL 3.6
#define ADSCALE 1023 //for unsigned conversion 10 sig bits
#define ADREF 3.3// 1.1 // Was 3.3 with Vdd connected to A/D Vrref+
#define ADOFFSET 0 // 2.2
#define CHARGE_TIME_US 750 // Capacitor charging time in microseconds

#define CURRENT_CAL_MODE 0
#define SENSOR_MEASURE_MODE 1
#define REVERSE_SENSOR_MODE 2
#define TRISTATE_CAP() TRISBbits.RB5 = 1
#define TRISTATE_RESISTOR() TRISBbits.RB3 = 1
#define SHORT_CAP_TO_GROUND() TRISBbits.RB5 = 0; PORTBbits.RB5 = 0
#define REVERSE_SHORT_CAP_TO_GROUND() TRISBbits.RB4 = 0; PORTBbits.RB4 = 0
#define SHORT_RESISTOR_TO_GROUND() TRISBbits.RB3 = 0; PORTBbits.RB3 = 0

#define THERMISTOR_TABLE_SIZE 11      

unsigned char numberOfSetpoints;
int I2CDataBytesReceived = 0;
unsigned char I2C_WRITEdata[MAXBUFFER], I2C_READdata[MAXBUFFER] = "START READ";

unsigned char errorFlag = 0;
unsigned char OutBuffer[64] = "You are my only diddy wah diddy, you understand me, don't you?";
unsigned char InBuffer[64];
                                                    //   0          5       10      15       20         25      30    35      40      45       50
float arrThermistorResistance[THERMISTOR_TABLE_SIZE] = {32624.0, 25381.0, 19897.0, 15711.0, 12493.0, 10000.0, 8056.0, 6529.7, 5323.9, 4365.3, 3598.7};

extern void putch(unsigned char ch);
extern void InitializeHardware();

float MeasureThermistorTemperature(void);

float CurrentCalibration(void);
float CapMeasurement(float CalCurrent);
float ReverseCapMeasurement(float CalCurrent);
unsigned char CTMUsetup(int Mode);

unsigned char RxBuffer[MAXBUFFER+1];
unsigned char RxBufferFull = false;

// CTMUISrc = 0; //float values stored for calcs

unsigned char  PIC_EEprom_write(unsigned char  address, unsigned char  data);
unsigned char PIC_EEprom_read(unsigned char  address);

#define SYSTEM_CLOCK 64000000
#define SHT85_CHANNEL 1
#define EEPROM_CHANNEL 0

#define MAX_RH_SETPOINTS 64
TableType arrRHCalTable[MAX_RH_SETPOINTS]; 

void main()
{          
    /*
    etError  error;          // error code
    uint32_t serialNumber;   // serial number
    unsigned char device;
    unsigned short startAddress;
    unsigned char OutData[64], InData[64];
    unsigned short numBytes;
    unsigned char MPXchannel = 0, MPXcheck = 0;
    float    temperature;    // temperature [°C]
    float    humidity;       // relative humidity [%RH]    
    unsigned char MultiplexerID = 0;
    */
    
    unsigned char OutBytes[MAXBUFFER], InBytes[MAXBUFFER];
    int length = 0;
    unsigned long trial = 0;
    unsigned char errorFlag = 0;
    short i;
    ConvertFloatType FConvert, BConvert;
    unsigned char FBytes[4];
    
    InitializeHardware();      
    
#ifdef SLAVE
    printf("\r\rSTARTING INIT TEST I2C SLAVE - 4123");
    Initialize_I2C_Slave();
#else    
    printf("\r\rSTARTING I2C MASTER  SSPxxADD = 27;... #0");
    DelayMs(400); 
    InitializeI2Cmaster(); 
#endif
       
    FConvert.floVal = 1234.56789;
    
    FBytes[0] = FConvert.b[0];
    FBytes[1] = FConvert.b[1];
    FBytes[2] = FConvert.b[2];
    FBytes[3] = FConvert.b[3];

    BConvert.b[0] = FBytes[0];
    FBytes[1] = FConvert.b[1];
    FBytes[2] = FConvert.b[2];
    FBytes[3] = FConvert.b[3];
    
    while(1);
    
    while(1)
    {        
#ifdef SLAVE  
        if (I2CDataBytesReceived)
        {
            printf("\r%s", I2C_WRITEdata);
            sprintf(I2C_READdata, "#%d: Don't get persnickety with me Captain America", trial++);
            I2CDataBytesReceived = 0;
        }
        if (PUSHBUTTON == 0)
        {
            printf("\rRESET SLAVE");
            while (PUSHBUTTON == 0);
            Initialize_I2C_Slave();
            printf("\rSLAVE I2C INIT");
        }
        if (TEST_OUT) TEST_OUT = 0;
        else TEST_OUT = 1;
        DelayMs(10);
#endif
#ifdef MASTER
        length = sprintf(OutBytes, "#%d: Yes yes yes said Chicken Little", trial++);
        TEST_OUT = 1;
        errorFlag = i2c_SlaveWrite(SLAVE_ADDRESS, length, OutBytes);
        TEST_OUT = 0;
        if (errorFlag) printf("\r#%d: SEND, ERRORS: %d", trial, errorFlag);
        //DelayMs(100);
        errorFlag = i2c_SlaveRead(SLAVE_ADDRESS, 52, InBytes);
        if (errorFlag) printf("\rRECV: %s, ERRORS: %d", InBytes, errorFlag);
        else printf("\rRECV: %s", InBytes);
#endif        
    }
}

/*
        while (SSP2IF) 
        {
            SSP2IF = 0;
            I2CDataBytesReceived = I2C_Slave_Handler(I2C_WRITEdata);
            if (I2CDataBytesReceived)
            {
                I2C_WRITEdata[I2CDataBytesReceived] = '\0';
                printf("\r%s", I2C_WRITEdata);
                for (i = 0; i < MAXBUFFER; i++) I2C_WRITEdata[i] = 0x00;
                I2CDataBytesReceived = 0;
            }
        }
 */


short ReadPIC_EEpromRegister(unsigned short address)
{
    short RegisterValue;
    ConvertIntType convert;        
    convert.b[0] = PIC_EEprom_read (address+0);
    convert.b[1] = PIC_EEprom_read (address+1);
    RegisterValue = (short)convert.integer;
    return(RegisterValue);
}


unsigned char PIC_EEprom_read(unsigned char  address)
{
    EEADR=address; // load address of EEPROM to read
    EECON1bits.EEPGD=0; // access EEPROM data memory
    EECON1bits.CFGS=0; // do not access configuration registers
    EECON1bits.RD=1; // initiate read
    return EEDATA; // return EEPROM byte
}

// Write Byte to internal EEPROM
unsigned char  PIC_EEprom_write(unsigned char  address, unsigned char  data)
{
    EECON1bits.WREN=1; // allow EEPROM writes
    EEADR=address; // load address of write to EEPROM
    EEDATA=data; // load data to write to EEPROM
    EECON1bits.EEPGD=0;// access EEPROM data memory
    EECON1bits.CFGS=0; // do not access configuration registers
    INTCONbits.GIE=0; // disable interrupts for critical EEPROM write sequence
    //===============//
    EECON2=0x55;
    EECON2=0xAA;
    EECON1bits.WR=1;
    //==============//
    INTCONbits.GIE=1; // enable interrupts, critical sequence complete
    while (EECON1bits.WR==1); // wait for write to complete
    EECON1bits.WREN=0; // do not allow EEPROM writes

    //Verify write operation
    if (PIC_EEprom_read(address)==data) // read the byte we just wrote to EEPROM
    return true; // write was successful
    else
    return false; // write failed
}



unsigned char CTMUsetup(int Mode)
{  
    
    // Initialize CTMU registers:
    // CTMU continues to run when emulator is stopped,CTMU continues
    // to run in idle mode,Time Generation mode disabled, Edges are blocked
    // No edge sequence order, Analog current source not grounded, trigger
    // output disabled, Edge2 polarity = positive level, Edge2 source =
    // source 0, Edge1 polarity = positive level, Edge1 source = source 0,
     
    CTMUCONH = 0x00;            
    CTMUCONHbits.CTMUEN = 0;    // Module disabled
    CTMUCONHbits.CTMUSIDL = 0;  // Continue operation in Idle
    CTMUCONHbits.TGEN = 0;      // Use measurement operating mode, not time delay
    CTMUCONHbits.EDGEN = 0;     // Edges are blocked
    CTMUCONHbits.EDGSEQEN = 0;  // No edge sequence
    CTMUCONHbits.IDISSEN = 0;   // Analog current source not grounded
    CTMUCONHbits.CTTRIG = 0;    // CTMU special event trigger disabled
         
    CTMUCONL = 0x00;
    CTMUCONLbits.EDG2POL = 1;   // Edge 2 positive
    CTMUCONLbits.EDG2SEL1 = 0;  // Edge 2 use source 0
    CTMUCONLbits.EDG2SEL0 = 0;
    CTMUCONLbits.EDG1POL = 1;   // Edge 1 positive
    CTMUCONLbits.EDG1SEL1 = 0;  // Edge 1 use source 0
    CTMUCONLbits.EDG1SEL0 = 0;
    CTMUCONLbits.EDG2STAT = 0;  // No edge events
    CTMUCONLbits.EDG1STAT = 0;  // No edge events
   
 
    //CTMUICON - CTMU Current Control Register
    CTMUICON = 0x00;            // No current trim adjustment
//    CTMUICONbits.IRNG0 = 0;     // Current = 5.5 uA
//    CTMUICONbits.IRNG1 = 1;
    
    CTMUICONbits.IRNG0 = 1;     // Current = 0.55 uA
    CTMUICONbits.IRNG1 = 0;   
 
        
    // For current cal, ground resistor and tristate sensor cap
    if (Mode == CURRENT_CAL_MODE)
    {
        // Make RB4 an analog input
        TRISBbits.TRISB4=1;            
        ANSELBbits.ANSB4=1;

        // Make sure RB3 and RB5 are digital
        ANSELBbits.ANSB3=0;
        ANSELBbits.ANSB5=0;        
        
        ADCON0bits.CHS=11;       // Select ADC channel 11
        
        TRISTATE_CAP();
        SHORT_RESISTOR_TO_GROUND();
    }
    // For sensor measurement, ground cap and tristate resistor   
    else if (Mode == SENSOR_MEASURE_MODE)
    {
        // Make RB4 an analog input
        TRISBbits.TRISB4=1;            
        ANSELBbits.ANSB4=1;

        // Make sure RB3 and RB5 are digital
        ANSELBbits.ANSB3=0;
        ANSELBbits.ANSB5=0;       
        
        ADCON0bits.CHS=11;       // Select ADC channel 11       
        
        TRISTATE_RESISTOR();
        SHORT_CAP_TO_GROUND();
    }
    // For reverse sensor measurement, ground other lead on cap and tristate resistor   
    else
    {
        // Make RB5 an analog input
        TRISBbits.TRISB5=1;            
        ANSELBbits.ANSB5=1;

        // Make sure RB3 and RB4 are digital
        ANSELBbits.ANSB3=0;
        ANSELBbits.ANSB4=0;
        
        ADCON0bits.CHS=13;       // Select ADC channel 13    
        
        TRISTATE_RESISTOR();
        REVERSE_SHORT_CAP_TO_GROUND();        
    }    
 
    // ADCON2
    ADCON2bits.ADFM=1;      // Results format 1= Right justified
    ADCON2bits.ACQT=1;      // Acquition time 7 = 20TAD 2 = 4TAD 1=2TAD
    ADCON2bits.ADCS=2;      // Clock conversion bits 6= FOSC/64 2=FOSC/32
    // ADCON1
    ADCON1bits.PVCFG0 =0;   // Vref+ = AVdd
    ADCON1bits.NVCFG1 =0;   // Vref- = AVss
    //ADCON1bits.PVCFG0 =0;   // Internal Vref+
    //ADCON1bits.PVCFG1 =0;   
    //ADCON1bits.NVCFG0 =1;   // External Vref- 
    //ADCON1bits.NVCFG1 =0;    
    
    // ADCON0

    ADCON0bits.ADON=1;      // Turn on ADC  
    return true;
}




float CurrentCalibration(void)
{
    int i;
    int j = 0; //index for loop
    unsigned int CalADCounts = 0;
    double CalADSum = 0;
    float CalADaverage=0, CalVoltage=0, CalCurrent=0;
    
    CTMUsetup(CURRENT_CAL_MODE);
    CTMUCONHbits.CTMUEN = 1; //Enable the CTMU
    CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
    CTMUCONLbits.EDG2STAT = 0; 
    
    for(j=0;j<10;j++)
    {
        CTMUCONHbits.IDISSEN = 1;   // Drain charge on resistor
        DelayUs(CHARGE_TIME_US);    // Allow time to discharge
        CTMUCONHbits.IDISSEN = 0; 
 
        CTMUCONLbits.EDG1STAT = 1;  // Turn on current source
        DelayUs(CHARGE_TIME_US);    // Allow time to charge
 
        PIR1bits.ADIF = 0;          // Clear AD flag
        ADCON0bits.GO = 1;          // Start A/D conversion
        while (!PIR1bits.ADIF);      // Wait until done
 
        CalADCounts = ADRES;            // Get A/D result
        CTMUCONLbits.EDG1STAT = 0;      // Turn off current source
        CalADSum += CalADCounts;        // Add the reading to sum
    }
 
    CalADaverage = (float)(CalADSum / 10.000); // Average of 10 readings
    CalVoltage = (float)((CalADaverage / ADSCALE) * ADREF) + ADOFFSET;
    CalCurrent = CalVoltage / RCAL;             
    printf("\rCal AD average: %0.0f counts, Cal Voltage: %0.3f volts, Cal current: %0.3f uA", CalADaverage, CalVoltage, CalCurrent);    
    return CalCurrent;
}


float CapMeasurement(float CalCurrent)
{
    int i;
    int j = 0; //index for loop
    unsigned int CapADCounts = 0;
    double CapADSum = 0;
    float CapADaverage=0, CapVoltage=0, CapMeasured=0;


    CTMUsetup(SENSOR_MEASURE_MODE);
    CTMUCONHbits.CTMUEN = 1; //Enable the CTMU
    CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
    CTMUCONLbits.EDG2STAT = 0; 
    
    for(j=0;j<10;j++)
    {
        CTMUCONHbits.IDISSEN = 1;   // Drain charge on cap
        DelayUs(CHARGE_TIME_US);    // Allow time to discharge
        CTMUCONHbits.IDISSEN = 0; 
 
        CTMUCONLbits.EDG1STAT = 1;  // Turn on current source
        DelayUs(CHARGE_TIME_US);    // Allow time to charge
        CTMUCONLbits.EDG1STAT = 0;   // Turn off current source        
 
        PIR1bits.ADIF = 0;          // Clear AD flag
        ADCON0bits.GO = 1;          // Start A/D conversion
        while (!PIR1bits.ADIF);      // Wait until done
 
        CapADCounts = ADRES;            // Get A/D result
        CapADSum += CapADCounts;        // Add the reading to sum
    }
 
    CapADaverage = (float)(CapADSum / 10.000); // Average of 10 readings
    CapVoltage = (float)((CapADaverage / ADSCALE) * ADREF) + ADOFFSET;
    
    if (CapVoltage == 0) return 0;
    
    // Capacitance = (Cal current * Charge Time) / CapVoltage
    CapMeasured = (CalCurrent * CHARGE_TIME_US) / CapVoltage;
    printf("\rCap AD average: %0.0f counts, Cap Voltage: %0.1f volts, Capacitance: %0.1f pF", CapADaverage, CapVoltage, CapMeasured);    
    return CapMeasured;
}

float ReverseCapMeasurement(float CalCurrent)
{
    int i;
    int j = 0; //index for loop
    unsigned int CapADCounts = 0;
    double CapADSum = 0;
    float CapADaverage=0, CapVoltage=0, CapMeasured=0;


    CTMUsetup(REVERSE_SENSOR_MODE);
    CTMUCONHbits.CTMUEN = 1; //Enable the CTMU
    CTMUCONLbits.EDG1STAT = 0; // Set Edge status bits to zero
    CTMUCONLbits.EDG2STAT = 0; 
    
    for(j=0;j<10;j++)
    {
        CTMUCONHbits.IDISSEN = 1;   // Drain charge on cap
        DelayUs(CHARGE_TIME_US);    // Allow time to discharge
        CTMUCONHbits.IDISSEN = 0; 
 
        CTMUCONLbits.EDG1STAT = 1;  // Turn on current source
        DelayUs(CHARGE_TIME_US);    // Allow time to charge
        CTMUCONLbits.EDG1STAT = 0;   // Turn off current source        
 
        PIR1bits.ADIF = 0;          // Clear AD flag
        ADCON0bits.GO = 1;          // Start A/D conversion
        while (!PIR1bits.ADIF);      // Wait until done
 
        CapADCounts = ADRES;            // Get A/D result
        CapADSum += CapADCounts;        // Add the reading to sum
    }
 
    CapADaverage = (float)(CapADSum / 10.000); // Average of 10 readings
    CapVoltage = (float)((CapADaverage / ADSCALE) * ADREF) + ADOFFSET;
    
    if (CapVoltage == 0) return 0;
    
    // Capacitance = (Cal current * Charge Time) / CapVoltage
    CapMeasured = (CalCurrent * CHARGE_TIME_US) / CapVoltage;
    // printf("\rREVERSE AD average: %0.0f counts, Cap Voltage: %0.3f volts, Capacitance: %0.1f pF", CapADaverage, CapVoltage, CapMeasured);    
    return CapMeasured;
}

float MeasureThermistorTemperature(void)
{  
short i = 0, ThermistorCounts = 0; 
float floCounts, ThermistorResistance, TemperatureCelsius, ResHigh, ResLow, ArrRes, DegreesC, Multiplier;

    // Make RB0 an analog input
    TRISBbits.TRISB0=1;            
    ANSELBbits.ANSB0=1;
        
    ADCON0bits.CHS=12;       // Select ADC channel 12 for thermistor        
 
    // ADCON2
    ADCON2bits.ADFM=1;      // Results format 1= Right justified
    ADCON2bits.ACQT=1;      // Acquition time 7 = 20TAD 2 = 4TAD 1=2TAD
    ADCON2bits.ADCS=2;      // Clock conversion bits 6= FOSC/64 2=FOSC/32
    // ADCON1
    ADCON1bits.PVCFG0 =0;   // Vref+ = AVdd
    ADCON1bits.NVCFG1 =0;   // Vref- = AVss
    //ADCON1bits.PVCFG0 =0;   // Internal Vref+
    //ADCON1bits.PVCFG1 =0;   
    //ADCON1bits.NVCFG0 =1;   // External Vref- 
    //ADCON1bits.NVCFG1 =0;    
    
    // ADCON0

    ADCON0bits.ADON=1;      // Turn on ADC  
    
    
    PIR1bits.ADIF = 0;          // Clear AD flag
    ADCON0bits.GO = 1;          // Start A/D conversion
    while (!PIR1bits.ADIF);      // Wait until done
 
    ThermistorCounts = (unsigned short) ADRES;            // Get A/D result
    floCounts = (float) ThermistorCounts;
    ThermistorResistance  = 10000 / ((1023.0 / floCounts) - 1);
    
    for (i = 1; i < THERMISTOR_TABLE_SIZE; i++)
    {
        ResHigh = arrThermistorResistance[i-1]; 
        ResLow = arrThermistorResistance[i]; 
        if (ThermistorResistance > ResLow) break;
    }
    if (ThermistorResistance == ResLow) 
        Multiplier = 0;
    else Multiplier = (ResHigh - ThermistorResistance) / (ResHigh - ResLow);
    DegreesC = (Multiplier + (float)(i-1)) * 5.0;
    return (float) DegreesC;
}


/****************************************************************************************************************/

// Returns the RH% as an integer multiplied times ten.
// Assumes the RH percent values in the table are stored as integers multiplied times 100.
// So the final value extrapolated below from this lookup table is divided by 10 to yield the RH times ten.
unsigned short CalculateRH_X10(short numSetpoints, short ADcounts, TableType *ptrTable)
{    
    short i;
    long AD1, AD2;
    long RHcalculated = 0;
    unsigned short RHint;
    long RH1, RH2, slopeNumerator, slopeDenominator;
      
    if (numSetpoints < 2) return (0);
    for (i = 0; i < numSetpoints; i++)
    {
        if (ADcounts <= ptrTable[i].ADcounts)
            break;
    }
    if (i >= numSetpoints) i = numSetpoints - 1;
    if (ADcounts == ptrTable[i].ADcounts)
    {
        RHcalculated = ptrTable[i].RHpercent;  // $$$$
        RHcalculated = RHcalculated / 10;
        return(RHcalculated);    
    }
    if (i==0) i = 1;
    RH1 = (long) ptrTable[i-1].RHpercent;
    RH2 = (long) ptrTable[i].RHpercent;
    AD1 = (long) ptrTable[i-1].ADcounts;
    AD2 = (long) ptrTable[i].ADcounts;
    if (AD2 == AD1) return 0;
    
    slopeNumerator = RH2 - RH1;
    slopeDenominator = AD2 - AD1;
    
    RHcalculated = (ADcounts - AD1) * slopeNumerator;
    RHcalculated = RHcalculated / slopeDenominator;
    RHcalculated = RHcalculated + RH1;
    RHcalculated = RHcalculated / 10;
    if (RHcalculated < 0) RHcalculated = 0;
    RHint = (unsigned short) RHcalculated;
    return RHint;
}



void loadCalibrationTable()
{
unsigned short i, address;
ConvertIntType convert;
    
    if (numberOfSetpoints < MAX_RH_SETPOINTS)
    {
        
        for (i = 0; i < numberOfSetpoints; i++)
        {   
            address = (i*4) + CAL_START_EEPROM_ADDRESS;
            convert.b[0] = PIC_EEprom_read (address+0);
            convert.b[1] = PIC_EEprom_read (address+1);
            arrRHCalTable[i].ADcounts = convert.integer;
        
            convert.b[0] = PIC_EEprom_read (address+2);
            convert.b[1] = PIC_EEprom_read (address+3);        
            arrRHCalTable[i].RHpercent = convert.integer;
            
            // printf("\rAD: %d => RHx10: %d", (int)arrRHCalTable[i].ADcounts, (int)arrRHCalTable[i].RHpercent);
        }
    }
}

