/*      SensirioinSHT85 for MC Pros PIC-P40-28 Prototype board using PIC18F44K22 
 *      Compiler: XC8 V2.36 MPLABX IDE V6.00
 *
 *      7-27-22:    Created new project. Board voltage set to 3.3V
 *      Got I2C writes and reads working for 24LC256 using I2C2
 *      7-29-22: Imported GitHub routines for reading Sensirion SHT85 humidity & temp sensor
 *      7-31-22: Added routines for the TCA9548A multiplexer - works great.
 *      8-8-22: 
 *      8-12-22:
 */
#include "Project.h"
#include "Config.h"



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
#define SHT85_CHANNEL 3
#define EEPROM_CHANNEL 0

void main()
{               
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
    unsigned char MPXChannel = 0;
    
    
    // short retries = 0;
    unsigned short trial = 0;
    
    InitializeHardware();      
    
    printf("\r\rSTARTING SENSIRION SHT85... #1");;
    InitializeI2Cmaster(); 
    
    DelayMs(200);  
    
    MPXChannel = 0x00;
    while (MPXChannel < 3)
    {
        temperature = humidity = 0;
        SetMultiplexerChannel (MultiplexerID, MPXChannel);
        printf("\r\rTesting sensor #%d", MPXChannel);
        printf("\rSoft reset...");
        error = SHT85_SoftReset();
        if (error) printf("error: %d", (int)error);
        else printf("OK!");
    
        printf("\rGeneral Call Reset...");
        error = I2c_GeneralCallReset();    
        if (error) printf(" error: %d", (int)error);    
        else printf("OK!");
  
        error = SHT85_ReadSerialNumber(&serialNumber);
        if (error) printf("Serial number error: %d", (int)error);
        else printf("\rSerial Number: %d", serialNumber);
  
        error = SHT85_SingleMeasurment(&temperature, &humidity, SINGLE_MEAS_HIGH, 50);    
        printf("\r#%d: Single shot Temp: %.1f%cF, Humidity: %.1f%% RH", MPXChannel, temperature, 248, humidity);
        if (error) printf(" error: %d", (int)error);
        MPXChannel++;
    }
        
   
    MPXChannel = 0;
    
    while(1)
    {
        printf("\rReading sensor #%d...", MPXChannel);
        SetMultiplexerChannel (MultiplexerID, MPXChannel);
        error= 0;
        // start periodic measurement, with high repeatability and 1 measurement per second
         //printf("\rSTART periodic measurements on sensor %d...", MPXChannel);
        // error = SHT85_StartPeriodicMeasurment(PERI_MEAS_HIGH_1_HZ); $$$$
        //error = SHT85_StartPeriodicMeasurment(CMD_SHT4X_MEASURE_TRH_HI_HIGH_PREC);
        //if (error) printf("\rStart measurement error: %d", (int) error);
        
        // loop while no error
        while (!error) 
        {
            // Keep reading buffer until a new measurement is ready:
            // error = SHT85_ReadMeasurementBuffer(&temperature, &humidity);
            error = SHT4X_ReadSensor(&temperature, &humidity); 
      
            if (!error) 
            {              
                printf("\r%d> sensor #d: Temp: %.1f %cF, Humidity: %.1f%% RH", trial++, MPXChannel, temperature, 248, humidity);
                break;
            }
            else if (error == ACK_ERROR) 
                error = NO_ERROR; // No new values in buffer -> ignore this error
            else
            {
                printf("\rRead measurement Error: %d", (int) error);
                break;  // Break on all other errors
            }   
            DelayMs(100);
        }     
              
        /*
        SetMultiplexerChannel (MultiplexerID, SHT85_CHANNEL);
        // If error occurs, do soft reset:
        error = SHT85_SoftReset();    
        // If soft reset didn't work, try general call reset:
        if (error) error = I2c_GeneralCallReset();    
        if (error) printf(" error: %d", (int)error);
        */
        MPXChannel++;
        if (MPXChannel > 2) MPXChannel = 0;
        DelayMs(100);  
    }
    
   
    
}


/*
short ReadPIC_EEpromRegister(unsigned short address)
{
    short RegisterValue;
    ConvertType convert;        
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
*/

unsigned short ADconvertAndRead(unsigned char channel)
{
    unsigned short highByte;
    short ADresult;
    
    ADCON0 = (channel << 2) | 0x03; // Set AD channel. Use +5V & GND for reference.
    ADCON0bits.ADON = 1; // Turn on converter    
    ADCON0bits.GO_DONE = 1;
    
    while (ADCON0bits.GO_DONE); // Wait for conversion to complete
    highByte = (unsigned short) ADRESH;
    highByte = (highByte << 8) & 0x0F00;  // Allow 12 bit AD!!!!
    ADresult = (short) (highByte | ADRESL);
    return (ADresult);
}


void StartADconversion(unsigned char channel)
{
    // while (ADCON0bits.GO_DONE); // If a conversion is running, wait for it to complete (shouldn't happen, but just in case)
    ADCON0 = (channel << 2) | 0x03; // Set AD channel. Use +5V & GND for reference.
    ADCON0bits.ADON = 1; // Turn on converter    
    ADCON0bits.GO_DONE = 1;    
}

unsigned short GetADconversion()
{
    unsigned short highByte;
    unsigned short ADresult;
    
    highByte = (unsigned short) ADRESH;
    highByte = (highByte << 8) & 0x0F00;  // Allow 12 bit AD!!!!
    ADresult = (short) (highByte | ADRESL);
    return (ADresult);
}

void loadCalibrationTable()
{
    ;
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


/*
        
                    if (!error) 
            {              
                SetMultiplexerChannel (MultiplexerID, EEPROM_CHANNEL);
                numBytes = sprintf(OutData, "#%d: Temp: %.1f %cF, Humidity: %.1f%% RH", trial++, temperature, 248, humidity);
                EepromWriteBlock(EEPROM, 0x0000, OutData, numBytes);
                DelayMs(10);
                InData[numBytes] = '\0';
                EepromReadBlock(EEPROM, 0x0000, InData, numBytes);
                printf("\rEE Read: %s", InData);
            }
            else if (error == ACK_ERROR) 
                error = NO_ERROR; // No new values in buffer -> ignore this error
            else
            {
                printf("\rRead measurement Error: %d", (int) error);
                break;  // Break on all other errors
            }   
            DelayMs(100);
            */