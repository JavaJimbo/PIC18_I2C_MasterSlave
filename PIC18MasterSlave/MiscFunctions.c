
#include "Project.h"    


// Global interrupt disable

void Hardware_Intoff(void) {
    do {
        INTCONbits.GIE = 0;
    } while (INTCONbits.GIE != 0);
}

// Global interrupt enable

void Hardware_Inton(void) {
    do {
        INTCONbits.GIE = 1;
    } while (INTCONbits.GIE != 1);
}


// This routine transmits the input byte ch from UART #2. 
// It is used by printf() to send formatted strings.

/*
void putch(unsigned char ch) 
{
    while (PIR3bits.TX2IF == 0); // If TX buffer is busy, wait for previous character to be sent  $$$$
    TXREG2 = ch; // Send new character	
}
*/
void putch(unsigned char ch) 
{
    while (PIR1bits.TX1IF == 0); // If TX buffer is busy, wait for previous character to be sent  $$$$
    TXREG1 = ch; // Send new character	
}




void InitializeHardware(void) 
{
    unsigned char dummyRead = 0x00;    

    
    GIE = 0; // Global interrupts disabled   
    
    // Set all IO pins to defaults 0 = Digital, 1 = Analog
    ANSELA = 0b00000000;
    ANSELB = 0b00010000; // Only RB4 is analog - for thermistor
    ANSELC = 0b00000000;
    ANSELD = 0b00000000;
    ANSELE = 0b00000000;     
    
    TRISA = 0b00111111;
    TRISB = 0b11111111;   
    TRISC = 0b10111011;
    TRISD = 0b11111111; 
    TRISE = 0b11111111; 
    
    // Set up UART #1
    BAUDCON1 = 0x00;
    RCSTA1 = 0x00;
    TXSTA1 = 0x00;            
    BAUDCON1bits.BRG16 = 0; // 8 bit mode
    BAUDCON1bits.DTRXP = 0; // Receive data is active high (not inverted))
    TXSTA1bits.BRGH = 1; // High speed baud rate
    SPBRG1 = 207;        // 19200 baud  51 @ 64 Mhz
    TXSTA1bits.SYNC = 0; // asynchronous
    RCSTA1bits.SPEN = 1; // enable serial port pins
    RCSTA1bits.CREN = 1; // enable reception
    RCSTA1bits.SREN = 0; // no effect
    TXSTA1bits.TX9 = 0;  // 8-bit transmission
    RCSTA1bits.RX9 = 0;  // 8-bit reception
    TXSTA1bits.TXEN = 1; // enable the transmitter
    dummyRead = RCREG1;       

    /*
    // Set up UART #2
    BAUDCON2 = 0x00;
    RCSTA2 = 0x00;
    TXSTA2 = 0x00;                
    BAUDCON2bits.BRG16 = 0; // 8 bit mode
    BAUDCON2bits.DTRXP = 0; // Receive data is active high (not inverted)
    TXSTA2bits.BRGH = 1; // High speed baud rate
    SPBRG2 = 207;        // 19200 baud  51 @ 64 Mhz
    // SPBRG2 = 103; // 9600 baud
    TXSTA2bits.SYNC = 0; // asynchronous
    RCSTA2bits.SPEN = 1; // enable serial port pins
    RCSTA2bits.CREN = 1; // enable reception
    RCSTA2bits.SREN = 0; // no effect
    TXSTA2bits.TX9 = 0;  // 8-bit transmission
    RCSTA2bits.RX9 = 0;  // 8-bit reception
    TXSTA2bits.TXEN = 1; // enable the transmitter
    dummyRead = RCREG2;       
    */
    

    // Set up Timer 2 for 10 kHz interrupts @ 64 Mhz clock
    // 64,000,000 / 4 / 16 / 1 / 99+1 = 10 kHz
    T2CKPS0 = 1;    // Prescaler: 1:16
    T2CKPS1 = 1; 
    T2OUTPS0 = 0;   // Postscaler: 1:1
    T2OUTPS1 = 0;
    T2OUTPS2 = 0;
    T2OUTPS3 = 0;
    PR2 = 99; 
    TMR2IF = 0;
    TMR2ON = 1; // Let her rip

    
    // Disable all peripheral interrupts    
    INTCON = 0x00;
    PIE1 = 0x00;
    PIE2 = 0x00;
    PIE3 = 0x00;
    PIE4 = 0x00;
    PIE5 = 0x00;
     
    INTCONbits.PEIE = 1;   // Enable peripheral interrupts.
    PIE1bits.RC1IE = 1; // Enable UART 1 RX interrupts 
    PIR1bits.RC1IF = 0;
   
    TMR1IE = 0; // Disable Timer 1 interrupts
    TMR2IE = 1; // Enable Timer 2 interrupts
    TMR3IE = 0; // Disable all other Timer interrupts
    TMR4IE = 0;      
    TMR0IE = 0;   
    PIE3bits.SSP2IE = 1; // Enable I2C slave interrupts
    INTCONbits.GIE = 1;     // Disable interrupts

}


