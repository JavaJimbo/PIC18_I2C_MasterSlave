/*
 *	Delay functions for 16F877 running at 16 Mhz
 *
 *	Functions available:
 *		DelayUs(x)	Delay specified number of microseconds
 *		DelayMs(x)	Delay specified number of milliseconds
 *
 *
 */



// extern void DelayUs(unsigned char);	// Accepts any value from 1 to 255 microseconds
extern void DelayMs(unsigned short);	// Accepts any value from 1 to 65536 milliseconds
extern void DelayUs (unsigned short i); // Accepts any value from 1 to 65536 microseconds
