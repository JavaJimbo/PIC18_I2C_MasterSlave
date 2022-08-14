/*
 *	Delay functions for 18F25K22 running at 16 Mhz
 */
#include "DELAY.h"

// Millisecond delay routine for 16 Mhz PICs
// Accepts any unsigned integer value
// Will create a delay in milliseconds
// up to 65535 milliseconds or 65 seconds
void DelayMs (unsigned short i) 
{
unsigned short j, dummy;
    
	for (j=0; j<i; j++)
    {
        dummy = 500;
		while (dummy) dummy--;				
	}
}

void DelayUs (unsigned short i) 
{
unsigned short j;

    j = i;
    while(j--);
}


 
