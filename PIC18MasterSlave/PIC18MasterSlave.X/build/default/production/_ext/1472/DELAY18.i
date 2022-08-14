
# 1 "../DELAY18.c"

# 14 "../DELAY.h"
extern void DelayMs(unsigned short);
extern void DelayUs (unsigned short i);

# 10 "../DELAY18.c"
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

