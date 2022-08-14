#ifndef STDDEFS_H
#define STDDEFS_H

#include <stdbool.h> 
#define FOREVER   1

#ifdef MAIN_C
#define GLOBAL 
#else
#define GLOBAL extern
#endif


#define FALSE false
#define TRUE true

#define CLEAR_WATCHDOG {ClrWdt();}

#endif // STDDEFS_H



