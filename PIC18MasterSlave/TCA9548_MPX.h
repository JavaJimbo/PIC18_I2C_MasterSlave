/* 
 * File:   TCA9548_MPX.h
 * Author: Jim Sedgwick
 *
 * Routines for setting and reading back the selected channel on the Adafruit TCA9548A breakout board.
 * Created on July 31, 2022, 2:56 AM
 * 
 */

#ifndef TCA9548_MPX_H
#define	TCA9548_MPX_H

#ifdef	__cplusplus
extern "C" {
#endif

unsigned char SetMultiplexerChannel(unsigned char multiplexerID, unsigned char channel);
unsigned char GetMultiplexerChannel(unsigned char multiplexerID, unsigned char *ptrChannel);

#ifdef	__cplusplus
}
#endif

#endif	/* TCA9548_MPX_H */

