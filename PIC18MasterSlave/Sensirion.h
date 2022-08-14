/* 
 * File:   Sensirion.h
 * Author: JimSe
 *
 * Created on July 28, 2022, 6:37 AM
 */

#ifndef SENSIRION_H
#define	SENSIRION_H

#ifdef	__cplusplus
extern "C" {
#endif

unsigned char GetSingleShotSensirionMeasurement(float *TemperatureF, float *RH);


#ifdef	__cplusplus
}
#endif

#endif	/* SENSIRION_H */

