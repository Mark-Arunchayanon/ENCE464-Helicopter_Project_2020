#ifndef ALTITUDE_H_
#define ALTITUDE_H_
//*****************************************************************************
//
// Altitude - File containing ADC and altitude calculations
//
// Author:  N. James
//          L. Trenberth
//          M. Arunchayanon
// Last modified:   20.4.2019
//*****************************************************************************

#include "circBufT.h"
#define BUF_SIZE 10

//  *****************************************************************************
//  ADCIntHandler: The handler for the ADC conversion complete interrupt.
//                 Writes to the circular buffer.
//  Taken from:    Week4Lab ADCDemo1.c
void
ADCIntHandler(void);


//  *****************************************************************************
//  initADC:    Configures and enables the ADC
//  Taken from: Week4Lab ADCDemo1.c
void
initADC (void);


//  *****************************************************************************
//  computeAltitude: Calculates the average altitude from the ADC.
//  Taken from:      Week4Lab ADCDemo1.c main function
//  RETURNS:         The calculated ADC altitude value as a int32_t
int32_t
computeAltitude(void);


//  *****************************************************************************
//  resetAltitude: Resets the refAltitude to be current ADC altitude.
void
resetAltitude(void);


//  *****************************************************************************
//  percentAltitude: Converts the ADC Altitude into a usable percentage altitude
//                   using a 0.8V difference as the maximum height
//  RETURNS: A Height Percentage as a int32_t from the reference height.
int32_t
percentAltitude(void);


//  *****************************************************************************
//  bufferLocation: Returns the location of the circular buffer
//  RETURNS:        A pointer to a circbuf_t
circBuf_t*
bufferLocation(void);

#endif /*ALTITUDE_H_*/
