#ifndef ALTITUDE_H_
#define ALTITUDE_H_
/***********************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * altitude:        Support for a set of FOUR specific buttons on the Tiva/Orbit.
 *                  ENCE361 sample code.
 *                  The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
 *                  LEFT and RIGHT on the Tiva.
 *
 * Note:            pin PF0 (the pin for the RIGHT pushbutton - SW2 on
 *                  the Tiva board) needs special treatment - See PhilsNotesOnTiva.rtf.
 *
 * Original Author:         N. James
 *                          L. Trenberth
 *                          M. Arunchayanon
 * Updated to FreeRTOS by:  G. Thiele
 *                          M. Arunchayanon
 *                          S. Goonatillake
 * Last modified:  21.08.2020
 *
 **********************************************************************************************/


/***********************************************************************************************
 * ADCIntHandler: The handler for the ADC conversion complete interrupt.
 *                Writes to the ADC queue.
 *
 * Authors: M Arunchyanon, S. Goonatillake
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void ADCIntHandler(void);


//  *****************************************************************************
//  initADC:    Configures and enables the ADC
//  Taken from: Week4Lab ADCDemo1.c
void
initADC (void);


//  *****************************************************************************
//  computeAltitude: Calculates the average altitude from the ADC.
//  Taken from:      Week4Lab ADCDemo1.c main function
//  RETURNS:         The calculated ADC altitude value as a int32_t
//int32_t
//computeAltitude(void);
int32_t getAlt (void);


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
//circBuf_t*
//bufferLocation(void);

void vADCSampleTask(void *pvParameters);

void vADCTask(void *pvParameters);

#endif /*ALTITUDE_H_*/
