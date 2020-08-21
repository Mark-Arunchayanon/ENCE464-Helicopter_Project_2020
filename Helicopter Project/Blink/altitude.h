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
 * Original Authors:        N. James
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


/***********************************************************************************************
 * initADC: Configures and enables the ADC peripherals and software declarations.
 *
 * Authors: M Arunchyanon, S. Goonatillake
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void initADC (void);


/***********************************************************************************************
 * getAlt: Getter method which returns the altitude converted to percentage format
 *
 * Authors: M Arunchyanon, S. Goonatillake
 * Last Modified: 21.08.2020
 **********************************************************************************************/
int32_t getAlt (void);


/***********************************************************************************************
 * resetAltitude: Resets the altitude reference to the most recently sampled ADC value.
 *
 * Authors: M Arunchyanon, S. Goonatillake
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void resetAltitude(void);


/***********************************************************************************************
 * percentAltitude: Converts the ADC sample into a readable percentage altitude.
 *
 * Authors: M Arunchyanon, Sasiru Goonatillake
 * Last Modified: 21.08.2020
 **********************************************************************************************/
int32_t percentAltitude(void);


/***********************************************************************************************
 * vADCSampleTask: Scheduled task that polls the ADC interrupt at minium 30Hz ??????????????????????????????????????????????????????????????????????????????
 *
 * Authors: M Arunchyanon, S. Goonatillake
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void vADCSampleTask(void *pvParameters);


/***********************************************************************************************
 * vADCSampleTask: Scheduled task reads ADC values from the ADC queue and calculates the
 *                 altitude value using the 0.8V as the maximum height.
 *
 * Authors: M Arunchyanon, S. Goonatillake
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void vADCTask(void *pvParameters);

#endif /*ALTITUDE_H_*/
