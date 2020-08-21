#ifndef MOTOR_H_
#define MOTOR_H_
/*************************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * motor:           Sets up the primary and secondary PWM signals for use
 *                  and creates functions for controls.
 *
 * Original Authors:       N. James
 *                         L. Trenberth
 *                         M. Arunchayanon
 * Updated to FreeRTOS by: G. Thiele
 *                         M. Arunchayanon
 *                         S. Goonatillake
 * Last modified:  21.08.2020
 *
 ************************************************************************************************/


/***********************************************************************************************
 * Global Variables
 **********************************************************************************************/
static int32_t mainPWM = 0;
static int32_t tailPWM = 0;


/***********************************************************************************************
 * setMainPWM:          Function to set the freq, duty cycle of M0PWM7
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void SetMainPWM(int32_t ui32Duty);


/***********************************************************************************************
 * initialiseMainPWM:   Function to set the freq, duty cycle of M0PWM7
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void initialiseMainPWM(void);


/***********************************************************************************************
 * setTailPWM:   Function to set the freq, duty cycle of M0PWM7
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void SetTailPWM(int32_t ui32Duty);


/***********************************************************************************************
 * initialiseTailPWM:   M1PWM5 (J3-10, PF1) is used for the secondary rotor motor
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void initialiseTailPWM(void);


/***********************************************************************************************
 * resetMotor:   Resets the Peripherals for the GPIO Pins and PWM pins
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void resetmotor(void);


/***********************************************************************************************
 * getMainPWM:   Getter function for the main rotor PWM value
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
uint32_t getMainPWM(void);


/***********************************************************************************************
 * getTailPWM:   Getter function for the tail rotor PWM value
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
uint32_t getTailPWM(void);


/***********************************************************************************************
 * initMotor:   Initializes the main and secondary PWM modules
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void initmotor(void);


#endif /* MOTOR_H_ */
