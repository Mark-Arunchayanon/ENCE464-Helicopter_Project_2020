/*
 * motor.h
 *
 *  Created on: 13/05/2019
 *      Author: ltr28
 */

#ifndef MOTOR_H_
#define MOTOR_H_

/**********************************************************
 * Constants
 **********************************************************/
// PWM configuration
#define PWM_RATE_HZ             300
#define PWM_DUTY_MAX            95
#define PWM_DUTY_MIN            5

#define PWM_DIVIDER_CODE        SYSCTL_PWMDIV_4
#define PWM_DIVIDER             1

//Second PWM Config
#define PWM_SEC_START_DUTY      0 //10
#define PWM_MAIN_START_DUTY     0 //50

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE           PWM0_BASE
#define PWM_MAIN_GEN            PWM_GEN_3
#define PWM_MAIN_OUTNUM         PWM_OUT_7
#define PWM_MAIN_OUTBIT         PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM     SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE      GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG    GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN       GPIO_PIN_5


void
SetMainPWM (uint32_t ui32Duty);

/*********************************************************
 * initialiseMainPWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 *********************************************************/

void
initialiseMainPWM (void);


/********************************************************
 * Function to set the freq, duty cycle of M1PWM5
 ********************************************************/
void
SetTailPWM (uint32_t ui32Duty);


/*********************************************************
 * initialiseTailPWM
 * M1PWM5 (J3-10, PF1) is used for the secondary rotor motor
 *********************************************************/
void
initialiseTailPWM (void);

void
resetmotor(void);

void
initmotor(void);

void
changeMainMotor(int change);

void
changeSecMotor(int change);


#endif /* MOTOR_H_ */
