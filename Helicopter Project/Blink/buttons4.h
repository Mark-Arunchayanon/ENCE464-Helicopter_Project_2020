#ifndef BUTTONS_H_
#define BUTTONS_H_

/***************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * buttons4:        Support for a set of FOUR specific buttons on the Tiva/Orbit.
 *                  ENCE361 sample code.
 *                  The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
 *                  LEFT and RIGHT on the Tiva.
 *
 * Note:             pin PF0 (the pin for the RIGHT pushbutton - SW2 on
 *                   the Tiva board) needs special treatment - See PhilsNotesOnTiva.rtf.
 *
 * Original Author:        P.J. Bones UCECE
 * Updated to FreeRTOS by: G. Thiele
 *                         M. Arunchayanon
 *                         S. Goonatillake
 * Last modified:  21.08.2020
 *
 ***************************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>

/*************************************************************************************************
 * Constants
 ************************************************************************************************/

enum butNames {UP = 0, DOWN, LEFT, RIGHT, NUM_BUTS};
enum butStates {RELEASED = 0, PUSHED, NO_CHANGE};
// UP button
#define UP_BUT_PERIPH  SYSCTL_PERIPH_GPIOE
#define UP_BUT_PORT_BASE  GPIO_PORTE_BASE
#define UP_BUT_PIN  GPIO_PIN_0
#define UP_BUT_NORMAL  false
// DOWN button
#define DOWN_BUT_PERIPH  SYSCTL_PERIPH_GPIOD
#define DOWN_BUT_PORT_BASE  GPIO_PORTD_BASE
#define DOWN_BUT_PIN  GPIO_PIN_2
#define DOWN_BUT_NORMAL  false
// LEFT button
#define LEFT_BUT_PERIPH  SYSCTL_PERIPH_GPIOF
#define LEFT_BUT_PORT_BASE  GPIO_PORTF_BASE
#define LEFT_BUT_PIN  GPIO_PIN_4
#define LEFT_BUT_NORMAL  true
// RIGHT button
#define RIGHT_BUT_PERIPH  SYSCTL_PERIPH_GPIOF
#define RIGHT_BUT_PORT_BASE  GPIO_PORTF_BASE
#define RIGHT_BUT_PIN  GPIO_PIN_0
#define RIGHT_BUT_NORMAL  true

#define NUM_BUT_POLLS 3

// *******************************************************
// Debounce algorithm:  A state machine is associated with each button.
//                      A state change occurs only after NUM_BUT_POLLS consecutive polls have
//                      read the pin in the opposite condition, before the state changes and
//                      a flag is set.  Set NUM_BUT_POLLS according to the polling rate.

// *******************************************************
// initButtons:         Initialise the variables associated with the set of buttons
//                      defined by the constants above.
void
initButtons (void);

// *******************************************************
// updateButtons:       Function designed to be called regularly. It polls all
//                      buttons once and updates variables associated with the buttons if
//                      necessary.  It is efficient enough to be part of an ISR, e.g. from
//                      a SysTick interrupt.
void
updateButtons (void);

// *******************************************************
// checkButton:         Function returns the new button state if the button state
//                      (PUSHED or RELEASED) has changed since the last call, otherwise returns
//                      NO_CHANGE.  The argument butName should be one of constants in the
//                      enumeration butStates, excluding 'NUM_BUTS'. Safe under interrupt.
uint8_t
checkButton (uint8_t butName);

// *******************************************************
// vButtonTask:     A task scheduled by FreeRTOS to manage button presses.
//                  Regularly polls the buttons and updates the associated variables.
//                  Has a task priority of 3.
void
vButtonTask (void *pvParameters);

#endif /*BUTTONS_H_*/
