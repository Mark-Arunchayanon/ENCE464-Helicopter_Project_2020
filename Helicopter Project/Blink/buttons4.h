#ifndef BUTTONS_H_
#define BUTTONS_H_
/*************************************************************************************************
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
 ************************************************************************************************/


/***********************************************************************************************
 * Global Variables
 **********************************************************************************************/
static bool but_state[NUM_BUTS];    // Corresponds to the electrical state
static uint8_t but_count[NUM_BUTS];
static bool but_flag[NUM_BUTS];
static bool but_normal[NUM_BUTS];   // Corresponds to the electrical state


/***********************************************************************************************
 * initButtons:         Initialise the variables associated with the set of buttons
 *                      defined by the constants above.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void initButtons (void);


/***********************************************************************************************
 * updateButtons:       Function designed to be called regularly. It polls all
 *                      buttons once and updates variables associated with the buttons if
 *                      necessary.  It is efficient enough to be part of an ISR, e.g. from
 *                      a SysTick interrupt.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void updateButtons (void);


/***********************************************************************************************
 * checkButton:         Function returns the new button state if the button state
 *                      (PUSHED or RELEASED) has changed since the last call, otherwise returns
 *                      NO_CHANGE.  The argument butName should be one of constants in the
 *                      enumeration butStates, excluding 'NUM_BUTS'. Safe under interrupt.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
uint8_t checkButton (uint8_t butName);


/***********************************************************************************************
 * vButtonTask:     A task scheduled by FreeRTOS to manage button presses.
 *                  Regularly polls the buttons and updates the associated variables.
 *                  Has a task priority of 3.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void vButtonTask (void *pvParameters);


#endif /*BUTTONS_H_*/
