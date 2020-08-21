/*************************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * control:         Includes PID control for the Altitude and Yaw, and 5 helicopter modes.
 *                  Landed, Initialising, TakeOff, Flying and Landing
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


/*************************************************************************************************
 * Includes
 ************************************************************************************************/
#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "timers.h"


/***********************************************************************************************
 * initSwitch_PC4:     Initialises and sets up switch on PC4
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void initSwitch_PC4(void);


/***********************************************************************************************
 * updateReset:        Reads the reset button, reset system if reset is pushed
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void updateReset(void);


/***********************************************************************************************
 * GetSwitchState:     Reads the switch, if the program starts with the switch on,the helicopter
 *                     will be paralysed (not be able to take off)
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void GetSwitchState(void);


/***********************************************************************************************
 * checkStability:       Checks if the helicopter has taken off, sets stable to true
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void checkStability(void);


/***********************************************************************************************
 * clamp:               Clamps a value between a minimum and a maximum.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
int32_t clamp(int32_t x, int32_t min, int32_t max);


/***********************************************************************************************
 * setAltRef:           Sets the altitude reference. Takes the new altitude reference as a
 *                      percentage
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void setAltRef(int32_t newAltRef);


/***********************************************************************************************
 * setYawRef:           Sets the yaw reference. Takes newYawRef, the new yaw reference as a
 *                      percentage
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void setYawRef(int32_t newYawRef);


/***********************************************************************************************
 * getAltRef:           Returns the current reference for the altitude. Returns altitude
 *                      reference as a int32_t
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
int32_t GetAltRef(void);


/***********************************************************************************************
 * getYawRef:           Returns the current reference for the yaw. Returns yaw
 *                      reference as an int32_t
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
int32_t GetYawRef(void);


/***********************************************************************************************
 * takeOff:             Checks if yaw is zero. If this is true, sets Altitude Reference to 50%
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void take_Off(void);


/***********************************************************************************************
 * specialButtonMode:   Controls when the tricks run.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void specialButtonMode(void);


/***********************************************************************************************
 * spiralTrick:        Makes the helicopter spiral upwards or downwards.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void spiralTrick(void);

// *******************************************************
// findYawRef:          Turns on main and tail motor. Spins the helicopter clockwise
//                      and  reads PC4 to check if the helicopter is at the reference
//                      Once the reference is found, resets yaw reference to 0 and current yaw to 0
//void
//findYawRef(void);


void spinTrick(void);
/***********************************************************************************************
 * spinTrick:          Spins the helicopter left or right 180 degrees.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/


/***********************************************************************************************
 * yawRefIntHandler:    Detects the reference position and resets the reference to 0 when it has been detected.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void YawRefIntHandler(void);


/***********************************************************************************************
 * landing:            Once yaw is within 10 degrees of 0, begin to decrease the reference
 *                     altitude. If yaw is outside this range, go to 30% altitude until the yaw
 *                     stabilises. If altitude is under 10%, shut off motors.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void landing(void);


/***********************************************************************************************
 * PIDControlYaw:      Uses PID control during TakeOff, Flying, Special and Landing modes.
 *                     Ensures the yaw follows the yaw reference
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void PIDControlYaw(void);


/***********************************************************************************************
 * PIDControlAlt:       Uses PID control during TakeOff, Flying, Special and Landing modes.
 *                      Ensures the altitude follows the altitude reference
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void PIDControlAlt(void);


/***********************************************************************************************
 * getMainDuty:        Returns main rotor duty cycle. Returns the main duty cycle as a uint32_t
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
uint32_t getMainDuty(void);


/***********************************************************************************************
 * getTailDuty:        Returns tail duty cycle. Returns the tail rotor duty cycle as a uint32_t
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
uint32_t getTailDuty(void);


/***********************************************************************************************
 * getMode:            Finds the current mode of the helicopter. Returns a char* containing the
 *                     current mode.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
char* getMode(void);


/***********************************************************************************************
 * resetIntControl:     Reset all error and integral error to 0
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void resetIntControl(void);


/***********************************************************************************************
 * refUpdate:        Only runs when the helicopter is in flying mode. Checks button status and
 *                   changes reference altitudes and yaws UP and DOWN are used to
 *                   increase/decrease altitude reference, LEFT and RIGHT are used to
 *                   increase/decrease yaw reference
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void RefUpdate(void);


/***********************************************************************************************
 * helicopterStates:  Switches mode between the 6 modes
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void helicopterStates(void);


/***********************************************************************************************
 * vControlTask:      A task scheduled by FreeRTOS to update the controls. Updates the PID
 *                    controllers, gets the switch position and updates the state machine. Has
 *                    a task priority of 4.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void vControlTask (void *pvParameters);


/***********************************************************************************************
 * switchTimerExpire:  Reads the reset button, reset system if reset is pushed
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void switchTimerExpire(TimerHandle_t pxTimer);

#endif /* CONTROL_H_ */
