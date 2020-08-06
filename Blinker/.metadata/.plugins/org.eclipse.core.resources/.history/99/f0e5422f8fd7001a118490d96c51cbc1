//*****************************************************************************
//
// control - Includes PID control for the Altitude and Yaw, and 5 helicopter modes.
//           Landed, Initialising, TakeOff, Flying and Landing
//
// Author:  N. James
//          L. Trenberth
//          M. Arunchayanon
// Last modified:   31.5.2019
//
//*****************************************************************************

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>


// *******************************************************
// initSwitch_PC4:      Initialises and sets up switch on PC4
void
initSwitch_PC4(void);


// *******************************************************
// updateReset:         Reads the reset button, reset system if reset is pushed
void
updateReset(void);


// *******************************************************
// GetSwitchState:      Reads the switch, if the program starts with the switch on,
//                      the helicopter will be paralysed (not be able to take of)
void
GetSwitchState(void);


// *******************************************************
// checkStability:      Checks if the helicopter has taken off, sets stable to true
void
checkStability(void);

// *******************************************************
// setAltRef:           Sets the altitude reference
// TAKES:               New altitude reference as a percentage
void
setAltRef(int32_t newAltRef);


// *******************************************************
// setYawRef:           Sets the yaw reference
// TAKES:               newYawRef, the new yaw reference as a percentage
void
setYawRef(int32_t newYawRef);


// *******************************************************
// GetAltRef:           Returns the current reference for the altitude
// RETURNS:             Altitude Reference as a int32_t
int32_t
GetAltRef(void);


// *******************************************************
// GetYawRef:           Returns the current reference for the yaw
// RETURNS:             Yaw Reference as a int32_t
int32_t
GetYawRef(void);

// *******************************************************
// take_Off:            Checks if yaw is zero.
//                      If this is true, sets Altitude Reference to 50%
void
take_Off(void);

// *******************************************************
// findYawRef:          Turns on main and tail motor. Spins the helicopter clockwise
//                      and  reads PC4 to check if the helicopter is at the reference
//                      Once the reference is found, resets yaw reference to 0 and current yaw to 0
void
findYawRef(void);


// *******************************************************
// landing:             Once yaw is within 5 degrees of 0,
//                      decrease altitude by 5% if over 10%
//                      If altitude is under 10%, shut off motors
void
landing(void);

// *******************************************************
//  PIDControlYaw:      Uses PID control during TakeOff, Flying and Landing modes
//                      Ensures the yaw follows the yaw reference
void
PIDControlYaw(void);

// *******************************************************
// PIDControlAlt:       Uses PID control during TakeOff, Flying and Landing modes
//                      Ensures the altitude follows the altitude reference
void
PIDControlAlt(void);

// *******************************************************
// getMainDuty:         Returns main rotor duty cycle
// RETURNS:             The main duty cycle as a uint32_t
uint32_t
getMainDuty(void);


// *******************************************************
// getTailDuty:         Returns tail duty cycle
// RETURNS:             The tail rotor duty cycle as a uint32_t
uint32_t
getTailDuty(void);


// *******************************************************
// getMode:             Finds the current mode of the helicopter
// RETURNS:             A char* containing the current mode
char*
getMode(void);


// *******************************************************
// resetIntControl:     Reset all error and integral error to 0
void
resetIntControl(void);


// *******************************************************
// RefUpdate:           Only runs when the helicopter is in flying mode
//                      Checks button status and changes reference altitudes and yaws
//                      UP and DOWN are used to increase/decrease altitude reference
//                      LEFT and RIGHT are used to increase/decrease yaw reference
void
RefUpdate(void);

// *******************************************************
// helicopterStates:    Switches mode between the 5 modes
void
helicopterStates(void);


#endif /* CONTROL_H_ */
