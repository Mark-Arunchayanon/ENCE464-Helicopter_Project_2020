#ifndef YAW_H_
#define YAW_H_


//*****************************************************************************
//
// Yaw - Calculating yaw slot numbers and angles functions through an Interrupt
//
// Author:  N. James
//          L. Trenberth
//          M. Arunchayanon
// Last modified:   23.4.2019
//*****************************************************************************


// *******************************************************
// getYaw:          Uses the current slot number on the disk to
//                  return an angle in degrees from the original reference point.
// RETURNS:         Angle value between -180 < Yaw < 180 degrees.
int32_t
getYaw(void);


// *******************************************************
// resetYaw:        Resets the slot number to 0
void
resetYaw (void);


// *******************************************************
//  YawIntHandler:  Interrupt handler for the yaw interrupt.
//                  Measures Phasse A and Phase B.
//                  If moving clockwise, add 1 to slot
//                  If moving anti-clockwise, minus 1 to slot
void
YawIntHandler (void);

// *******************************************************
//  YawIntHandler: Interrupt initialisation for the yaw interrupt.
//                 Sets PB0 and PB1 to be inputs, enables interrupts on GPIOB.
//                 An interrupt occurs on both edges of PB0 and PB1 and when triggered,
//                 runs the YawIntHandler function
void
initYaw (void);


#endif /* YAW_H_*/
