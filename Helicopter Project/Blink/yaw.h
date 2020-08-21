#ifndef YAW_H_
#define YAW_H_

/***********************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * yaw - Interrupt based calculations for yaw by tracking slot changes. Also contains support
 *       functions such as getters, setters and resets.
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
 * getyaw: Uses the current slot number on the disk to return an angle in degrees from the
 *         original reference point. Returns angle value between -180 < Yaw < 180 degrees.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
int32_t
getYaw(void);


int32_t
getYawTotal(void);

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


//BaseType_t takeYawSem (void);
//
//void giveYawSem (void);
//
//void
//vYawTask (void *pvParameters);


#endif /* YAW_H_*/
