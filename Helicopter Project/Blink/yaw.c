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
 * Includes
 **********************************************************************************************/
#include "inc/tm4c123gh6pm.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "yaw.h"

#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "semphr.h"


/***********************************************************************************************
 * Constants/Definitions
 **********************************************************************************************/
#define NUM_SLOTS               448
#define TOTAL_ANGLE             360
#define FIND_REF_MAIN           30 //duty cycle for finding the reference point
#define FIND_REF_TAIL           40


/***********************************************************************************************
 * Global Variables
 **********************************************************************************************/
enum quad {A = 0, B = 1, C = 3, D = 2}; // Sets quadrature encoding states A, B, C, D
enum quad State;
enum quad nextState;

int32_t slot;   //Sets the slot number, the number of slots moved around the disc.
extern int32_t degrees = 0;


// Uses the current slot number on the disk to return an angle in degrees from the original
// reference point. Returns angle value between -180 < Yaw < 180 degrees.
int32_t getYaw(void)
{
    int32_t angle = 0;
    int32_t refnum = slot;

    while (refnum > (NUM_SLOTS / 2))
    {
        refnum -= NUM_SLOTS;
    }
    while (refnum < (- NUM_SLOTS / 2))
    {
        refnum += NUM_SLOTS;
    }

    angle = TOTAL_ANGLE * refnum / NUM_SLOTS;   // Slots converted into an angle and returned as an angle.
    return angle;
}

// Calculates the yaw
int32_t getYawTotal(void)
{
    int32_t angle = 0;
    int32_t refnum = slot;

    angle = (2* (TOTAL_ANGLE * refnum)  + NUM_SLOTS) / 2 / NUM_SLOTS;

    return angle;
}


// Resets the slot number to 0
void resetYaw (void)
{
    slot = 0;
}


// Interrupt handler for the yaw interrupt. Measures Phase A and Phase B. If moving clockwise,
// add 1 to slot If moving anti-clockwise, minus 1 to slot.
void YawIntHandler (void)
{
    // Clear the interrupt bits
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);

    // Sets nextState based off status of Pin 0 & 1
    nextState = (enum quad)GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    switch(State)
    {
    case A:
        //In case A, can move to B or D.
        //D is clockwise, B is anti-clockwise
        switch (nextState)
        {
            case B:
                slot--;
                break;
            case D:
                slot++;
                break;
        }
        break;
    case B:
        //In case B, can move to A or C.
        //A is clockwise, C is anti-clockwise
        switch (nextState)
        {
        case C:
            slot--;
            break;
        case A:
            slot++;
            break;
        }
        break;
    case C:
    {
        //In case C, can move to D or B.
        //B is clockwise, D is anti-clockwise
        switch(nextState)
        {
        case D:
            slot--;
            break;
        case B:
            slot++;
            break;
        }
        break;
    }
    case D:
    {
        //In case D, can move to A or C.
        //C is clockwise, A is anti-clockwise
        switch(nextState)
        {
        case A:
            slot--;
            break;
        case C:
            slot++;
            break;
        }
        break;
    }
    }
    State = nextState;
}

// Initialization for associated with yaw. Interrupt initialisation for the yaw interrupt.
// Sets PB0 and PB1 to be inputs, enables interrupts on GPIOB. An interrupt occurs on both
// edges of PB0 and PB1 and when triggered, runs the YawIntHandler function.
void initYaw (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1); //Sets PIN) & PIN1 as inputs
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1, GPIO_BOTH_EDGES); //Trigger interrupts on both edges of wave changes on PB0 and PB1
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1); //Enable interrupts from PB0 and PB1
    GPIOIntRegister(GPIO_PORTB_BASE, YawIntHandler); //If interrupt occurs, run YawIntHandler
    IntEnable(INT_GPIOB); //Enable interrupts on B.

    resetYaw();
}
