//  *****************************************************************************
//
// System - System Related Files and includes
//
// Author:  N. James
//          L. Trenberth
//          M. Arunchayanon
// Last modified:   20.4.2019
//  *****************************************************************************


//  *****************************************************************************
//  System Constants and Includes
#define SAMPLE_RATE_HZ   40
#define SYSTICK_RATE_HZ  100

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "buttons4.h"
#include "circBufT.h"
#include "control.h"
#include "uart.h"
#include "display.h"
#include "yaw.h"

static uint32_t g_ulSampCnt;    // Counter for the interrupts
volatile uint8_t slowTick = false;
const uint8_t ticksPerSlow = 50; // to get 500ms
static uint8_t tickCount = 0;

//*****************************************************************************
// SysTickIntHandler:   Counts system ticks
// Taken from:          Week4Lab ADCDemo1
void SysTickIntHandler(void) {
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;
    updateButtons();

    updateButtons ();       // Poll the buttons
    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = true;
    }
        updateReset();
}

//*****************************************************************************
//  initSysTick:        Initialises system ticks for the system clock
//                      and sets SysTickIntHandler to handle interrupts from the clock
//  Taken from:         Week4Lab ADCDemo1
void
initSysTick (void)
{
    //
    // Set up the period for the SysTick timer.  The SysTick
    // timer period is set as a function of the system clock.
    SysTickPeriodSet (SysCtlClockGet() / SYSTICK_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister (SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable ();
    SysTickEnable ();
}

//*****************************************************************************
// initClock:           Initialisation functions for the clock (incl. SysTick), ADC, display
// Taken from:          Week4Lab ADCDemo1
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


