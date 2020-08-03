/**
 * Simple LED blinking example for the Tiva Launchpad
 */
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#include "FreeRTOS.h"
#include "task.h"

#include "altitude.h"


#define TASK_STACK_DEPTH    50
#define TASK_PRIORITY       4


int main(void)
{

    //
    // Create the ADC task.
    //
    if(initADC() != 0)
    {

        while(1)
        {
        }
    }

    vTaskStartScheduler();      // Start FreeRTOS!!

    while(1);                   // Should never get here since the RTOS should never "exit".
}

