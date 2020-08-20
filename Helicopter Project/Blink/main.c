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
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"

#include "utils/uartstdio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "altitude.h"
#include "uart.h"
#include "yaw.h"
#include "buttons4.h"
#include "motor.h"
#include "control.h"
#include "display.h"

#define BUF_SIZE            10
#define TASK_STACK_DEPTH    128
#define TASK_PRIORITY       4

static TaskHandle_t xPIDTask = NULL;

int main(void)
{
    // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
     SYSCTL_XTAL_16MHZ);


    initButtonCheck();
    initADC();
    initYaw();
    initmotor();
    initDisplay();
    initialiseUSB_UART();
    resetAltitude();
    initButtons();
    initSwitch_PC4();
    IntMasterEnable();

    if (pdTRUE != xTaskCreate(vButtonTask, "Buttons", TASK_STACK_DEPTH, NULL, 3,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vControlTask, "Control", TASK_STACK_DEPTH, NULL, 4,
                              &xPIDTask))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    // Create ADC Task
    if (pdTRUE != xTaskCreate(vADCSampleTask, "ADC Sampler", TASK_STACK_DEPTH, NULL, 2,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vADCTask, "ADC Calc", TASK_STACK_DEPTH, xPIDTask, 2,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vDisplayTask, "Display", 768, NULL, 3,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }


    vTaskStartScheduler();      // Start FreeRTOS!!

    while(1);                   // Should never get here since the RTOS should never "exit".
}
