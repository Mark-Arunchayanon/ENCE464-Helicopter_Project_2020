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

//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
//xSemaphoreHandle g_pUARTSemaphore;

//void BlinkLED(void *);

int main(void)
 {
    // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
     SYSCTL_XTAL_16MHZ);

//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // activate internal bus clocking for GPIO port F
//    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) ; // busy-wait until GPIOF's bus clock is ready
//
//    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
//
//    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); // off by default
//
//    end++;
//    if (pdTRUE != xTaskCreate(BlinkLED, "Blinker", 32, (void *) 1, 4, NULL))
//    { // (void *)1 is our pvParameters for our task func specifying PF_1
//        while (1); // error creating task, out of memory?
//    }
    initButtonCheck();
    initADC();
    initYaw();
    initButtons();

    initmotor();
    initDisplay();
//    introLine();
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    initialiseUSB_UART();
    resetAltitude();
    initSwitch_PC4();
    IntMasterEnable();

    // Create ADC Task
    if (pdTRUE != xTaskCreate(vADCSampleTask, "ADC Sampler", TASK_STACK_DEPTH, NULL, 2,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vADCTask, "ADC Calc", TASK_STACK_DEPTH, NULL, 2,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vButtonTask, "Buttons", TASK_STACK_DEPTH, NULL, 3,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vControlTask, "Control", TASK_STACK_DEPTH, NULL, 4,
                               NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vDisplayTask, "Display", 512, NULL, 3,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }
    vTaskStartScheduler();      // Start FreeRTOS!!

    while(1);                   // Should never get here since the RTOS should never "exit".
}

//Blinky function
//

//void BlinkLED(void *pvParameters) {
//    unsigned int whichLed = (unsigned int)pvParameters;
//    const uint8_t whichBit = 1 << whichLed; // TivaWare GPIO calls require the pin# as a binary bitmask, not a simple number.
//
//    uint8_t currentValue = 0;
//
//    while (1) {
//     currentValue ^= whichBit; // XOR keeps flipping the bit on / off alternately each time this runs.
//     GPIOPinWrite(GPIO_PORTF_BASE, whichBit, currentValue);
//
//     vTaskDelay(125 / portTICK_RATE_MS);
//    }
//}
