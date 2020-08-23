/*************************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * The main file for the UC ENCE464 2020 HeliRig assignment.
 * Controls a helicopter using a FreeRTOS scheduler on Tiva CPU. The helicopter can achieve
 * stable flight using a PID controller, and is controlled using the buttons on the Tiva board.
 *
 * main:            Initialises and resets peripherals, creates FreeRTOS tasks for
 *                  controlling the helicopter, and runs the program using a task
 *                  scheduler.
 *
 * Authors:            G. Thiele
 *                     M. Arunchayanon
 *                     S. Goonatillake
 * Last modified:  21.08.2020
 *
 ************************************************************************************************/


/*************************************************************************************************
 * Includes
 ************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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


/***********************************************************************************************
 * Constants/Definitions
 **********************************************************************************************/
#define BUF_SIZE                    10
#define TASK_STACK_DEPTH            256
#define DISPLAY_TASK_STACK_DEPTH    1024

// Task priority levels, higher numbers will be executed first.
#define BUTTON_TASK_PRIORITY        4
#define CONTROL_TASK_PRIORITY       4
#define ADCSAMPLE_TASK_PRIORITY     3
#define ADC_TASK_PRIORITY           3
#define DISPLAY_TASK_PRIORITY       2


/***********************************************************************************************
 * Global Variables
 **********************************************************************************************/
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



    // Create buttons Task
    if (pdTRUE != xTaskCreate(vButtonTask, "Buttons", TASK_STACK_DEPTH, NULL, BUTTON_TASK_PRIORITY, NULL))
    {
        while (1); // Error creating task
    }

    // Create control Task
    if (pdTRUE != xTaskCreate(vControlTask, "Control", TASK_STACK_DEPTH, NULL, CONTROL_TASK_PRIORITY, &xPIDTask))
    {
        while (1); // Error creating task
    }

    // Create ADC Sampler Task
    if (pdTRUE != xTaskCreate(vADCSampleTask, "ADC Sampler", TASK_STACK_DEPTH, NULL, ADCSAMPLE_TASK_PRIORITY, NULL))
    {
        while (1); // Error creating task
    }

    // Create ADC Task
    if (pdTRUE != xTaskCreate(vADCTask, "ADC Calc", TASK_STACK_DEPTH, xPIDTask, ADC_TASK_PRIORITY, NULL))
    {
        while (1); // Error creating task
    }

    // Create Display Task
    if (pdTRUE != xTaskCreate(vDisplayTask, "Display", 512, NULL, DISPLAY_TASK_PRIORITY, NULL))
    {
        while (1); // Error creating task
    }


    vTaskStartScheduler(); // Start FreeRTOS

    while(1);
}
