/*************************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * display:         Functions used to output values on Display and UART
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
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "stdlib.h"

#include "display.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
#include "altitude.h"
#include "yaw.h"
#include "control.h"
#include "uart.h"
#include "buttons4.h"
#include "motor.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


/***********************************************************************************************
 * Constants/Definitions
 **********************************************************************************************/
SemaphoreHandle_t xDisplayMutex; // Create mutex for controlling access to yaw and altitude.


// Initialises Display using OrbitLED functions
void initDisplay (void)
{
    OLEDInitialise ();
    xDisplayMutex = xSemaphoreCreateMutex();
}


// Prints the intro line on the OLED Display
void introLine (void)
{
    OLEDStringDraw ("Heli Project", 0, 0);
}


// Prints the input format and line contents on the given line number on OLED Display
void printString(char* restrict line_format, int32_t line_contents, uint8_t line_number)
{
    char string[MAX_STR_LEN + 1];
    usnprintf (string, sizeof(string), line_format, line_contents);
    OLEDStringDraw (string, 0, line_number); // Update line on display.
}


// Initialises left and up buttons on the micro-controller
void initButtonCheck (void) {
    SysCtlPeripheralReset (LEFT_BUT_PERIPH);  //setting up the LEFT button GPIO
    SysCtlPeripheralReset (UP_BUT_PERIPH);    //setting the UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);  //setting the DOWN button GPIO
    SysCtlPeripheralReset (RIGHT_BUT_PERIPH); //setting the RIGHT button GPIO
}


// A task scheduled by FreeRTOS to update the display. Prints a readout of
// position and control details to the display and UART. Has a task priority of 3.
void vDisplayTask (void *pvParameters)
{
    char statusStr[MAX_STR_LEN + 1];
    const TickType_t xDelay1s = pdMS_TO_TICKS(100);
    int32_t yawReading = 0;
    int32_t altReading = 0;
    int32_t PWMmain = 0;
    int32_t PWMtail = 0;

    for ( ;; )
    {
        // Read position and PWM values
        yawReading = getYaw();
        altReading = getAlt();
        PWMmain = getMainPWM();
        PWMtail = getTailPWM();

        UARTSend("\033[2J\033[H");

        // Print output
        printString("Altitude = %4d%%", altReading, 0);
        printString("Yaw      = %4d", yawReading, 1);
        printString("Main PWM = %4d%%", PWMmain, 2);
        printString("Tail PWM = %4d%%", PWMtail, 3);

        // Send output to UART
        usprintf (statusStr, "Alt = %2d | Yaw = %2d |", altReading, yawReading);
        UARTSend (statusStr);
        usprintf (statusStr, "Main PWM = %2d | Tail PWM = %2d |\n\r", getMainPWM(), getTailPWM());
        UARTSend (statusStr);
        usprintf (statusStr, "Alt Ref = %2d | Yaw Ref = %2d |\n\r", GetAltRef(), GetYawRef());
        UARTSend (statusStr);
        usprintf (statusStr, "Mode: %s | 360 Yaw: %d\n\r", getMode(), getYawTotal());
        UARTSend (statusStr);



        static char runtime_stats_buffer[512] = { 0 };

        vTaskGetRunTimeStats(runtime_stats_buffer);

        UARTSend(runtime_stats_buffer);

        vTaskDelay(xDelay1s);
    }
}
