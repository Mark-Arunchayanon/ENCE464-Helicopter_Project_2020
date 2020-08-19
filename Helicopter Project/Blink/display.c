//*****************************************************************************
//
//  Display - Functions used to output values on Display and UART
//
//  Author:  N. James
//           L. Trenberth
//           M. Arunchayanon
//     Last modified:   20.4.2019
//*****************************************************************************


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

//  *****************************************************************************
//  initDisplay:        Initialises Display using OrbitLED functions
void initDisplay (void)
{
    // Initialise the Orbit OLED display
    OLEDInitialise ();
}


//  *****************************************************************************
//  introLine:          Prints the intro line on the OLED Display
void introLine (void)
{
    OLEDStringDraw ("Heli Project", 0, 0);
}


//  *****************************************************************************
//  printString:        Prints the input format and line contents on the given line number on OLED Display
//  TAKES:              line_format - The format to print the string in, including a integer placeholder
//                      line_contents - The integer to print on the line
//                      line_number - The line number integer to print the string on.
void printString(char* restrict line_format, int32_t line_contents, uint8_t line_number)
{
    char string[MAX_STR_LEN + 1];
    usnprintf (string, sizeof(string), line_format, line_contents);
    OLEDStringDraw (string, 0, line_number); // Update line on display.
}


//*****************************************************************************
//  initButtonCheck:    Initialises left and up buttons on the micro-controller
void initButtonCheck (void) {
    SysCtlPeripheralReset (LEFT_BUT_PERIPH);//setting up the LEFT button GPIO
    SysCtlPeripheralReset (UP_BUT_PERIPH);//setting the UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);//setting the DOWN button GPIO
    SysCtlPeripheralReset (RIGHT_BUT_PERIPH);//setting the RIGHT button GPIO
}


////  *****************************************************************************
////  OutputToDisplay:    Displays the helicopter altitude, height and references
////  NOTE:               This function is not currently implemented, though is included for testing
//void OutputToDisplay (void)
//{
//    printString("Altitude = %4d%%", percentAlt, 0);
////    printString("Yaw Angle = %4d", getYaw(), 1);
////    printString("Alt Ref = %4d", GetAltRef(), 2);
////    printString("Yaw Ref = %4d", GetYawRef(), 3);
//}


////*****************************************************************************
////  OutputToDisplay:    Uses UART to send yaw and altitude references,
////                      duty cycles and the current mode
//void OutputToUART (void)
//{
//    char statusStr[MAX_STR_LEN + 1];
//    if (slowTick)
//    {
//        slowTick = false;
////        usprintf (statusStr, "YawRef=%2d Yaw=%2d | \r\n", GetYawRef(), getYaw()); // Form status message
////        UARTSend (statusStr); // Send to the console
//        usprintf (statusStr, "AltRef=%2d Alt=%2d | \r\n", percentAlt, percentAlt);
//        UARTSend (statusStr);
////        usprintf (statusStr, "MDut=%2d TDuty=%2d | \r\n", getMainDuty(), getTailDuty());
////        UARTSend (statusStr);
////        usprintf (statusStr, "Mode=%s | \r\n", getMode());
////        UARTSend (statusStr);
//    }
//}



void vDisplayTask (void *pvParameters)
{
    char statusStr[MAX_STR_LEN + 1];
    const TickType_t xDelay1s = pdMS_TO_TICKS(100);
    int32_t yawReading = 0;
    int32_t altReading = 0;

    for ( ;; )
    {
        yawReading = getYaw();
        altReading = getAlt();

        printString("Altitude = %4d%%", altReading, 0);
        printString("Yaw      = %4d", yawReading, 1);
        printString("Main PWM = %4d%%", getMainPWM(), 2);
        printString("Tail PWM = %4d%%", getTailPWM(), 3);

//        usprintf (statusStr, "\033[2J\033[H Alt = %2d | Yaw = %2d |\n\r"
//                "AltRef = %2d | YawRef = %2d |", percentAlt, degrees, AltRef, YawRef);
//        UARTSend (statusStr);
//        usprintf (statusStr, "\033[2J\033[H Alt = %2d | Yaw = %2d |\n\r", percentAlt, degrees);
//        UARTSend (statusStr);

        usprintf (statusStr, "Alt = %2d | Yaw = %2d |", altReading, yawReading);
        UARTSend (statusStr);
        usprintf (statusStr, "Main PWM = %2d | Tail PWM = %2d |\n\r", getMainPWM(), getTailPWM());
        UARTSend (statusStr);
        usprintf (statusStr, "Alt Ref = %2d | Yaw Ref = %2d |\n\r", GetAltRef(), GetYawRef());
        UARTSend (statusStr);
        usprintf (statusStr, "Mode: %s | 360 Yaw: %d\n\r", getMode(), getYawTotal());
        UARTSend (statusStr);

//        usprintf (statusStr, "\n<script src='https://foo.nz/heliplus-lite.js'></script>");
//        UARTSend (statusStr);

        vTaskDelay(xDelay1s);
    }
}
