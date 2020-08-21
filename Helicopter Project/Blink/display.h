#ifndef DISPLAY_H_
#define DISPLAY_H_
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


#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
#include "altitude.h"
#include "FreeRTOS.h"
#include "semphr.h"


/***********************************************************************************************
 * Constants/Definitions
 **********************************************************************************************/
extern SemaphoreHandle_t xDisplayMutex; // Create mutex for controlling access to yaw and altitude.


/***********************************************************************************************
 * initDisplay:        Initialises Display using OrbitLED functions
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void initDisplay (void);


/***********************************************************************************************
 * introLine:          Prints the intro line on the OLED Display
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void introLine (void);


/***********************************************************************************************
 * printString:        Prints the input format and line contents on the given line number on OLED Display
 * TAKES:              line_format - The format to print the string in, including a integer placeholder
 *                     line_contents - The integer to print on the line
 *                     line_number - The line number integer to print the string on.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void printString(char* restrict line_format, int32_t line_contents, uint8_t line_number);


/***********************************************************************************************
 * initButtonCheck:    Initialises left and up buttons on the micro-controller
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void initButtonCheck (void);


/***********************************************************************************************
 * vDisplayTask:       A task scheduled by FreeRTOS to update the display. Prints a readout of
 *                     position and control details to the display and UART. Has a task priority of 3.
 *
 * Authors: M Arunchyanon, S. Goonatillake, G.Thiele
 * Last Modified: 21.08.2020
 **********************************************************************************************/
void vDisplayTask (void *pvParameters);


#endif /*DISPLAY_H_*/
