/*
 * createTasks.c
 *
 *  Created on: 7/08/2020
 *      Author: par116
 */

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "altitude.h"
#include "circBufT.h"
#include "uart.h"
#include "yaw.h"
#include "buttons4.h"
#include "motor.h"
#include "control.h"

bool createTasks (void)
{

    // Create ADC Task
    if (pdTRUE != xTaskCreate(vADCSample, "ADC Sampler", TASK_STACK_DEPTH, NULL, 3,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vADCTask, "ADC Calc", TASK_STACK_DEPTH, NULL, 3,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vYawTask, "Yaw", TASK_STACK_DEPTH, NULL, 3,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vButtonTask, "Buttons", TASK_STACK_DEPTH, NULL, 3,
                           NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vMotorTask, "Motors", TASK_STACK_DEPTH, NULL, 3,
                               NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    if (pdTRUE != xTaskCreate(vControlTask, "Control", TASK_STACK_DEPTH, NULL, 3,
                               NULL))
    { // (void *)1 is our pvParameters for our task func specifying PF_1
        while (1); // error creating task, out of memory?
    }

    return true;
}



