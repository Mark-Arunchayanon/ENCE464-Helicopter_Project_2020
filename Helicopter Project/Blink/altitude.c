/***********************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * altitude: Reads the ADC and calculates the altitude as a percentage of maximum height. Also
 *           contains support funcitons.
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
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "stdlib.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "altitude.h"
#include "uart.h"
#include "queue.h"
#include "altitude.h"


/***********************************************************************************************
 * Constants/Definitions
 **********************************************************************************************/
#define RANGE_ALTITUDE      ((1000*4095)/3300)
#define BUF_SIZE            10
#define TASK_STACK_DEPTH    50
#define QUEUE_ITEM_SIZE     15
#define QUEUE_SIZE          15
#define MOVING_AVERAGE      5


/***********************************************************************************************
 * Global Variables
 **********************************************************************************************/
static int32_t refAltitude = 1000;
uint32_t ulValue;
int calibrate_flag=  0;
int calibrate_counter = 0;
static int32_t meanVal = 0;
int32_t percentAlt,testVal;

QueueHandle_t xADCQueue = NULL;     // Queue for ADC sample from pin to calculationSemaphoreHandle_t xADCCalibrationMutex;    // Mutex declaration


// Configures and enables the ADC peripherals and software declarations.
void initADC (void)
{
    char statusStr[MAX_STR_LEN + 1];
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // Configure step 0 on sequence 3.  Sample channel 9 (ADC_CTL_CH9) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);

    // Create Queue for ADC samples
    xADCQueue = xQueueCreate(QUEUE_ITEM_SIZE, sizeof(int32_t));
    if (xADCQueue == NULL)
    {
        usprintf (statusStr, "Could not create ADC queue");
        UARTSend (statusStr);
        while (1);
    }
    // Create mutex for use while altitude is being reset
//    xADCCalibrationMutex = xSemaphoreCreateMutex();
}


// The handler for the ADC conversion complete interrupt. Writes to the ADC queue.
void ADCIntHandler(void)
{
//    uint32_t ulValue;
    // Get the single sample from ADC0.  ADC_BASE is defined in inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

    // Place it in the circular buffer (advancing write index)
    //writeCircBuf (&g_inBuffer, ulValue);

    xQueueSendFromISR( xADCQueue, &ulValue, NULL);

    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

// Getter method which returns the altitude converted to percentage format/
int32_t getAlt (void)
{
    return percentAlt;
}
// Resets the altitude reference to the most recently sampled ADC value./
void resetAltitude (void)
{
    refAltitude = meanVal;
}

// Scheduled task that polls the ADC interrupt at minium 30Hz ??????????????????????????????????????????????????????????????????????????????/

void vADCSampleTask(void *pvParameters)
{
    TickType_t xDelay30s = pdMS_TO_TICKS(30);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for ( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, xDelay30s);

        ADCProcessorTrigger(ADC0_BASE, 3);
        // ADCIntHandler();
    }
}

// Scheduled task reads ADC values from the ADC queue and calculates the altitude value
// using the 0.8V as the maximum height./

void vADCTask(void *pvParameters)
{
    TaskHandle_t xPIDTask = pvParameters;
//    TickType_t xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
    const TickType_t xDelay30s = pdMS_TO_TICKS(30);
    BaseType_t xReceive = pdFALSE;
    uint32_t ADCSamples[QUEUE_SIZE];
    int i, j = 0;

    for ( ;; )
    {

        xReceive = xQueueReceive(xADCQueue, &ADCSamples[i], portMAX_DELAY); //Read from queue;
        if (xReceive == pdTRUE) // If value recieved
        {
            // Moving Average{
            i = (i + 1) % QUEUE_SIZE;

            if (i % MOVING_AVERAGE == 0)
            {
                int offset = ((((i /MOVING_AVERAGE) - 1) *MOVING_AVERAGE + QUEUE_SIZE) % QUEUE_SIZE);
                int32_t sum = 0;
                for (j = 0; j < MOVING_AVERAGE; ++j)
                {
                    sum += ADCSamples[offset + j];
                }
                meanVal = sum / 5;
                percentAlt = (int32_t)((330 * (float)((float)refAltitude - (float)meanVal))) >> 12;

                // clamp between 0 and 100
                if (percentAlt < 0)
                {
                    percentAlt = 0;
                }
                if (percentAlt > 100)
                {
                    percentAlt = 100;
                }

//                percentAlt = 100 *((testVal ));
                xTaskNotifyGive(xPIDTask);  // Notifies PID task that Altitude updated.
            }
        }

        // Altitude reset}
        if (calibrate_flag == 0)
        {
            if (calibrate_counter == 20)    // What is 20???????????????????????????????????????????????????????????????????)
            {
                resetAltitude();
                calibrate_flag = 1;
            }
            calibrate_counter++;
        }

        vTaskDelay(xDelay30s);

    }
}


