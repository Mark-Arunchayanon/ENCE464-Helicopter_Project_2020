#define RANGE_ALTITUDE      (1000*4095/3300)
#define BUF_SIZE            10
#define TASK_STACK_DEPTH    50
#define QUEUE_ITEM_SIZE     15
#define QUEUE_SIZE          15

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

static uint32_t refAltitude = 1000;       //Reference Altitude
//static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
uint32_t ulValue;
int calibrate_flag =    0;
int calibrate_counter = 0;
static int32_t percentAlt =    0;
//int32_t percentAlt =    0;
static int32_t meanVal =0;


// Queue for ADC sample from pin to calculation
QueueHandle_t xADCQueue = NULL;
extern xSemaphoreHandle g_pADCSemaphore;

//void vADCTask(void *pvParameters);



//  *****************************************************************************
//  ADCIntHandler: The handler for the ADC conversion complete interrupt.
//                 Writes to the circular buffer.
//  Taken from:    Week4Lab ADCDemo1.c
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


//  *****************************************************************************
//  initADC:    Configures and enables the ADC
//  Taken from: Week4Lab ADCDemo1.c
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

    //
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

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);

    //
    // Create Queue for ADC samples
    xADCQueue = xQueueCreate(QUEUE_ITEM_SIZE, sizeof(int32_t));
    if (xADCQueue == NULL)
    {
        usprintf (statusStr, "Could not create ADC queue");
        UARTSend (statusStr);
    }

}


//  *****************************************************************************
//  computeAltitude: Calculates the average altitude from the ADC.
//  Taken from:      Week4Lab ADCDemo1.c main function
//  RETURNS:         The calculated ADC altitude value as a int32_t
//int32_t computeAltitude (void)
//{
//    int AltSum = 0;
//    int i = 0;
//
//    for (i = 0; i < BUF_SIZE; i++) {
//            AltSum = AltSum + readCircBuf (&g_inBuffer); // Adds each item in the buffer to the sum.
//    }
//    return ((2 * AltSum + BUF_SIZE) / 2 / BUF_SIZE);    //returns an overall sum.
//}

int32_t getAlt (void)
{
    return percentAlt;
}


//  *****************************************************************************
//  resetAltitude: Resets the refAltitude to be current ADC altitude.
void resetAltitude (void)
{
    refAltitude = meanVal;
}


//  *****************************************************************************
//  percentAltitude: Converts the ADC Altitude into a usable percentage altitude
//                   using a 0.8V difference as the maximum height
//  RETURNS:         A Height Percentage as a int32_t from the reference height.
int32_t percentAltitude(void)
{
    int32_t percent = 1000;
    percent = 100*(refAltitude-meanVal);
    return percent/RANGE_ALTITUDE; //returns percentage of 0.8V change
}


//  *****************************************************************************
//  bufferLocation: Returns the location of the circular buffer
//  RETURNS:        A pointer to a circbuf_t
//circBuf_t* bufferLocation(void)
//{
//    return &g_inBuffer;
//}


void vADCSampleTask(void *pvParameters)
{
    TickType_t xDelay10s = pdMS_TO_TICKS(30);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for ( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, xDelay10s);

        ADCProcessorTrigger(ADC0_BASE, 3);

        ADCIntHandler();

    }
}


void vADCTask(void *pvParameters)
{
//    TickType_t xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
    const TickType_t xDelay1s = pdMS_TO_TICKS(20);
    BaseType_t xReceive = pdFALSE;
//    char statusStr[MAX_STR_LEN + 1];
    uint32_t ADCSamples[QUEUE_SIZE];
    int i, j = 0;


    for ( ;; )
    {

        xReceive = xQueueReceive(xADCQueue, &ADCSamples[i], portMAX_DELAY);
        if (xReceive == pdTRUE)
        {
            i = (i + 1) % QUEUE_SIZE;

            if (i % 5 == 0)
            {
                int offset = ((((i / 5) - 1) * 5 + QUEUE_SIZE) % QUEUE_SIZE);
                int sum = 0;
                for (j = 0; j< 5; ++j)
                {
                    sum += ADCSamples[offset + j];
                }
                meanVal = sum / 5;
                percentAlt = 100*((int32_t)refAltitude-meanVal) / RANGE_ALTITUDE;
            }
        }



        if (calibrate_flag == 0)
        {
            if (calibrate_counter == 20)
            {
                resetAltitude();
                calibrate_flag = 1;
            }
            calibrate_counter++;
        }

        //
        // Guard UART from concurrent access. Print the currently
        // blinking LED.
        //
//        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
//        printf("Left Button is pressed.\n");
//        xSemaphoreGive(g_pUARTSemaphore);

//        usprintf (statusStr, "\nulValue: %d\r", ulValue);
//        UARTSend (statusStr);
//
//        usprintf (statusStr, "\nADC Buffer: %d\r", height);
//        UARTSend (statusStr);
//
//        usprintf (statusStr, "\nReference: %d\r", refAltitude);
//        UARTSend (statusStr);

//        usprintf (statusStr, "\r\nMean: %d", meanVal);
//        UARTSend (statusStr);

//        usprintf (statusStr, "\r\nPercent: %d", percentAlt);
//        UARTSend (statusStr);

        vTaskDelay(xDelay1s);
    }

}
