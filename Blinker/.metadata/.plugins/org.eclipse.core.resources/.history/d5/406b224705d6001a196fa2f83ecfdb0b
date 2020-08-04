//*****************************************************************************
//
// Altitude - File containing ADC and altitude calculations
//
// Author:  N. James
//          L. Trenberth
//          M. Arunchayanon
// Last modified:   31.5.2019
//
//*****************************************************************************

#define RANGE_ALTITUDE 1000*4095/3300
#define BUF_SIZE 10


#include "circBufT.h"
#include "system.h"
#include "driverlib/adc.h"



static uint32_t refAltitude;       //Reference Altitude
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)




//  *****************************************************************************
//  ADCIntHandler: The handler for the ADC conversion complete interrupt.
//                 Writes to the circular buffer.
//  Taken from:    Week4Lab ADCDemo1.c
void ADCIntHandler(void)
{
    uint32_t ulValue;
    // Get the single sample from ADC0.  ADC_BASE is defined in inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);

    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}


//  *****************************************************************************
//  initADC:    Configures and enables the ADC
//  Taken from: Week4Lab ADCDemo1.c
void initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

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
}


//  *****************************************************************************
//  computeAltitude: Calculates the average altitude from the ADC.
//  Taken from:      Week4Lab ADCDemo1.c main function
//  RETURNS:         The calculated ADC altitude value as a int32_t
int32_t computeAltitude (void)
{
    int AltSum = 0;
    int i = 0;

    for (i = 0; i < BUF_SIZE; i++) {
            AltSum = AltSum + readCircBuf (&g_inBuffer); // Adds each item in the buffer to the sum.
    }
    return ((2 * AltSum + BUF_SIZE) / 2 / BUF_SIZE);    //returns an overall sum.
}


//  *****************************************************************************
//  resetAltitude: Resets the refAltitude to be current ADC altitude.
void resetAltitude (void)
{
    refAltitude = computeAltitude();
}


//  *****************************************************************************
//  percentAltitude: Converts the ADC Altitude into a usable percentage altitude
//                   using a 0.8V difference as the maximum height
//  RETURNS:         A Height Percentage as a int32_t from the reference height.
int32_t percentAltitude(void)
{
    int32_t percent = 0;
    percent = 100*(refAltitude-computeAltitude());
    return percent/RANGE_ALTITUDE; //returns percentage of 0.8V change
}


//  *****************************************************************************
//  bufferLocation: Returns the location of the circular buffer
//  RETURNS:        A pointer to a circbuf_t
circBuf_t* bufferLocation(void)
{
    return &g_inBuffer;
}
