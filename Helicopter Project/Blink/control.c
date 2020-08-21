/*************************************************************************************************
 *
 * ENCE464 FreeRTOS Helicopter Rig Controller Project
 *
 * control:         Includes PID control for the Altitude and Yaw, and 5 helicopter modes.
 *                  Landed, Initialising, TakeOff, Flying and Landing
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "math.h"

#include "stdlib.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"

#include "altitude.h"
#include "display.h"
#include "yaw.h"
#include "motor.h"
#include "buttons4.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "uart.h"
#include "timers.h"


/***********************************************************************************************
 * Constants/Definitions
 **********************************************************************************************/
#define ALT_REF_INIT        0    //Initial altitude reference
#define ALT_STEP_RATE       10   //Altitude step rate
#define ALT_MAX             100  //Maximum altitude
#define ALT_MIN             0    //Minimum altitude

#define YAW_REF_INIT        0    //Initial yaw reference
#define YAW_STEP_RATE       15   //Yaw step rate

#define ALT_PROP_CONTROL    0.5
#define ALT_INT_CONTROL     0.05
#define ALT_DIF_CONTROL     1.5

#define YAW_PROP_CONTROL    0.4
#define YAW_INT_CONTROL     0.04
#define YAW_DIF_CONTROL     1.0

#define DELTA_T             0.01 // 1/SYS_TICK_RATE

#define TAIL_OFFSET         15   //Tail offset 20
#define MAIN_OFFSET         20   //Main offset 40

#define MODE_CHANGE_TIME    500   //The time before flipping the switch will
                                 //land the heli instead of swapping mode.

#define TOTAL_ANGLE         360


typedef struct Data {
    int32_t Val;
    int32_t Ref;
    int32_t Err;
    int32_t PrvErr;
    int32_t Control;
    int32_t Duty;
    double Perr;
    double Ierr;
    double Derr;
} DataType;

DataType Alt, Yaw;



//Reading from PC4 to find reference
uint32_t PC4Read = 0;
uint32_t switchState = 0;
bool stable = false, paralysed = true, ref_Found = false;

TimerHandle_t switchTimer;
bool timerResetFlag = false;
bool AltSetUp = false;
bool spinSetUp = false;
int32_t error;

SemaphoreHandle_t xYawMutex, xPIDMutex;

typedef enum {Normal, SplatUp, SplatDown, Spin180Left, Spin180Right}specialMode;
specialMode specialTrick = Normal;

// Declaring modes: Landed, Initialising, TakeOff, Flying, Special and Landing
typedef enum {Landed, Initialising, TakeOff, Flying, Special, Landing} mode_type;
mode_type mode = Landed;  //Initial mode is landed


void YawRefIntHandler(void);

void switchTimerExpire(TimerHandle_t pxTimer);

// Initialises and sets up switch on PC4
void initSwitch_PC4(void)
{
    // Initialise SW1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet (GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA,
           GPIO_PIN_TYPE_STD_WPD);

    // Initialise PC4 used to find yaw ref
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE); //Trigger interrupts on both edges of wave changes on PC4
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4); //Enable interrupts from PC4
    GPIOIntRegister(GPIO_PORTC_BASE, YawRefIntHandler); //If interrupt occurs, run YawRefIntHandler
    IntEnable(INT_GPIOC); //Enable interrupts on C.

    //Initialise reset button
    GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);

    // Initialise switch timer
    switchTimer = xTimerCreate("switch timer", pdMS_TO_TICKS(MODE_CHANGE_TIME), pdFALSE, 0, switchTimerExpire);
    if(switchTimer == NULL)
    {
        while(1);
    }

    xPIDMutex = xSemaphoreCreateBinary();

}

// Reads the reset button, reset system if reset is pushed
void updateReset(void)
{
    uint32_t reset = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
    if(reset == 0) {
        SysCtlReset();
    }
}


// Reads the switch, if the program starts with the switch on,the helicopter will
// be paralysed (not be able to take off)
void GetSwitchState(void)
{
    switchState = GPIOPinRead (GPIO_PORTA_BASE, GPIO_PIN_7) / 128;
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);

    if((mode == Landed) && (switchState == 0) && paralysed)
    {
        paralysed = false;
    }
    if ((mode == Flying || mode == Special) && (switchState == 0)) {
        if(timerResetFlag == false)
        {
            if (xTimerReset(switchTimer, portMAX_DELAY) != pdPASS)
            {
                while(1);
            }
            timerResetFlag = true;
        }
    }
}


// Checks if the helicopter has taken off, sets stable to true
void checkStability(void)
{
    if(getAlt() >= 30) {
        stable = true;
    }
}


// Clamps a value between a minimum and a maximum.
int32_t clamp(int32_t x, int32_t min, int32_t max)
{
    int32_t value = x;
    if(value > max)
    {
        value = max;
    } else if(value < min){
        value = min;
    }
    return value;
}

// Sets the altitude reference
void setAltRef(int32_t newAltRef)
{
    Alt.Ref = clamp(newAltRef, 0, 100);
}


// Sets the yaw reference
void setYawRef(int32_t newYawRef)
{
    if(newYawRef == 0)
    {
        int32_t reference = newYawRef;
        int32_t yaw = getYawTotal() % TOTAL_ANGLE;
        if(yaw > 180)
        {
            Yaw.Ref = reference + (360 - yaw);
        }else {
            Yaw.Ref = reference - yaw;
        }
    }

    Yaw.Ref = newYawRef;
}


// Returns the current reference for the altitude
int32_t GetAltRef(void)
{
    return Alt.Ref;
}


// Returns the current reference for the yaw
int32_t GetYawRef(void)
{
    return Yaw.Ref;
}


// Checks if yaw is zero. If this is true, sets Altitude Reference to 50%
void take_Off(void)
{
    int32_t yaw;
    if(xDisplayMutex != NULL)
    {
        xSemaphoreTake(xDisplayMutex, portMAX_DELAY);
        yaw = getYaw();
        if (abs(yaw) < 10) {
            setAltRef(50);
        }
        xSemaphoreGive(xDisplayMutex);

    }

}


// Controls when the tricks run.
void specialButtonMode(void)
{
    Yaw.Val = getYawTotal();
    Alt.Val = getAlt();
    if(mode == Special)
    {
        if (checkButton (UP) == PUSHED)
        {
            specialTrick = SplatUp;
        }
        if (checkButton (DOWN) == PUSHED)
        {
            specialTrick = SplatDown;
        }
        if (checkButton (LEFT) == PUSHED )
        {
            specialTrick = Spin180Left;
        }
        if (checkButton (RIGHT) == PUSHED)
        {
            specialTrick = Spin180Right;
        }
    }
}


// Makes the helicopter spiral upwards or downwards.
void spiralTrick(void)
{
    if(specialTrick == SplatUp)
    {
        Alt.Val = getAlt();

        if(AltSetUp == false)
        {
            error = Alt.Val + 50;
            setAltRef(error);
            AltSetUp = true;
        } else {
            AltSetUp = false;
            specialTrick = Normal;
        }
    } else if(specialTrick == SplatDown)
    {
        Alt.Val = getAlt();

        if(AltSetUp == false)
        {
            error = Alt.Val - 50;
            setAltRef(error);
            AltSetUp = true;
        } else {
            AltSetUp = false;
            specialTrick = Normal;
        }
    }
}


// Spins the helicopter left or right 180 degrees.
void spinTrick(void)
{
    if(specialTrick == Spin180Left)
    {
        Yaw.Val = getYawTotal();

        if(spinSetUp == false)
        {
            error = Yaw.Val - 180;
            spinSetUp = true;
            setYawRef(error);
        } else {
            spinSetUp = false;
            specialTrick = Normal;
        }

    } else if(specialTrick == Spin180Right)
    {
        Yaw.Val = getYawTotal();

        if(spinSetUp == false)
        {
            error = Yaw.Val + 180;
            setYawRef(error);
            spinSetUp = true;
        } else {
            spinSetUp = false;
            specialTrick = Normal;
        }


// Detects the reference position and resets the reference to 0 when it has been detected.
void YawRefIntHandler(void)
{
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);

    if (ref_Found == false)
    {
        ref_Found = true; //Origin Found
        resetYaw(); //Reset current yaw value to 0
        setYawRef(0); // Resets yaw reference to 0
    }
}

// Once yaw is within 10 degrees of 0, begin to decrease the reference altitude. If yaw is
// outside this range, go to 30% altitude until the yaw stabilises. If altitude is
// under 10%, shut off motors
void landing(void)
{

    if (Yaw.Ref != 0)
    {
        setYawRef(0);
    }

    if(xDisplayMutex != NULL)
    {
        xSemaphoreTake(xDisplayMutex, portMAX_DELAY);
        Yaw.Val = getYaw();
        Alt.Val = getAlt();
        xSemaphoreGive(xDisplayMutex);
    }



    if (abs(Yaw.Val) < 10)
    {
        if (Alt.Val > 10)
        {
            setAltRef(Alt.Val - 15);
        }
        else
        {
            SetMainPWM(0);
            SetTailPWM(0);
        }
    } else {
        setAltRef(30);
    }


}


// Uses PID control during TakeOff, Flying, Special and Landing modes.
// Ensures the yaw follows the yaw reference
void PIDControlYaw(void)
{
    if((mode == TakeOff) || (mode == Flying) || (mode == Special) || (mode == Landing))
    {
        Yaw.Val = 0;
        if (mode == Landing)
        {
            if(xDisplayMutex != NULL)
            {
                xSemaphoreTake(xDisplayMutex, portMAX_DELAY);
                Yaw.Val = getYaw();
                xSemaphoreGive(xDisplayMutex);
            }

        } else {
            Yaw.Val = getYawTotal();
        }

        Yaw.Err = Yaw.Ref - Yaw.Val;  // Calculates the yaw error

        Yaw.Ierr += Yaw.Err * DELTA_T;  //Integral error
        Yaw.Derr  = Yaw.Err-Yaw.PrvErr;  //Derivative error

        Yaw.Control = clamp(Yaw.Err * YAW_PROP_CONTROL, -30, 30)      //yaw control based on PID terms
                    + Yaw.Ierr * YAW_INT_CONTROL
                    + clamp(Yaw.Derr * YAW_DIF_CONTROL, -50, 50)
                    + TAIL_OFFSET;


        SetTailPWM(Yaw.Control);  //Sets the tail duty cycle
        Yaw.PrvErr = Yaw.Err;
        Yaw.Duty = Yaw.Control;
    }
}


// Uses PID control during TakeOff, Flying, Special and Landing modes.
// Ensures the altitude follows the altitude reference
void PIDControlAlt(void)
{
    if ((mode == TakeOff) || (mode == Flying) || (mode == Special) || (mode == Landing)) {

        Alt.Err = Alt.Ref - getAlt();  //Calculates altitude error

        Alt.Ierr += Alt.Err * DELTA_T;  //Integral error
        Alt.Derr = (Alt.Err - Alt.PrvErr) * 100;  //Derivative error


        Alt.Control = clamp(Alt.Err * ALT_PROP_CONTROL, -20, 30)  //Altitude control based on the PID terms
                            + Alt.Ierr * ALT_INT_CONTROL
                            + clamp(Alt.Derr * ALT_DIF_CONTROL, -40, 60)
                            + MAIN_OFFSET;

        SetMainPWM(Alt.Control);  //Sets the main duty cycle

        Alt.PrvErr = Alt.Err;
        Alt.Duty = Alt.Control;
    }
}


// Returns main rotor duty cycle
uint32_t getMainDuty(void)
{
    return Alt.Duty;
}


// Returns tail duty cycle
uint32_t getTailDuty(void)
{
    return Yaw.Duty;
}


// Finds the current mode of the helicopter
char* getMode(void)
{
    switch(mode)
    {
    case Landed: return "Landed";
    case Initialising: return "Initialising";
    case TakeOff:  return "TakeOff";
    case Flying: return"Flying";
    case Special: return"Special";
    case Landing: return "Landing";
    }

   return NULL;
}


// Reset all error and integral error to 0
void resetIntControl(void)
{
    Alt.Err = 0;
    Alt.Ierr = 0;
    Alt.PrvErr = 0;
    Yaw.Err = 0;
    Yaw.Ierr = 0;
    Yaw.PrvErr = 0;
}


// Checks button status and changes reference altitudes and yaws
void RefUpdate(void)
{
    int32_t alt = GetAltRef();
    if(mode == Flying) {
        if ((checkButton (UP) == PUSHED) && (Alt.Ref < ALT_MAX))
        {
            setAltRef(Alt.Ref + ALT_STEP_RATE);
        }
        if ((checkButton (DOWN) == PUSHED) && (Alt.Ref > ALT_MIN))
        {
            setAltRef(Alt.Ref - ALT_STEP_RATE);
        }
        if (checkButton (LEFT) == PUSHED )
        {
            Yaw.Ref -= YAW_STEP_RATE;
        }
        if (checkButton (RIGHT) == PUSHED)
        {
            Yaw.Ref += YAW_STEP_RATE;
        }

    }
}


// Switches mode between the 6 modes
void helicopterStates(void){

    switch(mode) {
    case Landed:

        if (switchState == 1 && !paralysed) {  //If switch is flicked on and the helicopter is not paralysed
            mode = Initialising;               // Change mode to Initialising
            resetIntControl();                 //Reset any previous error terms

            //Sets initial power percentages
//            setAltRef(10);
            SetMainPWM(0);
            SetTailPWM(15);

        }
        break;

    case Initialising:

        if(ref_Found) {
            mode = TakeOff;
            ref_Found = false;//Change mode to takeoff once the reference point is found
        }
        break;

    case TakeOff:

        take_Off();                            //Set yaw to 0 and raises the helicopter up to 50% altitude
        checkStability();
        if(stable) {
            mode = Flying;                     //Once the reference point is met and the correct altitude is reached set the state to flying
        }
        break;

    case Flying:

        RefUpdate();                           //Checks if for button pushes and alter references
        break;

    case Special:

        specialButtonMode();
        spiralTrick();
        spinTrick();
        break;

    case Landing:
        landing();

        if (getAlt() == 0) {          //If altitude is at 0, change mode to landed
            mode = Landed;
        }
        break;
    }
}


// A task scheduled by FreeRTOS to update the controls. Updates the PID controllers, gets the
// switch position and updates the state machine. Has a task priority of 4.
void vControlTask (void *pvParameters)
{

    for ( ;; )
    {


        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        GetSwitchState();
        PIDControlAlt();
        PIDControlYaw();
        helicopterStates();

    }
}

// The handler for the switch timer. Should switch to landing mode or the second control mode.
void switchTimerExpire(TimerHandle_t pxTimer)
{
    if (switchState == 0)
    {
        mode = Landing;
        setYawRef(0);                      //Set yaw reference to 0
    }
    else
    {
        if (mode == Flying)
        {
            mode = Special;
        }
        else if (mode == Special)
        {
            mode = Flying;
        }
    }
    timerResetFlag = false;
}
