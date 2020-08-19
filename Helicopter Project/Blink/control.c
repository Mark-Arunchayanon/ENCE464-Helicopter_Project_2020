//*****************************************************************************
//
// control - Includes PID control for the Altitude and Yaw, and 5 helicopter modes.
//           Landed, Initialising, TakeOff, Flying and Landing
//
// Author:  N. James
//          L. Trenberth
//          M. Arunchayanon
// Last modified:   31.5.2019
//
//*****************************************************************************

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

#define ALT_REF_INIT        0    //Initial altitude reference
#define ALT_STEP_RATE       10   //Altitude step rate
#define ALT_MAX             100  //Maximum altitude
#define ALT_MIN             0    //Minimum altitude

#define YAW_REF_INIT        0    //Initial yaw reference
#define YAW_STEP_RATE       15   //Yaw step rate

// Helirig 1
//#define ALT_PROP_CONTROL    0.7  //Altitude PID control
//#define ALT_INT_CONTROL     0.2
//#define ALT_DIF_CONTROL     0.6

//#define ALT_PROP_CONTROL    0.45//Altitude PID control
//#define ALT_INT_CONTROL     0.0
//#define ALT_DIF_CONTROL     0.6

// Helirig 3
//#define ALT_PROP_CONTROL    1.0  //Altitude PID control
//#define ALT_INT_CONTROL     1.0
//#define ALT_DIF_CONTROL     1.0

//// Milestone
#define ALT_PROP_CONTROL    0.5  //Altitude PID control
#define ALT_INT_CONTROL     0.009 //0.16
#define ALT_DIF_CONTROL     10 //0.5 // 0.2 had a lot of overshoot

#define YAW_PROP_CONTROL    0.6  //Yaw PID control
#define YAW_INT_CONTROL     0.05
#define YAW_DIF_CONTROL     1.5

#define DELTA_T             0.01 // 1/SYS_TICK_RATE

#define TAIL_OFFSET         35   //Tail offset
#define MAIN_OFFSET         39   //Main offset

#define MODE_CHANGE_TIME    500   //The time before flipping the switch will
                                 //land the heli instead of swapping mode.

#define TOTAL_ANGLE         360 // Total degrees

//sets the intial value of the Altitude and
extern int32_t AltRef =  ALT_REF_INIT;
extern int32_t YawRef = YAW_REF_INIT;

//Sets integral error variables
static int32_t AltIntError = 0;
static int32_t AltPreviousError = 0;
static int32_t YawIntError = 0;
static int32_t YawPreviousError = 0;

//Yaw error and control variables
int32_t Yaw_error, YawDerivError;
uint32_t YawControl;

//Altitude error and control variables
int32_t Alt_error = 0, AltDerivError;
int32_t AltControl;

//Main and tail duty cycle
uint32_t mainDuty = 0, tailDuty = 0;

//Reading from PC4 to find reference
uint32_t PC4Read = 0;
uint32_t switchState = 0;
bool stable = false, paralysed = true, ref_Found = false;

TimerHandle_t switchTimer;
bool timerResetFlag = false;
bool spiralSetUp = false;
bool spinSetUp = false;
static int32_t error;

SemaphoreHandle_t xSemSpiralSetUp, xSemSpinSetUp, xSemTimerResetFlag;

typedef enum {Normal, SpiralUp, SpiralDown, Spin180Left, Spin180Right}specialMode;
specialMode specialTrick = Normal;

// *******************************************************
// Declaring modes Landed, Initialising, TakeOff, Flying and Landing
typedef enum {Landed, Initialising, TakeOff, Flying, Special, Landing} mode_type;
mode_type mode = Landed;  //Initial mode is landed


void YawRefIntHandler(void);

void switchTimerExpire(TimerHandle_t pxTimer);

// *******************************************************
// initSwitch_PC4:      Initialises and sets up switch on PC4
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

}

// *******************************************************
// updateReset:         Reads the reset button, reset system if reset is pushed
void updateReset(void)
{
    uint32_t reset = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
    if(reset == 0) {
        SysCtlReset();
    }
}


// *******************************************************
// GetSwitchState:      Reads the switch, if the program starts with the switch on,
//                      the helicopter will be paralysed (not be able to take of)
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


// *******************************************************
// checkStability:      Checks if the helicopter has taken off, sets stable to true
void checkStability(void)
{
    if(getAlt() >= 30) {
        stable = true;
    }
}


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

// *******************************************************
// setAltRef:           Sets the altitude reference
// TAKES:               New altitude reference as a percentage
void setAltRef(int32_t newAltRef)
{
    AltRef = clamp(newAltRef, 10, 100);
}


// *******************************************************
// setYawRef:           Sets the yaw reference
// TAKES:               newYawRef, the new yaw reference as a percentage
void setYawRef(int32_t newYawRef)
{
    if(newYawRef == 0)
    {
        int32_t reference = newYawRef;
        int32_t yaw = getYawTotal() % TOTAL_ANGLE;
        if(yaw > 180)
        {
            YawRef = reference + (360 - yaw);
        }else {
            YawRef = reference - yaw;
        }
    }

    YawRef = newYawRef;
}


// *******************************************************
// GetAltRef:           Returns the current reference for the altitude
// RETURNS:             Altitude Reference as a int32_t
int32_t GetAltRef(void)
{
    return AltRef;
}


// *******************************************************
// GetYawRef:           Returns the current reference for the yaw
// RETURNS:             Yaw Reference as a int32_t
int32_t GetYawRef(void)
{
    return YawRef;
}


// *******************************************************
// take_Off:            Checks if yaw is zero.
//                      If this is true, sets Altitude Reference to 50%
void take_Off(void)
{
    int32_t yaw = getYaw();
    if (abs(yaw) < 10) {
        setAltRef(50);
    }
}


void specialButtonMode(void)
{
    int32_t currentYaw = getYawTotal();
    int32_t currentAlt = getAlt();
    if(mode == Special)
    {
        if (checkButton (UP) == PUSHED)
        {
            specialTrick = SpiralUp;
        }
        if (checkButton (DOWN) == PUSHED)
        {
            specialTrick = SpiralDown;
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

void spiralTrick(void)
{
    if(specialTrick == SpiralUp)
    {
        int32_t currentYaw = getYawTotal();
        int32_t currentYaw180 = getYaw();
        int32_t currentAlt = getAlt();
        int32_t refAlt = GetAltRef();
        int32_t refYaw = GetYawRef();

        if(spiralSetUp == false)
        {
            setYawRef(0);
            setAltRef(10);
            if(currentAlt == 10)
            {
                spiralSetUp = true;
            }
        }

        if(spiralSetUp == true)
        {
            if ((currentAlt > 80 && currentAlt < 96) && abs(currentYaw180) < 5)
            {
                spiralSetUp = false;
                specialTrick = Normal;
            } else {
                if (((currentAlt < refAlt + 5) && (currentAlt < refAlt + 5)) &&
                        ((currentYaw < refYaw + 5) && (currentYaw < refYaw + 5)))
                {
                    setAltRef(GetAltRef() + 10);
                    setYawRef(GetYawRef() + 45);
                }
            }

        }
    } else if(specialTrick == SpiralDown)
    {
        int32_t currentYaw = getYawTotal();
        int32_t currentYaw180 = getYaw();
        int32_t currentAlt = getAlt();
        int32_t refAlt = GetAltRef();
        int32_t refYaw = GetYawRef();

        if(spiralSetUp == false)
        {
            setYawRef(0);
            setAltRef(90);
            if(currentAlt == 90)
            {
                spiralSetUp = true;
            }
        }

        if(spiralSetUp == true)
        {
            if ((currentAlt > 0 && currentAlt < 16) && abs(currentYaw180) < 5)
            {
                spiralSetUp = false;
                specialTrick = Normal;
            } else {
                if (((currentAlt < refAlt + 5) && (currentAlt < refAlt + 5)) &&
                        ((currentYaw < refYaw + 5) && (currentYaw < refYaw + 5)))
                {
                    setAltRef(GetAltRef() - 10);
                    setYawRef(GetYawRef() - 45);
                }
            }

        }
    }
}


// Here I switched around the + and - on Left and Right to match in IRL. Right now, spint right goes out of control, yaw keeps going doesnt stop
void spinTrick(void)
{
    if(specialTrick == Spin180Left)
    {
        int32_t currentYaw = getYawTotal();

        if(spinSetUp == false)
        {
            error = currentYaw - 180;
            spinSetUp = true;
        }

        if((currentYaw < error - 5) && (currentYaw > error + 5))
        {
            setYawRef(currentYaw - 10);
        } else {
            spinSetUp = false;
            specialTrick = Normal;
        }
    } else if(specialTrick == Spin180Right)
    {
        int32_t currentYaw = getYawTotal();

        if(spinSetUp == false)
        {
            error = currentYaw + 180;
            spinSetUp = true;
        }

        if((currentYaw < error - 5) && (currentYaw > error + 5))
        {
            setYawRef(currentYaw + 15);
        } else {
            spinSetUp = false;
            specialTrick = Normal;
        }
    }
}
// *******************************************************
// findYawRef:          Turns on main and tail motor. Spins the helicopter clockwise
//                      and  reads PC4 to check if the helicopter is at the reference
//                      Once the reference is found, resets yaw reference to 0 and current yaw to 0
//void findYawRef(void)
//{
//
//    //Reads the PC4 values
//    PC4Read = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);
//    // GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
//    if(PC4Read == 0) {
//        ref_Found = true; //Origin Found
//        resetYaw(); //Reset current yaw value to 0
//        setYawRef(0); // Resets yaw reference to 0
//    }
//}

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

// *******************************************************
// landing:             Once yaw is within 5 degrees of 0,
//                      decrease altitude by 5% if over 10%
//                      If altitude is under 10%, shut off motors
void landing(void)
{

    if (YawRef != 0)
    {
        setYawRef(0);
    }

    int32_t currentYaw = getYaw();
    int32_t currentAlt = getAlt();
    if (abs(currentYaw) < 10)
    {
        if (currentAlt > 10)
        {
            setAltRef(currentAlt - 15);
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


// *******************************************************
//  PIDControlYaw:      Uses PID control during TakeOff, Flying and Landing modes
//                      Ensures the yaw follows the yaw reference
void PIDControlYaw(void)
{
    if( (mode == TakeOff) || (mode == Flying) || (mode == Special) || (mode == Landing))
    {
        int32_t currentYaw = 0;
        if (mode == Landing)
        {
            currentYaw = getYaw();
        } else {
            currentYaw = getYawTotal();
        }

        Yaw_error = YawRef - currentYaw;  // Calculates the yaw error

        YawIntError += Yaw_error * DELTA_T;  //Integral error
        YawDerivError  = Yaw_error-YawPreviousError;  //Derivative error

        YawControl = Yaw_error * YAW_PROP_CONTROL      //yaw control based on PID terms
                    + YawIntError * YAW_INT_CONTROL
                    + YawDerivError * YAW_DIF_CONTROL
                    + TAIL_OFFSET;


        YawControl = clamp(YawControl, 10, 90);
        SetTailPWM(YawControl);  //Sets the tail duty cycle
        YawPreviousError = Yaw_error;
        tailDuty = YawControl;
    }
}


// *******************************************************
// PIDControlAlt:       Uses PID control during TakeOff, Flying and Landing modes
//                      Ensures the altitude follows the altitude reference
void PIDControlAlt(void)
{
    if ((mode == TakeOff) || (mode == Flying) || (mode == Special) || (mode == Landing)) {

        Alt_error = AltRef - getAlt();  //Calculates altitude error

        AltIntError += Alt_error * DELTA_T;  //Integral error
        AltDerivError = (Alt_error-AltPreviousError) * 100;  //Derivative error

        AltControl = Alt_error * ALT_PROP_CONTROL  //Altitude control based on the PID terms
                    + AltIntError * ALT_INT_CONTROL
                    + AltDerivError * ALT_DIF_CONTROL
                    + MAIN_OFFSET;

        AltControl = clamp(AltControl, 10, 90);

        SetMainPWM(AltControl);  //Sets the main duty cycle
        AltPreviousError = Alt_error;
        mainDuty = AltControl;
    }
}


// *******************************************************
// getMainDuty:         Returns main rotor duty cycle
// RETURNS:             The main duty cycle as a uint32_t
uint32_t getMainDuty(void)
{
    return mainDuty;
}


// *******************************************************
// getTailDuty:         Returns tail duty cycle
// RETURNS:             The tail rotor duty cycle as a uint32_t
uint32_t getTailDuty(void)
{
    return tailDuty;
}


// *******************************************************
// getMode:             Finds the current mode of the helicopter
// RETURNS:             A char* containing the current mode
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


// *******************************************************
// resetIntControl:     Reset all error and integral error to 0
void resetIntControl(void)
{
    Alt_error = 0;
    AltIntError = 0;
    AltPreviousError = 0;
    Yaw_error = 0;
    YawIntError = 0;
    YawPreviousError = 0;
}


// *******************************************************
// RefUpdate:           Only runs when the helicopter is in flying mode
//                      Checks button status and changes reference altitudes and yaws
//                      UP and DOWN are used to increase/decrease altitude reference
//                      LEFT and RIGHT are used to increase/decrease yaw reference
void RefUpdate(void)
{
    if(mode == Flying) {
        if ((checkButton (UP) == PUSHED) && (AltRef < ALT_MAX))
        {
            setAltRef(GetAltRef() + ALT_STEP_RATE);
        }
        if ((checkButton (DOWN) == PUSHED) && (AltRef > ALT_MIN))
        {
            setAltRef(GetAltRef() - ALT_STEP_RATE);
        }
        if (checkButton (LEFT) == PUSHED )
        {
            YawRef -= YAW_STEP_RATE;
        }
        if (checkButton (RIGHT) == PUSHED)
        {
            YawRef += YAW_STEP_RATE;
        }

    }
}


// *******************************************************
// helicopterStates:    Switches mode between the 5 modes
void helicopterStates(void){

    switch(mode) {
    case Landed:

        if (switchState == 1 && !paralysed) {  //If switch is flicked on and the helicopter is not paralysed
            mode = Initialising;               // Change mode to Initialising
            resetIntControl();                 //Reset any previous error terms

            //Sets initial power percentages
            SetMainPWM(15);
            SetTailPWM(25);
        }
        break;

    case Initialising:

//        findYawRef();                          //Spins clockwise until the reference point is found
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


void vControlTask (void *pvParameters)
{
    TickType_t xDelay10s = pdMS_TO_TICKS(30);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for ( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, xDelay10s);

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
