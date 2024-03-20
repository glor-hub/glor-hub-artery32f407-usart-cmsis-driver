//********************************************************************************
//software_timer.c
//********************************************************************************
#include "at32f403a_407.h"
#include <string.h>
#include "arm_driver.h"
#include "arm_timer.h"
#include "software_timer.h"

//********************************************************************************
//Macros
//********************************************************************************


//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************


//********************************************************************************
//Variables
//********************************************************************************

static TEST_APP_SOFTWARE_TIMER_Timer_t Timer[TEST_APP_SOFTWARE_TIMER_NUM_TIMERS];

//********************************************************************************
//Prototypes
//********************************************************************************

//================================================================================
//Public
//================================================================================

void TEST_APP_SOFTWARE_TIMER_TimerInit(void)
{
    memset(&Timer, 0, sizeof(TEST_APP_SOFTWARE_TIMER_Timer_t) *
           TEST_APP_SOFTWARE_TIMER_NUM_TIMERS);
    //to do
}


void TEST_APP_SOFTWARE_TIMER_TimerEnable(eTEST_APP_SOFTWARE_TIMER_TimerTypes_t timer, uint32_t time)
{
    Timer[timer].State = TRUE;
    Timer[timer].Time = time;
    Timer[timer].Counter = 0;
    Timer[timer].Flag = RESET;
}

void TEST_APP_SOFTWARE_TIMER_TimerDisable(eTEST_APP_SOFTWARE_TIMER_TimerTypes_t timer)
{
    Timer[timer].State = FALSE;
    Timer[timer].Time = 0;
    Timer[timer].Counter = 0;
    Timer[timer].Flag = RESET;
}

flag_status TEST_APP_SOFTWARE_TIMER_TimerTestFlag(eTEST_APP_SOFTWARE_TIMER_TimerTypes_t timer)
{
    return Timer[timer].Flag;
}

confirm_state TEST_APP_SOFTWARE_TIMER_TimerTestSet(eTEST_APP_SOFTWARE_TIMER_TimerTypes_t timer)
{
    return Timer[timer].State;
}


//================================================================================
//Private
//================================================================================

