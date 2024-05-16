//********************************************************************************
//systick_timer.c


//********************************************************************************
#include "at32f403a_407.h"
#include <string.h>
#include "arm_driver.h"
#include "systick_timer.h"
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

static TEST_APP_SOFTWARE_TIMER_Timer_t Timer[TEST_APP_SYSTICK_TIMER_NUM_TIMERS];

//********************************************************************************
//Prototypes
//********************************************************************************

//================================================================================
//Public
//================================================================================

void SysTick_Handler(void)
{
    eTEST_APP_SYSTICK_TIMER_TimerTypes_t timer;
    for(timer = TEST_APP_SYSTICK_TIMER_POLL;
        timer < TEST_APP_SYSTICK_TIMER_NUM_TIMERS;
        timer++) {
        if(Timer[timer].State) {
            if(++Timer[timer].Counter == Timer[timer].Time) {
                Timer[timer].Flag = SET;
            }
        }
    }
}

error_status TEST_APP_SYSTICK_TIMER_TimerInit(void)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    memset(&Timer, 0, sizeof(TEST_APP_SOFTWARE_TIMER_Timer_t) *
           TEST_APP_SYSTICK_TIMER_NUM_TIMERS);
    if(SysTick_Config(system_core_clock / 1000)) {
#ifdef _TEST_APP_DEBUG_
        LOG("In SysTick configuration error is occured.");
#endif//_TEST_APP_DEBUG_
        drv_status |= TEST_APP_ARM_DRIVER_ERROR;
    }
    TEST_APP_SYSTICK_TIMER_TimerEnable(TEST_APP_SYSTICK_TIMER_POLL, TEST_APP_SYSTICK_TIMER_POLL_TIME);
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}

void TEST_APP_SYSTICK_TIMER_TimerEnable(eTEST_APP_SYSTICK_TIMER_TimerTypes_t timer, uint32_t time)
{
    Timer[timer].State = TRUE;
    Timer[timer].Time = time;
    Timer[timer].Counter = 0;
    Timer[timer].Flag = RESET;
}

void TEST_APP_SYSTICK_TIMER_TimerDisable(eTEST_APP_SYSTICK_TIMER_TimerTypes_t timer)
{
    Timer[timer].State = FALSE;
    Timer[timer].Time = 0;
    Timer[timer].Counter = 0;
    Timer[timer].Flag = RESET;
}

flag_status TEST_APP_SYSTICK_TIMER_TimerTestFlag(eTEST_APP_SYSTICK_TIMER_TimerTypes_t timer)
{
    return Timer[timer].Flag;
}

confirm_state TEST_APP_SYSTICK_TIMER_TimerTestSet(eTEST_APP_SYSTICK_TIMER_TimerTypes_t timer)
{
    return Timer[timer].State;
}


void TEST_APP_SYSTICK_TIMER_DoDelay_ms(uint32_t time)
{
    TEST_APP_SYSTICK_TIMER_TimerEnable(TEST_APP_SYSTICK_TIMER_DELAY, time);
    while(TEST_APP_SYSTICK_TIMER_TimerTestFlag(TEST_APP_SYSTICK_TIMER_DELAY) == RESET);
    TEST_APP_SYSTICK_TIMER_TimerDisable(TEST_APP_SYSTICK_TIMER_DELAY);
}


//================================================================================
//Private
//================================================================================
