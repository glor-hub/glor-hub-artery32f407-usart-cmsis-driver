//********************************************************************************
//timer.c
//********************************************************************************
#include "at32f403a_407.h"
#include "app.h"
#include "timer.h"
#include <stdbool.h>
#include <string.h>
#include "assert.h"

//********************************************************************************
//Macros
//********************************************************************************

//Status of programming timers on base of SysTick core system.

#define TIMER_STA_READY                         ((uint32_t)0UL)
#define TIMER_STA_SYSTICK_ERR                   ((uint32_t)0UL)

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Variables
//********************************************************************************

typedef struct {
    confirm_state State;
    uint32_t Time;
    uint32_t Counter;
    flag_status Flag;
} Timer_t;

static Timer_t Timer[NUM_TIMERS];

//********************************************************************************
//Prototypes
//********************************************************************************

static bool SysTick_isReady(uint32_t status);

//================================================================================
//Public
//================================================================================

void SysTick_Handler(void)
{
    eTimerTypes timer;
    for(timer = TIMER_DELAY; timer < NUM_TIMERS; timer++) {
        if(Timer[timer].State) {
            if(++Timer[timer].Counter == Timer[timer].Time) {
                Timer[timer].Flag = SET;
            }
        }
    }
}

error_status TimerInit(void)
{
    memset(&Timer, 0, sizeof(Timer_t) * NUM_TIMERS);

    uint32_t status = TIMER_STA_READY;
    if(SysTick_Config(system_core_clock / 1000)) {
#ifdef _APP_DEBUG_
        LOG("In SysTick configuration error is occured.");
#endif//_APP_DEBUG_


        status |= TIMER_STA_SYSTICK_ERR;
    }
    return SysTick_isReady(status) ? SUCCESS : ERROR;
}

void TimerEnable(eTimerTypes timer, uint32_t time)
{
    Timer[timer].State = TRUE;
    Timer[timer].Time = time;
    Timer[timer].Counter = 0;
    Timer[timer].Flag = RESET;
}

void TimerDisable(eTimerTypes timer)
{
    Timer[timer].State = FALSE;
    Timer[timer].Time = 0;
    Timer[timer].Counter = 0;
    Timer[timer].Flag = RESET;
}

flag_status TimerTestFlag(eTimerTypes timer)
{
    return Timer[timer].Flag;
}

confirm_state TimerTestSet(eTimerTypes timer)
{
    return Timer[timer].State;
}

void TimerDoDelay_ms(uint32_t time)
{
    TimerEnable(TIMER_DELAY, time);
    while(TimerTestFlag(TIMER_DELAY) == RESET);
    TimerDisable(TIMER_DELAY);
}


//================================================================================
//Private
//================================================================================
static bool SysTick_isReady(uint32_t status)
{
    return (status == TIMER_STA_READY);
}
