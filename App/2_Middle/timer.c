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
    confirm_state state;
    uint32_t time_ms;
    uint32_t counter;
    flag_status flag;
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
        if(Timer[timer].state) {
            if(++Timer[timer].counter == Timer[timer].time_ms) {
                Timer[timer].flag = SET;
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
    Timer[timer].state = TRUE;
    Timer[timer].time_ms = time;
    Timer[timer].counter = 0;
    Timer[timer].flag = RESET;
}

void TimerDisable(eTimerTypes timer)
{
    Timer[timer].state = FALSE;
    Timer[timer].time_ms = 0;
    Timer[timer].counter = 0;
    Timer[timer].flag = RESET;
}

flag_status TimerTestFlag(eTimerTypes timer)
{
    return Timer[timer].flag;
}

confirm_state TimerTestSet(eTimerTypes timer)
{
    return Timer[timer].state;
}

void TimerDoDelay(uint32_t time)
{
    TimerEnable(TIMER_DELAY, time);
    while(TimerTestFlag(TIMER_DELAY)==RESET);
    TimerDisable(TIMER_DELAY);
}


//================================================================================
//Private
//================================================================================
static bool SysTick_isReady(uint32_t status)
{
    return (status == TIMER_STA_READY);
}
