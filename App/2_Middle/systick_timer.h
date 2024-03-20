#ifndef _SYSTICK_TIMER_H_
#define _SYSTICK_TIMER_H_

#include "at32f403a_407.h"

//Programming timers on base of SysTick core system.
typedef enum {
    TEST_APP_SYSTICK_TIMER_DELAY = 0,
    TEST_APP_SYSTICK_TIMER_NUM_TIMERS
} eTEST_APP_SYSTICK_TIMER_TimerTypes_t;

error_status TEST_APP_SYSTICK_TIMER_TimerInit(void);

void TEST_APP_SYSTICK_TIMER_DoDelay_ms(uint32_t time);

#endif //_SYSTICK_TIMER_H_
