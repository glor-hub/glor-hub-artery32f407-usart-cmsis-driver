#include "at32f403a_407.h"
#include <stdio.h>
#include <stdbool.h>
#include "app.h"
#include "test_periph.h"
#include "systick_timer.h"
#include "assert.h"

int main(void)
{
    error_status init_result;
    TEST_APP_StartUp();
    init_result = TEST_APP_AppInit();

#ifdef _TEST_APP_DEBUG_
    //set a breakpoint in this line for the Debug(print) function to work correctly
    //after switching the clock source to HEXT
    ASSERT(init_result == SUCCESS);
#endif//_TEST_APP_DEBUG_
    TEST_APP_PeriphTest();
    while(1) {
        if(TEST_APP_SYSTICK_TIMER_TimerTestFlag(TEST_APP_SYSTICK_TIMER_POLL)) {
            TEST_APP_AppIdleTask();
            TEST_APP_SYSTICK_TIMER_TimerEnable(TEST_APP_SYSTICK_TIMER_POLL, TEST_APP_SYSTICK_TIMER_POLL_TIME);
        }

    }
}
