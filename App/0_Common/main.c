#include "at32f403a_407.h"
#include <stdio.h>
#include <stdbool.h>
#include "app.h"
#include "test_periph.h"
#include "assert.h"

int main(void)
{
    error_status init_result;
    init_result = AppInit();

#ifdef _APP_DEBUG_
    //set a breakpoint in this line for the Debug(print) function to work correctly
    //after switching the clock source to HEXT
    ASSERT(init_result == SUCCESS);
#endif//_APP_DEBUG_
    PeriphTest();
    while(1) {
        // PeriphTest();
        AppIdleTask();
    }
}
