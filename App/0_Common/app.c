//********************************************************************************
//app.c
//********************************************************************************

#include "app.h"
#include "clock.h"
#include "assert.h"
#include "timer.h"

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


//********************************************************************************
//Prototypes
//********************************************************************************

//================================================================================
//Public
//================================================================================

void AppIdleTask(void)
{
}

error_status AppInit(void)
{
    error_status init_result = ERROR;

#ifdef _APP_DEBUG_
    AssertConfig();
#endif//_APP_DEBUG_

    init_result |= ClockInit();
    init_result |= TimerInit();
    return init_result;
}

//================================================================================
//Private
//================================================================================
