//********************************************************************************
//app.c
//********************************************************************************

#include "app.h"
#include "clock.h"
#include "assert.h"
#include "timer.h"
#include "LCD_2004.h"
#include "usart.h"

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
    USART_cb();
}

error_status AppInit(void)
{
    error_status init_result = SUCCESS;

#ifdef _APP_DEBUG_
    AssertConfig();
#endif//_APP_DEBUG_
    init_result &= ClockInit();
    TimerInit();
    LCD_Init();
    init_result &= USART_Init();
    return init_result;
}

//================================================================================
//Private
//================================================================================
