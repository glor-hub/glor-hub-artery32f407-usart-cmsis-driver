//********************************************************************************
//test_periph.c
//********************************************************************************

#include "test_periph.h"
#include "led.h"
#include "clock.h"
#include "assert.h"
#include "timer.h"

//********************************************************************************
//Macros
//********************************************************************************

#define PERIPH_TEST_NUMS 1000

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

void PeriphTest(void)
{
    if(ClockTestHEXTFailFlag()) {
        LOG("HEXT Clock failure is detected. Switch of the system clock to the HICK clock is occured");
        ClockResetHEXTFailFlag();
    }
    LED_Test();
    LCD_Test();
}


//================================================================================
//Private
//================================================================================

