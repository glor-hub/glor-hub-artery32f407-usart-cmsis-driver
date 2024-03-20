//********************************************************************************
//timer.c


//********************************************************************************
#include "at32f403a_407.h"
// #include "app.h"
#include "arm_driver.h"
#include "arm_timer.h"
#include "software_timer.h"
#include "timer.h"

extern TEST_APP_ARM_TIMER_Driver_t *pARM_TIMER_Driver[TEST_APP_ARM_TIMER_TYPES];

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

/*********************************************************************************
TIMER configuration:
one_circle_mode_en (TRUE/FALSE) - TRUE: timer count up to value in the PR register
and stops

period_buff_en (TRUE/FALSE) - TRUE: the values in the TMRx_PR and TMRx_DIV registers
are transferred to the their shadow registers only at overflow event

overflow_event_source (for flag and interrupt):
   - TEST_APP_ARM_TIMER_COUNTER_OVERFLOW_EVENT_SOURCE - only counter overflow
   - TEST_APP_ANY_OVERFLOW_EVENT_SOURCE - counter overflow, setting the OVFSWTR bit,
                                        overflow event from the slave controller

div_prescaler - prescaler division 0...65535

auto_reload_value(PR) - auto-reload register value 0...65535

F_overflow_event=F_clk/((1+div_prescaler)*(1+PR))

 ********************************************************************************/


error_status TEST_APP_TIMER_TimerInit(void)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    eTEST_APP_ARM_TIMER_Types_t timer_type;
    TEST_APP_ARM_TIMER_Driver_t *p_drv = pARM_TIMER_Driver[TEST_APP_ARM_TIMER6_BASIC];
    for(timer_type = TEST_APP_ARM_TIMER6_BASIC; timer_type < TEST_APP_ARM_TIMER_TYPES;
        timer_type++) {
        if(p_drv[timer_type].GetStatus().DrvStateOn) {
            drv_status |= p_drv[timer_type].Initialize(TEST_APP_ARM_TIMER_ONE_CIRCLE_MODE_ENABLE,
                          TEST_APP_ARM_TIMER_PERIOD_BUFFER_ENABLE,
                          TEST_APP_ARM_TIMER_COUNTER_OVERFLOW_EVENT_SOURCE);
        }
    }
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}

void TEST_APP_ARM_TIMER_DoDelay_usec(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                     uint8_t num_useconds)
{
    TEST_APP_ARM_TIMER_Driver_t *p_drv = pARM_TIMER_Driver[timer_type];

    p_drv->Config(TEST_APP_TIMER6_TIMER7_1MHZ_DIV_SCALER, num_useconds);
    p_drv->TimerEnable(TRUE);
    while(p_drv->GetInterruptFlag(TMR_OVF_FLAG) != SET);
    p_drv->ClearInterruptFlag(TMR_OVF_FLAG);
    p_drv->TimerEnable(FALSE);

}

//================================================================================
//Private
//================================================================================
