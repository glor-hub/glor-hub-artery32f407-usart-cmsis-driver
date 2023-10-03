//********************************************************************************
//clock.c
//********************************************************************************

#include "clock.h"
#include "arm_clock.h"
#include "assert.h"

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
static flag_status ClockHEXTFailFlag = RESET;

//********************************************************************************
//Prototypes
//********************************************************************************

//================================================================================
//Public
//================================================================================

error_status TEST_APP_ClockInit(void)
{
    uint32_t status;
    crm_sclk_type sclk_source;
    status = TEST_APP_ARM_CRM_HEXT_PLL_SysClock240MHzConfig();
    TEST_APP_ARM_CRM_BusClockConfig();
    sclk_source = TEST_APP_ARM_CRM_GetClockSourceForSwitch();
    status |= TEST_APP_ARM_CRM_SysClockSwitchCmd(sclk_source);
    system_core_clock_update();
//Enable clock failure detector: if a failure is detected on the HEXT clock
//system clock to the HICK clock to be switched of the , the
//CFD to be disabled , HEXT clock to be stopped, and even PLL to be disabled
//if the HEXT clock is selected as the system clock through PLL to the internal
//RC oscillator HICK and the NMI interrupt will occur.
    crm_clock_failure_detection_enable(TRUE);
    return TEST_APP_ARM_CRM_isReady(status) ? SUCCESS : ERROR;
}

void TEST_APP_ClockFailureDetectHandler(void)
{
    uint32_t status;
    crm_sclk_type sclk_source;
    if(crm_flag_get(CRM_CLOCK_FAILURE_INT_FLAG) == SET) {
        crm_clock_failure_detection_enable(FALSE);
        ClockHEXTFailFlag = SET;
        status = TEST_APP_ARM_CRM_HICK_PLL_SysClock240MHzConfig();
        TEST_APP_ARM_CRM_BusClockConfig();
        sclk_source = TEST_APP_ARM_CRM_GetClockSourceForSwitch();
        status |= TEST_APP_ARM_CRM_SysClockSwitchCmd(sclk_source);
        system_core_clock_update();
        if(TEST_APP_ARM_CRM_isReady(status)) {
            crm_flag_clear(CRM_CLOCK_FAILURE_INT_FLAG);
        }
    }

}

flag_status TEST_APP_ClockTestHEXTFailFlag(void)
{
    return ClockHEXTFailFlag;
}

void TEST_APP_ClockResetHEXTFailFlag(void)
{
    ClockHEXTFailFlag = RESET;
}

//================================================================================
//Private
//================================================================================

