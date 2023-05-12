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

error_status ClockInit(void)
{
    uint32_t status;
    crm_sclk_type sclk_source;
    status = ARM_CRM_HEXT_PLL_SysClock240MHzConfig();
    ARM_CRM_BusClockConfig();
    sclk_source = ARM_CRM_GetClockSourceForSwitch();
    status |= ARM_CRM_SysClockSwitchCmd(sclk_source);
    system_core_clock_update();
//активация защиты системы тактирования: в случае сбоя в работе
//внешнего кварца HEXT происходит автоматическое переключение
//на внутренний RC-генератор HICK и возникнет прерывание NMI.
    crm_clock_failure_detection_enable(TRUE);
    return ARM_CRM_isReady(status) ? SUCCESS : ERROR;

}

void ClockFailureDetectHandler(void)
{
    uint32_t status;
    crm_sclk_type sclk_source;
    if(crm_flag_get(CRM_CLOCK_FAILURE_INT_FLAG) == SET) {
        crm_clock_failure_detection_enable(FALSE);
        ClockHEXTFailFlag = SET;
        status = ARM_CRM_HICK_PLL_SysClock240MHzConfig();
        ARM_CRM_BusClockConfig();
        sclk_source = ARM_CRM_GetClockSourceForSwitch();
        status |= ARM_CRM_SysClockSwitchCmd(sclk_source);
        system_core_clock_update();
        if(ARM_CRM_isReady(status)) {
            crm_flag_clear(CRM_CLOCK_FAILURE_INT_FLAG);
        }
    }

}

flag_status ClockTestHEXTFailFlag(void)
{
    return ClockHEXTFailFlag;
}

void ClockResetHEXTFailFlag(void)
{
    ClockHEXTFailFlag = RESET;
}

//================================================================================
//Private
//================================================================================

