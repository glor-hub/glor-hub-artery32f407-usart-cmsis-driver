//********************************************************************************
//clock.c
//********************************************************************************

#include "clock.h"
#include "arm_clock.h"

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

error_status ClockInit(void)
{
    uint32_t status, sys_clock;
    crm_sclk_type sclk_source;
    status = ARM_CRM_HEXT_PLL_SysClock240MHzConfig();
    ARM_CRM_BusClockConfig();
    sclk_source = ARM_CRM_GetClockSourceForSwitch();
    status |= ARM_CRM_SysClockSwitchCmd(sclk_source);
    sys_clock = system_core_clock;
    system_core_clock_update();
    sys_clock = system_core_clock;
    system_core_clock_update();
//активация защиты системы тактирования: в случае сбоя в работе
//внешнего кварца HEXT происходит автоматическое переключение
//на внутренний RC-генератор HICK и возникнет прерывание NMI.
    crm_clock_failure_detection_enable(TRUE);
    return ARM_CRM_isReady(status) ? SUCCESS : ERROR;

}

void ClockFailureDetectHandler(void)
{
    uint32_t status, sys_clock;
    crm_sclk_type sclk_source;
    if(crm_flag_get(CRM_CLOCK_FAILURE_INT_FLAG) != ERROR) {
        crm_clock_failure_detection_enable(FALSE);
        status = ARM_CRM_HICK_PLL_SysClock240MHzConfig();
        ARM_CRM_BusClockConfig();
        sclk_source = ARM_CRM_GetClockSourceForSwitch();
        status |= ARM_CRM_SysClockSwitchCmd(sclk_source);
        system_core_clock_update();
        sys_clock = system_core_clock;
        if(ARM_CRM_isReady(status)) {
            crm_flag_clear(CRM_CLOCK_FAILURE_INT_FLAG);
        }
    }
}


//================================================================================
//Private
//================================================================================

