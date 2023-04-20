#include "arm_clock.h"
//********************************************************************************
//arm_clock.c
//********************************************************************************

#include "arm_clock.h"

//********************************************************************************
//Macros
//********************************************************************************

#define ARM_CRM_STARTUP_TIMEOUT ((uint32_t)0x3000)

//статусы системы тактировани€
#define ARM_CRM_STA_READY                ((uint32_t)0UL)
//высокочастотный внешний тактовый генератор HEXT
#define ARM_CRM_STA_HEXT_READY_ERR        ((uint32_t)1UL << 0)
////высокочастотный внутренный RC-генератор HICK
#define ARM_CRM_STA_HICK_READY_ERR        ((uint32_t)1UL << 1)
#define ARM_CRM_STA_PLL_READY_ERR        ((uint32_t)1UL << 2)
#define ARM_CRM_STA_PLL_CLOCK_SWITCH_ERR ((uint32_t)1UL << 3)

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

static uint32_t ARM_CRM_ClockSourceReady(crm_clock_source_type source);

//================================================================================
//Public
//================================================================================

bool ARM_CRM_isReady(uint32_t status)
{
    return (status == ARM_CRM_STA_READY);
}

//конфигурирование системы тактировани€ с использованием внешнего
//кварцевого резонатора. ≈го частота определ€етс€ в init.h. »спользуем ‘јЅ„.
//настраиваем частоту тактировани€ дл€ резонатора (дл€ 8 ћ√ц - 8/2*60=240 ћ√ц)

uint32_t ARM_CRM_HEXT_PLL_ClockSourceConfig(void)
{
    uint32_t drv_status = ARM_CRM_STA_READY;
    crm_reset();
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    drv_status |= ARM_CRM_ClockSourceReady(CRM_CLOCK_SOURCE_HEXT);
    crm_pll_config(CRM_PLL_SOURCE_HEXT_DIV, CRM_PLL_MULT_60,
                   CRM_PLL_OUTPUT_RANGE_GT72MHZ);
    crm_hext_clock_div_set(CRM_HEXT_DIV_2);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    drv_status |= ARM_CRM_ClockSourceReady(CRM_CLOCK_SOURCE_PLL);
    return drv_status;
}

void ARM_CRM_BusClockConfig(void)
{
//240ћ√ц - шина AHB
    crm_ahb_div_set(CRM_AHB_DIV_1);
//120ћ√ц - шина APB1
    crm_apb1_div_set(CRM_APB1_DIV_2);
//120ћ√ц - шина APB2
    crm_apb2_div_set(CRM_APB2_DIV_2);
}

crm_sclk_type ARM_CRM_GetClockSourceForSwitch(void)
{
    return CRM_SCLK_PLL;
}

uint32_t ARM_CRM_SysClockSwitchCmd(crm_sclk_type value)
{
    uint32_t drv_status = ARM_CRM_STA_READY;
//включаем режим автоматического переключени€ частоты (дл€ частот
//выше 108 ћ√ц)
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(value);
//ждем переключени€ на источник тактировани€
    while(crm_sysclk_switch_status_get() != value);
    crm_auto_step_mode_enable(FALSE);
    return drv_status;
}


//включение/отключение тактировани€ периферии

void ARM_CRM_ClockPeriphEnable(crm_periph_clock_type value, confirm_state state)
{
    crm_periph_clock_enable(value, state);
}

//сброс периферии

void ARM_CRM_ClockPeriphReset(crm_periph_reset_type value, confirm_state state)
{
    crm_periph_reset(value, state);
}

//================================================================================
//Private
//================================================================================

static uint32_t ARM_CRM_ClockSourceReady(crm_clock_source_type source)
{
    uint32_t counter = 0;
    uint32_t drv_status = ARM_CRM_STA_READY;
    switch(source) {
        case CRM_CLOCK_SOURCE_HEXT: {
            while((crm_flag_get(CRM_HEXT_STABLE_FLAG) == ERROR) && (counter != ARM_CRM_STARTUP_TIMEOUT)) {
                counter++;
            }
            if(crm_flag_get(CRM_HEXT_STABLE_FLAG) == ERROR) {
                drv_status |= ARM_CRM_STA_HEXT_READY_ERR;
            } else {
                drv_status &= ARM_CRM_STA_HEXT_READY_ERR;
            }
            return drv_status;
            break;
        }
        case CRM_CLOCK_SOURCE_PLL: {
            while((crm_flag_get(CRM_PLL_STABLE_FLAG) == ERROR) && (counter != ARM_CRM_STARTUP_TIMEOUT)) {
                counter++;
            }
            if(crm_flag_get(CRM_PLL_STABLE_FLAG) == ERROR) {
                drv_status |= ARM_CRM_STA_PLL_READY_ERR;
            } else {
                drv_status &= ARM_CRM_STA_PLL_READY_ERR;
            }
            return drv_status;
            break;
        }
    }
    return drv_status;
}
