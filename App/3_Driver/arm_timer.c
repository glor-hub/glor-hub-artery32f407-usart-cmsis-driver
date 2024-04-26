//********************************************************************************
//arm_timer.c
//********************************************************************************
#include <string.h>
#include "arm_driver.h"
#include "arm_timer.h"
#include "arm_clock.h"

//********************************************************************************
//Macros
//********************************************************************************

#define ARM_TIMER_EVENT_BUFF_SIZE 8

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Prototypes
//********************************************************************************

static void ARM_TIMER_SetResources(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                   confirm_state period_buff_en,
                                   confirm_state one_circle_mode_en,
                                   eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources);

static uint32_t ARM_TIMER_Initialize_6(confirm_state period_buff_en,
                                       confirm_state one_circle_mode_en,
                                       eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources);
static void ARM_TIMER_Uninitialize_6(void);
static void ARM_TIMER_Config_6(uint32_t div_prescaler, uint32_t auto_reload_value);
static void ARM_TIMER_TimerEnable_6(confirm_state new_state);
static void ARM_TIMER_InterruptEnable_6(uint32_t tmr_interrupt, confirm_state new_state);
static flag_status ARM_TIMER_GetInterruptFlag_6(uint32_t tmr_flag);
static void ARM_TIMER_ClearInterruptFlag_6(uint32_t tmr_flag);
static void ARM_TIMER_Event_cb_6(void);
static TEST_APP_ARM_TIMER_Status_t ARM_TIMER_GetStatus_6(void);

static uint32_t ARM_TIMER_Initialize_7(confirm_state period_buff_en,
                                       confirm_state one_circle_mode_en,
                                       eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources);
static void ARM_TIMER_Uninitialize_7(void);
static void ARM_TIMER_Config_7(uint32_t div_prescaler, uint32_t auto_reload_value);
static void ARM_TIMER_TimerEnable_7(confirm_state new_state);
static void ARM_TIMER_InterruptEnable_7(uint32_t tmr_interrupt, confirm_state new_state);
static flag_status ARM_TIMER_GetInterruptFlag_7(uint32_t tmr_flag);
static void ARM_TIMER_ClearInterruptFlag_7(uint32_t tmr_flag);
static void ARM_TIMER_Event_cb_7(void);
static TEST_APP_ARM_TIMER_Status_t ARM_TIMER_GetStatus_7(void);

static uint32_t ARM_TIMER_Initialize(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                     confirm_state period_buff_en,
                                     confirm_state one_circle_mode_en,
                                     eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources);

static uint32_t ARM_TIMER_Uninitialize(eTEST_APP_ARM_TIMER_Types_t timer_type);

static void ARM_TIMER_Config(eTEST_APP_ARM_TIMER_Types_t timer_type, uint32_t div_prescaler,
                             uint32_t auto_reload_value);

static void ARM_TIMER_TimerEnable(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                  confirm_state new_state);

static void ARM_TIMER_InterruptEnable(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                      uint32_t tmr_interrupt, confirm_state new_state);

static flag_status ARM_TIMER_GetInterruptFlag(eTEST_APP_ARM_TIMER_Types_t timer_type,
        uint32_t tmr_flag);

static void ARM_TIMER_ClearInterruptFlag(eTEST_APP_ARM_TIMER_Types_t timer_type,
        uint32_t tmr_flag);

static void ARM_TIMER_Event_cb(eTEST_APP_ARM_TIMER_Types_t timer_type);

static TEST_APP_ARM_TIMER_Status_t ARM_TIMER_GetStatus(eTEST_APP_ARM_TIMER_Types_t timer_type);


//********************************************************************************
//Variables
//********************************************************************************

TEST_APP_ARM_TIMER_Driver_t Driver_TIMER6 = {
    ARM_TIMER_Initialize_6,
    ARM_TIMER_Uninitialize_6,
    ARM_TIMER_Config_6,
    ARM_TIMER_TimerEnable_6,
    ARM_TIMER_InterruptEnable_6,
    ARM_TIMER_GetInterruptFlag_6,
    ARM_TIMER_ClearInterruptFlag_6,
    ARM_TIMER_Event_cb_6,
    ARM_TIMER_GetStatus_6
};

TEST_APP_ARM_TIMER_Driver_t Driver_TIMER7 = {
    ARM_TIMER_Initialize_7,
    ARM_TIMER_Uninitialize_7,
    ARM_TIMER_Config_7,
    ARM_TIMER_TimerEnable_7,
    ARM_TIMER_InterruptEnable_7,
    ARM_TIMER_GetInterruptFlag_7,
    ARM_TIMER_ClearInterruptFlag_7,
    ARM_TIMER_Event_cb_7,
    ARM_TIMER_GetStatus_7
};

TEST_APP_ARM_TIMER_Driver_t *pARM_TIMER_Driver[TEST_APP_ARM_TIMER_TYPES] = {
    &Driver_TIMER6,
    &Driver_TIMER7
};

static TEST_APP_ARM_TIMER_Resources_t ARM_TIMER_Resources[TEST_APP_ARM_TIMER_TYPES];
static uint32_t ARM_TIMER_EventBuff[TEST_APP_ARM_TIMER_TYPES][ARM_TIMER_EVENT_BUFF_SIZE];

static tmr_type *pARM_TIMER_Register[TEST_APP_ARM_TIMER_TYPES] = {
    TMR6,
    TMR7
};

static IRQn_Type ARM_TIMER_IrqNumber[TEST_APP_ARM_TIMER_TYPES] = {
    TMR6_GLOBAL_IRQn,
    TMR7_GLOBAL_IRQn
};

//================================================================================
//Public
//================================================================================

void TEST_APP_ARM_TIMER_StartUp(void)
{
    TEST_APP_ARM_TIMER_Resources_t *p_res = &ARM_TIMER_Resources[TEST_APP_ARM_TIMER6_BASIC];
#if _TEST_APP_TIMER6_BASIC_ENABLED_ > 0
    (p_res[TEST_APP_ARM_TIMER6_BASIC]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_TIMER6_BASIC]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_TIMER6_BASIC]).Status.DrvFlag = 0x0000;
#endif //_TEST_APP_TIMER6_BASIC_ENABLED_ > 0

#if _TEST_APP_TIMER7_BASIC_ENABLED_ > 0
    (p_res[TEST_APP_ARM_TIMER7_BASIC]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_TIMER7_BASIC]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_TIMER7_BASIC]).Status.DrvFlag = 0x0000;
#endif //_TEST_APP_TIMER7_BASIC_ENABLED_ > 0
}

//================================================================================
//Private
//================================================================================

static uint32_t ARM_TIMER_Initialize_6(confirm_state period_buff_en,
                                       confirm_state one_circle_mode_en,
                                       eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources)
{
    return ARM_TIMER_Initialize(TEST_APP_ARM_TIMER6_BASIC, period_buff_en, one_circle_mode_en,
                                overflow_event_sources);
}

static void ARM_TIMER_Uninitialize_6(void)
{
    ARM_TIMER_Uninitialize(TEST_APP_ARM_TIMER6_BASIC);
}

static void ARM_TIMER_Config_6(uint32_t div_prescaler, uint32_t auto_reload_value)
{
    ARM_TIMER_Config(TEST_APP_ARM_TIMER6_BASIC, div_prescaler,
                     auto_reload_value);
}

static void ARM_TIMER_TimerEnable_6(confirm_state new_state)
{
    ARM_TIMER_TimerEnable(TEST_APP_ARM_TIMER6_BASIC, new_state);
}
static void ARM_TIMER_InterruptEnable_6(uint32_t tmr_interrupt, confirm_state new_state)
{
    ARM_TIMER_InterruptEnable(TEST_APP_ARM_TIMER6_BASIC, tmr_interrupt, new_state);
}
static flag_status ARM_TIMER_GetInterruptFlag_6(uint32_t tmr_flag)
{
    return ARM_TIMER_GetInterruptFlag(TEST_APP_ARM_TIMER6_BASIC, tmr_flag);
}
static void ARM_TIMER_ClearInterruptFlag_6(uint32_t tmr_flag)
{
    ARM_TIMER_ClearInterruptFlag(TEST_APP_ARM_TIMER6_BASIC, tmr_flag);
}
static void ARM_TIMER_Event_cb_6(void)
{
    ARM_TIMER_Event_cb(TEST_APP_ARM_TIMER6_BASIC);
}

static TEST_APP_ARM_TIMER_Status_t ARM_TIMER_GetStatus_6(void)
{
    return ARM_TIMER_GetStatus(TEST_APP_ARM_TIMER6_BASIC);
}

static uint32_t ARM_TIMER_Initialize_7(confirm_state period_buff_en,
                                       confirm_state one_circle_mode_en,
                                       eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources)
{
    return ARM_TIMER_Initialize(TEST_APP_ARM_TIMER7_BASIC, period_buff_en, one_circle_mode_en,
                                overflow_event_sources);
}

static void ARM_TIMER_Uninitialize_7(void)
{
    ARM_TIMER_Uninitialize(TEST_APP_ARM_TIMER7_BASIC);
}

static void ARM_TIMER_Config_7(uint32_t div_prescaler, uint32_t auto_reload_value)
{
    ARM_TIMER_Config(TEST_APP_ARM_TIMER7_BASIC, div_prescaler,
                     auto_reload_value);
}

static void ARM_TIMER_TimerEnable_7(confirm_state new_state)
{
    ARM_TIMER_TimerEnable(TEST_APP_ARM_TIMER7_BASIC, new_state);
}
static void ARM_TIMER_InterruptEnable_7(uint32_t tmr_interrupt, confirm_state new_state)
{
    ARM_TIMER_InterruptEnable(TEST_APP_ARM_TIMER7_BASIC, tmr_interrupt, new_state);
}
static flag_status ARM_TIMER_GetInterruptFlag_7(uint32_t tmr_flag)
{
    return ARM_TIMER_GetInterruptFlag(TEST_APP_ARM_TIMER7_BASIC, tmr_flag);
}
static void ARM_TIMER_ClearInterruptFlag_7(uint32_t tmr_flag)
{
    ARM_TIMER_ClearInterruptFlag(TEST_APP_ARM_TIMER7_BASIC, tmr_flag);
}
static void ARM_TIMER_Event_cb_7(void)
{
    ARM_TIMER_Event_cb(TEST_APP_ARM_TIMER7_BASIC);
}

static TEST_APP_ARM_TIMER_Status_t ARM_TIMER_GetStatus_7(void)
{
    return ARM_TIMER_GetStatus(TEST_APP_ARM_TIMER7_BASIC);
}

static void ARM_TIMER_SetResources(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                   confirm_state period_buff_en,
                                   confirm_state one_circle_mode_en,
                                   eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources)
{
    TEST_APP_ARM_TIMER_Resources_t *p_res = &ARM_TIMER_Resources[timer_type];
    p_res->IrqNum = ARM_TIMER_IrqNumber[timer_type];
    p_res->Config.PeriodBuffEnable = period_buff_en;
    p_res->Config.OneCircleModeEnable = one_circle_mode_en;
    p_res->Config.OverflowEventSources = overflow_event_sources;
}

static uint32_t ARM_TIMER_Initialize(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                     confirm_state period_buff_en,
                                     confirm_state one_circle_mode_en,
                                     eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources)
{
    TEST_APP_ARM_TIMER_Resources_t *p_res = &ARM_TIMER_Resources[timer_type];
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    ARM_TIMER_SetResources(timer_type, period_buff_en, one_circle_mode_en,
                           overflow_event_sources);
    if(!(TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_TIMER, timer_type, TRUE))) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_TIMER, timer_type, TRUE);
    TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_TIMER, timer_type, FALSE);
//clear and enable TIMERx IRQ
    NVIC_ClearPendingIRQ(p_res->IrqNum);
    NVIC_EnableIRQ(p_res->IrqNum);
    tmr_interrupt_enable(pARM_TIMER_Register[timer_type], TMR_OVF_FLAG, FALSE);
    p_res->Status.DrvFlag |= TEST_APP_ARM_TIMER_DRIVER_FLAG_INITIALIZED;
    p_res->Status.DrvStatus |= drv_status;
    return drv_status;
}

static uint32_t ARM_TIMER_Uninitialize(eTEST_APP_ARM_TIMER_Types_t timer_type)
{
    TEST_APP_ARM_TIMER_Resources_t *p_res = &ARM_TIMER_Resources[timer_type];
    if(p_res->Status.DrvFlag & TEST_APP_ARM_TIMER_DRIVER_FLAG_INITIALIZED) {
        //disable and clear TIMERx IRQ
        NVIC_DisableIRQ(p_res->IrqNum);
        NVIC_ClearPendingIRQ(p_res->IrqNum);
        TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_TIMER, timer_type, TRUE);
        TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_TIMER, timer_type, FALSE);
        if(!(TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_TIMER, timer_type, FALSE))) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        memset(p_res, 0, sizeof(TEST_APP_ARM_TIMER_Resources_t));
        p_res->Status.DrvFlag &= ~TEST_APP_ARM_TIMER_DRIVER_FLAG_INITIALIZED;
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("TIMER not been initialized");
#endif//_TEST_APP_DEBUG_
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

static void ARM_TIMER_Config(eTEST_APP_ARM_TIMER_Types_t timer_type, uint32_t div_prescaler,
                             uint32_t auto_reload_value)
{
    tmr_base_init(pARM_TIMER_Register[timer_type], div_prescaler, auto_reload_value);
}

static void ARM_TIMER_TimerEnable(eTEST_APP_ARM_TIMER_Types_t timer_type, confirm_state new_state)
{
    tmr_counter_enable(pARM_TIMER_Register[timer_type], new_state);
}

static void ARM_TIMER_InterruptEnable(eTEST_APP_ARM_TIMER_Types_t timer_type, uint32_t tmr_interrupt,
                                      confirm_state new_state)
{
    tmr_interrupt_enable(pARM_TIMER_Register[timer_type], tmr_interrupt, new_state);
}

static flag_status ARM_TIMER_GetInterruptFlag(eTEST_APP_ARM_TIMER_Types_t timer_type,
        uint32_t tmr_flag)
{
    return tmr_flag_get(pARM_TIMER_Register[timer_type], tmr_flag);
}

static void ARM_TIMER_ClearInterruptFlag(eTEST_APP_ARM_TIMER_Types_t timer_type,
        uint32_t tmr_flag)
{
    tmr_flag_clear(pARM_TIMER_Register[timer_type], tmr_flag);
}

static void ARM_TIMER_Event_cb(eTEST_APP_ARM_TIMER_Types_t timer_type)
{

};

static TEST_APP_ARM_TIMER_Status_t ARM_TIMER_GetStatus(eTEST_APP_ARM_TIMER_Types_t timer_type)
{
    TEST_APP_ARM_TIMER_Resources_t *p_res = &ARM_TIMER_Resources[timer_type];
    TEST_APP_ARM_TIMER_Status_t status;
    status.DrvStateOn = p_res->Status.DrvStateOn;
    status.DrvFlag = p_res->Status.DrvFlag;
    status.DrvStatus = p_res->Status.DrvStatus;
    return status;
}
