#ifndef _ARM_TIMER_H_
#define _ARM_TIMER_H_

#include "at32f403a_407.h"

//Driver supports basic Timer6, Timer7.

/*******************************************
select TIMERx
********************************************/
// #define _TEST_APP_TIMER1_ADVANCED_ENABLED_  (0)
// #define _TEST_APP_TIMER2_GENERAL_ENABLED_   (0)
// #define _TEST_APP_TIMER3_GENERAL_ENABLED_   (0)
// #define _TEST_APP_TIMER4_GENERAL_ENABLED_   (0)
// #define _TEST_APP_TIMER5_GENERAL_ENABLED_   (0)
#define _TEST_APP_TIMER6_BASIC_ENABLED_     (1)
#define _TEST_APP_TIMER7_BASIC_ENABLED_     (1)
// #define _TEST_APP_TIMER8_ADVANCED_ENABLED_  (0)
// #define _TEST_APP_TIMER9_GENERAL_ENABLED_   (0)
// #define _TEST_APP_TIMER10_GENERAL_ENABLED_  (0)
// #define _TEST_APP_TIMER11_GENERAL_ENABLED_  (0)
// #define _TEST_APP_TIMER12_GENERAL_ENABLED_  (0)
// #define _TEST_APP_TIMER13_GENERAL_ENABLED_  (0)
// #define _TEST_APP_TIMER14_GENERAL_ENABLED_  (0)


typedef enum {
    TEST_APP_ARM_TIMER6_BASIC = 0,
    TEST_APP_ARM_TIMER7_BASIC,
    // TEST_APP_ARM_TIMER1_ADVANCED,
    // TEST_APP_ARM_TIMER2_GENERAL,
    // TEST_APP_ARM_TIMER3_GENERAL,
    // TEST_APP_ARM_TIMER4_GENERAL,
    // TEST_APP_ARM_TIMER5_GENERAL,
    // TEST_APP_ARM_TIMER8_ADVANCED,
    // TEST_APP_ARM_TIMER9_GENERAL_,
    // TEST_APP_ARM_TIMER10_GENERAL,
    // TEST_APP_ARM_TIMER11_GENERAL,
    // TEST_APP_ARM_TIMER12_GENERAL,
    // TEST_APP_ARM_TIMER13_GENERAL,
    // TEST_APP_ARM_TIMER14_GENERAL,
    TEST_APP_ARM_TIMER_TYPES
} eTEST_APP_ARM_TIMER_Types_t;

#define TEST_APP_TIMER6_TIMER7_1MHZ_DIV_SCALER 239


// TIMER Driver state
#define TEST_APP_ARM_TIMER_DRIVER_FLAG_INITIALIZED       (uint32_t)(1U << 0)
#define TEST_APP_ARM_TIMER_DRIVER_FLAG_ENABLED           (uint32_t)(1U << 1)

//initialization parameters
#define TEST_APP_ARM_TIMER_ONE_CIRCLE_MODE_ENABLE TRUE
#define TEST_APP_ARM_TIMER_ONE_CIRCLE_MODE_DISABLE FALSE

#define TEST_APP_ARM_TIMER_PERIOD_BUFFER_ENABLE TRUE
#define TEST_APP_ARM_TIMER_PERIOD_BUFFER_DISABLE FALSE

//overflow interrupt flag sources
typedef enum {
    TEST_APP_ARM_TIMER_COUNTER_OVERFLOW_EVENT_SOURCE = 0,
    TEST_APP_ARM_TIMER_ANY_OVERFLOW_EVENT_SOURCE, //counter,software or slave timer
    TEST_APP_ARM_TIMER_OVERFLOW_EVENT_SOURCE_TYPES
} eTEST_APP_ARM_TIMER_OverflowEventSources_t;

typedef struct {
    confirm_state DrvStateOn;
    uint32_t DrvStatus;
    uint32_t DrvFlag;
} TEST_APP_ARM_TIMER_Status_t;

typedef struct {
    confirm_state PeriodBuffEnable;
    confirm_state OneCircleModeEnable;
    eTEST_APP_ARM_TIMER_OverflowEventSources_t OverflowEventSources;
} TEST_APP_ARM_TIMER_Config_t;

typedef struct {
    IRQn_Type                           IrqNum;     // TIMER IRQ Number
    TEST_APP_ARM_TIMER_Config_t         Config;
    TEST_APP_ARM_TIMER_Status_t         Status;
} TEST_APP_ARM_TIMER_Resources_t;

typedef struct {
    uint32_t (*Initialize)(confirm_state period_buff_en,
                           confirm_state one_circle_mode_en,
                           eTEST_APP_ARM_TIMER_OverflowEventSources_t overflow_event_sources);
    void (*Uninitialize)(void);
    void (*Config)(uint32_t div_prescaler, uint32_t auto_reload_value);
    void (*TimerEnable)(confirm_state new_state);
    void (*InterruptEnable)(uint32_t tmr_interrupt, confirm_state new_state);
    flag_status(*GetInterruptFlag)(uint32_t tmr_flag);
    void (*ClearInterruptFlag)(uint32_t tmr_flag);
    void (*Event_cb)(void);
    TEST_APP_ARM_TIMER_Status_t (*GetStatus)(void);
} TEST_APP_ARM_TIMER_Driver_t;

void TEST_APP_ARM_TIMER_StartUp(void);
void TEST_APP_ARM_TIMER_TimerEnable(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                    confirm_state new_state);

#endif //_ARM_TIMER_H_
