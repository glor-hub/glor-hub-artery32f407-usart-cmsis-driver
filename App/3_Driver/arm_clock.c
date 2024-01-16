//********************************************************************************
//arm_clock.c
//********************************************************************************

#include "arm_clock.h"
#include "arm_dma.h"
#include "arm_gpio.h"
#include "arm_usart.h"
#include "arm_spi.h"

//********************************************************************************
//Macros
//********************************************************************************

#define ARM_CRM_STARTUP_TIMEOUT ((uint32_t)0x3000)

//clock system status

#define ARM_CRM_STA_NO_ERR                  ((uint32_t)0UL)
//high speed external clock oscillator HEXT
#define ARM_CRM_STA_HEXT_READY_ERR          ((uint32_t)1UL << 0)
//high speed internal RC-oscillator HICK
#define ARM_CRM_STA_HICK_READY_ERR          ((uint32_t)1UL << 1)
#define ARM_CRM_STA_PLL_READY_ERR           ((uint32_t)1UL << 2)
#define ARM_CRM_STA_PLL_CLOCK_SWITCH_ERR    ((uint32_t)1UL << 3)
#define ARM_CRM_STA_PERIPH_CLOCK_ERR        ((uint32_t)1UL << 4)
#define ARM_CRM_STA_PERIPH_RESET_ERR        ((uint32_t)1UL << 5)

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Variables
//********************************************************************************

//Enable/disable peripherals
static crm_periph_clock_type ARM_CRM_DMA_PeriphClockType[TEST_APP_ARM_DMA_CHANS] = {
    CRM_DMA1_PERIPH_CLOCK,
    CRM_DMA1_PERIPH_CLOCK,
    CRM_DMA1_PERIPH_CLOCK,
    CRM_DMA1_PERIPH_CLOCK,
    CRM_DMA1_PERIPH_CLOCK,
    CRM_DMA1_PERIPH_CLOCK,
    CRM_DMA1_PERIPH_CLOCK,
    CRM_DMA2_PERIPH_CLOCK,
    CRM_DMA2_PERIPH_CLOCK,
    CRM_DMA2_PERIPH_CLOCK,
    CRM_DMA2_PERIPH_CLOCK,
    CRM_DMA2_PERIPH_CLOCK,
    CRM_DMA2_PERIPH_CLOCK,
    CRM_DMA2_PERIPH_CLOCK
};

static crm_periph_clock_type ARM_CRM_GPIO_PeriphClockType[TEST_APP_ARM_GPIO_PORTS] = {
    CRM_GPIOA_PERIPH_CLOCK,
    CRM_GPIOB_PERIPH_CLOCK,
    CRM_GPIOC_PERIPH_CLOCK,
    CRM_GPIOD_PERIPH_CLOCK,
    CRM_GPIOE_PERIPH_CLOCK
};

static crm_periph_clock_type ARM_CRM_SPI_PeriphClockType[TEST_APP_ARM_SPI_TYPES] = {
    CRM_SPI1_PERIPH_CLOCK,
    CRM_SPI2_PERIPH_CLOCK,
    CRM_SPI3_PERIPH_CLOCK,
    CRM_SPI4_PERIPH_CLOCK
};

static crm_periph_clock_type ARM_CRM_USART_PeriphClockType[TEST_APP_ARM_USART_TYPES] = {
    CRM_USART1_PERIPH_CLOCK,
    CRM_USART2_PERIPH_CLOCK,
    CRM_USART3_PERIPH_CLOCK,
    CRM_UART4_PERIPH_CLOCK,
    CRM_UART5_PERIPH_CLOCK,
    CRM_USART6_PERIPH_CLOCK,
    CRM_UART7_PERIPH_CLOCK,
    CRM_UART8_PERIPH_CLOCK
};

static crm_periph_reset_type ARM_CRM_GPIO_PeriphResetType[TEST_APP_ARM_GPIO_PORTS] = {
    CRM_GPIOA_PERIPH_RESET,
    CRM_GPIOB_PERIPH_RESET,
    CRM_GPIOC_PERIPH_RESET,
    CRM_GPIOD_PERIPH_RESET,
    CRM_GPIOE_PERIPH_RESET
};

static crm_periph_reset_type ARM_CRM_SPI_PeriphResetType[TEST_APP_ARM_SPI_TYPES] = {
    CRM_SPI1_PERIPH_RESET,
    CRM_SPI2_PERIPH_RESET,
    CRM_SPI3_PERIPH_RESET,
    CRM_SPI4_PERIPH_RESET
};

static crm_periph_reset_type ARM_CRM_USART_PeriphResetType[TEST_APP_ARM_USART_TYPES] = {
    CRM_USART1_PERIPH_RESET,
    CRM_USART2_PERIPH_RESET,
    CRM_USART3_PERIPH_RESET,
    CRM_UART4_PERIPH_RESET,
    CRM_UART5_PERIPH_RESET,
    CRM_USART6_PERIPH_RESET,
    CRM_UART7_PERIPH_RESET,
    CRM_UART8_PERIPH_RESET
};

//********************************************************************************
//Prototypes
//********************************************************************************

static uint32_t ARM_CRM_ClockSourceReady(crm_clock_source_type source);

//================================================================================
//Public
//================================================================================

bool TEST_APP_ARM_CRM_isReady(uint32_t status)
{
    return (status == ARM_CRM_STA_NO_ERR);
}

//Configure a clock system using an external quartz resonator.
//Its frequency is defined in init.h. Use PLL.
//Customize the clock frequency for 8 MHz resonator (8/2*60=240 MHz)

uint32_t TEST_APP_ARM_CRM_HEXT_PLL_SysClock240MHzConfig(void)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
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

//Configure a clock system using an internal RC-oscillator 8 MHz.
//Use PLL.
//Customize the clock frequency (8/2*60=240 MHz)
uint32_t TEST_APP_ARM_CRM_HICK_PLL_SysClock240MHzConfig(void)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
    crm_reset();
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    drv_status |= ARM_CRM_ClockSourceReady(CRM_CLOCK_SOURCE_HICK);
    crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_60,
                   CRM_PLL_OUTPUT_RANGE_GT72MHZ);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    drv_status |= ARM_CRM_ClockSourceReady(CRM_CLOCK_SOURCE_PLL);
    return drv_status;
}

void TEST_APP_ARM_CRM_BusClockConfig(void)
{
//240 MHz - AHB bus
    crm_ahb_div_set(CRM_AHB_DIV_1);
//120 MHz - APB1 bus
    crm_apb1_div_set(CRM_APB1_DIV_2);
//120 MHz - APB2 bus
    crm_apb2_div_set(CRM_APB2_DIV_2);
}

crm_sclk_type TEST_APP_ARM_CRM_GetClockSourceForSwitch(void)
{
    return CRM_SCLK_PLL;
}

uint32_t TEST_APP_ARM_CRM_SysClockSwitchCmd(crm_sclk_type value)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
//Enable auto step-by-step system clock switch ( for frequencies - larger than 108 MHz)
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(value);
//Waiting for switch to the clock source
    while(crm_sysclk_switch_status_get() != value);
    crm_auto_step_mode_enable(FALSE);
    return drv_status;
}

//Enable/disable peripheral clock

bool TEST_APP_ARM_CRM_PeriphClockEnable(eTEST_APP_Periph_Types_t periph, uint8_t param, confirm_state new_state)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
    crm_periph_clock_type clock_type;
    switch(periph) {
        case TEST_APP_PERIPH_GPIO: {
            if((param < TEST_APP_ARM_GPIO_PORTA) || (param > TEST_APP_ARM_GPIO_PORTE)) {
#ifdef _TEST_APP_DEBUG_
                LOG("GPIO clock error");
#endif//_TEST_APP_DEBUG_
                drv_status |= ARM_CRM_STA_PERIPH_CLOCK_ERR;
            } else {
                clock_type = ARM_CRM_GPIO_PeriphClockType[param];
            }
            break;
        }
        case TEST_APP_PERIPH_USART: {
            if((param < TEST_APP_ARM_USART1) || (param > TEST_APP_ARM_UART8)) {
#ifdef _TEST_APP_DEBUG_
                LOG("USART clock error");
#endif//_TEST_APP_DEBUG_
                drv_status |= ARM_CRM_STA_PERIPH_CLOCK_ERR;
            } else {
                clock_type = ARM_CRM_USART_PeriphClockType[param];
            }
            break;
        }
        case TEST_APP_PERIPH_DMA: {
            if((param < TEST_APP_ARM_DMA1_CHAN1) || (param > TEST_APP_ARM_DMA2_CHAN7)) {
#ifdef _TEST_APP_DEBUG_
                LOG("DMA clock error");
#endif//_TEST_APP_DEBUG_
                drv_status |= ARM_CRM_STA_PERIPH_CLOCK_ERR;
            } else {
                clock_type = ARM_CRM_DMA_PeriphClockType[param];
            }
            break;
        }
        case TEST_APP_PERIPH_SPI: {
            if((param < TEST_APP_ARM_SPI1) || (param > TEST_APP_ARM_SPI4)) {
#ifdef _TEST_APP_DEBUG_
                LOG("SPI clock error");
#endif//_TEST_APP_DEBUG_
                drv_status |= ARM_CRM_STA_PERIPH_CLOCK_ERR;
            } else {
                clock_type = ARM_CRM_SPI_PeriphClockType[param];
            }
            break;
        }
        default:
            break;
    }
    crm_periph_clock_enable(clock_type, new_state);
    return TEST_APP_ARM_CRM_isReady(drv_status);
}

//peripheral reset

bool TEST_APP_ARM_CRM_PeriphReset(eTEST_APP_Periph_Types_t periph, uint8_t param, confirm_state new_state)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
    crm_periph_reset_type reset_type;
    switch(periph) {
        case TEST_APP_PERIPH_GPIO: {
            if((param < TEST_APP_ARM_GPIO_PORTA) || (param > TEST_APP_ARM_GPIO_PORTE)) {
#ifdef _TEST_APP_DEBUG_
                LOG("GPIO reset error");
#endif//_TEST_APP_DEBUG_
                drv_status |= ARM_CRM_STA_PERIPH_RESET_ERR;
            } else {
                reset_type = ARM_CRM_GPIO_PeriphResetType[param];
            }
            break;
        }
        case TEST_APP_PERIPH_USART: {
            if((param < TEST_APP_ARM_USART1) || (param > TEST_APP_ARM_UART8)) {
#ifdef _TEST_APP_DEBUG_
                LOG("USART reset error");
#endif//_TEST_APP_DEBUG_
                drv_status |= ARM_CRM_STA_PERIPH_RESET_ERR;
            } else {
                reset_type = ARM_CRM_USART_PeriphResetType[param];
            }
            break;
        }
        case TEST_APP_PERIPH_SPI: {
            if((param < TEST_APP_ARM_SPI1) || (param > TEST_APP_ARM_SPI4)) {
#ifdef _TEST_APP_DEBUG_
                LOG("SPI reset error");
#endif//_TEST_APP_DEBUG_
                drv_status |= ARM_CRM_STA_PERIPH_RESET_ERR;
            } else {
                reset_type = ARM_CRM_SPI_PeriphResetType[param];
            }
            break;
        }
        default:
            break;
    }
    crm_periph_reset(reset_type, new_state);
    return TEST_APP_ARM_CRM_isReady(drv_status);
}

//================================================================================
//Private
//================================================================================

static uint32_t ARM_CRM_ClockSourceReady(crm_clock_source_type source)
{
    uint32_t counter = 0;
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
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
            break;
        }
        case CRM_CLOCK_SOURCE_HICK: {
            while((crm_flag_get(CRM_HICK_STABLE_FLAG) == ERROR) && (counter != ARM_CRM_STARTUP_TIMEOUT)) {
                counter++;
            }
            if(crm_flag_get(CRM_HICK_STABLE_FLAG) == ERROR) {
                drv_status |= ARM_CRM_STA_HICK_READY_ERR;
            } else {
                drv_status &= ARM_CRM_STA_HICK_READY_ERR;
            }
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
            break;
        }
        default: {
            break;
        }
    }
    return drv_status;
}
