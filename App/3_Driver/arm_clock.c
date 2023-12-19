//********************************************************************************
//arm_clock.c
//********************************************************************************

#include "arm_clock.h"

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

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Variables
//********************************************************************************

static crm_periph_clock_type ARM_CRM_DMA_ClockType[TEST_APP_ARM_DMA_CHANS] = {
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

bool TEST_APP_ARM_CRM_GPIO_ClockEnable(gpio_type *pGPIO_x, confirm_state new_state)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
    if(pGPIO_x == GPIOA) {
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, new_state);
    } else if(pGPIO_x == GPIOB) {
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, new_state);
    } else if(pGPIO_x == GPIOC) {
        crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, new_state);
    } else if(pGPIO_x == GPIOD) {
        crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, new_state);
    } else if(pGPIO_x == GPIOE) {
        crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, new_state);
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("GPIO clock error");
#endif//_TEST_APP_DEBUG_
        drv_status |= ARM_CRM_STA_PERIPH_CLOCK_ERR;
    }
    return TEST_APP_ARM_CRM_isReady(drv_status);
}

bool TEST_APP_ARM_CRM_USART_ClockEnable(usart_type *pUSART_x, confirm_state new_state)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
    if(pUSART_x == UART4) {
        crm_periph_clock_enable(CRM_UART4_PERIPH_CLOCK, new_state);
    } else if(pUSART_x == UART5) {
        crm_periph_clock_enable(CRM_UART5_PERIPH_CLOCK, new_state);
    } else if(pUSART_x == UART7) {
        crm_periph_clock_enable(CRM_UART7_PERIPH_CLOCK, new_state);
    } else if(pUSART_x == UART8) {
        crm_periph_clock_enable(CRM_UART8_PERIPH_CLOCK, new_state);
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("USART clock error");
#endif//_TEST_APP_DEBUG_
        drv_status |= ARM_CRM_STA_PERIPH_CLOCK_ERR;
    }
    return TEST_APP_ARM_CRM_isReady(drv_status);
}

bool TEST_APP_ARM_CRM_SPI_ClockEnable(spi_type *pSPI_x, confirm_state new_state)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
    if(pSPI_x == SPI1) {
        crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, new_state);
    } else if(pSPI_x == SPI2) {
        crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, new_state);
    } else if(pSPI_x == SPI3) {
        crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, new_state);
    } else if(pSPI_x == SPI4) {
        crm_periph_clock_enable(CRM_SPI4_PERIPH_CLOCK, new_state);
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("USART clock error");
#endif//_TEST_APP_DEBUG_
        drv_status |= ARM_CRM_STA_PERIPH_CLOCK_ERR;
    }
    return TEST_APP_ARM_CRM_isReady(drv_status);
}

bool TEST_APP_ARM_CRM_DMA_ClockEnable(eTEST_APP_ARM_DMA_Chan_t chan, confirm_state new_state)
{
    uint32_t drv_status = ARM_CRM_STA_NO_ERR;
    crm_periph_clock_type clock_type;
    if((chan < TEST_APP_ARM_DMA1_CHAN1) || (chan > TEST_APP_ARM_DMA2_CHAN7)) {
#ifdef _TEST_APP_DEBUG_
        LOG("DMA clock error");
#endif//_TEST_APP_DEBUG_
        drv_status |= ARM_CRM_STA_PERIPH_CLOCK_ERR;
    } else {
        clock_type = ARM_CRM_DMA_ClockType[chan];
        crm_periph_clock_enable(clock_type, new_state);
    }
    return TEST_APP_ARM_CRM_isReady(drv_status);
}

//peripheral reset

void TEST_APP_ARM_CRM_ClockPeriphReset(crm_periph_reset_type value, confirm_state state)
{
    crm_periph_reset(value, state);
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
