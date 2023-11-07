#ifndef _ARM_CLOCK_H_
#define _ARM_CLOCK_H_

#include "at32f403a_407.h"
#include <stdbool.h>

bool TEST_APP_ARM_CRM_isReady(uint32_t status);
uint32_t TEST_APP_ARM_CRM_HEXT_PLL_SysClock240MHzConfig(void);
uint32_t TEST_APP_ARM_CRM_HICK_PLL_SysClock240MHzConfig(void);
void TEST_APP_ARM_CRM_BusClockConfig(void);
crm_sclk_type TEST_APP_ARM_CRM_GetClockSourceForSwitch(void);
uint32_t TEST_APP_ARM_CRM_SysClockSwitchCmd(crm_sclk_type value);

bool TEST_APP_ARM_CRM_GPIO_ClockEnable(gpio_type *pGPIO_x, confirm_state new_state);
bool TEST_APP_ARM_CRM_USART_ClockEnable(usart_type *pUSART_x, confirm_state new_state);
bool TEST_APP_ARM_CRM_SPI_ClockEnable(spi_type *pSPI_x, confirm_state new_state);
bool TEST_APP_ARM_CRM_DMA_ClockEnable(dma_channel_type *pDMAxChan_y, confirm_state new_state);
void TEST_APP_ARM_CRM_ClockPeriphReset(crm_periph_reset_type value, confirm_state state);

#endif //_ARM_CLOCK_H_ 
