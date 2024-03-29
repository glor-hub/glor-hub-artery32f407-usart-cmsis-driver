#ifndef _ARM_CLOCK_H_
#define _ARM_CLOCK_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "common.h"

bool TEST_APP_ARM_CRM_isReady(uint32_t status);
uint32_t TEST_APP_ARM_CRM_HEXT_PLL_SysClock240MHzConfig(void);
uint32_t TEST_APP_ARM_CRM_HICK_PLL_SysClock240MHzConfig(void);
void TEST_APP_ARM_CRM_BusClockConfig(void);
crm_sclk_type TEST_APP_ARM_CRM_GetClockSourceForSwitch(void);
uint32_t TEST_APP_ARM_CRM_SysClockSwitchCmd(crm_sclk_type value);

bool TEST_APP_ARM_CRM_PeriphClockEnable(eTEST_APP_Periph_Types_t periph, uint8_t param, confirm_state new_state);

bool TEST_APP_ARM_CRM_PeriphReset(eTEST_APP_Periph_Types_t periph, uint8_t param, confirm_state new_state);

#endif //_ARM_CLOCK_H_ 
