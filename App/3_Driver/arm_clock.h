#ifndef _ARM_CLOCK_H_
#define _ARM_CLOCK_H_

#include "at32f403a_407.h"

void ARM_CRM_ClockPeriphEnable(crm_periph_clock_type value, confirm_state state);
void ARM_CRM_ClockPeriphReset(crm_periph_reset_type value, confirm_state state);

#endif //_ARM_CLOCK_H_ 

