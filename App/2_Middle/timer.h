#ifndef _TIMER_H_
#define _TIMER_H_

#include "arm_timer.h"

error_status TEST_APP_TIMER_TimerInit(void);
void TEST_APP_TIMER_DoDelay_usec(eTEST_APP_ARM_TIMER_Types_t timer_type,
                                 uint8_t num_useconds);

#endif //_TIMER_H_
