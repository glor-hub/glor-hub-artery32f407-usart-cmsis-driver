#ifndef _CLOCK_H_
#define _CLOCK_H_

#include "at32f403a_407.h"

error_status TEST_APP_ClockInit(void);
void TEST_APP_ClockFailureDetectHandler(void);
flag_status TEST_APP_ClockTestHEXTFailFlag(void);
void TEST_APP_ClockResetHEXTFailFlag(void);

#endif //_CLOCK_H_ 
