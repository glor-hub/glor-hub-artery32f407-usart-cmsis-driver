#ifndef _CLOCK_H_
#define _CLOCK_H_

#include "at32f403a_407.h"

error_status ClockInit(void);
void ClockFailureDetectHandler(void);

#endif //_CLOCK_H_ 
