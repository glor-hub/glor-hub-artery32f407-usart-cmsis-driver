#ifndef _APP_H_
#define _APP_H_

#include "at32f403a_407.h"

#define _TEST_APP_DEBUG_

void TEST_APP_StartUp(void);
error_status TEST_APP_AppInit(void);
void TEST_APP_AppIdleTask(void);

#endif //_APP_H_
