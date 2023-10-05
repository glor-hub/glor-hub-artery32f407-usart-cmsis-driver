#ifndef _ARM_DRIVER_H_
#define _ARM_DRIVER_H_

#include "at32f403a_407.h"
#include <stdbool.h>

//CMSIS Driver Status
#define TEST_APP_ARM_DRIVER_NO_ERROR                ((uint32_t)0UL)
#define TEST_APP_ARM_DRIVER_ERROR                   ((uint32_t)1UL << 0)
#define TEST_APP_ARM_DRIVER_ERROR_PARAMETER         ((uint32_t)1UL << 1)
#define TEST_APP_ARM_DRIVER_ERROR_BUSY              ((uint32_t)1UL << 2)
#define TEST_APP_ARM_DRIVER_ERROR_TIMEOUT           ((uint32_t)1UL << 3)
#define TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED       ((uint32_t)1UL << 4)

bool TEST_APP_ARM_DRIVER_isReady(uint32_t status);

#endif //_ARM_DRIVER_H_ 
