#ifndef _USART_H_
#define _USART_H_

#include "at32f403a_407.h"
#include "app.h"
#include "arm_usart.h"

error_status TEST_APP_USART_Init(void);
error_status TEST_APP_USART_Uninitialize(TEST_APP_ARM_USART_Driver_t *p_drv);

#ifdef _TEST_APP_DEBUG_
error_status TEST_APP_USART_Test(void);
#endif//_TEST_APP_DEBUG_

void TEST_APP_USART_cb(void);
int8_t TEST_APP_USART_printf(TEST_APP_ARM_USART_Driver_t *p_drv, char *fmt, ...);

#endif //_USART_H_ 
