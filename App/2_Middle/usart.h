#ifndef _USART_H_
#define _USART_H_

#include "at32f403a_407.h"
#include "app.h"
#include "arm_usart.h"

error_status USART_Init(void);
error_status USART_Uninitialize(ARM_USART_Driver_t *p_drv);

#ifdef _APP_DEBUG_
error_status USART_Test(void);
#endif//_APP_DEBUG_

void USART_cb(void);
int8_t USART_printf(ARM_USART_Driver_t *p_drv, char *fmt, ...);

#endif //_USART_H_ 
